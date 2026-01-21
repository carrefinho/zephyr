/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/hci_types.h>

#include "hal/ccm.h"

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"

#include "ll.h"
#include "ll_settings.h"

#include "lll.h"
#include "ll_feat.h"
#include "lll/lll_df_types.h"
#include "lll_conn.h"
#include "lll_conn_iso.h"

#include "ull_tx_queue.h"

#include "isoal.h"
#include "ull_iso_types.h"
#include "ull_conn_iso_types.h"
#include "ull_conn_iso_internal.h"

#include "ull_conn_types.h"
#include "ull_internal.h"
#include "ull_llcp.h"
#include "ull_llcp_features.h"
#include "ull_llcp_internal.h"
#include "ull_conn_internal.h"

#include <soc.h>
#include "hal/debug.h"

#if defined(CONFIG_BT_CTLR_SUBRATING)

/* LLCP Local Procedure Subrate FSM states */
enum {
	LP_SUBRATE_STATE_IDLE = LLCP_STATE_IDLE,
	/* Central states */
	LP_SUBRATE_STATE_WAIT_TX_SUBRATE_IND,
	LP_SUBRATE_STATE_WAIT_TX_ACK_SUBRATE_IND,
	/* Peripheral states */
	LP_SUBRATE_STATE_WAIT_TX_SUBRATE_REQ,
	LP_SUBRATE_STATE_WAIT_TX_ACK_SUBRATE_REQ,
	LP_SUBRATE_STATE_WAIT_RX_SUBRATE_IND,
	LP_SUBRATE_STATE_WAIT_NTF_AVAIL,
};

/* LLCP Local Procedure Subrate FSM events */
enum {
	/* Procedure run */
	LP_SUBRATE_EVT_RUN,

	/* Indication received */
	LP_SUBRATE_EVT_SUBRATE_IND,

	/* Ack received */
	LP_SUBRATE_EVT_ACK,

	/* Ready to notify host */
	LP_SUBRATE_EVT_NTF,

	/* Reject response received */
	LP_SUBRATE_EVT_REJECT,

	/* Unknown response received */
	LP_SUBRATE_EVT_UNKNOWN,
};

/* LLCP Remote Procedure Subrate FSM states */
enum {
	RP_SUBRATE_STATE_IDLE = LLCP_STATE_IDLE,
	/* Central: receiving LL_SUBRATE_REQ from Peripheral */
	RP_SUBRATE_STATE_WAIT_RX_SUBRATE_REQ,
	RP_SUBRATE_STATE_WAIT_TX_SUBRATE_IND,
	RP_SUBRATE_STATE_WAIT_TX_ACK_SUBRATE_IND,
	RP_SUBRATE_STATE_WAIT_TX_REJECT_EXT_IND,
	RP_SUBRATE_STATE_WAIT_TX_ACK_REJECT_EXT_IND,
	/* Peripheral: receiving LL_SUBRATE_IND from Central */
	RP_SUBRATE_STATE_WAIT_RX_SUBRATE_IND,
	RP_SUBRATE_STATE_WAIT_NTF_AVAIL,
};

/* LLCP Remote Procedure Subrate FSM events */
enum {
	/* Procedure run */
	RP_SUBRATE_EVT_RUN,

	/* Request received */
	RP_SUBRATE_EVT_SUBRATE_REQ,

	/* Indication received */
	RP_SUBRATE_EVT_SUBRATE_IND,

	/* Ack received */
	RP_SUBRATE_EVT_ACK,

	/* Ready to notify host */
	RP_SUBRATE_EVT_NTF,
};

/*
 * Helper functions
 */

static void subrate_ntf(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct node_rx_pdu *ntf;
	struct node_rx_subrate *sr;

	ntf = ctx->node_ref.rx;
	ctx->node_ref.rx = NULL;
	LL_ASSERT(ntf);

	ntf->hdr.type = NODE_RX_TYPE_SUBRATE_CHANGE;
	ntf->hdr.handle = conn->lll.handle;

	sr = (struct node_rx_subrate *)ntf->pdu;
	sr->status = ctx->data.subrate.error;
	sr->factor = conn->subrate.factor;
	sr->latency = conn->subrate.latency;
	sr->continuation_number = conn->subrate.continuation_number;
	sr->timeout = ctx->data.subrate.timeout;

	ll_rx_put_sched(ntf->hdr.link, ntf);
}

static void subrate_apply_params(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct lll_conn *lll = &conn->lll;

	/* Update ULL connection state */
	conn->subrate.factor = ctx->data.subrate.subrate_factor;
	conn->subrate.base_event = ctx->data.subrate.subrate_base_event;
	conn->subrate.latency = ctx->data.subrate.max_latency;
	conn->subrate.continuation_number = ctx->data.subrate.continuation_number;
	conn->supervision_timeout = ctx->data.subrate.timeout * 10U;

	/* Sync parameters to LLL layer for connection event scheduling */
	lll->subrate_factor = ctx->data.subrate.subrate_factor;
	lll->subrate_base_event = ctx->data.subrate.subrate_base_event;
	lll->subrate_continuation = ctx->data.subrate.continuation_number;
	lll->subrate_continuation_count = 0U;
}

#if defined(CONFIG_BT_CENTRAL)
static uint16_t subrate_calc_base_event(struct ll_conn *conn)
{
	uint16_t event_counter = conn->lll.event_counter;

	/* Per spec: S15-14 must equal E15-14 */
	return event_counter;
}

static bool subrate_validate_request(struct ll_conn *conn, struct proc_ctx *ctx)
{
	/* Check against acceptable defaults set by host */
	if (ctx->data.subrate.max_latency > conn->subrate.defaults.max_latency) {
		return false;
	}
	if (ctx->data.subrate.timeout > conn->subrate.defaults.timeout) {
		return false;
	}
	if (ctx->data.subrate.subrate_factor_max < conn->subrate.defaults.subrate_min) {
		return false;
	}
	if (ctx->data.subrate.subrate_factor_min > conn->subrate.defaults.subrate_max) {
		return false;
	}

	return true;
}

static void subrate_prepare_ind(struct ll_conn *conn, struct proc_ctx *ctx)
{
	/* Choose subrate factor within acceptable range */
	uint16_t factor = ctx->data.subrate.subrate_factor_max;

	if (factor < conn->subrate.defaults.subrate_min) {
		factor = conn->subrate.defaults.subrate_min;
	}
	if (factor > conn->subrate.defaults.subrate_max) {
		factor = conn->subrate.defaults.subrate_max;
	}

	ctx->data.subrate.subrate_factor = factor;
	ctx->data.subrate.subrate_base_event = subrate_calc_base_event(conn);

	/* Continuation number is min of requested and (factor - 1) */
	if (ctx->data.subrate.continuation_number >= factor) {
		ctx->data.subrate.continuation_number = factor - 1U;
	}
}
#endif /* CONFIG_BT_CENTRAL */

/*
 * Local Procedure - Subrate
 */

static void lp_subrate_tx(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t opcode)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT(tx);

	pdu = (struct pdu_data *)tx->pdu;

	switch (opcode) {
#if defined(CONFIG_BT_CENTRAL)
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_IND:
		llcp_pdu_encode_subrate_ind(ctx, pdu);
		break;
#endif /* CONFIG_BT_CENTRAL */
#if defined(CONFIG_BT_PERIPHERAL)
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ:
		llcp_pdu_encode_subrate_req(ctx, pdu);
		break;
#endif /* CONFIG_BT_PERIPHERAL */
	default:
		LL_ASSERT(0);
	}

	ctx->tx_opcode = opcode;
	ctx->node_ref.tx_ack = tx;

	llcp_tx_enqueue(conn, tx);
}

static void lp_subrate_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_lr_complete(conn);
	ctx->state = LP_SUBRATE_STATE_IDLE;
}

#if defined(CONFIG_BT_CENTRAL)
static void lp_subrate_st_idle_central(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				       void *param)
{
	switch (evt) {
	case LP_SUBRATE_EVT_RUN:
		/* Central initiates subrate update by sending LL_SUBRATE_IND */
		ctx->data.subrate.subrate_factor = ctx->data.subrate.subrate_factor_max;
		ctx->data.subrate.subrate_base_event = subrate_calc_base_event(conn);

		/* Continuation number is min of requested and (factor - 1) */
		if (ctx->data.subrate.continuation_number >= ctx->data.subrate.subrate_factor) {
			ctx->data.subrate.continuation_number =
				ctx->data.subrate.subrate_factor - 1U;
		}

		if (llcp_lr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
			ctx->state = LP_SUBRATE_STATE_WAIT_TX_SUBRATE_IND;
		} else {
			lp_subrate_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_SUBRATE_IND);
			ctx->state = LP_SUBRATE_STATE_WAIT_TX_ACK_SUBRATE_IND;
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void lp_subrate_st_wait_tx_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx,
					      uint8_t evt, void *param)
{
	switch (evt) {
	case LP_SUBRATE_EVT_RUN:
		if (!llcp_lr_ispaused(conn) && llcp_tx_alloc_peek(conn, ctx)) {
			lp_subrate_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_SUBRATE_IND);
			ctx->state = LP_SUBRATE_STATE_WAIT_TX_ACK_SUBRATE_IND;
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void lp_subrate_st_wait_tx_ack_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx,
						  uint8_t evt, void *param)
{
	switch (evt) {
	case LP_SUBRATE_EVT_ACK:
		/* Apply parameters and notify host */
		ctx->data.subrate.error = BT_HCI_ERR_SUCCESS;
		subrate_apply_params(conn, ctx);

		if (ctx->data.subrate.host_initiated) {
			if (llcp_ntf_alloc_is_available()) {
				subrate_ntf(conn, ctx);
				lp_subrate_complete(conn, ctx);
			} else {
				ctx->state = LP_SUBRATE_STATE_WAIT_NTF_AVAIL;
			}
		} else {
			lp_subrate_complete(conn, ctx);
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}
#endif /* CONFIG_BT_CENTRAL */

#if defined(CONFIG_BT_PERIPHERAL)
static void lp_subrate_st_idle_peripheral(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					  void *param)
{
	switch (evt) {
	case LP_SUBRATE_EVT_RUN:
		/* Peripheral initiates subrate request by sending LL_SUBRATE_REQ */
		if (llcp_lr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
			ctx->state = LP_SUBRATE_STATE_WAIT_TX_SUBRATE_REQ;
		} else {
			lp_subrate_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ);
			ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_IND;
			ctx->state = LP_SUBRATE_STATE_WAIT_TX_ACK_SUBRATE_REQ;
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void lp_subrate_st_wait_tx_subrate_req(struct ll_conn *conn, struct proc_ctx *ctx,
					      uint8_t evt, void *param)
{
	switch (evt) {
	case LP_SUBRATE_EVT_RUN:
		if (!llcp_lr_ispaused(conn) && llcp_tx_alloc_peek(conn, ctx)) {
			lp_subrate_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ);
			ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_IND;
			ctx->state = LP_SUBRATE_STATE_WAIT_TX_ACK_SUBRATE_REQ;
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void lp_subrate_st_wait_tx_ack_subrate_req(struct ll_conn *conn, struct proc_ctx *ctx,
						  uint8_t evt, void *param)
{
	switch (evt) {
	case LP_SUBRATE_EVT_ACK:
		/* Wait for LL_SUBRATE_IND or LL_REJECT_EXT_IND */
		ctx->state = LP_SUBRATE_STATE_WAIT_RX_SUBRATE_IND;
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void lp_subrate_st_wait_rx_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx,
					      uint8_t evt, void *param)
{
	switch (evt) {
	case LP_SUBRATE_EVT_SUBRATE_IND:
		llcp_pdu_decode_subrate_ind(ctx, param);
		ctx->data.subrate.error = BT_HCI_ERR_SUCCESS;
		subrate_apply_params(conn, ctx);

		if (ctx->data.subrate.host_initiated) {
			if (llcp_ntf_alloc_is_available()) {
				subrate_ntf(conn, ctx);
				lp_subrate_complete(conn, ctx);
			} else {
				ctx->state = LP_SUBRATE_STATE_WAIT_NTF_AVAIL;
			}
		} else {
			lp_subrate_complete(conn, ctx);
		}
		break;
	case LP_SUBRATE_EVT_REJECT:
		llcp_pdu_decode_reject_ext_ind(ctx, param);
		ctx->data.subrate.error = ctx->reject_ext_ind.error_code;

		if (ctx->data.subrate.host_initiated) {
			if (llcp_ntf_alloc_is_available()) {
				subrate_ntf(conn, ctx);
				lp_subrate_complete(conn, ctx);
			} else {
				ctx->state = LP_SUBRATE_STATE_WAIT_NTF_AVAIL;
			}
		} else {
			lp_subrate_complete(conn, ctx);
		}
		break;
	case LP_SUBRATE_EVT_UNKNOWN:
		/* Peer doesn't support subrating */
		ctx->data.subrate.error = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE;

		if (ctx->data.subrate.host_initiated) {
			if (llcp_ntf_alloc_is_available()) {
				subrate_ntf(conn, ctx);
				lp_subrate_complete(conn, ctx);
			} else {
				ctx->state = LP_SUBRATE_STATE_WAIT_NTF_AVAIL;
			}
		} else {
			lp_subrate_complete(conn, ctx);
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}
#endif /* CONFIG_BT_PERIPHERAL */

static void lp_subrate_st_wait_ntf_avail(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					 void *param)
{
	switch (evt) {
	case LP_SUBRATE_EVT_RUN:
		if (llcp_ntf_alloc_is_available()) {
			subrate_ntf(conn, ctx);
			lp_subrate_complete(conn, ctx);
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void lp_subrate_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				   void *param)
{
	switch (ctx->state) {
	case LP_SUBRATE_STATE_IDLE:
#if defined(CONFIG_BT_CENTRAL)
		if (conn->lll.role == BT_HCI_ROLE_CENTRAL) {
			lp_subrate_st_idle_central(conn, ctx, evt, param);
			break;
		}
#endif /* CONFIG_BT_CENTRAL */
#if defined(CONFIG_BT_PERIPHERAL)
		if (conn->lll.role == BT_HCI_ROLE_PERIPHERAL) {
			lp_subrate_st_idle_peripheral(conn, ctx, evt, param);
			break;
		}
#endif /* CONFIG_BT_PERIPHERAL */
		break;
#if defined(CONFIG_BT_CENTRAL)
	case LP_SUBRATE_STATE_WAIT_TX_SUBRATE_IND:
		lp_subrate_st_wait_tx_subrate_ind(conn, ctx, evt, param);
		break;
	case LP_SUBRATE_STATE_WAIT_TX_ACK_SUBRATE_IND:
		lp_subrate_st_wait_tx_ack_subrate_ind(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_CENTRAL */
#if defined(CONFIG_BT_PERIPHERAL)
	case LP_SUBRATE_STATE_WAIT_TX_SUBRATE_REQ:
		lp_subrate_st_wait_tx_subrate_req(conn, ctx, evt, param);
		break;
	case LP_SUBRATE_STATE_WAIT_TX_ACK_SUBRATE_REQ:
		lp_subrate_st_wait_tx_ack_subrate_req(conn, ctx, evt, param);
		break;
	case LP_SUBRATE_STATE_WAIT_RX_SUBRATE_IND:
		lp_subrate_st_wait_rx_subrate_ind(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_PERIPHERAL */
	case LP_SUBRATE_STATE_WAIT_NTF_AVAIL:
		lp_subrate_st_wait_ntf_avail(conn, ctx, evt, param);
		break;
	default:
		/* Unknown state */
		LL_ASSERT(0);
		break;
	}
}

void llcp_lp_subrate_init_proc(struct proc_ctx *ctx)
{
	ctx->state = LP_SUBRATE_STATE_IDLE;
}

void llcp_lp_subrate_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	lp_subrate_execute_fsm(conn, ctx, LP_SUBRATE_EVT_RUN, param);
}

void llcp_lp_subrate_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_IND:
		lp_subrate_execute_fsm(conn, ctx, LP_SUBRATE_EVT_SUBRATE_IND, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND:
		lp_subrate_execute_fsm(conn, ctx, LP_SUBRATE_EVT_REJECT, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_UNKNOWN_RSP:
		lp_subrate_execute_fsm(conn, ctx, LP_SUBRATE_EVT_UNKNOWN, pdu);
		break;
	default:
		/* Unexpected opcode */
		LL_ASSERT(0);
		break;
	}
}

void llcp_lp_subrate_tx_ack(struct ll_conn *conn, struct proc_ctx *ctx, struct pdu_data *pdu)
{
	lp_subrate_execute_fsm(conn, ctx, LP_SUBRATE_EVT_ACK, pdu);
}

/*
 * Remote Procedure - Subrate
 */

static void rp_subrate_tx(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t opcode)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT(tx);

	pdu = (struct pdu_data *)tx->pdu;

	switch (opcode) {
#if defined(CONFIG_BT_CENTRAL)
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_IND:
		llcp_pdu_encode_subrate_ind(ctx, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND:
		llcp_pdu_encode_reject_ext_ind(pdu, ctx->reject_ext_ind.reject_opcode,
					       ctx->reject_ext_ind.error_code);
		break;
#endif /* CONFIG_BT_CENTRAL */
	default:
		LL_ASSERT(0);
	}

	ctx->tx_opcode = opcode;
	ctx->node_ref.tx_ack = tx;

	llcp_tx_enqueue(conn, tx);
}

static void rp_subrate_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_rr_complete(conn);
	ctx->state = RP_SUBRATE_STATE_IDLE;
}

#if defined(CONFIG_BT_CENTRAL)
static void rp_subrate_st_wait_rx_subrate_req(struct ll_conn *conn, struct proc_ctx *ctx,
					      uint8_t evt, void *param)
{
	switch (evt) {
	case RP_SUBRATE_EVT_SUBRATE_REQ:
		llcp_pdu_decode_subrate_req(ctx, param);

		/* Validate request against acceptable parameters */
		if (subrate_validate_request(conn, ctx)) {
			subrate_prepare_ind(conn, ctx);

			if (llcp_rr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
				ctx->state = RP_SUBRATE_STATE_WAIT_TX_SUBRATE_IND;
			} else {
				rp_subrate_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_SUBRATE_IND);
				ctx->state = RP_SUBRATE_STATE_WAIT_TX_ACK_SUBRATE_IND;
			}
		} else {
			/* Reject the request */
			ctx->reject_ext_ind.reject_opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ;
			ctx->reject_ext_ind.error_code = BT_HCI_ERR_UNSUPP_LL_PARAM_VAL;

			if (llcp_rr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
				ctx->state = RP_SUBRATE_STATE_WAIT_TX_REJECT_EXT_IND;
			} else {
				rp_subrate_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND);
				ctx->state = RP_SUBRATE_STATE_WAIT_TX_ACK_REJECT_EXT_IND;
			}
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void rp_subrate_st_wait_tx_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx,
					      uint8_t evt, void *param)
{
	switch (evt) {
	case RP_SUBRATE_EVT_RUN:
		if (!llcp_rr_ispaused(conn) && llcp_tx_alloc_peek(conn, ctx)) {
			rp_subrate_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_SUBRATE_IND);
			ctx->state = RP_SUBRATE_STATE_WAIT_TX_ACK_SUBRATE_IND;
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void rp_subrate_st_wait_tx_ack_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx,
						  uint8_t evt, void *param)
{
	switch (evt) {
	case RP_SUBRATE_EVT_ACK:
		/* Apply parameters and notify host */
		ctx->data.subrate.error = BT_HCI_ERR_SUCCESS;
		subrate_apply_params(conn, ctx);

		if (llcp_ntf_alloc_is_available()) {
			subrate_ntf(conn, ctx);
			rp_subrate_complete(conn, ctx);
		} else {
			ctx->state = RP_SUBRATE_STATE_WAIT_NTF_AVAIL;
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void rp_subrate_st_wait_tx_reject_ext_ind(struct ll_conn *conn, struct proc_ctx *ctx,
						 uint8_t evt, void *param)
{
	switch (evt) {
	case RP_SUBRATE_EVT_RUN:
		if (!llcp_rr_ispaused(conn) && llcp_tx_alloc_peek(conn, ctx)) {
			rp_subrate_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND);
			ctx->state = RP_SUBRATE_STATE_WAIT_TX_ACK_REJECT_EXT_IND;
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void rp_subrate_st_wait_tx_ack_reject_ext_ind(struct ll_conn *conn, struct proc_ctx *ctx,
						     uint8_t evt, void *param)
{
	switch (evt) {
	case RP_SUBRATE_EVT_ACK:
		rp_subrate_complete(conn, ctx);
		break;
	default:
		/* Ignore other events */
		break;
	}
}
#endif /* CONFIG_BT_CENTRAL */

#if defined(CONFIG_BT_PERIPHERAL)
static void rp_subrate_st_wait_rx_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx,
					      uint8_t evt, void *param)
{
	switch (evt) {
	case RP_SUBRATE_EVT_SUBRATE_IND:
		llcp_pdu_decode_subrate_ind(ctx, param);
		ctx->data.subrate.error = BT_HCI_ERR_SUCCESS;
		subrate_apply_params(conn, ctx);

		if (llcp_ntf_alloc_is_available()) {
			subrate_ntf(conn, ctx);
			rp_subrate_complete(conn, ctx);
		} else {
			ctx->state = RP_SUBRATE_STATE_WAIT_NTF_AVAIL;
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}
#endif /* CONFIG_BT_PERIPHERAL */

static void rp_subrate_st_wait_ntf_avail(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					 void *param)
{
	switch (evt) {
	case RP_SUBRATE_EVT_RUN:
		if (llcp_ntf_alloc_is_available()) {
			subrate_ntf(conn, ctx);
			rp_subrate_complete(conn, ctx);
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void rp_subrate_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				   void *param)
{
	switch (ctx->state) {
#if defined(CONFIG_BT_CENTRAL)
	case RP_SUBRATE_STATE_WAIT_RX_SUBRATE_REQ:
		rp_subrate_st_wait_rx_subrate_req(conn, ctx, evt, param);
		break;
	case RP_SUBRATE_STATE_WAIT_TX_SUBRATE_IND:
		rp_subrate_st_wait_tx_subrate_ind(conn, ctx, evt, param);
		break;
	case RP_SUBRATE_STATE_WAIT_TX_ACK_SUBRATE_IND:
		rp_subrate_st_wait_tx_ack_subrate_ind(conn, ctx, evt, param);
		break;
	case RP_SUBRATE_STATE_WAIT_TX_REJECT_EXT_IND:
		rp_subrate_st_wait_tx_reject_ext_ind(conn, ctx, evt, param);
		break;
	case RP_SUBRATE_STATE_WAIT_TX_ACK_REJECT_EXT_IND:
		rp_subrate_st_wait_tx_ack_reject_ext_ind(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_CENTRAL */
#if defined(CONFIG_BT_PERIPHERAL)
	case RP_SUBRATE_STATE_WAIT_RX_SUBRATE_IND:
		rp_subrate_st_wait_rx_subrate_ind(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_PERIPHERAL */
	case RP_SUBRATE_STATE_WAIT_NTF_AVAIL:
		rp_subrate_st_wait_ntf_avail(conn, ctx, evt, param);
		break;
	default:
		/* Unknown state */
		LL_ASSERT(0);
		break;
	}
}

void llcp_rp_subrate_init_proc(struct proc_ctx *ctx)
{
#if defined(CONFIG_BT_CENTRAL)
	ctx->state = RP_SUBRATE_STATE_WAIT_RX_SUBRATE_REQ;
#elif defined(CONFIG_BT_PERIPHERAL)
	ctx->state = RP_SUBRATE_STATE_WAIT_RX_SUBRATE_IND;
#endif
}

void llcp_rp_subrate_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	rp_subrate_execute_fsm(conn, ctx, RP_SUBRATE_EVT_RUN, param);
}

void llcp_rp_subrate_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ:
		rp_subrate_execute_fsm(conn, ctx, RP_SUBRATE_EVT_SUBRATE_REQ, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_IND:
		rp_subrate_execute_fsm(conn, ctx, RP_SUBRATE_EVT_SUBRATE_IND, pdu);
		break;
	default:
		/* Unexpected opcode */
		LL_ASSERT(0);
		break;
	}
}

void llcp_rp_subrate_tx_ack(struct ll_conn *conn, struct proc_ctx *ctx, struct pdu_data *pdu)
{
	rp_subrate_execute_fsm(conn, ctx, RP_SUBRATE_EVT_ACK, pdu);
}

#endif /* CONFIG_BT_CTLR_SUBRATING */
