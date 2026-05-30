/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * LE Frame Space Update, Bluetooth Core Specification v6.2:
 *   - Vol 6, Part B, Section 4.6.46  (feature, LL bit 65)
 *   - Vol 6, Part B, Section 5.1.30  (Frame Space Update procedure)
 *   - Vol 6, Part B, Section 2.4.2.54/55 (LL_FRAME_SPACE_REQ / LL_FRAME_SPACE_RSP)
 *
 * The procedure is role-symmetric: either the Central or the Peripheral may
 * initiate by sending LL_FRAME_SPACE_REQ; the peer responds with
 * LL_FRAME_SPACE_RSP (accept, carrying the selected frame space) or
 * LL_REJECT_EXT_IND. There is no instant.
 *
 * NOTE: This implements the control plane (negotiation, PDUs, HCI command +
 * Complete event). The actual radio inter-frame-space retiming and the
 * Section 5.1.30.1 transitional receive-window widening are wired separately
 * (see TODO(fsu-radio) in frame_space_apply); until then the negotiated frame
 * space is reported to the Host but does not yet change radio timing -- so the
 * link is unaffected. This mirrors the control-plane-first approach the
 * Connection Subrating port takes (TODO(subrate-sched)).
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
#include "ll_feat.h"
#include "ll_settings.h"

#include "lll.h"
#include "lll/lll_df_types.h"
#include "lll_conn.h"

#include "lll_conn_iso.h"

#include "ull_tx_queue.h"

#include "isoal.h"
#include "ull_iso_types.h"
#include "ull_conn_iso_types.h"
#include "ull_conn_iso_internal.h"

#include "ull_conn_internal.h"
#include "ull_conn_types.h"

#include "ull_internal.h"
#include "ull_llcp.h"
#include "ull_llcp_features.h"
#include "ull_llcp_internal.h"

#include <soc.h>
#include "hal/debug.h"

#if defined(CONFIG_BT_CTLR_FRAME_SPACE_UPDATE)

/* LLCP Local Procedure (initiator) Frame Space FSM states */
enum {
	LP_FS_STATE_IDLE = LLCP_STATE_IDLE,
	LP_FS_STATE_WAIT_TX_FRAME_SPACE_REQ,
	LP_FS_STATE_WAIT_RX_FRAME_SPACE_RSP,
	LP_FS_STATE_WAIT_NTF_AVAIL,
};

/* LLCP Local Procedure (initiator) Frame Space FSM events */
enum {
	LP_FS_EVT_RUN,
	/* Response received */
	LP_FS_EVT_FRAME_SPACE_RSP,
	/* Reject response received */
	LP_FS_EVT_REJECT,
	/* Unknown response received */
	LP_FS_EVT_UNKNOWN,
};

/* LLCP Remote Procedure (responder) Frame Space FSM states */
enum {
	RP_FS_STATE_IDLE = LLCP_STATE_IDLE,
	RP_FS_STATE_WAIT_RX_FRAME_SPACE_REQ,
	RP_FS_STATE_WAIT_TX_FRAME_SPACE_RSP,
	RP_FS_STATE_WAIT_TX_ACK_FRAME_SPACE_RSP,
	RP_FS_STATE_WAIT_TX_REJECT_EXT_IND,
	RP_FS_STATE_WAIT_NTF_AVAIL,
};

/* LLCP Remote Procedure (responder) Frame Space FSM events */
enum {
	RP_FS_EVT_RUN,
	/* Request received */
	RP_FS_EVT_FRAME_SPACE_REQ,
	/* Tx Ack received */
	RP_FS_EVT_ACK,
};

/*
 * Shared helpers
 */

/* Responder: choose the frame space to return in LL_FRAME_SPACE_RSP from the
 * requested [FS_Min, FS_Max] range (Core 6.2 5.1.30). Returns false (with
 * ctx->data.frame_space.error set) if the request must be rejected.
 */
static bool frame_space_negotiate(struct ll_conn *conn, struct proc_ctx *ctx)
{
	const uint16_t our_min = CONFIG_BT_CTLR_FSU_MIN_FRAME_SPACE_US;
	uint16_t fs_min = ctx->data.frame_space.fs_min;
	uint16_t fs_max = ctx->data.frame_space.fs_max;
	uint16_t types = ctx->data.frame_space.spacing_types;
	uint16_t fs;

	ARG_UNUSED(conn);

	/* This controller changes only the ACL inter-frame spaces (T_IFS_ACL_CP/PC);
	 * it keeps a single tIFS trio, not per-(type, PHY) values. Clear the MCES /
	 * CIS bits it does not act on -- a cleared RSP bit means "this spacing type
	 * is left unchanged" (Core 6.2 2.4.2.55).
	 */
	types &= (BT_HCI_LE_FRAME_SPACE_UPDATE_SPACING_TYPE_IFS_ACL_CP_MASK |
		  BT_HCI_LE_FRAME_SPACE_UPDATE_SPACING_TYPE_IFS_ACL_PC_MASK);
	if (types == 0U) {
		/* The request selected only spacing types this controller does not
		 * change (e.g. CIS-only); reject for an unsupported parameter value.
		 */
		ctx->data.frame_space.error = BT_HCI_ERR_UNSUPP_FEATURE_PARAM_VAL;
		return false;
	}

	/* Core 6.2 5.1.30: if FS_Min and FS_Max are both below the lowest frame
	 * space this controller supports, the request may be rejected. FS_Max >=
	 * FS_Min, so testing FS_Max suffices.
	 */
	if (fs_max < our_min) {
		ctx->data.frame_space.error = BT_HCI_ERR_UNSUPP_FEATURE_PARAM_VAL;
		return false;
	}

	/* Set FS to the lowest value supported within the requested range (5.1.30
	 * SHOULD). our_min <= FS_Max here; clamp up to FS_Min and down to FS_Max.
	 */
	fs = MAX(fs_min, our_min);
	if (fs > fs_max) {
		fs = fs_max;
	}

	ctx->data.frame_space.frame_space = fs;
	ctx->data.frame_space.spacing_types = types;

	return true;
}

/* Apply the negotiated frame space.
 *
 * TODO(fsu-radio): the radio-integration step writes
 *   conn->lll.tifs_tx_us = tifs_rx_us = tifs_hcto_us = frame_space;
 *   conn->lll.evt_len_upd = 1U;   (re-size ull.ticks_slot via the DLE path)
 * and adds the Section 5.1.30.1 transitional receive-window widening so the
 * mid-connection IFS change does not drop a packet during the swap. Until then
 * this is control-plane-only: the negotiated value is reported to the Host
 * (Complete event) but the radio timing and slot reservation are unchanged, so
 * the link is unaffected (mirrors the Connection Subrating TODO(subrate-sched)).
 */
static void frame_space_apply(struct ll_conn *conn, struct proc_ctx *ctx)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(ctx);
}

static void frame_space_ntf(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct node_rx_pdu *ntf;
	struct node_rx_frame_space_update_complete *fs;

	ntf = ctx->node_ref.rx;
	ctx->node_ref.rx = NULL;
	LL_ASSERT(ntf);

	ntf->hdr.type = NODE_RX_TYPE_FRAME_SPACE_UPDATE_COMPLETE;
	ntf->hdr.handle = conn->lll.handle;

	fs = (struct node_rx_frame_space_update_complete *)ntf->pdu;
	fs->status = ctx->data.frame_space.error;
	fs->initiator = ctx->data.frame_space.initiator;
	/* On failure the frame space is unchanged; report 0 (Host ignores it). */
	fs->frame_space = (ctx->data.frame_space.error == BT_HCI_ERR_SUCCESS) ?
			  ctx->data.frame_space.frame_space : 0U;
	fs->phys = ctx->data.frame_space.phys;
	fs->spacing_types = ctx->data.frame_space.spacing_types;

	ll_rx_put_sched(ntf->hdr.link, ntf);
}

/*
 * LLCP Local Procedure (initiator) Frame Space FSM
 */

static void lp_fs_tx_req(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT(tx);

	pdu = (struct pdu_data *)tx->pdu;

	llcp_pdu_encode_frame_space_req(ctx, pdu);
	ctx->tx_opcode = pdu->llctrl.opcode;

	llcp_tx_enqueue(conn, tx);

	/* Restart procedure response timeout timer */
	llcp_lr_prt_restart(conn);
}

static void lp_fs_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_lr_prt_stop(conn);
	llcp_lr_complete(conn);
	ctx->state = LP_FS_STATE_IDLE;
}

static void lp_fs_ntf_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	frame_space_ntf(conn, ctx);
	lp_fs_complete(conn, ctx);
}

static void lp_fs_send_frame_space_req(struct ll_conn *conn, struct proc_ctx *ctx)
{
	if (llcp_lr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = LP_FS_STATE_WAIT_TX_FRAME_SPACE_REQ;
	} else {
		lp_fs_tx_req(conn, ctx);
		ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_FRAME_SPACE_RSP;
		ctx->state = LP_FS_STATE_WAIT_RX_FRAME_SPACE_RSP;
	}
}

static void lp_fs_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (evt) {
	case LP_FS_EVT_RUN:
		lp_fs_send_frame_space_req(conn, ctx);
		break;
	default:
		break;
	}
}

static void lp_fs_st_wait_tx_frame_space_req(struct ll_conn *conn, struct proc_ctx *ctx,
					     uint8_t evt, void *param)
{
	switch (evt) {
	case LP_FS_EVT_RUN:
		lp_fs_send_frame_space_req(conn, ctx);
		break;
	default:
		break;
	}
}

static void lp_fs_st_wait_rx_frame_space_rsp(struct ll_conn *conn, struct proc_ctx *ctx,
					     uint8_t evt, void *param)
{
	struct pdu_data *pdu = (struct pdu_data *)param;

	switch (evt) {
	case LP_FS_EVT_FRAME_SPACE_RSP:
		llcp_pdu_decode_frame_space_rsp(ctx, pdu);
		frame_space_apply(conn, ctx);
		ctx->data.frame_space.error = BT_HCI_ERR_SUCCESS;
		break;
	case LP_FS_EVT_UNKNOWN:
		/* Peer does not support the feature. The feature bit is on page 1 and
		 * is not cleared via the page-0 unmask path; just report the error.
		 */
		ctx->data.frame_space.error = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE;
		break;
	case LP_FS_EVT_REJECT:
		ctx->data.frame_space.error = pdu->llctrl.reject_ext_ind.error_code;
		break;
	default:
		return;
	}

	if (llcp_ntf_alloc_is_available()) {
		ctx->node_ref.rx = llcp_ntf_alloc();
		lp_fs_ntf_complete(conn, ctx);
	} else {
		ctx->state = LP_FS_STATE_WAIT_NTF_AVAIL;
	}
}

static void lp_fs_st_wait_ntf_avail(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				    void *param)
{
	switch (evt) {
	case LP_FS_EVT_RUN:
		if (llcp_ntf_alloc_is_available()) {
			ctx->node_ref.rx = llcp_ntf_alloc();
			lp_fs_ntf_complete(conn, ctx);
		}
		break;
	default:
		break;
	}
}

static void lp_fs_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (ctx->state) {
	case LP_FS_STATE_IDLE:
		lp_fs_st_idle(conn, ctx, evt, param);
		break;
	case LP_FS_STATE_WAIT_TX_FRAME_SPACE_REQ:
		lp_fs_st_wait_tx_frame_space_req(conn, ctx, evt, param);
		break;
	case LP_FS_STATE_WAIT_RX_FRAME_SPACE_RSP:
		lp_fs_st_wait_rx_frame_space_rsp(conn, ctx, evt, param);
		break;
	case LP_FS_STATE_WAIT_NTF_AVAIL:
		lp_fs_st_wait_ntf_avail(conn, ctx, evt, param);
		break;
	default:
		LL_ASSERT(0);
		break;
	}
}

void llcp_lp_frame_space_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_FRAME_SPACE_RSP:
		lp_fs_execute_fsm(conn, ctx, LP_FS_EVT_FRAME_SPACE_RSP, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_UNKNOWN_RSP:
		lp_fs_execute_fsm(conn, ctx, LP_FS_EVT_UNKNOWN, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND:
		lp_fs_execute_fsm(conn, ctx, LP_FS_EVT_REJECT, pdu);
		break;
	default:
		/* Invalid PDU received, terminate connection */
		conn->llcp_terminate.reason_final = BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
		lp_fs_complete(conn, ctx);
		break;
	}
}

void llcp_lp_frame_space_init_proc(struct proc_ctx *ctx)
{
	ctx->state = LP_FS_STATE_IDLE;
}

void llcp_lp_frame_space_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	lp_fs_execute_fsm(conn, ctx, LP_FS_EVT_RUN, param);
}

/*
 * LLCP Remote Procedure (responder) Frame Space FSM
 */

static void rp_fs_tx(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t opcode)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT(tx);

	pdu = (struct pdu_data *)tx->pdu;

	switch (opcode) {
	case PDU_DATA_LLCTRL_TYPE_FRAME_SPACE_RSP:
		llcp_pdu_encode_frame_space_rsp(ctx, pdu);
		/* The procedure completes once the RSP is acknowledged. */
		ctx->node_ref.tx_ack = tx;
		break;
	case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND:
		llcp_pdu_encode_reject_ext_ind(pdu, PDU_DATA_LLCTRL_TYPE_FRAME_SPACE_REQ,
					       ctx->data.frame_space.error);
		break;
	default:
		LL_ASSERT(0);
		break;
	}

	ctx->tx_opcode = pdu->llctrl.opcode;

	llcp_tx_enqueue(conn, tx);

	llcp_rr_prt_restart(conn);
}

static void rp_fs_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_rr_prt_stop(conn);
	llcp_rr_complete(conn);
	ctx->state = RP_FS_STATE_IDLE;
}

static void rp_fs_ntf_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	frame_space_ntf(conn, ctx);
	rp_fs_complete(conn, ctx);
}

static void rp_fs_send_reject_ext_ind(struct ll_conn *conn, struct proc_ctx *ctx)
{
	if (llcp_rr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = RP_FS_STATE_WAIT_TX_REJECT_EXT_IND;
	} else {
		rp_fs_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND);
		rp_fs_complete(conn, ctx);
	}
}

static void rp_fs_send_frame_space_rsp(struct ll_conn *conn, struct proc_ctx *ctx)
{
	if (llcp_rr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = RP_FS_STATE_WAIT_TX_FRAME_SPACE_RSP;
	} else {
		rp_fs_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_FRAME_SPACE_RSP);
		ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_UNUSED;
		ctx->state = RP_FS_STATE_WAIT_TX_ACK_FRAME_SPACE_RSP;
	}
}

static void rp_fs_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (evt) {
	case RP_FS_EVT_RUN:
		ctx->state = RP_FS_STATE_WAIT_RX_FRAME_SPACE_REQ;
		break;
	default:
		break;
	}
}

static void rp_fs_st_wait_rx_frame_space_req(struct ll_conn *conn, struct proc_ctx *ctx,
					     uint8_t evt, void *param)
{
	switch (evt) {
	case RP_FS_EVT_FRAME_SPACE_REQ:
		llcp_pdu_decode_frame_space_req(ctx, (struct pdu_data *)param);
		ctx->data.frame_space.initiator =
			BT_HCI_LE_FRAME_SPACE_UPDATE_INITIATOR_PEER;
		if (ctx->data.frame_space.error != BT_HCI_ERR_SUCCESS) {
			/* decode flagged a zero PHYS/Spacing_Types field (0x1E) */
			rp_fs_send_reject_ext_ind(conn, ctx);
		} else if (frame_space_negotiate(conn, ctx)) {
			rp_fs_send_frame_space_rsp(conn, ctx);
		} else {
			rp_fs_send_reject_ext_ind(conn, ctx);
		}
		break;
	default:
		break;
	}
}

static void rp_fs_st_wait_tx_frame_space_rsp(struct ll_conn *conn, struct proc_ctx *ctx,
					     uint8_t evt, void *param)
{
	switch (evt) {
	case RP_FS_EVT_RUN:
		rp_fs_send_frame_space_rsp(conn, ctx);
		break;
	default:
		break;
	}
}

static void rp_fs_st_wait_tx_reject_ext_ind(struct ll_conn *conn, struct proc_ctx *ctx,
					    uint8_t evt, void *param)
{
	switch (evt) {
	case RP_FS_EVT_RUN:
		rp_fs_send_reject_ext_ind(conn, ctx);
		break;
	default:
		break;
	}
}

static void rp_fs_st_wait_tx_ack_frame_space_rsp(struct ll_conn *conn, struct proc_ctx *ctx,
						 uint8_t evt, void *param)
{
	switch (evt) {
	case RP_FS_EVT_ACK:
		frame_space_apply(conn, ctx);
		ctx->data.frame_space.error = BT_HCI_ERR_SUCCESS;
		if (llcp_ntf_alloc_is_available()) {
			ctx->node_ref.rx = llcp_ntf_alloc();
			rp_fs_ntf_complete(conn, ctx);
		} else {
			ctx->state = RP_FS_STATE_WAIT_NTF_AVAIL;
		}
		break;
	default:
		break;
	}
}

static void rp_fs_st_wait_ntf_avail(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				    void *param)
{
	switch (evt) {
	case RP_FS_EVT_RUN:
		if (llcp_ntf_alloc_is_available()) {
			ctx->node_ref.rx = llcp_ntf_alloc();
			rp_fs_ntf_complete(conn, ctx);
		}
		break;
	default:
		break;
	}
}

static void rp_fs_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (ctx->state) {
	case RP_FS_STATE_IDLE:
		rp_fs_st_idle(conn, ctx, evt, param);
		break;
	case RP_FS_STATE_WAIT_RX_FRAME_SPACE_REQ:
		rp_fs_st_wait_rx_frame_space_req(conn, ctx, evt, param);
		break;
	case RP_FS_STATE_WAIT_TX_FRAME_SPACE_RSP:
		rp_fs_st_wait_tx_frame_space_rsp(conn, ctx, evt, param);
		break;
	case RP_FS_STATE_WAIT_TX_ACK_FRAME_SPACE_RSP:
		rp_fs_st_wait_tx_ack_frame_space_rsp(conn, ctx, evt, param);
		break;
	case RP_FS_STATE_WAIT_TX_REJECT_EXT_IND:
		rp_fs_st_wait_tx_reject_ext_ind(conn, ctx, evt, param);
		break;
	case RP_FS_STATE_WAIT_NTF_AVAIL:
		rp_fs_st_wait_ntf_avail(conn, ctx, evt, param);
		break;
	default:
		LL_ASSERT(0);
		break;
	}
}

void llcp_rp_frame_space_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_FRAME_SPACE_REQ:
		rp_fs_execute_fsm(conn, ctx, RP_FS_EVT_FRAME_SPACE_REQ, pdu);
		break;
	default:
		/* Invalid PDU received, terminate connection */
		conn->llcp_terminate.reason_final = BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
		rp_fs_complete(conn, ctx);
		break;
	}
}

void llcp_rp_frame_space_tx_ack(struct ll_conn *conn, struct proc_ctx *ctx, struct node_tx *tx)
{
	rp_fs_execute_fsm(conn, ctx, RP_FS_EVT_ACK, tx);
}

void llcp_rp_frame_space_init_proc(struct proc_ctx *ctx)
{
	ctx->state = RP_FS_STATE_IDLE;
}

void llcp_rp_frame_space_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	rp_fs_execute_fsm(conn, ctx, RP_FS_EVT_RUN, param);
}

#endif /* CONFIG_BT_CTLR_FRAME_SPACE_UPDATE */
