/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * LE Shorter Connection Intervals (RCV tier), Bluetooth Core Specification v6.2:
 *   - Vol 6, Part B, Section 5.1.32  (Connection Rate Update procedure, Central)
 *   - Vol 6, Part B, Section 5.1.33  (Connection Rate Request procedure, Peripheral)
 *   - Vol 6, Part B, Section 2.4.2.57/58 (LL_CONNECTION_RATE_REQ / _IND)
 *
 * This is a new dual-role LLCP procedure that FUSES a connection-interval change
 * (applied at a synchronized instant, like Connection Update) with subrate
 * negotiation (like Connection Subrating). It is structurally the subrate FSM
 * (ull_llcp_subrate.c) with the instant/apply/collision machinery of
 * ull_llcp_conn_upd.c grafted on.
 *
 * Role asymmetry (mirrors Connection Parameter Request / Connection Update):
 *   - The Peripheral sends LL_CONNECTION_RATE_REQ to request (5.1.33).
 *   - The Central sends LL_CONNECTION_RATE_IND, which carries the Instant at
 *     which the new connection interval takes effect (5.1.32), in response to a
 *     Peripheral request or on its own Host's command.
 *   - Both roles apply the new parameters at the Instant, not on Tx ack.
 *
 * RCV scope: the on-air interval is in 125 us units but this controller keeps
 * its internal 1.25 ms grid; the PDU codec (ull_llcp_pdu.c) accepts only
 * multiples of 10 and converts by /10, so ctx->data.conn_rate.interval* are
 * already in internal 1.25 ms units here.
 *
 * NOTE: like subrating, this implements the control plane (negotiation, PDUs,
 * the instant-synchronized interval change, HCI event). The subrate
 * event-skipping in the LLL scheduler is shared with subrating and still gated
 * by TODO(subrate-sched); the interval-change half is fully functional.
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

#if defined(CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS)

/* Subrate parameter ranges, Core Spec Vol 4, Part E, Section 7.8.124 (shared
 * with Connection Subrating; SCI fuses the same subrate fields).
 */
#define CONN_RATE_SUBRATE_FACTOR_MIN          0x0001U
#define CONN_RATE_SUBRATE_FACTOR_MAX          0x01F4U /* 500 */
#define CONN_RATE_SUBRATE_FACTOR_LAT_PROD_MAX 500U

/* RCV connection-interval limits, internal 1.25 ms units (= 125 us value / 10).
 * Floor 1 == 1250 us (the RCV tier); the ECV sub-floor 375..1125 us is rejected
 * at the PDU codec, so any interval reaching the FSM is already on the grid.
 */
#define CONN_RATE_INTERVAL_MIN_UNITS          1U      /* 1250 us */
#define CONN_RATE_INTERVAL_MAX_UNITS          3200U   /* 4 s */

/* Place the instant / new connSubrateBaseEvent a few connection events into the
 * future so both devices converge on the same phase. Mirrors
 * CONN_UPDATE_INSTANT_DELTA / SUBRATE_BASE_EVENT_DELTA.
 */
#define CONN_RATE_INSTANT_DELTA               6U

/* The connection-interval collision tracker (conn_upd_curr) is only built with
 * Connection Parameter Request. When that is absent there is no CPR to be
 * mutually exclusive with; collapse the mutual-exclusion to a no-op so this
 * procedure still compiles. (CONN_UPDATE-without-CPR coexistence is a narrower
 * case; TODO(sci-collision) if a non-CPR build ever needs it.)
 */
#if defined(CONFIG_BT_CTLR_CONN_PARAM_REQ)
#define CONN_RATE_CPR_ACTIVE(conn) cpr_active_is_set(conn)
#define CONN_RATE_CPR_SET(conn)    cpr_active_set(conn)
#define CONN_RATE_CPR_RESET(conn)  cpr_active_check_and_reset(conn)
#else
#define CONN_RATE_CPR_ACTIVE(conn) false
#define CONN_RATE_CPR_SET(conn)    do { } while (0)
#define CONN_RATE_CPR_RESET(conn)  do { } while (0)
#endif /* CONFIG_BT_CTLR_CONN_PARAM_REQ */

/* LLCP Local Procedure Connection Rate FSM states */
enum {
	LP_CR_STATE_IDLE = LLCP_STATE_IDLE,
	/* Peripheral-initiated request (5.1.33) */
	LP_CR_STATE_WAIT_TX_CONN_RATE_REQ,
	LP_CR_STATE_WAIT_RX_CONN_RATE_IND,
	/* Central-initiated update (5.1.32) */
	LP_CR_STATE_WAIT_TX_CONN_RATE_IND,
	/* Both roles converge here once the binding IND is sent/received */
	LP_CR_STATE_WAIT_INSTANT,
	LP_CR_STATE_WAIT_NTF_AVAIL,
};

/* LLCP Local Procedure Connection Rate FSM events */
enum {
	LP_CR_EVT_RUN,
	LP_CR_EVT_CONN_RATE_IND,
	LP_CR_EVT_REJECT,
	LP_CR_EVT_UNKNOWN,
	LP_CR_EVT_ACK,
};

/* LLCP Remote Procedure Connection Rate FSM states */
enum {
	RP_CR_STATE_IDLE = LLCP_STATE_IDLE,
	/* Central handling a peripheral request (5.1.33 -> 5.1.32) */
	RP_CR_STATE_WAIT_RX_CONN_RATE_REQ,
	RP_CR_STATE_WAIT_TX_CONN_RATE_IND,
	RP_CR_STATE_WAIT_TX_REJECT_EXT_IND,
	/* Peripheral handling a central indication (5.1.32) */
	RP_CR_STATE_WAIT_RX_CONN_RATE_IND,
	/* Both roles converge here once the binding IND is sent/received */
	RP_CR_STATE_WAIT_INSTANT,
	RP_CR_STATE_WAIT_NTF_AVAIL,
};

/* LLCP Remote Procedure Connection Rate FSM events */
enum {
	RP_CR_EVT_RUN,
	RP_CR_EVT_CONN_RATE_REQ,
	RP_CR_EVT_CONN_RATE_IND,
	RP_CR_EVT_ACK,
};

/*
 * Shared helpers
 */

/* Validate a received LL_CONNECTION_RATE_REQ (Central side) against this
 * controller's limits. The PDU codec already enforced the RCV interval grid and
 * range, so this checks the subrate fields and the supervision-timeout vs the
 * requested (new) interval relationship. Returns true if acceptable.
 */
/* __maybe_unused: only the Central role validates a received REQ (the Peripheral
 * sends REQs / receives INDs), so this is unused in a peripheral-only build.
 */
static bool __maybe_unused conn_rate_req_acceptable(struct ll_conn *conn, struct proc_ctx *ctx)
{
	const bool is_ecv = ctx->data.conn_rate.ecv;
	const uint16_t interval_max = ctx->data.conn_rate.interval_max;
	const uint16_t sf_min = ctx->data.conn_rate.subrate_factor_min;
	const uint16_t sf_max = ctx->data.conn_rate.subrate_factor_max;
	const uint16_t max_latency = ctx->data.conn_rate.max_latency;
	const uint16_t timeout = ctx->data.conn_rate.timeout;
	/* Tier-native bounds: ECV is 125 us units [3 (375 us), 32000 (4 s)];
	 * RCV is 1.25 ms units [1 (1250 us), 3200 (4 s)].
	 */
	const uint16_t interval_min_lim =
		is_ecv ? CONFIG_BT_CTLR_SCI_ECV_INTERVAL_MIN_125US : CONN_RATE_INTERVAL_MIN_UNITS;
	const uint16_t interval_max_lim = is_ecv ? 32000U : CONN_RATE_INTERVAL_MAX_UNITS;

	ARG_UNUSED(conn);

	if ((interval_max < interval_min_lim) ||
	    (interval_max > interval_max_lim) ||
	    (ctx->data.conn_rate.interval_min > interval_max)) {
		return false;
	}

	if ((sf_min < CONN_RATE_SUBRATE_FACTOR_MIN) || (sf_max > CONN_RATE_SUBRATE_FACTOR_MAX) ||
	    (sf_min > sf_max)) {
		return false;
	}

	/* Supervision timeout (10 ms units) must exceed 2 x connInterval x
	 * SubrateFactorMin x (Max_Latency + 1); scaled by (10 ms / interval-unit) / 2:
	 * x4 for RCV (1.25 ms units), x40 for ECV (125 us units).
	 */
	if (((uint32_t)timeout * (is_ecv ? 40U : 4U)) <=
	    ((uint32_t)interval_max * sf_min * (max_latency + 1U))) {
		return false;
	}

	return true;
}

/* Choose the binding parameters for the LL_CONNECTION_RATE_IND (Central side),
 * compute the instant, and pick the new connSubrateBaseEvent. Driven by the
 * values in ctx (from the Host's request or a Peripheral's REQ, already range
 * checked). Picks the largest acceptable subrate factor for maximum power
 * saving, like subrate_ind_params_calc.
 */
/* __maybe_unused: only the Central role composes the IND, so this is unused in a
 * peripheral-only build (e.g. the sci_latency peripheral).
 */
static void __maybe_unused conn_rate_ind_params_calc(struct ll_conn *conn, struct proc_ctx *ctx)
{
	uint16_t factor = ctx->data.conn_rate.subrate_factor_max;
	uint16_t latency = ctx->data.conn_rate.max_latency;
	uint16_t cont = ctx->data.conn_rate.continuation_number;

	/* RCV: apply the requested (new) connection interval; min == max for a
	 * fixed target, otherwise take interval_max as Connection Update does.
	 */
	ctx->data.conn_rate.interval = ctx->data.conn_rate.interval_max;

	if (factor < CONN_RATE_SUBRATE_FACTOR_MIN) {
		factor = CONN_RATE_SUBRATE_FACTOR_MIN;
	}

	if (factor * (latency + 1U) > CONN_RATE_SUBRATE_FACTOR_LAT_PROD_MAX) {
		latency = (CONN_RATE_SUBRATE_FACTOR_LAT_PROD_MAX / factor) - 1U;
	}

	cont = MIN(cont, (uint16_t)(factor - 1U));

	ctx->data.conn_rate.subrate_factor = factor;
	ctx->data.conn_rate.latency = latency;
	ctx->data.conn_rate.continuation_number = cont;
	/* timeout already in ctx */

	/* No anchor-point move for RCV */
	ctx->data.conn_rate.win_size = 1U;
	ctx->data.conn_rate.win_offset_us = 0U;
	ctx->data.conn_rate.instant =
		ull_conn_event_counter(conn) + conn->lll.latency + CONN_RATE_INSTANT_DELTA;
}

/* Apply the negotiated parameters at the instant. The interval change is the
 * functional half (reprograms the radio anchor via ull_conn_update_parameters);
 * the subrate fields are stored on the connection like subrate_apply does.
 *
 * Ordering is load-bearing: ull_conn_update_parameters zeroes conn->subrate when
 * the interval changes (NOT gated on is_cu_proc), so conn->subrate.* MUST be
 * written AFTER it, in the same context.
 */
static void conn_rate_apply(struct ll_conn *conn, struct proc_ctx *ctx)
{
	uint16_t factor = ctx->data.conn_rate.subrate_factor;
	/* Core 6.2 5.1.32: at the instant, connSubrateBaseEvent is set to the Instant
	 * carried in LL_CONNECTION_RATE_IND. Both roles share that Instant (the Central
	 * computed it, the Peripheral decoded it), so deriving base_event from it keeps
	 * the subrated-event phase identical on both ends. (The IND carries no separate
	 * base-event field, per Fig 2.78.)
	 */
	uint16_t base_event = ctx->data.conn_rate.instant;

	/* Mark the tier BEFORE ull_conn_update_parameters reads the new interval so
	 * the interval-unit / tIFS / reservation branches in ull_conn.c interpret it
	 * correctly (neither is the proprietary 500 us low-latency path):
	 *   - ECV: 125 us grid + standard 150 us tIFS + a reduced CE reservation (a
	 *     sub-1.25 ms interval cannot fit a full-event slot).
	 *   - RCV: 1.25 ms grid + 150 us tIFS + the full reservation (unchanged).
	 */
	if (ctx->data.conn_rate.ecv) {
		conn->lll.ecv = 1U;
		conn->lll.rcv = 0U;
		conn->lll.reduced_ce = 1U;
	} else {
		conn->lll.rcv = 1U;
		conn->lll.ecv = 0U;
		conn->lll.reduced_ce = 0U;
	}

	/* Change the connection interval at the instant (is_cu_proc=true: this is
	 * an explicit Host/peer-driven update, not an internal one). This sets
	 * conn->supervision_timeout too, so we do not re-write it below.
	 */
	ull_conn_update_parameters(conn, 1U, ctx->data.conn_rate.win_size,
				   ctx->data.conn_rate.win_offset_us,
				   ctx->data.conn_rate.interval, ctx->data.conn_rate.latency,
				   ctx->data.conn_rate.timeout, ctx->data.conn_rate.instant);

	/* Normalise connSubrateBaseEvent to the most recent subrated event at or
	 * before now, preserving (base mod factor) (as in subrate_apply).
	 */
	if (factor > 1U) {
		int16_t event_diff = (int16_t)(ull_conn_event_counter(conn) - base_event);
		int16_t subrate_events_diff = event_diff / (int16_t)factor;

		base_event += (uint16_t)(factor * subrate_events_diff);
	}

	conn->subrate.factor = factor;
	conn->subrate.base_event = base_event;
	conn->subrate.continuation_number = ctx->data.conn_rate.continuation_number;
	conn->subrate.peripheral_latency = ctx->data.conn_rate.latency;
	conn->subrate.cont_num_left = 0U;
}

static void conn_rate_ntf(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct node_rx_pdu *ntf;
	struct node_rx_conn_rate_change *cr;

	ntf = ctx->node_ref.rx;
	ctx->node_ref.rx = NULL;
	LL_ASSERT(ntf);

	ntf->hdr.type = NODE_RX_TYPE_CONN_RATE_CHANGE;
	ntf->hdr.handle = conn->lll.handle;

	cr = (struct node_rx_conn_rate_change *)ntf->pdu;
	cr->status = ctx->data.conn_rate.error;
	/* Report the interval in canonical 125 us units so the HCI event encoder is
	 * tier-agnostic: RCV stores 1.25 ms units (x10), ECV stores 125 us units (x1).
	 */
	cr->conn_interval = conn->lll.interval * (conn->lll.ecv ? 1U : 10U);
	cr->subrate_factor = conn->subrate.factor;
	cr->peripheral_latency = conn->subrate.peripheral_latency;
	cr->continuation_number = conn->subrate.continuation_number;
	cr->supervision_timeout = conn->supervision_timeout;

	ll_rx_put_sched(ntf->hdr.link, ntf);
}

/*
 * LLCP Local Procedure Connection Rate FSM
 */

static void lp_cr_tx(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t opcode)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT(tx);

	pdu = (struct pdu_data *)tx->pdu;

	switch (opcode) {
#if defined(CONFIG_BT_PERIPHERAL)
	case PDU_DATA_LLCTRL_TYPE_CONN_RATE_REQ:
		llcp_pdu_encode_conn_rate_req(ctx, pdu);
		break;
#endif /* CONFIG_BT_PERIPHERAL */
#if defined(CONFIG_BT_CENTRAL)
	case PDU_DATA_LLCTRL_TYPE_CONN_RATE_IND:
		llcp_pdu_encode_conn_rate_ind(ctx, pdu);
		break;
#endif /* CONFIG_BT_CENTRAL */
	default:
		LL_ASSERT(0);
		break;
	}

	ctx->tx_opcode = pdu->llctrl.opcode;

	llcp_tx_enqueue(conn, tx);

	/* Restart procedure response timeout timer */
	llcp_lr_prt_restart(conn);
}

static void lp_cr_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_lr_prt_stop(conn);
	CONN_RATE_CPR_RESET(conn);
	llcp_rr_set_incompat(conn, INCOMPAT_NO_COLLISION);
	llcp_lr_complete(conn);
	ctx->state = LP_CR_STATE_IDLE;
}

static void lp_cr_ntf_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	conn_rate_ntf(conn, ctx);
	lp_cr_complete(conn, ctx);
}

/* Apply at the instant, then notify the Host (or complete silently). Shared by
 * both roles' WAIT_INSTANT handling. */
static void lp_cr_check_instant(struct ll_conn *conn, struct proc_ctx *ctx)
{
	if (is_instant_reached_or_passed(ctx->data.conn_rate.instant,
					 ull_conn_event_counter(conn))) {
		conn_rate_apply(conn, ctx);
		ctx->data.conn_rate.error = BT_HCI_ERR_SUCCESS;

		if (llcp_ntf_alloc_is_available()) {
			ctx->node_ref.rx = llcp_ntf_alloc();
			lp_cr_ntf_complete(conn, ctx);
		} else {
			ctx->state = LP_CR_STATE_WAIT_NTF_AVAIL;
		}
	}
}

#if defined(CONFIG_BT_PERIPHERAL)
static void lp_cr_send_conn_rate_req(struct ll_conn *conn, struct proc_ctx *ctx)
{
	/* Like the local Connection Parameter Request: claim the interval-change
	 * collision slot and mark a local instant-procedure as pending.
	 */
	if (CONN_RATE_CPR_ACTIVE(conn) || llcp_lr_ispaused(conn) ||
	    llcp_rr_get_collision(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = LP_CR_STATE_WAIT_TX_CONN_RATE_REQ;
	} else {
		llcp_rr_set_incompat(conn, INCOMPAT_RESOLVABLE);
		CONN_RATE_CPR_SET(conn);
		lp_cr_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_CONN_RATE_REQ);
		ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_CONN_RATE_IND;
		ctx->state = LP_CR_STATE_WAIT_RX_CONN_RATE_IND;
	}
}
#endif /* CONFIG_BT_PERIPHERAL */

#if defined(CONFIG_BT_CENTRAL)
static void lp_cr_send_conn_rate_ind(struct ll_conn *conn, struct proc_ctx *ctx)
{
	/* Mutually exclusive with a Connection Update / Connection Parameter
	 * Request (both touch the connection interval at an instant).
	 */
	if (CONN_RATE_CPR_ACTIVE(conn) || llcp_lr_ispaused(conn) ||
	    llcp_rr_get_collision(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = LP_CR_STATE_WAIT_TX_CONN_RATE_IND;
	} else {
		llcp_rr_set_incompat(conn, INCOMPAT_RESOLVABLE);
		CONN_RATE_CPR_SET(conn);
		conn_rate_ind_params_calc(conn, ctx);
		lp_cr_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_CONN_RATE_IND);
		ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_UNUSED;
		ctx->state = LP_CR_STATE_WAIT_INSTANT;
	}
}
#endif /* CONFIG_BT_CENTRAL */

static void lp_cr_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	ARG_UNUSED(param);

	switch (evt) {
	case LP_CR_EVT_RUN:
		switch (conn->lll.role) {
#if defined(CONFIG_BT_CENTRAL)
		case BT_HCI_ROLE_CENTRAL:
			lp_cr_send_conn_rate_ind(conn, ctx);
			break;
#endif /* CONFIG_BT_CENTRAL */
#if defined(CONFIG_BT_PERIPHERAL)
		case BT_HCI_ROLE_PERIPHERAL:
			lp_cr_send_conn_rate_req(conn, ctx);
			break;
#endif /* CONFIG_BT_PERIPHERAL */
		default:
			LL_ASSERT(0);
			break;
		}
		break;
	default:
		break;
	}
}

#if defined(CONFIG_BT_PERIPHERAL)
static void lp_cr_st_wait_tx_conn_rate_req(struct ll_conn *conn, struct proc_ctx *ctx,
					   uint8_t evt, void *param)
{
	ARG_UNUSED(param);

	switch (evt) {
	case LP_CR_EVT_RUN:
		lp_cr_send_conn_rate_req(conn, ctx);
		break;
	default:
		break;
	}
}

static void lp_cr_st_wait_rx_conn_rate_ind(struct ll_conn *conn, struct proc_ctx *ctx,
					   uint8_t evt, void *param)
{
	struct pdu_data *pdu = (struct pdu_data *)param;

	switch (evt) {
	case LP_CR_EVT_CONN_RATE_IND:
		llcp_pdu_decode_conn_rate_ind(ctx, pdu);
		if (ctx->data.conn_rate.error != BT_HCI_ERR_SUCCESS) {
			/* Out-of-range IND: notify failure and complete */
			goto ntf_error;
		}
		/* Wait for the instant, then apply (5.1.32) */
		ctx->state = LP_CR_STATE_WAIT_INSTANT;
		lp_cr_check_instant(conn, ctx);
		break;
	case LP_CR_EVT_UNKNOWN:
		/* Peer does not support SCI (a page-1 feature, so nothing to
		 * unmask in the page-0 used-features set).
		 */
		ctx->data.conn_rate.error = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE;
		goto ntf_error;
	case LP_CR_EVT_REJECT:
		ctx->data.conn_rate.error = pdu->llctrl.reject_ext_ind.error_code;
		goto ntf_error;
	default:
		break;
	}
	return;

ntf_error:
	if (llcp_ntf_alloc_is_available()) {
		ctx->node_ref.rx = llcp_ntf_alloc();
		lp_cr_ntf_complete(conn, ctx);
	} else {
		ctx->state = LP_CR_STATE_WAIT_NTF_AVAIL;
	}
}
#endif /* CONFIG_BT_PERIPHERAL */

#if defined(CONFIG_BT_CENTRAL)
static void lp_cr_st_wait_tx_conn_rate_ind(struct ll_conn *conn, struct proc_ctx *ctx,
					   uint8_t evt, void *param)
{
	ARG_UNUSED(param);

	switch (evt) {
	case LP_CR_EVT_RUN:
		lp_cr_send_conn_rate_ind(conn, ctx);
		break;
	default:
		break;
	}
}
#endif /* CONFIG_BT_CENTRAL */

static void lp_cr_st_wait_instant(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				  void *param)
{
	ARG_UNUSED(param);

	switch (evt) {
	case LP_CR_EVT_RUN:
		lp_cr_check_instant(conn, ctx);
		break;
	default:
		break;
	}
}

static void lp_cr_st_wait_ntf_avail(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				    void *param)
{
	ARG_UNUSED(param);

	switch (evt) {
	case LP_CR_EVT_RUN:
		if (llcp_ntf_alloc_is_available()) {
			ctx->node_ref.rx = llcp_ntf_alloc();
			lp_cr_ntf_complete(conn, ctx);
		}
		break;
	default:
		break;
	}
}

static void lp_cr_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
			      void *param)
{
	switch (ctx->state) {
	case LP_CR_STATE_IDLE:
		lp_cr_st_idle(conn, ctx, evt, param);
		break;
#if defined(CONFIG_BT_PERIPHERAL)
	case LP_CR_STATE_WAIT_TX_CONN_RATE_REQ:
		lp_cr_st_wait_tx_conn_rate_req(conn, ctx, evt, param);
		break;
	case LP_CR_STATE_WAIT_RX_CONN_RATE_IND:
		lp_cr_st_wait_rx_conn_rate_ind(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_PERIPHERAL */
#if defined(CONFIG_BT_CENTRAL)
	case LP_CR_STATE_WAIT_TX_CONN_RATE_IND:
		lp_cr_st_wait_tx_conn_rate_ind(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_CENTRAL */
	case LP_CR_STATE_WAIT_INSTANT:
		lp_cr_st_wait_instant(conn, ctx, evt, param);
		break;
	case LP_CR_STATE_WAIT_NTF_AVAIL:
		lp_cr_st_wait_ntf_avail(conn, ctx, evt, param);
		break;
	default:
		LL_ASSERT(0);
		break;
	}
}

void llcp_lp_conn_rate_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_CONN_RATE_IND:
		lp_cr_execute_fsm(conn, ctx, LP_CR_EVT_CONN_RATE_IND, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_UNKNOWN_RSP:
		lp_cr_execute_fsm(conn, ctx, LP_CR_EVT_UNKNOWN, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND:
		lp_cr_execute_fsm(conn, ctx, LP_CR_EVT_REJECT, pdu);
		break;
	default:
		/* Invalid PDU received, terminate connection */
		conn->llcp_terminate.reason_final = BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
		lp_cr_complete(conn, ctx);
		break;
	}
}

void llcp_lp_conn_rate_tx_ack(struct ll_conn *conn, struct proc_ctx *ctx, struct node_tx *tx)
{
	lp_cr_execute_fsm(conn, ctx, LP_CR_EVT_ACK, tx);
}

void llcp_lp_conn_rate_init_proc(struct proc_ctx *ctx)
{
	ctx->state = LP_CR_STATE_IDLE;
}

void llcp_lp_conn_rate_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	lp_cr_execute_fsm(conn, ctx, LP_CR_EVT_RUN, param);
}

/*
 * LLCP Remote Procedure Connection Rate FSM
 */

static void rp_cr_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_rr_prt_stop(conn);
	/* Release the interval-change slot we claimed; the local incompat is not
	 * ours to clear (remote procedure).
	 */
	CONN_RATE_CPR_RESET(conn);
	llcp_rr_complete(conn);
	ctx->state = RP_CR_STATE_IDLE;
}

static void rp_cr_ntf_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	conn_rate_ntf(conn, ctx);
	rp_cr_complete(conn, ctx);
}

static void rp_cr_check_instant(struct ll_conn *conn, struct proc_ctx *ctx)
{
	if (is_instant_reached_or_passed(ctx->data.conn_rate.instant,
					 ull_conn_event_counter(conn))) {
		conn_rate_apply(conn, ctx);
		ctx->data.conn_rate.error = BT_HCI_ERR_SUCCESS;

		if (llcp_ntf_alloc_is_available()) {
			ctx->node_ref.rx = llcp_ntf_alloc();
			rp_cr_ntf_complete(conn, ctx);
		} else {
			ctx->state = RP_CR_STATE_WAIT_NTF_AVAIL;
		}
	}
}

#if defined(CONFIG_BT_CENTRAL)
static void rp_cr_tx(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t opcode)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT(tx);

	pdu = (struct pdu_data *)tx->pdu;

	switch (opcode) {
	case PDU_DATA_LLCTRL_TYPE_CONN_RATE_IND:
		llcp_pdu_encode_conn_rate_ind(ctx, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND:
		llcp_pdu_encode_reject_ext_ind(pdu, PDU_DATA_LLCTRL_TYPE_CONN_RATE_REQ,
					       ctx->data.conn_rate.error);
		break;
	default:
		LL_ASSERT(0);
		break;
	}

	ctx->tx_opcode = pdu->llctrl.opcode;

	llcp_tx_enqueue(conn, tx);

	llcp_rr_prt_restart(conn);
}

static void rp_cr_send_reject_ext_ind(struct ll_conn *conn, struct proc_ctx *ctx)
{
	if (llcp_rr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = RP_CR_STATE_WAIT_TX_REJECT_EXT_IND;
	} else {
		rp_cr_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND);
		rp_cr_complete(conn, ctx);
	}
}

static void rp_cr_send_conn_rate_ind(struct ll_conn *conn, struct proc_ctx *ctx)
{
	/* Remote procedure: the collision arbiter (rr_st_idle) already decided to
	 * run us; we only claim the interval-change slot (cpr_active), we do NOT
	 * set the local incompat (that is the local procedure's signal).
	 */
	if (CONN_RATE_CPR_ACTIVE(conn) || llcp_rr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = RP_CR_STATE_WAIT_TX_CONN_RATE_IND;
	} else {
		CONN_RATE_CPR_SET(conn);
		conn_rate_ind_params_calc(conn, ctx);
		rp_cr_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_CONN_RATE_IND);
		ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_UNUSED;
		ctx->state = RP_CR_STATE_WAIT_INSTANT;
	}
}

static void rp_cr_st_wait_rx_conn_rate_req(struct ll_conn *conn, struct proc_ctx *ctx,
					   uint8_t evt, void *param)
{
	switch (evt) {
	case RP_CR_EVT_CONN_RATE_REQ:
		llcp_pdu_decode_conn_rate_req(ctx, (struct pdu_data *)param);
		if (ctx->data.conn_rate.error != BT_HCI_ERR_SUCCESS) {
			/* Out-of-range / ECV interval in the REQ */
			rp_cr_send_reject_ext_ind(conn, ctx);
		} else if (!ll_feat_sci_host_supported()) {
			/* Core 6.2 5.1.33 -> 5.1.20: the Central shall reject a
			 * Peripheral request if its OWN Host has not set the SCI
			 * (Host Support) bit. (Checking the peer's bit would also wrongly
			 * require the page-1 exchange to have completed first.)
			 */
			ctx->data.conn_rate.error = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE;
			rp_cr_send_reject_ext_ind(conn, ctx);
		} else if (conn_rate_req_acceptable(conn, ctx)) {
			rp_cr_send_conn_rate_ind(conn, ctx);
		} else {
			ctx->data.conn_rate.error = BT_HCI_ERR_UNSUPP_LL_PARAM_VAL;
			rp_cr_send_reject_ext_ind(conn, ctx);
		}
		break;
	default:
		break;
	}
}

static void rp_cr_st_wait_tx_conn_rate_ind(struct ll_conn *conn, struct proc_ctx *ctx,
					   uint8_t evt, void *param)
{
	ARG_UNUSED(param);

	switch (evt) {
	case RP_CR_EVT_RUN:
		rp_cr_send_conn_rate_ind(conn, ctx);
		break;
	default:
		break;
	}
}

static void rp_cr_st_wait_tx_reject_ext_ind(struct ll_conn *conn, struct proc_ctx *ctx,
					    uint8_t evt, void *param)
{
	ARG_UNUSED(param);

	switch (evt) {
	case RP_CR_EVT_RUN:
		rp_cr_send_reject_ext_ind(conn, ctx);
		break;
	default:
		break;
	}
}
#endif /* CONFIG_BT_CENTRAL */

#if defined(CONFIG_BT_PERIPHERAL)
static void rp_cr_st_wait_rx_conn_rate_ind(struct ll_conn *conn, struct proc_ctx *ctx,
					   uint8_t evt, void *param)
{
	switch (evt) {
	case RP_CR_EVT_CONN_RATE_IND:
		llcp_pdu_decode_conn_rate_ind(ctx, (struct pdu_data *)param);
		if (ctx->data.conn_rate.error != BT_HCI_ERR_SUCCESS) {
			conn->llcp_terminate.reason_final = BT_HCI_ERR_UNSUPP_LL_PARAM_VAL;
			rp_cr_complete(conn, ctx);
			break;
		}
		if (!is_instant_not_passed(ctx->data.conn_rate.instant,
					   ull_conn_event_counter(conn))) {
			/* Instant already passed (5.1.32) */
			conn->llcp_terminate.reason_final = BT_HCI_ERR_INSTANT_PASSED;
			rp_cr_complete(conn, ctx);
			break;
		}
		ctx->state = RP_CR_STATE_WAIT_INSTANT;
		rp_cr_check_instant(conn, ctx);
		break;
	default:
		break;
	}
}
#endif /* CONFIG_BT_PERIPHERAL */

static void rp_cr_st_wait_instant(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				  void *param)
{
	ARG_UNUSED(param);

	switch (evt) {
	case RP_CR_EVT_RUN:
		rp_cr_check_instant(conn, ctx);
		break;
	default:
		break;
	}
}

static void rp_cr_st_wait_ntf_avail(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				    void *param)
{
	ARG_UNUSED(param);

	switch (evt) {
	case RP_CR_EVT_RUN:
		if (llcp_ntf_alloc_is_available()) {
			ctx->node_ref.rx = llcp_ntf_alloc();
			rp_cr_ntf_complete(conn, ctx);
		}
		break;
	default:
		break;
	}
}

static void rp_cr_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	ARG_UNUSED(param);

	switch (evt) {
	case RP_CR_EVT_RUN:
		switch (conn->lll.role) {
#if defined(CONFIG_BT_CENTRAL)
		case BT_HCI_ROLE_CENTRAL:
			ctx->state = RP_CR_STATE_WAIT_RX_CONN_RATE_REQ;
			break;
#endif /* CONFIG_BT_CENTRAL */
#if defined(CONFIG_BT_PERIPHERAL)
		case BT_HCI_ROLE_PERIPHERAL:
			ctx->state = RP_CR_STATE_WAIT_RX_CONN_RATE_IND;
			break;
#endif /* CONFIG_BT_PERIPHERAL */
		default:
			LL_ASSERT(0);
			break;
		}
		break;
	default:
		break;
	}
}

static void rp_cr_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
			      void *param)
{
	switch (ctx->state) {
	case RP_CR_STATE_IDLE:
		rp_cr_st_idle(conn, ctx, evt, param);
		break;
#if defined(CONFIG_BT_CENTRAL)
	case RP_CR_STATE_WAIT_RX_CONN_RATE_REQ:
		rp_cr_st_wait_rx_conn_rate_req(conn, ctx, evt, param);
		break;
	case RP_CR_STATE_WAIT_TX_CONN_RATE_IND:
		rp_cr_st_wait_tx_conn_rate_ind(conn, ctx, evt, param);
		break;
	case RP_CR_STATE_WAIT_TX_REJECT_EXT_IND:
		rp_cr_st_wait_tx_reject_ext_ind(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_CENTRAL */
#if defined(CONFIG_BT_PERIPHERAL)
	case RP_CR_STATE_WAIT_RX_CONN_RATE_IND:
		rp_cr_st_wait_rx_conn_rate_ind(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_PERIPHERAL */
	case RP_CR_STATE_WAIT_INSTANT:
		rp_cr_st_wait_instant(conn, ctx, evt, param);
		break;
	case RP_CR_STATE_WAIT_NTF_AVAIL:
		rp_cr_st_wait_ntf_avail(conn, ctx, evt, param);
		break;
	default:
		LL_ASSERT(0);
		break;
	}
}

void llcp_rp_conn_rate_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_CONN_RATE_REQ:
		rp_cr_execute_fsm(conn, ctx, RP_CR_EVT_CONN_RATE_REQ, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_CONN_RATE_IND:
		rp_cr_execute_fsm(conn, ctx, RP_CR_EVT_CONN_RATE_IND, pdu);
		break;
	default:
		/* Invalid PDU received, terminate connection */
		conn->llcp_terminate.reason_final = BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
		rp_cr_complete(conn, ctx);
		break;
	}
}

void llcp_rp_conn_rate_tx_ack(struct ll_conn *conn, struct proc_ctx *ctx, struct node_tx *tx)
{
	rp_cr_execute_fsm(conn, ctx, RP_CR_EVT_ACK, tx);
}

void llcp_rp_conn_rate_init_proc(struct proc_ctx *ctx)
{
	ctx->state = RP_CR_STATE_IDLE;
}

void llcp_rp_conn_rate_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	rp_cr_execute_fsm(conn, ctx, RP_CR_EVT_RUN, param);
}

#endif /* CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS */
