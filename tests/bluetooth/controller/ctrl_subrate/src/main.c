/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/ztest.h>

#define ULL_LLCP_UNITTEST

#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>
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
#include "lll/lll_df_types.h"
#include "lll_conn.h"
#include "lll_conn_iso.h"

#include "ull_tx_queue.h"

#include "isoal.h"
#include "ull_iso_types.h"
#include "ull_conn_iso_types.h"
#include "ull_conn_types.h"
#include "ull_llcp.h"
#include "ull_conn_internal.h"
#include "ull_llcp_internal.h"

#include "helper_pdu.h"
#include "helper_util.h"

/* Connection defaults */
#define INTERVAL            6U   /* 7.5 ms */
#define SUPERVISION_TIMEOUT 100U /* 1 s (multiple of 10 ms) */

/* Requested / negotiated subrating parameters */
#define SUBRATE_MIN  1U
#define SUBRATE_MAX  4U
#define MAX_LATENCY  0U
#define CONT_NUMBER  0U
#define SUBRATE_TO   SUPERVISION_TIMEOUT

static struct ll_conn conn;

static void subrate_setup(void *data)
{
	test_setup(&conn);

	struct lll_conn *lll = &conn.lll;

	lll->interval = INTERVAL;
	lll->latency = 0U;
	lll->event_counter = 0U;
	conn.supervision_timeout = SUPERVISION_TIMEOUT;

	/* test_setup() does not clear subrating working state; reset it so each
	 * test starts unsubrated (the static conn is reused across the suite).
	 */
	conn.subrate.factor = 0U;
	conn.subrate.base_event = 0U;
	conn.subrate.continuation_number = 0U;
	conn.subrate.peripheral_latency = 0U;
	conn.subrate.cont_num_left = 0U;

	/* Allow a Central to grant subrate factors > 1 to a Peripheral request. */
	conn.subrate.acc_factor_min = 1U;
	conn.subrate.acc_factor_max = 16U;
	conn.subrate.acc_max_latency = 0U;
	conn.subrate.acc_continuation_number = 0U;
	conn.subrate.acc_supervision_timeout = 0x0C80U;

	/* The Central accepts Peripheral requests only if its own Host has enabled
	 * Connection Subrating (5.1.20); set it so the accept-path tests run.
	 */
	ll_set_host_feature(BT_LE_FEAT_BIT_CONN_SUBRATING_HOST_SUPP, 1U);
}

/* Expected LL_SUBRATE_REQ emitted by a Peripheral initiator */
static struct pdu_data_llctrl_subrate_req exp_req = {
	.subrate_factor_min = SUBRATE_MIN,
	.subrate_factor_max = SUBRATE_MAX,
	.max_latency = MAX_LATENCY,
	.continuation_number = CONT_NUMBER,
	.timeout = SUBRATE_TO,
};

/* Expected LL_SUBRATE_IND emitted by a Central (subrate_base_event is not
 * checked - the Central computes it dynamically).
 */
static struct pdu_data_llctrl_subrate_ind exp_ind = {
	.subrate_factor = SUBRATE_MAX,
	.latency = MAX_LATENCY,
	.continuation_number = CONT_NUMBER,
	.timeout = SUBRATE_TO,
};

/* LL_SUBRATE_IND injected by a peer Central */
static struct pdu_data_llctrl_subrate_ind in_ind = {
	.subrate_factor = SUBRATE_MAX,
	.subrate_base_event = 6U,
	.latency = MAX_LATENCY,
	.continuation_number = CONT_NUMBER,
	.timeout = SUBRATE_TO,
};

/* LL_SUBRATE_REQ injected by a peer Peripheral */
static struct pdu_data_llctrl_subrate_req in_req = {
	.subrate_factor_min = SUBRATE_MIN,
	.subrate_factor_max = SUBRATE_MAX,
	.max_latency = MAX_LATENCY,
	.continuation_number = CONT_NUMBER,
	.timeout = SUBRATE_TO,
};

/* Expected HCI LE Subrate Change event data */
static struct node_rx_subrate_change exp_ntf = {
	.status = BT_HCI_ERR_SUCCESS,
	.subrate_factor = SUBRATE_MAX,
	.peripheral_latency = MAX_LATENCY,
	.continuation_number = CONT_NUMBER,
	.supervision_timeout = SUBRATE_TO,
};

/*
 * Central-initiated Connection Subrate Update (Core Spec 5.1.19).
 * Host requests subrating; Central sends LL_SUBRATE_IND and applies the new
 * parameters once it is acknowledged.
 */
ZTEST(subrate_central_loc, test_subrate_central_initiated)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* The Central may only initiate once the peer advertises Connection
	 * Subrating (Host Support).
	 */
	conn.llcp.fex.features_peer |= BIT64(BT_LE_FEAT_BIT_CONN_SUBRATING_HOST_SUPP);

	err = ull_cp_subrate_req(&conn, SUBRATE_MIN, SUBRATE_MAX, MAX_LATENCY, CONT_NUMBER,
				 SUBRATE_TO);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, "err %u", err);

	event_prepare(&conn);
	lt_rx(LL_SUBRATE_IND, &conn, &tx, &exp_ind);
	lt_rx_q_is_empty(&conn);
	/* Acknowledge the LL_SUBRATE_IND within the event: params apply + notify */
	event_tx_ack(&conn, tx);
	event_done(&conn);
	ull_cp_release_tx(&conn, tx);

	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	zassert_equal(conn.subrate.factor, SUBRATE_MAX, "factor %u", conn.subrate.factor);

	release_ntf(ntf);
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(), "Free CTX buffers %d",
		      llcp_ctx_buffers_free());
}

/* Run one Central-initiated update with the event counter at 'event' and return
 * the SubrateBaseEvent sent in the LL_SUBRATE_IND.
 */
static uint16_t central_ind_base_event(uint16_t event, uint16_t factor)
{
	struct pdu_data_llctrl_subrate_ind ind = exp_ind;
	struct node_rx_subrate_change change = exp_ntf;
	struct node_rx_pdu *ntf;
	struct node_tx *tx;
	uint16_t base_event;
	uint8_t err;

	ind.subrate_factor = factor;
	change.subrate_factor = factor;
	conn.lll.event_counter = event;

	err = ull_cp_subrate_req(&conn, factor, factor, MAX_LATENCY, CONT_NUMBER, SUBRATE_TO);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, "err %u", err);

	event_prepare(&conn);
	lt_rx(LL_SUBRATE_IND, &conn, &tx, &ind);
	lt_rx_q_is_empty(&conn);
	base_event = sys_le16_to_cpu(
		((struct pdu_data *)tx->pdu)->llctrl.subrate_ind.subrate_base_event);
	event_tx_ack(&conn, tx);
	event_done(&conn);
	ull_cp_release_tx(&conn, tx);

	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, &change);
	ut_rx_q_is_empty();
	release_ntf(ntf);

	return base_event;
}

/*
 * SubrateBaseEvent near a quarter boundary of the event counter (Core Spec
 * 5.1.19): bits 15-14 must match connEventCount when the PDU is queued, and
 * the value must still give the intended phase (event + 6, modulo the factor).
 * Factor 5 does not divide 65536, so a wrong step back would change the phase.
 */
ZTEST(subrate_central_loc, test_subrate_central_base_event_wrap)
{
	const uint16_t events[] = {
		0x3FF9U, 0x3FFAU, 0x3FFBU, 0x3FFEU, 0x3FFFU, 0x4000U,
		0x7FFDU, 0xBFFCU, 0xFFF9U, 0xFFFAU, 0xFFFEU, 0xFFFFU, 0x0000U,
	};
	const uint16_t factor = 5U;

	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);
	conn.llcp.fex.features_peer |= BIT64(BT_LE_FEAT_BIT_CONN_SUBRATING_HOST_SUPP);

	for (size_t i = 0; i < ARRAY_SIZE(events); i++) {
		const uint16_t event = events[i];
		const uint16_t base_event = central_ind_base_event(event, factor);
		/* Distance back from the default placement, 6 events ahead */
		const uint16_t back = (uint16_t)(event + 6U - base_event);

		zassert_equal(base_event & 0xC000U, event & 0xC000U,
			      "event 0x%04x: base 0x%04x in another quarter", event, base_event);
		zassert_equal(back % factor, 0U, "event 0x%04x: base 0x%04x changes the phase",
			      event, base_event);
		zassert_true(back < 6U + factor, "event 0x%04x: base 0x%04x stepped too far",
			     event, base_event);
	}

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(), "Free CTX buffers %d",
		      llcp_ctx_buffers_free());
}

/*
 * Peripheral-initiated Connection Subrate Request (Core Spec 5.1.20).
 * Host requests subrating; Peripheral sends LL_SUBRATE_REQ and applies the
 * parameters in the Central's LL_SUBRATE_IND.
 */
ZTEST(subrate_periph_loc, test_subrate_periph_initiated)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	err = ull_cp_subrate_req(&conn, SUBRATE_MIN, SUBRATE_MAX, MAX_LATENCY, CONT_NUMBER,
				 SUBRATE_TO);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, "err %u", err);

	event_prepare(&conn);
	lt_rx(LL_SUBRATE_REQ, &conn, &tx, &exp_req);
	lt_rx_q_is_empty(&conn);
	event_done(&conn);
	ull_cp_release_tx(&conn, tx);

	/* Central responds with LL_SUBRATE_IND */
	event_prepare(&conn);
	lt_tx(LL_SUBRATE_IND, &conn, &in_ind);
	event_done(&conn);

	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	zassert_equal(conn.subrate.factor, SUBRATE_MAX, "factor %u", conn.subrate.factor);

	release_ntf(ntf);
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(), "Free CTX buffers %d",
		      llcp_ctx_buffers_free());
}

/*
 * Central receives LL_SUBRATE_REQ from the Peripheral, accepts it, and responds
 * with LL_SUBRATE_IND (Core Spec 5.1.20 -> 5.1.19).
 */
ZTEST(subrate_central_rem, test_subrate_central_responds)
{
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	event_prepare(&conn);
	lt_tx(LL_SUBRATE_REQ, &conn, &in_req);
	event_done(&conn);

	event_prepare(&conn);
	lt_rx(LL_SUBRATE_IND, &conn, &tx, &exp_ind);
	lt_rx_q_is_empty(&conn);
	event_tx_ack(&conn, tx);
	event_done(&conn);
	ull_cp_release_tx(&conn, tx);

	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	zassert_equal(conn.subrate.factor, SUBRATE_MAX, "factor %u", conn.subrate.factor);

	release_ntf(ntf);
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(), "Free CTX buffers %d",
		      llcp_ctx_buffers_free());
}

/*
 * Peripheral receives LL_SUBRATE_IND from the Central and applies it
 * immediately (Core Spec 5.1.19).
 */
ZTEST(subrate_periph_rem, test_subrate_periph_applies)
{
	struct node_rx_pdu *ntf;

	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	event_prepare(&conn);
	lt_tx(LL_SUBRATE_IND, &conn, &in_ind);
	event_done(&conn);

	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	zassert_equal(conn.subrate.factor, SUBRATE_MAX, "factor %u", conn.subrate.factor);

	release_ntf(ntf);
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(), "Free CTX buffers %d",
		      llcp_ctx_buffers_free());
}

/*
 * Central receives an unacceptable LL_SUBRATE_REQ (Max_Latency exceeds the
 * acceptable value) and rejects it with LL_REJECT_EXT_IND; subrating is left
 * unchanged (Core Spec 5.1.20).
 */
ZTEST(subrate_central_rem, test_subrate_central_rejects)
{
	struct node_tx *tx;
	struct pdu_data_llctrl_subrate_req in_req_unacceptable = {
		.subrate_factor_min = SUBRATE_MIN,
		.subrate_factor_max = SUBRATE_MAX,
		.max_latency = 5U, /* > acc_max_latency (0) -> not acceptable */
		.continuation_number = CONT_NUMBER,
		.timeout = SUBRATE_TO,
	};
	struct pdu_data_llctrl_reject_ext_ind reject = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ,
		.error_code = BT_HCI_ERR_UNSUPP_LL_PARAM_VAL,
	};

	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	event_prepare(&conn);
	lt_tx(LL_SUBRATE_REQ, &conn, &in_req_unacceptable);
	event_done(&conn);

	event_prepare(&conn);
	lt_rx(LL_REJECT_EXT_IND, &conn, &tx, &reject);
	lt_rx_q_is_empty(&conn);
	event_done(&conn);
	ull_cp_release_tx(&conn, tx);

	/* No notification, subrating unchanged */
	ut_rx_q_is_empty();
	zassert_equal(conn.subrate.factor, 0U, "factor %u", conn.subrate.factor);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(), "Free CTX buffers %d",
		      llcp_ctx_buffers_free());
}

/*
 * A Central whose own Host has not set the Connection Subrating (Host Support)
 * feature bit rejects an otherwise-acceptable Peripheral request (5.1.20).
 */
ZTEST(subrate_central_rem, test_subrate_central_no_host_support)
{
	struct node_tx *tx;
	struct pdu_data_llctrl_reject_ext_ind reject = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ,
		.error_code = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE,
	};

	/* Clear the local Host Support bit that subrate_setup enables. */
	ll_set_host_feature(BT_LE_FEAT_BIT_CONN_SUBRATING_HOST_SUPP, 0U);

	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	event_prepare(&conn);
	lt_tx(LL_SUBRATE_REQ, &conn, &in_req);
	event_done(&conn);

	event_prepare(&conn);
	lt_rx(LL_REJECT_EXT_IND, &conn, &tx, &reject);
	lt_rx_q_is_empty(&conn);
	event_done(&conn);
	ull_cp_release_tx(&conn, tx);

	ut_rx_q_is_empty();

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(), "Free CTX buffers %d",
		      llcp_ctx_buffers_free());
}

/*
 * Peripheral requests subrating but the Central does not support the feature
 * and replies LL_UNKNOWN_RSP; the feature is unmasked locally and the change is
 * reported with BT_HCI_ERR_UNSUPP_REMOTE_FEATURE (5.1.20).
 */
ZTEST(subrate_periph_loc, test_subrate_periph_unknown)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data_llctrl_unknown_rsp unknown = {
		.type = PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ,
	};
	struct node_rx_subrate_change exp_unsupp = {
		.status = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE,
		.subrate_factor = 0U,
		.peripheral_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout = SUPERVISION_TIMEOUT,
	};

	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	err = ull_cp_subrate_req(&conn, SUBRATE_MIN, SUBRATE_MAX, MAX_LATENCY, CONT_NUMBER,
				 SUBRATE_TO);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, "err %u", err);

	event_prepare(&conn);
	lt_rx(LL_SUBRATE_REQ, &conn, &tx, &exp_req);
	lt_rx_q_is_empty(&conn);
	event_done(&conn);
	ull_cp_release_tx(&conn, tx);

	event_prepare(&conn);
	lt_tx(LL_UNKNOWN_RSP, &conn, &unknown);
	event_done(&conn);

	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, &exp_unsupp);
	ut_rx_q_is_empty();

	release_ntf(ntf);
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(), "Free CTX buffers %d",
		      llcp_ctx_buffers_free());
}

ZTEST_SUITE(subrate_central_loc, NULL, NULL, subrate_setup, NULL, NULL);
ZTEST_SUITE(subrate_periph_loc, NULL, NULL, subrate_setup, NULL, NULL);
ZTEST_SUITE(subrate_central_rem, NULL, NULL, subrate_setup, NULL, NULL);
ZTEST_SUITE(subrate_periph_rem, NULL, NULL, subrate_setup, NULL, NULL);
