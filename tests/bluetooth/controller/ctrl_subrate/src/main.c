/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/ztest.h>

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
#include "ull_conn_types.h"
#include "ull_llcp.h"
#include "ull_conn_internal.h"
#include "ull_llcp_internal.h"
#include "ull_llcp_features.h"

#include "helper_pdu.h"
#include "helper_util.h"

static struct ll_conn conn;

static void subrate_setup(void *data)
{
	test_setup(&conn);
}

/*
 * Central-initiated Subrate Update (local procedure)
 *
 * +-----+                     +-------+            +-----+
 * | UT  |                     | LL_A  |            | LT  |
 * +-----+                     +-------+            +-----+
 *    |                            |                   |
 *    | Start                      |                   |
 *    | Subrate Update Proc.       |                   |
 *    |--------------------------->|                   |
 *    |                            |                   |
 *    |                            | LL_SUBRATE_IND    |
 *    |                            |------------------>|
 *    |                            |                   |
 *    |                            |       (ACK)       |
 *    |                            |<------------------|
 *    |                            |                   |
 *    | Subrate Change Event       |                   |
 *    |<---------------------------|                   |
 *    |                            |                   |
 */
ZTEST(subrate_central, test_subrate_central_loc)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data_llctrl_subrate_ind local_subrate_ind = {
		.subrate_factor = 10,
		.subrate_base_event = 0,
		.latency = 0,
		.continuation_number = 0,
		.timeout = 100,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Subrate Update Procedure */
	err = ull_cp_subrate_request(&conn, 10, 10, 0, 0, 100, 1);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_SUBRATE_IND, &conn, &tx, &local_subrate_ind);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Termination not 'triggered' */
	zassert_equal(conn.llcp_terminate.reason_final, 0,
		      "Terminate reason %d", conn.llcp_terminate.reason_final);

	/* There should be one notification due to Subrate Change */
	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, NULL);
	ut_rx_q_is_empty();

	/* Verify subrate parameters are applied */
	zassert_equal(conn.subrate.factor, 10, "Subrate factor mismatch");
	zassert_equal(conn.lll.subrate_factor, 10, "LLL subrate factor mismatch");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Peripheral-initiated Subrate Request (local procedure)
 *
 * +-----+                     +-------+            +-----+
 * | UT  |                     | LL_A  |            | LT  |
 * +-----+                     +-------+            +-----+
 *    |                            |                   |
 *    | Start                      |                   |
 *    | Subrate Request Proc.      |                   |
 *    |--------------------------->|                   |
 *    |                            |                   |
 *    |                            | LL_SUBRATE_REQ    |
 *    |                            |------------------>|
 *    |                            |                   |
 *    |                            |   LL_SUBRATE_IND  |
 *    |                            |<------------------|
 *    |                            |                   |
 *    | Subrate Change Event       |                   |
 *    |<---------------------------|                   |
 *    |                            |                   |
 */
ZTEST(subrate_periph, test_subrate_periph_loc)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data_llctrl_subrate_req local_subrate_req = {
		.subrate_factor_min = 5,
		.subrate_factor_max = 20,
		.max_latency = 10,
		.continuation_number = 2,
		.timeout = 100,
	};
	struct pdu_data_llctrl_subrate_ind remote_subrate_ind = {
		.subrate_factor = 10,
		.subrate_base_event = 0,
		.latency = 5,
		.continuation_number = 2,
		.timeout = 100,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Subrate Request Procedure */
	err = ull_cp_subrate_request(&conn, 5, 20, 10, 2, 100, 1);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_SUBRATE_REQ, &conn, &tx, &local_subrate_req);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Prepare */
	event_prepare(&conn);

	/* Rx LL_SUBRATE_IND from Central */
	lt_tx(LL_SUBRATE_IND, &conn, &remote_subrate_ind);

	/* Done */
	event_done(&conn);

	/* Termination not 'triggered' */
	zassert_equal(conn.llcp_terminate.reason_final, 0,
		      "Terminate reason %d", conn.llcp_terminate.reason_final);

	/* There should be one notification due to Subrate Change */
	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, NULL);
	ut_rx_q_is_empty();

	/* Verify subrate parameters are applied */
	zassert_equal(conn.subrate.factor, 10, "Subrate factor mismatch");
	zassert_equal(conn.subrate.continuation_number, 2, "Continuation number mismatch");
	zassert_equal(conn.lll.subrate_factor, 10, "LLL subrate factor mismatch");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Central receives Subrate Request from Peripheral (remote procedure)
 *
 * +-----+                     +-------+            +-----+
 * | UT  |                     | LL_A  |            | LT  |
 * +-----+                     +-------+            +-----+
 *    |                            |                   |
 *    |                            | LL_SUBRATE_REQ    |
 *    |                            |<------------------|
 *    |                            |                   |
 *    |                            | LL_SUBRATE_IND    |
 *    |                            |------------------>|
 *    |                            |                   |
 *    | Subrate Change Event       |                   |
 *    |<---------------------------|                   |
 *    |                            |                   |
 */
ZTEST(subrate_central, test_subrate_central_rem)
{
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data_llctrl_subrate_req remote_subrate_req = {
		.subrate_factor_min = 5,
		.subrate_factor_max = 20,
		.max_latency = 10,
		.continuation_number = 2,
		.timeout = 100,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Set acceptable defaults */
	conn.subrate.defaults.subrate_min = 1;
	conn.subrate.defaults.subrate_max = 500;
	conn.subrate.defaults.max_latency = 499;
	conn.subrate.defaults.continuation_number = 10;
	conn.subrate.defaults.timeout = 200;

	/* Prepare */
	event_prepare(&conn);

	/* Rx LL_SUBRATE_REQ from Peripheral */
	lt_tx(LL_SUBRATE_REQ, &conn, &remote_subrate_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU (LL_SUBRATE_IND) */
	lt_rx(LL_SUBRATE_IND, &conn, &tx, NULL);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Termination not 'triggered' */
	zassert_equal(conn.llcp_terminate.reason_final, 0,
		      "Terminate reason %d", conn.llcp_terminate.reason_final);

	/* There should be one notification due to Subrate Change */
	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, NULL);
	ut_rx_q_is_empty();

	/* Verify subrate is applied (factor should be max of requested range) */
	zassert_true(conn.subrate.factor >= 5 && conn.subrate.factor <= 20,
		     "Subrate factor out of range: %d", conn.subrate.factor);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Peripheral receives unsolicited Subrate Indication from Central (remote procedure)
 *
 * +-----+                     +-------+            +-----+
 * | UT  |                     | LL_A  |            | LT  |
 * +-----+                     +-------+            +-----+
 *    |                            |                   |
 *    |                            | LL_SUBRATE_IND    |
 *    |                            |<------------------|
 *    |                            |                   |
 *    | Subrate Change Event       |                   |
 *    |<---------------------------|                   |
 *    |                            |                   |
 */
ZTEST(subrate_periph, test_subrate_periph_rem)
{
	struct node_rx_pdu *ntf;
	struct pdu_data_llctrl_subrate_ind remote_subrate_ind = {
		.subrate_factor = 15,
		.subrate_base_event = 100,
		.latency = 5,
		.continuation_number = 3,
		.timeout = 150,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx LL_SUBRATE_IND from Central (unsolicited) */
	lt_tx(LL_SUBRATE_IND, &conn, &remote_subrate_ind);

	/* Done */
	event_done(&conn);

	/* Termination not 'triggered' */
	zassert_equal(conn.llcp_terminate.reason_final, 0,
		      "Terminate reason %d", conn.llcp_terminate.reason_final);

	/* There should be one notification due to Subrate Change */
	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, NULL);
	ut_rx_q_is_empty();

	/* Verify subrate parameters are applied */
	zassert_equal(conn.subrate.factor, 15, "Subrate factor mismatch");
	zassert_equal(conn.subrate.base_event, 100, "Base event mismatch");
	zassert_equal(conn.subrate.continuation_number, 3, "Continuation number mismatch");
	zassert_equal(conn.lll.subrate_factor, 15, "LLL subrate factor mismatch");
	zassert_equal(conn.lll.subrate_base_event, 100, "LLL base event mismatch");
	zassert_equal(conn.lll.subrate_continuation, 3, "LLL continuation mismatch");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Peripheral Subrate Request rejected by Central
 *
 * +-----+                     +-------+            +-----+
 * | UT  |                     | LL_A  |            | LT  |
 * +-----+                     +-------+            +-----+
 *    |                            |                   |
 *    | Start                      |                   |
 *    | Subrate Request Proc.      |                   |
 *    |--------------------------->|                   |
 *    |                            |                   |
 *    |                            | LL_SUBRATE_REQ    |
 *    |                            |------------------>|
 *    |                            |                   |
 *    |                            | LL_REJECT_EXT_IND |
 *    |                            |<------------------|
 *    |                            |                   |
 *    | Subrate Change Event       |                   |
 *    | (with error)               |                   |
 *    |<---------------------------|                   |
 *    |                            |                   |
 */
ZTEST(subrate_periph, test_subrate_periph_loc_reject)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data_llctrl_subrate_req local_subrate_req = {
		.subrate_factor_min = 5,
		.subrate_factor_max = 20,
		.max_latency = 10,
		.continuation_number = 2,
		.timeout = 100,
	};
	struct pdu_data_llctrl_reject_ext_ind reject_ext_ind = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ,
		.error_code = BT_HCI_ERR_UNSUPP_LL_PARAM_VAL,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Subrate Request Procedure */
	err = ull_cp_subrate_request(&conn, 5, 20, 10, 2, 100, 1);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_SUBRATE_REQ, &conn, &tx, &local_subrate_req);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Prepare */
	event_prepare(&conn);

	/* Rx LL_REJECT_EXT_IND from Central */
	lt_tx(LL_REJECT_EXT_IND, &conn, &reject_ext_ind);

	/* Done */
	event_done(&conn);

	/* Termination not 'triggered' */
	zassert_equal(conn.llcp_terminate.reason_final, 0,
		      "Terminate reason %d", conn.llcp_terminate.reason_final);

	/* There should be one notification with error status */
	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, NULL);
	ut_rx_q_is_empty();

	/* Subrate should NOT be changed (still default) */
	zassert_equal(conn.subrate.factor, 1, "Subrate factor should remain 1");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST_SUITE(subrate_central, NULL, NULL, subrate_setup, NULL, NULL);
ZTEST_SUITE(subrate_periph, NULL, NULL, subrate_setup, NULL, NULL);
