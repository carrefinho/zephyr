/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Subrating peripheral. Advertise, accept a connection, then initiate a
 * Connection Subrate Request (Core Spec 5.1.20) and confirm the negotiated
 * factor. In the transitions scenario the Central then re-negotiates (M->N)
 * and changes the interval; the peripheral's controller applies those, and the
 * Central drives the assertions. The actual event skipping is observed by the
 * Central (see central.c).
 */
#include <zephyr/kernel.h>

#include "bs_types.h"
#include "bs_tracing.h"
#include "time_machine.h"
#include "bstests.h"

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>

#include "conn_subrate.h"

#define WAIT_TIME 30 /* seconds */

extern enum bst_result_t bst_result;

#define FAIL(...)					\
	do {						\
		bst_result = Failed;			\
		bs_trace_error_time_line(__VA_ARGS__);	\
	} while (0)

#define PASS(...)					\
	do {						\
		bst_result = Passed;			\
		bs_trace_info_time(1, __VA_ARGS__);	\
	} while (0)

static struct bt_conn *default_conn;
static volatile bool connected_flag;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void subrate_changed(struct bt_conn *conn,
			    const struct bt_conn_le_subrate_changed *params)
{
	printk("Peripheral subrate changed: status %u factor %u cont %u "
	       "latency %u timeout %u\n", params->status, params->factor,
	       params->continuation_number, params->peripheral_latency,
	       params->supervision_timeout);

	if (params->status != BT_HCI_ERR_SUCCESS) {
		FAIL("Peripheral subrate change failed (status 0x%02x)\n",
		     params->status);
		return;
	}

	/* Negotiation succeeded; the Central validates the resulting skipping. */
	if (params->factor >= 2U) {
		PASS("Peripheral subrating active (factor %u)\n", params->factor);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		FAIL("Peripheral failed to connect (err 0x%02x)\n", err);
		return;
	}

	default_conn = bt_conn_ref(conn);
	connected_flag = true;
	printk("Peripheral connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Peripheral disconnected (reason 0x%02x)\n", reason);

	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
	connected_flag = false;
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.subrate_changed = subrate_changed,
};

/* Advertise, then once connected and settled request subrating with the given
 * factor and stay alive responding to whatever the Central drives next.
 */
static void peripheral_run(uint16_t subrate_min, uint16_t subrate_max)
{
	bool requested = false;
	int err;

	bt_conn_cb_register(&conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Peripheral Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		FAIL("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Peripheral advertising started\n");

	while (true) {
		k_sleep(K_MSEC(SETTLE_DELAY_MS));

		if (connected_flag && !requested && default_conn) {
			struct bt_conn_le_subrate_param param = {
				.subrate_min = subrate_min,
				.subrate_max = subrate_max,
				.max_latency = SUBRATE_REQ_MAX_LATENCY,
				.continuation_number = SUBRATE_REQ_CONT_NUMBER,
				.supervision_timeout = SUBRATE_REQ_TIMEOUT,
			};

			requested = true;
			err = bt_conn_le_subrate_request(default_conn, &param);
			if (err) {
				FAIL("Peripheral subrate request failed (err %d)\n", err);
				return;
			}
			printk("Peripheral requested subrating (min %u max %u)\n",
			       param.subrate_min, param.subrate_max);
		}
	}
}

static void test_peripheral_main(void)
{
	peripheral_run(SUBRATE_REQ_MIN, SUBRATE_REQ_MAX);
}

static void test_peripheral_main_transitions(void)
{
	/* Negotiate exactly factor M; the Central re-negotiates to N afterwards. */
	peripheral_run(SUBRATE_MTON_M, SUBRATE_MTON_M);
}

static void test_peripheral_init(void)
{
	bst_ticker_set_next_tick_absolute(WAIT_TIME * 1e6);
	bst_result = In_progress;
}

static void test_peripheral_tick(bs_time_t HW_device_time)
{
	if (bst_result != Passed) {
		FAIL("peripheral failed (not passed after %i seconds)\n", WAIT_TIME);
	}
}

static const struct bst_test_instance test_peripheral[] = {
	{
		.test_id = "peripheral",
		.test_descr = "Peripheral: connects, requests subrating (factor > 1) "
			      "and confirms the negotiated factor.",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main,
	},
	{
		.test_id = "peripheral_transitions",
		.test_descr = "Peripheral: negotiates factor M; the Central then "
			      "re-negotiates to N and changes the interval.",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main_transitions,
	},
	BSTEST_END_MARKER,
};

struct bst_test_list *test_peripheral_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_peripheral);
}
