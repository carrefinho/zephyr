/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Device 2 - the "host". A plain central that connects to the DUT at 7.5 ms and
 * blasts max-size writes at it, saturating the DUT's PERIPHERAL link (stands in
 * for the computer streaming HID to a ZMK central). Mirrors split's role on the
 * other side of the DUT.
 */
#include <zephyr/kernel.h>
#include <string.h>
#include <errno.h>

#include "bs_types.h"
#include "bs_tracing.h"
#include "time_machine.h"
#include "bstests.h"

#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>

#include "prep_pipeline.h"
#include "gatt_sink.h"

#define WAIT_TIME 60 /* seconds */

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

static struct bt_conn *dut_conn;

struct name_match {
	const char *want;
	bool found;
};

static bool ad_name_cb(struct bt_data *data, void *user_data)
{
	struct name_match *m = user_data;

	if (data->type == BT_DATA_NAME_COMPLETE &&
	    data->data_len == strlen(m->want) &&
	    memcmp(data->data, m->want, data->data_len) == 0) {
		m->found = true;
		return false;
	}
	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *adv)
{
	struct name_match m = { .want = DUT_NAME, .found = false };
	struct bt_le_conn_param *param;
	int err;

	if (dut_conn || (type != BT_GAP_ADV_TYPE_ADV_IND)) {
		return;
	}

	bt_data_parse(adv, ad_name_cb, &m);
	if (!m.found) {
		return;
	}

	(void)bt_le_scan_stop();

	/* Host is CENTRAL on the host link: 15 ms, latency 30 (matches the dump).
	 * The 15 ms vs the split's 7.5 ms + per-device xo_drift makes the DUT's two
	 * events sweep into collision. */
	param = BT_LE_CONN_PARAM(HOST_INTERVAL_UNITS, HOST_INTERVAL_UNITS,
				 CONN_LATENCY, CONN_TIMEOUT_UNITS);
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &dut_conn);
	if (err) {
		FAIL("Host: create DUT connection failed (err %d)\n", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		FAIL("Host failed to connect (err 0x%02x)\n", err);
		return;
	}
	printk("Host connected to DUT (central link)\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Host disconnected (reason 0x%02x)\n", reason);
	if (conn == dut_conn) {
		bt_conn_unref(dut_conn);
		dut_conn = NULL;
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void host_main(void)
{
	uint16_t dut_sink;
	int err;

	bt_conn_cb_register(&conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("Host bt_enable failed (err %d)\n", err);
		return;
	}

	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		FAIL("Host scan start failed (err %d)\n", err);
		return;
	}
	printk("Host scanning for %s\n", DUT_NAME);

	while (!dut_conn) {
		k_sleep(K_MSEC(20));
		if (bst_result == Failed) {
			return;
		}
	}
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	if (sink_discover(dut_conn, &dut_sink) != 0) {
		FAIL("Host could not discover DUT sink\n");
		return;
	}

	printk("Host blasting DUT peripheral link\n");
	while (dut_conn) {
		sink_blast_one(dut_conn, dut_sink);
	}
}

static void host_init(void)
{
	bst_ticker_set_next_tick_absolute(WAIT_TIME * 1e6);
	bst_result = In_progress;
}

static void host_tick(bs_time_t HW_device_time)
{
	if (bst_result == In_progress) {
		PASS("Host completed run\n");
	}
}

static const struct bst_test_instance host_tests[] = {
	{
		.test_id = "host",
		.test_descr = "Host central: connects to the DUT and saturates its "
			      "peripheral link.",
		.test_pre_init_f = host_init,
		.test_tick_f = host_tick,
		.test_main_f = host_main,
	},
	BSTEST_END_MARKER,
};

struct bst_test_list *test_host_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, host_tests);
}
