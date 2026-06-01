/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Device 1 - the DUT, dual-role like a ZMK split central. It simultaneously:
 *   - advertises as DUT_NAME so `host` connects to it (DUT = PERIPHERAL link),
 *   - scans for SPLIT_NAME and connects to it (DUT = CENTRAL link),
 * both at 7.5 ms. Once the split link is up it blasts max-size writes to the
 * split (long central-link events); the host blasts the DUT (long peripheral-
 * link events). Two long 7.5 ms events on one radio cannot avoid overlapping,
 * forcing the LL_SW preempt/duplicate path every event.
 *
 * PASS = survived WAIT_TIME without the controller asserting. If the prepare
 * pipeline overflows, LL_ASSERT(next) (lll.c:891) aborts the binary instead -
 * that abort, with "ASSERTION FAIL [next]" in the log, IS the reproduction.
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

static struct bt_conn *split_conn;  /* DUT is CENTRAL on this link */
static struct bt_conn *host_conn;   /* DUT is PERIPHERAL on this link */
static volatile bool scanning;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DUT_NAME, sizeof(DUT_NAME) - 1),
};

/* ---- find the split by advertised name ---------------------------------- */

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
		return false; /* stop parsing */
	}
	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *adv)
{
	struct name_match m = { .want = SPLIT_NAME, .found = false };
	struct bt_le_conn_param *param;
	int err;

	if (split_conn || (type != BT_GAP_ADV_TYPE_ADV_IND)) {
		return;
	}

	bt_data_parse(adv, ad_name_cb, &m);
	if (!m.found) {
		return;
	}

	if (bt_le_scan_stop() == 0) {
		scanning = false;
	}

	param = BT_LE_CONN_PARAM(CONN_INTERVAL_UNITS, CONN_INTERVAL_UNITS,
				 CONN_LATENCY, CONN_TIMEOUT_UNITS);
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &split_conn);
	if (err) {
		FAIL("DUT: create split connection failed (err %d)\n", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info;

	if (err) {
		printk("DUT connect failed (err 0x%02x)\n", err);
		return;
	}
	if (bt_conn_get_info(conn, &info) != 0) {
		return;
	}

	if (info.role == BT_CONN_ROLE_CENTRAL) {
		printk("DUT connected to split (central link)\n");
	} else {
		host_conn = bt_conn_ref(conn);
		printk("DUT connected to host (peripheral link)\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("DUT disconnected (reason 0x%02x)\n", reason);
	if (conn == split_conn) {
		bt_conn_unref(split_conn);
		split_conn = NULL;
	} else if (conn == host_conn) {
		bt_conn_unref(host_conn);
		host_conn = NULL;
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void dut_main(void)
{
	uint16_t split_sink;
	int err;

	bt_conn_cb_register(&conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("DUT bt_enable failed (err %d)\n", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		FAIL("DUT advertising failed (err %d)\n", err);
		return;
	}
	printk("DUT advertising as %s\n", DUT_NAME);

	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		FAIL("DUT scan start failed (err %d)\n", err);
		return;
	}
	scanning = true;
	printk("DUT scanning for %s\n", SPLIT_NAME);

	/* Wait for the central (split) link, then discover its sink. */
	while (!split_conn) {
		k_sleep(K_MSEC(20));
		if (bst_result == Failed) {
			return;
		}
	}
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	if (sink_discover(split_conn, &split_sink) != 0) {
		FAIL("DUT could not discover split sink\n");
		return;
	}

	printk("DUT blasting split link; host drives the peripheral link\n");

	/* Saturate the central link forever. The host saturates the peripheral
	 * link from its side. Both DUT events run long and overlap at 7.5 ms.
	 */
	while (split_conn) {
		sink_blast_one(split_conn, split_sink);
	}
}

static void dut_init(void)
{
	bst_ticker_set_next_tick_absolute(WAIT_TIME * 1e6);
	bst_result = In_progress;
}

static void dut_tick(bs_time_t HW_device_time)
{
	/* Reached the deadline without the controller asserting: the prepare
	 * pipeline did NOT overflow this run. (On the buggy controller, try more
	 * seeds / the EVENT_PIPELINE_MAX accelerator; on the fixed controller this
	 * is the expected PASS.)
	 */
	if (bst_result == In_progress) {
		PASS("DUT survived %i s with no prepare-pipeline overflow\n", WAIT_TIME);
	}
}

static const struct bst_test_instance dut_tests[] = {
	{
		.test_id = "dut",
		.test_descr = "Dual-role DUT (peripheral to host + central to split); "
			      "drives both 7.5 ms links to overlap.",
		.test_pre_init_f = dut_init,
		.test_tick_f = dut_tick,
		.test_main_f = dut_main,
	},
	BSTEST_END_MARKER,
};

struct bst_test_list *test_dut_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, dut_tests);
}
