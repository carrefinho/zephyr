/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Device 0 - the "split peripheral". Advertises connectably as SPLIT_NAME,
 * accepts the DUT's central connection, and idles as a write sink (the global
 * sink service in gatt_sink.c absorbs the DUT's max-size writes, lengthening the
 * split-link events). Never disconnects.
 */
#include <zephyr/kernel.h>
#include <string.h>

#include "bs_types.h"
#include "bs_tracing.h"
#include "time_machine.h"
#include "bstests.h"

#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#include "prep_pipeline.h"

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

static volatile bool connected_flag;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, SPLIT_NAME, sizeof(SPLIT_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		FAIL("Split failed to connect (err 0x%02x)\n", err);
		return;
	}
	connected_flag = true;
	printk("Split connected (central=DUT)\n");
	PASS("Split connected; serving as write sink\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Split disconnected (reason 0x%02x)\n", reason);
	connected_flag = false;
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void split_main(void)
{
	int err;

	bt_conn_cb_register(&conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("Split bt_enable failed (err %d)\n", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		FAIL("Split advertising failed (err %d)\n", err);
		return;
	}
	printk("Split advertising as %s\n", SPLIT_NAME);

	/* Idle; the sink service absorbs writes. Survive the whole sim. */
	while (true) {
		k_sleep(K_MSEC(SETTLE_DELAY_MS));
	}
}

static void split_init(void)
{
	bst_ticker_set_next_tick_absolute(WAIT_TIME * 1e6);
	bst_result = In_progress;
}

static void split_tick(bs_time_t HW_device_time)
{
	/* Surviving to the deadline still connected is success for this peer. */
	if (bst_result != Passed) {
		FAIL("split: not connected after %i seconds\n", WAIT_TIME);
	}
}

static const struct bst_test_instance split_tests[] = {
	{
		.test_id = "split",
		.test_descr = "Split peripheral: connectable write sink for the DUT.",
		.test_pre_init_f = split_init,
		.test_tick_f = split_tick,
		.test_main_f = split_main,
	},
	BSTEST_END_MARKER,
};

struct bst_test_list *test_split_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, split_tests);
}
