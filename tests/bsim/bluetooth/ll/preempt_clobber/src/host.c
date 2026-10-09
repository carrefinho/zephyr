/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Device 2 - the "host". A central that connects to the DUT, writes to it
 * (load=1 saturates, load=0 is typing-rate), and every cycle_ms drops the link,
 * waits gap_ms and reconnects - the host disconnects that trigger the ZMK stray write.
 *
 * Args (-argstest): cycle_ms=N (0 = never disconnect) gap_ms=N load=0|1
 *                   host_int=N (connection interval, 1.25 ms units)
 */
#include <zephyr/kernel.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

#include "bs_types.h"
#include "bs_tracing.h"
#include "time_machine.h"
#include "bstests.h"

#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>

#include "preempt_clobber.h"
#include "gatt_sink.h"


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
static volatile bool dut_connected;

static uint32_t cycle_ms = 20000;
static uint32_t gap_ms = 1000;
static uint32_t load;
static uint32_t host_int = HOST_INTERVAL_UNITS;

static uint32_t conn_ok;
static uint32_t conn_fail;

static void test_args_parse(int argc, char *argv[])
{
	for (int i = 0; i < argc; i++) {
		if (sscanf(argv[i], "cycle_ms=%u", &cycle_ms) == 1) {
			continue;
		}
		if (sscanf(argv[i], "gap_ms=%u", &gap_ms) == 1) {
			continue;
		}
		if (sscanf(argv[i], "load=%u", &load) == 1) {
			continue;
		}
		if (sscanf(argv[i], "host_int=%u", &host_int) == 1) {
			continue;
		}
	}
}

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

	param = BT_LE_CONN_PARAM(host_int, host_int, CONN_LATENCY, CONN_TIMEOUT_UNITS);
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &dut_conn);
	if (err) {
		printk("Host: create DUT connection failed (err %d)\n", err);
		dut_conn = NULL;
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		/* e.g. 0x3e, connection failed to be established */
		conn_fail++;
		printk("HOST_CONN_FAIL err=0x%02x t_ms=%lld\n", err, k_uptime_get());
		if (conn == dut_conn) {
			bt_conn_unref(dut_conn);
			dut_conn = NULL;
		}
		return;
	}
	conn_ok++;
	dut_connected = true;
	printk("Host connected to DUT t_ms=%lld\n", k_uptime_get());
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Host disconnected (reason 0x%02x) t_ms=%lld\n", reason, k_uptime_get());
	if (conn == dut_conn) {
		dut_connected = false;
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

	while (true) {
		int64_t up_since;

		err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
		if (err && err != -EALREADY) {
			printk("Host scan start failed (err %d)\n", err);
			k_sleep(K_MSEC(100));
			continue;
		}

		while (!dut_connected) {
			k_sleep(K_MSEC(20));
			if (!dut_conn) {
				/* Connect attempt failed; scan again. */
				(void)bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
			}
		}
		k_sleep(K_MSEC(SETTLE_DELAY_MS));

		dut_sink = 0U;
		if (dut_connected && sink_discover(dut_conn, &dut_sink) != 0) {
			printk("Host could not discover DUT sink\n");
		}

		up_since = k_uptime_get();
		while (dut_connected &&
		       (cycle_ms == 0 || (k_uptime_get() - up_since) < cycle_ms)) {
			if (dut_sink) {
				sink_blast_one(dut_conn, dut_sink);
			}
			if (!load || !dut_sink) {
				k_sleep(K_MSEC(LIGHT_WRITE_MS));
			}
		}

		if (dut_connected) {
			(void)bt_conn_disconnect(dut_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
		while (dut_connected || dut_conn) {
			k_sleep(K_MSEC(10));
		}
		k_sleep(K_MSEC(gap_ms));
	}
}

static void host_init(void)
{
	bst_ticker_set_next_tick_absolute(WAIT_TIME * 1e6);
	bst_result = In_progress;
}

static void host_tick(bs_time_t HW_device_time)
{
	printk("HOST_FINAL conn_ok=%u conn_fail=%u\n", conn_ok, conn_fail);
	if (bst_result == In_progress) {
		PASS("Host completed run\n");
	}
}

static const struct bst_test_instance host_tests[] = {
	{
		.test_id = "host",
		.test_descr = "Host central: connects to the DUT, loads its peripheral "
			      "link and cycles the connection.",
		.test_args_f = test_args_parse,
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
