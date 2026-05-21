/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Subrating central: connect to the peripheral at a fixed interval, let the
 * peripheral negotiate subrating (factor > 1), then probe the link with timed
 * GATT reads while the peripheral is idle. Because a subrated peripheral only
 * listens on subrated events, a read issued while it sleeps is answered only
 * after it next wakes, so the observed read latency reveals the skip cadence:
 * latencies far above one connection interval prove the peripheral skipped
 * events. The central itself (not yet subrating-aware) is present on every
 * event, so this isolates the peripheral skipping (Phase 2).
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
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

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
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_read_params read_params;

static volatile uint16_t name_value_handle;
static volatile uint16_t subrate_factor;
static volatile int read_err;
static K_SEM_DEFINE(read_done, 0, 1);

static uint8_t read_func(struct bt_conn *conn, uint8_t err,
			 struct bt_gatt_read_params *params,
			 const void *data, uint16_t length)
{
	read_err = err;
	k_sem_give(&read_done);
	return BT_GATT_ITER_STOP;
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	if (!attr) {
		printk("Central discovery complete\n");
		return BT_GATT_ITER_STOP;
	}

	name_value_handle = bt_gatt_attr_value_handle(attr);
	printk("Central found device-name char, value handle %u\n",
	       name_value_handle);

	return BT_GATT_ITER_STOP;
}

static void subrate_changed(struct bt_conn *conn,
			    const struct bt_conn_le_subrate_changed *params)
{
	printk("Central subrate changed: status %u factor %u cont %u "
	       "latency %u timeout %u\n", params->status, params->factor,
	       params->continuation_number, params->peripheral_latency,
	       params->supervision_timeout);

	if (params->status != BT_HCI_ERR_SUCCESS) {
		FAIL("Central subrate change failed (status 0x%02x)\n",
		     params->status);
		return;
	}

	subrate_factor = params->factor;
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;

	if (conn_err) {
		FAIL("Central failed to connect (err 0x%02x)\n", conn_err);
		return;
	}

	printk("Central connected\n");

	discover_params.uuid = BT_UUID_GAP_DEVICE_NAME;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

	err = bt_gatt_discover(conn, &discover_params);
	if (err) {
		FAIL("Central discover failed (err %d)\n", err);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Central disconnected (reason 0x%02x)\n", reason);

	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.subrate_changed = subrate_changed,
};

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	struct bt_le_conn_param *param;
	char dev[BT_ADDR_LE_STR_LEN];
	int err;

	/* Ignore further reports once a connection is being created. */
	if (default_conn) {
		return;
	}

	/* Only connectable advertising. */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	bt_addr_le_to_str(addr, dev, sizeof(dev));
	printk("Central found device %s, connecting\n", dev);

	err = bt_le_scan_stop();
	if (err) {
		FAIL("Stop scan failed (err %d)\n", err);
		return;
	}

	/* Fixed interval so the skip cadence is deterministic. */
	param = BT_LE_CONN_PARAM(CONN_INTERVAL_UNITS, CONN_INTERVAL_UNITS,
				 0, CONN_TIMEOUT_UNITS);
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &default_conn);
	if (err) {
		FAIL("Create connection failed (err %d)\n", err);
	}
}

/* Probe with a single read and return the round-trip latency in ms, or -1. */
static int64_t probe_read_latency(void)
{
	int64_t t0;
	int err;

	read_params.func = read_func;
	read_params.handle_count = 1;
	read_params.single.handle = name_value_handle;
	read_params.single.offset = 0;

	t0 = k_uptime_get();
	err = bt_gatt_read(default_conn, &read_params);
	if (err) {
		FAIL("Central read failed (err %d)\n", err);
		return -1;
	}

	if (k_sem_take(&read_done, K_SECONDS(5)) != 0) {
		FAIL("Central read timed out\n");
		return -1;
	}

	if (read_err) {
		FAIL("Central read returned ATT err 0x%02x\n", read_err);
		return -1;
	}

	return k_uptime_get() - t0;
}

static void test_central_main(void)
{
	struct bt_conn_le_subrate_param defaults = {
		.subrate_min = 1U,
		.subrate_max = SUBRATE_ACC_MAX,
		.max_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout = CONN_TIMEOUT_UNITS,
	};
	struct bt_conn_info info;
	int64_t max_latency = 0;
	uint32_t interval_ms;
	uint32_t threshold_ms;
	int err;

	bt_conn_cb_register(&conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Central Bluetooth initialized\n");

	/* Allow the peripheral's request to be granted with a large factor. */
	err = bt_conn_le_subrate_set_defaults(&defaults);
	if (err) {
		FAIL("Set default subrate failed (err %d)\n", err);
		return;
	}

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		FAIL("Scanning failed to start (err %d)\n", err);
		return;
	}

	/* Wait until the link is up, the device-name handle is discovered and
	 * subrating has been negotiated with factor > 1.
	 */
	while (!default_conn || !name_value_handle || subrate_factor < 2U) {
		k_sleep(K_MSEC(100));

		if (bst_result == Failed) {
			return;
		}
	}

	err = bt_conn_get_info(default_conn, &info);
	if (err) {
		FAIL("Central conn info failed (err %d)\n", err);
		return;
	}
	interval_ms = (info.le.interval * 5U) / 4U; /* 1.25 ms units -> ms */

	/* A subrated, idle peripheral listens once every (factor * interval); a
	 * probe issued while it sleeps waits up to that long. Require the worst
	 * observed latency to clear half the skip period - unreachable unless
	 * the peripheral actually skipped events.
	 */
	threshold_ms = (subrate_factor * interval_ms) / 2U;

	printk("Central probing: factor %u, interval %u ms, threshold %u ms\n",
	       subrate_factor, interval_ms, threshold_ms);

	for (int i = 0; i < NUM_READS; i++) {
		int64_t latency = probe_read_latency();

		if (latency < 0) {
			return; /* FAIL already set */
		}

		printk("Central read %d latency %lld ms\n", i, latency);
		max_latency = MAX(max_latency, latency);

		/* Let the peripheral go back to sleep before the next probe. */
		k_sleep(K_MSEC(READ_GAP_MS));
	}

	printk("Central max read latency %lld ms (threshold %u ms)\n",
	       max_latency, threshold_ms);

	if (max_latency < threshold_ms) {
		FAIL("Peripheral did not skip events: max read latency %lld ms "
		     "below threshold %u ms\n", max_latency, threshold_ms);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central observed subrating skip (factor %u, max latency %lld ms)\n",
	     subrate_factor, max_latency);
	bs_trace_silent_exit(0);
}

static void test_central_init(void)
{
	bst_ticker_set_next_tick_absolute(WAIT_TIME * 1e6);
	bst_result = In_progress;
}

static void test_central_tick(bs_time_t HW_device_time)
{
	if (bst_result != Passed) {
		FAIL("central failed (not passed after %i seconds)\n", WAIT_TIME);
	}
}

static const struct bst_test_instance test_central[] = {
	{
		.test_id = "central",
		.test_descr = "Subrating central: connects, lets the peripheral "
			      "negotiate subrating, and verifies via timed reads "
			      "that the peripheral skips connection events.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main,
	},
	BSTEST_END_MARKER,
};

struct bst_test_list *test_central_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_central);
}
