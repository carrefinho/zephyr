/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Subrating central. Two scenarios:
 *   "central"             - let the peripheral negotiate subrating (factor > 1)
 *                           and verify via timed GATT reads that the peripheral
 *                           skips connection events (Phase 2 / 3a).
 *   "central_transitions" - peripheral negotiates factor M, central re-negotiates
 *                           to factor N (M->N transition), verify both re-skip at
 *                           N; then change the connection interval while subrated
 *                           and verify subrating resets to factor 1 (gate handles
 *                           the update, no crash, both present every event).
 *
 * A subrated peer only listens on subrated events, so a read issued while it
 * sleeps is answered only after the next subrated event: large read latency
 * proves skipping, small latency proves no skipping.
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
static struct bt_gatt_read_params read_params;

static volatile uint16_t subrate_factor;
static volatile uint16_t conn_interval;
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

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	printk("Central conn params updated: interval %u latency %u timeout %u\n",
	       interval, latency, timeout);
	conn_interval = interval;
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	if (conn_err) {
		FAIL("Central failed to connect (err 0x%02x)\n", conn_err);
		return;
	}

	printk("Central connected\n");
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
	.le_param_updated = le_param_updated,
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

/* Probe with a single Read-By-UUID of the GAP Device Name (always present and
 * readable) and return the round-trip latency in ms, or -1 on failure.
 */
static int64_t probe_read_latency(void)
{
	int64_t t0;
	int err;

	read_params.func = read_func;
	read_params.handle_count = 0;
	read_params.by_uuid.uuid = BT_UUID_GAP_DEVICE_NAME;
	read_params.by_uuid.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	read_params.by_uuid.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;

	read_err = 0;

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

/* Run `count` probes spaced by READ_GAP_MS and return the worst-case latency,
 * or -1 on failure. The gap exceeds the skip period so the peripheral has gone
 * back to sleep before each probe.
 */
static int64_t probe_max_latency(int count)
{
	int64_t max_latency = 0;

	for (int i = 0; i < count; i++) {
		int64_t latency = probe_read_latency();

		if (latency < 0) {
			return -1;
		}

		printk("Central read %d latency %lld ms\n", i, latency);
		max_latency = MAX(max_latency, latency);
		k_sleep(K_MSEC(READ_GAP_MS));
	}

	return max_latency;
}

static int central_start(void)
{
	struct bt_conn_le_subrate_param defaults = {
		.subrate_min = 1U,
		.subrate_max = SUBRATE_ACC_MAX,
		.max_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout = CONN_TIMEOUT_UNITS,
	};
	int err;

	bt_conn_cb_register(&conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("Bluetooth init failed (err %d)\n", err);
		return err;
	}

	printk("Central Bluetooth initialized\n");

	/* Allow the peripheral's requests to be granted with a large factor. */
	err = bt_conn_le_subrate_set_defaults(&defaults);
	if (err) {
		FAIL("Set default subrate failed (err %d)\n", err);
		return err;
	}

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		FAIL("Scanning failed to start (err %d)\n", err);
		return err;
	}

	return 0;
}

/* Wait for a condition with the WAIT_TIME tick as the backstop, bailing on
 * an already-failed result.
 */
#define WAIT_FOR(_cond)							\
	do {								\
		while (!(_cond)) {					\
			k_sleep(K_MSEC(100));				\
			if (bst_result == Failed) {			\
				return;					\
			}						\
		}							\
	} while (0)

static uint32_t interval_to_ms(uint16_t units)
{
	return (units * 5U) / 4U; /* 1.25 ms units -> ms */
}

static void test_central_main(void)
{
	int64_t max_latency;
	uint32_t interval_ms;
	uint32_t threshold_ms;

	if (central_start()) {
		return;
	}

	/* Wait for the link and a negotiated factor > 1. */
	WAIT_FOR(default_conn && subrate_factor >= 2U);

	interval_ms = interval_to_ms(CONN_INTERVAL_UNITS);
	threshold_ms = (subrate_factor * interval_ms) / 2U;
	printk("Central probing: factor %u, interval %u ms, threshold %u ms\n",
	       subrate_factor, interval_ms, threshold_ms);

	max_latency = probe_max_latency(NUM_READS);
	if (max_latency < 0) {
		return;
	}

	printk("Central max read latency %lld ms (threshold %u ms)\n",
	       max_latency, threshold_ms);
	if (max_latency < threshold_ms) {
		FAIL("Peripheral did not skip events: max latency %lld ms < %u ms\n",
		     max_latency, threshold_ms);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central observed subrating skip (factor %u, max latency %lld ms)\n",
	     subrate_factor, max_latency);
	bs_trace_silent_exit(0);
}

static void test_central_main_transitions(void)
{
	struct bt_conn_le_subrate_param to_n = {
		.subrate_min = 1U,
		.subrate_max = SUBRATE_MTON_N,
		.max_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout = CONN_TIMEOUT_UNITS,
	};
	struct bt_le_conn_param *upd;
	uint32_t interval_ms;
	int64_t max_latency;
	int err;

	if (central_start()) {
		return;
	}

	/* Peripheral negotiates factor M first. */
	WAIT_FOR(default_conn && subrate_factor == SUBRATE_MTON_M);
	printk("Central: peripheral negotiated M=%u\n", subrate_factor);

	/* M->N: Central re-negotiates to factor N (central-initiated, 5.1.19). */
	err = bt_conn_le_subrate_request(default_conn, &to_n);
	if (err) {
		FAIL("Central M->N subrate request failed (err %d)\n", err);
		return;
	}
	WAIT_FOR(subrate_factor == SUBRATE_MTON_N);
	printk("Central: transitioned to N=%u\n", subrate_factor);

	/* Both should now skip at factor N. */
	interval_ms = interval_to_ms(CONN_INTERVAL_UNITS);
	max_latency = probe_max_latency(NUM_READS);
	if (max_latency < 0) {
		return;
	}
	printk("Central post-M->N max latency %lld ms\n", max_latency);
	if (max_latency < (SUBRATE_MTON_N * interval_ms) / 2U) {
		FAIL("No skip after M->N: max latency %lld ms\n", max_latency);
		return;
	}

	/* conn-update while subrated: change the interval, which resets subrating
	 * to factor 1. Validates that the procedure completes (gate keeps both
	 * present every event, #51 prevents the latency_upd crash) and that the
	 * link survives.
	 */
	conn_interval = 0U;
	upd = BT_LE_CONN_PARAM(CONN_UPDATE_INTERVAL_UNITS, CONN_UPDATE_INTERVAL_UNITS,
			       0, CONN_TIMEOUT_UNITS);
	err = bt_conn_le_param_update(default_conn, upd);
	if (err) {
		FAIL("Central conn param update failed (err %d)\n", err);
		return;
	}
	WAIT_FOR(conn_interval == CONN_UPDATE_INTERVAL_UNITS);
	printk("Central: interval updated to %u units while subrated\n", conn_interval);

	/* Subrating must have reset to factor 1 -> no more skipping -> low latency. */
	max_latency = probe_max_latency(NUM_READS / 2);
	if (max_latency < 0) {
		return;
	}
	printk("Central post-update max latency %lld ms\n", max_latency);
	if (max_latency >= (SUBRATE_MTON_N * interval_to_ms(CONN_UPDATE_INTERVAL_UNITS)) / 2U) {
		FAIL("Subrating not reset after interval change: max latency %lld ms\n",
		     max_latency);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central M->N + conn-update validated\n");
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
		.test_descr = "Central: peripheral negotiates subrating; verify the "
			      "peripheral skips connection events via timed reads.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main,
	},
	{
		.test_id = "central_transitions",
		.test_descr = "Central: M->N subrate transition then a conn-param "
			      "interval change while subrated (resets to factor 1).",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_transitions,
	},
	BSTEST_END_MARKER,
};

struct bst_test_list *test_central_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_central);
}
