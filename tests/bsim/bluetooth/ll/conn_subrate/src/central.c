/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Subrating central. Connects at a fixed interval, lets the peripheral
 * negotiate subrating, then probes the link with timed Read-By-UUID reads. A
 * subrated peer only listens on subrated events, so a read issued while it
 * sleeps is answered only after the next subrated event: large read latency
 * proves skipping, small latency proves no skipping / continuation.
 *
 * Scenarios: basic skip, M->N transition + conn-update reset, continuation
 * events, peripheral-latency stacking, and a simultaneous-request collision.
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
#include <zephyr/sys/byteorder.h>

#include "conn_subrate.h"

#define WAIT_TIME 30 /* seconds */

extern enum bst_result_t bst_result;

#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
/* Controller (ull_conn.c) per-connection count of events the device was present
 * for. Same binary in bsim, so this links directly. Used by central_lat_count.
 */
extern volatile uint32_t ll_test_conn_event_count[];
#endif

#if defined(CONFIG_BT_CTLR_TEST_CONN_TICKS_SLOT)
/* Controller (ull_conn.c) per-connection scheduler slot reservation (ticker
 * ticks). Same binary in bsim, so this links directly. Used by central_fsu to
 * prove a Frame Space Update actually shrinks the reservation.
 */
extern volatile uint32_t ll_test_conn_ticks_slot[];
#endif

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
static volatile bool collision_mode;
static volatile bool phy_updated;
static volatile bool central_connected;
static K_SEM_DEFINE(read_done, 0, 1);

static uint8_t read_func(struct bt_conn *conn, uint8_t err,
			 struct bt_gatt_read_params *params,
			 const void *data, uint16_t length)
{
	read_err = err;
	k_sem_give(&read_done);
	return BT_GATT_ITER_STOP;
}

#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
/* Multi-connection test state: the Central holds two links - one that
 * negotiates subrating and one that does not - to verify the subrated link
 * keeps its cadence while the second (full-rate) link is active, i.e. the
 * scheduler juggling both does not break the subrate skip. The per-conn event
 * counter (ll_test_conn_event_count[]) measures each link independently.
 */
static struct bt_conn *m_conn[2];
static volatile int m_created;
static volatile int m_nconns;
static volatile uint16_t m_subrated_handle = 0xFFFFU;
static volatile bool multi_mode;

static void device_found_multi(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			       struct net_buf_simple *ad)
{
	struct bt_le_conn_param *param;
	int err;

	if ((m_created >= 2) || (type != BT_GAP_ADV_TYPE_ADV_IND &&
				 type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND)) {
		return;
	}

	err = bt_le_scan_stop();
	if (err) {
		FAIL("Multi: stop scan failed (err %d)\n", err);
		return;
	}

	param = BT_LE_CONN_PARAM(CONN_INTERVAL_UNITS, CONN_INTERVAL_UNITS,
				 0, CONN_TIMEOUT_UNITS);
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &m_conn[m_created]);
	if (err) {
		FAIL("Multi: create connection %d failed (err %d)\n", m_created, err);
		return;
	}
	m_created++;
}
#endif /* CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT */

static void subrate_changed(struct bt_conn *conn,
			    const struct bt_conn_le_subrate_changed *params)
{
	printk("Central subrate changed: status %u factor %u cont %u "
	       "latency %u timeout %u\n", params->status, params->factor,
	       params->continuation_number, params->peripheral_latency,
	       params->supervision_timeout);

	if (params->status != BT_HCI_ERR_SUCCESS) {
		if (collision_mode) {
			return; /* a rejected request is a valid collision outcome */
		}
		FAIL("Central subrate change failed (status 0x%02x)\n",
		     params->status);
		return;
	}

	subrate_factor = params->factor;

#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
	if (multi_mode && params->factor > 1U) {
		uint16_t h;

		if (!bt_hci_get_conn_handle(conn, &h)) {
			m_subrated_handle = h;
		}
	}
#endif
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	printk("Central conn params updated: interval %u latency %u timeout %u\n",
	       interval, latency, timeout);
	conn_interval = interval;
}

static void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *info)
{
	printk("Central PHY updated: tx %u rx %u\n", info->tx_phy, info->rx_phy);
	phy_updated = true;
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	if (conn_err) {
		FAIL("Central failed to connect (err 0x%02x)\n", conn_err);
		return;
	}

	central_connected = true;
	printk("Central connected\n");

#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
	/* Just record completion; the test thread drives the scan for the next
	 * link (never issue a blocking HCI command from a connection callback).
	 */
	if (multi_mode) {
		m_nconns++;
	}
#endif
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Central disconnected (reason 0x%02x)\n", reason);

	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
	central_connected = false;
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.subrate_changed = subrate_changed,
	.le_param_updated = le_param_updated,
	.le_phy_updated = le_phy_updated,
};

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	struct bt_le_conn_param *param;
	char dev[BT_ADDR_LE_STR_LEN];
	int err;

	if (default_conn) {
		return; /* ignore further reports once connecting */
	}
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

/* Single Read-By-UUID of the GAP Device Name; returns latency in ms or -1. */
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

/* Run `count` probes spaced by `gap_ms`, reporting min and max latency.
 * Returns 0, or -1 on failure.
 */
static int probe_minmax(int count, int gap_ms, int64_t *out_min, int64_t *out_max)
{
	int64_t mn = -1;
	int64_t mx = 0;

	for (int i = 0; i < count; i++) {
		int64_t latency = probe_read_latency();

		if (latency < 0) {
			return -1;
		}
		printk("Central read %d latency %lld ms\n", i, latency);
		if (mn < 0 || latency < mn) {
			mn = latency;
		}
		if (latency > mx) {
			mx = latency;
		}
		if (i < count - 1) {
			k_sleep(K_MSEC(gap_ms));
		}
	}

	*out_min = mn;
	*out_max = mx;
	return 0;
}

static int central_start(void)
{
	/* Generous acceptable params so any reasonable peripheral request is
	 * granted as-is (negotiated = requested, clamped to these ceilings).
	 */
	struct bt_conn_le_subrate_param defaults = {
		.subrate_min = 1U,
		.subrate_max = SUBRATE_ACC_MAX,
		.max_latency = 10U,
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

#define SUBRATE_WAIT(_cond)						\
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
	int64_t mn, mx;
	uint32_t interval_ms, threshold_ms;

	if (central_start()) {
		return;
	}
	SUBRATE_WAIT(default_conn && subrate_factor >= 2U);

	interval_ms = interval_to_ms(CONN_INTERVAL_UNITS);
	threshold_ms = (subrate_factor * interval_ms) / 2U;
	printk("Central probing: factor %u, interval %u ms, threshold %u ms\n",
	       subrate_factor, interval_ms, threshold_ms);

	if (probe_minmax(NUM_READS, READ_GAP_MS, &mn, &mx)) {
		return;
	}
	printk("Central max read latency %lld ms (threshold %u ms)\n", mx, threshold_ms);
	if (mx < threshold_ms) {
		FAIL("Peripheral did not skip events: max latency %lld ms < %u ms\n",
		     mx, threshold_ms);
		return;
	}
	/* ...but not far beyond the negotiated cadence: a Central that over-skips
	 * (wrong factor/phase) would push the latency well past factor*interval.
	 */
	if (mx > 2U * subrate_factor * interval_ms) {
		FAIL("Skip exceeds negotiated factor %u cadence: max %lld ms > %u ms\n",
		     subrate_factor, mx, 2U * subrate_factor * interval_ms);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central observed subrating skip (factor %u, max latency %lld ms)\n",
	     subrate_factor, mx);
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
	int64_t mn, mx;
	int err;

	if (central_start()) {
		return;
	}

	SUBRATE_WAIT(default_conn && subrate_factor == SUBRATE_MTON_M);
	printk("Central: peripheral negotiated M=%u\n", subrate_factor);

	/* Out-of-range parameters must be rejected (subrate_max > 0x01F4). */
	{
		struct bt_conn_le_subrate_param bad = {
			.subrate_min = 1U,
			.subrate_max = 600U,
			.max_latency = 0U,
			.continuation_number = 0U,
			.supervision_timeout = CONN_TIMEOUT_UNITS,
		};

		if (bt_conn_le_subrate_request(default_conn, &bad) == 0) {
			FAIL("Out-of-range subrate request was accepted\n");
			return;
		}
		printk("Central: out-of-range subrate request rejected\n");
	}

	err = bt_conn_le_subrate_request(default_conn, &to_n);
	if (err) {
		FAIL("Central M->N subrate request failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(subrate_factor == SUBRATE_MTON_N);
	printk("Central: transitioned to N=%u\n", subrate_factor);

	interval_ms = interval_to_ms(CONN_INTERVAL_UNITS);
	if (probe_minmax(NUM_READS, READ_GAP_MS, &mn, &mx)) {
		return;
	}
	printk("Central post-M->N max latency %lld ms\n", mx);
	if (mx < (SUBRATE_MTON_N * interval_ms) / 2U) {
		FAIL("No skip after M->N: max latency %lld ms\n", mx);
		return;
	}

	/* conn-update while subrated -> resets subrating to factor 1. */
	conn_interval = 0U;
	upd = BT_LE_CONN_PARAM(CONN_UPDATE_INTERVAL_UNITS, CONN_UPDATE_INTERVAL_UNITS,
			       0, CONN_TIMEOUT_UNITS);
	err = bt_conn_le_param_update(default_conn, upd);
	if (err) {
		FAIL("Central conn param update failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(conn_interval == CONN_UPDATE_INTERVAL_UNITS);
	printk("Central: interval updated to %u units while subrated\n", conn_interval);

	if (probe_minmax(NUM_READS / 2, READ_GAP_MS, &mn, &mx)) {
		return;
	}
	printk("Central post-update max latency %lld ms\n", mx);
	if (mx >= (SUBRATE_MTON_N * interval_to_ms(CONN_UPDATE_INTERVAL_UNITS)) / 2U) {
		FAIL("Subrating not reset after interval change: max latency %lld ms\n", mx);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central M->N + conn-update validated\n");
	bs_trace_silent_exit(0);
}

static void test_central_main_continuation(void)
{
	uint32_t interval_ms;
	int64_t idle_min, idle_max, cont_min, cont_max;

	if (central_start()) {
		return;
	}
	SUBRATE_WAIT(default_conn && subrate_factor == SUBRATE_CONT_FACTOR);

	interval_ms = interval_to_ms(CONN_INTERVAL_UNITS);

	/* Idle, widely-spaced reads: the peripheral re-skips between them, so at
	 * least one read waits ~the skip period (continuation window expired).
	 */
	k_sleep(K_MSEC(CONT_IDLE_MS));
	if (probe_minmax(NUM_READS, READ_GAP_MS, &idle_min, &idle_max)) {
		return;
	}
	printk("Central continuation idle max %lld ms\n", idle_max);
	if (idle_max < (SUBRATE_CONT_FACTOR * interval_ms) / 2U) {
		FAIL("Idle reads did not show skip: max %lld ms\n", idle_max);
		return;
	}

	/* Close-spaced reads stay within the continuation window: each read's data
	 * reloads the window, so reads are answered immediately. Count reads that
	 * fall outside the window - tolerate one boundary slip but catch a window
	 * that is systematically broken (asserting only the min would hide that).
	 */
	int slow = 0;

	cont_min = -1;
	cont_max = 0;
	for (int i = 0; i < NUM_READS; i++) {
		int64_t lat = probe_read_latency();

		if (lat < 0) {
			return;
		}
		printk("Central continuation burst read %d latency %lld ms\n", i, lat);
		if (cont_min < 0 || lat < cont_min) {
			cont_min = lat;
		}
		if (lat > cont_max) {
			cont_max = lat;
		}
		if (lat > 2 * (int64_t)interval_ms) {
			slow++;
		}
		if (i < NUM_READS - 1) {
			k_sleep(K_MSEC(CONT_READ_GAP_MS));
		}
	}
	printk("Central continuation burst: min %lld max %lld, %d slow of %d\n",
	       cont_min, cont_max, slow, NUM_READS);
	if (cont_min > 2 * interval_ms) {
		FAIL("Continuation did not keep link awake: min %lld ms\n", cont_min);
		return;
	}
	if (slow > 2) {
		FAIL("Continuation window broke repeatedly: %d slow reads of %d\n",
		     slow, NUM_READS);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central continuation validated (idle %lld ms, burst min %lld max %lld)\n",
	     idle_max, cont_min, cont_max);
	bs_trace_silent_exit(0);
}

static void test_central_main_latency(void)
{
	uint32_t interval_ms, effective;
	int64_t mn, mx;

	if (central_start()) {
		return;
	}
	SUBRATE_WAIT(default_conn && subrate_factor == SUBRATE_LAT_FACTOR);

	interval_ms = interval_to_ms(CONN_INTERVAL_UNITS);
	/* Peripheral skips factor*(latency+1) events; a probe waits up to that. */
	effective = SUBRATE_LAT_FACTOR * (SUBRATE_LAT_LATENCY + 1U);
	printk("Central latency probing: factor %u, effective skip %u events\n",
	       subrate_factor, effective);

	if (probe_minmax(NUM_READS, READ_GAP_MS, &mn, &mx)) {
		return;
	}
	printk("Central latency max read %lld ms\n", mx);
	/* Must reach into the stacked cadence (factor*(latency+1)); a factor-only
	 * skip would top out around SUBRATE_LAT_FACTOR*interval.
	 */
	if (mx < (effective * interval_ms) / 2U) {
		FAIL("Peripheral latency not stacked: max %lld ms < %u ms\n",
		     mx, (effective * interval_ms) / 2U);
		return;
	}
	/* ...and every read, not just the worst sample, must reach into the stacked
	 * cadence - a peripheral that ignored max_latency would top out near
	 * factor*interval on every read.
	 */
	if (mn < (effective * interval_ms) / 2U) {
		FAIL("Peripheral latency not consistently stacked: min %lld ms < %u ms\n",
		     mn, (effective * interval_ms) / 2U);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central peripheral-latency validated (min %lld max %lld ms)\n", mn, mx);
	bs_trace_silent_exit(0);
}

#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
/* Regression lock for the central-burst bug: under peripheral max_latency
 * stacking the central must stay on its subrate cadence (present every `factor`
 * events) and must NOT break latency to ~full rate on the peripheral's expected
 * skipped events. Reuses the peripheral_latency peer (factor 4, max_latency 2,
 * cn 0, then idle) and counts the central's own on-air events over an idle
 * window - the only way to see the burst, which has no peripheral-observable
 * effect.
 */
static void test_central_main_lat_count(void)
{
	uint32_t interval_ms, expected, threshold, wakes;
	uint32_t before;

	if (central_start()) {
		return;
	}
	SUBRATE_WAIT(default_conn && subrate_factor == SUBRATE_LAT_FACTOR);

	/* Let the subrate negotiation and any feature/PHY exchange settle so the
	 * steady-state count is clean.
	 */
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	interval_ms = interval_to_ms(CONN_INTERVAL_UNITS);

	before = ll_test_conn_event_count[0];
	k_sleep(K_MSEC(LAT_COUNT_WINDOW_MS));
	wakes = ll_test_conn_event_count[0] - before;

	/* Subrate cadence: the central is present once per `factor` events,
	 * regardless of the peripheral coasting further on max_latency.
	 */
	expected = LAT_COUNT_WINDOW_MS / (SUBRATE_LAT_FACTOR * interval_ms);
	threshold = expected + (expected / 2U) + 2U; /* 1.5x + slack */
	printk("Central present %u events in %u ms (subrate cadence ~%u, "
	       "threshold %u, factor %u)\n",
	       wakes, LAT_COUNT_WINDOW_MS, expected, threshold, subrate_factor);

	if (wakes > threshold) {
		FAIL("Central over-present: %u > %u events - subrate skip broken by "
		     "peripheral latency (collapsed toward full rate)\n", wakes, threshold);
		return;
	}
	/* Sanity: still present at roughly the cadence (not stalled/disconnected). */
	if (wakes < (expected / 2U)) {
		FAIL("Central barely present: %u < %u events - link stalled?\n",
		     wakes, expected / 2U);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central held subrate cadence under peripheral latency (%u events "
	     "in %u ms, factor %u)\n", wakes, LAT_COUNT_WINDOW_MS, subrate_factor);
	bs_trace_silent_exit(0);
}

/* Exact central skip-cadence lock at the negotiated factor (no peripheral
 * latency stacking, so central and peripheral are aligned). The central must be
 * present ~once per `factor` events. Run against peripherals negotiating
 * different factors -- including non-power-of-2 -- to catch over-skip,
 * under-skip/burst and subrated-event phase-math errors, the dimension the
 * read-latency probes can't measure precisely.
 */
static void test_central_main_cadence(void)
{
	uint32_t interval_ms, expected, hi, lo, wakes, before, factor;

	if (central_start()) {
		return;
	}
	SUBRATE_WAIT(default_conn && subrate_factor >= 2U);
	factor = subrate_factor;

	/* Let the subrate negotiation and any feature/PHY exchange settle. */
	k_sleep(K_MSEC(SETTLE_DELAY_MS));
	interval_ms = interval_to_ms(CONN_INTERVAL_UNITS);

	before = ll_test_conn_event_count[0];
	k_sleep(K_MSEC(LAT_COUNT_WINDOW_MS));
	wakes = ll_test_conn_event_count[0] - before;

	expected = LAT_COUNT_WINDOW_MS / (factor * interval_ms);
	hi = expected + (expected / 2U) + 2U;
	lo = (expected > 2U) ? (expected / 2U) : 1U;
	printk("Central present %u events in %u ms (factor %u, cadence ~%u, [%u..%u])\n",
	       wakes, LAT_COUNT_WINDOW_MS, factor, expected, lo, hi);

	if (wakes > hi) {
		FAIL("Central over-present: %u > %u events - skip cadence broken "
		     "(factor %u)\n", wakes, hi, factor);
		return;
	}
	if (wakes < lo) {
		FAIL("Central under-present: %u < %u events - over-skip/phase error "
		     "(factor %u)\n", wakes, lo, factor);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central skip cadence correct (%u events in %u ms, factor %u)\n",
	     wakes, LAT_COUNT_WINDOW_MS, factor);
	bs_trace_silent_exit(0);
}

static int central_init_multi(void)
{
	struct bt_conn_le_subrate_param defaults = {
		.subrate_min = 1U,
		.subrate_max = SUBRATE_ACC_MAX,
		.max_latency = 10U,
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

	err = bt_conn_le_subrate_set_defaults(&defaults);
	if (err) {
		FAIL("Set default subrate failed (err %d)\n", err);
		return err;
	}

	return 0;
}

/* Multi-connection: the Central holds two links - one peer requests subrating,
 * the other never does (full rate). The subrated link must hold its skip
 * cadence while the full-rate link is active every event; a scheduler that
 * lets the busy link drag the subrated one back to full rate is caught here.
 * This mirrors the real split topology (subrated split link + non-subrated host
 * link on one Central).
 */
static void test_central_main_multi(void)
{
	uint32_t interval_ms, expected_sub, hi_sub, wakes_sub, wakes_plain;
	uint32_t before_sub, before_plain;
	uint16_t h0, h1, plain_handle, factor;
	int i;

	multi_mode = true;
	if (central_init_multi()) {
		return;
	}
	/* Connect to both peers one at a time, driving the scan from this thread.
	 * device_found_multi stops the scan and creates each link; the connected
	 * callback only bumps m_nconns (no HCI calls from callback context).
	 */
	for (i = 0; i < 2; i++) {
		int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found_multi);

		if (err) {
			FAIL("Multi: scan start for link %d failed (err %d)\n", i, err);
			return;
		}
		SUBRATE_WAIT(m_nconns > i);
	}
	/* One peer requests subrating; wait for it to be negotiated. */
	SUBRATE_WAIT(m_subrated_handle != 0xFFFFU);
	factor = subrate_factor;

	if (bt_hci_get_conn_handle(m_conn[0], &h0) ||
	    bt_hci_get_conn_handle(m_conn[1], &h1)) {
		FAIL("Multi: could not read conn handles\n");
		return;
	}
	plain_handle = (h0 == m_subrated_handle) ? h1 : h0;
	printk("Multi: subrated handle %u (factor %u), plain handle %u\n",
	       m_subrated_handle, factor, plain_handle);

	/* Let both links settle (negotiation, feature/PHY exchange). */
	k_sleep(K_MSEC(SETTLE_DELAY_MS));
	interval_ms = interval_to_ms(CONN_INTERVAL_UNITS);

	before_sub = ll_test_conn_event_count[m_subrated_handle];
	before_plain = ll_test_conn_event_count[plain_handle];
	k_sleep(K_MSEC(LAT_COUNT_WINDOW_MS));
	wakes_sub = ll_test_conn_event_count[m_subrated_handle] - before_sub;
	wakes_plain = ll_test_conn_event_count[plain_handle] - before_plain;

	expected_sub = LAT_COUNT_WINDOW_MS / (factor * interval_ms);
	hi_sub = expected_sub + (expected_sub / 2U) + 2U;
	printk("Multi: subrated link %u events (cadence ~%u, <=%u), plain link %u events "
	       "in %u ms\n", wakes_sub, expected_sub, hi_sub, wakes_plain, LAT_COUNT_WINDOW_MS);

	/* The subrated link keeps its cadence despite the concurrent full-rate link. */
	if (wakes_sub > hi_sub) {
		FAIL("Multi: subrated link over-present: %u > %u events - cadence "
		     "broken by the concurrent link\n", wakes_sub, hi_sub);
		return;
	}
	if (wakes_sub < (expected_sub / 2U)) {
		FAIL("Multi: subrated link stalled: %u events\n", wakes_sub);
		return;
	}
	/* Sanity: the other link really is full rate (clearly above the subrated
	 * cadence), confirming we measured two distinct links.
	 */
	if (wakes_plain < (2U * hi_sub)) {
		FAIL("Multi: plain link not full-rate: %u events (expected >> %u)\n",
		     wakes_plain, hi_sub);
		return;
	}

	for (i = 0; i < 2; i++) {
		(void)bt_conn_disconnect(m_conn[i], BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
	PASS("Multi: subrated link held cadence (%u events) alongside full-rate link "
	     "(%u events)\n", wakes_sub, wakes_plain);
	bs_trace_silent_exit(0);
}
#endif /* CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT */

static void test_central_main_collision(void)
{
	struct bt_conn_le_subrate_param to_central = {
		.subrate_min = 1U,
		.subrate_max = SUBRATE_COLL_CENTRAL,
		.max_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout = CONN_TIMEOUT_UNITS,
	};
	int64_t cmn, cmx;
	uint32_t civ;

	if (central_start()) {
		return;
	}
	SUBRATE_WAIT(default_conn && subrate_factor == SUBRATE_COLL_INITIAL);
	printk("Central: subrating established at factor %u\n", subrate_factor);

	/* Re-negotiate at the same uptime the peripheral does, forcing a collision. */
	while (k_uptime_get() < COLLISION_TIME_MS) {
		k_sleep(K_MSEC(20));
		if (bst_result == Failed) {
			return;
		}
	}
	collision_mode = true;
	(void)bt_conn_le_subrate_request(default_conn, &to_central);

	/* Let the collision resolve, then confirm the link is still functional and
	 * both sides converged: probe several times (a desync would drop the link
	 * or stall a read) and, if still subrated, check the skip cadence matches
	 * the Central's final factor. Subrate collisions have no spec-defined
	 * winner (no instant), so only agreement - not a particular factor - holds.
	 */
	k_sleep(K_MSEC(3000));
	if (!default_conn) {
		FAIL("Central lost connection after subrate collision\n");
		return;
	}
	civ = interval_to_ms(CONN_INTERVAL_UNITS);
	if (probe_minmax(NUM_READS / 2, READ_GAP_MS, &cmn, &cmx)) {
		return;
	}
	if (subrate_factor >= 2U && cmx < (subrate_factor * civ) / 2U) {
		FAIL("Post-collision skip inconsistent with factor %u: max %lld ms\n",
		     subrate_factor, cmx);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central survived subrate collision (final factor %u, max %lld ms)\n",
	     subrate_factor, cmx);
	bs_trace_silent_exit(0);
}

static void test_central_main_phy(void)
{
	uint32_t interval_ms;
	int64_t mn, mx;
	int err;

	if (central_start()) {
		return;
	}
	SUBRATE_WAIT(default_conn && subrate_factor >= 2U);
	interval_ms = interval_to_ms(CONN_INTERVAL_UNITS);

	if (probe_minmax(NUM_READS / 2, READ_GAP_MS, &mn, &mx)) {
		return;
	}
	if (mx < (subrate_factor * interval_ms) / 2U) {
		FAIL("No skip before PHY update: max %lld ms\n", mx);
		return;
	}

	/* A PHY update is an instant-based procedure: the steady-state gate must
	 * keep both present every event so it completes, and subrating must
	 * survive it (the interval is unchanged).
	 */
	phy_updated = false;
	err = bt_conn_le_phy_update(default_conn, BT_CONN_LE_PHY_PARAM_2M);
	if (err) {
		FAIL("Central PHY update request failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(phy_updated);
	printk("Central: PHY updated while subrated\n");

	if (probe_minmax(NUM_READS / 2, READ_GAP_MS, &mn, &mx)) {
		return;
	}
	if (mx < (subrate_factor * interval_ms) / 2U) {
		FAIL("Subrating lost after PHY update: max %lld ms\n", mx);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central PHY-update-while-subrated validated\n");
	bs_trace_silent_exit(0);
}

static void test_central_main_disable(void)
{
	struct bt_conn_le_subrate_param to_one = {
		.subrate_min = 1U,
		.subrate_max = 1U,
		.max_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout = CONN_TIMEOUT_UNITS,
	};
	uint32_t interval_ms;
	int64_t mn, mx;
	int err;

	if (central_start()) {
		return;
	}
	SUBRATE_WAIT(default_conn && subrate_factor >= 2U);
	interval_ms = interval_to_ms(CONN_INTERVAL_UNITS);

	if (probe_minmax(NUM_READS / 2, READ_GAP_MS, &mn, &mx)) {
		return;
	}
	if (mx < (subrate_factor * interval_ms) / 2U) {
		FAIL("No skip before disable: max %lld ms\n", mx);
		return;
	}

	/* Explicitly disable subrating (factor 1); skipping must stop. */
	err = bt_conn_le_subrate_request(default_conn, &to_one);
	if (err) {
		FAIL("Central N->1 subrate request failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(subrate_factor == 1U);
	printk("Central: subrating disabled (factor 1)\n");

	if (probe_minmax(NUM_READS / 2, READ_GAP_MS, &mn, &mx)) {
		return;
	}
	if (mx >= 3U * interval_ms) {
		FAIL("Still skipping after N->1 disable: max %lld ms\n", mx);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central N->1 disable validated (max latency %lld ms)\n", mx);
	bs_trace_silent_exit(0);
}

/* Notifications-under-subrating: discover HRS, subscribe, and count the
 * notifications that arrive while subrated (the peripheral can only send them
 * on subrated/continuation events, so this checks the data-TX path is alive).
 */
static struct bt_uuid_16 nfy_uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_discover_params nfy_disc;
static struct bt_gatt_subscribe_params nfy_sub;
static volatile int notify_count;
static volatile bool subscribed;

static uint8_t notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
			 const void *data, uint16_t length)
{
	if (!data) {
		return BT_GATT_ITER_STOP;
	}
	notify_count++;
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t notify_disc_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		return BT_GATT_ITER_STOP;
	}

	if (!bt_uuid_cmp(nfy_disc.uuid, BT_UUID_HRS)) {
		memcpy(&nfy_uuid, BT_UUID_HRS_MEASUREMENT, sizeof(nfy_uuid));
		nfy_disc.uuid = &nfy_uuid.uuid;
		nfy_disc.start_handle = attr->handle + 1;
		nfy_disc.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		(void)bt_gatt_discover(conn, &nfy_disc);
	} else if (!bt_uuid_cmp(nfy_disc.uuid, BT_UUID_HRS_MEASUREMENT)) {
		memcpy(&nfy_uuid, BT_UUID_GATT_CCC, sizeof(nfy_uuid));
		nfy_disc.uuid = &nfy_uuid.uuid;
		nfy_disc.start_handle = attr->handle + 2;
		nfy_disc.type = BT_GATT_DISCOVER_DESCRIPTOR;
		nfy_sub.value_handle = attr->handle + 1;
		(void)bt_gatt_discover(conn, &nfy_disc);
	} else {
		nfy_sub.notify = notify_cb;
		nfy_sub.value = BT_GATT_CCC_NOTIFY;
		nfy_sub.ccc_handle = attr->handle;
		err = bt_gatt_subscribe(conn, &nfy_sub);
		if (err && err != -EALREADY) {
			FAIL("Central subscribe failed (err %d)\n", err);
		} else {
			subscribed = true;
		}
	}

	return BT_GATT_ITER_STOP;
}

static void test_central_main_notify(void)
{
	int before;
	int err;

	if (central_start()) {
		return;
	}
	SUBRATE_WAIT(central_connected);

	memcpy(&nfy_uuid, BT_UUID_HRS, sizeof(nfy_uuid));
	nfy_disc.uuid = &nfy_uuid.uuid;
	nfy_disc.func = notify_disc_cb;
	nfy_disc.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	nfy_disc.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	nfy_disc.type = BT_GATT_DISCOVER_PRIMARY;
	err = bt_gatt_discover(default_conn, &nfy_disc);
	if (err) {
		FAIL("Central discover failed (err %d)\n", err);
		return;
	}

	SUBRATE_WAIT(subscribed && subrate_factor >= 2U);
	printk("Central: subscribed, subrating factor %u\n", subrate_factor);

	before = notify_count;
	k_sleep(K_SECONDS(4));
	printk("Central: %d notifications under subrating\n", notify_count - before);
	/* The peripheral notifies every SUBRATE_NOTIFY_PERIOD_MS (> the skip period),
	 * so ~4000/period are expected; each is queued during a skip and must wake
	 * the peripheral on a subrated event. Require most of them to arrive.
	 */
	if ((notify_count - before) < (4000 / SUBRATE_NOTIFY_PERIOD_MS) - 3) {
		FAIL("Too few notifications under subrating: %d\n", notify_count - before);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central notifications-under-subrating validated (%d in 4 s)\n",
	     notify_count - before);
	bs_trace_silent_exit(0);
}

static void test_central_main_supervision(void)
{
	struct bt_conn_le_subrate_param hi = {
		.subrate_min = 1U,
		.subrate_max = SUBRATE_SUPERVISION_FACTOR,
		.max_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout = CONN_TIMEOUT_UNITS,
	};
	int64_t lat;

	if (central_start()) {
		return;
	}
	/* Raise the acceptable factor ceiling so the large request is granted. */
	(void)bt_conn_le_subrate_set_defaults(&hi);

	SUBRATE_WAIT(default_conn && subrate_factor == SUBRATE_SUPERVISION_FACTOR);
	printk("Central: factor %u, skip ~%u ms, supervision %u ms\n", subrate_factor,
	       subrate_factor * interval_to_ms(CONN_INTERVAL_UNITS), CONN_TIMEOUT_UNITS * 10U);

	/* Idle for many skip cycles with no data: the link survives only if the
	 * peripheral keeps listening on subrated events within the supervision
	 * timeout - a too-long skip would drop the link.
	 */
	k_sleep(K_SECONDS(10));
	if (!default_conn) {
		FAIL("Link dropped while idle-subrated near the supervision bound\n");
		return;
	}
	/* And it is still functional. */
	lat = probe_read_latency();
	if (lat < 0) {
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central supervision-boundary validated (factor %u survived 10 s idle)\n",
	     subrate_factor);
	bs_trace_silent_exit(0);
}

/* Central->Peripheral continuation: discover the write-without-response
 * characteristic and burst data to it (one write per connection event).
 */
static struct bt_uuid_128 cw_uuid;
static struct bt_gatt_discover_params cw_disc;
static volatile uint16_t cwrite_handle;

static uint8_t cwrite_disc_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      struct bt_gatt_discover_params *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);
	if (!attr) {
		return BT_GATT_ITER_STOP;
	}
	cwrite_handle = bt_gatt_attr_value_handle(attr);
	return BT_GATT_ITER_STOP;
}

static void test_central_main_cwrite(void)
{
	uint8_t val = 0;
	uint32_t period_ms;
	int64_t end;
	int err;

	if (central_start()) {
		return;
	}
	SUBRATE_WAIT(central_connected && subrate_factor == SUBRATE_CONT_FACTOR);

	memcpy(&cw_uuid, BT_UUID_DECLARE_128(CWRITE_CHR_UUID), sizeof(cw_uuid));
	cw_disc.uuid = &cw_uuid.uuid;
	cw_disc.func = cwrite_disc_cb;
	cw_disc.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	cw_disc.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	cw_disc.type = BT_GATT_DISCOVER_CHARACTERISTIC;
	err = bt_gatt_discover(default_conn, &cw_disc);
	if (err) {
		FAIL("Central discover (cwrite) failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(cwrite_handle != 0U);
	printk("Central: bursting writes to handle %u\n", cwrite_handle);

	/* One write per connection event for several seconds. If the Central's own
	 * Tx opens the continuation window the writes go out at the event rate;
	 * otherwise they are throttled to the subrate cadence.
	 */
	period_ms = interval_to_ms(CONN_INTERVAL_UNITS);
	end = k_uptime_get() + 6000;
	while (k_uptime_get() < end) {
		err = bt_gatt_write_without_response(default_conn, cwrite_handle, &val,
						     sizeof(val), false);
		if (err == -ENOMEM) {
			k_sleep(K_MSEC(period_ms));
			continue;
		}
		if (err) {
			FAIL("Central write-without-response failed (err %d)\n", err);
			return;
		}
		val++;
		k_sleep(K_MSEC(period_ms));
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central C->P continuation burst sent\n");
	bs_trace_silent_exit(0);
}

#if defined(CONFIG_BT_LE_EXTENDED_FEAT_SET)
/* LL Extended Feature Set (Feature Page Exchange) end-to-end check. The central
 * connects to a single peripheral, then issues an LE Read All Remote Features
 * (HCI 0x2088) for page 1. The completion (meta-event subevent 0x2B) carries the
 * peer's 248-octet feature field plus max_remote_page / max_valid_page. Since
 * page 1 currently carries no feature bits (ll_feat_local_max_page() == 0), the
 * peer returns page 0 + an all-zero page 1 with max page 0. Asserting that the
 * page-0 octets came through non-zero (the peer's real features) while page 1 is
 * zero proves the procedure ran, the link survived, and the 248-octet carrier
 * was populated from the peer without corruption.
 */
static volatile bool efs_complete;
static volatile uint8_t efs_status;
static volatile uint8_t efs_max_remote_page;
static volatile uint8_t efs_max_valid_page;
static uint8_t efs_features[248];

static void efs_read_all_remote_feat_complete(
	struct bt_conn *conn,
	const struct bt_conn_le_read_all_remote_feat_complete *params)
{
	efs_status = params->status;
	efs_max_remote_page = params->max_remote_page;
	efs_max_valid_page = params->max_valid_page;
	if (params->status == BT_HCI_ERR_SUCCESS && params->features != NULL) {
		memcpy(efs_features, params->features, sizeof(efs_features));
	}
	efs_complete = true;
	printk("Central read-all-remote-features complete: status 0x%02x "
	       "max_remote_page %u max_valid_page %u\n", params->status,
	       params->max_remote_page, params->max_valid_page);
}

static struct bt_conn_cb efs_conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.read_all_remote_feat_complete = efs_read_all_remote_feat_complete,
};

static void test_central_main_efs(void)
{
	bool page0_nonzero = false;
	bool page1_zero = true;
	int err;
	int i;

	bt_conn_cb_register(&efs_conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("Bluetooth init failed (err %d)\n", err);
		return;
	}
	printk("Central Bluetooth initialized (EFS)\n");

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		FAIL("Scanning failed to start (err %d)\n", err);
		return;
	}

	SUBRATE_WAIT(central_connected && default_conn);

	/* Let the connection (and any autonomous feature exchange) settle. */
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	err = bt_conn_le_read_all_remote_features(default_conn, 1U);
	if (err) {
		FAIL("Central read-all-remote-features request failed (err %d)\n", err);
		return;
	}
	printk("Central requested all remote features (1 page)\n");

	/* Wait for the completion, mirroring SUBRATE_WAIT. */
	SUBRATE_WAIT(efs_complete);

	if (efs_status != BT_HCI_ERR_SUCCESS) {
		FAIL("Read-all-remote-features completed with status 0x%02x\n",
		     efs_status);
		return;
	}
	/* The link must not have dropped during the procedure. */
	if (!central_connected || !default_conn) {
		FAIL("Link dropped during the feature page exchange\n");
		return;
	}

	/* Page 0 occupies octets 0..7; the peer has page-0 features, so the
	 * carrier must be non-zero -> the 248-octet field was really populated.
	 */
	for (i = 0; i < BT_HCI_LE_BYTES_PAGE_0_FEATURE_PAGE; i++) {
		if (efs_features[i] != 0U) {
			page0_nonzero = true;
			break;
		}
	}
	/* Page 1 occupies octets 8..31; no page-1 bits are implemented yet, so it
	 * must be all-zero and the reported max pages must be 0.
	 */
	for (i = BT_HCI_LE_BYTES_PAGE_0_FEATURE_PAGE;
	     i < BT_HCI_LE_BYTES_PAGE_0_FEATURE_PAGE + BT_HCI_LE_BYTES_PER_FEATURE_PAGE;
	     i++) {
		if (efs_features[i] != 0U) {
			page1_zero = false;
			break;
		}
	}

	if (!page0_nonzero) {
		FAIL("Page-0 features all zero: 248-octet carrier not populated\n");
		return;
	}
	if (!page1_zero) {
		FAIL("Page-1 octets non-zero but no page-1 bits are implemented\n");
		return;
	}
	if (efs_max_remote_page != 0U || efs_max_valid_page != 0U) {
		FAIL("Unexpected max pages: remote %u valid %u (expected 0/0)\n",
		     efs_max_remote_page, efs_max_valid_page);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	printk("Central EFS page0 octets: %02x %02x %02x %02x %02x %02x %02x %02x, "
	       "max_remote_page %u max_valid_page %u\n",
	       efs_features[0], efs_features[1], efs_features[2], efs_features[3],
	       efs_features[4], efs_features[5], efs_features[6], efs_features[7],
	       efs_max_remote_page, efs_max_valid_page);
	PASS("Central feature page exchange validated (page0 populated, page1 zero, "
	     "max page 0)\n");
	bs_trace_silent_exit(0);
}
#endif /* CONFIG_BT_LE_EXTENDED_FEAT_SET */

#if defined(CONFIG_BT_FRAME_SPACE_UPDATE)
/* Frame Space Update (Core 6.2, LL feature bit 65) end-to-end. The Central
 * connects, exchanges feature page 1 (FSU lives on page 1, so the peer's bit 65
 * must be known before initiating), then runs bt_conn_le_frame_space_update.
 *
 * The responder (the peripheral controller) negotiates a frame space clamped to
 * its configured floor (CONFIG_BT_CTLR_FSU_MIN_FRAME_SPACE_US = 80 us in
 * overlay-fsu): a request for FS_Min = 60 us is granted as exactly 80 us. The
 * apply then retimes the radio (shorter tIFS) and shrinks the scheduler slot
 * reservation -- FSU's whole value -- which the test proves by observing
 * ll_test_conn_ticks_slot[] strictly decrease across the exchange (needs
 * CONFIG_BT_CTLR_TEST_CONN_TICKS_SLOT, set in overlay-fsu). On the idealized
 * bsim radio the link MUST survive the retiming; a drop would mean an FSM or
 * timing bug. A second request whose entire range sits below the floor
 * (FS_Max = 70 < 80) must be rejected (BT_HCI_ERR_UNSUPP_FEATURE_PARAM_VAL,
 * 0x11), never clamped below the floor.
 *
 * The peer is peripheral_plain: the controller answers the LL_FRAME_SPACE_REQ as
 * the responder with no host action, so no FSU-specific peripheral test is
 * needed (and peripheral_plain is unguarded, so it builds in the FSU-off configs
 * too).
 */
static volatile bool fsu_done;
static volatile uint8_t fsu_status = 0xFFU;
static volatile uint16_t fsu_frame_space;
static volatile uint8_t fsu_initiator = 0xFFU;

static void fsu_updated(struct bt_conn *conn, uint8_t status,
			const struct bt_conn_le_frame_space_info *params)
{
	ARG_UNUSED(conn);

	fsu_status = status;
	if (params != NULL) {
		fsu_frame_space = params->frame_space;
		fsu_initiator = params->initiator;
	}
	fsu_done = true;
	printk("Central frame space updated: status 0x%02x fs %u us initiator %u\n",
	       status, params != NULL ? params->frame_space : 0U,
	       params != NULL ? params->initiator : 0xFFU);
}

static struct bt_conn_cb fsu_conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	/* FSU is on feature page 1; reuse the EFS completion cb so the peer's
	 * page-1 features (incl. its FSU bit 65) are known before initiating.
	 */
	.read_all_remote_feat_complete = efs_read_all_remote_feat_complete,
	.frame_space_updated = fsu_updated,
};

static void test_central_main_fsu(void)
{
	struct bt_conn_le_frame_space_param param = {
		.frame_space_min = 60U,   /* below the 80 us floor -> clamped up */
		.frame_space_max = 150U,
		.phys = BT_HCI_LE_FRAME_SPACE_UPDATE_PHY_1M_MASK |
			BT_HCI_LE_FRAME_SPACE_UPDATE_PHY_2M_MASK,
		.spacing_types =
			BT_HCI_LE_FRAME_SPACE_UPDATE_SPACING_TYPE_IFS_ACL_CP_MASK |
			BT_HCI_LE_FRAME_SPACE_UPDATE_SPACING_TYPE_IFS_ACL_PC_MASK,
	};
	int err;
#if defined(CONFIG_BT_CTLR_TEST_CONN_TICKS_SLOT)
	uint32_t slot_before, slot_after;
	uint16_t fsu_handle;
#endif

	bt_conn_cb_register(&fsu_conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("Bluetooth init failed (err %d)\n", err);
		return;
	}
	printk("Central Bluetooth initialized (FSU)\n");

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		FAIL("Scanning failed to start (err %d)\n", err);
		return;
	}

	SUBRATE_WAIT(central_connected && default_conn);
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	/* Exchange feature page 1 so the peer's FSU bit 65 is known (Core 6.2). */
	efs_complete = false;
	err = bt_conn_le_read_all_remote_features(default_conn, 1U);
	if (err) {
		FAIL("Central read-all-remote-features request failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(efs_complete);
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

#if defined(CONFIG_BT_CTLR_TEST_CONN_TICKS_SLOT)
	/* Record the scheduler slot reservation at the established default 150 us
	 * inter-frame space, before the FSU shortens it. The link is a normal
	 * (non-reduced-CE) link, so the reservation is the full airtime slot and a
	 * 150->80 us tIFS drop (70 us, several ticker ticks) must shrink it.
	 */
	if (bt_hci_get_conn_handle(default_conn, &fsu_handle)) {
		FAIL("Could not read the FSU connection handle\n");
		return;
	}
	slot_before = ll_test_conn_ticks_slot[fsu_handle];
	printk("FSU slot reservation before update: %u ticks (handle %u)\n",
	       slot_before, fsu_handle);
#endif /* CONFIG_BT_CTLR_TEST_CONN_TICKS_SLOT */

	/* Request a frame space; FS_Min = 60 us is below the responder's 80 us
	 * floor, so the negotiated value must clamp up to exactly 80 us.
	 */
	fsu_done = false;
	err = bt_conn_le_frame_space_update(default_conn, &param);
	if (err) {
		FAIL("Central frame space update request failed (err %d)\n", err);
		return;
	}
	printk("Central requested a frame space update (min %u max %u us)\n",
	       param.frame_space_min, param.frame_space_max);

	SUBRATE_WAIT(fsu_done);
	if (fsu_status != BT_HCI_ERR_SUCCESS) {
		FAIL("Frame space update failed (status 0x%02x)\n", fsu_status);
		return;
	}
	if (fsu_frame_space != 80U) {
		FAIL("Negotiated frame space = %u us, expected 80 (floor-clamped from "
		     "%u)\n", fsu_frame_space, param.frame_space_min);
		return;
	}
	if (fsu_initiator != BT_HCI_LE_FRAME_SPACE_UPDATE_INITIATOR_LOCAL_HOST) {
		FAIL("Unexpected FSU initiator %u (expected LOCAL_HOST %u)\n",
		     fsu_initiator, BT_HCI_LE_FRAME_SPACE_UPDATE_INITIATOR_LOCAL_HOST);
		return;
	}
	printk("Central frame space negotiated at %u us (floor-clamped from %u)\n",
	       fsu_frame_space, param.frame_space_min);

	/* The apply retimes the radio (shorter tIFS) and shrinks the slot; on the
	 * idealized bsim radio the link must still survive (an FSM/timing bug would
	 * desync / drop it). Hold it to let the shorter event run and the
	 * ull_conn_done slot recompute settle over many connection events.
	 */
	k_sleep(K_MSEC(2000));
	if (!central_connected || !default_conn) {
		FAIL("Link dropped after the frame space update (the shorter "
		     "inter-frame space must not desync the link)\n");
		return;
	}

#if defined(CONFIG_BT_CTLR_TEST_CONN_TICKS_SLOT)
	/* Prove FSU's value: the shorter inter-frame space must have shrunk the
	 * scheduler slot reservation. The 70 us tIFS drop (150 -> 80) exceeds one
	 * HAL_TICKER_US_TO_TICKS_CEIL boundary, so on this normal (full-slot) link
	 * the recomputed reservation must be STRICTLY smaller than before.
	 */
	slot_after = ll_test_conn_ticks_slot[fsu_handle];
	printk("FSU slot reservation after update: %u ticks (was %u)\n",
	       slot_after, slot_before);
	if (slot_after >= slot_before) {
		FAIL("Slot reservation did not shrink after FSU: before %u, after %u "
		     "ticks (150->80 us tIFS should shorten the event)\n",
		     slot_before, slot_after);
		return;
	}
	printk("FSU slot reservation shrank: %u -> %u ticks\n", slot_before,
	       slot_after);
#endif /* CONFIG_BT_CTLR_TEST_CONN_TICKS_SLOT */

	/* A request whose entire range is below the floor (FS_Max = 70 < 80) must
	 * be rejected by the responder, not clamped below the floor. The host range
	 * check passes 40/70, so the controller responder is the gate.
	 */
	{
		struct bt_conn_le_frame_space_param below = param;

		below.frame_space_min = 40U;
		below.frame_space_max = 70U;
		fsu_done = false;
		fsu_status = 0xFFU;
		err = bt_conn_le_frame_space_update(default_conn, &below);
		if (err) {
			/* Rejected already at the request: also a valid below-floor
			 * outcome (not applied).
			 */
			printk("Below-floor frame space update rejected at request "
			       "(err %d)\n", err);
		} else {
			SUBRATE_WAIT(fsu_done);
			if (fsu_status == BT_HCI_ERR_SUCCESS) {
				FAIL("Below-floor frame space update {40,70} was "
				     "accepted\n");
				return;
			}
			printk("Below-floor frame space update correctly rejected "
			       "(status 0x%02x)\n", fsu_status);
		}
	}

	if (!central_connected || !default_conn) {
		FAIL("Link dropped after the below-floor reject\n");
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central FSU validated: 60->80 us floor clamp accepted, slot "
	     "reservation shrank, below-floor {40,70} rejected, link survived\n");
}
#endif /* CONFIG_BT_FRAME_SPACE_UPDATE */

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
static volatile bool sci_changed;
static volatile uint8_t sci_status = 0xFFU;
static volatile uint16_t sci_subrate_factor;

static void sci_conn_rate_changed(struct bt_conn *conn, uint8_t status,
				  const struct bt_conn_le_conn_rate_changed *params)
{
	ARG_UNUSED(conn);

	sci_status = status;
	if (params != NULL) {
		sci_subrate_factor = params->subrate_factor;
	}
	sci_changed = true;
	printk("Central connection rate changed: status 0x%02x factor %u\n", status,
	       params != NULL ? params->subrate_factor : 0U);
}

static struct bt_conn_cb sci_conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	/* Reuse the EFS page-exchange completion callback (SCI depends on EFS) so
	 * we know the peer's page-1 features (incl. the SCI Host Support bit) are
	 * available before initiating.
	 */
	.read_all_remote_feat_complete = efs_read_all_remote_feat_complete,
	.conn_rate_changed = sci_conn_rate_changed,
};

/* For the multi-link coexistence test: central_init_multi already registers
 * conn_callbacks (multi `connected` counts m_nconns), so this second registered
 * cb only adds the SCI-specific events. The fork's bt_conn_cb_register appends to
 * a list, so both fire.
 */
static struct bt_conn_cb sci_extra_callbacks = {
	.read_all_remote_feat_complete = efs_read_all_remote_feat_complete,
	.conn_rate_changed = sci_conn_rate_changed,
};

/* RCV-tier Shorter Connection Intervals end-to-end: a Central drives the link to
 * a 1.25 ms connection interval via the Connection Rate Update procedure and
 * verifies the host sees the change (0x37 event) with interval_us == 1250 and the
 * link survives. Requires the page-1 feature exchange first (the Central gates on
 * the peer's SCI Host Support bit 73).
 */
static void test_central_main_sci(void)
{
	struct bt_conn_le_conn_rate_param param = {
		.interval_min_125us = 10U,   /* 1250 us */
		.interval_max_125us = 10U,   /* 1250 us */
		.subrate_min = 1U,
		.subrate_max = 1U,
		.max_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout_10ms = 200U, /* 2 s */
		.min_ce_len_125us = 1U,
		.max_ce_len_125us = 1U,
	};
	struct bt_conn_info info;
	int err;
	int i;

	bt_conn_cb_register(&sci_conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("Bluetooth init failed (err %d)\n", err);
		return;
	}
	printk("Central Bluetooth initialized (SCI)\n");

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		FAIL("Scanning failed to start (err %d)\n", err);
		return;
	}

	SUBRATE_WAIT(central_connected && default_conn);
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	/* Exchange feature page 1 so the Central knows the peer's SCI Host Support
	 * bit before it may initiate (Core 6.2 5.1.32).
	 */
	efs_complete = false;
	err = bt_conn_le_read_all_remote_features(default_conn, 1U);
	if (err) {
		FAIL("Central read-all-remote-features request failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(efs_complete);
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	err = bt_conn_le_conn_rate_request(default_conn, &param);
	if (err) {
		FAIL("Central connection rate request failed (err %d)\n", err);
		return;
	}
	printk("Central requested a 1.25 ms connection rate\n");

	SUBRATE_WAIT(sci_changed);

	if (sci_status != BT_HCI_ERR_SUCCESS) {
		FAIL("Connection rate change failed (status 0x%02x)\n", sci_status);
		return;
	}
	if (!central_connected || !default_conn) {
		FAIL("Link dropped during the connection rate update\n");
		return;
	}

	err = bt_conn_get_info(default_conn, &info);
	if (err) {
		FAIL("bt_conn_get_info failed (err %d)\n", err);
		return;
	}
	printk("Central connection interval after SCI: %u us\n", info.le.interval_us);
	if (info.le.interval_us != 1250U) {
		FAIL("Expected a 1250 us interval, got %u us\n", info.le.interval_us);
		return;
	}

	/* ZMK relevance: the whole point of RCV is low-latency HID over the split
	 * link. On a 1.25 ms link a GATT round-trip should complete in a few ms --
	 * far below what a >=7.5 ms-interval link could deliver. Measure it.
	 */
	{
		int64_t mn, mx;

		if (probe_minmax(4, 100, &mn, &mx)) {
			return;
		}
		printk("Central SCI 1.25 ms link GATT read latency: min %lld ms max %lld ms\n",
		       mn, mx);
		/* A 1.25 ms link delivers a GATT round-trip in a few ms; a >=7.5 ms-min
		 * link (e.g. the default 30 ms) could not. 15 ms tolerates bsim jitter /
		 * first-read ATT setup while still proving the sub-7.5 ms benefit.
		 */
		if (mx > 15) {
			FAIL("1.25 ms link read latency too high (max %lld ms > 15 ms): "
			     "interval/anchor likely wrong\n", mx);
			return;
		}
	}

	/* HCI 0x20A3 (Read Min Supported Conn Interval): the controller must report
	 * the configured floor (625 us = 5 x 125 us) and the ECV-and-not-RCV band it
	 * sustains as one group {5, 9, 1} -- 625..1125 us in 125 us steps. This is the
	 * same floor the enforcement below rejects 500 us against.
	 */
	{
		struct bt_conn_le_min_conn_interval_info min_info;

		err = bt_conn_le_read_min_conn_interval_groups(&min_info);
		if (err) {
			FAIL("Read Min Supported Conn Interval failed (err %d)\n", err);
			return;
		}
		if (min_info.min_supported_conn_interval_us != 625U) {
			FAIL("reported min interval = %u us, expected 625\n",
			     min_info.min_supported_conn_interval_us);
			return;
		}
		if (min_info.num_groups != 1U ||
		    min_info.groups[0].min_125us != 5U ||
		    min_info.groups[0].max_125us != 9U ||
		    min_info.groups[0].stride_125us != 1U) {
			FAIL("unexpected ECV group list: num=%u {%u,%u,%u}\n",
			     min_info.num_groups, min_info.groups[0].min_125us,
			     min_info.groups[0].max_125us, min_info.groups[0].stride_125us);
			return;
		}
		printk("Read Min Supported Conn Interval: %u us, group {%u,%u,%u}\n",
		       min_info.min_supported_conn_interval_us, min_info.groups[0].min_125us,
		       min_info.groups[0].max_125us, min_info.groups[0].stride_125us);
	}

	/* Set Default Rate Parameters (HCI 0x20A2) must accept an ECV default on an
	 * ECV controller -- the old multiple-of-1.25ms gate rejected it -- and still
	 * reject a below-floor interval. Defaults are stored, not applied, so this
	 * exercises only the relaxed validation.
	 */
	{
		struct bt_conn_le_conn_rate_param ecv_def = param;

		ecv_def.interval_min_125us = 5U;   /* 625 us, ECV */
		ecv_def.interval_max_125us = 9U;   /* 1125 us, ECV */
		err = bt_conn_le_conn_rate_set_defaults(&ecv_def);
		if (err) {
			FAIL("ECV default rate params rejected (err %d)\n", err);
			return;
		}

		ecv_def.interval_min_125us = 4U;   /* 500 us, below the 625 us floor */
		ecv_def.interval_max_125us = 4U;
		err = bt_conn_le_conn_rate_set_defaults(&ecv_def);
		if (err == 0) {
			FAIL("Below-floor default rate params accepted\n");
			return;
		}
		printk("Set Default Rate Parameters: ECV accepted, below-floor rejected\n");
	}

	/* Interval enforcement: sub-floor and tier-straddle intervals the host range
	 * check (floor 375 us) lets through must be rejected by the controller. The
	 * controller's ECV floor is configurable (BT_CTLR_SCI_ECV_INTERVAL_MIN_125US,
	 * = 5 / 625 us here = the nRF54L15 HW floor), so even a spec-valid 500 us is
	 * rejected. (A valid ECV interval >= the floor, such as 625 us, is ACCEPTED --
	 * tested below.)
	 */
	{
		static const uint16_t bad_125us[][2] = {
			{2U, 2U},    /* 250 us, below the spec 375 us ECV floor */
			{4U, 4U},    /* 500 us, valid spec ECV but below the 625 us HW floor */
			{8U, 10U},   /* straddle: 8 (ECV) with 10 (RCV) */
		};

		for (i = 0; i < (int)ARRAY_SIZE(bad_125us); i++) {
			struct bt_conn_le_conn_rate_param bad = param;

			bad.interval_min_125us = bad_125us[i][0];
			bad.interval_max_125us = bad_125us[i][1];
			err = bt_conn_le_conn_rate_request(default_conn, &bad);
			if (err == 0) {
				FAIL("Invalid interval {%u,%u} (125us) was accepted, "
				     "expected reject\n", bad_125us[i][0], bad_125us[i][1]);
				return;
			}
			printk("Invalid interval {%u,%u} correctly rejected (err %d)\n",
			       bad_125us[i][0], bad_125us[i][1], err);
		}
	}

	/* ECV: a 125 us-granular sub-1.25 ms interval (625 us = 5 x 125 us, not a
	 * multiple of 1.25 ms) must be accepted and applied -- the controller is the
	 * sole gate (the host floor is 375 us).
	 */
	{
		struct bt_conn_le_conn_rate_param ecv = param;

		ecv.interval_min_125us = 5U;   /* 625 us */
		ecv.interval_max_125us = 5U;
		sci_changed = false;
		err = bt_conn_le_conn_rate_request(default_conn, &ecv);
		if (err) {
			FAIL("ECV 625 us request rejected (err %d)\n", err);
			return;
		}
		SUBRATE_WAIT(sci_changed);
		if (sci_status != BT_HCI_ERR_SUCCESS) {
			FAIL("ECV 625 us change failed (status 0x%02x)\n", sci_status);
			return;
		}
		err = bt_conn_get_info(default_conn, &info);
		if (err || info.le.interval_us != 625U) {
			FAIL("ECV interval after change = %u us, expected 625\n",
			     info.le.interval_us);
			return;
		}
		printk("ECV: 625 us interval applied (interval_us = %u)\n",
		       info.le.interval_us);
	}

	/* Hold the ECV 625 us link to confirm both ends stay anchored (no
	 * supervision timeout / desync after the instant).
	 */
	k_sleep(K_MSEC(2000));
	if (!central_connected || !default_conn) {
		FAIL("ECV 625 us link dropped after the connection rate update\n");
		return;
	}

	PASS("Central SCI test passed: RCV 1.25 ms + ECV 625 us, low-latency reads, "
	     "invalid intervals rejected\n");
}

/* RCV Shorter Connection Intervals fused with subrating (factor > 1). Exercises
 * the connSubrateBaseEvent fix: with a subrate factor the live subrate skipper
 * (ull_conn.c) makes both ends skip to the SAME subrated events only if they
 * agree on connSubrateBaseEvent (= the wire Instant); a disagreement desyncs the
 * anchor and drops the link on supervision timeout. So a held factor>1 1.25 ms
 * link validates the fix. Mirrors the ZMK idle scenario: a low-latency link that
 * subrates for power when idle.
 */
static void test_central_main_sci_subrate(void)
{
	struct bt_conn_le_conn_rate_param param = {
		.interval_min_125us = 10U,   /* 1250 us */
		.interval_max_125us = 10U,
		.subrate_min = 1U,
		.subrate_max = 4U,           /* effective 4 x 1.25 ms = 5 ms when idle */
		.max_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout_10ms = 200U, /* 2 s */
		.min_ce_len_125us = 1U,
		.max_ce_len_125us = 1U,
	};
	struct bt_conn_info info;
	int64_t mn, mx;
	int err;

	bt_conn_cb_register(&sci_conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("Bluetooth init failed (err %d)\n", err);
		return;
	}
	printk("Central Bluetooth initialized (SCI subrate)\n");

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		FAIL("Scanning failed to start (err %d)\n", err);
		return;
	}

	SUBRATE_WAIT(central_connected && default_conn);
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	efs_complete = false;
	err = bt_conn_le_read_all_remote_features(default_conn, 1U);
	if (err) {
		FAIL("Central read-all-remote-features request failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(efs_complete);
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	err = bt_conn_le_conn_rate_request(default_conn, &param);
	if (err) {
		FAIL("Central connection rate (subrate) request failed (err %d)\n", err);
		return;
	}
	printk("Central requested 1.25 ms + subrate factor up to %u\n", param.subrate_max);

	SUBRATE_WAIT(sci_changed);
	if (sci_status != BT_HCI_ERR_SUCCESS) {
		FAIL("Connection rate (subrate) change failed (status 0x%02x)\n", sci_status);
		return;
	}
	if (sci_subrate_factor < 2U) {
		FAIL("Expected a subrate factor >= 2, got %u\n", sci_subrate_factor);
		return;
	}
	err = bt_conn_get_info(default_conn, &info);
	if (err) {
		FAIL("bt_conn_get_info failed (err %d)\n", err);
		return;
	}
	if (info.le.interval_us != 1250U) {
		FAIL("Expected a 1250 us interval, got %u us\n", info.le.interval_us);
		return;
	}
	printk("Central SCI subrate: interval 1250 us, factor %u\n", sci_subrate_factor);

	/* With the skipper live, reads complete only if both ends agree on the
	 * subrated-event phase (the base_event fix). A desync would time out the
	 * read or drop the link. Max latency should sit around factor*interval.
	 */
	if (probe_minmax(NUM_READS, READ_GAP_MS, &mn, &mx)) {
		return;
	}
	printk("Central SCI subrate read latency: min %lld ms max %lld ms (factor %u)\n",
	       mn, mx, sci_subrate_factor);
	if (!central_connected || !default_conn) {
		FAIL("Subrated 1.25 ms link dropped (base_event desync?)\n");
		return;
	}
	if (mx > 4 * sci_subrate_factor * 2) { /* generous: 2x factor*interval(ms-ish) */
		FAIL("Subrate skip latency %lld ms exceeds factor %u cadence "
		     "(phase/base_event wrong)\n", mx, sci_subrate_factor);
		return;
	}

	PASS("Central SCI subrate test passed: factor %u on a 1.25 ms link, link held\n",
	     sci_subrate_factor);
}

#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
/* ZMK split-keyboard coexistence + the reduced-ce reservation gate: two links --
 * one driven to 1.25 ms via SCI, the other left at 30 ms -- run concurrently. The
 * SCI link must keep a fast cadence (not stalled by the slow link) AND the 30 ms
 * link must NOT be starved by the SCI link's per-1.25 ms slot reservation. If the
 * 30 ms link's event floor fails, the full-slot reservation starves coexisting
 * links and the reduced-ce reservation work is required.
 */
/* Coexistence under split load, parameterised by the fast-link interval (125 us
 * units): RCV 1.25 ms (units=10) or ECV 625 us (units=5). The ECV case is the
 * real test of the reduced-CE reservation at standard 150 us tIFS -- a sub-1.25 ms
 * link must not starve a co-resident 30 ms link.
 */
static void sci_coex_run(uint16_t interval_125us)
{
	struct bt_conn_le_conn_rate_param param = {
		.interval_min_125us = interval_125us, .interval_max_125us = interval_125us,
		.subrate_min = 1U, .subrate_max = 1U, .max_latency = 0U,
		.continuation_number = 0U, .supervision_timeout_10ms = 200U,
		.min_ce_len_125us = 1U, .max_ce_len_125us = 1U,
	};
	const uint32_t interval_us = (uint32_t)interval_125us * 125U;
	const bool is_ecv = (interval_125us % 10U) != 0U;
	uint32_t before_sci, before_plain, sci_events, plain_events, expected_sci;
	uint16_t h_sci, h_plain;
	int err, i;

	multi_mode = true;
	if (central_init_multi()) {
		return;
	}
	bt_conn_cb_register(&sci_extra_callbacks);

	for (i = 0; i < 2; i++) {
		err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found_multi);
		if (err) {
			FAIL("Coex: scan start for link %d failed (err %d)\n", i, err);
			return;
		}
		SUBRATE_WAIT(m_nconns > i);
	}
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	if (bt_hci_get_conn_handle(m_conn[0], &h_sci) ||
	    bt_hci_get_conn_handle(m_conn[1], &h_plain)) {
		FAIL("Coex: could not read conn handles\n");
		return;
	}

	/* Drive link 0 to the fast (RCV or ECV) interval via SCI; link 1 stays 30 ms. */
	efs_complete = false;
	err = bt_conn_le_read_all_remote_features(m_conn[0], 1U);
	if (err) {
		FAIL("Coex: read-all-remote-features failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(efs_complete);
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	sci_changed = false;
	err = bt_conn_le_conn_rate_request(m_conn[0], &param);
	if (err) {
		FAIL("Coex: conn rate request failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(sci_changed);
	if (sci_status != BT_HCI_ERR_SUCCESS) {
		FAIL("Coex: conn rate change failed (status 0x%02x)\n", sci_status);
		return;
	}
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	before_sci = ll_test_conn_event_count[h_sci];
	before_plain = ll_test_conn_event_count[h_plain];
	k_sleep(K_MSEC(LAT_COUNT_WINDOW_MS));
	sci_events = ll_test_conn_event_count[h_sci] - before_sci;
	plain_events = ll_test_conn_event_count[h_plain] - before_plain;

	printk("Coex: %s link (%u us) %u events, plain link (30 ms) %u events in %u ms\n",
	       is_ecv ? "ECV" : "RCV", interval_us, sci_events, plain_events,
	       LAT_COUNT_WINDOW_MS);

	/* The fast link runs near window/interval (a >=7.5 ms link could not deliver
	 * this many); require at least 1/3 of nominal, still far above a 7.5 ms
	 * link's ~400.
	 */
	expected_sci = ((uint32_t)LAT_COUNT_WINDOW_MS * 1000U) / interval_us;
	if (sci_events < (expected_sci / 3U)) {
		FAIL("Coex: fast link stalled: %u events (nominal ~%u for %u us)\n",
		     sci_events, expected_sci, interval_us);
		return;
	}
	/* The 30 ms link must not be starved by the fast link's slot reservation:
	 * ~window/30 ms ~= 100; require >= 40 (a dropped link gives ~0). This is the
	 * reduced-CE reservation working -- a full-slot reservation at a sub-1.25 ms
	 * interval would over-reserve and starve the co-resident link.
	 */
	if (plain_events < 40U) {
		FAIL("Coex: 30 ms link starved by the %u us link: %u events (expected "
		     "~100) -- reduced-ce reservation failing\n", interval_us, plain_events);
		return;
	}

	for (i = 0; i < 2; i++) {
		(void)bt_conn_disconnect(m_conn[i], BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
	PASS("Coex: %s %u us link (%u ev) coexisted with a 30 ms link (%u ev)\n",
	     is_ecv ? "ECV" : "RCV", interval_us, sci_events, plain_events);
}

static void test_central_main_sci_coex(void)
{
	sci_coex_run(10U);  /* RCV 1.25 ms */
}

static void test_central_main_ecv_coex(void)
{
	sci_coex_run(5U);   /* ECV 625 us */
}
#endif /* CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT */
#endif /* CONFIG_BT_SHORTER_CONNECTION_INTERVALS */

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

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
/* Connection Rate Update (instant) colliding with a peer Connection Update
 * (instant). The Central drives a 1.25 ms rate update at the same uptime the
 * Peripheral drives a Connection Parameters Request; the instant arbiter must
 * serialise the two. A desync would drop the link within supervision, so
 * survival + a settled interval is the correctness check.
 */
static void test_central_main_sci_collision(void)
{
	struct bt_conn_le_conn_rate_param rate = {
		.interval_min_125us = 10U,   /* 1250 us (RCV) */
		.interval_max_125us = 10U,
		.subrate_min = 1U,
		.subrate_max = 1U,
		.max_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout_10ms = CONN_TIMEOUT_UNITS,
		.min_ce_len_125us = 1U,
		.max_ce_len_125us = 1U,
	};
	struct bt_conn_info info;
	uint32_t iv1;
	int err;

	bt_conn_cb_register(&sci_conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("Bluetooth init failed (err %d)\n", err);
		return;
	}

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		FAIL("Scanning failed to start (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(central_connected && default_conn);
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	/* Feature page 1 must be exchanged before a Connection Rate Update. */
	efs_complete = false;
	err = bt_conn_le_read_all_remote_features(default_conn, 1U);
	if (err) {
		FAIL("Central read-all-remote-features failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(efs_complete);

	while (k_uptime_get() < COLLISION_TIME_MS) {
		k_sleep(K_MSEC(20));
		if (bst_result == Failed) {
			return;
		}
	}
	collision_mode = true;
	(void)bt_conn_le_conn_rate_request(default_conn, &rate);

	k_sleep(K_MSEC(3000));
	if (!central_connected || !default_conn) {
		FAIL("Central lost the link after the SCI/conn-update collision\n");
		return;
	}

	/* Survival across supervision proves the two instants did not desync; a
	 * settled interval (two reads agree) proves it is not still oscillating.
	 */
	if (bt_conn_get_info(default_conn, &info)) {
		FAIL("bt_conn_get_info failed after collision\n");
		return;
	}
	iv1 = info.le.interval_us;
	k_sleep(K_MSEC(500));
	if (bt_conn_get_info(default_conn, &info)) {
		FAIL("bt_conn_get_info (2nd) failed after collision\n");
		return;
	}
	if (iv1 == 0U || info.le.interval_us != iv1) {
		FAIL("Interval not settled after collision: %u then %u us\n",
		     iv1, info.le.interval_us);
		return;
	}
	printk("Central post-collision interval settled at %u us\n", iv1);

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("Central survived SCI/conn-update collision (interval %u us)\n", iv1);
}

/* One-way notification latency (the faithful HID-input direction): discover the
 * peripheral's latency characteristic, subscribe, and measure recv_us - sent_us
 * per notification. Valid because bsim runs both devices on one global sim clock
 * (k_uptime is shared, cf. central_sci_collision).
 */
static struct bt_uuid_128 lat_disc_uuid;
static struct bt_gatt_discover_params lat_disc;
static struct bt_gatt_subscribe_params lat_sub;
static volatile int lat_count;
static volatile bool lat_subscribed;
static int64_t lat_min_us = INT64_MAX;
static int64_t lat_max_us;
static int64_t lat_sum_us;

static uint8_t lat_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
			     const void *data, uint16_t length)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (!data) {
		return BT_GATT_ITER_STOP;
	}
	if (length >= sizeof(uint32_t)) {
		uint32_t sent_us = sys_get_le32(data);
		uint32_t now_us = (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
		int64_t lat = (uint32_t)(now_us - sent_us);

		if (lat < lat_min_us) {
			lat_min_us = lat;
		}
		if (lat > lat_max_us) {
			lat_max_us = lat;
		}
		lat_sum_us += lat;
		lat_count++;
	}
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t lat_disc_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		return BT_GATT_ITER_STOP;
	}

	if (lat_disc.type == BT_GATT_DISCOVER_PRIMARY) {
		memcpy(&lat_disc_uuid, BT_UUID_DECLARE_128(LAT_CHR_UUID), sizeof(lat_disc_uuid));
		lat_disc.uuid = &lat_disc_uuid.uuid;
		lat_disc.start_handle = attr->handle + 1;
		lat_disc.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		(void)bt_gatt_discover(conn, &lat_disc);
	} else if (lat_disc.type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		memcpy(&lat_disc_uuid, BT_UUID_GATT_CCC, sizeof(struct bt_uuid_16));
		lat_disc.uuid = &lat_disc_uuid.uuid;
		lat_disc.start_handle = attr->handle + 2;
		lat_disc.type = BT_GATT_DISCOVER_DESCRIPTOR;
		lat_sub.value_handle = attr->handle + 1;
		(void)bt_gatt_discover(conn, &lat_disc);
	} else {
		lat_sub.notify = lat_notify_cb;
		lat_sub.value = BT_GATT_CCC_NOTIFY;
		lat_sub.ccc_handle = attr->handle;
		err = bt_gatt_subscribe(conn, &lat_sub);
		if (err && err != -EALREADY) {
			FAIL("Central latency subscribe failed (err %d)\n", err);
		} else {
			lat_subscribed = true;
		}
	}

	return BT_GATT_ITER_STOP;
}

static void test_central_main_sci_latency(void)
{
	struct bt_conn_le_conn_rate_param rate = {
		.interval_min_125us = 10U,   /* 1250 us (RCV) */
		.interval_max_125us = 10U,
		.subrate_min = 1U,
		.subrate_max = 1U,
		.max_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout_10ms = CONN_TIMEOUT_UNITS,
		.min_ce_len_125us = 1U,
		.max_ce_len_125us = 1U,
	};
	int err;

	bt_conn_cb_register(&sci_conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("Bluetooth init failed (err %d)\n", err);
		return;
	}

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		FAIL("Scanning failed to start (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(central_connected && default_conn);
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	/* Drive the link to a 1.25 ms RCV interval before measuring. */
	efs_complete = false;
	err = bt_conn_le_read_all_remote_features(default_conn, 1U);
	if (err) {
		FAIL("Central read-all-remote-features failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(efs_complete);

	err = bt_conn_le_conn_rate_request(default_conn, &rate);
	if (err) {
		FAIL("Central connection rate request failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(sci_changed);
	if (sci_status != BT_HCI_ERR_SUCCESS) {
		FAIL("Connection rate change failed (status 0x%02x)\n", sci_status);
		return;
	}

	/* Discover + subscribe to the peripheral's latency characteristic. */
	memcpy(&lat_disc_uuid, BT_UUID_DECLARE_128(LAT_SVC_UUID), sizeof(lat_disc_uuid));
	lat_disc.uuid = &lat_disc_uuid.uuid;
	lat_disc.func = lat_disc_cb;
	lat_disc.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	lat_disc.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	lat_disc.type = BT_GATT_DISCOVER_PRIMARY;
	err = bt_gatt_discover(default_conn, &lat_disc);
	if (err) {
		FAIL("Central latency discover failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(lat_subscribed);

	SUBRATE_WAIT(lat_count >= LAT_NOTIFY_COUNT);
	if (!central_connected || !default_conn) {
		FAIL("Link dropped during latency sampling\n");
		return;
	}

	printk("One-way notification latency @1.25 ms over %d samples: "
	       "min %lld us, avg %lld us, max %lld us\n", lat_count, lat_min_us,
	       lat_sum_us / lat_count, lat_max_us);

	/* One-way (~1 interval + a single host traversal) must beat the round-trip
	 * GATT read (~3-4 ms at 1.25 ms). Loose ceiling; the value is the number.
	 */
	if (lat_max_us > 8000) {
		FAIL("One-way latency too high: max %lld us\n", lat_max_us);
		return;
	}

	(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	PASS("One-way latency @1.25 ms: min %lld / avg %lld / max %lld us (%d samples)\n",
	     lat_min_us, lat_sum_us / lat_count, lat_max_us, lat_count);
}
#endif /* CONFIG_BT_SHORTER_CONNECTION_INTERVALS */

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS) && defined(CONFIG_BT_FRAME_SPACE_UPDATE)
/* ECV floor under a realistic ZMK-split pointing load, with and without FSU --
 * the load-bearing re-test of the empty-PDU floor sweep. The peripheral
 * (peripheral_zmk_load) streams one 8-byte input-event notification per
 * connection interval (LL PDU 17 B unencrypted; +4 B MIC on a real paired
 * link); the central drives the link to the target ECV interval via the
 * Connection Rate procedure, optionally negotiates FSU down to the responder's
 * 80 us floor, subscribes, and measures the delivered notification rate and
 * sequence continuity over a multi-second soak.
 *
 * The link is first updated to 2M PHY: that is what a real ZMK split link
 * negotiates and what Nordic's SDC documents its 750 us floor against (2M +
 * 27-byte DLE). It also keeps every cell airtime-feasible (17 B data + empty
 * PDU + tIFS = ~306 us at 150 us tIFS, ~236 us at 80 us -- both under the
 * smallest 375 us interval), so any observed wall is the SCHEDULER, not raw
 * airtime. On 1M PHY the 375 us probe would be airtime-infeasible regardless
 * (~656 us at 150 us tIFS).
 *
 * Matrix (see the tests_scripts): {750, 625} us WITH FSU are the asserting
 * gate; {750, 625} us WITHOUT FSU are the airtime-vs-scheduler CONTROL (they
 * assert only link survival -- the delivered rate is the DATA and is logged
 * either way); {500, 375} us WITH FSU are informational probes that record
 * where degradation/failure happens without failing.
 */
enum ecv_load_mode {
	ECV_LOAD_GATE,    /* assert survival + rate >= floor + zero seq gaps */
	ECV_LOAD_CONTROL, /* assert survival; rate/gaps logged as DATA */
	ECV_LOAD_PROBE,   /* informational: outcome logged, never fails past setup */
};

static struct bt_uuid_128 load_disc_uuid;
static struct bt_gatt_discover_params load_disc;
static struct bt_gatt_subscribe_params load_sub;
static volatile uint32_t load_count;
static volatile uint32_t load_gaps;
static volatile bool load_seq_seen;
static uint32_t load_prev_seq;
static volatile bool load_subscribed;

static uint8_t load_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
			      const void *data, uint16_t length)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (!data) {
		return BT_GATT_ITER_STOP;
	}
	/* zmk_split_input_event_payload: u8 type, u16 code, u32 value, u8 sync;
	 * the sequence number rides in `value` (offset 3, LE). Baseline from the
	 * first notification seen: anything the peripheral streamed before the
	 * subscribe simply never arrives and is not a gap.
	 */
	if (length == 8U) {
		uint32_t seq = sys_get_le32((const uint8_t *)data + 3);

		if (load_seq_seen && (seq != (load_prev_seq + 1U))) {
			load_gaps++;
		}
		load_prev_seq = seq;
		load_seq_seen = true;
		load_count++;
	}
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t load_disc_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		return BT_GATT_ITER_STOP;
	}

	if (load_disc.type == BT_GATT_DISCOVER_PRIMARY) {
		memcpy(&load_disc_uuid, BT_UUID_DECLARE_128(ECV_LOAD_CHR_UUID),
		       sizeof(load_disc_uuid));
		load_disc.uuid = &load_disc_uuid.uuid;
		load_disc.start_handle = attr->handle + 1;
		load_disc.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		(void)bt_gatt_discover(conn, &load_disc);
	} else if (load_disc.type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		memcpy(&load_disc_uuid, BT_UUID_GATT_CCC, sizeof(struct bt_uuid_16));
		load_disc.uuid = &load_disc_uuid.uuid;
		load_disc.start_handle = attr->handle + 2;
		load_disc.type = BT_GATT_DISCOVER_DESCRIPTOR;
		load_sub.value_handle = attr->handle + 1;
		(void)bt_gatt_discover(conn, &load_disc);
	} else {
		load_sub.notify = load_notify_cb;
		load_sub.value = BT_GATT_CCC_NOTIFY;
		load_sub.ccc_handle = attr->handle;
		err = bt_gatt_subscribe(conn, &load_sub);
		if (err && err != -EALREADY) {
			FAIL("Central load subscribe failed (err %d)\n", err);
		} else {
			load_subscribed = true;
		}
	}

	return BT_GATT_ITER_STOP;
}

/* All the procedure completions the load flow needs: page-1 features (gate for
 * both SCI and knowing the peer's FSU bit), the conn-rate 0x37, the FSU 0x35,
 * and the PHY update. Reuses the handlers defined above.
 */
static struct bt_conn_cb load_conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.le_phy_updated = le_phy_updated,
	.read_all_remote_feat_complete = efs_read_all_remote_feat_complete,
	.conn_rate_changed = sci_conn_rate_changed,
	.frame_space_updated = fsu_updated,
};

/* Bounded wait for the probe cells, which may lose the link at any step. */
static bool ecv_load_wait(volatile bool *flag, int32_t timeout_ms)
{
	int64_t end = k_uptime_get() + timeout_ms;

	while (!*flag && (k_uptime_get() < end) && (bst_result != Failed)) {
		k_sleep(K_MSEC(20));
	}
	return *flag;
}

static void ecv_load_run(uint16_t interval_125us, bool use_fsu, enum ecv_load_mode mode)
{
	struct bt_conn_le_conn_rate_param rate = {
		.interval_min_125us = interval_125us,
		.interval_max_125us = interval_125us,
		.subrate_min = 1U,
		.subrate_max = 1U,
		.max_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout_10ms = CONN_TIMEOUT_UNITS,
		.min_ce_len_125us = 1U,
		.max_ce_len_125us = 1U,
	};
	const uint32_t interval_us = (uint32_t)interval_125us * 125U;
	const char *mode_str = (mode == ECV_LOAD_GATE) ? "GATE"
			     : (mode == ECV_LOAD_CONTROL) ? "CONTROL" : "PROBE";
	uint32_t delivered, gaps, expected, pct, before, gaps_before;
	uint32_t offered, pct_off;
	uint64_t period_ticks;
	uint32_t ce_before = 0U, ce_events = 0U;
	uint16_t applied_tifs_us = 150U;
	uint16_t load_handle = 0xFFFFU;
	struct bt_conn_info info;
	int64_t t0, elapsed_ms;
	bool survived;
	int err;

	bt_conn_cb_register(&load_conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("Bluetooth init failed (err %d)\n", err);
		return;
	}
	printk("Central Bluetooth initialized (ECV load, %u us, FSU %s, %s)\n",
	       interval_us, use_fsu ? "on" : "off", mode_str);

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		FAIL("Scanning failed to start (err %d)\n", err);
		return;
	}

	/* Setup at the default 30 ms interval is floor-independent: any failure
	 * up to (and including) the rate-change request is suite/config breakage
	 * and FAILs in every mode, probes included.
	 */
	SUBRATE_WAIT(central_connected && default_conn);
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	efs_complete = false;
	err = bt_conn_le_read_all_remote_features(default_conn, 1U);
	if (err) {
		FAIL("Central read-all-remote-features request failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(efs_complete);
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	/* 2M PHY first, while still at 30 ms (an instant procedure is safest on
	 * the slow link; the conn-rate apply does not touch the PHY).
	 */
	phy_updated = false;
	err = bt_conn_le_phy_update(default_conn, BT_CONN_LE_PHY_PARAM_2M);
	if (err) {
		FAIL("Central PHY update request failed (err %d)\n", err);
		return;
	}
	SUBRATE_WAIT(phy_updated);

	sci_changed = false;
	err = bt_conn_le_conn_rate_request(default_conn, &rate);
	if (err) {
		FAIL("Central conn rate request for %u us rejected (err %d) -- the "
		     "overlay ECV floor should accept it\n", interval_us, err);
		return;
	}
	SUBRATE_WAIT(sci_changed);
	if (sci_status != BT_HCI_ERR_SUCCESS) {
		FAIL("Conn rate change to %u us failed (status 0x%02x)\n",
		     interval_us, sci_status);
		return;
	}
	err = bt_conn_get_info(default_conn, &info);
	if (err || info.le.interval_us != interval_us) {
		FAIL("Interval after rate change = %u us, expected %u\n",
		     err ? 0U : info.le.interval_us, interval_us);
		return;
	}
	printk("Central ECV load: %u us interval applied\n", interval_us);
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	/* From here on the short interval is live: in PROBE mode outcomes are
	 * data, not failures.
	 */
	if (use_fsu) {
		/* FSU must run AFTER the rate change: the conn-rate apply resets
		 * the tIFS trio to the standard 150 us (ull_conn.c, the tIFS leg of
		 * the decoupled apply), so an earlier negotiation would be undone.
		 */
		struct bt_conn_le_frame_space_param fsp = {
			.frame_space_min = ECV_LOAD_FSU_REQ_MIN_US,
			.frame_space_max = ECV_LOAD_FSU_REQ_MAX_US,
			.phys = BT_HCI_LE_FRAME_SPACE_UPDATE_PHY_1M_MASK |
				BT_HCI_LE_FRAME_SPACE_UPDATE_PHY_2M_MASK,
			.spacing_types =
				BT_HCI_LE_FRAME_SPACE_UPDATE_SPACING_TYPE_IFS_ACL_CP_MASK |
				BT_HCI_LE_FRAME_SPACE_UPDATE_SPACING_TYPE_IFS_ACL_PC_MASK,
		};
		bool fsu_ok = false;

		fsu_done = false;
		err = bt_conn_le_frame_space_update(default_conn, &fsp);
		if (!err && ecv_load_wait(&fsu_done, ECV_LOAD_STEP_TIMEOUT_MS) &&
		    fsu_status == BT_HCI_ERR_SUCCESS &&
		    fsu_frame_space == ECV_LOAD_FSU_FLOOR_US) {
			fsu_ok = true;
			applied_tifs_us = ECV_LOAD_FSU_FLOOR_US;
		}
		if (!fsu_ok) {
			if (mode != ECV_LOAD_PROBE) {
				FAIL("FSU negotiation failed at %u us (err %d, status "
				     "0x%02x, fs %u us)\n", interval_us, err, fsu_status,
				     fsu_frame_space);
				return;
			}
			printk("ECV-LOAD PROBE %u us: FSU negotiation FAILED (err %d, "
			       "status 0x%02x) -- soaking at 150 us tIFS\n",
			       interval_us, err, fsu_status);
		} else {
			printk("Central ECV load: FSU negotiated %u us tIFS\n",
			       applied_tifs_us);
		}
		k_sleep(K_MSEC(500));
	}

	if (!central_connected || !default_conn) {
		if (mode == ECV_LOAD_PROBE) {
			PASS("ECV-LOAD PROBE %u us: link DROPPED before the soak "
			     "(post-rate-change) -- floor is above this interval\n",
			     interval_us);
			return;
		}
		FAIL("Link dropped before the soak at %u us\n", interval_us);
		return;
	}

	/* Discover + subscribe to the load characteristic. */
	memcpy(&load_disc_uuid, BT_UUID_DECLARE_128(ECV_LOAD_SVC_UUID),
	       sizeof(load_disc_uuid));
	load_disc.uuid = &load_disc_uuid.uuid;
	load_disc.func = load_disc_cb;
	load_disc.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	load_disc.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	load_disc.type = BT_GATT_DISCOVER_PRIMARY;
	err = bt_gatt_discover(default_conn, &load_disc);
	if (err) {
		FAIL("Central load discover failed (err %d)\n", err);
		return;
	}
	if (!ecv_load_wait(&load_subscribed, ECV_LOAD_STEP_TIMEOUT_MS)) {
		if (mode == ECV_LOAD_PROBE) {
			PASS("ECV-LOAD PROBE %u us: subscribe never completed (link "
			     "%s) -- link cannot carry GATT setup at this interval\n",
			     interval_us, central_connected ? "up" : "DROPPED");
			return;
		}
		FAIL("Central load subscribe timed out at %u us\n", interval_us);
		return;
	}

	/* Wait for the stream to start, then let the cadence settle. */
	{
		int64_t end = k_uptime_get() + ECV_LOAD_STEP_TIMEOUT_MS;

		while (load_count == 0U && k_uptime_get() < end &&
		       (bst_result != Failed)) {
			k_sleep(K_MSEC(20));
		}
	}
	if (load_count == 0U) {
		if (mode == ECV_LOAD_PROBE) {
			PASS("ECV-LOAD PROBE %u us: no notifications arrived (link "
			     "%s) -- no data flows at this interval\n", interval_us,
			     central_connected ? "up" : "DROPPED");
			return;
		}
		FAIL("No load notifications arrived at %u us\n", interval_us);
		return;
	}
	k_sleep(K_MSEC(500));

#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
	if (default_conn && !bt_hci_get_conn_handle(default_conn, &load_handle) &&
	    load_handle < CONFIG_BT_MAX_CONN) {
		ce_before = ll_test_conn_event_count[load_handle];
	} else {
		load_handle = 0xFFFFU;
	}
#else
	ARG_UNUSED(ce_before);
	ARG_UNUSED(load_handle);
#endif

	/* Soak. Sliced so a dropped link ends the window early with its data. */
	before = load_count;
	gaps_before = load_gaps;
	t0 = k_uptime_get();
	while ((k_uptime_get() - t0) < ECV_LOAD_SOAK_MS && central_connected) {
		k_sleep(K_MSEC(100));
	}
	elapsed_ms = k_uptime_get() - t0;
	survived = central_connected;
	delivered = load_count - before;
	gaps = load_gaps - gaps_before;
#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
	if (load_handle != 0xFFFFU) {
		ce_events = ll_test_conn_event_count[load_handle] - ce_before;
	}
#endif

	/* Nominal: one notification per connection interval over the window. */
	expected = (uint32_t)((elapsed_ms * 1000) / interval_us);
	pct = expected ? ((delivered * 100U) / expected) : 0U;

	/* Offered: what the generator could actually send. k_timer rounds the
	 * period UP to the system tick, and the tick is coarse on the bsim boards
	 * (100 us under GRTC on nRF54L, ~30.5 us under the 32 KHz RTC on nRF52),
	 * so e.g. a 625 us load timer really fires every 700 us on nRF54L. Gating
	 * delivery against the interval-nominal then fails on generator shortfall
	 * the controller never saw (the 2026-07-05 "89% @ 625 us drain deficit"
	 * was exactly this accounting artifact -- the controller had delivered
	 * 100% of the offered load). Rate verdicts therefore use delivered vs
	 * OFFERED; the interval-nominal stays in the RESULT line as context.
	 */
	period_ticks = ((uint64_t)interval_us * CONFIG_SYS_CLOCK_TICKS_PER_SEC +
			(USEC_PER_SEC - 1)) / USEC_PER_SEC;
	offered = (uint32_t)(((uint64_t)elapsed_ms * CONFIG_SYS_CLOCK_TICKS_PER_SEC) /
			     (MSEC_PER_SEC * period_ticks));
	pct_off = offered ? ((delivered * 100U) / offered) : 0U;

	/* The RESULT line is the scenario's data product -- one per cell, same
	 * shape for gate/control/probe, so the matrix can be read straight out
	 * of the CI logs. CEs is the on-air connection-event count over the same
	 * window (0 if the counter hook is off): it separates "CEs run but carry
	 * no data" from "CEs themselves are being skipped".
	 */
	printk("ECV-LOAD RESULT [%s]: interval %u us, tIFS %u us (FSU %s), PHY 2M, "
	       "load 1x17B-LL-PDU/CE: delivered %u of offered %u (%u%%; %u%% of "
	       "interval-nominal %u), seq gaps %u, CEs %u, link %s after %lld ms\n",
	       mode_str, interval_us, applied_tifs_us, use_fsu ? "on" : "off",
	       delivered, offered, pct_off, pct, expected, gaps, ce_events,
	       survived ? "up" : "DROPPED", elapsed_ms);

	switch (mode) {
	case ECV_LOAD_GATE:
		if (!survived) {
			FAIL("Gate %u us + FSU: link dropped %lld ms into the soak\n",
			     interval_us, elapsed_ms);
			return;
		}
		if (gaps != 0U) {
			FAIL("Gate %u us + FSU: %u sequence gaps (LL ACL is reliable "
			     "and ordered; the peripheral only advances the sequence "
			     "on a successful queue, so any gap is a real loss)\n",
			     interval_us, gaps);
			return;
		}
		if (pct_off < ECV_LOAD_RATE_MIN_PCT) {
			FAIL("Gate %u us + FSU: delivered only %u%% of the offered "
			     "load (floor %u%%)\n", interval_us, pct_off,
			     ECV_LOAD_RATE_MIN_PCT);
			return;
		}
		(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		PASS("ECV load gate held: %u us + FSU carried the ZMK pointing load "
		     "(%u%% of offered, 0 gaps)\n", interval_us, pct_off);
		return;
	case ECV_LOAD_CONTROL:
		/* The control asserts only survival; its delivered rate is the
		 * airtime-vs-scheduler DATA (compare against the FSU gate cell at
		 * the same interval in the RESULT lines).
		 */
		if (!survived) {
			FAIL("Control %u us (%u us tIFS): link dropped %lld ms into "
			     "the soak\n", interval_us, applied_tifs_us, elapsed_ms);
			return;
		}
		(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		PASS("ECV load control survived at %u us / %u us tIFS -- CONTROL "
		     "DATA: %u%% of offered, %u gaps (see RESULT line)\n",
		     interval_us, applied_tifs_us, pct_off, gaps);
		return;
	case ECV_LOAD_PROBE:
	default:
		/* Informational: the RESULT line is the product; record and pass. */
		if (survived && default_conn) {
			(void)bt_conn_disconnect(default_conn,
						 BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
		PASS("ECV-LOAD PROBE %u us recorded: %u%% of offered, %u gaps, "
		     "link %s\n", interval_us, pct_off, gaps,
		     survived ? "held" : "DROPPED");
		return;
	}
}

static void test_central_main_ecv_fsu_load_750(void)
{
	ecv_load_run(ECV_LOAD_INTERVAL_750_125US, true, ECV_LOAD_GATE);
}

static void test_central_main_ecv_fsu_load_625(void)
{
	ecv_load_run(ECV_LOAD_INTERVAL_625_125US, true, ECV_LOAD_GATE);
}

static void test_central_main_ecv_load_750_nofsu(void)
{
	ecv_load_run(ECV_LOAD_INTERVAL_750_125US, false, ECV_LOAD_CONTROL);
}

/* 625 us + FSU with a CONTROL verdict (assert survival, record the rate).
 * Historical: this existed because the 2026-07-05 interval-nominal accounting
 * read 89% on nRF54L and demoted the cell -- later shown to be the k_timer
 * tick-quantization artifact (the controller delivered 100% of the OFFERED
 * load; see the offered-based accounting at the RESULT computation). The 54L
 * script gates 625 us again; kept for ad-hoc runs that want a no-assert cell.
 */
static void test_central_main_ecv_fsu_load_625_rec(void)
{
	ecv_load_run(ECV_LOAD_INTERVAL_625_125US, true, ECV_LOAD_CONTROL);
}

static void test_central_main_ecv_load_625_nofsu(void)
{
	ecv_load_run(ECV_LOAD_INTERVAL_625_125US, false, ECV_LOAD_CONTROL);
}

static void test_central_main_ecv_fsu_probe_500(void)
{
	ecv_load_run(ECV_LOAD_INTERVAL_500_125US, true, ECV_LOAD_PROBE);
}

static void test_central_main_ecv_fsu_probe_375(void)
{
	ecv_load_run(ECV_LOAD_INTERVAL_375_125US, true, ECV_LOAD_PROBE);
}
#endif /* CONFIG_BT_SHORTER_CONNECTION_INTERVALS && CONFIG_BT_FRAME_SPACE_UPDATE */

static const struct bst_test_instance test_central[] = {
	{
		.test_id = "central",
		.test_descr = "Central: verify the peripheral skips events.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main,
	},
	{
		.test_id = "central_transitions",
		.test_descr = "Central: M->N transition then conn-update reset.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_transitions,
	},
	{
		.test_id = "central_continuation",
		.test_descr = "Central: continuation events keep the link awake "
			      "after data, idle reads still show the skip.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_continuation,
	},
	{
		.test_id = "central_latency",
		.test_descr = "Central: peripheral latency stacks on the factor; "
			      "they still rendezvous and the latency is higher.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_latency,
	},
#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
	{
		.test_id = "central_lat_count",
		.test_descr = "Central: under peripheral max_latency stacking the "
			      "central stays on its subrate cadence and does not break "
			      "latency to full rate on the peripheral's skipped events.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_lat_count,
	},
	{
		.test_id = "central_cadence",
		.test_descr = "Central: exact on-air skip cadence at the negotiated "
			      "factor (catches over/under-skip and phase errors; run "
			      "at several factors incl non-power-of-2).",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_cadence,
	},
	{
		.test_id = "central_multi",
		.test_descr = "Central: two links (one subrated, one full-rate); the "
			      "subrated link holds its cadence while the other is busy.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_multi,
	},
#endif
	{
		.test_id = "central_collision",
		.test_descr = "Central: re-negotiates subrating simultaneously with "
			      "the peripheral (LLCP collision); link must survive.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_collision,
	},
	{
		.test_id = "central_phy",
		.test_descr = "Central: PHY update while subrated; subrating survives "
			      "the instant-based procedure.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_phy,
	},
	{
		.test_id = "central_disable",
		.test_descr = "Central: explicitly disable subrating (N->1); skipping "
			      "stops.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_disable,
	},
	{
		.test_id = "central_notify",
		.test_descr = "Central: subscribe to HRS and verify notifications flow "
			      "while subrated.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_notify,
	},
	{
		.test_id = "central_supervision",
		.test_descr = "Central: large factor near the supervision-timeout bound; "
			      "idle link must survive.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_supervision,
	},
	{
		.test_id = "central_cwrite",
		.test_descr = "Central: bursts write-without-response to the peripheral "
			      "under continuation.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_cwrite,
	},
#if defined(CONFIG_BT_LE_EXTENDED_FEAT_SET)
	{
		.test_id = "central_efs",
		.test_descr = "Central: LL Extended Feature Set - read all remote "
			      "features (page 1); the page-1 exchange completes, the "
			      "link survives, page 0 is populated and page 1 is zero.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_efs,
	},
#endif
#if defined(CONFIG_BT_FRAME_SPACE_UPDATE)
	{
		.test_id = "central_fsu",
		.test_descr = "Central: Frame Space Update (Core 6.2) - exchange page 1 "
			      "then negotiate a frame space; the responder clamps to its "
			      "80 us floor, the slot reservation shrinks, a below-floor "
			      "request is rejected, and the link survives.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_fsu,
	},
#endif
#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
	{
		.test_id = "central_sci",
		.test_descr = "Central: RCV Shorter Connection Intervals - drive the link "
			      "to a 1.25 ms interval via Connection Rate Update; the 0x37 "
			      "event reports interval_us 1250 and the link holds.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_sci,
	},
	{
		.test_id = "central_sci_subrate",
		.test_descr = "Central: RCV SCI fused with subrating (factor>1) on a 1.25 ms "
			      "link; validates connSubrateBaseEvent agreement (link holds, "
			      "skip cadence sane) -- the ZMK idle power-saving scenario.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_sci_subrate,
	},
#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
	{
		.test_id = "central_sci_coex",
		.test_descr = "Central: a 1.25 ms SCI link coexisting with a 30 ms link "
			      "(ZMK split scenario / reduced-ce reservation gate) -- the SCI "
			      "link runs fast and the 30 ms link is not starved.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_sci_coex,
	},
	{
		.test_id = "central_ecv_coex",
		.test_descr = "Central: a 625 us ECV link (125 us grid, 150 us tIFS, "
			      "reduced CE) coexisting with a 30 ms link -- the under-split-"
			      "load proof that the reduced-CE reservation holds a sub-1.25 ms "
			      "interval without starving the co-resident link.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_ecv_coex,
	},
#endif
	{
		.test_id = "central_sci_collision",
		.test_descr = "Central: a Connection Rate Update collides with a peer "
			      "Connection Update (two instants at the same uptime); the "
			      "arbiter serialises them, the link survives and settles.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_sci_collision,
	},
	{
		.test_id = "central_sci_latency",
		.test_descr = "Central: measures one-way notification latency (the HID "
			      "input direction) on a 1.25 ms SCI link via a timestamped "
			      "notify characteristic; reports min/avg/max.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_sci_latency,
	},
#endif
#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS) && defined(CONFIG_BT_FRAME_SPACE_UPDATE)
	{
		.test_id = "central_ecv_fsu_load_750",
		.test_descr = "Central: GATE -- 750 us ECV + FSU (80 us tIFS) under the "
			      "ZMK-split pointing load (1x 8-byte notification per CE, "
			      "2M PHY); asserts >= 90% delivered, zero seq gaps, link "
			      "survives the soak.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_ecv_fsu_load_750,
	},
	{
		.test_id = "central_ecv_fsu_load_625",
		.test_descr = "Central: GATE -- 625 us ECV + FSU (80 us tIFS) under the "
			      "ZMK-split pointing load; asserts >= 90% delivered, zero "
			      "seq gaps, link survives the soak.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_ecv_fsu_load_625,
	},
	{
		.test_id = "central_ecv_load_750_nofsu",
		.test_descr = "Central: CONTROL -- 750 us ECV at the standard 150 us "
			      "tIFS under the same load; asserts link survival only, "
			      "the delivered rate is the airtime-vs-scheduler data.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_ecv_load_750_nofsu,
	},
	{
		.test_id = "central_ecv_fsu_load_625_rec",
		.test_descr = "Central: RECORDED -- 625 us ECV + FSU (80 us tIFS) under "
			      "the ZMK-split pointing load; asserts link survival only "
			      "(no rate gate). Historical no-assert variant; the 54L "
			      "script gates 625 us via central_ecv_fsu_load_625 again.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_ecv_fsu_load_625_rec,
	},
	{
		.test_id = "central_ecv_load_625_nofsu",
		.test_descr = "Central: CONTROL -- 625 us ECV at the standard 150 us "
			      "tIFS under the same load; asserts link survival only, "
			      "the delivered rate is the airtime-vs-scheduler data.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_ecv_load_625_nofsu,
	},
	{
		.test_id = "central_ecv_fsu_probe_500",
		.test_descr = "Central: PROBE (informational) -- 500 us ECV + FSU under "
			      "the load; records rate/gaps/survival, never fails past "
			      "setup.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_ecv_fsu_probe_500,
	},
	{
		.test_id = "central_ecv_fsu_probe_375",
		.test_descr = "Central: PROBE (informational) -- 375 us ECV + FSU under "
			      "the load; records rate/gaps/survival, never fails past "
			      "setup.",
		.test_pre_init_f = test_central_init,
		.test_tick_f = test_central_tick,
		.test_main_f = test_central_main_ecv_fsu_probe_375,
	},
#endif
	BSTEST_END_MARKER,
};

struct bst_test_list *test_central_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_central);
}
