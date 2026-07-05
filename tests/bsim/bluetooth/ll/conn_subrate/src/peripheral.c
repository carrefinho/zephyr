/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Subrating peripheral. Advertise, accept a connection, then initiate a
 * Connection Subrate Request (Core Spec 5.1.20). The Central observes the
 * resulting event skipping and drives the assertions; the peripheral mostly
 * negotiates the requested parameters and stays alive.
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
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/hrs.h>
#include <zephyr/sys/byteorder.h>

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
/* In the collision scenario both sides re-negotiate simultaneously; one request
 * may be rejected (a valid LLCP collision outcome) and the survival check, not
 * the subrate_changed callback, decides pass/fail.
 */
static volatile bool collision_mode;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Write-without-response characteristic for the central->peripheral
 * continuation test; each write the Central sends increments the counter.
 */
static volatile int cwrite_count;

static ssize_t cwrite_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(buf);
	ARG_UNUSED(offset);
	ARG_UNUSED(flags);
	cwrite_count++;
	return len;
}

BT_GATT_SERVICE_DEFINE(cwrite_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(CWRITE_SVC_UUID)),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(CWRITE_CHR_UUID),
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE,
			       NULL, cwrite_cb, NULL));

/* Notify characteristic carrying the peripheral's send timestamp (uint32 us)
 * for the one-way latency test. attrs[2] is the value attribute.
 */
static void lat_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	ARG_UNUSED(value);
}

BT_GATT_SERVICE_DEFINE(lat_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(LAT_SVC_UUID)),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(LAT_CHR_UUID),
			       BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(lat_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
/* ZMK-split pointing-load characteristic: the peripheral streams one 8-byte
 * input-event payload per connection interval on it, modelling a streaming
 * trackpad on the ZMK split link (see ECV_LOAD_SVC_UUID in conn_subrate.h for
 * the payload/wire-size accounting; a real paired link adds a 4-byte MIC).
 * attrs[2] is the value attribute (cf. lat_svc). Streaming is gated on the
 * central's CCC subscribe (like a real ZMK central) so bt_gatt_notify never
 * runs against a disabled CCC and the failure counter stays meaningful.
 */
static volatile bool load_ccc_enabled;

static void load_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	load_ccc_enabled = (value == BT_GATT_CCC_NOTIFY);
}

BT_GATT_SERVICE_DEFINE(load_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(ECV_LOAD_SVC_UUID)),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ECV_LOAD_CHR_UUID),
			       BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(load_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));
#endif /* CONFIG_BT_SHORTER_CONNECTION_INTERVALS */

static void subrate_changed(struct bt_conn *conn,
			    const struct bt_conn_le_subrate_changed *params)
{
	printk("Peripheral subrate changed: status %u factor %u cont %u "
	       "latency %u timeout %u\n", params->status, params->factor,
	       params->continuation_number, params->peripheral_latency,
	       params->supervision_timeout);

	if (params->status != BT_HCI_ERR_SUCCESS) {
		if (collision_mode) {
			/* A rejected request is an acceptable collision outcome. */
			return;
		}
		FAIL("Peripheral subrate change failed (status 0x%02x)\n",
		     params->status);
		return;
	}

	/* Negotiation succeeded; the Central validates the resulting skipping. */
	if (!collision_mode && params->factor >= 2U) {
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

static int peripheral_setup(void)
{
	int err;

	bt_conn_cb_register(&conn_callbacks);

	err = bt_enable(NULL);
	if (err) {
		FAIL("Bluetooth init failed (err %d)\n", err);
		return err;
	}

	printk("Peripheral Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		FAIL("Advertising failed to start (err %d)\n", err);
		return err;
	}

	printk("Peripheral advertising started\n");
	return 0;
}

static int periph_request_subrate(uint16_t subrate_min, uint16_t subrate_max,
				  uint16_t max_latency, uint16_t cont_number)
{
	struct bt_conn_le_subrate_param param = {
		.subrate_min = subrate_min,
		.subrate_max = subrate_max,
		.max_latency = max_latency,
		.continuation_number = cont_number,
		.supervision_timeout = SUBRATE_REQ_TIMEOUT,
	};
	int err = bt_conn_le_subrate_request(default_conn, &param);

	if (err) {
		FAIL("Peripheral subrate request failed (err %d)\n", err);
		return err;
	}
	printk("Peripheral requested subrating (min %u max %u lat %u cn %u)\n",
	       subrate_min, subrate_max, max_latency, cont_number);
	return 0;
}

/* Advertise, once connected and settled request subrating with the given
 * parameters, then stay alive responding to whatever the Central drives next.
 */
static void peripheral_run(uint16_t subrate_min, uint16_t subrate_max,
			   uint16_t max_latency, uint16_t cont_number)
{
	bool requested = false;

	if (peripheral_setup()) {
		return;
	}

	while (true) {
		k_sleep(K_MSEC(SETTLE_DELAY_MS));

		if (connected_flag && !requested && default_conn) {
			requested = true;
			if (periph_request_subrate(subrate_min, subrate_max,
						   max_latency, cont_number)) {
				return;
			}
		}
	}
}

static void test_peripheral_main(void)
{
	peripheral_run(SUBRATE_REQ_MIN, SUBRATE_REQ_MAX, 0U, 0U);
}

static void test_peripheral_main_transitions(void)
{
	/* Negotiate exactly factor M; the Central re-negotiates to N afterwards. */
	peripheral_run(SUBRATE_MTON_M, SUBRATE_MTON_M, 0U, 0U);
}

static void test_peripheral_main_continuation(void)
{
	peripheral_run(SUBRATE_CONT_FACTOR, SUBRATE_CONT_FACTOR, 0U, SUBRATE_CONT_CN);
}

static void test_peripheral_main_oddfactor(void)
{
	peripheral_run(SUBRATE_ODD_FACTOR, SUBRATE_ODD_FACTOR, 0U, 0U);
}

static void test_peripheral_main_supervision(void)
{
	peripheral_run(SUBRATE_SUPERVISION_FACTOR, SUBRATE_SUPERVISION_FACTOR, 0U, 0U);
}

static void test_peripheral_main_plain(void)
{
	if (peripheral_setup()) {
		return;
	}
	/* Full-rate peer for the multi-connection test: connect and idle, never
	 * request subrating. Mark pass once connected (the central drives and
	 * validates the cadence); unlike the subrating peers there is no
	 * subrate_changed to pass on. Then stay alive for the rest of the sim.
	 */
	while (!connected_flag) {
		k_sleep(K_MSEC(50));
	}
	PASS("Peripheral (plain) connected; full-rate peer\n");
	while (true) {
		k_sleep(K_MSEC(SETTLE_DELAY_MS));
	}
}

static void test_peripheral_main_cwrite(void)
{
	int before;

	if (peripheral_setup()) {
		return;
	}

	while (!connected_flag) {
		k_sleep(K_MSEC(50));
		if (bst_result == Failed) {
			return;
		}
	}
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	/* Subrate with continuation enabled, then count the Central's bursts. */
	if (periph_request_subrate(SUBRATE_CONT_FACTOR, SUBRATE_CONT_FACTOR, 0U,
				   SUBRATE_CONT_CN)) {
		return;
	}

	/* Let the Central discover the characteristic and start writing. */
	k_sleep(K_SECONDS(2));
	before = cwrite_count;
	k_sleep(K_SECONDS(3));
	printk("Peripheral received %d central writes under subrating\n",
	       cwrite_count - before);

	/* With the Central's own Tx opening the continuation window the burst
	 * arrives near the event rate (~100 in 3 s); throttled to the subrate
	 * cadence it would be ~factor times fewer (~12). Require well above that.
	 */
	if ((cwrite_count - before) < 30) {
		FAIL("Too few central writes under continuation: %d\n",
		     cwrite_count - before);
		return;
	}
	PASS("Peripheral C->P continuation validated (%d writes in 3 s)\n",
	     cwrite_count - before);
}

static void test_peripheral_main_latency(void)
{
	peripheral_run(SUBRATE_LAT_FACTOR, SUBRATE_LAT_FACTOR, SUBRATE_LAT_LATENCY, 0U);
}

static void test_peripheral_main_collision(void)
{
	if (peripheral_setup()) {
		return;
	}

	while (!connected_flag) {
		k_sleep(K_MSEC(50));
		if (bst_result == Failed) {
			return;
		}
	}
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	/* Establish subrating first. */
	if (periph_request_subrate(SUBRATE_COLL_INITIAL, SUBRATE_COLL_INITIAL, 0U, 0U)) {
		return;
	}

	/* Both sides re-negotiate at the same uptime to force an LLCP collision. */
	while (k_uptime_get() < COLLISION_TIME_MS) {
		k_sleep(K_MSEC(20));
	}
	collision_mode = true;
	(void)periph_request_subrate(SUBRATE_COLL_PERIPH, SUBRATE_COLL_PERIPH, 0U, 0U);

	/* Survive the collision: still connected a while later. Check before the
	 * central's end-of-test disconnect (it sleeps longer post-collision).
	 */
	k_sleep(K_MSEC(2000));
	if (!connected_flag) {
		FAIL("Peripheral lost connection after subrate collision\n");
		return;
	}
	PASS("Peripheral survived subrate collision\n");
}

/* Peer side of central_sci_collision: drive a Connection Parameters Request at
 * the same uptime the Central drives its Connection Rate Update, so two instant
 * procedures collide. The link must survive (no instant desync).
 */
static void test_peripheral_main_sci_collision(void)
{
	struct bt_le_conn_param *upd = BT_LE_CONN_PARAM(CONN_UPDATE_INTERVAL_UNITS,
							CONN_UPDATE_INTERVAL_UNITS,
							0, CONN_TIMEOUT_UNITS);

	if (peripheral_setup()) {
		return;
	}

	while (!connected_flag) {
		k_sleep(K_MSEC(50));
		if (bst_result == Failed) {
			return;
		}
	}
	k_sleep(K_MSEC(SETTLE_DELAY_MS));

	while (k_uptime_get() < COLLISION_TIME_MS) {
		k_sleep(K_MSEC(20));
	}
	collision_mode = true;
	(void)bt_conn_le_param_update(default_conn, upd);

	k_sleep(K_MSEC(3000));
	if (!connected_flag) {
		FAIL("Peripheral lost the link after the SCI/conn-update collision\n");
		return;
	}
	PASS("Peripheral survived SCI/conn-update collision\n");
}

/* Peer side of central_sci_latency: stream the send timestamp on the latency
 * characteristic. The Central drives the SCI rate, subscribes, and measures the
 * one-way delay. Notifications before the Central subscribes simply no-op.
 */
static void test_peripheral_main_sci_latency(void)
{
	if (peripheral_setup()) {
		return;
	}

	while (!connected_flag) {
		k_sleep(K_MSEC(50));
		if (bst_result == Failed) {
			return;
		}
	}

	for (int i = 0; i < 400 && connected_flag; i++) {
		uint32_t sent_us = (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
		uint8_t buf[sizeof(sent_us)];

		sys_put_le32(sent_us, buf);
		(void)bt_gatt_notify(default_conn, &lat_svc.attrs[2], buf, sizeof(buf));
		k_sleep(K_MSEC(LAT_NOTIFY_GAP_MS));
	}

	PASS("Peripheral streamed latency notifications\n");
}

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
/* Peer side of the central_ecv_* load cells: once the Central has driven the
 * link to its target ECV interval (learned from the conn_rate_changed event's
 * interval_us), stream exactly one 8-byte ZMK input-event notification per
 * connection interval, paced by a k_timer at the interval period -- a
 * streaming trackpad on the split link. The payload mirrors ZMK's
 * zmk_split_input_event_payload {u8 type, u16 code, u32 value, u8 sync}
 * (type = 2/EV_REL, code = 0/REL_X, sync = 1) with the sequence number in
 * `value` so the Central can count delivery gaps. The sequence advances ONLY
 * on a successful queue: a send failure (e.g. -ENOMEM under backpressure)
 * shows up at the Central as a rate shortfall, not as a fake gap, so gaps mean
 * genuine loss. The Central holds all the assertions; this side PASSes on the
 * first successful send and keeps streaming (so one testid serves the gate,
 * control, and probe cells alike -- on a probe the link may die under it,
 * which is the Central's data, not this side's failure).
 */
static volatile uint32_t load_interval_us;

static void load_conn_rate_changed(struct bt_conn *conn, uint8_t status,
				   const struct bt_conn_le_conn_rate_changed *params)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS && params != NULL) {
		load_interval_us = params->interval_us;
	}
	printk("Peripheral conn rate changed: status 0x%02x interval %u us\n",
	       status, params != NULL ? params->interval_us : 0U);
}

static struct bt_conn_cb load_rate_callbacks = {
	.conn_rate_changed = load_conn_rate_changed,
};

/* Semaphore depth 8: a small backlog survives a busy host thread; beyond that
 * ticks collapse and the Central sees the rate shortfall (the intended signal).
 */
static K_SEM_DEFINE(load_tick_sem, 0, 8);

static void load_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&load_tick_sem);
}

static K_TIMER_DEFINE(load_timer, load_timer_fn, NULL);

static void test_peripheral_main_zmk_load(void)
{
	uint32_t seq = 0U, sent = 0U, failed = 0U, us;
	bool passed = false;

	bt_conn_cb_register(&load_rate_callbacks);

	if (peripheral_setup()) {
		return;
	}

	while (!connected_flag) {
		k_sleep(K_MSEC(50));
		if (bst_result == Failed) {
			return;
		}
	}

	/* Wait for the Central to drive the link to its sub-7.5 ms target AND
	 * enable notifications (either order): input only flows on a subscribed
	 * characteristic, as on a real ZMK split link. A link lost while waiting
	 * is recorded, not failed: the central owns every cell's verdict (on a
	 * gate cell it FAILs the run; on a probe it is the recorded floor datum).
	 */
	while (load_interval_us == 0U || load_interval_us >= 7500U ||
	       !load_ccc_enabled) {
		k_sleep(K_MSEC(50));
		if (bst_result == Failed) {
			return;
		}
		if (!connected_flag) {
			PASS("Peripheral ZMK-load: link lost before streaming began "
			     "(central records the outcome)\n");
			while (true) {
				k_sleep(K_MSEC(SETTLE_DELAY_MS));
			}
		}
	}
	us = load_interval_us;
	printk("Peripheral ZMK-load: streaming 1 x 8-byte input event per %u us CE\n",
	       us);

	/* K_USEC ceil-rounds to system ticks (30.5 us at 32768 Hz), so the
	 * generator runs at most ~1 tick slow per period -- accounted for in the
	 * Central's 90% rate floor (see conn_subrate.h).
	 */
	k_timer_start(&load_timer, K_USEC(us), K_USEC(us));

	while (connected_flag && bst_result != Failed) {
		uint8_t payload[8];
		int err;

		if (k_sem_take(&load_tick_sem, K_MSEC(1000)) != 0) {
			continue; /* re-check connected_flag */
		}

		/* zmk_split_input_event_payload, built LE-explicit. */
		payload[0] = 0x02U;              /* type: EV_REL */
		sys_put_le16(0x0000U, &payload[1]); /* code: REL_X */
		sys_put_le32(seq, &payload[3]);  /* value: sequence number */
		payload[7] = 0x01U;              /* sync */

		err = bt_gatt_notify(default_conn, &load_svc.attrs[2], payload,
				     sizeof(payload));
		if (err == 0) {
			seq++;
			sent++;
			if (!passed) {
				passed = true;
				PASS("Peripheral ZMK-load streaming at %u us\n", us);
			}
		} else {
			failed++;
			if (failed <= 5U) {
				printk("Peripheral ZMK-load notify failed (err %d, "
				       "%u so far)\n", err, failed);
			}
		}
	}

	k_timer_stop(&load_timer);
	printk("Peripheral ZMK-load final: sent %u, failed %u (interval %u us)\n",
	       sent, failed, us);
	if (!passed) {
		/* Not one notification queued before the link went down: the
		 * central records/asserts this per its cell mode.
		 */
		PASS("Peripheral ZMK-load: link died before any send succeeded "
		     "(central records the outcome)\n");
	}

	/* Stay alive for the rest of the simulation (cf. peripheral_plain). */
	while (true) {
		k_sleep(K_MSEC(SETTLE_DELAY_MS));
	}
}
#endif /* CONFIG_BT_SHORTER_CONNECTION_INTERVALS */

static void test_peripheral_main_notify(void)
{
	uint16_t heartrate = 90U;
	bool requested = false;

	if (peripheral_setup()) {
		return;
	}

	/* Request subrating once connected+settled, then stream HRS notifications.
	 * Under subrating the controller can only send them on subrated/continuation
	 * events; the Central verifies they still flow.
	 */
	while (true) {
		k_sleep(K_MSEC(SUBRATE_NOTIFY_PERIOD_MS));

		if (connected_flag && !requested && default_conn) {
			requested = true;
			if (periph_request_subrate(SUBRATE_REQ_MIN, SUBRATE_REQ_MAX, 0U, 0U)) {
				return;
			}
		}

		if (connected_flag) {
			(void)bt_hrs_notify(heartrate);
			if (++heartrate > 160U) {
				heartrate = 90U;
			}
		}
	}
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
		.test_descr = "Peripheral: requests subrating (factor > 1).",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main,
	},
	{
		.test_id = "peripheral_transitions",
		.test_descr = "Peripheral: negotiates factor M; Central re-negotiates "
			      "to N and changes the interval.",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main_transitions,
	},
	{
		.test_id = "peripheral_continuation",
		.test_descr = "Peripheral: subrating with continuation_number > 0.",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main_continuation,
	},
	{
		.test_id = "peripheral_latency",
		.test_descr = "Peripheral: subrating with peripheral latency > 0.",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main_latency,
	},
	{
		.test_id = "peripheral_collision",
		.test_descr = "Peripheral: re-negotiates subrating simultaneously with "
			      "the Central (LLCP collision); link must survive.",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main_collision,
	},
	{
		.test_id = "peripheral_sci_collision",
		.test_descr = "Peripheral: drives a Connection Parameters Request "
			      "simultaneously with the Central's Connection Rate Update "
			      "(two instants collide); link must survive.",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main_sci_collision,
	},
	{
		.test_id = "peripheral_sci_latency",
		.test_descr = "Peripheral: streams send-timestamp notifications for the "
			      "Central's one-way latency measurement.",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main_sci_latency,
	},
	{
		.test_id = "peripheral_notify",
		.test_descr = "Peripheral: streams HRS notifications while subrated.",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main_notify,
	},
	{
		.test_id = "peripheral_oddfactor",
		.test_descr = "Peripheral: negotiates a non-power-of-2 factor (5).",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main_oddfactor,
	},
	{
		.test_id = "peripheral_supervision",
		.test_descr = "Peripheral: large factor near the supervision-timeout bound.",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main_supervision,
	},
	{
		.test_id = "peripheral_cwrite",
		.test_descr = "Peripheral: counts the Central's write-without-response "
			      "burst under continuation.",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main_cwrite,
	},
	{
		.test_id = "peripheral_plain",
		.test_descr = "Peripheral: connects and idles, never requests subrating "
			      "(full-rate peer for the multi-connection test).",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main_plain,
	},
#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
	{
		.test_id = "peripheral_zmk_load",
		.test_descr = "Peripheral: streams one 8-byte ZMK input-event "
			      "notification per connection interval once the Central "
			      "drives the link to its ECV target (the ZMK-split "
			      "pointing load for the central_ecv_* load cells).",
		.test_pre_init_f = test_peripheral_init,
		.test_tick_f = test_peripheral_tick,
		.test_main_f = test_peripheral_main_zmk_load,
	},
#endif
	BSTEST_END_MARKER,
};

struct bst_test_list *test_peripheral_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_peripheral);
}
