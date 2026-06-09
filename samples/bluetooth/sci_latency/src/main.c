/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shorter Connection Intervals (RCV) hardware-in-the-loop latency tester.
 *
 * Two nRF54L15 DKs: one CENTRAL, one PERIPHERAL (selected by central.conf /
 * peripheral.conf). The central connects, drives the link to a 1.25 ms
 * connection interval via the LE Connection Rate Update procedure, then sweeps
 * the subrate factor {1, 2, 4, 8, 16} and at each factor measures the GATT
 * round-trip latency (a Read of the peer's GAP Device Name), logging
 * min/avg/max over the VCOM UART. A J-Link/GDB reset of the central re-runs the
 * whole sweep; the peripheral just advertises and accepts.
 *
 * Latency note: this is a ROUND TRIP (ATT Read Request out + Read Response back)
 * measured at the host API, so it includes host-stack overhead. With subrating,
 * the peripheral is asleep between subrated events, so an isolated read waits up
 * to factor x interval -- which is exactly the latency/power trade-off being
 * characterised. A one-way (key-event) figure would be ~half the air component
 * and needs a scope + GPIO trigger, out of scope for UART-only logging.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sci_latency, LOG_LEVEL_INF);

#define DEVICE_NAME      CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN  (sizeof(DEVICE_NAME) - 1)

/* 1250 us in 125 us units (the RCV floor). */
#define SCI_INTERVAL_125US      10U
/* Supervision timeout: 2 s in 10 ms units (both the conn-rate and subrate
 * params take the supervision timeout in 10 ms units).
 */
#define SCI_TIMEOUT_10MS        200U
#define SUBRATE_TIMEOUT_10MS    200U

#if defined(CONFIG_SCI_LATENCY_CENTRAL)

static struct bt_conn *default_conn;

static const uint16_t sweep_factors[] = { 1U, 2U, 4U, 8U };
#define READS_PER_FACTOR   10
/* "idle" gap: > factor x interval so the peer re-sleeps between isolated reads. */
#define IDLE_GAP_MS        CONFIG_SCI_LATENCY_IDLE_GAP_MS
/* "burst" gap: < continuation window, so reads land inside the awake window. */
#define BURST_GAP_MS       CONFIG_SCI_LATENCY_BURST_GAP_MS

static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_feat, 0, 1);
static K_SEM_DEFINE(sem_conn_rate, 0, 1);
static K_SEM_DEFINE(sem_subrate, 0, 1);
static K_SEM_DEFINE(sem_read, 0, 1);
#if defined(CONFIG_SCI_LATENCY_PLAIN_SUBRATE)
static K_SEM_DEFINE(sem_param, 0, 1);
/* 7.5 ms in 1.25 ms units (BT_HCI_LE_INTERVAL_MIN -- the shortest standard,
 * non-SCI connection interval; the control for "is it SCI or subrating?").
 */
#define PLAIN_INTERVAL_125MS    ((uint16_t)CONFIG_SCI_LATENCY_PLAIN_INTERVAL_UNITS)
#define BASE_INTERVAL_US        (CONFIG_SCI_LATENCY_PLAIN_INTERVAL_UNITS * 1250U)
#else
#define BASE_INTERVAL_US        1250U
#endif

static volatile uint8_t conn_rate_status = 0xFFU;
static volatile uint8_t subrate_status = 0xFFU;
static volatile uint16_t negotiated_factor;
static volatile uint8_t read_att_err;

static bool ad_name_match(struct bt_data *data, void *user_data)
{
	bool *match = user_data;

	if (data->type == BT_DATA_NAME_COMPLETE || data->type == BT_DATA_NAME_SHORTENED) {
		if (data->data_len == DEVICE_NAME_LEN &&
		    memcmp(data->data, DEVICE_NAME, DEVICE_NAME_LEN) == 0) {
			*match = true;
			return false;
		}
	}

	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	bool match = false;
	int err;

	if (default_conn) {
		return;
	}
	if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	bt_data_parse(ad, ad_name_match, &match);
	if (!match) {
		return;
	}

	if (bt_le_scan_stop()) {
		return;
	}

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT,
				&default_conn);
	if (err) {
		LOG_ERR("create conn failed (err %d)", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("connect failed (0x%02x)", err);
		bt_conn_unref(default_conn);
		default_conn = NULL;
		return;
	}
	LOG_INF("Connected");
	k_sem_give(&sem_connected);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_WRN("Disconnected (reason 0x%02x)", reason);
	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
}

static void read_all_remote_feat_complete(struct bt_conn *conn,
					  const struct bt_conn_le_read_all_remote_feat_complete *params)
{
	k_sem_give(&sem_feat);
}

static void conn_rate_changed(struct bt_conn *conn, uint8_t status,
			      const struct bt_conn_le_conn_rate_changed *params)
{
	conn_rate_status = status;
	k_sem_give(&sem_conn_rate);
}

static void subrate_changed(struct bt_conn *conn,
			    const struct bt_conn_le_subrate_changed *params)
{
	subrate_status = params->status;
	if (params->status == BT_HCI_ERR_SUCCESS) {
		negotiated_factor = params->factor;
	}
	k_sem_give(&sem_subrate);
}

#if defined(CONFIG_SCI_LATENCY_PLAIN_SUBRATE)
static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	if (interval == PLAIN_INTERVAL_125MS) {
		k_sem_give(&sem_param);
	}
}
#endif

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.read_all_remote_feat_complete = read_all_remote_feat_complete,
	.conn_rate_changed = conn_rate_changed,
	.subrate_changed = subrate_changed,
#if defined(CONFIG_SCI_LATENCY_PLAIN_SUBRATE)
	.le_param_updated = le_param_updated,
#endif
};

static uint8_t read_cb(struct bt_conn *conn, uint8_t att_err,
		       struct bt_gatt_read_params *params, const void *data, uint16_t length)
{
	read_att_err = att_err;
	k_sem_give(&sem_read);
	return BT_GATT_ITER_STOP;
}

static struct bt_gatt_read_params read_params = {
	.func = read_cb,
	.handle_count = 0,
	.by_uuid.uuid = BT_UUID_GAP_DEVICE_NAME,
	.by_uuid.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
	.by_uuid.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
};

/* One round-trip GATT read; returns latency in microseconds, or -1 on error. */
static int32_t probe_once(void)
{
	uint32_t t0, dt;
	int err;

	read_att_err = 0xFFU;
	k_sem_reset(&sem_read);

	t0 = k_cycle_get_32();
	err = bt_gatt_read(default_conn, &read_params);
	if (err) {
		LOG_ERR("read failed (err %d)", err);
		return -1;
	}
	if (k_sem_take(&sem_read, K_SECONDS(5)) != 0) {
		LOG_ERR("read timed out");
		return -1;
	}
	dt = k_cycle_get_32() - t0;
	if (read_att_err != 0U) {
		LOG_ERR("ATT read error 0x%02x", read_att_err);
		return -1;
	}

	return (int32_t)k_cyc_to_us_floor32(dt);
}

static int measure(int gap_ms, int32_t *out_min, int32_t *out_avg, int32_t *out_max)
{
	int32_t mn = INT32_MAX, mx = 0;
	int64_t sum = 0;
	int n = 0;

	for (int i = 0; i < READS_PER_FACTOR; i++) {
		int32_t us = probe_once();

		if (us < 0) {
			return -1;
		}
		if (us < mn) {
			mn = us;
		}
		if (us > mx) {
			mx = us;
		}
		sum += us;
		n++;
		k_sleep(K_MSEC(gap_ms));
	}

	*out_min = mn;
	*out_avg = (int32_t)(sum / n);
	*out_max = mx;
	return 0;
}

/* Apply a subrate factor with the given continuation number, wait for it to take
 * effect. Returns 0 on success.
 */
static int apply_subrate(uint16_t factor, uint16_t continuation)
{
	struct bt_conn_le_subrate_param sub = {
		.subrate_min = factor,
		.subrate_max = factor,
		.max_latency = 0U,
		.continuation_number = continuation,
		.supervision_timeout = SUBRATE_TIMEOUT_10MS,
	};
	int err;

	subrate_status = 0xFFU;
	negotiated_factor = 0U;
	err = bt_conn_le_subrate_request(default_conn, &sub);
	if (err) {
		LOG_ERR("subrate request (factor %u) failed (err %d)", factor, err);
		return -1;
	}
	if (k_sem_take(&sem_subrate, K_SECONDS(5)) != 0 ||
	    subrate_status != BT_HCI_ERR_SUCCESS) {
		LOG_ERR("subrate factor %u not applied (status 0x%02x)", factor, subrate_status);
		return -1;
	}
	k_sleep(K_MSEC(300)); /* let the new subrate cadence settle */
	return 0;
}

int main(void)
{
	struct bt_conn_le_conn_rate_param rate = {
		.interval_min_125us = SCI_INTERVAL_125US,
		.interval_max_125us = SCI_INTERVAL_125US,
		.subrate_min = 1U,
		.subrate_max = 1U,
		.max_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout_10ms = SCI_TIMEOUT_10MS,
		.min_ce_len_125us = 1U,
		.max_ce_len_125us = 1U,
	};
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed (err %d)", err);
		return 0;
	}
	bt_conn_cb_register(&conn_callbacks);
	LOG_INF("SCI latency tester: CENTRAL. Scanning for \"%s\"", DEVICE_NAME);

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		LOG_ERR("scan start failed (err %d)", err);
		return 0;
	}

	k_sem_take(&sem_connected, K_FOREVER);
	k_sleep(K_MSEC(500));

	/* Learn the peer's page-1 SCI Host Support bit (the Central gates on it). */
	if (bt_conn_le_read_all_remote_features(default_conn, 1U) == 0) {
		(void)k_sem_take(&sem_feat, K_SECONDS(2));
	}
	k_sleep(K_MSEC(200));

#if defined(CONFIG_SCI_LATENCY_PLAIN_SUBRATE)
	/* CONTROL: no SCI. Standard param update to 7.5 ms, then sweep subrate.
	 * Isolates the original subrating path from the SCI/1.25 ms interaction.
	 */
	ARG_UNUSED(rate);
	err = bt_conn_le_param_update(
		default_conn,
		BT_LE_CONN_PARAM(PLAIN_INTERVAL_125MS, PLAIN_INTERVAL_125MS, 0U,
				 SCI_TIMEOUT_10MS));
	if (err) {
		LOG_ERR("param update failed (err %d)", err);
		return 0;
	}
	if (k_sem_take(&sem_param, K_SECONDS(5)) != 0) {
		LOG_ERR("param update to 7.5 ms not applied");
		return 0;
	}
	LOG_INF("Link now at %u us (PLAIN subrate control). Sweeping subrate factor.",
		BASE_INTERVAL_US);
#else
	err = bt_conn_le_conn_rate_request(default_conn, &rate);
	if (err) {
		LOG_ERR("conn rate request failed (err %d)", err);
		return 0;
	}
	if (k_sem_take(&sem_conn_rate, K_SECONDS(5)) != 0 ||
	    conn_rate_status != BT_HCI_ERR_SUCCESS) {
		LOG_ERR("conn rate update failed (status 0x%02x)", conn_rate_status);
		return 0;
	}
	LOG_INF("Link now at 1.25 ms. Sweeping subrate factor (round-trip GATT read).");
#endif
	LOG_INF("  idle  = isolated reads (peer sleeps between) -- continuation 0");
	LOG_INF("  burst = back-to-back reads (peer stays awake) -- continuation factor-1");
	LOG_INF("factor | effective | idle min/avg/max us | burst min/avg/max us");

	for (int i = 0; i < (int)ARRAY_SIZE(sweep_factors); i++) {
		uint16_t f = sweep_factors[i];
		int32_t imn, iavg, imx, bmn, bavg, bmx;

		/* Idle: continuation 0, large gap -> the peer re-sleeps between reads. */
		if (apply_subrate(f, 0U) || measure(IDLE_GAP_MS, &imn, &iavg, &imx)) {
			break;
		}
		/* Burst: continuation factor-1, tiny gap -> reads stay in the awake
		 * window, so latency should fall back to ~one interval if the high idle
		 * number is just the peer sleeping between subrated events.
		 */
		if (apply_subrate(f, (f > 1U) ? (uint16_t)(f - 1U) : 0U) ||
		    measure(BURST_GAP_MS, &bmn, &bavg, &bmx)) {
			break;
		}

		LOG_INF("%6u | %6u us | %6d/%6d/%6d | %6d/%6d/%6d",
			f, f * BASE_INTERVAL_US, imn, iavg, imx, bmn, bavg, bmx);

		if (!default_conn) {
			LOG_ERR("link dropped during sweep");
			break;
		}
	}

	LOG_INF("Sweep complete. Reset (J-Link/GDB) to re-run.");
	return 0;
}

#else /* peripheral */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void start_adv(void)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);

	if (err) {
		LOG_ERR("advertising start failed (err %d)", err);
	} else {
		LOG_INF("Advertising as \"%s\"", DEVICE_NAME);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("connect failed (0x%02x)", err);
		start_adv();
		return;
	}
	LOG_INF("Connected -- central will drive SCI + subrating");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_WRN("Disconnected (reason 0x%02x); re-advertising", reason);
	start_adv();
}

static void conn_rate_changed(struct bt_conn *conn, uint8_t status,
			      const struct bt_conn_le_conn_rate_changed *params)
{
	if (status == BT_HCI_ERR_SUCCESS && params != NULL) {
		LOG_INF("Conn rate change applied: interval %u us, factor %u",
			params->interval_us, params->subrate_factor);
	}
}

static void subrate_changed(struct bt_conn *conn,
			    const struct bt_conn_le_subrate_changed *params)
{
	if (params->status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("Subrate applied: factor %u, peripheral_latency %u",
			params->factor, params->peripheral_latency);
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.conn_rate_changed = conn_rate_changed,
	.subrate_changed = subrate_changed,
};

int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed (err %d)", err);
		return 0;
	}
	bt_conn_cb_register(&conn_callbacks);

	LOG_INF("SCI latency tester: PERIPHERAL \"%s\"", DEVICE_NAME);
	start_adv();
	return 0;
}

#endif /* CONFIG_SCI_LATENCY_CENTRAL */
