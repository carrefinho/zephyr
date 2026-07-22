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
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_SCI_LATENCY_GPIO_ONEWAY)
#include <zephyr/drivers/gpio.h>
#endif

LOG_MODULE_REGISTER(sci_latency, LOG_LEVEL_INF);

#define DEVICE_NAME      CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN  (sizeof(DEVICE_NAME) - 1)

#if defined(CONFIG_SCI_LATENCY_GPIO_ONEWAY)
/* DK-to-DK cross-wire for the one-way latency measurement: P1.11 on both DKs
 * (plus GND). Peripheral output, central GPIOTE-timestamped input.
 */
#define ONEWAY_GPIO_NODE DT_NODELABEL(gpio1)
#define ONEWAY_GPIO_PIN  11

/* Pipeline stage stamps written by the controller's HCI driver (hci_driver.c)
 * for the one-way breakdown: ACL node leaving the controller (prio_recv_thread
 * fifo put) and ACL encoded in recv_thread entering bt_recv. Referenced from
 * both roles (the peripheral also receives ACLs), defined once here.
 */
volatile uint32_t sci_dbg_fifo_cyc;
volatile uint32_t sci_dbg_acl_cyc;

/* Peripheral TX-path breakdown stamps + accumulators (written by the sample's
 * generator, ull_conn.c and lll_conn.c; see the pick-site in lll_conn.c).
 * sci_txp_acc layout: [0..3] gen->pick (min,sum,max,n), [4..7] host-enq->pick,
 * [8..11] lll-queue->pick.
 */
volatile uint32_t sci_dbg_gen;
volatile uint32_t sci_dbg_tx_enq;
volatile uint32_t sci_dbg_tx_lll;
volatile uint32_t sci_dbg_tx_flag;
volatile uint32_t sci_txp_acc[12];

/* Simple min/avg/max accumulator for the stage deltas. */
struct ow_acc {
	uint32_t min, max, n;
	uint64_t sum;
};

static void ow_acc_add(struct ow_acc *a, uint32_t us)
{
	if (a->n == 0U || us < a->min) {
		a->min = us;
	}
	if (us > a->max) {
		a->max = us;
	}
	a->sum += us;
	a->n++;
}
#endif

/* ZMK-split pointing-load characteristic (shared by both roles for the ECV_LOAD
 * mode). The peripheral streams one 8-byte input-event payload per connection
 * interval on it; the central subscribes and counts. Same UUIDs the conn_subrate
 * bsim uses (ECV_LOAD_SVC/CHR_UUID) so the wire model matches the sim.
 */
#define LOAD_SVC_UUID \
	BT_UUID_128_ENCODE(0x5ab12705, 0x1234, 0x4c0d, 0x9e1a, 0xc0ffee000005)
#define LOAD_CHR_UUID \
	BT_UUID_128_ENCODE(0x5ab12706, 0x1234, 0x4c0d, 0x9e1a, 0xc0ffee000006)

/* 1250 us in 125 us units (the RCV floor). */
#define SCI_INTERVAL_125US      10U
/* Supervision timeout: 2 s in 10 ms units (both the conn-rate and subrate
 * params take the supervision timeout in 10 ms units).
 */
#define SCI_TIMEOUT_10MS        200U
#define SUBRATE_TIMEOUT_10MS    200U

#if defined(CONFIG_SCI_LATENCY_CENTRAL)

static struct bt_conn *default_conn;

/* __maybe_unused: the ECV-interval-sweep build (CONFIG_SCI_LATENCY_ECV_SWEEP)
 * does not run the subrate-factor sweep, so these are unused there.
 */
static const uint16_t sweep_factors[] __maybe_unused = { 1U, 2U, 4U, 8U };
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
#if defined(CONFIG_SCI_LATENCY_MODE_FSU_SWAP) || defined(CONFIG_SCI_LATENCY_MODE_ECV_LOAD)
static K_SEM_DEFINE(sem_fsu, 0, 1);
static K_SEM_DEFINE(sem_pupd, 0, 1);
#if defined(CONFIG_BT_USER_PHY_UPDATE)
static K_SEM_DEFINE(sem_phy, 0, 1);
#endif
static volatile uint8_t fsu_status = 0xFFU;
static volatile uint16_t fsu_frame_space;
static volatile uint8_t fsu_initiator = 0xFFU;
/* Disconnect bookkeeping for the soak verdicts: total drops and, of those, the
 * ones whose reason is a supervision (connection) timeout (0x08).
 */
static volatile uint32_t disconnect_count;
static volatile uint32_t supervision_count;
static volatile uint16_t last_param_interval;
#endif
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

extern volatile uint32_t sci_dbg_cnt[8]; /* lll_scan.c bench counters */

static void connected(struct bt_conn *conn, uint8_t err)
{
	printk("DBG: scan cnt prep %u rx %u tx %u done %u win %u abrt %u dcln %u acb %u\n",
	       sci_dbg_cnt[0], sci_dbg_cnt[1], sci_dbg_cnt[2], sci_dbg_cnt[3],
	       sci_dbg_cnt[4], sci_dbg_cnt[5], sci_dbg_cnt[6], sci_dbg_cnt[7]);
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
#if defined(CONFIG_SCI_LATENCY_MODE_FSU_SWAP) || defined(CONFIG_SCI_LATENCY_MODE_ECV_LOAD)
	disconnect_count++;
	if (reason == BT_HCI_ERR_CONN_TIMEOUT) {
		supervision_count++;
	}
#endif
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

#if defined(CONFIG_SCI_LATENCY_MODE_FSU_SWAP) || defined(CONFIG_SCI_LATENCY_MODE_ECV_LOAD)
static void fsu_updated(struct bt_conn *conn, uint8_t status,
			const struct bt_conn_le_frame_space_info *params)
{
	fsu_status = status;
	if (params != NULL) {
		fsu_frame_space = params->frame_space;
		fsu_initiator = params->initiator;
	}
	k_sem_give(&sem_fsu);
}

#if defined(CONFIG_BT_USER_PHY_UPDATE)
static void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *info)
{
	k_sem_give(&sem_phy);
}
#endif

static void mode_param_updated(struct bt_conn *conn, uint16_t interval,
			       uint16_t latency, uint16_t timeout)
{
	last_param_interval = interval;
	k_sem_give(&sem_pupd);
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
#if defined(CONFIG_SCI_LATENCY_MODE_FSU_SWAP) || defined(CONFIG_SCI_LATENCY_MODE_ECV_LOAD)
	.frame_space_updated = fsu_updated,
#if defined(CONFIG_BT_USER_PHY_UPDATE)
	.le_phy_updated = le_phy_updated,
#endif
#if !defined(CONFIG_SCI_LATENCY_PLAIN_SUBRATE)
	.le_param_updated = mode_param_updated,
#endif
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

static int __maybe_unused measure(int gap_ms, int32_t *out_min, int32_t *out_avg,
				  int32_t *out_max)
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
static int __maybe_unused apply_subrate(uint16_t factor, uint16_t continuation)
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

#if defined(CONFIG_SCI_LATENCY_ECV_SWEEP)
/* ECV interval sweep: starting just below the known-good RCV 1.25 ms, drive the
 * link down the whole ECV-and-not-RCV band at factor 1, measuring round-trip GATT
 * latency at each interval and stopping at the first one the silicon cannot
 * sustain -- the HW floor, wherever it is. bsim (idealised radio) held 750/625 us;
 * real single-timer silicon (cumulative drift + on-air margin at the standard
 * 150 us tIFS) may floor higher, which is exactly what this sweep determines.
 */
static const uint16_t ecv_sweep_125us[] = {
	9U, 8U, 7U, 6U, 5U, 4U, 3U, /* 1125/1000/875/750/625/500/375 us */
};

static int ecv_interval_sweep(void)
{
	LOG_INF("ECV interval sweep (factor 1, round-trip GATT read):");
	LOG_INF("requested | applied | idle min/avg/max us | burst min/avg/max us");

	for (int i = 0; i < (int)ARRAY_SIZE(ecv_sweep_125us); i++) {
		uint16_t u = ecv_sweep_125us[i];
		uint32_t want_us = (uint32_t)u * 125U;
		struct bt_conn_le_conn_rate_param rate = {
			.interval_min_125us = u, .interval_max_125us = u,
			.subrate_min = 1U, .subrate_max = 1U, .max_latency = 0U,
			.continuation_number = 0U,
			.supervision_timeout_10ms = SCI_TIMEOUT_10MS,
			.min_ce_len_125us = 1U, .max_ce_len_125us = 1U,
		};
		struct bt_conn_info info;
		int32_t imn, iavg, imx, bmn, bavg, bmx;
		int err;

		conn_rate_status = 0xFFU;
		err = bt_conn_le_conn_rate_request(default_conn, &rate);
		if (err) {
			LOG_ERR("%6u us | REQ rejected (err %d) -- floor is above this", want_us, err);
			break;
		}
		if (k_sem_take(&sem_conn_rate, K_SECONDS(5)) != 0 ||
		    conn_rate_status != BT_HCI_ERR_SUCCESS) {
			LOG_ERR("%6u us | change failed (status 0x%02x) -- floor reached",
				want_us, conn_rate_status);
			break;
		}
		if (!default_conn) {
			LOG_ERR("%6u us | link DROPPED applying interval -- floor reached", want_us);
			break;
		}
		(void)bt_conn_get_info(default_conn, &info);

		if (measure(IDLE_GAP_MS, &imn, &iavg, &imx) ||
		    measure(BURST_GAP_MS, &bmn, &bavg, &bmx)) {
			LOG_ERR("%6u us | link DROPPED during read -- floor reached", want_us);
			break;
		}

		LOG_INF("%6u us | %5u us | %6d/%6d/%6d | %6d/%6d/%6d",
			want_us, info.le.interval_us, imn, iavg, imx, bmn, bavg, bmx);

		if (!default_conn) {
			LOG_ERR("link dropped after %u us", want_us);
			break;
		}
	}

	LOG_INF("ECV sweep complete. Reset (J-Link/GDB) to re-run.");
	return 0;
}
#endif /* CONFIG_SCI_LATENCY_ECV_SWEEP */

#if defined(CONFIG_SCI_LATENCY_MODE_FSU_SWAP) || defined(CONFIG_SCI_LATENCY_MODE_ECV_LOAD)
/* Negotiate a Frame Space Update to [fs_min, fs_max] us and wait for the
 * Complete event. Returns 0 once the Complete arrived (status in fsu_status).
 */
static int fsu_negotiate(uint16_t fs_min, uint16_t fs_max)
{
	struct bt_conn_le_frame_space_param p = {
		.frame_space_min = fs_min,
		.frame_space_max = fs_max,
		.phys = BT_HCI_LE_FRAME_SPACE_UPDATE_PHY_1M_MASK |
			BT_HCI_LE_FRAME_SPACE_UPDATE_PHY_2M_MASK,
		.spacing_types =
			BT_HCI_LE_FRAME_SPACE_UPDATE_SPACING_TYPE_IFS_ACL_CP_MASK |
			BT_HCI_LE_FRAME_SPACE_UPDATE_SPACING_TYPE_IFS_ACL_PC_MASK,
	};
	int err;

	fsu_status = 0xFFU;
	fsu_frame_space = 0U;
	fsu_initiator = 0xFFU;
	k_sem_reset(&sem_fsu);
	err = bt_conn_le_frame_space_update(default_conn, &p);
	if (err) {
		LOG_ERR("FSU request (%u-%u us) failed (err %d)", fs_min, fs_max, err);
		return err;
	}
	if (k_sem_take(&sem_fsu, K_SECONDS(5)) != 0) {
		LOG_ERR("FSU (%u-%u us) Complete timed out", fs_min, fs_max);
		return -ETIMEDOUT;
	}
	return 0;
}
#endif

#if defined(CONFIG_SCI_LATENCY_MODE_FSU_SWAP)
/* MODE_FSU_SWAP: isolate the §5.1.30.1 inter-frame-space swap from ECV timing.
 * On a quiet 7.5 ms link, swap tIFS 150->floor->150 with periodic reads across
 * each direction, proving the swap event never drops a packet on real radios
 * (the bench is the first OTA test of the naive apply, which has no transitional
 * RX-window widening).
 */
static int run_fsu_swap(void)
{
	const uint16_t floor_us = (uint16_t)CONFIG_BT_CTLR_FSU_MIN_FRAME_SPACE_US;
	struct bt_conn_info info;
	uint32_t read_fail = 0U;
	uint32_t interval_us = 0U;
	int err;

	/* Settle to a known 7.5 ms interval with nothing else in flight. */
	err = bt_conn_le_param_update(default_conn,
				      BT_LE_CONN_PARAM(6U, 6U, 0U, SCI_TIMEOUT_10MS));
	if (err == 0) {
		(void)k_sem_take(&sem_pupd, K_SECONDS(5));
	}
	k_sleep(K_MSEC(500));
	if (default_conn && bt_conn_get_info(default_conn, &info) == 0) {
		interval_us = info.le.interval_us;
	}
	printk("RESULT: FSU_SWAP setup: link at %u us interval, quiescent; swapping "
	       "tIFS 150 -> %u us -> 150\n", interval_us, floor_us);

	/* Down-swap: request below the floor so the responder clamps to its floor. */
	err = fsu_negotiate(60U, 150U);
	if (err || fsu_status != BT_HCI_ERR_SUCCESS || fsu_frame_space != floor_us) {
		printk("RESULT: FSU_SWAP down-swap: err %d, status 0x%02x, frame_space "
		       "%u us, initiator %u\n", err, fsu_status, fsu_frame_space,
		       fsu_initiator);
		printk("VERDICT: FAIL FSU down-swap did not reach the %u us floor\n",
		       floor_us);
		return 0;
	}
	printk("RESULT: FSU_SWAP down-swap complete: status 0x%02x, frame_space %u us, "
	       "initiator %u\n", fsu_status, fsu_frame_space, fsu_initiator);

	/* Soak 30 s at the floor: one GATT read/s; the link must never drop and reads
	 * must keep succeeding across the swap.
	 */
	for (int i = 0; i < 30 && default_conn; i++) {
		if (probe_once() < 0) {
			read_fail++;
		}
		k_sleep(K_MSEC(1000));
	}
	printk("RESULT: FSU_SWAP floor soak done (30 s): disconnects %u, read failures "
	       "%u\n", disconnect_count, read_fail);

	/* Up-swap back to the standard 150 us, exercising the swap the other way. */
	if (default_conn) {
		err = fsu_negotiate(150U, 150U);
		if (err || fsu_status != BT_HCI_ERR_SUCCESS || fsu_frame_space != 150U) {
			printk("RESULT: FSU_SWAP up-swap: err %d, status 0x%02x, frame_space "
			       "%u us, initiator %u\n", err, fsu_status, fsu_frame_space,
			       fsu_initiator);
			printk("VERDICT: FAIL FSU up-swap back to 150 us failed\n");
			return 0;
		}
		printk("RESULT: FSU_SWAP up-swap complete: status 0x%02x, frame_space "
		       "%u us, initiator %u\n", fsu_status, fsu_frame_space, fsu_initiator);
	}

	/* Soak 10 s more at 150 us. */
	for (int i = 0; i < 10 && default_conn; i++) {
		if (probe_once() < 0) {
			read_fail++;
		}
		k_sleep(K_MSEC(1000));
	}

	printk("RESULT: FSU_SWAP counts: disconnects %u, supervision events %u, read "
	       "failures %u\n", disconnect_count, supervision_count, read_fail);
	if (disconnect_count == 0U && read_fail == 0U && default_conn) {
		printk("VERDICT: PASS FSU swap held both directions (0 disconnects, 0 read "
		       "failures over 40 s)\n");
	} else {
		printk("VERDICT: FAIL FSU swap: disconnects %u, read failures %u, link %s\n",
		       disconnect_count, read_fail, default_conn ? "up" : "DOWN");
	}
	return 0;
}
#endif /* CONFIG_SCI_LATENCY_MODE_FSU_SWAP */

#if defined(CONFIG_SCI_LATENCY_MODE_ECV_LOAD)
#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
/* Controller (ull_conn.c) per-connection count of events the device was present
 * for. Same binary in a combined host+controller build, so this links directly.
 */
extern volatile uint32_t ll_test_conn_event_count[];
#endif

static struct bt_uuid_128 load_disc_uuid;
static struct bt_gatt_discover_params load_disc;
static struct bt_gatt_subscribe_params load_sub;
static volatile uint32_t load_count;
static volatile uint32_t load_gaps;
static volatile bool load_seq_seen;
static uint32_t load_prev_seq;
static volatile bool load_subscribed;

#if defined(CONFIG_SCI_LATENCY_GPIO_ONEWAY)
/* One-way latency: the peripheral drives P1.11 to the sequence parity before
 * each notify attempt, so edge k marks sequence k's first attempt. The GPIOTE
 * ISR ring-buffers edge timestamps; the notify callback pairs them by sequence
 * number -- both timestamps on this device's clock.
 */
#define OW_RING_SZ 64U
static uint32_t ow_edge_ts[OW_RING_SZ];
static volatile uint32_t ow_edge_cnt;
static uint32_t ow_seq0;
static bool ow_have_seq0;
static volatile uint32_t ow_min, ow_max, ow_n, ow_unmatched;
static volatile uint64_t ow_sum;
static struct gpio_callback ow_cb_data;

/* Stage deltas, computed per notification in the GATT callback:
 * fifo->acl = controller prio_recv_thread handoff through recv_thread encode;
 * acl->cb   = bt_recv -> RX workqueue -> L2CAP/ATT/GATT -> app callback.
 */
static struct ow_acc ow_stage_fifo2acl, ow_stage_acl2cb;

/* Latency distribution: 50 us buckets to 6.4 ms + overflow. Fine enough to
 * separate retransmission quantisation (peaks at k x interval) from wall-clock
 * interference blackouts (interval-independent smear).
 */
#define OW_HIST_BUCKET_US 50U
#define OW_HIST_BUCKETS   128U
static uint32_t ow_hist[OW_HIST_BUCKETS + 1U]; /* [last] = overflow */

static uint32_t ow_percentile(uint32_t pct_x100)
{
	/* pct_x100: e.g. 9999 = p99.99. Returns the bucket upper edge in us. */
	uint64_t target = ((uint64_t)ow_n * pct_x100 + 9999U) / 10000U;
	uint64_t cum = 0U;

	for (uint32_t i = 0U; i <= OW_HIST_BUCKETS; i++) {
		cum += ow_hist[i];
		if (cum >= target) {
			return (i + 1U) * OW_HIST_BUCKET_US;
		}
	}
	return ow_max;
}

static uint32_t ow_last_edge;
static volatile uint32_t ow_glitches;

static void ow_edge_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	uint32_t now = k_cycle_get_32();
	uint32_t cnt = ow_edge_cnt;

	/* Glitch rejection: real edges are >= one generator period (>= 375 us)
	 * apart, and the level alternates deterministically with the edge count
	 * (the peripheral writes seq parity). Contact bounce / EMI pickup on the
	 * cross-wire fails one of the two; count it instead of corrupting the
	 * pairing.
	 */
	if (cnt > 0U && k_cyc_to_us_floor32(now - ow_last_edge) < 150U) {
		ow_glitches++;
		return;
	}
	if (gpio_pin_get_raw(dev, ONEWAY_GPIO_PIN) != (int)((cnt & 1U) ^ 1U)) {
		ow_glitches++;
		return;
	}
	ow_last_edge = now;
	ow_edge_ts[cnt % OW_RING_SZ] = now;
	ow_edge_cnt = cnt + 1U;
}

static void ow_stats_reset(void)
{
	ow_min = UINT32_MAX;
	ow_max = 0U;
	ow_n = 0U;
	ow_sum = 0U;
	ow_unmatched = 0U;
	memset(ow_hist, 0, sizeof(ow_hist));
}

static int ow_capture_init(void)
{
	const struct device *port = DEVICE_DT_GET(ONEWAY_GPIO_NODE);
	int err;

	ow_stats_reset();
	err = gpio_pin_configure(port, ONEWAY_GPIO_PIN, GPIO_INPUT);
	if (!err) {
		gpio_init_callback(&ow_cb_data, ow_edge_isr, BIT(ONEWAY_GPIO_PIN));
		err = gpio_add_callback(port, &ow_cb_data);
	}
	if (!err) {
		err = gpio_pin_interrupt_configure(port, ONEWAY_GPIO_PIN,
						   GPIO_INT_EDGE_BOTH);
	}
	return err;
}

static void ow_pair(uint32_t seq, uint32_t now)
{
	uint32_t cnt = ow_edge_cnt;
	uint32_t idx;

	if (!ow_have_seq0) {
		/* First notification seen; its edge is the first edge captured. */
		ow_seq0 = seq;
		ow_have_seq0 = true;
	}
	idx = seq - ow_seq0;
	if (idx >= cnt || (cnt - idx) > OW_RING_SZ) {
		ow_unmatched++;
		return;
	}

	uint32_t us = k_cyc_to_us_floor32(now - ow_edge_ts[idx % OW_RING_SZ]);

	if (us < ow_min) {
		ow_min = us;
	}
	if (us > ow_max) {
		ow_max = us;
	}
	ow_sum += us;
	ow_n++;
	ow_hist[MIN(us / OW_HIST_BUCKET_US, OW_HIST_BUCKETS)]++;
}
#endif /* CONFIG_SCI_LATENCY_GPIO_ONEWAY */

static uint8_t load_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
			      const void *data, uint16_t length)
{
	if (!data) {
		return BT_GATT_ITER_STOP;
	}
	/* zmk_split_input_event_payload: u8 type, u16 code, u32 value, u8 sync; the
	 * sequence number rides in `value` (offset 3, LE). The baseline is the first
	 * notification seen, so anything streamed before the subscribe is not a gap.
	 */
	if (length == 8U) {
		uint32_t seq = sys_get_le32((const uint8_t *)data + 3);

#if defined(CONFIG_SCI_LATENCY_GPIO_ONEWAY)
		uint32_t now = k_cycle_get_32();
		uint32_t fifo_cyc = sci_dbg_fifo_cyc;
		uint32_t acl_cyc = sci_dbg_acl_cyc;

		ow_pair(seq, now);
		/* Stage deltas from the driver stamps (most-recent ACL; exact at
		 * one notification per callback, min values valid regardless).
		 */
		ow_acc_add(&ow_stage_fifo2acl, k_cyc_to_us_floor32(acl_cyc - fifo_cyc));
		ow_acc_add(&ow_stage_acl2cb, k_cyc_to_us_floor32(now - acl_cyc));
#endif
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
		memcpy(&load_disc_uuid, BT_UUID_DECLARE_128(LOAD_CHR_UUID),
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
			LOG_ERR("load subscribe failed (err %d)", err);
		} else {
			load_subscribed = true;
		}
	}
	return BT_GATT_ITER_STOP;
}

/* MODE_ECV_LOAD: drive the link to the ECV target interval, optionally negotiate
 * FSU (AFTER the conn-rate change -- the conn-rate apply resets tIFS to 150 us,
 * so an earlier FSU would be undone; ull_conn.c:2784), then soak the ZMK-split
 * pointing load (one 8-byte notification per interval) counting delivered vs
 * offered and sequence gaps.
 */
static int run_ecv_load(void)
{
	const uint16_t interval_125us = (uint16_t)CONFIG_SCI_LATENCY_ECV_INTERVAL_125US;
	const uint32_t interval_us = (uint32_t)interval_125us * 125U;
	const bool use_fsu = IS_ENABLED(CONFIG_SCI_LATENCY_ECV_FSU);
	uint16_t applied_tifs_us = 150U;
	struct bt_conn_le_conn_rate_param rate = {
		.interval_min_125us = interval_125us,
		.interval_max_125us = interval_125us,
		.subrate_min = 1U, .subrate_max = 1U, .max_latency = 0U,
		.continuation_number = 0U,
		.supervision_timeout_10ms = SCI_TIMEOUT_10MS,
		.min_ce_len_125us = 1U, .max_ce_len_125us = 1U,
	};
	struct bt_conn_info info;
	uint32_t delivered, gaps, expected, pct, pct_off, offered;
	uint32_t before, gaps_before;
	uint64_t period_ticks;
	uint32_t ce_events = 0U;
	int64_t t0, elapsed_ms, next_interim;
	uint32_t soak_ms = (uint32_t)CONFIG_SCI_LATENCY_SOAK_SECONDS * 1000U;
	bool survived;
	int err;
#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
	uint16_t load_handle = 0xFFFFU;
	uint32_t ce_before = 0U;
#endif

	/* 2M PHY first, while still at the default interval (an instant procedure is
	 * safest on the slow link; the conn-rate apply does not touch the PHY). This
	 * is what a real ZMK split link runs and what the loaded bsim matrix used.
	 */
	err = bt_conn_le_phy_update(default_conn, BT_CONN_LE_PHY_PARAM_2M);
	if (err == 0) {
		(void)k_sem_take(&sem_phy, K_SECONDS(5));
	}

	/* Drive to the ECV target via the Connection Rate procedure. */
	conn_rate_status = 0xFFU;
	err = bt_conn_le_conn_rate_request(default_conn, &rate);
	if (err || k_sem_take(&sem_conn_rate, K_SECONDS(5)) != 0 ||
	    conn_rate_status != BT_HCI_ERR_SUCCESS) {
		printk("RESULT: ECV_LOAD conn-rate to %u us failed (err %d, status 0x%02x)\n",
		       interval_us, err, conn_rate_status);
		printk("VERDICT: FAIL could not apply the %u us interval\n", interval_us);
		return 0;
	}
	if (!default_conn || bt_conn_get_info(default_conn, &info) != 0 ||
	    info.le.interval_us != interval_us) {
		printk("RESULT: ECV_LOAD interval after rate change = %u us, expected %u\n",
		       default_conn ? info.le.interval_us : 0U, interval_us);
		printk("VERDICT: FAIL interval not applied at %u us\n", interval_us);
		return 0;
	}
	printk("RESULT: ECV_LOAD %u us interval applied (FSU %s)\n", interval_us,
	       use_fsu ? "requested" : "off");
	k_sleep(K_MSEC(500));

	if (use_fsu) {
		/* AFTER the conn-rate change: the conn-rate apply reset tIFS to 150 us. */
		err = fsu_negotiate(60U, 150U);
		if (err || fsu_status != BT_HCI_ERR_SUCCESS ||
		    fsu_frame_space != (uint16_t)CONFIG_BT_CTLR_FSU_MIN_FRAME_SPACE_US) {
			printk("RESULT: ECV_LOAD FSU negotiation FAILED (err %d, status "
			       "0x%02x, fs %u us) -- soaking at 150 us tIFS\n", err,
			       fsu_status, fsu_frame_space);
		} else {
			applied_tifs_us = (uint16_t)CONFIG_BT_CTLR_FSU_MIN_FRAME_SPACE_US;
			printk("RESULT: ECV_LOAD FSU negotiated %u us tIFS (initiator %u)\n",
			       applied_tifs_us, fsu_initiator);
		}
		k_sleep(K_MSEC(500));
	}

	if (!default_conn) {
		printk("RESULT: ECV_LOAD link dropped before the soak at %u us\n", interval_us);
		printk("VERDICT: FAIL link dropped before streaming at %u us\n", interval_us);
		return 0;
	}

#if defined(CONFIG_SCI_LATENCY_GPIO_ONEWAY)
	/* Arm the edge capture BEFORE subscribing, so no edge precedes it. */
	err = ow_capture_init();
	if (err) {
		printk("RESULT: ONEWAY GPIO capture init failed (err %d) -- no one-way "
		       "figures this run\n", err);
	}
#endif

	/* Discover + subscribe to the load characteristic. */
	memcpy(&load_disc_uuid, BT_UUID_DECLARE_128(LOAD_SVC_UUID), sizeof(load_disc_uuid));
	load_disc.uuid = &load_disc_uuid.uuid;
	load_disc.func = load_disc_cb;
	load_disc.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	load_disc.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	load_disc.type = BT_GATT_DISCOVER_PRIMARY;
	err = bt_gatt_discover(default_conn, &load_disc);
	if (err) {
		printk("RESULT: ECV_LOAD discover failed (err %d) at %u us\n", err, interval_us);
		printk("VERDICT: FAIL GATT discover failed at %u us\n", interval_us);
		return 0;
	}
	for (int i = 0; i < 250 && !load_subscribed && default_conn; i++) {
		k_sleep(K_MSEC(20));
	}
	if (!load_subscribed) {
		printk("RESULT: ECV_LOAD subscribe never completed (link %s) at %u us\n",
		       default_conn ? "up" : "DROPPED", interval_us);
		printk("VERDICT: FAIL could not subscribe to the load characteristic\n");
		return 0;
	}

	/* Wait for the stream to start, then let the cadence settle. */
	for (int i = 0; i < 250 && load_count == 0U && default_conn; i++) {
		k_sleep(K_MSEC(20));
	}
	k_sleep(K_MSEC(500));

#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
	if (default_conn && !bt_hci_get_conn_handle(default_conn, &load_handle) &&
	    load_handle < CONFIG_BT_MAX_CONN) {
		ce_before = ll_test_conn_event_count[load_handle];
	} else {
		load_handle = 0xFFFFU;
	}
#endif

	/* Soak. Interim RESULT lines every 5 s so a hung run is diagnosable. */
#if defined(CONFIG_SCI_LATENCY_GPIO_ONEWAY)
	ow_stats_reset(); /* soak-only stats; pairing state carries over */
#endif
	before = load_count;
	gaps_before = load_gaps;
	t0 = k_uptime_get();
	next_interim = t0 + 5000;
	while ((k_uptime_get() - t0) < soak_ms && default_conn) {
		k_sleep(K_MSEC(100));
		if (k_uptime_get() >= next_interim) {
			printk("RESULT: ECV_LOAD interim %u us: %lld ms in, delivered %u, "
			       "seq gaps %u, link up\n", interval_us,
			       k_uptime_get() - t0, load_count - before,
			       load_gaps - gaps_before);
			next_interim += 5000;
		}
	}
	elapsed_ms = k_uptime_get() - t0;
	survived = (default_conn != NULL);
	delivered = load_count - before;
	gaps = load_gaps - gaps_before;
#if defined(CONFIG_BT_CTLR_TEST_CONN_EVENT_COUNT)
	if (load_handle != 0xFFFFU) {
		ce_events = ll_test_conn_event_count[load_handle] - ce_before;
	}
#endif

	/* Interval-nominal: one notification per connection interval over the window. */
	expected = (uint32_t)((elapsed_ms * 1000) / (int64_t)interval_us);
	pct = expected ? ((delivered * 100U) / expected) : 0U;

	/* Offered: what the peripheral's generator could actually send. Its k_timer
	 * rounds the period UP to the system tick (32 us on the nRF54L15 GRTC,
	 * CONFIG_SYS_CLOCK_TICKS_PER_SEC), so it fires slightly slower than nominal.
	 * Gating delivery against OFFERED (not interval-nominal) avoids failing on a
	 * generator shortfall the controller never saw; interval-nominal stays in the
	 * line as context.
	 */
	period_ticks = ((uint64_t)interval_us * CONFIG_SYS_CLOCK_TICKS_PER_SEC +
			(USEC_PER_SEC - 1)) / USEC_PER_SEC;
	offered = (uint32_t)(((uint64_t)elapsed_ms * CONFIG_SYS_CLOCK_TICKS_PER_SEC) /
			     (MSEC_PER_SEC * period_ticks));
	pct_off = offered ? ((delivered * 100U) / offered) : 0U;

	printk("RESULT: ECV_LOAD final: interval %u us, tIFS %u us (FSU %s), PHY 2M, "
	       "load 1x8B-payload(17B-LL-PDU)/CE: delivered %u of offered %u (%u%%; "
	       "%u%% of interval-nominal %u), seq gaps %u, CEs %u, link %s after %lld ms\n",
	       interval_us, applied_tifs_us, use_fsu ? "on" : "off", delivered, offered,
	       pct_off, pct, expected, gaps, ce_events, survived ? "up" : "DROPPED",
	       elapsed_ms);
#if defined(CONFIG_SCI_LATENCY_GPIO_ONEWAY)
	if (ow_n > 0U) {
		printk("RESULT: ONEWAY latency (generation edge -> notify callback) "
		       "min %u / avg %u / max %u us over %u samples (unmatched %u, "
		       "glitches %u)\n",
		       ow_min, (uint32_t)(ow_sum / ow_n), ow_max, ow_n, ow_unmatched,
		       ow_glitches);
		printk("RESULT: ONEWAY pct p50 %u p90 %u p99 %u p99.9 %u p99.99 %u "
		       "max %u us\n", ow_percentile(5000U), ow_percentile(9000U),
		       ow_percentile(9900U), ow_percentile(9990U), ow_percentile(9999U),
		       ow_max);
		if (ow_stage_acl2cb.n > 0U) {
			printk("RESULT: OWSTAGES ctlr-fifo->acl-encode min %u avg %u max "
			       "%u us; acl->gatt-cb min %u avg %u max %u us (n %u)\n",
			       ow_stage_fifo2acl.min,
			       (uint32_t)(ow_stage_fifo2acl.sum / ow_stage_fifo2acl.n),
			       ow_stage_fifo2acl.max, ow_stage_acl2cb.min,
			       (uint32_t)(ow_stage_acl2cb.sum / ow_stage_acl2cb.n),
			       ow_stage_acl2cb.max, ow_stage_acl2cb.n);
		}
		for (uint32_t i = 0U; i <= OW_HIST_BUCKETS; i++) {
			if (ow_hist[i] != 0U) {
				printk("RESULT: OWHIST %u %u\n", i * OW_HIST_BUCKET_US,
				       ow_hist[i]);
				/* Pace the dump so a slow serial reader keeps up. */
				k_sleep(K_MSEC(5));
			}
		}
	} else {
		printk("RESULT: ONEWAY no paired samples (edges %u, unmatched %u) -- "
		       "is the P1.11 cross-wire connected?\n", ow_edge_cnt, ow_unmatched);
	}
#endif

	if (survived && gaps == 0U && pct_off >= 90U) {
		if (default_conn) {
			(void)bt_conn_disconnect(default_conn,
						 BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
		printk("VERDICT: PASS ECV load held at %u us (FSU %s): %u%% of offered, 0 "
		       "gaps, link survived\n", interval_us, use_fsu ? "on" : "off", pct_off);
	} else {
		printk("VERDICT: FAIL ECV load at %u us (FSU %s): %u%% of offered (floor "
		       "90%%), %u gaps, link %s\n", interval_us, use_fsu ? "on" : "off",
		       pct_off, gaps, survived ? "up" : "DROPPED");
	}
	return 0;
}
#endif /* CONFIG_SCI_LATENCY_MODE_ECV_LOAD */

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

#if defined(CONFIG_SCI_LATENCY_MODE_FSU_SWAP)
	ARG_UNUSED(rate);
	return run_fsu_swap();
#elif defined(CONFIG_SCI_LATENCY_MODE_ECV_LOAD)
	ARG_UNUSED(rate);
	return run_ecv_load();
#elif defined(CONFIG_SCI_LATENCY_ECV_SWEEP)
	ARG_UNUSED(rate);
	return ecv_interval_sweep();
#else
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
#endif /* CONFIG_SCI_LATENCY_ECV_SWEEP */
}

#else /* peripheral */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct bt_conn *periph_conn;

/* ZMK-split pointing-load characteristic: the central subscribes and the
 * peripheral streams one 8-byte input-event payload per connection interval on
 * it (the ECV_LOAD mode). In every other mode the central never subscribes, so
 * the CCC stays disabled and nothing is streamed -- one peripheral image serves
 * all modes with no build variation beyond peripheral.conf.
 */
static volatile bool load_ccc_enabled;
static volatile uint32_t stream_interval_us;

static void load_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);

BT_GATT_SERVICE_DEFINE(load_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(LOAD_SVC_UUID)),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(LOAD_CHR_UUID),
			       BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(load_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

/* The generator: a k_timer paces one notification per connection interval, and
 * a semaphore hands each tick to the streaming thread (bt_gatt_notify must not
 * run from the timer's ISR context). Depth 8 tolerates a brief host-thread
 * backlog; beyond that ticks collapse and the central sees a rate shortfall (the
 * intended signal), not a fake sequence gap.
 */
static K_SEM_DEFINE(stream_tick, 0, 8);

static void stream_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&stream_tick);
}

static K_TIMER_DEFINE(stream_timer, stream_timer_fn, NULL);

/* Re-arm the generator to the live connection interval. Called on CCC enable and
 * whenever the interval changes (conn_rate_changed / le_param_updated), reading
 * the current interval from bt_conn_info -- so the stream always paces to the
 * interval the central has driven the link to.
 */
static void stream_rearm(void)
{
	struct bt_conn_info info;
	uint32_t us;

	if (!periph_conn || !load_ccc_enabled ||
	    bt_conn_get_info(periph_conn, &info) != 0) {
		return;
	}
	us = info.le.interval_us;
#if CONFIG_SCI_LATENCY_STREAM_PERIOD_US > 0
	/* Fixed-rate generator: decouple the offered load from the connection
	 * interval (e.g. a 1 kHz sensor on a 625 us link).
	 */
	us = CONFIG_SCI_LATENCY_STREAM_PERIOD_US;
#endif
	if (us == 0U || us == stream_interval_us) {
		return;
	}
	stream_interval_us = us;
	/* K_USEC ceil-rounds to system ticks, so the generator runs at most ~1 tick
	 * slow per period -- accounted for in the central's offered-load math.
	 */
	k_timer_start(&stream_timer, K_USEC(us), K_USEC(us));
	LOG_INF("Load stream armed at %u us CE", us);
}

static void load_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	load_ccc_enabled = (value == BT_GATT_CCC_NOTIFY);
	if (load_ccc_enabled) {
		stream_rearm();
	} else {
		k_timer_stop(&stream_timer);
		stream_interval_us = 0U;
	}
}

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
	periph_conn = bt_conn_ref(conn);
	LOG_INF("Connected -- central will drive SCI/FSU/subrating");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	/* Do NOT re-advertise here: the stack still holds a ref on the dropped
	 * conn, so a connectable bt_le_adv_start returns -ENOMEM and the
	 * peripheral goes permanently off-air. Re-advertise from recycled()
	 * (fires once the conn slot is actually free) -- same fix as the
	 * sci-interop bench (5a7e4cc72f2) and ZMK's split peripheral.
	 */
	LOG_WRN("Disconnected (reason 0x%02x); will re-advertise on recycle", reason);
	k_timer_stop(&stream_timer);
	load_ccc_enabled = false;
	stream_interval_us = 0U;
	if (periph_conn) {
		bt_conn_unref(periph_conn);
		periph_conn = NULL;
	}
}

static void recycled(void)
{
	start_adv();
}

static void conn_rate_changed(struct bt_conn *conn, uint8_t status,
			      const struct bt_conn_le_conn_rate_changed *params)
{
	if (status == BT_HCI_ERR_SUCCESS && params != NULL) {
		LOG_INF("Conn rate change applied: interval %u us, factor %u",
			params->interval_us, params->subrate_factor);
	}
	stream_rearm();
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	stream_rearm();
}

static void subrate_changed(struct bt_conn *conn,
			    const struct bt_conn_le_subrate_changed *params)
{
	if (params->status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("Subrate applied: factor %u, peripheral_latency %u",
			params->factor, params->peripheral_latency);
	}
}

#if defined(CONFIG_BT_FRAME_SPACE_UPDATE)
static void frame_space_updated(struct bt_conn *conn, uint8_t status,
				const struct bt_conn_le_frame_space_info *params)
{
	if (status == BT_HCI_ERR_SUCCESS && params != NULL) {
		LOG_INF("Frame space updated (responder): %u us, initiator %u",
			params->frame_space, params->initiator);
	}
}
#endif

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled,
	.conn_rate_changed = conn_rate_changed,
	.subrate_changed = subrate_changed,
	.le_param_updated = le_param_updated,
#if defined(CONFIG_BT_FRAME_SPACE_UPDATE)
	.frame_space_updated = frame_space_updated,
#endif
};

int main(void)
{
	uint32_t seq = 0U;
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed (err %d)", err);
		return 0;
	}
	bt_conn_cb_register(&conn_callbacks);

	LOG_INF("SCI latency tester: PERIPHERAL \"%s\"", DEVICE_NAME);
	start_adv();

	/* Streaming thread for the ECV_LOAD mode: one 8-byte zmk_split_input_event
	 * _payload-shaped notification per generator tick while subscribed. The
	 * sequence number rides in the payload's `value` field so the central can
	 * count delivery gaps; it advances ONLY on a successful queue, so a send
	 * failure shows up at the central as a rate shortfall, not a fake gap.
	 */
#if defined(CONFIG_SCI_LATENCY_GPIO_ONEWAY)
	const struct device *ow_port = DEVICE_DT_GET(ONEWAY_GPIO_NODE);
	static struct ow_acc notify_cost;

	(void)gpio_pin_configure(ow_port, ONEWAY_GPIO_PIN, GPIO_OUTPUT_LOW);
#endif

	while (true) {
		uint8_t payload[8];

		if (k_sem_take(&stream_tick, K_MSEC(1000)) != 0) {
			continue;
		}
		if (!periph_conn || !load_ccc_enabled) {
			continue;
		}

		payload[0] = 0x02U;                 /* type: EV_REL */
		sys_put_le16(0x0000U, &payload[1]); /* code: REL_X */
		sys_put_le32(seq, &payload[3]);     /* value: sequence number */
		payload[7] = 0x01U;                 /* sync */

#if defined(CONFIG_SCI_LATENCY_GPIO_ONEWAY)
		/* Mark sequence `seq`'s generation instant: level = parity, written
		 * before the notify attempt. A retry of the same seq re-writes the
		 * same level (no edge), so the peer sees exactly one edge per seq.
		 */
		(void)gpio_pin_set_raw(ow_port, ONEWAY_GPIO_PIN, (int)((seq & 1U) ^ 1U));

		uint32_t t0 = k_cycle_get_32();

		sci_dbg_gen = t0;
		if (bt_gatt_notify(periph_conn, &load_svc.attrs[2], payload,
				   sizeof(payload)) == 0) {
			ow_acc_add(&notify_cost, k_cyc_to_us_floor32(k_cycle_get_32() - t0));
			seq++;
			if ((seq & 0x3FFFU) == 0U) {
				printk("RESULT: NOTIFYCOST bt_gatt_notify min %u avg %u "
				       "max %u us (n %u)\n", notify_cost.min,
				       (uint32_t)(notify_cost.sum / notify_cost.n),
				       notify_cost.max, notify_cost.n);
				printk("RESULT: TXPATH gen->pick %u/%u/%u us; "
				       "enq->pick %u/%u/%u us; lllq->pick %u/%u/%u us "
				       "(n %u)\n",
				       sci_txp_acc[0],
				       sci_txp_acc[3] ? sci_txp_acc[1] / sci_txp_acc[3] : 0U,
				       sci_txp_acc[2], sci_txp_acc[4],
				       sci_txp_acc[7] ? sci_txp_acc[5] / sci_txp_acc[7] : 0U,
				       sci_txp_acc[6], sci_txp_acc[8],
				       sci_txp_acc[11] ? sci_txp_acc[9] / sci_txp_acc[11] : 0U,
				       sci_txp_acc[10], sci_txp_acc[3]);
			}
		}
#else
		if (bt_gatt_notify(periph_conn, &load_svc.attrs[2], payload,
				   sizeof(payload)) == 0) {
			seq++;
		}
#endif
	}

	return 0;
}

#endif /* CONFIG_SCI_LATENCY_CENTRAL */
