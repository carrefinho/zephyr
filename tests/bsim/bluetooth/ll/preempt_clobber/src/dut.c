/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Device 1 - the DUT, dual-role like a ZMK split central: peripheral to
 * `host`, central to `split`. The host drops and re-makes its link every
 * cycle_ms; on each host disconnect, with clobber=1, the DUT zeroes the LLL's
 * preempt_req from the system work queue, the way the stray
 * peripheral_battery_levels[234] = 0 write in ZMK would on a build where
 * that byte is preempt_req. A monitor thread samples the preempt state every
 * 1 ms and logs stale periods (preempt_req != preempt_ack with no preempt
 * ticker start pending, for longer than STALE_MS) and late prepares.
 *
 * Args (-argstest): clobber=0|1|2 (2 = preempt_stop_ack) load=0|1 yield=0|1 lat_burst=us lat_period=us lat_isr=n
 * Log lines parsed by the run script: CLOBBER, STALE_BEGIN, STALE_END, STATS,
 * FINAL.
 */
#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

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

static struct bt_conn *split_conn;  /* DUT is CENTRAL on this link */
static struct bt_conn *host_conn;   /* DUT is PERIPHERAL on this link */
static volatile bool scanning;

static uint32_t clobber;
static uint32_t load;
static uint32_t yield_en;

/* LLL test hooks (subsys/bluetooth/controller/ll_sw/nordic/lll/lll.c). */
extern volatile uint32_t lll_test_overhead_cnt;
extern volatile uint32_t lll_test_preempt_skip_cnt;
extern volatile uint32_t lll_test_preempt_start_cnt;
void lll_test_preempt_req_set(uint8_t val);
void lll_test_preempt_stop_ack_set(uint8_t val);
void lll_test_preempt_stop_state(uint8_t *stop_req, uint8_t *stop_ack);
void lll_test_preempt_state(uint8_t *req, uint8_t *ack, uint8_t *start_req,
			    uint8_t *start_ack);

static uint32_t host_disconnects;
static uint32_t clobbers;
static uint32_t clobbers_effective; /* the target byte was nonzero, so it changed */

/* ---- CPU-latency injector ------------------------------------------------
 *
 * bsim executes all CPU work in ZERO simulated time, so the ULL/ISR latency a
 * real ZMK central experiences (kernel spinlock critical sections, SPI display
 * + kscan ISRs, and internal-flash writes/erases that stall the whole CPU for
 * 41 us .. 85 ms on nRF52) never occurs, and the controller's prepares are
 * always enqueued, preempted, and dequeued exactly on time. The June sweeps
 * varied only link GEOMETRY (intervals/drift/phase) and never overflowed.
 *
 * This thread periodically masks interrupts and burns simulated time, delaying
 * the radio/ticker ISRs the way real critical sections and flash stalls do:
 *   lat_burst  = irq-locked busy-wait length in us (0 = injector disabled)
 *   lat_period = sleep between bursts in us
 * Passed per-run via bsim test args: -argstest lat_burst=N lat_period=N
 */
static uint32_t lat_burst_us;
static uint32_t lat_period_us = 3300;

/* Injection mode (-argstest lat_isr=):
 *   0 - thread + irq_lock() + k_busy_wait(). Faithful, but SIGSEGVs the
 *       3.5-3.7-era simulator: an IRQ latched during the locked burst is
 *       dispatched at irq_unlock() through an unpopulated irq_vector_table
 *       entry (jump to a garbage address in posix_irq_handler).
 *   1 - k_busy_wait() inside a k_timer ISR. Crash-free but INERT: the
 *       simulator advances time with nested dispatch during the busy-wait,
 *       so the radio is serviced on schedule and nothing is delayed
 *       (verified: 4.0/4.1 stop reproducing under this mode).
 *   2 - (default) thread that irq_disable()s only the controller's own
 *       IRQs (RADIO, TIMER0, RTC0, SWI4, SWI5) around the busy-wait. Their
 *       vectors are populated, so re-enabling dispatches pending IRQs
 *       through valid entries on every tree era, while the radio genuinely
 *       waits out the burst. Only currently-enabled IRQs are touched.
 */
static uint32_t lat_isr = 2;

/* nRF52 controller IRQ numbers: RADIO=1, TIMER0=8, RTC0=11, SWI4=20, SWI5=21 */
static const unsigned int lat_masked_irqs[] = {1, 8, 11, 20, 21};

/* Generous stack: on the 3.5-3.7-era kernels a 1024 B stack for this thread
 * silently overflowed on the POSIX arch (no MPU) - corrupted ACL traffic then
 * a native SIGSEGV ~30 ms after arming, which masqueraded as a passing rung
 * until the verdict logic required end-of-sim evidence.
 */
static K_THREAD_STACK_DEFINE(lat_stack, 4096);
static struct k_thread lat_thread;

static void lat_burst_timer_cb(struct k_timer *timer)
{
	/* ISR context: the busy-wait stalls the simulator's sequential IRQ
	 * dispatch loop, delaying radio/ticker processing by the burst length.
	 */
	k_busy_wait(lat_burst_us);
}

static K_TIMER_DEFINE(lat_burst_timer, lat_burst_timer_cb, NULL);

static void lat_injector(void *p1, void *p2, void *p3)
{
	if (lat_isr == 1) {
		k_timer_start(&lat_burst_timer, K_USEC(lat_period_us),
			      K_USEC(lat_period_us));
		return;
	}

	while (true) {
		k_usleep(lat_period_us);

		if (lat_isr == 2) {
			uint32_t was = 0;

			for (int i = 0; i < ARRAY_SIZE(lat_masked_irqs); i++) {
				if (arch_irq_is_enabled(lat_masked_irqs[i])) {
					was |= BIT(i);
					irq_disable(lat_masked_irqs[i]);
				}
			}

			k_busy_wait(lat_burst_us);

			for (int i = 0; i < ARRAY_SIZE(lat_masked_irqs); i++) {
				if (was & BIT(i)) {
					irq_enable(lat_masked_irqs[i]);
				}
			}
		} else {
			unsigned int key = irq_lock();

			k_busy_wait(lat_burst_us);
			irq_unlock(key);
		}
	}
}

/* ---- the modeled stray write ------------------------------------------ */

static void clobber_work_handler(struct k_work *work)
{
	uint8_t req, ack, sreq, sack, stop_req, stop_ack;

	lll_test_preempt_state(&req, &ack, &sreq, &sack);
	lll_test_preempt_stop_state(&stop_req, &stop_ack);
	clobbers++;
	if (((clobber == 2U) ? stop_ack : req) != 0U) {
		clobbers_effective++;
	}
	printk("CLOBBER n=%u target=%s t_ms=%lld req=%u ack=%u start_req=%u start_ack=%u "
	       "stop_req=%u stop_ack=%u\n",
	       clobbers, (clobber == 2U) ? "stop_ack" : "req", k_uptime_get(), req, ack,
	       sreq, sack, stop_req, stop_ack);
	if (clobber == 2U) {
		/* Coron's production layout: the write lands one byte later. */
		lll_test_preempt_stop_ack_set(0U);
	} else {
		lll_test_preempt_req_set(0U);
	}
}

static K_WORK_DEFINE(clobber_work, clobber_work_handler);

/* ---- preempt state monitor -------------------------------------------- */

/* A real preempt timeout expires within one event (a few ms); req != ack for
 * longer than this, with no ticker start in flight, means the state is stale.
 */
#define STALE_MS 50

static uint32_t stale_periods;
static int64_t stale_total_ms;
static int64_t stale_max_ms;
static int64_t stale_since = -1;
static bool stale_reported;

static K_THREAD_STACK_DEFINE(mon_stack, 2048);
static struct k_thread mon_thread;

static void print_stats(const char *tag)
{
	int64_t cur = (stale_since >= 0) ? (k_uptime_get() - stale_since) : 0;

	printk("%s t_ms=%lld host_disc=%u clobbers=%u effective=%u overhead=%u "
	       "preempt_start=%u preempt_skip=%u stale_periods=%u stale_total_ms=%lld "
	       "stale_max_ms=%lld stale_now_ms=%lld\n",
	       tag, k_uptime_get(), host_disconnects, clobbers, clobbers_effective,
	       lll_test_overhead_cnt, lll_test_preempt_start_cnt,
	       lll_test_preempt_skip_cnt, stale_periods, stale_total_ms, stale_max_ms,
	       (stale_since >= 0 && stale_reported) ? cur : 0);
}

static void monitor(void *p1, void *p2, void *p3)
{
	int64_t last_stats = k_uptime_get();
	uint32_t last_overhead = 0;

	while (true) {
		uint8_t req, ack, sreq, sack;
		int64_t now;

		k_sleep(K_MSEC(1));
		now = k_uptime_get();
		lll_test_preempt_state(&req, &ack, &sreq, &sack);

		if ((req != ack) && (sreq == sack)) {
			if (stale_since < 0) {
				stale_since = now;
				stale_reported = false;
			} else if (!stale_reported && (now - stale_since) > STALE_MS) {
				stale_reported = true;
				stale_periods++;
				printk("STALE_BEGIN t_ms=%lld req=%u ack=%u\n",
				       stale_since, req, ack);
			}
		} else if (stale_since >= 0) {
			if (stale_reported) {
				int64_t dur = now - stale_since;

				stale_total_ms += dur;
				stale_max_ms = MAX(stale_max_ms, dur);
				printk("STALE_END t_ms=%lld dur_ms=%lld\n", now, dur);
			}
			stale_since = -1;
		}

		if (lll_test_overhead_cnt != last_overhead) {
			printk("OVERHEAD t_ms=%lld count=%u stale_now=%d\n", now,
			       lll_test_overhead_cnt, stale_reported ? 1 : 0);
			last_overhead = lll_test_overhead_cnt;
		}

		if (now - last_stats >= 10000) {
			print_stats("STATS");
			last_stats = now;
		}
	}
}

static void test_args_parse(int argc, char *argv[])
{
	for (int i = 0; i < argc; i++) {
		if (sscanf(argv[i], "lat_burst=%u", &lat_burst_us) == 1) {
			continue;
		}
		if (sscanf(argv[i], "lat_isr=%u", &lat_isr) == 1) {
			continue;
		}
		if (sscanf(argv[i], "lat_period=%u", &lat_period_us) == 1) {
			continue;
		}
		if (sscanf(argv[i], "clobber=%u", &clobber) == 1) {
			continue;
		}
		if (sscanf(argv[i], "load=%u", &load) == 1) {
			continue;
		}
		if (sscanf(argv[i], "yield=%u", &yield_en) == 1) {
			continue;
		}
	}
}

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

	/* DUT is CENTRAL on the split link: 7.5 ms, latency 30 (matches the dump). */
	param = BT_LE_CONN_PARAM(SPLIT_INTERVAL_UNITS, SPLIT_INTERVAL_UNITS,
				 CONN_LATENCY, CONN_TIMEOUT_UNITS);
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &split_conn);
	if (err) {
		FAIL("DUT: create split connection failed (err %d)\n", err);
	}
}

/* ---- split link yield (models a config's split_yield.c) ----------------
 *
 * yield=1: while the host is disconnected, move the split link to 28.75 ms,
 * latency 7; switch back 150 ms after the host connects. Runs on the system
 * work queue, queued from the same disconnected callback as the clobber.
 */
#define YIELD_INTERVAL_UNITS 23U
#define YIELD_LATENCY        7U
#define YIELD_SETTLE_MS      150

static bool want_yield;

static void yield_work_handler(struct k_work *work)
{
	struct bt_le_conn_param param = {
		.interval_min = want_yield ? YIELD_INTERVAL_UNITS : SPLIT_INTERVAL_UNITS,
		.interval_max = want_yield ? YIELD_INTERVAL_UNITS : SPLIT_INTERVAL_UNITS,
		.latency = want_yield ? YIELD_LATENCY : CONN_LATENCY,
		.timeout = CONN_TIMEOUT_UNITS,
	};
	int err;

	if (!split_conn) {
		return;
	}
	err = bt_conn_le_param_update(split_conn, &param);
	printk("YIELD %s t_ms=%lld err=%d\n", want_yield ? "slow" : "fast", k_uptime_get(),
	       err);
}

static K_WORK_DELAYABLE_DEFINE(yield_work, yield_work_handler);

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
			     uint16_t timeout)
{
	if (conn == split_conn) {
		printk("SPLIT_PARAMS t_ms=%lld interval=%u latency=%u\n", k_uptime_get(),
		       interval, latency);
	}
}

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);

	if (err && err != -EALREADY) {
		printk("DUT re-advertise failed (err %d)\n", err);
	}
}

static K_WORK_DEFINE(adv_work, adv_work_handler);

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
		printk("DUT connected to host (peripheral link) t_ms=%lld\n", k_uptime_get());
		if (yield_en) {
			want_yield = false;
			k_work_reschedule(&yield_work, K_MSEC(YIELD_SETTLE_MS));
		}
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("DUT disconnected (reason 0x%02x) t_ms=%lld\n", reason, k_uptime_get());
	if (conn == split_conn) {
		FAIL("DUT lost the split link (reason 0x%02x)\n", reason);
		bt_conn_unref(split_conn);
		split_conn = NULL;
	} else if (conn == host_conn) {
		bt_conn_unref(host_conn);
		host_conn = NULL;
		host_disconnects++;
		/* ZMK's split_central_disconnected() fires for every conn and
		 * queues the bad battery event to the system work queue.
		 */
		if (clobber) {
			k_work_submit(&clobber_work);
		}
		if (yield_en) {
			want_yield = true;
			k_work_reschedule(&yield_work, K_NO_WAIT);
		}
	}
}

static void recycled(void)
{
	if (!host_conn) {
		k_work_submit(&adv_work);
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled,
	.le_param_updated = le_param_updated,
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

	k_thread_create(&mon_thread, mon_stack, K_THREAD_STACK_SIZEOF(mon_stack),
			monitor, NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	printk("DUT config: clobber=%u load=%u yield=%u lat_burst=%u lat_period=%u\n",
	       clobber, load, yield_en, lat_burst_us, lat_period_us);

	if (lat_burst_us > 0) {
		k_thread_create(&lat_thread, lat_stack, K_THREAD_STACK_SIZEOF(lat_stack),
				lat_injector, NULL, NULL, NULL,
				K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	}

	while (split_conn) {
		sink_blast_one(split_conn, split_sink);
		if (!load) {
			k_sleep(K_MSEC(LIGHT_WRITE_MS));
		}
	}
}

static void dut_init(void)
{
	bst_ticker_set_next_tick_absolute(WAIT_TIME * 1e6);
	bst_result = In_progress;
}

static void dut_tick(bs_time_t HW_device_time)
{
	print_stats("FINAL");
	if (bst_result == In_progress) {
		PASS("DUT survived %i s\n", WAIT_TIME);
	}
}

static const struct bst_test_instance dut_tests[] = {
	{
		.test_id = "dut",
		.test_descr = "Dual-role DUT; optionally zeroes preempt_req on each "
			      "host disconnect (ZMK stray-write model).",
		.test_args_f = test_args_parse,
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
