/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared parameters for the conn_subrate bsim test.
 */
#ifndef CONN_SUBRATE_H_
#define CONN_SUBRATE_H_

/* Fixed connection parameters used by the central when creating the link, so
 * the subrate skip cadence is deterministic. The interval is set at connection
 * setup (no separate update procedure) to avoid resetting subrating.
 */
#define CONN_INTERVAL_UNITS 24U  /* 30 ms (1.25 ms units) */
#define CONN_TIMEOUT_UNITS  200U /* 2 s (10 ms units) */

/* ECV scheduling feasibility spike (central_ecv_sweep): push link 0 below
 * 1.25 ms via the low-latency reduced-reservation path and sweep the sub-1.25 ms
 * band while a second link stays full-rate at 30 ms. The on-air interval is
 * (units + 1) * CONN_LOW_LAT_INT_UNIT_US; this spike branch sets that controller
 * constant (subsys/.../ll_sw/lll.h) to 125 us so the sweep lands on the exact
 * ECV grid. ECV_LOWLAT_UNIT_US MUST stay in sync with that lll.h value.
 */
#define ECV_LOWLAT_UNIT_US      125U
#define ECV_US_TO_LL_UNITS(_us) ((uint16_t)((_us) / ECV_LOWLAT_UNIT_US - 1U))
/* The go/no-go gate band, swept descending. Low-lat engages only for units <
 * BT_HCI_LE_INTERVAL_MIN (6) => <= 750 us at this 125 us unit (higher escapes to
 * the 1250 us grid). Capped at 625 us: that is the measured nRF54L floor and
 * going lower hard-asserts there.
 *
 * MEASURED FLOORS (bsim, 30 ms-coex split; run 26654390506 @ default pool 4 and
 * run 26668227699 @ BT_CTLR_EVENT_DONE_MAX=12):
 *   nRF54L (single-timer, primary): holds 750/625 us @ 95/93%. Below ~625 us it
 *     is SCHEDULING/PIPELINE-bound, NOT pool-bound -- raising the done-extra pool
 *     only moved the assert from done-extra exhaustion (lll_conn.c) to the prepare
 *     pipeline (lll.c:892, EVENT_PIPELINE_MAX=7). ~625 us is the practical floor.
 *   nRF52 (dual-timer): WAS pool-bound -- with the bigger pool it holds gracefully
 *     to 375 us @ 95% (the spec ECV floor) and only drops at 250 us (supervision
 *     timeout, below the radio exchange).
 * The gate stays the 750 us safe floor holding; 625 us is the stretch, validated. */
#define ECV_SWEEP_US_LIST       { 750U, 625U }
#define ECV_COUNT_WINDOW_MS     2000  /* per-interval cadence measurement window */
#define ECV_SETTLE_MS           1000  /* let each interval update take effect on air */
#define ECV_HOLD_PCT            75U   /* >= this cadence == anchor held at this interval */
#define ECV_GATE_PCT_750        75U   /* go/no-go gate: the safe ECV floor must hold */

/* Peripheral-initiated Connection Subrate Request parameters (5.1.20). A range
 * is requested so the central can grant the largest factor it accepts.
 */
#define SUBRATE_REQ_MIN          2U
#define SUBRATE_REQ_MAX          8U
#define SUBRATE_REQ_MAX_LATENCY  0U
#define SUBRATE_REQ_CONT_NUMBER  0U
#define SUBRATE_REQ_TIMEOUT      CONN_TIMEOUT_UNITS

/* Central acceptable subrate defaults (must allow the request above). */
#define SUBRATE_ACC_MAX 8U

/* M->N transition test: the Peripheral first negotiates SUBRATE_MTON_M, then the
 * Central re-negotiates to SUBRATE_MTON_N (both > 1 -> a true M->N transition).
 */
#define SUBRATE_MTON_M 4U
#define SUBRATE_MTON_N 8U

/* Non-power-of-2 factor: the subrated-event phase math must be correct when the
 * factor does not divide the 16-bit event counter (an unsigned (event - base) %
 * factor mis-anchors for these). Reuses the basic `central` skip probe.
 */
#define SUBRATE_ODD_FACTOR 5U

/* Supervision-timeout boundary: the largest factor the HCI bound allows here
 * (interval[1.25ms]*factor*(lat+1) < 2x supervision: 24*33 = 792 < 800). Its
 * skip period (~990 ms) is just under half the 2 s supervision timeout, so an
 * idle link survives only if the peripheral keeps listening on subrated events.
 */
#define SUBRATE_SUPERVISION_FACTOR 33U

/* conn-update test: the Central changes the interval while subrated, which must
 * reset subrating to factor 1. A different value from CONN_INTERVAL_UNITS so the
 * interval actually changes.
 */
#define CONN_UPDATE_INTERVAL_UNITS 36U /* 45 ms */

/* Continuation-events test: factor with a large continuation_number so that,
 * after data activity, both sides stay present for ~the whole subrate cycle.
 * Close-spaced reads (< CONT_WINDOW) stay fast; the first idle read is slow.
 */
#define SUBRATE_CONT_FACTOR 8U
#define SUBRATE_CONT_CN     7U  /* factor - 1 */
#define CONT_READ_GAP_MS    50  /* < CN*interval (210 ms) -> within window */
#define CONT_IDLE_MS        600 /* > skip period -> peripheral re-skips first */

/* Peripheral-latency test: the Peripheral stacks max_latency on top of the
 * factor, skipping factor*(latency+1) events while the Central stays on every
 * factor-th subrated event; they still rendezvous.
 */
#define SUBRATE_LAT_FACTOR  4U
#define SUBRATE_LAT_LATENCY 2U  /* effective skip = 4*(2+1) = 12 events */

/* Central-cadence regression lock (reuses peripheral_latency). With the
 * Peripheral coasting on its stacked max_latency, the Central must stay present
 * only on its own subrated events (every factor). A controller that breaks
 * latency on the Peripheral's expected skipped events collapses the Central to
 * ~full rate; counting the Central's events over an idle window catches that.
 * Window spans several stacked cycles (factor*(latency+1)*interval = 360 ms).
 */
#define LAT_COUNT_WINDOW_MS 3000

/* Collision test: both sides request a subrate change at the same time. */
#define SUBRATE_COLL_INITIAL 4U /* peripheral negotiates this first */
#define SUBRATE_COLL_CENTRAL 8U /* central's colliding request */
#define SUBRATE_COLL_PERIPH  2U /* peripheral's colliding request */
#define COLLISION_TIME_MS    3000 /* both fire at this uptime */

/* Central->Peripheral continuation test: a write-without-response characteristic
 * the Central bursts data to. Write-without-response is one-way (no GATT
 * response), so only the Central's own Tx can open its continuation window -
 * this isolates that path. With it working the burst arrives at ~the event
 * rate; otherwise it is throttled to the subrate cadence (factor x fewer).
 */
#define CWRITE_SVC_UUID \
	BT_UUID_128_ENCODE(0x5ab12701, 0x1234, 0x4c0d, 0x9e1a, 0xc0ffee000001)
#define CWRITE_CHR_UUID \
	BT_UUID_128_ENCODE(0x5ab12702, 0x1234, 0x4c0d, 0x9e1a, 0xc0ffee000002)

/* Notifications-under-subrating: notify slower than the skip period (factor *
 * interval = 240 ms at factor 8) so each notification is queued during a skip
 * and must wake the peripheral on a subrated event to be delivered - rather
 * than a fast stream that keeps the link awake and never really subrates.
 */
#define SUBRATE_NOTIFY_PERIOD_MS 400

/* Time the peripheral waits after connecting before requesting subrating, to
 * let feature exchange and the central's discovery complete first.
 */
#define SETTLE_DELAY_MS 1000

/* Central read probe: number of reads and the idle gap between them. The gap
 * must exceed the subrate skip period (factor * interval) so the peripheral has
 * gone back to sleep before each probe.
 */
#define NUM_READS    8
#define READ_GAP_MS  400

#endif /* CONN_SUBRATE_H_ */
