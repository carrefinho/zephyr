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

/* Collision test: both sides request a subrate change at the same time. */
#define SUBRATE_COLL_INITIAL 4U /* peripheral negotiates this first */
#define SUBRATE_COLL_CENTRAL 8U /* central's colliding request */
#define SUBRATE_COLL_PERIPH  2U /* peripheral's colliding request */
#define COLLISION_TIME_MS    3000 /* both fire at this uptime */

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
