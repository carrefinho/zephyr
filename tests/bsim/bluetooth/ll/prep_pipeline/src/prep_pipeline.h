/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared parameters for the prepare-pipeline-overflow repro.
 *
 * Topology (mirrors a ZMK split CENTRAL):
 *
 *   host (central) --7.5ms--> DUT (peripheral + central) --7.5ms--> split (peripheral)
 *
 * The DUT holds TWO simultaneous links on one radio - peripheral to `host`,
 * central to `split` - exactly like a ZMK central (BLE peripheral to the
 * computer, BLE central to its split half). Both links run at 7.5 ms with
 * max-size write-without-response traffic so each connection event is long
 * enough that the two events cannot coexist in one 7.5 ms interval without
 * overlapping. That forces the LL_SW preempt/duplicate path every event, which
 * (pre-fix) mis-enqueues duplicate prepares of the same conn into the 7-slot
 * `prep` MFIFO until ull_prepare_enqueue() returns NULL and LL_ASSERT(next)
 * (lll.c:891) trips: a K_ERR_KERNEL_OOPS, the crash seen in zmkfirmware/zmk#3370.
 */
#ifndef PREP_PIPELINE_H_
#define PREP_PIPELINE_H_

/* Real parameters read straight out of the zmk#3370 coredump (lll_conn structs):
 *   host link  (DUT=peripheral, to the computer): 15 ms, latency 30
 *   split link (DUT=central, to the peripheral):  7.5 ms, latency 30
 * Different intervals (2:1) is how it actually runs. The collision DRIVER is the
 * ~+-40 ppm crystal-drift difference between the keyboard's clock (which drives
 * the split link, DUT=central) and the host's clock (which drives the host link,
 * DUT=peripheral): over ~70 s the two anchors sweep into periodic collision, the
 * controller defers events, and the buggy dequeue leaks duplicate prepares.
 * bsim has ZERO drift by default (xo_drift=0), so the launch script must set a
 * per-device -xo_drift to reproduce the sweep (exaggerated to collide fast).
 * Auto param update is disabled (prj.conf) so the intervals stay put.
 */
/* The real intervals (split 6, host 12 or 9) are harmonic/semi-harmonic, so on
 * silicon they only sweep into collision via crystal drift. In bsim all clocks
 * are ideal (xo_drift=0), so harmonic intervals never collide, and forcing the
 * sweep with a large xo_drift instead breaks the link (the peripheral can't
 * track the drifted anchor -> supervision timeout). Instead pick COPRIME
 * intervals: 6 and 7 share no factor, so the host event beats through every
 * phase relative to the split events (~every 42 units = 52.5 ms) with ZERO
 * drift and rock-stable links -- reproducing the same different-rate-collision
 * mechanism the dumps show, deterministically. Latency 0: under continuous
 * traffic the real links don't skip either (latency_event=0 in the dump). */
#define SPLIT_INTERVAL_UNITS 6U   /* 7.5 ms   (DUT is central here) */
#define HOST_INTERVAL_UNITS  7U   /* 8.75 ms  (DUT is peripheral here; coprime to 6) */
#define CONN_LATENCY         0U
#define CONN_TIMEOUT_UNITS   400U /* 4 s (10 ms units) */

/* Advertised names, so each scanner connects to the right peer. */
#define DUT_NAME   "bsim_dut"
#define SPLIT_NAME "bsim_split"

/* A single write-without-response sink characteristic. Every device exposes it
 * (server side); clients (host->DUT, DUT->split) discover its value handle and
 * hammer it with max-size writes to lengthen connection events.
 */
#define SINK_SVC_UUID \
	BT_UUID_128_ENCODE(0x9a7c0001, 0xbeef, 0x4d0e, 0x9e1a, 0xc0ffee0f100d)
#define SINK_CHR_UUID \
	BT_UUID_128_ENCODE(0x9a7c0002, 0xbeef, 0x4d0e, 0x9e1a, 0xc0ffee0f100d)

/* Write-without-response payload. Kept at the default ATT MTU max (23 - 3 = 20)
 * so no MTU exchange is needed; the saturating client loop instead queues MANY
 * such writes per event (backed by the large tx/rx buffer pools in prj.conf), so
 * each 7.5 ms event packs enough PDUs to run long and overlap the other link. */
#define SINK_WRITE_LEN 20

#define SETTLE_DELAY_MS 500

#endif /* PREP_PIPELINE_H_ */
