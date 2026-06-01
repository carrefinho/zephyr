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

/* Both links pinned to 7.5 ms (6 * 1.25 ms), no slave latency, 4 s supervision.
 * Auto param update is disabled (prj.conf) so the intervals stay put and the two
 * events keep beating against each other.
 */
#define CONN_INTERVAL_UNITS 6U   /* 7.5 ms */
#define CONN_LATENCY        0U
#define CONN_TIMEOUT_UNITS  400U /* 4 s (10 ms units) */

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
