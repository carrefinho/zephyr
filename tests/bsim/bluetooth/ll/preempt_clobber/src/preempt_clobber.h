/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared parameters for the preempt_req clobber test.
 *
 * Topology (mirrors a ZMK split CENTRAL, reused from the prep_pipeline repro):
 *
 *   host (central) --> DUT (peripheral + central) --7.5ms--> split (peripheral)
 *
 * A ZMK bug report says that every host disconnect on a ZMK split central with
 * CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING=y writes a 0 byte 234
 * bytes past peripheral_battery_levels[], and claims that on one build that
 * byte is the LLL's static preempt_req, which then stalls preempt timeouts
 * for ~256 s and trips LL_ASSERT_OVERHEAD. bsim cannot reproduce the memory
 * layout, so the DUT models the stray write directly: on each host disconnect
 * it sets preempt_req = 0 through a test hook (LLL_TEST_PREEMPT_HOOKS), from
 * the system work queue like the ZMK write. clobber=0 is the control.
 */
#ifndef PREEMPT_CLOBBER_H_
#define PREEMPT_CLOBBER_H_

/* Sim length; the run script's -sim_length must be slightly longer. Long
 * enough to see the claimed ~256 s recovery (2^23 ticks at 32768 Hz).
 */
#define WAIT_TIME 300 /* seconds */

#define SPLIT_INTERVAL_UNITS 6U  /* 7.5 ms (DUT is central here) */
/* Default host interval: 7 units (8.75 ms), coprime to the split's 6 so the
 * two links beat through every phase with zero drift. host_int=12 gives the
 * 15 ms a PC typically picks (2x the split interval).
 */
#define HOST_INTERVAL_UNITS  7U
#define CONN_LATENCY         0U
#define CONN_TIMEOUT_UNITS   400U /* 4 s (10 ms units) */

#define DUT_NAME   "bsim_dut"
#define SPLIT_NAME "bsim_split"

#define SINK_SVC_UUID \
	BT_UUID_128_ENCODE(0x9a7c0001, 0xbeef, 0x4d0e, 0x9e1a, 0xc0ffee0f100d)
#define SINK_CHR_UUID \
	BT_UUID_128_ENCODE(0x9a7c0002, 0xbeef, 0x4d0e, 0x9e1a, 0xc0ffee0f100d)

#define SINK_WRITE_LEN 20

/* load=0 ("light"): one write every LIGHT_WRITE_MS, roughly typing-rate HID
 * traffic. load=1 ("sat"): back-to-back writes, long overlapping events.
 */
#define LIGHT_WRITE_MS 10

#define SETTLE_DELAY_MS 500

#endif /* PREEMPT_CLOBBER_H_ */
