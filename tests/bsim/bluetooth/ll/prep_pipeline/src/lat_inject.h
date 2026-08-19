/*
 * Copyright 2026 The ZMK Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared CPU-latency injector. Extracted from dut.c so the single-link
 * devices (split, host) can run the identical injection, which is the
 * control for "does the prepare-pipeline overflow require a dual-role
 * device?".
 */
#ifndef PREP_PIPELINE_LAT_INJECT_H_
#define PREP_PIPELINE_LAT_INJECT_H_

/* Parse -argstest lat_burst= / lat_period= / lat_isr= */
void lat_inject_args_parse(int argc, char *argv[]);

/* Start the injector thread if lat_burst > 0. Call once, after the device's
 * links are up, so connection setup runs clean.  `who` is used for logging.
 */
void lat_inject_start(const char *who);

#endif /* PREP_PIPELINE_LAT_INJECT_H_ */
