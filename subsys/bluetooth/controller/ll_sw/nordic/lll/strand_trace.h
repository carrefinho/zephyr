/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Opt-in event-by-event trace of the prepare pipeline, used to find what
 * produces the FIRST stranded prepare (zmk#3370). Enabled by
 * CONFIG_BT_CTLR_STRAND_TRACE (see overlay_trace.conf); compiles to nothing
 * otherwise.
 *
 * bsim charges zero simulated time for computation, so this instrumentation
 * does not perturb the timing it is measuring - unlike on hardware, where
 * printing from the radio ISR would itself create the latency under study.
 */
#ifndef LLL_STRAND_TRACE_H_
#define LLL_STRAND_TRACE_H_

#if defined(CONFIG_BT_CTLR_STRAND_TRACE)
#include <zephyr/sys/printk.h>
#define STRAND_TRACE(fmt, ...) printk("STRAND " fmt "\n", ##__VA_ARGS__)
#else
#define STRAND_TRACE(fmt, ...)
#endif

#endif /* LLL_STRAND_TRACE_H_ */
