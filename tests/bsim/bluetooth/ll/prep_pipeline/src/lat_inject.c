/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * CPU-latency injector, shared by all three devices in this test.
 *
 * bsim executes all CPU work in ZERO simulated time, so the ULL/ISR latency a
 * real ZMK central experiences (kernel spinlock critical sections, SPI display
 * + kscan ISRs, and internal-flash writes/erases that stall the whole CPU for
 * 41 us .. 85 ms on nRF52) never occurs, and the controller's prepares are
 * always enqueued, preempted, and dequeued exactly on time. The June 2026
 * sweeps varied only link GEOMETRY (intervals/drift/phase) and never
 * overflowed.
 *
 * This thread periodically masks interrupts and burns simulated time, delaying
 * the radio/ticker ISRs the way real critical sections and flash stalls do:
 *   lat_burst  = busy-wait length in us (0 = injector disabled)
 *   lat_period = sleep between bursts in us
 * Passed per-run via bsim test args: -argstest lat_burst=N lat_period=N
 *
 * Injection mode (-argstest lat_isr=):
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
#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>

#include "lat_inject.h"

static uint32_t lat_burst_us;
static uint32_t lat_period_us = 3300;
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

void lat_inject_args_parse(int argc, char *argv[])
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
	}
}

void lat_inject_start(const char *who)
{
	if (lat_burst_us == 0) {
		return;
	}

	k_thread_create(&lat_thread, lat_stack, K_THREAD_STACK_SIZEOF(lat_stack),
			lat_injector, NULL, NULL, NULL,
			K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	printk("%s latency injector (mode %u): %u us burst every %u us\n",
	       who, lat_isr, lat_burst_us, lat_period_us);
}
