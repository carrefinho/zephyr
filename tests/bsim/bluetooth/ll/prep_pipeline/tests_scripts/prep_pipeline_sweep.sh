#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Sweep the CPU-latency-injection matrix (burst x period) x seeds until the
# prepare-pipeline overflow reproduces. The June 2026 geometry-only sweeps
# (intervals/drift/phase, no injected latency) never overflowed - bsim charges
# zero CPU time, so the ULL/ISR latency real hardware has was missing entirely.
# The injector (see dut.c) irq-locks and burns simulated time periodically:
#   bursts span kernel-spinlock scale (~100 us), ISR/storm scale (~400-1500 us),
#   and internal-flash-stall scale (~6000 us; nRF52 page erase stalls the CPU
#   for tens of ms). Periods are chosen off-grid from the 7.5/8.75 ms intervals
#   so the bursts precess through every connection-event phase.
#
# Once a cell reproduces it is deterministic: re-run that single (seed, burst,
# period) via prep_pipeline_overflow.sh under gdb to inspect mfifo_fifo_prep.
#
# Env:  SEEDS (default "1 2"), BURSTS_US (default "100 400 1500 6000"),
#       PERIODS_US (default "3300 5700"), SIM_US (default 30e6)
# Exit: 0 and prints "SWEEP: reproduced ..." on first overflow; 1 if no cell
#       overflowed.

source "${ZEPHYR_BASE}/tests/bsim/sh_common.source"

seeds="${SEEDS:-1 2}"
bursts="${BURSTS_US:-100 400 1500 6000}"
periods="${PERIODS_US:-3300 5700}"
export SIM_US="${SIM_US:-30e6}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
clean=0
dead=0

for b in ${bursts}; do
  for p in ${periods}; do
    for s in ${seeds}; do
      echo "=== sweep burst=${b}us period=${p}us seed=${s} (sim ${SIM_US} us) ==="
      out="$(SEED="${s}" LAT_BURST_US="${b}" LAT_PERIOD_US="${p}" \
             "${here}/prep_pipeline_overflow.sh")"
      echo "${out}"
      if echo "${out}" | grep -q 'REPRODUCED'; then
        echo "SWEEP: reproduced at seed=${s} burst=${b} period=${p}"
        exit 0
      fi
      # Track POSITIVE completions separately. A cell that died in the harness
      # (segfault, link lost, simulation torn down early) is NOT evidence that
      # the controller survived, and must never be aggregated as "no overflow".
      if echo "${out}" | grep -q 'RESULT: NO_OVERFLOW'; then
        clean=$((clean + 1))
      else
        dead=$((dead + 1))
      fi
    done
  done
done

total=$((clean + dead))
if [ "${clean}" -eq 0 ]; then
  echo "SWEEP: NO VALID CELLS - ${dead}/${total} died in the harness; this sweep proves NOTHING"
  exit 2
fi
echo "SWEEP: no overflow across bursts [${bursts}] x periods [${periods}] x seeds [${seeds}]" \
     "(${clean}/${total} cells reached end-of-sim; ${dead} died in the harness)"
exit 1
