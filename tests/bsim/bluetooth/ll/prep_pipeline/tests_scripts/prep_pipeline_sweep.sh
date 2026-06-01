#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Sweep random seeds until the prepare-pipeline overflow reproduces. Each seed
# changes how the two links' anchors land relative to each other, so a sweep
# finds an overlap pattern that overflows; once found it is deterministic
# (re-run that single seed under gdb to inspect mfifo_fifo_prep).
#
# Env:  SEEDS (default "1..12"),  SIM_US (default 30e6)
# Exit: 0 and prints "SWEEP: reproduced at seed N" on first overflow;
#       1 if no seed overflowed (try more seeds / longer SIM_US / smaller
#       EVENT_PIPELINE_MAX / more saturation).

source "${ZEPHYR_BASE}/tests/bsim/sh_common.source"

# Coprime intervals beat the events through collision continuously (~every
# 52 ms), so a repro no longer depends on luck or a long soak; a few seeds
# (varying the initial phase) over a short sim suffice.
seeds="${SEEDS:-1 2 3 4 5}"
export SIM_US="${SIM_US:-30e6}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for s in ${seeds}; do
  echo "=== sweep seed ${s} (sim ${SIM_US} us) ==="
  out="$(SEED="${s}" "${here}/prep_pipeline_overflow.sh")"
  echo "${out}"
  if echo "${out}" | grep -q 'REPRODUCED'; then
    echo "SWEEP: reproduced at seed ${s}"
    exit 0
  fi
done

echo "SWEEP: no overflow across seeds [${seeds}]"
exit 1
