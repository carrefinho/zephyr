#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0
#
# One run of the prepare-pipeline overflow repro (zmkfirmware/zmk#3370).
# Topology:  host(central) --7.5ms--> DUT(periph+central) --7.5ms--> split(periph)
# The DUT holds both links; both are saturated so their events overlap, driving
# the LL_SW preempt/duplicate-prepare path until the 7-slot (here shrunk to 3)
# prepare pipeline overflows -> LL_ASSERT(next) @ lll.c aborts the DUT.
#
# Env:
#   SEED      base random seed (default 1); per-device seeds derive from it
#   SIM_US    sim length in microseconds (default 60e6 = 60 s)
#   BIN_SUFFIX which build to run; default the accelerated overlay build.
#   LAT_BURST_US   DUT CPU-latency injector: irq-locked busy-burn per burst
#                  (default 0 = off). Models the spinlock/ISR/flash-stall
#                  latency real hardware has and zero-CPU-time bsim lacks.
#   LAT_PERIOD_US  sleep between bursts (default 3300)
#
# Always exits 0; prints exactly one RESULT line:
#   RESULT: PREP_PIPELINE_OVERFLOW_REPRODUCED (seed=N)   <- the assert fired
#   RESULT: NO_OVERFLOW (seed=N)                         <- survived the sim
# so the CI job can treat either outcome as expected (repro vs fix-validation).

source "${ZEPHYR_BASE}/tests/bsim/sh_common.source"

seed="${SEED:-1}"
sim_us="${SIM_US:-60e6}"
verbosity_level="${verbosity_level:-2}"
bin_suffix="${BIN_SUFFIX:-prj_conf_overlay_accel_conf}"
bin="./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_prep_pipeline_${bin_suffix}"
lat_burst="${LAT_BURST_US:-0}"
lat_period="${LAT_PERIOD_US:-3300}"
sid="prep_pipe_s${seed}_b${lat_burst}_p${lat_period}"

# Per-device crystal drift (-xo_drift, fraction; -40e-6 = -40ppm). The coprime
# 6/7 intervals already beat the events through collision, so this is just a
# small REALISTIC perturbation (+20ppm host vs DUT, within the ~+-40ppm a real
# crystal does) layered on top -- it nudges the phase the way silicon does
# without the supervision-timeout a large drift causes.
HOST_DRIFT="${HOST_DRIFT:-20e-6}"
DUT_DRIFT="${DUT_DRIFT:-0}"

cd "${BSIM_OUT_PATH}/bin"

# LAT_TARGET selects WHICH device gets the injector (default dut). The
# single-link controls (split = peripheral-only, host = central-only) run the
# identical injector on a device with exactly ONE connection: if the overflow
# requires a dual-role device, those can never reproduce it.
lat_target="${LAT_TARGET:-dut}"
inj_split=""; inj_dut=""; inj_host=""
inj_args="lat_burst=${lat_burst} lat_period=${lat_period}"
[ -n "${LAT_ISR:-}" ] && inj_args="${inj_args} lat_isr=${LAT_ISR}"
case "${lat_target}" in
  dut)   inj_dut="-argstest ${inj_args}" ;;
  split) inj_split="-argstest ${inj_args}" ;;
  host)  inj_host="-argstest ${inj_args}" ;;
  *) echo "unknown LAT_TARGET=${lat_target}" >&2; exit 2 ;;
esac

"${bin}" -v=${verbosity_level} -s="${sid}" -d=0 -RealEncryption=0 \
  -testid=split -rs=$((seed + 10)) ${inj_split} > "${sid}_split.log" 2>&1 &
"${bin}" -v=${verbosity_level} -s="${sid}" -d=1 -RealEncryption=0 -xo_drift=${DUT_DRIFT} \
  -testid=dut   -rs=$((seed + 20)) ${inj_dut} > "${sid}_dut.log" 2>&1 &
"${bin}" -v=${verbosity_level} -s="${sid}" -d=2 -RealEncryption=0 -xo_drift=${HOST_DRIFT} \
  -testid=host  -rs=$((seed + 30)) ${inj_host} > "${sid}_host.log" 2>&1 &

./bs_2G4_phy_v1 -v=${verbosity_level} -s="${sid}" -D=3 -sim_length="${sim_us}" \
  > "${sid}_phy.log" 2>&1

wait

# Discriminate the asserts: the zmk#3370 field crash is specifically
# LL_ASSERT(next) in lll.c (prepare-pipeline enqueue overflow). Other
# controller asserts (e.g. the EVENT_OVERHEAD_START family in
# lll_central.c/lll_peripheral.c, the zmk#3331 sibling) are a DIFFERENT
# failure and must not count as this repro.
#
# A NO_OVERFLOW verdict additionally requires POSITIVE evidence that the DUT
# survived to the end of the simulation: the bstest ticker prints a PASSED /
# NOT PASSED line at sim end. A DUT that dies any other way (segfault of the
# native binary - seen on 3.5-3.7-era builds ~30 ms into the blast - or any
# silent termination) produces neither an assert nor the end marker, and MUST
# NOT be counted as a clean pass.
# Which device is expected to fail is the one being injected into: for the
# single-link controls the DUT is NOT under stress, so its survival says
# nothing - the control's question is whether SPLIT/HOST itself overflows.
victim="${sid}_${lat_target}.log"
cell="seed=${seed} burst=${lat_burst} period=${lat_period} target=${lat_target}"

if grep -qE 'ASSERTION FAIL \[next\] @ .*lll\.c' "${victim}" 2>/dev/null; then
  echo "----- assert context (${lat_target}) -----"
  grep -iE 'ASSERTION FAIL|lll\.c|\[next\]' "${victim}" | head -5
  echo "RESULT: PREP_PIPELINE_OVERFLOW_REPRODUCED (${cell})"
elif grep -qE 'ASSERTION FAIL \[next\] @ .*lll\.c' "${sid}"_*.log 2>/dev/null; then
  # The pipeline overflowed, but on a device we were NOT injecting into.
  echo "----- [next] assert on a non-injected device -----"
  grep -lE 'ASSERTION FAIL \[next\]' "${sid}"_*.log
  echo "RESULT: OVERFLOW_ELSEWHERE (${cell})"
elif grep -qiE 'ASSERTION FAIL' "${sid}"_*.log 2>/dev/null; then
  echo "----- other assert -----"
  grep -iE 'ASSERTION FAIL' "${sid}"_*.log | head -3
  echo "RESULT: OTHER_ASSERT (${cell})"
elif grep -qE 'TESTCASE (NOT )?PASSED at exit|PASSED at' "${victim}" 2>/dev/null; then
  echo "RESULT: NO_OVERFLOW (${cell})"
elif [ "${lat_target}" = "split" ] && \
     awk '/SPLIT alive t=/ { t = $0; sub(/.*SPLIT alive t=/, "", t); sub(/[^0-9].*/, "", t); if (t + 0 >= 20) found = 1 } END { exit !found }' "${victim}" 2>/dev/null; then
  # The split device never gets the framework's end-of-sim marker in its log,
  # so its survival is evidenced by its own heartbeat instead: reaching t>=20 s
  # of a 30 s sim means the controller ran the whole blast without asserting.
  echo "----- split heartbeat tail -----"
  grep 'SPLIT alive' "${victim}" | tail -2
  echo "RESULT: NO_OVERFLOW (${cell})"
else
  echo "----- ${lat_target} log tail (no assert, no end-of-sim marker) -----"
  tail -3 "${victim}" 2>/dev/null
  echo "RESULT: HARNESS_DEATH (${cell})"
fi
exit 0
