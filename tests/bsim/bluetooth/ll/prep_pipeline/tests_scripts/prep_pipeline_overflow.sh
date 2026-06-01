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
sid="prep_pipe_s${seed}"

# Per-device crystal drift (-xo_drift, fraction; -40e-6 = -40ppm). bsim defaults
# to 0 = no relative drift = the two anchors never sweep = no collision = no bug.
# The host link anchor follows the HOST clock; the split link follows the DUT
# (it is central there). So drifting the HOST relative to the DUT makes the DUT's
# two events sweep into collision. 300ppm (vs the real ~40ppm) just sweeps faster.
HOST_DRIFT="${HOST_DRIFT:-300e-6}"
DUT_DRIFT="${DUT_DRIFT:-0}"

cd "${BSIM_OUT_PATH}/bin"

"${bin}" -v=${verbosity_level} -s="${sid}" -d=0 -RealEncryption=0 \
  -testid=split -rs=$((seed + 10)) > "${sid}_split.log" 2>&1 &
"${bin}" -v=${verbosity_level} -s="${sid}" -d=1 -RealEncryption=0 -xo_drift=${DUT_DRIFT} \
  -testid=dut   -rs=$((seed + 20)) > "${sid}_dut.log" 2>&1 &
"${bin}" -v=${verbosity_level} -s="${sid}" -d=2 -RealEncryption=0 -xo_drift=${HOST_DRIFT} \
  -testid=host  -rs=$((seed + 30)) > "${sid}_host.log" 2>&1 &

./bs_2G4_phy_v1 -v=${verbosity_level} -s="${sid}" -D=3 -sim_length="${sim_us}" \
  > "${sid}_phy.log" 2>&1

wait

if grep -qiE 'ASSERTION FAIL' "${sid}"_*.log 2>/dev/null; then
  echo "----- assert context (DUT) -----"
  grep -iE 'ASSERTION FAIL|lll\.c|\[next\]' "${sid}_dut.log" | head -5
  echo "RESULT: PREP_PIPELINE_OVERFLOW_REPRODUCED (seed=${seed})"
else
  echo "RESULT: NO_OVERFLOW (seed=${seed})"
fi
exit 0
