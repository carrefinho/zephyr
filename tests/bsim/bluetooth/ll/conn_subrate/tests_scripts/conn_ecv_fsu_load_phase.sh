#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# Phase-sensitivity sweep of the ECV split-load cells.
#
# The loaded matrix is fully deterministic per seed pair (reproduced
# byte-identical across three CI runs), and its interval response is
# non-monotonic on nRF54L (100% delivered at 500 us vs 89% at 625 us and 93%
# at 750 us) -- the signature of a phase beat between the peripheral's
# free-running k_timer load generator and the connection-event anchor, not of
# a per-event capability cliff. This sweep re-runs the non-asserting cells at
# two additional bsim seed pairs; different seeds shift the advertising /
# connection-establishment timing and therefore the generator-vs-anchor phase.
#
# Read the per-cell "ECV-LOAD RESULT" lines across THREE samples per cell
# (the main matrix step runs the baseline pair 23/6; this script adds 101/42
# and 7/77): the min-max spread per cell is the phase contamination. A wide
# spread at 625/750 us with a stable 500 us means the interval ranking is
# phase-noise; a tight spread everywhere means the numbers are real per-event
# behavior and the beat hypothesis is wrong.
#
# Only survival-asserting testids are used (recorded/control/probe modes), so
# a bad phase cannot fail CI on a rate; the RESULT lines are the product.

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

verbosity_level=2
bin=./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_conn_subrate_prj_conf_overlay-ecv-load_conf

cd ${BSIM_OUT_PATH}/bin

run_cell() {
  local sim_id=$1
  local central_testid=$2
  local rs_periph=$3
  local rs_central=$4

  Execute ${bin} -v=${verbosity_level} -s=${sim_id} -d=0 -RealEncryption=0 \
    -testid=peripheral_zmk_load -rs=${rs_periph}
  Execute ${bin} -v=${verbosity_level} -s=${sim_id} -d=1 -RealEncryption=0 \
    -testid=${central_testid} -rs=${rs_central}
  Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${sim_id} -D=2 -sim_length=20e6

  wait_for_background_jobs
}

for pair in "101 42" "7 77"; do
  set -- ${pair}
  rs_p=$1
  rs_c=$2

  run_cell conn_ecv_ph_${rs_p}_625fsu   central_ecv_fsu_load_625_rec  ${rs_p} ${rs_c}
  run_cell conn_ecv_ph_${rs_p}_750nof   central_ecv_load_750_nofsu    ${rs_p} ${rs_c}
  run_cell conn_ecv_ph_${rs_p}_625nof   central_ecv_load_625_nofsu    ${rs_p} ${rs_c}
  run_cell conn_ecv_ph_${rs_p}_500probe central_ecv_fsu_probe_500     ${rs_p} ${rs_c}
  run_cell conn_ecv_ph_${rs_p}_375probe central_ecv_fsu_probe_375     ${rs_p} ${rs_c}
done
