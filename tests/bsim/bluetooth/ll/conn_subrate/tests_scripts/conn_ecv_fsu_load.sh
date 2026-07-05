#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# ECV floor under a realistic ZMK-split pointing load -- the asserting half of
# the interval x FSU matrix. The peripheral streams one 8-byte input-event
# notification per connection event (LL PDU 17 B unencrypted, 2M PHY); the
# central drives the interval via the Connection Rate procedure and measures
# delivered rate + sequence continuity over a multi-second soak.
#
#   GATE (assert >= 90% delivered, zero gaps, survival):
#     750 us + FSU (80 us tIFS)
#     625 us + FSU (80 us tIFS)
#   CONTROL (assert survival; rate is the airtime-vs-scheduler DATA):
#     750 us at the standard 150 us tIFS
#     625 us at the standard 150 us tIFS
#
# Compare each control's "ECV-LOAD RESULT" line against the FSU gate cell at
# the same interval: if a cell only meets its rate with FSU, that interval is
# AIRTIME-bound; if the FSU and no-FSU rates match, the wall (wherever it is)
# is SCHEDULER-bound. The sub-gate {500, 375} us probes live in
# conn_ecv_fsu_load_probe.sh (informational, continue-on-error in CI).

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

verbosity_level=2
bin=./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_conn_subrate_prj_conf_overlay-ecv-load_conf

cd ${BSIM_OUT_PATH}/bin

run_scenario() {
  local sim_id=$1
  local central_testid=$2

  Execute ${bin} -v=${verbosity_level} -s=${sim_id} -d=0 -RealEncryption=0 \
    -testid=peripheral_zmk_load -rs=23
  Execute ${bin} -v=${verbosity_level} -s=${sim_id} -d=1 -RealEncryption=0 \
    -testid=${central_testid} -rs=6
  Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${sim_id} -D=2 -sim_length=20e6

  wait_for_background_jobs
}

run_scenario conn_ecv_fsu_load_750    central_ecv_fsu_load_750
run_scenario conn_ecv_fsu_load_625    central_ecv_fsu_load_625
run_scenario conn_ecv_load_750_nofsu  central_ecv_load_750_nofsu
run_scenario conn_ecv_load_625_nofsu  central_ecv_load_625_nofsu
