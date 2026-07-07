#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# nRF54L variant of the ECV split-load floor matrix (conn_ecv_fsu_load.sh is
# the nRF52 matrix). Same load: one 8-byte input-event notification per
# connection event (17 B LL PDU unencrypted, 2M PHY).
#
# History: the 2026-07-05 runs read 93% @ 750 us and 89% @ 625 us and briefly
# demoted 625 us to a recorded cell as a "single-timer drain deficit" -- that
# was an ACCOUNTING ARTIFACT: this board's 100 us GRTC tick makes k_timer
# round a 625/750 us load period up to 700/800 us, so the generator offered
# less than interval-nominal and the controller had in fact delivered 100% of
# the offered load (seed/phase sweep confirmed, and 500 us exact-tick ran at
# a perfect 100%). Verdicts now gate on delivered-vs-OFFERED, and 625 us is a
# GATE cell again. The genuine limit under this load is 375 us, where the CE
# cadence itself halves. Hence on this SoC:
#
#   GATE (assert >= 90% of the OFFERED load, zero gaps, survival):
#     750 us + FSU (80 us tIFS)
#     625 us + FSU (80 us tIFS)
#   CONTROL (assert survival; airtime-vs-scheduler DATA):
#     750 us at the standard 150 us tIFS
#     625 us at the standard 150 us tIFS
#
# The sub-gate {500, 375} us probes live in conn_ecv_fsu_load_probe.sh.

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

run_scenario conn_ecv_fsu_load54_750  central_ecv_fsu_load_750
run_scenario conn_ecv_fsu_load54_625  central_ecv_fsu_load_625
run_scenario conn_ecv_load54_750_nof  central_ecv_load_750_nofsu
run_scenario conn_ecv_load54_625_nof  central_ecv_load_625_nofsu
