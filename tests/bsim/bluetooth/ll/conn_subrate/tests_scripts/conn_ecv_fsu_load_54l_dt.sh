#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# DUAL-TIMER A/B arm of the nRF54L ECV split-load floor matrix. Identical in
# every respect to conn_ecv_fsu_load_54l.sh -- same load (one 8-byte input-event
# notification per CE, 17 B LL PDU unencrypted, 2M PHY), same {750, 625} us
# points with and without FSU -- EXCEPT it runs the controller built from
# overlay-ecv-load-dt.conf, i.e. with CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER=n
# (EVENT_TIMER = NRF_TIMER00, dual-timer, HAL_RADIO_ISR_LATENCY_MAX_US = 0),
# backported from upstream PR #109528.
#
# The question this arm answers: does moving the nRF54L off the single-TIMER10
# architecture (per-event 80 us overhead inside the abort/overhead threshold)
# onto the nRF52840-style dual-timer architecture move the measured ECV floor
# (single-timer baseline: 93% @ 750 us, 89% @ 625 us with FSU)? Compare this
# script's delivered-rate lines directly against conn_ecv_fsu_load_54l.sh.
#
# Distinct simulation ids (suffix _dt) so both A/B arms can run in the same CI
# job without colliding on the bsim shared-memory sim directory. The whole arm
# is wired continue-on-error in CI: it is an experimental backport and must
# never gate the branch.

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

verbosity_level=2
bin=./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_conn_subrate_prj_conf_overlay-ecv-load-dt_conf

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

run_scenario conn_ecv_fsu_load54_750_dt  central_ecv_fsu_load_750
run_scenario conn_ecv_fsu_load54_625r_dt central_ecv_fsu_load_625_rec
run_scenario conn_ecv_load54_750_nof_dt  central_ecv_load_750_nofsu
run_scenario conn_ecv_load54_625_nof_dt  central_ecv_load_625_nofsu
