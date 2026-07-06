#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# DUAL-TIMER sub-gate probes: the dual-timer (CONFIG_BT_CTLR_SW_SWITCH_SINGLE_-
# TIMER=n, EVENT_TIMER = NRF_TIMER00) counterpart to conn_ecv_fsu_load_probe.sh.
# Same {500, 375} us informational probes under the same ZMK-split pointing load
# with FSU, but against the overlay-ecv-load-dt.conf build. The delivered-rate /
# sequence-gap / survival numbers in the "ECV-LOAD RESULT" line are the product:
# does the dual-timer architecture (HAL_RADIO_ISR_LATENCY_MAX_US = 0 instead of
# 80) move the sub-floor degradation the nRF54L single-timer arm records at
# {500, 375} us (single-timer: 100% @ 500 us, 50% @ 375 us as cadence halves)?
#
# These intervals sit below any measured floor, so a run may terminate in a
# controller assert -- that outcome is itself the recorded datum. Distinct sim
# ids (suffix _dt) avoid colliding with the single-timer probe arm. Wire the CI
# step continue-on-error; it must never gate the branch.

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

run_scenario conn_ecv_fsu_probe_500_dt central_ecv_fsu_probe_500
run_scenario conn_ecv_fsu_probe_375_dt central_ecv_fsu_probe_375
