#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# INFORMATIONAL probes below the ECV load gate: {500, 375} us with FSU under
# the same ZMK-split pointing load as conn_ecv_fsu_load.sh. The central records
# delivered rate / sequence gaps / link survival in an "ECV-LOAD RESULT" line
# and PASSes on any post-rate-change outcome -- the numbers, not a threshold,
# are the product (where does degradation start, and does the 80 us tIFS move
# it). These intervals sit below the previously measured empty-PDU floors
# (nRF54L ~625 us prepare-pipeline-bound; nRF52 needs a done-pool size this
# branch does not carry), so a run may even terminate in a controller assert:
# that outcome is itself the recorded floor datum. Wire this script's CI step
# with continue-on-error -- it must never gate the branch.

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

run_scenario conn_ecv_fsu_probe_500 central_ecv_fsu_probe_500
run_scenario conn_ecv_fsu_probe_375 central_ecv_fsu_probe_375
