#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# RCV-tier Shorter Connection Intervals end-to-end: a central connects to a
# peripheral, exchanges feature page 1 (to learn the peer's SCI Host Support
# bit), then runs the Connection Rate Update procedure to move the link to a
# 1.25 ms connection interval. The central asserts the conn_rate_changed event
# fired with success, bt_conn_get_info reports interval_us == 1250, and the
# 1.25 ms link holds for 2 s without dropping (no instant desync / supervision
# timeout) - exercising the full HCI -> procedure -> instant-apply -> 0x37 event
# path on the SCI overlay build (CONFIG_BT_SHORTER_CONNECTION_INTERVALS=y).

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

simulation_id="conn_sci"
verbosity_level=2

cd ${BSIM_OUT_PATH}/bin

bsim_exe=./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_conn_subrate_prj_conf_overlay-sci_conf

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=0 -RealEncryption=0 \
  -testid=peripheral_plain -rs=23

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=1 -RealEncryption=0 \
  -testid=central_sci -rs=6

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} \
  -D=2 -sim_length=20e6 $@

wait_for_background_jobs
