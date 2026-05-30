#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# One-way notification latency on an SCI link. The Central drives the link to a
# 1.25 ms RCV interval, subscribes to a custom characteristic the Peripheral
# notifies with its send timestamp (uint32 us), and measures recv_us - sent_us
# per notification -- the faithful HID-input direction (peripheral -> central),
# not the round-trip GATT read the other tests use. The measurement is valid
# because BabbleSim runs both devices on one global simulated clock, so the two
# k_uptime values are directly comparable (the same property the collision test
# relies on). The Central reports min/avg/max and asserts the one-way figure
# stays well under the round-trip (a loose 8 ms ceiling).

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

simulation_id="conn_sci_latency"
verbosity_level=2

cd ${BSIM_OUT_PATH}/bin

bsim_exe=./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_conn_subrate_prj_conf_overlay-sci_conf

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=0 -RealEncryption=0 \
  -testid=peripheral_sci_latency -rs=23

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=1 -RealEncryption=0 \
  -testid=central_sci_latency -rs=6

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} \
  -D=2 -sim_length=20e6 $@

wait_for_background_jobs
