#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# SCI instant-collision: at the same uptime the Central drives a Connection Rate
# Update (-> 1.25 ms, an instant procedure) while the Peripheral drives a
# Connection Parameters Request (-> Connection Update to 45 ms, also an instant).
# The Link Layer instant arbiter must serialise the two; a desync would drop the
# link within the 2 s supervision timeout. Both sides assert survival, and the
# Central additionally asserts the interval settled (two reads agree) -- so the
# new Connection Rate procedure composes with conn-update's instant/collision
# machinery without corrupting the shared anchor.

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

simulation_id="conn_sci_collision"
verbosity_level=2

cd ${BSIM_OUT_PATH}/bin

bsim_exe=./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_conn_subrate_prj_conf_overlay-sci_conf

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=0 -RealEncryption=0 \
  -testid=peripheral_sci_collision -rs=23

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=1 -RealEncryption=0 \
  -testid=central_sci_collision -rs=6

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} \
  -D=2 -sim_length=20e6 $@

wait_for_background_jobs
