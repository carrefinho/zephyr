#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# RCV Shorter Connection Intervals coexistence (the ZMK split-keyboard scenario
# and the reduced-ce slot-reservation gate): the central holds two links at once
# -- one driven to a 1.25 ms interval via the Connection Rate Update procedure,
# the other left at 30 ms. The central asserts the 1.25 ms link keeps a fast
# cadence (far more events than a >=7.5 ms link could give) AND the 30 ms link is
# not starved by the 1.25 ms link's per-event slot reservation. If the 30 ms link
# is starved, the full-slot reservation is too coarse for coexistence and the
# reduced-ce reservation work is needed.

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

simulation_id="conn_sci_coex"
verbosity_level=2

cd ${BSIM_OUT_PATH}/bin

bsim_exe=./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_conn_subrate_prj_conf_overlay-sci_conf

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=0 -RealEncryption=0 \
  -testid=central_sci_coex -rs=6

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=1 -RealEncryption=0 \
  -testid=peripheral_plain -rs=23

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=2 -RealEncryption=0 \
  -testid=peripheral_plain -rs=39

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} \
  -D=3 -sim_length=40e6 $@

wait_for_background_jobs
