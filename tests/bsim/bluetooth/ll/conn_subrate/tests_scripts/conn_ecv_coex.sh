#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# ECV (Extended Connection Interval Values) coexistence under split load -- the
# under-split-load proof of the reduced-CE reservation. The central holds two
# links: one driven to a 625 us ECV interval (125 us grid, standard 150 us tIFS,
# reduced CE) via the Connection Rate Update procedure, the other left at 30 ms.
# It asserts the 625 us link keeps a fast cadence AND the 30 ms link is not
# starved by the sub-1.25 ms link's per-event slot reservation. This is the real
# test the eval calls for: a full-slot reservation at 625 us would over-reserve
# (~813 us into a 625 us window) and starve the co-resident link; the reduced-CE
# reservation (overhead-only) coexists. Same family as conn_sci_coex.sh (RCV).

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

simulation_id="conn_ecv_coex"
verbosity_level=2

cd ${BSIM_OUT_PATH}/bin

bsim_exe=./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_conn_subrate_prj_conf_overlay-sci_conf

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=0 -RealEncryption=0 \
  -testid=central_ecv_coex -rs=6

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=1 -RealEncryption=0 \
  -testid=peripheral_plain -rs=23

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=2 -RealEncryption=0 \
  -testid=peripheral_plain -rs=39

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} \
  -D=3 -sim_length=40e6 $@

wait_for_background_jobs
