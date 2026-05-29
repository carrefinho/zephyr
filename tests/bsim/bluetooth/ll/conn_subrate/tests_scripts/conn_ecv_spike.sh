#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# ECV scheduling feasibility spike (the sub-1.25 ms go/no-go): a Central holds
# two links - one swept down the sub-1.25 ms band {1000,875,750,625,500} us via
# the low-latency reduced-reservation path, one full-rate at 30 ms (the split
# topology). The central_ecv_sweep test prints per-interval cadence and gates on
# the 1 ms (sanity) and 750 us (safe ECV floor) anchors holding under the
# concurrent full-rate link - the preempt-overhead failure mode that parked the
# CIS-alongside-split work. BOARD_TS selects nrf52 vs nrf54l15 (the latter is the
# primary single-timer target). Built with overlay-ecv-spike.conf.

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

simulation_id="conn_ecv_spike"
verbosity_level=2

cd ${BSIM_OUT_PATH}/bin

bsim_exe=./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_conn_subrate_prj_conf_overlay-ecv-spike_conf

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=0 -RealEncryption=0 \
  -testid=peripheral_plain -rs=23

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=1 -RealEncryption=0 \
  -testid=peripheral_plain -rs=44

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=2 -RealEncryption=0 \
  -testid=central_ecv_sweep -rs=6

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} \
  -D=3 -sim_length=40e6 $@

wait_for_background_jobs
