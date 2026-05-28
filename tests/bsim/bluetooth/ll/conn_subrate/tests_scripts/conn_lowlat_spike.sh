#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# SCI feasibility spike (the RCV go/no-go): a Central holds two links - one
# pushed below 7.5ms (~1ms via the low-latency reduced-reservation path) and one
# full rate at 30ms (the split topology). Asserts the 1ms link holds its anchor
# under the concurrent full-rate link instead of being starved or dropped (the
# preempt-overhead failure mode that parked the CIS-alongside-split work).

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

simulation_id="conn_lowlat_spike"
verbosity_level=2
bin=./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_conn_subrate_prj_conf

cd ${BSIM_OUT_PATH}/bin

Execute ${bin} -v=${verbosity_level} -s=${simulation_id} -d=0 -RealEncryption=0 \
  -testid=peripheral_plain -rs=23
Execute ${bin} -v=${verbosity_level} -s=${simulation_id} -d=1 -RealEncryption=0 \
  -testid=peripheral_plain -rs=44
Execute ${bin} -v=${verbosity_level} -s=${simulation_id} -d=2 -RealEncryption=0 \
  -testid=central_lowlat_multi -rs=6

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} \
  -D=3 -sim_length=20e6 $@

wait_for_background_jobs
