#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# RCV Shorter Connection Intervals fused with subrating: the central drives the
# link to a 1.25 ms interval with a subrate factor > 1 (effective ~5 ms when
# idle). With the subrate skipper live, both ends only converge on the same
# subrated events if they agree on connSubrateBaseEvent (= the wire Instant per
# Core 6.2 5.1.32); a disagreement desyncs the anchor and drops the link. The
# central asserts the negotiated factor >= 2, the link holds, and GATT reads
# complete within the factor*interval cadence -- validating the base_event fix
# and exercising the ZMK idle power-saving path on a low-latency link.

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

simulation_id="conn_sci_subrate"
verbosity_level=2

cd ${BSIM_OUT_PATH}/bin

bsim_exe=./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_conn_subrate_prj_conf_overlay-sci_conf

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=0 -RealEncryption=0 \
  -testid=peripheral_plain -rs=23

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=1 -RealEncryption=0 \
  -testid=central_sci_subrate -rs=6

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} \
  -D=2 -sim_length=40e6 $@

wait_for_background_jobs
