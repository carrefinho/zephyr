#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# Frame Space Update (Core 6.2, LL feature bit 65) end-to-end: a central connects
# to a peripheral, exchanges feature page 1 (to learn the peer's FSU bit 65),
# then runs the Frame Space Update procedure (HCI 0x209D -> LL_FRAME_SPACE_REQ /
# LL_FRAME_SPACE_RSP). The central asserts the 0x35 Frame Space Update Complete
# event fired with success, the negotiated frame space clamps up to the
# responder's 80 us floor (BT_CTLR_FSU_MIN_FRAME_SPACE_US), a below-floor request
# is rejected (0x11), and the link survives - the procedure is control-plane-only
# on this fork (frame_space_apply is a stub, TODO(fsu-radio)), so a sound FSM must
# not perturb the link. Exercises the full HCI -> procedure -> 0x35 event path on
# the FSU overlay build (CONFIG_BT_FRAME_SPACE_UPDATE=y).
#
# The peer is peripheral_plain: the controller answers the LL_FRAME_SPACE_REQ as
# the responder with no host action needed.

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

simulation_id="conn_fsu"
verbosity_level=2

cd ${BSIM_OUT_PATH}/bin

bsim_exe=./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_conn_subrate_prj_conf_overlay-fsu_conf

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=0 -RealEncryption=0 \
  -testid=peripheral_plain -rs=23

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=1 -RealEncryption=0 \
  -testid=central_fsu -rs=6

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} \
  -D=2 -sim_length=20e6 $@

wait_for_background_jobs
