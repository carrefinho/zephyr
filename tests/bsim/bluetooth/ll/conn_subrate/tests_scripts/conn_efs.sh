#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

# LL Extended Feature Set (Feature Page Exchange): a central connects to a
# peripheral, then issues an LE Read All Remote Features for page 1. The
# peripheral has no page-1 feature bits yet, so it returns page 0 plus an
# all-zero page 1 (max page 0). The central asserts the procedure completed, the
# link survived, page 0 came through populated, and page 1 is zero - proving the
# 248-octet completion carrier was filled from the peer without corruption.
#
# Runs the EFS overlay build (CONFIG_BT_LE_EXTENDED_FEAT_SET=y), so the binary
# name carries the overlay suffix.

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

simulation_id="conn_efs"
verbosity_level=2

cd ${BSIM_OUT_PATH}/bin

bsim_exe=./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_conn_subrate_prj_conf_overlay-efs_conf

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=0 -RealEncryption=0 \
  -testid=peripheral_plain -rs=23

Execute ${bsim_exe} \
  -v=${verbosity_level} -s=${simulation_id} -d=1 -RealEncryption=0 \
  -testid=central_efs -rs=6

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} \
  -D=2 -sim_length=20e6 $@

wait_for_background_jobs
