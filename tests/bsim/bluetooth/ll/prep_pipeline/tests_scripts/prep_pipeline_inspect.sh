#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Re-run ONE reproducing cell with the DUT under gdb, break at the assert, and
# dump the prepare-pipeline MFIFO. This is the signature check that separates
# THE bug from legitimate overload: the zmk#3370 coredumps show the pipeline
# full of DUPLICATE entries of the same connection - same prepare_param.param,
# different ticks_at_expire, is_resume=0, is_aborted=0. If the dumped entries
# below show that pattern, the bsim cell is the field crash.
#
# Env: SEED, LAT_BURST_US, LAT_PERIOD_US (set these to the reproducing cell),
#      SIM_US, BIN_SUFFIX as in prep_pipeline_overflow.sh.

source "${ZEPHYR_BASE}/tests/bsim/sh_common.source"

seed="${SEED:-1}"
sim_us="${SIM_US:-30e6}"
verbosity_level="${verbosity_level:-2}"
bin_suffix="${BIN_SUFFIX:-prj_conf_overlay_accel_conf}"
bin="./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_prep_pipeline_${bin_suffix}"
lat_burst="${LAT_BURST_US:-400}"
lat_period="${LAT_PERIOD_US:-3300}"
sid="prep_pipe_inspect"

HOST_DRIFT="${HOST_DRIFT:-20e-6}"
DUT_DRIFT="${DUT_DRIFT:-0}"

cd "${BSIM_OUT_PATH}/bin"

gdb_script="$(mktemp)"
cat > "${gdb_script}" <<'EOF'
set pagination off
set confirm off
break assert_post_action
break z_fatal_error
run
echo \n===== INSPECT: backtrace at assert =====\n
bt 12
echo \n===== INSPECT: prep MFIFO meta =====\n
print mfifo_prep
print/d mfifo_fifo_prep.f
print/d mfifo_fifo_prep.l
echo \n===== INSPECT: prep MFIFO entries =====\n
set $i = 0
while $i < mfifo_prep.n
  echo ---- slot ----\n
  print *(struct lll_event *)((char *)mfifo_fifo_prep.m + $i * mfifo_prep.s)
  set $i = $i + 1
end
quit
EOF

"${bin}" -v=${verbosity_level} -s="${sid}" -d=0 -RealEncryption=0 \
  -testid=split -rs=$((seed + 10)) > "${sid}_split.log" 2>&1 &
"${bin}" -v=${verbosity_level} -s="${sid}" -d=2 -RealEncryption=0 -xo_drift=${HOST_DRIFT} \
  -testid=host  -rs=$((seed + 30)) > "${sid}_host.log" 2>&1 &
./bs_2G4_phy_v1 -v=${verbosity_level} -s="${sid}" -D=3 -sim_length="${sim_us}" \
  > "${sid}_phy.log" 2>&1 &

gdb -batch -x "${gdb_script}" \
  --args "${bin}" -v=${verbosity_level} -s="${sid}" -d=1 -RealEncryption=0 \
  -xo_drift=${DUT_DRIFT} -testid=dut -rs=$((seed + 20)) \
  -argstest lat_burst=${lat_burst} lat_period=${lat_period} \
  2>&1 | tee "${sid}_dut_gdb.log"

wait
rm -f "${gdb_script}"
exit 0
