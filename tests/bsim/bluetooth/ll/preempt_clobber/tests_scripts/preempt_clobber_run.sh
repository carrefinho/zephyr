#!/usr/bin/env bash
# Copyright 2026 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0
#
# One run of the preempt_req clobber test.
#
# Env:
#   BIN_SUFFIX     prj_conf (count build) or prj_conf_overlay_assert_conf
#   SEED           base random seed (default 1)
#   CLOBBER        1 = zero preempt_req, 2 = zero preempt_stop_ack on each
#                  host disconnect, 0 = control
#   LOAD           1 = saturated links, 0 = typing-rate traffic
#   YIELD          1 = slow the split link while the host is away (28.75 ms)
#   CYCLE_MS       host link up-time per cycle (default 20000)
#   GAP_MS         host down-time per cycle (default 1000)
#   HOST_INT       host connection interval, 1.25 ms units (default 7)
#   LAT_BURST_US / LAT_PERIOD_US   DUT CPU-latency injector (default off)
#
# Always exits 0; prints the DUT's FINAL stats and one RESULT line:
#   OVERHEAD_ASSERT  LL_ASSERT_OVERHEAD fired (assert build)
#   OTHER_ASSERT     some other controller assert
#   COMPLETED        DUT reached the end of the sim
#   HARNESS_DEATH    no assert and no end-of-sim marker

source "${ZEPHYR_BASE}/tests/bsim/sh_common.source"

seed="${SEED:-1}"
verbosity_level="${verbosity_level:-2}"
bin_suffix="${BIN_SUFFIX:-prj_conf}"
bin="./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_preempt_clobber_${bin_suffix}"
clobber="${CLOBBER:-0}"
load="${LOAD:-0}"
yield="${YIELD:-0}"
cycle_ms="${CYCLE_MS:-20000}"
gap_ms="${GAP_MS:-1000}"
host_int="${HOST_INT:-7}"
lat_burst="${LAT_BURST_US:-0}"
lat_period="${LAT_PERIOD_US:-3300}"
# The DUT/host/split stop themselves at WAIT_TIME (300 s); give the phy a bit more.
sim_us="${SIM_US:-302e6}"
tag="c${clobber}_y${yield}_l${load}_h${host_int}_b${lat_burst}_s${seed}_${bin_suffix}"
sid="pc_${tag}"
params="clobber=${clobber} yield=${yield} load=${load} host_int=${host_int} lat_burst=${lat_burst} seed=${seed} build=${bin_suffix}"

cd "${BSIM_OUT_PATH}/bin"

"${bin}" -v=${verbosity_level} -s="${sid}" -d=0 -RealEncryption=0 \
  -testid=split -rs=$((seed + 10)) > "${sid}_split.log" 2>&1 &
"${bin}" -v=${verbosity_level} -s="${sid}" -d=1 -RealEncryption=0 \
  -testid=dut -rs=$((seed + 20)) \
  -argstest clobber=${clobber} load=${load} yield=${yield} lat_burst=${lat_burst} \
  lat_period=${lat_period} > "${sid}_dut.log" 2>&1 &
"${bin}" -v=${verbosity_level} -s="${sid}" -d=2 -RealEncryption=0 -xo_drift=20e-6 \
  -testid=host -rs=$((seed + 30)) \
  -argstest cycle_ms=${cycle_ms} gap_ms=${gap_ms} load=${load} host_int=${host_int} \
  > "${sid}_host.log" 2>&1 &

./bs_2G4_phy_v1 -v=${verbosity_level} -s="${sid}" -D=3 -sim_length="${sim_us}" \
  > "${sid}_phy.log" 2>&1

wait

last_stats="$(grep -hE '(FINAL|STATS) t_ms=' "${sid}_dut.log" | tail -1)"
echo "----- ${params}"
echo "  dut:  ${last_stats}"
echo "  host: $(grep -h 'HOST_FINAL' "${sid}_host.log" | tail -1)"
echo "  split param updates applied: $(grep -c 'SPLIT_PARAMS' "${sid}_dut.log")," \
     "first overhead: $(grep -h 'OVERHEAD t_ms' "${sid}_dut.log" | head -1 | sed 's/.*OVERHEAD //')"
echo "  stale periods: $(grep -c 'STALE_BEGIN' "${sid}_dut.log")," \
     "longest: $(grep -h 'STALE_END' "${sid}_dut.log" | sed 's/.*dur_ms=//' | sort -n | tail -1)"
if grep -qE 'ASSERTION FAIL' "${sid}"_*.log 2>/dev/null; then
  grep -hE -A1 'ASSERTION FAIL' "${sid}"_*.log | head -4 | sed 's/^/  /'
fi

if grep -qE 'EVENT_OVERHEAD_START_US' "${sid}"_*.log 2>/dev/null; then
  echo "RESULT: OVERHEAD_ASSERT (${params})"
elif grep -qE 'ASSERTION FAIL' "${sid}"_*.log 2>/dev/null; then
  echo "RESULT: OTHER_ASSERT (${params})"
elif grep -qE 'FINAL t_ms=' "${sid}_dut.log" 2>/dev/null; then
  echo "RESULT: COMPLETED (${params})"
else
  echo "----- DUT log tail"
  tail -3 "${sid}_dut.log" 2>/dev/null
  echo "RESULT: HARNESS_DEATH (${params})"
fi
exit 0
