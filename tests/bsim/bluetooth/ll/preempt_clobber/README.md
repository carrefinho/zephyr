# preempt_clobber

Does a stray zero written to the LLL's `preempt_req` hurt the controller?

ZMK split centrals with `CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING=y`
write `peripheral_battery_levels[234] = 0` on every host disconnect (the
disconnected callback fires for the host conn too, and `-EINVAL` becomes 234 in
a `uint8_t`). What that byte hits depends on the build's memory layout. One
report says it was `preempt_req`, and that this stalls preempt timeouts for
about 256 s and trips `LL_ASSERT_OVERHEAD`. bsim can't reproduce the layout, so
this test models the write directly.

Topology is the dual-role DUT from the prep_pipeline repro: peripheral to
`host`, central to `split` at 7.5 ms. The host drops and remakes its link every
`cycle_ms`. With `clobber=1` the DUT zeroes `preempt_req` from the system work
queue on each host disconnect. `clobber=0` is the control.

Controller hooks (only with `LLL_TEST_PREEMPT_HOOKS`, defined by this test's
CMakeLists): `lll_test_preempt_req_set()`, `lll_test_preempt_state()`, and
counters for late prepares (`LL_ASSERT_OVERHEAD` when the assert is off),
preempt ticker starts, and "already requested" skips.

The DUT logs `STALE_BEGIN`/`STALE_END` when `preempt_req != preempt_ack` with no
ticker start in flight for over 50 ms, `OVERHEAD` on each late prepare, and
`STATS`/`FINAL` totals.

Builds: `prj.conf` counts late prepares (`ASSERT_OVERHEAD_START=n`);
`overlay_assert.conf` restores the default assert.

Run one arm: `tests_scripts/preempt_clobber_run.sh` (env documented in the
script). CI: `.github/workflows/preempt-clobber.yml`.
