# prep_pipeline — deterministic bsim repro of the ZMK central LL_ASSERT(next) crash

Reproduces the prepare-pipeline overflow behind the ZMK split-central crashes in
[zmkfirmware/zmk#3370](https://github.com/zmkfirmware/zmk/pull/3370):

```
ASSERTION FAIL [next] @ .../ll_sw/nordic/lll/lll.c:891   (K_ERR_KERNEL_OOPS)
  next = ull_prepare_enqueue(...);   // returned NULL: the 7-slot prep MFIFO is full
  LL_ASSERT(next);
```

Root cause (from the coredump triage + the upstream fix): on a device holding
**two simultaneous connections on one radio**, when their connection events
overlap, the LL_SW preempt/dequeue logic mis-handles the overlapping prepares
and leaves **duplicate prepares of the same connection** (same `param`, different
`ticks_at_expire`) in the `prep` pipeline. They accumulate until
`ull_prepare_enqueue()` has no free slot and `LL_ASSERT(next)` trips. Fixed
upstream by `93b951d6fb3` + `0d1b4d2ba6b` (Zephyr v4.3.0; absent from
`v4.1.0+zmk-fixes`).

## Topology

```
 host (central) --7.5ms--> DUT (peripheral + central) --7.5ms--> split (peripheral)
```

The **DUT** is the device under test: like a ZMK central it is a BLE *peripheral*
to `host` (the computer) and a BLE *central* to `split` (its other half), both at
7.5 ms. Each link is saturated with continuous max-MTU write-without-response
traffic (large buffer pools in `prj.conf`), so each event runs long enough that
the DUT's two 7.5 ms events **cannot avoid overlapping** — forcing the buggy
path every event. This is the central-only, overlap-driven trigger established in
the triage; `split` and `host` are single-link and never hit it.

## Run (in CI — bsim is Linux-only)

The `.github/workflows/prep-pipeline-repro.yml` job compiles for `nrf52_bsim`
(the real nRF52 LL_SW controller + radio timing) and sweeps seeds. **Green = the
overflow reproduced.** The `ASSERTION FAIL [next]` line is in the uploaded logs.

Locally on a Linux box with BabbleSim at `/opt/bsim`:

```sh
export ZEPHYR_BASE=$PWD BOARD=nrf52_bsim
source tests/bsim/compile.source
app=tests/bsim/bluetooth/ll/prep_pipeline conf_overlay=overlay_accel.conf compile
wait_for_background_jobs
SEEDS="$(seq 1 16)" SIM_US=30e6 \
  tests/bsim/bluetooth/ll/prep_pipeline/tests_scripts/prep_pipeline_sweep.sh
```

## Knobs

- **`overlay_accel.conf`** shrinks `EVENT_PIPELINE_MAX` 7→3 (via the `#ifndef`
  guard added to `ll_sw/lll.h`) so the leak overflows in seconds. Drop the
  overlay (and raise `SIM_US`) for a faithful depth-7 soak.
- **`SEED` / `SEEDS`** vary how the two anchors land; sweep to find an overflow,
  then that single seed is deterministic.
- **`SIM_US`** sim length. **Buffer counts** in `prj.conf` set per-event airtime.

## Confirm it's the bug, and inspect deterministically

Once a seed reproduces, re-run just that seed and attach gdb to the DUT binary
(or load its coredump) and walk the pipeline — you should see the same signature
as the real dump: the `prep` MFIFO holding the **same connection pointer twice**
(host-link + split-link), differing only in `ticks_at_expire`:

```
mfifo_fifo_prep ... struct lll_event[8]:
  slot: param=<conn A> is_abort_cb=lll_conn_peripheral_is_abort_cb  (host link)
  slot: param=<conn A> ...                                          (DUPLICATE)
  slot: param=<conn B> is_abort_cb=lll_conn_central_is_abort_cb     (split link)
  slot: param=<conn B> ...                                          (DUPLICATE)
```

## A/B against the fix (regression test)

Cherry-pick the upstream fix onto this branch and re-run the **same** sweep — it
should report `NO_OVERFLOW` (fix holds even at the shrunk pipeline depth):

```sh
git remote add upstream https://github.com/zephyrproject-rtos/zephyr || true
git fetch upstream 93b951d6fb31fc499a0594dd4433fb9136944c4c \
                   0d1b4d2ba6b5f46e7b76b49500ddd3d8dde28f2d
git cherry-pick 93b951d6fb31fc499a0594dd4433fb9136944c4c \
                0d1b4d2ba6b5f46e7b76b49500ddd3d8dde28f2d
# rebuild + rerun the sweep -> RESULT: NO_OVERFLOW for every seed
```
