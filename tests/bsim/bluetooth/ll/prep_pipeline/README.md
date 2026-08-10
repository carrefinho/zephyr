# prep_pipeline — deterministic bsim repro of the ZMK central LL_ASSERT(next) crash

Reproduces the prepare-pipeline overflow behind the ZMK split-central lockups in
[zmkfirmware/zmk#3370](https://github.com/zmkfirmware/zmk/pull/3370):

```
ASSERTION FAIL [next] @ .../ll_sw/nordic/lll/lll.c:891   (K_ERR_KERNEL_OOPS)
  next = ull_prepare_enqueue(...);   // returned NULL: the 7-slot prep MFIFO is full
  LL_ASSERT(next);
```

(Field backtraces report lll.c:894 or :924 — that is the stacked return address
one instruction past the `svc`; the assert itself is the `movw r3, #891`
literal.)

## Root cause (bisected + verified on hardware)

First bad commit: **`abfe5f17a949` "Bluetooth: Controller: 1 ms connection"**
(first shipped v4.0.0). It made `lll_conn_{central,peripheral}_is_abort_cb`
return `-EBUSY` for a **same-event preempt** before the first trx
(`(next == curr) && (trx_cnt < 1U)`), and `preempt()` handles `-EBUSY` by
returning **without aborting or marking the ready prepare**. That prepare
strands in the pipeline (`is_resume=0, is_aborted=0`) and is never dequeued.
An event that starts late enough to miss its anchor keeps `trx_cnt == 0`, so
the condition re-arms and **one prepare strands per connection interval** until
`ull_prepare_enqueue()` has no slot. The guard is not gated behind
`BT_CTLR_CONN_INTERVAL_LOW_LATENCY`, so ordinary 7.5 ms links reach it whenever
interrupt latency delays a connection event into its own next interval.

Versions (this harness): v4.0.0 and v4.1.0 overflow; ≥v4.3 does not — the
prepare-deferred feature (`c2eb901`) bounds the case (though its
`LL_ASSERT_DBG(trx_busy_iteration < MAX)` still fires at 400 µs bursts on
v4.4.0 with `BT_CTLR_ASSERT_DEBUG=y`, the default). Pre-4.0 controllers test
GOOD via subtree transplant (see the bisect workflow); pre-4.0 *full trees*
cannot run this harness — their simulator crashes on any software unmask of a
pending controller IRQ.

**These do NOT fix it** (each A/B'd here, all still overflow): `93b951d6fb3`,
`0d1b4d2ba6b`, the "quick check" `diff != 0U` patch, reverting `ee844550b7e`,
reverting `2b30259e9f0`.

**Minimal fix for the 4.0/4.1/4.2 lines** (A/B'd in bsim and on nRF52840):
answer `-ECANCELED` instead of `-EBUSY` at the two sites in `lll_conn.c` —
restores pre-v4.0.0 semantics; the ready same-conn prepare aborts, dequeues,
and the ticker re-enqueues it next interval. (If `CONN_INTERVAL_LOW_LATENCY`
users matter, gate the `-EBUSY` on `IS_ENABLED(...)` instead — identical code
for stock builds.)

## Topology and the load-bearing ingredient

```
 host (central) --7.5ms--> DUT (peripheral + central) --7.5ms--> split (peripheral)
```

The DUT is shaped like a ZMK central: BLE peripheral to `host`, BLE central to
`split`, both links saturated with write-without-response traffic. **Geometry
alone never reproduces** — bsim charges zero CPU time, so no prepare is ever
late. The missing dimension is injected interrupt latency (real boards get it
from flash write stalls, display SPI, ISR load). The DUT's injector takes
per-run args, no rebuild:

- `-argstest lat_burst=<us> lat_period=<us>` — burst length / spacing.
  **Reproducing cell: `lat_burst=400 lat_period=3300`, seed 1 → assert < 1 s**
  after the blast starts (both at stock depth 7 and with `overlay_accel.conf`).
  `lat_burst=0` disables injection (control).
- `-argstest lat_isr=<mode>` — how the burst is applied:
  - `2` (default): `irq_disable()` of the controller's IRQs
    (RADIO/TIMER0/RTC0/SWI4/SWI5) around a busy-wait. Valid vectors on every
    tree era.
  - `0`: `irq_lock()` around the busy-wait. Equivalent results on ≥4.0;
    SIGSEGVs pre-4.0 simulators (unmask of a pending IRQ dispatches through a
    garbage vector).
  - `1`: busy-wait inside a timer ISR — **inert** (the simulator nests
    dispatch during it); kept only as a negative control.

## Reference CI runs

- [repro + A/B arms](https://github.com/carrefinho/zephyr/actions/runs/31297556296)
  — base overflows at the cell above; the `-ECANCELED` fix holds the full
  matrix; `93b951d6fb3`+`0d1b4d2ba6b`, the quick-check, the `ee844550` revert
  and the `2b30259` revert all still overflow. The gdb pipeline walk at the
  assert is in the `prep-pipeline-repro-logs` artifact.
- [bisect](https://github.com/carrefinho/zephyr/actions/runs/31296039722) —
  v3.7.0..v4.0.0 controller-subtree transplant into a v4.0.0 tree, endpoints
  validated, converges on `abfe5f17a949`; GOOD verdicts require a positively
  completed full sim.
- [version ladder](https://github.com/carrefinho/zephyr/actions/runs/31297570029)
  — v4.0.0 overflows, v4.4.0 does not (its cells instead die on the
  defer-budget `LL_ASSERT_DBG`); v3.6/v3.7 full trees are UNTESTABLE per the
  simulator limitation above.

## Run

Linux + BabbleSim at `/opt/bsim` (or use the workflows in
`.github/workflows/prep-pipeline-*.yml`: repro + A/B arms, version ladder,
bisect):

```sh
export ZEPHYR_BASE=$PWD BOARD=nrf52_bsim BOARD_TS=nrf52_bsim
source tests/bsim/compile.source
app=tests/bsim/bluetooth/ll/prep_pipeline compile          # stock depth-7
wait_for_background_jobs
BIN_SUFFIX=prj_conf SEEDS="1 2" BURSTS_US="400" PERIODS_US="3300" SIM_US=30e6 \
  tests/bsim/bluetooth/ll/prep_pipeline/tests_scripts/prep_pipeline_sweep.sh
```

Verdicts are positive-evidence: `REPRODUCED` keys on `ASSERTION FAIL [next]`
specifically (other controller asserts report `OTHER_ASSERT`), and
`NO_OVERFLOW` requires the bstest end-of-sim marker — a cell that dies any
other way is `HARNESS_DEATH`, never a pass. `prj.conf` sets
`BT_CTLR_ASSERT_OVERHEAD_START=n` so the unrelated EVENT_OVERHEAD_START assert
family (the zmk#3331 sibling) cannot fire first and mask the pipeline
overflow.

## Confirm the signature

Re-run a reproducing seed with the DUT under gdb (`tests_scripts/
prep_pipeline_inspect.sh`) and walk `mfifo_fifo_prep`: the same connection
`param` in multiple slots, `ticks_at_expire` exactly one connection interval
apart, every entry `is_resume=0, is_aborted=0`. Identical to both #3370 field
coredumps (decoded) and the nRF52840 hardware repro.
