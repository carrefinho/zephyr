.. zephyr:code-sample:: bluetooth_sci_latency
   :name: Shorter Connection Intervals latency tester

   Drive a 1.25 ms LE connection and measure round-trip GATT latency while
   sweeping the subrate factor, on two nRF54L15 DKs.

Overview
********

A hardware-in-the-loop tester for the RCV-tier Shorter Connection Intervals
(SCI) controller. Two nRF54L15 DKs are used: one **central**, one
**peripheral** (selected at build time). The central:

#. connects and exchanges feature page 1 (so it knows the peer's SCI Host
   Support bit),
#. drives the link to a **1.25 ms** connection interval via the LE Connection
   Rate Update procedure,
#. sweeps the **subrate factor** over ``{1, 2, 4, 8, 16}``, and at each factor
   measures the round-trip GATT read latency (10 isolated reads of the peer's
   GAP Device Name) and logs ``min``/``avg``/``max`` over the VCOM UART.

The latency reported is a **round trip** (ATT Read Request out + Read Response
back) measured at the host API, so it includes host-stack overhead. Because the
peripheral sleeps between subrated events, an isolated read waits up to
``factor x 1.25 ms`` -- the latency/power trade-off being characterised. A
one-way (key-event) figure is roughly half the air component and needs a scope +
GPIO trigger, which this UART-only tester does not provide.

Requirements
************

* Two nRF54L15 DKs.
* A controller build with SCI support (this tree's ``sci-rcv`` branch).

Building and running
********************

Build and flash the **peripheral** to the first DK::

   west build -b nrf54l15dk/nrf54l15/cpuapp -d build/periph \
       samples/bluetooth/sci_latency -- -DEXTRA_CONF_FILE=peripheral.conf
   west flash -d build/periph --dev-id <SEGGER_SN_1>

Build and flash the **central** to the second DK::

   west build -b nrf54l15dk/nrf54l15/cpuapp -d build/central \
       samples/bluetooth/sci_latency -- -DEXTRA_CONF_FILE=central.conf
   west flash -d build/central --dev-id <SEGGER_SN_2>

``--dev-id`` is the SEGGER serial of each DK's on-board J-Link
(``nrfjprog --ids`` or ``JLinkExe`` lists them); omit it if only one DK is
attached at a time.

ECV interval sweep (the sub-1.25 ms floor)
==========================================

To find the **hardware floor** for the optional ECV (Extended Connection
Interval Values) tier, build the central with ``ecv_sweep.conf`` instead of
``central.conf`` (the peripheral is unchanged)::

   west build -b nrf54l15dk/nrf54l15/cpuapp -d build/central \
       samples/bluetooth/sci_latency -- -DEXTRA_CONF_FILE=ecv_sweep.conf
   west flash -d build/central --dev-id <SEGGER_SN_2>

Starting just below the known-good RCV 1.25 ms, this drives the link down the
whole 125 us-granular ECV band -- **1125, 1000, 875, 750, 625, 500, 375 us** --
at subrate factor 1, on the standard 150 us tIFS with a reduced CE reservation,
measuring round-trip GATT latency at each and stopping at the first interval the
silicon cannot sustain. It finds the hardware floor wherever it is: bsim
(idealised radio) held 750/625 us, but real single-timer silicon (cumulative
drift + on-air margin) may floor higher -- which is exactly what this sweep
determines. Example output (the floor shown is illustrative, not a prediction)::

   ECV interval sweep (factor 1, round-trip GATT read):
   requested | applied | idle min/avg/max us | burst min/avg/max us
     1125 us | 1125 us |   ...  |   ...  |   ...
     1000 us | 1000 us |   ...  |   ...  |   ...
      875 us |  875 us |   ...  |   ...  |   ...
      750 us | link DROPPED applying interval -- floor reached
   ECV sweep complete. Reset (J-Link/GDB) to re-run.

FSU swap isolation (MODE_FSU_SWAP)
==================================

Isolates the Frame Space Update inter-frame-space swap (Core 6.2 Section
5.1.30.1) from ECV interval timing. On a quiescent 7.5 ms link with no other
traffic in flight, the central negotiates FSU **150 us -> the controller floor
(80 us)**, soaks 30 s with one GATT read/s, then negotiates **back up to 150
us** and soaks 10 s more -- exercising the swap in both directions. The naive
apply has no transitional RX-window widening, so the swap event may drop a
packet on a real radio; this is the first over-the-air test of that. Build the
central by stacking ``mode_fsu_swap.conf`` onto ``central.conf`` (the peripheral
is unchanged)::

   west build -b nrf54l15dk/nrf54l15/cpuapp -d build/central \
       samples/bluetooth/sci_latency \
       -- -DEXTRA_CONF_FILE="central.conf;mode_fsu_swap.conf"

Verdict is ``PASS`` iff there were zero disconnects and zero read failures over
the full 40 s.

ECV under the ZMK-split pointing load (MODE_ECV_LOAD)
====================================================

The load-bearing hardware re-test of the empty-PDU floor sweep. The central
drives the link to an ECV interval (default 750 us), requests 2M PHY, optionally
negotiates FSU down to the 80 us floor (**after** the interval change, since the
conn-rate apply resets tIFS to 150 us), subscribes to the peripheral's load
characteristic, and soaks while the peripheral streams **one 8-byte ZMK
input-event notification per connection interval**. It counts delivered vs
*offered* (a tick-ceil of the generator period), sequence gaps, and -- with the
controller test hooks enabled -- the connection-event count::

   west build -b nrf54l15dk/nrf54l15/cpuapp -d build/central \
       samples/bluetooth/sci_latency \
       -- -DEXTRA_CONF_FILE="central.conf;mode_ecv_load.conf"

Choose the interval, FSU on/off, and soak length with extra ``-D`` flags, e.g.
625 us with FSU off (the airtime-vs-scheduler control run)::

   -- -DEXTRA_CONF_FILE="central.conf;mode_ecv_load.conf" \
      -DCONFIG_SCI_LATENCY_ECV_INTERVAL_125US=5 \
      -DCONFIG_SCI_LATENCY_ECV_FSU=n

``mode_ecv_load.conf`` opens the controller ECV floor to the spec minimum
(``CONFIG_BT_CTLR_SCI_ECV_INTERVAL_MIN_125US=3``) for the sub-625 us probe
builds; the production default (5 = 625 us) is untouched. Verdict is ``PASS``
iff the link survived, delivered >= 90% of *offered*, and there were zero gaps.

Dual-timer A/B arm (overlay-dualtimer.conf)
===========================================

An orthogonal overlay that flips the nRF54L controller from the single-TIMER10
architecture to the dual-timer architecture (EVENT_TIMER = NRF_TIMER00),
dropping the 80 us per-event ISR-latency overhead to 0. Stack it **last** onto
any central or peripheral build to run its arm against the single-timer numbers::

   -- -DEXTRA_CONF_FILE="central.conf;mode_ecv_load.conf;overlay-dualtimer.conf"
   -- -DEXTRA_CONF_FILE="peripheral.conf;overlay-dualtimer.conf"

Result-line grammar
===================

Every data line the new modes emit is prefixed ``RESULT:`` and each run ends
with exactly one ``VERDICT: PASS ...`` or ``VERDICT: FAIL ...`` line, so a run
can be graded straight out of the console log (grep ``^.*RESULT:`` /
``^.*VERDICT:``).

Observe and drive
*****************

Each DK exposes a VCOM over its on-board J-Link (115200 8N1 by default). Open
the central's console to watch the sweep::

   west espressif monitor   # or: minicom/screen on /dev/tty.usbmodem<...>

Re-run the sweep at any time by resetting the central -- via J-Link::

   nrfjprog --reset --dev-id <SEGGER_SN_2>          # or: west flash -d build/central

or via GDB::

   west debug -d build/central
   (gdb) monitor reset
   (gdb) continue

Example central output::

   Link now at 1.25 ms. Sweeping subrate factor...
   factor | effective | min us | avg us | max us
        1 |      1 ms |   ...  |   ...  |   ...
        2 |      2 ms |   ...  |   ...  |   ...
        4 |      5 ms |   ...  |   ...  |   ...
        8 |     10 ms |   ...  |   ...  |   ...
       16 |     20 ms |   ...  |   ...  |   ...
   Sweep complete. Reset (J-Link/GDB) to re-run.
