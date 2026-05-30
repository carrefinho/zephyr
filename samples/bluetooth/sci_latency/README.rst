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

This drives the link down the 125 us-granular sub-1.25 ms band -- **750, 625,
500, 375 us** -- at subrate factor 1, on the standard 150 us tIFS with a reduced
CE reservation, measuring round-trip GATT latency at each and stopping at the
first interval the silicon cannot sustain. It is the on-silicon confirmation of
the bsim ~625 us nRF54L floor: single-timer cumulative drift and real on-air
margin, which the idealised bsim radio model cannot reproduce. Example output::

   ECV interval sweep (factor 1, round-trip GATT read):
   requested | applied | idle min/avg/max us | burst min/avg/max us
      750 us |  750 us |   ...  |   ...  |   ...
      625 us |  625 us |   ...  |   ...  |   ...
      500 us | link DROPPED applying interval -- floor reached
   ECV sweep complete. Reset (J-Link/GDB) to re-run.

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
