.. _ble_subrate_central:

Bluetooth: LE Connection Subrating Central
##########################################

Overview
********

Companion to the :ref:`ble_subrate_peripheral` sample. It scans for and connects
to that peripheral (advertised name ``subrate_demo``) at a fixed connection
interval, then **Button 1** cycles the subrate factor through active / idle /
dormant tiers (``1 / 8 / 33``) using a central-initiated Connection Subrate
Update (Core Spec, Vol 6, Part B, section 5.1.19).

This demonstrates LE Connection Subrating on real hardware and lets you measure
the central-side current at each factor tier. Factor 1 is the no-subrating
baseline; higher factors let both sides skip connection events. The LED stays
off while idle so it does not perturb a current measurement, and blinks
*tier-index* times (1/2/3) whenever a new factor is applied.

Requirements
************

* Two boards running the Zephyr LL_SW controller with
  ``CONFIG_BT_CTLR_SUBRATING`` (e.g. two nRF54L15 DKs): one running this sample,
  the other running :ref:`ble_subrate_peripheral`.

Building and Running
********************

.. code-block:: console

   west build -b nrf54l15dk/nrf54l15/cpuapp samples/bluetooth/subrate_central
   west flash

After the serial log shows ``Connected``, press **Button 1** to step the factor
(``1 -> 8 -> 33 -> 1``). The log line ``Subrate factor now N`` and the LED blink
count confirm the applied factor. Measure the board current at each tier to
quantify the saving.
