# rfCarrierWave

---

Example Summary
---------------
The carrier wave (CW) example sends a continuous carrier wave or pseudo-random
modulated signal on a fixed frequency.

Note that this example generates the command for continuous TX manually, since
SysConfig does not include commands for continuous TX/RX in ti_radio_config.c.
This means that the frequency and TX output power settings in SysConfig cannot
be used to configure this example.

Example Specific Settings
-----------------------
Update the following in rfCarrierWave.c to configure the example:
1. The frequency is given by the FREQUENCY definition:
    - 2440 MHz by default
2. The TX output power is given by the TX_POWER_DBM definition (signed integer)
   and TX_POWER_FRACTION definition (0 or 1), for a resolution of 0.5 dB:
    - Resulting TX output power = (TX_POWER_DBM + (TX_POWER_FRACTION / 2)) dBm
    - 5.0 dBm by default
3. The TX duration is given by the TX_DURATION_MS definition, where 0 = infinite
   duration (TX never stops). If used, TX will stop and the example will enter
   an infinite loop after the specified duration.
    - 0 by default (infinite duration)
4. Additional configuration is possible in the command setup. For example, set
   .config.sendCw = 0U to transmit a modulated signal instead of an unmodulated
   signal.

Example Usage
-------------
1. On another device, either run a continuous RX test in SmartRf Studio, or run
   the rfContinuousRxRssi example.
2. Observe that the measured RSSI matches the noise floor.
3. Run the example, and observe that the RSSI increases and stabilizes.
4. Move the device running the rfCarrierWave example, and observe that the RSSI
   changes.

Application Design Details
--------------------------
This example consists of a single task that:
- Initializes RCL
- Configures and submits a command to transmit a continuous carrier wave
- Waits for the command to stop (non-zero TX duration expired)
- Waits in an infinite loop

Switching PHYs
--------------
This example uses the BLE generic PHY by default. To select another PHY:
1. Open rfCarrierWave.syscfg in SysConfig
2. Select the "Custom" RF Stack to the left
3. Deselect the currently selected PHY and select another PHY. The example
   supports only one PHY at a time.
