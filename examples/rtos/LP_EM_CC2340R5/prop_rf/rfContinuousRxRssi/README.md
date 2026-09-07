# rfContinuousRxRssi

---

Example Summary
---------------
The continuous RX RSSI example enables reception on a fixed frequency, without
start-of-packet detection, and reports the received signal strength (RSSI, in
dBm) at a fixed interval over UART, for example to a terminal window.

Note that this example generates the command for continuous RX manually, since
SysConfig does not include commands for continuous TX/RX in ti_radio_config.c.
This means that the frequency setting in SysConfig cannot be used to configure
this example.

Example Specific Settings
-----------------------
Update the following in rfContinuousRxRssi.c to configure the example:
1. The frequency is given by the FREQUENCY definition:
    - 2440 MHz by default
2. The RSSI polling interval (added delay between measurements) is given by the
   RSSI_INTERVAL_US definition:
    - 100 milliseconds by default

Example Usage
-------------
1. Run the example, connected to a host PC terminal (for example, PuTTY or
   TeraTerm), configured for 115200 baud. Observe that the printed RSSI values
   match the noise floor.
2. Run the RF carrier wave example (rfCarrierWave) on another device, and
   observe that the RSSI increases and stabilizes.
3. Move the device running the rfCarrierWave example, and observe that the RSSI
   changes.

Application Design Details
--------------------------
This example consists of a single task that:
- Sets up RCL to receive with synchronization word detection disabled
- Repeatedly:
   - Waits for RSSI_INTERVAL_US
   - Reads the current RSSI value from RCL
   - Reports the RSSI (in dBm) over UART, one value per line

Switching PHYs
--------------
This example uses the BLE generic PHY by default. To select another PHY:
1. Open rfContinuousRxRssi.syscfg in SysConfig
2. Select the "Custom" RF Stack to the left
3. Deselect the currently selected PHY and select another PHY. The example
   supports only one PHY at a time.
