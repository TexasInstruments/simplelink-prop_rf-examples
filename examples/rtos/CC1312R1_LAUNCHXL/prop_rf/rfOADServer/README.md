# rfOADServer

---

Project Setup using the System Configuration Tool (SysConfig)
-------------------------
The purpose of SysConfig is to provide an easy to use interface for configuring
drivers, RF stacks, and more. The .syscfg file provided with each example
project has been configured and tested for that project. Changes to the .syscfg
file may alter the behavior of the example away from default. Some parameters
configured in SysConfig may require the use of specific APIs or additional
modifications in the application source code. More information can be found in
SysConfig by hovering over a configurable and clicking the question mark (?)
next to it's name.

Example Summary
---------------
The **rfOADServer** project together with **rfOADClientExtFlash**,
**rfOADClientIntFlashPApp** + **rfOADClientIntFlashUApp**,
or **rfOADClient**, illustrates how over-the-air download works. This example works by
sending a signed firmware image to the OAD Client one block at a time.

Peripherals Exercised
---------------------
* `CONFIG_GPIO_GLED` - Toggled when data is transmitted over the RF interface
* `CONFIG_GPIO_RLED` - Toggled when data is received over the RF interface

Board Specific Settings
-----------------------
1. The default frequency is:
    - 433.92 MHz for the CC1350-LAUNCHXL-433 and the CC1352P-4-LAUNCHXL
    - 2440 MHz on the CC2640R2-LAUNCHXL and the CC26X2R1-LAUNCHXL
    - 868.0 MHz for other launchpads
In order to change frequency, modify the ti_radio_config.c file. This can be
done using the code export feature in Smart RF Studio, or directly in the file
2. On the CC1352P1 the high PA is enabled (high output power) for all
Sub-1 GHz modes by default.
3. On the CC1352P-2 and CC1352P-4 the high PA operation for Sub-1 GHz modes is not supported
4. The CC2640R2 is setup to run all proprietary physical modes at a center
frequency of 2440 MHz, at a data rate of 250 Kbps

Required Firmware Images
------------------------
1. **rfOADServer Image**
    * From this rfOADServer Project: ```<CCS_WORKSPACE>\rfOADServer_<PLATFORM>_tirtos7_ticlang\Debug\rfOADServer_<PLATFORM>_tirtos7_ticlang.hex```
2. **A Signed Version 2 of the Client Image**  
    * If using the **Boot Image Manager (BIM)** as bootloader, the client image v2 must be built from **rfOADClientExtFlash** or **rfOADClientIntFlashUApp** project. This will be the image that **rfOADServer** will transmit over the air to the client launchpad. Refer to **rfOADClientIntFlashUApp**'s ```README.md``` for information on how to sign images.
    * If using **MCUBoot** as bootloader, the client image v2 must be built from the **rfOADClient** project. This will be the image that **rfOADServer** will transmit over the air to the client launchpad. Refer to **rfOADClient**'s ```README.md``` for information on how to sign images.

Example Usage
-------------
1. Open a serial session (e.g. PuTTY, TeraTerm) to appropriate COM port with the following settings:
    ```
    Baud-rate:      115200
    Data bits:           8
    Stop bits:           1
    Parity:           None
    Flow Control:     None
    ```
2. Using **BTN2** on the Launchpad, cycle through the options, and then select **Update Available FW** using **BTN1**
3. IMPORTANT: Close the serial session. This is required to allow the next step to use the COM port.
4. Open a terminal and run **oad_write_bin.py** to write the signed client image to flash in preparation for OAD.
    ```
    cd <SDK_INSTALL_DIR>/tools/proprf/oad
    ```
    ```
    python oad_write_bin.py COM96 path/to/signed_client_image_v2.bin
    ```
5. At this point, the image is now saved in the rfOADServer's external flash. Open a serial session again and cycle
through the options. This time, select **Update Client FW** to start over-the-air download to the client launchpad.
6. Once the OAD is complete, the client will reboot to the new client application.

Application Design Details
--------------------------

                          Init
                            |
                            |
    +----> Wait for OAD/Radio Events/Button Press
    |                       |
    |                       V
    |        Process OAD Client & Radio Events
    |                       |
    |                       V
    |           +-----------+----------+---------------------------+------------------------+-------------------+
    |           |                      |                           |                        |                   |
    |           V                      V                           V                        V                   V
    |     Get FW Version           Start OAD                  RX Block Req            Update Curr FW      Send Reset Req
    |           |                      |                           |                        |                   |
    |           |                      V                           V                        V                   |
    |           |            Send Img Identify Req        Read then Send Block       Write new blocks to flash  |
    |           |                      |                           |                                            |
    |           V                      |                           |                                            |
    |   Send FW Version Req            |                           |                                            |
    |                                  |                           |                                            |
    |                                  +---------------------------+--------------------------------------------+
    |                                                              |
    |                                                              V
    +-----> Packet RX                                          Packet TX