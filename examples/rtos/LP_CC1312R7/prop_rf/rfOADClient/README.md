# rfOADClient

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
The **rfOADClient** project, together with the **rfOADServer**
project, illustrates how on-chip over-the-air download works with MCUBoot. The primary slot holds
the main image that is run, while the secondary slot is used during the OAD process to temporarily
store the new incoming image. This example works by receiving a firmware image from the OAD Server
and placing it in the secondary slot. MCUBoot then copies this image into the primary slot, if needed.
Upon reset, the new image is now in the primary slot and is run.

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
The **rfOADClient** example requires two firmware images to be flashed onto the client:
1. **MCUBoot Image**
    * Project: ```<SDK_INSTALL_DIR>/examples/nortos/<PLATFORM>/mcuboot/mcuboot```
        * In CCS, after importing the MCUBoot project, modify ```<mcuboot_project>/flash_map_backend/flash_map_backend.h```
        to set the desired MCUBoot slot size. It is important that this matches the slot size set in **rfOADClient** image and in
        the **imgtool** sign command. We recommend setting the slot size to the following values as a starting point:
        ```
        #define BOOT_PRIMARY_1_SIZE                 0x0000D000
        #define BOOT_SECONDARY_1_SIZE               0x0000D000
        ```
        * The slots' base addresses must also be set properly in ```<mcuboot_project>/flash_map_backend/flash_map_backend.h``` depending on the target device family  
            * CC13x2x7_CC26x2x7
            ```
            #define BOOT_PRIMARY_1_BASE_ADDRESS         0x00000000
            #define BOOT_SECONDARY_1_BASE_ADDRESS       0x00056000
            ```
            * CC13x4_CC26x4
            ```
            #define BOOT_PRIMARY_1_BASE_ADDRESS         0x00006000
            #define BOOT_SECONDARY_1_BASE_ADDRESS       0x00083000
            ```
            NOTE: Refer to the Flash Layout section below for more details
        * In MCUBoot project, modify ```<mcuboot_project>/mcuboot_config/mcuboot_config.h``` to set MCUBoot's Upgrade Mode to overwrite only mode.
            ```
            #define MCUBOOT_OVERWRITE_ONLY
            // #define MCUBOOT_DIRECT_XIP
            ```
2. **A Signed rfOADClient Image**
    * Modify ```<rfOADClient>/oad/native_oad/oad_config.h``` to set the desired MCUBoot slot size. Make sure the value matches what is set in the MCUBoot project
    ```
    #define MCUBOOT_SLOT_SIZE                   0x0D000
    ```
    * Run **imgtool** on the rfOADClient image that was built from this example. The **imgtool** tool will sign the image. Again, make sure the slot size matches
    what is set in both MCUBoot project and rfOADClient project.
        ```
        cd <SDK_INSTALL_DIR>/tools/common/mcuboot
        ```
        Sign a version 1 of the image
        ```
        ./imgtool sign --header-size 0x20 --align 4 --slot-size 0xD000 --pad --version 1.0.0 --pad-header --key <SDK_INSTALL_DIR>/source/third_party/mcuboot/root-ec-p256.pem <CCS_WORKSPACE>\rfOADClient_<PLATFORM>_tirtos7_ticlang\Debug\rfOADClient_<PLATFORM>_tirtos7_ticlang.hex <CCS_WORKSPACE>\rfOADClient_<PLATFORM>_tirtos7_ticlang\Debug\rfOADClient_<PLATFORM>_tirtos7_ticlang_signed_v1.hex"
        ```
        After your firmware updates, rebuild the project and sign a version 2 of the image
        ```
        ./imgtool sign --header-size 0x20 --align 4 --slot-size 0xD000 --pad --version 2.0.0 --pad-header --key <SDK_INSTALL_DIR>/source/third_party/mcuboot/root-ec-p256.pem <CCS_WORKSPACE>\rfOADClient_<PLATFORM>_tirtos7_ticlang\Debug\rfOADClient_<PLATFORM>_tirtos7_ticlang.hex <CCS_WORKSPACE>\rfOADClient_<PLATFORM>_tirtos7_ticlang\Debug\rfOADClient_<PLATFORM>_tirtos7_ticlang_signed_v2.hex"
        ```
        NOTE: This is an example usage of the script. Refer to the **ImgTool**'s README for more information.

Once you have MCUBoot and the signed rfOADClient v1 image, use **Uniflash** to load both images onto the target device.

Flash Layout
-------------

* CC13x2x7_CC26x2x7
```
                    +-----------------------+ 0x000B0000
                    |    MCUBoot + CCFG     |
                    | CCFG: 0xAFFA8-0xAFFFF |
                    +-----------------------+ 0x000AC000
                    |       Padding         |
                    +-----------------------+ BOOT_SECONDARY_1_BASE_ADDRESS + BOOT_SECONDARY_1_SIZE
                    |                       |
                    |                       |
                    |    Secondary Slot     |
                    |                       |
                    |                       |
                    +-----------------------+ BOOT_SECONDARY_1_BASE_ADDRESS
                    |        Padding        |
        +->         +-----------------------+ BOOT_PRIMARY_1_SIZE
        |           |    MCUBoot Trailer    |
        |           +-----------------------+
        |           |        Padding        |
        |           +-----------------------+ Varies
        |           |                       |
    Primary Slot    |    Client User App    |
        |           |                       |
        |           +-----------------------+
        |           |      OAD Header       |
        |           +-----------------------+
        |           |     Entry Section     |
        |           +-----------------------+
        |           |    MCUBoot Header     |
        +->         +-----------------------+ 0x00000000
```

* CC13x4_CC26x4
```
                    +-----------------------+ 0x50000800
                    |         CCFG          |
                    +-----------------------+ 0x50000000
                    |     Padding/Holes     |
                    +-----------------------+ BOOT_SECONDARY_1_BASE_ADDRESS + BOOT_SECONDARY_1_SIZE
                    |                       |
                    |                       |
                    |    Secondary Slot     |
                    |                       |
                    |                       |
                    +-----------------------+ BOOT_SECONDARY_1_BASE_ADDRESS
                    |        Padding        |
        +->         +-----------------------+ BOOT_PRIMARY_1_SIZE + 0x00006000
        |           |    MCUBoot Trailer    |
        |           +-----------------------+
        |           |        Padding        |
        |           +-----------------------+ Varies
        |           |                       |
    Primary Slot    |    Client User App    |
        |           |                       |
        |           +-----------------------+
        |           |       OAD Header      |
        |           +-----------------------+
        |           |     Entry Section     |
        |           +-----------------------+
        |           |     MCUBoot Header    |
        +->         +-----------------------+ 0x00006000
                    |        Padding        |
                    +-----------------------+ 0x00005800
                    |        MCUBoot        |
                    +-----------------------+ 0x00000000
```

Example Usage
-------------
Once the **rfOADClient** application starts up, it waits for OAD
packets from the OAD Server. On another board then, run the OAD Server example.
Refer to **rfOADServer** README for information on how to use it.

Application Design Details
--------------------------

                          Init
                            |
                            |
    +---------> Wait for OAD/Radio Events
    |                       |
    |                       V
    |        Process OAD Client & Radio Events
    |                       |
    |                       V
    |           +-----------+----------+---------------------+------------------------+
    |           |                      |                     |                        |
    |           V                      V                     V                        V
    |       Block Rsp           FW Version Req        Img Identify Req            Block Req
    |           |                      |                     |                        |
    |           |                      V                     V                        V
    |           |               Send FW Version     Send Img Identify Rsp       Send Block Req
    |           |                      |                     |                        |
    |           V                      |                     |                        |
    |     Write Img Block              |                     |                        |
    |    to Internal Flash             |                     |                        |
    |                                  +---------------------+------------------------+
    |                                                        |
    |                                                        V
    +-----> Packet RX                                    Packet TX