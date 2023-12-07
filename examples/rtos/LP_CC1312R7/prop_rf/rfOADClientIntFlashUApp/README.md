# rfOADClientIntFlashUApp

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
The **rfOADClientIntFlashUApp** project, together with the **rfOADServer**
project, illustrates how on-chip over-the-air download works. This example works by
receiving a firmware image from the OAD Server, invalidating the current firmware,
and placing the new firmware where the old one was (internal flash). On reset,
the bootloader then validates the new user firmware and boots into it.

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
The **rfOADClientIntFlashUApp** example requires three firmware images to be flashed onto the client:
1. **BIM Image**
    * Prebuilt: ```<SDK_INSTALL_DIR>/examples/nortos/<PLATFORM>/bim/hexfiles/bim_onchip/Release/bim_onchip.hex```
    * Project (Release Configuration): ```<SDK_INSTALL_DIR>/examples/nortos/<PLATFORM>/bim/bim_onchip```
2. **A Signed rfOADClientIntFlashUApp Image**
    * Run **oad_image_tool.py** on the rfOADClientIntFlashUApp image that was built from this example. The **oad_image_tool.py** tool will sign the image.

        ```
        cd <SDK_INSTALL_DIR>/tools/common/oad
        ```
        ```
        python oad_image_tool.py -hex1 <CCS_WORKSPACE>\rfOADClientIntFlashUApp_<PLATFORM>_tirtos7_ticlang\Debug\rfOADClientIntFlashUApp_<PLATFORM>_tirtos7_ticlang.hex -o rfOADClientIntFlashUApp_<PLATFORM>_tirtos7_ticlang_signed_v1 -k private.pem ccs output_directory 7
        ```
        NOTE: For UApp, the BinaryType is "Application + Merged" or 7. Run ```python oad_image_tool.py -h``` for more information on the script's usage.

    * Create a version 2 of the rfOADClientIntFlashUApp
        * Make your firmware updates
        * Modify the value of ```#define SOFTWARE_VER``` in ```oad_image_header_app.c```
        * Rebuild the rfOADClientIntFlashUApp project to get a new image
        * Run **oad_image_tool.py** again on the new image  
            ```
            python oad_image_tool.py -hex1 <CCS_WORKSPACE>\rfOADClientIntFlashUApp_<PLATFORM>_tirtos7_ticlang\Debug\rfOADClientIntFlashUApp_<PLATFORM>_tirtos7_ticlang.hex -o rfOADClientIntFlashUApp_<PLATFORM>_tirtos7_ticlang_signed_v2 -k private.pem ccs output_directory 7
            ```
3. **A Signed rfOADClientIntFlashPApp Image**
    * Project: ```<SDK_INSTALL_DIR>/examples/rtos/<PLATFORM>/prop_rf/rfOADClientIntFlashPApp```
    * Run **oad_image_tool.py** on the rfOADClientIntFlashPApp that was built from the rfOADClientIntFlashPApp example. The **oad_image_tool.py** tool will sign the image.
        NOTE: For PApp, the BinaryType is "Persistent App" or 0. Run ```python oad_image_tool.py -h``` for more information on the script's usage.
        ```
        cd <SDK_INSTALL_DIR>/tools/common/oad
        ```
        ```
        python oad_image_tool.py -hex1 <CCS_WORKSPACE>\rfOADClientIntFlashPApp_<PLATFORM>_tirtos7_ticlang\Debug\rfOADClientIntFlashPApp_<PLATFORM>_tirtos7_ticlang.hex -o rfOADClientIntFlashPApp_<PLATFORM>_tirtos7_ticlang_signed -k private.pem ccs output_directory 0
        ```

Flashing the Images
------------------------
Once you have the BIM, the signed rfOADClientIntFlashUApp image, and the signed rfOADClientIntFlashPApp image, use **Uniflash** to load all three images onto the target device. Depending on the target device, the locations where these images must be flashed vary. Refer to the flash layout below.


          CC13x2_CC26x2
          CC13x1_CC26x1                                   CC13x2x7_CC26x2x7

    +-----------------------+                         +-----------------------+
    |      BIM + CCFG       |                         |      BIM + CCFG       |
    +-----------------------+ 0x00056000              +-----------------------+ 0x000AE000: BIM Flash Location
    |                       |                         |                       |
    |                       |                         |                       |
    |    PersistentApp      |                         |    PersistentApp      |
    +-----------------------+                         +-----------------------+
    |   PApp Image Header   |                         |   PApp Image Header   |
    +-----------------------+ 0x00044000              +-----------------------+ 0x0009C000: PApp Flash Location
    |          NVS          |                         |          NVS          |
    +-----------------------+                         +-----------------------+
    |                       |                         |                       |
    |                       |                         |                       |
    |        UserApp        |                         |        UserApp        |
    +-----------------------+                         +-----------------------+
    |   UApp Image Header   |                         |   UApp Image Header   |
    +-----------------------+ 0x00000000              +-----------------------+ 0x00000000: UApp Flash Location


Example Usage
-------------
Once the **rfOADClientIntFlashUApp** application starts up, it waits for OAD
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
    |           +-----------+----------+---------------------+------------------------+-------------------+
    |           |                      |                     |                        |                   |
    |           V                      V                     V                        V                   V
    |       Block Rsp           FW Version Req        Img Identify Req            Block Req           Reset Req
    |           |                      |                     |                        |                   |
    |           |                      V                     V                        V                   V
    |           |               Send FW Version     Send Img Identify Rsp       Send Block Req      Send Reset Rsp
    |           |                      |                     |                        |                   |
    |           V                      |                     |                        |                   |
    |     Write Img Block              |                     |                        |                   |
    |    to Internal Flash             |                     |                        |                   |
    |                                  +---------------------+------------------------+-------------------+
    |                                                        |
    |                                                        V
    +-----> Packet RX                                    Packet TX