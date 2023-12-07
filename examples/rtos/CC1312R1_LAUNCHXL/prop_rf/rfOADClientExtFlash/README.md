# rfOADClientExtFlash

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
The **OAD Client External Flash** project, together with the OAD Server project,
illustrates how off-chip over-the-air download works. This example works by
receiving a firmware image from the OAD Server and storing it in external
flash. On reset, the bootloader then copies the valid and latest firmware image
into the internal executable flash area.

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
The **OAD Client External Flash** example requires two firmware images to be flashed onto the client:
1. **BIM Image**
    * Prebuilt: ```<SDK_INSTALL_DIR>/examples/nortos/<PLATFORM>/bim/hexfiles/bim_offchip/Release/bim_offchip.hex```
    * Project (Release Configuration): ```<SDK_INSTALL_DIR>/examples/nortos/<PLATFORM>/bim/bim_offchip```
2. **A Signed rfOADClientExtFlash Image**
    * Run **oad_image_tool.py** on the rfOADClientExtFlash image that was built from this example. The **oad_image_tool.py** tool will sign the image.

        ```
        cd <SDK_INSTALL_DIR>/tools/common/oad
        ```
        ```
        python oad_image_tool.py -hex1 <CCS_WORKSPACE>\rfOADClientExtFlash_<PLATFORM>_tirtos7_ticlang\Debug\rfOADClientExtFlash_<PLATFORM>_tirtos7_ticlang.hex -o rfOADClientExtFlash_<PLATFORM>_tirtos7_ticlang -k private.pem ccs output_directory 7
        ```
        NOTE: This is an example usage of the script. Refer to the **OAD Image Tool**'s README for more information.

Once you have the BIM and the signed rfOADClientExtFlash image, use **Uniflash** to load both images onto the target device.

Example Usage
-------------
Once the **OAD Client External Flash** application starts up, it waits for OAD
packets from the OAD Server. On another board then, run the OAD Server example.
Refer to **OAD Server** README for information on how to use it.

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
    |    to External Flash             |                     |                        |                   |
    |                                  +---------------------+------------------------+-------------------+
    |                                                        |
    |                                                        V
    +-----> Packet RX                                    Packet TX