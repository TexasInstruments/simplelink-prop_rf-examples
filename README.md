# simplelink-prop_rf-examples

This GitHub project contains the F2 and F3 prop_rf examples for Linux.  Builds based on the 7.10 F2 and F3 SDKs.  Please note, F2 repository name will change from simplelink-lowpower-f2-sdk to cc13xx_cc26xx_sdk so it will match with the name of the existing F2 SDK.

## Setup Instructions

### SDK Build
By default, the F2 and F3 SDKs are included as submodules.  Depending on which devices are of interest, only one family of SDK needs to be built.  See [SDK/Board/Device/Association](#SDK_Association) for details. 

### Edit **imports.mak**
In each family subdirectory, there is a imports.mak file that is a generic sample and will not work out of the box.  This file must be updated with the tool (compilers, cmake, etc.) paths installed on your system.

For a Linux build, settings must be updated to match your build system's setup.  The only outlier may be Python, as most python3.6+ interpreters will work.  Please note cmake must be 3.21+, or the builds will fail.  If using CCS ccs1220, the sysconfig installed is incompatible with the SDKs.  Either upgrade CCS to ccs1230 or install sysconfig 1.16.1 from https://www.ti.com/tool/SYSCONFIG.  See [Resources](#Resources) for URL's of tools that need installation to build the SDKs and examples.  Please note, XDC_INSTALL_DIR was required in older SDKs but no longer needed.

By default TICLANG and GCC toolchains are enabled.  If a toolchain is not needed, unset the compiler, for example, `GCC_ARMCOMPILER ?=`.

### Default imports.mak

#`XDC_INSTALL_DIR`        ?= /home/username/ti/xdctools_3_62_01_15_core (Not required for 7.10+ SDK's)

`SYSCONFIG_TOOL`         ?= /home/username/ti/ccs1230/ccs/utils/sysconfig_1.16.1/sysconfig_cli.sh

`FREERTOS_INSTALL_DIR`   ?= /home/username/FreeRTOSv202104.00

`CMAKE`                 ?= /home/username/cmake-3.21.3/bin/cmake

`PYTHON`                 ?= python3

`TICLANG_ARMCOMPILER`    ?= /home/username/ti/ccs1230/ccs/tools/compiler/ti-cgt-armllvm_2.1.3.LTS-0

`GCC_ARMCOMPILER`        ?= /home/username/ti/ccs1230/ccs/tools/compiler/9.2019.q4.major-0

Edit **imports.mak** and update all of the above tool location variables to reflect the paths on your build system.

## Build SDK Libraries
SDK libraries must exist before building any examples. To build SDK libraries from the simplelink-prop_rf-examples:

cd simplelink-lowpower-f2-sdk/

or

cd simplelink-lowpower-f3-sdk/

make

The make will go through the SDK and build all libraries.  Depending on the build machine, the make will run for a few minutes.   Please note, in the production release, simplelink-lowpower-f2-sdk/ will be renamed cc13xx_cc26xx_sdk/.

## Build Examples From Command Line
After building the SDK(s), from simplelink-prop_rf-examples cd to examples/rtos/<board_name> (rtos examples) or examples/nortos/<board_name> for non rtos base examples.

for example:

cd examples/rtos/LP_EM_CC2340R5

make clean

make


## Build Examples From CCS

Before building from CCS, the SDKs must have been built (see Build SDK Libraries), and CCS must be configured with the locations of the SDK, FreeRTOS.  As in the command line build, the SDK's must have been built before examples will compile (see Build SDK Libraries). 

Add the location of simplelink-prop_rf-examples/simplelink-lowpower-f2-sdk and simplelink-prop_rf-examples/simplelink-lowpower-f3-sdk to CCS.

1. Preferences->Code Composer Studio->Products 
2. Select Add... 
3. Navigate to where simplelink-prop_rf-examples/simplelink-lowpower-f2-sdk and/or simplelink-prop_rf-examples/simplelink-lowpower-f3-sdk is installed
4. Select Open.  
 
The above steps will add the SDK to the build path.  Successful addition of the SDKs will be displayed under "Discovered Products:".  Note the highlighted section under "Discovered products:".

<img src="images/add_products.png"  width="600" height="400">


The location of FreeRTOS must be configured in CCS.  This is done by Preferences->Code Composer Studio->Build->Environment.  Then select Add... Add the variable name `FREERTOS_INSTALL_DIR` and set it to the absolute path of FreeRTOS on the build system.


<img src="images/FreeRTOS.png"  width="600" height="400">



 After re-starting CCS, File->import->Code Composer Studio->CCS Projects and press next.  From simplelink-prop_rf-examples/ navigate to examples/rtos/<board_name/prop_rf/<example_name>/<rtos> or examples/nortos/<board_name/prop_rf/<example_name>/. As a concrete example, navigate to **examples/rtos/LP_EM_CC2340R5/prop_rf/rfPacketTx**.


Press Browse, then Open.  Available examples will show up in the dialog box.  Select the example and press Finish.  After selecting Finish, the example chosen will import into your workspace.


<img src="images/select_ccsproject.png"  width="300" height="400">




## Resources

Tools Download Locations

SysConfig (SYSCONFIG_TOOL) https://www.ti.com/tool/SYSCONFIG, or if using ccs1230+ it is part of CCS

Free RTOS (FREERTOS_INSTALL_DIR) https://github.com/FreeRTOS/FreeRTOS/releases/download/202104.00/FreeRTOSv202104.00.zip

TI CLANG Compiler (TICLANG_ARMCOMPILER) https://www.ti.com/tool/CCSTUDIO 

ARM Gnu Toolchain (GCC_ARMCOMPILER) https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads/9-2019-q4-major

## Troubleshooting

When building on *nix platform (Linux/Mac) the library build will fail with an error similar to:

error: /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/ranlib: Unsupported triple for mach-o cpu type: thumbv6m-ti-none-eabi

To fix, make sure the arm version of ranlib is in the path before the OS version of ranlib located in /usr/bin. Simply set the location of the gcc ARM ranlib ahead in the shell's path.  Example:

export `PATH`=/Users/username/ti/gcc_arm_none_eabi_9_2_1/arm-none-eabi/bin:$PATH

## SDK_Association

[F2](images/simplelink_cc13xx_cc26xx_sdk.md)

[F3](images/simplelink_lowpower_f3_sdk.md)
