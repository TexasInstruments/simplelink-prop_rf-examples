SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR ?= $(abspath ../../../../../../..)

include $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/imports.mak

CC = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-gcc"
LNK = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-gcc"
HEX = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-objcopy"

SYSCONFIG_GUI_TOOL = $(dir $(SYSCONFIG_TOOL))sysconfig_gui$(suffix $(SYSCONFIG_TOOL))
SYSCFG_CMD_STUB = $(SYSCONFIG_TOOL) --compiler gcc --product $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/.metadata/product.json
SYSCFG_GUI_CMD_STUB = $(SYSCONFIG_GUI_TOOL) --compiler gcc --product $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/.metadata/product.json
SYSCFG_FILES := $(shell $(SYSCFG_CMD_STUB) --listGeneratedFiles --listReferencedFiles --output . ../../tirtos7/rfOADServer.syscfg)

SYSCFG_C_FILES = $(filter %.c,$(SYSCFG_FILES))
SYSCFG_H_FILES = $(filter %.h,$(SYSCFG_FILES))
SYSCFG_OPT_FILES = $(filter %.opt,$(SYSCFG_FILES))

OBJECTS = rfServer.obj oad_native_oad_oad_server.obj oad_native_oad_oad_protocol.obj oad_native_oad_oad_storage.obj common_cc26xx_flash_interface_external_flash_interface_ext_rtos_NVS.obj common_cc26xx_crc_crc32.obj radio_radio.obj radio_RFQueue.obj tirtos7_main_tirtos.obj $(patsubst %.c,%.obj,$(notdir $(SYSCFG_C_FILES)))
NAME = rfOADServer

# Each config-specific .mak file will initialize appropriate options (AFLAGS, CFLAGS, LFLAGS),
# which will then be appended to below.
CONFIG := DEBUG_BIM
ifeq ($(CONFIG), DEBUG_BIM)
	include Debug_BIM.mak
else ifeq ($(CONFIG), RELEASE_BIM)
	include Release_BIM.mak
else ifeq ($(CONFIG), DEBUG_MCUBOOT)
	include Debug_MCUBoot.mak
else ifeq ($(CONFIG), RELEASE_MCUBOOT)
	include Release_MCUBoot.mak
endif

CFLAGS += -I../.. \
    -I. \
    $(addprefix @,$(SYSCFG_OPT_FILES)) \
    -DFEATURE_OAD_SERVER_ONLY \
    -DCC13X2 \
    -DSECURITY \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/common/cc26xx/flash_interface" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/kernel/tirtos7/packages" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/posix/gcc" \
    -mcpu=cortex-m33 \
    -mthumb \
    -std=c99 \
    -mfloat-abi=hard \
    -mfpu=fpv5-sp-d16 \
    -ffunction-sections \
    -fdata-sections \
    -g \
    -gstrict-dwarf \
    -Wall \
    "-I$(GCC_ARMCOMPILER)/arm-none-eabi/include/newlib-nano" \
    "-I$(GCC_ARMCOMPILER)/arm-none-eabi/include"

LFLAGS += -Wl,--defsym=FEATURE_OAD_SERVER_ONLY=1 \
    -Wl,-T,../../tirtos7/gcc/cc13x4_cc26x4_tirtos7.lds \
    "-Wl,-Map,$(NAME).map" \
    "-L$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source" \
    -mcpu=cortex-m33 \
    "-L$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/kernel/tirtos7/packages" \
    ti_utils_build_linker.cmd.genlibs \
    -mthumb \
    -mfloat-abi=hard \
    -mfpu=fpv5-sp-d16 \
    -nostartfiles \
    -static \
    -Wl,--gc-sections \
    -lgcc \
    -lc \
    -lm \
    --specs=nano.specs

all: $(NAME).hex

.INTERMEDIATE: syscfg
$(SYSCFG_FILES): syscfg
	@ echo generation complete

syscfg: ../../tirtos7/rfOADServer.syscfg
	@ echo Generating configuration files...
	@ $(SYSCFG_CMD_STUB) --output $(@D) $<


# Helpful hint that the user needs to use a standalone SysConfig installation
$(SYSCONFIG_GUI_TOOL):
	$(error $(dir $(SYSCONFIG_TOOL)) does not contain the GUI framework \
        necessary to launch the SysConfig GUI.  Please set SYSCONFIG_TOOL \
        (in your SDK's imports.mak) to a standalone SysConfig installation \
        rather than one inside CCS)

syscfg-gui: ../../tirtos7/rfOADServer.syscfg $(SYSCONFIG_GUI_TOOL)
	@ echo Opening SysConfig GUI
	@ $(SYSCFG_GUI_CMD_STUB) $<

define C_RULE
$(basename $(notdir $(1))).obj: $(1) $(SYSCFG_H_FILES)
	@ echo Building $$@
	@ $(CC) $(CFLAGS) "$$<" -c -o $$@
endef

$(foreach c_file,$(SYSCFG_C_FILES),$(eval $(call C_RULE,$(c_file))))

rfServer.obj: ../../rfServer.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -c -o $@

oad_native_oad_oad_server.obj: ../../oad/native_oad/oad_server.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -c -o $@

oad_native_oad_oad_protocol.obj: ../../oad/native_oad/oad_protocol.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -c -o $@

oad_native_oad_oad_storage.obj: ../../oad/native_oad/oad_storage.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -c -o $@

common_cc26xx_flash_interface_external_flash_interface_ext_rtos_NVS.obj: ../../common/cc26xx/flash_interface/external/flash_interface_ext_rtos_NVS.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -c -o $@

common_cc26xx_crc_crc32.obj: ../../common/cc26xx/crc/crc32.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -c -o $@

radio_radio.obj: ../../radio/radio.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -c -o $@

radio_RFQueue.obj: ../../radio/RFQueue.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -c -o $@

tirtos7_main_tirtos.obj: ../../tirtos7/main_tirtos.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -c -o $@

$(NAME).out: $(OBJECTS)
	@ echo linking $@
	@ $(LNK)  $(OBJECTS)  $(LFLAGS) -o $(NAME).out

$(NAME).hex: $(NAME).out
	@ echo building $@
	@ $(HEX) -O ihex --remove-section .vtable --remove-section .dmaSpi0RxControlTableEntry --remove-section .dmaSpi0TxControlTableEntry --remove-section .dmaSpi1RxControlTableEntry --remove-section .dmaSpi1TxControlTableEntry --remove-section .dmaSpi0RxAltControlTableEntry --remove-section .dmaSpi0TxAltControlTableEntry --remove-section .dmaSpi1RxAltControlTableEntry --remove-section .dmaSpi1TxAltControlTableEntry $? $@

clean:
	@ echo Cleaning...
	@ $(RM) $(OBJECTS) > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).out > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).map > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).hex > $(DEVNULL) 2>&1
	@ $(RM) $(SYSCFG_FILES)> $(DEVNULL) 2>&1
