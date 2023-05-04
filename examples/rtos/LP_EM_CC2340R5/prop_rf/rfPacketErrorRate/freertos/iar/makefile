COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR ?= $(abspath ../../../../../../..)

include $(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/imports.mak

CC = "$(IAR_ARMCOMPILER)/bin/iccarm"
LNK = "$(IAR_ARMCOMPILER)/bin/ilinkarm"

SYSCFG_CMD_STUB = $(SYSCONFIG_TOOL) --compiler iar --product $(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/.metadata/product.json
SYSCFG_FILES := $(shell $(SYSCFG_CMD_STUB) --listGeneratedFiles --listReferencedFiles --output . ../../freertos/rfPacketErrorRate.syscfg)

SYSCFG_C_FILES = $(filter %.c,$(SYSCFG_FILES))
SYSCFG_H_FILES = $(filter %.h,$(SYSCFG_FILES))

OBJECTS = config.obj menu.obj rfPacketErrorRate.obj rx.obj tx.obj setup_rcl_settings_ble_generic.obj setup_rcl_settings_msk_250_kbps.obj setup_rcl_settings_msk_500_kbps.obj freertos_main_freertos.obj $(patsubst %.c,%.obj,$(notdir $(SYSCFG_C_FILES)))
NAME = rfPacketErrorRate

CFLAGS = -I../.. \
    -I. \
    -DVARIABLE_LENGTH_SETUP \
    -DDeviceFamily_CC23X0R5 \
    "-I$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source" \
    "-I$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/kernel/freertos" \
    "-I$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/posix/iar" \
    "-I$(FREERTOS_INSTALL_DIR)/FreeRTOS/Source/include" \
    "-I$(FREERTOS_INSTALL_DIR)/FreeRTOS/Source/portable/IAR/ARM_CM0" \
    --debug \
    --silent \
    -e \
    --aeabi \
    --thumb \
    --diag_suppress=Pa050 \
    --cpu=Cortex-M0+ \
    --vla

LFLAGS = "-L$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source" \
    "-L$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/kernel" \
    --threaded_lib \
    -fti_utils_build_linker.cmd.genlibs \
    "$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/devices/cc23x0r5/driverlib/lib/iar/driverlib.a" \
    "$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/drivers/rcl/lib/iar/m0p/rcl_cc23x0r5.a" \
    --config ../../freertos/iar/cc23x0r5_freertos.icf \
    --map "$(NAME).map" \
    --silent \
    --semihosting=iar_breakpoint \
    --cpu=Cortex-M0+
all: $(NAME).out

.INTERMEDIATE: syscfg
$(SYSCFG_FILES): syscfg
	@ echo generation complete

syscfg: ../../freertos/rfPacketErrorRate.syscfg
	@ echo Generating configuration files...
	@ $(SYSCFG_CMD_STUB) --output $(@D) $<

define C_RULE
$(basename $(notdir $(1))).obj: $(1) $(SYSCFG_H_FILES)
	@ echo Building $$@
	@ $(CC) $(CFLAGS) $$< -o $$@
endef

$(foreach c_file,$(SYSCFG_C_FILES),$(eval $(call C_RULE,$(c_file))))

config.obj: ../../config.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

menu.obj: ../../menu.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

rfPacketErrorRate.obj: ../../rfPacketErrorRate.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

rx.obj: ../../rx.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

tx.obj: ../../tx.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

setup_rcl_settings_ble_generic.obj: ../../setup/rcl_settings_ble_generic.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

setup_rcl_settings_msk_250_kbps.obj: ../../setup/rcl_settings_msk_250_kbps.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

setup_rcl_settings_msk_500_kbps.obj: ../../setup/rcl_settings_msk_500_kbps.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

freertos_main_freertos.obj: ../../freertos/main_freertos.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

$(NAME).out: $(OBJECTS)
	@ echo linking $@
	@ $(LNK)  $(OBJECTS)  $(LFLAGS) -o $(NAME).out

clean:
	@ echo Cleaning...
	@ $(RM) $(OBJECTS) > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).out > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).map > $(DEVNULL) 2>&1
	@ $(RM) $(SYSCFG_FILES)> $(DEVNULL) 2>&1
