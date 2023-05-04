COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR ?= $(abspath ../../../../../../..)

include $(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/imports.mak

CC = "$(TICLANG_ARMCOMPILER)/bin/tiarmclang"
LNK = "$(TICLANG_ARMCOMPILER)/bin/tiarmclang"

SYSCFG_CMD_STUB = $(SYSCONFIG_TOOL) --compiler ticlang --product $(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/.metadata/product.json
SYSCFG_FILES := $(shell $(SYSCFG_CMD_STUB) --listGeneratedFiles --listReferencedFiles --output . ../../freertos/rfCarrierWave.syscfg)

SYSCFG_C_FILES = $(filter %.c,$(SYSCFG_FILES))
SYSCFG_H_FILES = $(filter %.h,$(SYSCFG_FILES))

OBJECTS = rfCarrierWave.obj setup_rcl_settings_ble_generic.obj setup_rcl_settings_msk_250_kbps.obj setup_rcl_settings_msk_500_kbps.obj freertos_main_freertos.obj $(patsubst %.c,%.obj,$(notdir $(SYSCFG_C_FILES)))
NAME = rfCarrierWave

CFLAGS = -I../.. \
    -I. \
    -DVARIABLE_LENGTH_SETUP \
    -DDeviceFamily_CC23X0R5 \
    "-I$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source" \
    "-I$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/kernel/freertos" \
    "-I$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/posix/ticlang" \
    "-I$(FREERTOS_INSTALL_DIR)/FreeRTOS/Source/include" \
    "-I$(FREERTOS_INSTALL_DIR)/FreeRTOS/Source/portable/GCC/ARM_CM0" \
    -gdwarf-3 \
    -mcpu=cortex-m0plus \
    -march=thumbv6m \
    -mfloat-abi=soft \
    -mthumb

LFLAGS = "-L$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source" \
    "-L$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)source/ti/devices/cc23x0r5/rf_patches/lib/ticlang/lrf_cc23x0r5.a" \
    ti_utils_build_linker.cmd.genlibs \
    -lti/drivers/rcl/lib/ticlang/m0p/rcl_cc23x0r5.a \
    -lti/devices/cc23x0r5/driverlib/lib/ticlang/driverlib.a \
    ../../freertos/ticlang/cc23x0r5_freertos.cmd \
    "-Wl,-m,$(NAME).map" \
    -Wl,--rom_model \
    -Wl,--warn_sections \
    "-L$(TICLANG_ARMCOMPILER)/lib" \
    -llibc.a
all: $(NAME).out

.INTERMEDIATE: syscfg
$(SYSCFG_FILES): syscfg
	@ echo generation complete

syscfg: ../../freertos/rfCarrierWave.syscfg
	@ echo Generating configuration files...
	@ $(SYSCFG_CMD_STUB) --output $(@D) $<

define C_RULE
$(basename $(notdir $(1))).obj: $(1) $(SYSCFG_H_FILES)
	@ echo Building $$@
	@ $(CC) $(CFLAGS) -c $$< -o $$@
endef

$(foreach c_file,$(SYSCFG_C_FILES),$(eval $(call C_RULE,$(c_file))))

rfCarrierWave.obj: ../../rfCarrierWave.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

setup_rcl_settings_ble_generic.obj: ../../setup/rcl_settings_ble_generic.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

setup_rcl_settings_msk_250_kbps.obj: ../../setup/rcl_settings_msk_250_kbps.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

setup_rcl_settings_msk_500_kbps.obj: ../../setup/rcl_settings_msk_500_kbps.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

freertos_main_freertos.obj: ../../freertos/main_freertos.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

$(NAME).out: $(OBJECTS)
	@ echo linking $@
	@ $(LNK) -Wl,-u,_c_int00 $(OBJECTS)  $(LFLAGS) -o $(NAME).out

clean:
	@ echo Cleaning...
	@ $(RM) $(OBJECTS) > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).out > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).map > $(DEVNULL) 2>&1
	@ $(RM) $(SYSCFG_FILES)> $(DEVNULL) 2>&1
