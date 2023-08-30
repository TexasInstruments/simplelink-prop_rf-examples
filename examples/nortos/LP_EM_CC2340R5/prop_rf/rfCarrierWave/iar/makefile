COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR ?= $(abspath ../../../../../..)

include $(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/imports.mak

CC = "$(IAR_ARMCOMPILER)/bin/iccarm"
LNK = "$(IAR_ARMCOMPILER)/bin/ilinkarm"

SYSCFG_CMD_STUB = $(SYSCONFIG_TOOL) --compiler iar --product $(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/.metadata/product.json
SYSCFG_FILES := $(shell $(SYSCFG_CMD_STUB) --listGeneratedFiles --listReferencedFiles --output . ../rfCarrierWave.syscfg)

SYSCFG_C_FILES = $(filter %.c,$(SYSCFG_FILES))
SYSCFG_H_FILES = $(filter %.h,$(SYSCFG_FILES))
SYSCFG_OPT_FILES = $(filter %.opt,$(SYSCFG_FILES))

OBJECTS = rfCarrierWave.obj setup_rcl_settings_ble_generic.obj setup_rcl_settings_msk_250_kbps.obj setup_rcl_settings_msk_250_kbps_fec.obj main_nortos.obj $(patsubst %.c,%.obj,$(notdir $(SYSCFG_C_FILES)))
NAME = rfCarrierWave

CFLAGS += -I.. \
    -I. \
    $(addprefix -f,$(SYSCFG_OPT_FILES)) \
    -DVARIABLE_LENGTH_SETUP \
    "-I$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source" \
    "-I$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/kernel/nortos" \
    "-I$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/kernel/nortos/posix" \
    --debug \
    --silent \
    -e \
    --aeabi \
    --thumb \
    --diag_suppress=Pa050 \
    --cpu=Cortex-M0+ \
    --vla

LFLAGS += "-L$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source" \
    "-L$(COM_TI_SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/kernel/nortos" \
    -fti_utils_build_linker.cmd.genlibs \
    --config ../iar/cc23x0r5_nortos.icf \
    --map "$(NAME).map" \
    --silent \
    --semihosting=iar_breakpoint \
    --cpu=Cortex-M0+

all: $(NAME).out

.INTERMEDIATE: syscfg
$(SYSCFG_FILES): syscfg
	@ echo generation complete

syscfg: ../rfCarrierWave.syscfg
	@ echo Generating configuration files...
	@ $(SYSCFG_CMD_STUB) --output $(@D) $<

define C_RULE
$(basename $(notdir $(1))).obj: $(1) $(SYSCFG_H_FILES)
	@ echo Building $$@
	@ $(CC) $(CFLAGS) "$$<" -o $$@
endef

$(foreach c_file,$(SYSCFG_C_FILES),$(eval $(call C_RULE,$(c_file))))

rfCarrierWave.obj: ../rfCarrierWave.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

setup_rcl_settings_ble_generic.obj: ../setup/rcl_settings_ble_generic.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

setup_rcl_settings_msk_250_kbps.obj: ../setup/rcl_settings_msk_250_kbps.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

setup_rcl_settings_msk_250_kbps_fec.obj: ../setup/rcl_settings_msk_250_kbps_fec.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

main_nortos.obj: ../main_nortos.c $(SYSCFG_H_FILES)
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
