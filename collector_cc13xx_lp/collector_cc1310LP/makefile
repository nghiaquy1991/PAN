################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6

GEN_OPTS__FLAG := --cmd_file="configPkg/compiler.opt"
GEN_CMDS__FLAG := -l"configPkg/linker.cmd"

ORDERED_OBJS += \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/api_mac.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/ccfg.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/cllc.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/collector.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/csf.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/mac_user_config.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/main.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/timer.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/util.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/pwrmon.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/icall.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/icall_cc2650.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/nvoctp.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/CC1310_LAUNCHXL.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/board_key.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/board_lcd.obj" \
"C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP/board_led.obj" \
"../cc1310lp.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251/driverlib/bin/ccs/driverlib.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include Application/subdir_vars.mk
-include HAL/subdir_vars.mk
-include ICall/subdir_vars.mk
-include Services/subdir_vars.mk
-include launchpad/subdir_vars.mk
-include subdir_rules.mk
-include Application/subdir_rules.mk
-include HAL/subdir_rules.mk
-include ICall/subdir_rules.mk
-include Services/subdir_rules.mk
-include launchpad/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
collector_cc13xx_lp.out \

EXE_OUTPUTS__QUOTED += \
"collector_cc13xx_lp.out" \

BIN_OUTPUTS += \
collector_cc13xx_lp.hex \

BIN_OUTPUTS__QUOTED += \
"collector_cc13xx_lp.hex" \


# All Target
all: collector_cc13xx_lp.out secondary-outputs

# Tool invocations
collector_cc13xx_lp.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -O4 --opt_for_speed=0 -g --c99 --define=AUTO_START --define=CC1310_LAUNCHXL --define=TI_DRIVERS_LCD_INCLUDED --define=BOARD_DISPLAY_EXCLUDE_UART --define=MODULE_CC13XX_7X7 --define=SET_CCFG_BL_CONFIG_BL_LEVEL=0x00 --define=SET_CCFG_BL_CONFIG_BL_ENABLE=0xC5 --define=SET_CCFG_BL_CONFIG_BL_PIN_NUMBER=0x0D --define=SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE=0xC5 --define=USE_ICALL --define=HEAPMGR_SIZE=0 --define=FEATURE_MAC_SECURITY --define=FEATURE_GREEN_POWER --define=FEATURE_BEACON_MODE --define=FEATURE_ENHANCED_BEACON --define=FEATURE_ENHANCED_ACK --define=ICALL_HOOK_ABORT_FUNC=halAssertHandler --define=xdc_runtime_Log_DISABLE_ALL --define=xdc_runtime_Assert_DISABLE_ALL --define=NV_RESTORE --diag_warning=225 --display_error_number --diag_wrap=off --obj_directory="C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP" --list_directory="C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP" --asm_directory="C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP" --temp_directory="C:/Users/nghia/workspace/collector_cc13xx_lp/collector_cc1310LP" -z -m"collector_cc13xx_lp.map" --heap_size=0 --stack_size=256 -i"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/lib" -i"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/include" --reread_libs --warn_sections --diag_wrap=off --display_error_number --xml_link_info="collector_cc13xx_lp_linkInfo.xml" --rom_model -o "collector_cc13xx_lp.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

collector_cc13xx_lp.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/bin/armhex" --memwidth=8 --romwidth=8 --diag_suppress=21055-D --diag_suppress=21056-D --intel -o "collector_cc13xx_lp.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)$(GEN_FILES__QUOTED)$(BIN_OUTPUTS__QUOTED)$(GEN_OPTS__QUOTED)$(GEN_CMDS__QUOTED)
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-$(RM) "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\Application\api_mac.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\Application\ccfg.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\Application\cllc.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\Application\collector.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\Application\csf.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\Application\mac_user_config.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\Application\main.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\Application\timer.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\Application\util.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\HAL\pwrmon.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\ICall\icall.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\ICall\icall_cc2650.d" 
	-$(RM) "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\Services\nvoctp.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\launchpad\CC1310_LAUNCHXL.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\launchpad\board_key.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\launchpad\board_lcd.d" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\launchpad\board_led.d" 
	-$(RM) "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\api_mac.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\ccfg.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\cllc.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\collector.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\csf.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\mac_user_config.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\main.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\timer.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\util.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\pwrmon.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\icall.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\icall_cc2650.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\nvoctp.obj" 
	-$(RM) "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\CC1310_LAUNCHXL.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\board_key.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\board_lcd.obj" "C:\Users\nghia\workspace\collector_cc13xx_lp\collector_cc1310LP\board_led.obj" 
	-@echo 'Finished clean'
	-@echo ' '

secondary-outputs: $(BIN_OUTPUTS)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

