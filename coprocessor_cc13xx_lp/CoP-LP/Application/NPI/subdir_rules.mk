################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP/npi_frame_mt.obj: C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src/npi_frame_mt.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -O4 --opt_for_speed=0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/include" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/coprocessor/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/coprocessor/src/mt" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/api/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/util" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/cc13xx/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/cc13xx/inc/ffd" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/heapmgr" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/_common" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/_common/cc13xx" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/cc2650tirtos" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/icall/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/osal/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/nv" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/nv/cc26xx" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/saddr" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/sdata" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/stack/src/icall" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/fh" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/inc/cc13xx" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251/inc" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251/driverlib" -g --c99 --undefine=NPI_SREQRSP --define=CC1310_LAUNCHXL --define=MODULE_CC13XX_7X7 --define=SET_CCFG_BL_CONFIG_BL_LEVEL=0x00 --define=SET_CCFG_BL_CONFIG_BL_ENABLE=0xC5 --define=SET_CCFG_BL_CONFIG_BL_PIN_NUMBER=0x0D --define=SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE=0xC5 --define=xTI_DRIVERS_LCD_INCLUDED --define=BOARD_LED_BLINK_PERIOD=50 --define=HEAPMGR_SIZE=0 --define=FEATURE_MAC_SECURITY --define=FEATURE_GREEN_POWER --define=FEATURE_BEACON_MODE --define=FEATURE_ENHANCED_BEACON --define=FEATURE_ENHANCED_ACK --define=NPI_USE_UART --define=USE_ICALL --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=8 --define=ICALL_HOOK_ABORT_FUNC=Main_halAssertHandler --define=xdc_runtime_Log_DISABLE_ALL --define=xdc_runtime_Assert_DISABLE_ALL --define=NV_RESTORE --verbose --diag_warning=225 --display_error_number --diag_wrap=off --obj_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --list_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --asm_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --temp_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --preproc_with_compile --preproc_dependency="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP/Application/NPI/npi_frame_mt.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP/npi_rxbuf.obj: C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src/npi_rxbuf.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -O4 --opt_for_speed=0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/include" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/coprocessor/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/coprocessor/src/mt" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/api/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/util" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/cc13xx/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/cc13xx/inc/ffd" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/heapmgr" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/_common" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/_common/cc13xx" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/cc2650tirtos" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/icall/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/osal/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/nv" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/nv/cc26xx" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/saddr" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/sdata" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/stack/src/icall" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/fh" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/inc/cc13xx" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251/inc" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251/driverlib" -g --c99 --undefine=NPI_SREQRSP --define=CC1310_LAUNCHXL --define=MODULE_CC13XX_7X7 --define=SET_CCFG_BL_CONFIG_BL_LEVEL=0x00 --define=SET_CCFG_BL_CONFIG_BL_ENABLE=0xC5 --define=SET_CCFG_BL_CONFIG_BL_PIN_NUMBER=0x0D --define=SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE=0xC5 --define=xTI_DRIVERS_LCD_INCLUDED --define=BOARD_LED_BLINK_PERIOD=50 --define=HEAPMGR_SIZE=0 --define=FEATURE_MAC_SECURITY --define=FEATURE_GREEN_POWER --define=FEATURE_BEACON_MODE --define=FEATURE_ENHANCED_BEACON --define=FEATURE_ENHANCED_ACK --define=NPI_USE_UART --define=USE_ICALL --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=8 --define=ICALL_HOOK_ABORT_FUNC=Main_halAssertHandler --define=xdc_runtime_Log_DISABLE_ALL --define=xdc_runtime_Assert_DISABLE_ALL --define=NV_RESTORE --verbose --diag_warning=225 --display_error_number --diag_wrap=off --obj_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --list_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --asm_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --temp_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --preproc_with_compile --preproc_dependency="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP/Application/NPI/npi_rxbuf.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP/npi_task.obj: C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src/npi_task.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -O4 --opt_for_speed=0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/include" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/coprocessor/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/coprocessor/src/mt" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/api/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/util" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/cc13xx/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/cc13xx/inc/ffd" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/heapmgr" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/_common" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/_common/cc13xx" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/cc2650tirtos" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/icall/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/osal/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/nv" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/nv/cc26xx" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/saddr" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/sdata" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/stack/src/icall" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/fh" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/inc/cc13xx" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251/inc" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251/driverlib" -g --c99 --undefine=NPI_SREQRSP --define=CC1310_LAUNCHXL --define=MODULE_CC13XX_7X7 --define=SET_CCFG_BL_CONFIG_BL_LEVEL=0x00 --define=SET_CCFG_BL_CONFIG_BL_ENABLE=0xC5 --define=SET_CCFG_BL_CONFIG_BL_PIN_NUMBER=0x0D --define=SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE=0xC5 --define=xTI_DRIVERS_LCD_INCLUDED --define=BOARD_LED_BLINK_PERIOD=50 --define=HEAPMGR_SIZE=0 --define=FEATURE_MAC_SECURITY --define=FEATURE_GREEN_POWER --define=FEATURE_BEACON_MODE --define=FEATURE_ENHANCED_BEACON --define=FEATURE_ENHANCED_ACK --define=NPI_USE_UART --define=USE_ICALL --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=8 --define=ICALL_HOOK_ABORT_FUNC=Main_halAssertHandler --define=xdc_runtime_Log_DISABLE_ALL --define=xdc_runtime_Assert_DISABLE_ALL --define=NV_RESTORE --verbose --diag_warning=225 --display_error_number --diag_wrap=off --obj_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --list_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --asm_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --temp_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --preproc_with_compile --preproc_dependency="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP/Application/NPI/npi_task.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP/npi_tl.obj: C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src/npi_tl.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -O4 --opt_for_speed=0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/include" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/coprocessor/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/coprocessor/src/mt" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/api/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/util" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/cc13xx/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/cc13xx/inc/ffd" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/heapmgr" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/_common" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/_common/cc13xx" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/cc2650tirtos" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/icall/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/osal/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/nv" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/nv/cc26xx" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/saddr" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/sdata" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/stack/src/icall" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/fh" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/inc/cc13xx" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251/inc" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251/driverlib" -g --c99 --undefine=NPI_SREQRSP --define=CC1310_LAUNCHXL --define=MODULE_CC13XX_7X7 --define=SET_CCFG_BL_CONFIG_BL_LEVEL=0x00 --define=SET_CCFG_BL_CONFIG_BL_ENABLE=0xC5 --define=SET_CCFG_BL_CONFIG_BL_PIN_NUMBER=0x0D --define=SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE=0xC5 --define=xTI_DRIVERS_LCD_INCLUDED --define=BOARD_LED_BLINK_PERIOD=50 --define=HEAPMGR_SIZE=0 --define=FEATURE_MAC_SECURITY --define=FEATURE_GREEN_POWER --define=FEATURE_BEACON_MODE --define=FEATURE_ENHANCED_BEACON --define=FEATURE_ENHANCED_ACK --define=NPI_USE_UART --define=USE_ICALL --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=8 --define=ICALL_HOOK_ABORT_FUNC=Main_halAssertHandler --define=xdc_runtime_Log_DISABLE_ALL --define=xdc_runtime_Assert_DISABLE_ALL --define=NV_RESTORE --verbose --diag_warning=225 --display_error_number --diag_wrap=off --obj_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --list_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --asm_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --temp_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --preproc_with_compile --preproc_dependency="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP/Application/NPI/npi_tl.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP/npi_tl_uart.obj: C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src/npi_tl_uart.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -O4 --opt_for_speed=0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/include" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/coprocessor/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/coprocessor/src/mt" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/api/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/util" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/cc13xx/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/Stack/cc13xx/inc/ffd" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/heapmgr" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/_common" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/_common/cc13xx" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/cc2650tirtos" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/icall/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/npi/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/osal/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/nv" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/nv/cc26xx" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/saddr" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/sdata" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/stack/src/icall" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/fh" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/inc/cc13xx" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251/inc" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251/driverlib" -g --c99 --undefine=NPI_SREQRSP --define=CC1310_LAUNCHXL --define=MODULE_CC13XX_7X7 --define=SET_CCFG_BL_CONFIG_BL_LEVEL=0x00 --define=SET_CCFG_BL_CONFIG_BL_ENABLE=0xC5 --define=SET_CCFG_BL_CONFIG_BL_PIN_NUMBER=0x0D --define=SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE=0xC5 --define=xTI_DRIVERS_LCD_INCLUDED --define=BOARD_LED_BLINK_PERIOD=50 --define=HEAPMGR_SIZE=0 --define=FEATURE_MAC_SECURITY --define=FEATURE_GREEN_POWER --define=FEATURE_BEACON_MODE --define=FEATURE_ENHANCED_BEACON --define=FEATURE_ENHANCED_ACK --define=NPI_USE_UART --define=USE_ICALL --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=8 --define=ICALL_HOOK_ABORT_FUNC=Main_halAssertHandler --define=xdc_runtime_Log_DISABLE_ALL --define=xdc_runtime_Assert_DISABLE_ALL --define=NV_RESTORE --verbose --diag_warning=225 --display_error_number --diag_wrap=off --obj_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --list_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --asm_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --temp_directory="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP" --preproc_with_compile --preproc_dependency="C:/Users/nghia/workspace/coprocessor_cc13xx_lp/CoP-LP/Application/NPI/npi_tl_uart.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


