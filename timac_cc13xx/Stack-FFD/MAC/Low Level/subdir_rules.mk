################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
C:/Users/nghia/workspace/timac_cc13xx/Stack-FFD/mac_settings.obj: C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/low_level/cc13xx/mac_settings.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -O4 --opt_for_speed=0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/include" --include_path="C:/ti/xdctools_3_32_00_06_core/packages" --include_path="C:/Users/nghia/Desktop/timac/timac_cc13xx" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/bios_6_45_02_31/packages" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/bios_6_45_02_31/packages/ti/targets/arm/rtsarm/package/lib/lib/ti.targets.arm.rtsarm" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/stack/src" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/stack/cc13xx/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/stack/cc13xx/inc/ffd" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/boards" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/examples/common/boards/CC1310_LAUNCHXL" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/_common" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/_common/cc13xx" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/hal/src/target/cc2650tirtos" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/icall/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/osal/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/saddr" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/sdata" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/aes" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/common/services/src/appasrt" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/high_level" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/inc" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/fh" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/inc/cc13xx" --include_path="C:/ti/simplelink/ti-15.4-stack-sdk_2_00_00_25/components/core/src/low_level/cc13xx" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251/inc" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/cc13xxware_2_03_03_17251/driverlib" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/tidrivers_cc13xx_cc26xx_2_16_01_13/packages" -g --c99 --define=MAC_PNM_MAX_NUMBER_OF_NODE=50 --define=MAX_DEVICE_TABLE_ENTRIES=50 --define=NO_OSAL_SNV --define=RCN_APP_ASSERT --define=FEATURE_MAC_SECURITY --define=FEATURE_GREEN_POWER --define=FEATURE_BEACON_MODE --define=FEATURE_ENHANCED_BEACON --define=FEATURE_ENHANCED_ACK --define=DRIVERLIB_NOROM --define=USE_ICALL --define=HAL_ASSERT_SPIN --define=xHALNODEBUG --define=FEATURE_SYSTEM_STATS --define=FH_DH1CF --diag_warning=225 --display_error_number --diag_wrap=off --obj_directory="C:/Users/nghia/workspace/timac_cc13xx/Stack-FFD" --list_directory="C:/Users/nghia/workspace/timac_cc13xx/Stack-FFD" --asm_directory="C:/Users/nghia/workspace/timac_cc13xx/Stack-FFD" --temp_directory="C:/Users/nghia/workspace/timac_cc13xx/Stack-FFD" --preproc_with_compile --preproc_dependency="C:/Users/nghia/workspace/timac_cc13xx/Stack-FFD/MAC/Low Level/mac_settings.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


