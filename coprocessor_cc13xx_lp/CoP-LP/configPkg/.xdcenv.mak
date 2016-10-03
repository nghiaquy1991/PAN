#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/packages;C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/tidrivers_cc13xx_cc26xx_2_16_01_13/packages;C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/bios_6_45_02_31/packages;C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/uia_2_00_05_50/packages;C:/ti/ccsv6/ccs_base
override XDCROOT = C:/ti/xdctools_3_32_00_06_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/packages;C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/tidrivers_cc13xx_cc26xx_2_16_01_13/packages;C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/bios_6_45_02_31/packages;C:/ti/tirtos_cc13xx_cc26xx_2_18_01_04/products/uia_2_00_05_50/packages;C:/ti/ccsv6/ccs_base;C:/ti/xdctools_3_32_00_06_core/packages;..
HOSTOS = Windows
endif
