/******************************************************************************

 @file main.c

 @brief main entry of the example application

 Group: WCS LPC
 Target Device: CC13xx

 ******************************************************************************
 
 Copyright (c) 2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ti-15.4-stack-sdk_2_00_00_25
 Release Date: 2016-07-14 14:37:14
 *****************************************************************************/

/******************************************************************************
 Includes
 *****************************************************************************/

#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <ioc.h>

#include "sys_ctrl.h"

#include "board.h"
#include "timac_board.h"
#include <inc/hw_ccfg.h>
#include <inc/hw_ccfg_simple_struct.h>

/* Header files required for the temporary idle task function */
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <aon_rtc.h>
#include <prcm.h>

/* Header files required to enable instruction fetch cache */
#include <vims.h>
#include <hw_memmap.h>

#include "ti/sysbios/family/arm/m3/Hwi.h"


#include "cpu.h"

#ifdef NV_RESTORE
#include "macconfig.h"
#include "nvoctp.h"
#endif

#include <string.h>

#include "api_mac.h"
#include "ssf.h"

#include "sensor.h"
#if defined(ASSERT_LEDS)
#include "board_led.h"
#endif

#ifndef USE_DEFAULT_USER_CFG

#include "boards/mac_user_config.h"

/* MAC user defined configuration */
macUserCfg_t user0Cfg = MAC_USER_CFG;

#endif /* USE_DEFAULT_USER_CFG */

/******************************************************************************
 Constants
 *****************************************************************************/

/* Assert Reasons */
#define MAIN_ASSERT_HWI_TIRTOS   1
#define MAIN_ASSERT_ICALL        2
#define MAIN_ASSERT_MAC          3

#define MAX_ASSERT_TOGGLE_COUNT  500000

#define RFC_MODE_BLE                 PRCM_RFCMODESEL_CURR_MODE1
#define RFC_MODE_IEEE                PRCM_RFCMODESEL_CURR_MODE2
#define RFC_MODE_ANT                 PRCM_RFCMODESEL_CURR_MODE4
#define RFC_MODE_EVERYTHING_BUT_ANT  PRCM_RFCMODESEL_CURR_MODE5
#define RFC_MODE_EVERYTHING          PRCM_RFCMODESEL_CURR_MODE6

/* Extended Address offset in FCFG (LSB..MSB) */
#define EXTADDR_OFFSET 0x2F0

#define APP_TASK_STACK_SIZE 780

#define SET_RFC_MODE(mode) HWREG( PRCM_BASE + PRCM_O_RFCMODESEL ) = (mode)

/******************************************************************************
 External Variables
 *****************************************************************************/

extern ApiMac_sAddrExt_t ApiMac_extAddr;

/******************************************************************************
 Global Variables
 *****************************************************************************/

Task_Struct myTask;
Char myTaskStack[APP_TASK_STACK_SIZE];

#ifdef NV_RESTORE
mac_Config_t Main_user1Cfg = { 0 };
#endif

/******************************************************************************
 Local Variables
 *****************************************************************************/
/* Used to check for a valid extended address */
static const uint8_t dummyExtAddr[] =
    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/*!
 * @brief       Fill in your own assert function.
 *
 * @param       assertReason - reason: MAIN_ASSERT_HWI_TIRTOS,
 *                                     MAIN_ASSERT_ICALL, or
 *                                     MAIN_ASSERT_MAC
 */
void Main_assertHandler(uint8 assertReason)
{
#if defined(ASSERT_LEDS)
	int toggleCount = 0;
	bool toggle = true;

    Hwi_disable();

    while(1)
    {
    	if(toggleCount == 0)
    	{
    		if(toggle == false)
    		{
    			Board_Led_control(board_led_type_LED1, board_led_state_OFF);
    			Board_Led_control(board_led_type_LED2, board_led_state_OFF);
#if !defined(CC1310_LAUNCHXL)
    			Board_Led_control(board_led_type_LED3, board_led_state_OFF);
    			Board_Led_control(board_led_type_LED4, board_led_state_OFF);
#endif
    		}
    		else if(toggle == true)
    		{
    			Board_Led_control(board_led_type_LED1, board_led_state_ON);
    			Board_Led_control(board_led_type_LED2, board_led_state_ON);
#if !defined(CC1310_LAUNCHXL)
    			Board_Led_control(board_led_type_LED3, board_led_state_ON);
    			Board_Led_control(board_led_type_LED4, board_led_state_ON);
#endif
    		}
    	}

    	toggleCount++;
    	if(toggleCount >= MAX_ASSERT_TOGGLE_COUNT)
    	{
    		toggleCount = 0;
    		if(toggle == true)
    		{
    			toggle = false;
    		}
    		else
    		{
    			toggle = true;
    		}
    	}
    }
#else  /* ASSERT_LEDS */
    Ssf_assertInd(assertReason);

    /* Pull the plug and start over */
    SysCtrlSystemReset();
#endif /* !ASSERT_LEDS */
}


/*!
 * @brief       Main task function
 *
 * @param       a0 -
 * @param       a1 -
 */
Void taskFxn(UArg a0, UArg a1)
{
    /* Disallow shutting down JTAG, VIMS, SYSBUS during idle state
     * since TIMAC requires SYSBUS during idle. */
    Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);

    /* Initialize ICall module */
    ICall_init();

#ifdef FEATURE_MAC_SECURITY
    /*
     * Copy the extended address from the CCFG area
     * Assumption: the memory in CCFG_IEEE_MAC_0 and CCFG_IEEE_MAC_1
     * is contiguous and LSB first.
     */
    memcpy(ApiMac_extAddr, (uint8_t *)&(__ccfg.CCFG_IEEE_MAC_0),
           (APIMAC_SADDR_EXT_LEN));

    /* Check to see if the CCFG IEEE is valid */
    if(memcmp(ApiMac_extAddr, dummyExtAddr, APIMAC_SADDR_EXT_LEN) == 0)
    {
        /* No, it isn't valid.  Get the Primary IEEE Address */
        memcpy(ApiMac_extAddr, (uint8_t *)(FCFG1_BASE + EXTADDR_OFFSET),
               (APIMAC_SADDR_EXT_LEN));
    }
#endif


#ifdef NV_RESTORE
    /* Setup the NV driver */
    NVOCTP_loadApiPtrs(&Main_user1Cfg.nvFps);

    if(Main_user1Cfg.nvFps.initNV)
    {
        Main_user1Cfg.nvFps.initNV( NULL);
    }
#endif

    /* Start tasks of external images */
    ICall_createRemoteTasks();

    /* Initialize the application */
    Sensor_init();

    /* Kick off application - Forever loop */
    while(1)
    {
        Sensor_process();
    }
}

/*!
 * @brief       TIRTOS HWI Handler.  The name of this function is set to
 *              M3Hwi.excHandlerFunc in app.cfg, you can disable this by
 *              setting it to null.
 *
 * @param       excStack - TIROS variable
 * @param       lr - TIROS variable
 */
xdc_Void Main_excHandler(UInt *excStack, UInt lr)
{
    /* User defined function */
    Main_assertHandler(MAIN_ASSERT_HWI_TIRTOS);
}

/*!
 * @brief       HAL assert handler required by OSAL memory module.
 */
void halAssertHandler(void)
{
    /* User defined function */
    Main_assertHandler(MAIN_ASSERT_ICALL);
}

/*!
 * @brief       MAC HAL assert handler.
 */
void macHalAssertHandler(void)
{
    /* User defined function */
    Main_assertHandler(MAIN_ASSERT_MAC);
}

/*!
 * @brief       "main()" function - starting point
 */
Void main()
{
    Task_Params taskParams;

#ifndef USE_DEFAULT_USER_CFG
    user0Cfg.pAssertFP = macHalAssertHandler;
#endif

    /* enable iCache prefetching */
    VIMSConfigure(VIMS_BASE, TRUE, TRUE);

    /* Enable cache */
    VIMSModeSet( VIMS_BASE, VIMS_MODE_ENABLED);

    CPU_WriteBufferDisable();

    /*
     Initialization for board related stuff such as LEDs
     following TI-RTOS convention
     */
    PIN_init(BoardGpioInitTable);

    /* Configure task. */
    Task_Params_init(&taskParams);
    taskParams.stack = myTaskStack;
    taskParams.stackSize = APP_TASK_STACK_SIZE;
    taskParams.priority = 1;
    Task_construct(&myTask, taskFxn, &taskParams, NULL);

#ifdef DEBUG_SW_TRACE
    IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT
                    | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);
#endif /* DEBUG_SW_TRACE */

    BIOS_start(); /* enable interrupts and start SYS/BIOS */
}

