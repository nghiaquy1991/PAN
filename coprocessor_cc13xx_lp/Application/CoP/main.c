/******************************************************************************

 @file  main.c

 @brief Main entry of the MAC-CoProcessor application

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
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Error.h>

#include "icall.h"
#include "board.h"

#include <inc/hw_ccfg.h>
#include <inc/hw_ccfg_simple_struct.h>

/* Required for the idle task function */
#include <ti/drivers/power/PowerCC26XX.h>

/* Required to enable instruction fetch cache */
#include <vims.h>

#ifdef NV_RESTORE
#include "macconfig.h"
#include "nvoctp.h"
#endif

#include <string.h>

#include "api_mac.h"
#include "mt_sys.h"
#include "mcp.h"

#ifndef USE_DEFAULT_USER_CFG
#include "_hal_types.h"
#include "macs.h"
/* MAC user defined configuration */
macUserCfg_t user0Cfg = MAC_USER_CFG;
#endif /* USE_DEFAULT_USER_CFG */

/******************************************************************************
 Constants
 *****************************************************************************/
/*! Extended Address offset in FCFG (LSB..MSB) */
#define EXTADDR_OFFSET 0x2F0

/*! Memory location of cutomer-configured IEEE address */
#define CCFG_IEEE ((uint8_t *)&(__ccfg.CCFG_IEEE_MAC_0))

/*! Memory location of unique factory-programmed IEEE address */
#define PRIM_IEEE ((uint8_t *)(FCFG1_BASE + EXTADDR_OFFSET))

/*! Size of stack for MNP application */
#define APP_TASK_STACK_SIZE  600

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
mac_Config_t Main_user1Cfg = {0};
#endif

/******************************************************************************
 Local Variables
 *****************************************************************************/
/*! Used to check for a valid extended address */
static const uint8_t dummyExtAddr[] =
{
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/******************************************************************************
 Local Function Prototypes
 *****************************************************************************/
static void taskFxn(UArg a0, UArg a1);

/*! MAC assert handler */
static void macAssertHandler(void);

/*! USER assert handler */
static void userAssertHandler(uint8_t reason);

/******************************************************************************
 Public Functions
 *****************************************************************************/
/*!
 * @brief  "main()" function - starting point
 */
void main()
{
    Task_Params taskParams;

#ifndef USE_DEFAULT_USER_CFG
    /* Register our MAC assert handler */
    user0Cfg.pAssertFP = macAssertHandler;
#endif

    /* Enable iCache prefetching */
    VIMSConfigure(VIMS_BASE, TRUE, TRUE);

    /* Enable cache */
    VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);

    /*
     Initialization for board related stuff, such as LEDs
     following TI-RTOS convention
     */
    PIN_init(BoardGpioInitTable);

    /* Configure task */
    Task_Params_init(&taskParams);
    taskParams.stack = myTaskStack;
    taskParams.stackSize = APP_TASK_STACK_SIZE;
    taskParams.priority = 1;
    Task_construct(&myTask, taskFxn, &taskParams, NULL);

    BIOS_start(); /* enable interrupts and start SYS/BIOS */
}

/*!
 * @brief  TIRTOS HWI Handler. The name of this function is set to
 *         M3Hwi.excHandlerFunc in app.cfg, you can disable this by
 *         setting it to NULL.
 *
 * @param  excStack - TIROS variable
 * @param  lr - TIROS variable
 */
xdc_Void Main_excHandler(UInt *excStack, UInt lr)
{
    /* Intentionally not used */
    (void)excStack;
    (void)lr;

    /* Go to user-defined handler */
    userAssertHandler(MTSYS_ASSERT_RTOS);
}

/*!
 * @brief  HAL assert handler required by OSAL memory module.
 */
void Main_halAssertHandler(void)
{
    /* Go to user-defined handler */
    userAssertHandler(MTSYS_ASSERT_HAL);
}

/******************************************************************************
 Local Functions
 *****************************************************************************/
/*!
 * @brief  Function to initialize and run main applicatiion task
 *
 * @param  a0 - TIROS variable
 * @param  a1 - TIROS variable
 */
static void taskFxn(UArg a0, UArg a1)
{
    /* Intentionally not used */
    (void)a0;
    (void)a1;

    /* Disallow shutting down JTAG, VIMS, SYSBUS during idle state
     * since TIMAC requires SYSBUS during idle. */
    Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);

    /* Initialize ICall module */
    ICall_init();

    /*
     * Copy the extended address from the CCFG area in flash memory
     * Assumption: CCFG_IEEE_MAC_0 and CCFG_IEEE_MAC_1 are contiguous
     * 4-byte locations with data stored first.
     */
    memcpy(ApiMac_extAddr, CCFG_IEEE, APIMAC_SADDR_EXT_LEN);

    /* Check to see if the CCFG IEEE is valid */
    if(memcmp(ApiMac_extAddr, dummyExtAddr, APIMAC_SADDR_EXT_LEN) == 0)
    {
        /* Nothing at CCFG IEEE location, get the Primary IEEE */
        memcpy(ApiMac_extAddr, PRIM_IEEE, APIMAC_SADDR_EXT_LEN);
    }

#ifdef NV_RESTORE
    /* Setup the NV driver */
    NVOCTP_loadApiPtrs( &Main_user1Cfg.nvFps );

    if (Main_user1Cfg.nvFps.initNV)
    {
        Main_user1Cfg.nvFps.initNV(NULL);
    }
#endif

    /* Start tasks of external images */
    ICall_createRemoteTasks();

    /* Kick off co-processor application task */
    MCP_task();
}

/*!
 * @brief  MAC assert handler required by MAC Stack process.
 */
static void macAssertHandler(void)
{
    /* Go to user-defined handler */
    userAssertHandler(MTSYS_ASSERT_MAC);
}

/*!
 * @brief  User-defined assert handler
 *
 * @param  Reason for the assertion
 */
static void userAssertHandler(uint8_t reason)
{
    while(1)
    {
        /* Record assert reason, then reset device */
        MtSys_resetReq(reason);
    }
}
