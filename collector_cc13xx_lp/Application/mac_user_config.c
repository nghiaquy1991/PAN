/******************************************************************************

 @file  mac_user_config.c

 @brief User configurable variables for the TIMAC radio.

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
 * INCLUDES
 */

#if defined(CC1310_LAUNCHXL)
#include "CC1310_LAUNCHXL/Board.h"
#else
#include "CC1310DK_7XD/Board.h"
#endif
#include "mac_user_config.h"
#include <ti/drivers/crypto/CryptoCC26XX.h>

/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS
 */

// Tx Power
#define NUM_TX_POWER_VALUES (sizeof( txPowerTable ) / sizeof( txPwrVal_t ))

/******************************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */

/******************************************************************************
 * GLOBAL VARIABLES
 */

//
// RF patch pointers
//
const RF_Mode rfPropTable =
{
  .rfMode = RADIO_MODE_PROPRIETARY,
  .cpePatchFxn = &rf_patch_cpe_genfsk,
  .mcePatchFxn = 0,
  .rfePatchFxn = &rf_patch_rfe_genfsk,
};

//
// Tx Power Table Used Depends on Device Package
//

#if defined(CC1310EM_7XD_7793) || defined(CC1310EM_7ID)

// Tx Power Values (Pout, TC, GC, IB)
const txPwrVal_t txPowerTable[] =
  { { TX_POWER_14_DBM,       TX_POUT( 0xA7, 0, 0x3F ) },
    { TX_POWER_12_DBM,       TX_POUT( 0xB8, 0, 0x18 ) },
    { TX_POWER_11_DBM,       TX_POUT( 0x50, 3, 0x1A ) },
    { TX_POWER_10_DBM,       TX_POUT( 0x38, 3, 0x13 ) },
    { TX_POWER_9_DBM,        TX_POUT( 0x2C, 3, 0x0D ) },
    { TX_POWER_8_DBM,        TX_POUT( 0x24, 3, 0x0B ) },
    { TX_POWER_7_DBM,        TX_POUT( 0x20, 3, 0x09 ) },
    { TX_POWER_6_DBM,        TX_POUT( 0x1C, 3, 0x07 ) },
    { TX_POWER_5_DBM,        TX_POUT( 0x18, 3, 0x06 ) },
    { TX_POWER_4_DBM,        TX_POUT( 0x18, 3, 0x05 ) },
    { TX_POWER_3_DBM,        TX_POUT( 0x14, 3, 0x04 ) },
    { TX_POWER_2_DBM,        TX_POUT( 0x10, 1, 0x02 ) },
    { TX_POWER_1_DBM,        TX_POUT( 0x10, 3, 0x03 ) },
    { TX_POWER_0_DBM,        TX_POUT( 0x00, 1, 0x01 ) },
    { TX_POWER_MINUS_10_DBM, TX_POUT( 0x08, 3, 0x00 ) } };

#else // unknown device package

#error "***MAC USER CONFIG BUILD ERROR*** Unknown package type!"

#endif // CC1310EM_7XD_7793

// Tx Power Table
const txPwrTbl_t txPwrTbl = { txPowerTable,
                              NUM_TX_POWER_VALUES };

// RF Driver API Table
const uint32_t rfDriverTable[] =
  { (uint32_t) RF_open,
    (uint32_t) RF_close,
    (uint32_t) RF_postCmd,
    (uint32_t) RF_pendCmd,
    (uint32_t) RF_runCmd,
    (uint32_t) RF_cancelCmd,
    (uint32_t) RF_flushCmd,
    (uint32_t) RF_yield,
    (uint32_t) RF_Params_init,
    (uint32_t) RF_runImmediateCmd,
    (uint32_t) RF_runDirectCmd,
    (uint32_t) RF_ratCompare,
    (uint32_t) RF_ratCapture,
    (uint32_t) RF_ratHwOutput,
    (uint32_t) RF_ratDisableChannel,
    (uint32_t) RF_getCurrentTime,
    (uint32_t) RF_getRssi,
    (uint32_t) RF_getInfo,
    (uint32_t) RF_getCmdOp };

// Crypto Driver API Table
const uint32_t cryptoDriverTable[] =
  { (uint32_t) CryptoCC26XX_close,
    (uint32_t) CryptoCC26XX_init,
    (uint32_t) CryptoCC26XX_open,
    (uint32_t) CryptoCC26XX_Params_init,
    (uint32_t) CryptoCC26XX_Transac_init,
    (uint32_t) CryptoCC26XX_allocateKey,
    (uint32_t) CryptoCC26XX_releaseKey,
    (uint32_t) CryptoCC26XX_transact,
    (uint32_t) CryptoCC26XX_transactPolling,
    (uint32_t) CryptoCC26XX_transactCallback };

/******************************************************************************
 */

