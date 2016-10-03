/******************************************************************************

 @file fh_nt.c

 @brief TIMAC 2.0 FH neighbor table API

 Group: WCS LPC
 Target Device: CC13xx

 ******************************************************************************
 
 Copyright (c) 2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ti-15.4-stack-sdk_2_00_00_25
 Release Date: 2016-07-14 14:37:14
 *****************************************************************************/

/******************************************************************************
 Includes
 *****************************************************************************/

#include <string.h>
#include "fh_api.h"
#include "fh_nt.h"
#include "fh_ie.h"
#include "fh_util.h"
#include "fh_mgr.h"

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

#define CLOCK_LT(a,b)                   ((signed long)((a)-(b)) < 0)
#define NODE_TO_PURGE                   (0)
#define NODE_TO_KEEP                    (1)
#define CLOCK_DRIFT_UNKNOWN             (255)
#define CHANNEL_FUNCTION_UNKNOWN        (255)
#define ONE_HOUR_IN_MS                  (60*60*1000)
#define PURGE_TIMER_PERIOD              ONE_HOUR_IN_MS
#define PURGE_INTERVAL                  (10 * ONE_HOUR_IN_MS)

/******************************************************************************
 Local variables
 *****************************************************************************/

/******************************************************************************
 Glocal variables
 *****************************************************************************/

NWM_NEIGHBOR_TABLE_s FHNT_table;
FHNT_HND_s FHNT_hnd;

/******************************************************************************
 Local Function Prototypes
 *****************************************************************************/

inline static uint8_t FHNT_assessTime(uint32_t curTime,
                                      uint32_t oldTime,
                                      uint32_t validTime)
{
    uint32_t elapsedTime = FHUTIL_elapsedTime(curTime, oldTime);

    if(elapsedTime >= validTime)
    {
        return NODE_TO_PURGE;
    }
    else
    {
        return NODE_TO_KEEP;
    }
}

static void FHNT_purgeTimerIsrCb(uint8_t parameter)
{
    uint32_t curTime;

    curTime = ICall_getTicks();
    FHNT_hnd.purgeCount++;
    FHNT_hnd.purgeTime = curTime;
    FHNT_purgeEntry(curTime);
    FHMGR_macStartFHTimer(&FHNT_hnd.purgeTimer, TRUE);
}

static NODE_ENTRY_s *FHNT_getRemoveEntry(void)
{
    uint16_t i;
    NODE_ENTRY_s *pNodeEntry;
    uint32_t maxElapsedTime;
    uint32_t elapsedTime;
    uint32_t curTime;
    uint16_t maxIdx;
    sAddrExt_t parentEUI;

    pNodeEntry = FHNT_table.node;
    FHPIB_get(FHPIB_TRACK_PARENT_EUI, &parentEUI);

    curTime = ICall_getTicks();
    maxElapsedTime = 0;
    maxIdx = 0;

    for(i = 0; i < FHNT_MAX_NUMBER_OF_NODE; i++, pNodeEntry++)
    {
            if(memcmp(&parentEUI, pNodeEntry->dstAddr, sizeof(sAddrExt_t)))
            {
                elapsedTime = FHUTIL_elapsedTime(curTime,
                                                 pNodeEntry->ref_timeStamp);
                if(elapsedTime > maxElapsedTime)
                {
                    maxElapsedTime = elapsedTime;
                    maxIdx = i;
                }
            }
        }

    return(&FHNT_table.node[maxIdx]);
}

/******************************************************************************
 Public Functions
 *****************************************************************************/

/*!
 FHNT_reset

 Public function defined in fh_nt.h
 */
MAC_INTERNAL_API void FHNT_reset(void)
{
    memset(&FHNT_table, 0, sizeof(NWM_NEIGHBOR_TABLE_s));
    memset(&FHNT_hnd, 0, sizeof(FHNT_HND_s));
}

/*!
 FHNT_init

 Public function defined in fh_nt.h
 */
MAC_INTERNAL_API void FHNT_init(void)
{
    FHNT_reset();

    /*initialize the purge timer */
    FHNT_hnd.purgeTimer.pFunc = FHNT_purgeTimerIsrCb;

    FHNT_hnd.purgeTimer.duration = PURGE_TIMER_PERIOD * (TICKPERIOD_MS_US);

    /*Start purge Timer */
    FHMGR_macStartFHTimer(&FHNT_hnd.purgeTimer, TRUE);
}

/*!
 FHNT_purgeEntry

 Public function defined in fh_nt.h
 */
MAC_INTERNAL_API void FHNT_purgeEntry(uint32_t ts)
{
    halIntState_t intState;
    uint16_t i;
    NODE_ENTRY_s *pNodeEntry;
    sAddrExt_t parentEUI;

    if(!FHNT_table.num_node)
    {
        return;
    }

    pNodeEntry = FHNT_table.node;
    FHPIB_get(FHPIB_TRACK_PARENT_EUI, &parentEUI);

    for(i = 0; i < FHNT_MAX_NUMBER_OF_NODE; i++, pNodeEntry++)
    {
        if(pNodeEntry->valid & FHNT_NODE_W_UTIE)
        {
            if(memcmp(&parentEUI, pNodeEntry->dstAddr, sizeof(sAddrExt_t)))
            {
                if(FHNT_assessTime(ts, pNodeEntry->ref_timeStamp,
                                 ((uint32_t)PURGE_INTERVAL * TICKPERIOD_MS_US))
                                  == NODE_TO_PURGE)
                {
                    pNodeEntry->valid = FHNT_NODE_INVALID;
                    HAL_ENTER_CRITICAL_SECTION(intState);
                    if(FHNT_table.num_node)
                    {
                        FHNT_table.num_node--;
                    }
                    HAL_EXIT_CRITICAL_SECTION(intState);
                }
            }
        }
    }
}


/*!
 FHNT_removeEntry

 Public function defined in fh_nt.h
 */
MAC_INTERNAL_API void FHNT_removeEntry(sAddrExt_t *pAddr)
{
    halIntState_t intState;
    NODE_ENTRY_s *pNodeEntry;
    FHAPI_status status = FHNT_getEntry(pAddr, &pNodeEntry);

    if(pNodeEntry)
    {
        pNodeEntry->valid = FHNT_NODE_INVALID;
        HAL_ENTER_CRITICAL_SECTION(intState);
        if(FHNT_table.num_node)
        {
          FHNT_table.num_node--;
        }
        HAL_EXIT_CRITICAL_SECTION(intState);
    }
}

/*!
 FHNT_createEntry

 Public function defined in fh_nt.h
 */
MAC_INTERNAL_API NODE_ENTRY_s *FHNT_createEntry(sAddrExt_t *pAddr)
{
    uint16_t i;
    uint16_t noOfNode;
    halIntState_t intState;
    NODE_ENTRY_s *pNodeEntry = NULL;

    noOfNode = FHNT_table.num_node;

    if(noOfNode == FHNT_MAX_NUMBER_OF_NODE)
    {
        pNodeEntry = FHNT_getRemoveEntry();
    }
    else
    {
        pNodeEntry = FHNT_table.node;
        for(i = 0; i < FHNT_MAX_NUMBER_OF_NODE; i++, pNodeEntry++)
        {
            if(!pNodeEntry->valid)
            {
                HAL_ENTER_CRITICAL_SECTION(intState);
                FHNT_table.num_node++;
                HAL_EXIT_CRITICAL_SECTION(intState);
                break;
            }
        }
        if(i == FHNT_MAX_NUMBER_OF_NODE)	//this means problem happens
        {
            return(NULL);
        }
    }

    memset(pNodeEntry, 0, sizeof(NODE_ENTRY_s));
    memcpy(&(pNodeEntry->dstAddr), pAddr, sizeof(sAddrExt_t));
    pNodeEntry->UsieParams_s.clockDrift = CLOCK_DRIFT_UNKNOWN;
    pNodeEntry->UsieParams_s.channelFunc = CHANNEL_FUNCTION_UNKNOWN;
    pNodeEntry->valid = FHNT_NODE_CREATED;


    return(pNodeEntry);
}

/*!
 FHNT_getEntry

 Public function defined in fh_nt.h
 */
MAC_INTERNAL_API FHAPI_status FHNT_getEntry(sAddrExt_t *pAddr,
                                            NODE_ENTRY_s **pEntry)
{
    uint16_t i;
    NODE_ENTRY_s *pNodeEntry;
    uint16_t neighborValidTime;
    uint32_t curTime;

    *pEntry = NULL;
    pNodeEntry = FHNT_table.node;
    FHPIB_get(FHPIB_NEIGHBOR_VALID_TIME, &neighborValidTime);
    curTime = ICall_getTicks();

    for(i = 0; i < FHNT_MAX_NUMBER_OF_NODE; i++, pNodeEntry++)
    {
        if((pNodeEntry->valid)
        && !memcmp(&(pNodeEntry->dstAddr), pAddr, sizeof(sAddrExt_t)))
        {
            *pEntry = pNodeEntry;
            if(pNodeEntry->UsieParams_s.channelFunc == FHIE_CF_DH1CF)
            {
                if(FHNT_assessTime(curTime, pNodeEntry->ref_timeStamp,
                  (neighborValidTime * 60 * 1000 * TICKPERIOD_MS_US))
                   == NODE_TO_PURGE)
                {
                    pNodeEntry->valid |= FHNT_NODE_EXPIRED;
                    return(FHAPI_STATUS_ERR_EXPIRED_NODE);
                }
                else
                {
                    return(FHAPI_STATUS_SUCCESS);
                }
            }
            else if(pNodeEntry->UsieParams_s.channelFunc
                    == FHIE_CF_SINGLE_CHANNEL)
            {
                return(FHAPI_STATUS_SUCCESS);
            }
            else
            {
                return(FHAPI_STATUS_ERR);
            }
        }
    }

    return(FHAPI_STATUS_ERR_NO_ENTRY_IN_THE_NEIGHBOR);
}
