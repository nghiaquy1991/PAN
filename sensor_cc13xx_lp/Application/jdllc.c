/******************************************************************************

 @file jdllc.c

 @brief Joining Device Logical Link Controller
 This module is the Joining Device Logical Link Controller for the application.

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
#include <string.h>
#include <stdint.h>

#include "util.h"
#include "jdllc.h"
#include "sensor.h"
#include "ssf.h"
#include "config.h"

/******************************************************************************
 Constants and definitions
 *****************************************************************************/
#define JDLLC_CHAN_LOWEST             0
/* Returns if the specific bit in the scan channel map array is set */
#define JDLLC_IS_CHANNEL_MASK_SET(a, c) \
                  (*((uint8_t*)(a) + ((c) - JDLLC_CHAN_LOWEST) / 8) & \
                  ((uint8_t) 1 << (((c) - JDLLC_CHAN_LOWEST) % 8)))
#define JDLLC_DEFAULT_KEY_SOURCE {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33}
/*! FH channel function value for channel hopping */
#define JDLLC_FH_CHANNEL_HOPPPING  0x02
#define FH_ASSOC_DELAY                2000
#define JDLLC_BEACON_ORDER_NON_BEACON 15
#define JDLLC_INVALID_VALUE           -1
#define JDLLC_INVALID_PAN              0xFFFF
#define JDLLC_RETRY_POLL               500
#define DEFAULT_FH_SLEEP_FIXED_CHANNEL 0
#define WISUN_PANIE_PRESENT            0x1
#define WISUN_NETNAME_IE_PRESENT       0x2
#define WISUN_PANVER_IE_PRESENT        0x4
#define WISUN_GTKHASH_IE_PRESENT       0x8
#define GTK_HASH_LEN                   0x8
/******************************************************************************
 Security constants and definitions
 *****************************************************************************/

#define KEY_TABLE_ENTRIES 1
#define KEY_ID_LOOKUP_ENTRIES 1
#define KEY_DEVICE_TABLE_ENTRIES 8
#define KEY_USAGE_TABLE_ENTRIES 1
#define SECURITY_LEVEL_ENTRIES 1

#define MAC_FRAME_TYPE_DATA 1
#define MAC_DATA_REQ_FRAME 4

#define AUTO_REQUEST_SEC_LEVEL 0x00
/******************************************************************************
 Structures
 *****************************************************************************/
/* Device information, used to store default parameters */
typedef struct
{
    uint16_t panID;
    uint8_t channel;
    uint16_t coordShortAddr;
    uint8_t coordExtAddr[APIMAC_SADDR_EXT_LEN];
    uint16_t devShortAddr;
    uint8_t devExtAddr[APIMAC_SADDR_EXT_LEN];
    uint8_t beaconOrder;
    uint8_t superframeOrder;
    Jdllc_states_t currentJdllcState;
    Jdllc_states_t prevJdllcState;
    Jdllc_device_states_t currentDevState;
    Jdllc_device_states_t prevDevState;
    uint8_t dataFailures;
    uint32_t pollInterval;
} devInformation_t;

/******************************************************************************
 Global variables
 *****************************************************************************/
/* Task pending events */
uint16_t Jdllc_events = 0;
/* JDLLC statistics variable */
Jdllc_statistics_t Jdllc_statistics;
/*Default key for JDLLC */
uint8_t Jdllc_keySource[] = JDLLC_DEFAULT_KEY_SOURCE;

/******************************************************************************
 Local variables
 *****************************************************************************/
/* structure containing device and its parents information*/
STATIC devInformation_t devInfoBlock;
/* default channel mask */
STATIC uint8_t defaultChannelMask[APIMAC_154G_CHANNEL_BITMAP_SIZ] =
                CONFIG_CHANNEL_MASK;
/* copy of MAC API callbacks */
STATIC ApiMac_callbacks_t macCallbacksCopy =  { 0 };
/* copy of CLLC callbacks */
STATIC Jdllc_callbacks_t *pJdllcCallbacksCopy = (Jdllc_callbacks_t *)NULL;
/* current channel in FH sleep node hop sequence */
STATIC uint8_t sleepNodeChIdx = 0;
/* flag to control scan backoff */
STATIC bool continueScan = true;
/* flag to pick parent from incoming beacons or in FH networks send association
   request only once
 */
STATIC bool parentFound = false;
/* netname to identify node */
STATIC uint8_t fhNetname[APIMAC_FH_NET_NAME_SIZE_MAX] = CONFIG_FH_NETNAME;
STATIC uint8_t fhNumPASRcvdInTrickleWindow = 0;
STATIC uint8_t fhNumPCSRcvdInTrickleWindow = 0;
STATIC uint8_t fhAssociationAttempts = 0;
/* FH Channel Mask */
STATIC uint8_t fhChannelMask[] = CONFIG_FH_CHANNEL_MASK;

/******************************************************************************
 Local security variables
 *****************************************************************************/

static const ApiMac_keyIdLookupDescriptor_t keyIdLookupList[] =
    {
      {
        /* Key identity data */
        { 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x03 },
        0x01 /* 9 octets */
      }
    };

/* Key device list can be modified at run time */
static ApiMac_keyDeviceDescriptor_t keyDeviceList[] =
    {
      { 0x00, false, false },
      { 0x00, false, false },
      { 0x00, false, false },
      { 0x00, false, false },
      { 0x00, false, false },
      { 0x00, false, false },
      { 0x00, false, false },
      { 0x00, false, false }
    };

static const ApiMac_keyUsageDescriptor_t keyUsageList[] =
    {
      { MAC_FRAME_TYPE_DATA, MAC_DATA_REQ_FRAME }
    };

STATIC ApiMac_keyDescriptor_t keyTable[] =
    {
      {
        (ApiMac_keyIdLookupDescriptor_t *)keyIdLookupList,
        KEY_ID_LOOKUP_ENTRIES,
        (ApiMac_keyDeviceDescriptor_t *)keyDeviceList,
        KEY_DEVICE_TABLE_ENTRIES,
        (ApiMac_keyUsageDescriptor_t *)keyUsageList,
        KEY_USAGE_TABLE_ENTRIES,
        KEY_TABLE_DEFAULT_KEY,
        0 /* frame counter */
      }
    };

STATIC const ApiMac_securityPibSecurityLevelEntry_t securityLevelEntry =
    {
      0,
      { MAC_FRAME_TYPE_DATA, MAC_DATA_REQ_FRAME, 0, false }
    };

STATIC ApiMac_secLevel_t secLevel = ApiMac_secLevel_encMic32;

STATIC ApiMac_keyIdMode_t secKeyIdMode = ApiMac_keyIdMode_8;
STATIC uint8_t secKeyIndex = 3; /* cant be zero for implicit key identifier */

STATIC bool macSecurity = CONFIG_SECURE;

/******************************************************************************
 Local Function Prototypes
 *****************************************************************************/
/* CLLC callbacks */
static void assocCnfCb(ApiMac_mlmeAssociateCnf_t *pData);
static void beaconNotifyIndCb(ApiMac_mlmeBeaconNotifyInd_t *pData);
static void scanCnfCb(ApiMac_mlmeScanCnf_t *pData);
static void disassoCnfCb(ApiMac_mlmeDisassociateCnf_t *pData);
static void disassocIndCb(ApiMac_mlmeDisassociateInd_t *pData);
static void wsAsyncIndCb(ApiMac_mlmeWsAsyncInd_t *pData);
static void syncLossCb(ApiMac_mlmeSyncLossInd_t *pData);
static void dataCnfCb(ApiMac_mcpsDataCnf_t *pData);
static void pollCnfCb(ApiMac_mlmePollCnf_t *pData);

static void switchState(Jdllc_device_states_t newState);
static void processState(Jdllc_device_states_t state);
static bool checkBeaconOrder(uint16_t superframeSpec);
static void sendAssocReq(void);
static void updateState(Jdllc_states_t state);
static void sendPollReq(void);
static void processCoordRealign(void);
static void sendScanReq(ApiMac_scantype_t type);
static void sendAsyncReq(ApiMac_wisunAsyncFrame_t frameType);
static uint8_t getFHSleepNodeHopChannel(void);

/******************************************************************************
 Public Functions
 *****************************************************************************/
/*!
 Initialize this module.

 Public function defined in jdllc.h
 */
void Jdllc_init(ApiMac_callbacks_t *pMacCbs, Jdllc_callbacks_t *pJdllcCbs)
{
    /* initialize device information*/
    devInfoBlock.panID = CONFIG_PAN_ID;
    devInfoBlock.channel = JDLLC_CHAN_LOWEST;
    devInfoBlock.currentJdllcState = Jdllc_states_initWaiting;
    devInfoBlock.prevJdllcState = Jdllc_states_initWaiting;
    devInfoBlock.pollInterval = CONFIG_POLLING_INTERVAL;
    devInfoBlock.dataFailures = 0;
    memset(&devInfoBlock.coordExtAddr, 0, (APIMAC_SADDR_EXT_LEN));
    devInfoBlock.coordShortAddr = 0;
    devInfoBlock.devShortAddr = 0;
    devInfoBlock.beaconOrder = CONFIG_BEACON_ORDER;
    devInfoBlock.superframeOrder = CONFIG_SUPERFRAME_ORDER;

    if(CONFIG_BEACON_ORDER == JDLLC_BEACON_ORDER_NON_BEACON)
    {
        /* non beacon network */
        devInfoBlock.currentDevState = Jdllc_deviceStates_scanActive;
        devInfoBlock.prevDevState = Jdllc_deviceStates_scanActive;
    }
    else if(CONFIG_BEACON_ORDER > 0 && CONFIG_BEACON_ORDER < JDLLC_BEACON_ORDER_NON_BEACON)
    {
        /* beacon network */
        devInfoBlock.currentDevState = Jdllc_deviceStates_scanPassive;
        devInfoBlock.prevDevState = Jdllc_deviceStates_scanActive;
    }

    ApiMac_mlmeSetReqUint8(ApiMac_attribute_phyCurrentDescriptorId,
                           CONFIG_PHY_ID);
    ApiMac_mlmeSetReqBool(ApiMac_attribute_RxOnWhenIdle, true);

    /* Save callback */
    memcpy(&macCallbacksCopy, pMacCbs, sizeof(ApiMac_callbacks_t));
    pJdllcCallbacksCopy = pJdllcCbs;

    /* overwrite callbacks with llc callbacks */
    pMacCbs->pAssocCnfCb = assocCnfCb;
    pMacCbs->pBeaconNotifyIndCb = beaconNotifyIndCb;
    pMacCbs->pScanCnfCb = scanCnfCb;
    pMacCbs->pDisassociateCnfCb = disassoCnfCb;
    pMacCbs->pWsAsyncIndCb = wsAsyncIndCb;
    pMacCbs->pSyncLossIndCb = syncLossCb;
    pMacCbs->pDataCnfCb = dataCnfCb;
    pMacCbs->pPollCnfCb = pollCnfCb;
    pMacCbs->pDisassociateIndCb = disassocIndCb;

    /* Initialize poll clock */
    if(!CONFIG_RX_ON_IDLE)
    {
        Ssf_initializePollClock();
    }

    if(CONFIG_FH_ENABLE)
    {
        uint8_t sizeOfChannelMask, idx;
        sizeOfChannelMask = sizeof(fhChannelMask)/sizeof(uint8_t);

        /* PIB for FH are set when we receive the IEs*/
        /* initialize trickle timer clock */
        Ssf_initializeTrickleClock();
        /* initialize FH association delay clock */
        Ssf_initializeFHAssocClock();

        /* enable frequency hopping operation */

        ApiMac_mlmeSetFhReqUint8(ApiMac_FHAttribute_unicastDwellInterval,
                                 CONFIG_DWELL_TIME);
        ApiMac_mlmeSetFhReqUint8(ApiMac_FHAttribute_broadcastDwellInterval,
                                 CONFIG_DWELL_TIME);
        ApiMac_mlmeSetFhReqArray(ApiMac_FHAttribute_netName, &fhNetname[0]);
        /* if non sleepy FH device */
        if(CONFIG_RX_ON_IDLE)
        {
            uint8_t excludeChannels[APIMAC_154G_CHANNEL_BITMAP_SIZ];

            /* set PIB to enable channel hopping*/
            ApiMac_mlmeSetFhReqUint8(
                            ApiMac_FHAttribute_unicastChannelFunction,
                            JDLLC_FH_CHANNEL_HOPPPING);
            ApiMac_mlmeSetFhReqUint8(
                            ApiMac_FHAttribute_broadcastChannelFunction,
                            JDLLC_FH_CHANNEL_HOPPPING);
            /* set of Exclude Channels */
            if(sizeOfChannelMask > APIMAC_154G_CHANNEL_BITMAP_SIZ)
            {
                sizeOfChannelMask = APIMAC_154G_CHANNEL_BITMAP_SIZ;
            }
            memset(excludeChannels, 0, APIMAC_154G_CHANNEL_BITMAP_SIZ);
            for(idx = 0; idx < sizeOfChannelMask; idx++)
            {
                excludeChannels[idx] = ~fhChannelMask[idx];
            }
            ApiMac_mlmeSetFhReqArray(ApiMac_FHAttribute_unicastExcludedChannels,
                                     excludeChannels);
            ApiMac_mlmeSetFhReqArray(ApiMac_FHAttribute_broadcastExcludedChannels,
                                     excludeChannels);
        }
        else
        {
            uint8_t bitIndex, chIndex;
            /* set PIB to enable fixed channel*/
            ApiMac_mlmeSetFhReqUint8(ApiMac_FHAttribute_unicastChannelFunction,
                                     0);
            ApiMac_mlmeSetFhReqUint8(
                            ApiMac_FHAttribute_broadcastChannelFunction, 0);

            /*Initialize the hop sequence to account for maxChannels*/
            chIndex = 0;
            for(idx = 0; idx < sizeOfChannelMask; idx++)
            {
                for(bitIndex = 0; bitIndex < 8; bitIndex ++)
                {
                    if(chIndex >= APIMAC_154G_MAX_NUM_CHANNEL)
                    {
                        fhChannelMask[idx] &= ~(1 << bitIndex);
                    }
                    chIndex++;
                }
            }
            /* set fixed channel in FH PIB */
            ApiMac_mlmeSetFhReqUint16(ApiMac_FHAttribute_unicastFixedChannel,
                                      (uint16_t) getFHSleepNodeHopChannel());
        }

        /* Start FH */
        ApiMac_startFH();

    }
    else
    {
        /* Initialize scan backoff clock if sleepy device */
        if(!CONFIG_RX_ON_IDLE)
        {
            Ssf_initializeScanBackoffClock();
        }
    }
}

/*!
 Jdllc task processing.

 Public function defined in jdllc.h
 */
void Jdllc_process(void)
{
    /* The LLC has an event */
    if(Jdllc_events & JDLLC_PAS_EVT)
    {
        if(fhNumPASRcvdInTrickleWindow == 0)
        {
            /* Process LLC Event */
            sendAsyncReq(ApiMac_wisunAsyncFrame_advertisementSolicit);
        }

        fhNumPASRcvdInTrickleWindow = 0;

        Ssf_setTrickleClock(CONFIG_PAN_ADVERT_SOLICIT_CLK_DURATION,
                            ApiMac_wisunAsyncFrame_advertisementSolicit);

        /* Clear the event */
        Util_clearEvent(&Jdllc_events, JDLLC_PAS_EVT);
    }

    /* The LLC has an PC event */
    if(Jdllc_events & JDLLC_PCS_EVT)
    {
        if(fhNumPCSRcvdInTrickleWindow == 0)
        {
            /* Process LLC Event */
            sendAsyncReq(ApiMac_wisunAsyncFrame_configSolicit);
        }

        fhNumPCSRcvdInTrickleWindow = 0;

        Ssf_setTrickleClock(CONFIG_PAN_CONFIG_SOLICIT_CLK_DURATION,
                            ApiMac_wisunAsyncFrame_configSolicit);

        /* Clear the event */
        Util_clearEvent(&Jdllc_events, JDLLC_PCS_EVT);
    }

    /* Process state change event */
    if(Jdllc_events & JDLLC_STATE_CHANGE_EVT)
    {
        /* Process LLC Event */
        processState(devInfoBlock.currentDevState);

        /* Clear the event */
        Util_clearEvent(&Jdllc_events, JDLLC_STATE_CHANGE_EVT);
    }

    /* Process poll event */
    if(Jdllc_events & JDLLC_POLL_EVT)
    {
        if(CONFIG_BEACON_ORDER == JDLLC_BEACON_ORDER_NON_BEACON)
        {
            if((devInfoBlock.currentJdllcState == Jdllc_states_joined)
                || (devInfoBlock.currentJdllcState == Jdllc_states_rejoined))
            {
                /* set poll timer */
                Ssf_setPollClock(devInfoBlock.pollInterval);
            }

            if(CONFIG_FH_ENABLE)
            {
                /* set fixed channel in FH PIB */
                ApiMac_mlmeSetFhReqUint16(
                        ApiMac_FHAttribute_unicastFixedChannel,
                        (uint16_t)getFHSleepNodeHopChannel());
            }

            /* send poll request */
            sendPollReq();
        }

        /* Clear the event */
        Util_clearEvent(&Jdllc_events, JDLLC_POLL_EVT);
    }

    /* Send association request */
    if(Jdllc_events & JDLLC_ASSOCIATE_REQ_EVT)
    {
        /* Send Associate request */
        sendAssocReq();

        /* Clear the event */
        Util_clearEvent(&Jdllc_events, JDLLC_ASSOCIATE_REQ_EVT);
    }

    /* Process coordinator realignment */
    if(Jdllc_events & JDLLC_COORD_REALIGN)
    {
        processCoordRealign();

        /* Clear the event */
        Util_clearEvent(&Jdllc_events, JDLLC_COORD_REALIGN);
   }

    /* Process scan backoff in case of orphan scan */
    if(Jdllc_events & JDLLC_SCAN_BACKOFF)
    {
        if(devInfoBlock.currentDevState == Jdllc_deviceStates_scanBackoff)
        {
            /* restart scan from previous state */
            switchState(devInfoBlock.prevDevState);
            continueScan = true;
        }
        else
        {
            /* scan backoff , silent period */
            devInfoBlock.prevDevState = devInfoBlock.currentDevState;
            devInfoBlock.currentDevState = Jdllc_deviceStates_scanBackoff;
            continueScan = false;
        }

        Ssf_setScanBackoffClock(CONFIG_SCAN_BACKOFF_INTERVAL);

        /* Clear the event */
        Util_clearEvent(&Jdllc_events, JDLLC_SCAN_BACKOFF);
    }
}

/*!
 Join the network

 Public function defined in jdllc.h
 */
void Jdllc_join(void)
{
    /* set state */
    updateState(Jdllc_states_joining);

    /* if beacon enabled network  passive scan or
     * if non beacon network  active scan*/
    if(CONFIG_FH_ENABLE)
    {
        /* start trickle timer for PAS */
        Ssf_setTrickleClock(CONFIG_PAN_ADVERT_SOLICIT_CLK_DURATION,
                            ApiMac_wisunAsyncFrame_advertisementSolicit);
    }
    else
    {
        /* set scan backoff clock */
        if(!CONFIG_RX_ON_IDLE)
        {
            Ssf_setScanBackoffClock(CONFIG_SCAN_BACKOFF_INTERVAL);
        }
        if(CONFIG_BEACON_ORDER == JDLLC_BEACON_ORDER_NON_BEACON)
        {
            /* non beacon network */
            if(continueScan == true)
            {
                switchState(Jdllc_deviceStates_scanActive);
            }
        }
        else if(CONFIG_BEACON_ORDER > 0 && CONFIG_BEACON_ORDER < JDLLC_BEACON_ORDER_NON_BEACON)
        {
            /* beacon network */
            switchState(Jdllc_deviceStates_scanPassive);
        }
    }
}

/*!
 Rejoin network

 Public function defined in jdllc.h
 */
void Jdllc_rejoin(ApiMac_deviceDescriptor_t *pDevInfo,
                  Llc_netInfo_t *pParentInfo)
{
    /* set state */
    updateState(Jdllc_states_initRestoring);

    /* update device information variables */
    devInfoBlock.panID = pDevInfo->panID;
    devInfoBlock.channel = pParentInfo->channel;
    memcpy(devInfoBlock.devExtAddr, pDevInfo->extAddress,
           (APIMAC_SADDR_EXT_LEN));
    devInfoBlock.devShortAddr = pDevInfo->shortAddress;
    memcpy(devInfoBlock.coordExtAddr, pParentInfo->devInfo.extAddress,
           (APIMAC_SADDR_EXT_LEN));
    devInfoBlock.coordShortAddr = pParentInfo->devInfo.shortAddress;

    /* update MAC PIBs */
    ApiMac_mlmeSetReqUint16(ApiMac_attribute_panId, devInfoBlock.panID);
    ApiMac_mlmeSetReqUint16(ApiMac_attribute_shortAddress,
                            devInfoBlock.devShortAddr);
    ApiMac_mlmeSetReqArray(ApiMac_attribute_coordExtendedAddress,
                           devInfoBlock.coordExtAddr);

    if(!CONFIG_FH_ENABLE)
    {
        ApiMac_mlmeSetReqUint8(ApiMac_attribute_logicalChannel,
                               devInfoBlock.channel);
        ApiMac_mlmeSetReqUint16(ApiMac_attribute_coordShortAddress,
                                devInfoBlock.coordShortAddr);

        if(CONFIG_BEACON_ORDER > 0 && CONFIG_BEACON_ORDER
                        < JDLLC_BEACON_ORDER_NON_BEACON)
        {
            ApiMac_mlmeSetReqBool(ApiMac_attribute_RxOnWhenIdle,
                                  CONFIG_RX_ON_IDLE);

            /* send sync request for beacon enabled device */
            switchState(Jdllc_deviceStates_syncReq);

            /* device joined */
            if(pJdllcCallbacksCopy && pJdllcCallbacksCopy->pJoinedCb)
            {
                pJdllcCallbacksCopy->pJoinedCb(pDevInfo, pParentInfo);
            }
            updateState(Jdllc_states_rejoined);
        }
    }
    else
    {
        parentFound = false;
        ApiMac_mlmeSetReqBool(ApiMac_attribute_RxOnWhenIdle, true);
        /* start trickle timer for PCS */
        Ssf_setTrickleClock(CONFIG_PAN_CONFIG_SOLICIT_CLK_DURATION,
                            ApiMac_wisunAsyncFrame_configSolicit);

    }

    if((!CONFIG_RX_ON_IDLE) && (!CONFIG_FH_ENABLE))
    {
        /* set event for polling if sleepy device*/
        Util_setEvent(&Jdllc_events, JDLLC_POLL_EVT);
    }
}

/*!
 Set the poll interval.

 Public function defined in jdllc.h
 */
void Jdllc_setPollRate(uint32_t pollInterval)
{
    devInfoBlock.pollInterval = pollInterval;
}

/*!
 Send disassociation request.

 Public function defined in jdllc.h
 */
void Jdllc_sendDisassociationRequest(void)
{
    ApiMac_mlmeDisassociateReq_t disassocReq;
    memset(&disassocReq, 0, sizeof(ApiMac_mlmeDisassociateReq_t));

    if(CONFIG_FH_ENABLE)
    {
        disassocReq.deviceAddress.addrMode = ApiMac_addrType_extended;
        memcpy(disassocReq.deviceAddress.addr.extAddr,
               devInfoBlock.coordExtAddr, (APIMAC_SADDR_EXT_LEN));
    }
    else
    {
        disassocReq.deviceAddress.addrMode = ApiMac_addrType_short;
        disassocReq.deviceAddress.addr.shortAddr = devInfoBlock.coordShortAddr;

    }

    disassocReq.devicePanId = devInfoBlock.panID;
    disassocReq.disassociateReason = ApiMac_disassocateReason_device;
    disassocReq.txIndirect = false;
    ApiMac_mlmeDisassociateReq(&disassocReq);
}

/*!
 Initialize the MAC Security

 Public function defined in jdllc.h
 */
void Jdllc_securityInit(uint32_t frameCounter)
{
    if(macSecurity == true)
    {
        ApiMac_secAddKeyInitFrameCounter_t secInfo;

        memset(&secInfo, 0, sizeof(ApiMac_secAddKeyInitFrameCounter_t));

        memcpy(secInfo.key, keyTable[0].key, APIMAC_KEY_MAX_LEN);
        secInfo.frameCounter = frameCounter;
        secInfo.replaceKeyIndex = 0;
        secInfo.newKeyFlag = true;
        secInfo.lookupDataSize = APIMAC_KEY_LOOKUP_LONG_LEN;
        memcpy(secInfo.lookupData, keyIdLookupList[0].lookupData,
               (APIMAC_MAX_KEY_LOOKUP_LEN));

        ApiMac_secAddKeyInitFrameCounter(&secInfo);
        ApiMac_mlmeSetSecurityReqArray(
                                ApiMac_securityAttribute_defaultKeySource,
                                (void *) Jdllc_keySource);

        ApiMac_mlmeSetSecurityReqStruct(ApiMac_securityAttribute_keyTable,
                                        (void *)NULL);

        /* Write a security level entry to PIB */
        ApiMac_mlmeSetSecurityReqStruct(
                        ApiMac_securityAttribute_securityLevelEntry,
                        (void *)&securityLevelEntry);

        /* Set the MAC security */
        ApiMac_mlmeSetReqBool(ApiMac_attribute_securityEnabled, macSecurity);

        /* set security for auto request for beacon enabled network to zero */
        if(CONFIG_BEACON_ORDER > 0 && CONFIG_BEACON_ORDER
                        < JDLLC_BEACON_ORDER_NON_BEACON)
        {
            ApiMac_mlmeSetSecurityReqUint8(
                            ApiMac_securityAttribute_autoRequestSecurityLevel,
                            AUTO_REQUEST_SEC_LEVEL);
        }
    }
}

/*!
 Fill in the security structure

 Public function defined in jdllc.h
 */
void Jdllc_securityFill(ApiMac_sec_t *pSec)
{
    if(pSec)
    {
        memset(pSec, 0, sizeof(ApiMac_sec_t));

        if(macSecurity == true)
        {
            memcpy(pSec->keySource, keyIdLookupList[0].lookupData,
                   (APIMAC_KEY_SOURCE_MAX_LEN));
            pSec->securityLevel = secLevel;
            pSec->keyIdMode = secKeyIdMode;
            pSec->keyIndex = secKeyIndex;
        }
    }
}

/*!
 Check the security level against expected level

 Public function defined in jdllc.h
 */
bool Jdllc_securityCheck(ApiMac_sec_t *pSec)
{
    bool ret = false;

    if(macSecurity == true)
    {
        if(pSec)
        {
            if(pSec->securityLevel == secLevel)
            {
                ret = true;
            }
        }
    }
    else
    {
       ret = true;
    }

    return(ret);
}


/*!
 Add a device to the MAC security device table.

 Public function defined in sensor.h
 */
ApiMac_status_t Jdllc_addSecDevice(uint16_t panID, uint16_t shortAddr,
                         ApiMac_sAddrExt_t *pExtAddr, uint32_t frameCounter)
{
    if(macSecurity == true)
    {
        ApiMac_secAddDevice_t device;
        uint8_t keyIndex = 0;

        memset(&device, 0, sizeof(ApiMac_secAddDevice_t));

        device.panID = panID;
        device.shortAddr = shortAddr;
        memcpy(device.extAddr, pExtAddr, sizeof(ApiMac_sAddrExt_t));
        device.frameCounter = frameCounter;

        device.exempt = false;

        /* get the key lookup information from the initial loaded key */
        device.keyIdLookupDataSize = keyIdLookupList[keyIndex].lookupDataSize;
        memcpy(device.keyIdLookupData, keyIdLookupList[keyIndex].lookupData,
               (APIMAC_MAX_KEY_LOOKUP_LEN));

        device.uniqueDevice = false;
        device.duplicateDevFlag = false;

        return(ApiMac_secAddDevice(&device));
    }
    else
    {
        return(ApiMac_status_success);
    }
}

/******************************************************************************
 Local Functions
 *****************************************************************************/
/*!
 * @brief       Switch to the new  coordinator state and set the event bit.
 *
 * @param       newState - next state of coordinator
 */
static void switchState(Jdllc_device_states_t newState)
{
    devInfoBlock.currentDevState = newState;
    Util_setEvent(&Jdllc_events, JDLLC_STATE_CHANGE_EVT);
}

/*!
 * @brief       Function to transition various states involved with scan request
 *               and start request before the coordinator is started.
 *
 * @param       state - current startup state of coordinator
 */
static void processState(Jdllc_device_states_t state)
{
    ApiMac_mlmeSyncReq_t syncReq;

    switch(state)
    {
        case Jdllc_deviceStates_scanActive:
            /* Active scan */
            sendScanReq(ApiMac_scantype_active);
            break;

        case Jdllc_deviceStates_scanPassive:
            /* Passive scan */
            sendScanReq(ApiMac_scantype_passive);
            break;

        case Jdllc_deviceStates_syncReq:
            ApiMac_mlmeSetReqUint8(ApiMac_attribute_beaconOrder,
                                   devInfoBlock.beaconOrder);
            ApiMac_mlmeSetReqUint8(ApiMac_attribute_superframeOrder,
                                   devInfoBlock.superframeOrder);
            ApiMac_mlmeSetReqUint16(ApiMac_attribute_coordShortAddress,
                                    devInfoBlock.coordShortAddr);
            ApiMac_mlmeSetReqUint16(ApiMac_attribute_panId, devInfoBlock.panID);
            /* Sync request for beacon enabled devices */
            memset(&syncReq, 0, sizeof(ApiMac_mlmeSyncReq_t));
            syncReq.logicalChannel = devInfoBlock.channel;
            syncReq.channelPage = CONFIG_CHANNEL_PAGE;
            syncReq.phyID = CONFIG_PHY_ID;
            syncReq.trackBeacon = true;
            ApiMac_mlmeSyncReq(&syncReq);

            if(parentFound == true)
            {
                Util_setEvent(&Jdllc_events, JDLLC_ASSOCIATE_REQ_EVT);
            }
            break;

        case Jdllc_deviceStates_scanOrphan:
            /* Orphan scan */
            sendScanReq(ApiMac_scantype_orphan);
            break;

        default:
            break;
    }
}

/*!
 * @brief       Check if the incoming frame's beacon order matches the network
 *              type of device
 *
 * @param       superframeSpec - super frame spec of incoming beacon
 *
 * @return      true if matches, else false
 */
static bool checkBeaconOrder(uint16_t superframeSpec)
{
    if(CONFIG_BEACON_ORDER == JDLLC_BEACON_ORDER_NON_BEACON)
    {
        if(APIMAC_SFS_BEACON_ORDER(superframeSpec) == CONFIG_BEACON_ORDER)
        {
            return (true);
        }
    }
    else if((APIMAC_SFS_BEACON_ORDER(superframeSpec) <= CONFIG_BEACON_ORDER))
    {
        return (true);
    }

    return (false);
}

/*!
 * @brief       Get the next channel for sleep node hopping based on
 *              chanel mask and sleepNodeChIdx
 *
 * @param       none
 *
 * @return      Channel to be used by sleep node
 */
static uint8_t getFHSleepNodeHopChannel(void)
{
    uint8_t curChBitMap = 0;
    uint8_t curChListPos = 0, bitIdx;
    uint8_t i, chanBitMapSize, startChListPos, retCh;

    chanBitMapSize = sizeof(fhChannelMask)/sizeof(uint8_t);

    curChListPos = sleepNodeChIdx >> 3;
    startChListPos = curChListPos;
    bitIdx = sleepNodeChIdx & 7;

    if((!chanBitMapSize) || (curChListPos >= chanBitMapSize))
    {
        return(DEFAULT_FH_SLEEP_FIXED_CHANNEL);
    }

    curChBitMap = fhChannelMask[curChListPos] >> bitIdx;
    curChListPos++;

    while((!curChBitMap) && (curChListPos != startChListPos))
    {
        curChBitMap = fhChannelMask[curChListPos];
        sleepNodeChIdx = curChListPos * 8;

        curChListPos++;
        if(curChListPos > (chanBitMapSize - 1))
        {
            curChListPos = 0;
            curChBitMap = fhChannelMask[curChListPos];
            sleepNodeChIdx = curChListPos * 8;
        }
    }

    if(curChBitMap != 0)
    {
        i = sleepNodeChIdx;
        while(i < (8*(curChListPos + 1)))
        {
            if(curChBitMap & 1)
            {
                retCh = sleepNodeChIdx;
                /* Increment chIndex for next Call */
                sleepNodeChIdx += 1;
                return(retCh);
            }
            else
            {
                sleepNodeChIdx += 1;
                curChBitMap >>= 1;
            }
            i++;
        }
    }

    /* An array of all Zeros */
    return(DEFAULT_FH_SLEEP_FIXED_CHANNEL);
}

/*!
 * @brief       Process  Beacon Notification callback.
 *
 * @param       pData - pointer MAC Beacon indication info
 */
static void beaconNotifyIndCb(ApiMac_mlmeBeaconNotifyInd_t *pData)
{
    /* check beacon type */
    if(pData->beaconType == ApiMac_beaconType_normal)
    {
        if(parentFound == false)
        {
            /* check if the received beacon is from a
             * coordinator which is in not in the blacklist */
            if(Ssf_findBlackListIndex(
                            &pData->panDesc.coordAddress) ==
                                         JDLLC_INVALID_VALUE)
            {
                /* check if association bit permit is set */
                if(APIMAC_SFS_ASSOCIATION_PERMIT(
                                pData->panDesc.superframeSpec))
                {
                    /* check for beacon order match */
                    if(checkBeaconOrder(pData->panDesc.superframeSpec) == true)
                    {
                        if(devInfoBlock.panID == JDLLC_INVALID_PAN)
                        {
                            /* device can join any network , associate with
                             * first coordinator from which beacon is received*/
                            devInfoBlock.panID = pData->panDesc.coordPanId;
                            devInfoBlock.channel =
                                            pData->panDesc.logicalChannel;
                            devInfoBlock.coordShortAddr =
                                    pData->panDesc.coordAddress.addr.shortAddr;
                            if(APIMAC_SFS_BEACON_ORDER(
                                    pData->panDesc.superframeSpec)!=
                                                    JDLLC_BEACON_ORDER_NON_BEACON)
                            {
                                devInfoBlock.beaconOrder =
                                  APIMAC_SFS_BEACON_ORDER(
                                  pData->panDesc.superframeSpec);
                                devInfoBlock.superframeOrder =
                                  APIMAC_SFS_SUPERFRAME_ORDER(
                                  pData->panDesc.superframeSpec);
                            }
                            parentFound = true;
                        }
                        else
                        {
                            if(pData->panDesc.coordPanId == devInfoBlock.panID)
                            {
                                devInfoBlock.channel =
                                    pData->panDesc.logicalChannel;
                                devInfoBlock.coordShortAddr =
                                    pData->panDesc.coordAddress.addr.shortAddr;
                                if(APIMAC_SFS_BEACON_ORDER(
                                    pData->panDesc.superframeSpec) !=
                                                    JDLLC_BEACON_ORDER_NON_BEACON)
                                {
                                    devInfoBlock.beaconOrder =
                                        APIMAC_SFS_BEACON_ORDER(
                                        pData->panDesc.superframeSpec);
                                    devInfoBlock.superframeOrder =
                                        APIMAC_SFS_SUPERFRAME_ORDER(
                                        pData->panDesc.superframeSpec);
                                }
                                parentFound = true;
                            }
                        }
                    }
                }
            }
        }
    }

    /* Callback to MAC API */
    if(macCallbacksCopy.pBeaconNotifyIndCb != NULL)
    {
        macCallbacksCopy.pBeaconNotifyIndCb(pData);
    }
}

/*!
 * @brief       Process  Scan Confirm  callback.
 *
 * @param       pData - pointer to Scan Confirm
 */
static void scanCnfCb(ApiMac_mlmeScanCnf_t *pData)
{
    if(pData->status == ApiMac_status_success)
    {
        if(pData->scanType == ApiMac_scantype_active)
        {
            /* set event to send Association Request */
            Util_setEvent(&Jdllc_events, JDLLC_ASSOCIATE_REQ_EVT);
        }
        else if(pData->scanType == ApiMac_scantype_passive)
        {
            /* send sync request for beacon enabled device */
            switchState(Jdllc_deviceStates_syncReq);

        }
        else if(pData->scanType == ApiMac_scantype_orphan)
        {
            /* coordinator realignment received, set event to process it */
            Util_setEvent(&Jdllc_events, JDLLC_COORD_REALIGN);
        }
    }
    else
    {
        if((pData->scanType == ApiMac_scantype_orphan) && (pData->status
                        == ApiMac_status_noBeacon))
        {
            /* repeat orphan scan, no coordinator realignment was received */
            switchState(Jdllc_deviceStates_scanOrphan);
            updateState(Jdllc_states_orphan);
        }
        else
        {
            /* repeat scan in case of failure */
            if(CONFIG_BEACON_ORDER == JDLLC_BEACON_ORDER_NON_BEACON)
            {
                /* non beacon network */
                if(continueScan == true)
                {
                    switchState(Jdllc_deviceStates_scanActive);
                }
            }
            else if((CONFIG_BEACON_ORDER > 0) && (CONFIG_BEACON_ORDER <
                                            JDLLC_BEACON_ORDER_NON_BEACON))
            {
                /* beacon network */
                switchState(Jdllc_deviceStates_scanPassive);
            }
        }
    }

    if(macCallbacksCopy.pScanCnfCb != NULL)
    {
        macCallbacksCopy.pScanCnfCb(pData);
    }
}

/*!
 * @brief       Handle Jdllc callback for assoc Confirm
 *
 * @param       pData - pointer to Associate Confirm structure
 */
static void assocCnfCb(ApiMac_mlmeAssociateCnf_t *pData)
{
    Llc_netInfo_t parentInfo;
    ApiMac_deviceDescriptor_t devInfo;
    uint16_t randomNum = 0;

    if(pData->status == ApiMac_assocStatus_success)
    {
        parentInfo.devInfo.panID = devInfoBlock.panID;
        devInfo.shortAddress = pData->assocShortAddress;

        ApiMac_mlmeGetReqArray(ApiMac_attribute_extendedAddress,
                               devInfoBlock.devExtAddr);
        memcpy(devInfo.extAddress, devInfoBlock.devExtAddr,
               (APIMAC_SADDR_EXT_LEN));

        /* set device short address PIB */
        ApiMac_mlmeSetReqUint16(ApiMac_attribute_shortAddress,
                                pData->assocShortAddress);
        devInfo.panID = devInfoBlock.panID;

        ApiMac_mlmeGetReqArray(ApiMac_attribute_coordExtendedAddress,
                               devInfoBlock.coordExtAddr);

        memcpy(parentInfo.devInfo.extAddress, devInfoBlock.coordExtAddr,
               (APIMAC_SADDR_EXT_LEN));
        parentInfo.devInfo.shortAddress = devInfoBlock.coordShortAddr;

        if(CONFIG_FH_ENABLE)
        {
            parentInfo.fh = true;
            Ssf_setTrickleClock(0, ApiMac_wisunAsyncFrame_advertisementSolicit);
            Ssf_setTrickleClock(0, ApiMac_wisunAsyncFrame_configSolicit);
        }
        else
        {
            parentInfo.fh = false;
            parentInfo.channel = devInfoBlock.channel;
        }

        /* stop scan backoff timer */
        if(!CONFIG_RX_ON_IDLE)
        {
            Ssf_setScanBackoffClock(0);
        }

        ApiMac_mlmeSetReqBool(ApiMac_attribute_RxOnWhenIdle, CONFIG_RX_ON_IDLE);

        if(CONFIG_FH_ENABLE)
        {
            /* flag to ensure no action for further Async messages received */
            parentFound = true;
        }

        /* device joined */
        if(pJdllcCallbacksCopy && pJdllcCallbacksCopy->pJoinedCb)
        {
            pJdllcCallbacksCopy->pJoinedCb(&devInfo, &parentInfo);
        }
        if(CONFIG_FH_ENABLE)
        {
            if(devInfoBlock.currentJdllcState == Jdllc_states_initRestoring)
            {
                updateState(Jdllc_states_rejoined);
            }
            else if(devInfoBlock.currentJdllcState == Jdllc_states_orphan &&
            		devInfoBlock.prevJdllcState == Jdllc_states_rejoined)
            {
                updateState(Jdllc_states_rejoined);
            }
            else
            {
                updateState(Jdllc_states_joined);
            }
        }
        else
        {
             updateState(Jdllc_states_joined);
        }

        /* for sleepy devices */
        if((!CONFIG_RX_ON_IDLE))
        {
            if((devInfoBlock.currentJdllcState == Jdllc_states_joined) ||
                  (devInfoBlock.currentJdllcState == Jdllc_states_rejoined))
            {
                /* start poll timer  */
                if(CONFIG_FH_ENABLE)
                {
                    randomNum = ((ApiMac_randomByte() << 8) + ApiMac_randomByte());
                    Ssf_setPollClock( (uint32_t) randomNum %
                                      CONFIG_FH_START_POLL_DATA_RAND_WINDOW);
                }
                else
                {
                    Util_setEvent(&Jdllc_events, JDLLC_POLL_EVT);
                }
            }
        }

    }
    else
    {
        if(!CONFIG_FH_ENABLE)
        {
            /* could not associate with parent, scan for new parent */
            if(CONFIG_BEACON_ORDER == JDLLC_BEACON_ORDER_NON_BEACON)
            {
                /* non beacon network */
                if(continueScan == true)
                {
                    switchState(Jdllc_deviceStates_scanActive);
                }
            }
            else if((CONFIG_BEACON_ORDER > 0) && (CONFIG_BEACON_ORDER <
                                         JDLLC_BEACON_ORDER_NON_BEACON))
            {
                /* beacon network */
                switchState(Jdllc_deviceStates_scanPassive);
            }
        }
        else
        {
            fhAssociationAttempts++;
            if(fhAssociationAttempts < CONFIG_FH_MAX_ASSOCIATION_ATTEMPTS)
            {
                Ssf_setFHAssocClock(FH_ASSOC_DELAY);
            }
            else
            {
                parentFound = false;
                /* start trickle timer for PCS */
                Ssf_setTrickleClock(CONFIG_PAN_CONFIG_SOLICIT_CLK_DURATION,
                                    ApiMac_wisunAsyncFrame_configSolicit);
            }
        }
        /* Update stats */
        Sensor_msgStats.joinFails++;
    }

    if(macCallbacksCopy.pAssocCnfCb != NULL)
    {
        macCallbacksCopy.pAssocCnfCb(pData);
    }
}

/*!
 * @brief       Handle Disassociate Confirm callback
 *
 * @param       pData - pointer to Disassociate Confirm structure
 */
static void disassoCnfCb(ApiMac_mlmeDisassociateCnf_t *pData)
{
    if(pData->status == ApiMac_status_success)
    {
        /* stop polling */
        if(!CONFIG_RX_ON_IDLE)
        {
            Ssf_setPollClock(0);
        }
        /* enable looking for new parent */
        parentFound = false;
        updateState(Jdllc_states_initWaiting);
    }

    if(pJdllcCallbacksCopy && pJdllcCallbacksCopy->pDisassocCnfCb)
    {
        pJdllcCallbacksCopy->pDisassocCnfCb(&pData->deviceAddress.addr.extAddr,
                                            pData->status);
    }

    if(macCallbacksCopy.pDisassociateCnfCb != NULL)
    {
        macCallbacksCopy.pDisassociateCnfCb(pData);
    }
}

/*!
 * @brief       callback for Async indication
 *
 * @param       pData - pointer to Async indication structure
 */
static void wsAsyncIndCb(ApiMac_mlmeWsAsyncInd_t *pData)
{
    ApiMac_status_t status;
    ApiMac_payloadIeRec_t *pPayloadGroupRec = NULL;
    uint8_t wisunPiePresent = 0;
    uint8_t netname[APIMAC_FH_NET_NAME_SIZE_MAX] = {0};
    uint16_t panSize;
    uint8_t useParentBSIE;
    uint8_t routingMethod;
    uint8_t routeCost = 0;
    uint8_t eapolReady;
    uint8_t fanTpsVersion;
    uint16_t panVersion;
    uint8_t gtkHash0[GTK_HASH_LEN], gtkHash1[GTK_HASH_LEN];
    uint8_t gtkHash2[GTK_HASH_LEN], gtkHash3[GTK_HASH_LEN];

    /* Parse group IEs */
    status = ApiMac_parsePayloadGroupIEs(pData->pPayloadIE, pData->payloadIeLen,
                                         &pPayloadGroupRec);

    if((status == ApiMac_status_success) && (pPayloadGroupRec != NULL))
    {
        ApiMac_payloadIeRec_t *pGroup = pPayloadGroupRec;

        while(pGroup != NULL)
        {
            if(pGroup->item.ieId == ApiMac_payloadIEGroup_WiSUN)
            {
                ApiMac_payloadIeRec_t *pPayloadSubRec = NULL;

                status = ApiMac_parsePayloadSubIEs(pGroup->item.pIEContent,
                                                   pGroup->item.ieContentLen,
                                                   &pPayloadSubRec);
                if((status == ApiMac_status_success) && (pPayloadSubRec!= NULL))
                {
                    ApiMac_payloadIeRec_t *pSubGroup = pPayloadSubRec;

                    while(pSubGroup != NULL)
                    {
                        uint8_t *pIEContent = pSubGroup->item.pIEContent;

                        switch(pSubGroup->item.ieId)
                        {
                            case ApiMac_wisunSubIE_netNameIE:
                                if(pSubGroup->item.ieContentLen <=
                                    APIMAC_FH_NET_NAME_SIZE_MAX)
                                {
                                    memset(&netname, 0, APIMAC_FH_NET_NAME_SIZE_MAX);
                                    memcpy(&netname, pIEContent, pSubGroup->item.ieContentLen);
                                    wisunPiePresent |= WISUN_NETNAME_IE_PRESENT;
                                }
                                break;

                            case ApiMac_wisunSubIE_PANIE:
                                /* set PAN size */
                                memcpy(&panSize, pIEContent, sizeof(uint16_t));
                                ApiMac_mlmeSetFhReqUint16(
                                                ApiMac_FHAttribute_panSize,
                                                panSize);
                                pIEContent += sizeof(uint16_t);
                                 /*set routing cost */
                                routeCost = *pIEContent;
                                /* set if propagating parent BSIE*/
                                useParentBSIE = ((*pIEContent) >> 0) & 1;
                                /* set routing method */
                                routingMethod = ((*pIEContent) >> 1) & 1;
                                /* set EAPOL ready attribute */
                                eapolReady = ((*pIEContent) >> 2) & 1;
                                 /*set FAN TPS version */
                                fanTpsVersion = ((*pIEContent) >> 5) & 1;
                                wisunPiePresent |= WISUN_PANIE_PRESENT;
                                break;

                            case ApiMac_wisunSubIE_PANVersionIE:
                                memcpy(&panVersion, pIEContent,
                                       sizeof(uint16_t));
                                wisunPiePresent |= WISUN_PANVER_IE_PRESENT;

                                break;

                            case ApiMac_wisunSubIE_GTKHashIE:
                                /* get gtkHas0 */
                                memcpy(gtkHash0, pIEContent, GTK_HASH_LEN);
                                pIEContent += GTK_HASH_LEN;
                                /* get gtkHash1 */
                                memcpy(gtkHash1, pIEContent, GTK_HASH_LEN);
                                pIEContent += GTK_HASH_LEN;
                                /* get gtkHash2 */
                                memcpy(gtkHash2, pIEContent, GTK_HASH_LEN);
                                pIEContent += GTK_HASH_LEN;
                                /* get gtkHash3 */
                                memcpy(gtkHash3, pIEContent, GTK_HASH_LEN);
                                wisunPiePresent |= WISUN_GTKHASH_IE_PRESENT;
                                break;
                        }

                        /* move to the next item*/
                        pSubGroup = pSubGroup->pNext;
                    }

                  /* Free the IE List allocated by ApiMac_parsePayloadSubIEs() */
                    ApiMac_freeIEList(pPayloadSubRec);
                }
            }

            /* Move to next item*/
            pGroup = pGroup->pNext;
        }

         /* Free the IE List allocated by ApiMac_parsePayloadGroupIEs() */
        ApiMac_freeIEList(pPayloadGroupRec);
    }

    if((pData->fhFrameType != ApiMac_fhFrameType_config) &&
       ((!(wisunPiePresent & WISUN_NETNAME_IE_PRESENT)) ||
       (memcmp(netname, fhNetname, APIMAC_FH_NET_NAME_SIZE_MAX) != 0)))
    {
        /* Drop PAS, PCS or PA from other networks */
        return;
    }

    if(pData->fhFrameType == ApiMac_fhFrameType_panAdvertSolicit)
    {
        fhNumPASRcvdInTrickleWindow++;
    }

    if(pData->fhFrameType == ApiMac_fhFrameType_configSolicit)
    {
        fhNumPCSRcvdInTrickleWindow++;
    }

    if(pData->fhFrameType == ApiMac_fhFrameType_panAdvert)
    {
        /* PA is received , increment statistics */
        Jdllc_statistics.fhNumPAReceived++;
        if(wisunPiePresent & WISUN_PANIE_PRESENT)
        {
            /*
             set PAN coordinator source address if
             route cost = 0
            */
            if(routeCost == 0)
            {
                memcpy(devInfoBlock.coordExtAddr,
                       pData->srcAddr.addr.extAddr,
                       APIMAC_SADDR_EXT_LEN);
            }

            ApiMac_mlmeSetFhReqUint8(
                            ApiMac_FHAttribute_routingCost,
                            routeCost++);
            ApiMac_mlmeSetFhReqUint8(
                            ApiMac_FHAttribute_useParentBSIE,
                            useParentBSIE);
            ApiMac_mlmeSetFhReqUint8(
                           ApiMac_FHAttribute_routingMethod,
                           routingMethod);
            ApiMac_mlmeSetFhReqUint8(
                            ApiMac_FHAttribute_eapolReady,
                            eapolReady);
            ApiMac_mlmeSetFhReqUint8(
                           ApiMac_FHAttribute_fanTPSVersion,
                           fanTpsVersion);
        }

        /* set PIB to track parent */
        ApiMac_mlmeSetFhReqArray(ApiMac_FHAttribute_trackParentEUI,
                                 (uint8_t *) (pData->srcAddr.addr.extAddr));
        /* set PAN */
        ApiMac_mlmeSetReqUint16(ApiMac_attribute_panId, pData->srcPanId);
        /* add parent to security device table */
        Jdllc_addSecDevice(pData->srcPanId, pData->srcAddr.addr.shortAddr,
                           &(pData->srcAddr.addr.extAddr), pData->frameCntr);
        /* set join related PIBS */
        devInfoBlock.panID = pData->srcPanId;
        memcpy(devInfoBlock.coordExtAddr, pData->srcAddr.addr.extAddr,
               (APIMAC_SADDR_EXT_LEN));

        if(parentFound == false)
        {
            /* Stop PAS Timer */
            Ssf_setTrickleClock(0, ApiMac_wisunAsyncFrame_advertisementSolicit);
            /* set trickle timer for PCS */
            Ssf_setTrickleClock(CONFIG_PAN_CONFIG_SOLICIT_CLK_DURATION,
                                ApiMac_wisunAsyncFrame_configSolicit);
        }

    }
    else if(pData->fhFrameType == ApiMac_fhFrameType_config)
    {
        if(wisunPiePresent & WISUN_PANVER_IE_PRESENT)
        {
            ApiMac_mlmeSetFhReqUint16(
                            ApiMac_FHAttribute_panVersion,
                            panVersion);
        }

        if(wisunPiePresent & WISUN_GTKHASH_IE_PRESENT)
        {
            ApiMac_mlmeSetFhReqArray(
                            ApiMac_FHAttribute_gtk0Hash,
                            gtkHash0);
            ApiMac_mlmeSetFhReqArray(
                            ApiMac_FHAttribute_gtk1Hash,
                            gtkHash1);
            ApiMac_mlmeSetFhReqArray(
                            ApiMac_FHAttribute_gtk2Hash,
                            gtkHash2);
            ApiMac_mlmeSetFhReqArray(
                            ApiMac_FHAttribute_gtk3Hash,
                            gtkHash3);
        }

        /* PC is received , parse IEs and increment statistics */
        Jdllc_statistics.fhNumPANConfigReceived++;
        /* set PIB to track parent */
        ApiMac_mlmeSetFhReqArray(ApiMac_FHAttribute_trackParentEUI,
                                 (uint8_t *) (pData->srcAddr.addr.extAddr));
        /* stop PCS Timer */
        Ssf_setTrickleClock(0, ApiMac_wisunAsyncFrame_configSolicit);

        if(parentFound == false)
        {
            parentFound = true;
            fhAssociationAttempts = 0;
            /* Send association request if not associated previously */
            /* Increase delay for the first association attempt */
            Ssf_setFHAssocClock(FH_ASSOC_DELAY + FH_ASSOC_DELAY);
        }
    }

    if(macCallbacksCopy.pWsAsyncIndCb != NULL)
    {
        macCallbacksCopy.pWsAsyncIndCb(pData);
    }
}

/*!
 * @brief       Send Association request
 */
static void sendAssocReq(void)
{
    ApiMac_mlmeAssociateReq_t assocReq;
    memset(&assocReq, 0, sizeof(ApiMac_mlmeAssociateReq_t));
    assocReq.coordPanId = devInfoBlock.panID;

    if(CONFIG_FH_ENABLE)
    {
        assocReq.logicalChannel = 0;
        assocReq.coordAddress.addrMode = ApiMac_addrType_extended;
        memcpy(assocReq.coordAddress.addr.extAddr, devInfoBlock.coordExtAddr,
        (APIMAC_SADDR_EXT_LEN));
    }
    else
    {
        assocReq.logicalChannel = devInfoBlock.channel;
        assocReq.coordAddress.addrMode = ApiMac_addrType_short;
        assocReq.coordAddress.addr.shortAddr = devInfoBlock.coordShortAddr;
    }

    assocReq.channelPage = CONFIG_CHANNEL_PAGE;
    assocReq.phyID = CONFIG_PHY_ID;
    assocReq.sec.securityLevel = ApiMac_secLevel_none;
    assocReq.capabilityInformation.allocAddr = true;
    assocReq.capabilityInformation.ffd = false;
    assocReq.capabilityInformation.panCoord = false;
    assocReq.capabilityInformation.rxOnWhenIdle = CONFIG_RX_ON_IDLE;
    ApiMac_mlmeAssociateReq(&assocReq);

    /* Update stats */
    Sensor_msgStats.joinAttempts++;
}

/*!
 * @brief       Process sync loss callback
 *
 * @param       pData - Pointer to sync loss callback structure
 */
static void syncLossCb(ApiMac_mlmeSyncLossInd_t *pData)
{
    /* Update stats */
    Sensor_msgStats.syncLossIndications++;

    if(!CONFIG_RX_ON_IDLE)
    {
        /* Stop polling */
        Ssf_setPollClock(0);
    }
    /* set up orphan scan */
    switchState(Jdllc_deviceStates_scanOrphan);
    updateState(Jdllc_states_orphan);

    if(macCallbacksCopy.pSyncLossIndCb != NULL)
    {
        macCallbacksCopy.pSyncLossIndCb(pData);
    }
}

/*!
 * @brief       Update Jdllc state
 *
 * @param       state - new state
 */
static void updateState(Jdllc_states_t state)
{
    if(state != devInfoBlock.currentJdllcState)
    {
        devInfoBlock.prevJdllcState = devInfoBlock.currentJdllcState;
        devInfoBlock.currentJdllcState = state;
        if(pJdllcCallbacksCopy && pJdllcCallbacksCopy->pStateChangeCb)
        {
            /* state change callback */
            pJdllcCallbacksCopy->pStateChangeCb(devInfoBlock.currentJdllcState);
        }
    }
}

/*!
 * @brief       Process data confirm callback
 *
 * @param       pData - pointer to data confirm structure
 */
static void dataCnfCb(ApiMac_mcpsDataCnf_t *pData)
{
    if(pData->status == ApiMac_status_noAck)
    {
        /* track the number of failures  */
        devInfoBlock.dataFailures++;
        if(devInfoBlock.dataFailures == CONFIG_MAX_DATA_FAILURES)
        {
            if(!CONFIG_RX_ON_IDLE)
            {
                /* stop polling */
                Ssf_setPollClock(0);
            }

            ApiMac_mlmeSetReqBool(ApiMac_attribute_RxOnWhenIdle, true);

            if(CONFIG_FH_ENABLE)
            {
                updateState(Jdllc_states_orphan);
                parentFound = false;
                /* Stop the reporting timer */
                Ssf_setReadingClock(0);
                /* start trickle timer for PCS */
                Ssf_setTrickleClock(CONFIG_PAN_CONFIG_SOLICIT_CLK_DURATION,
                                    ApiMac_wisunAsyncFrame_configSolicit);
            }
            else
            {
                /* start orphan scan */
                switchState(Jdllc_deviceStates_scanOrphan);
                updateState(Jdllc_states_orphan);
            }

            devInfoBlock.dataFailures = 0;
        }
    }
    else if(pData->status == ApiMac_status_success)
    {
        devInfoBlock.dataFailures = 0;
    }

    if(macCallbacksCopy.pDataCnfCb != NULL)
    {
        macCallbacksCopy.pDataCnfCb(pData);
    }
}

/*!
 * @brief       Send Poll request
 */
static void sendPollReq()
{
    ApiMac_mlmePollReq_t pollReq;
    memset(&pollReq, 0, sizeof(ApiMac_mlmePollReq_t));
    pollReq.coordPanId = devInfoBlock.panID;
    if(CONFIG_FH_ENABLE)
    {
        pollReq.coordAddress.addrMode = ApiMac_addrType_extended;
        memcpy(pollReq.coordAddress.addr.extAddr, devInfoBlock.coordExtAddr,
        APIMAC_SADDR_EXT_LEN);
    }
    else
    {
        pollReq.coordAddress.addrMode = ApiMac_addrType_short;
        pollReq.coordAddress.addr.shortAddr = devInfoBlock.coordShortAddr;
    }
    ApiMac_mlmePollReq(&pollReq);
}

/*!
 * @brief       Process coordinator realignment
 */
static void processCoordRealign(void)
{
    uint16_t panID = 0;
    /* read PAN ID from PIB */
    ApiMac_mlmeGetReqUint16(ApiMac_attribute_panId, &panID);

    if(panID == devInfoBlock.panID)
    {
        /* transition to correct non orphan state */
        if(devInfoBlock.prevJdllcState == Jdllc_states_joined)
        {
            updateState(Jdllc_states_joined);
        }
        else if(devInfoBlock.prevJdllcState == Jdllc_states_rejoined)
        {
            updateState(Jdllc_states_rejoined);
        }
        else if(devInfoBlock.prevJdllcState == Jdllc_states_initRestoring)
        {
            ApiMac_deviceDescriptor_t devInfo;
            Llc_netInfo_t parentNetInfo;

            memcpy(devInfo.extAddress, devInfoBlock.devExtAddr,
                   (APIMAC_SADDR_EXT_LEN));
            devInfo.panID = devInfoBlock.panID;
            devInfo.shortAddress = devInfoBlock.devShortAddr;

            memcpy(parentNetInfo.devInfo.extAddress, devInfoBlock.coordExtAddr,
                   (APIMAC_SADDR_EXT_LEN));
            parentNetInfo.channel = devInfoBlock.channel;
            parentNetInfo.fh = CONFIG_FH_ENABLE;
            parentNetInfo.devInfo.panID = devInfoBlock.panID;
            parentNetInfo.devInfo.shortAddress = devInfoBlock.coordShortAddr;

            /* device joined */
            if(pJdllcCallbacksCopy && pJdllcCallbacksCopy->pJoinedCb)
            {
                pJdllcCallbacksCopy->pJoinedCb(&devInfo, &parentNetInfo);
            }
            updateState(Jdllc_states_rejoined);
        }

        /* if sleepy device before setting poll timer */
        if((!CONFIG_RX_ON_IDLE))
        {
            /* start polling if parent matches*/
            Ssf_setPollClock(devInfoBlock.pollInterval);
        }

        ApiMac_mlmeSetReqBool(ApiMac_attribute_RxOnWhenIdle, CONFIG_RX_ON_IDLE);
    }
    else
    {
        /* orphan scan */
        switchState(Jdllc_deviceStates_scanOrphan);
        updateState(Jdllc_states_orphan);
    }
}

/*!
 * @brief       Process poll confirm callback
 *
 * @param       pData - pointer to poll confirm structure
 */
static void pollCnfCb(ApiMac_mlmePollCnf_t *pData)
{
    if((pData->status == ApiMac_status_noData) ||
       (pData->status == ApiMac_status_success))
    {
        if(!CONFIG_FH_ENABLE)
        {
            if(devInfoBlock.currentJdllcState == Jdllc_states_initRestoring)
            {
                ApiMac_deviceDescriptor_t devInfo;
                Llc_netInfo_t parentNetInfo;

                memcpy(devInfo.extAddress, devInfoBlock.devExtAddr,
                       (APIMAC_SADDR_EXT_LEN));
                devInfo.panID = devInfoBlock.panID;
                devInfo.shortAddress = devInfoBlock.devShortAddr;

                memcpy(parentNetInfo.devInfo.extAddress, devInfoBlock.coordExtAddr,
                       (APIMAC_SADDR_EXT_LEN));
                parentNetInfo.channel = devInfoBlock.channel;
                parentNetInfo.fh = CONFIG_FH_ENABLE;
                parentNetInfo.devInfo.panID = devInfoBlock.panID;
                parentNetInfo.devInfo.shortAddress = devInfoBlock.coordShortAddr;

                ApiMac_mlmeSetReqBool(ApiMac_attribute_RxOnWhenIdle,
                                      CONFIG_RX_ON_IDLE);

                /* device joined */
                if(pJdllcCallbacksCopy && pJdllcCallbacksCopy->pJoinedCb)
                {
                    pJdllcCallbacksCopy->pJoinedCb(&devInfo, &parentNetInfo);
                }

                updateState(Jdllc_states_rejoined);

                if((!CONFIG_RX_ON_IDLE))
                {
                    /* set event for polling if sleepy device*/
                    Util_setEvent(&Jdllc_events, JDLLC_POLL_EVT);
                }
            }
        }
        devInfoBlock.dataFailures = 0;
    }
    else if(pData->status == ApiMac_status_noAck)
    {
        /* track the number of failures  */
        devInfoBlock.dataFailures++;

        if((devInfoBlock.currentJdllcState == Jdllc_states_joined)
           || (devInfoBlock.currentJdllcState == Jdllc_states_rejoined))
        {
            if(!CONFIG_FH_ENABLE)
            {
                /* retry poll with shorter interval in busy network */
                Ssf_setPollClock(JDLLC_RETRY_POLL);
            }
        }

        if(devInfoBlock.dataFailures == CONFIG_MAX_DATA_FAILURES)
        {
            if(!CONFIG_RX_ON_IDLE)
            {
                /* stop polling */
                Ssf_setPollClock(0);
            }

            ApiMac_mlmeSetReqBool(ApiMac_attribute_RxOnWhenIdle, true);

            if(CONFIG_FH_ENABLE)
            {
                updateState(Jdllc_states_orphan);
                parentFound = false;
                /* Stop the reporting timer */
                Ssf_setReadingClock(0);
                /* start trickle timer for PCS */
                Ssf_setTrickleClock(CONFIG_PAN_CONFIG_SOLICIT_CLK_DURATION,
                                    ApiMac_wisunAsyncFrame_configSolicit);
            }
            else
            {
                /* start orphan scan */
                switchState(Jdllc_deviceStates_scanOrphan);
                updateState(Jdllc_states_orphan);
            }

            devInfoBlock.dataFailures = 0;
        }
        else
        {
            if(devInfoBlock.currentJdllcState == Jdllc_states_initRestoring)
            {
                if((!CONFIG_RX_ON_IDLE) && (!CONFIG_FH_ENABLE))
                {
                    /* set event for polling if sleepy device*/
                    Util_setEvent(&Jdllc_events, JDLLC_POLL_EVT);
                }
            }
        }
    }
    else if(pData->status == ApiMac_status_channelAccessFailure)
    {
        if(!CONFIG_FH_ENABLE)
        {
            /* retry poll with shorter interval in busy network */
            Ssf_setPollClock(JDLLC_RETRY_POLL);
        }
    }

    if(macCallbacksCopy.pPollCnfCb != NULL)
    {
        macCallbacksCopy.pPollCnfCb(pData);
    }
}

/*!
 * @brief       Process Disassociate Indication callback
 *
 * @param       pData - pointer to disassociation indication structure
 */
static void disassocIndCb(ApiMac_mlmeDisassociateInd_t *pData)
{
    if(!CONFIG_RX_ON_IDLE)
    {
        /* stop polling */
        Ssf_setPollClock(0);
    }
    /* enable looking for new parent */
    parentFound = false;
    updateState(Jdllc_states_initWaiting);

    /* pass indication to app */
    if(pJdllcCallbacksCopy && pJdllcCallbacksCopy->pDisassocIndCb)
    {
        pJdllcCallbacksCopy->pDisassocIndCb(&pData->deviceAddress,
                                            pData->disassociateReason);
    }

    if(macCallbacksCopy.pDisassociateIndCb != NULL)
    {
        macCallbacksCopy.pDisassociateIndCb(pData);
    }
}

/*!
 * @brief       Send scan request
 *
 * @param       type - type of scan: active ,passive or orphan
 */
static void sendScanReq(ApiMac_scantype_t type)
{
    ApiMac_mlmeScanReq_t scanReq;
    /* set common parameters for all scans */
    memset(&scanReq, 0, sizeof(ApiMac_mlmeScanReq_t));
    /* set scan channels from channel mask*/
    memcpy(scanReq.scanChannels, defaultChannelMask,
           APIMAC_154G_CHANNEL_BITMAP_SIZ);
    scanReq.scanType = type;
    scanReq.scanDuration = CONFIG_SCAN_DURATION;
    scanReq.maxResults = 0;/* Expecting beacon notifications */
    scanReq.permitJoining = false;
    scanReq.linkQuality = CONFIG_LINKQUALITY;
    scanReq.percentFilter = CONFIG_PERCENTFILTER;
    scanReq.channelPage = CONFIG_CHANNEL_PAGE;
    scanReq.phyID = CONFIG_PHY_ID;
    /* using no security for scan request command */
    memset(&scanReq.sec, 0, sizeof(ApiMac_sec_t));
    /* send scan Req */
    ApiMac_mlmeScanReq(&scanReq);
}

/*!
 * @brief       Send Async request command
 *
 * @param       frameType - type of async frame to be sent
 */
static void sendAsyncReq(ApiMac_wisunAsyncFrame_t frameType)
{
    ApiMac_mlmeWSAsyncReq_t asyncReq;
    uint8_t sizeOfChannelMask;
    uint8_t asyncChannelMask[] = FH_ASYNC_CHANNEL_MASK;

    /* set of Exclude Channels */
    sizeOfChannelMask = sizeof(asyncChannelMask)/sizeof(uint8_t);
    if(sizeOfChannelMask > APIMAC_154G_CHANNEL_BITMAP_SIZ)
    {
        sizeOfChannelMask = APIMAC_154G_CHANNEL_BITMAP_SIZ;
    }
    memset(asyncReq.channels, 0, (APIMAC_154G_CHANNEL_BITMAP_SIZ));
    asyncReq.operation = ApiMac_wisunAsycnOperation_start;
    memcpy(asyncReq.channels, asyncChannelMask, sizeOfChannelMask);
    memset(asyncReq.sec.keySource, 0, APIMAC_KEY_SOURCE_MAX_LEN);

    /* send PAS or PCS according to frame type */
    if(frameType == ApiMac_wisunAsyncFrame_advertisementSolicit)
    {
        /* Fill in the information for async request */
        asyncReq.frameType = ApiMac_wisunAsyncFrame_advertisementSolicit;
        /* no security for PAS */
        asyncReq.sec.securityLevel = ApiMac_secLevel_none;
        Jdllc_statistics.fhNumPASolicitSent++;
    }
    else if(frameType == ApiMac_wisunAsyncFrame_configSolicit)
    {
        /* Fill in the information for async request */
        asyncReq.frameType = ApiMac_wisunAsyncFrame_configSolicit;
        /* no security for PAS */
        asyncReq.sec.securityLevel = ApiMac_secLevel_none;
        Jdllc_statistics.fhNumPANConfigSolicitsSent++;
    }

    ApiMac_mlmeWSAsyncReq(&asyncReq);
}
