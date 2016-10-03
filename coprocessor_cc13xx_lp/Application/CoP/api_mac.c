/******************************************************************************

 @file api_mac.c

 @brief TIMAC 2.0 API

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

#include "icall.h"
#include "api_mac.h"
#include "macstack.h"
#include "util.h"
#include "macs.h"

/*!
 This module is the ICall interface for the application and all ICall
 activity must go through this module, no ICall activity anywhere else.
 */

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

/*! Capability Information - Device is capable of becoming a PAN coordinator */
#define CAPABLE_PAN_COORD       0x01
/*! Capability Information - Device is an FFD  */
#define CAPABLE_FFD             0x02
/*!
 Capability Information - Device is mains powered rather than battery powered
 */
#define CAPABLE_MAINS_POWER     0x04
/*! Capability Information - Device has its receiver on when idle  */
#define CAPABLE_RX_ON_IDLE      0x08
/*!
 Capability Information - Device is capable of sending
 and receiving secured frames
 */
#define CAPABLE_SECURITY        0x40
/*!
 Capability Information - Request allocation of a short address in the
 associate procedure
 */
#define CAPABLE_ALLOC_ADDR      0x80

/*! Offset into the payload for the payload IEs */
#define PAYLOAD_IE_OFFSET                    0
/*! Offset into the IE for the subIE */
#define PAYLOAD_IE_SUBIE_OFFSET              0

/*! Macro to get the IE Type */
#define PAYLOAD_IE_TYPE(p) (((p)[PAYLOAD_IE_OFFSET+1] >> 7) & 0x01)

/*! Macro to get the IE Group ID */
#define PAYLOAD_IE_GROUP_ID(p) ((((uint8_t *)p)[PAYLOAD_IE_OFFSET+1] >> 3) & 0x0f)

/*! Macro to get the IE Content Length */
#define PAYLOAD_IE_CONTENT_LEN(p) ((((uint8_t *)p)[PAYLOAD_IE_OFFSET+0] & 0x00ff) +\
                ((((uint8_t *)p)[PAYLOAD_IE_OFFSET+1] & 0x0007) << 8))

/*! Type value for payload IE */
#define PAYLOAD_IE_TYPE_VAL 1
/*! Type value for payload IE */
#define PAYLOAD_IE_HEADER_LEN 2

/*! Macro to get the short subIE length */
#define PAYLOAD_IE_SHORT_SUBIE_LEN(p) ((p)[PAYLOAD_IE_SUBIE_OFFSET+0])

/*! Macro to get the long subIE length */
#define PAYLOAD_IE_LONG_SUBIE_LEN(p) (((p)[PAYLOAD_IE_OFFSET+0] & 0x00ff) + \
                              (((p)[PAYLOAD_IE_OFFSET+1] & 0x0007)  << 8))

/*! Macro to get the subIE type */
#define PAYLOAD_IE_SUBIE_TYPE(p) (((p)[PAYLOAD_IE_SUBIE_OFFSET+1] >> 7) & 0x01)

/*! Macro to get the subIE ID */
#define PAYLOAD_IE_SUBIE_ID(p) ((p)[PAYLOAD_IE_SUBIE_OFFSET+1] & 0x7f)

/*! subIE header length */
#define PAYLOAD_SUB_IE_HEADER_LEN 2

/*! Short subIE type */
#define PAYLOAD_SUB_ID_IE_TYPE_SHORT 0
/*! Long subIE type */
#define PAYLOAD_SUB_ID_IE_TYPE_LONG 1

/*! Short subIE header length */
#define PAYLOAD_SUB_ID_IE_SHORT_HEADER_LEN  2

/*! Payload IE SubIE Type Size */
#define PAYLOAD_IE_SUB_IE_TYPE_SIZE 1
/*! Payload IE SubIE Type Position */
#define PAYLOAD_IE_SUB_IE_TYPE_POSITION 15
/*! Payload IE SubIE ID Short Size */
#define PAYLOAD_IE_SUB_IE_ID_SHORT_SIZE 7
/*! Payload IE SubIE ID Short Position */
#define PAYLOAD_IE_SUB_IE_ID_SHORT_POSITION 8
/*! Payload IE SubIE Short Length Size */
#define PAYLOAD_IE_SUB_IE_LEN_SHORT_SIZE 8
/*! Payload IE SubIE Short Length Position */
#define PAYLOAD_IE_SUB_IE_LEN_SHORT_POSITION 0
/*! Payload IE SubIE ID Long Size */
#define PAYLOAD_IE_SUB_IE_ID_LONG_SIZE 4
/*! Payload IE SubIE ID Long Position */
#define PAYLOAD_IE_SUB_IE_SUB_ID_LONG_POSITION 11
/*! Payload IE SubIE ID Long Length Size */
#define PAYLOAD_IE_SUB_IE_LEN_LONG_SIZE 11
/*! Payload IE SubIE Long Length Position */
#define PAYLOAD_IE_SUB_IE_LEN_LONG_POSITION 0

/*! Unpack a field from a uint16_t */
#define IE_UNPACKING(var,size,position) (((uint16_t)(var)>>(position))\
                &(((uint16_t)1<<(size))-1))

/*! Make a uint16_t from 2 uint8_t */
#define MAKE_UINT16(low,high) (((low)&0x00FF)|(((high)&0x00FF)<<8))

/*! Get the SubIE type field (bool) */
#define GET_SUBIE_TYPE(ctl) (bool)(IE_UNPACKING(ctl,\
                 PAYLOAD_IE_SUB_IE_TYPE_SIZE, PAYLOAD_IE_SUB_IE_TYPE_POSITION))

/*! Get the SubIE Long ID  */
#define GET_SUBIE_ID_LONG(ctl) (uint8_t)(IE_UNPACKING(ctl,\
       PAYLOAD_IE_SUB_IE_ID_LONG_SIZE, PAYLOAD_IE_SUB_IE_SUB_ID_LONG_POSITION))

/*! Get the SubIE Long Length */
#define GET_SUBIE_LEN_LONG(ctl) (uint16_t)(IE_UNPACKING(ctl,\
         PAYLOAD_IE_SUB_IE_LEN_LONG_SIZE, PAYLOAD_IE_SUB_IE_LEN_LONG_POSITION))

/*! Get the SubIE Short ID  */
#define GET_SUBIE_ID_SHORT(ctl) (uint8_t)(IE_UNPACKING(ctl,\
         PAYLOAD_IE_SUB_IE_ID_SHORT_SIZE, PAYLOAD_IE_SUB_IE_ID_SHORT_POSITION))

/*! Get the SubIE Short Length */
#define GET_SUBIE_LEN_SHORT(ctl) (uint16_t)(IE_UNPACKING(ctl,\
       PAYLOAD_IE_SUB_IE_LEN_SHORT_SIZE, PAYLOAD_IE_SUB_IE_LEN_SHORT_POSITION))

/******************************************************************************
 Structures
 *****************************************************************************/

/******************************************************************************
 Global variables
 *****************************************************************************/

/*! ICall thread entity */
ICall_EntityID ApiMac_appEntity = 0;

/*!
 The ApiMac_extAddr is the MAC's IEEE address, setup with the Chip's
 IEEE addresses in main.c
 */
ApiMac_sAddrExt_t ApiMac_extAddr;

/******************************************************************************
 Local variables
 *****************************************************************************/
/*! Semaphore used to post events to the application thread */
STATIC ICall_Semaphore sem;

/*! MAC callback table, initialized to no callback table */
STATIC ApiMac_callbacks_t *pMacCallbacks = (ApiMac_callbacks_t *) NULL;

/*! Place to hold the scan results */
STATIC void *scanResults = (void *) NULL;

STATIC ICall_EntityID macEntityID;

/******************************************************************************
 Local Function Prototypes
 *****************************************************************************/
static ApiMac_status_t mlmeGetFhReq(uint16_t pibAttribute, void *pValue,
                                    uint16_t *pLen);
static ApiMac_status_t getTypeReq(uint8 eventId, ICall_MsgMatchFn matchFn,
                                  uint8_t pibAttribute,
                                  void *pValue, uint16_t *pLen);
static uint16_t processIncomingICallMsg(macCbackEvent_t *pMsg);
static void copyMacSecToApiMacSec(ApiMac_sec_t *pDst, macSec_t *pSrc);
static void copyMacAddrToApiMacAddr(ApiMac_sAddr_t *pDst, sAddr_t *pSrc);
static void copyMacPanDescToApiMacPanDesc(ApiMac_panDesc_t *pDst,
                                          macPanDesc_t *pSrc);
static void processBeaconNotifyInd(macMlmeBeaconNotifyInd_t *pInd);
static void processScanCnf(macMlmeScanCnf_t *pCnf);
static void macMsgDeallocate(macEventHdr_t *pData);
static void copyDataInd(ApiMac_mcpsDataInd_t *pDst, macMcpsDataInd_t *pSrc);
static bool matchMcpsDataAlloc(ICall_ServiceEnum src, ICall_EntityID dest,
                               const void *msg);
static macMcpsDataReq_t *mcpsDataAllocMsg(uint16_t len,
                                          uint8_t securityLevel,
                                          uint8_t keyIdMode,
                                          uint32 includeFhIEs,
                                          uint16_t payloadIeLen);
static void copyApiMacSecToMacSec(macSec_t *pDst, ApiMac_sec_t *pSrc);
static void copyApiMacAddrToMacAddr(sAddr_t *pDst, ApiMac_sAddr_t *pSrc);
static bool matchGetReq(ICall_ServiceEnum src, ICall_EntityID dest,
                        const void *msg);
static bool matchGetFhReq(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg);
static bool matchGetSecurityReq(ICall_ServiceEnum src, ICall_EntityID dest,
                                const void *msg);
static bool matchResetReq(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg);
static bool matchSetReq(ICall_ServiceEnum src, ICall_EntityID dest,
                        const void *msg);
static bool matchSetFhReq(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg);
static bool matchSetSecurityReq(ICall_ServiceEnum src, ICall_EntityID dest,
                                const void *msg);
static bool matchRandomByteReq(ICall_ServiceEnum src, ICall_EntityID dest,
                               const void *msg);
static bool matchEnableFH(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg);
static bool matchSecAddDevice(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg);
static bool matchSecDelDevice(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg);
static bool matchSecDelKeyAndDevices(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg);
static bool matchSecDelAllDevices(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg);
static bool matchSecGetDefaultSrcKey(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg);
static bool matchSecAddKeyInitFC(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg);
static ApiMac_status_t setTypeReq(uint8 eventId, ICall_MsgMatchFn matchFn,
                                  uint8_t pibAttribute,
                                  void *pValue);
static ApiMac_status_t sendEvtExpectStatus(uint8_t eventId,
                                           ICall_MsgMatchFn matchFn);
static ApiMac_status_t sendEvt(uint8_t eventId);
static ApiMac_status_t parsePayloadIEs(uint8_t *pContent, uint16_t contentLen,
                                       ApiMac_payloadIeRec_t **pList,
                                       bool group);
static uint16_t convertTxOptions(ApiMac_txOptions_t txOptions);
static ApiMac_status_t mlmeSetFhReq(uint16_t pibAttribute, void *pValue);

/******************************************************************************
 Public Functions
 *****************************************************************************/

/*!
 Initialize this module.

 Public function defined in api_mac.h
 */
void *ApiMac_init(bool enableFH)
{
    /* Allocate message buffer space */
    macStackInitParams_t *pMsg = (macStackInitParams_t *)ICall_allocMsg(
                    sizeof(macStackInitParams_t));

    /* Register the current thread as an ICall dispatcher application
     * so that the application can send and receive messages.
     */
    ICall_registerApp(&ApiMac_appEntity, &sem);

    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = MAC_STACK_INIT_PARAMS;
        pMsg->hdr.status = 0;
        pMsg->srctaskid = ApiMac_appEntity;
        pMsg->retransmit = 0;
        pMsg->pendingMsg = 0;
        pMsg->pMacCbackQueryRetransmit = NULL;
        pMsg->pMacCbackCheckPending = NULL;

        /* Send the message to ICALL_SERVICE_CLASS_TIMAC */
        ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                             (ICALL_MSG_FORMAT_3RD_CHAR_TASK_ID),
                             pMsg);
    }

    /* Enable frequency hopping? */
    if(enableFH)
    {
        ApiMac_enableFH();
    }

    /* Reset the MAC */
    ApiMac_mlmeResetReq(true);

    /* Set the device IEEE address */
    ApiMac_mlmeSetReqArray(ApiMac_attribute_extendedAddress, ApiMac_extAddr);

    /*
     Save the MAC Stack ICall Entity ID to be used to check received
     messages.
     */
    macEntityID = ICall_searchServiceEntity(ICALL_SERVICE_CLASS_TIMAC);

    return (sem);
}

/*!
 Register for MAC callbacks.

 Public function defined in api_mac.h
 */
void ApiMac_registerCallbacks(ApiMac_callbacks_t *pCallbacks)
{
    /* Save the application's callback table */
    pMacCallbacks = pCallbacks;
}

/*!
 Register for MAC callbacks.

 Public function defined in api_mac.h
 */
void ApiMac_processIncoming(void)
{
    ICall_EntityID src;
    ICall_EntityID dest;
    macCbackEvent_t *pMsg;

    /* Wait for response message */
    if(ICall_wait(ICALL_TIMEOUT_FOREVER) == ICALL_ERRNO_SUCCESS)
    {
        /* Retrieve the response message */
        if(ICall_fetchMsg(&src, &dest, (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
        {
            if(dest == ApiMac_appEntity)
            {
                if(src == macEntityID)
                {
                    /* Process the message from the MAC stack */
                    processIncomingICallMsg(pMsg);
                }
                else if(pMacCallbacks->pUnprocessedCb)
                {
                    /* Initiate the unprocessed message callback */
                    pMacCallbacks->pUnprocessedCb((uint16_t)src, 0,
                                                  (void *)pMsg);
                }
            }

            if(pMsg != NULL)
            {
                ICall_freeMsg(pMsg);
            }
        }
    }
}

/*!
 This function sends application data to the MAC for
 transmission in a MAC data frame.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mcpsDataReq(ApiMac_mcpsDataReq_t *pData)
{
    ApiMac_status_t ret = ApiMac_status_noResources;
    macMcpsDataReq_t *pMsg;

    pMsg = mcpsDataAllocMsg(pData->msdu.len, pData->sec.securityLevel,
                            pData->sec.keyIdMode,
                            pData->includeFhIEs,
                            pData->payloadIELen);
    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = MAC_MCPS_DATA_REQ;
        pMsg->hdr.status = 0;

        memcpy(pMsg->msdu.p, pData->msdu.p, pData->msdu.len);

        copyApiMacSecToMacSec(&(pMsg->sec), &(pData->sec));

        copyApiMacAddrToMacAddr(&(pMsg->mac.dstAddr), &(pData->dstAddr));
        pMsg->mac.dstPanId = pData->dstPanId;
        pMsg->mac.srcAddrMode = pData->srcAddrMode;
        pMsg->mac.msduHandle = pData->msduHandle;
        pMsg->mac.txOptions = convertTxOptions(pData->txOptions);
        pMsg->mac.channel = pData->channel;
        pMsg->mac.power = pData->power;

        pMsg->mac.payloadIELen = pData->payloadIELen;
        if(pMsg->mac.payloadIELen)
        {
            memcpy(pMsg->mac.pIEList, pData->pIEList, pMsg->mac.payloadIELen);
        }
        pMsg->mac.fhProtoDispatch = pData->fhProtoDispatch;
        pMsg->mac.includeFhIEs = pData->includeFhIEs;

        /* Skip FEATURE_GREEN_POWER for now */

        /* Send the message */
        if(ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                                (ICALL_MSG_FORMAT_KEEP),
                                pMsg)
           == ICALL_ERRNO_SUCCESS)
        {
            ret = ApiMac_status_success;
        }
    }

    return (ret);
}

/*!
 This function purges and discards a data request from the MAC
 data queue.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mcpsPurgeReq(uint8_t msduHandle)
{
    ApiMac_status_t ret = ApiMac_status_noResources;
    /* Allocate message buffer space */
    macPurgeReq_t *pMsg = (macPurgeReq_t *)ICall_allocMsg(
                    sizeof(macPurgeReq_t));

    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = MAC_MCPS_PURGE_REQ;
        pMsg->hdr.status = 0;
        pMsg->msduhandle = msduHandle;

        /* Send the message */
        if(ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                                (ICALL_MSG_FORMAT_KEEP),
                                pMsg)
           == ICALL_ERRNO_SUCCESS)
        {
            ret = ApiMac_status_success;
        }
    }

    return (ret);
}

/*!
 This function sends an associate request to a coordinator
 device.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeAssociateReq(ApiMac_mlmeAssociateReq_t *pData)
{
    ApiMac_status_t ret = ApiMac_status_noResources;
    /* Allocate message buffer space */
    macMlmeAssociateReqEvt_t *pMsg = (macMlmeAssociateReqEvt_t *)ICall_allocMsg(
                    sizeof(macMlmeAssociateReqEvt_t));

    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = MAC_MLME_ASSOCIATE_REQ;
        pMsg->hdr.status = 0;

        copyApiMacSecToMacSec(&(pMsg->associateReq.sec), &(pData->sec));

        pMsg->associateReq.logicalChannel = pData->logicalChannel;
        pMsg->associateReq.channelPage = pData->channelPage;
        pMsg->associateReq.phyID = pData->phyID;
        copyApiMacAddrToMacAddr(&(pMsg->associateReq.coordAddress),
                                &(pData->coordAddress));
        pMsg->associateReq.coordPanId = pData->coordPanId;
        pMsg->associateReq.capabilityInformation = ApiMac_convertCapabilityInfo(
                        &(pData->capabilityInformation));

        /* Send the message */
        if(ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                                (ICALL_MSG_FORMAT_KEEP),
                                pMsg)
           == ICALL_ERRNO_SUCCESS)
        {
            ret = ApiMac_status_success;
        }
    }

    return (ret);
}

/*!
 This function sends an associate response to a device
 requesting to associate.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeAssociateRsp(ApiMac_mlmeAssociateRsp_t *pData)
{
    ApiMac_status_t ret = ApiMac_status_noResources;
    /* Allocate message buffer space */
    macMlmeAssociateRspEvt_t *pMsg = (macMlmeAssociateRspEvt_t *)ICall_allocMsg(
                    sizeof(macMlmeAssociateRspEvt_t));

    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = MAC_MLME_ASSOCIATE_RSP;
        pMsg->hdr.status = 0;

        copyApiMacSecToMacSec(&(pMsg->associateRsp.sec), &(pData->sec));
        memcpy(&(pMsg->associateRsp.deviceAddress), &(pData->deviceAddress),
               sizeof(ApiMac_sAddrExt_t));
        pMsg->associateRsp.assocShortAddress = pData->assocShortAddress;
        pMsg->associateRsp.status = (uint8)pData->status;

        /* Send the message */
        if(ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                                (ICALL_MSG_FORMAT_KEEP),
                                pMsg)
           == ICALL_ERRNO_SUCCESS)
        {
            ret = ApiMac_status_success;
        }
    }

    return (ret);
}

/*!
 This function is used by an associated device to notify the
 coordinator of its intent to leave the PAN.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeDisassociateReq(ApiMac_mlmeDisassociateReq_t *pData)
{
    ApiMac_status_t ret = ApiMac_status_noResources;
    /* Allocate message buffer space */
    macMlmeDisassociateReqEvt_t *pMsg =
        (macMlmeDisassociateReqEvt_t *)ICall_allocMsg(
                        sizeof(macMlmeDisassociateReqEvt_t));

    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = MAC_MLME_DISASSOCIATE_REQ;
        pMsg->hdr.status = 0;

        copyApiMacSecToMacSec(&(pMsg->disAssociateReq.sec), &(pData->sec));
        copyApiMacAddrToMacAddr(&(pMsg->disAssociateReq.deviceAddress),
                                &(pData->deviceAddress));
        pMsg->disAssociateReq.devicePanId = pData->devicePanId;
        pMsg->disAssociateReq.disassociateReason = (uint8)pData
                        ->disassociateReason;
        pMsg->disAssociateReq.txIndirect = pData->txIndirect;

        /* Send the message */
        if(ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                                (ICALL_MSG_FORMAT_KEEP),
                                pMsg)
           == ICALL_ERRNO_SUCCESS)
        {
            ret = ApiMac_status_success;
        }
    }

    return (ret);
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetReqBool(ApiMac_attribute_bool_t pibAttribute,
bool *pValue)
{
    return (getTypeReq(MAC_GET_REQ, matchGetReq, (uint8_t)pibAttribute,
                       (void *)pValue,
                       NULL));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetReqUint8(ApiMac_attribute_uint8_t pibAttribute,
                                       uint8_t *pValue)
{
    return (getTypeReq(MAC_GET_REQ, matchGetReq, (uint8_t)pibAttribute,
                       (void *)pValue,
                       NULL));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetReqUint16(ApiMac_attribute_uint16_t pibAttribute,
                                        uint16_t *pValue)
{
    return (getTypeReq(MAC_GET_REQ, matchGetReq, (uint8_t)pibAttribute,
                       (void *)pValue,
                       NULL));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetReqUint32(ApiMac_attribute_uint32_t pibAttribute,
                                        uint32_t *pValue)
{
    return (getTypeReq(MAC_GET_REQ, matchGetReq, (uint8_t)pibAttribute,
                       (void *)pValue,
                       NULL));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetReqArray(ApiMac_attribute_array_t pibAttribute,
                                       uint8_t *pValue)
{
    return (getTypeReq(MAC_GET_REQ, matchGetReq, (uint8_t)pibAttribute,
                       (void *)pValue,
                       NULL));
}

/*!
 * @brief       This direct execute function retrieves an attribute value from
 *              the MAC PIB.
 *
 * @param       pibAttribute - The attribute identifier
 * @param       pValue - pointer to the attribute value
 * @param       pLen - pointer to the read length
 *
 * @return      The status of the request
 */
ApiMac_status_t ApiMac_mlmeGetReqArrayLen(ApiMac_attribute_array_t pibAttribute,
                                          uint8_t *pValue,
                                          uint16_t *pLen)
{
    return (getTypeReq(MAC_GET_REQ, matchGetReq, (uint8_t)pibAttribute,
                       (void *)pValue,
                       pLen));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC frequency Hopping PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetFhReqUint8(
                ApiMac_FHAttribute_uint8_t pibAttribute, uint8_t *pValue)
{
    return (mlmeGetFhReq((uint16_t)pibAttribute, (void *)pValue, NULL));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC frequency Hopping PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetFhReqUint16(
                ApiMac_FHAttribute_uint16_t pibAttribute, uint16_t *pValue)
{
    return (mlmeGetFhReq((uint16_t)pibAttribute, (void *)pValue, NULL));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC frequency Hopping PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetFhReqUint32(
                ApiMac_FHAttribute_uint32_t pibAttribute, uint32_t *pValue)
{
    return (mlmeGetFhReq((uint16_t)pibAttribute, (void *)pValue, NULL));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC frequency Hopping PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetFhReqArray(
                ApiMac_FHAttribute_array_t pibAttribute, uint8_t *pValue)
{
    return (mlmeGetFhReq((uint16_t)pibAttribute, (void *)pValue, NULL));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC frequency Hopping PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetFhReqArrayLen(
                ApiMac_FHAttribute_array_t pibAttribute,
                uint8_t *pValue,
                uint16_t *pLen)
{
    return (mlmeGetFhReq((uint16_t)pibAttribute, (void *)pValue, pLen));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC Secutity PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetSecurityReqUint8(
                ApiMac_securityAttribute_uint8_t pibAttribute, uint8_t *pValue)
{
    return (getTypeReq(MAC_GET_SECURITY_REQ, matchGetSecurityReq,
                       (uint8_t)pibAttribute,
                       (void*)pValue, NULL));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC Secutity PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetSecurityReqUint16(
                ApiMac_securityAttribute_uint16_t pibAttribute,
                uint16_t *pValue)
{
    return (getTypeReq(MAC_GET_SECURITY_REQ, matchGetSecurityReq,
                       (uint8_t)pibAttribute,
                       (void*)pValue, NULL));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC Secutity PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetSecurityReqArray(
                ApiMac_securityAttribute_array_t pibAttribute, uint8_t *pValue)
{
    return (getTypeReq(MAC_GET_SECURITY_REQ, matchGetSecurityReq,
                       (uint8_t)pibAttribute,
                       (void*)pValue, NULL));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC Secutity PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetSecurityReqArrayLen(
                ApiMac_securityAttribute_array_t pibAttribute,
                uint8_t *pValue,
                uint16_t *pLen
                )
{
    return (getTypeReq(MAC_GET_SECURITY_REQ, matchGetSecurityReq,
                       (uint8_t)pibAttribute,
                       (void*)pValue, pLen));
}

/*!
 This direct execute function retrieves an attribute value from
 the MAC Secutity PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeGetSecurityReqStruct(
                ApiMac_securityAttribute_struct_t pibAttribute, void *pValue)
{
    return (getTypeReq(MAC_GET_SECURITY_REQ, matchGetSecurityReq,
                       (uint8_t)pibAttribute,
                       (void*)pValue, NULL));
}

/*!
 This function is called in response to an orphan notification
 from a peer device.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeOrphanRsp(ApiMac_mlmeOrphanRsp_t *pData)
{
    ApiMac_status_t ret = ApiMac_status_noResources;
    /* Allocate message buffer space */
    macMlmeOrphanRspEvt_t *pMsg = (macMlmeOrphanRspEvt_t *)ICall_allocMsg(
                    sizeof(macMlmeOrphanRspEvt_t));

    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = MAC_MLME_ORPHAN_RSP;
        pMsg->hdr.status = 0;

        copyApiMacSecToMacSec(&(pMsg->orphanRsp.sec), &(pData->sec));
        memcpy(&(pMsg->orphanRsp.orphanAddress), &(pData->orphanAddress),
               sizeof(ApiMac_sAddrExt_t));
        pMsg->orphanRsp.shortAddress = pData->shortAddress;
        pMsg->orphanRsp.associatedMember = (uint8)pData->associatedMember;

        /* Send the message */
        if(ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                                (ICALL_MSG_FORMAT_KEEP),
                                pMsg)
           == ICALL_ERRNO_SUCCESS)
        {
            ret = ApiMac_status_success;
        }
    }

    return (ret);
}

/*!
 This function is used to request pending data from the coordinator.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmePollReq(ApiMac_mlmePollReq_t *pData)
{
    ApiMac_status_t ret = ApiMac_status_noResources;
    /* Allocate message buffer space */
    macMlmePollReqEvt_t *pMsg = (macMlmePollReqEvt_t *)ICall_allocMsg(
                    sizeof(macMlmePollReqEvt_t));

    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = MAC_MLME_POLL_REQ;
        pMsg->hdr.status = 0;

        copyApiMacAddrToMacAddr(&(pMsg->pollReq.coordAddress),
                                &(pData->coordAddress));
        pMsg->pollReq.coordPanId = pData->coordPanId;
        copyApiMacSecToMacSec(&(pMsg->pollReq.sec), &(pData->sec));

        /* Send the message */
        if(ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                                (ICALL_MSG_FORMAT_KEEP),
                                pMsg)
           == ICALL_ERRNO_SUCCESS)
        {
            ret = ApiMac_status_success;
        }
    }

    return (ret);
}

/*!
 This function must be called once at system startup before any other
 function in the management API is called.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeResetReq(bool setDefaultPib)
{
    ApiMac_status_t status = ApiMac_status_noResources;

    /* Allocate message buffer space */
    macResetReq_t *pMsg = (macResetReq_t *)ICall_allocMsg(
                    sizeof(macResetReq_t));

    if(pMsg != NULL)
    {
        ICall_Errno errno;

        /* Fill in the message content */
        pMsg->hdr.event = MAC_RESET_REQ;
        pMsg->hdr.status = 0;
        pMsg->setDefaultPib = setDefaultPib;

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     pMsg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            macResetReq_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, matchResetReq,
                                    (NULL),
                                    (NULL), (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (ApiMac_status_t)pCmdStatus->status;
            }
        }

        /* pCmdStatus is the same as msg */
        ICall_freeMsg(pMsg);
    }
    return (status);
}

/*!
 This function initiates an energy detect, active, passive, or
 orphan scan on one or more channels.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeScanReq(ApiMac_mlmeScanReq_t *pData)
{
    macMlmeScanReqEvt_t *pMsg;
    ApiMac_status_t status = ApiMac_status_noResources;

    /* Check for scan already in progress */
    if(scanResults != NULL)
    {
        return (ApiMac_status_scanInProgress);
    }

    /* Allocate message buffer space */
    pMsg = (macMlmeScanReqEvt_t *)ICall_allocMsg(
                                                 sizeof(macMlmeScanReqEvt_t));

    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = MAC_MLME_SCAN_REQ;
        pMsg->hdr.status = 0;

        memcpy(pMsg->scanReq.scanChannels, pData->scanChannels,
               (MAC_154G_CHANNEL_BITMAP_SIZ));

        pMsg->scanReq.scanType = (uint8)pData->scanType;
        pMsg->scanReq.scanDuration = (uint8)pData->scanDuration;
        pMsg->scanReq.channelPage = (uint8)pData->channelPage;
        pMsg->scanReq.phyID = (uint8)pData->phyID;
        pMsg->scanReq.maxResults = (uint8)pData->maxResults;
        pMsg->scanReq.permitJoining = (bool)pData->permitJoining;
        pMsg->scanReq.linkQuality = (uint8)pData->linkQuality;
        pMsg->scanReq.percentFilter = (uint8)pData->percentFilter;

        copyApiMacSecToMacSec(&(pMsg->scanReq.sec), &(pData->sec));

        pMsg->scanReq.MPMScan = pData->MPMScan;
        pMsg->scanReq.MPMScanType = (uint8)pData->MPMScanType;
        pMsg->scanReq.MPMScanDuration = (uint16)pData->MPMScanDuration;

        if(pData->scanType == ApiMac_scantype_energyDetect)
        {
            scanResults = (void *)ICall_malloc(APIMAC_154G_MAX_NUM_CHANNEL);
            pMsg->scanReq.result.pEnergyDetect = (uint8 *)scanResults;
        }
        else
        {
            scanResults = (void *)ICall_malloc(
                            sizeof(macPanDesc_t) * pMsg->scanReq.maxResults);
            pMsg->scanReq.result.pPanDescriptor = (macPanDesc_t *)scanResults;
        }

        if(scanResults != NULL)
        {
            /* Send the message */
            ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                                 (ICALL_MSG_FORMAT_KEEP),
                                 pMsg);
            status = ApiMac_status_success;
        }
    }

    return (status);
}

/*!
 This direct execute function sets an attribute value
 in the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSetReqBool(ApiMac_attribute_bool_t pibAttribute,
bool value)
{
    return (setTypeReq(MAC_SET_REQ, matchSetReq,
                       (uint8_t)pibAttribute,
                       &value));
}

/*!
 This direct execute function sets an attribute value
 in the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSetReqUint8(ApiMac_attribute_uint8_t pibAttribute,
                                       uint8_t value)
{
    return (setTypeReq(MAC_SET_REQ, matchSetReq,
                       (uint8_t)pibAttribute,
                       &value));
}

/*!
 This direct execute function sets an attribute value
 in the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSetReqUint16(ApiMac_attribute_uint16_t pibAttribute,
                                        uint16_t value)
{
    return (setTypeReq(MAC_SET_REQ, matchSetReq,
                       (uint8_t)pibAttribute,
                       &value));
}

/*!
 This direct execute function sets an attribute value
 in the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSetReqUint32(ApiMac_attribute_uint32_t pibAttribute,
                                        uint32_t value)
{
    return (setTypeReq(MAC_SET_REQ, matchSetReq,
                       (uint8_t)pibAttribute,
                       &value));
}

/*!
 This direct execute function sets an attribute value
 in the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSetReqArray(ApiMac_attribute_array_t pibAttribute,
                                       uint8_t *pValue)
{
    return (setTypeReq(MAC_SET_REQ, matchSetReq,
                       (uint8_t)pibAttribute,
                       pValue));
}

/*!
 This direct execute function sets a frequency hopping attribute value
 in the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSetFhReqUint8(
                ApiMac_FHAttribute_uint8_t pibAttribute, uint8_t value)
{
    return (mlmeSetFhReq((uint16_t)pibAttribute, (void *)&value));
}

/*!
 This direct execute function sets a frequency hopping attribute value
 in the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSetFhReqUint16(
                ApiMac_FHAttribute_uint16_t pibAttribute, uint16_t value)
{
    return (mlmeSetFhReq((uint16_t)pibAttribute, (void *)&value));
}

/*!
 This direct execute function sets a frequency hopping attribute value
 in the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSetFhReqUint32(
                ApiMac_FHAttribute_uint32_t pibAttribute, uint32_t value)
{
    return (mlmeSetFhReq((uint16_t)pibAttribute, (void *)&value));
}

/*!
 This direct execute function sets a frequency hopping attribute value
 in the MAC PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSetFhReqArray(
                ApiMac_FHAttribute_array_t pibAttribute, uint8_t *pValue)
{
    return (mlmeSetFhReq((uint16_t)pibAttribute, (void *)pValue));
}

/*!
 This direct execute function sets an attribute value
 in the MAC Security PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSetSecurityReqUint8(
                ApiMac_securityAttribute_uint8_t pibAttribute, uint8_t value)
{
    return (setTypeReq(MAC_SET_SECURITY_REQ, matchSetSecurityReq,
                       (uint8)pibAttribute,
                       (void *)&value));
}

/*!
 This direct execute function sets an attribute value
 in the MAC Security PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSetSecurityReqUint16(
                ApiMac_securityAttribute_uint16_t pibAttribute, uint16_t value)
{
    return (setTypeReq(MAC_SET_SECURITY_REQ, matchSetSecurityReq,
                       (uint8)pibAttribute,
                       (void *)&value));
}

/*!
 This direct execute function sets an attribute value
 in the MAC Security PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSetSecurityReqArray(
                ApiMac_securityAttribute_array_t pibAttribute, uint8_t *pValue)
{
    return (setTypeReq(MAC_SET_SECURITY_REQ, matchSetSecurityReq,
                       (uint8)pibAttribute,
                       (void *)pValue));
}

/*!
 This direct execute function sets an attribute value
 in the MAC Security PIB.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSetSecurityReqStruct(
                ApiMac_securityAttribute_struct_t pibAttribute, void *pValue)
{
    return (setTypeReq(MAC_SET_SECURITY_REQ, matchSetSecurityReq,
                       (uint8)pibAttribute,
                       (void *)pValue));
}

/*!
 This function is called by a coordinator or PAN coordinator
 to start or reconfigure a network.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeStartReq(ApiMac_mlmeStartReq_t *pData)
{
    ApiMac_status_t ret = ApiMac_status_noResources;
    /* Allocate message buffer space */
    macMlmeStartReqEvt_t *pMsg = (macMlmeStartReqEvt_t *)ICall_allocMsg(
                    (sizeof(macMlmeStartReqEvt_t) + pData->mpmParams.numIEs));

    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = MAC_START_REQ;
        pMsg->hdr.status = 0;

        pMsg->startReq.startTime = pData->startTime;
        pMsg->startReq.panId = pData->panId;
        pMsg->startReq.logicalChannel = pData->logicalChannel;
        pMsg->startReq.channelPage = pData->channelPage;
        pMsg->startReq.phyID = pData->phyID;
        pMsg->startReq.beaconOrder = pData->beaconOrder;
        pMsg->startReq.superframeOrder = pData->superframeOrder;
        pMsg->startReq.panCoordinator = pData->panCoordinator;
        pMsg->startReq.batteryLifeExt = pData->batteryLifeExt;
        pMsg->startReq.coordRealignment = pData->coordRealignment;

        copyApiMacSecToMacSec(&(pMsg->startReq.realignSec),
                              &(pData->realignSec));
        copyApiMacSecToMacSec(&(pMsg->startReq.beaconSec), &(pData->beaconSec));

        pMsg->startReq.mpmParams.eBeaconOrder = pData->mpmParams.eBeaconOrder;
        pMsg->startReq.mpmParams.offsetTimeSlot = pData->mpmParams
                        .offsetTimeSlot;
        pMsg->startReq.mpmParams.NBPANEBeaconOrder = pData->mpmParams
                        .NBPANEBeaconOrder;
        pMsg->startReq.startFH = pData->startFH;
        pMsg->startReq.mpmParams.numIEs = pData->mpmParams.numIEs;
        if(pData->mpmParams.numIEs)
        {
            /* the buffer was allocated at end of the buffer */
            pMsg->startReq.mpmParams.pIEIDs = (uint8_t *)(pMsg + 1);
            memcpy(pMsg->startReq.mpmParams.pIEIDs, pData->mpmParams.pIEIDs,
                   pData->mpmParams.numIEs);
        }
        else
        {
            pMsg->startReq.mpmParams.pIEIDs = NULL;
        }

        /* Send the message */
        if(ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                                (ICALL_MSG_FORMAT_KEEP),
                                pMsg)
           == ICALL_ERRNO_SUCCESS)
        {
            ret = ApiMac_status_success;
        }
    }

    return (ret);
}

/*!
 This function requests the MAC to synchronize with the
 coordinator by acquiring and optionally tracking its beacons.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeSyncReq(ApiMac_mlmeSyncReq_t *pData)
{
    ApiMac_status_t ret = ApiMac_status_noResources;
    /* Allocate message buffer space */
    macMlmeSyncReqEvt_t *pMsg = (macMlmeSyncReqEvt_t *)ICall_allocMsg(
                    sizeof(macMlmeSyncReqEvt_t));

    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = MAC_SYNC_REQ;
        pMsg->hdr.status = 0;

        pMsg->syncReq.logicalChannel = pData->logicalChannel;
        pMsg->syncReq.channelPage = pData->channelPage;
        pMsg->syncReq.phyID = pData->phyID;
        pMsg->syncReq.trackBeacon = pData->trackBeacon;

        /* Send the message */
        if(ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                                (ICALL_MSG_FORMAT_KEEP),
                                pMsg)
           == ICALL_ERRNO_SUCCESS)
        {
            ret = ApiMac_status_success;
        }
    }

    return (ret);
}

/*!
 This function returns a random byte from the MAC random number
 generator.

 Public function defined in api_mac.h
 */
uint8_t ApiMac_randomByte(void)
{
    uint8_t randByte = 0;

    /* Allocate message buffer space */
    randomByte_t *pMsg = (randomByte_t *)ICall_allocMsg(sizeof(randomByte_t));

    if(pMsg != NULL)
    {
        ICall_Errno errno;

        /* Fill in the message content */
        pMsg->hdr.event = MAC_RANDOM_BYTE;
        pMsg->hdr.status = 0;

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     pMsg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            randomByte_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, matchRandomByteReq,
                                    (NULL),
                                    (NULL), (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                randByte = pCmdStatus->randByte;
            }
        }

        /* pCmdStatus is the same as msg */
        ICall_freeMsg(pMsg);
    }
    return (randByte);
}

/*!
 Update Device Table entry and PIB with new Pan Id.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_updatePanId(uint16_t panId)
{
    ApiMac_status_t ret = ApiMac_status_noResources;
    /* Allocate message buffer space */
    macUpdatePanId_t *pMsg = (macUpdatePanId_t *)ICall_allocMsg(
                    sizeof(macUpdatePanId_t));

    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = MAC_UPDATE_PANID;
        pMsg->hdr.status = 0;
        pMsg->panId = panId;

        /* Send the message */
        if(ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                                (ICALL_MSG_FORMAT_KEEP),
                                pMsg)
           == ICALL_ERRNO_SUCCESS)
        {
            ret = ApiMac_status_success;
        }
    }

    return (ret);
}

/*!
 This functions handles the WiSUN async request.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_mlmeWSAsyncReq(ApiMac_mlmeWSAsyncReq_t* pData)
{
    ApiMac_status_t ret = ApiMac_status_noResources;
    /* Allocate message buffer space */
    macMlmeWSAsyncReqEvt_t *pMsg = (macMlmeWSAsyncReqEvt_t *)ICall_allocMsg(
                    sizeof(macMlmeWSAsyncReqEvt_t));

    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = MAC_MLME_WS_ASYNC_REQ;
        pMsg->hdr.status = 0;

        copyApiMacSecToMacSec(&(pMsg->asyncReq.sec), &(pData->sec));
        pMsg->asyncReq.operation = pData->operation;
        pMsg->asyncReq.frameType = pData->frameType;
        memcpy(pMsg->asyncReq.channels, pData->channels,
        APIMAC_154G_CHANNEL_BITMAP_SIZ);

        /* Send the message */
        if(ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                                (ICALL_MSG_FORMAT_KEEP),
                                pMsg)
           == ICALL_ERRNO_SUCCESS)
        {
            ret = ApiMac_status_success;
        }
    }

    return (ret);
}

/*!
 This function start the Frequency hopping operation.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_startFH(void)
{
    return (sendEvt(MAC_START_FH));
}

/*!
 Enables the Frequency hopping operation.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_enableFH(void)
{
    return (sendEvtExpectStatus(MAC_ENABLE_FH, matchEnableFH));
}

/*!
 Parses the payload information elements.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_parsePayloadGroupIEs(uint8_t *pPayload,
                                            uint16_t payloadLen,
                                            ApiMac_payloadIeRec_t **pList)
{
    return (parsePayloadIEs(pPayload, payloadLen, pList, true));
}

/*!
 Parses the payload Sub Information Elements.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_parsePayloadSubIEs(uint8_t *pContent,
                                          uint16_t contentLen,
                                          ApiMac_payloadIeRec_t **pList)
{
    return (parsePayloadIEs(pContent, contentLen, pList, false));
}

/*!
 Free memory allocated by ApiMac.

 Public function defined in api_mac.h
 */
void ApiMac_freeIEList(ApiMac_payloadIeRec_t *pList)
{
    /* Loop through the list */
    while(pList)
    {
        ApiMac_payloadIeRec_t *pTmp = pList;

        /* Move to the next item in the list */
        pList = pTmp->pNext;

        /* free the current item */
        ICall_free(pTmp);
    }
}

/*!
 Convert ApiMac_capabilityInfo_t data type to uint8_t capInfo

 Public function defined in api_mac.h
 */
uint8_t ApiMac_convertCapabilityInfo(ApiMac_capabilityInfo_t *pMsgcapInfo)
{
    uint8 capInfo = 0;

    if(pMsgcapInfo->panCoord)
    {
        capInfo |= CAPABLE_PAN_COORD;
    }

    if(pMsgcapInfo->ffd)
    {
        capInfo |= CAPABLE_FFD;
    }

    if(pMsgcapInfo->mainsPower)
    {
        capInfo |= CAPABLE_MAINS_POWER;
    }

    if(pMsgcapInfo->rxOnWhenIdle)
    {
        capInfo |= CAPABLE_RX_ON_IDLE;
    }

    if(pMsgcapInfo->security)
    {
        capInfo |= CAPABLE_SECURITY;
    }

    if(pMsgcapInfo->allocAddr)
    {
        capInfo |= CAPABLE_ALLOC_ADDR;
    }

    return (capInfo);
}

/*!
 Convert from bitmask byte to API MAC capInfo

 Public function defined in api_mac.h
 */
void ApiMac_buildMsgCapInfo(uint8_t cInfo, ApiMac_capabilityInfo_t *pPBcapInfo)
{
    if(cInfo & CAPABLE_PAN_COORD)
    {
        pPBcapInfo->panCoord = 1;
    }

    if(cInfo & CAPABLE_FFD)
    {
        pPBcapInfo->ffd = 1;
    }

    if(cInfo & CAPABLE_MAINS_POWER)
    {
        pPBcapInfo->mainsPower = 1;
    }

    if(cInfo & CAPABLE_RX_ON_IDLE)
    {
        pPBcapInfo->rxOnWhenIdle = 1;
    }

    if(cInfo & CAPABLE_SECURITY)
    {
        pPBcapInfo->security = 1;
    }

    if(cInfo & CAPABLE_ALLOC_ADDR)
    {
        pPBcapInfo->allocAddr = 1;
    }
}

/*!
 Adds a new MAC device table entry.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_secAddDevice(ApiMac_secAddDevice_t *pAddDevice)
{
    macSecAddDevice_t *pMsg;
    ApiMac_status_t status = ApiMac_status_noResources;

    /* Allocate message buffer space */
    pMsg = (macSecAddDevice_t *)ICall_allocMsg(sizeof(macSecAddDevice_t));

    if(pMsg != NULL)
    {
        ICall_Errno errno;

        /* Fill in the message content */
        pMsg->hdr.event = MAC_SEC_ADD_DEVICE;
        pMsg->hdr.status = 0;

        pMsg->panId = pAddDevice->panID;
        pMsg->shortAddr = pAddDevice->shortAddr;
        memcpy(&pMsg->extAddr, &pAddDevice->extAddr, APIMAC_SADDR_EXT_LEN);
        pMsg->exempt = pAddDevice->exempt;

        pMsg->keyIdLookupDataSize = pAddDevice->keyIdLookupDataSize;
        memcpy(pMsg->keyIdLookupData, pAddDevice->keyIdLookupData,
               (APIMAC_MAX_KEY_LOOKUP_LEN));
        pMsg->frameCounter = pAddDevice->frameCounter;
        pMsg->uniqueDevice = pAddDevice->uniqueDevice;
        pMsg->duplicateDevFlag = pAddDevice->duplicateDevFlag;

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     pMsg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            macSecAddDevice_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, matchSecAddDevice,
                                    (NULL), (NULL), (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (ApiMac_status_t)pCmdStatus->hdr.status;
            }
        }

        /* pCmdStatus is the same as msg */
        ICall_freeMsg(pMsg);
    }

    return (status);
}

/*!
 Removes MAC device table entries.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_secDeleteDevice(ApiMac_sAddrExt_t *pExtAddr)
{
    macSecDelDevice_t *pMsg;
    ApiMac_status_t status = ApiMac_status_noResources;

    /* Allocate message buffer space */
    pMsg = (macSecDelDevice_t *)ICall_allocMsg(sizeof(macSecDelDevice_t));

    if(pMsg != NULL)
    {
        ICall_Errno errno;

        /* Fill in the message content */
        pMsg->hdr.event = MAC_SEC_DEL_DEVICE;
        pMsg->hdr.status = 0;

        memcpy(&pMsg->extAddr, pExtAddr, APIMAC_SADDR_EXT_LEN);

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     pMsg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            macSecDelDevice_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, matchSecDelDevice,
                                    (NULL), (NULL), (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (ApiMac_status_t)pCmdStatus->hdr.status;
            }
        }

        /* pCmdStatus is the same as msg */
        ICall_freeMsg(pMsg);
    }

    return (status);
}

/*!
 Removes the key at the specified key Index and removes all MAC device table
 enteries associated with this key.

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_secDeleteKeyAndAssocDevices(uint8_t keyIndex)
{
    macSecDelKeyAndDevices_t *pMsg;
    ApiMac_status_t status = ApiMac_status_noResources;

    /* Allocate message buffer space */
    pMsg = (macSecDelKeyAndDevices_t *)ICall_allocMsg(
                    sizeof(macSecDelKeyAndDevices_t));

    if(pMsg != NULL)
    {
        ICall_Errno errno;

        /* Fill in the message content */
        pMsg->hdr.event = MAC_SEC_DEL_KEY_AND_DEVICES;
        pMsg->hdr.status = 0;

        pMsg->keyIndex = keyIndex;

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     pMsg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            macSecDelKeyAndDevices_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER,
                                    matchSecDelKeyAndDevices,
                                    (NULL), (NULL), (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (ApiMac_status_t)pCmdStatus->hdr.status;
            }
        }

        /* pCmdStatus is the same as msg */
        ICall_freeMsg(pMsg);
    }

    return (status);
}

/*!
 Removes all MAC device table entries

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_secDeleteAllDevices(void)
{
    macEventHdr_t *pMsg;
    ApiMac_status_t status = ApiMac_status_noResources;

    /* Allocate message buffer space */
    pMsg = (macEventHdr_t *)ICall_allocMsg(sizeof(macEventHdr_t));

    if(pMsg != NULL)
    {
        ICall_Errno errno;

        /* Fill in the message content */
        pMsg->event = MAC_SEC_DEL_ALL_DEVICES;
        pMsg->status = 0;

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     pMsg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            macEventHdr_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER,
                                    matchSecDelAllDevices,
                                    (NULL), (NULL), (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (ApiMac_status_t)pCmdStatus->status;
            }
        }

        /* pCmdStatus is the same as msg */
        ICall_freeMsg(pMsg);
    }

    return (status);
}

/*!
 Reads the frame counter value associated with a MAC security key indexed
 by the designated key identifier and the default key source

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_secGetDefaultSourceKey(uint8_t keyId,
                                              uint32_t *pFrameCounter)
{
    macSecGetDefaultSrcKey_t *pMsg;
    ApiMac_status_t status = ApiMac_status_noResources;

    /* Allocate message buffer space */
    pMsg = (macSecGetDefaultSrcKey_t *)ICall_allocMsg(
                    sizeof(macSecGetDefaultSrcKey_t));

    if(pMsg != NULL)
    {
        ICall_Errno errno;

        /* Fill in the message content */
        pMsg->hdr.event = MAC_SEC_GET_DEFAULT_SOURCE_KEY;
        pMsg->hdr.status = 0;

        pMsg->keyId = keyId;

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     pMsg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            macSecGetDefaultSrcKey_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER,
                                    matchSecGetDefaultSrcKey,
                                    (NULL), (NULL), (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (ApiMac_status_t)pCmdStatus->hdr.status;
                *pFrameCounter = pCmdStatus->frameCounter;
            }
        }

        /* pCmdStatus is the same as msg */
        ICall_freeMsg(pMsg);
    }

    return (status);
}

/*!
 Adds the MAC security key, adds the associated lookup list for the key,
 initializes the frame counter to the value provided

 Public function defined in api_mac.h
 */
ApiMac_status_t ApiMac_secAddKeyInitFrameCounter(
                ApiMac_secAddKeyInitFrameCounter_t *pInfo)
{
    macSecAddKeyInitFC_t *pMsg;
    ApiMac_status_t status = ApiMac_status_noResources;

    /* Allocate message buffer space */
    pMsg = (macSecAddKeyInitFC_t *)ICall_allocMsg(sizeof(macSecAddKeyInitFC_t));

    if(pMsg != NULL)
    {
        ICall_Errno errno;

        /* Fill in the message content */
        pMsg->hdr.event = MAC_SEC_ADD_KEY_INIT_FC;
        pMsg->hdr.status = 0;

        memcpy(pMsg->key, pInfo->key, APIMAC_KEY_MAX_LEN);
        pMsg->frameCounter = pInfo->frameCounter;
        pMsg->replaceKeyIndex = pInfo->replaceKeyIndex;
        pMsg->newKeyFlag = pInfo->newKeyFlag;
        pMsg->lookupDataSize = pInfo->lookupDataSize;
        memcpy(pMsg->lookupData, pInfo->lookupData, APIMAC_MAX_KEY_LOOKUP_LEN);

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     pMsg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            macSecAddKeyInitFC_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER,
                                    matchSecAddKeyInitFC,
                                    (NULL), (NULL), (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (ApiMac_status_t)pCmdStatus->hdr.status;
            }
        }

        /* pCmdStatus is the same as msg */
        ICall_freeMsg(pMsg);
    }

    return (status);
}

/******************************************************************************
 Local Functions
 *****************************************************************************/

/*!
 * @brief       Generic function to get an FH attribute
 *
 * @param       pibAttribute - attribute to get
 * @param       pValue - pointer to put the attribute value
 * @param       pLen - pointer to place to put length
 *
 * @return      status result
 */
static ApiMac_status_t mlmeGetFhReq(uint16_t pibAttribute, void *pValue,
                                    uint16_t *pLen)
{
    ApiMac_status_t status = ApiMac_status_noResources;
    /* Allocate message buffer space */
    macFHGetParam_t *pMsg = (macFHGetParam_t *)ICall_allocMsg(
                    sizeof(macFHGetParam_t));

    if(pMsg != NULL)
    {
        ICall_Errno errno;

        memset(pMsg, 0, sizeof(macFHGetParam_t));

        /* Fill in the message content */
        pMsg->hdr.event = MAC_FH_GET_REQ;
        pMsg->hdr.status = 0;
        pMsg->paramID = pibAttribute;

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     pMsg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            macFHGetParam_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, matchGetFhReq,
                                    (NULL),
                                    (NULL), (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (ApiMac_status_t)pCmdStatus->hdr.status;
                if((status == MAC_SUCCESS) && (pCmdStatus->len > 0))
                {
                    memcpy(pValue, pCmdStatus->pValue, pCmdStatus->len);
                    if(pLen)
                    {
                        *pLen = pCmdStatus->len;
                    }
                }
            }
        }

        /* pCmdStatus is the same as msg */
        if(pMsg->pValue)
        {
            ICall_free(pMsg->pValue);
        }
        ICall_freeMsg(pMsg);
    }
    return (status);
}

/*!
 * @brief       This function process incoming ICall callback messages.
 *
 * @param       pMsg - pointer to the incoming message
 */
static uint16_t processIncomingICallMsg(macCbackEvent_t *pMsg)
{
    /* Only process message if callbacks are setup */
    if(pMacCallbacks != NULL)
    {
        /* Determine the callback type */
        switch(pMsg->hdr.event)
        {
            case MAC_MLME_ASSOCIATE_IND:
                if(pMacCallbacks->pAssocIndCb)
                {
                    /* Indication structure */
                    ApiMac_mlmeAssociateInd_t ind;

                    /* Initialize the structure */
                    memset(&ind, 0, sizeof(ApiMac_mlmeAssociateInd_t));

                    /* copy the message to the indication structure */
                    memcpy(ind.deviceAddress, pMsg->associateInd.deviceAddress,
                           sizeof(ApiMac_sAddrExt_t));
                    ApiMac_buildMsgCapInfo(
                                    pMsg->associateInd.capabilityInformation,
                                    &(ind.capabilityInformation));
                    copyMacSecToApiMacSec(&(ind.sec),
                                          &(pMsg->associateInd.sec));

                    /* Initiate the callback */
                    pMacCallbacks->pAssocIndCb(&ind);
                }
                break;

            case MAC_MLME_ASSOCIATE_CNF:
                if(pMacCallbacks->pAssocCnfCb)
                {
                    /* Confirmation structure */
                    ApiMac_mlmeAssociateCnf_t cnf;

                    /* Initialize the structure */
                    memset(&cnf, 0, sizeof(ApiMac_mlmeAssociateCnf_t));

                    /* copy the message to the confirmation structure */
                    cnf.status =
                        (ApiMac_assocStatus_t)pMsg->associateCnf.hdr
                                                                    .status;
                    cnf.assocShortAddress = (uint16_t)pMsg->associateCnf
                                    .assocShortAddress;
                    copyMacSecToApiMacSec(&(cnf.sec),
                                          &(pMsg->associateCnf.sec));

                    /* Initiate the callback */
                    pMacCallbacks->pAssocCnfCb(&cnf);
                }
                break;

            case MAC_MLME_DISASSOCIATE_IND:
                if(pMacCallbacks->pDisassociateIndCb)
                {
                    /* Indication structure */
                    ApiMac_mlmeDisassociateInd_t ind;

                    /* Initialize the structure */
                    memset(&ind, 0, sizeof(ApiMac_mlmeDisassociateInd_t));

                    /* copy the message to the indication structure */
                    memcpy(ind.deviceAddress,
                           pMsg->disassociateInd.deviceAddress,
                           sizeof(ApiMac_sAddrExt_t));
                    ind.disassociateReason = (ApiMac_disassocateReason_t)pMsg
                                    ->disassociateInd.disassociateReason;
                    copyMacSecToApiMacSec(&(ind.sec),
                                          &(pMsg->disassociateInd.sec));

                    /* Initiate the callback */
                    pMacCallbacks->pDisassociateIndCb(&ind);
                }
                break;

            case MAC_MLME_DISASSOCIATE_CNF:
                if(pMacCallbacks->pDisassociateCnfCb)
                {
                    /* Confirmation structure */
                    ApiMac_mlmeDisassociateCnf_t cnf;

                    /* Initialize the structure */
                    memset(&cnf, 0, sizeof(ApiMac_mlmeDisassociateCnf_t));

                    /* copy the message to the confirmation structure */
                    cnf.status = (ApiMac_status_t)pMsg->disassociateCnf.hdr
                                    .status;
                    copyMacAddrToApiMacAddr(
                                    &(cnf.deviceAddress),
                                    &(pMsg->disassociateCnf.deviceAddress));
                    cnf.panId = (uint16_t)pMsg->disassociateCnf.panId;

                    /* Initiate the callback */
                    pMacCallbacks->pDisassociateCnfCb(&cnf);
                }
                break;

            case MAC_MLME_BEACON_NOTIFY_IND:
                if(pMacCallbacks->pBeaconNotifyIndCb)
                {
                    processBeaconNotifyInd(&(pMsg->beaconNotifyInd));
                }
                break;

            case MAC_MLME_ORPHAN_IND:
                if(pMacCallbacks->pOrphanIndCb)
                {
                    /* Indication structure */
                    ApiMac_mlmeOrphanInd_t ind;

                    /* Initialize the structure */
                    memset(&ind, 0, sizeof(ApiMac_mlmeOrphanInd_t));

                    /* copy the message to the indication structure */
                    memcpy(ind.orphanAddress, pMsg->orphanInd.orphanAddress,
                           sizeof(ApiMac_sAddrExt_t));
                    copyMacSecToApiMacSec(&(ind.sec), &(pMsg->orphanInd.sec));

                    /* Initiate the callback */
                    pMacCallbacks->pOrphanIndCb(&ind);
                }
                break;

            case MAC_MLME_SCAN_CNF:
                if(pMacCallbacks->pScanCnfCb)
                {
                    processScanCnf(&(pMsg->scanCnf));
                }
                else
                {
                    /*
                     If there's no callback, make sure the scanResuls
                     are freed
                     */
                    if(scanResults != NULL)
                    {
                        ICall_free(scanResults);
                        scanResults = NULL;
                    }

                }
                break;

            case MAC_MLME_START_CNF:
                if(pMacCallbacks->pStartCnfCb)
                {
                    /* Confirmation structure */
                    ApiMac_mlmeStartCnf_t cnf;

                    /* Initialize the structure */
                    memset(&cnf, 0, sizeof(ApiMac_mlmeStartCnf_t));

                    /* copy the message to the confirmation structure */
                    cnf.status = (ApiMac_status_t)pMsg->startCnf.hdr.status;

                    /* Initiate the callback */
                    pMacCallbacks->pStartCnfCb(&cnf);
                }
                break;

            case MAC_MLME_SYNC_LOSS_IND:
                if(pMacCallbacks->pSyncLossIndCb)
                {
                    /* Indication structure */
                    ApiMac_mlmeSyncLossInd_t ind;

                    /* Initialize the structure */
                    memset(&ind, 0, sizeof(ApiMac_mlmeSyncLossInd_t));

                    /* copy the message to the indication structure */
                    ind.reason = (ApiMac_status_t)pMsg->syncLossInd.hdr.status;
                    ind.panId = pMsg->syncLossInd.panId;
                    ind.logicalChannel = pMsg->syncLossInd.logicalChannel;
                    ind.channelPage = pMsg->syncLossInd.channelPage;
                    ind.phyID = pMsg->syncLossInd.phyID;
                    copyMacSecToApiMacSec(&(ind.sec), &(pMsg->syncLossInd.sec));

                    /* Initiate the callback */
                    pMacCallbacks->pSyncLossIndCb(&ind);
                }
                break;

            case MAC_MLME_POLL_CNF:
                if(pMacCallbacks->pPollCnfCb)
                {
                    /* Confirmation structure */
                    ApiMac_mlmePollCnf_t cnf;

                    /* Initialize the structure */
                    memset(&cnf, 0, sizeof(ApiMac_mlmePollCnf_t));

                    /* copy the message to the confirmation structure */
                    cnf.status = (ApiMac_status_t)pMsg->pollCnf.hdr.status;
                    cnf.framePending = pMsg->pollCnf.framePending;

                    /* Initiate the callback */
                    pMacCallbacks->pPollCnfCb(&cnf);
                }
                break;

            case MAC_MLME_POLL_IND:
                if(pMacCallbacks->pPollIndCb)
                {
                    /* Indication structure */
                    ApiMac_mlmePollInd_t ind;

                    /* Initialize the structure */
                    memset(&ind, 0, sizeof(ApiMac_mlmePollInd_t));

                    /* copy the message to the indication structure */
                    copyMacAddrToApiMacAddr(&(ind.srcAddr),
                                            &(pMsg->pollInd.srcAddr));
                    ind.srcPanId = pMsg->pollInd.srcPanId;
                    ind.noRsp = pMsg->pollInd.noRsp;

                    /* Initiate the callback */
                    pMacCallbacks->pPollIndCb(&ind);
                }
                break;

            case MAC_MLME_COMM_STATUS_IND:
                if(pMacCallbacks->pCommStatusCb)
                {
                    /* Indication structure */
                    ApiMac_mlmeCommStatusInd_t ind;

                    /* Initialize the structure */
                    memset(&ind, 0, sizeof(ApiMac_mlmeCommStatusInd_t));

                    /* copy the message to the indication structure */
                    ind.status = (ApiMac_status_t)pMsg->hdr.status;
                    copyMacAddrToApiMacAddr(&(ind.srcAddr),
                                            &(pMsg->commStatusInd.srcAddr));
                    copyMacAddrToApiMacAddr(&(ind.dstAddr),
                                            &(pMsg->commStatusInd.dstAddr));
                    ind.panId = (uint16_t)pMsg->commStatusInd.panId;
                    ind.reason = (ApiMac_commStatusReason_t)pMsg->commStatusInd
                                    .reason;
                    copyMacSecToApiMacSec(&(ind.sec),
                                          &(pMsg->commStatusInd.sec));

                    /* Initiate the callback */
                    pMacCallbacks->pCommStatusCb(&ind);
                }
                break;

            case MAC_MCPS_DATA_CNF:
                if(pMacCallbacks->pDataCnfCb)
                {
                    /* Confirmation structure */
                    ApiMac_mcpsDataCnf_t cnf;

                    /* Initialize the structure */
                    memset(&cnf, 0, sizeof(ApiMac_mcpsDataCnf_t));

                    /* copy the message to the confirmation structure */
                    cnf.status = (ApiMac_status_t)pMsg->dataCnf.hdr.status;
                    cnf.msduHandle = pMsg->dataCnf.msduHandle;
                    cnf.timestamp = pMsg->dataCnf.timestamp;
                    cnf.timestamp2 = pMsg->dataCnf.timestamp2;
                    cnf.retries = pMsg->dataCnf.retries;
                    cnf.mpduLinkQuality = pMsg->dataCnf.mpduLinkQuality;
                    cnf.correlation = pMsg->dataCnf.correlation;
                    cnf.rssi = pMsg->dataCnf.rssi;
                    cnf.frameCntr = pMsg->dataCnf.frameCntr;

                    /* Initiate the callback */
                    pMacCallbacks->pDataCnfCb(&cnf);
                }

                if(pMsg->dataCnf.pDataReq)
                {
                    /* Deallocate the original data request structure */
                    macMsgDeallocate((macEventHdr_t *)pMsg->dataCnf.pDataReq);
                }
                break;

            case MAC_MCPS_DATA_IND:
                if(pMacCallbacks->pDataIndCb)
                {
                    /* Indication structure */
                    ApiMac_mcpsDataInd_t ind;

                    /* copy structure to structure */
                    copyDataInd(&ind, &(pMsg->dataInd));

                    /* Initiate the callback */
                    pMacCallbacks->pDataIndCb(&ind);
                }
                break;

            case MAC_MCPS_PURGE_CNF:
                if(pMacCallbacks->pPurgeCnfCb)
                {
                    /* Confirmation structure */
                    ApiMac_mcpsPurgeCnf_t cnf;

                    /* Initialize the structure */
                    memset(&cnf, 0, sizeof(ApiMac_mcpsPurgeCnf_t));

                    /* copy the message to the confirmation structure */
                    cnf.status = (ApiMac_status_t)pMsg->purgeCnf.hdr.status;
                    cnf.msduHandle = pMsg->purgeCnf.msduHandle;

                    /* Initiate the callback */
                    pMacCallbacks->pPurgeCnfCb(&cnf);
                }
                break;

            case MAC_MLME_WS_ASYNC_IND:
                if(pMacCallbacks->pWsAsyncIndCb)
                {
                    /* Indication structure */
                    ApiMac_mlmeWsAsyncInd_t ind;

                    /* copy structure to structure */
                    copyDataInd((ApiMac_mcpsDataInd_t *)&ind,
                                (macMcpsDataInd_t *)&(pMsg->asyncInd));

                    /* Initiate the callback */
                    pMacCallbacks->pWsAsyncIndCb(&ind);
                }
                break;

            case MAC_MLME_WS_ASYNC_CNF:
                if(pMacCallbacks->pWsAsyncCnfCb)
                {
                    /* Confirmation structure */
                    ApiMac_mlmeWsAsyncCnf_t cnf;

                    /* Initialize the structure */
                    memset(&cnf, 0, sizeof(ApiMac_mlmeWsAsyncCnf_t));

                    /* copy the message to the confirmation structure */
                    cnf.status = (ApiMac_status_t)pMsg->asyncCnf.hdr.status;

                    /* Initiate the callback */
                    pMacCallbacks->pWsAsyncCnfCb(&cnf);
                }
                break;

            default:
                break;
        }
    }
    return (0);
}

/*!
 * @brief       Copy the common security type from Mac Stack type to App type.
 *
 * @param       pDst - pointer to the application type
 * @param       pSrc - pointer to the mac stack type
 */
static void copyMacSecToApiMacSec(ApiMac_sec_t *pDst, macSec_t *pSrc)
{
    /* Copy each element of the structure */
    memcpy(pDst->keySource, pSrc->keySource, APIMAC_KEY_SOURCE_MAX_LEN);
    pDst->securityLevel = pSrc->securityLevel;
    pDst->keyIdMode = pSrc->keyIdMode;
    pDst->keyIndex = pSrc->keyIndex;
}

/*!
 * @brief       Copy the common address type from Mac Stack type to App type.
 *
 * @param       pDst - pointer to the application type
 * @param       pSrc - pointer to the mac stack type
 */
static void copyMacAddrToApiMacAddr(ApiMac_sAddr_t *pDst, sAddr_t *pSrc)
{
    /* Copy each element of the structure */
    pDst->addrMode = (ApiMac_addrType_t)pSrc->addrMode;
    if(pDst->addrMode == ApiMac_addrType_short)
    {
        pDst->addr.shortAddr = pSrc->addr.shortAddr;
    }
    else
    {
        memcpy(pDst->addr.extAddr, pSrc->addr.extAddr,
               sizeof(ApiMac_sAddrExt_t));
    }
}

/*!
 * @brief       Copy the common address type from Mac Stack type to App type.
 *
 * @param       pDst - pointer to the application type
 * @param       pSrc - pointer to the mac stack type
 */
static void copyMacPanDescToApiMacPanDesc(ApiMac_panDesc_t *pDst,
                                          macPanDesc_t *pSrc)
{
    /* Copy each element of the structure */
    copyMacAddrToApiMacAddr(&(pDst->coordAddress), &(pSrc->coordAddress));
    pDst->coordPanId = (uint16_t)pSrc->coordPanId;
    pDst->superframeSpec = (uint16_t)pSrc->superframeSpec;
    pDst->logicalChannel = (uint8_t)pSrc->logicalChannel;
    pDst->channelPage = (uint8_t)pSrc->channelPage;
    pDst->gtsPermit = (bool)pSrc->gtsPermit;
    pDst->linkQuality = (uint8_t)pSrc->linkQuality;
    pDst->timestamp = (uint32_t)pSrc->timestamp;
    pDst->securityFailure = (bool)pSrc->securityFailure;
    copyMacSecToApiMacSec(&(pDst->sec), &(pSrc->sec));
}

/*!
 * @brief       Process the incoming Beacon Notification callback.
 *
 * @param       pInd - pointer MAC Beacon indication info
 */
static void processBeaconNotifyInd(macMlmeBeaconNotifyInd_t *pInd)
{
    /* Indication structure */
    ApiMac_mlmeBeaconNotifyInd_t ind;

    /* Initialize the structure */
    memset(&ind, 0, sizeof(ApiMac_mlmeBeaconNotifyInd_t));

    /* copy the message to the indication structure */
    ind.beaconType = (ApiMac_beaconType_t)pInd->beaconType;
    ind.bsn = pInd->bsn;

    if(ind.beaconType == ApiMac_beaconType_normal)
    {
        uint8_t *pAddrList;

        /* Fill in the PAN descriptor */
        if(pInd->info.beaconData.pPanDesc)
        {
            copyMacPanDescToApiMacPanDesc(&ind.panDesc,
                                          pInd->info.beaconData.pPanDesc);
        }

        /* Add the pending address lists for short address and extended address */
        pAddrList = pInd->info.beaconData.pAddrList;
        ind.beaconData.beacon.numPendShortAddr = MAC_PEND_NUM_SHORT(
                        pInd->info.beaconData.pendAddrSpec);
        ind.beaconData.beacon.numPendExtAddr = MAC_PEND_NUM_EXT(
                        pInd->info.beaconData.pendAddrSpec);
        if(ind.beaconData.beacon.numPendShortAddr)
        {
            ind.beaconData.beacon.pShortAddrList = (uint16_t *)pAddrList;
            pAddrList += ind.beaconData.beacon.numPendShortAddr * 2;
        }
        if(ind.beaconData.beacon.numPendExtAddr)
        {
            ind.beaconData.beacon.pExtAddrList = (uint8_t *)pAddrList;
        }

        /* Add the beacon payload */
        ind.beaconData.beacon.sduLength = pInd->info.beaconData.sduLength;
        ind.beaconData.beacon.pSdu = pInd->info.beaconData.pSdu;
    }
    else
    {
        /* Fill in the PAN descriptor */
        if(pInd->info.eBeaconData.pPanDesc)
        {
            copyMacPanDescToApiMacPanDesc(&ind.panDesc,
                                          pInd->info.eBeaconData.pPanDesc);
        }

        /* Must be an enhanced beacon */
        ind.beaconData.eBeacon.coexist.beaconOrder = pInd->info.eBeaconData
                        .coexist.beaconOrder;
        ind.beaconData.eBeacon.coexist.superFrameOrder = pInd->info.eBeaconData
                        .coexist.superFrameOrder;
        ind.beaconData.eBeacon.coexist.finalCapSlot = pInd->info.eBeaconData
                        .coexist.finalCapSlot;
        ind.beaconData.eBeacon.coexist.eBeaconOrder = pInd->info.eBeaconData
                        .coexist.eBeaconOrder;
        ind.beaconData.eBeacon.coexist.offsetTimeSlot = pInd->info.eBeaconData
                        .coexist.offsetTimeSlot;
        ind.beaconData.eBeacon.coexist.capBackOff = pInd->info.eBeaconData
                        .coexist.capBackOff;
        ind.beaconData.eBeacon.coexist.eBeaconOrderNBPAN = pInd->info
                        .eBeaconData.coexist.eBeaconOrderNBPAN;
    }

    /*
     * Initiate the callback, no need to check pMacCallbacks or the function
     * pointer for non-null, the calling function will check the function
     * pointer
     */
    pMacCallbacks->pBeaconNotifyIndCb(&ind);
}

/*!
 * @brief       Process the incoming Scan Confirm callback.
 *
 * @param       pCnf - pointer MAC Scan Confirm info
 */
static void processScanCnf(macMlmeScanCnf_t *pCnf)
{
    /* Confirmation structure */
    ApiMac_mlmeScanCnf_t cnf;

    /* Initialize the structure */
    memset(&cnf, 0, sizeof(ApiMac_mlmeScanCnf_t));

    /* copy the message to the confirmation structure */
    cnf.status = (ApiMac_status_t)pCnf->hdr.status;

    cnf.scanType = (ApiMac_scantype_t)pCnf->scanType;
    cnf.channelPage = pCnf->channelPage;
    cnf.phyId = pCnf->phyID;
    memcpy(cnf.unscannedChannels, pCnf->unscannedChannels,
    APIMAC_154G_CHANNEL_BITMAP_SIZ);
    cnf.resultListSize = pCnf->resultListSize;

    if(cnf.resultListSize)
    {
        if(cnf.scanType == ApiMac_scantype_energyDetect)
        {
            cnf.result.pEnergyDetect = (uint8_t *)ICall_malloc(
                            cnf.resultListSize * sizeof(uint8_t));
            if(cnf.result.pEnergyDetect)
            {
                memcpy(cnf.result.pEnergyDetect, pCnf->result.pEnergyDetect,
                       cnf.resultListSize);
            }
            else
            {
                cnf.status = ApiMac_status_noResources;
                cnf.resultListSize = 0;
            }
        }
        else
        {
            cnf.result.pPanDescriptor = (ApiMac_panDesc_t *)ICall_malloc(
                            cnf.resultListSize * sizeof(ApiMac_panDesc_t));
            if(cnf.result.pPanDescriptor)
            {
                uint8_t x;
                ApiMac_panDesc_t *pDstPanDesc = cnf.result.pPanDescriptor;
                macPanDesc_t *pSrcPanDesc = pCnf->result.pPanDescriptor;

                for(x = 0; x < cnf.resultListSize;
                                x++, pDstPanDesc++, pSrcPanDesc++)
                {
                    copyMacPanDescToApiMacPanDesc(pDstPanDesc, pSrcPanDesc);
                }
            }
            else
            {
                cnf.status = ApiMac_status_noResources;
                cnf.resultListSize = 0;
            }
        }
    }

    /* We processed the scan confirm, so free the results */
    if(scanResults != NULL)
    {
        ICall_free(scanResults);
        scanResults = NULL;
    }

    /*
     * Initiate the callback, no need to check pMacCallbacks or the function
     * pointer for non-null, the calling function will check the function
     * pointer
     */
    pMacCallbacks->pScanCnfCb(&cnf);

    if(cnf.resultListSize)
    {
        if(cnf.scanType == ApiMac_scantype_energyDetect)
        {
            ICall_free(cnf.result.pEnergyDetect);
        }
        else
        {
            ICall_free(cnf.result.pPanDescriptor);
        }
    }
}

/*!
 * @brief       Deallocate message function, MAC will deallocate the message.
 *
 * @param       pData - pointer to message to deallocate.
 */
static void macMsgDeallocate(macEventHdr_t *pData)
{
    if(pData != NULL)
    {
        /* Fill in the message content */
        pData->event = MAC_MSG_DEALLOCATE;
        pData->status = 0;

        /* Send the message */
        ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                             (ICALL_MSG_FORMAT_KEEP),
                             pData);
    }
}

/*!
 * @brief       Copy the MAC data indication to the API MAC data indication
 *
 * @param       pDst - pointer to the API MAC data indication
 * @param       pSrc - pointer to the MAC data indication
 */
static void copyDataInd(ApiMac_mcpsDataInd_t *pDst, macMcpsDataInd_t *pSrc)
{
    /* Initialize the structure */
    memset(pDst, 0, sizeof(ApiMac_mcpsDataInd_t));

    /* copy the message to the indication structure */
    copyMacAddrToApiMacAddr(&(pDst->srcAddr), &(pSrc->mac.srcAddr));
    copyMacAddrToApiMacAddr(&(pDst->dstAddr), &(pSrc->mac.dstAddr));
    pDst->timestamp = pSrc->mac.timestamp;
    pDst->timestamp2 = pSrc->mac.timestamp2;
    pDst->srcPanId = pSrc->mac.srcPanId;
    pDst->dstPanId = pSrc->mac.dstPanId;
    pDst->mpduLinkQuality = pSrc->mac.mpduLinkQuality;
    pDst->correlation = pSrc->mac.correlation;
    pDst->rssi = pSrc->mac.rssi;
    pDst->dsn = pSrc->mac.dsn;
    pDst->payloadIeLen = pSrc->mac.payloadIeLen;
    pDst->pPayloadIE = pSrc->mac.pPayloadIE;
    pDst->fhFrameType = (ApiMac_fhFrameType_t)pSrc->internal.fhFrameType;
    pDst->fhProtoDispatch = (ApiMac_fhDispatchType_t)pSrc->mac.fhProtoDispatch;
    pDst->frameCntr = (uint32_t)pSrc->mac.frameCntr;
    copyMacSecToApiMacSec(&(pDst->sec), &(pSrc->sec));

    /* Copy the payload information */
    pDst->msdu.len = pSrc->msdu.len;
    pDst->msdu.p = pSrc->msdu.p;
}

/*!
 * @brief       Compare a received TIMAC MCPS Data Alloc Command Status message
 *              for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchMcpsDataAlloc(ICall_ServiceEnum src, ICall_EntityID dest,
                               const void *msg)
{
    macMcpsDataAlloc_t *pMsg = (macMcpsDataAlloc_t *)msg;

    return ((pMsg->hdr.event == MAC_MCPS_DATA_ALLOC) ? true : false);
}

/*!
 * @brief       Allocates the MAC buffer for data request.
 *
 * @param       len - payload length
 * @param       securityLevel - MAC security level
 * @param       securityLevel - MAC security level
 * @param       includeFhIEs - bitmap indicating which FH IE's
 *                             need to be included.
 *
 * @return      pointer to allocated buffer or NULL if not allocated
 */
static macMcpsDataReq_t *mcpsDataAllocMsg(uint16_t len,
                                          uint8_t securityLevel,
                                          uint8_t keyIdMode,
                                          uint32 includeFhIEs,
                                          uint16_t payloadIeLen)
{
    macMcpsDataReq_t *pDataReq = NULL;
    macMcpsDataAlloc_t *msg = (macMcpsDataAlloc_t *)ICall_allocMsg(
                    sizeof(macMcpsDataAlloc_t));

    if(msg != NULL)
    {
        ICall_Errno errno;

        /* Fill in the message content */
        msg->hdr.event = MAC_MCPS_DATA_ALLOC;
        msg->hdr.status = 0;
        msg->len = len;
        msg->securityLevel = securityLevel;
        msg->keyIdMode = keyIdMode;
        msg->includeFhIEs = includeFhIEs;
        msg->payloadIeLen = payloadIeLen;

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     msg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            macMcpsDataAlloc_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, matchMcpsDataAlloc,
                                    (NULL),
                                    (NULL), (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                pDataReq = pCmdStatus->pDataReq;
            }
        }

        /* pCmdStatus is the same as msg */
        ICall_freeMsg(msg);
    }

    return (pDataReq);
}

/*!
 * @brief       Copy the common security type from App type to Mac Stack type.
 *
 * @param       pDst - pointer to the mac stack  type
 * @param       pSrc - pointer to the application type
 */
static void copyApiMacSecToMacSec(macSec_t *pDst, ApiMac_sec_t *pSrc)
{
    /* Copy each element of the structure */
    memcpy(pDst->keySource, pSrc->keySource, APIMAC_KEY_SOURCE_MAX_LEN);
    pDst->securityLevel = pSrc->securityLevel;
    pDst->keyIdMode = pSrc->keyIdMode;
    pDst->keyIndex = pSrc->keyIndex;
}

/*!
 * @brief       Copy the common address type from App type to Mac Stack type.
 *
 * @param       pDst - pointer to the mac stack type
 * @param       pSrc - pointer to the application type
 */
static void copyApiMacAddrToMacAddr(sAddr_t *pDst, ApiMac_sAddr_t *pSrc)
{
    /* Copy each element of the structure */
    pDst->addrMode = pSrc->addrMode;
    if(pSrc->addrMode == ApiMac_addrType_short)
    {
        pDst->addr.shortAddr = pSrc->addr.shortAddr;
    }
    else
    {
        memcpy(pDst->addr.extAddr, pSrc->addr.extAddr, sizeof(sAddrExt_t));
    }
}

/*!
 * @brief       Compare a received TIMAC Get Request Status message
 *              for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchGetReq(ICall_ServiceEnum src, ICall_EntityID dest,
                        const void *msg)
{
    macGetParam_t *pMsg = (macGetParam_t *)msg;

    return ((pMsg->hdr.event == MAC_GET_REQ) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Get Frequency Hopping Request Status
 *              message for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchGetFhReq(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg)
{
    macGetParam_t *pMsg = (macGetParam_t *)msg;

    return ((pMsg->hdr.event == MAC_FH_GET_REQ) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Get Security Request Status message
 *              for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchGetSecurityReq(ICall_ServiceEnum src, ICall_EntityID dest,
                                const void *msg)
{
    macGetParam_t *pMsg = (macGetParam_t *)msg;

    return ((pMsg->hdr.event == MAC_GET_SECURITY_REQ) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Reset Request Status message
 *              for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchResetReq(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg)
{
    macGetParam_t *pMsg = (macGetParam_t *)msg;

    return ((pMsg->hdr.event == MAC_RESET_REQ) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Set Request Status message
 *              for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSetReq(ICall_ServiceEnum src, ICall_EntityID dest,
                        const void *msg)
{
    macSetParam_t *pMsg = (macSetParam_t *)msg;

    return ((pMsg->hdr.event == MAC_SET_REQ) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Set Frequency Hopping Request Status
 *              message for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSetFhReq(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg)
{
    macSetParam_t *pMsg = (macSetParam_t *)msg;

    return ((pMsg->hdr.event == MAC_FH_SET_REQ) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Set Security Request Status message
 *              for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSetSecurityReq(ICall_ServiceEnum src, ICall_EntityID dest,
                                const void *msg)
{
    macSetParam_t *pMsg = (macSetParam_t *)msg;

    return ((pMsg->hdr.event == MAC_SET_SECURITY_REQ) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Random Byte Request Status message
 *              for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchRandomByteReq(ICall_ServiceEnum src, ICall_EntityID dest,
                               const void *msg)
{
    randomByte_t *pMsg = (randomByte_t *)msg;

    return ((pMsg->hdr.event == MAC_RANDOM_BYTE) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Enable FH Status message
 *              for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchEnableFH(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg)
{
    macEventHdr_t *pMsg = (macEventHdr_t *)msg;

    return ((pMsg->event == MAC_ENABLE_FH) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Security Add Device Status message
 *              for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecAddDevice(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg)
{
    macGetParam_t *pMsg = (macGetParam_t *)msg;

    return ((pMsg->hdr.event == MAC_SEC_ADD_DEVICE) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Security Delete Device Status message
 *              for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecDelDevice(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg)
{
    macGetParam_t *pMsg = (macGetParam_t *)msg;

    return ((pMsg->hdr.event == MAC_SEC_DEL_DEVICE) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Security Delete Key Status message
 *              for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecDelKeyAndDevices(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg)
{
    macGetParam_t *pMsg = (macGetParam_t *)msg;

    return ((pMsg->hdr.event == MAC_SEC_DEL_KEY_AND_DEVICES) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Security Delete All Devices Status
 *              message for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecDelAllDevices(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg)
{
    macGetParam_t *pMsg = (macGetParam_t *)msg;

    return ((pMsg->hdr.event == MAC_SEC_DEL_ALL_DEVICES) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Security Get Default Source Key
 *              Status message for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecGetDefaultSrcKey(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg)
{
    macGetParam_t *pMsg = (macGetParam_t *)msg;

    return ((pMsg->hdr.event == MAC_SEC_GET_DEFAULT_SOURCE_KEY) ? true : false);
}

/*!
 * @brief       Compare a received TIMAC Security Add Key and Init Frame Counter
 *              Status message for a match.
 *
 * @param       src - originator of the message as a service enumeration
 * @param       dest - destination entity id of the message
 * @param       msg - pointer to the message body
 *
 * @return      TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecAddKeyInitFC(ICall_ServiceEnum src, ICall_EntityID dest,
                          const void *msg)
{
    macGetParam_t *pMsg = (macGetParam_t *)msg;

    return ((pMsg->hdr.event == MAC_SEC_ADD_KEY_INIT_FC) ? true : false);
}

/*!
 * @brief       Generic function for both Get Req and Get Security Req.
 *
 * @param       eventId - ICall Message Event
 * @param       matchFn - function pointer to function to wait for right
 *                        response message
 * @param       pibAttribute - attribute Id
 * @param       pValue - pointer place to put value
 * @param       pLen - pointer to place to put the length
 *
 * @return      ApiMac_status_t
 */
static ApiMac_status_t getTypeReq(uint8 eventId, ICall_MsgMatchFn matchFn,
                                  uint8_t pibAttribute,
                                  void *pValue, uint16_t *pLen)
{
    ApiMac_status_t status = ApiMac_status_noResources;

    /* Allocate message buffer space */
    macGetParam_t *pMsg = (macGetParam_t *)ICall_allocMsg(
                    sizeof(macGetParam_t));

    if(pMsg != NULL)
    {
        ICall_Errno errno;

        /* Fill in the message content */
        pMsg->hdr.event = eventId;
        pMsg->hdr.status = 0;
        pMsg->paramID = pibAttribute;
        pMsg->pValue = pValue;

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     pMsg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            macGetParam_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, matchFn, (NULL),
                                    (NULL),
                                    (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (ApiMac_status_t)pCmdStatus->hdr.status;
                if((status == MAC_SUCCESS) && (pCmdStatus->len > 0))
                {
                    memcpy(pValue, pCmdStatus->pValue, pCmdStatus->len);
                    if(pLen)
                    {
                        *pLen = pCmdStatus->len;
                    }
                    ICall_free(pMsg->pValue);
                }
            }
        }

        /* pCmdStatus is the same as msg */
        ICall_freeMsg(pMsg);
    }
    return (status);
}

/*!
 * @brief       Generic function for both Set Req and Set Security Req.
 *
 * @param       eventId - ICall Message Event
 * @param       matchFn - function pointer to function to wait for right
 *                        response message
 * @param       pibAttribute - attribute Id
 * @param       pValue - pointer place to put value
 *
 * @return      ApiMac_status_t
 */
static ApiMac_status_t setTypeReq(uint8 eventId, ICall_MsgMatchFn matchFn,
                                  uint8_t pibAttribute,
                                  void *pValue)
{
    ApiMac_status_t status = ApiMac_status_noResources;

    /* Allocate message buffer space */
    macSetParam_t *pMsg = (macSetParam_t *)ICall_allocMsg(
                    sizeof(macSetParam_t));

    if(pMsg != NULL)
    {
        ICall_Errno errno;

        /* Fill in the message content */
        pMsg->hdr.event = eventId;
        pMsg->hdr.status = 0;
        pMsg->paramID = pibAttribute;
        pMsg->paramValue = pValue;

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     pMsg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            macSetParam_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, matchFn, (NULL),
                                    (NULL),
                                    (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (ApiMac_status_t)pCmdStatus->hdr.status;
            }
        }

        /* pCmdStatus is the same as msg */
        ICall_freeMsg(pMsg);
    }
    return (status);
}

/*!
 * @brief       Generic function to send a macEventHdr_t message and
 *              expect a status returned.
 *
 * @param       eventId - ICall Message Event
 * @param       matchFn - function pointer to function to wait for right
 *                        response message
 *
 * @return      ApiMac_status_t
 */
static ApiMac_status_t sendEvtExpectStatus(uint8_t eventId,
                                           ICall_MsgMatchFn matchFn)
{
    ApiMac_status_t status = ApiMac_status_noResources;

    /* Allocate message buffer space */
    macEventHdr_t *pMsg = (macEventHdr_t *)ICall_allocMsg(
                    sizeof(macEventHdr_t));

    if(pMsg != NULL)
    {
        ICall_Errno errno;

        /* Fill in the message content */
        pMsg->event = eventId;
        pMsg->status = 0;

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     pMsg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            macResetReq_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, matchFn, (NULL),
                                    (NULL),
                                    (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (ApiMac_status_t)pCmdStatus->hdr.status;
            }
        }

        /* pCmdStatus is the same as msg */
        ICall_freeMsg(pMsg);
    }
    return (status);
}

/*!
 * @brief       Generic function to send a macEventHdr_t message with
 *              no expected return.
 *
 * @param       eventId - ICall Message Event
 *
 * @return      The status of the request, as follows:
 *              ApiMac_status_success - Operation successful
 *              ApiMac_status_noResources - Resources not available
 */
static ApiMac_status_t sendEvt(uint8_t eventId)
{
    ApiMac_status_t ret = ApiMac_status_noResources;
    /* Allocate message buffer space */
    macEventHdr_t *pMsg = (macEventHdr_t *)ICall_allocMsg(
                    sizeof(macEventHdr_t));

    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->event = eventId;
        pMsg->status = 0;

        /* Send the message */
        if(ICall_sendServiceMsg(ApiMac_appEntity, ICALL_SERVICE_CLASS_TIMAC,
                                (ICALL_MSG_FORMAT_KEEP),
                                pMsg)
           == ICALL_ERRNO_SUCCESS)
        {
            ret = ApiMac_status_success;
        }
    }

    return (ret);
}

/*!
 * @brief Parses the payload information element.
 *
 * @param pPayload - pointer to the buffer with the payload IEs.
 * @param payloadLen - length of the buffer with the payload IEs.
 * @param pList - pointer to point of place to allocated the link list.
 * @param group - true to check for termination IE.
 *
 * @return      ApiMac_status_t
 */
static ApiMac_status_t parsePayloadIEs(uint8_t *pContent, uint16_t contentLen,
                                       ApiMac_payloadIeRec_t **pList,
                                       bool group)
{
    ApiMac_payloadIeRec_t* pIe = (ApiMac_payloadIeRec_t*) NULL;
    ApiMac_payloadIeRec_t* pTempIe;
    uint16_t lenContent = 0;
    ApiMac_status_t status = ApiMac_status_success;

    if((pContent == NULL) || (contentLen == 0))
    {
        return (ApiMac_status_noData);
    }

    /* Initialize the list pointer */
    *pList = (ApiMac_payloadIeRec_t*) NULL;

    while(lenContent < contentLen)
    {
        uint16_t hdr;
        bool typeLong;
        uint8_t ieId;

        hdr = MAKE_UINT16(pContent[0], pContent[1]);
        pContent += PAYLOAD_IE_HEADER_LEN; /* Move past the header */

        typeLong = GET_SUBIE_TYPE(hdr);
        if(typeLong)
        {
            ieId = GET_SUBIE_ID_LONG(hdr);
        }
        else
        {
            ieId = GET_SUBIE_ID_SHORT(hdr);
        }

        if(group)
        {
            if(!typeLong)
            {
                /* Only long IE types when parsing Group IEs */
                status = ApiMac_status_unsupported;
                break;
            }

            if(ApiMac_payloadIEGroup_term == ieId)
            {
                /* Termination IE found */
                break;
            }
        }

        pTempIe = (ApiMac_payloadIeRec_t *)ICall_malloc(
                        sizeof(ApiMac_payloadIeRec_t));

        if(pTempIe)
        {
            memset(pTempIe, 0, sizeof(ApiMac_payloadIeRec_t));

            /* If nothing in the list, add the node first otherwise
             add it to the end of the list */
            if(*pList == NULL)
            {
                *pList = pTempIe;
            }
            else
            {
                /* pIe should point to the previous node,
                 since it was allocated in the previous iteration */
                pIe->pNext = pTempIe;
            }

            pIe = pTempIe;
            pTempIe = NULL;

            /* Fill in the IE information */
            pIe->item.ieTypeLong = typeLong;
            pIe->item.ieId = ieId;

            if(pIe->item.ieTypeLong)
            {
                pIe->item.ieContentLen = GET_SUBIE_LEN_LONG(hdr);
            }
            else
            {
                pIe->item.ieContentLen = GET_SUBIE_LEN_SHORT(hdr);
            }
            pIe->item.pIEContent = pContent;

            /* Update length and pointer */
            lenContent += PAYLOAD_IE_HEADER_LEN + pIe->item.ieContentLen;
            pContent += pIe->item.ieContentLen;
        }
        else
        {
            status = ApiMac_status_noResources;
            break;
        }
    }

    if((status != ApiMac_status_success) && (NULL != *pList))
    {
        /* not successful in parsing all header ie's, free the linked list */
        pIe = *pList;
        while(NULL != pIe)
        {
            pTempIe = pIe->pNext;
            ICall_free(pIe);
            pIe = pTempIe;
        }
        *pList = NULL;
    }

    return (status);
}

/*!
 * @brief       Convert API txOptions to bitmasked txOptions.
 *
 * @param       txOptions - tx options structure
 *
 * @return      bitmasked txoptions
 */
static uint16_t convertTxOptions(ApiMac_txOptions_t txOptions)
{
    uint16_t retVal = 0;

    if(txOptions.ack == true)
    {
        retVal |= MAC_TXOPTION_ACK;
    }
    if(txOptions.indirect == true)
    {
        retVal |= MAC_TXOPTION_INDIRECT;
    }
    if(txOptions.pendingBit == true)
    {
        retVal |= MAC_TXOPTION_PEND_BIT;
    }
    if(txOptions.noRetransmits == true)
    {
        retVal |= MAC_TXOPTION_NO_RETRANS;
    }
    if(txOptions.noConfirm == true)
    {
        retVal |= MAC_TXOPTION_NO_CNF;
    }
    if(txOptions.useAltBE == true)
    {
        retVal |= MAC_TXOPTION_ALT_BE;
    }
    if(txOptions.usePowerAndChannel == true)
    {
        retVal |= MAC_TXOPTION_PWR_CHAN;
    }

    return (retVal);
}

/*!
 * @brief       Generic function to set an FH attribute
 *
 * @param       pibAttribute - attribute to set
 * @param       pValue - pointer to the attribute value
 *
 * @return      status result
 */
static ApiMac_status_t mlmeSetFhReq(uint16_t pibAttribute, void *pValue)
{
    ApiMac_status_t status = ApiMac_status_noResources;

    /* Allocate message buffer space */
    macFHSetParam_t *pMsg = (macFHSetParam_t *)ICall_allocMsg(
                    sizeof(macFHSetParam_t));

    if(pMsg != NULL)
    {
        ICall_Errno errno;

        /* Fill in the message content */
        pMsg->hdr.event = MAC_FH_SET_REQ;
        pMsg->hdr.status = 0;
        pMsg->paramID = pibAttribute;
        pMsg->paramValue = pValue;

        /* Send the message */
        errno = ICall_sendServiceMsg(ApiMac_appEntity,
                                     (ICALL_SERVICE_CLASS_TIMAC),
                                     (ICALL_MSG_FORMAT_KEEP),
                                     pMsg);

        if(errno == ICALL_ERRNO_SUCCESS)
        {
            macFHSetParam_t *pCmdStatus = NULL;

            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, matchSetFhReq,
                                    (NULL),
                                    (NULL), (void **)&pCmdStatus);

            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (ApiMac_status_t)pCmdStatus->hdr.status;
            }
        }

        /* pCmdStatus is the same as msg */
        ICall_freeMsg(pMsg);
    }
    return (status);
}

