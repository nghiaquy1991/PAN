/******************************************************************************

 @file csf.h

 @brief Collector Specific Functions API

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
#ifndef CSF_H
#define CSF_H

/******************************************************************************
 Includes
 *****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include "llc.h"
#include "cllc.h"
#include "smsgs.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 Constants
 *****************************************************************************/

/*! CSF Events - Key Event */
#define CSF_KEY_EVENT 0x0001

#define CSF_INVALID_SHORT_ADDR   0xFFFF

/******************************************************************************
 Typedefs
 *****************************************************************************/


/******************************************************************************
 Function Prototypes
 *****************************************************************************/

/*!
 * @brief       The application calls this function during initialization.
 *
 * @param       sem - pointer to semaphore used by MAC API
 */
extern void Csf_init(void *sem);

/*!
 * @brief       The application must call this function periodically to
 *              process any events that this module needs to process.
 */
extern void Csf_processEvents(void);

/*!
 * @brief       The application calls this function to retrieve the stored
 *              network information.  The stored network information was saved
 *              after starting a network.
 *              <BR>
 *              NOTE: If the "fh" is true you will need to free the
 *              pInfo->info.fhNetInfo.fhInfo.bcNumChans and
 *              pInfo->info.fhNetInfo.fhInfo.pUnicastChans buffers through
 *              Csf_free().
 *
 * @param       pInfo - pointer to network information structure
 *
 * @return      True if the network information is available
 */
extern bool Csf_getNetworkInformation(Llc_netInfo_t *pInfo);

/*!
 * @brief       The application calls this function to indicate that it has 
 *              started or restored the device in a network.
 *
 *              The information will be saved and used to determine if a 
 *              network was already started and should be restored instead
 *              of started.
 *
 * @param       restored - true if restored in network
 * @param       pNetworkInfo - network information structure
 */
extern void Csf_networkUpdate(bool restored, Llc_netInfo_t *pNetworkInfo);

/*!
 * @brief       The application calls this function to indicate that a device
 *              has joined the network.
 *
 *              The information will be saved.
 *
 * @param       pDevInfo - pointer to the device information
 * @param       capInfo - capability information of the joining device.
 *
 * @return      ApiMac_assocStatus_success, ApiMac_assocStatus_panAtCapacity,
 *              or ApiMac_assocStatus_panAccessDenied
 */
extern ApiMac_assocStatus_t Csf_deviceUpdate(
                ApiMac_deviceDescriptor_t *pDevInfo,
                ApiMac_capabilityInfo_t *pCapInfo);

/*!
 * @brief       The application calls this function to indicate that a device
 *              is no longer active in the network.  This function will be
 *              called when the device doesn't respond to the tracking request.
 *
 *              The information will be saved.
 *
 * @param       pDevInfo - pointer to the device information
 * @param       timeout - true if not active because of tracking timeout.
 *              meaning that the device didn't respond to the tracking request
 *              within the timeout period.
 */
extern void Csf_deviceNotActiveUpdate(ApiMac_deviceDescriptor_t *pDevInfo,
                                      bool timeout);

/*!
 * @brief       The application calls this function to indicate that a device
 *              has responded to a Config Request.
 *
 *              The information will be saved.
 *
 * @param       pSrcAddr - short address of the device that sent the message
 * @param       rssi - the received packet's signal strength
 * @param       pMsg - pointer to the Config Response message
 */
extern void Csf_deviceConfigUpdate(ApiMac_sAddr_t *pSrcAddr, int8_t rssi,
                                   Smsgs_configRspMsg_t *pMsg);

/*!
 * @brief       The application calls this function to indicate that a device
 *              has reported sensor data.
 *
 *              The information will be saved.
 *
 * @param       pSrcAddr - short address of the device that sent the message
 * @param       rssi - the received packet's signal strength
 * @param       pMsg - pointer to the Sensor Data message
 */
extern void Csf_deviceSensorDataUpdate(ApiMac_sAddr_t *pSrcAddr, int8_t rssi,
                                       Smsgs_sensorMsg_t *pMsg);

/*!
 * @brief       The application calls this function to indicate that a device
 *              set a Toggle LED Response message.
 *
 * @param       pSrcAddr - short address of the device that sent the message
 * @param       ledState - 0 is off, 1 is on
 */
extern void Csf_toggleResponseReceived(ApiMac_sAddr_t *pSrcAddr, bool ledState);

/*!
 * @brief       The application calls this function to indicate that the 
 *              Coordinator's state has changed.
 *
 * @param       state - new state
 */
extern void Csf_stateChangeUpdate(Cllc_states_t state);

/*!
 * @brief       Initialize the tracking clock.
 */
extern void Csf_initializeTrackingClock(void);

/*!
 * @brief       set the tracking clock.
 *
 * @param       trackingTime - set timer this value (in msec)
 */
extern void Csf_setTrackingClock(uint32_t trackingTime);

/*!
 * @brief       Initialize the trickle timer clock
 */
extern void Csf_initializeTrickleClock(void);

/*!
 * @brief       Initialize the clock setting join permit duration
 */
extern void Csf_initializeJoinPermitClock(void);

/*!
 * @brief       Initialize the clock setting config request delay
 */
extern void Csf_initializeConfigClock(void);

/*!
 * @brief       Set trickle clock
 *
 * @param       trickleTime - duration of trickle timer( in msec)
 * @param       frameType - type of Async frame
 */
extern void Csf_setTrickleClock(uint32_t trickleTime, uint8_t frameType);

/*!
 * @brief       Set Join Permit clock
 *
 * @param       joinDuration - duration for which join permit is TRUE( in msec)
 */
extern void Csf_setJoinPermitClock(uint32_t joinDuration);

/*!
 * @brief       Set Config request delay clock
 *
 * @param       delay - duration config request event is set( in msec)
 */
extern void Csf_setConfigClock(uint32_t delay);

/*!
 * @brief       Read the number of device list items stored
 *
 * @return      number of entries in the device list
 */
extern uint16_t Csf_getNumDeviceListEntries(void);

/*!
 * @brief       Find the short address from a given extended address
 *
 * @param       pExtAddr - extended address
 *
 * @return      CSF_INVALID_SHORT_ADDR if not found, otherwise the short address
 */
extern uint16_t Csf_getDeviceShort(ApiMac_sAddrExt_t *pExtAddr);

/*!
 * @brief       Find entry in device list from an address
 *
 * @param       pDevAddr - device address
 * @param       pItem - place to put the device information
 *
 * @return      true if found, false if not
 */
extern bool Csf_getDevice(ApiMac_sAddr_t *pDevAddr, Llc_deviceListItem_t *pItem);

/*!
 * @brief       Find entry in device list
 *
 * @param       devIndex - Device number (not address)
 * @param       pItem - place to put the device information
 *
 * @return      true if found, false if not
 */
extern bool Csf_getDeviceItem(uint16_t devIndex, Llc_deviceListItem_t *pItem);

/*!
 * @brief       Find entry in device list
 *
 * @param       size - number of bytes to allocate from heap
 *
 * @return      true if found, false if not
 */
extern void *Csf_malloc(uint16_t size);

/*!
 * @brief       Csf implementation for memory de-allocation
 *
 * @param       ptr - a valid pointer to the memory to free
 */
extern void Csf_free(void *ptr);

/*!
 * @brief       Update the Frame Counter
 *
 * @param       pDevAddr - pointer to device's address. If this pointer
 *                         is NULL, it means that this is the frame counter
 *                         for this device.
 * @param       frameCntr -  valur of frame counter
 */
extern void Csf_updateFrameCounter(ApiMac_sAddr_t *pDevAddr,
                                   uint32_t frameCntr);

/*!
 * @brief       Get the Frame Counter
 *
 * @param       pDevAddr - pointer to device's address. If this pointer
 *                         is NULL, it means that this is the frame counter
 *                         for this device.
 * @param       pFrameCntr -  pointer to place to put the frame counter
 *
 * @return      true if the frame counter existed, false if not.
 */
extern bool Csf_getFrameCounter(ApiMac_sAddr_t *pDevAddr,
                                   uint32_t *pFrameCntr);

/*!
 * @brief       Delete an entry from the device list
 *
 * @param       pAddr - address to remove from device list.
 */
extern void Csf_removeDeviceListItem(ApiMac_sAddrExt_t *pAddr);

/*!
 * @brief       Assert Indication
 *
 * @param       reason - Reason for Assert
 *                     2 - HAL/ICALL
 *                     3 - MAC
 *                     4 - TIRTOS
 */
extern void Csf_assertInd(uint8_t reason);

/*!
 * @brief       Clear all the NV Items
 */
extern void Csf_clearAllNVItems(void);

/*!
 * @brief       Add an entry into the black list
 *
 * @param       pAddr - address to add into the black list. The address
 *                      can be either short or extended.
 *
 * @return      true if added or already existed, false if problem
 */
extern bool Csf_addBlackListItem(ApiMac_sAddr_t *pAddr);

/*!
 * @brief       Check if config timer is active
 *
 * @return      true if active, false if not active
 */
extern bool Csf_isConfigTimerActive(void);

#ifdef __cplusplus
}
#endif

#endif /* CSF_H */

