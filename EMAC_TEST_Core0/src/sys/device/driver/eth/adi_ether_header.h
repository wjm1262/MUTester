/*
 * adi_ether_header.h
 *
 *  Created on: 2015-6-6
 *      Author: Administrator
 */


/*******************************************************************************
 *
 * Copyright(c) 2011 Analog Devices, Inc. All Rights Reserved.
 *
 * This software is proprietary and confidential.  By using this software you
 * agree to the terms of the associated Analog Devices License Agreement.
 *
 ******************************************************************************/

/*!
* @file      adi_ether.h
*
* @brief     Ethernet Device Driver Interface
*
* @details
*            Ethernet device driver interface (EDDI) defines the critical
*            functions that are required to be supported by all ethernet
*            device drivers. These primtive EDDI functions are used by the
*            stack interface, all other features supported by the ethernet
*            controllers has to be supplied as controller specific interface
*            functions.
*/

/** @addtogroup  Ethernet_Driver Ethernet Driver Interface
 *  @{
 */

#ifndef __ADI_ETHER_H__
#define __ADI_ETHER_H__

#ifdef _MISRA_RULES
#pragma diag(push)
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <adi_types.h>
#include <drivers/ethernet/adi_ether_misra.h>

/*!  Ethenet driver handle */
typedef void* ADI_ETHER_HANDLE;

/*! Ethernet callback function */
typedef void (*ADI_ETHER_CALLBACK_FN) (void*, uint32_t, void*);

/*  \enum ADI_ETHER_RESULT
 *
 *  Generic result codes returned by the ethernet drivers
 */
typedef enum ADI_ETHER_RESULT
{
    ADI_ETHER_RESULT_SUCCESS =  0,     /*!< Ethernet API is successful.       */
    ADI_ETHER_RESULT_NO_MEMORY,        /*!< No memory in the driver.          */
    ADI_ETHER_RESULT_INVALID_SEQUENCE, /*!< Invalid API execution sequence.   */
    ADI_ETHER_RESULT_INVALID_PARAM,    /*!< Invalid Input Parameter.          */
    ADI_ETHER_RESULT_RESET_FAILED,     /*!< Reset of the controller failed.   */
    ADI_ETHER_RESULT_NULL_BUFFER,      /*!< NULL buffer passed.               */
    ADI_ETHER_RESULT_PHYINIT_FAILED,   /*!< PHY initialization failed.        */
    ADI_ETHER_RESULT_RETRY,            /*!< Re-try send / receive             */
    ADI_ETHER_RESULT_INVALID_DEVICE_ENTRY, /* Invalid Device Entry            */
    ADI_ETHER_RESULT_FAILED            /*!< Generic API failure.              */

} ADI_ETHER_RESULT;

/*  \enum ADI_ETHER_EVENT
 *
 *   Events codes returned by the ethernet driver
 */
typedef enum  ADI_ETHER_EVENT
{
    ADI_ETHER_EVENT_START     =  0x124,  /*!< First value of ethernet events.   */  /* FIXIT */
    ADI_ETHER_EVENT_FRAME_RCVD,          /*!< One or more frames received.      */
    ADI_ETHER_EVENT_FRAME_XMIT,          /*!< One or more frames tramsmitted.   */
    ADI_ETHER_EVENT_INTERRUPT,           /*!< Ethernet event has occured.       */
    ADI_ETHER_EVENT_PHY_INTERRUPT        /*!< PHY interrupt has occured.        */

} ADI_ETHER_EVENT;

/*  \enum ADI_ETHER_PHY_INTERRUPT
 *
 *   For PHY interrupt event the argument specifies the actual event, one or
 *   more events could be encoded in the status word.
 */
typedef enum  ADI_ETHER_PHY_STATUS
{
    ADI_ETHER_PHY_LINK_DOWN        = (1UL << 0),   /*!< Link is down.              */
    ADI_ETHER_PHY_LINK_UP          = (1UL << 1),   /*!< Link is up.                */
    ADI_ETHER_PHY_AN_COMPLETE      = (1UL << 2),   /*!< Completed autonegotiation  */
    ADI_ETHER_PHY_10T_HALF_DUPLEX  = (1UL << 3),   /*!< 10Base-T half duplex.      */
    ADI_ETHER_PHY_10T_FULL_DUPLEX  = (1UL << 4),   /*!< 10Base-T full duplex.      */
    ADI_ETHER_PHY_100T_HALF_DUPLEX = (1UL << 5),   /*!< 100Base-T half duplex.     */
    ADI_ETHER_PHY_100T_FULL_DUPLEX = (1UL << 6),   /*!< 100Base-T full duplex.     */
    ADI_ETHER_PHY_LOOPBACK         = (1UL << 7)    /*!< Loopback on.               */

} ADI_ETHER_PHY_STATUS;

/*
 * \struct ADI_ETHER_BUFFER
 *
 * Ethernet buffer structure
 */

#define ADI_ETHER_DRIVER_MEM  (20)            /*!< Driver memory - primarily used for DMA */
typedef struct ADI_ETHER_BUFFER
{
    char_t   Reserved[ADI_ETHER_DRIVER_MEM];  /*!< Reserved for physical device */
    void     *Data;                           /*!< Pointer to data.             */
    uint32_t ElementCount;                    /*!< Data element count.          */
    uint32_t ElementWidth;                    /*!< Data element width in bytes. */  /* BC */
    void*    CallbackParameter;               /*!< Callback flag/pArg value.    */
    uint32_t ProcessedFlag;                   /*!< Buffer processed flag.       */
    uint32_t ProcessedElementCount;           /*!< Actual bytes read or sent.   */
    struct   ADI_ETHER_BUFFER  *pNext;        /*!< Next buffer pointer.         */
    void     *PayLoad;                        /*!< Pointer to IP Payload.       */  /* BC */
    uint16_t IPHdrChksum;                     /*!< IP header checksum.          */
    uint16_t IPPayloadChksum;                 /*!< IP header & payload checksum.*/
    uint16_t StatusWord;                      /*!< The frame status word.       */  /* BC */
    void*    x;                               /*!< Network interface.           */  /* BC */

} ADI_ETHER_BUFFER;

/*
 * \struct ADI_ETHER_STATISTICS_COUNTS
 *
 *  Ethernet statistical counters
 */
typedef struct ADI_ETHER_STATISTICS_COUNTS
{
    uint64_t cEMAC_RX_CNT_OK;      /*!< RX frame successful count.             */
    uint64_t cEMAC_RX_CNT_FCS;     /*!< RX frame FCS failure count.            */
    uint64_t cEMAC_RX_CNT_ALIGN;   /*!< RX alignment error count.              */
    uint64_t cEMAC_RX_CNT_OCTET;   /*!< RX octets successfully received count. */
    uint64_t cEMAC_RX_CNT_LOST;    /*!< MAC sublayer error RX frame count.     */
    uint64_t cEMAC_RX_CNT_UNI;     /*!< Unicast RX frame count.                */
    uint64_t cEMAC_RX_CNT_MULTI;   /*!< Multicast RX frame count.              */
    uint64_t cEMAC_RX_CNT_BROAD;   /*!< Broadcast RX frame count.              */
    uint64_t cEMAC_RX_CNT_IRL;     /*!< RX frame in range error count.         */
    uint64_t cEMAC_RX_CNT_ORL;     /*!< RX frame out of range error count.     */
    uint64_t cEMAC_RX_CNT_LONG;    /*!< RX frame too long count.               */
    uint64_t cEMAC_RX_CNT_MACCTL;  /*!< MAC control RX frame count.            */
    uint64_t cEMAC_RX_CNT_OPCODE;  /*!< Unsupported op-code RX frame count.    */
    uint64_t cEMAC_RX_CNT_PAUSE;   /*!< MAC control pause RX frame count.      */
    uint64_t cEMAC_RX_CNT_ALLF;    /*!< Overall RX frame count.                */
    uint64_t cEMAC_RX_CNT_ALLO;    /*!< Overall RX octet count.                */
    uint64_t cEMAC_RX_CNTD;        /*!< Type/Length consistent RX frame count. */
    uint64_t cEMAC_RX_CNT_SHORT;   /*!< RX frame fragment count,count x < 64.  */
    uint64_t cEMAC_RX_CNT_EQ64;    /*!< Good RX frames,count x = 64.           */
    uint64_t cEMAC_RX_CNT_LT128;   /*!< Good RX frames,count x 64 <= x < 128.  */
    uint64_t cEMAC_RX_CNT_LT256;   /*!< Good RX frames,count x 128 <= x < 256. */
    uint64_t cEMAC_RX_CNT_LT512;   /*!< Good RX frames,count x 256 <= x < 512. */
    uint64_t cEMAC_RX_CNT_LT1024;  /*!< Good RX frames,count x 512 <= x < 1024 */
    uint64_t cEMAC_RX_CNT_EQ1024;  /*!< Good RX frames,count x >= 1024.        */
    uint64_t cEMAC_TX_CNT_OK;      /*!< TX frame successful count.             */
    uint64_t cEMAC_TX_CNT_SCOLL;   /*!< Successful tx frames after single collision.    */
    uint64_t cEMAC_TX_CNT_MCOLL;   /*!< TX frames successful after multiple collisions. */
    uint64_t cEMAC_TX_CNT_OCTET;   /*!< TX octets successfully received count.*/
    uint64_t cEMAC_TX_CNT_DEFER;   /*!< TX frame delayed due to busy count.   */
    uint64_t cEMAC_TX_CNT_LATE;    /*!< Late TX collisions count.             */
    uint64_t cEMAC_TX_CNT_ABORTC;  /*!< TX frame failed due to Excessive collisions.    */
    uint64_t cEMAC_TX_CNT_LOST;    /*!< Internal MAC sublayer error TX frame count.     */
    uint64_t cEMAC_TX_CNT_CRS;     /*!< Carrier sense deasserted during TX frame count. */
    uint64_t cEMAC_TX_CNT_UNI;     /*!< Unicast TX frame count.                */
    uint64_t cEMAC_TX_CNT_MULTI;   /*!< Multicast TX frame count.              */
    uint64_t cEMAC_TX_CNT_BROAD;   /*!< Broadcast TX frame count.              */
    uint64_t cEMAC_TX_CNT_EXDEF;   /*!< TX frames with excessive deferral count.       */
    uint64_t cEMAC_TX_CNT_MACCTL;  /*!< MAC control TX frame count.            */
    uint64_t cEMAC_TX_CNT_ALLF;    /*!< Overall TX frame count.                */
    uint64_t cEMAC_TX_CNT_ALLO;    /*!< Overall TX octet count.                */
    uint64_t cEMAC_TX_CNT_EQ64;    /*!< Good TX frames,count x = 64.           */
    uint64_t cEMAC_TX_CNT_LT128;   /*!< Good TX frames,count x  64 <= x < 128. */
    uint64_t cEMAC_TX_CNT_LT256;   /*!< Good TX frames,count x 128 <= x < 256. */
    uint64_t cEMAC_TX_CNT_LT512;   /*!< Good TX frames,count x 256 <= x < 512. */
    uint64_t cEMAC_TX_CNT_LT1024;  /*!< Good TX frames,count x 512 <= x < 1024 */
    uint64_t cEMAC_TX_CNT_EQ1024;  /*!< Good TX frames,count x >= 1024.        */
    uint64_t cEMAC_TX_CNT_ABORT;   /*!< Total TX frames aborted count.         */
} ADI_ETHER_STATISTICS_COUNTS;



/*
 * \struct ADI_ETHER_MEM
 *
 * Ethernet driver initialization structure
 */
typedef struct ADI_ETHER_MEM
{
    uint8_t   *pRecvMem;       /*!< Driver memory used for rx operations.     */
    uint32_t   RecvMemLen;     /*!< Supplied memory in bytes for receive.     */
    uint8_t   *pTransmitMem;   /*!< Driver memory used for tx operations.     */
    uint32_t   TransmitMemLen; /*!< Supplied memory in bytes for transmit.    */
    uint8_t   *pBaseMem;       /*!< Driver base memory used for statistics.   */
    uint32_t   BaseMemLen;     /*!< Supplied base memory in bytes.            */

} ADI_ETHER_MEM;

/*
 * \struct ADI_ETHER_DEV_INIT
 *
 * Ethernet driver initialization structure
 */
typedef struct ADI_ETHER_DEV_INIT
{
    bool             Cache;        /*!< Supplied Ethernet Frame is cached or not */
    ADI_ETHER_MEM   *pEtherMemory; /*!< Supply memory to the driver              */

} ADI_ETHER_DEV_INIT;


/**
 * \struct driver entry point
 */
typedef struct ADI_ETHER_DRIVER_ENTRY
{
    ADI_ETHER_RESULT (*adi_ether_Open)( struct ADI_ETHER_DRIVER_ENTRY* const pEntryPoint,
                                        ADI_ETHER_DEV_INIT  *   const pDeviceInit,
                                        ADI_ETHER_CALLBACK_FN   const pfCallback,
                                        ADI_ETHER_HANDLE*       const phDevice );

    ADI_ETHER_RESULT (*adi_ether_Read)(ADI_ETHER_HANDLE phDevice,
                                       ADI_ETHER_BUFFER *pBuffer);

    ADI_ETHER_RESULT (*adi_ether_Write)(ADI_ETHER_HANDLE phDevice,
                                        ADI_ETHER_BUFFER *pBuffer);

    ADI_ETHER_RESULT (*adi_ether_Close)(ADI_ETHER_HANDLE phDevice);

    bool             (*adi_ether_GetLinkStatus)(ADI_ETHER_HANDLE pEtherHandle);

    ADI_ETHER_RESULT (*adi_ether_AddMulticastFilter)(ADI_ETHER_HANDLE pEtherHandle,
                                                     const uint32_t MultiCastGroupAddr);

    ADI_ETHER_RESULT (*adi_ether_DelMulticastFilter)(ADI_ETHER_HANDLE pEtherHandle,
                                                     const uint32_t MultiCastGroupAddr);

    ADI_ETHER_RESULT (*adi_ether_GetBufferPrefix)(ADI_ETHER_HANDLE pEtherHandle,
                                                  uint32_t* const  pBufferPrefix);

    ADI_ETHER_RESULT (*adi_ether_GetMACAddress)(ADI_ETHER_HANDLE pEtherHandle,
                                                uint8_t *pMacAddress);

    ADI_ETHER_RESULT (*adi_ether_SetMACAddress)(ADI_ETHER_HANDLE pEtherHandle,
                                                const uint8_t *pMacAddress);

    ADI_ETHER_RESULT (*adi_ether_EnableMAC)(ADI_ETHER_HANDLE phDevice);

} ADI_ETHER_DRIVER_ENTRY;

/* Opens the Ethernet Device Driver */
ADI_ETHER_RESULT adi_ether_Open(
                                ADI_ETHER_DRIVER_ENTRY* const pEntryPoint,
                                ADI_ETHER_DEV_INIT*     const pDeviceInit,
                                ADI_ETHER_CALLBACK_FN   const pfCallback,
                                ADI_ETHER_HANDLE*       const phDevice
                                );

/* Submits single or list of buffers for receiving ethernet packets */
ADI_ETHER_RESULT adi_ether_Read (
                                 ADI_ETHER_HANDLE const hDevice,
                                 ADI_ETHER_BUFFER *pBuffer
                                 );

/* Submits single or list of buffers for transmission over ethernet */
ADI_ETHER_RESULT adi_ether_Write (
                                  ADI_ETHER_HANDLE const hDevice,
                                  ADI_ETHER_BUFFER *pBuffer
                                  );

/* Close the ethernet device driver */
ADI_ETHER_RESULT adi_ether_Close(ADI_ETHER_HANDLE hDevice);


/* Return the Link Status */
bool adi_ether_GetLinkStatus(ADI_ETHER_HANDLE hDevice);

/* Configures and Enables MAC and PHY */
ADI_ETHER_RESULT adi_ether_EnableMAC(ADI_ETHER_HANDLE hDevice);


/* Add multicast group */
ADI_ETHER_RESULT adi_ether_AddMulticastFilter (
                                               ADI_ETHER_HANDLE hDevice,
                                               const uint32_t MultiCastGroupAddr
                                               );

/* Delete multicast group */
ADI_ETHER_RESULT adi_ether_DelMulticastFilter (
                                               ADI_ETHER_HANDLE hDevice,
                                               const uint32_t MultiCastGroupAddr
                                               );

/* Get the MAC address of the interface */
ADI_ETHER_RESULT adi_ether_GetMACAddress (
                                          ADI_ETHER_HANDLE hDevice,
                                          uint8_t *pMacAddress
                                          );

/* Set the MAC address of the interface */
ADI_ETHER_RESULT adi_ether_SetMACAddress (
                                          ADI_ETHER_HANDLE hDevice,
                                          const uint8_t *pMacAddress
                                          );

/* Get the Buffer Prefix */
ADI_ETHER_RESULT adi_ether_GetBufferPrefix (
                                            ADI_ETHER_HANDLE hDevice,
                                            uint32_t* const  pBufferPrefix
                                            );

#ifdef __cplusplus
}
#endif

/*@}*/


#ifdef _MISRA_RULES
#pragma diag(pop)
#endif

#endif  /* __ADI_ETHER_H__  */



