/*******************************************************************************
 *
 * Copyright(c) 2011 Analog Devices, Inc. All Rights Reserved.
 *
 * This software is proprietary and confidential.  By using this software you 
 * agree to the terms of the associated Analog Devices License Agreement.
 *
 ******************************************************************************/

/*!
* @file      adi_ether_emac.h
*
* @brief     EMAC ethernet driver header file
*
* @details
*            Ethernet device driver interface (EDDI) for EMAC defines all
*            the EMAC specific functions, structures and defines required
*            for the EMAC Interface.
*/

#ifndef __ADI_ETHER_EMAC_H_
#define __ADI_ETHER_EMAC_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <drivers/ethernet/adi_ether.h>
#include <adi_types.h>

/*==============  D E F I N E S  ===============*/

/* BF537 Memory Requirements */
#define ADI_ETHER_MEM_EMAC_BASE_SIZE     0x180
#define ADI_ETHER_MEM_EMAC_PER_RECV      0x3C
#define ADI_ETHER_MEM_EMAC_PER_XMIT      0x48

/*=============  D A T A    T Y P E S   =============*/

/* Trace Info Structure */
typedef struct adi_ether_emac_trace_info {
    void     *Mem;          /* memory area to be used to hold trace data */
    int32_t   LnthMem;      /* length of memory area */
    int32_t   FirstByte;    /* offset of first byte of each frame to be trace */
    int32_t   MaxBytes;     /* max. number of bytes of frame to be added to trace buffer */
} ADI_ETHER_EMAC_TRACE_INFO;


/* Trace Entry Structure */
typedef struct adi_ether_emac_trace_entry {
    uint16_t NoBytes;       /* no of bytes of frame data in */
    uint8_t  Dirn;          /* 'T' for transmit , 'R' for received */
    uint8_t  Seqn;          /* incrmeenting sequence number */
    uint8_t  Data[1];       /* data from the frame */
} ADI_ETHER_EMAC_TRACE_ENTRY;


/* Trace Data Structure */
typedef struct adi_ether_emac_trace_data {
    ADI_ETHER_EMAC_TRACE_ENTRY *BaseEntry;       /* start of the trace area */
    ADI_ETHER_EMAC_TRACE_ENTRY *EndOfData;       /* address imm. following end of buffer */
    ADI_ETHER_EMAC_TRACE_ENTRY *OldestEntry;     /* pointer to oldest entry */
    int32_t EntryLnth;                            /* length of each trace entry */
    int32_t NoOfEntries;                          /* no. of filled entries */
} ADI_ETHER_EMAC_TRACE_DATA;

/*=============  D A T A  =============*/

/* LAN83C185 Driver Entry Point */
extern ADI_ETHER_DRIVER_ENTRY EmacDriverEntry;

#ifdef __cplusplus
}
#endif

#endif   /* __ADI_ETHER_EMAC_H_ */
