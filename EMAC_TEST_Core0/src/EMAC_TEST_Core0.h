/*****************************************************************************
 * EMAC_TEST_Core0.h
 *****************************************************************************/

#ifndef __EMAC_TEST_CORE0_H__
#define __EMAC_TEST_CORE0_H__

/* Add your custom header content here */

/***************************************************************
 *
 * ADSP-BF60x family configuration
 */
#if defined( __ADSPBF60x__)
#include <adi_osal.h>
#include <drivers/ethernet/adi_ether_gemac.h>
#include <services/int/adi_int.h>
#include <services/int/adi_sec.h>

#if defined(__ADSPBF609__)
#include <cdefbf609.h>
#include <defbf609.h>
#elif defined(__ADSPBF608__)
#include <cdefbf608.h>
#include <defbf608.h>
#elif defined(__ADSPBF607__)
#include <cdefbf607.h>
#include <defbf607.h>
#elif defined(__ADSPBF606__)
#include <cdefbf606.h>
#include <defbf606.h>
#endif


#endif /* __ADSPBF60x__ */
/***********************************************************/
#if USE_OS

#include "os.h"

extern OS_TCB  Eth0_Rx_TaskTCB;

extern OS_TCB  Eth0_Tx_TaskTCB;

extern OS_TCB  Eth1_Tx_TaskTCB;

extern OS_TCB  Eth1_Rx_TaskTCB;

#endif

/////
extern char VersionString[128] ;
extern char VerDescripString[64];

#endif /* __EMAC_TEST_CORE0_H__ */
