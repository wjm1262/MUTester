/*
 * dev_eth.c
 *
 *  Created on: 2015-2-6
 *      Author: Administrator
 */


#include "sys.h"
#include "dev_eth.h"

#include "post_debug.h"

#define EMAC0_NUM_RECV_DESC    (1200)  /*! Number of receive DMA descriptors  */
#define EMAC0_NUM_XMIT_DESC    (2)  //EMAC0 ONLY RECV THE PACKET/*! Number of transmit DMA descriptors */
#define EMAC1_NUM_RECV_DESC    (500)  /*! Number of receive DMA descriptors  */
#define EMAC1_NUM_XMIT_DESC    (1200)  /*! Number of transmit DMA descriptors */

////////////////////////////////



////////////////////////////////


/* Initialize the Memory Table */
#pragma alignment_region (32)
section ("sdram_bank0") static uint8_t BaseMemSize0[32];
section ("sdram_bank0") static uint8_t MemXmit0[EMAC0_NUM_XMIT_DESC * 32]; /*! Transmit DMA descriptor memory */
section ("sdram_bank0") static uint8_t MemRcve0[EMAC0_NUM_RECV_DESC * 32]; /*! Receive DMA descriptor memory  */

section ("sdram_bank0") static uint8_t BaseMemSize1[32];
section ("sdram_bank0") static uint8_t MemRcve1[EMAC1_NUM_RECV_DESC * 32]; /*! Receive DMA descriptor memory  */
section ("sdram_bank0") static uint8_t MemXmit1[EMAC1_NUM_XMIT_DESC * 32]; /*! Transmit DMA descriptor memory */
#pragma alignment_region_end

static ADI_ETHER_MEM memtable[MAX_NETWORK_IF] =
{
	{
		MemRcve0, sizeof ( MemRcve0 ),
		MemXmit0, sizeof ( MemXmit0 ),
		BaseMemSize0, sizeof ( BaseMemSize0 )
	},
	{
		MemRcve1, sizeof ( MemRcve1 ),
		MemXmit1, sizeof ( MemXmit1 ),
		BaseMemSize1, sizeof ( BaseMemSize1 )
	}
};

/////////////////////////////////

static void* OpenEth0(void)
{
	ADI_ETHER_HANDLE   hEthernet;
	ADI_ETHER_RESULT   etherResult;
	ADI_ETHER_DEV_INIT EtherInitData=  { true, &memtable[0] } ; // data-cache,driver memory

	etherResult = Ether_GemacOpen ( ETHER_EMAC0, &EtherInitData, &hEthernet );

	if ( etherResult != ADI_ETHER_RESULT_SUCCESS )
	{
		DEBUG_PRINT ( "adi_ether_Open: %d failed to open Ethernet driver\n\n",etherResult );
		return NULL;
	}

//	g_hDev[0] = hEthernet;
	return hEthernet;
}

static void* OpenEth1(void)
{
	ADI_ETHER_HANDLE   hEthernet;
	ADI_ETHER_RESULT   etherResult;
	ADI_ETHER_DEV_INIT EtherInitData =  { true, &memtable[1] }; // data-cache,driver memory

	etherResult = Ether_GemacOpen ( ETHER_EMAC1, &EtherInitData, &hEthernet );

	if ( etherResult != ADI_ETHER_RESULT_SUCCESS )
	{
		DEBUG_PRINT ( "adi_ether_Open: %d failed to open Ethernet driver\n\n",etherResult );
		return NULL;
	}

//	g_hDev[1] = hEthernet;
	return hEthernet;
}

static int ConfigureEth(void* hDev)
{
	ADI_ETHER_RESULT   etherResult;

	/* Enable the MAC */
	etherResult = Ether_GemacCfgMAC ( hDev );
	if ( etherResult != ADI_ETHER_RESULT_SUCCESS )
	{
		DEBUG_STATEMENT ( " adi_ether_EnableMAC: failed to enable EMAC\n\n" );
		return 0;
	}
	return 1;
}

static void SetCallbakFn(void* hDev, void* CallbakFn )
{
	set_callbak_fn(hDev, CallbakFn);
}

static void EnableRxTimeStamped(void* hDev)
{
	enable_rx_timestamped(hDev);
}

static void EnableTxTimeStamped(void* hDev)
{
	enable_tx_timestamped(hDev);
}

static void EnableAuxiTimeStamped(void* hDev)
{
	enable_auxi_timestamped(hDev);
}

static void EnableTimingTx(void* hDev)
{
	enable_timing_tx(hDev);
}

static void EnableGemacDMA(void* hDev, int nMode)
{

	if(nMode == TR)
	{
		enable_gemac_txrx_dma(hDev);
	}
	else if(nMode == TX)
	{
		enable_gemac_tx_dma(hDev);
	}
	else if (nMode == RX)
	{
		enable_gemac_rx_dma(hDev);
	}
}


static void EnableGemacInt(void* hDev)
{
	enable_gemac_int(hDev);
}

static void EnableGemacTx(void* hDev)
{
	enable_gemac_tx(hDev);
}

static void EnableGemacRx(void* hDev)
{
	enable_gemac_rx(hDev);
}

static void EnableGemacTxRx(void* hDev)
{
	enable_gemac_tx_rx(hDev);
}

//static int EtherSend ( ADI_ETHER_HANDLE  const hDevice, ADI_ETHER_BUFFER *tx_frame )
//{
//
//	if ( !hDevice || !tx_frame )
//	{
//		DEBUG_STATEMENT ( " EtherSend: Input Params IS NULL \n\n" );
//		return 0;
//	}
//
//	ADI_ETHER_RESULT eResult = adi_ether_GemacWrite ( hDevice, tx_frame );
//
//	if ( eResult != ADI_ETHER_RESULT_SUCCESS )
//	{
//		DEBUG_STATEMENT ( " EtherSend: adi_ether_Write failed \n\n" );
//		return 0;
//	}
//
//	return 1;
//}

static ADI_ETHER_BUFFER *EtherRecv ( ADI_ETHER_HANDLE  const hDevice )
{

	ADI_ETHER_BUFFER *pack = NULL;

	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) hDevice;

	//一次返回一个
	pack = pop_queue ( &(pDev->Rx.Completed) );

	return pack;

}

void RegisterEthnetModual(void)
{
	MuTesterSystem.Device.Eth0.Open 	= OpenEth0;
	MuTesterSystem.Device.Eth0.Close 	= adi_ether_GemacClose;
	MuTesterSystem.Device.Eth0.Read 	= adi_ether_GemacRead;
	MuTesterSystem.Device.Eth0.Write	= adi_ether_GemacWrite;
	MuTesterSystem.Device.Eth0.Recv 	= EtherRecv;

	MuTesterSystem.Device.Eth0.Configure 		= ConfigureEth;
	MuTesterSystem.Device.Eth0.SetCallbakFn 	= SetCallbakFn;

	MuTesterSystem.Device.Eth0.EnableRxTimeStamped 	= EnableRxTimeStamped;
	MuTesterSystem.Device.Eth0.EnableTxTimeStamped 	= EnableTxTimeStamped;
	MuTesterSystem.Device.Eth0.EnableAuxiTimeStamped 	= EnableAuxiTimeStamped;
	MuTesterSystem.Device.Eth0.EnableTimingTx 			= EnableTimingTx;

	MuTesterSystem.Device.Eth0.EnableGemacDMA 	= EnableGemacDMA;
	MuTesterSystem.Device.Eth0.EnableGemacInt 	= EnableGemacInt;
	MuTesterSystem.Device.Eth0.EnableGemacTx	= EnableGemacTx;
	MuTesterSystem.Device.Eth0.EnableGemacRx 	= EnableGemacRx;
	MuTesterSystem.Device.Eth0.EnableGemacTxRx 	= EnableGemacTxRx;


	MuTesterSystem.Device.Eth1.Open 	= OpenEth1;
	MuTesterSystem.Device.Eth1.Close 	= adi_ether_GemacClose;
	MuTesterSystem.Device.Eth1.Read 	= adi_ether_GemacRead;
	MuTesterSystem.Device.Eth1.Write	= adi_ether_GemacWrite;
	MuTesterSystem.Device.Eth1.Recv 	= EtherRecv;

	MuTesterSystem.Device.Eth1.Configure 		= ConfigureEth;
	MuTesterSystem.Device.Eth1.SetCallbakFn 	= SetCallbakFn;

	MuTesterSystem.Device.Eth1.EnableRxTimeStamped 	= EnableRxTimeStamped;
	MuTesterSystem.Device.Eth1.EnableTxTimeStamped 	= EnableTxTimeStamped;
	MuTesterSystem.Device.Eth1.EnableAuxiTimeStamped 	= EnableAuxiTimeStamped;
	MuTesterSystem.Device.Eth1.EnableTimingTx 			= EnableTimingTx;

	MuTesterSystem.Device.Eth1.EnableGemacDMA 	= EnableGemacDMA;
	MuTesterSystem.Device.Eth1.EnableGemacInt 	= EnableGemacInt;
	MuTesterSystem.Device.Eth1.EnableGemacTx	= EnableGemacTx;
	MuTesterSystem.Device.Eth1.EnableGemacRx 	= EnableGemacRx;
	MuTesterSystem.Device.Eth1.EnableGemacTxRx 	= EnableGemacTxRx;
}



