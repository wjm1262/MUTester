/*
 * task_ad.c
 *
 *  Created on: 2015-3-17
 *      Author: Wu JM
 */


#include "EMAC_TEST_Core0.h"

#include "task.h"
#include "sys.h"

#include "myapp_cfg.h"

#include "comm_pc_protocol.h"

#include "msg.h"


//FT3
#include "FT3_Frame_Definition_Test.h"
#include "FT3_Test.h"
#include "post_debug.h"


#define _TEST_ft3_send_in_sport 1


TASK_AD_PARA g_TaskADPara ={NULL, 0};


///////////////////
static int ADData2StandardSmpData(STAND_SAMP_TYPE* OutStandardSmpData,
		void *pAD_Value,
		uint8_t AD_ByteLength,
		uint16_t SmpCnt)
{

	STAND_SAMP_TYPE  *pPktAD = OutStandardSmpData;

	AD7608_DATA*pAD7608_Data = (AD7608_DATA*)pAD_Value;

	if( !pPktAD || !pAD7608_Data)
	{
		return 0;
	}

	pPktAD->stateLabel		= 0;		//状态标识
 	pPktAD->sampCnt			= SmpCnt;			//采样序号
//	pPktAD->sampTMark		=;		//采样延时
//	pPktAD->netSendTMark	=;	//点对点发送延时
//	pPktAD->FT3SendTMark	=;	//FT3发送延时


	pPktAD->UaSampData   = (pAD7608_Data[0].ad *5000./0x1ffff);
	pPktAD->UbSampData   = (pAD7608_Data[1].ad *5000./0x1ffff);
	pPktAD->UcSampData   = (pAD7608_Data[2].ad *5000./0x1ffff);

	pPktAD->IaSampData   = (pAD7608_Data[3].ad *5000./0x1ffff);
	pPktAD->IbSampData   = (pAD7608_Data[4].ad *5000./0x1ffff);
	pPktAD->IbSampData   = (pAD7608_Data[5].ad *5000./0x1ffff);

	return 1;
}

int OutputStandardADFrm( const STAND_SAMP_TYPE*  pStandSmpData)
{
	ADI_ETHER_BUFFER *pNewBuffer = NULL;
	ADI_ETHER_BUFFER *pBuf = NULL;
	uint8_t* pADDataBuf = NULL;

	int nFrmLen = 0;


	/* user process the AD data*/
	pBuf = MuTesterSystem.Device.exEth.PopProcessedElem( &g_ExEthXmtQueueAD, 1 );
	if(!pBuf)
	{
		DEBUG_PRINT("%s[#%d]:exEth.PopProcessedElem failed...\n\n", __FILE__, __LINE__);
		return 0;
	}

	pADDataBuf = (uint8_t*)pBuf->Data + 2
					+ sizeof(MUTestMsgHeader)
					+ sizeof(STAND_SAMP_HEAD_TYPE);


	//注意：在这里，StandSmpData填充到FRM里面没有大小端转换
	memcpy(pADDataBuf, pStandSmpData, sizeof(STAND_SAMP_TYPE));
	nFrmLen = msgPackStandADFrm( (uint8_t*)pBuf->Data + 2, 1);

	*( unsigned short * ) pBuf->Data = nFrmLen;
	pBuf->ElementCount  = nFrmLen + 2;
	pBuf->PayLoad =  0; 	// payload is part of the packet
	pBuf->StatusWord = 0; 	// changes from 0 to the status info

	//send by exEth
	MuTesterSystem.Device.exEth.PushUnprocessElem(&g_ExEthXmtQueueAD, pBuf);

	return 1;
}

int OutputStandardSmvFrm(const STAND_SAMP_TYPE*  pStandSmpData )
{
	ADI_ETHER_BUFFER *pBuf = NULL;
	int nFrmLen = 0;
	uint8_t PortNo = 0;


	/* user process the AD data*/
	if( g_rtParams.U8Parameter[U8PARA_NET_SEND1] )
	{
		//send by eth0
		pBuf = pop_queue( &user_net_config_info[0].xmt_buffers_queue );
		PortNo = 1;
	}
	else if( g_rtParams.U8Parameter[U8PARA_NET_SEND2]  )
	{
		//send by eth1
		pBuf = pop_queue( &user_net_config_info[1].xmt_buffers_queue );
		PortNo = 2;
	}
	else
	{
		DEBUG_PRINT("%s[#%d]:bad U8 Parameter setting for no eth send port...\n\n", __FILE__, __LINE__);
		return 0;
	}

	if(!pBuf)
	{
		//ERROR
		DEBUG_PRINT("%s[#%d]:pop_queue failed...\n\n", __FILE__, __LINE__);
		return 0;
	}

	nFrmLen = PackSmvFrm( (uint8_t*)pBuf->Data + 2,
								pStandSmpData,
								PortNo );

	*( unsigned short * ) pBuf->Data = nFrmLen;
	pBuf->ElementCount  = nFrmLen + 2;
	pBuf->PayLoad =  0; 	// payload is part of the packet
	pBuf->StatusWord = 0; 	// changes from 0 to the status info

	if(1 == PortNo )
	{
		//send by eth0
		MuTesterSystem.Device.Eth0.Write ( g_hEthDev[0], pBuf);
	}
	else if(2 == PortNo )
	{
		//send by eth1
		MuTesterSystem.Device.Eth1.Write ( g_hEthDev[1], pBuf);
	}
	else
	{
		DEBUG_PRINT("%s[#%d]:bad U8 Parameter setting for no eth send port...\n\n", __FILE__, __LINE__);

		return 0;
	}

	return 1;

}





/*
 * net to 16
 */
static inline uint16_t myHtons(uint16_t netShort)
{
	uint16_t retValue;
	*(((uint8_t*)&retValue)+0) = (uint8_t)(netShort >> 8);
	*(((uint8_t*)&retValue)+1) = (uint8_t)(netShort);
	return retValue;
}


static int ADData2Ft3_EX(uint8_t* OutBuf, const STAND_SAMP_TYPE* pStandSmpData)
{

	uint16_t crc;
	uint16_t len = 0;
	FT3_TEST_DATA *pBF609_FPGA_FT3 = (FT3_TEST_DATA*)OutBuf;

	len = sizeof(FT3_TEST_DATA) ;
	memset(OutBuf, 0xee, len);


	/* set the FPGA trigger pin in low */
	adi_gpio_Clear(ADI_GPIO_PORT_E,ADI_GPIO_PIN_2);

#if 1

	uint16_t tmp_cnt = myHtons(pStandSmpData->sampCnt);
	/*
	 *
	 *  single phase FT3 data
	 *
	 */
	pBF609_FPGA_FT3->SinglePhaseData.StartCode = 0x6405;

	pBF609_FPGA_FT3->SinglePhaseData.SmpCnt    = tmp_cnt;
	pBF609_FPGA_FT3->SinglePhaseData.data1[0] = 1;
	pBF609_FPGA_FT3->SinglePhaseData.CRC1     = 1;
	pBF609_FPGA_FT3->SinglePhaseData.CRC2     = 1;
//	/* cal 16 bytes crc*/
//	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->SinglePhaseData.data1), 16);
//	pBF609_FPGA_FT3->SinglePhaseData.CRC1      = myHtons(crc);
//
//	/* cal 4 bytes crc */
//	crc = Cal_CRC16_ByByte( &(pBF609_FPGA_FT3->SinglePhaseData.Status), 4);
//	pBF609_FPGA_FT3->SinglePhaseData.CRC2      = myHtons(crc);


	//c2
	pBF609_FPGA_FT3->ThreePhaseCData.StartCode = 0x6405;
	pBF609_FPGA_FT3->ThreePhaseCData.SmpCnt = tmp_cnt;
	pBF609_FPGA_FT3->ThreePhaseCData.data1[0] = 2;

	pBF609_FPGA_FT3->ThreePhaseCData.CRC1      = 2;
	pBF609_FPGA_FT3->ThreePhaseCData.CRC2      = 2;

//	/* cal 16 bytes crc*/
//	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseCData.data1), 16);
//	pBF609_FPGA_FT3->ThreePhaseCData.CRC1      = myHtons(crc);
//
//	/* cal 12 bytes crc */
//	crc = Cal_CRC16_ByByte( &(pBF609_FPGA_FT3->ThreePhaseCData.phaseData), 12);
//	pBF609_FPGA_FT3->SinglePhaseData.CRC2      = myHtons(crc);

	//c3
	pBF609_FPGA_FT3->ThreePhaseVData.StartCode = 0x6405;
	pBF609_FPGA_FT3->ThreePhaseVData.SmpCnt    = tmp_cnt;
	pBF609_FPGA_FT3->ThreePhaseVData.data1[0] = 3;

	pBF609_FPGA_FT3->ThreePhaseVData.CRC1      = 3;
	pBF609_FPGA_FT3->ThreePhaseVData.CRC2      = 3;

//	/* cal 16 bytes crc*/
//	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseVData.data1), 16);
//	pBF609_FPGA_FT3->ThreePhaseVData.CRC1      = myHtons(crc);
//
//	/* cal 6 bytes crc */
//	crc = Cal_CRC16_ByByte(&(pBF609_FPGA_FT3->ThreePhaseVData.Status1), 6);
//	pBF609_FPGA_FT3->SinglePhaseData.CRC2      = myHtons(crc);

	//c4
	pBF609_FPGA_FT3->ThreePhaseVCData.StartCode = 0x6405;
	pBF609_FPGA_FT3->ThreePhaseVCData.SmpCnt = tmp_cnt;
	pBF609_FPGA_FT3->ThreePhaseVCData.data1[0] = 4;

	pBF609_FPGA_FT3->ThreePhaseVCData.CRC1 = 4;
	pBF609_FPGA_FT3->ThreePhaseVCData.CRC2 = 4;
	pBF609_FPGA_FT3->ThreePhaseVCData.CRC3 = 4;

	//c5
	pBF609_FPGA_FT3->STDPhaseVCData.StartCode = 0x6405;
	pBF609_FPGA_FT3->STDPhaseVCData.SmpCnt = tmp_cnt;
	pBF609_FPGA_FT3->STDPhaseVCData.Len = myHtons( 44);
	pBF609_FPGA_FT3->STDPhaseVCData.LNName = 2;
	pBF609_FPGA_FT3->STDPhaseVCData.DataSetName = 0x01;

	pBF609_FPGA_FT3->STDPhaseVCData.CRC1 = 5;
	pBF609_FPGA_FT3->STDPhaseVCData.CRC2 = 5;
	pBF609_FPGA_FT3->STDPhaseVCData.CRC3 = 5;

	//c6 v
	pBF609_FPGA_FT3->GDWData.StartCode = 0x6405;
	pBF609_FPGA_FT3->GDWData.SmpCnt = tmp_cnt;
	pBF609_FPGA_FT3->GDWData.Len = myHtons( 62);
	pBF609_FPGA_FT3->GDWData.LNName = 2;
	pBF609_FPGA_FT3->GDWData.DataSetName = 0xFE;


	/* cal 16 bytes crc*/
	crc = Cal_CRC16_ByByte( &(pBF609_FPGA_FT3->GDWData.Len), 16);
	pBF609_FPGA_FT3->GDWData.CRC1      = myHtons(crc);

	crc = Cal_CRC16_ByByte( (pBF609_FPGA_FT3->GDWData.data2), 16);
	pBF609_FPGA_FT3->GDWData.CRC2      = myHtons(crc);

	crc = Cal_CRC16_ByByte( (pBF609_FPGA_FT3->GDWData.data3), 16);
	pBF609_FPGA_FT3->GDWData.CRC3      = myHtons(crc);

	crc = Cal_CRC16_ByByte( (pBF609_FPGA_FT3->GDWData.data4), 16);
	pBF609_FPGA_FT3->GDWData.CRC4      = myHtons(crc);




#if _TEST_ft3_send_in_sport

	Send_FT3_Data(pBF609_FPGA_FT3, sizeof(FT3_TEST_DATA));
	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);
#endif


	return sizeof(FT3_TEST_DATA);

#else   /* 12.8k sample rate */
	SmpCnt = (SmpCnt + 1) % 12800;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].data1[15] = SmpCnt;


	crc = Cal_CRC16_ByByte(pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data1, 16);
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC1      = crc;


	crc = Cal_CRC16_ByByte(pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data2, 16);
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC2      = crc;

	crc = Cal_CRC16_ByByte(pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data3, 16);
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC3      = crc;

	crc = Cal_CRC16_ByByte(pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data4, 16);
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC4      = crc;
//	Send_FT3_Data(pBF609_FPGA_FT3_ex, sizeof(EX_FT3_TEST_DATA));
//	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

#endif

}

int CreateFT3Frm( const STAND_SAMP_TYPE*  pStandSmpData)
{
	//get buffer
	Ft3FrmItem* pFrmItem = PushFt3FrmQueue(&g_Ft3FrmQueue );
	if(!pFrmItem)
	{
//		DEBUG_PRINT("%s[#%d]:PushFt3FrmQueue failed...\n\n", __FILE__, __LINE__);
		return 0;
	}

	pFrmItem->FrmLen = ADData2Ft3_EX(pFrmItem->Ft3FrmData, pStandSmpData);

//	pFrmItem->FrmLen = PackFT3Frm(pFrmItem->Ft3FrmData, pStandSmpData);

	return 1;
}

int OutputFT3Frm(void)
{
	Ft3FrmItem* pFrmItem = PopFt3FrmQueue(&g_Ft3FrmQueue );
	if(!pFrmItem)
	{
//		DEBUG_PRINT("%s[#%d]:PopFt3FrmQueue failed...\n\n", __FILE__, __LINE__);
		return 0;
	}

	/* set the FPGA trigger pin in low */
	adi_gpio_Clear(ADI_GPIO_PORT_E,ADI_GPIO_PIN_2);

	Send_FT3_Data(pFrmItem->Ft3FrmData, pFrmItem->FrmLen );

	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

	return 1;
}

//StandardSmpData convert to SV and FT3
int StandardSmpDataFormatConverter()
{
	STAND_SAMP_TYPE* pStandSmpData = PopQueue( &g_StandardSmpDataQueue );
	if(!pStandSmpData)
	{
//		DEBUG_PRINT("%s[#%d]:PopFt3FrmQueue failed...\n\n", __FILE__, __LINE__);
		return 0;
	}

	//FT3组帧发送
	if( (g_rtParams.U8Parameter[U8PARA_FT3_SEND1])
		   || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND2] )
		   || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND3] )
		   || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND4] )
		   || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND5] )
		   || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND6] )
		   )
	{
		CreateFT3Frm( pStandSmpData);
		OutputFT3Frm();
	}

	//标准数据组帧发送
	OutputStandardADFrm( pStandSmpData );

	//9-2组帧发送
	if( (g_rtParams.U8Parameter[U8PARA_NET_SEND1])
			|| ( g_rtParams.U8Parameter[U8PARA_NET_SEND2] ) )
	{
		OutputStandardSmvFrm(pStandSmpData );
	}


	return 1;
}

//////////////////////////
static void AD7608_Busy_ISR(ADI_GPIO_PIN_INTERRUPT const ePinInt, const uint32_t event, void *pArg)
{
#if 1
	SubmitBuffer_SPORT1B();

	/* Enable the SPORT2-B Rx  D0 and DMA */
	Enable_SPORT1B();
#endif
}



static void SPORTCallbackRx2(
    void        *pAppHandle,
    uint32_t     nEvent,
    void        *pArg)
{

    ADI_SPORT_RESULT eResult;

    AD7608_DATA *pAD7608_Data = pArg;
    STAND_SAMP_TYPE StandardSmpData;
    STAND_SAMP_TYPE* pStandardSmpData, *pOutStandardSmpData;
   /*
	 * Disable the SPORT1-B, stop the SPORT process
	 */
    Disable_SPORT1B();

	/*
	 * Avoiding data offset, we should cancel the SPORT1-B-CLK, set clk in high default.
	 */

    CancelClk_SPORT1B();


    /* CASEOF (event type) */
    switch (nEvent)
    {
        /* CASE (buffer processed) */
        case (uint32_t)ADI_SPORT_EVENT_RX_BUFFER_PROCESSED:
        	/* user process the AD data*/

//        	pStandardSmpData = PushQueue( &g_StandardSmpDataQueue );
//        	if(!pStandardSmpData)
//        	{
//        		DEBUG_STATEMENT("in SPORTCallbackRx2: PushQueue failed!\n\n");
//        		return;
//        	}

        	ADData2StandardSmpData(&StandardSmpData,
									pArg,
									32,
									g_TaskADPara.usSmpCnt);

            //

//        	pOutStandardSmpData = PopQueue( &g_StandardSmpDataQueue );

  			//标准数据组帧发送
			OutputStandardADFrm( &StandardSmpData );

			//9-2组帧发送
			if( (g_rtParams.U8Parameter[U8PARA_NET_SEND1])
					|| ( g_rtParams.U8Parameter[U8PARA_NET_SEND2] ) )
			{
				OutputStandardSmvFrm(&StandardSmpData);
			}

			//FT3组帧发送
			if( (g_rtParams.U8Parameter[U8PARA_FT3_SEND1])
				   || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND2] )
				   || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND3] )
				   || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND4] )
				   || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND5] )
				   || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND6] )
				   )
			{
#if _TEST_ft3_send_in_sport
				//ADData2Ft3( (uint8_t*)g_pBF609_FPGA_FT3, pArg, 32, g_TaskADPara.usSmpCnt);
				ADData2Ft3_EX( (uint8_t*)g_pBF609_FPGA_FT3, &StandardSmpData);
#else
				CreateFT3Frm( &StandardSmpData);
#endif
			}

			g_TaskADPara.usSmpCnt = (g_TaskADPara.usSmpCnt + 1)%4000;

            break;
        default:
            break;
    }
    /* return */
}



///////////////////////////////////////////

void Task_AD7608( void* p_arg )
{
//	OS_ERR osErr;

	g_TaskADPara.pStandardADFrmSendBuf = NULL;

	g_TaskADPara.usSmpCnt = 0;


	MuTesterSystem.Device.AD7608.InitADDevice();

	MuTesterSystem.Device.AD7608.RegisterBuzyIOCallback( AD7608_Busy_ISR );


	MuTesterSystem.Device.AD7608.RegisterSportCallback( SPORTCallbackRx2 );

	MuTesterSystem.Device.AD7608.EnableBuzyIOInterrupt( true );


//	OSTaskDel((OS_TCB*)0, &osErr);

}




