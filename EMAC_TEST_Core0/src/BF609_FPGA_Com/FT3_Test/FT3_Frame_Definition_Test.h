/*
 * FT3_Frame_Definition.h
 *
 *  Created on: 2015-5-8
 *      Author: Administrator
 */

#ifndef FT3_FRAME_DEFINITION_H_
#define FT3_FRAME_DEFINITION_H_
#include <stdint.h>

#pragma pack(1)
/*
 * single phase data, 26-Bytes
 */
//1
typedef struct FT3_SinglePhase_data
{
	uint16_t StartCode;  //0x6405
	uint8_t   data1[16];//1
	uint16_t CRC1;

	uint16_t  Status;
	uint16_t  SmpCnt;
	uint16_t CRC2;
}FT3_SINGLE_PHASE_DATA;
/*
 * Three phase data, 34-Bytes
 */
//2
typedef struct FT3_ThreePhase_current_data
{
	uint16_t StartCode;  //0x6405

	uint8_t   data1[16];
	uint16_t CRC1;


	uint16_t  phaseData[3];
	uint16_t  Status1;
	uint16_t  Status2;
	uint16_t  SmpCnt;
	uint16_t  CRC2;

}FT3_THREE_PHASE_C_DATA;

//3
typedef struct FT3_ThreePhase_voltage_data
{
	uint16_t StartCode;  //0x6405
	uint8_t   data1[16];
	uint16_t CRC1;


	uint16_t  Status1;
	uint16_t  Status2;
	uint16_t  SmpCnt;
	uint16_t  CRC2;

}FT3_THREE_PHASE_V_DATA;



//4
typedef struct FT3_ThreePhase_voltageC_data
{
	uint16_t StartCode;  //0x6405
	uint8_t   data1[16];
	uint16_t CRC1;


	uint8_t   data2[16];
	uint16_t CRC2;

	uint16_t  phaseData;

	uint16_t  Status1;
	uint16_t  Status2;
	uint16_t  SmpCnt;

	uint16_t  CRC3;

}FT3_THREE_PHASE_VC_DATA;

//
typedef struct FT3_STD_ThreePhase_data
{
	uint16_t StartCode;  //0x6405

	uint16_t Len;//44

	uint8_t LNName;//2
	uint8_t DataSetName;//0x01;
	uint16_t LDName;
	uint16_t PhsA_c;
	uint16_t Neut_c;
	uint16_t PhsA_v;
	uint16_t Tdr;
	uint16_t xx;

	uint16_t CRC1;

	uint8_t   data2[16];
	uint16_t CRC2;

	uint16_t   DataChannel9[4];
	uint16_t  Status1;
	uint16_t  Status2;
	uint16_t  SmpCnt;
	uint16_t researved;

	uint16_t CRC3;

}FT3_STD_THREE_PHASE_DATA;

typedef struct FT3_GDW_ThreePhase_data
{
	uint16_t StartCode;  //0x6405

	uint16_t Len;//62
	uint8_t LNName;//2
	uint8_t DataSetName;//0xFE

	uint16_t LDName;
	uint16_t PhsA_c;
	uint16_t Neut_c;
	uint16_t PhsA_v;
	uint16_t Tdr;
	uint16_t SmpCnt;

	uint16_t CRC1;

	uint8_t   data2[16];
	uint16_t CRC2;

	uint8_t   data3[16];
	uint16_t CRC3;

	uint8_t   data4[16];
	uint16_t  CRC4;
}FT3_GDW_THREE_PHASE_DATA;
/*
 * Three phase data, 48-Bytes
 */


typedef struct FT3_Test_Data
{
	FT3_SINGLE_PHASE_DATA SinglePhaseData;
	FT3_THREE_PHASE_C_DATA  ThreePhaseCData;
	FT3_THREE_PHASE_V_DATA  ThreePhaseVData;
	FT3_THREE_PHASE_VC_DATA  ThreePhaseVCData;
	FT3_STD_THREE_PHASE_DATA STDPhaseVCData;
	FT3_GDW_THREE_PHASE_DATA  GDWData;
}FT3_TEST_DATA;

/*
 * FT3_Extend, 74
 */
typedef struct FT3_Extend_data
{
	uint16_t StartCode;  //0x6405

	uint8_t   data1[16];
	uint16_t  CRC1;

	uint8_t   data2[16];
	uint16_t  CRC2;

	uint8_t   data3[16];
	uint16_t  CRC3;

	uint8_t   data4[16];
	uint16_t  CRC4;

}FT3_EXTEND_DATA;

typedef struct EX_FT3_TEST_DATA
{
	FT3_EXTEND_DATA FT3_Extend_Data_sa[6];
}EX_FT3_TEST_DATA;

#pragma pack()
#endif /* FT3_FRAME_DEFINITION_H_ */
