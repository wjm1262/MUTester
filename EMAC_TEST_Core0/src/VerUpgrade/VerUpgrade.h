/*
 * VerUpdate.h
 *
 *  Created on: 2014-8-27
 *      Author: ChiliWang
 */

#ifndef VERUPDATE_H_
#define VERUPDATE_H_
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define HEAPID_LDR    1   //define the ldr data buffer area in heap.

#define LDR_VER_NUMBER    100   //�汾���
#define HARDWARE_VER_NUMBER    20   //�汾���
#define PC_COMM_VER_NUMBER    112   //�汾���
#define FPGA_VER_NUMBER    10   //�汾���
#define FPGA_COMM_VER_NUMBER    10   //�汾���

/* �汾�����ı�  */
#define SET_VER_NUM()\
sprintf(VerDesc,"Software version           : V%d\n"\
				"Hardware version           : V%d\n"\
				"PC_BF609_Com_ver           : V%d\n"\
				"FPGA Software Version      : V%d\n"\
				"FPGA Communication Version : V%d\n",\
				LDR_VER_NUMBER,\
				HARDWARE_VER_NUMBER,\
				PC_COMM_VER_NUMBER,\
				FPGA_VER_NUMBER,\
				FPGA_COMM_VER_NUMBER)


extern char VerDesc[256];
extern uint8_t  *g_pLdrFileBuffer ;//Global parameter, pointer of ldr data buffer
extern uint32_t	g_LdrFileBufSize ;


/*
 * the ERROR Code for Negative Acknowledge (NAK)
 */
typedef enum VER_UPDATE_ACK_CODE
{
	//positive acknowledgement
	ACK_OK = 0X00,
	ACK_FRM_OK =0X01,

	//Negative Acknowledge (NAK)
	NAK_ERROR_MEM_ALLOC_FAILED = 0X02,
	NAK_ERROR_INDEX = 0X03,
	NAK_ERROR_FILE_CRC = 0X04,
	NAK_ERROR_UNKNOWN_COMMAND=0X05,
	NAK_ERROR_DESTADDR_UNMATCH=0X06,
}VER_UPDATE_ACK_CODE;

typedef struct
{
	VER_UPDATE_ACK_CODE code;
	char desc[128];
}VER_UPDATE_ACK_INFO;

extern VER_UPDATE_ACK_INFO VerAckInfo[7];



/*
 * Get the ldr file data.
 * the ldr file data saved in global parameter :
 * 				pGucUpdate.
 *

 */
VER_UPDATE_ACK_CODE GetLdrFileData(void *pLdrDataIn);


/*
 * Upgrade the program.
 * notes:
 * 		  before call this function, user must has ldr data saved in global parameter pGucUpdate.
 */
void VerUpgrade();

#endif /* VERUPDATE_H_ */
