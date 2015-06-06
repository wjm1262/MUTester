/*
 * comm_cmd_process.h
 *
 *  Created on: 2015-4-22
 *      Author: Wu JM
 */

#ifndef COMM_CMD_PROCESS_H_
#define COMM_CMD_PROCESS_H_

#include "comm_pc_protocol.h"

/*
功能：	PC通信口参数通信
参数：	recvData，从PC通信口接收到的数据；
		recvSize，recvData数组大小；

返回值：1，正确；0，不正确。返回1即将数据发送到PC通信口
*/
UINT8 Comm_processCmd(UINT8 *recvData,UINT16 recvSize );

#endif /* COMM_CMD_PROCESS_H_ */
