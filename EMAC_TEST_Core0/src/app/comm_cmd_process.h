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
���ܣ�	PCͨ�ſڲ���ͨ��
������	recvData����PCͨ�ſڽ��յ������ݣ�
		recvSize��recvData�����С��

����ֵ��1����ȷ��0������ȷ������1�������ݷ��͵�PCͨ�ſ�
*/
UINT8 Comm_processCmd(UINT8 *recvData,UINT16 recvSize );

#endif /* COMM_CMD_PROCESS_H_ */
