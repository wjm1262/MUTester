/*
 * Task_goose.c
 *
 *  Created on: 2015-7-2
 *      Author: Wu JM
 */

#include "EMAC_TEST_Core0.h"

#include "task.h"
#include "sys.h"
#include <limits.h>
void InitTime(struct Goose_Object*pObj, long t0, long t1 , long t2, long t3 , long t4 )
{
	//��ʼ���������
	pObj->m_nTmInitVal[0] = t0;
	//��ʼ���ط����
	pObj->m_nTmInitVal[1] = t1;
	pObj->m_nTmInitVal[2] = (t2+t1);
	pObj->m_nTmInitVal[3] = (t3+t2+t1);
	pObj->m_nTmInitVal[4] = (t4+t3+t2+t1);
}

//��ʱ����TimeUnit������TimeUnitһ���Ƕ�ʱ�������ڡ�
void TimescaleNormalization(struct Goose_Object*pObj, long TimeUnit)
{

	int i  =0;
	for(i = 0; i < GOOSE_TIME_NUM; i++)
	{
		pObj->m_nTmInitVal[i] /= TimeUnit;
	}

	pObj->m_nTime[0] = pObj->m_nTmInitVal[0];
	pObj->m_nTime[1] = -1;
	pObj->m_nTime[2] = -1;
	pObj->m_nTime[3] = -1;
	pObj->m_nTime[4] = -1;
}

static void ResetTime(struct Goose_Object*pObj)
{

	int i  =0;
	for(i = 0; i < GOOSE_TIME_NUM; i++)
	{
		pObj->m_nTime[i] = pObj->m_nTmInitVal[i];
	}
}

static void IncreaseSqNum(struct Goose_Object*pObj)
{
	pObj->m_sqNum++;
}

static void IncreaseStNum(struct Goose_Object*pObj)
{
	pObj->m_stNum++;
	pObj->m_sqNum = 0;
}

static void UpdateSqNumPackage(struct Goose_Object*pObj )
{
	unsigned char ResBuf[16];

	int	nSize = sizeof(pObj->m_sqNum);

	ToBigEndian((unsigned char*)&pObj->m_sqNum, nSize, ResBuf);

	memcpy( pObj->m_pFrame + pObj->SqNumIndex, ResBuf, nSize);//val

}

static void UpdateStNumPackage(struct Goose_Object*pObj )
{
	unsigned char ResBuf[16];

	int	nSize = sizeof(pObj->m_stNum);

	ToBigEndian((unsigned char*)&pObj->m_stNum, nSize, ResBuf);
	memcpy( pObj->m_pFrame + pObj->StNumIndex, ResBuf, nSize); //val

}


void OnStValChangedEvent(struct Goose_Object*pObj)
{

	ResetTime(pObj);//���ظ�����4�Σ������ط�ʱ�䣬T1��T2��T3��T4

	IncreaseStNum(pObj);//StNum�ı䣬stNum++ sqNum = 0;

	UpdateSqNumPackage(pObj);
	UpdateStNumPackage(pObj);
	UpdateSubDataPackage(pObj);

	Send(pObj);//���Ƿ��ͱ�λ��
}


static void TimeEvent(struct Goose_Object*pObj)
{
	IncreaseSqNum(pObj);//SqNum����
	UpdateSqNumPackage(pObj);
	Send(pObj);//����
}

void OnMyTimer( struct Goose_Object*pObj )
{

	int i  =0;
	for(i = 0; i < GOOSE_TIME_NUM; i++)
	{
		pObj->m_nTime[i]--;

		if ( LONG_MIN == pObj->m_nTime[i] )
		{
			pObj->m_nTime[i] = -1;
		}
	}


	if ( 0 == pObj->m_nTime[0]  )
	{
		//T0������ʱ��
		TimeEvent(pObj);
		pObj->m_nTime[0] = pObj->m_nTmInitVal[0]; //reset T0
	}

	if ( 0 == pObj->m_nTime[4] )
	{
		//  t4 == 0
		TimeEvent(pObj);
		pObj->m_nTime[0] = pObj->m_nTmInitVal[0]; //reset T0
	}

	if ( 0 == pObj->m_nTime[1] || 0 == pObj->m_nTime[2] || 0 == pObj->m_nTime[3]  )
	{
		// t1,t2,t3 == 0
		TimeEvent(pObj);
	}

}




void UpdateSubDataPackage(struct Goose_Object*pObj )
{

	unsigned int unStartPos;

	unStartPos = m_unPackageBufSize-1;
	m_chPacket[unStartPos] = '\0';// ���Ľ�β

	unStartPos--; // ������m_chPacket�еĿ�ʼλ��

	unsigned int unOldPos = unStartPos;//��¼��ʼλ�ã��Լ��㱨�ĳ���

	//��� subData�� ����ֵ�����ⲿ�ֵı��볤�ȣ�m_unStartPos ���ص��Ǳ��Ŀ�ʼλ�õ�ǰһ��λ�ã����´α��ĵ�д��λ��
	m_unSubDataBinCodeLen = PacketSubData(m_chPacket, &unStartPos, &m_GooseSubData );

		//�����ݲ��ֱ��볤��
	assert( m_GooseBinaryData.subData_Length == m_unSubDataBinCodeLen );//����Ϊ��ʱ������ǰ������˵��subData�仯ǰ��ı��ı��볤�Ȳ�һ�£��Ǵ���ģ�����Ӧ�÷�����


}


////////////////////////////////////
void Goose_publish(void *pCBParam, uint32_t Event, void *pArg)
{
	struct Goose_Object*pObj;

	pObj->OnMyTimer(pObj);

//	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);
}
