/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved.
 *
 * File			: MldNet.c
 * Author		: King
 * Description	: ���ļ���Ҫ�������󲿷֣�
	1. Ӧ�ò�(Mqtt��ModbusTCPͨѶ)����ں���: NetOpenHook()
	2. TCP/IPͨѶ�����빤��(����DHCP��DNS��)��ժ��TI�Ŀ��ļ�����ں���: NetCommTask(), DNSResolve()

 * Date			: 2017-08-30
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|				|		 |
***************************************************************************/

/***************************************************************************
 						macro definition
***************************************************************************/
#define _MDL_NET_C_		/* exclude redefinition */

/***************************************************************************
 						include files
***************************************************************************/
 /* ����ϵͳͷ�ļ� */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* ��Ŀ����ͷ�ļ� */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"
#include "BoardSupport.h"

/* �����Լ��¼�ģ��ͷ�ļ� */
#include "MdlNet.h"
#include "MdlDataAccess.h"
#include "MdlGUI.h"
//#include "MdlUARTnModbus.h"

#if SUPPORT_V4_MASTER
#include "MdlV4Master.h"
#endif
#if SUPPORT_FLOW_METER
#include "MdlFlow.h"
#endif
#if SUPPORT_CAMERA
#include "MdlCamera.h"
#endif
#if (SUPPORT_IEC104 && (!SUPPORT_W5500))
#include "Mdl101n104.h"
#endif
#if SUPPORT_GPS
#include "MdlExtUartApp.h"
#endif

/* MQTT header files */
#include "libemqtt.h"

#if SUPPORT_GPRS
#include "MdlGprs.h"
#endif


#define SOCKET_RCV_TIMEOUT_s		10				/* socket��ȡ��ʱ */
#define SOCKET_SND_TIMEOUT_s		5				/* socket���ͳ�ʱ */
#define MAX_MQTT_SERVER_NUM			3				/* Pub:s1~s3, Sub:s4~s6 */
#define GPRS_MAX_SEND_LEN			0				/* GPRS���ͻ�������󳤶�, �����ͻ������ĳ��ȴ�������Ͳ�Ҫ�����¶����� */
/***************************************************************************
 						global variables definition
***************************************************************************/
#define MQTT_CONN_FAIL_DEAL		NOP

/***************************************************************************
						internal functions declaration
***************************************************************************/
void SyncRealTimeBySNTP(MQTT_COMM* pMqttComm);

/***************************************************************************
 							functions definition
***************************************************************************/


MQTT_COMM g_MqttComm[MQTT_TASK_NUM];


void DrvNetTick_1KHz(void)
{
	if(g_TcpIPComm.uTmr_MqttPubRun_ms) {
		g_TcpIPComm.uTmr_MqttPubRun_ms--;
	}
}

/*==========================================================================
\=========================================================================*/
//extern void MqttPubTask(void* argument);
//extern void MqttSubTask(void const * argument);
/*==========================================================================
| Description	: ��MqttSocket��������֮
| G/Out var 	:
| Author		:			Date	:
\=========================================================================*/
int MqttSendPacket(int32 i32Socket, const void* buf, unsigned int count);
BOOL RecvFromMqttServer(MQTT_COMM* pMqttComm);
BOOL PubDevInfo(uint8 u8MqttTaskNo);
void PrintMqttClientId(uint8** ppTxBuf, uint8 u8MqttTaskNo);
BOOL OpenMqttSocketAndConnMqttServer(uint8 u8MqttTaskNo)
{
	MQTT_COMM* pMqttComm = &g_MqttComm[u8MqttTaskNo];
	uint8* pTxBuf = pMqttComm->u8TRBuf;
	pMqttComm->GprsNewAdd.bSentPingFlag = FALSE;
	/* ������DNS����:  �� MqttͨѶ��TRBuf��ΪDNS������ʱʹ�õ�ram��һ��DNS������ʱԼ12~16ms,������������ڴ������700B */
	if(!CheckDebugVersion()) {
		PrintStringNoOvChk(&pTxBuf, "s0.mqtt.dev.yoplore.com");
	} else {
		*pTxBuf++ = 's';	/* ����������ˣ��Է�ֹ��ȡBIN�ļ����޸ķ���������ָ�� */
		*pTxBuf++ = '0';
		*pTxBuf++ = '.';
		*pTxBuf++ = 'm';
		*pTxBuf++ = 'q';
		*pTxBuf++ = 't';
		*pTxBuf++ = 't';
		*pTxBuf++ = '.';
		*pTxBuf++ = 'y';
		*pTxBuf++ = 'o';
		*pTxBuf++ = 'p';
		*pTxBuf++ = 'l';
		*pTxBuf++ = 'o';
		*pTxBuf++ = 'r';
		*pTxBuf++ = 'e';
		*pTxBuf++ = '.';
		*pTxBuf++ = 'c';
		*pTxBuf++ = 'o';
		*pTxBuf++ = 'm';
	}
	*pTxBuf++ = 0;
	int16 i, j;
	uint8 u8DnsAndTcpConRes = 0;	/* b5~4,b3~2,b1~0:����������dns+tcp�������--0δ����,1dnsʧ��,2dns�ɹ�tcpʧ��,3tcp�ɹ� */
	SocketFunction *pSocketFunc = NULL;
	for(j = 0; j < MAX_INTERNET_ACCESS_NUM; j++) {
		pSocketFunc = &AllSocketFun[j];
		uint8 u8DnsAndTcpConRes = 0;	/* b5~4,b3~2,b1~0:����������dns+tcp�������--0δ����,1dnsʧ��,2dns�ɹ�tcpʧ��,3tcp�ɹ� */
		for(i = 3; i > 0; i--) {
			pMqttComm->u8ServerNo++;
			if(pMqttComm->u8ServerNo > MAX_MQTT_SERVER_NUM) {
				pMqttComm->u8ServerNo = 1;
			}
			pMqttComm->u8TRBuf[1] = '0' + pMqttComm->u8ServerNo + u8MqttTaskNo*MAX_MQTT_SERVER_NUM;

			uint32 u32IpResolved = 0;

			if(!pSocketFunc->DnsQuery(pMqttComm->u8TRBuf, &u32IpResolved)) {
				u8DnsAndTcpConRes |= (1<<((pMqttComm->u8ServerNo-1)*2));		/* DNSʧ�� */
			} else {
				/* �򿪲�����MQTT��socket */
				if(((int32)(pMqttComm->MqttSocket = pSocketFunc->NewSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP))) <= 0) {
					pMqttComm->u8ConnRes = 1;
					return FALSE;
				}
				/* �ɸ�������㷨 */
				int32 flag = 1;			/* ��������Nagle�㷨: flag = 1������ѡ�0Ϊ�رո�ѡ�� */
				if(pSocketFunc->SetSockopt(pMqttComm->MqttSocket, IPPROTO_TCP, TCP_NODELAY, (int8*)&flag, sizeof(flag)) < 0) {
					pSocketFunc->Close(pMqttComm->MqttSocket);
					pMqttComm->u8ConnRes = 2;
					return FALSE;
				}
				/* ���ý��ճ�ʱ */
				struct timeval timeout;
				timeout.tv_sec = SOCKET_RCV_TIMEOUT_s*OS_TICK_KHz;
				timeout.tv_usec = 0*OS_TICK_KHz;
				if(pSocketFunc->SetSockopt(pMqttComm->MqttSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
					pSocketFunc->Close(pMqttComm->MqttSocket);
					pMqttComm->u8ConnRes = 3;
					return FALSE;
				}
				/* ���÷��ͳ�ʱ */
				timeout.tv_sec = SOCKET_SND_TIMEOUT_s*OS_TICK_KHz;
				timeout.tv_usec = 0*OS_TICK_KHz;
				if(pSocketFunc->SetSockopt(pMqttComm->MqttSocket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0) {
					pSocketFunc->Close(pMqttComm->MqttSocket);
					pMqttComm->u8ConnRes = 11;
					return FALSE;
				}
				/* ����socket */
				struct sockaddr_in socket_address;
				socket_address.sin_addr.s_addr = u32IpResolved;		//TODO: ETHERNET:((HOSTENT*)(&pMqttComm->u8TRBuf[DNS_DOMAIN_MAX_LEN]))->h_addr[0]
				socket_address.sin_family = AF_INET;
				socket_address.sin_port = htons(1883);		/* �̶��˿� */
				if(pSocketFunc->Connect(pMqttComm->MqttSocket, (struct sockaddr*)&socket_address, sizeof(socket_address)) < 0) {	/* TCP����ʧ�� */
					u8DnsAndTcpConRes |= (2<<((pMqttComm->u8ServerNo-1)*2));
					pSocketFunc->Close(pMqttComm->MqttSocket);
				} else {
					u8DnsAndTcpConRes |= (3<<((pMqttComm->u8ServerNo-1)*2));
					break;
				}
			}
			Task_sleep(OS_TICK_KHz*1000);
		}
	}

	pMqttComm->u8DnsAndTcpConRes = u8DnsAndTcpConRes;
	if(i == 0) {
		pMqttComm->u8ConnRes = 4;
		return FALSE;
	}

	/* ����Mqtt������, socket */
	mqtt_broker_handle_t *pBroker = &pMqttComm->broker;
	pTxBuf = pMqttComm->u8TRBuf;
	PrintMqttClientId(&pTxBuf, u8MqttTaskNo);
	*pTxBuf++ = 0;
	mqtt_init(pBroker, (const char*)pMqttComm->u8TRBuf);						/* ϵͳ������mqtt��ʼ�� */
	mqtt_init_auth(pBroker, "RwRVZXjkUO", "EGsWBH303D");						/* ϵͳ��������ʼ����¼�û��������� */
	mqtt_set_alive(pBroker, 60);												/* ���MQTT�ı���ʱ��:30�� */
	pBroker->socket_info = pMqttComm->MqttSocket;								/* ���MQTT���׽��� */
	pBroker->pSocketFunc = pSocketFunc;

	/* Mqtt���ӣ������� "MQTT_CONNECT", �������ӣ����ȴ���Ӧ */
	if(mqtt_connect(pBroker) != 1) {
		pMqttComm->u8ConnRes = 5;
	} else if(!GPRS_WaitSocketIdle(pMqttComm->MqttSocket, 0, 20, 10000)) {	/* �ȴ��ϸ�������ȫ���� */
		pMqttComm->u8ConnRes = 11;
	} else if(!RecvFromMqttServer(pMqttComm)) {
		pMqttComm->u8ConnRes = 6;
	} else if(MQTTParseMessageType(pMqttComm->u8TRBuf) != MQTT_MSG_CONNACK) {
		pMqttComm->u8ConnRes = 7;
	} else if(pMqttComm->u8TRBuf[3] != 0x00) {		/* CONNACK�ظ��ɹ� */
		pMqttComm->u8ConnRes = 8;
	} else {
#if 0
//		/* ����ID�����ں����Topic */
//	#if((DEVICE_TYPE == V5_YYB) || (DEVICE_TYPE == V5_YYD) || (DEVICE_TYPE == V5_YYG))
//		pBroker->clientid[3] = '/';	/* ���ӵ�ʱ���ƺ��ᱻ�ĳ�0 (�ޣ��鿴���벢δ���ִ�����)*/
//	#else
//		pBroker->clientid[4] = '/';	/* ���ӵ�ʱ���ƺ��ᱻ�ĳ�0 (�ޣ��鿴���벢δ���ִ�����)*/
//	#endif
#else
		*(SkipCharInString((uint8*)pBroker->clientid, (uint8*)pBroker->clientid + sizeof(pBroker->clientid), '_', 1) - 1) = '/';		/* ��clientid�ĵ�һ��_�滻��/���Ե��������sn����ʹ�� */
#endif
		*(SkipCharInString((uint8*)pBroker->clientid, (uint8*)pBroker->clientid + sizeof(pBroker->clientid), '_', 1) - 1) = 0;			/* ��clientid�ĵڶ���_�滻��\0���Ե��������sn����ʹ�� */
		if(!PubDevInfo(u8MqttTaskNo)) {
			pMqttComm->u8ConnRes = 9;
		} else {
			pMqttComm->u32SerialNo = g_Sys.SerialNo.u32Dat;
			pMqttComm->u8ConnRes = 10;
			return TRUE;
		}
	}

	/* �Ͽ����� */
	mqtt_disconnect(pBroker);
	pMqttComm->broker.pSocketFunc->Close(pMqttComm->MqttSocket);
	return FALSE;
}

/*==========================================================================
| Description	: ��������Mqtt����������Ϣ, ÿ��ֻ����һ��MQTT��
| G/Out var 	:
| Author		:					Date	:
\=========================================================================*/
BOOL RecvFromMqttServer(MQTT_COMM* pMqttComm)
{
	SocketFunction* pSocketFunc = pMqttComm->broker.pSocketFunc;
#if 0
	/* ��ȡ���������ֶ� */
	if(pMqttComm->GprsNewAdd.bFlag) {	/* ������ͷ�� */
		pMqttComm->u8TRBuf[0] = pMqttComm->GprsNewAdd.u8Header;
		pMqttComm->u8TRBuf[1] = pMqttComm->GprsNewAdd.u8LessLen;
	} else {	/* ��ȡ��ͷ */
		g_CodeTest.i32Val[23] = pSocketFunc->Recv(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0);
		if(g_CodeTest.i32Val[23] != 2) {	/* ��С��Ҳ�����ֽ� */
			if(g_CodeTest.uVal[23] && g_CodeTest.u32Val[49] == 1){
				g_CodeTest.u32Val[45]++;
			}
			return FALSE;
		}
	}
#else
	if(pSocketFunc->Recv(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0) != 2) {
		return FALSE;
	}
	while(pMqttComm->u8TRBuf[0] == MQTT_MSG_PINGRESP) {	/* PING������Ӧ, �ٴν������ֽ� */
		pMqttComm->GprsNewAdd.bSentPingFlag = FALSE;
		if(pSocketFunc->Recv(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0) != 2) {
			return FALSE;
		}
	}
#endif
	/* ������ȡֱ��MSB��Ϊ1, �򳬹�4λ�ɱ䳤���ֶ� */
	uint8* pBuf = pMqttComm->u8TRBuf + 1;
	while((*pBuf & 0x80) && (pBuf < pMqttComm->u8TRBuf + 5)) {
		if(pSocketFunc->Recv(pMqttComm->MqttSocket, ++pBuf, 1, 0) <= 0) {		/* ������ȡʧ�� */
			return FALSE;
		}
	}
	pBuf++;	/* ָ�򳤶Ⱥ����Buf�ռ� */

	/* ��ȡMqttʣ��ĳ��� */
	int16 iRemainLen = mqtt_parse_rem_len(pMqttComm->u8TRBuf);
	if(pBuf + iRemainLen > pMqttComm->u8TRBuf + MQTT_TR_BUF_BLEN) {	/* ������� */
		return FALSE;
	}

	if(iRemainLen>500) {
		g_CodeTest.u32Val[44]++;
	}
	/* ���ѽ������ȶ�ʣ�²��� */
	int16 iRcvLen;
	uint8 u8Cnt = 3;
	while((iRemainLen > 0) && u8Cnt) {
		iRcvLen = pSocketFunc->Recv(pMqttComm->MqttSocket, pBuf, iRemainLen, 0);
		if(iRcvLen == GPRS_ERR_NO_DATA) {		/* ����������û������, �����������ݴ���� */
			u8Cnt--;
			continue;
		} else if(iRcvLen < 0) {	/* ͨ��ʧ���� */
			break;
		}
		pBuf += iRcvLen;
		iRemainLen -= iRcvLen;
	}
	if(u8Cnt == 0) {	/* ����3����Ȼû����, ��Ϊ�����д�����, Ϊ�˷�ֹͻȻ�ֽ��յ�ʣ�������, ����ֱ�ӶϿ����� */
		g_GprsComm.Socket[pMqttComm->MqttSocket-1].u8Res = GPRS_ERR_CONNECTION_ABN;
	}
	return (iRemainLen <= 0);
}

BOOL WaitAndChkMqttEcho(MQTT_COMM* pMqttComm, uint16 uMsgId, uint8 MqttEchoType)
{
	if(RecvFromMqttServer(pMqttComm)
		&& (MQTTParseMessageType(pMqttComm->u8TRBuf) == MqttEchoType)
		&& (uMsgId == mqtt_parse_msg_id(pMqttComm->u8TRBuf)))
	{
		return TRUE;
	} else {
		return FALSE;
	}
}

/*==========================================================================
| Description	: ���������Ա���MQTT���������ӣ�TRUE�����������ͽ���˳����ɣ�FALSE�������������⣬��Ҫ����
| G/Out var 	:
| Author		: Cui yehong				Date	:
\=========================================================================*/
/* ����GPRS��Nagle, ���·���PING����ʱ��ֻ�����ֽ�һֱ������ȥ, ʹ���������PubһЩû�õ����ݽ�PING��ǿ�з��ͳ�ȥ */
BOOL GPRS_PubTest(MQTT_COMM *pMqttComm)
{
	/* ������Ϣ����:��ʼ�� */
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5;	/* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
	uint8 u8QoS = 0;	/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
	/* ������Ϣ����:��ӡTopic */
	PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
	PrintStringNoOvChk(&pTxBuf, "/DATA");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
	}
	/* ������Ϣ����:��ӡ���ݳ�Json��ʽ */
	*pTxBuf++ = '{';					/* Json��ʼ */
	PrintStringNoOvChk(&pTxBuf, "\"msg\":\"PING���������\"");
	*pTxBuf++ = '}';					/* Json��β */
	/* ������Ϣ����:���� */
	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
}
BOOL PingMqttServer(MQTT_COMM *pMqttComm)
{
	pMqttComm->u8TRBuf[0] = MQTT_MSG_PINGREQ;
	uint16 uRecvLen = 0;
	pMqttComm->u8TRBuf[1] = 0x00;
	SocketFunction* pSocketFunc = pMqttComm->broker.pSocketFunc;
#if 0
	/* �����������, ����ping����ʱ��������ڽ��շ������˵�����, �����յ�����Ӧ������ping����
	 * ����Nagle�㷨��ԭ��, ֻ�������ֽ�M25��ȵ������㹻��ʱ�Żᷢ�ͳ�ȥ, �������Ҫ��2s���Ҳ��ܳɹ�����.
	 * ���������߼�����������ı�ͷ��0x30 ��Pub�ı��ľ��ñ�־λbReadHeadFlagΪTRUE����ʹ��RecvFromMqttServer��������ʱ
	 * ����Ҫ���ձ�ͷ�� */
	if(pMqttComm->GprsNewAdd.bFlag) {	/* RecvFromMqttServer������û���յ�PING������Ӧ �����ٶ�һ��, ��û�оͷ���FALSE */
		/* �����ٴν���ping������Ӧ, �����м�С�ĸ�����֡���ݻ�����ping������Ӧ, �������Ҳ����� */
		if((uRecvLen = pSocketFunc->Recv(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0)) == 2) {
			if(pMqttComm->u8TRBuf[0] == MQTT_MSG_PINGRESP) {	/* PING����Ӧ */
				pMqttComm->GprsNewAdd.bFlag = FALSE;
				return TRUE;
			} else {	/* ��Ȼ��������Ӧ��, ������Ȼ���Ƚ���������Ӧ��, PING�����ż���, ��ΪMqtt�ı���ʱ��ֻҪ���һ��ͨ�žͻ����¼�ʱ */
				pMqttComm->GprsNewAdd.u8Header = pMqttComm->u8TRBuf[0];
				pMqttComm->GprsNewAdd.u8LessLen = pMqttComm->u8TRBuf[1];
				return TRUE;
			}
		}
		pMqttComm->GprsNewAdd.bFlag = FALSE;
		return FALSE;	/* û�յ���Ӧ����û�յ�PING������Ӧ */
	}

	if(((g_CodeTest.i32Val[2] = pSocketFunc->Send(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0)) == 2)
		&& GPRS_PubTest(pMqttComm)) {	/* ����PING��ʱ����һЩû�õ�����, ǿ�н�PING�����ͳ�ȥ */
		if((uRecvLen = pSocketFunc->Recv(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0)) == 2) {
			if(pMqttComm->u8TRBuf[0] == MQTT_MSG_PINGRESP) {	/* Ping����Ӧ */
				return TRUE;
			} else  {	/* ������Ӧͷ�� */
				pMqttComm->GprsNewAdd.bFlag = TRUE;	/* ��RecvFromMqttServer�����н���pub��ʱ�����ٽ��ձ�ͷ�� */
				pMqttComm->GprsNewAdd.u8Header = pMqttComm->u8TRBuf[0];
				pMqttComm->GprsNewAdd.u8LessLen = pMqttComm->u8TRBuf[1];
				return TRUE;
			}
		}
	}
	return FALSE;
#else
	/* �����������, ����ֻ����PING��, ��������Ӧ У����Ӧ��RecvFromMqttServer������. */
	if(g_GprsComm.Socket[pMqttComm->MqttSocket - 1].u8Res == GPRS_ERR_CONNECTION_ABN) {	/* �����ѶϿ� */
		return FALSE;
	}
	if(!pMqttComm->GprsNewAdd.bSentPingFlag) {	/* ��û���͹�PING�� */
		/* ����PING�� */
		if(pSocketFunc->Send(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0) == 2) {
			pMqttComm->GprsNewAdd.bSentPingFlag = TRUE;
			pMqttComm->GprsNewAdd.uRecvPingRespCnt = 10000;	/* ��ʱʱ��10S */
			return TRUE;
		} else {
			return FALSE;
		}
	} else if(pMqttComm->GprsNewAdd.uRecvPingRespCnt == 0) {	/* �Ѿ����͹�PING��, ���ҽ��ճ�ʱ�� */
		return FALSE;
	}
	return TRUE;	/* ���͹�PING��, ���ջ�û��ʱ */
#endif
}

/*==========================================================================
| Description	: ����Mqtt��Ϣ
	1. ����libemqtt.c��mqtt_publish_with_qos()����ʹ���ڴ濽�����ܿ���ʹ�������ջ����ʹ��
		Ϊ�˼����ڴ濽�������з��������ʹ��pMqttComm->u8TRBuf����Topic�������¼��裺
		����+��Ϣ���ִ洢�� pMqttComm->u8TRBuf[3]��ʼ�ĵ�ַ����ǰ��3��ByteԤ���� Mqtt����(1B)+����(2B)
		����ʹ�÷����ο���Ӧ�ĵ���λ��
	2. ���ݲ�ͬ��QoS���ȴ���Ӧ����Ӧ��
| G/Out var 	:
| Author		: King					Date	: 2017.09.11
\=========================================================================*/
BOOL PublishMqtt(MQTT_COMM* pMqttComm, uint8* pTxBuf, uint16 uMsgIDPt, uint8 u8QoS)
{
	uint8 u8MqttFirstByte;
	SocketFunction* pSocketFunc = pMqttComm->broker.pSocketFunc;

	/* MsgID��ά������� */
	if(u8QoS) {
		pMqttComm->broker.seq++;
		if(pMqttComm->broker.seq >= 256) {
			pMqttComm->broker.seq = 0;
		}
		pMqttComm->u8TRBuf[uMsgIDPt] = pMqttComm->broker.seq>>8;
		pMqttComm->u8TRBuf[uMsgIDPt+1] = pMqttComm->broker.seq&0xFF;
	}

	/* ���Topic���� */
	uMsgIDPt -= 5;
	pMqttComm->u8TRBuf[3] = uMsgIDPt/0x100;
	pMqttComm->u8TRBuf[4] = uMsgIDPt&0xFF;

	/* �����̶�ͷ����һ���ֽ� */
	u8MqttFirstByte = ((MQTT_MSG_PUBLISH) | (u8QoS<<1));
/*	�����Ҫ bRetain��DUP��־���������޸�
	if(bRetain) {
		u8MqttFirstByte |= MQTT_RETAIN_FLAG;
   	} */

	/* ���̶�ͷ��(����+����)������ */
	uint16 uTotalLen = pTxBuf - pMqttComm->u8TRBuf - 3;	/* ����Ϊ�̶�ͷ��Ԥ����3�ֽ� */
	if(uTotalLen <= 127) {
		pMqttComm->u8TRBuf[1] = u8MqttFirstByte;
		pMqttComm->u8TRBuf[2] = uTotalLen;
		uTotalLen += 2;
		if((g_CodeTest.i32Val[1] = pSocketFunc->Send(pMqttComm->MqttSocket, &pMqttComm->u8TRBuf[1], uTotalLen, 0)) != uTotalLen) {
			return FALSE;
		}
	} else {
		pMqttComm->u8TRBuf[0] = u8MqttFirstByte;
		pMqttComm->u8TRBuf[1] = ((uTotalLen % 0x80) | 0x80);
		pMqttComm->u8TRBuf[2] = uTotalLen / 0x80;
		uTotalLen += 3;
		if((g_CodeTest.u32Val[20]=pSocketFunc->Send(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, uTotalLen, 0)) != uTotalLen) {
			g_CodeTest.u32Val[22]++;
			return FALSE;
		}
	}

	/* �ȴ���Ӧ */
	if((u8QoS == 0)
		|| ((u8QoS == 1)
			&& WaitAndChkMqttEcho(pMqttComm, pMqttComm->broker.seq, MQTT_MSG_PUBACK))	/* �ȴ�PUBACK */
		|| ((u8QoS == 2)
			&& WaitAndChkMqttEcho(pMqttComm, pMqttComm->broker.seq, MQTT_MSG_PUBREC)		/* �ȴ�PUBREC */
			&& (mqtt_pubrel(&pMqttComm->broker, pMqttComm->broker.seq) == 1)				/* ���� PUBREL */
			&& WaitAndChkMqttEcho(pMqttComm, pMqttComm->broker.seq, MQTT_MSG_PUBCOMP)))	/* ���� PUBCOMP */
	{
		return TRUE;
	} else {
		return FALSE;
	}
}

/*==========================================================================
| Description	: �����豸��Ϣ
| G/Out var		:
| Author		: King			Date	: 2017-11-4
\=========================================================================*/
BOOL PubDevInfo(uint8 u8MqttTaskNo)
{
	MQTT_COMM* pMqttComm = &g_MqttComm[u8MqttTaskNo];
	
	/* ������Ϣ�����������ͺš�Ӳ���汾�š�����汾�š�IP�ȣ���QoS1���� */
	/* ������Ϣ����:��ʼ�� */
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5;	/* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
	uint8 u8QoS = 0;	/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
	/* ������Ϣ����:��ӡTopic */
	PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
	PrintStringNoOvChk(&pTxBuf, "/DEVINFO");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
	}
	/* ������Ϣ����:��ӡ���ݳ�Json��ʽ */
	*pTxBuf++ = '{';					/* Json��ʼ */
	PrintStringToJson(&pTxBuf, "device_type", DEVICE_TYPE_String);
	BOOT_SOFT_VERSION BootSoftVer;		/* ��ȡBoot����汾��Ϣ */
	GetSoftVersion(&BootSoftVer);
	PrintSoftVerToJson(&pTxBuf, "soft_ver", SOFTWARE_VER);						/* �����е�����汾 */
	PrintSoftVerToJson(&pTxBuf, "low_flash_ver", BootSoftVer.u32LowFlashVer);	/* ��flash�ռ������汾 */
	PrintSoftVerToJson(&pTxBuf, "high_flash_ver", BootSoftVer.u32HighFlashVer);	/* ��flash�ռ������汾 */
//	PrintT32ToJson(&pTxBuf, "lic_deadline", g_Sys.u32License_Seconds);			/* ��Ȩ��ɵ���ʱ�� */
//	PrintMqttConnResToJson(&pTxBuf);
	PrintStringNoOvChk(&pTxBuf, "\"mqtt_client_id\":\"");
	PrintMqttClientId(&pTxBuf, u8MqttTaskNo);
	*pTxBuf++ = '"';
	*pTxBuf++ = ',';
	PrintU32DatToJson(&pTxBuf, "rst_count", g_Sys.uRstCount, 0);
	PrintU32DatToJson(&pTxBuf, "sn", g_Sys.SerialNo.u32Dat, 0);
	pTxBuf--;	/* ���, */
	*pTxBuf++ = '}';					/* Json��β */
	/* ������Ϣ����:���� */
	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
}

void PrintMqttClientId(uint8** ppTxBuf, uint8 u8MqttTaskNo)
{
	uint8* pTxBuf = *ppTxBuf;

	PrintStringNoOvChk(&pTxBuf, DEV_TYPE_MQTT_TOPIC_String);
	pTxBuf--;			/* ��/�ĳ�_ */
	*pTxBuf++ = '_';
	PrintU32(&pTxBuf, pTxBuf+16, g_Sys.SerialNo.u32Dat, 0);
	*pTxBuf++ = '_';
	if(u8MqttTaskNo == 0) {
		*pTxBuf++ = 'P';
	} else {
		*pTxBuf++ = 'S';
	}
	if(CheckDebugVersion()) {
		*pTxBuf++ = '_';
		*pTxBuf++ = 'D';
	} else {
		*pTxBuf++ = 'U';
		*pTxBuf++ = 'B';
	}
	*pTxBuf++ = '_';
	*pTxBuf++ = 'S';
	*pTxBuf++ = '0' + g_MqttComm[u8MqttTaskNo].u8ServerNo + u8MqttTaskNo*MAX_MQTT_SERVER_NUM;

	*ppTxBuf = pTxBuf;
}

void PrintMqttConnResToJson(uint8** ppTxBuf)
{
	uint8* pTxBuf = *ppTxBuf;

	PrintStringNoOvChk(&pTxBuf, "\"mqtt_conn\":{");
	PrintH16DatToJson(&pTxBuf, "pub_res", g_MqttComm[0].u8ConnRes*0x100 + g_MqttComm[0].u8DnsAndTcpConRes);
	PrintU32DatToJson(&pTxBuf, "pub_try", g_MqttComm[0].u8Count_ConnTry, 0);
	PrintH16DatToJson(&pTxBuf, "sub_res", g_MqttComm[1].u8ConnRes*0x100 + g_MqttComm[1].u8DnsAndTcpConRes);
	PrintU32DatToJson(&pTxBuf, "sub_try", g_MqttComm[1].u8Count_ConnTry, 0);
	pTxBuf--;
	*pTxBuf++ = '}';			/* �� dev:{ ��� */
	*pTxBuf++ = ',';

	*ppTxBuf = pTxBuf;
}

/*==========================================================================
| Description	: MQTT�����ˣ����𷢲���Ϣ�����ݵ�MQTT������
		��������Ϣ�����ࣺ
		1. �ϳ����ڵĶ�ʱ���񣬶�ʱ���ڽϳ�������1s
		2. ����Ӧ�ٶȵ������ڳ���ʱ����������ĸ�ԣʱ���ڣ��ö����ڷ�����ѯ�Ƿ��и���Ӧ�ٶȵķ�������
| G/Out var		:
| Author		: king			Date	: 2017-9-4
\=========================================================================*/
extern BOOL PubDebugChn(MQTT_COMM* pMqttComm, uint8 u8DebugChnNo);
extern BOOL PubMqttPrint(MQTT_COMM* pMqttComm);
extern BOOL PubDebugPack(MQTT_COMM* pMqttComm);
extern BOOL PubSampleData(MQTT_COMM* pMqttComm);
extern BOOL PubYKFnXMS(MQTT_COMM* pMqttComm);
extern BOOL PubBeiDouHub(MQTT_COMM* pMqttComm);
/* ��Ҫ�ɵ����߱�֤��һ��������ΪNULL */
extern BOOL PubSpecialData(MQTT_COMM* pMqttComm);
void MqttPubTask(const void* argument)
{
	uint8 u8MqttTaskNo = MQTT_TYPE_PUB;
	MQTT_COMM* pMqttComm = &g_MqttComm[u8MqttTaskNo];

	while(1) {
//				while(1) {
//					Task_sleep(10000);
//				}
		OFF_INDICATOR_SERVER;		/* ��λ������ָʾ */
		pMqttComm->u8Count_ConnTry = 0;
		while(!OpenMqttSocketAndConnMqttServer(u8MqttTaskNo)) {
			if(pMqttComm->u8Count_ConnTry++ > 5) {
				g_GprsComm.GprsStatus = GPRS_NOT_INIT;
				GPRS_Reset();
				pMqttComm->u8Count_ConnTry = 0;
			}
			MQTT_CONN_FAIL_DEAL;
		};
		ON_INDICATOR_SERVER;		/* �ɹ����ӷ����� */
		
		do {
		    /* Semaphore_pend(,0)���ȼ���Ƿ���Sem�Ѿ�post������Ѿ���Sem�������᷵��TRUE
		    ����Pub������������PingMqttServer(),������70~80ms;
		    �����ʱ���ڣ���������д������������(��Semephore_post)�����³�ʱ��ִ��if()���ݣ�������ִ��else��������
		    �����Semaphore_pend()��Ҫ���uTmr_MqttPubRun_ms;		    �������ȼ��uTmr_MqttPubRun_ms����������Sem�������� 		    */
		    uint16 uTmr_MqttPubRun_ms = g_TcpIPComm.uTmr_MqttPubRun_ms;
//			uint16 uTmr_MqttPubRun_ms = 1000;
			if(Semaphore_pend(SEM_MqttPubReq, OS_TICK_KHz*g_TcpIPComm.uTmr_MqttPubRun_ms) && uTmr_MqttPubRun_ms) {
				if(QueryAndPubMqttConfResAndPage(pMqttComm)		/* ��������/ͬ����Ϣ */
				    && PubMsgAndAcq(pMqttComm))
				 { } else {
					 g_CodeTest.i32Val[0] = 1;
					 break;
				 }
			} else {
				if(GPRS_GetUnsentBLen(pMqttComm->MqttSocket) > GPRS_MAX_SEND_LEN) {	/* ���д�����Ϣ���ڷ�����, �ȴ� */
					Task_sleep(2000);
					continue;
				}
				g_TcpIPComm.uTmr_MqttPubRun_ms = 1000;
				if((pMqttComm->u32SerialNo == g_Sys.SerialNo.u32Dat)		/* ������к��Ƿ����仯, ��������仯������Ҫ�������� */
					&& QueryAndPubFlashReq(pMqttComm, TRUE) 				/* ����������� */
					&& QueryAndPubMqttConfResAndPage(pMqttComm)				/* ����ӡ���ý�� */
					&& PubMsgAndAcq(pMqttComm)                              /* ��Ϣ���� */
					&& (((g_CommConf.u32MqttPubIntv_ms != 1001
						&& g_CommConf.u32MqttPubIntv_ms != 1002))			/* �������������ڵ��Թ��ܣ���Ҫ���ó�1001���Ŵ򿪸ù��� */
						|| (PubMqttPrint(pMqttComm)							/* ����Mqtt��ӡ���� */
							&& PubDebugPack(pMqttComm)))					/* ����Mqtt�������� */
				#if SUPPORT_V4_MASTER
					&& PubYKFnXMS(pMqttComm)
				#endif
				#if SUPPORT_BEIDOU_HUB
					&& PubBeiDouHub(pMqttComm)
				#endif
					&& PubSpecialData(pMqttComm))							/* ÿ���豸ר�е����� */
				{ } else {
					g_CodeTest.i32Val[0] = 2;
					break;
				}
			}
			Task_sleep(1000);
			RecvFromMqttServer(pMqttComm);
		} while(PingMqttServer(pMqttComm));

		/* �Ͽ��Դ����� */
		mqtt_disconnect(&pMqttComm->broker);
		pMqttComm->broker.pSocketFunc->Close(pMqttComm->MqttSocket);
	}
}

/*==========================================================================
| Description	: MQTT���Ķˣ������MQTT�����������ȡ���ݣ�����������:
	MqttSubTask()	: ��������
	SubscribeMqtt()	: �������ⶩ��
| G/Out var		:
| Author		: King			Date	: 2017-9-11
\=========================================================================*/
extern void ProcRmtDataFromMqtt(uint8* pU8Topic, uint8* pU8Msg, uint8* pU8MsgEnd);
extern BOOL ProcMqttCmdInstr(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd);
extern BOOL ProcMqttOtherReq(MQTT_COMM* pMqttComm, uint8* pU8Topic, uint8* pU8Msg, uint8* pU8MsgEnd);
void MqttSubTask(const void* argument)
{
#if GPRS_TEST
	SocketFunction *pSocketFunc = &AllSocketFun[0];

	//����tcp����

	while(1) {
		int32 MqttSocket = pSocketFunc->NewSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		struct sockaddr_in socket_address;
		socket_address.sin_addr.s_addr = 0x9a3d682f;  //47.104.61.154;		//TODO: ETHERNET:((HOSTENT*)(&pMqttComm->u8TRBuf[DNS_DOMAIN_MAX_LEN]))->h_addr[0]
		socket_address.sin_family = AF_INET;
		socket_address.sin_port = htons(18888);		/* �̶��˿� */
		if(pSocketFunc->Connect(MqttSocket, (struct sockaddr*)&socket_address, sizeof(socket_address)) == 0) {	/* TCP����ʧ�� */
			g_CodeTest.u32Val[30] = g_CodeTest.u32Val[31] = g_CodeTest.u32Val[32] = g_CodeTest.u32Val[33] = g_CodeTest.u32Val[34] = 0;
			while(1) {

//				MQTT_COMM* pMqttComm = &g_MqttComm[1];
//				uint8 *pBuf = pMqttComm->u8TRBuf;
//				memset(pBuf, 'x', 1900);
//				uint16 crc = CalCRC16ForModbus(pBuf, 1900);
//				pBuf += 1900;
//				PrintU32WithLen(&pBuf, crc, 4);
//				uint32 u32SentLen = pSocketFunc->Send(MqttSocket, pMqttComm->u8TRBuf, 1904, 0);
//				Task_sleep(3000);
//				u32SentLen = pSocketFunc->Send(MqttSocket, "sucess", 6, 0);

				uint32 u32SentLen = pSocketFunc->Send(MqttSocket, "sucess", 6, 0);
				MQTT_COMM* pMqttComm = &g_MqttComm[1];
				uint8 *pBuf = pMqttComm->u8TRBuf;
				memset(pBuf, 'x', 1900);

				uint32 i = 0;
				uint32 sum  = 0;
				g_CodeTest.u32Val[30] = g_CodeTest.u32Val[31] = g_CodeTest.u32Val[32] = g_CodeTest.u32Val[33] = g_CodeTest.u32Val[34] = 0;
				for(; i< 20; i++) {
					int32 i32Recvlen = pSocketFunc->Recv(MqttSocket, pBuf, 2000, 0);
					if(i32Recvlen > 0) {
						g_CodeTest.u32Val[30] = i32Recvlen;
						sum += i32Recvlen;
						pBuf += i32Recvlen;
						g_CodeTest.u32Val[32]++;
					} else if(i32Recvlen == 0) {
						g_CodeTest.u32Val[33]++;
						break;
					} else {
						g_CodeTest.u32Val[34]++;
						break;
					}
				}



//				uint8 *pBuf = "0123456789";
//				uint8 rxBuf[strlen(pBuf)+10];
//				uint32 u32SentLen = pSocketFunc->Send(MqttSocket, pBuf, strlen(pBuf), 0);
//				if(u32SentLen == strlen(pBuf)) {
//					//����11223344556677889900
//					uint8 u32len = 0;
//					if((u32len = pSocketFunc->Recv(MqttSocket, rxBuf, strlen("11223344556677889900"), 0)) > 0) {
//						rxBuf[u32len] = 0;
//						if(strcmp(rxBuf, "11223344556677889900") == 0) {
//							g_CodeTest.i32Val[4]++;
//						} else {
//							g_CodeTest.i32Val[5]++;
//						}
//					} else {
//						g_CodeTest.i32Val[2]++;
//					}
//				} else{
//					g_CodeTest.i32Val[3]++;
//					g_CodeTest.u32Val[3] = u32SentLen;
//				}

				Task_sleep(1000);
			}
		}
		pSocketFunc->Close(MqttSocket);
		Task_sleep(5000);
		g_CodeTest.i32Val[1]++;
	}
#else
	uint32 u32MqttTaskNo = MQTT_TYPE_SUB;
	MQTT_COMM* pMqttComm = &g_MqttComm[(uint8)u32MqttTaskNo];
	uint8* pU8Topic;
	uint8* pU8TopicEnd;
	uint8* pU8Msg;
	uint8* pU8MsgEnd;
	uint8* pTxBuf;
	uint32 u32TopicSN;

	/* ���Ķ˵����� */
	while(1) {
//		while(1) {
//			Task_sleep(10000);
//		}
		pMqttComm->u8Count_ConnTry = 0;
		while(1) {
			if(!OpenMqttSocketAndConnMqttServer((uint8)u32MqttTaskNo)) {
				if(pMqttComm->u8Count_ConnTry++ > 5) {
					GPRS_Reset();
					g_GprsComm.GprsStatus = GPRS_NOT_INIT;
					pMqttComm->u8Count_ConnTry = 0;
				}
				MQTT_CONN_FAIL_DEAL;
			} else {
				/* ����ͨѶ���йص�����(��������) */
				pTxBuf = pMqttComm->u8TRBuf;
				PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
				PrintStringNoOvChk(&pTxBuf, "/SUB/#");
				*pTxBuf++ = 0;		/* ָ��Ľ�β */
				if(!SubscribeMqtt(pMqttComm, pMqttComm->u8TRBuf)) {
					continue;
				}
			#if SOFT_RUN1_TEST0
				/* �������к�����Ȩ���� */
				BOOL bSubSnLicReq = (g_PubSoftAuthCtr.uPubSnCnt || g_PubSoftAuthCtr.uPubLicCnt);
				if(bSubSnLicReq) {
					pTxBuf = pMqttComm->u8TRBuf;
					PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
					PrintStringNoOvChk(&pTxBuf, "/AUTH/#");
					*pTxBuf++ = 0;		/* ָ��Ľ�β */
					if(!SubscribeMqtt(pMqttComm, pMqttComm->u8TRBuf)) {
						continue;
					}
				}
				g_TcpIPComm.bSubSnLicReq = bSubSnLicReq;

				/* ��������ͨѶ�е��������� */
				BOOL bSubSoftReq = (g_PubSoftAuthCtr.uPubSoftInstallCnt || g_PubSoftAuthCtr.uPubSoftUpdateCnt);
				if(bSubSoftReq) {
					pTxBuf = pMqttComm->u8TRBuf;
					PrintStringNoOvChk(&pTxBuf, DEV_TYPE_MQTT_TOPIC_String);
					PrintStringNoOvChk(&pTxBuf, "+/AUTH/");
					*pTxBuf++ = '0' + (SOFTWARE_VER/1000)%10;
					*pTxBuf++ = '0' + (SOFTWARE_VER/100)%10;
					*pTxBuf++ = '0' + (SOFTWARE_VER/10)%10;
					*pTxBuf++ = '0' + (SOFTWARE_VER/1)%10;
					*pTxBuf++ = 0;		/* ָ��Ľ�β */
					if(!SubscribeMqtt(pMqttComm, pMqttComm->u8TRBuf)) {
						continue;
					}
				}
				g_TcpIPComm.bSubSoftReq = bSubSoftReq;
			#endif

			#if (SUPPORT_RMT_SEN_TOPIC || REMOTE_DEBUG_TERMINAL)
				if(!CheckString(g_CommConf.u8MqttSubTopic, sizeof(g_CommConf.u8MqttSubTopic))) {
					g_CommConf.u8MqttSubTopic[0] = 0;	/* ��ֹ�ַ�������������Ƚϳ��� */
				} else if((!CompareTopic(g_CommConf.u8MqttSubTopic + 5, "00000000")) && (!SubscribeMqtt(pMqttComm, g_CommConf.u8MqttSubTopic))) {
					continue;
				}
				memcpy(g_TcpIPComm.u8MqttSubTopic, g_CommConf.u8MqttSubTopic, sizeof(g_TcpIPComm.u8MqttSubTopic));
			#endif

				break;
			}
		}

	#if SUPPORT_V4_MASTER
		InitSubYKFTopic();
	#endif

		extern BOOL SubYKFTopic(MQTT_COMM* pMqttComm);
		while(PingMqttServer(pMqttComm)) {
			/* ÿ12��Сʱ����һ�������ʱ: g_Sys.u32Seconds_LastSyncRTC ��SetRTCSeconds()����ʱ���� */
			if((g_Sys.u32Seconds_LastSyncRTC == 0)
				|| (labs(GetRTCSeconds() - g_Sys.u32Seconds_LastSyncRTC) > 12*3600UL))
			{
//				SyncRealTimeBySNTP(pMqttComm);
			}

			/* �������Ƿ����仯 */
			if((pMqttComm->u32SerialNo != g_Sys.SerialNo.u32Dat)	/* ������к��Ƿ����仯, ��������仯������Ҫ�������� */
			#if (SUPPORT_RMT_SEN_TOPIC || REMOTE_DEBUG_TERMINAL)
				|| (strcmp((const char*)g_TcpIPComm.u8MqttSubTopic, (const char*)g_CommConf.u8MqttSubTopic) != 0)	/* Topic���� */
			#endif
			#if SOFT_RUN1_TEST0
				|| (g_TcpIPComm.bSubSnLicReq != (g_PubSoftAuthCtr.uPubSnCnt || g_PubSoftAuthCtr.uPubLicCnt))/* ĸ�幦�� */
				|| (g_TcpIPComm.bSubSoftReq != (g_PubSoftAuthCtr.uPubSoftInstallCnt || g_PubSoftAuthCtr.uPubSoftUpdateCnt))
			#endif
			#if SUPPORT_V4_MASTER
				|| (!SubYKFTopic(pMqttComm))	/* YKF���ⶩ���Ƿ�ɹ� */
			#endif
				|| 0)
			{
				break;

			/* ѭ���������ͬʱ�����Ķ����Ϣ */
			} else {
				g_CodeTest.u32Val[21]++;
				g_CodeTest.u32Val[49] = 1;
				/* ������if ��ֹRecvFromMqttServer��PING������Ӧ��ȡ�� */
				while(RecvFromMqttServer(pMqttComm) && (MQTTParseMessageType(pMqttComm->u8TRBuf) == MQTT_MSG_PUBLISH)) {
//				if(RecvFromMqttServer(pMqttComm) && (MQTTParseMessageType(pMqttComm->u8TRBuf) == MQTT_MSG_PUBLISH)) {
					/* ��Ϣ����:���Topic, Msg�ķֶ� */
					uint16 uTopicLen = mqtt_parse_pub_topic_ptr(pMqttComm->u8TRBuf, (const uint8_t**)&pU8Topic);	/* ���Topic��ʼָ������ݳ��� */
					pU8TopicEnd = pU8Topic + uTopicLen;
					uint16 uMsgLen = mqtt_parse_pub_msg_ptr(pMqttComm->u8TRBuf, (const uint8_t**)&pU8Msg);			/* ���Msg��ʼָ������ݳ��� */
					pU8MsgEnd = pU8Msg + uMsgLen;

					if(0) {	/* ����Ϊ������Ĵ������ʹ��else if���Ե����� */
				#if (SUPPORT_RMT_SEN_TOPIC)
					} else if(CompareTopic(pU8Topic, (const char*)g_TcpIPComm.u8MqttSubTopic)) { /* Զ�̴��������ĵ�ֵ */
						pU8Topic = SkipCharInString(pU8Topic, pU8TopicEnd, '/', 2);
						if(CompareTopic(pU8Topic, "RMTS")) {
							GetRmtSenFromMqttComm(&g_TcpIPComm.RmtSenFromMqtt, pU8Msg, pU8MsgEnd);
						} else {
							ProcRmtDataFromMqtt(pU8Topic, pU8Msg, pU8MsgEnd);		/* ������Ϣת�ɰ������к������� */
						}
				#endif
					} else if(CompareTopic(pU8Topic, DEV_TYPE_MQTT_TOPIC_String)) {		/* �豸 */
						pU8Topic = SkipCharInString(pU8Topic, pU8TopicEnd, '/', 1); /* �����ͺź� */
						u32TopicSN = atoi((const char*)pU8Topic);
						pU8Topic = SkipCharInString(pU8Topic, pU8TopicEnd, '/', 1);
						if(SOFT_RUN1_TEST0 && CompareTopic(pU8Topic, "AUTH/")) {	/* ĸ�岿��:������������Ȩ */
							pU8Topic += 5;
							if(CompareTopic(pU8Topic, "CMD")) {
								if(!ProcMqttAuth_CMD(pMqttComm, pU8Msg, pU8MsgEnd)) {
									break;
								}
							} else if(g_PubSoftAuthCtr.uPubSnCnt && CompareTopic(pU8Topic, "SN")) {	/* ��������кŸ������� */
								if(!ProcMqttAuth_SN(pMqttComm, pU8Msg, pU8MsgEnd)) {
									break;
								}
							} else if(g_PubSoftAuthCtr.uPubLicCnt && CompareTopic(pU8Topic, "LICENSE")) {	/* �������Ȩ���� */
								if(!ProcMqttAuth_LIC(pMqttComm, pU8Msg, pU8MsgEnd)) {
									break;
								}
							} else if((g_PubSoftAuthCtr.uPubSoftInstallCnt || g_PubSoftAuthCtr.uPubSoftUpdateCnt) /* ����Ǵ���Ƭ���� */
										&& (ReadU32(&pU8Topic, pU8TopicEnd) == SOFTWARE_VER) && (u32TopicSN >= 10000000UL))
							{
								if(!ProcMqttAuth_SOFT(pMqttComm, pU8Msg, pU8MsgEnd, u32TopicSN)) {
									break;
								}
							}
						} else if((u32TopicSN == g_Sys.SerialNo.u32Dat) && CompareTopic(pU8Topic, "SUB/")) {
							pU8Topic += 4;
							if(CompareTopic(pU8Topic, "UPDATE/")) {		        /* ����Ǳ�������Ҫ�Ĵ���Ƭ */
								if(!ProcMqttUpdate(pMqttComm, pU8Topic + 7, pU8TopicEnd, pU8Msg, pU8MsgEnd)) {
									break;
								}
                            } else if(CompareTopic(pU8Topic, "CMD")) {          /* ���ĵ�������Ϊ����ָ�� */
                                if(!ProcMqttCmdInstr(pMqttComm, pU8Msg, pU8MsgEnd)) {
                                    break;
                                }
							} else if(CompareTopic(pU8Topic, "SET/")) {
								pU8Topic += 4;
								if(CompareTopic(pU8Topic, "RST_CODE_TEST_U32")) {	/* ��ʱ�ԣ�������� */
									InitDataWithZero((uint8*)g_CodeTest.u32Val, sizeof(g_CodeTest.u32Val));
								} else if(CompareTopic(pU8Topic, "MSGPT")) {
#if DEV
									RegisterPageReq(DATA_PAGE_MSG, pU8Msg, pU8MsgEnd);	/* ItemNumΪ0���������Զ�������ָ�� */
#endif
								}
							} else if(CompareTopic(pU8Topic, "REQ/")) {				/* ���ĵ�������Ϊ������Ϣ */
								pU8Topic += 4;
								if(CompareTopic(pU8Topic, "DEVINFO")) {				/* ���ĵ�������Ϊ�����豸��Ϣ */
									if(!PubDevInfo(u32MqttTaskNo)) {
										break;
									}
                                } else if(CompareTopic(pU8Topic, "SYNC")) {         /* ͬ������ */
#if DEV
                                    ProcMqttConfSync(pU8Msg, pU8MsgEnd);
#endif
								} else if(CompareTopic(pU8Topic, "CONF")) {			/* ����ҳ�� */
//#if DEV
									ProcMqttConfInstr(pU8Msg, pU8MsgEnd);
//#endif
                                } else if(CompareTopic(pU8Topic, "MSG")) {          /* MSGҳ�� */
#if DEV
                                    RegisterPageReq(DATA_PAGE_MSG, pU8Msg, pU8MsgEnd);
#endif
                                } else if(CompareTopic(pU8Topic, "ACQ")) {          /* ACQҳ�� */
#if DEV
                                    RegisterPageReq(DATA_PAGE_ACQ, pU8Msg, pU8MsgEnd);
#endif
								} else if(CompareTopic(pU8Topic, "CRASH_LOG")) {
#if DEV
									PubCrashLog(pMqttComm);
#endif
                                } else if(CompareTopic(pU8Topic, "SAMPLE_DATA")) {
#if DEV
									PubSampleData(pMqttComm);
#endif
								} else if(CompareTopic(pU8Topic, "DEBUG_CHN")) { 	/* ����DEBUG_CHN���� */
#if DEV
									if(!PubDebugChn(pMqttComm, ReadI16(&pU8Msg, pU8MsgEnd))) {
										break;
									}
								} else if(!ProcMqttOtherReq(pMqttComm, pU8Topic, pU8Msg, pU8MsgEnd)) {
								    break;
#endif
								}
						#if SUPPORT_V4_MASTER
							} else if(CompareTopic(pU8Topic, "MODBUS")) {		/* �������ָ�� */
								ProcMqttRandAcsReq(pU8Topic, pU8Msg, pU8MsgEnd);
						#endif
						#if SUPPORT_FLOW_METER
							} else if(CompareTopic(pU8Topic, "CAMERA")) {	/* ���ջ���HTTP�������ָ�� */
								/* ��Ϣת������й�߳� */
								if(uMsgLen < FLOW_NET_CAMERA_RAW_CMD_BUF_LEN - 1) {
									uint8 *pU8CameraCmdBuf = g_HikDevConf.u8CameraRawCMDTRBuf;
									while(uMsgLen > 0) {
										*pU8CameraCmdBuf++ = *pU8Msg++;
										uMsgLen--;
									}
									*pU8CameraCmdBuf = 0;
								}
						#endif
							}
						}
				#if SUPPORT_V4_MASTER
					} else if(CompareTopic(pU8Topic, "YKF/")) {						/* YKF������ */
						ProcMqttMsgForYKF(pU8Topic, pU8Msg, pU8MsgEnd);
				#endif
					}
				}
				Task_sleep(1000);	/* ����һ�����һ���߳�һ����� */
#if SUPPORT_GPRS && SUPPORT_ETHERNET
				if((GetCurrAccess() == INTERNET_ACCESS_GPRS) && CheckEthernet()) {
					break;		/* �����̫�����ӻָ���Ͽ�GPRS(��̫������) */
				}
#endif
			}
		};

		/* �Ͽ��Դ����� */
		mqtt_disconnect(&pMqttComm->broker);
		pMqttComm->broker.pSocketFunc->Close(pMqttComm->MqttSocket);
	}
#endif		/* end of GPRS_TEST */
}

BOOL SubscribeMqtt(MQTT_COMM* pMqttComm, uint8* pU8Topic)
{
	uint16 uMsgId;

	if((mqtt_subscribe(&pMqttComm->broker, (const char*)pU8Topic, &uMsgId) == 1)
		&& GPRS_WaitSocketIdle(pMqttComm->MqttSocket, 0, 20, 10000)
		&& WaitAndChkMqttEcho(pMqttComm, uMsgId, MQTT_MSG_SUBACK)
		&& (pMqttComm->u8TRBuf[4] <= 2)) 	/* QoS��־ */
	{
		return TRUE;
	}

	/* �Ͽ��Դ����� */
	mqtt_disconnect(&pMqttComm->broker);
	pMqttComm->broker.pSocketFunc->Close(pMqttComm->MqttSocket);
	return FALSE;
}

/*==========================================================================
| Description	: ModbusTCP�������������޸ĵ�������ģ��Դ���롣
| G/Out var		:
| Author		: Meng Yaoyao			Date	: 2016-8-7
\=========================================================================*/
#if MODBUS_TCP_INT_TASK_NUM
void ModbusTcpTask(void const *pArgs);
void ModbusTcpSeverTask(uint32 u32Arg1, uint32 u32Arg2)
{
    int32 i32Socket;
    struct sockaddr_in ServerAddr;
    struct sockaddr_in ClientAddr;

	/* ��ʼ�� */
	while(1) {
		/* ����Server */
		while(1) {
			if(((int32)(i32Socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP))) <= 0) {
				Task_sleep(OS_TICK_KHz*100);
				continue;
			}
			memset(&ServerAddr, 0, sizeof(ServerAddr));
			memset(&ClientAddr, 0, sizeof(ClientAddr));
			ServerAddr.sin_family = AF_INET;
			ServerAddr.sin_port = htons(502);
			ServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);

			int32 optval = 1;
			int32 optlen = sizeof(optval);
			if((-1 == bind(i32Socket, (struct sockaddr *)&ServerAddr, sizeof(ServerAddr)))
				|| (-1 == listen(i32Socket, 2))	/* �����ӽ�����2������ע�������Ӻ���������������� */
				|| (setsockopt(i32Socket, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0))
			{
				Task_sleep(OS_TICK_KHz*100);
				fdClose(i32Socket);
				continue;
			} else {
				break;
			}
		}

		/* �ȴ��ͻ�������:  �����Ƿ�Ҫ����ͨѶ��ʱ������Ҫ�������ã����Ǹ������Ự�����ԣ� */
		int32 Clientfd;
		socklen_t AddrLen = sizeof(ClientAddr);
		while(((int32)(Clientfd = accept(i32Socket, (struct sockaddr *)&ClientAddr, &AddrLen))) > 0) {
			/* ����ͨѶ��ʱ����   */
			int32 flag = 1;
			struct timeval rcvTimeout;
			rcvTimeout.tv_sec = OS_TICK_KHz * 50;
			rcvTimeout.tv_usec = 0;
			struct timeval sndTimeout;
			sndTimeout.tv_sec = OS_TICK_KHz * 5;
			sndTimeout.tv_usec = 0;
			if((setsockopt(Clientfd, IPPROTO_TCP, TCP_NODELAY, (int8*)&flag, sizeof(flag)) < 0)		/* �ɸ�������㷨 */
				|| (setsockopt(Clientfd, SOL_SOCKET, SO_RCVTIMEO, &rcvTimeout, sizeof(rcvTimeout)) < 0)	/* ���ý��ճ�ʱ */
				|| (setsockopt(Clientfd, SOL_SOCKET, SO_SNDTIMEO, &sndTimeout, sizeof(sndTimeout)) < 0))	/* ���÷��ͳ�ʱ */
			{
				fdClose(Clientfd);
				continue;
			}

			/* ���ҿ��������ռ� */
			int16 i;
			for(i = 0; i < MODBUS_TCP_INT_TASK_NUM; i++) {
				if(g_ModbusTcpComm[i].bFree) {
					g_ModbusTcpComm[i].bFree = FALSE;
					break;
				}
			}

			if(i == MODBUS_TCP_INT_TASK_NUM) {	/* ������ƿ鶼��ռ�� */
				fdClose(Clientfd);
			} else {
		        int arg[] = {i, Clientfd};		TODO:
		        if(TaskCreate("modbusTcpClient", ModbusTcpTask, arg, 4, MODBUS_TCP_COMM_MAX_NUM, (uint8*)g_ModbusTcpComm[i].ModbusTcpSessionStack,
		        		MODBUS_TCP_TASK_STACK, &g_ModbusTcpComm[i].ModbusTCPSessionStackCtlBlk) == NULL) {
		        	break;
		        }
			}

	        AddrLen = sizeof(ClientAddr);	/* addrlen is a value-result param, must reset for next accept call */
	    }
		fdClose(i32Socket);
	}
}

/*==========================================================================
| Description	: ModbusTCP���ݴ����޸ĵ�������ģ��Դ���롣
					1������ModbusTcpЭ�飬ת����ModbusRtuЭ�顣
					2���ѻ��鷵�ص�ModbusRtuЭ�飬ת����ModbusTcpЭ�飬���ͳ�ȥ��
					3������д��ָ��������Ӧ��
					4��ΪʲôҪ����RespoundModbusTCP0A(0A�Ż�������) / RespoundModbusTCP(��0A�Ż�������)?
					��̬���� 0A���鷢�����Ĳ�ѯָ���Ǹ���Χ���� 0A0426480016+CRC �г��ȡ�
					������0A���������Ķ����鷢�����Ĳ�ѯָ��͵�����ַ�ĳ��� ������ 140426480001+CRC 140426490001 +CRC��
					���͹��������ͬ��
| G/Out var		:
| Author		: king			Date	: 2017-8-31
\=========================================================================*/
void ModbusTcpTask(void const *pArgs)
{
	MODBUS_TCP_COMM* pModbusTcpComm;
	int32 i32Clientfd;
	uint16 uModbusTcpTaskNo;

	/* ��ʼ�� */
	uModbusTcpTaskNo = (uint16)((uint32*)pArgs)[0];
	i32Clientfd = (int32)((uint32*)pArgs)[1];
	pModbusTcpComm = &g_ModbusTcpComm[uModbusTcpTaskNo];

    while(1) {
		/* ������Ϣ������֡ */
		uint16 uRxBufPt = 0;
		do {
			int32 i32RcvLen = recv(i32Clientfd, pModbusTcpComm->u8TRBuf + uRxBufPt, UART_TR_BUF_BLEN - uRxBufPt, 0);
			if(i32RcvLen <= 0) {
				fdClose(i32Clientfd);
				pModbusTcpComm->bFree = TRUE;
				return;
			}
			uRxBufPt += i32RcvLen;
		} while(uRxBufPt <= 9);
		pModbusTcpComm->uRxBufPt = uRxBufPt;

		/* ������Ϣ */
	#if(SUPPORT_V4_MASTER || SUPPORT_V5_MASTER)		/* ����Master���豸����Ҫ����ַ��飬��ȷ�����ʶ��� */
		if(pModbusTcpComm->u8TRBuf[6] == 0xFF) {
			RespondModbus(MAX_UART_NUM + uModbusTcpTaskNo, 0xFF);
		} else {
			RespondModbusTCPForOthers(pModbusTcpComm, TRUE);
		}
	#else	/* �����豸������ַ��飬��Ϊֻ�ܷ��ʱ��� */
		RespondModbus(MAX_UART_NUM + uModbusTcpTaskNo, pModbusTcpComm->u8TRBuf[6]);
	#endif

		if(pModbusTcpComm->uTxBufPt) {
			send(i32Clientfd, pModbusTcpComm->u8TRBuf, pModbusTcpComm->uTxBufPt, 0);
			pModbusTcpComm->uTxBufPt = 0;
		}
    }
}
#endif

#if SUPPORT_ETHERNET
/*==========================================================================
| Description	: TCP/IPͨѶ��ʼ��ģ�飬����App.cfg�Զ����ɵĴ��������²��㣺
	1. DHCP�����ܸ�������ֵ�������Ƿ�����
	2. ����֧�ֲ��ֽ���DHCP�Ľ��
	3. DNS�����ڴ�̫�࣬����Ҫ��̬�ڴ���䣬������
	NetCommTask()����ں�������App.cfg���ɵ�App_pem4f.c����ti_ndk_config_Global_stackThread()����
| G/Out var 	:
| Author		:					Date	:
\=========================================================================*/

void NetCommTask(void const * pArgs)
{
	/* �ؼ�������ʼ�� */
//	g_TcpIPComm.u32LocalIP = 0;
//	g_TcpIPComm.uTmr_MqttPubRun_ms = 0;
#if (SUPPORT_RMT_SEN_TOPIC || REMOTE_DEBUG_TERMINAL)
	g_TcpIPComm.RmtSenFromMqtt.u32RcvTime_RTCSeconds = 0;
	g_TcpIPComm.u8MqttSubTopic[0] = 0;
#endif
	int8 i;
	for(i = 0; i < MQTT_TASK_NUM; i++) {
		g_MqttComm[i].i32MqttSocket = -1;		/* ֮ǰTI��NULL�����ݴ��뿴��-1Ҳ���ԣ�bsd socket�ӿڵ�socket���ͱ�������int��ti��Ҫ�޸�ָ��Ϊint */
//		g_MqttComm[i].u8ServerNo = 0;
//		g_MqttComm[i].u8ConnRes = 0;
//		g_MqttComm[i].u8DnsAndTcpConRes = 0;
//		g_MqttComm[i].u8Count_ConnTry = 0;
//		g_MqttComm[i].u32TotalConnCnt = 0;
//		g_MqttComm[i].u32ConnDuration_ms = 0;
//		g_MqttComm[i].u32DNSDuration_ms = 0;
	}
}

/* �������ñ仯�ص� */
void DrvLinkIndicator(int i32LinkST_On1_Off0)
{
	if(i32LinkST_On1_Off0 && g_TcpIPComm.u32LocalIP) {
		ON_INDICATOR_NET;
	} else {
#if CPU_0ST_1TI
		/* ������·�Ͽ�������DHCP������ */
		StartTI_DHCP();
#endif
		OFF_INDICATOR_NET;
	}
}

/* ժ��dnsclnt.c���Լ����ڴ�ʹ�ã�����ԭ�����������static�ģ����ܱ��ⲿ���á�����DNSResolveQuery()������DNSQuery()
 * ���⣬������ʱ��ɾ���˶�IPv6��֧��
 * �����󲿷ֶѸĳ���ջ���������˲���Ҫ�Ŀ�����
 * �����ⲿbuf�������˶�ջ�����ģ�������uBufLen�Ա�ʶ���bufʹ�ó��ȣ�
 * ���Ӷ��������͵�ƥ�䣬ɾ���˷��������͵Ķ����Ӧ�ֶ������Ӷ��������ڴ�ʹ��
 * pQueryBuf��С�������DNS_TOTAL_BUF_LEN(1318) = DNS_DOMAIN_LEN(50) + DNS_SCRAP_MAX_LEN(268) + MAX_DNS_PACKET(1000) */
int DNSResolve(uint8 *pQueryBuf, uint16 uBufLen, uint8 af_family, uint16 uReqType)
{
    DNSREC      *pQuery_o_Replay = (DNSREC*)(pQueryBuf + DNS_DOMAIN_MAX_LEN);		/* ԭpScrapBuf*/
    uint8		*pPktBuf = pQueryBuf + DNS_DOMAIN_MAX_LEN + DNS_SCRAP_MAX_LEN;
    DNSREC 		*pRec;
    DNSREPLY    Reply;		/* pReply-> Reply�Ѹ�ջ*/
    HOSTENT     *phe;
    UINT8       *pbWrite;
    int         rc, writeused;
    IPN         IPTmp;

    /* validate that query str passed does not exceed DNS record name length��
       	   ������(size < sizeof(DNSREC))��ԭ��DNSREC����mmAloc���䣬���︴��pScrapBuf */
    if((strlen((char *)pQueryBuf) > DNS_NAME_MAX) || (uBufLen < DNS_TOTAL_BUF_LEN)) {
        return OVERFLOW;
    }

    /* Init query record */
    bzero(pQuery_o_Replay, sizeof(DNSREC));
    strcpy((char*)pQuery_o_Replay->Name, (char*)pQueryBuf);
    pQuery_o_Replay->Type  = uReqType;
    pQuery_o_Replay->Class = C_IN;
    bzero( &Reply, sizeof(DNSREPLY) );

    /* Resolve the query */
    rc = DNSQuery(DNS_OP_STD, pQuery_o_Replay, &Reply, (char*)pPktBuf);

    if(!rc) {
        return NODNSREPLY;
    }

    /* If the reply contains an error, return it */
    rc = (Reply.Flags & MASK_DNS_RCODE);
    if(rc) {
        goto drleave;
    }

    /* Decode the reply */
    /* "Allocate" the HOSTENT structure */
    phe = (HOSTENT *)pQuery_o_Replay;
    pbWrite = (UINT8 *)pQuery_o_Replay + sizeof(HOSTENT);

    /* Initialize the structure */
    phe->h_name     = 0;
    phe->h_addrtype = AF_INET;
    phe->h_length   = 4;
    phe->h_addrcnt  = 0;

    /* Read the Answer */
    pRec = &Reply.Ans;
	if( pRec->Class == C_IN ) {
		/* Copy the name to the write buffer */
		writeused = strlen((char*)(pRec->Name)) + 1;
		if(writeused > (DNS_SCRAP_MAX_LEN - sizeof( HOSTENT ))) {
			rc = OVERFLOW;
			goto drleave;
		}
		strcpy((char *)pbWrite, (char*)(pRec->Name));

		/* What we do next is dependent on the type */
		switch(pRec->Type) {
			case T_A:  /* IP Address Record */
				if(pRec->DataLength == 4) {	/* We only use this if the record size is == 4 */
					/* If we don't have a CNAME yet, we'll use the "owner" of this record. */
					if(!phe->h_name) {
						phe->h_name = (char *)pbWrite;
						/* we need to advance the pbWrite pointer to skip over the Hostname and also the null
						 * byte termination after it, Thus the (writeused + 1). */
						pbWrite += (writeused + 1);
					}

					/* Get the IP address */
					IPTmp =  ((UINT32)pRec->Data[0] << 24);
					IPTmp |= ((UINT32)pRec->Data[1] << 16);
					IPTmp |= ((UINT32)pRec->Data[2] << 8);
					IPTmp |= ((UINT32)pRec->Data[3]);
					IPTmp = htonl( IPTmp );

					if(phe->h_addrcnt < MAXIPADDR) {
						phe->h_addr[phe->h_addrcnt++] = IPTmp;
					}
				}
				break;

			case T_PTR:   /* Pointer Record (owner of an IP) */
				if(af_family == AF_INET) {	/* We'll take the IP address from this record if we can */
					IPTmp = inet_addr( (char *)pbWrite );

					/* The IPaddress obtained in DNS reverse name lookup reply is in reverse dotted notation.
					 * So convert the IP address to format we understand. */
					IPTmp = ((IPTmp>>24)&0xFF) | ((IPTmp>>8)&0xFF00) |
							((IPTmp<<8)&0xFF0000) | ((IPTmp<<24)&0xFF000000);

					if(phe->h_addrcnt < MAXIPADDR) {
						phe->h_addr[phe->h_addrcnt++] = IPTmp;
					}
				}   /* Fall through */
			case T_CNAME:   /* CNAME Record */
				/* Copy the data to the write buffer */
				writeused = pRec->DataLength + 1;
				if(writeused > (DNS_SCRAP_MAX_LEN - sizeof( HOSTENT ))) {
					rc = OVERFLOW;
					goto drleave;
				}

				/* Copy the "Host Name" present in the Domain Name field of the DNS reply. */
				strncpy((char *)pbWrite, (char *)(pRec->Data), writeused);

				/* Save the CNAME as the host name */
				phe->h_name = (char*)pbWrite;

				/* we need to advance the pbWrite pointer to skip over the Hostname and also the null
				 * byte termination after it, Thus the (writeused + 1).  */
				pbWrite += (writeused + 1);
				break;

			default:
				break;
		}
    }

drleave:
    return(rc);
}
#endif		/* SUPPORT_ETHERNET */

/****************SNTP client*****************
 *  The NTP timebase is 00:00 Jan 1 1900.  The local time base is 00:00 Jan 1 1970.  Convert between
 *  these two by added or substracting 70 years worth of time.  Note that 17 of these years were leap years.
 */
#define TIME_BASEDIFF        ((((unsigned int)70 * 365 + 17) * 24 * 3600) - (8 * 3600)) /*GMT+8*/
#define TIME_NTP_TO_LOCAL(t) ((t) - TIME_BASEDIFF)

/* SNTP Header (as specified in RFC 4330) */
typedef struct SNTPHeader {
    /*
     *  'flags' stores three values:
     *    - 2 bit Leap Indicator (LI)
     *    - 3 bit Version Number (VN)
     *    - 3 bit Mode.
     */
    uint8 	flags;
    uint8 	stratum;
    uint8 	poll;
    int8 	precision;
    int32   rootDelay;
    uint32  rootDispersion;
    uint32  referenceID;

    /* NTP time stamps */
    uint32 	referenceTS[2];
    uint32 	originateTS[2];
    uint32 	receiveTS[2];
    uint32 	transmitTS[2];
} SNTPHeader;

#define MAX_NTP_SERVER_NUM 4
const uint32 cnst_u32NTPServers[MAX_NTP_SERVER_NUM] = {
	0x78196C0B, 	/*120.25.108.11*/
	0xCB6B0658,		/*203.107.6.88*/
	0x7618EC2B,		/*118.24.236.43*/
	0xB65C0C0B 		/*182.92.12.11*/
};

/* Sync system datetime from source, currentlly is SNTP,
 * will block until time synchronized from source or tried all known servers  */
void SyncRealTimeBySNTP(MQTT_COMM* pMqttComm)
{
	SocketFunction *pSocketFun = pMqttComm->broker.pSocketFunc;
	int32 i32SocketSNTP;
	if(g_TcpIPComm.u32LocalIP && (((int32)(i32SocketSNTP = pSocketFun->NewSocket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))) > 0)) {
		struct timeval timeout;
		timeout.tv_sec = OS_TICK_KHz * 5;
		timeout.tv_usec = 0;
		if((pSocketFun->SetSockopt(i32SocketSNTP, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) >= 0)
			&& (pSocketFun->SetSockopt(i32SocketSNTP, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) >= 0))
		{
			/* loop all NTP servers until we get a valid datetime*/
			int16 i = 0;
			for( ; i < MAX_NTP_SERVER_NUM; i++) {
				struct sockaddr_in socket_address;
				socket_address.sin_addr.s_addr = htonl(cnst_u32NTPServers[i]);
				socket_address.sin_family = AF_INET;
				socket_address.sin_port = htons(123);	/* fix 123 */
				if(pSocketFun->Connect(i32SocketSNTP, (struct sockaddr*)&socket_address, sizeof(socket_address)) >= 0) {
					/* Initialize the SNTP packet, setting version and mode = client */
					SNTPHeader sntpPkt;
					memset(&sntpPkt, 0, sizeof(SNTPHeader));
					sntpPkt.flags = 4 /* SNTP_VERSION */ << 3;
					sntpPkt.flags |= 3;//SNTP_MODE_CLIENT;
					sntpPkt.transmitTS[0] = htonl(0);

					/* Send out our SNTP request to the current server */
					if((pSocketFun->Send(i32SocketSNTP, (void *)&sntpPkt, sizeof(SNTPHeader), 0) == sizeof(SNTPHeader))
						&& (pSocketFun->Recv(i32SocketSNTP, &sntpPkt, sizeof(SNTPHeader), MSG_WAITALL) == sizeof(SNTPHeader))
						&& (sntpPkt.stratum != 0) && (sntpPkt.stratum != 16))	/* Check for errors in server response */
					{
						/* server's transmit time is what we want */
						uint32 u32SNTPSeconds = TIME_NTP_TO_LOCAL(ntohl(sntpPkt.transmitTS[0]));
						SetRTCSeconds(u32SNTPSeconds);
				#if SUPPORT_V4_MASTER
					#if(DEVICE_TYPE == V5_YYT3) || (DEVICE_TYPE == V5_YYT4)
						if(labs(u32SNTPSeconds - g_GpsData.u32RTCSeconds) > 5)
					#else
						if(1)
					#endif
						{
							REAL_TIME_VAR RealTime;
							CalDateByRTCSeconds(u32SNTPSeconds, &RealTime);
							SetYKFTimeByDate(RealTime.u8Year, RealTime.u8Month, RealTime.u8Day,
											RealTime.u8Hour, RealTime.u8Minute, RealTime.u8Second);
						}
				#endif
					}
					break;	/* ������� */
				}
			}
		}
		pSocketFun->Close(i32SocketSNTP);
	}
}
/******************************** FILE END ********************************/
