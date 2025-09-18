/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved.
 *
 * File			: MldNet.c
 * Author		: King
 * Description	: 本文件主要包括两大部分：
	1. 应用层(Mqtt与ModbusTCP通讯)，入口函数: NetOpenHook()
	2. TCP/IP通讯配置与工具(包括DHCP、DNS等)，摘自TI的库文件，入口函数: NetCommTask(), DNSResolve()

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
 /* 操作系统头文件 */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* 项目公共头文件 */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"
#include "BoardSupport.h"

/* 本级以及下级模块头文件 */
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


#define SOCKET_RCV_TIMEOUT_s		10				/* socket读取超时 */
#define SOCKET_SND_TIMEOUT_s		5				/* socket发送超时 */
#define MAX_MQTT_SERVER_NUM			3				/* Pub:s1~s3, Sub:s4~s6 */
#define GPRS_MAX_SEND_LEN			2				/* GPRS发送缓冲区最大长度, 即发送缓冲区的长度大于这个就不要发送新东西了 */
/***************************************************************************
 						global variables definition
***************************************************************************/
#define MQTT_CONN_FAIL_DEAL		NOP

/***************************************************************************
						internal functions declaration
***************************************************************************/
BOOL SyncRealTimeBySNTP(MQTT_COMM* pMqttComm, const uint32 cnst_u32NTPSever);

/***************************************************************************
 							functions definition
***************************************************************************/

#define MAX_NTP_SERVER_NUM 4
const uint32 cnst_u32NTPServers[MAX_NTP_SERVER_NUM] = {
	0x78196C0B, 	/*120.25.108.11*/
	0xCB6B0658,		/*203.107.6.88*/
	0x7618EC2B,		/*118.24.236.43*/
	0xB65C0C0B 		/*182.92.12.11*/
};
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
| Description	: 打开MqttSocket，并连接之
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
	pMqttComm->GprsNewAdd.uTimer_SendPing_ms = 0xFFFF;
	/* 服务器DNS解析:  用 Mqtt通讯的TRBuf作为DNS解析临时使用的ram，一次DNS解析耗时约12~16ms,这个函数消耗内存估计在700B */
	if(!CheckDebugVersion()) {
		PrintStringNoOvChk(&pTxBuf, "s0.mqtt.dev.yoplore.com");
	} else {
		*pTxBuf++ = 's';	/* 域名必须如此，以防止获取BIN文件后修改服务器域名指向 */
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
	uint8 u8DnsAndTcpConRes = 0;	/* b5~4,b3~2,b1~0:三个服务器dns+tcp连接情况--0未尝试,1dns失败,2dns成功tcp失败,3tcp成功 */
	const SocketFunction *pSocketFunc = NULL;
	for(j = 0; j < MAX_INTERNET_ACCESS_NUM; j++) {
		pSocketFunc = &AllSocketFun[j];
		uint8 u8DnsAndTcpConRes = 0;	/* b5~4,b3~2,b1~0:三个服务器dns+tcp连接情况--0未尝试,1dns失败,2dns成功tcp失败,3tcp成功 */
		for(i = 3; i > 0; i--) {
			pMqttComm->u8ServerNo++;
			if(pMqttComm->u8ServerNo > MAX_MQTT_SERVER_NUM) {
				pMqttComm->u8ServerNo = 1;
			}
			pMqttComm->u8TRBuf[1] = '0' + pMqttComm->u8ServerNo + u8MqttTaskNo*MAX_MQTT_SERVER_NUM;

			uint32 u32IpResolved = 0;

			if(!pSocketFunc->DnsQuery(pMqttComm->u8TRBuf, &u32IpResolved)) {
				u8DnsAndTcpConRes |= (1<<((pMqttComm->u8ServerNo-1)*2));		/* DNS失败 */
			} else {
				/* 打开并设置MQTT的socket */
				if(((int32)(pMqttComm->MqttSocket = pSocketFunc->NewSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP))) <= 0) {
					pMqttComm->u8ConnRes = 1;
					return FALSE;
				}
				/* 纳格尔禁用算法 */
				int32 flag = 1;			/* 开启禁用Nagle算法: flag = 1开启该选项，0为关闭该选项 */
				if(pSocketFunc->SetSockopt(pMqttComm->MqttSocket, IPPROTO_TCP, TCP_NODELAY, (int8*)&flag, sizeof(flag)) < 0) {
					pSocketFunc->Close(pMqttComm->MqttSocket);
					pMqttComm->u8ConnRes = 2;
					return FALSE;
				}
				/* 设置接收超时 */
				struct timeval timeout;
				if(MQTT_TYPE_PUB == u8MqttTaskNo) {	/* PUB的接收不需要这么慢 */
					timeout.tv_sec = 1*OS_TICK_KHz;
				} else {
					timeout.tv_sec = SOCKET_RCV_TIMEOUT_s*OS_TICK_KHz;
				}
				timeout.tv_usec = 0*OS_TICK_KHz;
				if(pSocketFunc->SetSockopt(pMqttComm->MqttSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
					pSocketFunc->Close(pMqttComm->MqttSocket);
					pMqttComm->u8ConnRes = 3;
					return FALSE;
				}
				/* 设置发送超时 */
				timeout.tv_sec = SOCKET_SND_TIMEOUT_s*OS_TICK_KHz;
				timeout.tv_usec = 0*OS_TICK_KHz;
				if(pSocketFunc->SetSockopt(pMqttComm->MqttSocket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0) {
					pSocketFunc->Close(pMqttComm->MqttSocket);
					pMqttComm->u8ConnRes = 11;
					return FALSE;
				}
				/* 连接socket */
				struct sockaddr_in socket_address;
				socket_address.sin_addr.s_addr = u32IpResolved;		//TODO: ETHERNET:((HOSTENT*)(&pMqttComm->u8TRBuf[DNS_DOMAIN_MAX_LEN]))->h_addr[0]
				socket_address.sin_family = AF_INET;
				socket_address.sin_port = htons(1883);		/* 固定端口 */
				if(pSocketFunc->Connect(pMqttComm->MqttSocket, (struct sockaddr*)&socket_address, sizeof(socket_address)) < 0) {	/* TCP连接失败 */
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

	/* 连接Mqtt服务器, socket */
	mqtt_broker_handle_t *pBroker = &pMqttComm->broker;
	pTxBuf = pMqttComm->u8TRBuf;
	PrintMqttClientId(&pTxBuf, u8MqttTaskNo);
	*pTxBuf++ = 0;
	mqtt_init(pBroker, (const char*)pMqttComm->u8TRBuf);						/* 系统函数，mqtt初始化 */
	mqtt_init_auth(pBroker, "RwRVZXjkUO", "EGsWBH303D");						/* 系统函数，初始化登录用户名、密码 */
	mqtt_set_alive(pBroker, 60);												/* 填充MQTT的保持时间:30秒 */
	pBroker->socket_info = pMqttComm->MqttSocket;								/* 填充MQTT的套接字 */
	pBroker->pSocketFunc = pSocketFunc;

	/* Mqtt连接，即发出 "MQTT_CONNECT", 请求连接，并等待响应 */
	if(mqtt_connect(pBroker) != 1) {
		pMqttComm->u8ConnRes = 5;
	} else if(!GPRS_WaitSocketIdle(pMqttComm->MqttSocket, 0, 20, 10000)) {	/* 等待上个报文完全发送 */
		pMqttComm->u8ConnRes = 11;
	} else if(!RecvFromMqttServer(pMqttComm)) {
		pMqttComm->u8ConnRes = 6;
	} else if(MQTTParseMessageType(pMqttComm->u8TRBuf) != MQTT_MSG_CONNACK) {
		pMqttComm->u8ConnRes = 7;
	} else if(pMqttComm->u8TRBuf[3] != 0x00) {		/* CONNACK回复成功 */
		pMqttComm->u8ConnRes = 8;
	} else {
#if 0
//		/* 修正ID，用于后面的Topic */
//	#if((DEVICE_TYPE == V5_YYB) || (DEVICE_TYPE == V5_YYD) || (DEVICE_TYPE == V5_YYG))
//		pBroker->clientid[3] = '/';	/* 连接的时候，似乎会被改成0 (崔：查看代码并未发现此问题)*/
//	#else
//		pBroker->clientid[4] = '/';	/* 连接的时候，似乎会被改成0 (崔：查看代码并未发现此问题)*/
//	#endif
#else
		*(SkipCharInString((uint8*)pBroker->clientid, (uint8*)pBroker->clientid + sizeof(pBroker->clientid), '_', 1) - 1) = '/';		/* 将clientid的第一个_替换成/，以当成主题的sn部分使用 */
#endif
		*(SkipCharInString((uint8*)pBroker->clientid, (uint8*)pBroker->clientid + sizeof(pBroker->clientid), '_', 1) - 1) = 0;			/* 将clientid的第二个_替换成\0，以当成主题的sn部分使用 */
		if(!PubDevInfo(u8MqttTaskNo)) {
			pMqttComm->u8ConnRes = 9;
		} else {
			pMqttComm->u32SerialNo = g_Sys.SerialNo.u32Dat;
			pMqttComm->u8ConnRes = 10;
			return TRUE;
		}
	}

	/* 断开链接 */
	mqtt_disconnect(pBroker);
	pMqttComm->broker.pSocketFunc->Close(pMqttComm->MqttSocket);
	return FALSE;
}

/*==========================================================================
| Description	: 接收来自Mqtt服务器的消息, 每次只解析一个MQTT包
| G/Out var 	:
| Author		:					Date	:
\=========================================================================*/
BOOL RecvFromMqttServer(MQTT_COMM* pMqttComm)
{
	SocketFunction* pSocketFunc = pMqttComm->broker.pSocketFunc;
#if 0
	/* 读取解析长度字段 */
	if(pMqttComm->GprsNewAdd.bFlag) {	/* 读过报头了 */
		pMqttComm->u8TRBuf[0] = pMqttComm->GprsNewAdd.u8Header;
		pMqttComm->u8TRBuf[1] = pMqttComm->GprsNewAdd.u8LessLen;
	} else {	/* 读取报头 */
		g_CodeTest.i32Val[23] = pSocketFunc->Recv(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0);
		if(g_CodeTest.i32Val[23] != 2) {	/* 最小包也有两字节 */
			if(g_CodeTest.uVal[23] && g_CodeTest.u32Val[49] == 1){
				g_CodeTest.u32Val[45]++;
			}
			return FALSE;
		}
	}
#else
//	do {	/* While里的判断仅仅是为了让第一个判断成立时，标志位被清除. */
//		if(pSocketFunc->Recv(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0) != 2) {
//				return FALSE;
//		}
//	}while(pMqttComm->u8TRBuf[0] == MQTT_MSG_PINGRESP && ((pMqttComm->GprsNewAdd.bSentPingFlag = FALSE) || 1));

	if(pSocketFunc->Recv(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0) != 2) {
		return FALSE;
	}
	while(pMqttComm->u8TRBuf[0] == MQTT_MSG_PINGRESP) {	/* PING包的响应, 再次接收两字节 */
		pMqttComm->GprsNewAdd.bSentPingFlag = FALSE;
		if(pSocketFunc->Recv(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0) != 2) {
			return FALSE;
		}
	}
#endif
	/* 继续读取直到MSB不为1, 或超过4位可变长度字段 */
	uint8* pBuf = pMqttComm->u8TRBuf + 1;
	while((*pBuf & 0x80) && (pBuf < pMqttComm->u8TRBuf + 5)) {
		if(pSocketFunc->Recv(pMqttComm->MqttSocket, ++pBuf, 1, 0) <= 0) {		/* 发生读取失败 */
			return FALSE;
		}
	}
	pBuf++;	/* 指向长度后面的Buf空间 */

	/* 获取Mqtt剩余的长度 */
	int16 iRemainLen = mqtt_parse_rem_len(pMqttComm->u8TRBuf);
	if(pBuf + iRemainLen > pMqttComm->u8TRBuf + MQTT_TR_BUF_BLEN) {	/* 避免溢出 */
		return FALSE;
	}

	if(iRemainLen>500) {
		g_CodeTest.u32Val[44]++;
	}
	/* 按已解析长度读剩下部分 */
	int16 iRcvLen;
	uint8 u8Cnt = 3;
	while((iRemainLen > 0) && u8Cnt) {
		iRcvLen = pSocketFunc->Recv(pMqttComm->MqttSocket, pBuf, iRemainLen, 0);
		if(iRcvLen == 0) {		/* 可能网不好没接收完, 这里有三次容错机会 */
			u8Cnt--;
			continue;
		} else if(iRcvLen < 0) {	/* 通信失败了 */
			break;
		}
		pBuf += iRcvLen;
		iRemainLen -= iRcvLen;
	}
	if(u8Cnt == 0) {	/* 尝试3次仍然没读完, 认为网络有大问题, 为了防止突然又接收到剩余的数据, 这里直接断开重连 */
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
| Description	: 发送心跳以保持MQTT服务器连接，TRUE代表心跳发送接收顺利完成，FALSE代表网络有问题，需要重连
| G/Out var 	:
| Author		: Cui yehong				Date	:
\=========================================================================*/
/* 由于GPRS有Nagle, 导致发送PING包的时候只有两字节一直发不出去, 使用这个函数Pub一些没用的数据将PING包强行发送出去 */
BOOL GPRS_PubTest(MQTT_COMM *pMqttComm)
{
	/* 本机信息发布:初始化 */
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5;	/* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
	uint8 u8QoS = 0;	/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
	/* 本机信息发布:打印Topic */
	PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
	PrintStringNoOvChk(&pTxBuf, "/DATA");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
	}
	/* 本机信息发布:打印数据成Json格式 */
	*pTxBuf++ = '{';					/* Json开始 */
	PrintStringNoOvChk(&pTxBuf, "\"msg\":\"PING包填充数据\"");
	*pTxBuf++ = '}';					/* Json结尾 */
	/* 本机信息发布:传输 */
	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS修改必须在本段代码开始的位置 */
}
BOOL PingMqttServer(MQTT_COMM *pMqttComm)
{
	pMqttComm->u8TRBuf[0] = MQTT_MSG_PINGREQ;
	pMqttComm->u8TRBuf[1] = 0x00;
	SocketFunction* pSocketFunc = pMqttComm->broker.pSocketFunc;
#if 0
	/* 由于网络较慢, 发送ping包的时候可能正在接收服务器端的数据, 导致收到的响应并不是ping包的
	 * 由于Nagle算法的原因, 只发送两字节M25会等到数据足够长时才会发送出去, 如果死等要等2s左右才能成功发送.
	 * 因此这里的逻辑是如果读到的报头是0x30 即Pub的报文就置标志位bReadHeadFlag为TRUE后面使用RecvFromMqttServer接受数据时
	 * 不需要接收报头了 */
	if(pMqttComm->GprsNewAdd.bFlag) {	/* RecvFromMqttServer函数中没有收到PING包的响应 这里再读一次, 还没有就返回FALSE */
		/* 这里再次接收ping包的响应, 但是有极小的概率这帧数据还不是ping包的响应, 这里暂且不考虑 */
		if((uRecvLen = pSocketFunc->Recv(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0)) == 2) {
			if(pMqttComm->u8TRBuf[0] == MQTT_MSG_PINGRESP) {	/* PING包响应 */
				pMqttComm->GprsNewAdd.bFlag = FALSE;
				return TRUE;
			} else {	/* 仍然是其他响应包, 这里仍然优先接收其他响应包, PING包不着急发, 因为Mqtt的保活时间只要完成一次通信就会重新计时 */
				pMqttComm->GprsNewAdd.u8Header = pMqttComm->u8TRBuf[0];
				pMqttComm->GprsNewAdd.u8LessLen = pMqttComm->u8TRBuf[1];
				return TRUE;
			}
		}
		pMqttComm->GprsNewAdd.bFlag = FALSE;
		return FALSE;	/* 没收到响应或者没收到PING包的响应 */
	}

	if(((g_CodeTest.i32Val[2] = pSocketFunc->Send(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0)) == 2)
		&& GPRS_PubTest(pMqttComm)) {	/* 发送PING包时发送一些没用的数据, 强行将PING包发送出去 */
		if((uRecvLen = pSocketFunc->Recv(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0)) == 2) {
			if(pMqttComm->u8TRBuf[0] == MQTT_MSG_PINGRESP) {	/* Ping包响应 */
				return TRUE;
			} else  {	/* 其他响应头包 */
				pMqttComm->GprsNewAdd.bFlag = TRUE;	/* 在RecvFromMqttServer函数中接收pub包时不用再接收报头了 */
				pMqttComm->GprsNewAdd.u8Header = pMqttComm->u8TRBuf[0];
				pMqttComm->GprsNewAdd.u8LessLen = pMqttComm->u8TRBuf[1];
				return TRUE;
			}
		}
	}
	return FALSE;
#else
	/* 由于网络较慢, 这里只发送PING包, 不接收响应 校验响应在RecvFromMqttServer函数中. */
	if(!g_GprsComm.Socket[pMqttComm->MqttSocket - 1].bIsConnect) {	/* 连接已断开 */
		return FALSE;
	}
	if(!pMqttComm->GprsNewAdd.bSentPingFlag) {	/* 还没发送过PING包, 防止发送多个ping包 */
		if(pMqttComm->GprsNewAdd.uTimer_SendPing_ms > 20000) {		/* 每隔20s发送一次Ping包 */
			pMqttComm->GprsNewAdd.uTimer_SendPing_ms = 0;
			/* 发送PING包 */
			if(pSocketFunc->Send(pMqttComm->MqttSocket, pMqttComm->u8TRBuf, 2, 0) == 2) {
				pMqttComm->GprsNewAdd.bSentPingFlag = TRUE;
				pMqttComm->GprsNewAdd.uTmr_RecvPingResp_ms = 30000;	/* 超时时间30S */
				return TRUE;
			} else {
				return FALSE;
			}
		}
	} else if(pMqttComm->GprsNewAdd.uTmr_RecvPingResp_ms == 0) {	/* 已经发送过PING包, 并且接收超时了 */
		return FALSE;
	}
	return TRUE;	/* 发送过PING包, 接收还没超时 */
#endif
}

/*==========================================================================
| Description	: 发布Mqtt消息
	1. 由于libemqtt.c的mqtt_publish_with_qos()大量使用内存拷贝，很可能使得任务堆栈大量使用
		为了减少内存拷贝，所有发布任务均使用pMqttComm->u8TRBuf整理Topic，有如下假设：
		主题+消息部分存储在 pMqttComm->u8TRBuf[3]开始的地址，即前面3个Byte预留给 Mqtt类型(1B)+长度(2B)
		具体使用方法参考相应的调用位置
	2. 根据不同的QoS，等待相应的响应。
| G/Out var 	:
| Author		: King					Date	: 2017.09.11
\=========================================================================*/
BOOL PublishMqtt(MQTT_COMM* pMqttComm, uint8* pTxBuf, uint16 uMsgIDPt, uint8 u8QoS)
{
	uint8 u8MqttFirstByte;
	SocketFunction* pSocketFunc = pMqttComm->broker.pSocketFunc;

	/* MsgID的维护与填充 */
	if(u8QoS) {
		pMqttComm->broker.seq++;
		if(pMqttComm->broker.seq >= 256) {
			pMqttComm->broker.seq = 0;
		}
		pMqttComm->u8TRBuf[uMsgIDPt] = pMqttComm->broker.seq>>8;
		pMqttComm->u8TRBuf[uMsgIDPt+1] = pMqttComm->broker.seq&0xFF;
	}

	/* 填充Topic长度 */
	uMsgIDPt -= 5;
	pMqttComm->u8TRBuf[3] = uMsgIDPt/0x100;
	pMqttComm->u8TRBuf[4] = uMsgIDPt&0xFF;

	/* 产生固定头部第一个字节 */
	u8MqttFirstByte = ((MQTT_MSG_PUBLISH) | (u8QoS<<1));
/*	如果需要 bRetain、DUP标志，在这里修改
	if(bRetain) {
		u8MqttFirstByte |= MQTT_RETAIN_FLAG;
   	} */

	/* 填充固定头部(类型+长度)并传输 */
	uint16 uTotalLen = pTxBuf - pMqttComm->u8TRBuf - 3;	/* 减掉为固定头部预留的3字节 */
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

	/* 等待响应 */
	if((u8QoS == 0)
		|| ((u8QoS == 1)
			&& WaitAndChkMqttEcho(pMqttComm, pMqttComm->broker.seq, MQTT_MSG_PUBACK))	/* 等待PUBACK */
		|| ((u8QoS == 2)
			&& WaitAndChkMqttEcho(pMqttComm, pMqttComm->broker.seq, MQTT_MSG_PUBREC)		/* 等待PUBREC */
			&& (mqtt_pubrel(&pMqttComm->broker, pMqttComm->broker.seq) == 1)				/* 发送 PUBREL */
			&& WaitAndChkMqttEcho(pMqttComm, pMqttComm->broker.seq, MQTT_MSG_PUBCOMP)))	/* 发送 PUBCOMP */
	{
		return TRUE;
	} else {
		return FALSE;
	}
}

/*==========================================================================
| Description	: 发布设备信息
| G/Out var		:
| Author		: King			Date	: 2017-11-4
\=========================================================================*/
BOOL PubDevInfo(uint8 u8MqttTaskNo)
{
	MQTT_COMM* pMqttComm = &g_MqttComm[u8MqttTaskNo];
	
	/* 本机信息发布，包括型号、硬件版本号、软件版本号、IP等，以QoS1发布 */
	/* 本机信息发布:初始化 */
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5;	/* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
	uint8 u8QoS = 0;	/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
	/* 本机信息发布:打印Topic */
	PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
	PrintStringNoOvChk(&pTxBuf, "/DEVINFO");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
	}
	/* 本机信息发布:打印数据成Json格式 */
	*pTxBuf++ = '{';					/* Json开始 */
	PrintStringToJson(&pTxBuf, "device_type", DEVICE_TYPE_String);
	BOOT_SOFT_VERSION BootSoftVer;		/* 获取Boot区域版本信息 */
	GetSoftVersion(&BootSoftVer);
	PrintSoftVerToJson(&pTxBuf, "soft_ver", SOFTWARE_VER);						/* 在运行的软件版本 */
	PrintSoftVerToJson(&pTxBuf, "low_flash_ver", BootSoftVer.u32LowFlashVer);	/* 低flash空间的软件版本 */
	PrintSoftVerToJson(&pTxBuf, "high_flash_ver", BootSoftVer.u32HighFlashVer);	/* 高flash空间的软件版本 */
//	PrintT32ToJson(&pTxBuf, "lic_deadline", g_Sys.u32License_Seconds);			/* 授权许可到期时间 */
//	PrintMqttConnResToJson(&pTxBuf);
	PrintStringNoOvChk(&pTxBuf, "\"mqtt_client_id\":\"");
	PrintMqttClientId(&pTxBuf, u8MqttTaskNo);
	*pTxBuf++ = '"';
	*pTxBuf++ = ',';
	PrintU32DatToJson(&pTxBuf, "rst_count", g_Sys.uRstCount, 0);
	PrintU32DatToJson(&pTxBuf, "sn", g_Sys.SerialNo.u32Dat, 0);
	pTxBuf--;	/* 多个, */
	*pTxBuf++ = '}';					/* Json结尾 */
	/* 本机信息发布:传输 */
	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS修改必须在本段代码开始的位置 */
}

void PrintMqttClientId(uint8** ppTxBuf, uint8 u8MqttTaskNo)
{
	uint8* pTxBuf = *ppTxBuf;

	PrintStringNoOvChk(&pTxBuf, DEV_TYPE_MQTT_TOPIC_String);
	pTxBuf--;			/* 把/改成_ */
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
	*pTxBuf++ = '}';			/* 和 dev:{ 配对 */
	*pTxBuf++ = ',';

	*ppTxBuf = pTxBuf;
}

/*==========================================================================
| Description	: MQTT发布端，负责发布消息和数据到MQTT服务器
		发布的信息分两类：
		1. 较长周期的定时任务，定时周期较长，例如1s
		2. 高响应速度的任务，在长定时的任务完后后的富裕时间内，用短周期反复查询是否有高响应速度的发布工作
| G/Out var		:
| Author		: king			Date	: 2017-9-4
\=========================================================================*/
extern BOOL PubDebugChn(MQTT_COMM* pMqttComm, uint8 u8DebugChnNo);
extern BOOL PubMqttPrint(MQTT_COMM* pMqttComm);
extern BOOL PubDebugPack(MQTT_COMM* pMqttComm);
extern BOOL PubSampleData(MQTT_COMM* pMqttComm);
extern BOOL PubYKFnXMS(MQTT_COMM* pMqttComm);
extern BOOL PubBeiDouHub(MQTT_COMM* pMqttComm);
/* 需要由调用者保证第一个参数不为NULL */
extern BOOL PubSpecialData(MQTT_COMM* pMqttComm);
void MqttPubTask(const void* argument)
{
	uint8 u8MqttTaskNo = MQTT_TYPE_PUB;
	MQTT_COMM* pMqttComm = &g_MqttComm[u8MqttTaskNo];

	while(1) {

		OFF_INDICATOR_SERVER;		/* 复位服务器指示 */
		pMqttComm->u8Count_ConnTry = 0;
		while(!OpenMqttSocketAndConnMqttServer(u8MqttTaskNo)) {
			pMqttComm->u8Count_ConnTry++;
			MQTT_CONN_FAIL_DEAL;
		};
		ON_INDICATOR_SERVER;		/* 成功连接服务器 */
		
		do {
		    /* Semaphore_pend(,0)会先检查是否有Sem已经post，如果已经有Sem就绪，会返回TRUE
		    由于Pub任务最后进行了PingMqttServer(),会消耗70~80ms;
		    在这段时间内，如果不断有待发送任务就绪(即Semephore_post)，导致长时间执行if()内容，而不会执行else部分内容
		    因此在Semaphore_pend()后，要检查uTmr_MqttPubRun_ms;		    但不能先检查uTmr_MqttPubRun_ms，那样会让Sem不被清零 		    */
		    uint16 uTmr_MqttPubRun_ms = g_TcpIPComm.uTmr_MqttPubRun_ms;
//			uint16 uTmr_MqttPubRun_ms = 1000;
			if(Semaphore_pend(SEM_MqttPubReq, OS_TICK_KHz*g_TcpIPComm.uTmr_MqttPubRun_ms) && uTmr_MqttPubRun_ms) {
				if(QueryAndPubMqttConfResAndPage(pMqttComm)		/* 发布配置/同步消息 */
				    && PubMsgAndAcq(pMqttComm))
				 { } else {
					 g_CodeTest.i32Val[0] = 1;
					 break;
				 }
			} else {
				if(GPRS_GetUnsentBLen(pMqttComm->MqttSocket) > GPRS_MAX_SEND_LEN) {	/* 还有大量消息正在发送中, 等待 */
					Task_sleep(10);
					continue;
				}
				g_TcpIPComm.uTmr_MqttPubRun_ms = 1000;
				if((pMqttComm->u32SerialNo == g_Sys.SerialNo.u32Dat)		/* 检查序列号是否发生变化, 如果发生变化，就需要重新连接 */
					&& QueryAndPubFlashReq(pMqttComm, TRUE) 				/* 软件升级部分 */
					&& QueryAndPubMqttConfResAndPage(pMqttComm)				/* 检查打印配置结果 */
					&& PubMsgAndAcq(pMqttComm)                              /* 消息部分 */
					&& (((g_CommConf.u32MqttPubIntv_ms != 1001
						&& g_CommConf.u32MqttPubIntv_ms != 1002))			/* 下面这两项属于调试功能，需要配置成1001，才打开该功能 */
						|| (PubMqttPrint(pMqttComm)							/* 发布Mqtt打印部分 */
							&& PubDebugPack(pMqttComm)))					/* 发布Mqtt调试数据 */
				#if SUPPORT_V4_MASTER
					&& PubYKFnXMS(pMqttComm)
				#endif
				#if SUPPORT_BEIDOU_HUB
					&& PubBeiDouHub(pMqttComm)
				#endif
					&& PubSpecialData(pMqttComm))							/* 每个设备专有的数据 */
				{ } else {
					g_CodeTest.i32Val[0] = 2;
					break;
				}
			}
//			Task_sleep(1000);
			RecvFromMqttServer(pMqttComm);
//			GPRS_GetSigStrong();	/* 看一下信号强度 */
		} while(PingMqttServer(pMqttComm));

		/* 断开以待重连 */
		mqtt_disconnect(&pMqttComm->broker);
		pMqttComm->broker.pSocketFunc->Close(pMqttComm->MqttSocket);
	}
}

/*==========================================================================
| Description	: MQTT订阅端，负责从MQTT服务器上面获取数据，分两个函数:
	MqttSubTask()	: 订阅任务
	SubscribeMqtt()	: 进行主题订阅
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

	//建立tcp链接

	while(1) {
		int32 MqttSocket = pSocketFunc->NewSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		struct sockaddr_in socket_address;
		socket_address.sin_addr.s_addr = 0x9a3d682f;  //47.104.61.154;		//TODO: ETHERNET:((HOSTENT*)(&pMqttComm->u8TRBuf[DNS_DOMAIN_MAX_LEN]))->h_addr[0]
		socket_address.sin_family = AF_INET;
		socket_address.sin_port = htons(18888);		/* 固定端口 */
		if(pSocketFunc->Connect(MqttSocket, (struct sockaddr*)&socket_address, sizeof(socket_address)) == 0) {	/* TCP连接失败 */
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
//					//接收11223344556677889900
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

	/* 订阅端的主题 */
	while(1) {
		pMqttComm->u8Count_ConnTry = 0;
		while(1) {
			if(!OpenMqttSocketAndConnMqttServer((uint8)u32MqttTaskNo)) {
				pMqttComm->u8Count_ConnTry++;
				MQTT_CONN_FAIL_DEAL;
			} else {
				/* 订阅通讯盒有关的主题(升级程序) */
				pTxBuf = pMqttComm->u8TRBuf;
				PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
				PrintStringNoOvChk(&pTxBuf, "/SUB/#");
				*pTxBuf++ = 0;		/* 指针的结尾 */
				if(!SubscribeMqtt(pMqttComm, pMqttComm->u8TRBuf)) {
					continue;
				}
			#if SOFT_RUN1_TEST0
				/* 订阅序列号与授权加密 */
				BOOL bSubSnLicReq = (g_PubSoftAuthCtr.uPubSnCnt || g_PubSoftAuthCtr.uPubLicCnt);
				if(bSubSnLicReq) {
					pTxBuf = pMqttComm->u8TRBuf;
					PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
					PrintStringNoOvChk(&pTxBuf, "/AUTH/#");
					*pTxBuf++ = 0;		/* 指针的结尾 */
					if(!SubscribeMqtt(pMqttComm, pMqttComm->u8TRBuf)) {
						continue;
					}
				}
				g_TcpIPComm.bSubSnLicReq = bSubSnLicReq;

				/* 订阅其他通讯盒的升级请求 */
				BOOL bSubSoftReq = (g_PubSoftAuthCtr.uPubSoftInstallCnt || g_PubSoftAuthCtr.uPubSoftUpdateCnt);
				if(bSubSoftReq) {
					pTxBuf = pMqttComm->u8TRBuf;
					PrintStringNoOvChk(&pTxBuf, DEV_TYPE_MQTT_TOPIC_String);
					PrintStringNoOvChk(&pTxBuf, "+/AUTH/");
					*pTxBuf++ = '0' + (SOFTWARE_VER/1000)%10;
					*pTxBuf++ = '0' + (SOFTWARE_VER/100)%10;
					*pTxBuf++ = '0' + (SOFTWARE_VER/10)%10;
					*pTxBuf++ = '0' + (SOFTWARE_VER/1)%10;
					*pTxBuf++ = 0;		/* 指针的结尾 */
					if(!SubscribeMqtt(pMqttComm, pMqttComm->u8TRBuf)) {
						continue;
					}
				}
				g_TcpIPComm.bSubSoftReq = bSubSoftReq;
			#endif

			#if (SUPPORT_RMT_SEN_TOPIC || REMOTE_DEBUG_TERMINAL)
				if(!CheckString(g_CommConf.u8MqttSubTopic, sizeof(g_CommConf.u8MqttSubTopic))) {
					g_CommConf.u8MqttSubTopic[0] = 0;	/* 防止字符串超长，后面比较出错 */
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
			/* 每12个小时进行一次网络对时: g_Sys.u32Seconds_LastSyncRTC 在SetRTCSeconds()调用时设置 */
			if((g_Sys.u32Seconds_LastSyncRTC == 0)
				|| (labs(GetRTCSeconds() - g_Sys.u32Seconds_LastSyncRTC) > 12*3600UL))
			{
				for(uint16 i = 0; i < MAX_NTP_SERVER_NUM; i++) {
					if(SyncRealTimeBySNTP(pMqttComm, cnst_u32NTPServers[i])) {
						break;
					}
				}
			}

			/* 检查参数是否发生变化 */
			if((pMqttComm->u32SerialNo != g_Sys.SerialNo.u32Dat)	/* 检查序列号是否发生变化, 如果发生变化，就需要重新连接 */
			#if (SUPPORT_RMT_SEN_TOPIC || REMOTE_DEBUG_TERMINAL)
				|| (strcmp((const char*)g_TcpIPComm.u8MqttSubTopic, (const char*)g_CommConf.u8MqttSubTopic) != 0)	/* Topic订阅 */
			#endif
			#if SOFT_RUN1_TEST0
				|| (g_TcpIPComm.bSubSnLicReq != (g_PubSoftAuthCtr.uPubSnCnt || g_PubSoftAuthCtr.uPubLicCnt))/* 母板功能 */
				|| (g_TcpIPComm.bSubSoftReq != (g_PubSoftAuthCtr.uPubSoftInstallCnt || g_PubSoftAuthCtr.uPubSoftUpdateCnt))
			#endif
			#if SUPPORT_V4_MASTER
				|| (!SubYKFTopic(pMqttComm))	/* YKF主题订阅是否成功 */
			#endif
				|| 0)
			{
				break;

			/* 循环处理可能同时发来的多个消息 */
			} else {
				g_CodeTest.u32Val[21]++;
				g_CodeTest.u32Val[49] = 1;
				/* 这里用if 防止RecvFromMqttServer将PING包的响应读取了 */
				while(RecvFromMqttServer(pMqttComm) && (MQTTParseMessageType(pMqttComm->u8TRBuf) == MQTT_MSG_PUBLISH)) {
					/* 消息解析:获得Topic, Msg的分段 */
					uint16 uTopicLen = mqtt_parse_pub_topic_ptr(pMqttComm->u8TRBuf, (const uint8_t**)&pU8Topic);	/* 获得Topic起始指针和数据长度 */
					pU8TopicEnd = pU8Topic + uTopicLen;
					uint16 uMsgLen = mqtt_parse_pub_msg_ptr(pMqttComm->u8TRBuf, (const uint8_t**)&pU8Msg);			/* 获得Msg起始指针和数据长度 */
					pU8MsgEnd = pU8Msg + uMsgLen;

					if(0) {	/* 这是为了下面的代码可以使用else if，显得整齐 */
				#if (SUPPORT_RMT_SEN_TOPIC)
					} else if(CompareTopic(pU8Topic, (const char*)g_TcpIPComm.u8MqttSubTopic)) { /* 远程传感器订阅的值 */
						pU8Topic = SkipCharInString(pU8Topic, pU8TopicEnd, '/', 2);
						if(CompareTopic(pU8Topic, "RMTS")) {
							GetRmtSenFromMqttComm(&g_TcpIPComm.RmtSenFromMqtt, pU8Msg, pU8MsgEnd);
						} else {
							ProcRmtDataFromMqtt(pU8Topic, pU8Msg, pU8MsgEnd);		/* 其他消息转由板子特有函数处理 */
						}
				#endif
					} else if(CompareTopic(pU8Topic, DEV_TYPE_MQTT_TOPIC_String)) {		/* 设备 */
						pU8Topic = SkipCharInString(pU8Topic, pU8TopicEnd, '/', 1); /* 跳到型号后 */
						u32TopicSN = atoi((const char*)pU8Topic);
						pU8Topic = SkipCharInString(pU8Topic, pU8TopicEnd, '/', 1);
						if(SOFT_RUN1_TEST0 && CompareTopic(pU8Topic, "AUTH/")) {	/* 母板部分:代码升级与授权 */
							pU8Topic += 5;
							if(CompareTopic(pU8Topic, "CMD")) {
								if(!ProcMqttAuth_CMD(pMqttComm, pU8Msg, pU8MsgEnd)) {
									break;
								}
							} else if(g_PubSoftAuthCtr.uPubSnCnt && CompareTopic(pU8Topic, "SN")) {	/* 如果是序列号更新请求 */
								if(!ProcMqttAuth_SN(pMqttComm, pU8Msg, pU8MsgEnd)) {
									break;
								}
							} else if(g_PubSoftAuthCtr.uPubLicCnt && CompareTopic(pU8Topic, "LICENSE")) {	/* 如果是授权请求 */
								if(!ProcMqttAuth_LIC(pMqttComm, pU8Msg, pU8MsgEnd)) {
									break;
								}
							} else if((g_PubSoftAuthCtr.uPubSoftInstallCnt || g_PubSoftAuthCtr.uPubSoftUpdateCnt) /* 如果是代码片请求 */
										&& (ReadU32(&pU8Topic, pU8TopicEnd) == SOFTWARE_VER) && (u32TopicSN >= 10000000UL))
							{
								if(!ProcMqttAuth_SOFT(pMqttComm, pU8Msg, pU8MsgEnd, u32TopicSN)) {
									break;
								}
							}
						} else if((u32TopicSN == g_Sys.SerialNo.u32Dat) && CompareTopic(pU8Topic, "SUB/")) {
							pU8Topic += 4;
							if(CompareTopic(pU8Topic, "UPDATE/")) {		        /* 如果是本机所需要的代码片 */
								if(!ProcMqttUpdate(pMqttComm, pU8Topic + 7, pU8TopicEnd, pU8Msg, pU8MsgEnd)) {
									break;
								}
                            } else if(CompareTopic(pU8Topic, "CMD")) {          /* 订阅到的内容为控制指令 */
                                if(!ProcMqttCmdInstr(pMqttComm, pU8Msg, pU8MsgEnd)) {
                                    break;
                                }
							} else if(CompareTopic(pU8Topic, "SET/")) {
								pU8Topic += 4;
								if(CompareTopic(pU8Topic, "RST_CODE_TEST_U32")) {	/* 临时性，方便调试 */
									InitDataWithZero((uint8*)g_CodeTest.u32Val, sizeof(g_CodeTest.u32Val));
								} else if(CompareTopic(pU8Topic, "MSGPT")) {
#if DEV
									RegisterPageReq(DATA_PAGE_MSG, pU8Msg, pU8MsgEnd);	/* ItemNum为0代表设置自动发布的指针 */
#endif
								}
							} else if(CompareTopic(pU8Topic, "REQ/")) {				/* 订阅到的内容为请求信息 */
								pU8Topic += 4;
								if(CompareTopic(pU8Topic, "DEVINFO")) {				/* 订阅到的内容为请求设备信息 */
									if(!PubDevInfo(u32MqttTaskNo)) {
										break;
									}
                                } else if(CompareTopic(pU8Topic, "SYNC")) {         /* 同步配置 */
#if DEV
                                    ProcMqttConfSync(pU8Msg, pU8MsgEnd);
#endif
								} else if(CompareTopic(pU8Topic, "CONF")) {			/* 配置页面 */
//#if DEV
									ProcMqttConfInstr(pU8Msg, pU8MsgEnd);
//#endif
                                } else if(CompareTopic(pU8Topic, "MSG")) {          /* MSG页面 */
#if DEV
                                    RegisterPageReq(DATA_PAGE_MSG, pU8Msg, pU8MsgEnd);
#endif
                                } else if(CompareTopic(pU8Topic, "ACQ")) {          /* ACQ页面 */
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
								} else if(CompareTopic(pU8Topic, "DEBUG_CHN")) { 	/* 请求DEBUG_CHN数据 */
#if DEV
									if(!PubDebugChn(pMqttComm, ReadI16(&pU8Msg, pU8MsgEnd))) {
										break;
									}
								} else if(!ProcMqttOtherReq(pMqttComm, pU8Topic, pU8Msg, pU8MsgEnd)) {
								    break;
#endif
								}
						#if SUPPORT_V4_MASTER
							} else if(CompareTopic(pU8Topic, "MODBUS")) {		/* 随机访问指令 */
								ProcMqttRandAcsReq(pU8Topic, pU8Msg, pU8MsgEnd);
						#endif
						#if SUPPORT_FLOW_METER
							} else if(CompareTopic(pU8Topic, "CAMERA")) {	/* 接收基于HTTP的摄像机指令 */
								/* 消息转发给下泄线程 */
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
					} else if(CompareTopic(pU8Topic, "YKF/")) {						/* YKF的数据 */
						ProcMqttMsgForYKF(pU8Topic, pU8Msg, pU8MsgEnd);
				#endif
					}
				}
//				Task_sleep(100);	/* 休眠一会给另一个线程一点机会 */
#if SUPPORT_GPRS && SUPPORT_ETHERNET
				if((GetCurrAccess() == INTERNET_ACCESS_GPRS) && CheckEthernet()) {
					break;		/* 如果以太网连接恢复则断开GPRS(以太网优先) */
				}
#endif
			}
		};

		/* 断开以待重连 */
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
		&& (pMqttComm->u8TRBuf[4] <= 2)) 	/* QoS标志 */
	{
		return TRUE;
	}

	/* 断开以待重连 */
	mqtt_disconnect(&pMqttComm->broker);
	pMqttComm->broker.pSocketFunc->Close(pMqttComm->MqttSocket);
	return FALSE;
}

/*==========================================================================
| Description	: ModbusTCP服务器创建。修改的是例程模板源代码。
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

	/* 初始化 */
	while(1) {
		/* 创建Server */
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
				|| (-1 == listen(i32Socket, 2))	/* 后备链接仅允许2个，备注：后备链接和最大连接是两码事 */
				|| (setsockopt(i32Socket, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0))
			{
				Task_sleep(OS_TICK_KHz*100);
				fdClose(i32Socket);
				continue;
			} else {
				break;
			}
		}

		/* 等待客户端连接:  这里是否要设置通讯超时？是需要独立设置，还是复制主会话的属性？ */
		int32 Clientfd;
		socklen_t AddrLen = sizeof(ClientAddr);
		while(((int32)(Clientfd = accept(i32Socket, (struct sockaddr *)&ClientAddr, &AddrLen))) > 0) {
			/* 设置通讯超时属性   */
			int32 flag = 1;
			struct timeval rcvTimeout;
			rcvTimeout.tv_sec = OS_TICK_KHz * 50;
			rcvTimeout.tv_usec = 0;
			struct timeval sndTimeout;
			sndTimeout.tv_sec = OS_TICK_KHz * 5;
			sndTimeout.tv_usec = 0;
			if((setsockopt(Clientfd, IPPROTO_TCP, TCP_NODELAY, (int8*)&flag, sizeof(flag)) < 0)		/* 纳格尔禁用算法 */
				|| (setsockopt(Clientfd, SOL_SOCKET, SO_RCVTIMEO, &rcvTimeout, sizeof(rcvTimeout)) < 0)	/* 设置接收超时 */
				|| (setsockopt(Clientfd, SOL_SOCKET, SO_SNDTIMEO, &sndTimeout, sizeof(sndTimeout)) < 0))	/* 设置发送超时 */
			{
				fdClose(Clientfd);
				continue;
			}

			/* 查找空余的任务空间 */
			int16 i;
			for(i = 0; i < MODBUS_TCP_INT_TASK_NUM; i++) {
				if(g_ModbusTcpComm[i].bFree) {
					g_ModbusTcpComm[i].bFree = FALSE;
					break;
				}
			}

			if(i == MODBUS_TCP_INT_TASK_NUM) {	/* 任务控制块都被占了 */
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
| Description	: ModbusTCP数据处理：修改的是例程模板源代码。
					1、接受ModbusTcp协议，转换成ModbusRtu协议。
					2、把机组返回的ModbusRtu协议，转换成ModbusTcp协议，发送出去。
					3、接受写入指令，处理和响应。
					4、为什么要区分RespoundModbusTCP0A(0A号机组数据) / RespoundModbusTCP(非0A号机组数据)?
					组态界面 0A机组发过来的查询指令是个范围比如 0A0426480016+CRC 有长度。
					而除了0A机组其他的都机组发过来的查询指令就单个地址的长度 ，比如 140426480001+CRC 140426490001 +CRC。
					发送过来的命令不同。
| G/Out var		:
| Author		: king			Date	: 2017-8-31
\=========================================================================*/
void ModbusTcpTask(void const *pArgs)
{
	MODBUS_TCP_COMM* pModbusTcpComm;
	int32 i32Clientfd;
	uint16 uModbusTcpTaskNo;

	/* 初始化 */
	uModbusTcpTaskNo = (uint16)((uint32*)pArgs)[0];
	i32Clientfd = (int32)((uint32*)pArgs)[1];
	pModbusTcpComm = &g_ModbusTcpComm[uModbusTcpTaskNo];

    while(1) {
		/* 接收消息，并成帧 */
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

		/* 处理消息 */
	#if(SUPPORT_V4_MASTER || SUPPORT_V5_MASTER)		/* 对于Master类设备，需要做地址检查，以确定访问对象 */
		if(pModbusTcpComm->u8TRBuf[6] == 0xFF) {
			RespondModbus(MAX_UART_NUM + uModbusTcpTaskNo, 0xFF);
		} else {
			RespondModbusTCPForOthers(pModbusTcpComm, TRUE);
		}
	#else	/* 其他设备不做地址检查，因为只能访问本机 */
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
| Description	: TCP/IP通讯初始化模块，由于App.cfg自动生成的代码有如下不足：
	1. DHCP服务不能根据配置值来决定是否运行
	2. 不能支持部分接受DHCP的结果
	3. DNS消耗内存太多，且需要动态内存分配，有隐患
	NetCommTask()是入口函数，由App.cfg生成的App_pem4f.c任务ti_ndk_config_Global_stackThread()调用
| G/Out var 	:
| Author		:					Date	:
\=========================================================================*/

void NetCommTask(void const * pArgs)
{
	/* 关键变量初始化 */
//	g_TcpIPComm.u32LocalIP = 0;
//	g_TcpIPComm.uTmr_MqttPubRun_ms = 0;
#if (SUPPORT_RMT_SEN_TOPIC || REMOTE_DEBUG_TERMINAL)
	g_TcpIPComm.RmtSenFromMqtt.u32RcvTime_RTCSeconds = 0;
	g_TcpIPComm.u8MqttSubTopic[0] = 0;
#endif
	int8 i;
	for(i = 0; i < MQTT_TASK_NUM; i++) {
		g_MqttComm[i].i32MqttSocket = -1;		/* 之前TI是NULL，根据代码看，-1也可以，bsd socket接口的socket类型本来就是int，ti需要修改指针为int */
//		g_MqttComm[i].u8ServerNo = 0;
//		g_MqttComm[i].u8ConnRes = 0;
//		g_MqttComm[i].u8DnsAndTcpConRes = 0;
//		g_MqttComm[i].u8Count_ConnTry = 0;
//		g_MqttComm[i].u32TotalConnCnt = 0;
//		g_MqttComm[i].u32ConnDuration_ms = 0;
//		g_MqttComm[i].u32DNSDuration_ms = 0;
	}
}

/* 网络配置变化回调 */
void DrvLinkIndicator(int i32LinkST_On1_Off0)
{
	if(i32LinkST_On1_Off0 && g_TcpIPComm.u32LocalIP) {
		ON_INDICATOR_NET;
	} else {
#if CPU_0ST_1TI
		/* 物理线路断开就重启DHCP服务器 */
		StartTI_DHCP();
#endif
		OFF_INDICATOR_NET;
	}
}

/* 摘自dnsclnt.c，以减少内存使用，并且原来这个函数是static的，不能被外部引用。并将DNSResolveQuery()改名成DNSQuery()
 * 另外，拷贝的时候，删除了对IPv6的支持
 * 将绝大部分堆改成了栈，并减少了不必要的拷贝，
 * 引入外部buf，减少了对栈的消耗，增加了uBufLen以标识最大buf使用长度，
 * 增加对请求类型的匹配，删掉了非请求类型的多个响应字段链，从而减少了内存使用
 * pQueryBuf大小必须大于DNS_TOTAL_BUF_LEN(1318) = DNS_DOMAIN_LEN(50) + DNS_SCRAP_MAX_LEN(268) + MAX_DNS_PACKET(1000) */
int DNSResolve(uint8 *pQueryBuf, uint16 uBufLen, uint8 af_family, uint16 uReqType)
{
    DNSREC      *pQuery_o_Replay = (DNSREC*)(pQueryBuf + DNS_DOMAIN_MAX_LEN);		/* 原pScrapBuf*/
    uint8		*pPktBuf = pQueryBuf + DNS_DOMAIN_MAX_LEN + DNS_SCRAP_MAX_LEN;
    DNSREC 		*pRec;
    DNSREPLY    Reply;		/* pReply-> Reply堆改栈*/
    HOSTENT     *phe;
    UINT8       *pbWrite;
    int         rc, writeused;
    IPN         IPTmp;

    /* validate that query str passed does not exceed DNS record name length，
       	   增加了(size < sizeof(DNSREC))，原来DNSREC是用mmAloc分配，这里复用pScrapBuf */
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

/* Sync system datetime from source, currentlly is SNTP,
 * will block until time synchronized from source or tried all known servers  */
BOOL SyncRealTimeBySNTP(MQTT_COMM* pMqttComm, const uint32 cnst_u32NTPSever)
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
			struct sockaddr_in socket_address;
			socket_address.sin_addr.s_addr = htonl(cnst_u32NTPSever);
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
					&& (pSocketFun->Recv(i32SocketSNTP, (uint8 *)&sntpPkt, sizeof(SNTPHeader), 20) == sizeof(SNTPHeader))
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
					pSocketFun->Close(i32SocketSNTP);
					return TRUE;
				}
			}
		}
		pSocketFun->Close(i32SocketSNTP);
	}
	return FALSE;
}
/******************************** FILE END ********************************/
