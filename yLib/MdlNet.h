/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved
 * File			: MdlNet.h
 * Author		: Meng Yaoyao
 * Description	:
 *				:
 * Comments		:
 * Date			: 2017-5-31
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 |
 **************************************************************************/

/***************************************************************************
 				exclude redefinition and c++ environment
***************************************************************************/
#ifndef _MDL_NET_H_
#define _MDL_NET_H_

#ifdef __cplusplus
extern "C" {
#endif


/***************************************************************************
 							include files
***************************************************************************/
#include "GlobalVar.h"
#if SUPPORT_GPRS
#include "MdlGprs.h"
#endif
#include "libemqtt.h"


/***************************************************************************
				hardware parameter and global definitions
***************************************************************************/


/***************************************************************************
 		use EXT to distinguish stencil.c or others source files
***************************************************************************/
#ifdef _MDL_NET_C_
#define EXT
#else
#define EXT extern
#endif


/***************************************************************************
 					variables could be used as extern
***************************************************************************/
EXT SEM_Handle SEM_MqttPubReq;

#define MODUBS_TCP_SERVER_STACK 1024					/* ModbusTCP任务堆栈大小 */
typedef struct {
	uint32 u32LocalIP;
	uint16 uTmr_MqttPubRun_ms;
	BOOL bSubSnLicReq;
	BOOL bSubSoftReq;
#if SUPPORT_MODBUS_TCP
	uint32 u32ModbusTCPServerStack[MODUBS_TCP_SERVER_STACK/4];	/* ModbusTCP Server任务堆栈 */
#endif
#if (SUPPORT_RMT_SEN_TOPIC || REMOTE_DEBUG_TERMINAL)
	RMT_SEN_DAT RmtSenFromMqtt; 				/* 远程传感器数据 */
	uint8 u8MqttSubTopic[CONF_ANSI_BYTE_LEN];	/* 订阅后的主题	*/
#endif
}TCPIP_COMM;
EXT SECTION(".NOT_ZeroInit") TCPIP_COMM g_TcpIPComm;		/* TODO：新改爲存 NOT_ZeroInit ，需要測試*/

typedef enum {
	ETHERNET	= 0,
	GPRS
}INTERNET_ACCESS_TYPE;

#define MODBUS_TCP_TASK_STACK	1024
#define MQTT_TASK_STACK 		2048			/* MQTT堆栈大小 */
#define MQTT_TR_BUF_BLEN		2048			/* MQTT收发消息长度 */
#define MQTT_TR_BASE64_BLEN		((MQTT_TR_BUF_BLEN - 128)*3/4)	/* MQTT以base64编码收发数据长度 */
/* 为了GPRS单增的结构体： */
typedef struct {
	BOOL bFlag;		/* 是否需要滞后接收PING包的响应标志位 */
	uint8 u8Header;	/* 其他类型的报头 */
	uint8 u8LessLen;	/* 剩余长度 */
}GPRS_MqttNewAdd;
typedef struct {
	uint32 u32SerialNo;
	SOCKET MqttSocket;
	uint8 u8TRBuf[MQTT_TR_BUF_BLEN];			/* 由于Mqtt数据收发是串行进行，因此收发可以共用一个Buf */
	uint32 u32TaskStack[MQTT_TASK_STACK/4];
	mqtt_broker_handle_t broker;
	uint8 u8ServerNo;
	/* 连接结果   ，发布的时候，u8ConnRes,u8DnsAndTcpConRes组合成一个16b的数据 */
	uint8 u8ConnRes;			/* 连接退出位置 */
	uint8 u8DnsAndTcpConRes;	/* b5~4,b3~2,b1~0:三个服务器dns+tcp连接情况--0未尝试,1dns失败,2dns成功tcp失败,3tcp成功 */
	uint8 u8Count_ConnTry;		/* 连接尝试次数 */
	BOOL bNeedReConn;
	uint16 uRsvd;
	GPRS_MqttNewAdd GprsNewAdd;			/* GPRS新增, 在PingMqttServer函数中有解释 */
}MQTT_COMM;
typedef enum {
	MQTT_TYPE_PUB		= 0,
	MQTT_TYPE_SUB,
	MQTT_TASK_NUM					/* MQTT连接类型，支持两个发布、两个订阅任务，新建此结构是为避免在非必要暴露MdlNet的地方引用MdlNet.h */
}MQTT_CONN_TYPE;
EXT MQTT_COMM g_MqttComm[MQTT_TASK_NUM];

#ifdef _MDL_NET_C_
	const SocketFunction AllSocketFun[MAX_INTERNET_ACCESS_NUM] = {
		#if SUPPORT_ETHERNET
				{socket, connect, send, recv, fdClose, setsockopt, DnsQuery},
		#endif
		#if SUPPORT_GPRS
				{GprsSocket, GprsConnect, GprsSend, GprsRecv, GprsClose, GprsSetSockopt, GprsDnsQuery}
		#endif
	};
#else
	extern const SocketFunction AllSocketFun[MAX_INTERNET_ACCESS_NUM];
#endif

#define DNS_DOMAIN_MAX_LEN		50					/* 域名不得超过这个长度 */
#define DNS_SCRAP_MAX_LEN		(sizeof(DNSREC))
#define DNS_PKTBUF_MAX_LEN		MAX_DNS_PACKET
#define DNS_TOTAL_BUF_LEN		(DNS_DOMAIN_MAX_LEN + DNS_SCRAP_MAX_LEN + DNS_PKTBUF_MAX_LEN)
/*===========================================================================
 * Input Var
 *==========================================================================*/
/* <NULL> */

/*===========================================================================
 * Output Var
 *==========================================================================*/
/* <NULL> */

/*---------------------------------------------------------------------------
 * DebugRec
 *--------------------------------------------------------------------------*/


/***************************************************************************
 					functions must be driven by extern
***************************************************************************/
EXT void DrvNetTick_1KHz(void);


/***************************************************************************
 					functions could be used as extern
***************************************************************************/
EXT int DNSResolve(uint8 *pQueryBuf, uint16 uBufLen, uint8 af_family, uint16 uReqType);
EXT	BOOL PublishMqtt(MQTT_COMM* pMqttComm, uint8* pTxBuf, uint16 uMsgIDPt, uint8 u8QoS);
EXT BOOL WaitAndChkMqttEcho(MQTT_COMM* pMqttComm, uint16 uMsgId, uint8 MqttEchoType);
EXT BOOL SubscribeMqtt(MQTT_COMM* pMqttComm, uint8* pU8Topic);
EXT void PrintMqttConnResToJson(uint8** ppTxBuf);
/************   exclude redefinition and c++ environment   ****************/
#ifdef __cplusplus
}
#endif 					/* extern "C" */

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
/******************************** FILE END ********************************/

