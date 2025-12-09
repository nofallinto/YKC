/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved.	模板V1.4
 *				:
 * File			:
 * Author		: Cui
 * Description	:

 * Date			:
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * V1.0	| 2017-3-24	  |	king |
 **************************************************************************/

/***************************************************************************
 				exclude redefinition and c++ environment
***************************************************************************/

#ifndef GPRS_H_
#define GPRS_H_

#include "GlobalVar.h"
#include "MdlNet.h"

#ifdef _GPRS_C_
#define EXT
#else
#define EXT extern
#endif

#if !SUPPORT_ETHERNET

/*------------------调试使用------------------*/
#define DEBUG_GPRS_UART_INFO		FALSE		/* 打开GPRS通信时的调试信息 */
#define USE_GPRS_02G_14G			1			/* 使用GPRS模块类型 0：2G，1：4G */
#define GPRS_SIMCARD_OPERATOR_APN	"CMNET"		/* 运营商的APN, 移动：CMNET, 联通：3gnet/uninet, 电信：CTNET */
#define GPRS_CONTEXT_TYPE			1			/* 协议类型, 1：IPv4, 2：IPv6, 3：IPv4v6 */
#define GPRS_DEFAULT_UART_BAUD		115200		/* GPRS模块串口波特率 */
/*------------------end------------------*/

struct in_addr {
    uint32  s_addr;             /* 32 bit long IP address, net order */
};

typedef struct sockaddr_in {
    uint16 	sin_family;         /* address family */
    uint16  sin_port;           /* port */
    struct  in_addr sin_addr;
    int8    sin_zero[8];        /* fixed length address value */
}SOCKADDR_IN;

typedef struct {
	char	*h_name;
	int32	h_addrtype;
	int32	h_length;
	int32	h_addrcnt;
	uint32	h_addr[8];
}HOSTENT;

#define AF_INET						0
#define SOCK_STREAM 				0		/* socket() option */
#define SOCK_DGRAM					1		/* socket() option */
#define IPPROTO_UDP					1		/* SOCK_OPTION category*/
#define IPPROTO_TCP					2		/* SOCK_OPTION category*/
#define SOL_SOCKET					0		/* SOCK_OPTION category*/
#define TCP_NODELAY					0		/* SOCK_OPTION value*/
#define SO_RCVTIMEO					1		/* SOCK_OPTION value*/
#define SO_SNDTIMEO					2		/* SOCK_OPTION value*/
#define MSG_WAITALL					0		/* revc option */

///* Used by fdSelect() */
//struct timeval {
//    int32 tv_sec;
//    int32 tv_usec;
//};
#include <sys/_timeval.h>

#define htons(val) __rev16(val)
#define htonl(val) __rev(val)
#define ntohl(val) __rev(val)
#endif

#define GPRS_UART_PORT				2			/* 模块端口*/

#define MAX_GPRS_SOCKET_NUM			6			/* 2G模块6个, 4G模块12个 */

typedef enum {
	GPRS_SUC 			= 0,
	GPRS_ERR_TIMEOUT	= -1,			/* 拿锁超时 */
	GPRS_ERR_ARG		= -2,			/* 参数错误 */
	GPRS_ERR_OFFLINE	= -3,			/* GPRS掉线 */
	GPRS_ERR_PARSE		= -4,			/* 无法解析 */
	GPRS_ERR_UART_ABN	= -5,			/* 串口错误 */
	GPRS_ERR_CONNECTION_ABN	= -6,		/* 连接错误 */
	GPRS_ERR_NO_RESP	= -7,			/* 设备无响应, 就是发送指令什么都没收到 */
	GPRS_RES_ERROR 			= -8,		/* 收到了ERROR响应 */
	GPRS_RES_FAIL			= -9,		/* 收到了FAIL响应 */
	GPRS_NO_EX_RESP		= -10,			/* 没有收到期望的响应 */
	GPRS_NO_EX_END		= -11,			/* 没收到期望的结束符 */
}GPRS_RES;

typedef enum {
	GPRS_NORMAL 	= 0,		/* 正常 */
	GPRS_NOT_INIT	= -1,		/* 未初始化 */
	GPRS_NO_SIM		= -2,		/* 没有SIM卡 */
	GPRS_NO_INTERNET = -3,		/* 无法上网 */
	GPRS_UART_FAILD = -4,		/* 串口通信失败 */
}GPRS_STATUS;

//#define GPRS_SEND_TIMEOUT_s	5			/* 秒 */
//#define GPRS_RECV_TIMEOUT_s	5			/* 秒 */

#define GPRS_REQ_TIMEOUT_ms	10000			/* 毫秒 */

typedef struct {
	uint32				protocl;
	uint32				pTRBufMaxLen;
	GPRS_RES			u8Res;			/* gprs结果 */
	BOOL				bIsConnect;		/* 是否在连接中 */
	BOOL 				bIsFree;
	uint8 				u8MuxId;
	struct sockaddr* 	name;
	const char*			pServer;
	uint16 uSendTimeoutMs;		/* 发送超时时间 */
	uint16 uRecvTimeoutMs;		/* 接收超时时间(这个超时时间仅仅是规定时间内没有接收到数据才会返回. 如果接收到数据会一直接收, 即使超时也会继续接收.) */
	uint32 u32UnreadBLen;		/* 未读取的长度 */
	uint32 u32LastQirdiIndex;	/* 最近收到的QIRDI编号初始化成0, 编号从1开始 */
}GPRS_SOCKET;

typedef struct {
	GPRS_STATUS 				GprsStatus;
	SEM_Handle					Sem_GprsReq;		/* Gprs同步信号量 */
	osStaticSemaphoreDef_t 		SemCtlBlk;
	GPRS_SOCKET 				Socket[MAX_GPRS_SOCKET_NUM];
}GPRS_COMM;
EXT GPRS_COMM g_GprsComm;		/* 不能放Not_ZeroInit */

///*---------------环形缓冲区相关函数声明---------------*/
///*---------------end---------------*/

/*---------------GPRS相关函数声明---------------*/
BOOL GPRS_WaitSocketIdle(SOCKET SocketId, uint32 u32MaxNAcked, uint8 u8MaxStuckCnt, uint16 uTotalTimeoutMs);	/* 等待指定socket发送完成 */
uint16 GPRS_GetUnsentBLen(SOCKET SocketId);						/* 获取指定socket未发送的字节数 */
//uint16 StrRemoveHashData(uint8 *pSrc, uint16 uDataLen);			/* 去除掉无用的数据 */
uint16 StrRemoveHashData(uint8 *pSrc, uint16 uHead, uint16 uRear, uint16 uRingBuffLen);
uint8 GPRS_GetSigStrong(void);									/* 获取GPRS信号强度 */
void DMA_ReStart(DMA_HandleTypeDef *hdma);						/* 重新启动DMA */
/*---------------end---------------*/

/*---------------GPRS 网络通信相关函数---------------*/
EXT SOCKET GprsSocket(int32 i32Domain, int32 i32Type, int32 i32Protocl);
EXT int32 GprsConnect(SOCKET socketFd, struct sockaddr_in* pName, int32 i32Len);
EXT int32 GprsRecv(SOCKET socketFd, uint8 *pDstBuf, int32 iReadBLen, int32 flags);
EXT int32 GprsSend(SOCKET socketFd, uint8 *pSrcBuf, uint16 uSendBlen, uint32 u32Flag);
EXT int32 GprsClose(SOCKET socketFd);
EXT int32 GprsSetSockopt(SOCKET socketFd, int32 u32Level, int32 u32Op, void *pbuf, int32 i32Bufsize);
EXT BOOL GprsDnsQuery(uint8* domain, uint32 *pU32IpResolved);
EXT void SyncRealTimeByGPRS(void);
/*---------------end---------------*/

#endif /* GPRS_H_ */
