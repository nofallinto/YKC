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
#define DEBUG_GPRS_UART_INFO	TRUE		/* 打开GPRS通信时的调试信息 */
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

#define MAX_GPRS_SOCKET_NUM			6

typedef enum {
	GPRS_SUC 			= 0,
	GPRS_ERR_TIMEOUT	= -1,
	GPRS_ERR_ARG		= -2,
	GPRS_ERR_OFFLINE	= -3,
	GPRS_ERR_PARSE		= -4,
	GPRS_ERR_UART_ABN	= -5,
	GPRS_ERR_CONNECTION_ABN	= -6,
	GPRS_ERR_NO_DATA	= -7
}GPRS_RES;

typedef enum {
	GPRS_NORMAL 	= 1,
	GPRS_NOT_INIT	= 0,
	GPRS_NO_SIM		= -1,
	GPRS_INVALID	= -2,
	GPRS_COMM_FAIL	= -3,
	GPRS_PIN_LOCK	= -4,
	GPRS_ERROR 		= -5,
	GPRS_NO_INTERNET = -6,
}GPRS_STATUS;

//#define GPRS_SEND_TIMEOUT_s	5			/* 秒 */
//#define GPRS_RECV_TIMEOUT_s	5			/* 秒 */

#define GPRS_REQ_TIMEOUT_ms	10000			/* 毫秒 */

typedef struct {
	uint32				protocl;
	uint32				pTRBufMaxLen;
	GPRS_RES			u8Res;			/* gprs结果*/
	BOOL 				bIsFree;
	uint8 				u8MuxId;
	struct sockaddr* 	name;
	const char*			pServer;
}GPRS_SOCKET;

typedef struct {
	GPRS_STATUS 				GprsStatus;
	SEM_Handle					Sem_GprsReq;		/* Gprs同步信号量 */
	osStaticSemaphoreDef_t 		SemCtlBlk;
	GPRS_SOCKET 				Socket[MAX_GPRS_SOCKET_NUM];
}GPRS_COMM;
EXT GPRS_COMM g_GprsComm;		/* 不能放Not_ZeroInit */

#define RING_BUFFER_MAX_SIZE		1024		/* 环形缓冲区大小, 要足够大. 否则如果不及时读会覆盖旧数据导致数据错乱 */
typedef struct {	/* 用于保存从GPRS接收的数据的环形缓冲区 */
    uint8 aU8Buffer[RING_BUFFER_MAX_SIZE];    	/* 缓冲区指针 */
    uint16 uHead;      /* 头指针 */
    uint16 uRear;      /* 尾指针 */
}RingBuffer_t;
EXT RingBuffer_t g_GPRSRingBuffer;	/* GPRS的环形队列 */

/*---------------环形缓冲区相关函数声明---------------*/
void RingBuffer_Init(RingBuffer_t *pRingBuffer);		/* 初始化 */
void RingBuffer_Write(RingBuffer_t *pRingBuffer, const uint8 *cnst_pU8Data, uint16 uLen);	/* 写数据, 缓冲区满了会覆盖旧数据 */
uint16 RingBuffer_Read(RingBuffer_t *pRingBuffer, uint8 *pU8Data, uint16 uReadLen);			/* 读取数据, 如果想读所有数据直接传RING_BUFFER_MAX_SIZE */
/*---------------end---------------*/

/*---------------GPRS相关函数声明---------------*/
BOOL GPRS_WaitSocketIdle(SOCKET SocketId, uint32 u32MaxNAcked, uint8 u8MaxStuckCnt, uint16 uTotalTimeoutMs);	/* 等待指定socket发送完成 */
uint16 GPRS_GetUnsentBLen(SOCKET SocketId);						/* 获取指定socket未发送的字节数 */
uint16 StrRemoveHashData(uint8 *pSrc, uint16 uDataLen);			/* 去除掉无用的数据 */
uint8 GPRS_GetSigStrong(void);									/* 获取GPRS信号强度 */
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
