/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved. 模板V1.3
 *
 * File			: Gprs.c
 * Author		: Cui
 * Description	:
 * Date			: 2024-01-29
 *
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 |
 **************************************************************************/

/***************************************************************************
 						macro definition
***************************************************************************/
#define _GPRS_C_		/* exclude redefinition */

/***************************************************************************
 						include files
***************************************************************************/
/* 项目公共头文件 */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"

/* 操作系统头文件 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/* 本级以及下级模块头文件 */
#include "MdlUARTnModbus.h"
#include "MdlGprs.h"
/***************************************************************************
 						global variables definition
***************************************************************************/
extern TSK_Handle MqttSubTaskHandle;
GPRS_COMM g_GprsComm;
RingBuffer_t g_GPRSRingBuffer;
/***************************************************************************
						internal functions declaration
***************************************************************************/
uint8* SkipStrInString(uint8* pU8Msg, uint8* pU8MsgEnd, uint8* u8String);

/***************************************************************************
 							functions definition
***************************************************************************/
/*==========================================================================
| Description	: 从字符串中查找特征字符串，并返回特征字符串末尾指针。
| In/Out/G var	:
| Author		: Su Mingyang			Date	: 2022-10-19
\=========================================================================*/
uint8* SkipStrInString(uint8* pU8Msg, uint8* pU8MsgEnd, uint8* u8String)
{
	int32 i = pU8MsgEnd - pU8Msg;
    uint8* pU8FirstCharIn = pU8Msg;
    uint8* pU8DesStrHead = u8String;
	for( ; i > 0; i--) {
        pU8FirstCharIn = pU8Msg;
        if ((*pU8Msg) && (*pU8FirstCharIn == *pU8DesStrHead)) {
            u8String = pU8DesStrHead;
            for (int j = 0; (j < i) && (*u8String) && (*u8String == *pU8FirstCharIn); j++) {
                u8String++;
                pU8FirstCharIn ++;
            }
            if (*u8String == 0) {
                return pU8FirstCharIn;
            }
        }
        pU8Msg++;
    }
	return NULL;
}

/*==========================================================================
| Description	: Gprs M25移远模块通信
| G/Out var		:
| Author		: Su Mingyang			Date	: 2024-01-29
\=========================================================================*/
#define MuxId(skt)	((((GPRS_SOCKET*)skt) - &g_GprsComm) / sizeof(GPRS_COMM))
#define GPRS_UART_TIMEOUT	500
#define MAX_GPRS_RETRY_CNT	5		/* 有时需要多次通信，单次最大尝试次数 */
/* 检查GPRS基础状态，如果不正常先尝试复位重新初始化，如果还是失败则此次不再尝试
 * 此函数不负责同步，同步需由外部处理
 * 返回GPRS基础状态是否正常
 * 务必断开stlink的3。3v线 */

/* 发送AT指令并判断响应是否正确, 并且可以指定结束符, 当没收到结束符时会再次尝试接收, 直到收到结束符认为一帧数据接收完成.使用参5指定尝试次数.
 * 参1: 要发送的AT指令, 可以为NULL, 为NULL则直接读响应是否符合, 不会发指令.
 * 		注: 指令为字符串, 因此必须以\0结尾, 并且要包括指令的结束符\r
 * 参2: 期望的响应, 当一次没有读到并且参5指定了多次尝试, 就会再次发送此指令再次读, 注意与参3区别开.可以为NULL, 为NULL时只要读到数据就返回TRUE, 读不到返回FALSE.
 * 参3: 结束符, 有时一帧数据是分两次发送, 因此需要接收两次, 直到接收到指定的字符串视为接受完一帧 (当通信过于频繁时, 大部分过长的响应都会分两次发送).
 * 	注: 尝试次数必须大于等于2, 因为如果一帧没接收完, 需要再次接收, 并且指令只会发送一次.
 * 		可以为NULL, 为NULL时读到一帧数据就返回了.
 * 参4: 接收的响应字节长度, 如果需要获取响应的长度, 就传参, 不需要传NULL即可
 * 参5: 尝试次数.
 * 参6: 超时时间, 注意是单次读取的超时时间 */
BOOL GPRS_SendATCmd(const char *cnst_pChCmd, const char *cnst_pChExpectResp, const char *cnst_pChEndStr, uint16 *uRxBufLen, uint8 u8TryCnt, uint16 uTimeoutMs)
{
    uint8* pStart = g_UartComm[GPRS_UART_PORT].u8TRBuf;
	uint8* pBuf;
    uint16 uRxLen = 0;		/* 接收到的数据长度 */
    BOOL bRet = FALSE;		/* 返回值 */
    BOOL bFlag = TRUE;		/* 是否需要重新发送命令的标志 */
	while(u8TryCnt--) {
		if(cnst_pChCmd && bFlag) {	/* 需要发送指令 */
			pBuf = pStart;
			/* 填充AT指令 */
			PrintString(&pBuf, pBuf + 20, cnst_pChCmd);	/* 注意这个指令的长度不是可变的, 根据需要调整, 也可以直接设置为最大. */
			/* 发送命令 */
			WriteToUart(GPRS_UART_PORT, pBuf - pStart);
		}
		/* 读取响应 */
		if((uRxLen = ReadFromUart(GPRS_UART_PORT, uTimeoutMs))) {
			if(cnst_pChEndStr == NULL) {	/* 没有结束标记 */
				if((cnst_pChExpectResp == NULL) || SkipStrInString(pStart, pStart + uRxLen, (uint8 *)cnst_pChExpectResp)) {
					bRet = TRUE;
					break;
				}
			} else {	/* 传入了结束标记 */
				bFlag = FALSE;	/* 不用再次发送指令了 */
				uRxLen += g_UartComm[GPRS_UART_PORT].uRxFrameIndex;
				if(SkipStrInString(pStart, pStart + uRxLen, (uint8 *)cnst_pChEndStr)) {	/* 收到了结束标记 */
					if((cnst_pChExpectResp == NULL) || SkipStrInString(pStart, pStart + uRxLen, (uint8 *)cnst_pChExpectResp)) {
						bRet = TRUE;
					} else {	/* 这里是收到了结束标记, 但是期望响应不对, 返回false */
						bRet = FALSE;
					}
					break;
				} else {	/* 没收到结束标记, 继续接收. 这里记录一下索引值, 继续接收时从该处继续接收. */
					g_UartComm[GPRS_UART_PORT].uRxFrameIndex = uRxLen;
				}
			}
		} else {
			uRxLen += g_UartComm[GPRS_UART_PORT].uRxFrameIndex;
		}
	}
	if(uRxBufLen != NULL) {
		*uRxBufLen = uRxLen;
	}
	g_UartComm[GPRS_UART_PORT].uRxFrameIndex = 0;
    return bRet;  /* 超时或未收到期望响应 */
}

/* 清空缓冲区, 因为使用环形缓冲区接收的数据, 频发发送AT指令可能会导致消息错位
 * 因此每次初始化的时候最好清空一下缓冲区 */
void GPRS_ClearBuffer(void)
{
	RingBuffer_Init(&g_GPRSRingBuffer);
}

/* 获取GPRS信号强度 */
uint8 GPRS_GetSigStrong(void)
{
	uint16 uRxLen = 0;
	uint8 *pU8Buf;
	if(GPRS_SendATCmd("AT+CSQ\r", "+CSQ:", "OK", &uRxLen, 2, 1000)) {
		if((pU8Buf = SkipStrInString(g_UartComm[GPRS_UART_PORT].u8TRBuf, g_UartComm[GPRS_UART_PORT].u8TRBuf + uRxLen, (uint8 *)"CSQ:"))) {
			return ReadU32(&pU8Buf, pU8Buf + 3);
		}
	}
	return 0;
}

/* 修改GPRS波特率 */
BOOL GPRS_SetBaud(uint32 u32Baud)
{
	char aChCmd[30];
	uint8 *pU8Buf = (uint8 *)aChCmd;
	uint32 u32LastBaud = g_UartComm[GPRS_UART_PORT].Handle->Init.BaudRate;
	PrintString(&pU8Buf, pU8Buf + 10, "AT+IPR=");
	PrintU32(&pU8Buf, pU8Buf + 12, u32Baud, 0);
	*pU8Buf++ = '\r';
	*pU8Buf++ = '\0';
	/* 设置GPRS的波特率 应该是没响应的, 通过发送AT指令查看是否配置成功 */
	GPRS_SendATCmd(aChCmd, NULL, NULL, NULL, 1, 100);

	OpenUartComm(GPRS_UART_PORT, u32Baud, 0, 30);	/* 重新打开串口 */
	if(GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 2, 100)) {	/* 是否响应 */
		return TRUE;
	}
	OpenUartComm(GPRS_UART_PORT, u32LastBaud, 0, 30);	/* 配置失败, 恢复串口波特率 */
	return FALSE;
}

/* 获取SIM卡网络状态 */
BOOL GPRS_GetInternetState(void)
{
	return GPRS_SendATCmd("AT+CREG?\r", "+CREG: 0,1", "\r\n\r\nOK", NULL, 2, GPRS_UART_TIMEOUT);
}

/* 开启网络附着 */
BOOL GPRS_OpenInternet(void)
{
	return (GPRS_SendATCmd("AT+CGATT=1\r", "OK", NULL, NULL, 1, 2000) &&
	GPRS_SendATCmd("AT+CGATT?\r", "+CGATT: 1", "\r\n\r\nOK", NULL, 2, 2000));
}

/* 获取SIM卡状态 只是在线或未在线 */
BOOL GPRS_GetSIMCardState(void)
{
	return GPRS_SendATCmd("AT+CPIN?\r", "READY", "\r\n\r\nOK", NULL, 2, 2000);
}

/* TCP通信相关配置 */
BOOL GPRS_TcpConf(void)
{
	BOOL bRet = TRUE;
	if(!GPRS_SendATCmd("AT+QIMUX?\r", "+QIMUX: 1", "\r\n\r\nOK", NULL, 2, GPRS_UART_TIMEOUT)) {	/* 如果不是多连接就配置成多连接 */
		bRet = GPRS_SendATCmd("AT+QIMUX=1\r", "OK", NULL, NULL, MAX_GPRS_RETRY_CNT, GPRS_UART_TIMEOUT);
	}
	if(!GPRS_SendATCmd("AT+QIDNSIP?\r", "+QIDNSIP: 1", "\r\n\r\nOK", NULL, 1, GPRS_UART_TIMEOUT)) {	/* 如果不是按域名连接 */
		bRet &= GPRS_SendATCmd("AT+QIDNSIP=1\r", "OK", NULL, NULL, MAX_GPRS_RETRY_CNT, GPRS_UART_TIMEOUT);
	}
	if(!GPRS_SendATCmd("AT+QINDI?\r", "+QINDI: 2", "\r\n\r\nOK", NULL, 1, GPRS_UART_TIMEOUT)) {	/* 如果不是缓存模式2 */
		bRet &= GPRS_SendATCmd("AT+QINDI=2\r", "OK", NULL, NULL, MAX_GPRS_RETRY_CNT, GPRS_UART_TIMEOUT);
	}
	return bRet;
}

/* 软件复位GPRS */
BOOL GPRS_Reset(void)
{
	if(GPRS_SendATCmd("AT+CFUN=0,1\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)) {
		Task_sleep(2000);	/* 复位要等待一会 复位操作不会立马完成. */
		return TRUE;
	}
	return FALSE;
}

/* 获取TCP通信状态 */
BOOL GPRS_GetTcpState(void)
{
	return GPRS_SendATCmd("AT+QISTATE\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT);
}

/* 获取未发送出去的字节数 */
uint16 GPRS_GetUnsentBLen(SOCKET SocketId)
{
	uint8 *pStart = g_UartComm[GPRS_UART_PORT].u8TRBuf;
	uint16 uAcked = 0;			/* 未发送出去的数据长度 */
	uint16 uRxLen = 0;			/* 接收的响应数据长度 */
	char aChCmd[20];
	uint8 *pBuff = (uint8 *)aChCmd;
	PrintString(&pBuff, pBuff + 12, "AT+QISACK=");
	*pBuff++ = g_GprsComm.Socket[SocketId-1].u8MuxId + '0';
	*pBuff++ = '\r';
	*pBuff++ = '\0';
	if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
		printf("\n[p]\n");
		if(GPRS_SendATCmd(aChCmd, "+QISACK: ", "OK", &uRxLen, 2, 2000)
			&& (pBuff = SkipCharInString(pStart, pStart + uRxLen, ',', 2))) {	/* 响应成功 */
			uAcked = ReadU32(&pBuff, pBuff + 5);
		}
		Semaphore_post(g_GprsComm.Sem_GprsReq);
		printf("\n[P]\n");
	} else {	/* 没拿到锁 */
	}
	return uAcked;
}

/* 等待指定socket的发送缓冲区发送数据, 直到发送到剩余指定的字节数或超时.
 * 适用场景：
 * 		1. 等待上一帧数据发送完成, 比如发送connect报文与sub报文需要等待响应, 接收这个响应的前提是报文已经发送出去了.
 * 		   因此可以使用此函数等待数据完全发送出去, 再接收响应. 注意: 所有需要收响应的报文都应该调用此函数来等待消息发送出去,
 * 		   否则直接调用recv可能什么都收不到, 参数中的NAcked是not acked.
 * 		2. 不希望发送缓冲区中堆叠太多的数据没有发送出去, 可以调用此函数等待直到缓冲区剩余指定的字节数, 再发送下一帧数据.
 * 参1: SocketId
 * 参2: 剩余发送字节数小于等于u32MaxNAcked时认为发送完成了, 可以发送下一帧数据了.
 * 参3: 最大卡住次数, 如果多次读取的未发送字节数都一样认为是卡住了, 可能是网络不好.为了防止卡死, 认为是发送失败. 为了保证通信正常, 应该重新连接一下.
 * 参4: 总超时时间. */
BOOL GPRS_WaitSocketIdle(SOCKET SocketId, uint32 u32MaxNAcked, uint8 u8MaxStuckCnt, uint16 uTotalTimeoutMs)
{
	uint32 u32StartTime = HAL_GetTick();
	uint16 uRxLen = 0;			/* 接收的响应数据长度 */
	uint16 uAcked = 0;			/* 未发送出去的数据长度 */
	uint16 uPreAcked = 0xFFFF;	/* 上次记录的未发送出去的数据长度 */
	uint8 u8StuckCount = u8MaxStuckCnt;
	BOOL bRes = FALSE;
	while(HAL_GetTick() - u32StartTime < uTotalTimeoutMs) {
		uAcked = GPRS_GetUnsentBLen(SocketId);
		/* 判断是否可以发送下一帧数据 */
		if(uAcked <= u32MaxNAcked) {
			bRes = TRUE;
			break;
		} else if((uPreAcked == uAcked) && (!u8StuckCount--)) {	/* 卡住超时了 */
			break;
		} else {
			u8StuckCount = u8MaxStuckCnt;
			uPreAcked = uAcked;
			Task_sleep(200);
		}
	}
	return bRes;
}

/* 初始化GPRS模块 */
void GPRS_Init(void)
{
	for(uint16 i = 0; i < MAX_GPRS_SOCKET_NUM; i++) {
		GPRS_SOCKET *pSocket = &g_GprsComm.Socket[i];
		pSocket->name = NULL;
		pSocket->protocl = 0;
		pSocket->u8MuxId = 0;
		pSocket->u8Res = 0;
		pSocket->bIsFree = TRUE;
		pSocket->u8MuxId = i;
	}
	GPRS_ClearBuffer();								/* 清空GPRS的接收缓冲区 */
	OpenUartComm(GPRS_UART_PORT, 115200, 0, 30);	/* 与GPRS通信串口 */

	GPRS_Reset();	/* 复位模块 */

	if(!GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 1, GPRS_UART_TIMEOUT)) {
		/* 通信失败, 设置波特率, 出厂时是自适应波特率, 好像每次复位之后都需要设置一下波特率, 否则通信不上. */
		if(!GPRS_SetBaud(115200)) {
			return;
		}
	}

	/* 初始化配置 */
	if(!GPRS_SendATCmd("ATE0\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT * 2)		/* 显示命令回显 0: 关闭回显 1：开启回显  */
		|| !GPRS_SendATCmd("AT+QISDE=0\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)	/* 关闭SEND指令数据回显，0不回显 */
		|| !GPRS_SendATCmd("AT&W\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)		/* 保存配置 */
		|| !GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)			/* 测试通信是否正常？ */
		|| !GPRS_SendATCmd("AT+CFUN=1\r", "OK", "SMS Ready", NULL, 5, 5000)) {		/* 设置模块功能为全模式 0:最小功能模式, 1:全功能模式, 4:飞行模式 无卡会返回 +CPIN: NOT INSERTED */
		return;
	}

	/* 获取卡状态 */
	if(!GPRS_GetSIMCardState()) {
		g_GprsComm.GprsStatus = GPRS_NO_SIM;
		return;
	}

	/* 查看网络相关状态 */
	if(!GPRS_GetInternetState() || !GPRS_OpenInternet() || !GPRS_TcpConf()) {
		g_GprsComm.GprsStatus = GPRS_NO_INTERNET;
		return;
	}

	g_GprsComm.GprsStatus = GPRS_NORMAL;
}

BOOL CheckGprsStatus(void)
{
	if(!GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)) {	/* AT指令都不响应, 可能DMA有问题, 也可能模块有问题, 重新初始化一下 */
		g_GprsComm.GprsStatus = GPRS_NOT_INIT;
	}
	switch(g_GprsComm.GprsStatus) {
		case GPRS_NORMAL:	/* 正常状态, 需要查询网络状态、卡状态、信号强度. */
			if(!GPRS_GetSIMCardState()) {				/* 检查卡状态 */
				g_GprsComm.GprsStatus = GPRS_NO_SIM;
			} else if(!GPRS_GetInternetState()			/* 检查网络状态 */
					&& (GPRS_GetSigStrong() != 99)) {	/* 检查信号强度 */
				g_GprsComm.GprsStatus = GPRS_NO_INTERNET;
			}
			break;
		case GPRS_NOT_INIT:		/* 还没初始化 */
			GPRS_Init();
			break;
		case GPRS_NO_SIM:		/* 未插SIM卡, 直接重新初始化模块 */
			g_GprsComm.GprsStatus = GPRS_NOT_INIT;
			break;
		case GPRS_NO_INTERNET:	/* 无法上网, 可能是网络信号差或未注册 */
			if((GPRS_GetSigStrong() == 99)) {	/* 查看信号强度 */
				g_GprsComm.GprsStatus = GPRS_NOT_INIT;
			} else if(GPRS_GetInternetState() && GPRS_OpenInternet() && GPRS_TcpConf()) {	/* 查看网络状态、打开网络附着状态、配置TCP通信 */
				g_GprsComm.GprsStatus = GPRS_NORMAL;
			}
			break;
		default:
			g_GprsComm.GprsStatus = GPRS_NOT_INIT;		/* 其他的状态暂时都设置为未初始化 */
			break;
	}
	return g_GprsComm.GprsStatus == GPRS_NORMAL;
}

/* 创建socket函数，socket作为全局变量的索引，索引统一+1是为避开NULL，保持原应用侧代码兼容 */
SOCKET GprsSocket(int32 domain, int32 type, int32 protocl)
{
	SOCKET sk = 0;
	if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
		printf("\n[p]\n");
		if(CheckGprsStatus() && ((protocl == IPPROTO_TCP) || (protocl == IPPROTO_UDP))) {
			for(int32 i = 0; i < MAX_GPRS_SOCKET_NUM; i++) {
				if(g_GprsComm.Socket[i].bIsFree) {
					sk = i+1;
					g_GprsComm.Socket[i].u8MuxId = i;
					g_GprsComm.Socket[i].u8Res = 0;
					g_GprsComm.Socket[i].protocl = protocl;
					g_GprsComm.Socket[i].bIsFree = FALSE;
					break;
				}
			}
		}
		Semaphore_post(g_GprsComm.Sem_GprsReq);
		printf("\n[P]\n");
	}
	return sk;
}
int32 GprsConnect(SOCKET socketFd, struct sockaddr_in* pName, int32 i32Len)
{
	int32 i32Result = 0;
	GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
	if((socketFd <= 0) || (!pName) || (!i32Len)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
			printf("\n[p]\n");
			if((g_GprsComm.GprsStatus == GPRS_NORMAL) && (!pSocket->bIsFree)) {
				/* 通过串口下发 */
				UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
				uint8* pBuf = pUartComm->u8TRBuf;
				PrintString(&pBuf, pBuf + 20, "AT+QIOPEN=");
				*pBuf++ = '0' + pSocket->u8MuxId;
				*pBuf++ = ',';
				*pBuf++ = '"';
				if(pSocket->protocl == IPPROTO_UDP) {
					*pBuf++ = 'U';
					*pBuf++ = 'D';
				} else {
					*pBuf++ = 'T';
					*pBuf++ = 'C';
				}
				*pBuf++ = 'P';
				*pBuf++ = '"';
				*pBuf++ = ',';
				*pBuf++ = '"';
				/* 这里其实也支持直接打印域名 */
				PrintIPv4(&pBuf, pBuf+16, __rev(pName->sin_addr.s_addr));
				*pBuf++ = '"';
				*pBuf++ = ',';
				*pBuf++ = '"';
				PrintU32(&pBuf, pBuf + 12, __rev16(pName->sin_port), 0);
				*pBuf++ = '"';
				*pBuf++ = '\r';
				WriteToUart(GPRS_UART_PORT, pBuf - pUartComm->u8TRBuf);
				/* 从串口读取返回内容,300ms内先会收到OK，然后10秒内会受到CONNECT OK或CONNECT FAIL */
				if(GPRS_SendATCmd(NULL, "CONNECT OK", "CONNECT", NULL, 2, 5000)) {	/* 查看响应 */
					i32Result = GPRS_SUC;
				} else {	/* 连接失败. */
					i32Result = GPRS_ERR_CONNECTION_ABN;
				}
				Task_sleep(100*OS_TICK_KHz);
			} else {
				i32Result = GPRS_ERR_OFFLINE;
			}
			Semaphore_post(g_GprsComm.Sem_GprsReq);
			printf("\n[P]\n");
		} else {
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}
	pSocket->u8Res = i32Result;
	return i32Result;
}

int32 GprsSend(SOCKET socketFd, uint8 *pSrcBuf, uint16 uSendBlen, uint32 u32Flag)
{
#define MAX_ONCE_SEND_BLEN	1000
	int32 i32Result = 0;
	uint32 u32AllSentLen = 0;	/* 所有已经发送的字节数 */
	uint32 u32AllSendLen = uSendBlen;	/* 一共要发送的字节数 */
	if((socketFd <= 0) || (!uSendBlen) || (!pSrcBuf)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
			printf("\n[p]\n");
//			/* 调试专用 */
//			if(socketFd == g_MqttComm[MQTT_TYPE_PUB].MqttSocket) {
//				printf("《Pub Send：》\n");
//			} else {
//				printf("《Sub Send：》1111\n");
//			}
			if((g_GprsComm.GprsStatus == GPRS_NORMAL) && (!pSocket->bIsFree)) {
				UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
				/* 模块最大支持MAX_ONCE_SEND_BLEN字节分次发送 */
//				uint8 u8SendTimes = (uSendBlen + MAX_ONCE_SEND_BLEN - 1) / MAX_ONCE_SEND_BLEN;
				for(; u32AllSentLen < u32AllSendLen;) {
					/* 大于MAX_ONCE_SEND_BLEN的分次发送 */
					uint16 uEverySendBlen;
					if(uSendBlen > MAX_ONCE_SEND_BLEN) {
						uEverySendBlen = MAX_ONCE_SEND_BLEN;
						uSendBlen -= MAX_ONCE_SEND_BLEN;
					} else {
						uEverySendBlen = uSendBlen;
					}
					uint16 uReadNum = 0;
					/* 填充打印 */
					uint8 aU8Cmd[30];
					uint8* pBuf = aU8Cmd;
					PrintString(&pBuf, pBuf + 20, "AT+QISEND=");
					*pBuf++ = '0' + pSocket->u8MuxId;
					*pBuf++ = ',';
					PrintU32(&pBuf, pBuf + 12, uEverySendBlen, 0);
					*pBuf++ = '\r';
					*pBuf++ = '\0';
					uint16 uSentLen = 0;
					/* 这个有时候发送并不回复'>'号, 不清楚什么原因因此这里最多发送几次 */
					if(GPRS_SendATCmd((char *)aU8Cmd, ">", NULL, &uReadNum, 1, 2000)) {	/* 响应成功 */
						/* 开始发送数据 */
						while(uSentLen < uEverySendBlen) {
							uint16 uSendLen = (uEverySendBlen - uSentLen) < UART_TR_BUF_BLEN ? (uEverySendBlen - uSentLen) : UART_TR_BUF_BLEN;
							memcpy(pUartComm->u8TRBuf, pSrcBuf + uSentLen + u32AllSentLen, uSendLen);
							WriteToUart(GPRS_UART_PORT, uSendLen);
							uSentLen += uSendLen;
						}
						if(GPRS_SendATCmd(NULL, "SEND OK", NULL, NULL, 1, 2000)) {	/* 发送成功 */
							u32AllSentLen += uEverySendBlen;
						} else {	/* 发送失败 认为是断开连接 */
							i32Result = GPRS_ERR_CONNECTION_ABN;
							pSocket->u8Res = GPRS_ERR_CONNECTION_ABN;
							break;
						}
					} else if(uReadNum != 0){	/* 响应了其他消息 */
						if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"SEND FAIL")) {
							/* 连接存在 但是发送失败 认为串口有问题 */
							i32Result = GPRS_ERR_UART_ABN;
							break;
						} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"SEND FAIL")
								|| SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8 *)"ERROR")) {
							/* 未建立TCP连接 */
							i32Result = GPRS_ERR_CONNECTION_ABN;
							pSocket->u8Res = GPRS_ERR_CONNECTION_ABN;
							break;
						} else {	/* 不能解析的响应 */
							i32Result = GPRS_ERR_PARSE;
						}
					} else {	/* 未响应, 可能是设备卡死了, 直接重连 */
						i32Result = GPRS_ERR_CONNECTION_ABN;
						pSocket->u8Res = GPRS_ERR_CONNECTION_ABN;
						break;
					}
				}
			} else {	/* GPRS没网或者socket不可用 */
				i32Result = GPRS_ERR_CONNECTION_ABN;
				pSocket->u8Res = GPRS_ERR_CONNECTION_ABN;
			}
			Semaphore_post(g_GprsComm.Sem_GprsReq);
			printf("\n[P]\n");
		} else {	/* 没拿到锁 */
			i32Result = GPRS_ERR_TIMEOUT;
//			printf("\r\n[Sem : %lu]\r\n", uxSemaphoreGetCount(g_GprsComm.Sem_GprsReq));
		}
	}

	if(u32AllSentLen == u32AllSendLen) {	/* 发送完成 */
		return u32AllSentLen;
	} else {	/* 发送失败 */
		return i32Result;
	}


//						if((uReadNum = ReadFromUart(GPRS_UART_PORT, 1000)) > 0) {
//							if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)">")) {		/* 如果M25返回> 说明可以继续发送待发送内容了 */
//								uint8 *pSrcBufEnd = pSrcBuf + uEverySendBlen;
//								/* 按单次最大UART_TR_BUF_BLEN字节，分次发送全部内容 */
//								while (pSrcBuf < pSrcBufEnd) {
//									uint32 u32CopyBlen = (((pSrcBufEnd - pSrcBuf) < UART_TR_BUF_BLEN) ? (pSrcBufEnd - pSrcBuf) : UART_TR_BUF_BLEN);
//									memcpy(pUartComm->u8TRBuf, pSrcBuf, u32CopyBlen);
//									if (pSrcBufEnd - pSrcBuf <= u32CopyBlen) {
//										pUartComm->u8TRBuf[u32CopyBlen] = '\r';  	/* 发送结束符号 */
//										WriteToUart(GPRS_UART_PORT, u32CopyBlen + 1);
//									} else {
//										WriteToUart(GPRS_UART_PORT, u32CopyBlen);
//									}
//									pSrcBuf += u32CopyBlen; // 更新指针
//								}
//								/* 接收模块回的是否发送成功反馈 */
////								memset(pUartComm->u8TRBuf, 0 , UART_TR_BUF_BLEN);
//								for(; i32Result == 0; ) {		/* 循环防止被别的socket主动上报的内容乱入 */
//									if((uReadNum = ReadFromUart(GPRS_UART_PORT, 1000)) > 0) {
//										if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"SEND OK")) {
//											i32Result = (int32)uEverySendBlen;		/* 发送成功 */
//											break;
//										} else if(!SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"ERROR")) {	/* 读到了内容，但也没有ERROR，则忽略，也就是可能被别的消息插队了 */
//											continue;
//										} else {
//											i32Result = GPRS_ERR_PARSE;
//										}
//									} else {
//										i32Result = GPRS_ERR_UART_ABN;
//									}
//								}
//								if(i32Result != 0) {
//									break;		/* 如果收到了确定的结果就跳出此次接收 */
//								}
//							} else if(!SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"ERROR")) {	/* 读到了内容，但也没有ERROR，则忽略，也就是可能被别的消息插队了 */
//								continue;
//							} else {
//								i32Result = GPRS_ERR_PARSE;
//							}
//						} else {
//							i32Result = GPRS_ERR_UART_ABN;
//						}
//					}
//					if(i32Result < 0) {
//						break;		/* 如果分次发送中有一次遇到问题，就不再尝试继续发送 */
//					}
//				}
//			}
//			Semaphore_post(g_GprsComm.Sem_GprsReq);
//		} else {
//			i32Result = GPRS_ERR_TIMEOUT;
//		}
//	}
//	return i32Result;
}


#define MAX_QIRD_HEADER_LEN 44
#define	MAX_READ_LEN		(UART_TR_BUF_BLEN - MAX_QIRD_HEADER_LEN)

/* 这里参数3的作用是, 如果没读到数据 持续重试次数. 主要是NTP通信时, 使用UDP通信没办法知道数据是否已经发送出去了, 因此读的时候需要持续读 */
int32 GprsRecv(SOCKET socketFd, uint8 *pU8DstBuf, int32 i32ReadBLen, int32 i32Flags)
{
	int32 i32Result = 0;
	uint32 u32ResultLen = 0;	/* 读取的字节数 */
	if((socketFd <= 0) || (!i32ReadBLen) || (!pU8DstBuf)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
		/* 如果没读到，重新申请信号量再试，最多重试2次 */
		uint8 u8EmptyRecvTryCnt = i32Flags > 0 ? i32Flags : 2;
			if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
				printf("\n[p]\n");
				if((g_GprsComm.GprsStatus == GPRS_NORMAL) && (!pSocket->bIsFree)) {	/* GPRS联网并且套接字可用 */
					/* 以MAX_READ_LEN为最大单次读取大小，循环读取全部长度内容 */
					/* 以uart buf大小为单元循环查询读取M25缓存中全部长度数据（M25每socket最大缓存400K） */
					UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
					char aChCmd[30];
					for(; i32ReadBLen > 0 && u8EmptyRecvTryCnt; ) {	/* 循环接收剩余字节 */
#if 1
						uint8* pBuf = (uint8 *)aChCmd;
						uint16 uUartReadBLen = 0;
						PrintString(&pBuf, pBuf + 20, "AT+QIRD=0,1,");
						*pBuf++ = '0' + pSocket->u8MuxId;
						*pBuf++ = ',';
						PrintU32(&pBuf, pBuf+ 12, (i32ReadBLen > MAX_READ_LEN ? MAX_READ_LEN : i32ReadBLen), 0);
						*pBuf++ = '\r';
						*pBuf++ = '\0';
						/* 接收响应, 读到OK认为读取一帧完成. */
						pBuf = pUartComm->u8TRBuf;
						if(GPRS_SendATCmd(aChCmd, "+QIRD:", "OK", &uUartReadBLen, 2, 2000)) {
							if((pBuf = SkipCharInString(pBuf, pBuf + uUartReadBLen, ',', 2))) {	/* 定位到数据长度的位置 */
								uint16 uRecvFrameLen = ReadU32(&pBuf, pBuf + 5);	/* QIRD接收内容长度 */
								/* 数据有效 */
								if(uRecvFrameLen
									&& (uRecvFrameLen <= uUartReadBLen)
									&& (pBuf = SkipStrInString(pBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"\r\n")))
								{
									i32ReadBLen -= uRecvFrameLen;		/* 更新剩余字节数 */
									u32ResultLen += uRecvFrameLen;		/* 更新读到字节数 */
									for(; uRecvFrameLen > 0; uRecvFrameLen--) {	/* 将响应数据拷贝到用户缓冲区 */
										*pU8DstBuf++ = *pBuf++;
									}
								} else {	/* 无法识别数据帧 */
									i32Result = GPRS_ERR_PARSE;
									break;
								}
							} else {	/* 无法识别数据帧 */
								i32Result = GPRS_ERR_PARSE;
								break;
							}
						} else if(uUartReadBLen != 0) {	/* 收到了其他类型的响应 */
							if((pBuf = SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"OK"))) {	/* 只有OK 说明没数据了 */
								i32Result = GPRS_ERR_NO_DATA;
								if(u32ResultLen == 0) {	/* 还没读到数据, 继续读 */
									u8EmptyRecvTryCnt--;
									Task_sleep(100);
									continue;
								} else {			/* 已经读到过数据了可以直接返回了 */
									break;
								}
							} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"CLOSE")
									|| SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"ERROR")
									|| SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"+PDP DEACT")) {	/* 是否断开连接 */
								i32Result = GPRS_ERR_CONNECTION_ABN;	/* 断开连接 */
								pSocket->u8Res = GPRS_ERR_CONNECTION_ABN;
								break;
							} else {	/* 响应了其他未识别的消息, 直接断开连接, 可能数据已经乱了 */
								i32Result = GPRS_ERR_PARSE;
								pSocket->u8Res = GPRS_ERR_CONNECTION_ABN;
								break;
							}
						} else {	/* 读超时,没有响应, 连OK都没回可能设备卡死了, 重新连接. */
							pSocket->u8Res = GPRS_ERR_CONNECTION_ABN;
							i32Result = GPRS_ERR_TIMEOUT;
							break;
						}
					}
					if(i32Result != GPRS_ERR_CONNECTION_ABN) {
						i32Result = u32ResultLen > 0 ? u32ResultLen : i32Result;
					}
					Semaphore_post(g_GprsComm.Sem_GprsReq);
					printf("\n[P]\n");
				} else {	/* GPRS没联网, 或套接字不可用 */
					i32Result = GPRS_ERR_OFFLINE;
					pSocket->u8Res = GPRS_ERR_CONNECTION_ABN;
				}
			} else {	/* 没拿到锁, 返回读超时 */
				i32Result = GPRS_ERR_TIMEOUT;
//				printf("\r\n[Sem : %lu]\r\n", uxSemaphoreGetCount(g_GprsComm.Sem_GprsReq));
			}
	}
	if(i32Result > 0) {
		return i32Result;
	} else {
		return i32Result;
	}
	return i32Result;
#else
						uint16 uUartReadBLen = ReadFromUart(GPRS_UART_PORT, 300);




						if(uUartReadBLen > 0) {
							if((pBuf = SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"+QIRD:")) != NULL) {
								/* 定位到QIRD接收内容长度字段 */
								pBuf = SkipCharInString(pBuf, pUartComm->u8TRBuf + uUartReadBLen, ',', 2);
//								if(pBuf < (pUartComm->u8TRBuf + uUartReadBLen)) {
								uint16 uRecvFrameLen = ReadU32(&pBuf, pBuf + 5);		/* QIRD接收内容长度 */
								if(uRecvFrameLen && (uRecvFrameLen <= uUartReadBLen) && (pBuf = SkipStrInString(pBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"\r\n"))) {
									i32ReadBLen -= uRecvFrameLen;		/* 更新剩余字节数 */
									i32Result += uRecvFrameLen;			/* 更新读到字节数 */
									/* 拷贝到用户buf */
									for(; uRecvFrameLen > 0; uRecvFrameLen--) {
										*pU8DstBuf++ = *pBuf++;
									}
									continue;		/* 如果有大于MAX_READ_LEN的内容再次请求读取 */
								} else {
									i32Result = GPRS_ERR_PARSE;
									break;
								}
//								} else {
//									i32Result = GPRS_ERR_PARSE;
//									break;
//								}
							} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"+CME ERROR: 58") > 0) {		/* 请求过于频繁，不用断开直接重试 */
								break;
							} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"CLOSED") > 0) {		/*  */
								i32Result = GPRS_ERR_CONNECTION_ABN;
								break;
							} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"ERROR") > 0) {		/* 连接断开了可能 */
								i32Result = GPRS_ERR_CONNECTION_ABN;
								break;
							} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"OK") > 0) {		/* 读完全部内容和没读到任何数据都会返回OK */
								break;		/* M25 buf里没数据了 */
							} else {
								break;
							}
						} else {
							i32Result = GPRS_ERR_UART_ABN;
							break;
						}
					}
				} else {
					i32Result = GPRS_ERR_OFFLINE;
				}
				Semaphore_post(g_GprsComm.Sem_GprsReq);
			} else {
				if(u8EmptyRecvTryCnt == 1) {
					i32Result = GPRS_ERR_TIMEOUT;
				}
			}
			if(i32Result != 0) {
				break;
			} else if(u8EmptyRecvTryCnt > 1) {
				Task_sleep(500);
			}
		}
	}
	if(i32Result == 0) {
		i32Result = GPRS_ERR_NO_DATA;
	}
	return i32Result;
#endif
}

int32 GprsClose(SOCKET socketFd)
{
	int32 i32Result = 0;
	if(socketFd <= 0) {
		i32Result = GPRS_ERR_ARG;
	} else {
		GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
			printf("\n[p]\n");
			if((g_GprsComm.GprsStatus == GPRS_NORMAL) && (!pSocket->bIsFree)) {
				UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
				uint8* pBuf= pUartComm->u8TRBuf;
				uint16 uReadNum = 0;
				PrintStringNoOvChk(&pBuf, "AT+QICLOSE=");
				*pBuf++ = '0' + pSocket->u8MuxId;
				*pBuf++ = '\r';
				WriteToUart(GPRS_UART_PORT, pBuf - pUartComm->u8TRBuf);
				pBuf = pUartComm->u8TRBuf;
				if((uReadNum = ReadFromUart(GPRS_UART_PORT, 5000)) > 0) {
					if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"OK")
							|| SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"ERROR"))
					{
						i32Result = GPRS_SUC;
					} else {
						i32Result = GPRS_ERR_OFFLINE;
					}
				} else {
					i32Result = GPRS_ERR_UART_ABN;
				}
				pSocket->bIsFree = TRUE;
			} else {
				g_GprsComm.GprsStatus = GPRS_ERR_OFFLINE;
			}
			Semaphore_post(g_GprsComm.Sem_GprsReq);
			printf("\n[P]\n");
		} else {
//			printf("\r\n[Sem : %lu]\r\n", uxSemaphoreGetCount(g_GprsComm.Sem_GprsReq));
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}
	return i32Result;
}

int32 GprsSetSockopt(SOCKET socketFd, int32 u32Level, int32 u32Op, void *pbuf, int32 i32Bufsize)
{
	return 0;
}

/* 返回查到的IP（小端），0代表失败 */
extern TSK_Handle MqttSubTaskHandle;
BOOL GprsDnsQuery(uint8* domain, uint32 *pU32IpResolved)
{
#if !USE_DNS
	*pU32IpResolved = __rev(0x76BE3FE3);
	return TRUE;
#else
	int32 i32Result = 0;
	if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
		if(CheckGprsStatus()) {
			UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
			uint8* pBuf = pUartComm->u8TRBuf;
			uint16 uReadNum = 0;
			PrintStringNoOvChk(&pBuf, "AT+QIDNSGIP=\"");
			PrintStringWithMaxLen(&pBuf, (char*)domain, DNS_DOMAIN_MAX_LEN);
			PrintStringNoOvChk(&pBuf, "\"\r");
			pUartComm->uTime_RxMaxIntv_ms = 1000;
			WriteToUart(GPRS_UART_PORT, pBuf - pUartComm->u8TRBuf);
			if((uReadNum = ReadFromUart(GPRS_UART_PORT, 300)) > 0) {
				if((uReadNum > 6)		/* 0x0D 0x0A 0x0D 0x0A O K  ip地址最多16个字符 */
						&& (pBuf = SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"OK\r\n\r\n"))
						&& !GetIPv4(&pBuf, pBuf + 16, &i32Result))
				{
					i32Result = GPRS_ERR_PARSE;
				} else {
					*pU32IpResolved = __rev(i32Result);
				}
			} else {
				i32Result = GPRS_ERR_UART_ABN;
			}
			Task_sleep(3000*OS_TICK_KHz);
		}
		Semaphore_post(g_GprsComm.Sem_GprsReq);
	} else {
		i32Result = GPRS_ERR_TIMEOUT;
	}
	return i32Result > 0;
#endif
}

/* 初始化环形缓冲区 */
void RingBuffer_Init(RingBuffer_t *pRingBuffer)
{
	pRingBuffer->uRear = 0;
	pRingBuffer->uHead = 0;
}

/* 往环形队列里写数据, 会覆盖旧数据 */
void RingBuffer_Write(RingBuffer_t *pRingBuffer, const uint8 *cnst_pU8Data, uint16 uLen)
{
    for (uint16 i = 0; i < uLen; i++) {
    	pRingBuffer->aU8Buffer[pRingBuffer->uRear] = cnst_pU8Data[i];
    	pRingBuffer->uRear = (pRingBuffer->uRear + 1) % RING_BUFFER_MAX_SIZE;
        /* 如果 head 追上了 rear，说明满了，自动前移丢弃旧数据 */
        if (pRingBuffer->uHead == pRingBuffer->uRear) {
        	pRingBuffer->uHead = (pRingBuffer->uHead + 1) % RING_BUFFER_MAX_SIZE;
        }
    }
}

/* 读取环形队列中所有数据 */
uint16 RingBuffer_Read(RingBuffer_t *pRingBuffer, uint8 *pU8Data, uint16 uReadLen)
{
	uint16 uLen = 0;	/* 队列长度 */
    if (pRingBuffer->uRear >= pRingBuffer->uHead) {
    	uLen = pRingBuffer->uRear - pRingBuffer->uHead;
    } else {
    	uLen = RING_BUFFER_MAX_SIZE - pRingBuffer->uHead + pRingBuffer->uRear;
    }
    uLen = uReadLen >= uLen ? uLen : uReadLen;
	for (uint16 i = 0; i < uLen; i++) {
		pU8Data[i] = pRingBuffer->aU8Buffer[pRingBuffer->uHead];
		pRingBuffer->uHead = (pRingBuffer->uHead + 1) % RING_BUFFER_MAX_SIZE;
	}
	return uLen;
}

/* 去除掉没用的信息: (+QIRDI: ....) 这个消息是GPRS多连接模式时主动上报的, 无法关闭.
 * 这个信息还有可能包含在消息帧中并不是单独发送的, 所以还不能忽略掉, 在这里给它去除.
 * 注：这个函数专门服务于ReadFromUart通信中的GPRS, 内容是写死的, 并不通用 */
uint16 StrRemoveHashData(uint8 *pSrc, uint16 uDataLen)
{
	uint8 *pStart = pSrc;
	uint8 *pSrcEnd = pSrc + uDataLen;
	uint8 *pEnd;
	if(uDataLen < 15) {	/* 数据过短不可能存在+QIRDI:数据帧 */
		return uDataLen;
	}
	while((pStart = SkipStrInString(pStart, pSrcEnd, (uint8 *)"+QIRDI:"))) {
		/* 这帧数据中存在无用数据准备去除 */
		if(!(pEnd = SkipStrInString(pStart, pSrcEnd, (uint8 *)"\r\n"))) {	/* 有+QIRDI:但是没找到\r\n, 就没办法确定结束位置, 只能返回不做处理了. */
			return uDataLen;
		}
		pStart -= 9;	/* 移动到\r\n处() */
		pStart = pStart < pSrc ? pSrc : pStart;    /* 防止向上溢出 */
		/* 覆盖掉的长度 */
		uint16 uCoverLen = pEnd - pStart;
		if(pSrcEnd - pEnd > 0) {

		}
		/* 开始拷贝 */
		memcpy(pStart, pEnd, pSrcEnd - pEnd);
		/* 更新数据 */
		if(uDataLen < uCoverLen) {
			return 0;
		}
		uDataLen -= uCoverLen;
		pSrcEnd -= uCoverLen;
	}
	return uDataLen;
}
