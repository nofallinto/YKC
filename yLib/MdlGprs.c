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
#include "MdlSys.h"
#include "LibMath.h"
#include "main.h"

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

/***************************************************************************
						internal functions declaration
***************************************************************************/
uint8* SkipStrInString(uint8* pU8Msg, uint8* pU8MsgEnd, uint8* u8String);
/* 从环形缓冲区中读取uint32类型的数据 */
uint32 ReadU32FromRingBuff(uint8* pSrc, uint16 uHead, uint16 uRear, uint16 uRingBuffLen);
/***************************************************************************
 							functions definition
***************************************************************************/
#define DEBUG_PRINT_SIG		FALSE		/* 信号量调试打印 */
#define GPRS_ENABLE()		HAL_GPIO_WritePin(GPRS_4G_EN_Port, GPRS_4G_EN_Pin, GPIO_PIN_SET)
#define GPRS_DISABLE()		HAL_GPIO_WritePin(GPRS_4G_EN_Port, GPRS_4G_EN_Pin, GPIO_PIN_RESET)
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
| Description	: 从环形缓冲区中查找第n个特征字符，并返回特征字符末尾指针。
| In/Out/G var	:pU8RingBuff: 环形缓冲区起始地址 pU8Pattern: 目标字符串必须\0结尾. uHead: 环形缓冲区头指针
				 uRear: 环形缓冲区尾指针. uRingBuffLen: 环形缓冲区总长度
| Author		: 			Date	: 2022-10-19
\=========================================================================*/
uint8 *SkipCharInRingBuff(uint8 *pU8RingBuff, uint8 u8Char, uint16 uHead, uint16 uRear, uint8 uCharNum, uint16 uRingBuffLen)
{
	uint8 *pU8Src = pU8RingBuff + uHead;
	int32 i = uRear - uHead;
	if(uHead <= uRear) {	/* 线性查找 */
		for(; (i > 0) && uCharNum; i--) {
			if(*pU8Src++ == u8Char) {
				uCharNum--;
			}
		}
	} else {
		i = uRingBuffLen - uHead;
		for(; (i > 0) && uCharNum; i--) {
			if(*pU8Src++ == u8Char) {
				uCharNum--;
			}
		}
		if(uCharNum != 0) {	/* 还没找到 */
			i = uRear;
			pU8Src = pU8RingBuff;
			for(; (i > 0) && uCharNum; i--) {
				if(*pU8Src++ == u8Char) {
					uCharNum--;
				}
			}
		}
	}
	if(uCharNum == 0) {
		return pU8Src;
	} else {
		return NULL;
	}
}

/*==========================================================================
| Description	: 从环形缓冲区中查找特征字符串，并返回特征字符串末尾指针。
| In/Out/G var	:pU8RingBuff: 环形缓冲区起始地址 pU8Pattern: 目标字符串必须\0结尾. uHead: 环形缓冲区头指针
				 uRear: 环形缓冲区尾指针. uRingBuffLen: 环形缓冲区总长度
| Author		: 			Date	: 2022-10-19
\=========================================================================*/
uint8 *SkipStrInRingBuff(uint8 *pU8RingBuff, uint8 *pU8Pattern, uint16 uHead, uint16 uRear, uint16 uRingBuffLen)
{
    if(uHead == uRear) {	/* 对列为空 */
        return NULL;
    }
    uint8 *pSrcBuf = pU8RingBuff + uHead;
    uint8 *pSrcBufEnd;
    uint8 *pPattern = pU8Pattern;
    uint16 uLen = uRear - uHead;
    if(uHead < uRear) {
        pSrcBufEnd = pU8RingBuff + uRear;
        for(int i = 0; i < uLen; i++) {
            pSrcBuf = pU8RingBuff + uHead + i;
            pPattern = pU8Pattern;
            for(; (pSrcBuf != pSrcBufEnd) && (*pPattern) && (*pSrcBuf == *pPattern); pSrcBuf++, pPattern++);
            if(*pPattern == '\0') {		/* 匹配完成 */
                if(pSrcBuf - pU8RingBuff >= uRingBuffLen) { /* 刚好在末尾处 */
                    return pU8RingBuff;
                }
                return pSrcBuf;
            }
        }
    } else {
        /* 分段处理, 先处理head->buff末尾, 在处理buff开头->rear */
        uLen = uRingBuffLen - uHead;	/* head->buff末尾部分的长度 */
        for(int i = 0; i < uLen; i++) {
            pSrcBuf = pU8RingBuff + uHead + i;
            pSrcBufEnd = pU8RingBuff + uRingBuffLen;
            pPattern = pU8Pattern;
            for(; (pSrcBuf != pSrcBufEnd) && (*pPattern) && (*pSrcBuf == *pPattern); pSrcBuf++, pPattern++);
            if(pSrcBuf == pSrcBufEnd) {  /* 查到了最后, 再从头查 */
                pSrcBuf = pU8RingBuff;
                pSrcBufEnd = pU8RingBuff + uRear;
                for(; (pSrcBuf != pSrcBufEnd) && (*pPattern) && (*pSrcBuf == *pPattern); pSrcBuf++, pPattern++);
            }
            if(*pPattern == '\0') {		/* 匹配完成 */
                if(pSrcBuf - pU8RingBuff >= uRingBuffLen) { /* 刚好在末尾处 */
                    return pU8RingBuff;
                }
                return pSrcBuf;
            }
        }

        /* 继续找buff开头->rear */
        uLen = uRear;	/* head->buff末尾部分的长度 */
        pSrcBufEnd = pU8RingBuff + uRear;
        for(int i = 0; i < uLen; i++) {
            pSrcBuf = pU8RingBuff + i;
            pPattern = pU8Pattern;
            for(; (pSrcBuf != pSrcBufEnd) && (*pPattern) && (*pSrcBuf == *pPattern); pSrcBuf++, pPattern++);
            if(*pPattern == '\0') {		/* 匹配完成 */
                return pSrcBuf;
            }
        }
    }
    return NULL;	/* 没找到返回NULL */
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
 * 参4: 接收的响应的环形队列的Head
 * 参5: 接收的响应的环形队列的Rear
 * 参6: 尝试次数.
 * 参7: 超时时间, 注意是单次读取的超时时间 */
GPRS_RES GPRS_SendATCmd(const char *cnst_pChCmd, const char *cnst_pChExpectResp, const char *cnst_pChEndStr, uint16 *pUHead, uint16 *pURear, uint8 u8TryCnt, uint16 uTimeoutMs)
{
	UART_COMM *pUartComm = &g_UartComm[GPRS_UART_PORT];
	uint8* pStart = pUartComm->u8TRBuf;
	uint8* pBuf;
    uint16 uRxLen = 0, uRear = pUartComm->uRear;		/* 接收到的数据长度 */
    GPRS_RES eRet = GPRS_SUC;		/* 返回值 */
    BOOL bFlag = TRUE;		/* 是否需要重新发送命令的标志 */
	while(u8TryCnt--) {
		if(cnst_pChCmd && bFlag) {	/* 需要发送指令 */
			pBuf = pStart;
			/* 填充AT指令 */
			PrintString(&pBuf, pBuf + 40, cnst_pChCmd);	/* 注意这个指令的长度不是可变的, 根据需要调整, 也可以直接设置为最大. */
			/* 发送命令 */
			WriteToUart(GPRS_UART_PORT, pBuf - pStart);
			bFlag = FALSE;		/* 指令只发送一次 */
		}
		/* 读取响应 */
		if((uRxLen = ReadFromUart(GPRS_UART_PORT, uTimeoutMs))) {
			uRear = (pUartComm->uHead + uRxLen) % UART_TR_BUF_BLEN;		/* 这里手动计算队尾, 队尾指针在中断里更新的，可能会变 */
			if(cnst_pChEndStr == NULL) {	/* 没有结束标记 */
				if((cnst_pChExpectResp == NULL) || SkipStrInRingBuff(pStart, (uint8 *)cnst_pChExpectResp, pUartComm->uHead, uRear, UART_TR_BUF_BLEN)) {
					eRet = GPRS_SUC;
					break;
				} else {	/* 没有收到期望的响应 */
					eRet = GPRS_NO_EX_RESP;
				}
			} else {	/* 传入了结束标记 */
				if((pBuf = SkipStrInRingBuff(pStart, (uint8 *)cnst_pChEndStr, pUartComm->uHead, uRear, UART_TR_BUF_BLEN))) {	/* 收到了结束标记 */
					if((cnst_pChExpectResp == NULL) || SkipStrInRingBuff(pStart, (uint8 *)cnst_pChExpectResp, pUartComm->uHead, uRear, UART_TR_BUF_BLEN)) {
						eRet = GPRS_SUC;
					} else {	/* 这里是收到了结束标记, 但是期望响应不对 */
						eRet = GPRS_NO_EX_RESP;
					}
					break;
				} else if(u8TryCnt == 0) {	/* 没收到结束标记并且是最后一次机会. */
					eRet = GPRS_NO_EX_END;
				}
			}
			/* 检查是否收到了错误响应 */
			if(SkipStrInRingBuff(pStart, (uint8 *)"ERROR", pUartComm->uHead, uRear, UART_TR_BUF_BLEN)) {
				eRet = GPRS_RES_ERROR;
				break;
			} else if(SkipStrInRingBuff(pStart, (uint8 *)"FAIL", pUartComm->uHead, uRear, UART_TR_BUF_BLEN)) {
				eRet = GPRS_RES_FAIL;
				break;
			}
		} else {	/* 无响应 */
			eRet = GPRS_ERR_NO_RESP;
		}
	}
	if(pUHead != NULL) {
		*pUHead = pUartComm->uHead;
	}
	if(pURear != NULL) {
		*pURear = uRear;
	}
//	if(uRear != pUartComm->uRear) {
//		NOP;
//	}
	pUartComm->uHead = uRear;		/* 更新队首指针 */
    return eRet;
}

/* 清空缓冲区, 因为使用环形缓冲区接收的数据, 频发发送AT指令可能会导致消息错位
 * 因此每次初始化的时候最好清空一下缓冲区 */
void GPRS_ClearBuffer(void)
{
	g_UartComm[GPRS_UART_PORT].uHead = g_UartComm[GPRS_UART_PORT].uRear;
}

/* 获取GPRS信号强度 */
uint8 GPRS_GetSigStrong(void)
{
	uint8 *pU8Buf;
	uint16 uHead, uRear;
	if(GPRS_SendATCmd("AT+CSQ\r", NULL, "OK", &uHead, &uRear, 2, 2000) == GPRS_SUC) {
		if((pU8Buf = SkipStrInRingBuff(g_UartComm[GPRS_UART_PORT].u8TRBuf, (uint8 *)"CSQ:", uHead, uRear, UART_TR_BUF_BLEN))) {
			return ReadU32FromRingBuff(g_UartComm[GPRS_UART_PORT].u8TRBuf, pU8Buf - g_UartComm[GPRS_UART_PORT].u8TRBuf, uRear, UART_TR_BUF_BLEN);
		}
	} else {
		return 99;
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
	GPRS_SendATCmd(aChCmd, NULL, NULL, NULL, NULL, 1, 100);

	OpenUartComm(GPRS_UART_PORT, u32Baud, 0, 30);	/* 重新打开串口 */
	if(GPRS_SendATCmd("AT\r", "OK", "OK", NULL, NULL, 2, 1000) == GPRS_SUC) {	/* 是否响应 */
		return TRUE;
	}
	OpenUartComm(GPRS_UART_PORT, u32LastBaud, 0, 30);	/* 配置失败, 恢复串口波特率 */
	return FALSE;
}

/* 查看SIM卡是否能上网 */
BOOL GPRS_GetInternetState(void)
{
	return GPRS_SendATCmd("AT+CREG?\r", "+CREG: 0,1", "\r\n\r\nOK", NULL, NULL, 2, 2000) == GPRS_SUC;
}

/* 开启网络附着 */
BOOL GPRS_OpenInternet(void)
{
	return ((GPRS_SendATCmd("AT+CGATT=1\r", "OK", NULL, NULL, NULL, 1, 3000)  == GPRS_SUC) &&
	(GPRS_SendATCmd("AT+CGATT?\r", "+CGATT: 1", "\r\n\r\nOK", NULL, NULL, 2, 2000)  == GPRS_SUC));
}

/* 获取SIM卡状态 只是在线或未在线 */
BOOL GPRS_GetSIMCardState(void)
{
	return GPRS_SendATCmd("AT+CPIN?\r", "READY", "\r\n\r\nOK", NULL, NULL, 5, 500) == GPRS_SUC;
}

/* 获取TCP通信状态 */
BOOL GPRS_GetTcpState(void)
{
	return GPRS_SendATCmd("AT+QISTATE\r", "OK", NULL, NULL, NULL, 4, GPRS_UART_TIMEOUT) == GPRS_SUC;
}

/* 软件复位GPRS */
BOOL GPRS_Reset(void)
{
	GPRS_DISABLE();
	Task_sleep(10);
	GPRS_ENABLE();
	return GPRS_SendATCmd("AT+CFUN=0,1\r", "OK", "RDY", NULL, NULL, 10, GPRS_UART_TIMEOUT * 2) == GPRS_SUC;
}

#if USE_GPRS_02G_14G

/* 获取未发送出去的字节数 */
uint16 GPRS_GetUnsentBLen(SOCKET SocketId)
{
	uint8 *pStart = g_UartComm[GPRS_UART_PORT].u8TRBuf;
	uint16 uAcked = 0;			/* 未发送出去的数据长度 */
	uint16 uHead = 0, uRear = 0;			/* 接收的环形数据head、rear */
	uint8 u8SkId = g_GprsComm.Socket[SocketId - 1].u8MuxId;
	char aChCmd[20];
	uint8 *pBuff = (uint8 *)aChCmd;
	PrintString(&pBuff, pBuff + 12, "AT+QISEND=");
	if(u8SkId < 10) {
		*pBuff++ = u8SkId + '0';
	} else {
		*pBuff++ = u8SkId / 10 + '0';
		*pBuff++ = u8SkId % 10 + '0';
	}
	*pBuff++ = ',';
	*pBuff++ = '0';
	*pBuff++ = '\r';
	*pBuff++ = '\0';
	if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
#if DEBUG_PRINT_SIG
		printf("\n[p]\n");
#endif
		if((GPRS_SendATCmd(aChCmd, "+QISEND:", "OK", &uHead, &uRear, 5, 100) == GPRS_SUC)
			&& (pBuff = SkipStrInRingBuff(pStart, (uint8 *)"+QISEND:", uHead, uRear, UART_TR_BUF_BLEN))
			&& (pBuff = SkipCharInRingBuff(pStart, ',', pBuff - pStart, uRear, 2, UART_TR_BUF_BLEN))) {	/* 响应成功 */
			uAcked = ReadU32FromRingBuff(pStart, pBuff - pStart, uRear, UART_TR_BUF_BLEN);
		}
		Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
		printf("\n[P]\n");
#endif
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

/* 获取某个PDP场景id激活状态 */
BOOL GPRS_GetPDPActState(uint8 u8PDPId)
{
	uint16 uHead, uRear;
	uint8 *pStart = g_UartComm[GPRS_UART_PORT].u8TRBuf;
	if(GPRS_SendATCmd("AT+QIACT?\r", NULL, "OK", &uHead, &uRear, 4, GPRS_UART_TIMEOUT) == GPRS_SUC) {
		char aChExStr[12] = "+QIACT: x,";
		if(u8PDPId < 10) {
			aChExStr[8] = u8PDPId + '0';
		} else {
			aChExStr[8] = u8PDPId / 10 + '0';
			aChExStr[9] = u8PDPId % 10 + '0';
			aChExStr[10] = ',';
		}
		if((pStart = SkipStrInRingBuff(pStart, (uint8 *)aChExStr, uHead, uRear, UART_TR_BUF_BLEN))) {
			return (*pStart - '0') == 1;
		}
	}
	return FALSE;
}

/* 卡网络配置 */
BOOL GPRS_SIMCardNetConf(void)
{
	/* 先查看当前卡是否已激活PDP场景 */
	if(GPRS_GetPDPActState(1)) {
		return TRUE;
	} else {	/* 未激活, 先激活 */
		UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
		uint8* pBuf = pUartComm->u8TRBuf;
		PrintString(&pBuf, pBuf + 20, "AT+QICSGP=1,");
		PrintU32(&pBuf, pBuf + 1, GPRS_CONTEXT_TYPE, 1);
		*pBuf++ = GPRS_CONTEXT_TYPE + '0';
		*pBuf++ = ',';
		*pBuf++ = '"';
		PrintString(&pBuf, pBuf + 20, GPRS_SIMCARD_OPERATOR_APN);
		*pBuf++ = '"';
		PrintString(&pBuf, pBuf + 20, ",\"\",\"\",1\r");
		WriteToUart(GPRS_UART_PORT, pBuf - pUartComm->u8TRBuf);
		/* 激活PDP场景 */
		if((GPRS_SendATCmd(NULL, "OK", NULL, NULL, NULL, 1, GPRS_UART_TIMEOUT) == GPRS_SUC)
		&& (GPRS_SendATCmd("AT+QIACT=1\r", "OK", NULL, NULL, NULL, 5, GPRS_UART_TIMEOUT * 4) == GPRS_SUC)) {
			return TRUE;
		}
	}
	return FALSE;
}

/* GPRS初始化通信 */
BOOL GPRS_InitComm(void)
{
	OpenUartComm(GPRS_UART_PORT, GPRS_DEFAULT_UART_BAUD, 0, 30);	/* 与GPRS通信串口 */
	if(GPRS_SendATCmd("AT\r", "OK", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT) != GPRS_SUC) {
		/* 通信失败, 设置波特率, 出厂时是自适应波特率, 好像每次复位之后都需要设置一下波特率, 否则通信不上. */
		return GPRS_SetBaud(GPRS_DEFAULT_UART_BAUD);
	}
	return TRUE;
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
		pSocket->u32LastQirdiIndex = 0;
		pSocket->u32UnreadBLen = 0;
	}
	GPRS_ClearBuffer();								/* 清空GPRS的接收缓冲区 */
	GPRS_Reset();									/* 复位模块 */

	/* 初始化配置 */
	if(GPRS_SendATCmd("ATE0\r", "OK", NULL, NULL, NULL, 2, GPRS_UART_TIMEOUT * 2)		/* 显示命令回显 0: 关闭回显 1：开启回显  */
		|| GPRS_SendATCmd("AT+QISDE=0\r", "OK", NULL, NULL, NULL, 2, GPRS_UART_TIMEOUT)	/* 关闭SEND指令数据回显，0不回显 */
		|| GPRS_SendATCmd("AT&W\r", "OK", NULL, NULL, NULL, 2, GPRS_UART_TIMEOUT)		/* 保存配置 */
		|| GPRS_SendATCmd("AT\r", "OK", NULL, NULL, NULL, 2, GPRS_UART_TIMEOUT)			/* 测试通信是否正常？ */
		|| GPRS_SendATCmd("AT+CFUN=1\r", "OK", "OK", NULL, NULL, 1, GPRS_UART_TIMEOUT * 6)) {		/* 设置模块功能为全模式 0:最小功能模式, 1:全功能模式, 4:飞行模式 无卡会返回 +CPIN: NOT INSERTED */
		return;
	}
	Task_sleep(2000);		/* 实际测试开启全功能模式后要等一会，否则卡会一直上不了网. */
	/* 获取卡状态 */
	if(!GPRS_GetSIMCardState()) {
		g_GprsComm.GprsStatus = GPRS_NO_SIM;
		return;
	}

	/* 查看卡网络相关状态 */
	if(!GPRS_GetInternetState() || !GPRS_OpenInternet() || !GPRS_SIMCardNetConf()) {
		g_GprsComm.GprsStatus = GPRS_NO_INTERNET;
		return;
	}

	g_GprsComm.GprsStatus = GPRS_NORMAL;
}

BOOL CheckGprsStatus(void)
{
	if(GPRS_SendATCmd("AT\r", "OK", NULL, NULL, NULL, 2, GPRS_UART_TIMEOUT) != GPRS_SUC) {	/* AT指令都不响应, 可能DMA有问题, 也可能模块有问题, 重新初始化一下 */
		g_GprsComm.GprsStatus = GPRS_UART_FAILD;
	}
	switch(g_GprsComm.GprsStatus) {
		case GPRS_NORMAL:	/* 正常状态, 需要查询网络状态、卡状态、信号强度. */
			if(!GPRS_GetSIMCardState()) {				/* 检查卡状态 */
				g_GprsComm.GprsStatus = GPRS_NO_SIM;
			} else if(!GPRS_GetInternetState()			/* 检查网络状态 */
					|| (GPRS_GetSigStrong() == 99)) {	/* 检查信号强度 */
				g_GprsComm.GprsStatus = GPRS_NO_INTERNET;
			}
			break;
		case GPRS_UART_FAILD:
			GPRS_InitComm();
			g_GprsComm.GprsStatus = GPRS_NOT_INIT;	/* 故意不写break */
		case GPRS_NOT_INIT:			/* 还没初始化 */
			GPRS_Init();
			break;
		case GPRS_NO_SIM:			/* 未插SIM卡, 直接重新初始化模块 */
			g_GprsComm.GprsStatus = GPRS_NOT_INIT;
			break;
		case GPRS_NO_INTERNET:		/* 无法上网, 可能是网络信号差或未注册 */
			if((GPRS_GetSigStrong() == 99)) {	/* 查看信号强度 */
				g_GprsComm.GprsStatus = GPRS_NOT_INIT;
			} else if(GPRS_GetInternetState() && GPRS_OpenInternet() && GPRS_SIMCardNetConf()) {	/* 查看网络状态、打开网络附着状态、配置SIM卡PDP激活 */
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
#if DEBUG_PRINT_SIG
		printf("\n[p]\n");
#endif
		if(CheckGprsStatus() && ((protocl == IPPROTO_TCP) || (protocl == IPPROTO_UDP))) {
			for(int32 i = 0; i < MAX_GPRS_SOCKET_NUM; i++) {
				GPRS_SOCKET *pSocket = &g_GprsComm.Socket[i];
				if(pSocket->bIsFree) {
					sk = i+1;
					pSocket->u8MuxId = i;
					pSocket->u8Res = 0;
					pSocket->protocl = protocl;
					pSocket->bIsFree = FALSE;
					pSocket->bIsConnect = FALSE;
					pSocket->uRecvTimeoutMs = 5000;		/* 默认接收超时时间是5s */
					pSocket->uSendTimeoutMs = 1000;		/* 默认发送超时时间1s */
					pSocket->u32LastQirdiIndex = 0;
					pSocket->u32UnreadBLen = 0;
					break;
				}
			}
		}
		Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
		printf("\n[P]\n");
#endif
	}
	return sk;
}

int32 GprsConnect(SOCKET socketFd, struct sockaddr_in* pName, int32 i32Len)
{
	int32 i32Result = 0;
	GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
	if((socketFd <= 0) || (!pName) || (!i32Len)) {
		i32Result = GPRS_ERR_ARG;
	} else if(pSocket->bIsConnect) {	/* 当前socket已经连接上服务器了 返回错误即可 */
		i32Result = GPRS_ERR_CONNECTION_ABN;
	} else {
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
#if DEBUG_PRINT_SIG
			printf("\n[p]\n");
#endif
			if((g_GprsComm.GprsStatus == GPRS_NORMAL) && (!pSocket->bIsFree)) {
				/* 通过串口下发 */
				UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
				uint8* pBuf = pUartComm->u8TRBuf;
				char aChExStr[15] = "+QIOPEN: xx,x";		/* 期望的回复. 参1是socketId, 参2是errcode, 为0则正常. */
				PrintString(&pBuf, pBuf + 20, "AT+QIOPEN=");
				*pBuf++ = 1 + '0';			/* 场景ID */
				*pBuf++ = ',';
				if(pSocket->u8MuxId >= 10) {		/* Socket ID */
					*pBuf++ = aChExStr[9] = pSocket->u8MuxId / 10 + '0';
					*pBuf++ = aChExStr[10] = pSocket->u8MuxId % 10 + '0';
					aChExStr[11] = ',';
					aChExStr[12] = '0';
					aChExStr[13] = '\0';
				} else {
					*pBuf++ = aChExStr[9] = '0' + pSocket->u8MuxId;
					aChExStr[10] = ',';
					aChExStr[11] = '0';
					aChExStr[12] = '\0';
				}
				*pBuf++ = ',';
				*pBuf++ = '"';
				if(pSocket->protocl == IPPROTO_UDP) {	/* Socket 服务器类型 */
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
				PrintIPv4(&pBuf, pBuf+16, __rev(pName->sin_addr.s_addr));	/* 服务器IP */
				*pBuf++ = '"';
				*pBuf++ = ',';
				PrintU32(&pBuf, pBuf + 12, __rev16(pName->sin_port), 0);	/* 服务器端口 */
				*pBuf++ = ',';
				*pBuf++ = '0';		/* 本地端口 0：自动划分 */
				*pBuf++ = ',';
				*pBuf++ = '0';		/* 数据传输模式 0：缓存模式, 1：直吐模式, 2：透传模式 */
				*pBuf++ = '\r';
				WriteToUart(GPRS_UART_PORT, pBuf - pUartComm->u8TRBuf);
				/* 从串口读取返回内容,300ms内先会收到OK，然后10秒内会受到CONNECT OK或CONNECT FAIL */
				if(GPRS_SendATCmd(NULL, NULL, aChExStr, NULL, NULL, 10, 1000) == GPRS_SUC) {	/* 查看响应 */
					i32Result = GPRS_SUC;
					pSocket->bIsConnect = TRUE;
				} else {	/* 连接失败. */
					i32Result = GPRS_ERR_CONNECTION_ABN;
				}
			} else {
				i32Result = GPRS_ERR_OFFLINE;
			}
			Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
			printf("\n[P]\n");
#endif
		} else {
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}
//	pSocket->u8Res = i32Result;
	return i32Result;
}

#define MAX_ONCE_SEND_BLEN	1000
/* 阻塞式发送数据 */
int32 GprsSend(SOCKET socketFd, uint8 *pSrcBuf, uint16 uSendBlen, uint32 u32Flag)
{
	int32 i32Result = 0;
	GPRS_RES eRes;
	uint32 u32AllSentLen = 0;			/* 所有已经发送的字节数 */
	uint32 u32AllSendLen = uSendBlen;	/* 一共要发送的字节数 */
	GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd - 1];
	if((socketFd <= 0) || (!uSendBlen) || (!pSrcBuf)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
#if DEBUG_PRINT_SIG
			printf("\n[p]\n");
#endif
			if((g_GprsComm.GprsStatus == GPRS_NORMAL) && pSocket->bIsConnect) {
				UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
				uint8 aU8Cmd[30] = "AT+QISEND=xx,xxx\r";	/* 发送数据的AT指令，参数1：socketId，参数2：发送长度 */
				uint8 *pU8SendLenIndex = &aU8Cmd[10];
				if(pSocket->u8MuxId < 10) {
					*pU8SendLenIndex++ = pSocket->u8MuxId + '0';
				} else {
					*pU8SendLenIndex++ = pSocket->u8MuxId / 10 + '0';
					*pU8SendLenIndex++ = pSocket->u8MuxId % 10 + '0';
				}
				*pU8SendLenIndex++ = ',';
				/* 模块最大支持MAX_ONCE_SEND_BLEN字节分次发送 */
				for(; u32AllSentLen < u32AllSendLen;) {
					/* 大于MAX_ONCE_SEND_BLEN的分次发送 */
					uint16 uEverySendBlen;
					if(uSendBlen > MAX_ONCE_SEND_BLEN) {
						uEverySendBlen = MAX_ONCE_SEND_BLEN;
					} else {
						uEverySendBlen = uSendBlen;
					}
					/* 填充打印 */
					uint8* pBuf = pU8SendLenIndex;
					PrintU32(&pBuf, pBuf + 12, uEverySendBlen, 0);
					*pBuf++ = '\r';
					*pBuf++ = '\0';
					uint16 uSentLen = 0;
					if((eRes = GPRS_SendATCmd((char *)aU8Cmd, ">", NULL, NULL, NULL, 5, 200)) == GPRS_SUC) {	/* 响应成功 */
						/* 开始发送数据 */
						while(uSentLen < uEverySendBlen) {
							uint16 uSendLen = (uEverySendBlen - uSentLen) < UART_TR_BUF_BLEN ? (uEverySendBlen - uSentLen) : UART_TR_BUF_BLEN;
							memcpy(pUartComm->u8TRBuf, pSrcBuf + uSentLen + u32AllSentLen, uSendLen);
							WriteToUart(GPRS_UART_PORT, uSendLen);
							uSentLen += uSendLen;
						}
						if(GPRS_SendATCmd(NULL, "SEND OK", NULL, NULL, NULL, 5, 100) == GPRS_SUC) {	/* 发送成功 */
							u32AllSentLen += uEverySendBlen;
							uSendBlen -= uEverySendBlen;
						} else {	/* 发送失败 认为是断开连接 */
							i32Result = GPRS_ERR_CONNECTION_ABN;
							break;
						}
					} else if(eRes == GPRS_RES_FAIL) {		/* 响应FAIL */
						i32Result = GPRS_ERR_UART_ABN;		/* 发送缓冲区已满 */
					} else if(eRes == GPRS_RES_ERROR) {
						/* TCP连接断开 */
						i32Result = GPRS_ERR_CONNECTION_ABN;
						break;
					} else {	/* 其他响应 */
						i32Result = eRes;
						break;
					}
				}
			} else {	/* GPRS没网或者socket未连接 */
				i32Result = GPRS_ERR_CONNECTION_ABN;
			}
			Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
			printf("\n[P]\n");
#endif
		} else {	/* 没拿到锁 */
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}
	pSocket->u8Res = i32Result;
	return u32AllSentLen == 0 ? i32Result : u32AllSentLen;
}


#define MAX_QIRD_HEADER_LEN 100				/* 读指令的最大帧头帧尾长度 */
#define	MAX_READ_LEN		(UART_TR_BUF_BLEN - MAX_QIRD_HEADER_LEN)

/* 注意接收的长度 + 帧头帧尾 + 无用数据长度(均按照最大长度计算) 必须小于单个DMA的接收缓冲区大小.
 * 因为要确保接收的无用数据至少有一个完整的帧在缓冲区中, 方便去除 */
int32 GprsRecv(SOCKET socketFd, uint8 *pU8DstBuf, int32 i32ReadBLen, int32 i32Flags)
{
	int32 i32Result = 0;
	uint32 u32ReadAllBLen = 0;	/* 读取的总字节数 */
	GPRS_RES eRes = GPRS_SUC;
	GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
	if((socketFd <= 0) || (!i32ReadBLen) || (!pU8DstBuf)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		if(pSocket->bIsConnect) {		/* 连接正常再读 */
			/* 开始读数据 */
			UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
			char aChCmd[30] = "AT+QIRD=x,xxx\r";		/* 参数1：socketId，参数2：读取长度 */
			uint8* pU8ReadLenIndex = (uint8 *)&aChCmd[10];
			if(pSocket->u8MuxId < 10) {
				aChCmd[8] = pSocket->u8MuxId + '0';
			} else {
				aChCmd[8] = pSocket->u8MuxId / 10 + '0';
				aChCmd[9] = pSocket->u8MuxId % 10 + '0';
				aChCmd[10] = ',';
				pU8ReadLenIndex++;
			}
			uint32 u32StartTimeMs = HAL_GetTick();
			for(; (i32ReadBLen > 0) && (HAL_GetTick() - u32StartTimeMs < pSocket->uRecvTimeoutMs); ) {
				if(g_GprsComm.GprsStatus == GPRS_NORMAL) {
					uint16 uEveryReadBLen = i32ReadBLen > MAX_READ_LEN ? MAX_READ_LEN : i32ReadBLen;	/* 单次读取长度 */
					uint8* pBuf = pU8ReadLenIndex;
					uint16 uActualReadBLen = 0;				/* 实际读到的字节数 */
					PrintU32(&pBuf, pBuf+ 12, uEveryReadBLen, 0);
					*pBuf++ = '\r';
					*pBuf++ = '\0';
					pBuf = pUartComm->u8TRBuf;
					if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {	/* 先拿锁读数据 */
	#if DEBUG_PRINT_SIG
						printf("\n[p]\n");
	#endif
						/* 接收响应, 读到OK认为读取一帧完成. */
						uint16 uHead, uRear;		/* 队首索引 */
						if((eRes = GPRS_SendATCmd(aChCmd, "+QIRD: ", "\r\nOK", &uHead, &uRear, 10, 10)) == GPRS_SUC) {		/* 存在读一次分三次响应的情况. */
							if((pBuf = SkipStrInRingBuff(pBuf, (uint8 *)"+QIRD: ", uHead, uRear, UART_TR_BUF_BLEN))) {		/* 定位到数据长度的位置 */
								uActualReadBLen = ReadU32FromRingBuff(pUartComm->u8TRBuf, pBuf - pUartComm->u8TRBuf, uRear, UART_TR_BUF_BLEN);	/* 实际读到的字节数 */
								if(uActualReadBLen == 0) {	/* 没数据, 也认为是通信成功 */
									i32Result = GPRS_SUC;
								} else if((uActualReadBLen <= uEveryReadBLen)	/* 实际读到的字节数要小于等于期望读取的字节数 */
									&& (uActualReadBLen < (uRear - uHead + UART_TR_BUF_BLEN) % UART_TR_BUF_BLEN)				/* 实际读到的字节数要小于响应的总长度 */
									&& (pBuf = SkipStrInRingBuff(pUartComm->u8TRBuf, (uint8*)"\r\n", pBuf - pUartComm->u8TRBuf, uRear, UART_TR_BUF_BLEN)))	/* 数据区域的开始位置 */
								{
									i32ReadBLen -= uActualReadBLen;			/* 更新剩余字节数 */
									u32ReadAllBLen += uActualReadBLen;		/* 更新读到的总字节数 */
									if(u32ReadAllBLen > MQTT_TR_BUF_BLEN) {
										NOP;
									}
									/* 将响应数据拷贝到用户缓冲区 */
									uHead = pBuf - pUartComm->u8TRBuf;
									if((pUartComm->u8TRBuf[(uHead + uActualReadBLen) % UART_TR_BUF_BLEN] == '\r')
										&& (pUartComm->u8TRBuf[(uHead + uActualReadBLen + 1) % UART_TR_BUF_BLEN] == '\n')) {
										/* 读取完之后的两个字符必是\r\n. */
										if((uHead <= uRear)
											|| (UART_TR_BUF_BLEN - uHead >= uActualReadBLen)) {	/* 一次cpy完成 */
											memcpy(pU8DstBuf, pBuf, uActualReadBLen);
										} else {		/* 需要两次cpy */
											uint16 CpyLen = UART_TR_BUF_BLEN - uHead;
											memcpy(pU8DstBuf, pBuf, CpyLen);		/* 拷贝第一段 */
											memcpy(pU8DstBuf + CpyLen, pUartComm->u8TRBuf, uActualReadBLen - CpyLen);	/* 拷贝第二段 */
										}
										pU8DstBuf += uActualReadBLen;
										i32Result = GPRS_SUC;
									} else {	/* 无法识别数据帧 */
										i32Result = GPRS_ERR_PARSE;
									}
								} else {	/* 无法识别数据帧 */
									i32Result = GPRS_ERR_PARSE;
								}
							} else {	/* 无法识别数据帧 */
								i32Result = GPRS_ERR_PARSE;
							}
						} else if(eRes == GPRS_RES_ERROR) {		/* 收到了ERROR响应 */
							i32Result = GPRS_ERR_CONNECTION_ABN;	/* 断开连接 */
						} else {	/* 其他响应. */
							i32Result = eRes;
						}
						Semaphore_post(g_GprsComm.Sem_GprsReq);
	#if DEBUG_PRINT_SIG
						printf("\n[P]\n");
	#endif
						if(i32Result == GPRS_SUC) {
							if(uActualReadBLen == 0) {	/* 没读到数据 */
								Task_sleep(10);
							}
						} else {	/* 通信错误，退出. */
							break;
						}
					} else {	/* 没拿到锁 */
						i32Result = GPRS_ERR_TIMEOUT;
					}
				} else {	/* 连接错误 */
					i32Result = GPRS_ERR_CONNECTION_ABN;
					break;
				}
			}
		} else {	/* 套接字不可用 */
			i32Result = GPRS_ERR_OFFLINE;
		}
	}
	pSocket->u8Res = i32Result;
	if(i32Result == GPRS_SUC) {
		return u32ReadAllBLen;
	} else {
		return i32Result;
	}
}

int32 GprsClose(SOCKET socketFd)
{
	int32 i32Result = 0;
	if(socketFd <= 0) {
		i32Result = GPRS_ERR_ARG;
	} else {
		GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
#if DEBUG_PRINT_SIG
			printf("\n[p]\n");
#endif
			if(g_GprsComm.GprsStatus == GPRS_NORMAL) {
				char aChCmd[30];
				uint8* pBuf = (uint8 *)aChCmd;
				PrintStringNoOvChk(&pBuf, "AT+QICLOSE=");
				if(pSocket->u8MuxId < 10) {
					*pBuf++ = pSocket->u8MuxId + '0';
				} else {
					*pBuf++ = pSocket->u8MuxId / 10 + '0';
					*pBuf++ = pSocket->u8MuxId % 10 + '0';
				}
				*pBuf++ = '\r';
				*pBuf++ = '\0';
				i32Result = GPRS_SendATCmd(aChCmd, "OK", "OK", NULL, NULL, 5, 500);
			}
			pSocket->bIsFree = TRUE;
			pSocket->bIsConnect = FALSE;
			Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
			printf("\n[P]\n");
#endif
		} else {
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}
	return i32Result;
}

/* 目前仅支持配置收发超时时间, 对于发送来说配置是无效的, send函数只是把数据写到M25的发送缓冲区中, 并不会等待数据发送出去再返回那样太耗费时间了.
 * 因此对于发送来说这个超时时间没有任何意义.只有接收时才有真正意义上的超时 */
int32 GprsSetSockopt(SOCKET socketFd, int32 u32Level, int32 u32Op, void *pbuf, int32 i32Bufsize)
{
	if(socketFd <= 0) {
		return -1;
	}
	GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
	switch(u32Level) {
		case SOL_SOCKET:
			struct timeval *pTimeout = (struct timeval *)pbuf;
			switch(u32Op) {
				case SO_SNDTIMEO:
					pSocket->uSendTimeoutMs = pTimeout->tv_sec * 1000 + (int)((pTimeout->tv_usec + 999) / 1000);
					break;
				case SO_RCVTIMEO:
					pSocket->uRecvTimeoutMs = pTimeout->tv_sec * 1000 + (int)((pTimeout->tv_usec + 999) / 1000);
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
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

/* 高效解析逗号分隔的4个整型, 只服务于特定函数, 并不通用(并没有溢出判断, 但是函数内条件很苛刻, 应该不存在溢出) */
BOOL ParseQuadU16(const char* str, uint16* s1, uint16* s2, uint16* s3, uint16* s4) {
    *s1 = 0; *s2 = 0; *s3 = 0; *s4 = 0;

    // 解析第一个数字
    while(*str && *str >= '0' && *str <= '9') {
        *s1 = *s1 * 10 + (*str - '0');
        str++;
    }
    if(*str == ',') {
    	str++;
    } else {	/* 帧错误 */
    	return FALSE;
    }

    // 解析第二个数字
    while(*str && *str >= '0' && *str <= '9') {
        *s2 = *s2 * 10 + (*str - '0');
        str++;
    }
    if(*str == ',') {
		str++;
	} else {	/* 帧错误 */
		return FALSE;
	}

    // 解析第三个数字
    while(*str && *str >= '0' && *str <= '9') {
        *s3 = *s3 * 10 + (*str - '0');
        str++;
    }
    if(*str == ',') {
		str++;
	} else {	/* 帧错误 */
		return FALSE;
	}

    // 解析第四个数字
    while(*str && *str >= '0' && *str <= '9') {
        *s4 = *s4 * 10 + (*str - '0');
        str++;
    }
    if(*str) {	/* 正常来说解析完是不会到字符串的结尾的 */
    	return TRUE;
    }
    return FALSE;
}

/* 从环形缓冲区中读取uint32类型的数据 */
uint32 ReadU32FromRingBuff(uint8* pSrc, uint16 uHead, uint16 uRear, uint16 uRingBuffLen)
{
	uint32 u32Data = 0;
	for(; uHead != uRear; ) {
		uint8 u8Val = pSrc[uHead];
		if(('0' <= u8Val) && (u8Val <= '9')) {
			u32Data = u32Data*10 + ((uint32)(u8Val - '0'));
		} else {
			break;
		}
		uHead = (uHead + 1) % uRingBuffLen;
	}
	return u32Data;
}

/* 去除掉没用的信息: (+QIURC: "<通知类型>",...) 这个消息是GPRS多连接模式时主动上报的, 无法关闭.
 * 这个信息还有可能包含在消息帧中并不是单独发送的, 所以还不能忽略掉, 在这里给它去除.
 * 注：这个函数专门服务于ReadFromUart通信中的GPRS, 内容是写死的, 并不通用
 * 通知类型分为：
 * closed：连接断开通知，当TCP Socket服务连接被远程客户端断开或者因为网络异常导致断开，模块将上报该URC，同时该Socket 将处于"Closing"状态（<socket_state>=4）
 * recv：数据接收通知，缓存模式下：URC格式为+QIURC: "recv",<connectID>；URC上报后，Host可通过AT+QIRD读取数据。
  		请注意，如果缓存不为空且模块再次接收数据的情况下，只有当Host通过AT+QIRD读取所有接收的数据后，模块才会上报新的URC。
 * 	incoming full：客户端连接已满通知，如果客户端连接已达限额，或者已经没有Socket系统资源可供分配，有新的客户端连接请求时模块会
		上报URC：+QIURC: "incoming full"
 * 	incoming：客户端连接通知，如果<service_type>为"TCP LISTENER"，当一个远程客户端连接到这个服务器时，Host会给新连接
		自动分配一个空闲的<connectID>，其中<connectID>范围是 0~11。此时模块会上报该 URC。新连接的
		<service_type>是"TCP INCOMING"，<access_mode>是缓存模式。
 * 	pdpdeact：PDP去激活通知，PDP可以被网络去激活。PDP被去激活以后，模块会上报该URC通知Host，Host需执行AT+QIDEACT
		去激活场景并重置所有连接。
 *  */
#define IS_BETWEEN(idx, head, rear) \
    (((head) <= (rear)) ? ((idx) >= (head) && (idx) < (rear)) : ((idx) >= (head) || (idx) < (rear)))	/* 检查索引值是否在队列里 */

uint16 StrRemoveHashData(uint8 *pSrc, uint16 uHead, uint16 uRear, uint16 uRingBuffLen)
{
	uint8 *pStart = pSrc + uHead;
	uint8 *pSrcEnd = pSrc + uRear;
	uint8 *pU8URCEnd, *pU8URCStart;		/* URC帧结束位置 */

	while((pU8URCStart = SkipStrInRingBuff(pSrc, (uint8 *)"+QIURC: ", uHead, uRear, uRingBuffLen))
		&& (pU8URCEnd = SkipStrInRingBuff(pSrc, (uint8 *)"\r\n", pU8URCStart - pSrc, uRear, uRingBuffLen))) {	/* 有主动上报的数据帧 */
		pStart += 1;			/* 后移一位到主动上报数据的数据类型 */
		uint8 *pU8StrIndex;
		uint16 uFHead = pU8URCStart - pSrc;
		uint16 uFRear = pU8URCEnd - pSrc;
		if((pU8StrIndex = SkipStrInRingBuff(pSrc, (uint8 *)"closed", uFHead, uFRear, uRingBuffLen))) {		/* 有socket断开了 */
			pU8StrIndex += 2;			/* 移动到参数部分 */
			uint8 u8SocketId = (uint8)ReadU32FromRingBuff(pSrc, pU8StrIndex - pSrc, uRear, uRingBuffLen);
			if(u8SocketId < MAX_GPRS_SOCKET_NUM) {
				g_GprsComm.Socket[u8SocketId].bIsConnect = FALSE;			/* 断开连接 */
			}
		} else if((pU8StrIndex = SkipStrInRingBuff(pSrc, (uint8 *)"recv", uFHead, uFRear, uRingBuffLen))) {	/* 有socket收到数据了 */
			pU8StrIndex += 2;			/* 移动到参数部分 */
//			uint8 u8SocketId = (uint8)ReadU32FromRingBuff(pSrc, pU8StrIndex - pSrc, uRear, uRingBuffLen);
		} else if(SkipStrInRingBuff(pSrc, (uint8 *)"dnsgip", uFHead, uFRear, uRingBuffLen)) {	/* DNS类型，不需要屏蔽 */
			pStart = pU8URCEnd;
		} else {	/* 其他类型，屏蔽就行，暂时不需要多余操作 */

		}

		/* 将URC数据帧去除 */
		uFHead = (uFHead - 11 + uRingBuffLen) % uRingBuffLen;				/* 移动到\r\n处() */
		if(!IS_BETWEEN(uFHead, uHead, uRear)) {	/* 向上溢出 */
			uFHead = uHead;
		}
		pU8URCStart = pSrc + uFHead;
		uint16 uCoverLen = (pU8URCEnd - pU8URCStart + uRingBuffLen) % uRingBuffLen;		/* 需要拷贝的长度 */
		/* 开始拷贝 */
		memcpy(pStart, pU8URCEnd, pSrcEnd - pU8URCEnd);
		/* 更新数据 */
//		if(uDataLen < uCoverLen) {	/* 溢出了, 实际应该不会有这种情况, 不过为了安全还是判断一下. */
//			return 0;
//		}
//		uDataLen -= uCoverLen;
		pSrcEnd -= uCoverLen;
	}
	return 0;
}

#else
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
#if DEBUG_PRINT_SIG
		printf("\n[p]\n");
#endif
		if(GPRS_SendATCmd(aChCmd, "+QISACK: ", "OK", &uRxLen, 2, 2000)
			&& (pBuff = SkipCharInString(pStart, pStart + uRxLen, ',', 2))) {	/* 响应成功 */
			uAcked = ReadU32(&pBuff, pBuff + 5);
		}
		Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
		printf("\n[P]\n");
#endif
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
	return ;
	for(uint16 i = 0; i < MAX_GPRS_SOCKET_NUM; i++) {
		GPRS_SOCKET *pSocket = &g_GprsComm.Socket[i];
		pSocket->name = NULL;
		pSocket->protocl = 0;
		pSocket->u8MuxId = 0;
		pSocket->u8Res = 0;
		pSocket->bIsFree = TRUE;
		pSocket->u8MuxId = i;
		pSocket->u32LastQirdiIndex = 0;
		pSocket->u32UnreadBLen = 0;
	}
	GPRS_ClearBuffer();								/* 清空GPRS的接收缓冲区 */
	OpenUartComm(GPRS_UART_PORT, 115200, 0, 30);	/* 与GPRS通信串口 */

	GPRS_Reset();	/* 复位模块 */
	GPRS_ClearBuffer();								/* 清空GPRS的接收缓冲区 */

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
	return FALSE;
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
#if DEBUG_PRINT_SIG
		printf("\n[p]\n");
#endif
		if(CheckGprsStatus() && ((protocl == IPPROTO_TCP) || (protocl == IPPROTO_UDP))) {
			for(int32 i = 0; i < MAX_GPRS_SOCKET_NUM; i++) {
				GPRS_SOCKET *pSocket = &g_GprsComm.Socket[i];
				if(pSocket->bIsFree) {
					sk = i+1;
					pSocket->u8MuxId = i;
					pSocket->u8Res = 0;
					pSocket->protocl = protocl;
					pSocket->bIsFree = FALSE;
					pSocket->bIsConnect = FALSE;
					pSocket->uRecvTimeoutMs = 5000;		/* 默认接收超时时间是5s */
					pSocket->uSendTimeoutMs = 1000;		/* 默认发送超时时间1s */
					pSocket->u32LastQirdiIndex = 0;
					pSocket->u32UnreadBLen = 0;
					break;
				}
			}
		}
		Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
		printf("\n[P]\n");
#endif
	}
	return sk;
}

int32 GprsConnect(SOCKET socketFd, struct sockaddr_in* pName, int32 i32Len)
{
	int32 i32Result = 0;
	GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
	if((socketFd <= 0) || (!pName) || (!i32Len)) {
		i32Result = GPRS_ERR_ARG;
	} else if(pSocket->bIsConnect) {	/* 当前socket已经连接上服务器了 返回错误即可 */
		i32Result = GPRS_ERR_CONNECTION_ABN;
	} else {
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
#if DEBUG_PRINT_SIG
			printf("\n[p]\n");
#endif
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
					pSocket->bIsConnect = TRUE;
				} else {	/* 连接失败. */
					i32Result = GPRS_ERR_CONNECTION_ABN;
				}
			} else {
				i32Result = GPRS_ERR_OFFLINE;
			}
			Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
			printf("\n[P]\n");
#endif
		} else {
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}
//	pSocket->u8Res = i32Result;
	return i32Result;
}

/* 阻塞式发送数据 */
int32 GprsSend(SOCKET socketFd, uint8 *pSrcBuf, uint16 uSendBlen, uint32 u32Flag)
{
#define MAX_ONCE_SEND_BLEN	1000
	int32 i32Result = 0;
	uint32 u32AllSentLen = 0;			/* 所有已经发送的字节数 */
	uint32 u32AllSendLen = uSendBlen;	/* 一共要发送的字节数 */
	GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
	if((socketFd <= 0) || (!uSendBlen) || (!pSrcBuf)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
#if DEBUG_PRINT_SIG
			printf("\n[p]\n");
#endif
			if((g_GprsComm.GprsStatus == GPRS_NORMAL) && pSocket->bIsConnect) {
				UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
				/* 模块最大支持MAX_ONCE_SEND_BLEN字节分次发送 */
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
					/* 这个有时候发送一次并不回复'>'号, 不清楚什么原因因此这里最多发送几次 */
					if(GPRS_SendATCmd((char *)aU8Cmd, ">", NULL, &uReadNum, 2, 2000)) {	/* 响应成功 */
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
							break;
						}
					} else if(uReadNum != 0) {	/* 响应了其他消息 */
						if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"SEND FAIL")) {
							/* 连接存在 但是发送失败 认为串口有问题 */
							i32Result = GPRS_ERR_UART_ABN;
							break;
						} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"SEND FAIL")
								|| SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8 *)"ERROR")) {
							/* 未建立TCP连接 */
							i32Result = GPRS_ERR_CONNECTION_ABN;
							break;
						} else {	/* 不能解析的响应 */
							i32Result = GPRS_ERR_PARSE;
						}
					} else {	/* 未响应 */
						i32Result = GPRS_ERR_NO_RESP;
						break;
					}
				}
			} else {	/* GPRS没网或者socket未连接 */
				i32Result = GPRS_ERR_CONNECTION_ABN;
			}
			Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
			printf("\n[P]\n");
#endif
		} else {	/* 没拿到锁 */
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}
	pSocket->u8Res = i32Result;
	if(u32AllSentLen == u32AllSendLen) {	/* 发送完成 */
		return u32AllSentLen;
	} else {	/* 发送失败 */
		pSocket->bIsConnect = FALSE;
		return i32Result;
	}
}


#define MAX_QIRD_HEADER_LEN 100				/* 读指令的最大帧头帧尾长度 */
#define	MAX_READ_LEN		(UART_TR_BUF_BLEN - MAX_QIRD_HEADER_LEN)

/* 注意接收的长度 + 帧头帧尾 + 无用数据长度(均按照最大长度计算) 必须小于单个DMA的接收缓冲区大小.
 * 因为要确保接收的无用数据至少有一个完整的帧在缓冲区中, 方便去除 */
int32 GprsRecv(SOCKET socketFd, uint8 *pU8DstBuf, int32 i32ReadBLen, int32 i32Flags)
{
	int32 i32Result = 0;
	uint32 u32ResultLen = 0;	/* 读取的字节数 */
	GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
	if((socketFd <= 0) || (!i32ReadBLen) || (!pU8DstBuf)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		/* 如果没读到，重新申请信号量再试，最多重试2次 */
		uint32 u32StartTimeMs = HAL_GetTick();
		while((HAL_GetTick() - u32StartTimeMs < pSocket->uRecvTimeoutMs) && i32ReadBLen > 0) {	/* 没超时并且没读到规定的数据继续读 */
			if(pSocket->bIsConnect) {		/* 连接正常再读 */
				if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {	/* 先拿锁读数据 */
#if DEBUG_PRINT_SIG
				printf("\n[p]\n");
#endif
					/* 有数据再读, 没数据需要保持与M25通信. 因为M25的主动上报并不是完全主动的,
					 * 而是会在通信过程中"顺便夹带"当前socket是否有新数据接收的数据. 因此这里发送AT指令保持通信 */
					if(!pSocket->u32UnreadBLen) {
						GPRS_SendATCmd("AT\r", "OK", "OK", NULL, 3, 100);
					} else {
						/* 以uart buf大小为单元循环查询读取M25缓存中全部长度数据（M25每socket最大缓存400K） */
						UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
						/* 本次需要读取的字节数 */
						uint16 uReadBLen = i32ReadBLen > pSocket->u32UnreadBLen ? pSocket->u32UnreadBLen : i32ReadBLen;
						char aChCmd[30];
						for(; uReadBLen > 0 ; ) {	/* 循环接收剩余字节 */
							uint16 uEveryReadBLen = uReadBLen > MAX_READ_LEN ? MAX_READ_LEN : uReadBLen;	/* 单次读取长度 */
							uint8* pBuf = (uint8 *)aChCmd;
							uint16 uUartReadBLen = 0;
							PrintString(&pBuf, pBuf + 20, "AT+QIRD=0,1,");
							*pBuf++ = '0' + pSocket->u8MuxId;
							*pBuf++ = ',';
							PrintU32(&pBuf, pBuf+ 12, uEveryReadBLen, 0);
							*pBuf++ = '\r';
							*pBuf++ = '\0';
							pBuf = pUartComm->u8TRBuf;
							/* 接收响应, 读到OK认为读取一帧完成. */
							if(GPRS_SendATCmd(aChCmd, "+QIRD:", "OK", &uUartReadBLen, 3, 1000)) {	/* 存在读一次分三次响应的情况. */
								if((pBuf = SkipCharInString(pBuf, pBuf + uUartReadBLen, ',', 2))) {	/* 定位到数据长度的位置 */
									uint16 uRecvFrameLen = ReadU32(&pBuf, pBuf + 5);	/* QIRD接收内容长度 */
									/* 数据有效 */
									if((uEveryReadBLen == uRecvFrameLen)	/* 能进来说明一定有这么多数据, 如果读到的数据没这么多按照错误处理 */
										&& (uRecvFrameLen < uUartReadBLen)
										&& (pBuf = SkipStrInString(pBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"\r\n")))
									{
										i32ReadBLen -= uRecvFrameLen;		/* 更新剩余字节数 */
										uReadBLen -= uRecvFrameLen;			/* 更新本次需要读取的字节数 */
										u32ResultLen += uRecvFrameLen;		/* 更新读到字节数 */
										if(u32ResultLen > MQTT_TR_BUF_BLEN) {
											NOP;
										}
										pSocket->u32UnreadBLen -= uRecvFrameLen;
										for(; uRecvFrameLen > 0; uRecvFrameLen--) {	/* 将响应数据拷贝到用户缓冲区 */
											*pU8DstBuf++ = *pBuf++;
										}
										i32Result = GPRS_SUC;
									} else {	/* 无法识别数据帧 */
										i32Result = GPRS_ERR_PARSE;
										break;
									}
								} else {	/* 无法识别数据帧 */
									i32Result = GPRS_ERR_PARSE;
									break;
								}
							} else if(uUartReadBLen != 0) {	/* 收到了其他类型的响应 */
								if((uUartReadBLen == 6
								&& SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"OK"))) {	/* 只有OK 可能数据丢失了 直接重连 */
									i32Result = GPRS_ERR_UART_ABN;
									break;
								} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"CLOSE")
										|| SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"ERROR")
										|| SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"+PDP DEACT")) {	/* 是否断开连接 */
									i32Result = GPRS_ERR_CONNECTION_ABN;	/* 断开连接 */
									break;
								} else {	/* 响应了其他未识别的消息 */
									i32Result = GPRS_ERR_PARSE;
									break;
								}
							} else {	/* 设备无响应 可能卡死或者DMA有问题 重新连接. */
								i32Result = GPRS_ERR_NO_RESP;
								break;
							}
						}
					}
					Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
					printf("\n[P]\n");
#endif
				} else {	/* 没拿到锁或没数据, 继续读 */
					i32Result = GPRS_SUC;
				}
			} else {	/* 套接字不可用 */
				i32Result = GPRS_ERR_OFFLINE;
				break;
			}
			if(i32Result < 0) {		/* 有一次通信失败就退出重连, 并返回错误 */
				break;
			}
			Task_sleep(10);		/* 没拿到锁, 或者没读到数据在这里休眠一会继续读 */
		}
	}
	pSocket->u8Res = i32Result;
	if(i32Result == GPRS_SUC) {					/* 通信成功返回读到的字节数 */
		return u32ResultLen;
	} else if(i32Result == GPRS_ERR_TIMEOUT) {	/* 超时不算错误 */
		return i32Result;
	} else {				/* 通信错误, 大概率是connect断开了 */
		pSocket->bIsConnect = FALSE;
		return i32Result;
	}
}

int32 GprsClose(SOCKET socketFd)
{
	int32 i32Result = 0;
	if(socketFd <= 0) {
		i32Result = GPRS_ERR_ARG;
	} else {
		GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
#if DEBUG_PRINT_SIG
			printf("\n[p]\n");
#endif
			if((g_GprsComm.GprsStatus == GPRS_NORMAL)) {
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
				} else {		/* 设备不响应 */
					i32Result = GPRS_ERR_NO_RESP;
				}
				pSocket->bIsFree = TRUE;
				pSocket->bIsConnect = FALSE;
			}
			Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
			printf("\n[P]\n");
#endif
		} else {
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}
	return i32Result;
}

/* 目前仅支持配置收发超时时间, 对于发送来说配置是无效的, send函数只是把数据写到M25的发送缓冲区中, 并不会等待数据发送出去再返回那样太耗费时间了.
 * 因此对于发送来说这个超时时间没有任何意义.只有接收时才有真正意义上的超时 */
int32 GprsSetSockopt(SOCKET socketFd, int32 u32Level, int32 u32Op, void *pbuf, int32 i32Bufsize)
{
	if(socketFd <= 0) {
		return -1;
	}
	GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
	switch(u32Level) {
		case SOL_SOCKET:
			struct timeval *pTimeout = (struct timeval *)pbuf;
			switch(u32Op) {
				case SO_SNDTIMEO:
					pSocket->uSendTimeoutMs = pTimeout->tv_sec * 1000 + pTimeout->tv_usec;
					break;
				case SO_RCVTIMEO:
					pSocket->uRecvTimeoutMs = pTimeout->tv_sec * 1000 + pTimeout->tv_usec;
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
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

/* 高效解析逗号分隔的4个整型, 只服务于特定函数, 并不通用(并没有溢出判断, 但是函数内条件很苛刻, 应该不存在溢出) */
BOOL ParseQuadU16(const char* str, uint16* s1, uint16* s2, uint16* s3, uint16* s4) {
    *s1 = 0; *s2 = 0; *s3 = 0; *s4 = 0;

    // 解析第一个数字
    while(*str && *str >= '0' && *str <= '9') {
        *s1 = *s1 * 10 + (*str - '0');
        str++;
    }
    if(*str == ',') {
    	str++;
    } else {	/* 帧错误 */
    	return FALSE;
    }

    // 解析第二个数字
    while(*str && *str >= '0' && *str <= '9') {
        *s2 = *s2 * 10 + (*str - '0');
        str++;
    }
    if(*str == ',') {
		str++;
	} else {	/* 帧错误 */
		return FALSE;
	}

    // 解析第三个数字
    while(*str && *str >= '0' && *str <= '9') {
        *s3 = *s3 * 10 + (*str - '0');
        str++;
    }
    if(*str == ',') {
		str++;
	} else {	/* 帧错误 */
		return FALSE;
	}

    // 解析第四个数字
    while(*str && *str >= '0' && *str <= '9') {
        *s4 = *s4 * 10 + (*str - '0');
        str++;
    }
    if(*str) {	/* 正常来说解析完是不会到字符串的结尾的 */
    	return TRUE;
    }
    return FALSE;
}

/* 去除掉没用的信息: (+QIRDI: ....) 这个消息是GPRS多连接模式时主动上报的, 无法关闭.
 * 这个信息还有可能包含在消息帧中并不是单独发送的, 所以还不能忽略掉, 在这里给它去除.
 * 注：这个函数专门服务于ReadFromUart通信中的GPRS, 内容是写死的, 并不通用
 * QIRDI消息格式：+QIRDI: 0,1,3,1,4,4
 * 数1：前置场景一般都是0;数2：作为clietn还是server 1是client 2是server;数3：当前sokcet编号;
 * 数4：当前消息的编号;数5：本次接收的长度;数6：当前sokcet剩余未读取长度; */
uint16 StrRemoveHashData(uint8 *pSrc, uint16 uDataLen)
{
	uint8 *pStart = pSrc;
	uint8 *pSrcEnd = pSrc + uDataLen;
	uint8 *pEnd;
	if(uDataLen < 15) {	/* 数据过短不可能存在+QIRDI:数据帧 */
		return uDataLen;
	}
	while((pStart = SkipStrInString(pStart, pSrcEnd, (uint8 *)"+QIRDI: "))) {
		if(*pStart == '0') {
			if(*(pStart + 2) == '1') {	/* 应该是作为client接收的数据 */
				uint16 uSocketId, uMsgId;		/* 当前socket id、消息id */
				uint16 uBlockBLen, uTotalBLen;	/* 本次接收的长度、此socket剩余未读取的长度 */
				*pSrcEnd = '\0';	/* 防止溢出 */
				/* 解析QIRDI帧 */
				if(ParseQuadU16((char *)(pStart + 4), &uSocketId, &uMsgId, &uBlockBLen, &uTotalBLen)
				&& (uSocketId < MAX_GPRS_SOCKET_NUM)
				&& (uBlockBLen <= uTotalBLen)) {
//				&& (uMsgId == (g_GprsComm.Socket[uSocketId].u32LastQirdiIndex + 1))) {	/* 读取成功 */
					g_GprsComm.Socket[uSocketId].u32LastQirdiIndex = uMsgId;
					g_GprsComm.Socket[uSocketId].u32UnreadBLen = uTotalBLen;
				} else {	/* 最坏的情况, 该字段被分开了.剩余部分可能在另外半个缓冲区中 */
					NOP;
				}
			}
		}
		/* 这帧数据中存在无用数据准备去除 */
		if(!(pEnd = SkipStrInString(pStart, pSrcEnd, (uint8 *)"\r\n"))) {	/* 有+QIRDI:但是没找到\r\n, 就没办法确定结束位置, 只能返回不做处理了. */
			return uDataLen;
		}
		pStart -= 10;	/* 移动到\r\n处() */
		pStart = pStart < pSrc ? pSrc : pStart;    /* 防止向上溢出 */
		/* 覆盖掉的长度 */
		uint16 uCoverLen = pEnd - pStart;
		if(pSrcEnd - pEnd > 0) {

		}
		/* 开始拷贝 */
		memcpy(pStart, pEnd, pSrcEnd - pEnd);
		/* 更新数据 */
		if(uDataLen < uCoverLen) {	/* 溢出了, 实际应该不会有这种情况, 不过为了安全还是判断一下. */
			return 0;
		}
		uDataLen -= uCoverLen;
		pSrcEnd -= uCoverLen;
	}
	return uDataLen;
}
#endif
