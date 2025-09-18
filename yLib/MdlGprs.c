/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved. ģ��V1.3
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
/* ��Ŀ����ͷ�ļ� */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"

/* ����ϵͳͷ�ļ� */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/* �����Լ��¼�ģ��ͷ�ļ� */
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
#define DEBUG_PRINT_SIG		FALSE		/* �ź������Դ�ӡ */
/*==========================================================================
| Description	: ���ַ����в��������ַ����������������ַ���ĩβָ�롣
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
| Description	: Gprs M25��Զģ��ͨ��
| G/Out var		:
| Author		: Su Mingyang			Date	: 2024-01-29
\=========================================================================*/
#define MuxId(skt)	((((GPRS_SOCKET*)skt) - &g_GprsComm) / sizeof(GPRS_COMM))
#define GPRS_UART_TIMEOUT	500
#define MAX_GPRS_RETRY_CNT	5		/* ��ʱ��Ҫ���ͨ�ţ���������Դ��� */
/* ���GPRS����״̬������������ȳ��Ը�λ���³�ʼ�����������ʧ����˴β��ٳ���
 * �˺���������ͬ����ͬ�������ⲿ����
 * ����GPRS����״̬�Ƿ�����
 * ��ضϿ�stlink��3��3v�� */

/* ����ATָ��ж���Ӧ�Ƿ���ȷ, ���ҿ���ָ��������, ��û�յ�������ʱ���ٴγ��Խ���, ֱ���յ���������Ϊһ֡���ݽ������.ʹ�ò�5ָ�����Դ���.
 * ��1: Ҫ���͵�ATָ��, ����ΪNULL, ΪNULL��ֱ�Ӷ���Ӧ�Ƿ����, ���ָᷢ��.
 * 		ע: ָ��Ϊ�ַ���, ��˱�����\0��β, ����Ҫ����ָ��Ľ�����\r
 * ��2: ��������Ӧ, ��һ��û�ж������Ҳ�5ָ���˶�γ���, �ͻ��ٴη��ʹ�ָ���ٴζ�, ע�����3����.����ΪNULL, ΪNULLʱֻҪ�������ݾͷ���TRUE, ����������FALSE.
 * ��3: ������, ��ʱһ֡�����Ƿ����η���, �����Ҫ��������, ֱ�����յ�ָ�����ַ�����Ϊ������һ֡ (��ͨ�Ź���Ƶ��ʱ, �󲿷ֹ�������Ӧ��������η���).
 * 	ע: ���Դ���������ڵ���2, ��Ϊ���һ֡û������, ��Ҫ�ٴν���, ����ָ��ֻ�ᷢ��һ��.
 * 		����ΪNULL, ΪNULLʱ����һ֡���ݾͷ�����.
 * ��4: ���յ���Ӧ�ֽڳ���, �����Ҫ��ȡ��Ӧ�ĳ���, �ʹ���, ����Ҫ��NULL����
 * ��5: ���Դ���.
 * ��6: ��ʱʱ��, ע���ǵ��ζ�ȡ�ĳ�ʱʱ�� */
BOOL GPRS_SendATCmd(const char *cnst_pChCmd, const char *cnst_pChExpectResp, const char *cnst_pChEndStr, uint16 *uRxBufLen, uint8 u8TryCnt, uint16 uTimeoutMs)
{
    uint8* pStart = g_UartComm[GPRS_UART_PORT].u8TRBuf;
	uint8* pBuf;
    uint16 uRxLen = 0;		/* ���յ������ݳ��� */
    BOOL bRet = FALSE;		/* ����ֵ */
    BOOL bFlag = TRUE;		/* �Ƿ���Ҫ���·�������ı�־ */
	while(u8TryCnt--) {
		if(cnst_pChCmd && bFlag) {	/* ��Ҫ����ָ�� */
			pBuf = pStart;
			/* ���ATָ�� */
			PrintString(&pBuf, pBuf + 20, cnst_pChCmd);	/* ע�����ָ��ĳ��Ȳ��ǿɱ��, ������Ҫ����, Ҳ����ֱ������Ϊ���. */
			/* �������� */
			WriteToUart(GPRS_UART_PORT, pBuf - pStart);
		}
		/* ��ȡ��Ӧ */
		if((uRxLen = ReadFromUart(GPRS_UART_PORT, uTimeoutMs))) {
			if(cnst_pChEndStr == NULL) {	/* û�н������ */
				if((cnst_pChExpectResp == NULL) || SkipStrInString(pStart, pStart + uRxLen, (uint8 *)cnst_pChExpectResp)) {
					bRet = TRUE;
					break;
				}
			} else {	/* �����˽������ */
				bFlag = FALSE;	/* �����ٴη���ָ���� */
				uRxLen += g_UartComm[GPRS_UART_PORT].uRxFrameIndex;
				if(SkipStrInString(pStart, pStart + uRxLen, (uint8 *)cnst_pChEndStr)) {	/* �յ��˽������ */
					if((cnst_pChExpectResp == NULL) || SkipStrInString(pStart, pStart + uRxLen, (uint8 *)cnst_pChExpectResp)) {
						bRet = TRUE;
					} else {	/* �������յ��˽������, ����������Ӧ����, ����false */
						bRet = FALSE;
					}
					break;
				} else {	/* û�յ��������, ��������. �����¼һ������ֵ, ��������ʱ�Ӹô���������. */
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
    return bRet;  /* ��ʱ��δ�յ�������Ӧ */
}

/* ��ջ�����, ��Ϊʹ�û��λ��������յ�����, Ƶ������ATָ����ܻᵼ����Ϣ��λ
 * ���ÿ�γ�ʼ����ʱ��������һ�»����� */
void GPRS_ClearBuffer(void)
{
	g_GPRSRingBuffer.uHead = g_GPRSRingBuffer.uRear;
	memset(g_GPRSRingBuffer.aU8Buffer, 0, RING_BUFFER_MAX_SIZE);
}

/* ��ʼ�����λ����� */
void RingBuffer_Init(RingBuffer_t *pRingBuffer)
{
	pRingBuffer->uRear = 0;
	pRingBuffer->uHead = 0;
	memset(pRingBuffer->aU8Buffer, 0, RING_BUFFER_MAX_SIZE);
}

/* �����ζ�����д����, �Ḳ�Ǿ����� */
void RingBuffer_Write(RingBuffer_t *pRingBuffer, const uint8 *cnst_pU8Data, uint16 uLen)
{
    for (uint16 i = 0; i < uLen; i++) {
    	pRingBuffer->aU8Buffer[pRingBuffer->uRear] = cnst_pU8Data[i];
    	pRingBuffer->uRear = (pRingBuffer->uRear + 1) % RING_BUFFER_MAX_SIZE;
        /* ��� head ׷���� rear��˵�����ˣ��Զ�ǰ�ƶ��������� */
        if (pRingBuffer->uHead == pRingBuffer->uRear) {
        	pRingBuffer->uHead = (pRingBuffer->uHead + 1) % RING_BUFFER_MAX_SIZE;
        }
    }
}

/* ��ȡ���ζ�����ָ�����ȵ�����, ע�������ж�Ϊ�յı�׼�Ƕ�β���ڶ���.ʹ��DMA��������ʱҲ���ڶ�βָ��պ������ָ���غ�
 * ��˻�����Ӧ���㹻���������������, ��Ϊһ��ͨ�ſ϶����ܳ������ݸ��ǵ�����ᵼ�����ݲ���������.
 * ���Զ���ָ��պ����βָ���غ����ּ������Ҳ�պÿ��Ա��� */
uint16 RingBuffer_Read(RingBuffer_t *pRingBuffer, uint8 *pU8Data, uint16 uReadLen)
{
	uint16 uLen = 0;	/* ���г��� */
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

/* ��ȡGPRS�ź�ǿ�� */
uint8 GPRS_GetSigStrong(void)
{
	uint16 uRxLen = 0;
	uint8 *pU8Buf;
	if(GPRS_SendATCmd("AT+CSQ\r", "+CSQ:", "OK", &uRxLen, 2, 2000)) {
		if((pU8Buf = SkipStrInString(g_UartComm[GPRS_UART_PORT].u8TRBuf, g_UartComm[GPRS_UART_PORT].u8TRBuf + uRxLen, (uint8 *)"CSQ:"))) {
			return ReadU32(&pU8Buf, pU8Buf + 3);
		}
	} else {
		return 99;
	}
	return 0;
}

/* �޸�GPRS������ */
BOOL GPRS_SetBaud(uint32 u32Baud)
{
	char aChCmd[30];
	uint8 *pU8Buf = (uint8 *)aChCmd;
	uint32 u32LastBaud = g_UartComm[GPRS_UART_PORT].Handle->Init.BaudRate;
	PrintString(&pU8Buf, pU8Buf + 10, "AT+IPR=");
	PrintU32(&pU8Buf, pU8Buf + 12, u32Baud, 0);
	*pU8Buf++ = '\r';
	*pU8Buf++ = '\0';
	/* ����GPRS�Ĳ����� Ӧ����û��Ӧ��, ͨ������ATָ��鿴�Ƿ����óɹ� */
	GPRS_SendATCmd(aChCmd, NULL, NULL, NULL, 1, 100);

	OpenUartComm(GPRS_UART_PORT, u32Baud, 0, 30);	/* ���´򿪴��� */
	if(GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 2, 100)) {	/* �Ƿ���Ӧ */
		return TRUE;
	}
	OpenUartComm(GPRS_UART_PORT, u32LastBaud, 0, 30);	/* ����ʧ��, �ָ����ڲ����� */
	return FALSE;
}

/* �鿴SIM���Ƿ������� */
BOOL GPRS_GetInternetState(void)
{
	return GPRS_SendATCmd("AT+CREG?\r", "+CREG: 0,1", "\r\n\r\nOK", NULL, 2, 2000);
}

/* �������總�� */
BOOL GPRS_OpenInternet(void)
{
	return (GPRS_SendATCmd("AT+CGATT=1\r", "OK", NULL, NULL, 1, 3000) &&
	GPRS_SendATCmd("AT+CGATT?\r", "+CGATT: 1", "\r\n\r\nOK", NULL, 2, 2000));
}

/* ��ȡSIM��״̬ ֻ�����߻�δ���� */
BOOL GPRS_GetSIMCardState(void)
{
	return GPRS_SendATCmd("AT+CPIN?\r", "READY", "\r\n\r\nOK", NULL, 2, 2000);
}

/* ��ȡTCPͨ��״̬ */
BOOL GPRS_GetTcpState(void)
{
	return GPRS_SendATCmd("AT+QISTATE\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT);
}

/* �����λGPRS */
BOOL GPRS_Reset(void)
{
	if(GPRS_SendATCmd("AT+CFUN=0,1\r", "OK", "RDY", NULL, 10, GPRS_UART_TIMEOUT * 2)) {
		return TRUE;
	}
	return FALSE;
}

#if USE_GPRS_02G_14G

/* ��ȡδ���ͳ�ȥ���ֽ��� */
uint16 GPRS_GetUnsentBLen(SOCKET SocketId)
{
	uint8 *pStart = g_UartComm[GPRS_UART_PORT].u8TRBuf;
	uint16 uAcked = 0;			/* δ���ͳ�ȥ�����ݳ��� */
	uint16 uRxLen = 0;			/* ���յ���Ӧ���ݳ��� */
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
		if(GPRS_SendATCmd(aChCmd, "+QISEND:", "OK", &uRxLen, 2, 2000)
			&& (pBuff = SkipCharInString(pStart, pStart + uRxLen, ',', 2))) {	/* ��Ӧ�ɹ� */
			uAcked = ReadU32(&pBuff, pBuff + 5);
		}
		Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
		printf("\n[P]\n");
#endif
	} else {	/* û�õ��� */
	}
	return uAcked;
}

/* �ȴ�ָ��socket�ķ��ͻ�������������, ֱ�����͵�ʣ��ָ�����ֽ�����ʱ.
 * ���ó�����
 * 		1. �ȴ���һ֡���ݷ������, ���緢��connect������sub������Ҫ�ȴ���Ӧ, ���������Ӧ��ǰ���Ǳ����Ѿ����ͳ�ȥ��.
 * 		   ��˿���ʹ�ô˺����ȴ�������ȫ���ͳ�ȥ, �ٽ�����Ӧ. ע��: ������Ҫ����Ӧ�ı��Ķ�Ӧ�õ��ô˺������ȴ���Ϣ���ͳ�ȥ,
 * 		   ����ֱ�ӵ���recv����ʲô���ղ���, �����е�NAcked��not acked.
 * 		2. ��ϣ�����ͻ������жѵ�̫�������û�з��ͳ�ȥ, ���Ե��ô˺����ȴ�ֱ��������ʣ��ָ�����ֽ���, �ٷ�����һ֡����.
 * ��1: SocketId
 * ��2: ʣ�෢���ֽ���С�ڵ���u32MaxNAckedʱ��Ϊ���������, ���Է�����һ֡������.
 * ��3: ���ס����, �����ζ�ȡ��δ�����ֽ�����һ����Ϊ�ǿ�ס��, ���������粻��.Ϊ�˷�ֹ����, ��Ϊ�Ƿ���ʧ��. Ϊ�˱�֤ͨ������, Ӧ����������һ��.
 * ��4: �ܳ�ʱʱ��. */
BOOL GPRS_WaitSocketIdle(SOCKET SocketId, uint32 u32MaxNAcked, uint8 u8MaxStuckCnt, uint16 uTotalTimeoutMs)
{
	uint32 u32StartTime = HAL_GetTick();
	uint16 uAcked = 0;			/* δ���ͳ�ȥ�����ݳ��� */
	uint16 uPreAcked = 0xFFFF;	/* �ϴμ�¼��δ���ͳ�ȥ�����ݳ��� */
	uint8 u8StuckCount = u8MaxStuckCnt;
	BOOL bRes = FALSE;
	while(HAL_GetTick() - u32StartTime < uTotalTimeoutMs) {
		uAcked = GPRS_GetUnsentBLen(SocketId);
		/* �ж��Ƿ���Է�����һ֡���� */
		if(uAcked <= u32MaxNAcked) {
			bRes = TRUE;
			break;
		} else if((uPreAcked == uAcked) && (!u8StuckCount--)) {	/* ��ס��ʱ�� */
			break;
		} else {
			u8StuckCount = u8MaxStuckCnt;
			uPreAcked = uAcked;
			Task_sleep(200);
		}
	}
	return bRes;
}

/* ��ȡĳ��PDP����id����״̬ */
BOOL GPRS_GetPDPActState(uint8 u8PDPId)
{
	uint16 uRxLen = 0;
	uint8 *pStart = g_UartComm[GPRS_UART_PORT].u8TRBuf;
	if(GPRS_SendATCmd("AT+QIACT?\r", NULL, "OK", &uRxLen, 4, GPRS_UART_TIMEOUT)) {
		char aChExStr[12] = "+QIACT: 1,";
		aChExStr[8] = u8PDPId + '0';
		if((pStart = SkipStrInString(pStart, pStart + uRxLen, (uint8 *)aChExStr))) {
			return (*pStart - '0') == 1;
		}
	}
	return FALSE;
}

/* ���������� */
BOOL GPRS_SIMCardNetConf(void)
{
	/* �Ȳ鿴��ǰ���Ƿ��Ѽ���PDP���� */
	if(GPRS_GetPDPActState(1)) {
		return TRUE;
	} else {	/* δ����, �ȼ��� */
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
		/* ����PDP���� */
		if(GPRS_SendATCmd(NULL, "OK", NULL, NULL, 1, GPRS_UART_TIMEOUT)
		&& GPRS_SendATCmd("AT+QIACT=1\r", "OK", NULL, NULL, 1, GPRS_UART_TIMEOUT * 4)) {
			return TRUE;
		}
	}
	return FALSE;
}

/* ��ʼ��GPRSģ�� */
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
	GPRS_ClearBuffer();								/* ���GPRS�Ľ��ջ����� */
	OpenUartComm(GPRS_UART_PORT, 115200, 0, 30);	/* ��GPRSͨ�Ŵ��� */

	GPRS_Reset();									/* ��λģ�� */
	GPRS_ClearBuffer();								/* ���GPRS�Ľ��ջ����� */

	if(!GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 1, GPRS_UART_TIMEOUT)) {
		/* ͨ��ʧ��, ���ò�����, ����ʱ������Ӧ������, ����ÿ�θ�λ֮����Ҫ����һ�²�����, ����ͨ�Ų���. */
		if(!GPRS_SetBaud(115200)) {
			return;
		}
	}

	/* ��ʼ������ */
	if(!GPRS_SendATCmd("ATE0\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT * 2)		/* ��ʾ������� 0: �رջ��� 1����������  */
		|| !GPRS_SendATCmd("AT+QISDE=0\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)	/* �ر�SENDָ�����ݻ��ԣ�0������ */
		|| !GPRS_SendATCmd("AT&W\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)		/* �������� */
		|| !GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)			/* ����ͨ���Ƿ������� */
		|| !GPRS_SendATCmd("AT+CFUN=1\r", "OK", "OK", NULL, 1, GPRS_UART_TIMEOUT * 6)) {		/* ����ģ�鹦��Ϊȫģʽ 0:��С����ģʽ, 1:ȫ����ģʽ, 4:����ģʽ �޿��᷵�� +CPIN: NOT INSERTED */
		return;
	}
	Task_sleep(2000);		/* ʵ�ʲ��Կ���ȫ����ģʽ��Ҫ��һ�ᣬ���򿨻�һֱ�ϲ�����. */
	/* ��ȡ��״̬ */
	if(!GPRS_GetSIMCardState()) {
		g_GprsComm.GprsStatus = GPRS_NO_SIM;
		return;
	}

	/* �鿴���������״̬ */
	if(!GPRS_GetInternetState() || !GPRS_OpenInternet() || !GPRS_SIMCardNetConf()) {
		g_GprsComm.GprsStatus = GPRS_NO_INTERNET;
		return;
	}

	g_GprsComm.GprsStatus = GPRS_NORMAL;
}

BOOL CheckGprsStatus(void)
{
//	return FALSE;
	if(!GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)) {	/* ATָ�����Ӧ, ����DMA������, Ҳ����ģ��������, ���³�ʼ��һ�� */
		g_GprsComm.GprsStatus = GPRS_NOT_INIT;
	}
	switch(g_GprsComm.GprsStatus) {
		case GPRS_NORMAL:	/* ����״̬, ��Ҫ��ѯ����״̬����״̬���ź�ǿ��. */
			if(!GPRS_GetSIMCardState()) {				/* ��鿨״̬ */
				g_GprsComm.GprsStatus = GPRS_NO_SIM;
			} else if(!GPRS_GetInternetState()			/* �������״̬ */
					&& (GPRS_GetSigStrong() != 99)) {	/* ����ź�ǿ�� */
				g_GprsComm.GprsStatus = GPRS_NO_INTERNET;
			}
			break;
		case GPRS_NOT_INIT:		/* ��û��ʼ�� */
			GPRS_Init();
			break;
		case GPRS_NO_SIM:		/* δ��SIM��, ֱ�����³�ʼ��ģ�� */
			g_GprsComm.GprsStatus = GPRS_NOT_INIT;
			break;
		case GPRS_NO_INTERNET:	/* �޷�����, �����������źŲ��δע�� */
			if((GPRS_GetSigStrong() == 99)) {	/* �鿴�ź�ǿ�� */
				g_GprsComm.GprsStatus = GPRS_NOT_INIT;
			} else if(GPRS_GetInternetState() && GPRS_OpenInternet() && GPRS_SIMCardNetConf()) {	/* �鿴����״̬�������總��״̬������SIM��PDP���� */
				g_GprsComm.GprsStatus = GPRS_NORMAL;
			}
			break;
		default:
			g_GprsComm.GprsStatus = GPRS_NOT_INIT;		/* ������״̬��ʱ������Ϊδ��ʼ�� */
			break;
	}
	return g_GprsComm.GprsStatus == GPRS_NORMAL;
}

/* ����socket������socket��Ϊȫ�ֱ���������������ͳһ+1��Ϊ�ܿ�NULL������ԭӦ�ò������� */
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
					pSocket->uRecvTimeoutMs = 5000;		/* Ĭ�Ͻ��ճ�ʱʱ����5s */
					pSocket->uSendTimeoutMs = 1000;		/* Ĭ�Ϸ��ͳ�ʱʱ��1s */
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
	} else if(pSocket->bIsConnect) {	/* ��ǰsocket�Ѿ������Ϸ������� ���ش��󼴿� */
		i32Result = GPRS_ERR_CONNECTION_ABN;
	} else {
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
#if DEBUG_PRINT_SIG
			printf("\n[p]\n");
#endif
			if((g_GprsComm.GprsStatus == GPRS_NORMAL) && (!pSocket->bIsFree)) {
				/* ͨ�������·� */
				UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
				uint8* pBuf = pUartComm->u8TRBuf;
				char aChExStr[15] = "+QIOPEN: xx,x";		/* �����Ļظ�. ��1��socketId, ��2��errcode, Ϊ0������. */
				PrintString(&pBuf, pBuf + 20, "AT+QIOPEN=");
				*pBuf++ = 1 + '0';			/* ����ID */
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
				if(pSocket->protocl == IPPROTO_UDP) {	/* Socket ���������� */
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
				/* ������ʵҲ֧��ֱ�Ӵ�ӡ���� */
				PrintIPv4(&pBuf, pBuf+16, __rev(pName->sin_addr.s_addr));	/* ������IP */
				*pBuf++ = '"';
				*pBuf++ = ',';
				PrintU32(&pBuf, pBuf + 12, __rev16(pName->sin_port), 0);	/* �������˿� */
				*pBuf++ = ',';
				*pBuf++ = '0';		/* ���ض˿� 0���Զ����� */
				*pBuf++ = ',';
				*pBuf++ = '0';		/* ���ݴ���ģʽ 0������ģʽ, 1��ֱ��ģʽ, 2��͸��ģʽ */
				*pBuf++ = '\r';
				WriteToUart(GPRS_UART_PORT, pBuf - pUartComm->u8TRBuf);
				/* �Ӵ��ڶ�ȡ��������,300ms���Ȼ��յ�OK��Ȼ��10���ڻ��ܵ�CONNECT OK��CONNECT FAIL */
				if(GPRS_SendATCmd(NULL, NULL, aChExStr, NULL, 2, 5000)) {	/* �鿴��Ӧ */
					i32Result = GPRS_SUC;
					pSocket->bIsConnect = TRUE;
				} else {	/* ����ʧ��. */
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

/* ����ʽ�������� */
int32 GprsSend(SOCKET socketFd, uint8 *pSrcBuf, uint16 uSendBlen, uint32 u32Flag)
{
#define MAX_ONCE_SEND_BLEN	1000
	int32 i32Result = 0;
	uint32 u32AllSentLen = 0;			/* �����Ѿ����͵��ֽ��� */
	uint32 u32AllSendLen = uSendBlen;	/* һ��Ҫ���͵��ֽ��� */
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
				uint8 aU8Cmd[30] = "AT+QISEND=xx,xxx\r";	/* �������ݵ�ATָ�����1��socketId������2�����ͳ��� */
				uint8 *pU8SendLenIndex = &aU8Cmd[10];
				if(pSocket->u8MuxId < 10) {
					*pU8SendLenIndex++ = pSocket->u8MuxId + '0';
				} else {
					*pU8SendLenIndex++ = pSocket->u8MuxId / 10 + '0';
					*pU8SendLenIndex++ = pSocket->u8MuxId % 10 + '0';
				}
				*pU8SendLenIndex++ = ',';
				/* ģ�����֧��MAX_ONCE_SEND_BLEN�ֽڷִη��� */
				for(; u32AllSentLen < u32AllSendLen;) {
					/* ����MAX_ONCE_SEND_BLEN�ķִη��� */
					uint16 uEverySendBlen;
					if(uSendBlen > MAX_ONCE_SEND_BLEN) {
						uEverySendBlen = MAX_ONCE_SEND_BLEN;
					} else {
						uEverySendBlen = uSendBlen;
					}
					uint16 uReadNum = 0;
					/* ����ӡ */
					uint8* pBuf = pU8SendLenIndex;
					PrintU32(&pBuf, pBuf + 12, uEverySendBlen, 0);
					*pBuf++ = '\r';
					*pBuf++ = '\0';
					uint16 uSentLen = 0;
					/* �����ʱ����һ�β����ظ�'>'��, �����ʲôԭ�����������෢�ͼ��� */
					if(GPRS_SendATCmd((char *)aU8Cmd, ">", NULL, &uReadNum, 1, 200)) {	/* ��Ӧ�ɹ� */
						/* ��ʼ�������� */
						while(uSentLen < uEverySendBlen) {
							uint16 uSendLen = (uEverySendBlen - uSentLen) < UART_TR_BUF_BLEN ? (uEverySendBlen - uSentLen) : UART_TR_BUF_BLEN;
							memcpy(pUartComm->u8TRBuf, pSrcBuf + uSentLen + u32AllSentLen, uSendLen);
							WriteToUart(GPRS_UART_PORT, uSendLen);
							uSentLen += uSendLen;
						}
						if(GPRS_SendATCmd(NULL, "SEND OK", NULL, NULL, 1, 2000)) {	/* ���ͳɹ� */
							u32AllSentLen += uEverySendBlen;
							uSendBlen -= uEverySendBlen;
						} else {	/* ����ʧ�� ��Ϊ�ǶϿ����� */
							i32Result = GPRS_ERR_CONNECTION_ABN;
							break;
						}
					} else if(uReadNum != 0) {	/* ��Ӧ��������Ϣ */
						if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"SEND FAIL")) {
							/* ���ͻ��������� */
							i32Result = GPRS_ERR_UART_ABN;
							break;
						} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8 *)"ERROR")) {
							/* TCP���ӶϿ� */
							i32Result = GPRS_ERR_CONNECTION_ABN;
							break;
						} else {	/* ���ܽ�������Ӧ */
							i32Result = GPRS_ERR_PARSE;
						}
					} else {	/* δ��Ӧ */
						i32Result = GPRS_ERR_NO_RESP;
						break;
					}
				}
			} else {	/* GPRSû������socketδ���� */
				i32Result = GPRS_ERR_CONNECTION_ABN;
			}
			Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
			printf("\n[P]\n");
#endif
		} else {	/* û�õ��� */
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}
	pSocket->u8Res = i32Result;
	return u32AllSentLen == 0 ? i32Result : u32AllSentLen;
}


#define MAX_QIRD_HEADER_LEN 100				/* ��ָ������֡ͷ֡β���� */
#define	MAX_READ_LEN		(UART_TR_BUF_BLEN - MAX_QIRD_HEADER_LEN)

/* ע����յĳ��� + ֡ͷ֡β + �������ݳ���(��������󳤶ȼ���) ����С�ڵ���DMA�Ľ��ջ�������С.
 * ��ΪҪȷ�����յ���������������һ��������֡�ڻ�������, ����ȥ�� */
int32 GprsRecv(SOCKET socketFd, uint8 *pU8DstBuf, int32 i32ReadBLen, int32 i32Flags)
{
	int32 i32Result = 0;
	uint32 u32ReadAllBLen = 0;	/* ��ȡ�����ֽ��� */
	GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
	if((socketFd <= 0) || (!i32ReadBLen) || (!pU8DstBuf)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		/* ���û���������������ź������ԣ��������2�� */
		uint32 u32StartTimeMs = HAL_GetTick();
		if(pSocket->bIsConnect) {		/* ���������ٶ� */
			/* ��ʼ������ */
			UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
			char aChCmd[30] = "AT+QIRD=x,xxx\r";		/* ����1��socketId������2����ȡ���� */
			uint8* pU8ReadLenIndex = (uint8 *)&aChCmd[10];
			if(pSocket->u8MuxId < 10) {
				aChCmd[8] = pSocket->u8MuxId + '0';
			} else {
				aChCmd[8] = pSocket->u8MuxId / 10 + '0';
				aChCmd[9] = pSocket->u8MuxId % 10 + '0';
				aChCmd[10] = ',';
				pU8ReadLenIndex++;
			}
			for(; (i32ReadBLen > 0) && (HAL_GetTick() - u32StartTimeMs < pSocket->uRecvTimeoutMs); ) {
				uint16 uEveryReadBLen = i32ReadBLen > MAX_READ_LEN ? MAX_READ_LEN : i32ReadBLen;	/* ���ζ�ȡ���� */
				uint8* pBuf = pU8ReadLenIndex;
				uint16 uActualReadBLen = 0;				/* ʵ�ʶ������ֽ��� */
				uint16 uRepLen = 0;						/* ��Ӧ�ĳ��� */
				PrintU32(&pBuf, pBuf+ 12, uEveryReadBLen, 0);
				*pBuf++ = '\r';
				*pBuf++ = '\0';
				pBuf = pUartComm->u8TRBuf;
				if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {	/* ������������ */
#if DEBUG_PRINT_SIG
					printf("\n[p]\n");
#endif
					/* ������Ӧ, ����OK��Ϊ��ȡһ֡���. */
					if(GPRS_SendATCmd(aChCmd, "+QIRD:", "OK", &uRepLen, 3, 10)) {		/* ���ڶ�һ�η�������Ӧ�����. */
						if((pBuf = SkipCharInString(pBuf, pBuf + uRepLen, ':', 1))) {	/* ��λ�����ݳ��ȵ�λ�� */
							uActualReadBLen = ReadU32(&pBuf, pBuf + 5);	/* ʵ�ʶ������ֽ��� */
							if(uActualReadBLen == 0) {	/* û���� */
								i32Result = GPRS_SUC;
							} else if((uActualReadBLen <= uEveryReadBLen)	/* ʵ�ʶ������ֽ���ҪС�ڵ���������ȡ���ֽ��� */
								&& (uActualReadBLen < uRepLen)			/* ʵ�ʶ������ֽ���ҪС����Ӧ���ܳ��� */
								&& (pBuf = SkipStrInString(pBuf, pUartComm->u8TRBuf + uRepLen, (uint8*)"\r\n")))	/* ��������Ŀ�ʼλ�� */
							{
								i32ReadBLen -= uActualReadBLen;			/* ����ʣ���ֽ��� */
								u32ReadAllBLen += uActualReadBLen;		/* ���¶��������ֽ��� */
								if(u32ReadAllBLen > MQTT_TR_BUF_BLEN) {
									NOP;
								}
								for(; uActualReadBLen > 0; uActualReadBLen--) {	/* ����Ӧ���ݿ������û������� */
									*pU8DstBuf++ = *pBuf++;
								}
								i32Result = GPRS_SUC;
							} else {	/* �޷�ʶ������֡ */
								i32Result = GPRS_ERR_PARSE;
							}
						} else {	/* �޷�ʶ������֡ */
							i32Result = GPRS_ERR_PARSE;
						}
					} else if(uRepLen != 0) {		/* �յ����������͵���Ӧ */
						if(SkipStrInString(pBuf, pBuf + uRepLen, (uint8 *)"ERROR")) {	/* ���Ӳ����� */
							i32Result = GPRS_ERR_CONNECTION_ABN;	/* �Ͽ����� */
						} else {		/* �޷�ʶ������֡ */
							i32Result = GPRS_ERR_PARSE;
						}
					} else {	/* �豸����Ӧ ���ܿ�������DMA������ ��������. */
						i32Result = GPRS_ERR_NO_RESP;
					}
					Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
					printf("\n[P]\n");
#endif
					if(i32Result == GPRS_SUC && uActualReadBLen == 0) {	/* û�������� */
						Task_sleep(10);
					}
					if(i32Result != GPRS_SUC) {
						break;
					}
				} else {	/* û�õ��� */
					i32Result = GPRS_ERR_TIMEOUT;
				}
			}
		} else {	/* �׽��ֲ����� */
			i32Result = GPRS_ERR_OFFLINE;
		}
	}
	pSocket->u8Res = i32Result;
	return (u32ReadAllBLen == 0 ? i32Result : u32ReadAllBLen);
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
				if(pSocket->u8MuxId < 10) {
					*pBuf++ = pSocket->u8MuxId + '0';
				} else {
					*pBuf++ = pSocket->u8MuxId / 10 + '0';
					*pBuf++ = pSocket->u8MuxId % 10 + '0';
				}
				*pBuf++ = '\r';
				WriteToUart(GPRS_UART_PORT, pBuf - pUartComm->u8TRBuf);
				pBuf = pUartComm->u8TRBuf;
				if((uReadNum = ReadFromUart(GPRS_UART_PORT, 5000)) > 0) {
					if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"OK")
							|| SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"ERROR"))
					{
						i32Result = GPRS_SUC;
					} else {
						i32Result = GPRS_ERR_PARSE;		/* ����ʶ�������֡ */
					}
				} else {		/* �豸����Ӧ */
					i32Result = GPRS_ERR_NO_RESP;
				}
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

/* Ŀǰ��֧�������շ���ʱʱ��, ���ڷ�����˵��������Ч��, send����ֻ�ǰ�����д��M25�ķ��ͻ�������, ������ȴ����ݷ��ͳ�ȥ�ٷ�������̫�ķ�ʱ����.
 * ��˶��ڷ�����˵�����ʱʱ��û���κ�����.ֻ�н���ʱ�������������ϵĳ�ʱ */
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

/* ���ز鵽��IP��С�ˣ���0����ʧ�� */
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
				if((uReadNum > 6)		/* 0x0D 0x0A 0x0D 0x0A O K  ip��ַ���16���ַ� */
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

/* ��Ч�������ŷָ���4������, ֻ�������ض�����, ����ͨ��(��û������ж�, ���Ǻ����������ܿ���, Ӧ�ò��������) */
BOOL ParseQuadU16(const char* str, uint16* s1, uint16* s2, uint16* s3, uint16* s4) {
    *s1 = 0; *s2 = 0; *s3 = 0; *s4 = 0;

    // ������һ������
    while(*str && *str >= '0' && *str <= '9') {
        *s1 = *s1 * 10 + (*str - '0');
        str++;
    }
    if(*str == ',') {
    	str++;
    } else {	/* ֡���� */
    	return FALSE;
    }

    // �����ڶ�������
    while(*str && *str >= '0' && *str <= '9') {
        *s2 = *s2 * 10 + (*str - '0');
        str++;
    }
    if(*str == ',') {
		str++;
	} else {	/* ֡���� */
		return FALSE;
	}

    // ��������������
    while(*str && *str >= '0' && *str <= '9') {
        *s3 = *s3 * 10 + (*str - '0');
        str++;
    }
    if(*str == ',') {
		str++;
	} else {	/* ֡���� */
		return FALSE;
	}

    // �������ĸ�����
    while(*str && *str >= '0' && *str <= '9') {
        *s4 = *s4 * 10 + (*str - '0');
        str++;
    }
    if(*str) {	/* ������˵�������ǲ��ᵽ�ַ����Ľ�β�� */
    	return TRUE;
    }
    return FALSE;
}

/* ȥ����û�õ���Ϣ: (+QIURC: "<֪ͨ����>",...) �����Ϣ��GPRS������ģʽʱ�����ϱ���, �޷��ر�.
 * �����Ϣ���п��ܰ�������Ϣ֡�в����ǵ������͵�, ���Ի����ܺ��Ե�, ���������ȥ��.
 * ע���������ר�ŷ�����ReadFromUartͨ���е�GPRS, ������д����, ����ͨ��
 * ֪ͨ���ͷ�Ϊ��
 * closed�����ӶϿ�֪ͨ����TCP Socket�������ӱ�Զ�̿ͻ��˶Ͽ�������Ϊ�����쳣���¶Ͽ���ģ�齫�ϱ���URC��ͬʱ��Socket ������"Closing"״̬��<socket_state>=4��
 * recv�����ݽ���֪ͨ������ģʽ�£�URC��ʽΪ+QIURC: "recv",<connectID>��URC�ϱ���Host��ͨ��AT+QIRD��ȡ���ݡ�
  		��ע�⣬������治Ϊ����ģ���ٴν������ݵ�����£�ֻ�е�Hostͨ��AT+QIRD��ȡ���н��յ����ݺ�ģ��Ż��ϱ��µ�URC��
 * 	incoming full���ͻ�����������֪ͨ������ͻ��������Ѵ��޶�����Ѿ�û��Socketϵͳ��Դ�ɹ����䣬���µĿͻ�����������ʱģ���
		�ϱ�URC��+QIURC: "incoming full"
 * 	incoming���ͻ�������֪ͨ�����<service_type>Ϊ"TCP LISTENER"����һ��Զ�̿ͻ������ӵ����������ʱ��Host���������
		�Զ�����һ�����е�<connectID>������<connectID>��Χ�� 0~11����ʱģ����ϱ��� URC�������ӵ�
		<service_type>��"TCP INCOMING"��<access_mode>�ǻ���ģʽ��
 * 	pdpdeact��PDPȥ����֪ͨ��PDP���Ա�����ȥ���PDP��ȥ�����Ժ�ģ����ϱ���URC֪ͨHost��Host��ִ��AT+QIDEACT
		ȥ������������������ӡ�
 *  */
uint16 StrRemoveHashData(uint8 *pSrc, uint16 uDataLen)
{
	uint8 *pStart = pSrc;
	uint8 *pSrcEnd = pSrc + uDataLen;
	uint8 *pU8URCEnd;		/* URC֡����λ�� */
//	if(uDataLen < 15) {	/* ���ݹ��̲����ܴ���+QIRDI:����֡ */
//		return uDataLen;
//	}
	while((pStart = SkipStrInString(pStart, pSrcEnd, (uint8 *)"+QIz : "))
		&& (pU8URCEnd = SkipStrInString(pStart, pSrcEnd, (uint8 *)"\r\n"))) {	/* �������ϱ�������֡ */
		pStart += 1;			/* ����һλ�������ϱ����ݵ��������� */
		uint8 *pU8StrIndex;
		if((pU8StrIndex = SkipStrInString(pStart, pU8URCEnd, (uint8 *)"closed"))) {	/* ��socket�Ͽ��� */
			pU8StrIndex += 2;			/* �ƶ����������� */
			uint8 u8SocketId = (uint8)ReadU32(&pU8StrIndex, pU8StrIndex + 2);
			g_GprsComm.Socket[u8SocketId].bIsConnect = FALSE;			/* �Ͽ����� */
		} else if((pU8StrIndex = SkipStrInString(pStart, pU8URCEnd, (uint8 *)"recv"))) {	/* ��socket�յ������� */
			pU8StrIndex += 2;			/* �ƶ����������� */
			uint8 u8SocketId = (uint8)ReadU32(&pU8StrIndex, pU8StrIndex + 2);
		} else if(SkipStrInString(pStart, pU8URCEnd, (uint8 *)"dnsgip")){	/* DNS���ͣ�����Ҫ���� */
			pStart = pU8URCEnd;
		} else {	/* �������ͣ����ξ��У���ʱ����Ҫ������� */

		}

		/* ��URC����֡ȥ�� */
		pStart -= 11;	/* �ƶ���\r\n��() */
		pStart = pStart < pSrc ? pSrc : pStart;    /* ��ֹ������� */
		/* ���ǵ��ĳ��� */
		uint16 uCoverLen = pU8URCEnd - pStart;
		/* ��ʼ���� */
		memcpy(pStart, pU8URCEnd, pSrcEnd - pU8URCEnd);
		/* �������� */
		if(uDataLen < uCoverLen) {	/* �����, ʵ��Ӧ�ò������������, ����Ϊ�˰�ȫ�����ж�һ��. */
			return 0;
		}
		uDataLen -= uCoverLen;
		pSrcEnd -= uCoverLen;
	}
	return uDataLen;
}

#else
/* TCPͨ��������� */
BOOL GPRS_TcpConf(void)
{
	BOOL bRet = TRUE;
	if(!GPRS_SendATCmd("AT+QIMUX?\r", "+QIMUX: 1", "\r\n\r\nOK", NULL, 2, GPRS_UART_TIMEOUT)) {	/* ������Ƕ����Ӿ����óɶ����� */
		bRet = GPRS_SendATCmd("AT+QIMUX=1\r", "OK", NULL, NULL, MAX_GPRS_RETRY_CNT, GPRS_UART_TIMEOUT);
	}
	if(!GPRS_SendATCmd("AT+QIDNSIP?\r", "+QIDNSIP: 1", "\r\n\r\nOK", NULL, 1, GPRS_UART_TIMEOUT)) {	/* ������ǰ��������� */
		bRet &= GPRS_SendATCmd("AT+QIDNSIP=1\r", "OK", NULL, NULL, MAX_GPRS_RETRY_CNT, GPRS_UART_TIMEOUT);
	}
	if(!GPRS_SendATCmd("AT+QINDI?\r", "+QINDI: 2", "\r\n\r\nOK", NULL, 1, GPRS_UART_TIMEOUT)) {	/* ������ǻ���ģʽ2 */
		bRet &= GPRS_SendATCmd("AT+QINDI=2\r", "OK", NULL, NULL, MAX_GPRS_RETRY_CNT, GPRS_UART_TIMEOUT);
	}
	return bRet;
}

/* ��ȡδ���ͳ�ȥ���ֽ��� */
uint16 GPRS_GetUnsentBLen(SOCKET SocketId)
{
	uint8 *pStart = g_UartComm[GPRS_UART_PORT].u8TRBuf;
	uint16 uAcked = 0;			/* δ���ͳ�ȥ�����ݳ��� */
	uint16 uRxLen = 0;			/* ���յ���Ӧ���ݳ��� */
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
			&& (pBuff = SkipCharInString(pStart, pStart + uRxLen, ',', 2))) {	/* ��Ӧ�ɹ� */
			uAcked = ReadU32(&pBuff, pBuff + 5);
		}
		Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
		printf("\n[P]\n");
#endif
	} else {	/* û�õ��� */
	}
	return uAcked;
}

/* �ȴ�ָ��socket�ķ��ͻ�������������, ֱ�����͵�ʣ��ָ�����ֽ�����ʱ.
 * ���ó�����
 * 		1. �ȴ���һ֡���ݷ������, ���緢��connect������sub������Ҫ�ȴ���Ӧ, ���������Ӧ��ǰ���Ǳ����Ѿ����ͳ�ȥ��.
 * 		   ��˿���ʹ�ô˺����ȴ�������ȫ���ͳ�ȥ, �ٽ�����Ӧ. ע��: ������Ҫ����Ӧ�ı��Ķ�Ӧ�õ��ô˺������ȴ���Ϣ���ͳ�ȥ,
 * 		   ����ֱ�ӵ���recv����ʲô���ղ���, �����е�NAcked��not acked.
 * 		2. ��ϣ�����ͻ������жѵ�̫�������û�з��ͳ�ȥ, ���Ե��ô˺����ȴ�ֱ��������ʣ��ָ�����ֽ���, �ٷ�����һ֡����.
 * ��1: SocketId
 * ��2: ʣ�෢���ֽ���С�ڵ���u32MaxNAckedʱ��Ϊ���������, ���Է�����һ֡������.
 * ��3: ���ס����, �����ζ�ȡ��δ�����ֽ�����һ����Ϊ�ǿ�ס��, ���������粻��.Ϊ�˷�ֹ����, ��Ϊ�Ƿ���ʧ��. Ϊ�˱�֤ͨ������, Ӧ����������һ��.
 * ��4: �ܳ�ʱʱ��. */
BOOL GPRS_WaitSocketIdle(SOCKET SocketId, uint32 u32MaxNAcked, uint8 u8MaxStuckCnt, uint16 uTotalTimeoutMs)
{
	uint32 u32StartTime = HAL_GetTick();
	uint16 uAcked = 0;			/* δ���ͳ�ȥ�����ݳ��� */
	uint16 uPreAcked = 0xFFFF;	/* �ϴμ�¼��δ���ͳ�ȥ�����ݳ��� */
	uint8 u8StuckCount = u8MaxStuckCnt;
	BOOL bRes = FALSE;
	while(HAL_GetTick() - u32StartTime < uTotalTimeoutMs) {
		uAcked = GPRS_GetUnsentBLen(SocketId);
		/* �ж��Ƿ���Է�����һ֡���� */
		if(uAcked <= u32MaxNAcked) {
			bRes = TRUE;
			break;
		} else if((uPreAcked == uAcked) && (!u8StuckCount--)) {	/* ��ס��ʱ�� */
			break;
		} else {
			u8StuckCount = u8MaxStuckCnt;
			uPreAcked = uAcked;
			Task_sleep(200);
		}
	}
	return bRes;
}

/* ��ʼ��GPRSģ�� */
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
	GPRS_ClearBuffer();								/* ���GPRS�Ľ��ջ����� */
	OpenUartComm(GPRS_UART_PORT, 115200, 0, 30);	/* ��GPRSͨ�Ŵ��� */

	GPRS_Reset();	/* ��λģ�� */
	GPRS_ClearBuffer();								/* ���GPRS�Ľ��ջ����� */

	if(!GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 1, GPRS_UART_TIMEOUT)) {
		/* ͨ��ʧ��, ���ò�����, ����ʱ������Ӧ������, ����ÿ�θ�λ֮����Ҫ����һ�²�����, ����ͨ�Ų���. */
		if(!GPRS_SetBaud(115200)) {
			return;
		}
	}

	/* ��ʼ������ */
	if(!GPRS_SendATCmd("ATE0\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT * 2)		/* ��ʾ������� 0: �رջ��� 1����������  */
		|| !GPRS_SendATCmd("AT+QISDE=0\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)	/* �ر�SENDָ�����ݻ��ԣ�0������ */
		|| !GPRS_SendATCmd("AT&W\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)		/* �������� */
		|| !GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)			/* ����ͨ���Ƿ������� */
		|| !GPRS_SendATCmd("AT+CFUN=1\r", "OK", "SMS Ready", NULL, 5, 5000)) {		/* ����ģ�鹦��Ϊȫģʽ 0:��С����ģʽ, 1:ȫ����ģʽ, 4:����ģʽ �޿��᷵�� +CPIN: NOT INSERTED */
		return;
	}

	/* ��ȡ��״̬ */
	if(!GPRS_GetSIMCardState()) {
		g_GprsComm.GprsStatus = GPRS_NO_SIM;
		return;
	}

	/* �鿴�������״̬ */
	if(!GPRS_GetInternetState() || !GPRS_OpenInternet() || !GPRS_TcpConf()) {
		g_GprsComm.GprsStatus = GPRS_NO_INTERNET;
		return;
	}

	g_GprsComm.GprsStatus = GPRS_NORMAL;
}

BOOL CheckGprsStatus(void)
{
//	return FALSE;
	if(!GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)) {	/* ATָ�����Ӧ, ����DMA������, Ҳ����ģ��������, ���³�ʼ��һ�� */
		g_GprsComm.GprsStatus = GPRS_NOT_INIT;
	}
	switch(g_GprsComm.GprsStatus) {
		case GPRS_NORMAL:	/* ����״̬, ��Ҫ��ѯ����״̬����״̬���ź�ǿ��. */
			if(!GPRS_GetSIMCardState()) {				/* ��鿨״̬ */
				g_GprsComm.GprsStatus = GPRS_NO_SIM;
			} else if(!GPRS_GetInternetState()			/* �������״̬ */
					&& (GPRS_GetSigStrong() != 99)) {	/* ����ź�ǿ�� */
				g_GprsComm.GprsStatus = GPRS_NO_INTERNET;
			}
			break;
		case GPRS_NOT_INIT:		/* ��û��ʼ�� */
			GPRS_Init();
			break;
		case GPRS_NO_SIM:		/* δ��SIM��, ֱ�����³�ʼ��ģ�� */
			g_GprsComm.GprsStatus = GPRS_NOT_INIT;
			break;
		case GPRS_NO_INTERNET:	/* �޷�����, �����������źŲ��δע�� */
			if((GPRS_GetSigStrong() == 99)) {	/* �鿴�ź�ǿ�� */
				g_GprsComm.GprsStatus = GPRS_NOT_INIT;
			} else if(GPRS_GetInternetState() && GPRS_OpenInternet() && GPRS_TcpConf()) {	/* �鿴����״̬�������總��״̬������TCPͨ�� */
				g_GprsComm.GprsStatus = GPRS_NORMAL;
			}
			break;
		default:
			g_GprsComm.GprsStatus = GPRS_NOT_INIT;		/* ������״̬��ʱ������Ϊδ��ʼ�� */
			break;
	}
	return g_GprsComm.GprsStatus == GPRS_NORMAL;
}

/* ����socket������socket��Ϊȫ�ֱ���������������ͳһ+1��Ϊ�ܿ�NULL������ԭӦ�ò������� */
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
					pSocket->uRecvTimeoutMs = 5000;		/* Ĭ�Ͻ��ճ�ʱʱ����5s */
					pSocket->uSendTimeoutMs = 1000;		/* Ĭ�Ϸ��ͳ�ʱʱ��1s */
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
	} else if(pSocket->bIsConnect) {	/* ��ǰsocket�Ѿ������Ϸ������� ���ش��󼴿� */
		i32Result = GPRS_ERR_CONNECTION_ABN;
	} else {
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
#if DEBUG_PRINT_SIG
			printf("\n[p]\n");
#endif
			if((g_GprsComm.GprsStatus == GPRS_NORMAL) && (!pSocket->bIsFree)) {
				/* ͨ�������·� */
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
				/* ������ʵҲ֧��ֱ�Ӵ�ӡ���� */
				PrintIPv4(&pBuf, pBuf+16, __rev(pName->sin_addr.s_addr));
				*pBuf++ = '"';
				*pBuf++ = ',';
				*pBuf++ = '"';
				PrintU32(&pBuf, pBuf + 12, __rev16(pName->sin_port), 0);
				*pBuf++ = '"';
				*pBuf++ = '\r';
				WriteToUart(GPRS_UART_PORT, pBuf - pUartComm->u8TRBuf);
				/* �Ӵ��ڶ�ȡ��������,300ms���Ȼ��յ�OK��Ȼ��10���ڻ��ܵ�CONNECT OK��CONNECT FAIL */
				if(GPRS_SendATCmd(NULL, "CONNECT OK", "CONNECT", NULL, 2, 5000)) {	/* �鿴��Ӧ */
					i32Result = GPRS_SUC;
					pSocket->bIsConnect = TRUE;
				} else {	/* ����ʧ��. */
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

/* ����ʽ�������� */
int32 GprsSend(SOCKET socketFd, uint8 *pSrcBuf, uint16 uSendBlen, uint32 u32Flag)
{
#define MAX_ONCE_SEND_BLEN	1000
	int32 i32Result = 0;
	uint32 u32AllSentLen = 0;			/* �����Ѿ����͵��ֽ��� */
	uint32 u32AllSendLen = uSendBlen;	/* һ��Ҫ���͵��ֽ��� */
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
				/* ģ�����֧��MAX_ONCE_SEND_BLEN�ֽڷִη��� */
				for(; u32AllSentLen < u32AllSendLen;) {
					/* ����MAX_ONCE_SEND_BLEN�ķִη��� */
					uint16 uEverySendBlen;
					if(uSendBlen > MAX_ONCE_SEND_BLEN) {
						uEverySendBlen = MAX_ONCE_SEND_BLEN;
						uSendBlen -= MAX_ONCE_SEND_BLEN;
					} else {
						uEverySendBlen = uSendBlen;
					}
					uint16 uReadNum = 0;
					/* ����ӡ */
					uint8 aU8Cmd[30];
					uint8* pBuf = aU8Cmd;
					PrintString(&pBuf, pBuf + 20, "AT+QISEND=");
					*pBuf++ = '0' + pSocket->u8MuxId;
					*pBuf++ = ',';
					PrintU32(&pBuf, pBuf + 12, uEverySendBlen, 0);
					*pBuf++ = '\r';
					*pBuf++ = '\0';
					uint16 uSentLen = 0;
					/* �����ʱ����һ�β����ظ�'>'��, �����ʲôԭ�����������෢�ͼ��� */
					if(GPRS_SendATCmd((char *)aU8Cmd, ">", NULL, &uReadNum, 2, 2000)) {	/* ��Ӧ�ɹ� */
						/* ��ʼ�������� */
						while(uSentLen < uEverySendBlen) {
							uint16 uSendLen = (uEverySendBlen - uSentLen) < UART_TR_BUF_BLEN ? (uEverySendBlen - uSentLen) : UART_TR_BUF_BLEN;
							memcpy(pUartComm->u8TRBuf, pSrcBuf + uSentLen + u32AllSentLen, uSendLen);
							WriteToUart(GPRS_UART_PORT, uSendLen);
							uSentLen += uSendLen;
						}
						if(GPRS_SendATCmd(NULL, "SEND OK", NULL, NULL, 1, 2000)) {	/* ���ͳɹ� */
							u32AllSentLen += uEverySendBlen;
						} else {	/* ����ʧ�� ��Ϊ�ǶϿ����� */
							i32Result = GPRS_ERR_CONNECTION_ABN;
							break;
						}
					} else if(uReadNum != 0) {	/* ��Ӧ��������Ϣ */
						if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"SEND FAIL")) {
							/* ���Ӵ��� ���Ƿ���ʧ�� ��Ϊ���������� */
							i32Result = GPRS_ERR_UART_ABN;
							break;
						} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"SEND FAIL")
								|| SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8 *)"ERROR")) {
							/* δ����TCP���� */
							i32Result = GPRS_ERR_CONNECTION_ABN;
							break;
						} else {	/* ���ܽ�������Ӧ */
							i32Result = GPRS_ERR_PARSE;
						}
					} else {	/* δ��Ӧ */
						i32Result = GPRS_ERR_NO_RESP;
						break;
					}
				}
			} else {	/* GPRSû������socketδ���� */
				i32Result = GPRS_ERR_CONNECTION_ABN;
			}
			Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
			printf("\n[P]\n");
#endif
		} else {	/* û�õ��� */
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}
	pSocket->u8Res = i32Result;
	if(u32AllSentLen == u32AllSendLen) {	/* ������� */
		return u32AllSentLen;
	} else {	/* ����ʧ�� */
		pSocket->bIsConnect = FALSE;
		return i32Result;
	}
}


#define MAX_QIRD_HEADER_LEN 100				/* ��ָ������֡ͷ֡β���� */
#define	MAX_READ_LEN		(UART_TR_BUF_BLEN - MAX_QIRD_HEADER_LEN)

/* ע����յĳ��� + ֡ͷ֡β + �������ݳ���(��������󳤶ȼ���) ����С�ڵ���DMA�Ľ��ջ�������С.
 * ��ΪҪȷ�����յ���������������һ��������֡�ڻ�������, ����ȥ�� */
int32 GprsRecv(SOCKET socketFd, uint8 *pU8DstBuf, int32 i32ReadBLen, int32 i32Flags)
{
	int32 i32Result = 0;
	uint32 u32ResultLen = 0;	/* ��ȡ���ֽ��� */
	GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
	if((socketFd <= 0) || (!i32ReadBLen) || (!pU8DstBuf)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		/* ���û���������������ź������ԣ��������2�� */
		uint32 u32StartTimeMs = HAL_GetTick();
		while((HAL_GetTick() - u32StartTimeMs < pSocket->uRecvTimeoutMs) && i32ReadBLen > 0) {	/* û��ʱ����û�����涨�����ݼ����� */
			if(pSocket->bIsConnect) {		/* ���������ٶ� */
				if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {	/* ������������ */
#if DEBUG_PRINT_SIG
				printf("\n[p]\n");
#endif
					/* �������ٶ�, û������Ҫ������M25ͨ��. ��ΪM25�������ϱ���������ȫ������,
					 * ���ǻ���ͨ�Ź�����"˳��д�"��ǰsocket�Ƿ��������ݽ��յ�����. ������﷢��ATָ���ͨ�� */
					if(!pSocket->u32UnreadBLen) {
						GPRS_SendATCmd("AT\r", "OK", "OK", NULL, 3, 100);
					} else {
						/* ��uart buf��СΪ��Ԫѭ����ѯ��ȡM25������ȫ���������ݣ�M25ÿsocket��󻺴�400K�� */
						UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
						/* ������Ҫ��ȡ���ֽ��� */
						uint16 uReadBLen = i32ReadBLen > pSocket->u32UnreadBLen ? pSocket->u32UnreadBLen : i32ReadBLen;
						char aChCmd[30];
						for(; uReadBLen > 0 ; ) {	/* ѭ������ʣ���ֽ� */
							uint16 uEveryReadBLen = uReadBLen > MAX_READ_LEN ? MAX_READ_LEN : uReadBLen;	/* ���ζ�ȡ���� */
							uint8* pBuf = (uint8 *)aChCmd;
							uint16 uUartReadBLen = 0;
							PrintString(&pBuf, pBuf + 20, "AT+QIRD=0,1,");
							*pBuf++ = '0' + pSocket->u8MuxId;
							*pBuf++ = ',';
							PrintU32(&pBuf, pBuf+ 12, uEveryReadBLen, 0);
							*pBuf++ = '\r';
							*pBuf++ = '\0';
							pBuf = pUartComm->u8TRBuf;
							/* ������Ӧ, ����OK��Ϊ��ȡһ֡���. */
							if(GPRS_SendATCmd(aChCmd, "+QIRD:", "OK", &uUartReadBLen, 3, 1000)) {	/* ���ڶ�һ�η�������Ӧ�����. */
								if((pBuf = SkipCharInString(pBuf, pBuf + uUartReadBLen, ',', 2))) {	/* ��λ�����ݳ��ȵ�λ�� */
									uint16 uRecvFrameLen = ReadU32(&pBuf, pBuf + 5);	/* QIRD�������ݳ��� */
									/* ������Ч */
									if((uEveryReadBLen == uRecvFrameLen)	/* �ܽ���˵��һ������ô������, �������������û��ô�ఴ�մ����� */
										&& (uRecvFrameLen < uUartReadBLen)
										&& (pBuf = SkipStrInString(pBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"\r\n")))
									{
										i32ReadBLen -= uRecvFrameLen;		/* ����ʣ���ֽ��� */
										uReadBLen -= uRecvFrameLen;			/* ���±�����Ҫ��ȡ���ֽ��� */
										u32ResultLen += uRecvFrameLen;		/* ���¶����ֽ��� */
										if(u32ResultLen > MQTT_TR_BUF_BLEN) {
											NOP;
										}
										pSocket->u32UnreadBLen -= uRecvFrameLen;
										for(; uRecvFrameLen > 0; uRecvFrameLen--) {	/* ����Ӧ���ݿ������û������� */
											*pU8DstBuf++ = *pBuf++;
										}
										i32Result = GPRS_SUC;
									} else {	/* �޷�ʶ������֡ */
										i32Result = GPRS_ERR_PARSE;
										break;
									}
								} else {	/* �޷�ʶ������֡ */
									i32Result = GPRS_ERR_PARSE;
									break;
								}
							} else if(uUartReadBLen != 0) {	/* �յ����������͵���Ӧ */
								if((uUartReadBLen == 6
								&& SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"OK"))) {	/* ֻ��OK �������ݶ�ʧ�� ֱ������ */
									i32Result = GPRS_ERR_UART_ABN;
									break;
								} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"CLOSE")
										|| SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"ERROR")
										|| SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"+PDP DEACT")) {	/* �Ƿ�Ͽ����� */
									i32Result = GPRS_ERR_CONNECTION_ABN;	/* �Ͽ����� */
									break;
								} else {	/* ��Ӧ������δʶ�����Ϣ */
									i32Result = GPRS_ERR_PARSE;
									break;
								}
							} else {	/* �豸����Ӧ ���ܿ�������DMA������ ��������. */
								i32Result = GPRS_ERR_NO_RESP;
								break;
							}
						}
					}
					Semaphore_post(g_GprsComm.Sem_GprsReq);
#if DEBUG_PRINT_SIG
					printf("\n[P]\n");
#endif
				} else {	/* û�õ�����û����, ������ */
					i32Result = GPRS_SUC;
				}
			} else {	/* �׽��ֲ����� */
				i32Result = GPRS_ERR_OFFLINE;
				break;
			}
			if(i32Result < 0) {		/* ��һ��ͨ��ʧ�ܾ��˳�����, �����ش��� */
				break;
			}
			Task_sleep(10);		/* û�õ���, ����û������������������һ������� */
		}
	}
	pSocket->u8Res = i32Result;
	if(i32Result == GPRS_SUC) {					/* ͨ�ųɹ����ض������ֽ��� */
		return u32ResultLen;
	} else if(i32Result == GPRS_ERR_TIMEOUT) {	/* ��ʱ������� */
		return i32Result;
	} else {				/* ͨ�Ŵ���, �������connect�Ͽ��� */
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
				} else {		/* �豸����Ӧ */
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

/* Ŀǰ��֧�������շ���ʱʱ��, ���ڷ�����˵��������Ч��, send����ֻ�ǰ�����д��M25�ķ��ͻ�������, ������ȴ����ݷ��ͳ�ȥ�ٷ�������̫�ķ�ʱ����.
 * ��˶��ڷ�����˵�����ʱʱ��û���κ�����.ֻ�н���ʱ�������������ϵĳ�ʱ */
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

/* ���ز鵽��IP��С�ˣ���0����ʧ�� */
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
				if((uReadNum > 6)		/* 0x0D 0x0A 0x0D 0x0A O K  ip��ַ���16���ַ� */
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

/* ��Ч�������ŷָ���4������, ֻ�������ض�����, ����ͨ��(��û������ж�, ���Ǻ����������ܿ���, Ӧ�ò��������) */
BOOL ParseQuadU16(const char* str, uint16* s1, uint16* s2, uint16* s3, uint16* s4) {
    *s1 = 0; *s2 = 0; *s3 = 0; *s4 = 0;

    // ������һ������
    while(*str && *str >= '0' && *str <= '9') {
        *s1 = *s1 * 10 + (*str - '0');
        str++;
    }
    if(*str == ',') {
    	str++;
    } else {	/* ֡���� */
    	return FALSE;
    }

    // �����ڶ�������
    while(*str && *str >= '0' && *str <= '9') {
        *s2 = *s2 * 10 + (*str - '0');
        str++;
    }
    if(*str == ',') {
		str++;
	} else {	/* ֡���� */
		return FALSE;
	}

    // ��������������
    while(*str && *str >= '0' && *str <= '9') {
        *s3 = *s3 * 10 + (*str - '0');
        str++;
    }
    if(*str == ',') {
		str++;
	} else {	/* ֡���� */
		return FALSE;
	}

    // �������ĸ�����
    while(*str && *str >= '0' && *str <= '9') {
        *s4 = *s4 * 10 + (*str - '0');
        str++;
    }
    if(*str) {	/* ������˵�������ǲ��ᵽ�ַ����Ľ�β�� */
    	return TRUE;
    }
    return FALSE;
}

/* ȥ����û�õ���Ϣ: (+QIRDI: ....) �����Ϣ��GPRS������ģʽʱ�����ϱ���, �޷��ر�.
 * �����Ϣ���п��ܰ�������Ϣ֡�в����ǵ������͵�, ���Ի����ܺ��Ե�, ���������ȥ��.
 * ע���������ר�ŷ�����ReadFromUartͨ���е�GPRS, ������д����, ����ͨ��
 * QIRDI��Ϣ��ʽ��+QIRDI: 0,1,3,1,4,4
 * ��1��ǰ�ó���һ�㶼��0;��2����Ϊclietn����server 1��client 2��server;��3����ǰsokcet���;
 * ��4����ǰ��Ϣ�ı��;��5�����ν��յĳ���;��6����ǰsokcetʣ��δ��ȡ����; */
uint16 StrRemoveHashData(uint8 *pSrc, uint16 uDataLen)
{
	uint8 *pStart = pSrc;
	uint8 *pSrcEnd = pSrc + uDataLen;
	uint8 *pEnd;
	if(uDataLen < 15) {	/* ���ݹ��̲����ܴ���+QIRDI:����֡ */
		return uDataLen;
	}
	while((pStart = SkipStrInString(pStart, pSrcEnd, (uint8 *)"+QIRDI: "))) {
		if(*pStart == '0') {
			if(*(pStart + 2) == '1') {	/* Ӧ������Ϊclient���յ����� */
				uint16 uSocketId, uMsgId;		/* ��ǰsocket id����Ϣid */
				uint16 uBlockBLen, uTotalBLen;	/* ���ν��յĳ��ȡ���socketʣ��δ��ȡ�ĳ��� */
				*pSrcEnd = '\0';	/* ��ֹ��� */
				/* ����QIRDI֡ */
				if(ParseQuadU16((char *)(pStart + 4), &uSocketId, &uMsgId, &uBlockBLen, &uTotalBLen)
				&& (uSocketId < MAX_GPRS_SOCKET_NUM)
				&& (uBlockBLen <= uTotalBLen)) {
//				&& (uMsgId == (g_GprsComm.Socket[uSocketId].u32LastQirdiIndex + 1))) {	/* ��ȡ�ɹ� */
					g_GprsComm.Socket[uSocketId].u32LastQirdiIndex = uMsgId;
					g_GprsComm.Socket[uSocketId].u32UnreadBLen = uTotalBLen;
				} else {	/* ������, ���ֶα��ֿ���.ʣ�ಿ�ֿ������������������� */
					NOP;
				}
			}
		}
		/* ��֡�����д�����������׼��ȥ�� */
		if(!(pEnd = SkipStrInString(pStart, pSrcEnd, (uint8 *)"\r\n"))) {	/* ��+QIRDI:����û�ҵ�\r\n, ��û�취ȷ������λ��, ֻ�ܷ��ز���������. */
			return uDataLen;
		}
		pStart -= 10;	/* �ƶ���\r\n��() */
		pStart = pStart < pSrc ? pSrc : pStart;    /* ��ֹ������� */
		/* ���ǵ��ĳ��� */
		uint16 uCoverLen = pEnd - pStart;
		if(pSrcEnd - pEnd > 0) {

		}
		/* ��ʼ���� */
		memcpy(pStart, pEnd, pSrcEnd - pEnd);
		/* �������� */
		if(uDataLen < uCoverLen) {	/* �����, ʵ��Ӧ�ò������������, ����Ϊ�˰�ȫ�����ж�һ��. */
			return 0;
		}
		uDataLen -= uCoverLen;
		pSrcEnd -= uCoverLen;
	}
	return uDataLen;
}
#endif
