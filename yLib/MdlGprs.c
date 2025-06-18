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

/* ����ATָ�� ���ж���Ӧ�Ƿ���ȷ, ��ȷ����Ӧʱ����NULL
 * ��1: Ҫ���͵�ATָ��, ΪNULL��ֱ�Ӷ���Ӧ�Ƿ����, ���ᷢ��Ϣ.
 * 		ע: ָ��Ϊ�ַ���, ��˱�����\0��β, ����Ҫ����ָ��Ľ�����\r
 * ��2: ��������Ӧ, ΪNULLʱ�������ݾͷ���TRUE.
 * ��3: �������, ��ʱһ֡�����Ƿ����η�����Ҫ��������, �����Ҫ��ν���, ֱ�����յ�ָ�����ַ�����Ϊ������һ֡, ��Ҫ����AT+QIRDָ��.
 * 	ע: ���Դ���������ڵ���2, ��Ϊ���һ֡û������, ��Ҫ�ٴν���, ��������ֻ�ᷢ��һ��.3
 * 		ΪNULLʱ�������ݾͻ᷵��.
 * ��4: ���յ���Ӧ�ֽڳ���
 * ��5: ���Դ���
 * ��6: ��ʱʱ��, ע���ǵ��ζ�ȡ�ĳ�ʱʱ��*/
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
//				printf("\r\nTCPͨ��:\r\n");
//				HAL_UART_Transmit(g_UartComm[0].Handle, g_UartComm[GPRS_UART_PORT].u8TRBuf, uRxLen + g_UartComm[GPRS_UART_PORT].uRxFrameIndex, 300);
//				printf("\n");
				bFlag = FALSE;	/* �����ٴη���ָ���� */
				uRxLen += g_UartComm[GPRS_UART_PORT].uRxFrameIndex;
				if(SkipStrInString(pStart, pStart + uRxLen, (uint8 *)cnst_pChEndStr)) {	/* �յ��˽������ */
					if((cnst_pChExpectResp == NULL) || SkipStrInString(pStart, pStart + uRxLen, (uint8 *)cnst_pChExpectResp)) {
						bRet = TRUE;
					} else {	/* �������յ��˽������, ����������Ӧ����, ����false */
						bRet = FALSE;
					}
					break;
				} else {	/* û�յ��������, �������� */
					g_UartComm[GPRS_UART_PORT].uRxFrameIndex = uRxLen;
				}
			}
		}
	}
	if(uRxBufLen != NULL) {
		*uRxBufLen = uRxLen;
	}
	g_UartComm[GPRS_UART_PORT].uRxFrameIndex = 0;
    return bRet;  /* ��ʱ��δ�յ�������Ӧ */
}

/* ��ջ�����, ���ʹ�û��λ��������յ�����, Ƶ������ATָ����ܻᵼ����Ϣ��λ
 * ��˳�ʼ����ʱ��������һ�»����� */
void GPRS_ClearBuffer(void)
{
	RingBuffer_Init(&g_GPRSRingBuffer);
}

/* ��ȡGPRS�ź�ǿ�� */
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

/* �޸�GPRS������ */
BOOL GPRS_SetBaud(uint32 u32Baud)
{
	char aChCmd[30];
	uint8 *pU8Buf = (uint8 *)aChCmd;
	uint32 u32LastBaud = g_UartComm[GPRS_UART_PORT].Handle->Init.BaudRate;
//	if(u32Baud == u32LastBaud) {
//		return TRUE;
//	}
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

/* ��ȡSIM������״̬ */
BOOL GPRS_GetInternetState(void)
{
	return GPRS_SendATCmd("AT+CREG?\r", "+CREG: 0,1", "\r\n\r\nOK", NULL, 2, GPRS_UART_TIMEOUT);
}

/* �������總�� */
BOOL GPRS_OpenInternet(void)
{
	return (GPRS_SendATCmd("AT+CGATT=1\r", "OK", NULL, NULL, MAX_GPRS_RETRY_CNT, GPRS_UART_TIMEOUT) &&
	GPRS_SendATCmd("AT+CGATT?\r", "+CGATT: 1", "\r\n\r\nOK", NULL, 2, GPRS_UART_TIMEOUT));
}

/* ��ȡSIM��״̬ ���Խ׶�, SIMֻ���������������״̬ */
BOOL GPRS_GetSIMCardState(void)
{
	return GPRS_SendATCmd("AT+CPIN?\r", "READY", "\r\n\r\nOK", NULL, 2, 2000);
//	return uRxLen > 0 && SkipStrInString(pStart, pStart + uRxLen, (uint8*)"READY");

//		else if((pU8Buf = SkipStrInString(pStart, pStart + uRxLen, (uint8*)"+CME ERROR:"))) {
//			uint16 uErrorCode = ReadU32(&pU8Buf, pU8Buf + 4);
//			if(uErrorCode == 10) {
//				g_GprsComm.GprsStatus = GPRS_NO_SIM;	/* δ�忨 */
//			} else {
//				g_GprsComm.GprsStatus = GPRS_ERROR;		/* ����CME���� */
//			}
//		} else {	/* �ظ���, �����ǿ�δ׼����, ��״̬��Ȼ����Ϊerror */
//			g_GprsComm.GprsStatus = GPRS_ERROR;
//		}
//	} else {	/* ûͨ���� */
//		g_GprsComm.GprsStatus = GPRS_COMM_FAIL;
//	}
//	return g_GprsComm.GprsStatus;
}

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

/* �����λGPRS */
BOOL GPRS_Reset(void)
{
	return GPRS_SendATCmd("AT+CFUN=0,1\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT);
}

/* ��ȡTCPͨ��״̬ */
BOOL GPRS_GetTcpState(void)
{
	return GPRS_SendATCmd("AT+QISTATE\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT);
}

/* �ȴ���һ֡���ݷ������, ��ΪM25�Ե���socket��˵�շ������Ǵ��е�, ��һ֡����û�������֮ǰ��һ֡������Ҫ�Ŷӵȴ�
 * ��Ҫ�Ǹ�Mqtt����������Ping��ʱ����֮ǰ������̫����Ϣ, ����Ping��һֱ���Ŷ���, ���Ƿ���Ping���ĳ�ʱʱ���ַǳ���,
 * �͵���ż������Ϊ��ʱ�����ش�����Mqtt�Ͽ�, ���ÿ��Ping֮ǰ��Ҫ�ȴ�Socket����.
 * ��1: SocketId
 * ��2: ʣ�෢���ֽ���С�ڵ���u32MaxNAckedʱ��Ϊ���Է�����һ֡������.
 * ��3: ���ס����, δ�����ֽ�����ζ�һ����Ϊ�ǿ�ס��, ���������粻��, ��Ϊ�Ƿ���ʧ��.
 * ��4: �ܳ�ʱʱ��. */
extern BOOL PubSpecialData(MQTT_COMM* pMqttComm);
BOOL GPRS_WaitSocketIdle(SOCKET SocketId, uint32 u32MaxNAcked, uint8 u8MaxStuckCnt, uint16 uTotalTimeoutMs)
{
	uint8 *pStart = g_UartComm[GPRS_UART_PORT].u8TRBuf;
	uint32 u32StartTime = HAL_GetTick();
	char aChCmd[20];
	uint8 *pBuff = (uint8 *)aChCmd;
	uint16 uRxLen = 0;			/* ���յ���Ӧ���ݳ��� */
	uint16 uAcked = 0;			/* δ���ͳ�ȥ�����ݳ��� */
	uint16 uPreAcked = 0xFFFF;	/* �ϴμ�¼��δ���ͳ�ȥ�����ݳ��� */
	uint8 u8StuckCount = u8MaxStuckCnt;
	BOOL bFlag = TRUE;			/* �Ƿ��Ѿ����͹��� */
	PrintString(&pBuff, pBuff + 12, "AT+QISACK=");
	*pBuff++ = g_GprsComm.Socket[SocketId-1].u8MuxId + '0';
	*pBuff++ = '\r';
	*pBuff++ = '\0';
	while(HAL_GetTick() - u32StartTime < uTotalTimeoutMs) {
		if(!GPRS_SendATCmd(aChCmd, "+QISACK: ", "OK", &uRxLen, 4, 2000)) {	/* ��Ӧʧ��, ����ģ���쳣. */
			return FALSE;
		}
		/* ��ȡnAcked */
		if(!(pBuff = SkipCharInString(pStart, pStart + uRxLen, ',', 2))) {	/* ��λ��nAckedλ�� */
			return FALSE;
		}
		uAcked = ReadU32(&pBuff, pBuff + 5);
		/* �ж��Ƿ���Է�����һ֡���� */
		if(uAcked <= u32MaxNAcked) {
			return TRUE;
		}
		/* �ж��Ƿ�ס�� */
		if(uPreAcked == uAcked) {
			/* ����M25��Nagle�㷨, ��������ֽڷ�����ȥ, ����ֱ�ӷ���һЩ���������ѷ��ͻ�����ǿ��ˢ��һ�� */
			if(!u8StuckCount--) {
				if(bFlag && uAcked < 10) {
					if(g_MqttComm[0].MqttSocket == SocketId) {	/* �������׽��� */
						PubSpecialData(&g_MqttComm[0]);
					} else {		/* ���ĵ��׽��� */
						PubSpecialData(&g_MqttComm[1]);
					}
					bFlag = FALSE;
					u32StartTime = HAL_GetTick();	/* ���¼�ʱ */
					u8StuckCount = u8MaxStuckCnt;	/* ���¼��� */
					continue;
				}
				return FALSE;
			}
		} else {
			u8StuckCount = u8MaxStuckCnt;
			uPreAcked = uAcked;
		}
		Task_sleep(200);
	}
	/* ��ʱ�˳� */
	return FALSE;
}

BOOL CheckGprsStatus(void)
{
	if(g_GprsComm.GprsStatus == GPRS_NORMAL) {
		/* ��鿨״̬������״̬ */
		if(!GPRS_GetSIMCardState()) {
			g_GprsComm.GprsStatus = GPRS_NO_SIM;
		} else if(!GPRS_GetInternetState()) {	/* �鿴����״̬ */
			g_GprsComm.GprsStatus = GPRS_NO_INTERNET;
		}
	} else if(g_GprsComm.GprsStatus == GPRS_NOT_INIT) {
		uint8 i = 0;
		uint8 u8RstCnt = 5;
		for(; u8RstCnt > 0; u8RstCnt--) {
			for(i = 0;i < MAX_GPRS_SOCKET_NUM; i++) {
				GPRS_SOCKET *pSocket = &g_GprsComm.Socket[i];
				pSocket->name = NULL;
				pSocket->protocl = 0;
				pSocket->u8MuxId = 0;
				pSocket->u8Res = 0;
				pSocket->bIsFree = TRUE;
				pSocket->u8MuxId = i;
			}
			GPRS_ClearBuffer();		/* ���GPRS�Ľ��ջ����� */
			OpenUartComm(0, 115200, 0, 10);				/* ���Դ��� */
			OpenUartComm(GPRS_UART_PORT, 9600, 0, 30);	/* ��GPRSͨ�Ŵ��� */

			/* ����ͨ���Ƿ�����, �������͸�λģ�� */
			if(!GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)) {
				GPRS_Reset();	/* ��λģ�� */
				return FALSE;
			}
//			Task_sleep(1000);

			/* ���ò����ʣ�����ʱ������Ӧ������ */
			if(!GPRS_SetBaud(9600) && GPRS_Reset()) {
				return FALSE;
			}
//			Task_sleep(1000);

			/* ��ʼ������ */
			if(!GPRS_SendATCmd("ATE0\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT * 2)	/* ��ʾ������� 0: �رջ��� 1����������  */
				|| !GPRS_SendATCmd("AT+QISDE=0\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)	/* �ر�SENDָ�����ݻ��ԣ�0������ */
				|| !GPRS_SendATCmd("AT&W\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)	/* �������� */
				|| !GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 2, GPRS_UART_TIMEOUT)	/* ����ͨ���Ƿ������� */
				|| !GPRS_SendATCmd("AT+CFUN=1\r", "OK", NULL, NULL, 2, 5000)) {	/* ����ģ�鹦��Ϊȫģʽ 0:��С����ģʽ, 1:ȫ����ģʽ, 4:����ģʽ �޿��᷵�� +CPIN: NOT INSERTED */
				return FALSE;
			}
			Task_sleep(1000);

			/* ��ȡ��״̬ */
			if(!GPRS_GetSIMCardState()) {
				g_GprsComm.GprsStatus = GPRS_NO_SIM;
				return FALSE;
			}
//			Task_sleep(1000);

			/* �鿴�������״̬ */
			if(!GPRS_GetInternetState() || !GPRS_OpenInternet()) {
				g_GprsComm.GprsStatus = GPRS_NO_INTERNET;
				return FALSE;
			}
//			Task_sleep(1000);

			/* TCPͨ��������� */
			if(!GPRS_TcpConf()){
				g_GprsComm.GprsStatus = GPRS_NOT_INIT;	/* ״̬����Ϊδ��ʼ�� */
				return FALSE;
			}
//			Task_sleep(1000);
			g_GprsComm.GprsStatus = GPRS_NORMAL;
			return TRUE;
		}
	} else if(g_GprsComm.GprsStatus == GPRS_NO_SIM && GPRS_Reset()) {	/* �Ҳ���SIM��, �������Ȳ��, ��λһ������ʶ��SIM�� */
			g_GprsComm.GprsStatus = GPRS_NOT_INIT;
			return FALSE;
	} else if(g_GprsComm.GprsStatus == GPRS_NO_INTERNET) {	/* �޷����� */
		/* �鿴����״̬�������總��״̬������TCPͨ�� */
		if(GPRS_GetInternetState() && GPRS_OpenInternet() && GPRS_TcpConf()) {
			g_GprsComm.GprsStatus = GPRS_NORMAL;
			return TRUE;
		} else if((GPRS_GetSigStrong() == 99) && GPRS_Reset()) {	/* �鿴�ź�ǿ�� */
			g_GprsComm.GprsStatus = GPRS_NOT_INIT;
			return FALSE;
		} else if(!GPRS_SendATCmd("AT\r", "OK", NULL, NULL, 3, GPRS_UART_TIMEOUT)) {	/* ģ�鿨���� */
			GPRS_Reset();	/* ��λģ�� */
			g_GprsComm.GprsStatus = GPRS_NOT_INIT;
			return FALSE;
		}
	} else {	/* ��֪�����Ļḳֵ��GPRS_COMM_FAIL */
		g_GprsComm.GprsStatus = GPRS_NOT_INIT;
	}

	return g_GprsComm.GprsStatus == GPRS_NORMAL;
}

/* ����socket������socket��Ϊȫ�ֱ���������������ͳһ+1��Ϊ�ܿ�NULL������ԭӦ�ò������� */
SOCKET GprsSocket(int32 domain, int32 type, int32 protocl)
{
	SOCKET sk = 0;
	if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
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
	}
	return sk;
}
int32 GprsConnect(SOCKET socketFd, struct sockaddr_in* pName, int32 i32Len)
{
	int32 i32Result = 0;
	if((socketFd <= 0) || (!pName) || (!i32Len)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
			/* ����ר�� */
			if(socketFd == g_MqttComm[MQTT_TYPE_PUB].MqttSocket) {
				printf("��Pub connect����\n");
			} else {
				printf("��Sub connect����\n");
			}
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
				if(GPRS_SendATCmd(NULL, "CONNECT OK", "CONNECT", NULL, 2, 10000)) {	/* �鿴��Ӧ */
					i32Result = GPRS_SUC;
				} else {	/* ��Ӧ��ʱ���͵ĸ�ʽ����. */
					i32Result = GPRS_ERR_CONNECTION_ABN;
				}
				Task_sleep(100*OS_TICK_KHz);
			} else {
				i32Result = GPRS_ERR_OFFLINE;
			}
			Semaphore_post(g_GprsComm.Sem_GprsReq);
		} else {
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}
	return i32Result;
}

int32 GprsSend(SOCKET socketFd, uint8 *pSrcBuf, uint16 uSendBlen, uint32 u32Flag)
{
#define MAX_ONCE_SEND_BLEN	1400
	int32 i32Result = 0;
	uint32 u32AllSentLen = 0;	/* �����Ѿ����͵��ֽ��� */
	uint32 u32AllSendLen = uSendBlen;	/* һ��Ҫ���͵��ֽ��� */
	if((socketFd <= 0) || (!uSendBlen) || (!pSrcBuf)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
			/* ����ר�� */
			if(socketFd == g_MqttComm[MQTT_TYPE_PUB].MqttSocket) {
				printf("��Pub Send����\n");
			} else {
				printf("��Sub Send����\n");
			}
			if((g_GprsComm.GprsStatus == GPRS_NORMAL) && (!pSocket->bIsFree)) {
				UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
				/* ģ�����֧��MAX_ONCE_SEND_BLEN�ֽڷִη��� */
//				uint8 u8SendTimes = (uSendBlen + MAX_ONCE_SEND_BLEN - 1) / MAX_ONCE_SEND_BLEN;
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
//					WriteToUart(GPRS_UART_PORT, pBuf - pUartComm->u8TRBuf);
					/* ���� */
					uint16 uSentLen = 0;
//					for(; i32Result == 0; ) {		/* ѭ����ֹ�����socket�����ϱ����������� */
					/* �����ʱ���Ͳ����ظ�'>'��, �����ʲôԭ�����������෢������ */
					if(GPRS_SendATCmd((char *)aU8Cmd, ">", NULL, &uReadNum, 3, 2000)) {	/* ��Ӧ�ɹ� */
						/* ��ʼ�������� */
						while(uSentLen < uEverySendBlen) {
							uint16 uSendLen = (uEverySendBlen - uSentLen) < UART_TR_BUF_BLEN ? (uEverySendBlen - uSentLen) : UART_TR_BUF_BLEN;
							memcpy(pUartComm->u8TRBuf, pSrcBuf + uSentLen, uSendLen);
							WriteToUart(GPRS_UART_PORT, uSendLen);
							uSentLen += uSendLen;
						}
						if(GPRS_SendATCmd(NULL, "SEND OK", NULL, NULL, 1, 1000)) {	/* ���ͳɹ� */
							u32AllSentLen += uEverySendBlen;
						} else {	/* ����ʧ�� ��Ϊ�ǶϿ����� */
							i32Result = GPRS_ERR_CONNECTION_ABN;
							break;
						}
					} else if(uReadNum != 0){	/* ��Ӧ��������Ϣ */
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
					} else {	/* δ��Ӧ, Ӧ���Ǵ������� */
						i32Result = GPRS_ERR_UART_ABN;
						break;
					}
				}
			} else {	/* GPRSû������socket������ */
				i32Result = GPRS_ERR_CONNECTION_ABN;
			}
			Semaphore_post(g_GprsComm.Sem_GprsReq);
		} else {	/* û�õ��� */
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}

	if(u32AllSentLen == u32AllSendLen) {	/* ������� */
		return u32AllSentLen;
	} else {	/* ����ʧ�� */
		return i32Result;
	}


//						if((uReadNum = ReadFromUart(GPRS_UART_PORT, 1000)) > 0) {
//							if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)">")) {		/* ���M25����> ˵�����Լ������ʹ����������� */
//								uint8 *pSrcBufEnd = pSrcBuf + uEverySendBlen;
//								/* ���������UART_TR_BUF_BLEN�ֽڣ��ִη���ȫ������ */
//								while (pSrcBuf < pSrcBufEnd) {
//									uint32 u32CopyBlen = (((pSrcBufEnd - pSrcBuf) < UART_TR_BUF_BLEN) ? (pSrcBufEnd - pSrcBuf) : UART_TR_BUF_BLEN);
//									memcpy(pUartComm->u8TRBuf, pSrcBuf, u32CopyBlen);
//									if (pSrcBufEnd - pSrcBuf <= u32CopyBlen) {
//										pUartComm->u8TRBuf[u32CopyBlen] = '\r';  	/* ���ͽ������� */
//										WriteToUart(GPRS_UART_PORT, u32CopyBlen + 1);
//									} else {
//										WriteToUart(GPRS_UART_PORT, u32CopyBlen);
//									}
//									pSrcBuf += u32CopyBlen; // ����ָ��
//								}
//								/* ����ģ��ص��Ƿ��ͳɹ����� */
////								memset(pUartComm->u8TRBuf, 0 , UART_TR_BUF_BLEN);
//								for(; i32Result == 0; ) {		/* ѭ����ֹ�����socket�����ϱ����������� */
//									if((uReadNum = ReadFromUart(GPRS_UART_PORT, 1000)) > 0) {
//										if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"SEND OK")) {
//											i32Result = (int32)uEverySendBlen;		/* ���ͳɹ� */
//											break;
//										} else if(!SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"ERROR")) {	/* ���������ݣ���Ҳû��ERROR������ԣ�Ҳ���ǿ��ܱ������Ϣ����� */
//											continue;
//										} else {
//											i32Result = GPRS_ERR_PARSE;
//										}
//									} else {
//										i32Result = GPRS_ERR_UART_ABN;
//									}
//								}
//								if(i32Result != 0) {
//									break;		/* ����յ���ȷ���Ľ���������˴ν��� */
//								}
//							} else if(!SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"ERROR")) {	/* ���������ݣ���Ҳû��ERROR������ԣ�Ҳ���ǿ��ܱ������Ϣ����� */
//								continue;
//							} else {
//								i32Result = GPRS_ERR_PARSE;
//							}
//						} else {
//							i32Result = GPRS_ERR_UART_ABN;
//						}
//					}
//					if(i32Result < 0) {
//						break;		/* ����ִη�������һ���������⣬�Ͳ��ٳ��Լ������� */
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
int32 GprsRecv(SOCKET socketFd, uint8 *pU8DstBuf, int32 i32ReadBLen, int32 i32Flags)
{
	int32 i32Result = 0;
	uint32 u32ResultLen = 0;	/* ��ȡ���ֽ��� */
	if((socketFd <= 0) || (!i32ReadBLen) || (!pU8DstBuf)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
		/* ���û���������������ź������ԣ��������20�� */
		uint8 u8EmptyRecvTryCnt = 2;
//		for(; u8EmptyRecvTryCnt > 0; u8EmptyRecvTryCnt--) {
			if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
				/* ����ר�� */
				if(socketFd == g_MqttComm[MQTT_TYPE_PUB].MqttSocket) {
					printf("��Pub Recv����\n");
				} else {
					printf("��Sub Recv����\n");
				}
				if((g_GprsComm.GprsStatus == GPRS_NORMAL) && (!pSocket->bIsFree)) {	/* GPRS���������׽��ֿ��� */
					/* ��ȷ���ϴε������Ѿ�ȫ������, �����Ƕѵ���M25�ķ��ͻ������� */
					if(!GPRS_WaitSocketIdle(socketFd, 0, 20, 10000)) {
						Semaphore_post(g_GprsComm.Sem_GprsReq);
						return GPRS_ERR_TIMEOUT;
					}
					/* ��MAX_READ_LENΪ��󵥴ζ�ȡ��С��ѭ����ȡȫ���������� */
					/* ��uart buf��СΪ��Ԫѭ����ѯ��ȡM25������ȫ���������ݣ�M25ÿsocket��󻺴�400K�� */
					UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
					char aChCmd[20];
					uint8* pBuf = (uint8 *)aChCmd;
					PrintString(&pBuf, pBuf + 20, "AT+QIRD=0,1,");
					*pBuf++ = '0' + pSocket->u8MuxId;
					*pBuf++ = ',';
					PrintU32(&pBuf, pBuf+ 12, (i32ReadBLen > MAX_READ_LEN ? MAX_READ_LEN : i32ReadBLen), 0);
					*pBuf++ = '\r';
					*pBuf++ = '\0';
					for(; i32ReadBLen > 0 && u8EmptyRecvTryCnt; ) {	/* ѭ������ʣ���ֽ� */
#if 1
						uint16 uUartReadBLen = 0;
						/* ������Ӧ, ����OK��Ϊ��ȡһ֡���. */
						pBuf = pUartComm->u8TRBuf;
						if(GPRS_SendATCmd(aChCmd, "+QIRD:", "OK", &uUartReadBLen, 2, 2000)) {
							if((pBuf = SkipCharInString(pBuf, pBuf + uUartReadBLen, ',', 2))) {	/* ��λ�����ݳ��ȵ�λ�� */
								uint16 uRecvFrameLen = ReadU32(&pBuf, pBuf + 5);	/* QIRD�������ݳ��� */
								/* ������Ч */
								if(uRecvFrameLen
									&& (uRecvFrameLen <= uUartReadBLen)
									&& (pBuf = SkipStrInString(pBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"\r\n")))
								{
									i32ReadBLen -= uRecvFrameLen;		/* ����ʣ���ֽ��� */
									u32ResultLen += uRecvFrameLen;		/* ���¶����ֽ��� */
									for(; uRecvFrameLen > 0; uRecvFrameLen--) {	/* ����Ӧ���ݿ������û������� */
										*pU8DstBuf++ = *pBuf++;
									}
								} else {	/* �޷�ʶ������֡ */
									i32Result = GPRS_ERR_PARSE;
									break;
								}
							} else {	/* �޷�ʶ������֡ */
								i32Result = GPRS_ERR_PARSE;
								break;
							}
						} else if(uUartReadBLen != 0) {	/* �յ����������͵���Ӧ */
							if((pBuf = SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"OK"))) {	/* ֻ��OK ˵��û������ */
								i32Result = GPRS_ERR_NO_DATA;
								if(u32ResultLen == 0) {	/* ��û��������, ������ */
									u8EmptyRecvTryCnt--;
									Task_sleep(200);
									continue;
								} else {			/* �Ѿ������������˿���ֱ�ӷ����� */
									break;
								}
							} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"CLOSE")
									|| SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8 *)"ERROR")) {	/* �Ƿ�Ͽ����� */
								i32Result = GPRS_ERR_CONNECTION_ABN;	/* �Ͽ����� */
								break;
							} else {	/* ��Ӧ������δʶ�����Ϣ, ���Ϊδʶ��� */
								i32Result = GPRS_ERR_PARSE;
								break;
							}
						} else {	/* ����ʱ,û����Ӧ */
//							if(GPRS_GetTcpState()) {
//								continue;
//							}
							i32Result = GPRS_ERR_TIMEOUT;
							break;
						}
					}
					if(i32Result != GPRS_ERR_CONNECTION_ABN) {
						i32Result = u32ResultLen > 0 ? u32ResultLen : i32Result;
					}
					Semaphore_post(g_GprsComm.Sem_GprsReq);
				} else {	/* GPRSû����, ���׽��ֲ����� */
					i32Result = GPRS_ERR_OFFLINE;
				}
			} else {	/* û�õ���, ���ض���ʱ */
				i32Result = GPRS_ERR_TIMEOUT;
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
								/* ��λ��QIRD�������ݳ����ֶ� */
								pBuf = SkipCharInString(pBuf, pUartComm->u8TRBuf + uUartReadBLen, ',', 2);
//								if(pBuf < (pUartComm->u8TRBuf + uUartReadBLen)) {
								uint16 uRecvFrameLen = ReadU32(&pBuf, pBuf + 5);		/* QIRD�������ݳ��� */
								if(uRecvFrameLen && (uRecvFrameLen <= uUartReadBLen) && (pBuf = SkipStrInString(pBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"\r\n"))) {
									i32ReadBLen -= uRecvFrameLen;		/* ����ʣ���ֽ��� */
									i32Result += uRecvFrameLen;			/* ���¶����ֽ��� */
									/* �������û�buf */
									for(; uRecvFrameLen > 0; uRecvFrameLen--) {
										*pU8DstBuf++ = *pBuf++;
									}
									continue;		/* ����д���MAX_READ_LEN�������ٴ������ȡ */
								} else {
									i32Result = GPRS_ERR_PARSE;
									break;
								}
//								} else {
//									i32Result = GPRS_ERR_PARSE;
//									break;
//								}
							} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"+CME ERROR: 58") > 0) {		/* �������Ƶ�������öϿ�ֱ������ */
								break;
							} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"CLOSED") > 0) {		/*  */
								i32Result = GPRS_ERR_CONNECTION_ABN;
								break;
							} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"ERROR") > 0) {		/* ���ӶϿ��˿��� */
								i32Result = GPRS_ERR_CONNECTION_ABN;
								break;
							} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"OK") > 0) {		/* ����ȫ�����ݺ�û�����κ����ݶ��᷵��OK */
								break;		/* M25 buf��û������ */
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
			/* ����ר�� */
			if(socketFd == g_MqttComm[MQTT_TYPE_PUB].MqttSocket) {
				printf("��Pub Close����\n");
			} else {
				printf("��Sub Close����\n");
			}
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
		} else {
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}
	return i32Result;
}

int32 GprsSetSockopt(SOCKET socketFd, int32 u32Level, int32 u32Op, void *pbuf, int32 i32Bufsize)
{
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

/* ��ʼ�����λ����� */
void RingBuffer_Init(RingBuffer_t *pRingBuffer)
{
	pRingBuffer->uRear = 0;
	pRingBuffer->uHead = 0;
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

/* ��ȡ���ζ������������� */
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
