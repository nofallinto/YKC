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
#define MAX_GPRS_RETRY_CNT	3		/* ��ʱ��Ҫ���ͨ�ţ���������Դ��� */
/* ���GPRS����״̬������������ȳ��Ը�λ���³�ʼ�����������ʧ����˴β��ٳ���
 * �˺���������ͬ����ͬ�������ⲿ����
 * ����GPRS����״̬�Ƿ�����
 * ��ضϿ�stlink��3��3v�� */
BOOL CheckGprsStatus(void)
{
	extern UART_HandleTypeDef huart4;

	if(g_GprsComm.GprsStatus != GPRS_NORMAL) {
		uint8 i = 0;
		uint16 uReadNum = 0;
		uint8* pStart = g_UartComm[GPRS_UART_PORT].u8TRBuf;
		uint8* pBuf = g_UartComm[GPRS_UART_PORT].u8TRBuf;
		BOOL bCommSuc = FALSE;
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

			OpenUartComm(0, 115200, 0, 10);	/* ���Դ��� */

			/* ͬ�������ʣ�����ʱ������Ӧ������ */
			pBuf = g_UartComm[GPRS_UART_PORT].u8TRBuf;
			memset(pBuf, 0, UART_TR_BUF_BLEN);
			OpenUartComm(GPRS_UART_PORT, 9600, 0, 30);
			PrintString(&pBuf, pBuf + 16, "AT+IPR=115200\r");
			WriteToUart(GPRS_UART_PORT, pBuf - pStart);
			BOOL needSaveBaud = FALSE;
			uReadNum = 0;
			if((uReadNum = ReadFromUart(GPRS_UART_PORT, GPRS_UART_TIMEOUT)) > 0) {
				if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"OK")) {
					needSaveBaud = TRUE;
				}
			}

			/* ��M25ģ�鷢�ͳ�ʼ��ָ�� */
			memset(pBuf, 0, UART_TR_BUF_BLEN);
			Task_sleep(OS_TICK_KHz*1000);
			OpenUartComm(GPRS_UART_PORT, 115200, 0, 15);
			if(needSaveBaud == TRUE) {
				pBuf = pStart;
				PrintString(&pBuf, pBuf + 16, "AT&W\r");
				WriteToUart(GPRS_UART_PORT, pBuf - pStart);
			}
			g_UartComm[GPRS_UART_PORT].uTmr_Run_ms = 3000;
			bCommSuc = FALSE;

			pBuf = pStart;
			memset(pBuf, 0, UART_TR_BUF_BLEN);
			PrintString(&pBuf, pBuf + 10, "AT\r");
			WriteToUart(GPRS_UART_PORT, pBuf - pStart);
			if((uReadNum = ReadFromUart(GPRS_UART_PORT, GPRS_UART_TIMEOUT)) > 0) {
				if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"OK")) {
					/* GPRS ͨ������, ��Ҫ�����������mqtt�� */
					bCommSuc = TRUE;
					g_CodeTest.u32Val[0] = 1;
				} else {
					/* GPRS �쳣����Ҫ�����������mqtt�� */
					g_CodeTest.u32Val[0] = 2;
				}
			} else {
				g_CodeTest.u32Val[0] = 11;
			}
			Task_sleep(1000);

			/* �ر�������� */
			if(bCommSuc) {
				bCommSuc = FALSE;
				for(i = MAX_GPRS_RETRY_CNT; i > 0; i--) {
					pBuf = pStart;
					memset(pBuf, 0, UART_TR_BUF_BLEN);
					PrintString(&pBuf, pBuf + 20, "ATE0\r");
					WriteToUart(GPRS_UART_PORT, pBuf - pStart);
					if((uReadNum = ReadFromUart(GPRS_UART_PORT, GPRS_UART_TIMEOUT)) > 0) {
						if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"OK")) {
							bCommSuc = TRUE;
							break;
						}
					}
					Task_sleep(100);
				}
			}

			/* �ر�SENDָ�����ݻ��ԣ�0������ */
			if(bCommSuc) {
				bCommSuc = FALSE;
				for(i = MAX_GPRS_RETRY_CNT; i > 0; i--) {
					pBuf = pStart;
					memset(pBuf, 0, UART_TR_BUF_BLEN);
					PrintString(&pBuf, pBuf + 20, "AT+QISDE=0\r");
					WriteToUart(GPRS_UART_PORT, pBuf - pStart);
					if((uReadNum = ReadFromUart(GPRS_UART_PORT, GPRS_UART_TIMEOUT)) > 0) {
						if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"OK")) {
							bCommSuc = TRUE;
							break;
						}
					}

					Task_sleep(1000);
				}
			}
			if(bCommSuc) {
				bCommSuc = FALSE;
				for(i = MAX_GPRS_RETRY_CNT; i > 0; i--) {
					pBuf = pStart;
					memset(pBuf, 0, UART_TR_BUF_BLEN);
					PrintString(&pBuf, pBuf + 20, "AT+QISDE=0\r");
					WriteToUart(GPRS_UART_PORT, pBuf - pStart);
					if((uReadNum = ReadFromUart(GPRS_UART_PORT, GPRS_UART_TIMEOUT)) > 0) {
						if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"OK")) {
							bCommSuc = TRUE;
							break;
						}
					}

				}
			}

			/* ����SIM����ע������ */
			if(bCommSuc && (g_GprsComm.GprsStatus == GPRS_NO_SIM)) {
				Task_sleep(100);
				bCommSuc = FALSE;
				for(i = MAX_GPRS_RETRY_CNT; i > 0; i--) {
					pBuf = pStart;
					memset(pBuf, 0, UART_TR_BUF_BLEN);
					PrintString(&pBuf, pBuf + 20, "AT+CFUN=1\r");
					WriteToUart(GPRS_UART_PORT, pBuf - pStart);
					if((uReadNum = ReadFromUart(GPRS_UART_PORT, 1000)) > 0) {
						if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"OK")) {
							bCommSuc = TRUE;
							Task_sleep(500);
							break;
						}
					}

					Task_sleep(100);
				}
			}

			/* SIM �� PIN �Ƿ��Ѿ��� */
			if(bCommSuc) {
				Task_sleep(100);
				bCommSuc = FALSE;
				for(i = MAX_GPRS_RETRY_CNT; i > 0; i--) {
					pBuf = pStart;
					memset(pBuf, 0, UART_TR_BUF_BLEN);
					PrintString(&pBuf, pBuf + 20, "AT+CPIN?\r");
					WriteToUart(GPRS_UART_PORT, pBuf - pStart);
					if((uReadNum = ReadFromUart(GPRS_UART_PORT, GPRS_UART_TIMEOUT)) > 0) {
						if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"READY")) {
							bCommSuc = TRUE;		/* (U)SIM ���� PIN ���ѽ� */
							Task_sleep(200);
							break;
						} else if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"SIM not inserted")) {
							g_GprsComm.GprsStatus = GPRS_NO_SIM;		/* δ�忨�����mqtt  �ı�sim�����״̬ */
						} else {
							g_GprsComm.GprsStatus = GPRS_NO_SIM;		/* ����ظ������mqtt */
						}
					}

					Task_sleep(100);
				}
			}

			if(bCommSuc) {
				Task_sleep(100);
				bCommSuc = FALSE;
				for(i = MAX_GPRS_RETRY_CNT; i > 0; i--) {
					pBuf = pStart;
					memset(pBuf, 0, UART_TR_BUF_BLEN);
					PrintString(&pBuf, pBuf + 20, "AT+CREG?\r");
					WriteToUart(GPRS_UART_PORT, pBuf - pStart);
					/* SIM �� PIN �Ƿ��Ѿ��� */
					if((uReadNum = ReadFromUart(GPRS_UART_PORT, GPRS_UART_TIMEOUT)) > 0) {
						if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"+CREG: 0,1")) {
							bCommSuc = TRUE;		/* �����ɹ� */
							break;
						}
	#if 0
							Task_sleep(100);
							memset(pBuf, 0, UART_TR_BUF_BLEN);
							PrintString(&pBuf, pBuf + 20, "AT+CSQ\r");
							WriteToUart(GPRS_UART_PORT, pBuf - pStart);
							if((uReadNum = ReadFromUart(GPRS_UART_PORT, GPRS_UART_TIMEOUT)) > 0) {
								/* ��ѯģ����ź�ǿ�ȣ���һ��ֵΪ0-31����������������8��������22��99Ϊ������ */
							}
	#endif
					}
					Task_sleep(100);
				}
			}

			if(bCommSuc) {
				Task_sleep(100);
				bCommSuc = FALSE;
				for(i = MAX_GPRS_RETRY_CNT; i > 0; i--) {
					pBuf = pStart;
					memset(pBuf, 0, UART_TR_BUF_BLEN);
					PrintString(&pBuf, pBuf + 20, "AT+CGATT?\r");
					WriteToUart(GPRS_UART_PORT, pBuf - pStart);
					/* GPRS ���總��״̬ */
					if((uReadNum = ReadFromUart(GPRS_UART_PORT, GPRS_UART_TIMEOUT)) > 0) {
						if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"+CGATT: 1")) {
							bCommSuc = TRUE;		/* GPRS ���總�ųɹ� */
							Task_sleep(100);
							break;
						} else {
							/* Ƿ�ѿ��ܵ���CGATT: 0 */
						}
					}

					Task_sleep(100);
				}
			}

			/* AT+QIMUX = 1 �������ö�·����*/
			if(bCommSuc) {
				for(i = MAX_GPRS_RETRY_CNT; i > 0; i--) {
					bCommSuc = FALSE;
					pBuf = pStart;
					PrintString(&pBuf, pBuf + 20, "AT+QIMUX?\r");
					WriteToUart(GPRS_UART_PORT, pBuf - pStart);
					if((uReadNum = ReadFromUart(GPRS_UART_PORT, 300)) > 0) {
						uint8* pRes = NULL;
						if((pRes = SkipStrInString(pStart, pStart + uReadNum, (uint8*)"+QIMUX: 1"))) {
							bCommSuc = TRUE;
							Task_sleep(100);
							break;		/* �Ѿ��Ƕ�·����״̬ */
						} else {
							int8 j;
							for(j = MAX_GPRS_RETRY_CNT; j > 0; j--) {
								pBuf = pStart;
								memset(pBuf, 0, UART_TR_BUF_BLEN);
								PrintString(&pBuf, pBuf + 20, "AT+QIMUX=1\r");
								WriteToUart(GPRS_UART_PORT, pBuf - pStart);
								if((uReadNum = ReadFromUart(GPRS_UART_PORT, GPRS_UART_TIMEOUT + 500)) > 0) {
									if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"OK")) {
										bCommSuc = TRUE;
										Task_sleep(100);
										break;
									}
								}
								Task_sleep(100);
							}
							break;
						}
					}
					Task_sleep(100);
				}
			}

			if(bCommSuc) {
				Task_sleep(100);
				bCommSuc = FALSE;
				for(i = MAX_GPRS_RETRY_CNT; i > 0; i--) {
					pBuf = pStart;
					memset(pBuf, 0, UART_TR_BUF_BLEN);
					PrintString(&pBuf, pBuf + 20, "AT+QIDNSIP=1\r");
					WriteToUart(GPRS_UART_PORT, pBuf - pStart);
					/* GPRS ���總��״̬ */
					if((uReadNum = ReadFromUart(GPRS_UART_PORT, GPRS_UART_TIMEOUT)) > 0) {
						if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"OK")) {
							bCommSuc = TRUE;		/* GPRS ���總�ųɹ� */
							Task_sleep(100);
							break;
						}
					}
					Task_sleep(100);
				}
			}

			/* AT+QINDI=2 ��͸�������յ������ݷŵ��������У�����ÿһ��socket��һ��������*/
			if(bCommSuc) {
				Task_sleep(100);
				bCommSuc = FALSE;
				for(i = MAX_GPRS_RETRY_CNT; i > 0; i--) {
					pBuf = pStart;
					memset(pBuf, 0, UART_TR_BUF_BLEN);
					PrintString(&pBuf, pBuf + 20, "AT+QINDI=2\r");
					WriteToUart(GPRS_UART_PORT, pBuf - pStart);
					if((uReadNum = ReadFromUart(GPRS_UART_PORT, GPRS_UART_TIMEOUT)) > 0) {
						if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"OK")) {
							bCommSuc = TRUE;
							Task_sleep(100);
							break;
						}
					}
					Task_sleep(100);
				}
			}

			if(bCommSuc) {
				g_UartComm[GPRS_UART_PORT].bUartFuncTrySuc = TRUE;
				g_UartComm[GPRS_UART_PORT].uTimer_WatchDog_ms = 0;
				g_CodeTest.u32Val[1] = 200;
				Task_sleep(300);
				g_GprsComm.GprsStatus = GPRS_NORMAL;
				break;
			}
		}
		/* GPRS ͨ��ʧ��, ��Ҫ�����������mqtt�� */
		if(!bCommSuc) {
			for(i = MAX_GPRS_RETRY_CNT; i > 0; i--) {
				pBuf = pStart;
				memset(pBuf, 0, UART_TR_BUF_BLEN);
				PrintString(&pBuf, pBuf + 20, "AT+CFUN=0,1\r");
				WriteToUart(GPRS_UART_PORT, pBuf - pStart);
				if((uReadNum = ReadFromUart(GPRS_UART_PORT, 1000)) > 0) {
					if(SkipStrInString(pStart, pStart + uReadNum, (uint8*)"OK")) {
						Task_sleep(500);
						break;
					}
				}
				Task_sleep(100);
			}
			Task_sleep(g_UartComm[GPRS_UART_PORT].uTmr_Run_ms);
		}
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
				uint16 uTimeout = 300;
				uint16 uReadNum = 0;
				int8 i;
				for(i = 2; i > 0; i--) {		/* ֮������ѭ�����ζ�ȡ����ΪOK��֮���CONNECT OK֮��ļ���ɴ��С */
					if((uReadNum = ReadFromUart(GPRS_UART_PORT, uTimeout)) > 0) {
						if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"CONNECT OK")) {
							i32Result = GPRS_SUC;
							break;
						} else if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"OK")) {
							uTimeout = 2000;
							i32Result = GPRS_ERR_TIMEOUT;
						} else {
							i32Result = GPRS_ERR_PARSE;
							i++;
						}
					} else {
						i32Result = GPRS_ERR_UART_ABN;
						break;
					}
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

//	sprintf(Rx_buff,"connect:%ld",i32Result);
//	HAL_UART_Transmit(&huart4, (const uint8 *)Rx_buff, strlen(Rx_buff), 500);
	return i32Result;
}

int32 GprsSend(SOCKET socketFd, uint8 *pSrcBuf, uint16 uSendBlen, uint32 u32Flag)
{
#define MAX_ONCE_SEND_BLEN	1400
	int32 i32Result = 0;
	if((socketFd <= 0) || (!uSendBlen) || (!pSrcBuf)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
			if((g_GprsComm.GprsStatus == GPRS_NORMAL) && (!pSocket->bIsFree)) {
				UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
				/* ģ�����֧��MAX_ONCE_SEND_BLEN�ֽڷִη��� */
				uint8 u8SendTimes = (uSendBlen + MAX_ONCE_SEND_BLEN - 1) / MAX_ONCE_SEND_BLEN;
				for(; u8SendTimes > 0; u8SendTimes--) {
					/* ����MAX_ONCE_SEND_BLEN�ķִη��� */
					uint16 uEverySendBlen;
					if(uSendBlen > MAX_ONCE_SEND_BLEN) {
						uEverySendBlen = MAX_ONCE_SEND_BLEN;
						uSendBlen -= MAX_ONCE_SEND_BLEN;
					} else {
						uEverySendBlen = uSendBlen;
					}
					/* ����ӡ */
					uint8* pBuf = pUartComm->u8TRBuf;
					PrintString(&pBuf, pBuf + 20, "AT+QISEND=");
					*pBuf++ = '0' + pSocket->u8MuxId;
					*pBuf++ = ',';
					PrintU32(&pBuf, pBuf + 12, uEverySendBlen, 0);
					*pBuf++ = '\r';
					/* ִ�з��� */
					WriteToUart(GPRS_UART_PORT, pBuf - pUartComm->u8TRBuf);
					uint16 uReadNum = 0;
					/* ���� */
					for(; i32Result == 0; ) {		/* ѭ����ֹ�����socket�����ϱ����������� */
						if((uReadNum = ReadFromUart(GPRS_UART_PORT, 300)) > 0) {
							if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)">")) {		/* ���M25����> ˵�����Լ������ʹ����������� */
								uint8 *pSrcBufEnd = pSrcBuf + uEverySendBlen;
								/* ���������UART_TR_BUF_BLEN�ֽڣ��ִη���ȫ������ */
								while (pSrcBuf < pSrcBufEnd) {
									uint32 u32CopyBlen = (((pSrcBufEnd - pSrcBuf) < UART_TR_BUF_BLEN) ? (pSrcBufEnd - pSrcBuf) : UART_TR_BUF_BLEN);
									memcpy(pUartComm->u8TRBuf, pSrcBuf, u32CopyBlen);
									if (pSrcBufEnd - pSrcBuf <= u32CopyBlen) {
										pUartComm->u8TRBuf[u32CopyBlen] = '\r';  	/* ���ͽ������� */
										WriteToUart(GPRS_UART_PORT, u32CopyBlen + 1);
									} else {
										WriteToUart(GPRS_UART_PORT, u32CopyBlen);
									}
									pSrcBuf += u32CopyBlen; // ����ָ��
								}
								/* ����ģ��ص��Ƿ��ͳɹ����� */
								memset(pUartComm->u8TRBuf, 0 , UART_TR_BUF_BLEN);
								for(; i32Result == 0; ) {		/* ѭ����ֹ�����socket�����ϱ����������� */
									if((uReadNum = ReadFromUart(GPRS_UART_PORT, 1000)) > 0) {
										if(SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + pUartComm->uRxBufPt, (uint8*)"SEND OK")) {
											i32Result = (int32)uEverySendBlen;		/* ���ͳɹ� */
											break;
										} else if(!SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"ERROR")) {	/* ���������ݣ���Ҳû��ERROR������ԣ�Ҳ���ǿ��ܱ������Ϣ����� */
											continue;
										} else {
											i32Result = GPRS_ERR_PARSE;
										}
									} else {
										i32Result = GPRS_ERR_UART_ABN;
									}
								}
								if(i32Result != 0) {
									break;		/* ����յ���ȷ���Ľ���������˴ν��� */
								}
							} else if(!SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uReadNum, (uint8*)"ERROR")) {	/* ���������ݣ���Ҳû��ERROR������ԣ�Ҳ���ǿ��ܱ������Ϣ����� */
								continue;
							} else {
								i32Result = GPRS_ERR_PARSE;
							}
						} else {
							i32Result = GPRS_ERR_UART_ABN;
						}
					}
					if(i32Result < 0) {
						break;		/* ����ִη�������һ���������⣬�Ͳ��ٳ��Լ������� */
					}
	//				Task_sleep(20*OS_TICK_KHz);
				}
			}
			Semaphore_post(g_GprsComm.Sem_GprsReq);
		} else {
			i32Result = GPRS_ERR_TIMEOUT;
		}
	}
	return i32Result;
}


#define MAX_QIRD_HEADER_LEN 44
#define	MAX_READ_LEN		(UART_TR_BUF_BLEN - MAX_QIRD_HEADER_LEN)
int32 GprsRecv(SOCKET socketFd, uint8 *pU8DstBuf, int32 i32ReadBLen, int32 i32Flags)
{
	int32 i32Result = 0;
	if((socketFd <= 0) || (!i32ReadBLen) || (!pU8DstBuf)) {
		i32Result = GPRS_ERR_ARG;
	} else {
		GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
		/* ���û���������������ź������ԣ��������20�� */
		uint8 u8EmptyRecvTryCnt = 20;
		for(; u8EmptyRecvTryCnt > 0; u8EmptyRecvTryCnt--) {
			if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
				if((g_GprsComm.GprsStatus == GPRS_NORMAL) && (!pSocket->bIsFree)) {
					/* ��MAX_READ_LENΪ��󵥴ζ�ȡ��С��ѭ����ȡȫ���������� */
					/* ��uart buf��СΪ��Ԫѭ����ѯ��ȡM25������ȫ���������ݣ�M25ÿsocket��󻺴�400K�� */
					UART_COMM* pUartComm = &g_UartComm[GPRS_UART_PORT];
					uint8* pBuf = NULL;
					for(; i32ReadBLen > 0; ) {
						pBuf = pUartComm->u8TRBuf;
						PrintString(&pBuf, pUartComm->u8TRBuf + 20, "AT+QIRD=0,1,");
						*pBuf++ = '0' + pSocket->u8MuxId;
						*pBuf++ = ',';
						PrintU32(&pBuf, pBuf+ 12, (i32ReadBLen > MAX_READ_LEN ? MAX_READ_LEN : i32ReadBLen), 0);
						*pBuf++ = '\r';
						WriteToUart(GPRS_UART_PORT, pBuf - pUartComm->u8TRBuf);
						memset(pUartComm->u8TRBuf, 0 , UART_TR_BUF_BLEN);
						uint16 uUartReadBLen = ReadFromUart(GPRS_UART_PORT, 300);
						if(uUartReadBLen > 0) {
							if((pBuf = SkipStrInString(pUartComm->u8TRBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"+QIRD:")) != NULL) {
								/* ��λ��QIRD�������ݳ����ֶ� */
								pBuf = SkipCharInString(pBuf, pUartComm->u8TRBuf + uUartReadBLen, ',', 2);
								if(pBuf < (pUartComm->u8TRBuf + uUartReadBLen)) {
									uint16 uRecvFrameLen = ReadU32(&pBuf, pBuf + 5);		/* QIRD�������ݳ��� */
									if(uRecvFrameLen && (uRecvFrameLen <= uUartReadBLen) && (pBuf = SkipStrInString(pBuf, pUartComm->u8TRBuf + uUartReadBLen, (uint8*)"\r\n"))) {
										i32ReadBLen -= uRecvFrameLen;			/* ����ʣ���ֽ��� */
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
								} else {
									i32Result = GPRS_ERR_PARSE;
									break;
								}
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
}

int32 GprsClose(SOCKET socketFd)
{
	int32 i32Result = 0;
	if(socketFd <= 0) {
		i32Result = GPRS_ERR_ARG;
	} else {
		GPRS_SOCKET* pSocket = &g_GprsComm.Socket[socketFd-1];
		if(Semaphore_pend(g_GprsComm.Sem_GprsReq, OS_TICK_KHz*GPRS_REQ_TIMEOUT_ms)) {
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
