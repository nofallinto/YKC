/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved. ģ��V1.4
 *
 * File			: MdlViewTech.c
 * Author		: Wang Renfei
 * Description	: V5��Ʒƽ̨ͳһUI��ʾģ�飬֧�֣�
 			1. ����(ViewTech)��ʾ��(ȫ��ҳ��),����UART(RS485)
 			2. MCGS��Ļ(ItemPageҳ�沿��),����UART(RS485)
 			3. MQTTԶ�����ã���������

			ҳ����Ҫ��Ϊ��������: ItemPageҳ��(������ʾConf, Stat, Msg, Acq)���Լ�����ҳ��(����ҳ�桢����ҳ�����Ҫ��ʾ��ֵ��Ϣ��)
 			ItemPage�ֿ��Խ�һ����Ϊ����: ����ͳ��ҳ�棬��Ϣ�ɼ�ҳ��
 				����ͳ��ҳ��ĺ��Ĺ����Ƕ� CONF_n_STAT_PROP �ṹ��Ľ���
 				��Ϣ�ɼ�ҳ��ĺ��Ĺ�����ͨ�����ʽӿڻ�ȡ�Ѿ��洢������

 			ViewTech���ǵ���ʾ�㷨��:
 			1. ����Ļ���豸�Ϲ���һһ��Ӧ����ʾԪ��(���󲿷����ı���ʾ�ؼ�)�����������ʾԪ��ͨ��ͨѶ��ַ�����������������������Ʋ���
 			2. ���豸�ϰѵ�ǰҳ�����ʾ�������н���(��Ҫ��ת����ı�)
 			3. ͨ��ͨѶ�ӿ�д����Ļ��Ӧ����ʾԪ����ַ�ϣ��Ӷ���ɱ�������ʾ

 			MCGS������ҳ�棬��mcgsͨ��modbusRTUЭ����ʾ�������ı������ϲ���modbusRTUЭ��ʵ���ϣ�������λ������һ�����������⴦��

 * Date			: 2017-3-11
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * V1.0	| 2005-2-28   |	king | 1. Create
 **************************************************************************/

/***************************************************************************
 						macro definition
***************************************************************************/
#define _MDL_GUI_C_							/* exclude redefinition */

/***************************************************************************
 include files
 ***************************************************************************/
/* ��ؿ��ļ� */
//#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "string.h"

/* ��Ŀ����ͷ�ļ� */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"
#include "MdlDataAccess.h"

/* �����Լ��¼�ģ��ͷ�ļ� */
#include "MdlNet.h"
#include "MdlUARTnModbus.h"
#include "MdlGUI.h"
#include "BoardSupport.h"

#if SUPPORT_IEC101 || SUPPORT_IEC104
#include "Mdl101n104.h"
#endif

/***************************************************************************
 						global variables definition
***************************************************************************/

/***************************************************************************
 						ViewTech
***************************************************************************/
#if SUPPORT_SCREEN
#define OFFSCREEN_TIMEOUT							10			/* Ĭ��Ϣ����ʱ */

/* ���� */
#define VT_REG_MODE_CONFIG							0x12		/* VIEWTECH ����Ϣ�����ȼĴ�����д0ʵ��Ϣ�� */
#define VT_REG_R0ToRC_REG_CONFIRM					0x1D		/* VIEWTECH дR0-Rc�Ĵ�����Ч�Ĵ�����д��0xA5Ӧ���޸� */
#define VT_VAL_DISABLE_OFF_SCREEN					0x16		/* VIEWTECH �ر����� */

/* ����ҳ */
#define VT_VAL_CONFIG_INPT_ITM_FIRST 				0			/* ���ּ��̣�Item�����������-��һ��ֵ*/
#define VT_VAL_CONFIG_INPT_ITM_SECOND 				1			/* ���ּ��̣�Item�����������-�ڶ���ֵ*/
#define VT_VAL_CONFIG_INPT_ITM_PASSWORD 			2			/* �������*/
#define VT_VAL_CONFIG_INPT_ITM_ASCII 				3			/* Ӣ�ļ���*/

#define VT_LEN_CONFIG_UNIT_PAGER 					12			/* ҳ���ֶγ��ȣ� �ֽ�*/
#define VT_VAR_CONFIG_UNIT_TITLE					0x0100		/* ����		62�ֽ�   */
#define VT_VAR_CONFIG_UNIT_LINE4					0x017C		/* ����������	62�ֽ�   */

#define VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE1		0x01F0  	/* ��һ�и���ͼƬ��0�رգ�1��ʾ   */
#define VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE2		0x01F1		/* �ڶ��и���ͼƬ��0�رգ�1��ʾ   */
#define VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE3		0x01F2  	/* �����и���ͼƬ��0�رգ�1��ʾ   */
#define VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE4		0x01F3  	/* �����и���ͼƬ��0�رգ�1��ʾ   */
#define VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE5		0x01F4  	/* �����и���ͼƬ��0�رգ�1��ʾ   */
#define VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE6		0x01F5   	/* �����и���ͼƬ��0�رգ�1��ʾ   */

#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE1_NON_2ND	0x3021		/* ��ʾ��Ŀ��һ�е�һλ���ɱ༭��   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE1_2ND		0x3020     	/* ��ʾ��Ŀ��һ�еڶ�λ���ɱ༭��   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE2_NON_2ND	0x3031     	/* ��ʾ��Ŀ�ڶ��е�һλ���ɱ༭��   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE2_2ND		0x3030     	/* ��ʾ��Ŀ�ڶ��еڶ�λ���ɱ༭��   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE3_NON_2ND	0x3041     	/* ��ʾ��Ŀ�����е�һλ���ɱ༭��   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE3_2ND		0x3040     	/* ��ʾ��Ŀ�����еڶ�λ���ɱ༭��   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE4_NON_2ND	0x3051   	/* ��ʾ��Ŀ�����е�һλ���ɱ༭��   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE4_2ND		0x3050    	/* ��ʾ��Ŀ�����еڶ�λ���ɱ༭��   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE5_NON_2ND	0x3061     	/* ��ʾ��Ŀ�����е�һλ���ɱ༭��   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE5_2ND		0x3060     	/* ��ʾ��Ŀ�����еڶ�λ���ɱ༭��   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE6_NON_2ND	0x3071     	/* ��ʾ��Ŀ�����е�һλ���ɱ༭��   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE6_2ND		0x3070     	/* ��ʾ��Ŀ�����еڶ�λ���ɱ༭��   */

#define VT_VAL_CONFIG_PNLKEY_CLICK_BACK				0x4000		/* ���ذ��� */
#define VT_VAL_CONFIG_PNLKEY_CLICK_UP				0x4001		/* ���ϰ��� */
#define VT_VAL_CONFIG_PNLKEY_CLICK_DOWN				0x4002		/* ���°��� */

/* ��Ϣ���ɼ�ҳ */
#define VT_VAR_MSGACQ_UNIT_LINE1					0x0300		/* ��һ��*/
#define VT_VAR_MSGACQ_UNIT_LINE4 					0x0360		/* ������*/
#define VT_VAR_MSGACQ_UNIT_LINE7 					0x03C0		/* ������*/
#define VT_VAR_MSGACQ_DEBUG_BTN_ENABLE				0x03FF		/* ���԰�ť�Ƿ���� */
#define VT_VAR_MSGACQ_COLOR_LINE1					0x0453		/* ��һ����ɫ����ָ�� */
#define VT_VAR_MSGACQ_COLOR_LINE2					0x0463		/* �ڶ�����ɫ����ָ�� */
#define VT_VAR_MSGACQ_COLOR_LINE3					0x0473		/* ��������ɫ����ָ�� */
#define VT_VAR_MSGACQ_COLOR_LINE4					0x0483		/* ��������ɫ����ָ�� */
#define VT_VAR_MSGACQ_COLOR_LINE5					0x0493		/* ��������ɫ����ָ�� */
#define VT_VAR_MSGACQ_COLOR_LINE6					0x04A3		/* ��������ɫ����ָ�� */
#define VT_VAR_MSGACQ_COLOR_LINE7					0x04B3		/* ��������ɫ����ָ�� */
#define VT_VAR_MSGACQ_COLOR_LINE8					0x04C3		/* �ڰ�����ɫ����ָ�� */

#define VT_VAL_MSGACQ_PNLKEY_CLICK_UP	 			0x4001     	/* ���ϰ��� */
#define VT_VAL_MSGACQ_PNLKEY_CLICK_DOWN	 			0x4002     	/* ���°��� */

typedef struct {
	uint8 u8Title[VT_LEN_ITEM_CHAR];  			/* 62�ֽ�	����*/
	uint8 u8TextLines[3][VT_LEN_ITEM_CHAR]; 	/* 3*62�ֽ�  3���ı�*/
}CONF_DAT_FRAME1;

typedef struct {
	uint8 u8TextLines[3][VT_LEN_ITEM_CHAR];  	/* 3*62�ֽ� 3���ı�*/
	uint16 uEditerStyleLines[6];				/* 6�б༭ͼƬ��ʽ��0����1���༭��2˫�༭��3�󵥱༭��	6*2�ֽ�   */
	uint8 u8UNIT_PAGER[12];						/* ҳ�� 65535/65535		12�ֽ�   */
}CONF_DAT_FRAME2;

/***************************************************************************
						internal functions declaration
***************************************************************************/
BOOL CheckRxBuf_VT(UART_COMM* pUartComm, uint16 uDevAddr);
void HandlePageWelcome_VT(uint8 u8UartPort);
void HandlePageConf_VT(uint8 u8UartPort, uint16 uPnlKey);
void HandlePageMsgAcq_VT(uint8 u8UartPort, uint16 uPnlKey);
void DealGuiConfWinBack(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq);
void DealGuiConfWinUp(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq);
void DealGuiConfWinDown(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq);

void LeftAlignString_MCGS(uint8* pDest, uint32 u32TextBufBLen);
ITEM_DATA_TYPE GetItemUIType(CONF_n_STAT_PROP* pDataAcsProp);

/***************************************************************************
 							functions definition
***************************************************************************/
/*==========================================================================
| Description	: ViewTech��ҳ�桢����ҳ�桢����ҳ��ԭ��
| G/Out var		:
| Author		: Wang Renfei			Date	: 2020-2-15
\=========================================================================*/
__weak void HandlePageMain_VT(uint8 u8UartPort, uint16 uPnlKey)
{
}
__weak void HandlePageAssi_VT(uint8 u8UartPort)
{
}
__weak void HandlePageDebug_VT(uint8 u8UartPort, uint16 uPnlKey)
{
}
__weak void HandlePageSystem_VT(uint8 u8UartPort, uint16 uPnlKey)
{
}
__weak uint16 FindMsgAcqIDColor(DATA_PAGE DataPage, uint16 uMsgID)			/* MsgID ת ��ɫ ���� */
{
	return VT_VAL_COLOR_YELLOW;
}
__weak void HandleExtScn_VT(uint8 u8UartPort)
{
}

/*==========================================================================
| Description	: ģ���ʼ��
| G/Out var		:
| Author		: Wang Renfei			Date	: 2008-3-4
\=========================================================================*/
void TryViewTech(uint8 u8UartPort)
{
	BOOL bSuccess = FALSE;
	ITEM_PAGE_REQ* pPageReq = &g_UartComm[u8UartPort].ItemPageReq;
	OpenUartComm(u8UartPort, 115200UL, 0, 0);

	/* ���Ժ�������Ļͨ�ţ�ͬʱҲ���ڶ�ȡ��Ļ���� */
	if(ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_VAR, VT_VAR_DEVICE_TYPE, 1)) {
		pPageReq->VTCommVer = (VT_COMM_VER)g_UartComm[u8UartPort].u8TRBuf[VT_BUF_RDVAR+1];
		bSuccess = TRUE;
	}
	bSuccess |= ReadData_VT(u8UartPort, VT_DEV_EXT, VIEW_TECH_READ_VAR, VT_VAR_DEVICE_TYPE, 1);

	/* ֻҪ��һ����Ļͨ�ųɹ�����ɹ� */
	if(bSuccess) {
		/* ȫ�ֱ�����ʼ�� */
		pPageReq->u8PageRefresh1Hz_100ms = 0;
		pPageReq->u8SlowRefresh_1Hz = 0;
		pPageReq->u32ItemNo = 0;
		pPageReq->u8InputingType = 0;
		pPageReq->bUnprocessedInput = FALSE;
		pPageReq->u8Tmr_OffScn_1s = OFFSCREEN_TIMEOUT;
		pPageReq->DataPage = DATA_PAGE_MAIN;

		pPageReq->ExtPage = DATA_PAGE_TEMP_SINGLE;
		pPageReq->u8Tmr_ExtOffScn_1s = OFFSCREEN_TIMEOUT;
		pPageReq->u8ExtItemNo = 0;
	}
}

/*==========================================================================
| Description	: ����ViewTech��Ļ����
| G/Out var		:
| Author		: Wang Renfei			Date	: 2017-3-11
\=========================================================================*/
void RunViewTech(uint8 u8UartPort)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	ITEM_PAGE_REQ* pItemPageReq = &pUartComm->ItemPageReq;
	pUartComm->uTmr_Run_ms = 100;

	/* �����������ݴ���״̬����Ϣ�� */
	if(g_CommConf.u32GuiIdleTimeOut == 0) {
		WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_LED_CTL, 0x40, 1);			/* ȷ��������Ļ */
	} else if(ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_REG, VT_REG_TOUCH_STATUS, 1)) {
		if((pUartComm->u8TRBuf[VT_BUF_RDREG] == 1) || (pUartComm->u8TRBuf[VT_BUF_RDREG] == 3)   /* (1���£�2̧��3�����У�������Ч)*/
		    || ((g_Ctr.bBeepAbornmal || g_Ctr.bBeepProgram) && (pItemPageReq->DataPage == DATA_PAGE_MAIN))) /* ��ҳ�汨�� */
		{
			pItemPageReq->u8Tmr_OffScn_1s = g_CommConf.u32GuiIdleTimeOut;
			WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_LED_CTL, 0x40, 1);		/* ȷ��������Ļ */

		/* ��ӭҳ���£�����10�����Ϩ�� */
		} else if((pItemPageReq->DataPage != DATA_PAGE_WELCOME) && (pItemPageReq->u8Tmr_OffScn_1s == 0)) {	/* ���ѳ�ʱ,����ת����ӭҳ */
			pItemPageReq->u8Tmr_OffScn_1s = OFFSCREEN_TIMEOUT;
            pItemPageReq->DataPage = DATA_PAGE_WELCOME;
			WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_LED_CTL, 0x40, 1);		/* ȷ��������Ļ */
		}
	}

	/* ����������ȡ�����밴����ֵ��������Ƿ���Ҫ��ҳ */
	if(ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_VAR, VT_VAR_PNLKEY, 1)) {
		/* PanelKey��� */
		uint16 uPnlKey = g_UartComm[u8UartPort].u8TRBuf[VT_BUF_RDVAR]*0x100 + g_UartComm[u8UartPort].u8TRBuf[VT_BUF_RDVAR+1];	/* ���PnlKey */
		if(uPnlKey) {
			/* ���״̬��ť֮���PNLKEY */
			if((uPnlKey < VT_VAL_PNLKEY_STATUS_BTN_MIN_VAL) || (uPnlKey > VT_VAL_PNLKEY_STATUS_BTN_MAX_VAL)) {
				ClearVar_VT(u8UartPort, VT_DEV_MON, VT_VAR_PNLKEY, 1);
			}
			/* С�� VT_VAL_PNLKEY_MAX_PAGE_VAL ��ҳ���л�ָ���ťIDֵ���Ǵ��л�VTPage */
			if(uPnlKey < VT_VAL_PNLKEY_MAX_PAGE_VAL) {
                /* ��PnlKeyӳ�䵽DataPage */
                DATA_PAGE DataPage = DATA_PAGE_NULL;
                for(; (DataPage < MAX_DATA_PAGE) && (cnst_uDataPage2VTPageBtn[DataPage] != uPnlKey); DataPage++) {
                }
                if(DataPage >= MAX_DATA_PAGE) {
                    DataPage = DATA_PAGE_NULL;
                }
                /* ҳ���л� */
			    if(pItemPageReq->DataPage != DataPage) {
                    /* ��Щҳ����ר�ó�ʼ���������ں�pItemPageReq->DataPage = DataPage */
                    if((DataPage == DATA_PAGE_CONF) || (DataPage == DATA_PAGE_MSG) || (DataPage == DATA_PAGE_ACQ)) {
                        ProcItemPageInstr(DATA_USER_VIEWTECH, DataPage, 0xFFFFFFFFUL, NULL, NULL, pItemPageReq);
                    } else {    /* ͨ�ó�ʼ�� */
                        pItemPageReq->DataPage = DataPage;
                    }
                    pItemPageReq->u8PageRefresh1Hz_100ms = 0;
                }
			}
		}

		/* ���ݵ�ǰҳ�����ʹ���ҳ�����ݺ����� */
		switch(pItemPageReq->DataPage) {
			case DATA_PAGE_NULL:
			case DATA_PAGE_WELCOME:								/* ��ӭҳ�� */
				HandlePageWelcome_VT(u8UartPort);
				break;

			case DATA_PAGE_CONF:								/* ����ҳ�� */
				HandlePageConf_VT(u8UartPort, uPnlKey);
				break;

			case DATA_PAGE_MSG:									/* ��Ϣҳ�� */
			case DATA_PAGE_ACQ:									/* �ɼ�ҳ�� */
				HandlePageMsgAcq_VT(u8UartPort, uPnlKey);
				break;

			case DATA_PAGE_MAIN:								/* ��ҳ�� */
				HandlePageMain_VT(u8UartPort, uPnlKey);
				break;

			case DATA_PAGE_ASSIT:								/* ����ҳ�� */
				HandlePageAssi_VT(u8UartPort);
				break;

			case DATA_PAGE_SYS:
				HandlePageSystem_VT(u8UartPort, uPnlKey);
				break;

			case DATA_PAGE_DEBUG:								/* ����ҳ�� */
				HandlePageDebug_VT(u8UartPort, uPnlKey);
				break;

			default:
				pItemPageReq->DataPage = DATA_PAGE_WELCOME;
		}

		/* ȷ����ǰҳ�� */
		WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_PIC_ID, DataPage2VTPage(pItemPageReq->VTCommVer, pItemPageReq->DataPage), 2);

		/* ���������� */
		if((g_Ctr.bBeepAbornmal || g_Ctr.bBeepProgram) && (!CheckDebugVersion())) {
			WriteReg_VT(u8UartPort, VT_DEV_MON, 0x02, 20, 1);			/* ��0.2s */
		}
	}
	
	/* ������չ�����������ʾ */
#if ((DEVICE_TYPE == V5_YKF3) || (DEVICE_TYPE == V5_YKF5))
	HandleExtScn_VT(u8UartPort);
#endif

	/* ���¼�ʱ�� */
	Task_sleep(OS_TICK_KHz*pUartComm->uTmr_Run_ms);			/* Ϊ��ֹˢ�¹��죬��Ļ��ʱ�Դ��ط�Ӧ������������С��� */
	if(pItemPageReq->u8PageRefresh1Hz_100ms == 0) {		        /* ����1Hz�ź� */
		pItemPageReq->u8PageRefresh1Hz_100ms = 10 - 1;          /* 0~9, ������ 10*100ms = 1s */
		if(pItemPageReq->u8SlowRefresh_1Hz) {                   /* 0~3, ������ 4 �� */
			pItemPageReq->u8SlowRefresh_1Hz--;
		} else {
			pItemPageReq->u8SlowRefresh_1Hz = 3;
		}
		if(pItemPageReq->u8Tmr_OffScn_1s) {
    		pItemPageReq->u8Tmr_OffScn_1s--;
		}
		if(pItemPageReq->u8Tmr_ExtOffScn_1s) {
    		pItemPageReq->u8Tmr_ExtOffScn_1s--;
		}
	} else {
		pItemPageReq->u8PageRefresh1Hz_100ms--;
	}
}

/* ���TxBuf�����͸�VIEWTECH����������ȡ��������
 * ViewTechCmd֧��VIEW_TECH_READ_REG/VIEW_TECH_READ_VAR
 * ��u8ActΪVIEW_TECH_READ_DATAʱ��uAddr��8λ�ᱻ����VIEWTECH��
 * ����ֵ:TRUE���ɹ�ȡ������, FALSE:ûȡ������ */
BOOL ReadData_VT(uint8 u8UartPort, uint16 uDevAddr, VIEW_TECH_CMD ViewTechCmd, uint16 uDataAddr, uint16 uWordLen)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	uint8* pTxBuf = pUartComm->u8TRBuf;
	*pTxBuf++ = uDevAddr/0x100;
	*pTxBuf++ = uDevAddr%0x100;
	pTxBuf++;
	*pTxBuf++ = ViewTechCmd;
	if(ViewTechCmd == VIEW_TECH_READ_VAR) { /*����Ϊ16λ�ֳ�ʱ*/
		*pTxBuf++ = (uint8) (uDataAddr / 0x100);
	}
	*pTxBuf++ = (uint8) (uDataAddr & 0xFF);
	*pTxBuf++ = uWordLen;
	SendData_VT(u8UartPort, pTxBuf);
	if(!ReadFromUart(u8UartPort, 20)) {
	    return FALSE;
	} else {    /* ��鷵�ص����� */
        uint16 uRxBufPt = pUartComm->uRxBufPt;
        uint16 uCRC = (uint16)(pUartComm->u8TRBuf[uRxBufPt - 2])*0x100 + pUartComm->u8TRBuf[uRxBufPt - 1];

        if((pUartComm->u8TRBuf[0]*0x100 + pUartComm->u8TRBuf[1] == uDevAddr)      /* ��ַ */
            /* ���ݳ��ȶ�:��Ҫ��ϸ�µ�У�飬������̨�豸485���ӣ��Զ�ģʽ�»ᶼͣ����VTģʽ */
            && (((pUartComm->u8TRBuf[2] == 2) && (uRxBufPt == 7) && (pUartComm->u8TRBuf[4] == 0xFF))   /* CRCУ����Ӧ�� */
                || ((ViewTechCmd == VIEW_TECH_READ_REG) && (uRxBufPt == uWordLen + 8))
                || ((ViewTechCmd == VIEW_TECH_READ_VAR) && (uRxBufPt == uWordLen*2 + 9)))
            && (CalCRC16ForViewTech(pUartComm->u8TRBuf + 3, uRxBufPt - 5) == uCRC))     /* CRC */
        {
            pUartComm->uTimer_WatchDog_ms = 0;  /* ͨѶ�ɹ� */
            pUartComm->bUartFuncTrySuc = TRUE;
            return TRUE;
        } else {
            return FALSE;
        }
	}
}

/* ����1��2�ֽڶ�ָ�VIEWTECH��Ļ�����Ĵ������򣬲��������ء�
 * ���TxBuf�����͸�VIEWTECH����������ȡ��������,
 * u8Len��1��2 */
void WriteReg_VT(uint8 u8UartPort, uint16 uDevAddr, uint16 uDataAddr, uint16 uValue, uint8 u8ByteLen)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	uint8* pTxBuf = pUartComm->u8TRBuf;
	*pTxBuf++ = uDevAddr/0x100;
	*pTxBuf++ = uDevAddr%0x100;
	pTxBuf++;
	*pTxBuf++ = VIEW_TECH_WRITE_REG;
	*pTxBuf++ = (uint8)(uDataAddr & 0xFF);
	if(u8ByteLen > 1) {
		*pTxBuf++ = uValue/0x100;
	}
	*pTxBuf++ = uValue & 0xFF;
	SendData_VT(u8UartPort, pTxBuf);
}
/* ����ָ�VIEWTECH��Ļ��չ�Ĵ�������֧���255�ֽڵ�д�룬���������ء�
 * ���TxBuf�����͸�VIEWTECH����������ȡ��������
 * */
void WriteVar_VT(uint8 u8UartPort, uint16 uDevAddr, uint16 uDataAddr, uint8 *pValue, uint8 u8WordLen)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	uint8* pTxBuf = pUartComm->u8TRBuf;
	*pTxBuf++ = uDevAddr/0x100;
	*pTxBuf++ = uDevAddr%0x100;
	pTxBuf++;
	*pTxBuf++ = VIEW_TECH_WRITE_VAR;
	*pTxBuf++ = uDataAddr/0x100;
	*pTxBuf++ = uDataAddr%0x100;;
	for( ; u8WordLen != 0; u8WordLen--) {
		*pTxBuf++ = *pValue++;
		*pTxBuf++ = *pValue++;
	}
	SendData_VT(u8UartPort, pTxBuf);
}

/* ��λ��尴ť */
void ClearVar_VT(uint8 u8UartPort, uint16 uDevAddr, uint16 uDataAddr, uint8 u8WordLenToClear)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	uint8* pTxBuf = pUartComm->u8TRBuf;
	*pTxBuf++ = uDevAddr/0x100;
	*pTxBuf++ = uDevAddr%0x100;
	pTxBuf++;
	*pTxBuf++ = VIEW_TECH_WRITE_VAR;
	*pTxBuf++ = uDataAddr/0x100;
	*pTxBuf++ = uDataAddr&0xFF;
	while(u8WordLenToClear--) {
		*pTxBuf++ = 0;
		*pTxBuf++ = 0;
	}
	SendData_VT(u8UartPort, pTxBuf);
}

uint16 DataPage2VTPage(VT_COMM_VER VTCommVer, DATA_PAGE DataPage)
{
    if(VTCommVer < VT_COMM_VER_MERGE) {
        return cnst_uDataPage2VTPage[0][DataPage];
    } else {
        return cnst_uDataPage2VTPage[1][DataPage];
    }
}

/* ����Viewtech��ӭҳ������Ӧ */
void HandlePageWelcome_VT(uint8 u8UartPort)
{
    /* ������������л�����ҳ�� */
	if(g_Ctr.bBeepAbornmal || g_Ctr.bBeepProgram) {
        g_UartComm[u8UartPort].ItemPageReq.DataPage = DATA_PAGE_MAIN;

    /* ��ȡ��Ļ���� */
	} else if(ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_VAR, VT_VAR_DEVICE_TYPE, 1)) {
		g_UartComm[u8UartPort].ItemPageReq.VTCommVer = (VT_COMM_VER)g_UartComm[u8UartPort].u8TRBuf[VT_BUF_RDVAR+1];
		if((g_UartComm[u8UartPort].ItemPageReq.u8Tmr_OffScn_1s == 0) && g_CommConf.u32GuiIdleTimeOut) {
			WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_LED_CTL, 0, 1);	/* Ϣ�� */
		}
	}
}

/*==========================================================================
| Description	: ��������ҳ��������Ӧ
	��ҳ����ʾ����Լ��8���ı� ���� 6���ı�+1�б���(����ռ����)��ҳ�����ݣ���0��Ϊ���⣬��1~9��Ϊ�ı���һ������Ϊ60Byte
	���ڵ������ݳ����Ϊ255-8 = 247�����һ�������ʾһ�룬����ҳ�������Ҫ�ֳ�2~3�����ڴ���
| G/Out var		:
| Author		: Wang Renfei			Date	: 2017-3-11
\=========================================================================*/
void HandlePageConf_VT(uint8 u8UartPort, uint16 uPnlKey)
{
	ITEM_PAGE_REQ *pPageReq = &g_UartComm[u8UartPort].ItemPageReq;
	uint8* pTRBuf = g_UartComm[u8UartPort].u8TRBuf;
	int16 iLineNo = -1;
	uint8 u8BytesToWrite[12];

	uint8 u8ItemCharLen = VT_LEN_ITEM_CHAR;
	if(pPageReq->VTCommVer == VT_VAL_DEV_SDW043) {
		u8ItemCharLen = 46;
	}

	/* ��¼δ��������� */
	if(ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_REG, VT_REG_INPUT_STATUS, 1) && pTRBuf[VT_BUF_RDREG]) {	/* ��������0���������� */
		pPageReq->bUnprocessedInput = TRUE;
	} else if(pPageReq->bUnprocessedInput) {
		/* �����¼�����״̬��������ǰ������ */
		if(ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_VAR, VT_VAR_KEYBOARD_STATUS, 2)
			&& (pTRBuf[VT_BUF_RDVAR] == 0x5A))			/*��һ�ֽ�������״̬ */
		{
			uint8 u8InputByteLen = pTRBuf[VT_BUF_RDVAR+1];	 /*�ڶ��ֽ������볤��*/
			/*��ȡ�ڼĴ���VIEWADD_KEYBOARD_START����������*/
			if(u8InputByteLen && ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_VAR, VT_VAR_KEYBOARD_INPUT, u8InputByteLen/2 + 1)) {
				uint8* pU8Instr = NULL;
				if (pPageReq->u8InputingType == VT_VAL_CONFIG_INPT_ITM_SECOND) { /*������˼��������⡢��һ��ֵ���������� ������Ե�һ��ֵ���޸�*/
					pTRBuf[6] = 0; /*����ǰһλд0����ʶҪ�޸ĵڶ���ֵ*/
					pU8Instr = pTRBuf + 6;
					u8InputByteLen++;
				} else {
					pU8Instr = pTRBuf + 7;
				}
				*(pU8Instr + u8InputByteLen) = 0; /*�ַ���ĩβ��0xFF�滻�ɱ�׼��0*/
				ProcItemPageInstr(DATA_USER_VIEWTECH, pPageReq->DataPage, pPageReq->u32ItemNo, &pU8Instr, pU8Instr + u8InputByteLen, pPageReq);
			}
			/*ȡ������*/
			memset(u8BytesToWrite, 0, 12);
			WriteVar_VT(u8UartPort, VT_DEV_MON, VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE1, u8BytesToWrite, 6); /* ����ͼƬ��0�رգ�1��ʾ   */
		}
		pPageReq->bUnprocessedInput = FALSE;
	} else {	/* ���û�н����еļ������� */
		/* ��Ļ��ť���� */
		uint16 uItemNo;
		CONF_n_STAT_PROP* pDataAcsProp;
		ITEM_DATA_TYPE ItemUIType;
		switch(uPnlKey) {
			case VT_VAL_CONFIG_PNLKEY_CLICK_BACK: /* ���ذ��� */
				if(pPageReq->uConfPageTopicItemNo > 0) {	/* ����ҳ����Ҫִ�з��� */
					DealGuiConfWinBack(DATA_USER_VIEWTECH, pPageReq);
				} else {
					/* ���ظ���ҳ�� */
					pPageReq->DataPage = DATA_PAGE_ASSIT;
					return;
				}
				break;

			case VT_VAL_CONFIG_PNLKEY_CLICK_UP: /* ���ϰ��� */
				DealGuiConfWinUp(DATA_USER_VIEWTECH, pPageReq);
				break;

			case VT_VAL_CONFIG_PNLKEY_CLICK_DOWN: /* ���°��� */
				DealGuiConfWinDown(DATA_USER_VIEWTECH, pPageReq);
				break;

			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE1_NON_2ND: /* ��ʾ��Ŀ��һ�е�һλ����¼�   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE1_2ND: /* ��ʾ��Ŀ��һ�еڶ�λ����¼�   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE2_NON_2ND: /* ��ʾ��Ŀ�ڶ��е�һλ����¼�   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE2_2ND: /* ��ʾ��Ŀ�ڶ��еڶ�λ����¼�   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE3_NON_2ND: /* ��ʾ��Ŀ�����е�һλ����¼�   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE3_2ND: /* ��ʾ��Ŀ�����еڶ�λ����¼�   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE4_NON_2ND: /* ��ʾ��Ŀ�����е�һλ����¼�   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE4_2ND: /* ��ʾ��Ŀ�����еڶ�λ����¼�   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE5_NON_2ND: /* ��ʾ��Ŀ�����е�һλ����¼�   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE5_2ND: /* ��ʾ��Ŀ�����еڶ�λ����¼�   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE6_NON_2ND: /* ��ʾ��Ŀ�����е�һλ����¼�   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE6_2ND: /* ��ʾ��Ŀ�����еڶ�λ����¼�   */
				iLineNo = ((uPnlKey - VT_VAL_CONFIG_PNLKEY_CLICK_LINE1_2ND) >> 4); /* start from 0*/
				uItemNo = pPageReq->uConfWinFirstLineItemNo + iLineNo;
				pDataAcsProp = pBootLoaderConf->pConfProp + uItemNo;
				ItemUIType = GetItemUIType(pDataAcsProp);
				if(ItemUIType == ITEM_UI_CLICK) {
					uint8* pU8Instr = (uint8*)"click";
					ProcItemPageInstr(DATA_USER_VIEWTECH, pPageReq->DataPage, uItemNo, &pU8Instr, pU8Instr + 6, pPageReq);
				} else if((ItemUIType == ITEM_UI_ONEBOX)
						|| (ItemUIType == ITEM_UI_TWOBOX)
						|| (ItemUIType == ITEM_UI_ONEDBOX)
						|| (ItemUIType == ITEM_UI_ANSI)			/* ���ʻ��޸ģ�����ANSI�����޷��༭ */
						|| (ItemUIType == ITEM_UI_LOGIN)) 
				{
					/* �������� */
					u8BytesToWrite[0] = 0;
					u8BytesToWrite[1] = 1;
					WriteVar_VT(u8UartPort, VT_DEV_MON, VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE1 + iLineNo, u8BytesToWrite, 1); /* ����ͼƬ��0�رգ�1��ʾ   */

					/*�ж�����¼�뻹��ascii¼��*/
					switch((pBootLoaderConf->pConfProp + uItemNo)->ItemType.DataType) {
						case ITEM_DAT_ANSI:
						case ITEM_DAT_RMTS:
							pPageReq->u8InputingType = VT_VAL_CONFIG_INPT_ITM_ASCII;
							WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_KEYBOARD_CTL, VT_VAL_KEYBOARD_ASCII, 1);
							break;

						default:
							if(ItemUIType == ITEM_UI_LOGIN) {		/*�����������  */
								pPageReq->u8InputingType = VT_VAL_CONFIG_INPT_ITM_PASSWORD;
								WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_KEYBOARD_CTL, VT_VAL_KEYBOARD_PWD, 1);
							} else { 								/*�������ּ���  */
								if(ItemUIType == ITEM_UI_TWOBOX && (uPnlKey & 3)==0) {
									pPageReq->u8InputingType = VT_VAL_CONFIG_INPT_ITM_SECOND;
								} else {
									pPageReq->u8InputingType = VT_VAL_CONFIG_INPT_ITM_FIRST;
								}
								WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_KEYBOARD_CTL, VT_VAL_KEYBOARD_NUM, 1);
							}
					}
					/*��������״̬*/
					pPageReq->u32ItemNo = uItemNo;
				}
				break;

			default:
				;
		}
	}

	/* ��Ҫ����ˢ�µ� */
	if(pPageReq->u8PageRefresh1Hz_100ms == 0) {
		pTRBuf = g_UartComm[u8UartPort].u8TRBuf;
    	*pTRBuf++ = VT_DEV_MON/0x100;
    	*pTRBuf++ = VT_DEV_MON%0x100;
		pTRBuf++;
		*pTRBuf++ = VIEW_TECH_WRITE_VAR;

		/* ��һ֡ */
		*pTRBuf++ = VT_VAR_CONFIG_UNIT_TITLE/0x100;
		*pTRBuf++ = VT_VAR_CONFIG_UNIT_TITLE&0xFF;
		CONF_DAT_FRAME1 *pConfFrame1 = (CONF_DAT_FRAME1*)(pTRBuf);
		InitDataWithZero(pTRBuf, sizeof(CONF_DAT_FRAME1));
		/* ������ */
		CONF_n_STAT_PROP* pTopicAcsProp = pBootLoaderConf->pConfProp + pPageReq->uConfPageTopicItemNo;
		pTRBuf = PrintLocalConfItem(DATA_USER_VIEWTECH, pTopicAcsProp, pConfFrame1->u8Title, u8ItemCharLen*0.72);	/*0.72�Ǳ����������������С��������*/
		/* ��1~3�� */
		uint16 uCurLineItemNo = pPageReq->uConfWinFirstLineItemNo;
		uint16 uCurPageMaxItemNo = pTopicAcsProp->LkId_Val_ItN + pPageReq->uConfPageTopicItemNo;
		for(iLineNo = 0; iLineNo < 3; iLineNo++) {
			if(uCurLineItemNo <= uCurPageMaxItemNo) {
				pTRBuf = PrintLocalConfItem(DATA_USER_VIEWTECH, pBootLoaderConf->pConfProp + uCurLineItemNo, pConfFrame1->u8TextLines[iLineNo], u8ItemCharLen);
			}
			uCurLineItemNo++;
		}
		SendData_VT(u8UartPort, (uint8*)(pConfFrame1 + 1));	/* ���ͣ�����+1��3��һ֡�� */

		/* �ڶ�֡ */
		pTRBuf = &g_UartComm[u8UartPort].u8TRBuf[4];
		*pTRBuf++ = VT_VAR_CONFIG_UNIT_LINE4/0x100;
		*pTRBuf++ = VT_VAR_CONFIG_UNIT_LINE4&0xFF;
		CONF_DAT_FRAME2 *pConfFrame2 = (CONF_DAT_FRAME2*)(pTRBuf);
		InitDataWithZero(pTRBuf, sizeof(CONF_DAT_FRAME2));
		/* ��4~6�� */
		uCurLineItemNo = pPageReq->uConfWinFirstLineItemNo + 3;
		for(iLineNo = 0; iLineNo < 3; iLineNo++) {
			if(uCurLineItemNo <= uCurPageMaxItemNo) {
				pTRBuf = PrintLocalConfItem(DATA_USER_VIEWTECH, pBootLoaderConf->pConfProp + uCurLineItemNo, pConfFrame2->u8TextLines[iLineNo], u8ItemCharLen);
			}
			uCurLineItemNo++;
		}

		/* ������б༭ͼƬ��ʽ��0����1���༭��2˫�༭��	2�ֽ�   */
		uCurLineItemNo = pPageReq->uConfWinFirstLineItemNo;
		for(iLineNo = 0; iLineNo < 6; iLineNo++) {
			switch(GetItemUIType(pBootLoaderConf->pConfProp + uCurLineItemNo)) {
				case ITEM_UI_ONEDBOX:
				case ITEM_UI_ANSI:
					pConfFrame2->uEditerStyleLines[iLineNo] = 0x03<<8;
					break;

				case ITEM_UI_TWOBOX:
					pConfFrame2->uEditerStyleLines[iLineNo] = 0x02<<8;
					break;
				case ITEM_UI_ONEBOX:
					pConfFrame2->uEditerStyleLines[iLineNo] = 0x01<<8;
					break;
				case ITEM_UI_TOPIC:
				case ITEM_UI_TEXT:
				case ITEM_UI_CLICK:
				case ITEM_UI_LOGIN:
				default:
					pConfFrame2->uEditerStyleLines[iLineNo] = 0x00<<8;
			}
			uCurLineItemNo++;
		}
		/* ���ҳ�� 65535/65535	12�ֽ�   */
		pTRBuf = pConfFrame2->u8UNIT_PAGER;
		PrintU32WithLen(&pTRBuf, pPageReq->uConfWinFirstLineItemNo - pPageReq->uConfPageTopicItemNo, 5);
		*pTRBuf++ = '/';
		PrintU32WithLen(&pTRBuf, pTopicAcsProp->LkId_Val_ItN, 5);
		SendData_VT(u8UartPort, (uint8*)(pConfFrame2 + 1));	/* 4��6��+ʣ�µ�����һ֡ */
	}
}

/*������Ϣ\�ɼ�ҳ��������Ӧ*/
void HandlePageMsgAcq_VT(uint8 u8UartPort, uint16 uPnlKey)
{
	ITEM_PAGE_REQ *pPageReq = &g_UartComm[u8UartPort].ItemPageReq;
	BOOL bPnlWinUpOrWinDown = FALSE;

	/* ��Ļ��ť���� */
	switch(uPnlKey) {
		case VT_VAL_MSGACQ_PNLKEY_CLICK_UP:		/* �������ϼ�ͷ */
			if(pPageReq->VTCommVer == VT_VAL_DEV_SDW043) {
				pPageReq->u32ItemNo += 6;
			} else {
				pPageReq->u32ItemNo += 16;
			}
			bPnlWinUpOrWinDown = TRUE;
			break;

		case  VT_VAL_MSGACQ_PNLKEY_CLICK_DOWN:	/* �������¼�ͷ */
			if(pPageReq->u32ItemNo > 1) {
				bPnlWinUpOrWinDown = TRUE;
			}
			break;

		default:;
	}
	
	/* ��Ҫˢ��ҳ��ģ�ҳ���л���ʱ��pPageReq->u32ItemNo�ᱻ��ʼ���� 0xFFFFFFFFUL */
	if((pPageReq->u32ItemNo == 0xFFFFFFFFUL) || bPnlWinUpOrWinDown) {
		uint16 uMsgAcqID[3];
		int8 i;
		uint8* pTxBufEnd = &g_UartComm[u8UartPort].u8TRBuf[UART_TR_BUF_BLEN] - 2;	/* ��Ҫ�ճ�CRC */
		uint8* pTRBuf = g_UartComm[u8UartPort].u8TRBuf;		
    	*pTRBuf++ = VT_DEV_MON/0x100;
    	*pTRBuf++ = VT_DEV_MON%0x100;
		pTRBuf++;
		*pTRBuf++ = VIEW_TECH_WRITE_VAR;

		/* ��һ֡ */
		*pTRBuf++ = VT_VAR_MSGACQ_UNIT_LINE1/0x100;
		*pTRBuf++ = VT_VAR_MSGACQ_UNIT_LINE1&0xFF;
		int32 i32ReadItemNum = -3;
		GetTextForMsgOrAcq(DATA_USER_VIEWTECH, pPageReq->DataPage, &pPageReq->u32ItemNo, &i32ReadItemNum, &pTRBuf, pTxBufEnd);
		SendData_VT(u8UartPort, pTRBuf);	/* ���� */

		/* ����ǰ3��������ɫ */
		for(i = 0; i < 3; i++) {			/* ��¼MsgAcqID */
			uMsgAcqID[i] = *((uint16*)(&g_UartComm[u8UartPort].u8TRBuf[6 + VT_LEN_ITEM_BYTE*(i+1) - 2]));
		}
		if(pPageReq->VTCommVer == VT_VAL_DEV_SDW043) {
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE1, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[0]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE2, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[0]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE3, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[1]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE4, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[1]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE5, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[2]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE6, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[2]));
		} else {		/* ��4.3������ӡ���ڶ�֡�͵���֡����ʣ���5����Ϣ */
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE1, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[0]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE2, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[1]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE3, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[2]));
			/* �ڶ�֡ */
			pTRBuf = &g_UartComm[u8UartPort].u8TRBuf[4];
			*pTRBuf++ = VT_VAR_MSGACQ_UNIT_LINE4/0x100;
			*pTRBuf++ = VT_VAR_MSGACQ_UNIT_LINE4&0xFF;
			i32ReadItemNum = -3;
			GetTextForMsgOrAcq(DATA_USER_VIEWTECH, pPageReq->DataPage, &pPageReq->u32ItemNo, &i32ReadItemNum, &pTRBuf, pTxBufEnd);
			SendData_VT(u8UartPort, pTRBuf);	/* ���� */

			/* ������3��������ɫ */
			for(i = 0; i < 3; i++) {			/* ��¼MsgAcqID */
				uMsgAcqID[i] = *((uint16*)(&g_UartComm[u8UartPort].u8TRBuf[6 + VT_LEN_ITEM_BYTE*(i+1) - 2]));
			}
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE4, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[0]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE5, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[1]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE6, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[2]));

			/* ����֡ */
			pTRBuf = &g_UartComm[u8UartPort].u8TRBuf[4];
			*pTRBuf++ = VT_VAR_MSGACQ_UNIT_LINE7/0x100;
			*pTRBuf++ = VT_VAR_MSGACQ_UNIT_LINE7&0xFF;
			i32ReadItemNum = -2;
			GetTextForMsgOrAcq(DATA_USER_VIEWTECH, pPageReq->DataPage, &pPageReq->u32ItemNo, &i32ReadItemNum, &pTRBuf, pTxBufEnd);
			SendData_VT(u8UartPort, pTRBuf);	/* ���� */
			/* ���ú�2��������ɫ */
			for(i = 0; i < 2; i++) {			/* ��¼MsgAcqID */
				uMsgAcqID[i] = *((uint16*)(&g_UartComm[u8UartPort].u8TRBuf[6 + VT_LEN_ITEM_BYTE*(i+1) - 2]));
			}
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE7, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[0]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE8, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[1]));
		}
	}
	/* �����԰�ť�Ƿ���� */
	uint16 uDbgEnb = g_Ctr.bEngDebug;		/* ��չ��16λ */
	WriteVar_VT(u8UartPort, VT_DEV_MON, VT_VAR_MSGACQ_DEBUG_BTN_ENABLE, (uint8*)&uDbgEnb, 1);
}

/*==========================================================================
| Description	: �Ѵ����ͻ�����������, ���ϳ��ȱ�־��CRC, ���ͳ�ȥ
| G/Out var		:
| Author		: Wang Renfei			Date	: 2017-3-11
\=========================================================================*/
void SendData_VT(uint8 u8UartPort, uint8* pTxBuf)
{
	uint8* pU8TRBufStart = g_UartComm[u8UartPort].u8TRBuf;	/* ��ָ�뷽ʽ���� */

	/* ��д�������ݳ�����CRC */
	uint32 u32NeedTxByte = pTxBuf - pU8TRBufStart;
	if(u32NeedTxByte > 3) {
		uint16 uCRC = CalCRC16ForViewTech(pU8TRBufStart + 3, u32NeedTxByte - 3);
		*pTxBuf++ = uCRC/0x100;
		*pTxBuf++ = uCRC&0xFF;
		u32NeedTxByte += 2;
		pU8TRBufStart[2] = u32NeedTxByte - 3;
	} else {
		pU8TRBufStart[2] = 0;
	}
	WriteToUart(u8UartPort, u32NeedTxByte);	/* �������� */
}

/* �������256���ַ��������������ƣ�strcpy���ڴ�*/
uint8 PrintStringFromText_VT(uint8 *pDstBuf, const char *pSrcText, uint8 u8MaxLen)
{
    uint8 u8Count = 0;
    for(; u8Count < u8MaxLen; u8Count++) {
        if(*pSrcText == 0) {
			*pDstBuf = 0;
            return u8Count + 1;
        } else {
            *pDstBuf++ = *pSrcText++;
        }
    }
    return u8MaxLen;
}

#if TEMP_SEN_PLC_NUM
/* ��ӡ�¶�����,�ɹ��򷵻�������λ��+1,����¶�������TEMP_SEN_NUL,��ʲô������ӡ,ֱ�ӷ���pText */
uint8* PrintTemperatureName(uint8* pText, TEMPERATURE_DATA* pTempData, char TailChar)
{
	int16 i8Placement = pTempData->i8Placement;
	/* �����������ж� */
	if(i8Placement != TEMP_SEN_NUL) {
		if(i8Placement < 0) {
			i8Placement = 0 - i8Placement;
		}
		if(i8Placement >= TEMP_SEN_PLC_NUM) {	/* ��ֹ����������� */
			i8Placement = TEMP_SEN_PLC_ANY;
		}

		/* ��ʾ������ */
		const char* pVarName = cnst_TempPlacementName[i8Placement][(LANG_TYPE)g_Sys.u32LocalLang];
		while(*pVarName) {
			*pText++ = *pVarName++;
		}
		uint8 u8PlaceNo = pTempData->u8PlaceNo;
		if(u8PlaceNo) {
			if(u8PlaceNo >= 10) {
				*pText++ = '0' + u8PlaceNo/10;
				u8PlaceNo = u8PlaceNo%10;
			}
			*pText++ = '0' + u8PlaceNo;
		}
	}
    *pText++ = (uint8)TailChar;
	return pText;
}

/* ��ӡ�¶���ֵ,�ɹ��򷵻�������λ��+1,����¶�������TEMP_SEN_NUL,��ʲô������ӡ,ֱ�ӷ���pText*/
uint8* PrintTemperatureValue(uint8* pText, TEMPERATURE_DATA* pTempData, const char* pDiscText)
{
	int16 i8Placement = pTempData->i8Placement;
	if(i8Placement != TEMP_SEN_NUL) {
		/* �¶���ֵ���� */
		if((i8Placement < 0)) { 	/* �紫�����������ӡpDiscText���� */
			while(*pDiscText) {
				*pText++ = *pDiscText++;
			}
		} else {
			PrintI16(&pText, pText+10, pTempData->iDegree_100mC, 1);
		}
	}
	*pText++ = 0;
	return pText;
}
#endif
/*==========================================================================
| MCGS��ĻItemPage���֣����� ͳ��&����&�¼���Ϣ&�ɼ�����
\=========================================================================*/
/*==========================================================================
| ���º���ΪMCGS��Ļ����ItemPage��������Ҫ��
| ������ʾ: ����ҳ�桢ͳ��ҳ��Ⱦ�����/��Ŀ������ҳ��
| ���ڱ���:
\=========================================================================*/
/*==========================================================================
| Description	: ���ItemPage����Ӧ��
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2015-12-26
\=========================================================================*/
void FillItemPageAck_MCGS(ITEM_PAGE_REQ* pItemPageReq, uint8* pMCGSItemPageAck)
{
	#define MCGS_TOPIC_TEXT_BLEN	30
	CONF_n_STAT_PROP* pDataAcsProp;
	uint8* pTextStart = pMCGSItemPageAck + MCGS_ITEM_PAGE_PAYLOAD_BSTART;	/* ��Ҫ��ǰ��ճ�ItemType */
	uint32 u32ItemNo = pItemPageReq->u32ItemNo;
	ITEM_DATA_TYPE ItemDataType;

	/* ��ͬ���͵�ҳ�洦�� */
	if((pItemPageReq->DataPage == DATA_PAGE_MSG) || (pItemPageReq->DataPage == DATA_PAGE_ACQ)) {
		uint8* pText = pTextStart + MCGS_ITEM_PAGE_ADDI_BLEN;		/* ��Ҫ�ճ������Ŀռ䣬MCGS��Ļ���ֱ���16λ���� */
		int32 i32ItemNum_InNeed_OutSuc;

		if((u32ItemNo == 0xFFFFFFFFUL) || pItemPageReq->bMsgAcqPageWinDown) {	/* ��λ �� ����� */
			i32ItemNum_InNeed_OutSuc = -1;
		} else {											/* ����� */
			i32ItemNum_InNeed_OutSuc = 1;
		}
		GetTextForMsgOrAcq(DATA_USER_MCGS, pItemPageReq->DataPage, &u32ItemNo, &i32ItemNum_InNeed_OutSuc, &pText, pText + MCGS_LEN_ITEM_CHAR);
		LeftAlignString_MCGS(pTextStart, MCGS_LEN_ITEM_CHAR);
		pItemPageReq->u32ItemNo = u32ItemNo;
		pItemPageReq->bMsgAcqPageWinDown = TRUE;
		ItemDataType = ITEM_UI_TEXT;

	} else if(pItemPageReq->DataPage == DATA_PAGE_CONF) {
		BOOL bBlankLine = FALSE;
			/* ��ǰ��ʾItemNo��Χ��飺�����˵�ǰ��ʾ���� */
		if((u32ItemNo < pItemPageReq->uConfWinFirstLineItemNo) || (u32ItemNo > pItemPageReq->uConfWinFirstLineItemNo + 5)) {
			u32ItemNo = pItemPageReq->uConfPageTopicItemNo;
		}	/* ��ǰ��ʾItemNo��Χ��飺�����˵�ǰҳ�����һ��(ҳ��С�ڴ���),��ʾΪ�հ��� */
		else if(u32ItemNo > pItemPageReq->uConfPageTopicItemNo + pBootLoaderConf->pConfProp[pItemPageReq->uConfPageTopicItemNo].LkId_Val_ItN) {
			bBlankLine = TRUE;
		}

		/* ��ӡ��ǰ�� */
		if(bBlankLine) {	/* �����˵�ǰҳ�棬��ʾΪ�հ��� */
			memset((void*)pTextStart, ' ', MCGS_LEN_ITEM_CHAR);
			ItemDataType = ITEM_UI_TEXT;
		} else {
			pDataAcsProp = pBootLoaderConf->pConfProp + u32ItemNo;
			PrintLocalConfItem(DATA_USER_MCGS, pDataAcsProp, pTextStart, MCGS_LEN_ITEM_CHAR);
			ItemDataType = GetItemUIType(pDataAcsProp);
		}
		
		/* ��һ��׼�� */
		if(u32ItemNo == pItemPageReq->uConfPageTopicItemNo) {
			pItemPageReq->u32ItemNo = pItemPageReq->uConfWinFirstLineItemNo;
		} else {
			pItemPageReq->u32ItemNo = u32ItemNo + 1;
		}
	} else {
		InitDataWithZero(pMCGSItemPageAck, MCGS_ITEM_PAGE_LINE_WLEN*2);
	}

	/* ��ɴ��� */
	pMCGSItemPageAck[MCGS_ITEM_PAGE_ITEMNO_BSTART + 0] = u32ItemNo/0x1000000UL;			/* �����ߵ��ֽ�˳�� */
	pMCGSItemPageAck[MCGS_ITEM_PAGE_ITEMNO_BSTART + 1] = (u32ItemNo/0x10000UL)%0x100UL;	/* �����ߵ��ֽ�˳�� */
	pMCGSItemPageAck[MCGS_ITEM_PAGE_ITEMNO_BSTART + 2] = (u32ItemNo/0x100UL)%0x100UL;	/* �����ߵ��ֽ�˳�� */
	pMCGSItemPageAck[MCGS_ITEM_PAGE_ITEMNO_BSTART + 3] = u32ItemNo%0x100UL;				/* �����ߵ��ֽ�˳�� */
	pMCGSItemPageAck[MCGS_ITEM_PAGE_TYPE_BSTART + 0] = 0;
	pMCGSItemPageAck[MCGS_ITEM_PAGE_TYPE_BSTART + 1] = ItemDataType;
}

/*==========================================================================
| Description	: �����ݵ����ɺ���MCGS��ItemPage�ĸ�ʽ--����mcgs��Ļ��������������ޣ���˴����������Ҫ��������Լ��:
			1. ������Ҫ��һ��Modbus/RTU��16bit����������ܷ�ɢ������
			2. ����ĵط�����ո�
			3. ������Ҫ���������ռ䣬�����Ҫ�����ITEMPAGE_ADDI_BLEN�����ڴ�������������(����һ��ʼ���ǿ�����õ�)
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2017-07-22
\=========================================================================*/
void LeftAlignString_MCGS(uint8* pDest, uint32 u32TextBufBLen)
{
	uint8* pSrc;
	int32 i;

	pSrc = pDest + MCGS_ITEM_PAGE_ADDI_BLEN;
	for(i = 0; (i < u32TextBufBLen) && (*pSrc); i++) {
		if((i & 1) && (pDest[i-1] <= 0x7F) && (*pSrc > 0x7F)) {
			pDest[i] = 0;
		} else {
			pDest[i] = *pSrc;
			pSrc++;
		}
	}
	for( ; i < u32TextBufBLen; i++) {
		pDest[i] = ' ';
	}
}
#endif

/*==========================================================================
| Description	: ��ʵ���������õ�����(ITEM_DATA_TYPE)ת��ΪUI��ʾ����Ҫ������(ITEM_UI_TYPE, ��ITEM_DATA_TYPE��һ���Ӽ�)
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2020-02-19
\=========================================================================*/
ITEM_DATA_TYPE GetItemUIType(CONF_n_STAT_PROP* pDataAcsProp)
{
	if(pDataAcsProp->ItemType.DataType == ITEM_DAT_LINK) {
		if((pDataAcsProp->pData == NULL) || (*((uint32*)pDataAcsProp->pData) == 0) || g_Ctr.bEngDebug) {
			return ITEM_UI_CLICK;
		} else {
			return ITEM_UI_LOGIN;
		}
	} else {
		return (ITEM_DATA_TYPE)(pDataAcsProp->ItemType.DataType>>4);
	}
}

/*==========================================================================
| Description	: ���º���Ϊmqtt����ҳ��ӿڣ�����Ϊ CONF_n_STAT_PROP ��ʽ
��Ϊ���ֲ��������á�ͬ��
����ҳ�棺���ڲ�������ҳ�棬���������Ա�Ķ�����һ�ν�����һ������ҳ��
����ͬ�������ڲ���ͬ�����ݣ��ɶ��Բ��Ҫ�������õı���/�ָ���������һ�η���ȫ����������

ҳ��ӿں���:
	ProcMqttConfInstr()	: �����ⲿ����ָ����� ProcItemPageInstr()��������
	ProcMqttConfSync()	: �����ⲿͬ��ָ��
	QueryAndPubMqttConfResAndPage()	: ��ѯ������Mqttҳ�洦��ָ����
	
���ýӿ�
	ProcItemPageInstr()	: �����ⲿд�ӿ�
	PrintMqttConfPage()	: ��ӡMqtt����ҳ��
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2017-12-25
\=========================================================================*/
void ProcMqttConfInstr(uint8* pU8Msg, uint8* pU8MsgEnd)
{
	if(g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP == 0) {	/* ����ӿڿ� */
		uint32 u32ItemNo = 0;
		pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 3); 		/* ������no��ֵλ�� */
		if(CompareMsg(pU8Msg, "ini")) { 							/* idΪini����λ��������ҳ�� */
			u32ItemNo = 0xFFFFFFFFUL;								/* 0xFFFFFFFFUL�����ʼ��ҳ�� */
		} else {
			u32ItemNo = ReadU32(&pU8Msg, pU8MsgEnd);				/* ���ItemNo, �ú������Զ�����ǰ��ķ������ַ� */
		}
		pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 4); 		/* ��ָ��Ĳ����� */
		g_MqttItemProc.uPageOpItemNo = u32ItemNo;
		g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP
			= ProcItemPageInstr(DATA_USER_NET, DATA_PAGE_CONF, u32ItemNo, &pU8Msg, pU8MsgEnd, &g_MqttItemProc.ItemPageReq);
		/* ����������ȴ�,������֪ͨpub���񷢲���� */
		if(g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP <= 0) {
			Semaphore_post(SEM_MqttPubReq);
		}
	}
}

void ProcMqttConfSync(uint8* pU8Msg, uint8* pU8MsgEnd)
{
	if(g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP == 0) {	/* ����ӿڿ� */
		/* ��ʼ�� */
		g_MqttItemProc.uStartConfID = 0;
		g_MqttItemProc.uEndConfID = 0;

		/* ��ʼ���� */
		pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 1);		/* ָ���һ���ֶ� */
		if(CompareInstr(&pU8Msg, pU8MsgEnd, "instr\":\"backup")) {	/* �����󱸷����ݵ�ָ�� */
			g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = -1;
			Semaphore_post(SEM_MqttPubReq);		//Semaphore_post(SEM_MqttPubReq);
			return;
		} else if((!CompareMsg(pU8Msg, "device_type"))				/* �豸�ͺż�� */
				|| ((pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 2)) >= pU8MsgEnd)
				|| (!CompareMsg(pU8Msg, DEVICE_TYPE_String)))
		{
			g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
				Semaphore_post(SEM_MqttPubReq);	//Semaphore_post(SEM_MqttPubReq);
			return;
		} else {													/* �������ûָ�ָ�� */
			pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '{', 1);	/* �����ͺš��汾�����к��ֶ� */
			uint16 uConfID = ReadU32(&pU8Msg, pU8MsgEnd);
			g_MqttItemProc.uStartConfID = uConfID;	                /* Ϊ�˻�ȡ��ʼID����˴��� */
			while(uConfID && (pU8Msg < pU8MsgEnd)) {
				if(pU8Msg[1] != ':') {	/* ��Ϣ��ʽ���� */
					g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
					return;
				}
				
				/* ����ConfID����ItemNo */
				CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
				uint16 uItemNo;
				for(uItemNo = 0; (uItemNo < pBootLoaderConf->u32ItemNum) && (pDataAcsProp->u32ItemID != uConfID); uItemNo++) {
					pDataAcsProp++;
				}

				BOOL bArray = (pU8Msg[2] == '[');
				/* �ҵ���Ӧ��ItemNo������ָ��. ��Ҫ�ų����ϴ洢�Ĳ��� */
				if((uItemNo < pBootLoaderConf->u32ItemNum) && (pDataAcsProp->ItemType.SaveGrp != SAVE_GRP_BRD)) {
					pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 2);
					ProcItemPageInstr(DATA_USER_NET, DATA_PAGE_CONF, uItemNo, &pU8Msg, pU8MsgEnd, NULL);
				} else if(bArray) {
					pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, ']', 1);
				} else {
					pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, ',', 1);
				}

				/* Ϊ��һ��׼�� */
				g_MqttItemProc.uEndConfID = uConfID;
				uConfID = ReadU32(&pU8Msg, pU8MsgEnd);
			}
			g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = 5;
			Semaphore_post(g_DataAccessBlock.Sem_Op);		//Semaphore_post(SEM_NvMemAcs);
		}
	}
}

/*==========================================================================
| Description	: ����MqttConfд����Լ�MqttConfҳ�棬��������ҳ�����������ͬ������
				���д��һ�������ĺ�������Ϊ����MqttPubTask()�Եü��
| G/Out var		:
| Author		: King			Date	: 2017-12-28
\=========================================================================*/
/* ��ӡ��ǰ����ҳ��
    {"no":"21", "title":"ͨѶ1��������", "cont":{
        {"no":"22", "prompt":"����汾��", "text":"2.2.55"},
        {"no":"23", "prompt":"���������", "set":["200"]}}}    */
void PrintMqttConfPage(uint8** ppTxBuf, uint8* pTxBufEnd);
BOOL PrintMqttConfItemValue(uint8** ppTxBuf, uint8* pTxBufEnd, CONF_n_STAT_PROP* pDataAcsProp, uint32 u32ItemData);
BOOL QueryAndPubMqttConfResAndPage(MQTT_COMM* pMqttComm)
{
	/* ����ҳ������������ */
    if(g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP > 1) {      /* �ȴ�MdlDataAccess������,>1��Ϊ�����䵽0,��pub��������Ѿ���Ӧ������һ������ */
        g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP--;
    } else if(g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP == 1) {
        g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;      /* ������tryCnt,Ҳû�ܵ���MdlDataAccess����������ź�,�ж�Ϊʧ�� */
    }
	
	/* ����ҳ�������������� */
	if(g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP < 0) {
		/* ��������ҳ�� */
		uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
		uint8 u8QoS = 0;			/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
		/* ����ҳ�淢��:��ӡTopic */
		PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
		PrintStringNoOvChk(&pTxBuf, "/PAGE/CONF");
		uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
		if(u8QoS) {
			pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
		}
		PrintMqttConfPage(&pTxBuf, &pMqttComm->u8TRBuf[MQTT_TR_BUF_BLEN]);
		/* ����ҳ�淢��:���� */
		if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) {	/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
			return FALSE;
		}

		/* ����д���ý�� */
		pTxBuf = pMqttComm->u8TRBuf + 5; /* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
		u8QoS = 0;			/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
		/* д���ý������:��ӡTopic */
		PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
		PrintStringNoOvChk(&pTxBuf, "/RES/CONF");
		uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
		if(u8QoS) {
			pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
		}
		*pTxBuf++ = '{';		/* JSon��ʼ */
		if(g_MqttItemProc.uPageOpItemNo == 0xFFFF) {
			PrintStringToJson(&pTxBuf, "no", "ini");	/* no */
		} else {
			PrintU32DatToJson(&pTxBuf, "no", g_MqttItemProc.uPageOpItemNo, 0);	/* no */
		}
		/* ��ӡ��� */
		if(g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP == -1) {
			PrintStringToJson(&pTxBuf, "res", "success");
		} else {
			PrintStringToJson(&pTxBuf, "res", "fail");
		}
		pTxBuf--;
		*pTxBuf++ = '}';	/* Json��β */
		/* д����ҳ��������:���� */
		if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) { 	/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
			return FALSE;
		}
		g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP = 0; 			/* �����Ѿ�������� */
	}

	/* ����ͬ������������� */
    if(g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP > 1) {      /* �ȴ�MdlDataAccess������,>1��Ϊ�����䵽0,��pub��������Ѿ���Ӧ������һ������ */
        g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP--;
    } else if(g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP == 1) {
        g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;      /* ������tryCnt,Ҳû�ܵ���MdlDataAccess����������ź�,�ж�Ϊʧ�� */
    }

	/* ����ͬ��������������� */
	if(g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP < 0) {
		/* ��������ҳ�� */
		CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
		uint16 uItemNum = pBootLoaderConf->u32ItemNum;
		int8 u8PageNum = 0;
		while(uItemNum) {
			uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
			uint8 u8QoS = 1;			/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
			/* д���ý������:��ӡTopic */
			PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
			PrintStringNoOvChk(&pTxBuf, "/PAGE/SYNC");
			uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
			if(u8QoS) {
				pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
			}
			*pTxBuf++ = '{';		/* JSon��ʼ */
			PrintStringToJson(&pTxBuf, "device_type", DEVICE_TYPE_String);
			PrintSoftVerToJson(&pTxBuf, "soft_ver", SOFTWARE_VER);
			PrintU32DatToJson(&pTxBuf, "sn", g_Sys.SerialNo.u32Dat, 0);
			PrintU32DatToJson(&pTxBuf, "page_no", ++u8PageNum, 0);
			PrintStringNoOvChk(&pTxBuf, "\"conf\":{");
			while(uItemNum) {
				if((pDataAcsProp->pData !=	NULL) && (pDataAcsProp->ItemType.SaveGrp != SAVE_GRP_NUL)) {
					if(&pMqttComm->u8TRBuf[MQTT_TR_BUF_BLEN] - pTxBuf < 100) {	/* ȷ���˺�����Ȼ������ */
						break;
					} else {	/* ������Ŀ�   44byte-- "5b":["32b"],  */
						uint8* pTxBufBak = pTxBuf;
						*pTxBuf++ = '"';
						PrintU32WithLen(&pTxBuf, pDataAcsProp->u32ItemID, 4);
						*pTxBuf++ = '"';
						if(PrintMqttConfItemValue(&pTxBuf, &pMqttComm->u8TRBuf[MQTT_TR_BUF_BLEN], pDataAcsProp, *((uint32*)pDataAcsProp->pData))) {
							*pTxBuf++ = ',';
						} else {
							pTxBuf = pTxBufBak;
						}
					}
				}
				pDataAcsProp++;
				uItemNum--;
			}
			pTxBuf--;
			*pTxBuf++ = '}';	/* �� "\"conf\":{" ��� */
			*pTxBuf++ = ',';
			PrintBoolToJson(&pTxBuf, "to_be_continued", uItemNum);	/* �Ƿ���һҳ */
			pTxBuf--;
			*pTxBuf++ = '}';	/* Json��β */
			if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) {		/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
				return FALSE;
			}
		}
		
		/* ��������ͬ����� */
		uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
		uint8 u8QoS = 0;			/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
		/* д���ý������:��ӡTopic */
		PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
		PrintStringNoOvChk(&pTxBuf, "/RES/SYNC");
		uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
		if(u8QoS) {
			pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
		}
		*pTxBuf++ = '{';		/* JSon��ʼ */
		/* ��ӡ��� */
		if(g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP == -1) {
			PrintStringToJson(&pTxBuf, "res", "success");
		} else {
			PrintStringToJson(&pTxBuf, "res", "fail");
		}
		PrintU32DatToJson(&pTxBuf, "start_load_id", g_MqttItemProc.uStartConfID, 0);
		PrintU32DatToJson(&pTxBuf, "end_load_id", g_MqttItemProc.uEndConfID, 0);
		PrintU32DatToJson(&pTxBuf, "conf_page_num", u8PageNum, 0);
		pTxBuf--;
		*pTxBuf++ = '}';	/* Json��β */
		/* д����ҳ��������:���� */
		if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) { 	/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
			return FALSE;
		}
		g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = 0; 	/* �����Ѿ�������� */
	}

	return TRUE;
}

/* ��ӡ��ǰ����ҳ��
	{"no":"21", "title":"ͨѶ1��������", "cont":{
		{"no":"22", "prompt":"����汾��", "text":"2.2.55"},
		{"no":"23", "prompt":"���������", "set":["200"]}}}    */
void PrintAcqU32F32Conf(DATA_USER DataUser, LANG_TYPE Language, uint8** ppText, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp);
void PrintTempConf(DATA_USER DataUser, LANG_TYPE Language, uint8** ppText, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp);
void PrintAcqSummConf(DATA_USER DataUser, LANG_TYPE Language, uint8** ppText, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp);
void PrintSysFuncRes(uint8** ppTxBuf, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp, LANG_TYPE Language);
void PrintMqttConfPage(uint8** ppTxBuf, uint8* pTxBufEnd)
{
	uint8* pTxBuf = *ppTxBuf;
	uint16 uItemNo = g_MqttItemProc.ItemPageReq.uConfPageTopicItemNo;
	LANG_TYPE Language = g_MqttItemProc.ItemPageReq.Language;			/* ���棬�����ں���������б����� */

	CONF_n_STAT_PROP* pDataAcsProp = &pBootLoaderConf->pConfProp[uItemNo];
	if(pDataAcsProp->ItemType.DataType == ITEM_DAT_TOPIC) {			/* ������: �ж��ǲ���ҳ��Topic */
		/* ��ӡTopic�� */
		*pTxBuf++ = '{';
		PrintU32DatToJson(&pTxBuf, "no", uItemNo, 0);	/* id */
		PrintStringToJson(&pTxBuf, "title", pDataAcsProp->pName[Language]);		/* title���� */
		PrintStringNoOvChk(&pTxBuf, "\"cont\":[");

		/* ҳ�����ݶ�ȡ */
		uint32 u32PageData = 0;
		if(pDataAcsProp->pData != NULL) {
			u32PageData = *((uint32*)pDataAcsProp->pData);
		}

		/* ��ӡҳ�������� */
		uint32 u32ItemData = 0;		/* ��������U32������ */
		uint16 uBitStart = 0;
		int16 i;
		for(i = pDataAcsProp->LkId_Val_ItN; i > 0; i--) {
			uItemNo++;
			pDataAcsProp++;

			*pTxBuf++ = '{';
			PrintU32DatToJson(&pTxBuf, "no", uItemNo, 0);			/* ��ӡ��ǰ��id */
			if(pDataAcsProp->pData != NULL) {
				u32ItemData = *((uint32*)pDataAcsProp->pData);
			} else if(pDataAcsProp->ItemType.i8SgnDigits_u8BitNum	/* ȷ��Ϊλ���� */
					&& ((pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL)
						|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM)
						|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32)
						|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_RO)))
			{
				uint16 uBitEnd = uBitStart + pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
				u32ItemData = (u32PageData & ((1<<uBitEnd) - 1))>>uBitStart;
				uBitStart = uBitEnd;
			} else {												/* Ԥ�������δ����ֵ */
				u32ItemData = pDataAcsProp->LkId_Val_ItN;
			}
			
			if((pDataAcsProp->ItemType.DataType == ITEM_DAT_ACQ_U32) || (pDataAcsProp->ItemType.DataType == ITEM_DAT_ACQ_F32)) {
				PrintAcqU32F32Conf(DATA_USER_NET, Language, &pTxBuf, pTxBufEnd, pDataAcsProp);
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_TEMPSENCONF) {
				PrintTempConf(DATA_USER_NET, Language, &pTxBuf, pTxBufEnd, pDataAcsProp);
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_SUMM) {
				PrintAcqSummConf(DATA_USER_NET, Language, &pTxBuf, pTxBufEnd, pDataAcsProp);
			} else {
				PrintStringToJson(&pTxBuf, "prompt", pDataAcsProp->pName[Language]);	/* ��ʾ�� */
			}
		
			/* ֵ���� */
			if(pDataAcsProp->ItemType.DataType == ITEM_DAT_LINK) {
				if(1 || (pDataAcsProp->pData == NULL) || (*((uint32*)pDataAcsProp->pData) == 0)) {	/* 2021.7.2 Ŀǰ�������������ڲ�ʹ�ã��������� */
					PrintStringToJson(&pTxBuf, "jump", "");
				} else {
					PrintStringToJson(&pTxBuf, "jump", "***");
				}
				pTxBuf--;
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_S32) {
				PrintU32DatToJson(&pTxBuf, "jump", *((uint32*)pDataAcsProp->pData), 0);
				pTxBuf--;
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL) {
				PrintBoolToJson(&pTxBuf, "box", u32ItemData);
				pTxBuf--;
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM) {
				PrintU32DatToJson(&pTxBuf, "radio", u32ItemData, 0);	/* ö��ֵ���� */
				/* ö����Ŀ���� */
				PrintStringNoOvChk(&pTxBuf, "\"cont\":[");
				pDataAcsProp++;
				uint16 uCount = 0;
				for(uCount = 0; pDataAcsProp->ItemType.DataType == ITEM_DAT_ECONT; uCount++) {
					*pTxBuf++ = '{';
					PrintEnumContToJson(&pTxBuf, pDataAcsProp->pName[Language], pDataAcsProp->LkId_Val_ItN, 0);
					pTxBuf--;
					*pTxBuf++ = '}';
					*pTxBuf++ = ',';
					pDataAcsProp++;
				}
				i -= uCount;
				uItemNo += uCount;
				pDataAcsProp--;
				pTxBuf--;
				*pTxBuf++ = ']';
			} else {
				if((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_TEXT) {
					PrintStringNoOvChk(&pTxBuf, "\"text\"");
				} else {
					PrintStringNoOvChk(&pTxBuf, "\"set\"");
				}
				PrintMqttConfItemValue(&pTxBuf, pTxBufEnd, pDataAcsProp, u32ItemData);
			}
			*pTxBuf++ = '}';
			*pTxBuf++ = ',';
		}
		
		/* ��ӡ��β���� */
		pTxBuf--;
		*pTxBuf++ = ']';
		*pTxBuf++ = '}';
	}

	*ppTxBuf = pTxBuf;
}

BOOL PrintMqttConfItemValue(uint8** ppTxBuf, uint8* pTxBufEnd, CONF_n_STAT_PROP* pDataAcsProp, uint32 u32ItemData)
{
	int64 i64Data;

	/* Ԥ��׼���ø���������Чλ�����Ǹ���������Ҫ */
	int8 i8SgnDigits = pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
	if(i8SgnDigits == 0) {
		i8SgnDigits = 4;	/* Ĭ��ʹ��4λ��Ч���� */
	}

	uint8* pTxBuf = *ppTxBuf;
	*pTxBuf++ = ':';
    if((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_TWOBOX) {
        *pTxBuf++ = '[';
    }
	*pTxBuf++ = '"';

	switch(pDataAcsProp->ItemType.DataType) {
		/* PrintMqttConfPage()���������������������SYNC����ʱ��ʹ�� */
		case ITEM_DAT_TOPIC:
		case ITEM_DAT_BOOL:
		case ITEM_DAT_ENUM:
			PrintU32(&pTxBuf, pTxBufEnd, u32ItemData, 0);
			break;

        case ITEM_DAT_FUNC:     /* ���ݲ�����ִ����� */
            PrintSysFuncRes(&pTxBuf, pTxBufEnd, pDataAcsProp, g_MqttItemProc.ItemPageReq.Language);
            break;
            
        case ITEM_DAT_U32_RO:
        case ITEM_DAT_U32_ID_RO:
		case ITEM_DAT_U32:
		case ITEM_DAT_ACQ_U32:
		case ITEM_DAT_TEMPSENCONF:
			PrintU32(&pTxBuf, pTxBufEnd, u32ItemData, pDataAcsProp->ItemType.uBcdQ);
			break;

		case ITEM_DAT_IPv4:
			PrintIPv4(&pTxBuf, pTxBufEnd, *((uint32*)pDataAcsProp->pData));
			break;

	#if SUPPORT_IEC101 || SUPPORT_IEC104
		case ITEM_DAT_TELE_ITEM:
			PrintTeleSigMetConf(&pTxBuf, pTxBufEnd, (TELE_101_104_ITEM_CONF*)pDataAcsProp->pData);
			break;
	#endif

		case ITEM_DAT_F32:
		case ITEM_DAT_ACQ_F32:
			PrintF32(&pTxBuf, pTxBufEnd, *((float32*)pDataAcsProp->pData), i8SgnDigits);
			break;

		case ITEM_DAT_I64:
        case ITEM_DAT_I64_RO:
			Swi_disable();
			i64Data = (*((int64*)pDataAcsProp->pData));
			Swi_enable();
			PrintI64(&pTxBuf, pTxBufEnd, i64Data, pDataAcsProp->ItemType.uBcdQ);
			break;

		case ITEM_DAT_D2U32:
			PrintU32(&pTxBuf, pTxBufEnd, *((uint32*)pDataAcsProp->pData), pDataAcsProp->ItemType.uBcdQ);
        	*pTxBuf++ = '"';
			*pTxBuf++ = ',';
        	*pTxBuf++ = '"';
			PrintU32(&pTxBuf, pTxBufEnd, *(((uint32*)pDataAcsProp->pData) + 1), pDataAcsProp->ItemType.uBcdQ);
			break;

		case ITEM_DAT_D2F32:
			PrintF32(&pTxBuf, pTxBufEnd, *((float32*)pDataAcsProp->pData), i8SgnDigits);
        	*pTxBuf++ = '"';
			*pTxBuf++ = ',';
        	*pTxBuf++ = '"';
			PrintF32(&pTxBuf, pTxBufEnd, *(((float32*)pDataAcsProp->pData) + 1), i8SgnDigits);
			break;

		case ITEM_DAT_SUMM:
			PrintI32(&pTxBuf, pTxBufEnd, *((int32*)pDataAcsProp->pData +0), 0);
        	*pTxBuf++ = '"';
			*pTxBuf++ = ',';
        	*pTxBuf++ = '"';
			PrintF32(&pTxBuf, pTxBufEnd, *((float32*)pDataAcsProp->pData + 1), i8SgnDigits);
			break;

		case ITEM_DAT_pTIME:
			PrintPriceTime(&pTxBuf, pTxBufEnd, *((uint32*)pDataAcsProp->pData + 0));
        	*pTxBuf++ = '"';
			*pTxBuf++ = ',';
        	*pTxBuf++ = '"';
			PrintPriceTime(&pTxBuf, pTxBufEnd, *((uint32*)pDataAcsProp->pData + 1));
			break;

		case ITEM_DAT_RMTS:
		case ITEM_DAT_DIGITSTR:
		case ITEM_DAT_ANSI:
        case ITEM_DAT_ANSI_RO:
			PrintString(&pTxBuf, pTxBufEnd, (char*)(pDataAcsProp->pData));
			break;

		case ITEM_DAT_TXT:	/* ����Ҫ�����Ҫ��Ȼ�ͻ����default�� */
			break;

		case ITEM_DAT_SOFTVER:
			PrintSoftVer(&pTxBuf, pTxBufEnd, pDataAcsProp->LkId_Val_ItN);
			break;

		case ITEM_DAT_T64_RO:
			Swi_disable();
			PrintT64(DATA_USER_NET, &pTxBuf, pTxBufEnd, (REAL_TIME_VAR*)pDataAcsProp->pData);
			Swi_enable();
			break;

		case ITEM_DAT_T32_RO:
			PrintT32(DATA_USER_NET, &pTxBuf, pTxBufEnd, *(uint32*)pDataAcsProp->pData);
			break;

		default:
			return FALSE;
	}
	
	*pTxBuf++ = '"';
	if((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_TWOBOX) {
        *pTxBuf++ = ']';
    }
	*ppTxBuf = pTxBuf;

	return TRUE;
}

/*==========================================================================
	�����ǹ����Ĺ��ߺ���
\=========================================================================*/
/*==========================================================================
| Description	: ���ڰ�CONF_n_STAT_PROP��ʽ��conf(stat)��Ŀ��ӡ���ı�, 				��Ҫ���ڱ���MCGS, ViewTech��Ļ���ļ��洢
		F32���10byte, U32���10byte, IPV4 15byte, U64���20byte, T64������21byte, mqtt���ĵ�TOPIC��18byte
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2018-07-31
\=========================================================================*/
uint8* PrintLocalConfItem(DATA_USER DataUser, CONF_n_STAT_PROP* pDataAcsProp, uint8* pBuf, uint16 uTextByteLen)
{
	CONF_n_STAT_PROP* pDataAcsPropBak = pDataAcsProp;
	uint8* pBufStart = pBuf;
	int16 i;
	
	uint8* pDataSegEnd = pBufStart + uTextByteLen;				/* ���ݶν�β, ��ָ��BufEnd���ٸ��ݲ�ͬ�����Ϳ۳� */
	uint8* pPromptSegEnd = pDataSegEnd;							/* ��ʾ���ν�β, ��ָ��BufEnd���ٸ��ݲ�ͬ�����Ϳ۳� */
	if(DataUser == DATA_USER_FILE) {
		PrintU32WithLen(&pBuf, pDataAcsProp->u32ItemID, 4);		/* �γ�4λ��ʽ */
		*pBuf++ = ' ';
		*pBuf++ = ' ';	/* Ϊ�˼��� ��������ʾ�� �İ汾 */
#if SUPPORT_SCREEN
	} else if(DataUser == DATA_USER_MCGS) {	/* mcgs��ĻBufҪ�ȴ��䳤�ȶ��MCGS_ITEM_PAGE_ADDIPAYLOAD_BSTART�Ա���� */
		pBuf += MCGS_ITEM_PAGE_ADDI_BLEN;
		pPromptSegEnd += MCGS_ITEM_PAGE_ADDI_BLEN;
#endif
	}

	/* ��ʾλ�õ��� */
	if((pDataAcsProp->ItemType.DataType == ITEM_DAT_TXT)
		|| ((pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM) && (DataUser != DATA_USER_FILE)))
	{
	} else if((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_TWOBOX) {
		pPromptSegEnd -= 22;
		pDataSegEnd -= 11;
	} else if((DataUser == DATA_USER_FILE)
			|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_ANSI_RO)			
			|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_T64_RO)
			|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_T32_RO)
			|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_I64_RO)
			|| ((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_ONEDBOX)
			|| ((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_ANSI))
	{
		pPromptSegEnd -= 22;
	} else {
		pPromptSegEnd -= 11;
	}

	/* ��ʾ������ */
	if(g_Sys.u32LocalLang >= MAX_LANG_TYPE) {
		g_Sys.u32LocalLang = CHINESE;
	}
	if(pDataAcsProp->ItemType.DataType == ITEM_DAT_TEMPSENCONF) {
		PrintTempConf(DataUser, (LANG_TYPE)g_Sys.u32LocalLang, &pBuf, pPromptSegEnd, pDataAcsProp);
	} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_SUMM) {
		PrintAcqSummConf(DataUser, (LANG_TYPE)g_Sys.u32LocalLang, &pBuf, pPromptSegEnd, pDataAcsProp);
	} else if(DataUser == DATA_USER_FILE) {		/* �ļ��洢����������ʾ�����Լ��ٴ洢�� */
	} else if((pDataAcsProp->ItemType.DataType == ITEM_DAT_ACQ_U32) || (pDataAcsProp->ItemType.DataType == ITEM_DAT_ACQ_F32)) {
		PrintAcqU32F32Conf(DataUser, (LANG_TYPE)g_Sys.u32LocalLang, &pBuf, pPromptSegEnd, pDataAcsProp);
	} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_TOPIC) {
		PrintTopicString(&pBuf, uTextByteLen, pDataAcsProp->pName[g_Sys.u32LocalLang]); /* ��Ļ�����ܹ�ռ��30byte��� */
	} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ECONT) {
		*pBuf++ = ' ';
		*pBuf++ = ' ';
		PrintString(&pBuf, pPromptSegEnd, pDataAcsProp->pName[g_Sys.u32LocalLang]);
	} else {
		PrintString(&pBuf, pPromptSegEnd, pDataAcsProp->pName[g_Sys.u32LocalLang]);
	}
	if(0) {
#if SUPPORT_SCREEN
	} else if(DataUser == DATA_USER_MCGS) {
		*pBuf = 0;
		pPromptSegEnd -= MCGS_ITEM_PAGE_ADDI_BLEN;			/* ������ȥ */
		LeftAlignString_MCGS(pBufStart, pPromptSegEnd - pBufStart);	/* MCGS��Ҫ�Ѻ��ַ���һ��Word���� */
		pBuf = pPromptSegEnd;
#endif
	} else if(DataUser == DATA_USER_FILE) {
	} else {						/* ������ǰ�� */
		for(i = pPromptSegEnd - pBuf; i > 0; i--) {
			*pBuf++ = ' ';
		}
	}
	
	/* �����ǰ������ECONT, ��Ҫ�ѵ�ǰָ���Ƶ�ENUM������������ı���λ�� */
	uint32 u32ItemData;
	if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ECONT) {
		do {
			pDataAcsProp--;
		} while(pDataAcsProp->ItemType.DataType != ITEM_DAT_ENUM);
	}
	if(pDataAcsProp->pData != NULL) {
		u32ItemData = *((uint32*)pDataAcsProp->pData);
	/* FILE�����ӡ����λ���� */
	} else if((DataUser == DATA_USER_FILE) || (pDataAcsProp->ItemType.DataType == ITEM_DAT_LINK)) {
		u32ItemData = 0;
	} else {									/* ����λ���� */
		uint16 uVarBitNum = 0;					/* ��ǰ����λ�� */
		uint16 uBitStart = 0;					/* ��ǰ������ʼλ */
		/* ����ֱ����ǰҳ��Topic */
		while(pDataAcsProp->ItemType.DataType != ITEM_DAT_TOPIC) {
			/* ȷ��Ϊλ���� */
			if((pDataAcsProp->pData == NULL) 
				&& pDataAcsProp->ItemType.i8SgnDigits_u8BitNum
				&& ((pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_RO)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_ID_RO)))
			{
				if(uVarBitNum == 0) {
					uVarBitNum = pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
				} else {
					uBitStart += pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
				}
			}
			pDataAcsProp--;
		}
		if(pDataAcsProp->pData == NULL) {
			u32ItemData = 0;
		} else {
			u32ItemData = ((*((uint32*)pDataAcsProp->pData))>>uBitStart) & ((1<<uVarBitNum) - 1);
		}
	}
	pDataAcsProp = pDataAcsPropBak;

	/* Ԥ��׼���ø���������Чλ�����Ǹ���������Ҫ */
	int8 i8SgnDigits = pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
	if(i8SgnDigits == 0) {
		i8SgnDigits = 4;	/* Ĭ��ʹ��4λ��Ч���� */
	}
	
	/* �������������ӡ������ */
	int64 i64Data;
	switch(pDataAcsProp->ItemType.DataType) {
		case ITEM_DAT_LINK:
			if(u32ItemData && (!g_Ctr.bEngDebug)) {
				PrintString(&pBuf, pDataSegEnd, "  ******");
			}
			break;
			
        case ITEM_DAT_FUNC:     /* ���ݲ�����ִ����� */
            PrintSysFuncRes(&pBuf, pDataSegEnd, pDataAcsProp, (LANG_TYPE)g_Sys.u32LocalLang);
            break;

		case ITEM_DAT_TOPIC:	/* TOPIC�����ݲŴ�ӡ */
			if(pDataAcsProp->pData == NULL) {
				break;
			}
		case ITEM_DAT_ENUM:		/* �����DATA_USER_UI��pDataSegEnd�Ѿ��޸ĵ�ָ��pBufEnd */
		case ITEM_DAT_S32:
		case ITEM_DAT_U32:
		case ITEM_DAT_U32_ID_RO:
		case ITEM_DAT_ACQ_U32:
		case ITEM_DAT_U32_RO:
		case ITEM_DAT_TEMPSENCONF:
			PrintU32(&pBuf, pDataSegEnd, u32ItemData, pDataAcsProp->ItemType.uBcdQ);
			break;

		case ITEM_DAT_IPv4:
			PrintIPv4(&pBuf, pDataSegEnd, *((uint32*)pDataAcsProp->pData));
			break;

	#if SUPPORT_IEC101 || SUPPORT_IEC104
		case ITEM_DAT_TELE_ITEM:
			PrintTeleSigMetConf(&pBuf, pDataSegEnd, (TELE_101_104_ITEM_CONF*)pDataAcsProp->pData);
			break;
	#endif
	
		case ITEM_DAT_F32:
		case ITEM_DAT_ACQ_F32:
			PrintF32(&pBuf, pDataSegEnd, *((float32*)pDataAcsProp->pData), i8SgnDigits);		/* ʹ��4λ��Ч���� */
			break;
	
		case ITEM_DAT_I64:
        case ITEM_DAT_I64_RO:
			Swi_disable();
			i64Data = *((int64*)pDataAcsProp->pData);
			Swi_enable();
			PrintI64(&pBuf, pDataSegEnd, i64Data, pDataAcsProp->ItemType.uBcdQ);
			break;
	
		case ITEM_DAT_D2U32:
			PrintU32(&pBuf, pDataSegEnd, *((uint32*)pDataAcsProp->pData), pDataAcsProp->ItemType.uBcdQ);
			if(DataUser == DATA_USER_FILE) {
				*pBuf++ = ' ';
			} else {
				for(i = pDataSegEnd - pBuf; i > 0; i--) {
					*pBuf++ = ' ';
				}
			}
			pDataSegEnd += 11;
			PrintU32(&pBuf, pDataSegEnd, *(((uint32*)pDataAcsProp->pData) + 1), pDataAcsProp->ItemType.uBcdQ);
			break;
			
		case ITEM_DAT_D2F32:
			PrintF32(&pBuf, pDataSegEnd, *((float32*)pDataAcsProp->pData), i8SgnDigits);		/* ʹ��4λ��Ч���� */
			if(DataUser == DATA_USER_FILE) {
				*pBuf++ = ' ';
			} else {
				for(i = pDataSegEnd - pBuf; i > 0; i--) {
					*pBuf++ = ' ';
				}
			}
			pDataSegEnd += 11;
			PrintF32(&pBuf, pDataSegEnd, *(((float32*)pDataAcsProp->pData) + 1), i8SgnDigits);/* ʹ��4λ��Ч���� */
			break;
			
		case ITEM_DAT_SUMM:
			PrintI32(&pBuf, pDataSegEnd, *((int32*)pDataAcsProp->pData + 0), 0);
			if(DataUser == DATA_USER_FILE) {
				*pBuf++ = ' ';
			} else {
				for(i = pDataSegEnd - pBuf; i > 0; i--) {
					*pBuf++ = ' ';
				}
			}
			pDataSegEnd += 11;
			PrintF32(&pBuf, pDataSegEnd, *((float32*)pDataAcsProp->pData + 1), i8SgnDigits);	/* ʹ��4λ��Ч���� */
			break;
		
		case ITEM_DAT_pTIME:
			PrintPriceTime(&pBuf, pDataSegEnd, *((uint32*)pDataAcsProp->pData + 0));
			if(DataUser == DATA_USER_FILE) {
				*pBuf++ = ' ';
			} else {
				for(i = pDataSegEnd - pBuf; i > 0; i--) {
					*pBuf++ = ' ';
				}
			}
			pDataSegEnd += 11;
			PrintPriceTime(&pBuf, pDataSegEnd, *((uint32*)pDataAcsProp->pData + 1));
			break;
			
		case ITEM_DAT_RMTS:
		case ITEM_DAT_DIGITSTR:
		case ITEM_DAT_ANSI:
		case ITEM_DAT_ANSI_RO:
			Swi_disable();
			PrintString(&pBuf, pDataSegEnd, (char*)(pDataAcsProp->pData));
			Swi_enable();
			break;

		case ITEM_DAT_ECONT:
			if(pDataAcsProp->LkId_Val_ItN == u32ItemData) {
				PrintString(&pBuf, pDataSegEnd, " ��");
			} else {
				PrintString(&pBuf, pDataSegEnd, " ��");
			}
			break;
			
		case ITEM_DAT_BOOL:
			if(DataUser == DATA_USER_FILE) {
				PrintU32(&pBuf, pDataSegEnd, u32ItemData, 0);
			} else if(u32ItemData) {
				PrintString(&pBuf, pDataSegEnd, " ��");
			} else {
				PrintString(&pBuf, pDataSegEnd, " ��");
			}
			break;

		case ITEM_DAT_T64_RO:
			Swi_disable();
			PrintT64(DataUser, &pBuf, pDataSegEnd, (REAL_TIME_VAR*)pDataAcsProp->pData);
			Swi_enable();
			break;

		case ITEM_DAT_T32_RO:
			PrintT32(DataUser, &pBuf, pDataSegEnd, *(uint32*)pDataAcsProp->pData);
			break;
			
		case ITEM_DAT_SOFTVER:
			PrintSoftVer(&pBuf, pDataSegEnd, pDataAcsProp->LkId_Val_ItN);
			break;
		
		default:
			break;
	}

	/* ��ӽ�β */
	if((DataUser == DATA_USER_MCGS) || (DataUser == DATA_USER_VIEWTECH)) {
		for(i = pDataSegEnd - pBuf; i > 0; i--) {
			*pBuf++ = ' ';
		}
	} else if(DataUser == DATA_USER_FILE) {	/* FILE��ʵ����Ҫ������� */
		*pBuf++ = 0x0D;
		*pBuf++ = 0x0A;
 	} else if(pBuf < pDataSegEnd) {
		*pBuf++ = 0;
	}

	/* ���� */
	return pBuf;
}

/*==========================================================================
| Description	: �Ѿ����Զ��ɼ����ܵ�����ת��ΪItemPage����Ҫ���ı�
	��ʾ����Ļ��: ��ǰʱ��(s)��ǰ:0.1; �趨:
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2019-5-15
\=========================================================================*/
void PrintAcqU32F32Conf(DATA_USER DataUser, LANG_TYPE Language, uint8** ppText, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp)
{
	if(DataUser == DATA_USER_NET) {
		PrintStringNoOvChk(ppText, "\"prompt\":\"");
	}

	PrintString(ppText, pTextEnd, pDataAcsProp->pName[Language]);
	
	if((DataUser == DATA_USER_MCGS) || (DataUser == DATA_USER_VIEWTECH) ||  (DataUser == DATA_USER_NET)) {
		if(Language == CHINESE) {
			PrintString(ppText, pTextEnd, " ����:");
		} else if(Language == ENGLISH) {
			PrintString(ppText, pTextEnd, " Cali:");
		}
		/* ���㸡��������Чλ�� */
		int8 i8SgnDigits = pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
		if(i8SgnDigits == 0) {
			i8SgnDigits = 4;	/* Ĭ��ʹ��4λ��Ч���� */
		}
		PrintF32(ppText, pTextEnd, *((float32*)(pDataAcsProp->pData) + 1), i8SgnDigits);
		
		if(DataUser == DATA_USER_NET) {
			*(*ppText)++ = '"';
			*(*ppText)++ = ',';
		}
	}
}

/*==========================================================================
| Description	: ���¶ȴ��������ô�ӡΪ�ı�
��ʾ����Ļ��: 01#�¶ȴ�������ǰ�¶�:127.8 ����ʧ�ܼ���:999999999 ����λ��:1 							    	//����"01#�¶ȴ�����"��PropText����
�洢���ļ���: 01#�¶ȴ�����SN:0123456789ABCDEF ����ʧ�ܼ���:999999999 ����λ��:1							//����"01#�¶ȴ�����"��PropText����
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-06-20
\=========================================================================*/
void PrintTempConf(DATA_USER DataUser, LANG_TYPE Language, uint8** ppText, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp)
{
#if MAX_TEMP_SEN_NUM
	if(DataUser == DATA_USER_NET) {
		PrintStringNoOvChk(ppText, "\"prompt\":\"");
	}

	if(DataUser == DATA_USER_FILE) {
		Swi_disable();
		uint64 u64TempSenRom = ((TEMP_SEN_CONF*)(pDataAcsProp->pData))->u64TempSenRom;
		Swi_enable();
		PrintH64(ppText, pTextEnd, u64TempSenRom);
		*(*ppText)++ = ' ';
		PrintU32(ppText, pTextEnd, ((TEMP_SEN_CONF*)(pDataAcsProp->pData))->u32Count_TempSen_Change, 0);
		*(*ppText)++ = ' ';
	} else {
		char* pPropText = pDataAcsProp->pName[Language];
		PrintString(ppText, pTextEnd, pPropText);
		if(Language == CHINESE) {
			PrintString(ppText, pTextEnd, "��ǰ�¶�:");
		} else if(Language == ENGLISH) {
			PrintString(ppText, pTextEnd, "Temp(c):");
		}
		/* �������¶� */
		uint8 u8TempSenNo = (*pPropText - '0')*10 + (*(pPropText+1) - '0');
		u8TempSenNo--;
		if(u8TempSenNo < MAX_TEMP_SEN_NUM) {
			PrintI16(ppText, pTextEnd, g_AnaRes.Temperature[u8TempSenNo].iDegree_100mC, 1);
		} else {
			*(*ppText)++ = '0';
		}

		if(Language == CHINESE) {
			PrintString(ppText, pTextEnd, " ���߼���:");
			PrintU32(ppText, pTextEnd, ((TEMP_SEN_CONF*)(pDataAcsProp->pData))->u32Count_TempSen_Change, 0);
			PrintString(ppText, pTextEnd, " ����λ��:");
		} else if(Language == ENGLISH) {
			PrintString(ppText, pTextEnd, " Count:");
			PrintU32(ppText, pTextEnd, ((TEMP_SEN_CONF*)(pDataAcsProp->pData))->u32Count_TempSen_Change, 0);
			PrintString(ppText, pTextEnd, " Placement:");
		}

		if(DataUser == DATA_USER_NET) {
			*(*ppText)++ = '"';
			*(*ppText)++ = ',';
		}
	}
#endif
}

/*==========================================================================
| Description	: �Ѿ����Զ��ɼ����ۺϹ��ܵ�����ת��ΪItemPage����Ҫ���ı�
	��ʾ����Ļ��: ˮ������K1��ǰ:0.1�ۺ�:0.0987; (����:��ʼֵ):
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-10-19
\=========================================================================*/
void PrintAcqSummConf(DATA_USER DataUser, LANG_TYPE Language, uint8** ppText, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp)
{
	int8 i8SgnDigits = pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
	if(i8SgnDigits == 0) {
		i8SgnDigits = 4;	/* Ĭ��ʹ��4λ��Ч���� */
	}

	if(DataUser == DATA_USER_FILE) {
		PrintF32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->fAcqSummary, i8SgnDigits);
		*(*ppText)++ = ' ';
		PrintU32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->u32SummaryRealNum, 0);
		*(*ppText)++ = ' ';
	} else {
		if(DataUser == DATA_USER_NET) {
			PrintStringNoOvChk(ppText, "\"prompt\":\"");
		}
		
		PrintString(ppText, pTextEnd, pDataAcsProp->pName[Language]);
		if(Language == CHINESE) {
			PrintString(ppText, pTextEnd, "��");
		} else if(Language == ENGLISH) {
			PrintString(ppText, pTextEnd, "Now");
		}
		PrintF32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->fCurAcq, i8SgnDigits);
	
		if(Language == CHINESE) {
			PrintString(ppText, pTextEnd, "��");
			PrintF32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->fAcqSummary, i8SgnDigits);
			PrintString(ppText, pTextEnd, "ʵ");
			PrintU32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->u32SummaryRealNum, 0);
			PrintString(ppText, pTextEnd, ";���");
		} else if(Language == ENGLISH) {
			PrintString(ppText, pTextEnd, "Comp");
			PrintF32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->fAcqSummary, i8SgnDigits);
			PrintString(ppText, pTextEnd, "Real");
			PrintU32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->u32SummaryRealNum, 0);
			PrintString(ppText, pTextEnd, ";NumIni");
		}
		
		if(DataUser == DATA_USER_NET) {
			*(*ppText)++ = '"';
			*(*ppText)++ = ',';
		}
	}
}

/* ��ӡ g_DataAcsIntf.i8SysFunc �Ĳ������ */
void PrintSysFuncRes(uint8** ppTxBuf, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp, LANG_TYPE Language)
{
    /* ������ */
    if((Language >= MAX_LANG_TYPE) || (pDataAcsProp->LkId_Val_ItN >= MAX_SYS_FUNC)) {
        return;
    }
    int8 i8DataOperation = g_DataAcsIntf.i8SysFunc[pDataAcsProp->LkId_Val_ItN];
    if(i8DataOperation < 0) {           /* ������ʽ����� */
        if(i8DataOperation < DATA_ACS_RES_MAX) {
            PrintStringNoOvChk(ppTxBuf, cnst_DataAcsOprRes[0 - DATA_ACS_RES_MAX][Language]);
        } else {                        /* ִ�н�� */
            PrintStringNoOvChk(ppTxBuf, cnst_DataAcsOprRes[0 - i8DataOperation][Language]);
        }
    } else if(i8DataOperation > 0) {    /* ��ֵ���γɵ���ʱ */
        PrintU32(ppTxBuf, pTextEnd, i8DataOperation, 0);
    }
}

/*==========================================================================
| Description	: ���ڴ���DATA_USER_GUI�µ�conf,stat,acq,msgָ���Լ�DATA_USER_NET��confָ��(stat������confҳ��֮��)
		1. ��ʼ��ָ��: ItemNoд��0xFFFFFFFFUL
		2. CONF(��STAT)ҳ�棬Topic��֧������ҳ�����ָ�
		   a. english, chinese �趨ҳ����ʾ���ԣ�����mqtt����
		   b. back: ������һ��ҳ��
		   c. up, down��ǰҳ����ʾ�������¹�������������ö���б���һ����������ʾ������UI����
		   d. exit: �˳�����״̬�������������洢
		3. S32(д����ֵ)\LINK(�������д����ȷ���룬����д����ֵ)������һ��ҳ��
		4. BOOL֧��true(��),false(��),click(ȡ��)
		5. ECONT֧��true,click�����嶼�ǰѵ�ǰ��ECONTֵ�����б���ָ���ENUM����
			��ȻҲ����ֱ��д��ENUM����
		6. ������������ֱ��д��Ӧ��ֵ��U32���Ϳ���֧�ָ���������
| In/Out/G var	: ����ֵ: -1:�ɹ� -2:ʧ�� >0:�ȴ�ʱ��(���波�Դ���)
| Author		: Wang Renfei			Date	: 2018-07-31
\=========================================================================*/
int8 ProcItemPageInstr(DATA_USER DataUser, DATA_PAGE DataPage, uint32 u32ItemNo, uint8** ppU8Instr, uint8* pU8InstrEnd, ITEM_PAGE_REQ* pItemPageReq)
{
	CONF_n_STAT_PROP* pDataAcsProp;
	int16 i;
	int8 i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = 0;

	/* ��ʼ��ҳ�� */
	if(pItemPageReq == NULL) {
		pDataAcsProp = pBootLoaderConf->pConfProp + u32ItemNo;
	} else if((DataPage != pItemPageReq->DataPage) || (u32ItemNo == 0xFFFFFFFFUL)) {
		pItemPageReq->DataPage = DataPage;
		if((DataPage == DATA_PAGE_MSG) || (DataPage == DATA_PAGE_ACQ)) {	/* ������Ӧ�����д��ָ�� */
			pItemPageReq->u32ItemNo = 0xFFFFFFFFUL;
			return -1;
		}

		u32ItemNo = 0;									/* �������pDataAcsProp */
		pDataAcsProp = pBootLoaderConf->pConfProp;
		pItemPageReq->u32ItemNo = u32ItemNo;
		pItemPageReq->uConfPageTopicItemNo = u32ItemNo;
		pItemPageReq->uConfWinFirstLineItemNo = u32ItemNo + 1;
	} else if((DataPage == DATA_PAGE_MSG) || (DataPage == DATA_PAGE_ACQ)) {			/* ������Ӧ�����д��ָ�� */
		pItemPageReq->bMsgAcqPageWinDown = (u32ItemNo < pItemPageReq->u32ItemNo);
		pItemPageReq->u32ItemNo = u32ItemNo;
		return -1;
	} else if((u32ItemNo < pItemPageReq->uConfPageTopicItemNo)							/* ������ */
			|| (u32ItemNo > pItemPageReq->uConfPageTopicItemNo + pBootLoaderConf->pConfProp[pItemPageReq->uConfPageTopicItemNo].LkId_Val_ItN))
	{
		return -2;
	} else {
		pDataAcsProp = pBootLoaderConf->pConfProp + u32ItemNo;
	}
	
	/* ����ҳ�����ݴ��� */
	if(ppU8Instr == NULL) {
		return -1;
	}
	uint8* pU8Instr = *ppU8Instr;
	if(pU8Instr >= pU8InstrEnd) {
	} else if((*pU8Instr == 0) || (*pU8Instr == '"')) {	/* ��һ���ַ���û�� */
		pU8Instr++;		/* ����ָ�룬���������ý�β */
	} else {
		i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -1;
		uint32 u32ItemData = 0;
		BOOL bEditConfValueInstr = FALSE;
		
		/* SYNC����ҳ��ά��ָ�� */
		if(pItemPageReq == NULL) {
			bEditConfValueInstr = TRUE;
			if(pDataAcsProp->ItemType.DataType == ITEM_DAT_TOPIC) {
				if(!GetU32(&pU8Instr, pU8InstrEnd, &u32ItemData, 0)) {
					i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
				}
			}
		} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_TOPIC) {	/* ���� refresh,back,up,down */
			if(CompareInstr(&pU8Instr, pU8InstrEnd, "refresh")) { 			/* ��־���Ϊ-1�ͻᵼ��ˢ�� */
			} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "exit")) {
			} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "back")) {		/* ���ص��ô��ڳ�ʼλ�ã����������ת�������ڵ�һ������ */
				if(pItemPageReq->uConfPageTopicItemNo > 0) {	/* 2021-08-21 ����ҳ����Ҫִ�з��� */
					void DealGuiConfWinBack(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq);
					DealGuiConfWinBack(DataUser, pItemPageReq);
//				} else {	/* ������ϴ��߼�,iconfig������쳣 */
//					i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = 0;	/* ���򷵻�ʧ�� */
				}
			} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "up")) {
				void DealGuiConfWinUp(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq);
				DealGuiConfWinUp(DataUser, pItemPageReq);
			} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "down")) {
				void DealGuiConfWinDown(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq);
				DealGuiConfWinDown(DataUser, pItemPageReq);
			} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "chinese")) {
				pItemPageReq->Language = CHINESE;
			} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "english")) {
				pItemPageReq->Language = ENGLISH;
			} else {
				i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
			}
		} else if((pDataAcsProp->ItemType.DataType == ITEM_DAT_LINK)	/* ���������һ��ҳ�� */
				|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_S32))
		{
			if((pDataAcsProp->ItemType.DataType == ITEM_DAT_S32)		/* �������������Ҫ����"click" */
				|| g_Ctr.bEngDebug 
				|| (DataUser == DATA_USER_NET)							/* 2021.7.2 Ŀǰ�������������ڲ�ʹ�ã��������� */
				|| (pDataAcsProp->pData == NULL)
				|| (*((uint32*)pDataAcsProp->pData) == 0)
				|| ((pDataAcsProp->ItemType.DataType == ITEM_DAT_LINK)	/* ������֤ */
					&& (*((uint32*)pDataAcsProp->pData) == ReadU32(&pU8Instr, pU8InstrEnd))))
			{
			    for(i = pBootLoaderConf->u32ItemNum - 1; 
			            (i >= 0) 
			            && ((pBootLoaderConf->pConfProp[i].ItemType.DataType != ITEM_DAT_TOPIC)  /* ȷ������TOPIC */
			                || (pDataAcsProp->LkId_Val_ItN != pBootLoaderConf->pConfProp[i].u32ItemID)); 
			                i--) 
			    { }
			    if(i >= 0) {
    				pItemPageReq->uConfPageTopicItemNo = i;
    				pItemPageReq->uConfWinFirstLineItemNo = pItemPageReq->uConfPageTopicItemNo + 1;
    		    } else {
                    i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
    		    }
			} else {
				i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
			}
        } else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_FUNC) {
            if(RegisterSysFunc((SYS_FUNCTION)pDataAcsProp->LkId_Val_ItN, 
                    (pDataAcsProp->pData == NULL) || (*(uint32*)pDataAcsProp->pData == ReadU32(&pU8Instr, pU8InstrEnd))))
            {
                i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = 5;
            } else {
                i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
            }
		} else {
			bEditConfValueInstr = TRUE;
		}

		/* �����޸�ָ�� */
		if(bEditConfValueInstr) {
			uint16 uBitStart = 0;
			uint16 uVarBitWidth = 0;
			BOOL bGetSuc = TRUE;

			/* ECONT��ҪԤ�ȴ�����Ϊ���������Ҫ��ָ���Ƶ�ENUM */
			BOOL bItemIsECONT = FALSE;
			if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ECONT) {
				bItemIsECONT = TRUE;
				if(CompareInstr(&pU8Instr, pU8InstrEnd, "click") || CompareInstr(&pU8Instr, pU8InstrEnd, "true")) {	/* ѡ�е�ǰ��Ŀ */
					u32ItemData = pDataAcsProp->LkId_Val_ItN;
					do {
						pDataAcsProp--;
					} while(pDataAcsProp->ItemType.DataType != ITEM_DAT_ENUM);	/* �ѵ�ǰָ���Ƶ�ENUM������������ı���λ�� */
				} else {
					bGetSuc = FALSE;
				}
			}

			/* ��ǰ������λ��������Ҫ������ʼλ��λ�� */
			if((pItemPageReq != NULL)
				&& (pDataAcsProp->pData == NULL) 
				&& pDataAcsProp->ItemType.i8SgnDigits_u8BitNum
				&& ((pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32)))
			{
				CONF_n_STAT_PROP* pDataAcsPropBak = pDataAcsProp;		/* ���ݸ�ָ�룬�������������̵��޸� */
				uVarBitWidth = pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
				pDataAcsProp--;
				for(i = pDataAcsProp - (pBootLoaderConf->pConfProp + pItemPageReq->uConfPageTopicItemNo); i > 0; i--) {
					if((pDataAcsProp->pData == NULL) 	/* ȷ��Ϊλ���� */
						&& pDataAcsProp->ItemType.i8SgnDigits_u8BitNum
						&& ((pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL)
							|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM)
							|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32)
							|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_RO))) 
					{
						uBitStart += pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
					}
					pDataAcsProp--;
				}
				pDataAcsProp = pDataAcsPropBak;
			}

			/* ������ĿԤ�ȴ��� */
			if(bItemIsECONT) {		/* ָ���Ѿ�������ENUM����ֹ�������ENUM */
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM) {
				bGetSuc = GetU32(&pU8Instr, pU8InstrEnd, &u32ItemData, 0);
			} else if((pDataAcsProp->ItemType.DataType == ITEM_DAT_U32) || (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_RO)) {
				bGetSuc = GetU32(&pU8Instr, pU8InstrEnd, &u32ItemData, pDataAcsProp->ItemType.uBcdQ);
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL) {
				if(CompareInstr(&pU8Instr, pU8InstrEnd, "true")) {
					u32ItemData = TRUE;
				} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "false")) {
					u32ItemData = FALSE;
				} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "click")) {
					if(pDataAcsProp->pData != NULL) {
						u32ItemData = !(*((uint32*)pDataAcsProp->pData));
					} else if(pDataAcsProp->ItemType.i8SgnDigits_u8BitNum && (pItemPageReq != NULL)) {
						u32ItemData = *((uint32*)((pBootLoaderConf->pConfProp + pItemPageReq->uConfPageTopicItemNo)->pData));
						u32ItemData = (u32ItemData>>uBitStart) & ((1<<uVarBitWidth) - 1);
						u32ItemData = !u32ItemData;
					} else {
						bGetSuc = FALSE;
					}
				} else {
					u32ItemData = (ReadU32(&pU8Instr, pU8InstrEnd) != 0);
				}
			} 

			/* λ������������ݴ洢��topic���ڵ��� */
			if(bGetSuc && (pDataAcsProp->pData == NULL) && (pItemPageReq != NULL)
				&& pDataAcsProp->ItemType.i8SgnDigits_u8BitNum
				&& ((pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32)))
			{
				pDataAcsProp = pBootLoaderConf->pConfProp + pItemPageReq->uConfPageTopicItemNo;	/* �޸�pDataAcsPropָ��Topic�� */
				
				/* ����ҳ����� */
				u32ItemData = ((*(uint32*)pDataAcsProp->pData)									/* ȡ��Topic�ı��� */
							  & ((0 - (1<<(uBitStart + uVarBitWidth))) | ((1<<uBitStart) - 1)))	/* ��ձ�����ռ�ݵ�λ�ռ� */
				 			  | ((u32ItemData & ((1<<uVarBitWidth) - 1))<<uBitStart);			/* ��񻯵�ǰ��������Ҫռ�ݳ���λ�� */
			}

			/* ���и�ֵ */
			if((!bGetSuc) || (pDataAcsProp->pData == NULL)		/* ʧ�ܣ�д���ݿ� */
				|| (((pItemPageReq != NULL)				/* ֻ��������������ͬ�����ҷ�SAVE_GRP_BRD���ݣ��ſ�д�� */
						|| (pDataAcsProp->ItemType.SaveGrp == SAVE_GRP_NUL)
						|| (pDataAcsProp->ItemType.SaveGrp == SAVE_GRP_BRD))
					&& ((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_TEXT)
					&& (pDataAcsProp->ItemType.DataType != ITEM_DAT_ENUM)))
			{
				i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
			} else {											/* ���ݲ��� */
				uint8* pU8Dat;
				switch(pDataAcsProp->ItemType.DataType) {
					case ITEM_DAT_TOPIC:
					case ITEM_DAT_U32:
					case ITEM_DAT_U32_RO:
					case ITEM_DAT_BOOL:
					case ITEM_DAT_ENUM:
						*((uint32*)pDataAcsProp->pData) = u32ItemData;
						break;
						
					case ITEM_DAT_TEMPSENCONF:
					case ITEM_DAT_ACQ_U32:
					case ITEM_DAT_D2U32:
						bGetSuc = GetU32(&pU8Instr, pU8InstrEnd, (uint32*)pDataAcsProp->pData, pDataAcsProp->ItemType.uBcdQ);
						break;
					
					case ITEM_DAT_IPv4:
						bGetSuc = GetIPv4(&pU8Instr, pU8InstrEnd, (uint32*)pDataAcsProp->pData);
						break;
					
					case ITEM_DAT_pTIME:
						bGetSuc = GetPriceTime(&pU8Instr, pU8InstrEnd, (uint32*)pDataAcsProp->pData);
						break;
					
				#if SUPPORT_IEC101 || SUPPORT_IEC104
					case ITEM_DAT_TELE_ITEM:
						bGetSuc = GetTeleSigMetConfFromText(&pU8Instr, pU8InstrEnd, (TELE_101_104_ITEM_CONF*)pDataAcsProp->pData);
						break;
				#endif
					
					case ITEM_DAT_F32:
					case ITEM_DAT_ACQ_F32:
					case ITEM_DAT_D2F32:
						bGetSuc = GetF32(&pU8Instr, pU8InstrEnd, (float32*)pDataAcsProp->pData);
						break;
					
					case ITEM_DAT_SUMM:
						bGetSuc = GetI32(&pU8Instr, pU8InstrEnd, (int32*)pDataAcsProp->pData, 0);
						InformAcqSummConfChange(pDataAcsProp->pData);
						break;
					
                    case ITEM_DAT_I64_RO:
					case ITEM_DAT_I64:
						bGetSuc = GetI64(&pU8Instr, pU8InstrEnd, (int64*)pDataAcsProp->pData, pDataAcsProp->ItemType.uBcdQ);
						break;
					
					case ITEM_DAT_RMTS:
						if(DataUser != DATA_USER_NET) {	/* �����UI����,���滻���кŲ��� */
							/* ���������ַ������� */
							pU8Dat = SkipCharInString(pU8Instr, pU8InstrEnd, 0, 1);
							i = pU8Dat - pU8Instr;
							pU8Dat--;
							if(*pU8Dat == 0) {
								i--;
							}
							/* ����ԭʼ�������кų��� */
							pU8Dat = (uint8*)pDataAcsProp->pData;
							pU8Dat = SkipCharInString(pU8Dat, pU8Dat + CONF_ANSI_BYTE_LEN, '/', 1);
							if(i == SkipCharInString(pU8Dat, (uint8*)pDataAcsProp->pData + CONF_ANSI_BYTE_LEN, '/', 1) - pU8Dat - 1)  {
								for( ; i > 0; i--) {
									*pU8Dat++ = *pU8Instr++;
								}
							} else {
								bGetSuc = FALSE;
							}
							break;
						}	/* ���򣬺�ITEM_DAT_ANSIһ������ */
					case ITEM_DAT_DIGITSTR:
					case ITEM_DAT_ANSI:
					case ITEM_DAT_ANSI_RO:
						pU8Dat = (uint8*)pDataAcsProp->pData;
						Swi_disable();
						if(DataUser == DATA_USER_NET) {
							for(i = CONF_ANSI_BYTE_LEN - 1; (i > 0) && (*pU8Instr != '"'); i--) {
								*pU8Dat++ = *pU8Instr++;
							}
						} else {
							for(i = CONF_ANSI_BYTE_LEN - 1; (i > 0) && (*pU8Instr != 0); i--) {
								*pU8Dat++ = *pU8Instr++;
							}
						}
						*pU8Dat++ = 0;						
						Swi_enable();
						bGetSuc = (i != 0);		/* ˵��Buf���� */
						break;

					case ITEM_DAT_T64_RO:
						Swi_disable();
						bGetSuc = GetT64(DataUser, &pU8Instr, pU8InstrEnd, (REAL_TIME_VAR*)pDataAcsProp->pData);
						Swi_enable();
						break;

					case ITEM_DAT_T32_RO:
						bGetSuc = GetT32(&pU8Instr, pU8InstrEnd, (uint32*)pDataAcsProp->pData);
						break;

					default:
						bGetSuc = FALSE;
						break;
				}
				
				if(bGetSuc) {
					i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = 5;
				} else {
					i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
				}
			}
		}

		/* �޸�ָ��ָ�룬ָ��ǰָ���β */
		if(DataUser == DATA_USER_NET) {
			/* ָ�����������ʽ: "name":"1","name":["3,4"],"name":["3","4"] */
			for(i = pU8InstrEnd - pU8Instr; (i > 0) && (*pU8Instr != ','); i--) {	/* ����',' [�������������ֶ���,1:����֮�䣬2:D2�������] */
				pU8Instr++;
			}
			/* ����ָ�룬���������ý�β */
			if((pU8Instr[1] == '"') && ((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_TWOBOX)) {	/* "name":["3","4"]��� */
				pU8Instr += 2;
			} else {	/* "name":"1","name":["3,4"]��� */
				pU8Instr++;
			}
		} else {
			for(i = pU8InstrEnd - pU8Instr; (i > 0) && (*pU8Instr != 0); i--) {
				pU8Instr++;
			}
			pU8Instr++;	/* ����ָ�룬���������ý�β */
		}
	}

	/* �Ƿ�ڶ��������� */
	if((pU8Instr < pU8InstrEnd) && (*pU8Instr != 0) && (*pU8Instr != '"')) {
		BOOL bGetSuc = FALSE;
		switch(pDataAcsProp->ItemType.DataType) {
			case ITEM_DAT_D2U32:
				bGetSuc = GetU32(&pU8Instr, pU8InstrEnd, (uint32*)pDataAcsProp->pData + 1, pDataAcsProp->ItemType.uBcdQ);
				break;

			case ITEM_DAT_SUMM:				
				bGetSuc = GetF32(&pU8Instr, pU8InstrEnd, (float32*)pDataAcsProp->pData + 1);
				InformAcqSummConfChange(pDataAcsProp->pData);
				break;
				
			case ITEM_DAT_D2F32:
				bGetSuc = GetF32(&pU8Instr, pU8InstrEnd, (float32*)pDataAcsProp->pData + 1);
				break;
			
			case ITEM_DAT_pTIME:
				bGetSuc = GetPriceTime(&pU8Instr, pU8InstrEnd, (uint32*)pDataAcsProp->pData + 1);
				break;

			default:
				break;
		}

		if(!bGetSuc) {
			i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
		} else if(i8ST_Idle0_SucN1_FailN2_MaxWaitCntP == 0) {	/* ǰ��û��ʧ�ܣ������ֳɹ��� */
			i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = 5;
		}
	}

	/* ������� */
 /* �洢����:���ڴ洢����(tf��/�ⲿI2C�ӿڵ�EEPROM��)�������٣�Ϊ�˷�ֹͨѶ����ʱ����Ҫ�Ѵ洢���ֶ�����һ������
	ͨѶ����ʹ洢�����ͨ��"����+�ź���"���д��� */
	if((i8ST_Idle0_SucN1_FailN2_MaxWaitCntP > 0) && (pDataAcsProp->ItemType.SaveGrp != SAVE_GRP_NUL)) {
		g_DataAcsIntf.bConfHaveUnsavedChange[pDataAcsProp->ItemType.SaveGrp] = TRUE;
		g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = 5;		/* ���޸Ĳ���������Ҫ����ͬ��(����)���� */
		Semaphore_post(g_DataAccessBlock.Sem_Op);		//Semaphore_post(SEM_NvMemAcs);
	}

	if(i8ST_Idle0_SucN1_FailN2_MaxWaitCntP == 0) {		/* ָ�����κβ��� */
		return -2;
	} else {
		*ppU8Instr = pU8Instr;
		return i8ST_Idle0_SucN1_FailN2_MaxWaitCntP;
	}
}

/* GUI����ҳ�淵�� */
void DealGuiConfWinBack(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq)
{
	uint16 uConfID = pBootLoaderConf->pConfProp[pItemPageReq->uConfPageTopicItemNo].u32ItemID;
	int16 i;

	CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
	for(i = pBootLoaderConf->u32ItemNum; i > 0; i--) {	/* ������ת�����е�LINK����S32 */
		if(((pDataAcsProp->ItemType.DataType == ITEM_DAT_LINK) || (pDataAcsProp->ItemType.DataType == ITEM_DAT_S32))
			&& (pDataAcsProp->LkId_Val_ItN == uConfID))
		{
			break;
		}
		pDataAcsProp++;
	}
	pDataAcsProp--;
	/* ��������ҳ���Topic */
	for(i = pDataAcsProp - pBootLoaderConf->pConfProp; (i >= 0) && (pDataAcsProp->ItemType.DataType != ITEM_DAT_TOPIC); i--) {
		pDataAcsProp--;
	}
	pItemPageReq->uConfPageTopicItemNo = pDataAcsProp - pBootLoaderConf->pConfProp;
	pItemPageReq->uConfWinFirstLineItemNo = pItemPageReq->uConfPageTopicItemNo + 1;
	if(DataUser == DATA_USER_MCGS) {
		pItemPageReq->u32ItemNo = pItemPageReq->uConfPageTopicItemNo;
	}
}

/* GUI����ҳ���Ϸ�ҳ: ��ҳ��ʱ�������ö��������һҳ���� */
void DealGuiConfWinUp(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq)
{
	CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp + pItemPageReq->uConfWinFirstLineItemNo;
	int16 i;
	BOOL bEnumCmplt = FALSE;		/* ���γ�������ö���б� */
	
	/* �����ǰ���ڵ�һ����ECONT����˵��ǰ����һ��ENUM����Ҫ��������ǰ���ڴӵ�һ�п�ʼ���������һ��ECONT
	   Ȼ����ǰ��ENUM, ����ܰѵ�ǰ"�ض�"��ö���б����һ��ҳ�����棬�������һ��ECONT���ڽ���ʾ���ڵ�ĩβ
	   ���򣬾Ͳ�����ά��ö���б�������� */
	if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ECONT) {
		pDataAcsProp++;
		for(i = 5; i > 0; i--) {	/* ������ǰ���ڴӵ�һ�п�ʼ���������һ��ECONT */
			if(pDataAcsProp->ItemType.DataType != ITEM_DAT_ECONT) {
				pDataAcsProp--;
				break;
			}
			pDataAcsProp++;
		}
		if(i) { 	/* ˵����ǰ�������һ�в���ECONT, ���γ�����ö���б�Ŀ����� */
			for(i = 2; i <= 6; i++) {	/* ��ǰָ�����һ��ECONT, ��ǰ����ENUM��i���ڵ�ǰ�б���Ŀ */
				pDataAcsProp--;
				if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM) {
					break;
				}
			}
			if(i <= 6) {	/* (6 - i)��ö���б�ǰ�滹���Է��ö��ٸ���Ŀ */
				pItemPageReq->uConfWinFirstLineItemNo = pDataAcsProp - pBootLoaderConf->pConfProp - (6 - i);
				bEnumCmplt = TRUE;
			}
		}
	} 
	if(!bEnumCmplt) {	/* �������γ�������ö���б� */
		i = pItemPageReq->uConfWinFirstLineItemNo - 6;
		if(i < (int16)(pItemPageReq->uConfPageTopicItemNo + 1)) {		/* ��ֹǰ�������ֵ */
			pItemPageReq->uConfWinFirstLineItemNo = pItemPageReq->uConfPageTopicItemNo + 1;
		} else {
			pItemPageReq->uConfWinFirstLineItemNo = i;
		}
	}
	if(DataUser == DATA_USER_MCGS) {
		pItemPageReq->u32ItemNo = pItemPageReq->uConfPageTopicItemNo;
	}
}

/* GUI����ҳ���·�ҳ: ��ҳ��ʱ�������ö��������һҳ���� */
void DealGuiConfWinDown(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq)
{
	CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp + pItemPageReq->uConfPageTopicItemNo;

	/* ���㵽��β��Ҫ�������� */
	int16 i = pItemPageReq->uConfPageTopicItemNo + pDataAcsProp->LkId_Val_ItN + 1 - (pItemPageReq->uConfWinFirstLineItemNo + 6);
	if(i > 6) { 	/* һ�η�ҳ��಻����6 */
		i = 6;
	}
	if(i > 0) {
		/* δ������ö������ */
		if(pBootLoaderConf->pConfProp[pItemPageReq->uConfWinFirstLineItemNo + 6].ItemType.DataType == ITEM_DAT_ECONT) {
			pDataAcsProp = pBootLoaderConf->pConfProp + pItemPageReq->uConfWinFirstLineItemNo + 1;
			while((i > 0) && (pDataAcsProp->ItemType.DataType != ITEM_DAT_ENUM)) {
				pDataAcsProp++;
				i--;
			}
			pItemPageReq->uConfWinFirstLineItemNo = pDataAcsProp - pBootLoaderConf->pConfProp;
		} else {
			pItemPageReq->uConfWinFirstLineItemNo += i;
		}
	}
	if(DataUser == DATA_USER_MCGS) {
		pItemPageReq->u32ItemNo = pItemPageReq->uConfPageTopicItemNo;
	}
}

/******************************** FILE END ********************************/
