/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved.	ģ��V1.4
 *				: 
 * File			: 
 * Author		: Wang Renfei
 * Description	: 

2017.3.24 δ�������
1. ���룬�����Ҫ����Ļ�����޲�bug�ſ��Խ�һ�����¹���
2. ����ҳ��������ҳ��8����ֵ��������ʾ��û�л���״̬���ı�ö���ͱ�������ʾ
3. �����ҳ����: ������ϵͳ���߼�����������ҳ��
4. �������ItemPageҳ�����ʾ����������ȱ�����룬����ҳ��ȱ����ת���ܣ������һ������
5. ȱ��CRC����
6. ViewTech��Ļ������ʾ�ؼ���ַ�Ĺ滮

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
#ifndef _MDL_GUI_H_
#define _MDL_GUI_H_

/***************************************************************************
 							include files
***************************************************************************/
#include "MdlNet.h"


/***************************************************************************
				hardware parameter and global definitions
***************************************************************************/
/* <NULL> */


/***************************************************************************
 		use EXT to distinguish stencil.c or others source files
***************************************************************************/
#ifdef _MDL_GUI_C_
#define EXT
#else
#define EXT extern
#endif


/***************************************************************************
 					variables could be used as extern
***************************************************************************/
#if SUPPORT_SCREEN
/* VT��ĻͨѶ�ӿ� */
typedef enum{
	/* ��ͨѶ�ӿ� */
	VT_COMM_VER_DEFAULT			= 0,
	VT_COMM_VER_OLD1		    = 1,			/* YKF2\3 16· */
	VT_COMM_VER_OLD2		    = 2,			/* YKF2\3 16· */
	VT_COMM_VER_OLD3		    = 3,			/* YKF2\3 16·, YBU3, YYT3 */
	VT_VAL_DEV_SDW043			= 4,			/* ViewTech YYD4.3���� */
	VT_COMM_VER_MERGE			= 5,			/* ViewTech VT�ϲ���Ļ��YKF4\5 10���� */
	MAX_VT_COMM_VER
}VT_COMM_VER;
#endif

typedef struct {
	/* ����������ҳ��������ά�� */
	DATA_PAGE DataPage;							/* �ñ�������MCGS��ViewTech ҳ�� */
	LANG_TYPE Language;							/* �ñ�������MQTT����ҳ�� */
	/* ����ҳ�� */
	uint16 uConfPageTopicItemNo;				/* ����ҳ������ItemNo */
	uint16 uConfWinFirstLineItemNo;				/* ��ǰ��ʾ���ڵ�һ��ItemNo */

#if SUPPORT_SCREEN
	/* ViewTech��Ļ */
	uint8 u8PageRefresh1Hz_100ms;				/* ҳ��1Hzˢ�¶�ʱ�� */
	uint8 u8SlowRefresh_1Hz;					/* ������ˢ��ʱ��*/
	VT_COMM_VER VTCommVer;						/* ��VT��ͨѶ�ͺ�*/
	DATA_PAGE ExtPage;                          /* ��չ��Ļҳ�� */
	uint8 u8ExtItemNo;							/* ��չ��Ļ��Ŀ���(Ŀǰ��Ҫ�����¶����) */
#endif

	/* Mcgs��Ļ��ǰ����ʾ����; VT��Ļ����ҳ�������޸ĵ���; VT��Ļ����ҳ����Ŀ(�����¶����);  */
	BOOL bMsgAcqPageWinDown;					/* Msg,Acqҳ�����ҳ������ż��� */
	uint32 u32ItemNo;							/* ҳ���ʼ��ʱ���Ὣ�ñ�����ʼ����0xFFFFFFFFUL */

	/*��������״̬*/
	uint8 u8InputingType;						/* ����λ�ã��Ƿ�˫������һ������ */
	BOOL bUnprocessedInput;

	/* ���� */
	uint8 u8Tmr_OffScn_1s;						/* ����Ϩ�� */
	uint8 u8Tmr_ExtOffScn_1s;					/* ��չ��ĻϨ�� */
}ITEM_PAGE_REQ;

#if SUPPORT_SCREEN
/* Viewtech��ǰ׺��������  *
 * VT_DEV������Ļ�豸��ַ
 * VT_REG����Ĵ���(REG)��ַ  *
 * VT_VAR�������(VAR)��ַ  *
 * VT_VAL����ȡֵ
 * VT_LEN�����ֽڳ���
 * VT_BUF����������ͨѶBuf�е�λ��
 * VT_PAGE��VT��Ļҳ����
 */

/* �豸�ͺ� */
#define VT_VAR_DEVICE_TYPE						0x0000		/* ViewTech��Ļ�ͺ� */

#define VT_DEV_MON 	                            0xA55A      /* ����Ļ��ַ */
#define VT_DEV_EXT	                            0x5AA5      /* ��չ��Ļ��ַ */
#define VT_REG_PIC_ID							0x03		/* VIEWTECH ���Ƶ�ǰ��ʾҳ�ļĴ��� */
#define VT_REG_LED_CTL							0x01		/* VIEWTECH ���ȿ��ƼĴ��� */
#define VT_REG_TOUCH_STATUS						0x06		/* VIEWTECH ��ȡ����״̬�Ĵ��� */

#define VT_VAL_PNLKEY_MAX_PAGE_VAL				0x3000		/* ��Ļ����PNLKEY��ʽ������ҳ���������+1 */
#define VT_VAL_PNLKEY_STATUS_BTN_MIN_VAL		0x3001		/* ��Ļ����PNLKEY��ʽ������״̬��ťֵ����С���ܣ����� */
#define VT_VAL_PNLKEY_STATUS_BTN_MAX_VAL		0x301F		/* ��Ļ����PNLKEY��ʽ������״̬��ťֵ�������ܣ����� */

#define VT_BUF_RDREG							0x06		/* VT��ȡ��Ļ�Ĵ���ָ�������λ�� */
#define VT_BUF_RDVAR							0x07		/* VT��ȡ��Ļ�ڴ�ָ�������λ�� */

#define VT_REG_INPUT_STATUS						0xE9		/* VT�Ƿ�������״̬��VT���û����룩 */
#define VT_REG_KEYBOARD_CTL						0x4F		/* �������̺��е�ַ */

/* ��Ļ��ť */
#define VT_VAR_PNLKEY							0xF100		/* ��������   */
#define VT_VAR_KEYBOARD_STATUS					0xF000		/* VT��������״̬�����ֽ�ֵ��0x5A�������������ǣ�����ʾ���������0x00����ʾ����Һ���������������״̬�С� */
#define VT_VAR_KEYBOARD_INPUT					0xF001		/* VT������������  */
#define VT_VAL_KEYBOARD_ASCII					20			/* VT����Ӣ�ļ��̺��б�� */
#define VT_VAL_KEYBOARD_NUM						10			/* VT�������ּ��̺��б�� */
#define VT_VAL_KEYBOARD_PWD						11			/* VT����������̺��б�� */

/* ��ͬ��Ʒ�߼�ҳ��(DataPage)��VT��ĻPage֮���ӳ���ϵ��ҳ���л���ť��ֵ�Ǵ��л�ҳ��ID */
#define VT_PAGE_DEFAULT							00			/* ͨ��ҳ�� : Ĭ��ҳ��  */
#define VT_PAGE_CONFIG							10			/* ͨ��ҳ�� : ����  */
#define VT_PAGE_MSGoACQ							12			/* ͨ��ҳ�� : ��Ϣ  */
#define VT_PAGE_BTN_ACQ 						14			/* ע�⣺����ǲɼ�ҳ�水ť����ʾ��ʱ����VT_PAGE_MSGoACQ */
#ifdef _MDL_GUI_C_
const uint16 cnst_uDataPage2VTPageBtn[MAX_DATA_PAGE]
 = {00, 10, 12, 14,                                         /* ��ӭҳ�桢����ҳ�桢��Ϣҳ�桢�ɼ�ҳ�� */
     01, 50, 52, 56,                                        /* ��ӭҳ�桢��ҳ�桢����ҳ�桢ϵͳҳ�� */
     54, 58, 60};                                           /* ����ҳ�桢��ͨ���¶ȡ���ͨ���¶� */
#endif
EXT const uint16 cnst_uDataPage2VTPage[][MAX_DATA_PAGE]       /* ������������ģ��Ƿ�ͨ��ҳ�棬��Ҫ��VT��Ŀ����һ�� */
#ifndef _MDL_GUI_C_
;
#elif Is_YKF(DEVICE_TYPE)
= {{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* ��ӭҳ�桢����ҳ�桢��Ϣҳ�桢�ɼ�ҳ�� */
		01, 50, 52, 56,                                                     /* ��ӭҳ�桢��ҳ�桢����ҳ�桢ϵͳҳ�� */
		54, 58, 60},                                                      	/* ����ҳ�桢��ͨ���¶ȡ���ͨ���¶� */
	/* V5_YKF2 �¶�24· */
	{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ, 	/* ��ӭҳ�桢����ҳ�桢��Ϣҳ�桢�ɼ�ҳ�� */
		01, 50, 90, 92,                                                     /* ��ӭҳ�桢��ҳ�桢����ҳ�桢ϵͳҳ�� */
		54, 58, 60}                                                        	/* ����ҳ�桢��ͨ���¶ȡ���ͨ���¶� */
};
#elif((DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4))
= {{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* ��ҳ�桢����ҳ�桢��Ϣҳ�桢�ɼ�ҳ�� */
		01, 50, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, 							/* ��ӭҳ�桢��ҳ�桢����ҳ�桢ϵͳҳ�� */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT},                 /* ����ҳ�桢��ͨ���¶ȡ���ͨ���¶� */
	/* V5_YBU�ϲ���Ļ���ҳ�� */
	{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ, 				/* ��ҳ�桢����ҳ�桢��Ϣҳ�桢�ɼ�ҳ�� */
		01, 80, 80, VT_PAGE_DEFAULT, 							/* ��ӭҳ�桢��ҳ�桢����ҳ�桢ϵͳҳ�� */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT}                  /* ����ҳ�桢��ͨ���¶ȡ���ͨ���¶� */
};
#elif(DEVICE_TYPE == V5_YYG)
= {{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* ��ӭҳ�桢����ҳ�桢��Ϣҳ�桢�ɼ�ҳ�� */
		01, 50, 52, 56,                                                     /* ��ӭҳ�桢��ҳ�桢����ҳ�桢ϵͳҳ�� */
		54, 58, 60},                                                      	/* ����ҳ�桢��ͨ���¶ȡ���ͨ���¶� */
		/* V5_YYG */
	{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* ��ҳ�桢����ҳ�桢��Ϣҳ�桢�ɼ�ҳ�� */
		01, 62, 64, 56, 													/* ��ӭҳ�桢��ҳ�桢����ҳ�桢ϵͳҳ�� */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT}            		/* ����ҳ�桢��ͨ���¶ȡ���ͨ���¶� */
};
#elif((DEVICE_TYPE == V5_YYD) || (DEVICE_TYPE == V5_YYD2))
= {{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,  	/* ��ӭҳ�桢����ҳ�桢��Ϣҳ�桢�ɼ�ҳ�� */
		01, 50, 52, VT_PAGE_DEFAULT,                                        /* ��ӭҳ�桢��ҳ�桢����ҳ�桢ϵͳҳ�� */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT},                 /* ����ҳ�桢��ͨ���¶ȡ���ͨ���¶� */
	/* V5_YYD 7�� */
	{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ, 	/* ��ҳ�桢����ҳ�桢��Ϣҳ�桢�ɼ�ҳ�� */
		01, 70, 72, VT_PAGE_DEFAULT, 										/* ��ӭҳ�桢��ҳ�桢����ҳ�桢ϵͳҳ�� */
		54, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT}               				/* ����ҳ�桢��ͨ���¶ȡ���ͨ���¶� */
};
#elif(DEVICE_TYPE == V5_YYT3) || (DEVICE_TYPE == V5_YYT4)
= {{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* ��ҳ�桢����ҳ�桢��Ϣҳ�桢�ɼ�ҳ�� */
		01, 50, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, 							/* ��ӭҳ�桢��ҳ�桢����ҳ�桢ϵͳҳ�� */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT},                 /* ����ҳ�桢��ͨ���¶ȡ���ͨ���¶� */
	{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* ��ҳ�桢����ҳ�桢��Ϣҳ�桢�ɼ�ҳ�� */
		01, 50, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, 							/* ��ӭҳ�桢��ҳ�桢����ҳ�桢ϵͳҳ�� */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT}                  /* ����ҳ�桢��ͨ���¶ȡ���ͨ���¶� */
};
#else
= {{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* ��ҳ�桢����ҳ�桢��Ϣҳ�桢�ɼ�ҳ�� */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT,	/* ��ӭҳ�桢��ҳ�桢����ҳ�桢ϵͳҳ�� */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT},	                /* ����ҳ�桢��ͨ���¶ȡ���ͨ���¶� */
	{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* ��ҳ�桢����ҳ�桢��Ϣҳ�桢�ɼ�ҳ�� */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT,	/* ��ӭҳ�桢��ҳ�桢����ҳ�桢ϵͳҳ�� */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT}	                /* ����ҳ�桢��ͨ���¶ȡ���ͨ���¶� */
};
#endif

/*VIEW_TECH�Ĵ���������������*/
typedef enum {
	VIEW_TECH_WRITE_REG		= 0x80,					/*����д��ַ��Χ��0-0xFF�ļĴ��� */
	VIEW_TECH_READ_REG		= 0x81,					/*���ڶ���ַ��Χ��0-0xFF�ļĴ��� */
	VIEW_TECH_WRITE_VAR		= 0x82,					/*����д��ַ��Χ��0-0xFFFF�ı��� */
	VIEW_TECH_READ_VAR		= 0x83,					/*���ڶ���ַ��Χ��0-0xFFFF�ı��� */
}VIEW_TECH_CMD;

#define VT_LEN_ITEM_CHAR				62			/* ViewTech��Ļ��ItemPage�����ַ�(Ӣ��)���ȣ�������ż�� */
#define VT_LEN_ITEM_BYTE	(VT_LEN_ITEM_CHAR + 2)	/* ViewTech��Ļ��ItemPage����buf���Ȼ����2��byte    ����MsgID��AcqID */
#define MCGS_LEN_ITEM_CHAR				60			/* MCGS��Ļ��ItemPage�����ַ�(Ӣ��)���ȣ�������ż�� */

#define MCGS_ITEM_PAGE_LINE_WLEN		(MCGS_LEN_ITEM_CHAR + 6)/2
#define MCGS_ITEM_PAGE_ADDI_BLEN		10				/* ������Ļ���Ҫ���ֱ��������16bitΪ��λ�����һ��ʼ���ɵ����ݱ��뾭��������������Ļ�������Ҫ����Ҫ�����ram�������� */
#define MCGS_ITEM_PAGE_ACK_MAX_BLEN		(MCGS_LEN_ITEM_CHAR + 6 + MCGS_ITEM_PAGE_ADDI_BLEN)
#define MCGS_ITEM_PAGE_TYPE_BSTART		0
#define MCGS_ITEM_PAGE_PAYLOAD_BSTART	2
#define MCGS_ITEM_PAGE_ADDIPAYLOAD_BSTART	(MCGS_ITEM_PAGE_PAYLOAD_BSTART + MCGS_ITEM_PAGE_ADDI_BLEN)
#define MCGS_ITEM_PAGE_ITEMNO_BSTART	(2 + MCGS_LEN_ITEM_CHAR)
typedef struct {									/* MCGSҳ���� ItemPageAck ����Item���ݽṹ, ��ͬ�����µ�ϸ�½ṹ�μ���Ӧ�ĵ� */
	uint16 uItemType;								/* ָ�������PayLoad�����õľ���ҳ��Item�ṹ,��λ�ڳ�ʼ�� */
	uint16 uPayLoad[MCGS_LEN_ITEM_CHAR/2]; 			/* ���ݽṹ֮���ز��֣����ڴ洢������Ҫ��ʾ������ */
	uint32 u32ItemNo; 								/* ��Page��λ����Ҫ���� */
}MCGS_ITEM_PAGE_ACK;

typedef struct {									/* ViewTech��Ļ MsgAcqҳ��һ�е����ݽṹ */
	uint8 u8Text[VT_LEN_ITEM_CHAR];
	uint16 uMsgAcqID;
}VIEW_TECH_MSG_ACQ_ITEM;

#define VT_VAL_COLOR_RED										0xF800
#define VT_VAL_COLOR_YELLOW										0xFFE0

#define SetMsgAcqLineColor(u8UartPort, uLineColorAddr, uColor) do{\
	uint8 *pTRBuf = &g_UartComm[u8UartPort].u8TRBuf[4];	\
	*pTRBuf++ = uLineColorAddr/0x100;				\
	*pTRBuf++ = uLineColorAddr&0xFF;				\
	*pTRBuf++ = uColor/0x100;                       \
	*pTRBuf++ = uColor%0x100;                       \
	SendData_VT(u8UartPort, pTRBuf);				\
}while(0);
#endif

/* MQTT conf/msg/acq���� */
#define MQTT_ITEM_PROC_BUF_BLEN     (MQTT_TR_BUF_BLEN - 100)
typedef struct {
	/* ����ͬ��(���ݡ�����)���� */
	uint16 uStartConfID;							 /* ��ʼ����ID */
	uint16 uEndConfID;								 /* ��������ID */
	int8 i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP;	 /* ���ݡ�����״̬--0:����;-1:�ɹ�;-2:ʧ��;>0:���Ŭ������ */

	/* ����ҳ����� */
	int8 i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP;	/* д����״̬--0:����;-1:�ɹ�;-2:ʧ��;>0:���Ŭ������ */
	uint16 uPageOpItemNo;							/* 0xFFFF�����ʼ��ҳ�棬��ʵ�к�ȡ0 */
	ITEM_PAGE_REQ ItemPageReq;		/* ProcItemPageInstr()�������õı�׼�ṹ�� */

    /* �Ѿ����������ݿ��MsgNo */
    uint32 u32MsgNo_HavePubedForDB;

    /* msg��acqҳ����� */
	uint32 u32ItemNo_InStart;
	uint32 u32ItemNo_OutEnd;
	int16 iItemNum_InNeed;
	int16 iItemNum_OutSuc;
    int8 i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage;
    DATA_ACS_OPR DataAcsRes;
    uint16 uContLen;
	uint8 u8Buf[MQTT_ITEM_PROC_BUF_BLEN];                  /* DataAccess������д����Buf */
}MQTT_ITEM_PROC;
EXT MQTT_ITEM_PROC g_MqttItemProc;

/*===========================================================================
 * Input Var
 *==========================================================================*/
/* <NULL> */

/*===========================================================================
 * Output Var
 *==========================================================================*/
/* <NULL> */

/*===========================================================================
 * Conf Var
 *==========================================================================*/
/*---------------------------------------------------------------------------
 * ���ñ���
 *--------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * ���ñ�������
 *--------------------------------------------------------------------------*/
/* <NULL> */

/*---------------------------------------------------------------------------
 * ���õ�½ҳ��
 *--------------------------------------------------------------------------*/
/* <NULL> */

/*---------------------------------------------------------------------------
 * ���ýṹ��
 *--------------------------------------------------------------------------*/
/* <NULL> */

/***************************************************************************
 					functions must be driven by extern
***************************************************************************/
#if SUPPORT_SCREEN
EXT	void TryViewTech(uint8 u8UartPort);
EXT	void RunViewTech(uint8 u8UartPort);

/***************************************************************************
 					functions could be used as extern
***************************************************************************/
/* MCGS��Ļ���� */
EXT void FillItemPageAck_MCGS(ITEM_PAGE_REQ* pItemPageReq, uint8* pMCGSItemPageAck);

/* ViewTech��Ļ���� */
EXT BOOL ReadData_VT(uint8 u8UartPort, uint16 uDevAddr, VIEW_TECH_CMD ViewTechCmd, uint16 uDataAddr, uint16 uWordLen);
EXT void WriteVar_VT(uint8 u8UartPort, uint16 uDevAddr, uint16 uDataAddr, uint8 *pValue, uint8 u8WordLen);
EXT void WriteReg_VT(uint8 u8UartPort, uint16 uDevAddr, uint16 uDataAddr, uint16 uValue, uint8 u8ByteLen);
EXT void ClearVar_VT(uint8 u8UartPort, uint16 uDevAddr, uint16 uDataAddr, uint8 u8WordLenToClear);
EXT void SendData_VT(uint8 u8UartPort, uint8* pTxBuf);
EXT uint16 DataPage2VTPage(VT_COMM_VER VTCommVer, DATA_PAGE DataPage);
EXT void ChangePage_VT(ITEM_PAGE_REQ *pItemPageReq, uint16 uVTPage);
EXT uint8 PrintStringFromText_VT(uint8 *pDstBuf, const char *pSrcText, uint8 u8MaxLen);
EXT uint8* PrintTemperatureName(uint8* pText, TEMPERATURE_DATA* pTempData, char TailChar);
EXT uint8* PrintTemperatureValue(uint8* pText, TEMPERATURE_DATA* pTempData, const char* pDiscText);
#endif

/* mqtt���� CONF_n_STAT_PROP��ʽ �ӿ� */
EXT void ProcMqttConfInstr(uint8* pU8Msg, uint8* pU8MsgEnd);
EXT void ProcMqttConfSync(uint8* pU8Msg, uint8* pU8MsgEnd);
EXT BOOL QueryAndPubMqttConfResAndPage(MQTT_COMM* pMqttComm);

/* ��ӡCONF_n_STAT_PROP��ʽ���ñ���������ItemPageָ�� */
EXT uint8* PrintLocalConfItem(DATA_USER DataUser, CONF_n_STAT_PROP* pDataAcsProp, uint8* pBuf, uint16 uTextByteLen);
EXT int8 ProcItemPageInstr(DATA_USER DataUser, DATA_PAGE DataPage, uint32 u32ItemNo, uint8** ppU8Instr, uint8* pU8InstrEnd, ITEM_PAGE_REQ* pItemPageReq);

/************   exclude redefinition and c++ environment   ****************/

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
/******************************** FILE END ********************************/
