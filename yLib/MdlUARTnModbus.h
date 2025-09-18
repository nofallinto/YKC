/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved.	ģ��V1.3
 *				: 
 * File			: stencil.h
 * Author		: Wang Renfei
 * Description	: 
 * Date			: 2009-5-31
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 **************************************************************************/

/***************************************************************************
 				exclude redefinition and c++ environment
***************************************************************************/
#ifndef _MDL_RS485_n_MODBUS_H_
#define _MDL_RS485_n_MODBUS_H_

/***************************************************************************
 							include files
***************************************************************************/
#include "BoardSupport.h"

#include "GlobalVar.h"
#include "MdlDataAccess.h"
#include "MdlGUI.h"


/***************************************************************************
				hardware parameter and global definitions
***************************************************************************/
/* <NULL> */


/***************************************************************************
 		use EXT to distinguish stencil.c or others source files
***************************************************************************/
#ifdef _MDL_RS485_n_MODBUS_C_
#define EXT
#else
#define EXT extern
#endif


/***************************************************************************
 					variables could be used as extern
***************************************************************************/
/*===========================================================================
 * RS485ͨѶ����
 *==========================================================================*/
typedef enum {
	RS485_APP_NULL 			= 0,			/* �� */
	RS485_APP_AUTO			= 1,			/* �������Զ�����ÿһ��APP���ܽ��г��ԣ�������Գɹ��򱣳����ֹ��ܷ�ʽ */
	UART_VIEW_TECH			= 2,			/* ViewTech��Ļ���� */
	RS485_MCGS_RTU_SLAVE	= 3,			/* MCGS��Ļ�ӿ�(modbus�ӻ������а�38.4kbps�����԰�115.2kbps) */
	RS485_YKFR1_XMS_9600	= 4,			/* ��������YKF������R1, XMG, XMS, ELE(ֱ����), RCX(��·���ѹ������) */
	RS485_YKFR2_38400		= 5,			/* ��������YKF������R2 */
	RS485_SOFT_UPDATE		= 6,			/* ���ڴ����а滻�ز��԰棬����Զ��������ͨ��RS485�߶Կ����� */
	RS485_V5YKF_PERIPHERAL	= 7,			/* YKF23���裺(�¶Ȳɼ�ģ��\ѹ��������)��ͨѶ������9600 */

	RS485_MODBUS_RTU_SLAVE	= 21,			/* ��׼ModbusRTU�ӻ������ʡ���ַ����, �ӻ�������� */
	RS485_DATA_RADIO_SUB	= 22,			/* ������̨վ���� */
	RS485_IEC101			= 23,			/* IEC101��Լ */
	RS485_FLOW_METER		= 24,			/* �����ƽӿ� */
	RS485_BEIDOU_TERM		= 25,			/* �����ն� */
	RS485_BEIDOU_HUB		= 26,			/* ����Hub */
	RS485_DEBUG_PACK		= 27,			/* ����DebugPack���� */
	RS485_TRIPHASE_METER	= 28,			/* ����������ܱ� */
	/* ���ڷ�����������modbusTCPתmodbusRTU�����豸��ַ������: 2(ˮλ�Ǳ��ַ),10*N(YKF��ַ),10*N+3(�¶�Ѳ���ǵ�ַ),10*N+7(��ѹ������װ�õ�ַ) */
	RS485_SERIAL_SERVER		= 29,
	RS485_APP_GNLMODBUS_MASTER = 30,		/* ͨ��MODBUS����Դ�豸(���ʺϷ����Զ�) */
	RS485_WATERLV_PGA460 	= 32,			/* PGA460������ˮλ�� */
	RS485_EXT_DIN_RELAY		= 33,			/* ��չ���뿪��ģ��*/
	RS485_AIR_COMPRESSOR	= 34,			/* ��ѹ�� */
	UART_GPS				= 35,			/* GPS */
	UART_WATERLVL_OPT_TX	= 36,			/* ����ˮλ����*/
	UART_WATERLVL_OPT_RX	= 37,			/* ����ˮλ����*/
	UART_APP_MAX
}UART_APP;

#define UART_TR_BUF_BLEN	(255 + 9)		/* �����0x10ָ�� ͷ(7byte)+����(255byte)+CRC(2byte) */
typedef struct {
    UART_Handle Handle;
    void* Sem_TxCplt;
    void* Sem_RxGet;
    
	UART_APP UartApp;						/* RS485Ӧ�� */
	UART_APP LastUartApp;					/* ��һ�δ�RS485ʱ��Ӧ�� */
	BOOL bUartFuncTrySuc;					/* ���Գɹ� */
	uint8 u8RS485AppAutoTablePt;			/* RS485�Զ�Ӧ���� cnst_RS485AppAutoTable[] ����ָ�� */
	uint16 uLastBaud_100bps;				/* ��һ�δ�RS485ʱ��ͨѶ������ */
	uint16 uTROneByteTime_us;				/* ����һ���ֽ������ʱ��,���ڼ���uMaxTransferTmr_ms */

	/* ͨѶ���� */
	uint16 uTimer_WatchDog_ms;				/* ͨѶ���Ź���ʱ����������������޴ӻ���Ӧ������Ǵӻ�����CRC��ȷ������ָ��(��һ����������) */
	uint16 uTmr_Run_ms;						/* ����ͨѶ����ʱ����ʹ��ͨѶ���񱣳̶ֹ���Ƶ�� */
    uint16 uTime_RxMaxIntv_ms;				/* ֡���ֽڼ������ʱ�� */
	uint16 uRxBufPt;
	uint16 uRxFrameIndex;					/* GPRSר��, ��һ֡����û��������ʱ, ��������������������, ��������Ӧ��������0 */

	/* DMAͨ��ģʽר�� */


	/* ͨѶBuf����:32b���� */
	uint8 u8TRBuf[UART_TR_BUF_BLEN];

	/* ItemPage���󣬼���mcgs,viewtech ���ڴ���conf,msg,acq */
	ITEM_PAGE_REQ ItemPageReq;
}UART_COMM;
EXT SECTION(".NOT_ZeroInit") UART_COMM g_UartComm[MAX_UART_NUM];

typedef enum {
	MOD_WR_05B,			/* 05ָ��д  д������Ȧֵ */
	MOD_WR_06W,			/* 06ָ��д  д�����Ĵ���ֵ,16bit���,��2-1˳�� */
	MOD_WR_0FB,			/* 0Fָ��д  д�����Ȧֵ */
	MOD_WR_10B,			/* 10ָ��д  д����Ĵ���ֵ,����ע�ֽ�˳�򣬹�˾�ڲ��ɶ�ʹ�� */
	MOD_WR_10W,			/* 10ָ��д  д����Ĵ���ֵ,16bit���,��2-1˳�� */
	MOD_WR_10D,			/* 10ָ��д  д����Ĵ���ֵ,��4-3-2-1˳��, ����ȫ���� */
	MOD_WR_10D2143,		/* 10ָ��д  д����Ĵ���ֵ,TODO:��2-1-4-3˳��,��ʱ��֧�� */

	MOD_RD_01B, 		/* 01ָ���, 1bit��� */
	MOD_RD_02B,			/* 02ָ���, 1bit��� */
	MOD_RD_03B, 		/* 03ָ���, �������ֽ�˳�� */
	MOD_RD_03W,			/* 03ָ���, 16bit���,��2-1˳�� */
	MOD_RD_03D,			/* 03ָ���, 32bit���,��4-3-2-1˳��, ����ȫ���� */
	MOD_RD_03L, 		/* 03ָ���, 64bit���,��8-7-6-5-4-3-2-1˳��, ����ȫ���� */
	MOD_RD_03D2143, 	/* 03ָ���, 32bit���,��2-1-4-3˳�� */
	MOD_RD_04B,			/* 04ָ���, �������ֽ�˳�� */
	MOD_RD_04W,			/* 04ָ���, 16bit���,��2-1˳�� */
	MOD_RD_04D,			/* 04ָ���, 32bit���,��4-3-2-1˳��, ����ȫ���� */
	MOD_RD_04D2143,		/* 04ָ���, 32bit���,��2-1-4-3˳�� */
	MOD_RD_04D_I2F,		/* 04ָ���, 32bit���,��4-3-2-1˳��, ����ȫ����; ��ȡ��������I32,ת��F32д��Buf */
	MOD_RD_04L 			/* 04ָ���, 64bit���,��8-7-6-5-4-3-2-1˳��, ����ȫ���� */
}MOD_ACS_TYPE;

/* RS485��������֮�ϵ�ͨѶ�������     <=0:���ϴ��� >0:ͨѶ�ɹ� */
typedef enum {
    UART_ACS_NULL           = 0,    /* �����ڳ�ʼ��״̬����û��ִ����Ӧ�Ķ�д���� */
    UART_RD_TIME_OUT        = -2,   /* ��ȡ��ʱ */
    UART_NUM_NOT_ENOUGH     = -3,   /* ��ȡ���ֽ������� */
    UART_CRC_ERR            = -4,   /* ͨѶcrc���� */
    UART_MOD_DEVADD_NEQ     = -5,   /* ModbusͨѶ��ַ�뷢��ȥ�ĵ�ַ��һ�� */
    UART_MOD_CMD_NEQ        = -6,   /* ModbusͨѶָ���뷢��ȥ��ָ�һ�� */
    UART_MOD_RD_FAIL        = -7,   /* Modbus��ȡʧ�ܣ�ԭ��δ֪ */
	UART_MOD_WR_FAIL        = -8,   /* Modbusд��ʧ�ܣ�ԭ��δ֪ */
    UART_ACS_SUC            = 1     /* ͨѶ�ɹ� */
}UART_ACS_RES;

/* modbus�����豸����Ϊ���ࣺ
	a. ��һ̨�豸����ַ�궨��Ϊ MODDEV_ADD_***
	b. ��̨�豸����ַΪ    10*N + ADDi������ADDiΪβ������ */
#define MODDEV_ADD_XMG			2
#define MODDEV_ADD_YYT			2
#define MODDEV_ADD_ELE			3
#define	MODDEV_ADD_RAIN			5		/* ������ */
#define MODDEV_ADDi_YKF			0
#define MODDEV_ADDi_XMS			3
#define	MODDEV_ADDi_GOV			6		/* ������ */
#define	MODDEV_ADDi_YYB			7
#define	MODDEV_ADDi_DLT			8		/* DLT645���ܱ� */
#define	MODDEV_ADDi_TPM			9		/* �๦�������� */

/*===========================================================================
 * ModbusTCPͨѶ����--Ϊ�˱�֤ͨ��ModbusRTU��ModbusTCPӵ����ȫһ�������ݷ������ԣ���ȡ����RespondModbus()������
 * ����ԭ�����ڻ�����չ���ڣ�ÿһ�����Ӷ���Ҫһ�����������
 * 0 ~ (MODBUS_TCP_INT_TASK_NUM-1) �ṩ��ԭ�����ڣ� MODBUS_TCK_INT_TASK_NUM֮���ṩ����չ����
 *==========================================================================*/
#if SUPPORT_MODBUS_TCP
#if (DEVICE_TYPE == V5_YYT3)
#define MODBUS_TCP_INT_TASK_NUM			8		/* ����ԭ�����ڵ�ModbusTCP�������� */
#else
#define MODBUS_TCP_INT_TASK_NUM			4		/* ����ԭ�����ڵ�ModbusTCP�������� */
#endif
#if (DEVICE_TYPE == V5_YBT3)
	#define MODBUS_TCP_EXT_COMM_NUM		4		/* ������չ���ڵ�ModbusTCP�������� */
#else
	#define MODBUS_TCP_EXT_COMM_NUM 	0
#endif
#define MODBUS_TCP_COMM_MAX_NUM 	(MODBUS_TCP_INT_TASK_NUM + MODBUS_TCP_EXT_COMM_NUM)

typedef struct {
	/* ͨѶ���� */
	BOOL bFree;									/* ������ԭ�����ڣ������������Լ���Ӧ�������ջδ��ʹ�� */
	int8 i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP;/* ͨ��RS485�ڽ���modbusRTU�������   --0:����;-1:�ɹ�;-2:ʧ��;>0:���Ŭ������ */
	uint16 uRxBufPt;
	uint16 uTxBufPt;
	
	/* ͨѶBuf���� */
	uint8 u8TRBuf[UART_TR_BUF_BLEN];
	osStaticThreadDef_t ModbusTCPSessionStackCtlBlk;	/* ��̬���� */
	uint32 ModbusTcpSessionStack[MODBUS_TCP_TASK_STACK/4];
}MODBUS_TCP_COMM;
EXT SECTION(".NOT_ZeroInit") MODBUS_TCP_COMM g_ModbusTcpComm[MODBUS_TCP_COMM_MAX_NUM];
#endif
/*===========================================================================
 * ͨѶȷ��
 *==========================================================================*/
#define KEY_VALID_TIME_ms 		250		        /* GUI������Чʱ�䣨���룩 */
/*  Ϊ��ͳһ VT��Ļ,modbusRTU(MCGS),modbusTCP(��̬),webǰ��(mqtt/json)���ʣ����ư�ť(CtrBtn)�ֿ��Է�Ϊ��
    	B1. ���нӿڶ����Է��ʵİ�ť����Ҫ��modbusTCP(��̬)
    	B2. ���ֳ�������Ļ(VT��Ļ,MCGS/485/modbusRTU)���Է��ʵİ�ť
    Ϊ��֧�ֶ�Դд�룬ÿ����ť��������һ����ʱ��:     д��ð�ť��ʱ�򣬶�ʱ������250ms
    ���尴ť��Ϊ���ɸ�����Ʒ���ж��壻��ʹ��ͨ�ð�ť����ͬ��Ʒ�԰�ť������Ч��Ҳ���Բ�һ����

    ����Ҫʹ�ÿ��ư�ť����Ҫ�ڲ�Ʒͷ�ļ��ж��壬������������:
    #define CTR_BTN_MODTCP_NUM     4      //modbusTcp���Է��ʵİ�ť����
    #define CTR_BTN_TOTAL_NUM      4      //�ܵĿ��ư�ť����
*/ 

/* modbusͨѶ״̬ */
typedef struct {
	COMM_DEV_STATUS_VAR CommDevStatus;			/* �����豸ͨѶ��������ؿ��������ԣ�ż������Ҳ����ν */

    /* ��尴ť */
#ifdef CTR_BTN_TOTAL_NUM
    uint8 u8Tmr_PnlCtrBtn_Valid_ms[CTR_BTN_TOTAL_NUM];  /* ��尴ť��Ч�Ա�־��д����尴ť���Ϳ�ʼ��Ӧ�ĵ���ʱ */
#endif
    uint16 uPnlCtrBtnEcho;                      /* ����,��尴ť��Ӧ */
    uint16 uPnlCtrBtnLock;                      /* ����,��尴ť��,��Ҫ����"���ɰ�"��Ч�� */

    ITEM_PAGE_REQ ItemPageReq;                  /* ModbusTCP����ItemPage���ã�����ModbusTCP����һ�� */
}MISC_COMM_DATA;
EXT MISC_COMM_DATA g_MiscCommData;

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
/* <NULL> */

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
EXT void InitUartComm(void);
EXT void UartCommTask(void const * argument);
EXT void DrvUartTick_1KHz(void);

/***************************************************************************
 					functions could be used as extern
***************************************************************************/
/* ModbusͨѶ��غ��� */
EXT void TxDataForModbusRTU(uint8 u8UartPort, uint8* pTxBuf);
EXT UART_ACS_RES CheckCrcForModbusRTU(uint8* pRxBufStart, uint16 uRxNum);
EXT UART_ACS_RES CheckRxBufForModbusRTUMaster(UART_COMM* pUartComm, uint8 u8DevAdd, uint8 u8ModCmd, uint16 uReaddRegLen);
EXT UART_ACS_RES RdDataFrModbusRTU(uint8 u8UartPort, uint16 uMaxDelay_ms, uint8 u8DevAdd, uint16 uRegAdd, uint16 uRegLen, uint16* pU16Dat, MOD_ACS_TYPE ModAcsType);
EXT UART_ACS_RES WrDataToModbusRTU(uint8 u8UartPort, uint16 uMaxDelay_ms, uint8 u8DevAdd, uint16 uRegAdd, uint16 uRegLen, uint16* pU16Dat, MOD_ACS_TYPE ModAcsType);
EXT UART_ACS_RES WaitDataFromModbusRTU(uint8 u8UartPort, uint16 uMaxDelay_ms);
/* ��������modubs��Ӧ���� uModbusPort < RS485�˿���(MAX_UART_NUM)������Ϊ��ModbusRTU��������Ϊ��ModbusTCP */
EXT void RespondModbus(uint8 u8ModbusPort, uint8 u8DevAdd);
EXT uint16 WriteMosbusMon(uint16 uRegAddr, uint8* pRxBuf, uint16 uRegLen);

/* canͨѶ������modbusѰַ��ʽ�����豸���� */
EXT BOOL ReadModbus64bitsForCan(uint16 uRegAddr, uint32* pU32Dat);
EXT BOOL WriteModbusForCan(uint16 uRegAddr, uint8* pRxBuf, uint16 uRegLen);

/* �Ǳ�������Modbus��Ӧ����������YBTn, YYTn��ͨѶ��Լ */
#if SUPPORT_MODBUS_TCP
EXT void RespondModbusTCPForOthers(MODBUS_TCP_COMM* pModbusTcpComm, BOOL bBlock);
EXT void RespondModbusRTUForOthers(uint8 u8ModbusPort);
#endif
/************   exclude redefinition and c++ environment   ****************/

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
/******************************** FILE END ********************************/
