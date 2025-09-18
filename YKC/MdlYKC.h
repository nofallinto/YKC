/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved
 *				: 
 * File			: stencil.h
 * Author		: Wang Renfei
 * Description	: head file's stencil, V1.2 inherit from V1.1
 *				: ģ�黯������ṹ��ÿһ��ģ����ͷ�ļ��ж����ⲿ�ӿڱ���,�ⲿ��������,ģ����÷�����������Դ�ļ��ж���ģ���ڲ�����������
 *				: 
 * Comments		:
 * Date			: 2009-5-31
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 * V0.1	| 2023-12-8   |	king | 1. Create
 **************************************************************************/

/***************************************************************************
 				exclude redefinition and c++ environment
***************************************************************************/
#ifndef _MDL_YKC_H_
#define _MDL_YKC_H_

#ifdef __cplusplus
extern "C" {
#endif


/***************************************************************************
 							include files
***************************************************************************/
#include "LibMath.h"
#include "GlobalVar.h"

/***************************************************************************
				hardware parameter and global definitions
***************************************************************************/
#define OS_TICK_KHz				1				/* ����ϵͳtickƵ�� */
#define PERIOD_CTR_TASK_ms		10

#define MAX_INTERNET_ACCESS_NUM	1
#define SUPPORT_GPRS			TRUE
#define SUPPORT_ETHERNET		FALSE
#define SUPPORT_MODBUS_TCP		FALSE
#define SUPPORT_SCREEN			FALSE

#define MAX_MSG_TYPE			0x0060			    /* ������㣬������ҪMSGģ�� */

/***************************************************************************
 		use EXT to distinguish stencil.c or others source files
***************************************************************************/
#ifdef _MDL_YKC_C_
#define EXT
#else
#define EXT extern
#endif

/***************************************************************************
 					variables could be used as extern
***************************************************************************/
/*===========================================================================
 * ����ϵͳ����
 *==========================================================================*/
#define MOTOR_HISTORY_CUR_MAX_CNT		15		/* �Ĵ������ʷ�������� */
typedef struct {								/* modbusͨѶ��ַ: 9200��ע���ַ������2 */
	float32 fCPUTemperature;					/* CPU�¶ȣ���λ:���϶� */
	float32 fVrefint;			/* �ڲ��ο���Դ��ѹ��Ӧ��1.2V���� */
	float32 fDCPwrVol;			/* ��Դ��ѹ */
	float32 fRightCur; 			/* �Ҳ���(�Ĵ�)���� */
	float32 fLeftCur;			/* �����(�Ĵ�)���� */
	float32 fLastRightCur;		/* �ϴ��Ҳ���(�Ĵ�)���� */
	float32 fLastLeftCur;		/* �ϴ������(�Ĵ�)���� */

	F32_QUEUE_t qRightMotorCurrHistory;		/* ���Ĵ�������ʷ���� */
	F32_QUEUE_t qLeftMotorCurrHistory;		/* ���Ĵ�������ʷ���� */

	F32_QUEUE_t qMotorAHistory;					/* �Ĵ�������ٶ���ʷ���� */

	float32 fRightMaxI;
	float32 fLeftMaxI;

	/* ��ѹ�� */
	float32 fAirPressure;		/* ��ն� */
	float32 fAirTemp;			/* ��ѹ���¶� */
	uint8 u8TryCnt_AirSensor;	/* ��ѹ�ƴ�����ͨѶָʾ */

	/* IMU���� */
	uint8 u8TryCnt_IMU;
	uint16 uRsvd;
	int16 iAccX;				/* X����ٶ� */
	int16 iAccY;				/* Y����ٶ� */
	int16 iAccZ;				/* Z����ٶ� */
	int16 iGyroX;
	int16 iGyroY;
	int16 iGyroZ;
	float32 fPitch;				/* �� */
	float32 fRoll;				/* �� */
	float32 fYaw;

	/* ת�ټ� */
	float32 fPumpFreq;
	float32 fRightFreq;
	float32 fLeftFreq;
	uint16 uRightCount;
	uint16 uLeftCount;
}MSR_RES;
EXT SECTION(".NOT_ZeroInit") MSR_RES g_MsrRes;

/*===========================================================================
 * ����ң����
 *==========================================================================*/
#define IR_BTN_NONE		0
#define	IR_BTN_0		152
#define IR_BTN_1		162
#define	IR_BTN_2		98
#define	IR_BTN_3		226
#define	IR_BTN_4		34
#define	IR_BTN_5		2
#define	IR_BTN_6		194
#define	IR_BTN_7		224
#define	IR_BTN_8		168
#define	IR_BTN_9		144
#define	IR_BTN_START	104
#define	IR_BTN_SHARP	176
#define	IR_BTN_UP		24
#define	IR_BTN_DOWN		74
#define	IR_BTN_LEFT		16
#define	IR_BTN_RIGHT	90
#define	IR_BTN_OK		56
#define IR_RX_SEQ_BLEN	33

typedef struct {
	uint8 u8BtnPressing;							/* ��ǰ���µİ��� */
    uint8 u8TryCnt;									/* ������Ч�� */
    uint8 u8SelectedrIndex;							/* ��ǰָ���ĺ����־ */
    uint8 u8Rsvd;									/* ռλ�� */
    struct {
    	BOOL bBtnRdyDecode;							/* �д����������֡ */
    	uint8 u8TimerUpdatCount;					/* timer������� */
    	uint16 uCanlCnt;							/* ͨ����Ч�� */

    	int32 i32Wait;
    	uint8 u8CapPol;								/* ������ */
    	uint8 u8CapPulseCount;						/* ����������� */
    	uint16 uRepeatCount;						/* �����ظ������(��ס������) */
    	uint8 bIsIdle;								/* ����״̬ */
        uint8 u8Rsvd2;								/* ռλ��2 */

    	uint16 rxFrame[IR_RX_SEQ_BLEN * 2];			/* ����֡�������ࣩ */
    	uint16 srcData[IR_RX_SEQ_BLEN * 2];			/* ����֡��Ӧ�òࣩ */


    	union{
    	    uint32 u32AllData;
    	    struct {
    	        BITBOOL key_val_n:8;
    	        BITBOOL key_val  :8;
    	        BITBOOL addr_n   :8;
    	        BITBOOL addr     :8;
    	    } bit;
    	} data;
    } DevComm[2];
}IR_CTRL;

EXT IR_CTRL g_IRCtrl;		/* ����ң������ȫ�ֱ��� */

/* �������� */
typedef enum {
	SILENCE			= 0,
	VOICE_WELCOM,							/* ��ӭʹ��Զ������������ */
	VOICE_AIR_PRES_LOW,						/* ��ѹ���� */
	VOICE_AIR_PRES_ABN,	 					/* ��ѹ�쳣 */
	VOICE_IMU_ABN, 							/* �������쳣 */
	VOICE_LIQUID_LOW, 						/* ˮ��ˮλ�� */
	VOICE_STARTCLEAN, 						/* ��ɨ��ʼ */
	VOICE_FINISHCLEAN, 						/* ��ɨ��� */
	VOICE_AC_UNPLUGGED,						/* ����Ͻ�����Դ */
	VOICE_CLIFF_DETECTED,					/* �����¿��ش�������״̬ */
	VOICE_VOLTAGE_ERROR, 					/* ��ѹ�쳣 */
	VOICE_CPU_TEMPERATURE_ERROR, 			/* CPU�¶��쳣 */
	VOICE_BYEBYE, 							/* �ɹ��˳� */
#if (!SOFT_RUN1_TEST0)
	VOICE_HIT_SIDE_L_FRONT_DINNO, 			/* ���ϼ�⵽��ײ */
	VOICE_HIT_SIDE_L_REAR_DINNO, 			/* ���¼�⵽��ײ */
	VOICE_HIT_SIDE_R_FRONT_DINNO, 			/* ���ϼ�⵽��ײ */
	VOICE_HIT_SIDE_R_REAR_DINNO, 			/* ���¼�⵽��ײ */
	VOICE_HIT_FRONT_LEFT_DINNO, 			/* ǰ���⵽��ײ */
	VOICE_HIT_FRONT_RIGHT_DINNO, 			/* ǰ�Ҽ�⵽��ײ */
	VOICE_HIT_REAR_LEFT_DINNO, 				/* ����⵽��ײ */
	VOICE_HIT_REAR_RIGHT_DINNO, 			/* �Ҳ��⵽��ײ */
	VOICE_CLIFF_FRONT_LEFT_DINNO, 			/* �������ϼ�⵽��ײ */
	VOICE_CLIFF_FRONT_RIGHT_DINNO, 			/* �������ϼ�⵽��ײ */
	VOICE_CLIFF_REAR_LEFT_DINNO, 			/* �������¼�⵽��ײ */
	VOICE_CLIFF_REAR_RIGHT_DINNO 			/* �������¼�⵽��ײ */
#endif
}TTS_DIALOG_TYPE;

typedef enum {
	MP3_CMD_QUERY_STATE = 0x01,			/* ��ѯ����״̬(0~2: ֹͣ/����/��ͣ) */
	MP3_CMD_PLAY = 0x02,				/* ���� */
	MP3_CMD_PAUSE = 0x03,				/* ��ͣ */
	MP3_CMD_STOP = 0x04,				/* ֹͣ */
	MP3_CMD_SELECT_SONG_PLAY = 0x07,	/* ѡ����Ŀ���� */
	MP3_CMD_SET_VOLUME = 0x13,			/* ��������(0~30) */
	MP3_CMD_VOLUME_UP = 0x14,			/* ������ */
	MP3_CMD_VOLUME_DOWN = 0x15,			/* ������ */
}MP3_CMD;	/* MP3ģ��ָ�� */

EXT const char* cnst_TtsDialog[]
#ifndef _MDL_YKC_C_
;
#else
= {
		"[v16][m0][t5]",
		"[v16][m0][t5]��ӭʹ��Զ������������",
		"[v16][m0][t5]��ѹ����",
		"[v16][m0][t5]��ѹ�쳣",
		"[v16][m0][t5]�������쳣",
		"[v16][m0][t5]ˮ��ˮλ��",
		"[v16][m0][t5]��ɨ��ʼ",
		"[v16][m0][t5]��ɨ���",
		"[v16][m0][t5]����Ͻ�����Դ",
		"[v16][m0][t5]������Ľ�����",
		"[v16][m0][t5]�ڲ������ο���ѹ�쳣",
		"[v16][m0][t5]CPU�¶��쳣",
		"[v16][m0][t5]�ɹ��˳�",
		"[v16][m0][t5]���ϼ�⵽��ײ",
		"[v16][m0][t5]���¼�⵽��ײ",
		"[v16][m0][t5]���ϼ�⵽��ײ",
		"[v16][m0][t5]���¼�⵽��ײ",
		"[v16][m0][t5]ǰ���⵽��ײ",
		"[v16][m0][t5]ǰ�Ҽ�⵽��ײ",
		"[v16][m0][t5]����⵽��ײ",
		"[v16][m0][t5]�Ҳ��⵽��ײ",
		"[v16][m0][t5]�������ϼ�⵽��ײ",
		"[v16][m0][t5]�������ϼ�⵽��ײ",
		"[v16][m0][t5]�������¼�⵽��ײ",
		"[v16][m0][t5]�������¼�⵽��ײ"
};
#endif

EXT BOOL InitTtsSpeaking();
EXT BOOL TtsSpeak(TTS_DIALOG_TYPE DlgType, BOOL bIgnoreDup);		/* ����ɹ����벥�Ŷ����򷵻�TRUE��������������򷵻�FALSE */

/* �쳣��ţ��쳣����� g_AnaRes.u32Abnormal */
#define ABN_TIME_INVERSE_START_No       0  	/* �����Ƿ�ʱ���쳣 */
#define ABN_T_INV_MOTOR_RIGHT_No		0	/* ����A�������ʱ��, YKF2/3/YYD/YYG֧��, ����:��բ�ػ� */
#define ABN_T_INV_MOTOR_LEFT_No			1	/* ����B�������ʱ��, YKF2/3/YYD/YYG֧��, ����:��բ�ػ� */
#define TOTAL_ABN_NUM                   2	/* ���쳣���� */
#define ABNORMAL_BUF_b32LEN             ((TOTAL_ABN_NUM + 31)/32)

/* �п�������ĵ�Ԫ, �ṩ������ģ�飬���߹�����Ա����ʱʹ�� */
typedef struct {							/* modbusͨѶ��ַ: 9600��ע���ַ������2 */
	float32 fLeftSpeed;
	float32 fRightSpeed;
	uint32 u32Abnormal[ABNORMAL_BUF_b32LEN];
}ANA_RES_VAR;
EXT SECTION(".NOT_ZeroInit") ANA_RES_VAR g_AnaRes;

/* ͳ�Ʊ����������ȷ���STAT_CUM_ITEM */
typedef struct {
}STAT_CUM_DATA;
EXT SECTION(".NOT_ZeroInit") STAT_CUM_DATA g_StatCum;

typedef struct {
}STAT_COUNT_DATA;
EXT SECTION(".NOT_ZeroInit") STAT_COUNT_DATA g_StatCount;

#define COMM_DEV_STATUS_VAR	uint32
/*===========================================================================
*  Զ�����ֳ����ƽӿ�
*==========================================================================*/
#define CTR_BTN_IncRelay_No         0           /* ���� */
#define CTR_BTN_DecRelay_No         1           /* ���� */
#define CTR_BTN_TOTAL_NUM       2           /* �ܵĿ��ư�ť���� */

/*===========================================================================
*  6. g_Ctr: ��������״̬
*==========================================================================*/
typedef enum {				/* ����ģʽ */
	RUN_MODE_ABN_OCCUR						/* �쳣����,������һ������ģʽ, ���ڲ����ػ���Ϣ�����Ǳ�̷��� */
}RUN_MODE_STATE;

/*===========================================================================
 * MSG����
 *==========================================================================*/
typedef enum {
	MSG_NULL						= 0x0000,		/* �� */
	MSG_HuanYinShiYong				= 0x0001,		/* ��ӭʹ��--������Ϣ�����ڿ�����ʾ����Ӧ����AddMsg���(�洢) */
	
	/* �����쳣��Ϣ��ʼ */
	MSG_MqttDisc    				= 0x0050,		/* MQTT���������Ӷ��� */
	MSG_LicenseTimeOut				= 0x0051,		/* ���������֤����,����ϵ��˾ */
	MSG_LicenseRenewAlert			= 0x0052,		/* ���������ü�������,��ʣ��,�뼰ʱ�ɷ� */
    MSG_SoftIntegrityAbn            = 0x0053,       /* ����������쳣,����ϵ��˾,�쳣����: */
    MSG_SoftUpgradeFail             = 0x0054,       /* �������ʧ��,���ϴ���: */
    MSG_Rsvd55                      = 0x0055,       /* ���� */
    MSG_Rsvd56                      = 0x0056,       /* ���� */
    MSG_Rsvd57                      = 0x0057,       /* ���� */
	MSG_Rsvd58				        = 0x0058,		/* ���� */
	MSG_Rsvd59			            = 0x0059,		/* ���� */
	MSG_Rsvd5A 		                = 0x005A,		/* ���� */
	MSG_Rsvd5B 		                = 0x005B,		/* ���� */
	MSG_Rsvd5C						= 0x005C,		/* ���� */
	MSG_Rsvd5D						= 0x005D,		/* ���� */
    MSG_Rsvd5E                      = 0x005E,       /* ���� */
    MSG_Rsvd5F                      = 0x005F,       /* ���� */

}MSG_ID;

EXT const MSG_PROP cnst_MsgProp[MAX_MSG_TYPE]	/* Ϊ�˱���ʶ��,�и���ֵ����Ϣ���Ĳ��ֲ����Դ����� */
#ifndef _MDL_YKC_C_
;
#else
= {	{-1, F32_DEFAULT,	{"", ""}},
	{-1, F32_DEFAULT,	{"��ӭʹ��YKC", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},

	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},

	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},

	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},

	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	
	{-1, F32_DEFAULT,	{"���ӷ���������", "Reserved"}},
	{-1, F32_DEFAULT,	{"���õ���,����ϵ��˾www.yoplore.com", "Reserved"}},
	{9,  F32_INT,		{"���ü�������,��ʣ��,�뼰ʱ�ɷ�", "Reserved"}},
	{19, F32_INT,	    {"����������쳣,����ϵ��˾,�쳣����:", "Reserved"}},
	{12, F32_INT,		{"�������ʧ��,���ϴ���:", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}},
	{-1, F32_DEFAULT,	{"����", "Reserved"}}
};
#endif

/*===========================================================================
 * ���ñ���
 *==========================================================================*/
/* ͨѶ��������, ��ֵΪ0˵��δ����ʼ�� */
typedef struct {
	/* ���������в�Ʒ���� */
	uint32 u32MqttPubIntv_ms;					/* Mqtt������� */
	uint32 u32GuiIdleTimeOut;					/* ����GUI��תĬ��ҳ��ʱ(s) */

}COMM_CONF;
EXT SECTION(".NOT_ZeroInit") COMM_CONF g_CommConf;

#if CPU_0ST_1TI
/*---------------------------------------------------------------------------
 * �ɼ�����Ϣeeprom�洢��ַ
 *--------------------------------------------------------------------------*/
/* EEProm��ַ����, ��64byteΪ��Ԫ���ܹ�96Ƭ
	0		: BootLoader
	1~43	: ���ô洢����
	44~95	: Acq, Msg���� */
#define EEPROM_ADDR_MSG				((96-20)*64)			/* 64*5*32bit, Msgÿ��ռ��20byte */
#define EEPROM_ADDR_ACQ				((96-20-32)*64)			/* 64*8*32bit, Acqÿ��ռ��32byte */
#define EEPROM_MSG_ITEM_NUM			64						/* ����ǰ�����Ŀռ䣬���洢64����Ϣ */
#define EEPROM_ACQ_ITEM_NUM			64						/* ����ǰ�����Ŀռ䣬���洢64��Acq */
#endif
/***************************************************************************
 					functions must be driven by extern
***************************************************************************/
EXT void InitMdlCtr(void);
EXT void RunMdlCtr(void);
EXT void DrvMdlCtrTick_1KHz(void);

/***************************************************************************
 					functions could be used as extern
***************************************************************************/

/************   exclude redefinition and c++ environment   ****************/
#ifdef __cplusplus
}
#endif 					/* extern "C" */

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
/******************************** FILE END ********************************/

