
/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved
 *				: 
 * File			: GlobalVar.h
 * Author		: Wang Renfei
 * Description	: 
 * Comments		:
 * Date			: 2015-8-29
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 **************************************************************************/
/*===========================================================================
 * ��Ʒ�ͺ�������汾�Ŷ��壬���ڿ�ʼ���ڲ�ͬ��Ʒ�޸�
 Device Type Code Definition:	��Ʒ��(4bit) + ����(4bit) + �������ܻ�(8bit)
������
	0x0		Reserved
	0x1		CH based Controller
	0x2		YKF based Controller
	0x5		V5 ��Ʒ��
*==========================================================================*/
#define V5_YKF2 				0x5102
#define V5_YKF3 				0x5103
#define V5_YKF4                 0x5104
#define V5_YKF5                 0x5105
#define V5_YKF6					0x5106
#define Is_YKF(DEVICE_TYPE)		((DEVICE_TYPE == V5_YKF2) || (DEVICE_TYPE == V5_YKF3) || (DEVICE_TYPE == V5_YKF4) || (DEVICE_TYPE == V5_YKF5) || (DEVICE_TYPE == V5_YKF6))
#define V5_YYT3 				0x5203
#define V5_YYT4 				0x5204
#define V5_YBU2 				0x5300
#define V5_YBU3 				0x5303
#define V5_YBU4 				0x5304
#define V5_YBT2 				0x5420
#define V5_YBT3 				0x5423
#define V5_YBT4 				0x5424
#define V5_YYB					0x5500
#define V5_YYG					0x5510
#define V5_YYD					0x5520
#define V5_YYD2					0x5522
#define YKC						0x7001

/***************************************************************************
 				exclude redefinition and c++ environment
***************************************************************************/
#ifndef _GLOBAL_VAR_H_
#define _GLOBAL_VAR_H_

/***************************************************************************
				hardware parameter and global definitions
***************************************************************************/
/*===========================================================================
 * �������ͱ���
 *==========================================================================*/
#ifndef CPU_DATA_TYPES
#define CPU_DATA_TYPES
	typedef unsigned char		BOOL;
	typedef signed char			triST;		/* ��̬��������:�����㡢������״̬ */
	typedef signed char			int8;
	typedef unsigned char		uint8;
	typedef short 				int16;
	typedef unsigned short		uint16;
	typedef long				int32;
	typedef unsigned long		uint32;
	typedef long long			int64;
	typedef unsigned long long	uint64;
	typedef float				float32;
	typedef long double 		float64;
	typedef unsigned int		BITBOOL;
#endif

#ifndef TRUE
	#define TRUE 1
#endif
#ifndef FALSE
	#define FALSE 0
#endif

/*===========================================================================
 * ָ��
 *==========================================================================*/
#define	 NOP			        do{asm (" nop");	}	while(0)
#define SECTION(sectionName) 	__attribute__((section(sectionName)))   /* ������ָ��ȫ�ֱ����ͺ����Ķ�(section) */
#define __weak                  __attribute__((weak))					/* ����麯�� */

/***************************************************************************
 		use EXT to distinguish stencil.c or others source files
***************************************************************************/
#ifdef _MDL_SYS_C_
#define EXT
#else
#define EXT extern
#endif

/***************************************************************************
 					��������
***************************************************************************/
/*===========================================================================
 * ���ýṹ�嶨��
 *==========================================================================*/
typedef struct {		/* ʱ�䶨�� */
	uint8 u8Year;								/* 1-99�������λΪ0���������ʱ����Ч */
	uint8 u8Month;								/* 1-12 */
	uint8 u8Day;								/* 1-31 */
	uint8 u8Hour;								/* 0-23 */
	uint8 u8Minute;								/* 0-59 */
	uint8 u8Second;								/* 0-59 */
	uint16 uMilliSec;							/* 0-999 */
}REAL_TIME_VAR;

typedef struct {		/* �¶ȴ��������� */
	int16 iDegree_100mC;						/* �¶����ݣ���λ0.1���϶� */
	uint8 u8PlaceNo;							/* ͬһ��λ�õı���, 0:����ֻ��һ�����������; >0:��� */
	int8 i8Placement;							/* �¶ȴ�����λ�ã�0:���¶ȴ�����; >0:�¶���Ч; <0:����������    */
}TEMPERATURE_DATA;

/* ����ͳ�� */
typedef struct {
	int64 i64TotalCumulant;
	float32 fFragCumulant;			/* һ��ʱ��Ƭ�ڵĻ��������ڼ���ƽ��ֵ */
	float32 fCumulantRem;			/* �������������Է�ֹһ��ʱ��Ƭ�����ݲ�����ͻ�����׼ */
}STAT_CUM_ITEM;

/* ����ͳ�� */
typedef struct {
	uint32 u32StatThr;
	uint32 u32StatVal;
}STAT_COUNT_ITEM;

typedef enum {	/* F32�����������壬���ڱ�ﲻͬ����������������ͬ����ʾ��ʽ */
	F32_DEFAULT			= 0,	/* ����Ĭ�ϵ�������ʾ��ʽ */
	F32_SgnDigits_1		= 1,	/* 1~10Ϊָ����Чλ */
	F32_SgnDigits_10 	= 10,	/* 1~10Ϊָ����Чλ */
	F32_INT,	/* ����, ��С����. [-999,9999]:sabc,s�Ƿ���,    <-999:-999, >9999:9999 */
	F32_CENT,	/* �ٷֱ�, [0,100):ab.c%, [100,999]:abc%,    >999:999% */
	F32_TEMP,	/* �¶�: [0,100):ab.c��, [100,999):abc��, >=999:999�� */
	F32_WATER, 	/* ˮλ: [0,9.999]:a.bcdm, (9.999,99.99):ab.cdm, >99.99:99.99m */
    F32_MPa,    /* ѹ��: [0,9.999]:a.bcdMPa, (9.999,99.99]:ab.cdMPa, >99.99:99.99Mpa */
	F32_FREQ,	/* Ƶ��:	 [0,99.99]:ab.cdHz, [100,999.9):abc.dHz, >=999.9:999.9Hz */
	F32_VOL,	/* ��ѹ: [0,10):a.bcV, [10,100):ab.cV, [100,8e3):  abcdV, [8e3,1e4):a.bcKV, [1e4, 1e5):ab.cKV, >=1e5:abcKV */
	F32_CUR,	/* ����: [0,10):a.bcA, [10,100):ab.cA, [100,8e3):  abcdA, [8e3,1e4):a.bcKA, [1e4, 1e5):ab.cKA, >=1e5:abcKA */
	F32_PwrP,	/* �й�����:  [0,10):a.bcKW, [10,100):ab.cKW, [100,1000):  abcKW, [1000, 1e4):a.bcMW, [1e4, 1e5):ab.cMW, [1e5, 1e6):abcMW */
	F32_PwrQ,	/* �޹�����: ͬ�й����ʣ�ֻ�ǵ�λKVar, MVar */
	F32_COS,	/* ��������:[-1.1,1.1]:a.bcd, ����֮�⣬����ʾ�������� */
/* ��/�޹����(��):[0,10):a.bc��, [10,1e3):abc.d��, [1e3,1e4):abcd��, [1e4,1e5)a.bc���, [1e5,1e7):ab.c���, [1e7,1e8):abcd���, ���9999�ڶ�
   �й����(Ӣ):[0,100):ab.cdKWH, [100,1e3):abc.dKWH, [1e3,1e5):ab.cdMWH, [1e5,1e6)abc.dMWH, ���999.9TWH
   �޹����(Ӣ):[0,100):ab.cdKVarH, [100,1e3):abc.dKVarH, [1e3,1e5):ab.cdMVarH, [1e5,1e6)abc.dMVarH, ���999.9TVarH */
	F32_EngP,
	F32_EngQ,	/* �޹����:ͬ�޹���� */
	F32_BatAH,	/* ����:[0,99.99]:ab.cdAH, [100,999.9):abc.dAH, >=999.9:999.9AH  */
/* �迹: [0,10):a.bcOhm, [10,100):ab.cOhm, [100,1e3):  abcOhm, [1e3,1e4):a.bcKOhm, [1e4, 1e5):ab.cKOhm, [1e5, 1e6):abcKOhm,
        [1e6,1e7):a.bcMOhm, [1e7,1e8):ab.cMOhm, [1e8,1e9):abcMOhm, >=1e9:999MOhm   */
    F32_OHM,
	F32_FlowRate,/* ����: (-inf, 10]��ʾΪ -ab.cdem3/s; (-10, 0)��ʾΪ-a.bcdem3/s; [0, 10)��ʾΪa.bcdem3/s; [10, inf)��ʾΪ ab.cdem3/s */
	F32_FlowVlct,/* ����: (-inf, 10]��ʾΪ -ab.cdem/s; (-10, 0)��ʾΪ-a.bcdem/s; [0, 10)��ʾΪa.bcdem/s; [10, inf)��ʾΪ ab.cdem/s */
	F32_TIME,	 /* ʱ�䣺(100H,1H]��ʾΪ aH; (60min,1min]��ʾΪ aMin; (60s,1s]��ʾΪ as;*/
/* ��ѹ�仯�ʣ� [0,1mV):0.abcmV/m, [1mV,10mV):a.bcmV/m, [10mV,100mV):ab.cmV/m, [100mV,1V):abcmV/m, [1V,10V):a.bcV/m,
		[10V,100V):ab.cV/m, [100,1000):abcV/m, [10,inf):999V/m */
	F32_VpM
}F32_MEANING;

/*===========================================================================
 * 2. ������ʾ�����á��¼���Ϣ���ɼ����������ݸ�ʽ����
 *==========================================================================*/
typedef enum {			/* UI�����ҳ�� */
    /* �⼸��ҳ�棬��ͬ��ƷӦ����һ���� */
	DATA_PAGE_NULL				= 0x00,             /* ��ҳ�� */
	DATA_PAGE_CONF				= 0x01,				/* ����ҳ�� */
	DATA_PAGE_MSG				= 0x02,				/* ��Ϣҳ�� */
	DATA_PAGE_ACQ				= 0x03,				/* �ɼ�ҳ�� */
    /* ����ҳ�棬��ͬ��Ʒ��һ�� */
	DATA_PAGE_WELCOME			= 0x04, 			/* ��ӭҳ�� */
	DATA_PAGE_MAIN				= 0x05, 			/* ��ҳ�� */
	DATA_PAGE_ASSIT 			= 0x06, 			/* ����ҳ�� */
	DATA_PAGE_SYS				= 0x07, 			/* ϵͳ����ҳ�� */
	DATA_PAGE_DEBUG				= 0x08,
	DATA_PAGE_TEMP_SINGLE       = 0x09,             /* ��չ��Ļ(�¶�), ��ͨ��Ѳ�� */
	DATA_PAGE_TEMP_MULTI        = 0x0A,              /* ��չ��Ļ(�¶�), ��ͨ��Ѳ�� */
	MAX_DATA_PAGE
}DATA_PAGE;

typedef enum {
	ITEM_DAT_NUL				= 0x00,
	/* ����7������ΪUI��Ļ��ʾ��������� */
	ITEM_UI_TOPIC				= 0x01, 			/* ����,���Բ������ء�ˢ�¡����¹����� */
	ITEM_UI_TEXT				= 0x02, 			/* ���ı�,�������,  */
	ITEM_UI_CLICK				= 0x03, 			/* �ɵ�������������, ��BOOL,ECONT,S32,������LINK��ӳ�䵽���� */
	ITEM_UI_LOGIN				= 0x04, 			/* һ�����������, ��LINK������Ĺ��� */
	ITEM_UI_ONEBOX				= 0x05, 			/* һ����������� */
	ITEM_UI_TWOBOX				= 0x06, 			/* ������������� */
	ITEM_UI_ONEDBOX				= 0x07, 			/* һ������������������ */
	ITEM_UI_ANSI				= 0x08, 			/* һ��32�ַ�ANSI�ı�����(Ŀǰ֧��ASCII�͹�) */

	/* ����Ϊʵ������ʹ�õ����ͣ�����4λ�õ�UI���ͣ����⣺LINK */
	ITEM_DAT_TOPIC				= 0x10, 			/* ����, LkId_Val_ItN�����б�����; ����б�������Ϊҳ�����, �Ӻ�����λ�������ñ������λ��ʼ������ */
	ITEM_DAT_TXT				= 0x20, 			/* ���ı�,�������,  */
	ITEM_DAT_SOFTVER			= 0x21, 			/* ����汾��,��LkId_Val_ItN�洢���ޱ��� */
	ITEM_DAT_ENUM				= 0x22, 			/* ö�� */
	ITEM_DAT_T64_RO 			= 0x23, 			/* 64λʱ��(ֻ��),  REAL_TIME_VAR��ʽ */
	ITEM_DAT_U32_RO 			= 0x24, 			/* ֻ����U32λ���� */
	ITEM_DAT_ANSI_RO			= 0x25,				/* Ӳ���汾�ţ���ʵ��ֻ����ANSI�ַ��� */
	ITEM_DAT_T32_RO 			= 0x26, 			/* 32λʱ��(ֻ��),  seconds��ʽ�洢����/��/�� ʱ:��:�� ��ʽ��ʾ */
    ITEM_DAT_I64_RO             = 0x27,             /* ֻ����I64λ���� */
    ITEM_DAT_U32_ID_RO          = 0x28,             /* ��оƬID(Hashֵ)�ӳֵı���, ֻ��ʾ����ֵ���޸ĵ�ʱ����Ҫ��¼оƬID������sn,license */
	ITEM_DAT_S32				= 0x30, 			/* 32λ�ṹ��, ��ȡ����������ҳ���TOPIC����ָ������ݽ�������ʾ�����ں��� */
	ITEM_DAT_BOOL				= 0x31, 			/* bool���� */
	ITEM_DAT_ECONT				= 0x32, 			/* ö������, LkId_Val_ItN ����ö��ֵ(����ֵ) */
	ITEM_DAT_LINK				= 0x40, 			/* ���ӣ����ӵı���������,LkId_Val_ItN�������ӵ�ַ */
	ITEM_DAT_FUNC               = 0x41,             /* �ض����ܣ��������룬ִ��ĳ�ֶ���(�磺���ͳ������)��LkId_Val_ItNֵ�� SYS_FUNCTION */
	ITEM_DAT_U32 				= 0x50, 			/* 32λ�޷����� */
	ITEM_DAT_F32 				= 0x51, 			/* 32λ������ */
	ITEM_DAT_TEMPSENCONF		= 0x52, 			/* �����¶ȴ���������,���ڴ����������ITEM_DAT_U32һ������ */
	ITEM_DAT_ACQ_U32			= 0x53, 			/* ���ɼ����ܵ�32λ�޷�����(�ɼ��Ĳ����ǵڶ�������Ϊ������) */
	ITEM_DAT_ACQ_F32			= 0x54,				/* ���ɼ����ܵ�32λ������(�ɼ��Ĳ����ǵڶ�������Ϊ������) */
	ITEM_DAT_D2U32				= 0x60, 			/* 2��32λ�޷�����--����ͳ��ҳ�� */
	ITEM_DAT_D2F32				= 0x61, 			/* 2��32λ�޷�����--����ͳ��ҳ�� */
	ITEM_DAT_SUMM				= 0x62, 			/* ���ڴ��Զ��ɼ����ܱ��������ã����ڴ��������,Data1��float32, data2��int32 */
	ITEM_DAT_pTIME				= 0x63,				/* ��ȵ�ʱ�����--��ʾΪp#hh:mm; ����#:������.����,Ҳ����; �洢����#. */
	ITEM_DAT_I64				= 0x70, 			/* 64λ������--����ͳ��ҳ�� */
	ITEM_DAT_IPv4				= 0x71, 			/* IPv4��ַ����ʾ��ʱ����Ҫ��ʾΪ abc.abc.abc.abc */
	ITEM_DAT_RMTS				= 0x72,				/* Զ�̴����������������кŲ��� */
	ITEM_DAT_DIGITSTR			= 0x73, 			/* �����ֹ��ɵ��ַ��������ڱ��롢���� */
	ITEM_DAT_TELE_ITEM			= 0x74,				/* 101,104�ĵ�������� */
	ITEM_DAT_ANSI 				= 0x80, 			/* ANSI���� */
}ITEM_DATA_TYPE;
#define CONF_ANSI_BYTE_LEN	32						/* ITEM_DAT_ANSI��ʽ���ݳ��ȣ���ҪΪ4�ı����Է������ */

typedef enum {										/* ����/ͳ�����ݴ洢���з���洢��ÿһ�����ݽ��洢����Ӧ������ */
	SAVE_GRP_NUL				= 0,
	SAVE_GRP_BRD				= 1,				/* ��������࣬����eeprom���� */
	SAVE_GRP_MCONF				= 2,				/* ��Ҫ���������� */
	SAVE_GRP_TSEN				= 3,				/* �¶ȴ����������� */
	SAVE_GRP_RSVD				= 4,				/* ���� */
	SAVE_GRP_STAT 				= 5,				/* ͳ������������ */
MAX_SAVE_GRP_NUM	= 6
}CONF_SAVE_GROUP;

typedef struct {
	ITEM_DATA_TYPE DataType;						/* �������� ITEM_DATA_TYPE */
	uint8 uBcdQ;									/* С����λ�� */
	/* ����Ǹ������ͣ���Ϊ��ʾʱ�ĸ�������Чλ��(0Ϊ����Ĭ��ֵ4)������Ϊλ���, �����������ñ�����ҳ�������(Topic����Ӧ) */
	int8 i8SgnDigits_u8BitNum;
	uint8 SaveGrp;									/* ���ݴ�ȡ����, ���洢ϵͳʹ��, ����� CONF_SAVE_GROUP */
}ITEM_TYPE;

typedef enum {
	CHINESE			= 0,
	ENGLISH			= 1,
#if (DEVICE_TYPE == V5_YYT3 || DEVICE_TYPE == V5_YYT4)
	NUMBERIC		= 2,							/* ����� */
#endif
	MAX_LANG_TYPE
}LANG_TYPE;
typedef struct {									/* Conf,Stat ������������ */
	uint32 u32ItemID;								/* ����ID */
	ITEM_TYPE ItemType;
	uint32 LkId_Val_ItN;							/* ����ID/�趨Ĭ��ֵ/����ֵ/�б����� */
	void* pData;
	char* pName[MAX_LANG_TYPE];
}CONF_n_STAT_PROP;
#define CONF_PAGE_LVL_NUM				4			/* ����ҳ��㼶����, Ҫ����ʵ�㼶��1������Ѱַ */

/* �����������ýṹ�� */
typedef struct {									/* ITEM_DAT_ACQ_F32 */
	float32 fSetVal;								/* �˹��趨����--Ϊ�˼�UI�����������ʵĵ�һ������ */
	float32 fCaliVal;								/* ����ֵ */
}ACQ_F32;

typedef struct {									/* ITEM_DAT_ACQ_U32 */
	uint32 u32SetVal;								/* �˹��趨����--Ϊ�˼�UI�����������ʵĵ�һ������ */
	float32 fCaliVal;								/* ����ֵ */
}ACQ_U32;

typedef struct {	/* �����¶ȴ���������,�����ʽ: 1#�¶ȴ��������к�HHHHHHHHHHHHHH, ��ǰ�¶�: 27.8 ʱ��: 99999 ����λ��:	*/
	uint32 u32TempSenPosition;						/* ����������λ��--Ϊ�˼�UI�����������ʵĵ�һ������ */
	uint32 u32Count_TempSen_Change;					/* ������ͨѶ���߼����������¶ȴ������仯(������߶Ͽ�)��Ϊ������λ */
	uint64 u64TempSenRom;
}TEMP_SEN_CONF;

typedef struct {				/* ITEM_DAT_SUMM ��ز���,���±���˳�򲻵ø��� */
	int32 i32SummaryPlanNum;    /* 0:�ۺϵ�ʱ���м������ݣ��ü�������; ��0:ȡ����ֵ���ۺϵ�ʱ�����õ�����; >0ʹ���ۺ�ֵ; <0���ۺ�,��ʹ�� */
	float32 fSetVal;			/* �˹��趨����--Ϊ�˼�UI�����������ʵĵ�һ������ */
	float32 fAcqSummary;		/* �ɼ��ۺ�ֵ--Ϊ�˼����ݴ洢��������ʵĵڶ������� */
	uint32 u32SummaryRealNum;	/* �ۺ�ֵ��ʵ�������� */
	float32 fCurAcq;			/* ����ֵ */
}ACQ_SUMM;
#define ACQ_SUMM_USE_SET(i32SummaryPlanNum)		(i32SummaryPlanNum <= 0)
#define ACQ_SUMM_USE_SUMM(i32SummaryPlanNum)	(i32SummaryPlanNum > 1)
#define ACQ_SUMM_USE_CUR(i32SummaryPlanNum)		(i32SummaryPlanNum == 1)

/*===========================================================================
* 3. �洢ϵͳ�ķ��ʽӿ�: ���á���Ϣ���ɼ� �Լ�ϵͳ����: ���ݳ�ʼ�������
*==========================================================================*/
/* ���ݲ�������Ϊ: IDLE-MISSION-(RES)-END-IDLE���������ʱ���Ȳ���������󷽿�����ΪEND
	IDLE	: ���Խ������ݲ�������
	MISSION : �в�������,
	RES		: �в������
	END		: ���������յ�������������߳�ʱ�����ȴ� */
typedef enum {
	DATA_ACS_PROC				= 1,
		
	DATA_ACS_IDLE				= 0,
	DATA_ACS_RES_SUC			= -1,		/* 0xFF */
	DATA_ACS_RES_FAIL			= -2,		/* 0xFE */
	DATA_ACS_RES_FULL			= -3,		/* 0xFD */
	DATA_ACS_RES_EMPTY			= -4,		/* 0xFC */
	DATA_ACS_RES_CRC_ERR		= -5,		/* 0xFB */		
	DATA_ACS_RES_NOT_CMPLT		= -6,		/* 0xFA */
	DATA_ACS_RES_SUC_BUT_OLD	= -7,
	DATA_ACS_RES_OPEN_FILE_FAIL	= -8,
	DATA_ACS_RES_URGENT_QUIT	= -9,		/* �н����洢�����˳���ǰ���� */
	DATA_ACS_RES_PWD_ERR		= -10,		/* ����������� */
	DATA_ACS_RES_BUSY			= -11,		/* ϵͳæ */
	DATA_ACS_RES_TIMEOUT		= -12,		/* ��ʱ */
	DATA_ACS_RES_CRC_CHG		= -13,		/* CRC״̬�����仯����crc�õ�crc���ã����߷����� */
    DATA_ACS_RES_INPUT_ERR      = -14,      /* ������� */
	DATA_ACS_RES_MAX			= -15,
	
	DATA_ACS_END				= -20,		/* ��ʶ���������Ѿ����������� */
	
	DATA_ACS_READ_DATA_PAGE		= -40,
	DATA_ACS_READ_MSG			= DATA_ACS_READ_DATA_PAGE - DATA_PAGE_MSG,
	DATA_ACS_READ_ACQ			= DATA_ACS_READ_DATA_PAGE - DATA_PAGE_ACQ,

	DATA_ACS_RST_MSG 			= -59,		/* ϵͳ���� */
	DATA_ACS_REQ				= -60,
	DATA_ACS_CHK_GLB_CONF		= -61,
	DATA_ACS_PUSH_GLB_CONF		= -62,
	DATA_ACS_RST_ACQ 			= -63,
	DATA_ACS_RST_STAT			= -64
}DATA_ACS_OPR;
#define DATA_ACS_RES(State)			((DATA_ACS_END <= State) && (State < 0))
#define DATA_ACS_MISSION(State)		(State <= DATA_ACS_READ_DATA_PAGE)
EXT const char* cnst_DataAcsOprRes[][MAX_LANG_TYPE]
#ifndef _MDL_SYS_C_
;
#else
= {	/* ���ݴ����� */
    {"******", "******"},
	{"�ɹ�", "success"},
	{"ʧ��", "fail"},
	{"�洢������", "storage full"},
	{"����Դ��", "empty"},
	{"crcУ��ʧ��", "crc error"},
	{"�������", "NOT complete "},
	{"�ɹ����汾����", "success but version old"},
	{"���ļ�ʧ��", "open file error"},
	{"��������,�˳�����", "urgent quit"},
	{"�������", "password error"},
	{"ϵͳæ", "sys busy, abort"},
	{"��ʱ", "please wait"},
    {"CRC״̬�����仯", "please wait"},
	{"�������", "please wait"},
	{"������,���Ժ�", "please wait"},         /* DATA_ACS_RES_MAX����Ӧ��ֵ */
};
#endif

typedef enum {		/* �����û�����ͬ���û���Ҫ��ͬ�ĸ�ʽ�����ݶ� */
	DATA_USER_NULL				= 0,
	DATA_USER_MCGS,
	DATA_USER_VIEWTECH,
	DATA_USER_NET,
	DATA_USER_FILE
}DATA_USER;

typedef enum {      /* ITEM_DAT_FUNC LkId_Val_ItN��ֵ����Ӧ�Ĺ��� */
    SYS_FUNC_RST_STAT   = 0,
    SYS_FUNC_RST_ACQ    = 1,
    SYS_FUNC_ENG_DEBUG  = 2,
    MAX_SYS_FUNC
}SYS_FUNCTION;
typedef struct {									/* ���ݴ�ȡ�������ӿ� */
    int8 i8SysFunc[MAX_SYS_FUNC];

	/* conf�洢��� */
	BOOL bConfHaveUnsavedChange[MAX_SAVE_GRP_NUM];	/* ������δ������޸� */
	BOOL bConfFree[MAX_SAVE_GRP_NUM];				/* ���ڴ洢��������ȼ���ͣ������������ֻ���жϸñ�־��������� */
	DATA_ACS_OPR eConfInitRes[MAX_SAVE_GRP_NUM];	/* �ϵ��ʼ���ɹ� */

	triST tSaveUrgent_0Idle_pReq_nCmplt;			/* �����洢����Ҫ���ڼ����������߼����ϵ����� 0:�޲��� >0:����洢 <0:��ɴ洢 */
	BOOL bTestRes_TFCard;
	uint8 u8Rsvd;
}DATA_ACS_INTF;
EXT SECTION(".NOT_ZeroInit") DATA_ACS_INTF g_DataAcsIntf;

/* ��Ϣ��������Ҫ�Ľṹ */
typedef struct {
	int8 i8DataPosition;				/* ��Ϣ��ֵ����λ�ã�-1���������� */
	F32_MEANING F32Meaning;
	char* pText[MAX_LANG_TYPE];		/* ��Ϣ�ı� */
}MSG_PROP;

/*===========================================================================
* 4. ������
	��������Ȩ:Ϊ�˰�ȫ�����벻�ô洢�ڴ�����,ֻ���������в���, 					��ͬ�Ĳ�Ʒ���ò�һ��������
	GetAuthKey()���ڲ�Ʒ���룬ChkAuthKey()����У�����룬��ͬ�Ĳ�Ʒ��Ҫ�޸ģ��Բ�����һ��������
*==========================================================================*/
typedef struct {									/* ���ñ����洢��ַ */
	uint32 u32OrigEEAddr;							/* ����EEProm��ַ(ByteΪ������Ԫ) */
	uint32 u32BackEEAddr;							/* ����EEProm��ַ(ByteΪ������Ԫ) */
	const uint8* pOrigPath;							/* �Ѿ�����ʹ���ˣ�����Ϊ������ʱ����Լ����ϰ汾 */
	const uint8* pBackPath;							/* �Ѿ�����ʹ���ˣ�����Ϊ������ʱ����Լ����ϰ汾 */
}CONF_ACS_MEDIA;
typedef struct {									/* ������ñ��������̶�����0x4000λ�� */
	uint32 _c_int00_add;							/* _c_int00()�����ĵ�ַ��BootLoaderͨ���ñ�����ת��������� */
	CONF_n_STAT_PROP* pConfProp;					/* ���ñ������Ա��ַ */
	uint32 u32ItemNum;								/* ���ñ������� */
	uint32 u32ConfSaveGroupNum; 					/*	*/
	CONF_ACS_MEDIA AcsMedia[MAX_SAVE_GRP_NUM];
}BOOT_LOADER_CONF;

typedef enum {			/* ������Ȩ���ͣ�AUTH_AES_*** Ϊ4��32bit������ */
	AUTH_KEY_FLASH_REQ_KEY = 0,
	AUTH_AES_FLASH_REQ,
	AUTH_AES_FLASH_DATA,
	AUTH_AES_PACK_SERIAL,
	AUTH_AES_PACK_LICENSE,
	AUTH_KEY_FLASH_INTEGRITY_V1,/* ��YKF2/3,YYD��Ҫ������Ҫ���ֲ��� */
	AUTH_KEY_FLASH_INTEGRITY_V2,/* �����������ֵ���Ը��� */
	AUTH_AES_COMM_SN_LIC,		/* ���������ĸ�洫�� sn_new\lic ��Ϣ */
	AUTH_TYPE_NUM
}AUTH_TYPE;
/* Ϊ��֧��������ģ���Ҫ�洢����   ���룬���������AuthType = AUTH_*** + AUTH_TYPE_NUM*GrpNo
   ���µ�GrpNo=0�����µ�GrpNo=1... */
EXT void GetAuthKey(AUTH_TYPE AuthType, uint32* pAuthKey);
EXT BOOL ChkAuthKey(AUTH_TYPE AuthType, uint32* pAuthKey);

typedef struct {				/* ��������洢��eeprom EEADD_BOOT_SOFT_VERSION �� */
	uint32 u32RunVersion;		/* ָ�������еİ汾 */
	uint32 u32LowFlashVer;		/* �͵�ַ�ռ�İ汾, ��ȡ�ߵ�16λ��������ʽУ�� */
	uint32 u32HighFlashVer;		/* �ߵ�ַ�ռ�İ汾, ��ȡ�ߵ�16λ��������ʽУ�� */
}BOOT_SOFT_VERSION;
/*===========================================================================
 * 5. ����/վ��/Զ�� ���ݽ���/����ָ��, ���Խ��մ���CAN/RS485/��̫���������ݣ���˲�������
 *==========================================================================*/
#define MAX_YKF_NUM						12		/* һ����վ������������ */
#define MAX_YYB_NUM						6		/* һ����վ������ѹ������ */

/* Զ�̴���������������mqtt(����)�����ˡ�����������̨(UART) */
#define MAX_RMT_SEN_NUM	2
#define MAX_TRY_CNT_EXT_RMT_SEN_FROM_MQTT	10		/* ������ʱ(��) */
typedef struct {
	uint16 uNum_DataValid;				/* ���fSenData�У��м�����������Ч�� */
	uint16 uRsvd;
	uint32 u32RcvTime_RTCSeconds;		/* ����յ����ݵ�ʱ�䣬��RTC          32λ�������Ϊ��׼ */
	float32	fDCPwrVol;					/* �������Դ��ѹ */
	float32 fSenData[MAX_RMT_SEN_NUM];	/* ���������� */
}RMT_SEN_DAT;

/***************************************************************************
 					functions must be driven by extern
***************************************************************************/
/* <NULL> */


/***************************************************************************
 					functions could be used as extern
***************************************************************************/
//__weak void InitMdlCtr(void);			from InitYxx()
//__weak void RunMdlCtr(void);			from RunYxx()
//__weak void DrvMdlCtrTick_1KHz(void);	from DrvYxxTick_1KHz()

/************   exclude redefinition and c++ environment   ****************/
#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */

/*===========================================================================
 * ���: ��ͬ���豸��������ӳ��
 *==========================================================================*/
#if CPU_0ST_1TI
    #include "DrvTI.h"
#else
    #include "DrvST.h"
	#include "BoardSupport.h"
#endif
#if ((DEVICE_TYPE == V5_YBT2) || (DEVICE_TYPE == V5_YBT3) || (DEVICE_TYPE == V5_YBT4))
	#include "MdlYBT.h"
#elif (DEVICE_TYPE == V5_YYT3) || (DEVICE_TYPE == V5_YYT4)
	#include "MdlYYT.h"
#elif ((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4))
	#include "MdlYBU.h"
#elif (Is_YKF(DEVICE_TYPE) || (DEVICE_TYPE == V5_YYD) || (DEVICE_TYPE == V5_YYD2) || (DEVICE_TYPE == V5_YYG))
	#include "MdlYKF.h"
#elif (DEVICE_TYPE == V5_YYB)
	#include "MdlYYB.h"
#elif (DEVICE_TYPE == YKC)
	#include "MdlYKC.h"
#endif

/******************************** FILE END ********************************/
