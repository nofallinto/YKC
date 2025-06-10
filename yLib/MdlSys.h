/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved
 * File			: MdlSys.h
 * Author		: Wang Renfei
 * Description	: 
 *				: 
 * Comments		:
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
#ifndef _MDL_SYS_H_
#define _MDL_SYS_H_

#ifdef __cplusplus
extern "C" {
#endif


/***************************************************************************
 							include files
***************************************************************************/
#include "GlobalVar.h"
#include "MdlNet.h"
#include "BoardSupport.h"


/***************************************************************************
				hardware parameter and global definitions
***************************************************************************/


/***************************************************************************
 		use EXT to distinguish stencil.c or others source files
***************************************************************************/
#ifdef _MDL_SYS_C_
#define EXT
#else
#define EXT extern
#endif

/***************************************************************************
 					functions must be driven by extern
***************************************************************************/
EXT void SysTask(void const * pArgs);

/***************************************************************************
 					variables could be used as extern
***************************************************************************/
typedef struct {
    uint32 u32Dat;
    uint32 u32ChipIdHash;               /* оƬID Hashֵ */
}U32_WITH_IDHash;

typedef struct {
    /* flash hashֵ����Ҫ����һ��������ܵõ� */
	uint32 u32FlashHash_OrigEEAddr;     /* flash Hashֵ,��cnst_BootLoaderConf.AcsMedia[0].OrigEEAddr�Ա� */
	uint32 u32FlashHash_BackEEAddr;     /* flash Hashֵ,��cnst_BootLoaderConf.AcsMedia[0].BackEEAddr�Ա� */
	/* ������������ */
	U32_WITH_IDHash SerialNo;           /* ���к� */
	uint32 u32LocalLang;				/* �����������ԣ�ָGui */
	uint32 u32License_Seconds;			/* ��Ȩʹ��ʱ�� */

	/* ����״̬ */
	uint16 uRstCount;					/* ��λ������. 0:�ϵ縴λ; >0: ���� */
	BOOL bCtrMdlOK;
	triST tReboot_0Null_nUrgent_pWaitIdle;	/* �豸��Ҫ���� 0:����, <0:����(���ڲ���), >0:�ȴ����� */
	uint8 u8Tmr_RstExtNet_tick;			/* �Կ���������������λ�ⲿ�����豸 */
	uint8 u8CpuLoad;					/* CPU���� */
	uint16 uTmr_EraseFlash_DistPwr_ms;	/* ����flash����ĵ�Դ��ѹ���� */
	/* ��������Լ��: <0�����ʱ; >0ʧ�ܴ��� b0���к�λ������8λ, b1��Ȩ��оƬID���� b2 flashУ����� , ���崦����������Ʒ����, 0���� */
	int16 iTimer_nSoftChkWaitTick_pErrCode;

	/* RTC���� */
	int16 iDaysToLicDeadLine;			/* ����Ȩ����. >0: ��ʣ������; <0: ���ڶ�����; 0:���� */
	uint32 u32Seconds_LastSyncRTC;		/* ��һ��RTCͬ��ʱ�䣬��SNTP,GPSͬ�� */

#if CPU_0ST_1TI == 0
	uint32 u32UpTimer_s;				/* ���ϵ�ʱ��(Ŀǰ��Ҫ��������ʧ�ܵ��Զ��ع�) */
	uint32 u32RstTimer_s;				/* ����ʱ�� */
	uint16 uTmr_1Hz;					/* 1Hz��ʱ��(Ŀǰ��Ҫ��������ʧ�ܵ��Զ��ع�, �պ󻹿�����Ϊ��ѯ����ʱ�������:��ǰʱ���ȥ��ֵ) */
#endif
}SYS_VAR;
EXT SECTION(".NOT_ZeroInit") SYS_VAR g_Sys;
/*===========================================================================
*  6. g_Ctr: ��������״̬����Ҫ������ʾ
*==========================================================================*/
typedef struct {                            /* modbusͨѶ��ַ: 1010, ���ʷ�ʽ32b */
    uint32      u32CtrState;                /* ��ͬ�Ĳ�Ʒ����Ҫ���¶���ñ�����modbus��ַ1010 */
    uint8       u8RunMode;                  /* ����ģʽ, modbus��ַ1012 b0~b7 */
    BOOL        bEngDebug;                  /* ���̵���, modbus��ַ1012 b7~b15 */
    BOOL        bBeepAbornmal;              /* �쳣��������Ҫ�˹���λ����ֹͣ, modbus��ַ1012 b16~b23 */
    BOOL        bBeepProgram;               /* ��������Ԥ������ʱ�Զ���λ, modbus��ַ1012 b24~b31 */
    uint32      u32DinDat;                  /* ����, modbus��ַ1014 */
    uint32      u32RelayDat[(MAX_RELAY_NUM+31)/32]; /* ����, modbus��ַ1016 */

    /* ����modbusͨѶ */
    uint32      u32CommStatus;              /* �豸ͨѶ״̬��COMM_DEV_STATUS_VAR�����˸�λ */
}CTR_VAR;
EXT SECTION(".NOT_ZeroInit") CTR_VAR g_Ctr;

/* ��ʱ���ɸ�cpu tick */
EXT void Boot_SysCtlDelay(uint32 ui32Count);    /*  */

/* JTAG �� WatchDog���� */
EXT BOOL CheckDebugVersion(void);	/* �жϵ�ǰ���а汾 */

/* ���оƬID��ַ */
EXT uint32 GetChipIDHash(void);

/*===========================================================================
 * �����������:	���а�->���԰棬���ܹ�ͨ��RS485�ڽ��У����԰�->���а���ܹ�ͨ�����ڽ���
 *==========================================================================*/
#define FLASH_FRAG_DEFAULT_BLEN		512		/* ���������ͨ����̫�����д��䣬ת��ASCII�󲻵ó�����̫����һ֡ */
typedef struct {
	uint32 u32FlashFragBLen;
	BOOL bWhichHalf_0Fst_1Sec;		/* ��һ�뻹����һ����� */
	uint8 u8Rand;
	BOOL bSoft_Run1_Test0;
	BOOL bIniSoftUpdate;			/* ��ʼ������ */
	uint32 u32FlashReqKey;
	uint32 u32FlashAddr;

	/* �����ǷǼ��ܲ��� */
	uint16 uFlashFragNo;			/* ����Ƭ��� */
	uint16 uFlashFragNum;			/* ����Ƭ���� */
}FLASH_REQ;
#define FLASH_REQ_ENCRYPT_LEN		(sizeof(FLASH_REQ) - 4)

/* ������FLASH_PACKԭ�ͣ�Ϊ��֧�ֳ��̲�һ�ĵ���Flash���䳤�ȣ�u32FlashData���ܲ���ȫһ��
   Ϊ�˷���AES�ӽ��ܣ�FLASH_PACK���ȱ���Ϊ128bit��������
   Ϊ�˷���flash��̣����δ���Flash����Ƭ���ȱ���2^n */
typedef struct {
	uint16 uRand1;
	uint8 u8Rand2;
	BOOL bWhichHalf_0Fst_1Sec;
	uint32 u32FlashData[0];
	uint32 u32SerialNo;
	uint32 u32ReqSoftVer;
	uint32 u32FlashAddr;
}FLASH_PACK;

/* ������������� */
typedef struct {
	uint32 u32FlashEndAddr; 				/* flash����������ַ, Ϊ0��ʾ��δ��ʼ�����������Ѿ�����������, 1����������ɣ�2��������ʧ�� */
	uint32 u32FlashCurAddr; 				/* flash������ǰ��ַ */
	uint16 uReqSoftVer; 					/* ���������汾�� */
	uint16 uTmr_ReqWait_ms; 				/* �ȴ��µĴ���Ƭ�Ķ�ʱ�� */

	uint32 u32LastFragAddr;                 /* ��һƬ����Ƭ��ַ */
}SOFT_UPDATE_CTR;
EXT SOFT_UPDATE_CTR g_SoftUpdateCtr;

typedef enum {
	UPDATE_NULL 		= 0,
	UPDATE_CMPLT,				/* ���������ɣ�У��ɹ� */
	UPDATE_FAIL, 				/* �������ʧ�� */
	MAX_UPDATE_STATUS
}SOFT_UPDATE_RES;

/* ����������� */
typedef struct {
	/*modbusͨѶʵ�ʶ���u8PubSoftPwdGrpNum��ǰ��u8PubSnLicPwdGrpNo�ں�������֧��modbus����޸� */
	uint8 u8PubSnLicPwdGrpNo;		/* ����SN,Lic���õ�������ţ��������еڼ��� */
	uint8 u8PubSoftPwdGrpNum;		/* ������������������� */
	/* ���±���������modbusRTUͨѶʱ��ֻ��д��һ�� */
	uint16 uPubSoftUpdateCnt;		/* ����������������� */
	uint16 uPubSoftInstallCnt;		/* �Ӳ��������������������� */
	uint16 uPubSnCnt;				/* ����SN������ */
	uint16 uPubLicCnt;				/* ����License������ */
	uint16 uTmr_IniPubCntNoPwd_ms;	/* �������ʼ�������������Ķ�ʱ�� */
	/* ĸ��ָʾ����״̬����Ҫ�ǲ��԰汾��Ҫ */
	uint16 uFlashFragNo;
	uint16 uFlashFragNum;			/* ����Ƭ���� */
	uint32 u32SnLicCommAesKey[2];
}PUB_SOFT_AUTH_CTR;
EXT PUB_SOFT_AUTH_CTR g_PubSoftAuthCtr;

typedef struct {
	uint32 u32IniSerialNo;
	uint32 u32NewSerialNo;
	uint32 u32AuthTime;
	uint32 u32Rand;
}SERIAL_PACK;

typedef struct {
	uint32 u32DevSerialNo;
	uint32 u32License_Seconds;
	uint32 u32AuthTime;
	uint32 u32Rand;
}LICENSE_PACK;

#define MQTT_FLASH_PACK_ASCII_BLEN	((MQTT_TR_BUF_BLEN - 128)/2)
#define REBOOT_BY_SOFT_UPDATE		0xAAAAAAAAUL
#define MAX_FLASH_REQ_WAIT_TIME_ms	1000
EXT BOOL ProcMqttUpdate(MQTT_COMM* pMqttComm, uint8* pU8Topic, uint8* pU8TopicEnd, uint8* pU8Msg, uint8* pU8MsgEnd);
EXT BOOL CreateFlashReq(FLASH_REQ* pFlashReq, uint32 u32MaxFlashPackBLen);
EXT BOOL QueryAndPubFlashReq(MQTT_COMM* pMqttComm, BOOL bNeedWait);
EXT BOOL IniSoftUpdate(uint16 uSoftVersion);
EXT BOOL UpdateSoft(uint32* pU32FlashPack, uint32 u32FlashPackBLen);
EXT uint32 PubSoftware(FLASH_REQ* pFlashReq, uint32* pU32FlashPack, uint32 u32MaxFlashPackBLen, uint32 u32SerialNo);
EXT BOOL ProcMqttAuth_SOFT(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd, uint32 u32TopicSN);

/* ����汾�������� */
EXT void FixSoftVerAndEEConf(void);

/* ���к�����Ȩ���� */
EXT BOOL ProcMqttAuth_SN(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd);
EXT BOOL ProcMqttAuth_LIC(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd);

/* ��ͨ����<>ĸ���л����� */
EXT void ProcVersionMasterBCD(uint8* pRxBuf, uint16 uRegNum);
EXT BOOL ProcMqttAuth_CMD(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd);

/* �ж��Ƿ����������������� */
EXT BOOL IsRebootBySoftUpdate(void);

/* debug���ݼ�¼���� */
#define CODE_TEST_uFlag_NUM			16
#define CODE_TEST_UINT8_NUM			256
#define CODE_TEST_INT16_NUM			64
#define CODE_TEST_UINT16_NUM		64
#define CODE_TEST_UINT32_NUM		100
#define CODE_TEST_FLOAT32_NUM		100
#define CODE_TEST_UINT64_NUM		16
#define CODE_TEST_p_NUM				16
#define CODE_TEST_SLANT_BUF_LEN		25
typedef struct {
	uint16 uCodeTestSW; 							/* ���Կ��� */
	uint16 uFlag[CODE_TEST_uFlag_NUM];
	int16 Val[CODE_TEST_INT16_NUM];
	uint16 uVal[CODE_TEST_UINT16_NUM];
	uint16 uVal2[CODE_TEST_UINT16_NUM];
	uint16 uVal3[CODE_TEST_UINT16_NUM];
	uint32 u32Val[CODE_TEST_UINT32_NUM];
	int32 i32Val[CODE_TEST_UINT32_NUM];
	float32 fVal[CODE_TEST_FLOAT32_NUM];
	float32 fVal2[CODE_TEST_FLOAT32_NUM];
	uint8 u8Val[CODE_TEST_UINT8_NUM];
	uint8** ppTxBuf;
}CODE_TEST;
EXT	SECTION(".NOT_ZeroInit") CODE_TEST g_CodeTest;
EXT void InitDebugDat(void);
/* CPU����ʱ��������� */
EXT void RunTime_StartCali(uint8 u8Chn);
EXT uint32 RunTime_EndCali(uint8 u8Chn);
EXT uint32 RunTime_Interval(uint8 u8Chn);

/* debug���ݼ�¼���� */
EXT void Rec_DebugChn(uint8 u8RecChn, int32 i32Dat);
EXT void Rec_DebugPack(uint32 u32Flag, float32 fDat0, float32 fDat1, float32 fDat2, 
					float32 fDat3, float32 fDat4, float32 fDat5, float32 fDat6, 
					float32 fDat7, float32 fDat8, float32 fDat9, float32 fDat10);
EXT void CopyDebugChnToTxBuf(uint8** ppTxBuf, uint16 uRegOff);
EXT void TryDebugPack(uint8 u8UartPort);	/* RS485ͨѶ����������ͨ��485ͨѶ�ӿڷ���DebugPack���� */
EXT void RunDebugPack(uint8 u8UartPort);	/* RS485ͨѶ����������ͨ��485ͨѶ�ӿڷ���DebugPack���� */

/* ͨ��mqtt�ĵ��Խӿ� */
EXT BOOL PrintStringByMqtt(const char* pString);		/* ���ַ���ͨ��mqtt��ӡ��ȥ */
EXT BOOL PubCrashLog(MQTT_COMM* pMqttComm);
EXT void PrintBufByMqtt(const uint8* pMsg, const uint8 u8MsgSeq, const uint8* pBuf, uint16 uByteLen);

/************   exclude redefinition and c++ environment   ****************/
#ifdef __cplusplus
}
#endif 					/* extern "C" */

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
/******************************** FILE END ********************************/
