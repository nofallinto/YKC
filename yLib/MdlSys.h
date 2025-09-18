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
    uint32 u32ChipIdHash;               /* 芯片ID Hash值 */
}U32_WITH_IDHash;

typedef struct {
    /* flash hash值，需要运行一下软件才能得到 */
	uint32 u32FlashHash_OrigEEAddr;     /* flash Hash值,与cnst_BootLoaderConf.AcsMedia[0].OrigEEAddr对比 */
	uint32 u32FlashHash_BackEEAddr;     /* flash Hash值,与cnst_BootLoaderConf.AcsMedia[0].BackEEAddr对比 */
	/* 以下是配置项 */
	U32_WITH_IDHash SerialNo;           /* 序列号 */
	uint32 u32LocalLang;				/* 本地所用语言，指Gui */
	uint32 u32License_Seconds;			/* 授权使用时间 */

	/* 启动状态 */
	uint16 uRstCount;					/* 复位计数器. 0:上电复位; >0: 其他 */
	BOOL bCtrMdlOK;
	triST tReboot_0Null_nUrgent_pWaitIdle;	/* 设备需要重启 0:无需, <0:立即(用于测试), >0:等待空闲 */
	uint8 u8Tmr_RstExtNet_tick;			/* 以控制任务驱动，复位外部网络设备 */
	uint8 u8CpuLoad;					/* CPU负载 */
	uint16 uTmr_EraseFlash_DistPwr_ms;	/* 擦除flash引起的电源电压不稳 */
	/* 软件完整性检查: <0检查延时; >0失败代码 b0序列号位数不够8位, b1授权与芯片ID不符 b2 flash校验出错 , 具体处理依赖各产品代码, 0正常 */
	int16 iTimer_nSoftChkWaitTick_pErrCode;

	/* RTC所需 */
	int16 iDaysToLicDeadLine;			/* 到授权日期. >0: 还剩多少天; <0: 过期多少天; 0:正常 */
	uint32 u32Seconds_LastSyncRTC;		/* 上一次RTC同步时间，含SNTP,GPS同步 */

#if CPU_0ST_1TI == 0
	uint32 u32UpTimer_s;				/* 重上电时间(目前主要用于启动失败的自动回滚) */
	uint32 u32RstTimer_s;				/* 启动时间 */
	uint16 uTmr_1Hz;					/* 1Hz计时器(目前主要用于启动失败的自动回滚, 日后还可以作为查询上线时间的依据:当前时间减去此值) */
#endif
}SYS_VAR;
EXT SECTION(".NOT_ZeroInit") SYS_VAR g_Sys;
/*===========================================================================
*  6. g_Ctr: 主控制器状态，主要用于显示
*==========================================================================*/
typedef struct {                            /* modbus通讯地址: 1010, 访问方式32b */
    uint32      u32CtrState;                /* 不同的产品，需要重新定义该变量，modbus地址1010 */
    uint8       u8RunMode;                  /* 运行模式, modbus地址1012 b0~b7 */
    BOOL        bEngDebug;                  /* 工程调试, modbus地址1012 b7~b15 */
    BOOL        bBeepAbornmal;              /* 异常报警，需要人工复位才能停止, modbus地址1012 b16~b23 */
    BOOL        bBeepProgram;               /* 程序启动预警，定时自动复位, modbus地址1012 b24~b31 */
    uint32      u32DinDat;                  /* 开入, modbus地址1014 */
    uint32      u32RelayDat[(MAX_RELAY_NUM+31)/32]; /* 开出, modbus地址1016 */

    /* 无需modbus通讯 */
    uint32      u32CommStatus;              /* 设备通讯状态，COMM_DEV_STATUS_VAR定义了各位 */
}CTR_VAR;
EXT SECTION(".NOT_ZeroInit") CTR_VAR g_Ctr;

/* 延时若干个cpu tick */
EXT void Boot_SysCtlDelay(uint32 ui32Count);    /*  */

/* JTAG 与 WatchDog部分 */
EXT BOOL CheckDebugVersion(void);	/* 判断当前运行版本 */

/* 获得芯片ID地址 */
EXT uint32 GetChipIDHash(void);

/*===========================================================================
 * 软件升级所需:	运行版->测试版，仅能够通过RS485口进行；测试版->运行版仅能够通过网口进行
 *==========================================================================*/
#define FLASH_FRAG_DEFAULT_BLEN		512		/* 运行软件，通过以太网进行传输，转成ASCII后不得超过以太网的一帧 */
typedef struct {
	uint32 u32FlashFragBLen;
	BOOL bWhichHalf_0Fst_1Sec;		/* 上一半还是下一半软件 */
	uint8 u8Rand;
	BOOL bSoft_Run1_Test0;
	BOOL bIniSoftUpdate;			/* 初始化升级 */
	uint32 u32FlashReqKey;
	uint32 u32FlashAddr;

	/* 以下是非加密部分 */
	uint16 uFlashFragNo;			/* 代码片序号 */
	uint16 uFlashFragNum;			/* 代码片总量 */
}FLASH_REQ;
#define FLASH_REQ_ENCRYPT_LEN		(sizeof(FLASH_REQ) - 4)

/* 以下是FLASH_PACK原型，为了支持长短不一的单次Flash传输长度，u32FlashData可能不完全一样
   为了方便AES加解密，FLASH_PACK长度必须为128bit整数倍；
   为了方便flash编程，单次传输Flash代码片长度必须2^n */
typedef struct {
	uint16 uRand1;
	uint8 u8Rand2;
	BOOL bWhichHalf_0Fst_1Sec;
	uint32 u32FlashData[0];
	uint32 u32SerialNo;
	uint32 u32ReqSoftVer;
	uint32 u32FlashAddr;
}FLASH_PACK;

/* 软件升级控制器 */
typedef struct {
	uint32 u32FlashEndAddr; 				/* flash升级结束地址, 为0表示尚未开始升级，或者已经结束了升级, 1代表升级完成，2代表升级失败 */
	uint32 u32FlashCurAddr; 				/* flash升级当前地址 */
	uint16 uReqSoftVer; 					/* 请求的软件版本号 */
	uint16 uTmr_ReqWait_ms; 				/* 等待新的代码片的定时器 */

	uint32 u32LastFragAddr;                 /* 上一片代码片地址 */
}SOFT_UPDATE_CTR;
EXT SOFT_UPDATE_CTR g_SoftUpdateCtr;

typedef enum {
	UPDATE_NULL 		= 0,
	UPDATE_CMPLT,				/* 软件升级完成，校验成功 */
	UPDATE_FAIL, 				/* 软件升级失败 */
	MAX_UPDATE_STATUS
}SOFT_UPDATE_RES;

/* 发布软件升级 */
typedef struct {
	/*modbus通讯实际读出u8PubSoftPwdGrpNum在前，u8PubSnLicPwdGrpNo在后，这两个支持modbus多次修改 */
	uint8 u8PubSnLicPwdGrpNo;		/* 发布SN,Lic所用的密码序号，即总组中第几组 */
	uint8 u8PubSoftPwdGrpNum;		/* 发布软件的密码总组数 */
	/* 以下变量，进行modbusRTU通讯时，只能写入一次 */
	uint16 uPubSoftUpdateCnt;		/* 升级运行软件计数器 */
	uint16 uPubSoftInstallCnt;		/* 从测试软件到运行软件计数器 */
	uint16 uPubSnCnt;				/* 发布SN计数器 */
	uint16 uPubLicCnt;				/* 发布License计数器 */
	uint16 uTmr_IniPubCntNoPwd_ms;	/* 无密码初始化发布计数器的定时器 */
	/* 母版指示升级状态，主要是测试版本需要 */
	uint16 uFlashFragNo;
	uint16 uFlashFragNum;			/* 代码片总量 */
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
#define MAX_FLASH_REQ_WAIT_TIME_ms	10000
EXT BOOL ProcMqttUpdate(MQTT_COMM* pMqttComm, uint8* pU8Topic, uint8* pU8TopicEnd, uint8* pU8Msg, uint8* pU8MsgEnd);
EXT BOOL CreateFlashReq(FLASH_REQ* pFlashReq, uint32 u32MaxFlashPackBLen);
EXT BOOL QueryAndPubFlashReq(MQTT_COMM* pMqttComm, BOOL bNeedWait);
EXT BOOL IniSoftUpdate(uint16 uSoftVersion);
EXT BOOL UpdateSoft(uint32* pU32FlashPack, uint32 u32FlashPackBLen);
EXT uint32 PubSoftware(FLASH_REQ* pFlashReq, uint32* pU32FlashPack, uint32 u32MaxFlashPackBLen, uint32 u32SerialNo);
EXT BOOL ProcMqttAuth_SOFT(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd, uint32 u32TopicSN);

/* 软件版本管理所需 */
EXT void FixSoftVerAndEEConf(void);

/* 序列号与授权所需 */
EXT BOOL ProcMqttAuth_SN(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd);
EXT BOOL ProcMqttAuth_LIC(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd);

/* 普通板子<>母板切换所需 */
EXT void ProcVersionMasterBCD(uint8* pRxBuf, uint16 uRegNum);
EXT BOOL ProcMqttAuth_CMD(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd);

/* 判断是否软件更新引起的重启 */
EXT BOOL IsRebootBySoftUpdate(void);

/* debug数据记录工具 */
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
	uint16 uCodeTestSW; 							/* 测试开关 */
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
/* CPU运行时间测量工具 */
EXT void RunTime_StartCali(uint8 u8Chn);
EXT uint32 RunTime_EndCali(uint8 u8Chn);
EXT uint32 RunTime_Interval(uint8 u8Chn);

/* debug数据记录工具 */
EXT void Rec_DebugChn(uint8 u8RecChn, int32 i32Dat);
EXT void Rec_DebugPack(uint32 u32Flag, float32 fDat0, float32 fDat1, float32 fDat2, 
					float32 fDat3, float32 fDat4, float32 fDat5, float32 fDat6, 
					float32 fDat7, float32 fDat8, float32 fDat9, float32 fDat10);
EXT void CopyDebugChnToTxBuf(uint8** ppTxBuf, uint16 uRegOff);
EXT void TryDebugPack(uint8 u8UartPort);	/* RS485通讯函数，用于通过485通讯接口访问DebugPack数据 */
EXT void RunDebugPack(uint8 u8UartPort);	/* RS485通讯函数，用于通过485通讯接口访问DebugPack数据 */

/* 通过mqtt的调试接口 */
EXT BOOL PrintStringByMqtt(const char* pString);		/* 把字符串通过mqtt打印出去 */
EXT BOOL PubCrashLog(MQTT_COMM* pMqttComm);
EXT void PrintBufByMqtt(const uint8* pMsg, const uint8 u8MsgSeq, const uint8* pBuf, uint16 uByteLen);

/************   exclude redefinition and c++ environment   ****************/
#ifdef __cplusplus
}
#endif 					/* extern "C" */

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
/******************************** FILE END ********************************/
