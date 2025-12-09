/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved
 *				: 
 * File			: DrvST.h
 * Author		: Wang Renfei
 * Description	:
 * Comments		:
 * Date			: 2014-11-24
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 **************************************************************************/

/***************************************************************************
 				头部：exclude redefinition and c++ environment
***************************************************************************/
#ifndef _DRV_ST_H_
#define _DRV_ST_H_

#define USE_DC_CORR_2           TRUE                /* 使用ANALOG_SIG_CORR2矫正 */
/* 这个位置定义软件是发布版(RDP1)还是debug版(RDP0) */
//#define SOFT_RELEASE1_DEBUG0	((!SOFT_RUN1_TEST0) || 0)	/* 确保测试版断开JTAG，免得改两个地方 */
#define SOFT_RELEASE1_DEBUG0 0

/*===========================================================================
 * 包含的文件
 *==========================================================================*/
//#include "GlobalVar.h"
#include "cmsis_os.h"
#if (DEVICE_TYPE == YKC)
#include "stm32f1xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif
#include "BoardSupport.h"
//#include "MdlNet.h"

#include MspIncludeFile

#ifdef _DRVST_C_
	#define EXT
#else
	#define EXT extern
#endif

typedef uint32              UInt;

/***************************************************************************
		 freeRTOS 操作系统接口
***************************************************************************/
/* freeRTOS 操作系统接口 */
#define Task_sleep(time_ms)                     osDelay(time_ms)
#define Task_setPri(taskHandle, Pri)			osThreadSetPriority(taskHandle, Pri)		/* 参数类型: osThreadId, osPriority */
#define Semaphore_pend(Sem, u32MaxPendTime_ms)  (osOK==osSemaphoreWait(Sem, u32MaxPendTime_ms))
#define Semaphore_post(Sem)                     osSemaphoreRelease(Sem)
#define SEM_Handle                              osSemaphoreId
#define TSK_Handle                              osThreadId
#define BIOS_WAIT_FOREVER                       0xFFFFFFFFUL
#define Swi_disable()							vPortEnterCritical()
#define Swi_enable()							vPortExitCritical()
#define Hwi_disable(void)                       portSET_INTERRUPT_MASK_FROM_ISR()
#define Hwi_restore(HwiKey)						portCLEAR_INTERRUPT_MASK_FROM_ISR(HwiKey)
#define Hwi_enable()							portENABLE_INTERRUPTS()
#define fdClose(fd)								close(fd)
#define TASK_LOWEST_PRIORITY					osPriorityLow		/* TI:2 */

EXT TSK_Handle TaskCreate(char *pName, os_pthread func, void *pArgs, osPriority prio, uint8 u8MaxInstanceNum, uint8 *pStackBuf, uint32 u32StackLen, osStaticThreadDef_t *pCtlBlkContainer);
EXT SEM_Handle SemCreate(char* pName, uint8 u8Count, osStaticSemaphoreDef_t* pCtlBlkContainer);

#define __rev(u32Data)                          __REV(u32Data)
#define __rev16(uData)                          __REV16(uData)
#define __rbit(u32Data)                         __RBIT(u32Data)
#define SDSPI_Handle                            void*
#define Load_update()
#define Load_getCPULoad()                       0

#define Interrupt								/* 首字母大写是为避免和TI的interrupt关键字冲突 */

extern void NVIC_SystemReset(void);
#define SysCtlReset()                           NVIC_SystemReset()        /* 重启 */

/*+-------+------------+--------------+------------+--------------+-------------------+----------------+-------------------------+--------------------------------------+--+
|         | F4xx       |              | F1xx       |              | section           | 分区            | 详情                     |                                     |  |
+---------+------------+--------------+------------+--------------+-------------------+----------------+-------------------------+--------------------------------------+--+
|         | 起始地址     | 长度          | 起始地址     | 长度          |                   |                |                         |                      			   |  |
|半个flash | 0x08000000 | 16K          | 0x08000000 | 2K           | FLASH_BOOT_SEG    | BootResetVecs  | VTOR 中断向量表           |                                     |  |
|         |            |              |            |              |                   | BootSection    | bootloader              |                                     |  |
|         | 0x08004000 | 16K          | 0x08000800 | 2K           | FLASH_EEPROM_EMUL |                | EEPROM模拟               |                                     |  |
|         |            |              |            | 2K           |                   |                |                         |                                     |  |
|         |            |              |            | 2K           |                   |                |                         |                                     |  |
|         | 0x08008000 | 16K          | 0x08002000 | 2K           | FLASH_BOOT_VER    | runVer         | 运行版本号，32位，两个16位互补方式实现校验 。升级时暂时缓存FLASH_BOOT_SEG区 |  |
|         | 0x0800C000 | 半FLASH长减48K| 0x08002800  | 半FLASH长减10K| FLASH_OTHER       | softver        | 版本号，32位，两个16位互补方式实现校验                                 |  |
|         |            |              |            |              |                   | BootLoaderConf | cinit00                 |                                     |  |
+---------+------------+--------------+------------+--------------+-------------------+----------------+-------------------------+-------------------------------------+--+*/

/***************************************************************************
 							Global Include and Define
***************************************************************************/
/***************************************************************************
		复位函数, 看门狗, 延时等
***************************************************************************/
EXT void ResetFunc(void);				/* startup.s文件需要引用这个函数 */
EXT void DiscJtagAndStartWDT(void);
EXT void KickWatchDog(void);
EXT void Boot_SysCtlDelay(uint32 ui32Count);

/***************************************************************************
        启动，升级，软件版本
***************************************************************************/
#define SYS_TASK_STACK_BLEN		1024
#define CTR_TASK_STACK_BLEN		1024
typedef struct {
	uint8 										SysStackBuf[SYS_TASK_STACK_BLEN];	/* sys堆栈空间 */
	osStaticThreadDef_t 						SysTaskCtlBlk;						/* 用于静态创建sys */

	osSemaphoreId 								SEM_Ctr;							/* SEM_Handle */
	osStaticSemaphoreDef_t 						RunSemCtlBlk;						/* 用于静态创建 */
	uint8 										CtrStackBuf[CTR_TASK_STACK_BLEN];	/* ctr堆栈空间 */
	osStaticThreadDef_t 						CtrTaskCtlBlk;						/* 用于静态创建ctr */
}SYS_BLOCK;
EXT SYS_BLOCK g_SysBlock;

extern const uint32 g_u32SoftVer;				/* 软件版本号（高低16位补码校验），真身在.c里 */
extern const uint32 g_u32RunVer;				/* 运行软件版本号（高低16位补码校验）真身在.c里 */

/* F427的flash: 总共两个等比例大小的bank,可反转,读写不需要考虑反转状态，擦写需要，
 * 每个bank为8个sector大小分别是[16K/16K/16K/16K/64K/128K/128K/128K]
 * 地址: bankA：0x08000000 ~ 0x0807FFFF， bankB：0x08080000 ~ 0x080FFFFF*/
 
 /* F103的flash: 总共N个2K的page
 * 地址: 0x08000000 ~ 0x0807FFFF */

/*
F4xx		F1xx		section	分区	详情
起始地址	长度	起始地址	长度
半个flash	0x08000000	16K	0x08000000	2K	FLASH_BOOT_SEG	BootResetVecs	VTOR 中断向量表
					BootSection	bootloader
0x08004000	16K	0x08000800	2K	FLASH_EEPROM_EMUL		EEPROM模拟
			2K
			2K
0x08008000	16K	0x08002000	2K	FLASH_BOOT_VER	runVer	运行版本号，32位，两个16位互补方式实现校验, 升级时暂时缓存FLASH_BOOT_SEG区
0x0800C000	半FLASH尺寸减48K	0x08002800	半FLASH尺寸减10K	FLASH_OTHER	softver	版本号，32位，两个16位互补方式实现校验
					BootLoaderConf	cinit00
						配置变量属性表地址
						配置变量数量
						u32ConfSaveGroupNum
						CONF_ACS_MEDIA AcsMedia[MAX_SAVE_GRP_NUM
						程序正文
*/
 
//#define FLASH_HALF_BLEN					/* 见BoardSupport.H */  											/* 半个flash的长度 */
#define FLASHADD_UPGRADABLE_START			(FLASH_BASE) 	/*TI:FLASH_HALF_BLEN-BOOT_LOADER_CONF_BLEN*/	/* 可升级区域起始点（包含需要跳过的模拟EEPROM） */

/* 1，中断向量表区 */
#define FLASHADD_IVT_START					FLASH_BASE														/* 中断向量表(IVT)区地址 */
//#define FLASH_IVT_BLEN						(2*1024)													/* 中断向量表(IVT)区长度, 见McuSupport_Fxxx.h */

/* 2，模拟EEPROM区 */
#define EEADD_START							(FLASHADD_IVT_START + FLASH_IVT_BLEN)		/* TI:0 */			/* FLASH中EEPROM模拟区开始地址 */
#define EEADD_BOOT_LOADER_CONF				(EEADD_START + 0*64)											/* BootLoader所用配置变量位置，由于st是用flash模拟的EEPROM，所以st并不必须要求必须64Byte对齐，但为保持尽量兼容，所以沿用64字节 */
#define BOOT_LOADER_CONF_BLEN				0		/*TI:64bytes(1 sector)*/								/* BootLoader配置项目长度 */
#define EEADD_DEV_CONF_START				(EEADD_BOOT_LOADER_CONF + BOOT_LOADER_CONF_BLEN)				/* FLASH中EEPROM模拟区中配置存储开始地址，由于st是用flash模拟的EEPROM，所以st并不必须要求必须64Byte对齐，但为保持尽量兼容，所以沿用64字节 */
//#define EEPROM_BLEN							(6*1024)	/*(TI:96*64)*/									/* FLASH中EEPROM模拟区长度 , 见McuSupport_Fxxx.h */

/* 3，启动参数区 */
#if !USE_DUAL_BANK_SWAP
	#define BOOT_VER_START					(EEADD_START + EEPROM_BLEN)
//	#define BOOT_VER_BLEN					(2*1024)														/* FLASH中FLASH_BOOT_VER区长度 , 见McuSupport_Fxxx.h */
#endif

/* 4，程序正文区 */
#if USE_DUAL_BANK_SWAP
	#define FLASHADD_PROG_START				(EEADD_START + EEPROM_BLEN)				/*TI:FLASH_HALF_BLEN+BOOT_LOADER_CONF_BLEN*/								/* 程序的flash升级地址 */
	#define FLASH_PROG_BLEN					(FLASH_HALF_BLEN - FLASH_IVT_BLEN - EEPROM_BLEN) /*TI:FLASH_HALF_BLEN-BOOT_LOADER_CONF_BLEN*/						/* 不包括中断向量表和模拟EEPROM和所属flash sector的程序sector的flash大小 */
#else
	#define FLASHADD_PROG_START				(BOOT_VER_START + BOOT_VER_BLEN)		/*TI:FLASH_HALF_BLEN+BOOT_LOADER_CONF_BLEN*/								/* 程序的flash升级地址 */
	#define FLASH_PROG_BLEN					(FLASH_HALF_BLEN - FLASH_IVT_BLEN - EEPROM_BLEN - BOOT_VER_BLEN) /*TI:FLASH_HALF_BLEN-BOOT_LOADER_CONF_BLEN*/		/* 不包括中断向量表和模拟EEPROM和启动版本区和所属flash sector的程序sector的flash大小 */
#endif

/* 升级时需要跳过的区 */
#define FLASHADD_UPGRADE_SKIP_START			EEADD_START						/*TI:FLASH_HALF_BLEN+BOOT_LOADER_CONF_BLEN*/				/* 程序的flash升级地址 */
#if USE_DUAL_BANK_SWAP
	#define FLASH_UPGRADE_SKIP_BLEN			EEPROM_BLEN 					/*TI:FLASH_HALF_BLEN-BOOT_LOADER_CONF_BLEN*/				/* 不包括中断向量表和模拟EEPROM和所属flash sector的程序sector的flash大小 */
#else
	#define FLASH_UPGRADE_SKIP_BLEN			(EEPROM_BLEN + BOOT_VER_BLEN) 	/*TI:FLASH_HALF_BLEN-BOOT_LOADER_CONF_BLEN*/				/* 不包括中断向量表和模拟EEPROM和启动版本区和所属flash sector的程序sector的flash大小 */
#endif

extern volatile const BOOT_LOADER_CONF cnst_BootLoaderConf;
#define pBootLoaderConf 					(&cnst_BootLoaderConf)			/* 实际上就是flash映射开始运行的起始地址， TI：((BOOT_LOADER_CONF*)((void*)BOOT_LOADER_BLEN)) */
#define pOldBootLoaderConf					((BOOT_LOADER_CONF*)((uint32)(((uint8*)pBootLoaderConf) + (1-2*((uint8)CheckExeFlash_0Low_1High())) * FLASH_HALF_BLEN)))		/* 另一半地址的cnst_BootLoaderConf, 1-2*x为实现±FLASH_HALF_BLEN */
#define FLASH_UPDATEABLE_BLEN				(FLASH_HALF_BLEN)				/*TI:TI:FLASH_HALF_BLEN-BOOT_LOADER_CONF_BLEN*/							/* 可升级部分大小（注意：这个长度是包含了实际需要跳过的EEPROM持久化区） */

EXT BOOL CheckExeFlash_0Low_1High(void);
EXT BOOL CheckDebugVersion(void);											/* 判断当前运行版本 */
EXT void OnPostLiveUpdate(void);
EXT void SoftRollBack(uint32 u32RunVersion);

EXT uint32 GetVerVar(uint32 u32VersionVar);
EXT uint32* GetCounterpartAddr(uint32 *pVal);
EXT uint32 GetVolatileFromU32Addr(volatile const uint32 *pU32Val);			/* 读取指定存储器地址中当前实际的内容 */
EXT BOOL CheckSoftVer(void);												/* 检测软件版本，返回是否是全新flash上运行 */
EXT void GetSoftVersion(BOOT_SOFT_VERSION* pBootSoftVer);					/* 获取软件版本号 */
EXT void InvalidateAnotherSoft(void);										/* 破坏另一个版本的软件完整性标志 */
EXT BOOL FlashUpgradeFrag(BOOL bWhichHalf_0Fst_1Sec, uint32 u32FlashAddr, uint32* pU32FlashData, uint32 u32FlashFragBLen);		/* 更新代码片 */
EXT void CalcUid(uint32 * pU32Uid);
EXT uint32 CalcUid16B(void);
#define UID_U32LEN	4
EXT uint32 GetChipIDHash(void);
/***************************************************************************
		片上存储
***************************************************************************/
#define DATA_ACCESS_TASK_STACK_LEN	3072
typedef struct {
	osSemaphoreId 					Sem_Op;
	osStaticSemaphoreDef_t 			OpSemCtlBlk;		/* 用于静态创建 */
	osThreadId 						DataAcsTSK;
	uint8 							Stack[DATA_ACCESS_TASK_STACK_LEN];
	osStaticThreadDef_t 			TaskCtlBlk;
}DATA_ACCESS_BLOCK;
EXT DATA_ACCESS_BLOCK g_DataAccessBlock;

EXT void EEPromErase(uint32 ui32Address);
EXT void EEPROMRead(uint32 *pui32Data, uint32 ui32Address, uint32 ui32Count);
EXT uint32 EEPROMProgram(uint32 *pui32Data, uint32 ui32Address, uint32 ui32Count);

EXT void PersistSaveGrp(void);		/* 持久化SAVE_GRP */

EXT void FlashErase2(uint32 u32StartAddr, uint32 u32BLen);					/* 之所以用2是避免和TI重名 */
EXT void FlashRead2(uint32 ReadAddr, uint32 *pBuffer, uint32 u32ByteLen);	/* 之所以用2是避免和TI重名 */
EXT BOOL FlashProgram2(uint32_t *pSrc, uint32_t u32Dst, uint32_t u32BLen, BOOL bValidate);	/* 之所以用2是避免和TI重名 */
EXT uint32 FlashReadU32(uint32 faddr);	/* 之所以用2是避免和TI重名 */

/***************************************************************************
		CPU温度 & CPU Ref
***************************************************************************/
EXT float32 CalCPUTemperature(uint8 u8DCSigNo);
EXT float32 CalVrefint(uint8 u8DCSigNo);

/***************************************************************************
		RTC模块
***************************************************************************/
EXT void InitRTC(void);
EXT void GetRealTime(REAL_TIME_VAR* pRealTime);
EXT uint8 GetWeekDay(void);
EXT uint32 GetRTCSeconds(void);
EXT uint16 GetRTCMilliSec(void);
EXT void SetRTCSeconds(uint32 u32Seconds);

/***************************************************************************
		GPIO驱动
***************************************************************************/
typedef struct {
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
}GPIO_PINCFG;
EXT void GPIO_write(uint8 u8DOutNo, BOOL bHigh1_Low0);
EXT void GPIO_toggle(uint8 u8DOutNo);
EXT BOOL GPIO_read(uint8 u8DInNo);

/***************************************************************************
		Uart驱动
***************************************************************************/
#define MAX_UART_TASK_STACK_LEN		1024
typedef struct {
	uint8 									Stack[MAX_UART_TASK_STACK_LEN];
	osStaticSemaphoreDef_t 					TxSemCtlBlk;		/* 用于静态创建 */
	osStaticSemaphoreDef_t 					RxSemCtlBlk;		/* 用于静态创建 */
	osStaticThreadDef_t 					TaskCtlBlk;			/* 用于静态创建ctr */
}UART_BLOCK;
EXT UART_BLOCK g_UartBlock[MAX_UART_NUM];

#define UART_Handle							UART_HandleTypeDef*

EXT void OpenUartComm(uint8 u8Port, uint32 u32Baud, uint8 u8Parity_0Nul_1Odd_2Even, uint16 uReadTickOut_0Default);
EXT void WriteToUart(uint8 u8Port, uint16 uNeedTxByte);
EXT uint16 ReadFromUart(uint8 u8Port, uint16 uIniWaitTime_ms);

/***************************************************************************
        I2C驱动
***************************************************************************/
#if I2C_COMM_NUM
#define I2C_Handle void*
typedef struct {
	I2C_Handle Handle;
    SEM_Handle Sem_TxCplt;
    SEM_Handle Sem_RxGet;
}I2C_COMM;
EXT SECTION(".NOT_ZeroInit") I2C_COMM g_I2CComm[I2C_COMM_NUM];
EXT void OpenI2CComm(uint8 u8Port);
EXT BOOL WriteToI2C(uint8 u8Port, uint8 u8DevAdd, const uint8* pU8Dat, uint16 uBLen, uint16 uTimeOut_ms);
EXT BOOL ReadFromI2C(uint8 u8Port, uint8 u8DevAdd, uint8* pU8Dat, uint16 uBLen, uint16 uTimeOut_ms);
#endif

/***************************************************************************
		SPI驱动
***************************************************************************/
#if SPI_COMM_NUM
#define SPI_Handle			SPI_HandleTypeDef*
typedef struct {
	SPI_Handle Handle;
    SEM_Handle Sem_TxCplt;
    SEM_Handle Sem_RxGet;
}SPI_COMM;
EXT SECTION(".NOT_ZeroInit") SPI_COMM g_SPIComm[SPI_COMM_NUM];
EXT void OpenSPIComm(uint8 u8Port);
EXT BOOL WriteToSPI(uint8 u8Port, const uint8 *pU8, uint16 uBLen, uint16 uTimeOut_ms);
EXT BOOL ReadFromSPI(uint8 u8Port, uint8 *pU8, uint16 uBLen, uint16 uTimeOut_ms);
#endif

/***************************************************************************
        can驱动
***************************************************************************/
EXT void InitCanFilter(void* hCan);
EXT void SetCanMsgFilter(void* hCan, uint32 u32MsgID, uint32 u32Mask, uint8 u8RxInt);
EXT void WriteDataToCan(void* hCan, uint8 u8DataGroup, uint8 u8DevAdd, uint16 uDataPt, uint8* pU8Dat, uint8 u8ByteNum, BOOL bTxIntEnable);
EXT void ChkCanBusOff(void);

/***************************************************************************
		网络
***************************************************************************/

#define NET_TASK_STACK_BLEN			2048
typedef struct {
	osSemaphoreId 					Sem_DnsReq;
	osStaticSemaphoreDef_t 			DnsSemCtlBlk;						/* 用于静态创建 */
	osStaticThreadDef_t 			ModbusTCPServerTaskCtlBlk;			/* 用于静态创建 */
}NET_BLOCK;
EXT NET_BLOCK g_NetBlock;

EXT osStaticSemaphoreDef_t MqttPubReqSemCtlBlk;							/* 用于静态创建 */

#define	SOCKET						int32_t          					/* OS Socket Type */
#define INVALID_SOCKET 				(-1)   								/* Used by socket() and accept() */
EXT int GetDnsAddrFromDhcpRecord(uint8 u8ServerIdx, uint8 *pIPServer);
EXT void NetworkLoop(void);

typedef struct {
	osStaticThreadDef_t 			StackCtlBlk;						/* 用于静态创建 */
}MQTT_BLOCK;
EXT MQTT_BLOCK g_MqttBlock[2];	//MQTT_TASK_NUM

/***************************************************************************
		TF卡存储
***************************************************************************/
EXT void InitSDSPI(void);

/***************************************************************************
		临时性
***************************************************************************/
EXT float32 CalCPUTemperature(uint8 u8DCSigNo);

/************   exclude redefinition    ****************/
#undef EXT				/* release EXT */
#endif /* _DRV_ST_H_ */
