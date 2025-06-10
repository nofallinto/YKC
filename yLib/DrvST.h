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
 				ͷ����exclude redefinition and c++ environment
***************************************************************************/
#ifndef _DRV_ST_H_
#define _DRV_ST_H_

#define USE_DC_CORR_2           TRUE                /* ʹ��ANALOG_SIG_CORR2���� */
/* ���λ�ö�������Ƿ�����(RDP1)����debug��(RDP0) */
//#define SOFT_RELEASE1_DEBUG0	((!SOFT_RUN1_TEST0) || 0)	/* ȷ�����԰�Ͽ�JTAG����ø������ط� */
#define SOFT_RELEASE1_DEBUG0 0

/*===========================================================================
 * �������ļ�
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
		 freeRTOS ����ϵͳ�ӿ�
***************************************************************************/
/* freeRTOS ����ϵͳ�ӿ� */
#define Task_sleep(time_ms)                     osDelay(time_ms)
#define Task_setPri(taskHandle, Pri)			osThreadSetPriority(taskHandle, Pri)		/* ��������: osThreadId, osPriority */
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

#define Interrupt								/* ����ĸ��д��Ϊ�����TI��interrupt�ؼ��ֳ�ͻ */

extern void NVIC_SystemReset(void);
#define SysCtlReset()                           NVIC_SystemReset()        /* ���� */

/*+-------+------------+--------------+------------+--------------+-------------------+----------------+-------------------------+--------------------------------------+--+
|         | F4xx       |              | F1xx       |              | section           | ����            | ����                     |                                     |  |
+---------+------------+--------------+------------+--------------+-------------------+----------------+-------------------------+--------------------------------------+--+
|         | ��ʼ��ַ     | ����          | ��ʼ��ַ     | ����          |                   |                |                         |                      			   |  |
|���flash | 0x08000000 | 16K          | 0x08000000 | 2K           | FLASH_BOOT_SEG    | BootResetVecs  | VTOR �ж�������           |                                     |  |
|         |            |              |            |              |                   | BootSection    | bootloader              |                                     |  |
|         | 0x08004000 | 16K          | 0x08000800 | 2K           | FLASH_EEPROM_EMUL |                | EEPROMģ��               |                                     |  |
|         |            |              |            | 2K           |                   |                |                         |                                     |  |
|         |            |              |            | 2K           |                   |                |                         |                                     |  |
|         | 0x08008000 | 16K          | 0x08002000 | 2K           | FLASH_BOOT_VER    | runVer         | ���а汾�ţ�32λ������16λ������ʽʵ��У�� ������ʱ��ʱ����FLASH_BOOT_SEG�� |  |
|         | 0x0800C000 | ��FLASH����48K| 0x08002800  | ��FLASH����10K| FLASH_OTHER       | softver        | �汾�ţ�32λ������16λ������ʽʵ��У��                                 |  |
|         |            |              |            |              |                   | BootLoaderConf | cinit00                 |                                     |  |
+---------+------------+--------------+------------+--------------+-------------------+----------------+-------------------------+-------------------------------------+--+*/

/***************************************************************************
 							Global Include and Define
***************************************************************************/
/***************************************************************************
		��λ����, ���Ź�, ��ʱ��
***************************************************************************/
EXT void ResetFunc(void);				/* startup.s�ļ���Ҫ����������� */
EXT void DiscJtagAndStartWDT(void);
EXT void KickWatchDog(void);
EXT void Boot_SysCtlDelay(uint32 ui32Count);

/***************************************************************************
        ����������������汾
***************************************************************************/
#define SYS_TASK_STACK_BLEN		1024
#define CTR_TASK_STACK_BLEN		1024
typedef struct {
	uint8 										SysStackBuf[SYS_TASK_STACK_BLEN];	/* sys��ջ�ռ� */
	osStaticThreadDef_t 						SysTaskCtlBlk;						/* ���ھ�̬����sys */

	osSemaphoreId 								SEM_Ctr;							/* SEM_Handle */
	osStaticSemaphoreDef_t 						RunSemCtlBlk;						/* ���ھ�̬���� */
	uint8 										CtrStackBuf[CTR_TASK_STACK_BLEN];	/* ctr��ջ�ռ� */
	osStaticThreadDef_t 						CtrTaskCtlBlk;						/* ���ھ�̬����ctr */
}SYS_BLOCK;
EXT SYS_BLOCK g_SysBlock;

extern const uint32 g_u32SoftVer;				/* ����汾�ţ��ߵ�16λ����У�飩��������.c�� */
extern const uint32 g_u32RunVer;				/* ��������汾�ţ��ߵ�16λ����У�飩������.c�� */

/* F427��flash: �ܹ������ȱ�����С��bank,�ɷ�ת,��д����Ҫ���Ƿ�ת״̬����д��Ҫ��
 * ÿ��bankΪ8��sector��С�ֱ���[16K/16K/16K/16K/64K/128K/128K/128K]
 * ��ַ: bankA��0x08000000 ~ 0x0807FFFF�� bankB��0x08080000 ~ 0x080FFFFF*/
 
 /* F103��flash: �ܹ�N��2K��page
 * ��ַ: 0x08000000 ~ 0x0807FFFF */

/*
F4xx		F1xx		section	����	����
��ʼ��ַ	����	��ʼ��ַ	����
���flash	0x08000000	16K	0x08000000	2K	FLASH_BOOT_SEG	BootResetVecs	VTOR �ж�������
					BootSection	bootloader
0x08004000	16K	0x08000800	2K	FLASH_EEPROM_EMUL		EEPROMģ��
			2K
			2K
0x08008000	16K	0x08002000	2K	FLASH_BOOT_VER	runVer	���а汾�ţ�32λ������16λ������ʽʵ��У��, ����ʱ��ʱ����FLASH_BOOT_SEG��
0x0800C000	��FLASH�ߴ��48K	0x08002800	��FLASH�ߴ��10K	FLASH_OTHER	softver	�汾�ţ�32λ������16λ������ʽʵ��У��
					BootLoaderConf	cinit00
						���ñ������Ա��ַ
						���ñ�������
						u32ConfSaveGroupNum
						CONF_ACS_MEDIA AcsMedia[MAX_SAVE_GRP_NUM
						��������
*/
 
//#define FLASH_HALF_BLEN					/* ��BoardSupport.H */  											/* ���flash�ĳ��� */
#define FLASHADD_UPGRADABLE_START			(FLASH_BASE) 	/*TI:FLASH_HALF_BLEN-BOOT_LOADER_CONF_BLEN*/	/* ������������ʼ�㣨������Ҫ������ģ��EEPROM�� */

/* 1���ж��������� */
#define FLASHADD_IVT_START					FLASH_BASE														/* �ж�������(IVT)����ַ */
//#define FLASH_IVT_BLEN						(2*1024)													/* �ж�������(IVT)������, ��McuSupport_Fxxx.h */

/* 2��ģ��EEPROM�� */
#define EEADD_START							(FLASHADD_IVT_START + FLASH_IVT_BLEN)		/* TI:0 */			/* FLASH��EEPROMģ������ʼ��ַ */
#define EEADD_BOOT_LOADER_CONF				(EEADD_START + 0*64)											/* BootLoader�������ñ���λ�ã�����st����flashģ���EEPROM������st��������Ҫ�����64Byte���룬��Ϊ���־������ݣ���������64�ֽ� */
#define BOOT_LOADER_CONF_BLEN				0		/*TI:64bytes(1 sector)*/								/* BootLoader������Ŀ���� */
#define EEADD_DEV_CONF_START				(EEADD_BOOT_LOADER_CONF + BOOT_LOADER_CONF_BLEN)				/* FLASH��EEPROMģ���������ô洢��ʼ��ַ������st����flashģ���EEPROM������st��������Ҫ�����64Byte���룬��Ϊ���־������ݣ���������64�ֽ� */
//#define EEPROM_BLEN							(6*1024)	/*(TI:96*64)*/									/* FLASH��EEPROMģ�������� , ��McuSupport_Fxxx.h */

/* 3������������ */
#if !USE_DUAL_BANK_SWAP
	#define BOOT_VER_START					(EEADD_START + EEPROM_BLEN)
//	#define BOOT_VER_BLEN					(2*1024)														/* FLASH��FLASH_BOOT_VER������ , ��McuSupport_Fxxx.h */
#endif

/* 4������������ */
#if USE_DUAL_BANK_SWAP
	#define FLASHADD_PROG_START				(EEADD_START + EEPROM_BLEN)				/*TI:FLASH_HALF_BLEN+BOOT_LOADER_CONF_BLEN*/								/* �����flash������ַ */
	#define FLASH_PROG_BLEN					(FLASH_HALF_BLEN - FLASH_IVT_BLEN - EEPROM_BLEN) /*TI:FLASH_HALF_BLEN-BOOT_LOADER_CONF_BLEN*/						/* �������ж��������ģ��EEPROM������flash sector�ĳ���sector��flash��С */
#else
	#define FLASHADD_PROG_START				(BOOT_VER_START + BOOT_VER_BLEN)		/*TI:FLASH_HALF_BLEN+BOOT_LOADER_CONF_BLEN*/								/* �����flash������ַ */
	#define FLASH_PROG_BLEN					(FLASH_HALF_BLEN - FLASH_IVT_BLEN - EEPROM_BLEN - BOOT_VER_BLEN) /*TI:FLASH_HALF_BLEN-BOOT_LOADER_CONF_BLEN*/		/* �������ж��������ģ��EEPROM�������汾��������flash sector�ĳ���sector��flash��С */
#endif

/* ����ʱ��Ҫ�������� */
#define FLASHADD_UPGRADE_SKIP_START			EEADD_START						/*TI:FLASH_HALF_BLEN+BOOT_LOADER_CONF_BLEN*/				/* �����flash������ַ */
#if USE_DUAL_BANK_SWAP
	#define FLASH_UPGRADE_SKIP_BLEN			EEPROM_BLEN 					/*TI:FLASH_HALF_BLEN-BOOT_LOADER_CONF_BLEN*/				/* �������ж��������ģ��EEPROM������flash sector�ĳ���sector��flash��С */
#else
	#define FLASH_UPGRADE_SKIP_BLEN			(EEPROM_BLEN + BOOT_VER_BLEN) 	/*TI:FLASH_HALF_BLEN-BOOT_LOADER_CONF_BLEN*/				/* �������ж��������ģ��EEPROM�������汾��������flash sector�ĳ���sector��flash��С */
#endif

extern volatile const BOOT_LOADER_CONF cnst_BootLoaderConf;
#define pBootLoaderConf 					(&cnst_BootLoaderConf)			/* ʵ���Ͼ���flashӳ�俪ʼ���е���ʼ��ַ�� TI��((BOOT_LOADER_CONF*)((void*)BOOT_LOADER_BLEN)) */
#define pOldBootLoaderConf					((BOOT_LOADER_CONF*)((uint32)(((uint8*)pBootLoaderConf) + (1-2*((uint8)CheckExeFlash_0Low_1High())) * FLASH_HALF_BLEN)))		/* ��һ���ַ��cnst_BootLoaderConf, 1-2*xΪʵ�֡�FLASH_HALF_BLEN */
#define FLASH_UPDATEABLE_BLEN				(FLASH_HALF_BLEN)				/*TI:TI:FLASH_HALF_BLEN-BOOT_LOADER_CONF_BLEN*/							/* ���������ִ�С��ע�⣺��������ǰ�����ʵ����Ҫ������EEPROM�־û����� */

EXT BOOL CheckExeFlash_0Low_1High(void);
EXT BOOL CheckDebugVersion(void);											/* �жϵ�ǰ���а汾 */
EXT void OnPostLiveUpdate(void);
EXT void SoftRollBack(uint32 u32RunVersion);

EXT uint32 GetVerVar(uint32 u32VersionVar);
EXT uint32 GetVolatileFromU32Addr(const uint32 *pU32Val);					/* ��ȡָ���洢����ַ�е�ǰʵ�ʵ����� */
EXT BOOL CheckSoftVer(void);												/* �������汾�������Ƿ���ȫ��flash������ */
EXT void GetSoftVersion(BOOT_SOFT_VERSION* pBootSoftVer);					/* ��ȡ����汾�� */
EXT void InvalidateAnotherSoft(void);										/* �ƻ���һ���汾����������Ա�־ */
EXT BOOL FlashUpgradeFrag(BOOL bWhichHalf_0Fst_1Sec, uint32 u32FlashAddr, uint32* pU32FlashData, uint32 u32FlashFragBLen);		/* ���´���Ƭ */
EXT void CalcUid(uint32 * pU32Uid);
EXT uint32 CalcUid16B(void);
#define UID_U32LEN	4
EXT uint32 GetChipIDHash(void);
/***************************************************************************
		Ƭ�ϴ洢
***************************************************************************/
#define DATA_ACCESS_TASK_STACK_LEN	3072
typedef struct {
	osSemaphoreId 					Sem_Op;
	osStaticSemaphoreDef_t 			OpSemCtlBlk;		/* ���ھ�̬���� */
	osThreadId 						DataAcsTSK;
	uint8 							Stack[DATA_ACCESS_TASK_STACK_LEN];
	osStaticThreadDef_t 			TaskCtlBlk;
}DATA_ACCESS_BLOCK;
EXT DATA_ACCESS_BLOCK g_DataAccessBlock;

EXT void EEPromErase(uint32 ui32Address);
EXT void EEPROMRead(uint32 *pui32Data, uint32 ui32Address, uint32 ui32Count);
EXT uint32 EEPROMProgram(uint32 *pui32Data, uint32 ui32Address, uint32 ui32Count);

EXT void PersistSaveGrp(void);		/* �־û�SAVE_GRP */

EXT void FlashErase2(uint32 u32StartAddr, uint32 u32BLen);					/* ֮������2�Ǳ����TI���� */
EXT void FlashRead2(uint32 ReadAddr, uint32 *pBuffer, uint32 u32ByteLen);	/* ֮������2�Ǳ����TI���� */
EXT BOOL FlashProgram2(uint32_t *pSrc, uint32_t u32Dst, uint32_t u32BLen, BOOL bValidate);	/* ֮������2�Ǳ����TI���� */
EXT uint32 FlashReadU32(uint32 faddr);	/* ֮������2�Ǳ����TI���� */

/***************************************************************************
		CPU�¶� & CPU Ref
***************************************************************************/
EXT float32 CalCPUTemperature(uint8 u8DCSigNo);
EXT float32 CalVrefint(uint8 u8DCSigNo);

/***************************************************************************
		RTCģ��
***************************************************************************/
EXT void InitRTC(void);
EXT void GetRealTime(REAL_TIME_VAR* pRealTime);
EXT uint8 GetWeekDay(void);
EXT uint32 GetRTCSeconds(void);
EXT uint16 GetRTCMilliSec(void);
EXT void SetRTCSeconds(uint32 u32Seconds);

/***************************************************************************
		GPIO����
***************************************************************************/
typedef struct {
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
}GPIO_PINCFG;
EXT void GPIO_write(uint8 u8DOutNo, BOOL bHigh1_Low0);
EXT void GPIO_toggle(uint8 u8DOutNo);
EXT BOOL GPIO_read(uint8 u8DInNo);

/***************************************************************************
		Uart����
***************************************************************************/
#define MAX_UART_TASK_STACK_LEN		1024
typedef struct {
	uint8 									Stack[MAX_UART_TASK_STACK_LEN];
	osStaticSemaphoreDef_t 					TxSemCtlBlk;		/* ���ھ�̬���� */
	osStaticSemaphoreDef_t 					RxSemCtlBlk;		/* ���ھ�̬���� */
	osStaticThreadDef_t 					TaskCtlBlk;			/* ���ھ�̬����ctr */
}UART_BLOCK;
EXT UART_BLOCK g_UartBlock[MAX_UART_NUM];

#define UART_Handle							UART_HandleTypeDef*

EXT void OpenUartComm(uint8 u8Port, uint32 u32Baud, uint8 u8Parity_0Nul_1Odd_2Even, uint16 uReadTickOut_0Default);
EXT void WriteToUart(uint8 u8Port, uint16 uNeedTxByte);
EXT uint16 ReadFromUart(uint8 u8Port, uint16 uIniWaitTime_ms);

/***************************************************************************
        I2C����
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
		SPI����
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
        can����
***************************************************************************/
EXT void InitCanFilter(void* hCan);
EXT void SetCanMsgFilter(void* hCan, uint32 u32MsgID, uint32 u32Mask, uint8 u8RxInt);
EXT void WriteDataToCan(void* hCan, uint8 u8DataGroup, uint8 u8DevAdd, uint16 uDataPt, uint8* pU8Dat, uint8 u8ByteNum, BOOL bTxIntEnable);
EXT void ChkCanBusOff(void);

/***************************************************************************
		����
***************************************************************************/

#define NET_TASK_STACK_BLEN			2048
typedef struct {
	osSemaphoreId 					Sem_DnsReq;
	osStaticSemaphoreDef_t 			DnsSemCtlBlk;						/* ���ھ�̬���� */
	osStaticThreadDef_t 			ModbusTCPServerTaskCtlBlk;			/* ���ھ�̬���� */
}NET_BLOCK;
EXT NET_BLOCK g_NetBlock;

EXT osStaticSemaphoreDef_t MqttPubReqSemCtlBlk;							/* ���ھ�̬���� */

#define	SOCKET						int32_t          					/* OS Socket Type */
#define INVALID_SOCKET 				(-1)   								/* Used by socket() and accept() */
EXT int GetDnsAddrFromDhcpRecord(uint8 u8ServerIdx, uint8 *pIPServer);
EXT void NetworkLoop(void);

typedef struct {
	osStaticThreadDef_t 			StackCtlBlk;						/* ���ھ�̬���� */
}MQTT_BLOCK;
EXT MQTT_BLOCK g_MqttBlock[2];	//MQTT_TASK_NUM

/***************************************************************************
		TF���洢
***************************************************************************/
EXT void InitSDSPI(void);

/***************************************************************************
		��ʱ��
***************************************************************************/
EXT float32 CalCPUTemperature(uint8 u8DCSigNo);

/************   exclude redefinition    ****************/
#undef EXT				/* release EXT */
#endif /* _DRV_ST_H_ */
