/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved
 *
 * File			: DrvTI.c
 * Author		: Wang Renfei
 * Description	: ��֧�����
 
 *				: 
 * Date			: 2022-7-4
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		| 2022-07-04  |	W.R.F| ����
 **************************************************************************/

/***************************************************************************
 						macro definition
***************************************************************************/
#define _DRVST_C_		/* exclude redefinition */
/***************************************************************************
 						include files
***************************************************************************/
//#include "lwip.h"
#include "GlobalVar.h"
#include "MdlDataAccess.h"
#include "MdlSys.h"

extern void Error_Handler(void);

/***************************************************************************
 						global variables definition
***************************************************************************/
const SECTION(".runver") uint32 g_u32RunVer = (uint32)SOFTWARE_VER | (((uint32)0-SOFTWARE_VER) << 16);			/* 16λԭ��+16λ���뷽ʽУ��洢 */
const SECTION(".softver") uint32 g_u32SoftVer = (uint32)SOFTWARE_VER | (((uint32)0-SOFTWARE_VER) << 16);		/* 16λԭ��+16λ���뷽ʽУ��洢 */

/***************************************************************************
						internal functions declaration
***************************************************************************/
uint32* GetCounterpartAddr(uint32 *pVal);
SECTION(".BootSection") void Reset_Handler(void)
{
	__disable_irq();

#if DEVICE_TYPE == V5_YYT4
	/* ����˫bankģʽ����flash��ַ���������㣬��ֹ�Ѿ������˴�ѡ��İ��������쳣 */
	if((FLASH->OPTCR & FLASH_OPTCR_DB1M) == 0) {
		for(;;) {
			if((FLASH_WaitForLastOperation((uint32_t)50) == HAL_OK)
				&& (HAL_FLASH_OB_Unlock() == HAL_OK)
				&& (FLASH->OPTCR |= FLASH_OPTCR_DB1M)		/* ����˫bankģʽ��DB1Mλ��cubeIDE�￴��������st�ĵ����У�ʵ��Ҳ�У�1Mflash��Ĭ����0��2M��Ĭ����1�� */
				&& (HAL_FLASH_OB_Launch() == HAL_OK)
				&& (HAL_FLASH_OB_Lock() == HAL_OK))
			{
				break;
			}
		}
	}

	/* �ر�˫bank��������ֹ�Ѿ������˴�ѡ��İ��������쳣 */
	if((FLASH->OPTCR & FLASH_OPTCR_BFB2) != 0) {
		HAL_FLASH_Unlock();
		HAL_FLASH_OB_Unlock();
		FLASH_AdvOBProgramInitTypeDef AdvOBInit;
		AdvOBInit.OptionType = OPTIONBYTE_BOOTCONFIG;
		HAL_FLASHEx_AdvOBGetConfig(&AdvOBInit);
		AdvOBInit.BootConfig = OB_DUAL_BOOT_DISABLE;
		HAL_FLASHEx_AdvOBProgram (&AdvOBInit);
		if (HAL_FLASH_OB_Launch() != HAL_OK) {
			while (1) {}
		}
		HAL_FLASH_OB_Lock();
		HAL_FLASH_Lock();
	}
#endif

	/* �����������������*/
	BOOL bRunInHighFlash = CheckExeFlash_0Low_1High();
	BOOL bNeedUpdateRunVer = FALSE;
	BOOT_SOFT_VERSION BootSoftVer;
	GetSoftVersion(&BootSoftVer);

	/* �����������ָ���汾�Ų����þ���������ģ�����������Ҳ�����ã��Ǿ�ֻ��ȡ������ʵ�ʰ汾�ŵĸ߰汾�� */
	if((BootSoftVer.u32RunVersion = GetVerVar(GetVolatileFromU32Addr(&g_u32RunVer))) == 0) {
		BootSoftVer.u32RunVersion = GetVerVar(GetVolatileFromU32Addr(GetCounterpartAddr((uint32*)&g_u32RunVer)));
		bNeedUpdateRunVer = TRUE;
	}

	/* ���������������Զ��ܺ�,�����ָ����; ���û��ָ������Ĭ�ϸ߰汾��.
	   ����ѡ��������������õ�; ���������������Զ������⣬��ѡ��͵�ַ�� */
	if(pOldBootLoaderConf->_c_int00_add == 0xFFFFFFFFUL) {		/* �����һ��flash�ǿյģ���û��ѡ */
	} else if(BootSoftVer.u32LowFlashVer && BootSoftVer.u32HighFlashVer) {
		if(BootSoftVer.u32RunVersion) {
			if(BootSoftVer.u32RunVersion == BootSoftVer.u32LowFlashVer) {
			} else if(BootSoftVer.u32RunVersion == BootSoftVer.u32HighFlashVer) {
				bRunInHighFlash = TRUE;
			} else if(BootSoftVer.u32HighFlashVer > BootSoftVer.u32LowFlashVer) {
				bRunInHighFlash = TRUE;
				bNeedUpdateRunVer = TRUE;
			}
		} else if(BootSoftVer.u32HighFlashVer > BootSoftVer.u32LowFlashVer) {
			bRunInHighFlash = TRUE;
			bNeedUpdateRunVer = TRUE;
		}
		/* ������ǹ���������������ʱ����̣���ع�����һ���汾 */
#define RESET_COUNT_TO_ROOLBACK		10			/* times */
#define MINIMAL_TIME_TO_ROOLBACK	(60 * 2)	/* seconds */
		if(RCC->CSR & RCC_CSR_PORRSTF) {	/* ��������������ʼ������������ر��� */
			g_Sys.uRstCount = 0;
			g_Sys.u32RstTimer_s = 0;
		} else if((g_Sys.uRstCount > RESET_COUNT_TO_ROOLBACK) && (g_Sys.u32RstTimer_s < MINIMAL_TIME_TO_ROOLBACK)) {
			bRunInHighFlash = !bRunInHighFlash;
			bNeedUpdateRunVer = TRUE;
			g_Sys.u32RstTimer_s = MINIMAL_TIME_TO_ROOLBACK;
		}
	} else if(BootSoftVer.u32HighFlashVer) {
		bRunInHighFlash = TRUE;
		bNeedUpdateRunVer = TRUE;
	}

	/* ������������汾�ֶΣ���ֹ�����汾������ */
	if(bNeedUpdateRunVer) {
		uint32 u32RunVer;
		/* ����ʵ�ʵİ汾�Ÿ���Ҫ���еİ汾�� */
		if(CheckExeFlash_0Low_1High() ^ bRunInHighFlash) {
			u32RunVer = GetVolatileFromU32Addr(GetCounterpartAddr((uint32*)&g_u32SoftVer));
		} else {
			u32RunVer = GetVolatileFromU32Addr(&g_u32SoftVer);
		}
		/* ��д���������а汾�ţ��־û���flash�� */
		FlashErase2((uint32)((uint8*)&g_u32RunVer), sizeof(g_u32RunVer));
		FlashProgram2(&u32RunVer, (uint32)&g_u32RunVer, 4, TRUE);

		/* ��д����������а汾�ţ��־û���flash�� */
		FlashErase2((uint32)((uint8*)GetCounterpartAddr((uint32*)&g_u32RunVer)), sizeof(g_u32RunVer));
		FlashProgram2(&u32RunVer, (uint32)((uint8*)GetCounterpartAddr((uint32*)&g_u32RunVer)), 4, TRUE);
	}

	/* �Ƿ���Ҫ��ת���߰��� */
	if(CheckExeFlash_0Low_1High() ^ bRunInHighFlash) {					/* ��Ҫ��ת����һ�� */
		SCB->VTOR = FLASH_BASE + bRunInHighFlash*FLASH_HALF_BLEN;		/* ����ȷ���ж��������׵�ַ */
		uint32 * pVtor = (uint32*)(FLASH_BASE + bRunInHighFlash*FLASH_HALF_BLEN);
		__DSB();
	    // ��������ջָ�루MSP��
	    asm volatile ("MSR MSP, %0" :: "r" (*pVtor));
	    pVtor++;
	    // ���ø�λ������ַ��PC��
	    asm volatile ("BX %0" :: "r" (*pVtor));
	    __DSB();
		for(;;) {}
	}

	(*(void(*)())pBootLoaderConf->_c_int00_add)();	/*	_c_int00(); */
}

/***************************************************************************
		����, �ź���
***************************************************************************/
/*******************************************************************************
 * ��������
 * pName:  ��������
 * func:����������	(void (*os_pthread) (void const *argument))
 * pArgs: ����(TODO:����һ������)
 * prio: ��ʼ���ȼ�
 * u8MaxInstanceNum: ���ʵ������(��ֵΪ0����instances�����ĵ�˵Ӧ���Ǻ���ᴴ������Thread����ʵ��Ū��0Ҳ����)
 * pStackBuf:��ջ�ռ�
 * u32StackLen:��ջ��С���ֽڣ�
 * pCtlBlkContainer: ����֧�־�̬�����ļܹ���˵����Ҫͨ���˲�����ʽָ����������ƿ�ĳ��ؿռ�
 ********************************************************************************/
TSK_Handle TaskCreate(char *pName, os_pthread func, void *pArg, osPriority prio, uint8 u8MaxInstanceNum, uint8 *pStackBuf, uint32 u32StackLen, osStaticThreadDef_t *pCtlBlkContainer)
{
	osThreadDef_t ThreadDef = {pName, func, prio, u8MaxInstanceNum, (uint32)(u32StackLen/4)
#if( configSUPPORT_STATIC_ALLOCATION == 1 )
			,(uint32*)pStackBuf
			,pCtlBlkContainer
#endif
	};
	return osThreadCreate(&ThreadDef, pArg);
}

/*******************************************************************************
 *  �����ź��� ��uCountΪ1ʱ����ʼֵʵ��Ϊ0����ʱû�ҵ��޸ĵĵط��������Ҫ��ʼֵΪ1����Ҫ���ֶ�postһ�Σ�
 * pName:  �ź�������
 * u8Count: �ź�������
 * pCtlBlkContainer:	  �����ƽ̨֧�־�̬�����������ھ�̬���������ݽṹ���أ������֧����ֱ�Ӵ�NULL
 * 				��̬����Ҫ����һ����ַ������osStaticSemaphoreDef_t����
 * 				��̬�������ַΪNULL�����Ӷ��д���
 * ע�����ѹ�ʱ��vSemaphoreCreateBinary()�������ź�����ʼֵ��1��
 * 	   �����½ӿ�xSemaphoreCreateBinary()�������ź�����ʼֵ��0
 *******************************************************************************/
SEM_Handle SemCreate(char* pName, uint8 u8Count, osStaticSemaphoreDef_t* pCtlBlkContainer)
{
	osSemaphoreDef_t semDef = { 0
#if( configSUPPORT_STATIC_ALLOCATION == 1 )
			, pCtlBlkContainer
#endif
	};
	return osSemaphoreCreate(&semDef, u8Count);
}

/***************************************************************************
		��λ����, ���Ź�, ��ʱ��
***************************************************************************/
/* �Ͽ�������(ͨ����OptionByte��OB����RDP��Ϊ1��ʵ��)
 * RDP == 0 ���ޱ���
 * RDP == 1 ��������
 * RDP == 2 ��ש����
 * ��������RPD1��ԭ��RDP0��������stlink-utility�ȹ����޸ģ�RDP2�����޷���ԭ�� */
void DiscJtagAndStartWDT(void)
{
#if 0
	if(SOFT_RELEASE1_DEBUG0 || (g_Sys.SerialNo.u32Dat >= 10000000UL)) {
		FLASH_WaitForLastOperation((uint32_t)50);
		FLASH_OBProgramInitTypeDef OBInit;
		HAL_FLASHEx_OBGetConfig(&OBInit);
		if (OBInit.RDPLevel == OB_RDP_LEVEL_0) {
			HAL_FLASH_Unlock();
			HAL_FLASH_OB_Unlock();
			OBInit.OptionType = OPTIONBYTE_RDP;
			OBInit.RDPLevel = OB_RDP_LEVEL_1;		/* ����RDP Level 1 */
			if (HAL_FLASHEx_OBProgram(&OBInit) == HAL_OK ) {
			}
			HAL_FLASH_OB_Launch();
			HAL_FLASH_Lock();
		}
	}
#endif
}

/* ���������ʽ���а汾��ι����������4��ι��һ�Σ�����MCU����λ�� */
/* ���Ź�һ����ʼ���������������޷��رջ���ͣ��������main.c�ﰴ�迪������ʵҲ����ͨ���ر�LSI RCʱ�ӷ�ʽ��ͣ */
void KickWatchDog(void)
{
	FLASH_WaitForLastOperation((uint32_t)50);
	FLASH_OBProgramInitTypeDef OBInit;
	HAL_FLASHEx_OBGetConfig(&OBInit);
	if (OBInit.RDPLevel != OB_RDP_LEVEL_0) {
		extern IWDG_HandleTypeDef hiwdg;
		HAL_IWDG_Refresh(&hiwdg);
	}
}

/* ��ת��ʱ */
void Boot_SysCtlDelay(uint32 ui32Count)
{
    for( ; ui32Count != 0; ui32Count--) {
        NOP;
    }
}

/***************************************************************************
        ����������������汾
***************************************************************************/
/* �Ӹߵ�16��Ϊ����У��İ汾�ŵ�4�ֽڱ����н����汾�ţ������⵽У��ʧ�ܷ���0�����򷵻ذ汾�ţ���3688 */
uint32 GetVerVar(uint32 u32VersionVar)
{
	if((u32VersionVar&0xFFFF) + (u32VersionVar>>16) == 0x10000UL) {
		return u32VersionVar&0xFFFF;
	} else {
		return 0;
	}
}

/* ��ȡ�˵�ַ���ڵ�ȫ�ֱ�������������ı���ֵ */
uint32 GetCounterpart(uint32 *pVal)
{
	if(CheckExeFlash_0Low_1High()) {
		return *(uint32*)(((uint8*)pVal) - FLASH_HALF_BLEN);			/* ��һ���ַ�����ȫ�ֱ��� */
	} else {
		return *(uint32*)(((uint8*)pVal) + FLASH_HALF_BLEN);			/* ��һ���ַ�����ȫ�ֱ��� */
	}
}

/* ��ȡ�˵�ַ���ڵ�ȫ�ֱ�������������ĵ�ַ */
uint32* GetCounterpartAddr(uint32 *pVal)
{
	if(CheckExeFlash_0Low_1High()) {
		return (uint32*)(((uint8*)pVal) - FLASH_HALF_BLEN);			/* ��һ���ַ�����ȫ�ֱ��� */
	} else {
		return (uint32*)(((uint8*)pVal) + FLASH_HALF_BLEN);			/* ��һ���ַ�����ȫ�ֱ��� */
	}
}

uint32 GetVolatileFromU32Addr(const uint32 *pU32Val)
{
	return (*(volatile uint32*)pU32Val);
}

/* ���flash���������汾 */
void GetSoftVersion(BOOT_SOFT_VERSION* pBootSoftVer)
{
	pBootSoftVer->u32RunVersion = SOFTWARE_VER;		/* ��ǰ���е�ʵ������汾�� */
	pBootSoftVer->u32LowFlashVer = GetVolatileFromU32Addr((uint32*)(((uint8*)&g_u32SoftVer) - ((uint8)CheckExeFlash_0Low_1High() * FLASH_HALF_BLEN)));
	if((pBootSoftVer->u32LowFlashVer&0xFFFF) + (pBootSoftVer->u32LowFlashVer>>16) == 0x10000UL) {
		pBootSoftVer->u32LowFlashVer &= 0xFFFF;
	} else {
		pBootSoftVer->u32LowFlashVer = 0;
	}
	pBootSoftVer->u32HighFlashVer = GetVolatileFromU32Addr((uint32*)(((uint8*)&g_u32SoftVer) + ((uint8)(!CheckExeFlash_0Low_1High()) * FLASH_HALF_BLEN)));
	if((pBootSoftVer->u32HighFlashVer&0xFFFF) + (pBootSoftVer->u32HighFlashVer>>16) == 0x10000UL) {
		pBootSoftVer->u32HighFlashVer &= 0xFFFF;
	} else {
		pBootSoftVer->u32HighFlashVer = 0;
	}
}

/* �ƻ���һ������汾�������Ա�־ */
void InvalidateAnotherSoft(void)
{
	FlashErase2((uint32)GetCounterpartAddr((uint32*)&g_u32RunVer), BOOT_VER_BLEN);			/* �������汾������������ʱ��ʱ�ᱻ�����ݴ���յ���bootloader�����򣬵��������ٻָ��������汾���� */
	g_Sys.uTmr_EraseFlash_DistPwr_ms = 4000;	/* ����flash���̣����ܻ������Դ��ѹ���ȣ�����ɲ����쳣 */
}

#if (CPU_0ST_1TI == 1)
	uint32 u32SoftIntergrity = 0;
	/* �����ǰ���е��Ǹߵ�ַ����д��ľ��ǵ͵�ַ */
	EEPROMProgram((uint32_t*)&u32SoftIntergrity,
					EEADD_BOOT_LOADER_CONF + 4 + 4*(!IsMirrorMode()), sizeof(u32SoftIntergrity));
	g_Sys.uTmr_EraseFlash_DistPwr_ms = 1000;	/* ����flash���̣����ܻ������Դ��ѹ���ȣ�����ɲ����쳣 */
#endif

/* ��������flash bank */
BOOL SwapSoftware(void)
{
#if 0
extern UART_HandleTypeDef huart4;
uint8 printfBuf[80] = {0};
sprintf(printfBuf, "CurrentMirrorModeIs:%d,BankIsAboutToSwap", (uint8)CheckExeFlash_0Low_1High());
HAL_UART_Transmit(&huart4, printfBuf, 40, 1000);
#endif

#if USE_DUAL_BANK_SWAP
	#if 0
		FLASH_WaitForLastOperation((uint32_t)50);
		HAL_FLASH_OB_Unlock();
		FLASH->OPTCR |= FLASH_OPTCR_BFB2;
		HAL_FLASH_OB_Launch();
		HAL_FLASH_OB_Lock();
	#else
		/* ����BFB2 = !UFB */
		HAL_FLASH_Unlock();

		/* Allow Access to option bytes sector */
		HAL_FLASH_OB_Unlock();

		/* Get the Dual boot configuration status */
		FLASH_AdvOBProgramInitTypeDef AdvOBInit;
		AdvOBInit.OptionType = OPTIONBYTE_BOOTCONFIG;
		HAL_FLASHEx_AdvOBGetConfig(&AdvOBInit);

		/* Enable/Disable dual boot feature */
		if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_UFB_MODE)) {
			AdvOBInit.BootConfig = OB_DUAL_BOOT_DISABLE;
			HAL_FLASHEx_AdvOBProgram (&AdvOBInit);
		} else {
			AdvOBInit.BootConfig = OB_DUAL_BOOT_ENABLE;
			g_CodeTest.u32Val[93] = HAL_FLASHEx_AdvOBProgram (&AdvOBInit);
		}

		/* Start the Option Bytes programming process */
		if (HAL_FLASH_OB_Launch() != HAL_OK) {
			/* User can add here some code to deal with this error */
			while (1) {}
		}

		/* Prevent Access to option bytes sector */
		HAL_FLASH_OB_Lock();

		/* Disable the Flash option control register access (recommended to protect
		the option Bytes against possible unwanted operations) */
		HAL_FLASH_Lock();
	#endif
		return TRUE;
#else
		/* ��������������߱�������FLASH_BOOT_VER��g_u32RunVerд����һ���汾�İ汾�ţ��������ɴ���һ���汾���� */
		uint32 u32RunVer = GetVolatileFromU32Addr(GetCounterpartAddr((uint32*)&g_u32SoftVer));
		if(u32RunVer != 0xFFFFFFFF) {			/* ��һ���ַ��һ��4�ֽڲ�����0xFFFFFFFF(�յ�) */
			g_CodeTest.u32Val[12] = GetVolatileFromU32Addr(&g_u32RunVer) & 0xFFFF;
			/* ��д���������а汾�� */
			FlashErase2((uint32)((uint8*)&g_u32RunVer), sizeof(g_u32RunVer));
			FlashProgram2(&u32RunVer, (uint32)&g_u32RunVer, 4, TRUE);

			/* ��д����������а汾�� */
			FlashErase2((uint32)((uint8*)GetCounterpartAddr((uint32*)&g_u32RunVer)), sizeof(g_u32RunVer));
			FlashProgram2(&u32RunVer, (uint32)((uint8*)GetCounterpartAddr((uint32*)&g_u32RunVer)), 4, TRUE);
			g_CodeTest.u32Val[10]++;
			return TRUE;
		} else {
			g_CodeTest.u32Val[11]++;
			return FALSE;
		}
#endif
}

/* ST ver,
 * bWhichHalf_0Fst_1SecҪд���ĸ�����
 * u32FlashAddr���ϰ�������Ե�ַ��������ԣ���Ҫ��bWhichHalf_0Fst_1Sec����
 * pU32FlashData����Ƭ��
 * u32FlashFragBLen����Ƭ���� */
BOOL FlashUpgradeFrag(BOOL bWhichHalf_0Fst_1Sec, uint32 u32FlashAddr, uint32* pU32FlashData, uint32 u32FlashFragBLen)
{
	/* ��ʵд���Flash��ַ������Ҫд�뱸���� */
    uint32 u32SrcStart = u32FlashAddr;
    uint32 u32SrcEnd = u32SrcStart + u32FlashFragBLen;
    uint32 u32DstStart = FLASHADD_UPGRADE_SKIP_START;
    uint32 u32DstEnd = u32DstStart + FLASH_UPGRADE_SKIP_BLEN;

	g_CodeTest.u32Val[50] = u32SrcStart;		//08003DE0
	g_CodeTest.u32Val[51] = u32SrcEnd;			//08004A40
	g_CodeTest.u32Val[52] = u32DstStart;		//08004000
	g_CodeTest.u32Val[53] = u32DstEnd;			//0800C000
	g_CodeTest.u32Val[54] = (uint32)pU32FlashData;		//20012F80
	g_CodeTest.u32Val[56] = u32SrcStart;		//08003DE0
	g_CodeTest.u32Val[57] = u32FlashFragBLen - ((((uint8)(u32SrcEnd > u32DstStart)) * (u32SrcEnd - u32DstStart)) - (((uint8)(u32SrcStart > u32DstStart)) * (u32SrcStart - u32DstStart)));	//00000220
	g_CodeTest.u32Val[58] = (u32SrcStart > u32DstEnd) ? (uint32)pU32FlashData : (uint32)(((uint8*)pU32FlashData) + (u32DstEnd - u32SrcStart));		//2001B1A0
	g_CodeTest.u32Val[59] = (u32SrcStart > u32DstEnd) ? u32SrcStart : u32DstEnd;		//0800C000
	g_CodeTest.u32Val[60] = (u32SrcStart > u32DstEnd) ? u32FlashFragBLen : (u32FlashFragBLen - ((((uint8)(u32SrcStart < u32DstEnd)) * (u32DstEnd - u32SrcStart)) - (((uint8)(u32SrcEnd < u32DstEnd)) * (u32DstEnd - u32SrcEnd))));		//00000000
	g_CodeTest.u32Val[70] = (((uint8)(u32SrcStart < u32DstEnd)) * (u32DstEnd - u32SrcStart)) - (((uint8)(u32SrcEnd < u32DstEnd)) * (u32DstEnd - u32SrcEnd));		//00000C60

	if(u32SrcStart >= (0x08084000+(bWhichHalf_0Fst_1Sec*FLASH_HALF_BLEN)) && u32SrcStart < (0x08088000+(bWhichHalf_0Fst_1Sec*FLASH_HALF_BLEN))) {
		g_CodeTest.u32Val[63]++;		//00000000
	} else if(u32SrcStart >= (0x08088000+(bWhichHalf_0Fst_1Sec*FLASH_HALF_BLEN)) && u32SrcStart < (0x0808C000+(bWhichHalf_0Fst_1Sec*FLASH_HALF_BLEN))) {
		g_CodeTest.u32Val[64]++;		//00000000
	}

	int8 i8TryCnt;		/* ������Դ��� */
	/* ����bootloader���������ַ����bootloader�����򿽵�FLASH_BOOT_VER���ݴ棬��������Ƭȫ����������, ������һ�𿽱���bootloader������FLASH_BOOT_SEG��, ע�⣬��ǰ�㷨ֻ��֤��ʼ��ַ >= ������ַ */
	if((u32FlashAddr >= FLASHADD_IVT_START) && ((u32FlashAddr-FLASHADD_IVT_START) < FLASH_IVT_BLEN)) {
		g_CodeTest.u32Val[80]++;		//00000006
		/* ����д�����Σ�Ϊ�ݴ� */
		for(i8TryCnt = 2; i8TryCnt > 0; i8TryCnt--) {
			g_CodeTest.u32Val[87] = (uint32)pU32FlashData;	//20012F80
			g_CodeTest.u32Val[88] = BOOT_VER_START + (u32FlashAddr - FLASHADD_IVT_START) + bWhichHalf_0Fst_1Sec*FLASH_HALF_BLEN;		//0808BDE0
			g_CodeTest.u32Val[89] = u32FlashFragBLen - ((((uint8)(u32SrcEnd > u32DstStart)) * (u32SrcEnd - u32DstStart)) - (((uint8)(u32SrcStart > u32DstStart)) * (u32SrcStart - u32DstStart)));		//00000220
			if(FlashProgram2(pU32FlashData
					, BOOT_VER_START + (u32FlashAddr - FLASHADD_IVT_START) + bWhichHalf_0Fst_1Sec*FLASH_HALF_BLEN
					, u32FlashFragBLen - ((((uint8)(u32SrcEnd > u32DstStart)) * (u32SrcEnd - u32DstStart)) - (((uint8)(u32SrcStart > u32DstStart)) * (u32SrcStart - u32DstStart)))
					, TRUE))
			{
				/* �����������ȫ��bootloader�ڣ���ֻ����ɹ� */
				if((u32SrcEnd - FLASHADD_IVT_START) <= FLASH_IVT_BLEN) {
					g_CodeTest.u32Val[81]++;		//00000005
					return TRUE;
				} else {
					g_CodeTest.u32Val[82]++;		//00000001
					/* ��Ƭ���е����ݻ���bootloader֮��Ĳ��֣���������ַ���������FlashProgram2ͳһ���������� */
					u32SrcStart = FLASHADD_IVT_START + FLASH_IVT_BLEN;
					u32FlashFragBLen = u32SrcEnd - (FLASHADD_IVT_START + FLASH_IVT_BLEN);
					break;
				}
			}
		}
		if(i8TryCnt == 0) {
			g_CodeTest.u32Val[83]++;		//00000000
			return FALSE;		/* ���bootloader�����ݴ棬��ֱ��������ʧ�� */
		}
	}

	/* �����bootloader����Ϊ֧������ĳ������Ŀ����������п��������������ȿ���������֮ǰ���ٿ�֮��� */
	for(i8TryCnt = 2; i8TryCnt > 0; i8TryCnt--) {
#if 0
		g_CodeTest.u32Val[84]++;	//0000003C
		g_CodeTest.u32Val[90] = pU32FlashData;	//20012F80
		g_CodeTest.u32Val[91] = u32SrcStart + bWhichHalf_0Fst_1Sec*FLASH_HALF_BLEN;		//08084000
		g_CodeTest.u32Val[92] = u32FlashFragBLen - ((((uint8)(u32SrcEnd > u32DstStart)) * (u32SrcEnd - u32DstStart)) - (((uint8)(u32SrcStart > u32DstStart)) * (u32SrcStart - u32DstStart)));	//00000C60
#endif
		if(FlashProgram2(pU32FlashData
				, u32SrcStart + bWhichHalf_0Fst_1Sec*FLASH_HALF_BLEN
				, u32FlashFragBLen - ((((uint8)(u32SrcEnd > u32DstStart)) * (u32SrcEnd - u32DstStart)) - (((uint8)(u32SrcStart > u32DstStart)) * (u32SrcStart - u32DstStart)))
				, TRUE)
			&& FlashProgram2(((u32SrcStart > u32DstEnd) ? pU32FlashData : (uint32*)(((uint8*)pU32FlashData) + (u32DstEnd - u32SrcStart)))	//2001EF80
					, ((u32SrcStart > u32DstEnd) ? (u32SrcStart + bWhichHalf_0Fst_1Sec*FLASH_HALF_BLEN) : (u32DstEnd + bWhichHalf_0Fst_1Sec*FLASH_HALF_BLEN))
					, (u32SrcStart > u32DstEnd) ? u32FlashFragBLen : (u32FlashFragBLen - ((((uint8)(u32SrcStart < u32DstEnd)) * (u32DstEnd - u32SrcStart)) - (((uint8)(u32SrcEnd < u32DstEnd)) * (u32DstEnd - u32SrcEnd))))
					, TRUE))
		{
			g_CodeTest.u32Val[85]++;		//0000003A
			return TRUE;
		}
	}
	g_CodeTest.u32Val[86]++;	//00000001
	return FALSE;
}

#if 0
/* TI ver */
BOOL FlashUpgradeFrag(u32FlashAddr, pU32FlashData, u32FlashFragBLen)
{
	return (FlashProgram2((uint32_t*)pU32FlashData, u32SrcStart, u32FlashFragBLen, TRUE));	/* д��flash����ʱԼ58ms������Ӱ���������, Ϊ����STM32��ԭ��u32RealAddr�����ĳɺ�SRC_START */
}
#endif

/* ST ver */
void OnPostLiveUpdate(void)
{
//	extern uint32 cnst_BootLoaderResetVectors[];
	/* ����bootloader�����������������������bootloader�Ƚϣ�����в�ͬ�ٸ���bootloader����ǰ������bootloader��������Ϊbootloader������������汾�ߣ� */
	uint32 u32SrcStart = (uint32)GetCounterpartAddr((uint32*)&g_u32RunVer);		/* ���������н�����g_u32RunVer����ͷ��section*/
	extern uint32 cnst_BootLoaderResetVectors[];
	uint32 u32DstStart = (uint32)GetCounterpartAddr((uint32*)cnst_BootLoaderResetVectors);
//	g_CodeTest.u32Val[30] = u32SrcStart;
//	g_CodeTest.u32Val[31] = u32DstStart;
	uint32 i = 0;
	for(; i < FLASH_IVT_BLEN; i += 4) {
		if((*((uint32*)(u32SrcStart + i))) != (*((uint32*)(u32DstStart + i)))) {		/* ������ֲ�ͬ�������bootloader */
			g_CodeTest.u32Val[32]++;
			FlashErase2(u32DstStart, FLASH_IVT_BLEN);			/* ����ж�������sector */
			/* bootloader����ʧ�ܣ���Ҫ�������ԣ���Ϊ����������ǵͰ汾����޷������� �ȳ��Ը����������صģ�ʧ�ܾͳ�������һ�������Ļָ� */
			if(FlashProgram2((uint32*)u32SrcStart, u32DstStart, FLASH_IVT_BLEN, TRUE)
					|| FlashProgram2((uint32*)u32SrcStart, u32DstStart, FLASH_IVT_BLEN, TRUE)
					|| FlashProgram2((uint32*)u32SrcStart, u32DstStart, FLASH_IVT_BLEN, TRUE))
			{
				/* ������θ���bootloader��ʧ��ֻ�ܷ�������ֹflashд�� */
//				g_CodeTest.u32Val[33]++;
			}
			break;
		}
	}
	g_Sys.uRstCount = 0;
	g_Sys.u32RstTimer_s = 0;
	SwapSoftware();
}

#if 0
/* TI ver */
void OnPostLiveUpdate(void)
	if(!IsMirrorMode()) {
		/* ����һ��flash����һ��flash��bootloader������У��Ƿ���� */
		uint32* pU32Src = (uint32*)(FLASHADD_BOOTLOADER);
		uint32* pU32Dest = (uint32*)(FLASHADD_BOOTLOADER + FLASH_HALF_BLEN);
		for(i = BOOT_LOADER_BLEN/4; (i > 0) && (*pU32Src++ == *pU32Dest++); i--) {
		}
		if(i) {	/* ��һ��flash�͵�һ��flash��bootloader��ͬ����Ҫ���� */
			FlashErase2(FLASHADD_BOOTLOADER, BOOT_LOADER_BLEN);		/* ����BOOT_LOADER���� */

			/* �ѵ�һ��flash�� bootloader���ڵ�flash��Ԫ ������ ��һ��flash�� ������һ����С������Ԫ */
			FlashProgram2((uint32_t*)(FLASHADD_BOOTLOADER), (FLASHADD_BOOTLOADER + FLASH_HALF_BLEN), BOOT_LOADER_BLEN, TRUE);
		}
	}

	/* ����������������������� */
	uint32 u32RebootFlag = REBOOT_BY_SOFT_UPDATE;
	EEPROMProgram((uint32_t*)&u32RebootFlag, EEADD_REBOOT_FLAG, sizeof(u32RebootFlag));

	/* ��־��������� */
	uint32 u32SoftVersion = (((uint32)(0 - g_SoftUpdateCtr.uReqSoftVer))<<16) | g_SoftUpdateCtr.uReqSoftVer;
	/* �����ǰ���е��Ǹߵ�ַ����д��ľ��ǵ͵�ַ */
	EEPROMProgram((uint32_t*)&u32SoftVersion, EEADD_BOOT_LOADER_CONF + 4 + 4*(!IsMirrorMode()), sizeof(u32SoftVersion));	/* д��ͻ�߰汾�� */
	EEPROMProgram((uint32_t*)&u32SoftVersion, EEADD_BOOT_LOADER_CONF, sizeof(u32SoftVersion));		/* д�����а汾�� */
}
#endif

/* �ع�֮�� */
void SoftRollBack(uint32 u32RunVersion)
{
	/* �жϰ汾, ����汾����һ��bank�İ汾�������ع� */
	volatile uint32 u32SoftVerDat = GetVerVar(GetVolatileFromU32Addr(GetCounterpartAddr((uint32*)&g_u32SoftVer)));
	if(u32SoftVerDat && (u32RunVersion == u32SoftVerDat)) {		/* У����, �汾�ź˶� */
		SwapSoftware();
		g_Sys.tReboot_0Null_nUrgent_pWaitIdle = 1;
	}
}
#if (CPU_0ST_1TI == 1)
void SoftRollBack(uint32 u32RunVersion)
{
	/* ����������������������� */
	SoftRollBack(u32RunVersion);
	uint32 u32RebootFlag = REBOOT_BY_SOFT_UPDATE;
	EEPROMProgram((uint32_t*)&u32RebootFlag, EEADD_REBOOT_FLAG, sizeof(u32RebootFlag));
	/* ����Ŀ������汾�� */
	u32RunVersion = ((0-u32RunVersion)<<16) | u32RunVersion;
	EEPROMProgram((uint32_t*)&u32RunVersion, EEADD_BOOT_LOADER_CONF, sizeof(u32RunVersion));
	g_Sys.tReboot_0Null_nUrgent_pWaitIdle = 1;
}
#endif

#if CPU_0ST_1TI == 0
/* ���ڼ���Ĭ��sn */
uint32 CalcUid16B(void)
{
    uint32 u32ChipID[UID_U32LEN];
    CalcUid(u32ChipID);
	return (uint32)CalCRC16ForModbus((uint8*)u32ChipID, UID_U32LEN * 4);
}
#else
/* ���ڼ���Ĭ��sn */
uint32 CalcUid16B(void)
{
	return (((uint32)rand()<<16) + rand())%1000000UL;
}
#endif

/* ��оƬSystem Control Register �� UniqueID��ַ������ֵ���� 0x400FEF20������Ӧ��ֱ���ø���������ַ���������ױ����� */
uint32 GetChipIDHash(void)
{
	uint32 u32ChipID[UID_U32LEN];
	CalcUid(u32ChipID);
    /* ��оƬID�ȼ��ܣ��ټ���hashֵ */
    EncryptWithAES(AUTH_AES_PACK_SERIAL, u32ChipID, 16, 0);
    return CalCRC32ForFileByIni((uint8*)u32ChipID, 16, 0);
}

/***************************************************************************
        Ƭ�ϴ洢: ��flashģ��eeprom
        Ŀǰ��֧���������ݣ���֧��Msg,Acq��flash��ҳ��
***************************************************************************/
void EEPROMRead(uint32 *pui32Data, uint32 u32Address, uint32 u32ByteLen)
{
	FlashRead2(u32Address, pui32Data, u32ByteLen);
}

void EEPromErase(uint32 ui32Address)
{
	FlashErase2(ui32Address, EEPROM_BLEN);		/* stm32��ʱ������sector��flashģ��eeprom��������� */
}

uint32 EEPROMProgram(uint32 *pui32Data, uint32 ui32Address, uint32 ui32Count)
{
	FlashProgram2(pui32Data, ui32Address, ui32Count, FALSE);
	return 0;		/* Ϊ����TI�ĺ���ǩ�� */
}

/* ��ȡָ����ַ�İ���(16λ����)
* faddr:����ַ
* ����ֵ:��Ӧ����.*/
uint32_t FlashReadU32(uint32_t u32FlashAddr) {
	return *(volatile unsigned long*) u32FlashAddr;
}

/* ��ָ����ַ��ʼ����ָ�����ȵ�����, ֮�������ּӸ�2������Ϊ�����TI������
* ReadAddr:��ʼ��ַ
* pBuffer:����ָ��
* u32ByteLen:�ֽ�����������4������ */
void FlashRead2(uint32_t u32ReadAddr, uint32_t *pBuffer, uint32_t u32ByteLen) {
	uint32_t i = 0;
	for(i = 0; i < (u32ByteLen/4); i++) {
		pBuffer[i] = FlashReadU32(u32ReadAddr);	/* ��ȡ4���ֽ� */
		u32ReadAddr += 4;
	}
}


/* ��ָ����ַ��ʼд��ָ�����ȵ�����, ֮�������ּӸ�2������Ϊ�����TI������
* u32Dst��flash��ʼ��ַ, ��Ҫ4����������ַ
* pSrc:����ָ��
* u32BLen:�ֽ�����������4����������
* bValidate �Ƿ���֤
* �����Ƿ�д������ɹ�����������ɹ��ҿ�����֤���򷵻���֤��� */
BOOL FlashProgram2(uint32_t *pSrc, uint32_t u32Dst, uint32_t u32BLen, BOOL bValidate) {
	BOOL bSuccess = FALSE;
	if(u32BLen > 0) {
		const uint32* pOriginSrc = pSrc;
		const uint32* pEndSrc = pSrc + (u32BLen / 4);
		__IO uint32_t* pDst = ((__IO uint32*)u32Dst);

		/* ˳��ָ������д������ */
		HAL_FLASH_Unlock();		/* Unlock the Flash to enable the flash control register access */
		for(; pSrc < pEndSrc; pSrc += 1) {
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32)pDst, *pSrc) == HAL_OK) {
				pDst++;
			} else {
				break;
			}
		}
		HAL_FLASH_Lock();		/* Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) */

		if(pSrc == pEndSrc) {
			if(bValidate) {
				/* ��ȡ��֤�����Ƿ�һģһ�� */
				pSrc = (uint32*)pOriginSrc;
				pDst =  ((__IO uint32*)u32Dst);
				for(; pSrc < pEndSrc; pSrc += 1) {
					if (*pSrc != *pDst) {
						break;
					} else {
						pDst++;
					}
				}
				/* �����ȫһ��������TRUE */
				if (pSrc == pEndSrc) {
					bSuccess = TRUE;
				}
			} else {
				bSuccess = TRUE;
			}
		}
	} else if(u32BLen == 0) {		/* Ϊ�����ⲿͳһ�㷨������0��Ҳ�ж��ɹ� */
		bSuccess = TRUE;
	}
	return bSuccess;
}

BOOL IsRebootBySoftUpdate(void)
{
    return TRUE;		/* ��EEPROM����ȡ */
}

/* ����Ƿ�Ͽ��˵�������ͨ�����RDP�Ƿ�Ϊ0ʵ�֣� */
BOOL CheckDebugVersion(void)
{
	FLASH_WaitForLastOperation((uint32_t)50);
	FLASH_OBProgramInitTypeDef OBInit;
	HAL_FLASHEx_OBGetConfig(&OBInit);
	return OBInit.RDPLevel == OB_RDP_LEVEL_0;
}

BOOL CheckExeFlash_0Low_1High(void)
{
#if TI_DUAL_BANK
    return (HWREG(FLASH_CONF) & FLASH_CONF_FMME) != 0;
#else
    extern int main(void);
    return ((((uint32)main) - FLASH_BASE) / FLASH_HALF_BLEN) == 1;
#endif
}


#if CPU_0ST_1TI == 0
/* �������汾�������Ƿ���ȫ��flash������ */
BOOL CheckSoftVer(void)
{
#if 0
	/* �жϽ�����оƬ����������������Ϳ��Գ�Ϊ���������ĸ�� */
	BOOT_SOFT_VERSION BootSoftVer;
	EEPROMRead((uint32_t*)&BootSoftVer, EEADD_BOOT_LOADER_CONF, sizeof(BOOT_SOFT_VERSION));

	/* flash��һ���Ǳ�������, ��ȫ�����е� */
	return (SOFT_RUN1_TEST0
		&& (pOldBootLoaderConf->_c_int00_add == 0xFFFFFFFFUL)
		&& ((uint32)(pOldBootLoaderConf->pConfProp) == 0xFFFFFFFFUL)
		&& (pOldBootLoaderConf->u32ItemNum == 0xFFFFFFFFUL)
		&& (pOldBootLoaderConf->u32ConfSaveGroupNum == 0xFFFFFFFFUL)
		&& (GetVerVar(GetVolatileFromU32Addr(GetCounterpartAddr((uint32*)&g_u32RunVer))) == 0xFFFFFFFFUL));
#else
	return TRUE;
#endif
}
#else
/* �������汾�������Ƿ���ȫ��flash������ */
BOOL CheckSoftVer(void)
{
	/* flash��һ���Ǳ�������, ��ȫ�����е� */
	if((!CheckExeFlash_0Low_1High())
		&& (pOldBootLoaderConf->_c_int00_add == 0xFFFFFFFFUL)
		&& ((uint32)(pOldBootLoaderConf->pConfProp) == 0xFFFFFFFFUL)
		&& (pOldBootLoaderConf->u32ItemNum == 0xFFFFFFFFUL)
		&& (pOldBootLoaderConf->u32ConfSaveGroupNum == 0xFFFFFFFFUL))
	{
		/* �жϽ�����оƬ����������������Ϳ��Գ�Ϊ���������ĸ�� */
		BOOT_SOFT_VERSION BootSoftVer;
		EEPROMRead((uint32_t*)&BootSoftVer, EEADD_BOOT_LOADER_CONF, sizeof(BOOT_SOFT_VERSION));
		if(SOFT_RUN1_TEST0
			&& (BootSoftVer.u32RunVersion == 0xFFFFFFFFUL)		/* eeprom������--��Ҫ��ͨ������-jtag���ع��� */
			&& (BootSoftVer.u32LowFlashVer == 0xFFFFFFFFUL)
			&& (BootSoftVer.u32HighFlashVer == 0xFFFFFFFFUL))
		{
			g_PubSoftAuthCtr.uTmr_IniPubCntNoPwd_ms = 60000;
		}

		/* �������а汾 */
		BootSoftVer.u32LowFlashVer = ((((uint32)(0-SOFTWARE_VER))<<16) | SOFTWARE_VER);
		BootSoftVer.u32RunVersion = BootSoftVer.u32LowFlashVer;
		BootSoftVer.u32HighFlashVer = 0;
		EEPROMProgram((uint32_t*)&BootSoftVer, EEADD_BOOT_LOADER_CONF, sizeof(BOOT_SOFT_VERSION));
	}
}
#endif

/* �־û�SAVE_GRP */
void PersistSaveGrp(void)
{
	/* ȷ���Ƿ�����Ҫ�����SAVE_GRP */
	CONF_SAVE_GROUP ConfAcsGrp;
	for(ConfAcsGrp = SAVE_GRP_BRD; ConfAcsGrp < MAX_SAVE_GRP_NUM; ConfAcsGrp++) {
		if(g_DataAcsIntf.bConfHaveUnsavedChange[ConfAcsGrp] || (g_DataAcsIntf.bConfHaveUnsavedChange[SAVE_GRP_NUL])) {		/* ����stm32Ŀǰ�洢������������д������ֻҪ��һ��SAVE_GRP��Ҫ�洢��ȫ��������Ȼ�����¸��´洢����SAVE_GRP */
			EEPromErase(EEADD_START);						/* ����һ��BANK��EEPROMģ������sector��,TODO:����Ϊ�ֱ����,������;�ж϶����ݸ��� */
			EEPromErase(EEADD_START + FLASH_HALF_BLEN);		/* ������һ��BANK��EEPROMģ������sector��,TODO:����Ϊ�ֱ����,������;�ж϶����ݸ��� */
			/* ����ȫ��SAVE_GRP */
			for(ConfAcsGrp = SAVE_GRP_BRD; ConfAcsGrp < MAX_SAVE_GRP_NUM; ConfAcsGrp++) {
				g_DataAcsIntf.bConfHaveUnsavedChange[ConfAcsGrp] = FALSE;
				g_DataAcsIntf.bConfFree[ConfAcsGrp] = FALSE;
				/* ��EEPROMģ�������� */
				if(pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr != 0) {
					SaveConfToEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr);
					if(!CheckConfFromEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr)) {
						SaveConfToEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr);
					}
				}
				/* ��EEPROMģ�������� */
				if(pBootLoaderConf->AcsMedia[ConfAcsGrp].u32BackEEAddr != 0) {
					SaveConfToEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32BackEEAddr);
					if(!CheckConfFromEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32BackEEAddr)) {
						SaveConfToEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32BackEEAddr);
					}
				}
#if SUPPORT_FAT_FS
				/* ��SD�� */
				SaveConfToFile(ConfAcsGrp) ;
				if(ReadOrCheckConfFromFile(ConfAcsGrp, -2) != DATA_ACS_RES_SUC) {
					SaveConfToFile(ConfAcsGrp);
				}
#endif
			}
			g_DataAcsIntf.bConfFree[ConfAcsGrp] = TRUE;
			g_DataAcsIntf.bConfHaveUnsavedChange[SAVE_GRP_NUL] = FALSE;	/* ����ccs����״̬��ʹ�� */
			break;
		}
	}
}

#if CPU_0ST_1TI == 1
/* �־û�SAVE_GRP */
void PersistSaveGrp(void)
{
	CONF_SAVE_GROUP ConfAcsGrp;
	triST tSaveUrgent_0Idle_pReq_nCmplt = g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt;
	for(ConfAcsGrp = SAVE_GRP_BRD; ConfAcsGrp < MAX_SAVE_GRP_NUM; ConfAcsGrp++) {
		Swi_disable();      /* �����жϺ��������л� */
		if((!g_DataAcsIntf.bConfHaveUnsavedChange[ConfAcsGrp]) && (!g_DataAcsIntf.bConfHaveUnsavedChange[SAVE_GRP_NUL])) {
			Swi_enable();
		} else {
			g_DataAcsIntf.bConfHaveUnsavedChange[ConfAcsGrp] = FALSE;
			g_DataAcsIntf.bConfFree[ConfAcsGrp] = FALSE;
			Swi_enable();

			if(pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr != 0) {
				SaveConfToEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr);
				if(!CheckConfFromEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr)) {
					SaveConfToEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr);
				}
			}
			if(pBootLoaderConf->AcsMedia[ConfAcsGrp].u32BackEEAddr != 0) {
				SaveConfToEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32BackEEAddr);
				if(!CheckConfFromEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32BackEEAddr)) {
					SaveConfToEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32BackEEAddr);
				}
			}

		#if SUPPORT_FAT_FS
			SaveConfToFile(ConfAcsGrp) ;
			if(ReadOrCheckConfFromFile(ConfAcsGrp, -2) != DATA_ACS_RES_SUC) {
				SaveConfToFile(ConfAcsGrp);
			}
		#endif
		}
		g_DataAcsIntf.bConfFree[ConfAcsGrp] = TRUE;
	}
	/* ��λ�洢 */
	g_DataAcsIntf.bConfHaveUnsavedChange[SAVE_GRP_NUL] = FALSE;	/* ����ccs����״̬��ʹ�� */
}
#endif
/***************************************************************************
	CPU�¶� & CPU Ref & ��ʱ
***************************************************************************/
#if DC_TOTAL_CHN
float32 CalCPUTemperature(uint8 u8DCSigNo)
{
    /* Temperature(in ��C) = {(VSENSE �C V25)/Avg_Slope} + 25, ���� V25 = 0.76V, Avg_Slope = 2.5mV/��C     */
	if(ProcUniRecUDatAsWave((uDAT_UNI_REC*)(&g_DCSigADSamp[u8DCSigNo]), ANALOG_ADSAMP_BUF_LEN, ANALOG_ADSAMP_BUF_LEN)) {
		//return (g_DCSigADSamp[u8DCSigNo].fSig_avr*ADC_RANGE/ADC_MAX_RESULT - 0.76f)/0.0025f + 25.0f;
        return (1.43f - g_DCSigADSamp[u8DCSigNo].fSig_avr*ADC_RANGE/ADC_MAX_RESULT)/0.0043f + 25.0f;
	} else {
		return 0;
	}
}
float32 CalVrefint(uint8 u8DCSigNo)
{
	if(ProcUniRecUDatAsWave((uDAT_UNI_REC*)(&g_DCSigADSamp[u8DCSigNo]), ANALOG_ADSAMP_BUF_LEN, ANALOG_ADSAMP_BUF_LEN)) {
        return g_DCSigADSamp[u8DCSigNo].fSig_avr*ADC_RANGE/ADC_MAX_RESULT;
	} else {
		return 0;
	}
}
#endif

/***************************************************************************
    RTCģ�飬��Ҫioc���ó�:
    1. ʱ��ѡ���ⲿ32.768KHz����(LSE)
    2. Asynchronous Predivider value 31��Synchronous Predivider value 1023
***************************************************************************/
//#include "stm32f4xx_hal_rtc.h"
extern RTC_HandleTypeDef hrtc;
void GetRealTime(REAL_TIME_VAR* pRealTime)
{
    RTC_TimeTypeDef RtcTime;
    RTC_DateTypeDef RtcDate;
    /* Ϊ�˱������������ԣ��ڶ�ȡ��λ�Ĵ���(��:��)��Ӳ�����Զ������λ(��:����)--��ȡ��ʱ����Ҫ7��RTC_Clk(32.768KHz) */
    HAL_RTC_GetTime(&hrtc, &RtcTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &RtcDate, RTC_FORMAT_BIN);
    pRealTime->u8Year = RtcDate.Year;
    pRealTime->u8Month = RtcDate.Month;
    pRealTime->u8Day = RtcDate.Date;
    pRealTime->u8Hour = RtcTime.Hours;
    pRealTime->u8Minute = RtcTime.Minutes;
    pRealTime->u8Second = RtcTime.Seconds;
#if (DEVICE_TYPE != YKC)
    pRealTime->uMilliSec = ((RtcTime.SecondFraction - RtcTime.SubSeconds)*1000UL)/(RtcTime.SecondFraction + 1);
#else
    pRealTime->uMilliSec = (uint16)((uint32)RTC->DIVL*1000/32768);
#endif

    /* ��Ȩ��֤ */
    uint32 u32Seconds = CalRTCSecondsByDate(pRealTime->u8Year, pRealTime->u8Month, pRealTime->u8Day, 
                                            pRealTime->u8Hour, pRealTime->u8Minute, pRealTime->u8Second);
    if((u32Seconds < 1800)                                  /* �ϵ���Сʱ��, ���ʱ��û�г�ʼ�����, ��������Ȩ��� */
        || (g_Sys.u32License_Seconds == 4102358400UL))      /* ������Ȩ, 99/12/31     00:00:00 */
    {
        g_Sys.iDaysToLicDeadLine = 0;
    } else if(u32Seconds > g_Sys.u32License_Seconds) {
        g_Sys.iDaysToLicDeadLine = (u32Seconds - g_Sys.u32License_Seconds)/(3600*24);
        g_Sys.iDaysToLicDeadLine = -1 - g_Sys.iDaysToLicDeadLine;
    } else {
        g_Sys.iDaysToLicDeadLine = (g_Sys.u32License_Seconds - u32Seconds)/(3600*24);
    }
}

uint32 GetRTCSeconds(void)
{
    RTC_TimeTypeDef RtcTime;
    RTC_DateTypeDef RtcDate;
    HAL_RTC_GetTime(&hrtc, &RtcTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &RtcDate, RTC_FORMAT_BIN);
    return CalRTCSecondsByDate(RtcDate.Year, RtcDate.Month, RtcDate.Date, RtcTime.Hours, RtcTime.Minutes, RtcTime.Seconds);
}

uint16 GetRTCMilliSec(void)
{
#if (DEVICE_TYPE != YKC)
    RTC_TimeTypeDef RtcTime;
    RTC_DateTypeDef RtcDate;
    HAL_RTC_GetTime(&hrtc, &RtcTime, RTC_FORMAT_BIN);
    /* ��������ʱ������ʵû�ã�����������������´ζ�ȡ��ʱ����Ӧ�Ĵ����ͻᱣ������һ�ζ�SSR(�����ʱ��)ʱ�� */
    HAL_RTC_GetDate(&hrtc, &RtcDate, RTC_FORMAT_BIN);
    return ((RtcTime.SecondFraction - RtcTime.SubSeconds)*1000UL)/(RtcTime.SecondFraction + 1);
#else
    return (uint16)((uint32)RTC->DIVL*1000/32768);
#endif
}

void SetRTCSeconds(uint32 u32Seconds)
{
    REAL_TIME_VAR RealTime;
    CalDateByRTCSeconds(u32Seconds, &RealTime);

    /* HAL_RTC_SetTime() HAL_RTC_SetDate()ִ�к󣬶������SSR(�����ʱ��);
    ����δ���ִ��ʱ��Զ����1�룬��˲��������������Ե����⣬������λ��������ݴ��� */
    RTC_TimeTypeDef RtcTime;
    RtcTime.Hours = RealTime.u8Hour;
    RtcTime.Minutes = RealTime.u8Minute;
    RtcTime.Seconds = RealTime.u8Second;
    HAL_RTC_SetTime(&hrtc, &RtcTime, RTC_FORMAT_BIN);
    
    RTC_DateTypeDef RtcDate;
    RtcDate.Year = RealTime.u8Year;
    RtcDate.Month = RealTime.u8Month;
    RtcDate.Date = RealTime.u8Day;
    HAL_RTC_SetDate(&hrtc, &RtcDate, RTC_FORMAT_BIN);
    g_Sys.u32Seconds_LastSyncRTC = u32Seconds;
}

/***************************************************************************
   GPIO
***************************************************************************/
extern const GPIO_PINCFG cnst_GpioPinCfg[];
void GPIO_write(uint8 u8DOutNo, BOOL bHigh1_Low0)
{
    HAL_GPIO_WritePin(cnst_GpioPinCfg[u8DOutNo].GPIOx, cnst_GpioPinCfg[u8DOutNo].GPIO_Pin, bHigh1_Low0);
}

void GPIO_toggle(uint8 u8DOutNo)
{
    HAL_GPIO_TogglePin(cnst_GpioPinCfg[u8DOutNo].GPIOx, cnst_GpioPinCfg[u8DOutNo].GPIO_Pin);
}

BOOL GPIO_read(uint8 u8DInNo)
{
    return HAL_GPIO_ReadPin(cnst_GpioPinCfg[u8DInNo].GPIOx, cnst_GpioPinCfg[u8DInNo].GPIO_Pin);
}

/***************************************************************************
 							Uartģ��
***************************************************************************/
#include "MdlUARTnModbus.h"
#define MIN_READ_TIMEOUT_ms 2
/* ��RS485ͨѶ�˿ڣ�����ʼ����Ӧ���ڴ���� */
extern UART_HandleTypeDef huart1;

///*---------------GPRS����˫������---------------*/
//#define GPRS_UART_RX_DOUBLE_BUFF_MAX_LEN		128		/* gprs�Ĵ��ڽ���˫������buff��С */
//typedef struct {
//	uint8 aU8RxBufA[GPRS_UART_RX_DOUBLE_BUFF_MAX_LEN];
//	uint8 aU8RxBufB[GPRS_UART_RX_DOUBLE_BUFF_MAX_LEN];
//	uint8 *pU8CurrentBuf;
//	uint8 *pU8NextBuf;
//}UART_DoubleBuff_t;
//UART_DoubleBuff_t g_GPRS_DoubleBuffer = {
//	.pU8CurrentBuf = NULL,
//	.pU8NextBuf = NULL,
//};
///*---------------end---------------*/

/* ��ʼ��˫������ */
void DoubleBuff_Init(UART_DoubleBuff_t *pDoubleBuff)
{
	pDoubleBuff->pU8CurrentBuf = pDoubleBuff->aU8RxBufA;
	pDoubleBuff->pU8NextBuf = pDoubleBuff->aU8RxBufB;
}

//uint8 aU8GPRS_Buff[UART_TR_BUF_BLEN] = {0};
void OpenUartComm(uint8 u8Port, uint32 u32Baud, uint8 u8Parity_0Nul_1Odd_2Even, uint16 uReadTickOut_0Default)
{
    /* ��ʼ���ڴ� */
	g_UartComm[u8Port].uTimer_WatchDog_ms = 0xFFFF;
	g_UartComm[u8Port].uLastBaud_100bps = u32Baud/100;
	g_UartComm[u8Port].uTime_RxMaxIntv_ms = uReadTickOut_0Default / OS_TICK_KHz;

	/* Ӳ����ʼ�� */
	//g_UartComm[u8Port].Handle->Instance->CR1 &= (~USART_CR1_UE);
    g_UartComm[u8Port].Handle->Init.BaudRate = u32Baud;
    g_UartComm[u8Port].Handle->Init.WordLength = UART_WORDLENGTH_8B;
    g_UartComm[u8Port].Handle->Init.StopBits = UART_STOPBITS_1;
    if(u8Parity_0Nul_1Odd_2Even == 0) {
        g_UartComm[u8Port].Handle->Init.Parity = UART_PARITY_NONE;
    } else if(u8Parity_0Nul_1Odd_2Even == 0) {
        g_UartComm[u8Port].Handle->Init.Parity = UART_PARITY_ODD;
    } else if(u8Parity_0Nul_1Odd_2Even == 0) {
        g_UartComm[u8Port].Handle->Init.Parity = UART_PARITY_EVEN;
    }
    g_UartComm[u8Port].Handle->Init.Mode = UART_MODE_TX_RX;
    g_UartComm[u8Port].Handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_UartComm[u8Port].Handle->Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(g_UartComm[u8Port].Handle) != HAL_OK) {
        Error_Handler();
    }

    if(u8Port == GPRS_UART_PORT) {	/* GPRSͨ��ʹ��DMAѭ������ */
    	g_GPRSRingBuffer.uHead = g_GPRSRingBuffer.uRear = (RING_BUFFER_MAX_SIZE - __HAL_DMA_GET_COUNTER(g_UartComm[u8Port].Handle->hdmarx)) % RING_BUFFER_MAX_SIZE;
//    	RingBuffer_Init(&g_GPRSRingBuffer);
//    	HAL_UART_DMAStop(g_UartComm[u8Port].Handle);		/* ֹͣDMA */
    	HAL_UART_Receive_DMA(g_UartComm[u8Port].Handle, g_GPRSRingBuffer.aU8Buffer, RING_BUFFER_MAX_SIZE);	/* ��ʼѭ������ */
    }

	/* ���Sem��ע����ʼ����ʱ��SEM������Ѿ����ˣ���� */
	osSemaphoreWait(g_UartComm[u8Port].Sem_TxCplt, 0);
	osSemaphoreWait(g_UartComm[u8Port].Sem_RxGet, 0);
}

#include "LibMath.h"
#if !CIH_UART_DRV
void WriteToUart(uint8 u8Port, uint16 uNeedTxByte)
{
	HAL_UART_Transmit(g_UartComm[u8Port].Handle, g_UartComm[u8Port].u8TRBuf, uNeedTxByte, 500);
#if DEBUG_GPRS_UART_INFO
	if(u8Port == GPRS_UART_PORT) {
		HAL_UART_Transmit(g_UartComm[0].Handle, g_UartComm[u8Port].u8TRBuf, uNeedTxByte, 500);
	}
#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* �������huart����Ҫת��u8Port */
    uint8 u8Port;
    for(u8Port = 0; (u8Port < UART_COMM_NUM) && (g_UartComm[u8Port].Handle != huart); u8Port++) {
    }
	if(u8Port < UART_COMM_NUM) {
		osSemaphoreRelease(g_UartComm[u8Port].Sem_TxCplt);
		HAL_UART_AbortTransmit(g_UartComm[u8Port].Handle);
	}
}

uint16 ReadFromUart(uint8 u8Port, uint16 uIniWaitTime_ms)
{
	uint16 uRxBufPt = 0;
	uint8 *pU8Buf = g_UartComm[u8Port].u8TRBuf + g_UartComm[u8Port].uRxFrameIndex;
	uint16 uMaxLen = UART_TR_BUF_BLEN - g_UartComm[u8Port].uRxFrameIndex;	/* ���Զ�ȡ������ֽ��� */
	if(u8Port == GPRS_UART_PORT) {	/* ��GPRSͨ��ʱ���õ����λ����� */
		while(uIniWaitTime_ms--) {
			if((uRxBufPt = RingBuffer_Read(&g_GPRSRingBuffer, pU8Buf, uMaxLen))
			&& (uRxBufPt = StrRemoveHashData(pU8Buf, uRxBufPt)) != 0) {		/* ��鲢������IRQI��֡ */
				break;
			} else {
				Task_sleep(1);
			}
		}
	} else {
		HAL_UART_Receive(g_UartComm[u8Port].Handle, pU8Buf, uMaxLen, uIniWaitTime_ms);
	}

#if DEBUG_GPRS_UART_INFO
	if((u8Port == GPRS_UART_PORT) && uRxBufPt) {
		HAL_UART_Transmit(g_UartComm[0].Handle, (uint8 *)"��", 2, 300);
		HAL_UART_Transmit(g_UartComm[0].Handle, g_UartComm[u8Port].u8TRBuf, uRxBufPt + g_UartComm[GPRS_UART_PORT].uRxFrameIndex, 300);
		HAL_UART_Transmit(g_UartComm[0].Handle, (uint8 *)"��", 2, 300);
	}
#endif
	return uRxBufPt;
}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	if((HAL_UART_RXEVENT_IDLE == HAL_UARTEx_GetRxEventType(huart)) || (HAL_UART_RXEVENT_TC == HAL_UARTEx_GetRxEventType(huart))) {		/* ��DMA������ʱ���꣬���ᴥ��IDLE�����������ж��¼���Ҫ���� */
//		if(huart == g_UartComm[GPRS_UART_PORT].Handle) {
//			g_UartComm[GPRS_UART_PORT].uRxBufPt = Size;
//			/* ������DMA��ֹ��ʧ���� */
//			if(HAL_UARTEx_ReceiveToIdle_DMA(huart, g_GPRS_DoubleBuffer.pU8NextBuf, GPRS_UART_RX_DOUBLE_BUFF_MAX_LEN) == HAL_BUSY) {
//				/* DMA����ʧ��, һ�㲻�����, ���ǳ���ż��������GPRSͨ�Ų��ϵ����. ������DMAû�����������ʧ�ܾ�������һ��. */
//				HAL_UARTEx_ReceiveToIdle_DMA(huart, g_GPRS_DoubleBuffer.pU8NextBuf, GPRS_UART_RX_DOUBLE_BUFF_MAX_LEN);
//			}
//			RingBuffer_Write(&g_GPRSRingBuffer, g_GPRS_DoubleBuffer.pU8CurrentBuf, Size);	/* д�뻷�λ����� */
//			/* ����ָ�� */
//			uint8 *pU8Ptr = g_GPRS_DoubleBuffer.pU8CurrentBuf;
//			g_GPRS_DoubleBuffer.pU8CurrentBuf = g_GPRS_DoubleBuffer.pU8NextBuf;
//			g_GPRS_DoubleBuffer.pU8NextBuf = pU8Ptr;
//		} else {
//			uint8 u8Port;
//			for(u8Port = 0; (u8Port < UART_COMM_NUM) && (g_UartComm[u8Port].Handle != huart); u8Port++);
//			if(u8Port < UART_COMM_NUM) {
//				HAL_UARTEx_ReceiveToIdle_DMA(g_UartComm[u8Port].Handle, &g_UartComm[u8Port].u8TRBuf[0], UART_TR_BUF_BLEN);
//			}
//		}
//	}
//}
#else
void WriteToUart(uint8 u8Port, uint16 uNeedTxByte)
{
	HAL_UART_Transmit_DMA(g_UartComm[u8Port].Handle, g_UartComm[u8Port].u8TRBuf, uNeedTxByte);
	/*  */
	uint32_t u32MaxSendTime_ms = ((uint32)uNeedTxByte*15*1000)/g_UartComm[u8Port].Handle->Init.BaudRate;
	if(u32MaxSendTime_ms < 2) {
	    u32MaxSendTime_ms = 2;
	}
	if(osSemaphoreWait(g_UartComm[u8Port].Sem_TxCplt, u32MaxSendTime_ms) == osOK) {
	} else {
	}
}
extern uint8 GetUartPort(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* �������huart����Ҫת��u8Port */
    uint8 u8Port;
    for(u8Port = 0; (u8Port < UART_COMM_NUM) && (g_UartComm[u8Port].Handle != huart); u8Port++) {
    }
	if(u8Port < UART_COMM_NUM) {
		osSemaphoreRelease(g_UartComm[u8Port].Sem_TxCplt);
	}
}

uint16_t ReadFromUart(uint8 u8Port, uint16 uIniWaitTime_ms)
{
    /* 0����ʹ��Ĭ��ֵ: 3.5�ַ����(modbus��׼) */
    uint8 u8ByteIntvTime_ms = (3500 * 10)/g_UartComm[u8Port].Handle->Init.BaudRate;
    if(u8ByteIntvTime_ms < 2) {
        u8ByteIntvTime_ms = 2;
    }

	uint16 uRxBufPt = 0;
	HAL_UART_Receive_IT(g_UartComm[u8Port].Handle, &g_UartComm[u8Port].u8TRBuf[uRxBufPt], 1);
	while(osSemaphoreWait(g_UartComm[u8Port].Sem_RxGet, uIniWaitTime_ms) == osOK) {
		uRxBufPt++;
		if(uRxBufPt < UART_TR_BUF_BLEN) {
			HAL_UART_Receive_IT(g_UartComm[u8Port].Handle, &g_UartComm[u8Port].u8TRBuf[uRxBufPt], 1);
		} else {
			break;
		}
		uIniWaitTime_ms = u8ByteIntvTime_ms;
	}
	g_UartComm[u8Port].uRxBufPt = uRxBufPt;
	return uRxBufPt;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8 u8Port;
    for(u8Port = 0; (u8Port < UART_COMM_NUM) && (g_UartComm[u8Port].Handle != huart); u8Port++) {
    }
	if(u8Port < UART_COMM_NUM) {
		osSemaphoreRelease(g_UartComm[u8Port].Sem_RxGet);
	}
}
#endif

/***************************************************************************
		I2Cģ��
***************************************************************************/
/* I2CдSCL���ŵ�ƽ */
void I2C_W_SCL(uint8 u8Port, GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(cnst_I2C_BSP[u8Port].u32SclBank, cnst_I2C_BSP[u8Port].u32SclPin, PinState);
}

/* I2CдSDA���ŵ�ƽ */
void I2C_W_SDA(uint8 u8Port, GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(cnst_I2C_BSP[u8Port].u32SdaBank, cnst_I2C_BSP[u8Port].u32SdaPin, PinState);
}

/* I2C��SDA���ŵ�ƽ */
uint8_t I2C_R_SDA(uint8 u8Port)
{
	uint8_t BitValue;
	BitValue = (uint8_t)HAL_GPIO_ReadPin(cnst_I2C_BSP[u8Port].u32SdaBank, cnst_I2C_BSP[u8Port].u32SdaPin);
	return BitValue;
}

/* I2C��ʼ */
void I2C_Start(uint8 u8Port)
{
	I2C_W_SDA(u8Port, GPIO_PIN_SET);
	I2C_W_SCL(u8Port, GPIO_PIN_SET);
	I2C_W_SDA(u8Port, GPIO_PIN_RESET);
	I2C_W_SCL(u8Port, GPIO_PIN_RESET);
}

/* I2C��ֹ */
void I2C_Stop(uint8 u8Port)
{
	I2C_W_SDA(u8Port, GPIO_PIN_RESET);
	I2C_W_SCL(u8Port, GPIO_PIN_SET);
	I2C_W_SDA(u8Port, GPIO_PIN_SET);
}

/* I2C����һ���ֽ� */
void I2C_SendByte(uint8 u8Port, uint8_t Byte)
{
	uint8_t i;
	for(i = 0; i < 8; i++) {
		I2C_W_SDA(u8Port, Byte & (0x80 >> i));
		I2C_W_SCL(u8Port, GPIO_PIN_SET);
		I2C_W_SCL(u8Port, GPIO_PIN_RESET);
	}
}

/* I2C����һ���ֽ� */
uint8_t I2C_ReceiveByte(uint8 u8Port)
{
	uint8_t i, Byte = 0x00;
	I2C_W_SDA(u8Port, GPIO_PIN_SET);
	for(i = 0; i < 8; i++) {
		I2C_W_SCL(u8Port, GPIO_PIN_SET);
		if (I2C_R_SDA(u8Port) == 1) {
			Byte |= (0x80 >> i);
		}
		I2C_W_SCL(u8Port, GPIO_PIN_RESET);
	}
	return Byte;
}

/* I2C����Ӧ��λ */
void I2C_SendAck(uint8 u8Port, uint8_t AckBit)
{
	I2C_W_SDA(u8Port, AckBit);
	I2C_W_SCL(u8Port, GPIO_PIN_SET);
	I2C_W_SCL(u8Port, GPIO_PIN_RESET);
}

/* I2C����Ӧ��λ */
uint8_t I2C_ReceiveAck(uint8 u8Port)
{
	uint8_t AckBit;
	I2C_W_SDA(u8Port, GPIO_PIN_SET);
	I2C_W_SCL(u8Port, GPIO_PIN_SET);
	AckBit = I2C_R_SDA(u8Port);
	I2C_W_SCL(u8Port, GPIO_PIN_RESET);
	return AckBit;
}

/* I2Cд�Ĵ��� */
void I2C_WriteReg(uint8 u8Port, uint8_t u8DevAdd, const uint8_t* pUDat, uint16 uBLen)
{
	uint8 i;
	I2C_Start(u8Port);
	I2C_SendByte(u8Port, u8DevAdd);
	I2C_ReceiveAck(u8Port);
	for(i = 0; i < uBLen; i++,pUDat++) {
		I2C_SendByte(u8Port, *pUDat);
		I2C_ReceiveAck(u8Port);
	}
	I2C_Stop(u8Port);
}

/* I2C���Ĵ��� */
void I2C_ReadReg(uint8 u8Port, uint8_t u8DevAdd, uint8* pUDat, uint16 uBLen)
{
	uint8_t i;
	I2C_Start(u8Port);
	I2C_SendByte(u8Port, u8DevAdd | 0x01);
	I2C_ReceiveAck(u8Port);

	for(i = 0; i < uBLen; i++) {
		*pUDat++ = I2C_ReceiveByte(u8Port);
		if(i<uBLen-1) {
			I2C_SendAck(u8Port, 0);
		} else {
			I2C_SendAck(u8Port, 1);
		}
	}
	I2C_Stop(u8Port);
}
void OpenI2CComm(uint8 u8Port)
{
}

BOOL WriteToI2C(uint8 u8Port, uint8 u8DevAdd, const uint8* pUDat, uint16 uBLen, uint16 uTimeOut_ms)
{
	/* ���I2C���ͺ����滻 */
	taskDISABLE_INTERRUPTS();
	I2C_WriteReg(u8Port, u8DevAdd, (uint8*)pUDat, uBLen);
	taskENABLE_INTERRUPTS();
	return TRUE;
}

BOOL ReadFromI2C(uint8 u8Port, uint8 u8DevAdd, uint8* pUDat, uint16 uBLen, uint16 uTimeOut_ms)
{
	/* ���I2C���պ����滻 */
	taskDISABLE_INTERRUPTS();
	I2C_ReadReg(u8Port, u8DevAdd, pUDat, uBLen);
	taskENABLE_INTERRUPTS();
	return TRUE;
}
/***************************************************************************
		SPIģ��
***************************************************************************/
#if SPI_COMM_NUM
void OpenSPIComm(uint8 u8Port)
{
	SPI_COMM *pSPIComm = &g_SPIComm[u8Port];
	/* �����ź��� */
	osSemaphoreDef(SPIComm0_RxGet);
	pSPIComm->Sem_RxGet = osSemaphoreCreate(osSemaphore(SPIComm0_RxGet),1);
	osSemaphoreWait(pSPIComm->Sem_RxGet, 0);
	osSemaphoreDef(SPIComm0_TxGet);
	pSPIComm->Sem_TxCplt = osSemaphoreCreate(osSemaphore(SPIComm0_TxGet),1);
	osSemaphoreWait(pSPIComm->Sem_TxCplt, 0);
}
/*==========================================================================
| Description	: spi�շ�
| Author		: Cui				Date	: 2024-01-27
\=========================================================================*/
BOOL WriteToSPI(uint8 u8Port, const uint8 *pU8, uint16 uBLen, uint16 uTimeOut_ms)
{
	HAL_SPI_Transmit_IT(g_SPIComm[u8Port].Handle, (uint8*)pU8, uBLen);
	if(uTimeOut_ms == 0) {		/* 0Ϊ���ݴ����ͳ��ȼ��� */
		/* ÿ��ͨѶλ����uBLen*8������������ϣ�ÿB�࿼��1b�������Լ���������1ms����ʱ����1ms */
		uTimeOut_ms = (uBLen*9)/(HAL_RCC_GetPCLK2Freq()/g_SPIComm[u8Port].Handle->Init.BaudRatePrescaler) + 2;
	}
	return Semaphore_pend(g_SPIComm[u8Port].Sem_TxCplt, uTimeOut_ms);
}

BOOL ReadFromSPI(uint8 u8Port, uint8 *pU8, uint16 uBLen, uint16 uTimeOut_ms)
{
	HAL_SPI_Receive_IT(g_SPIComm[u8Port].Handle, pU8, uBLen);
	if(uTimeOut_ms == 0) {		/* 0Ϊ���ݴ����ͳ��ȼ��� */
		/* ÿ��ͨѶλ����uBLen*8������������ϣ�ÿB�࿼��1b�������Լ���������1ms����ʱ����1ms */
		uTimeOut_ms = (uBLen*9)/(HAL_RCC_GetPCLK2Freq()/g_SPIComm[u8Port].Handle->Init.BaudRatePrescaler) + 2;
	}
	return Semaphore_pend(g_SPIComm[u8Port].Sem_RxGet, uTimeOut_ms);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	uint8 u8Port;
	for(u8Port = 0; (u8Port < SPI_COMM_NUM) && (g_SPIComm[u8Port].Handle != hspi); u8Port++) {
	}
	if(u8Port < SPI_COMM_NUM) {
		osSemaphoreRelease(g_SPIComm[u8Port].Sem_TxCplt);
	}
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	uint8 u8Port;
	for(u8Port = 0; (u8Port < SPI_COMM_NUM) && (g_SPIComm[u8Port].Handle != hspi); u8Port++) {
	}
	if(u8Port < SPI_COMM_NUM) {
		osSemaphoreRelease(g_SPIComm[u8Port].Sem_RxGet);
	}
}
#endif

#if SUPPORT_FATFS
/***************************************************************************
		TF���洢
***************************************************************************/
void InitSDSPI(void)
{
	MX_FATFS_Init();
}
#endif

#if SUPPORT_NET
/***************************************************************************
		����
***************************************************************************/
int GetDnsAddrFromDhcpRecord(uint8 u8ServerIdx, uint8 *pIPServer)
{
#if DEV
	rc = CfgGetImmediate( 0, CFGTAG_SYSINFO,
								 CFGITEM_DHCP_DOMAINNAMESERVER,
								 serverIdx, 4, (UINT8 *)&IPServer );
#endif
	return 0x72727272;		/* 114.144.144.144 */
}
#endif

#ifdef CAN_BASE
/***************************************************************************
   canģ��
***************************************************************************/
/*Can FiFo�������жϣ�ע��can1��can2���á������ֿ���Ҫ���Դ��⺯����һ��*/
#include "MdlCan.h"
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* ��ȡ�жϺţ�����ĵ�ַ�����ݡ����� */
	uint32 u32CanDat[2] = {0, 0};	/* ��Ϊ������ܷ�8byte�������ҪԤ������ */
	CAN_RxHeaderTypeDef RxHeader;
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, (uint8*)u32CanDat) != HAL_OK) {
		return ;
	}
    Isr_CanRx(hcan, RxHeader.ExtId, RxHeader.FilterMatchIndex, u32CanDat, RxHeader.DLC);
}
/*Can FiFo���ж�*/
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
	//GPIO_toggle(GPIO_INDICATOR_StartNo + 5);
}
/*Can����mailbox0�����ж�*/
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	Isr_CanTx(hcan);
	//GPIO_toggle(GPIO_INDICATOR_StartNo + 5);
}
/*Can����mailbox1�����ж�*/
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	Isr_CanTx(hcan);
	//GPIO_toggle(GPIO_INDICATOR_StartNo + 5);
}
/*Can����mailbox2�����ж�*/
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	Isr_CanTx(hcan);
	//GPIO_toggle(GPIO_INDICATOR_StartNo + 5);
}

/* ��ʼ��can������ */
void InitCanFilter(void* hCan)
{
	CAN_HandleTypeDef* pCan = hCan;
	SET_BIT(pCan->Instance->FMR, CAN_FMR_FINIT);
	WRITE_REG(pCan->Instance->FS1R,0x0FFFFFFF);
	CLEAR_BIT(pCan->Instance->FMR, CAN_FMR_FINIT);
}

/* ����can������ */
void SetCanMsgFilter(void* hCan, uint32 u32MsgID, uint32 u32Mask, uint8 u8RxInt)
{
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.SlaveStartFilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterBank = u8RxInt;
	sFilterConfig.FilterIdHigh = (((uint32)u32MsgID<<3)&0xFFFF0000)>>16; //the ID that the filter looks for(switch this for the other microcontroller)
	sFilterConfig.FilterIdLow = (((uint32)u32MsgID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;
	sFilterConfig.FilterMaskIdHigh = (((uint32)u32Mask<<3)&0xFFFF0000)>>16;
	sFilterConfig.FilterMaskIdLow = (((uint32)u32Mask<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;
	HAL_CAN_ConfigFilter(hCan, &sFilterConfig); //configure CAN filter
}

/* ͨ��can���߷������� */
void WriteDataToCan(void* hCan, uint8 u8DataGroup, uint8 u8DevAdd, uint16 uDataPt, uint8* pU8Dat, uint8 u8ByteNum, BOOL bTxIntEnable)
{
	uint32 HwiKey = Hwi_disable();

    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.ExtId = (uint32)u8DataGroup*0x1000000 + (uint32)u8DevAdd*0x10000 + (uint32)uDataPt;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = u8ByteNum;
    TxHeader.TransmitGlobalTime = DISABLE;
    if(HAL_CAN_IsTxMessagePending(hCan, CAN_TX_MAILBOX0) == 0) {
    	HAL_CAN_AddTxMessage(hCan, &TxHeader, pU8Dat, (uint32_t*)CAN_TX_MAILBOX0);
    } else if(HAL_CAN_IsTxMessagePending(hCan, CAN_TX_MAILBOX1) == 0) {
    	HAL_CAN_AddTxMessage(hCan, &TxHeader, pU8Dat, (uint32_t*)CAN_TX_MAILBOX1);
    } else if(HAL_CAN_IsTxMessagePending(hCan, CAN_TX_MAILBOX2) == 0) {
    	HAL_CAN_AddTxMessage(hCan, &TxHeader, pU8Dat, (uint32_t*)CAN_TX_MAILBOX2);
    } else {
    	//return 0;
    }

    Hwi_restore(HwiKey);
}

void ChkCanBusOff(void)
{

}
#endif


/******************************** FILE END ********************************/
