/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved
 *
 * File			: DrvTI.c
 * Author		: Wang Renfei
 * Description	: 板支持软件
 
 *				: 
 * Date			: 2022-7-4
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		| 2022-07-04  |	W.R.F| 创建
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
const SECTION(".runver") uint32 g_u32RunVer = (uint32)SOFTWARE_VER | (((uint32)0-SOFTWARE_VER) << 16);			/* 16位原码+16位补码方式校验存储 */
const SECTION(".softver") uint32 g_u32SoftVer = (uint32)SOFTWARE_VER | (((uint32)0-SOFTWARE_VER) << 16);		/* 16位原码+16位补码方式校验存储 */

/***************************************************************************
						internal functions declaration
***************************************************************************/
uint32* GetCounterpartAddr(uint32 *pVal);
SECTION(".BootSection") void Reset_Handler(void)
{
	__disable_irq();

#if DEVICE_TYPE == V5_YYT4
	/* 开启双bank模式，让flash地址操作更方便，防止已经开启了此选项的板子运行异常 */
	if((FLASH->OPTCR & FLASH_OPTCR_DB1M) == 0) {
		for(;;) {
			if((FLASH_WaitForLastOperation((uint32_t)50) == HAL_OK)
				&& (HAL_FLASH_OB_Unlock() == HAL_OK)
				&& (FLASH->OPTCR |= FLASH_OPTCR_DB1M)		/* 开启双bank模式（DB1M位在cubeIDE里看不到，但st文档里有，实际也有，1Mflash的默认是0，2M的默认是1） */
				&& (HAL_FLASH_OB_Launch() == HAL_OK)
				&& (HAL_FLASH_OB_Lock() == HAL_OK))
			{
				break;
			}
		}
	}

	/* 关闭双bank启动，防止已经开启了此选项的板子运行异常 */
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

	/* 读出软件完整性数据*/
	BOOL bRunInHighFlash = CheckExeFlash_0Low_1High();
	BOOL bNeedUpdateRunVer = FALSE;
	BOOT_SOFT_VERSION BootSoftVer;
	GetSoftVersion(&BootSoftVer);

	/* 如果本半区的指定版本号不可用就用另半区的，如果另半区的也不可用，那就只能取两个区实际版本号的高版本了 */
	if((BootSoftVer.u32RunVersion = GetVerVar(GetVolatileFromU32Addr(&g_u32RunVer))) == 0) {
		BootSoftVer.u32RunVersion = GetVerVar(GetVolatileFromU32Addr(GetCounterpartAddr((uint32*)&g_u32RunVer)));
		bNeedUpdateRunVer = TRUE;
	}

	/* 如果两个软件完整性都很好,则根据指定来; 如果没有指定，则默认高版本的.
	   否则，选择软件完整性良好的; 如果两个软件完整性都有问题，则选择低地址的 */
	if(pOldBootLoaderConf->_c_int00_add == 0xFFFFFFFFUL) {		/* 如果另一半flash是空的，则没得选 */
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
		/* 如果不是故意重启，且启动时间过短，则回滚回另一个版本 */
#define RESET_COUNT_TO_ROOLBACK		10			/* times */
#define MINIMAL_TIME_TO_ROOLBACK	(60 * 2)	/* seconds */
		if(RCC->CSR & RCC_CSR_PORRSTF) {	/* 如果是冷启动则初始化重启计数相关变量 */
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

	/* 按需更新启动版本字段，防止两个版本互相跳 */
	if(bNeedUpdateRunVer) {
		uint32 u32RunVer;
		/* 根据实际的版本号更新要运行的版本号 */
		if(CheckExeFlash_0Low_1High() ^ bRunInHighFlash) {
			u32RunVer = GetVolatileFromU32Addr(GetCounterpartAddr((uint32*)&g_u32SoftVer));
		} else {
			u32RunVer = GetVolatileFromU32Addr(&g_u32SoftVer);
		}
		/* 改写本半区运行版本号（持久化到flash） */
		FlashErase2((uint32)((uint8*)&g_u32RunVer), sizeof(g_u32RunVer));
		FlashProgram2(&u32RunVer, (uint32)&g_u32RunVer, 4, TRUE);

		/* 改写另外半区运行版本号（持久化到flash） */
		FlashErase2((uint32)((uint8*)GetCounterpartAddr((uint32*)&g_u32RunVer)), sizeof(g_u32RunVer));
		FlashProgram2(&u32RunVer, (uint32)((uint8*)GetCounterpartAddr((uint32*)&g_u32RunVer)), 4, TRUE);
	}

	/* 是否需要跳转到高半区 */
	if(CheckExeFlash_0Low_1High() ^ bRunInHighFlash) {					/* 需要跳转到另一半 */
		SCB->VTOR = FLASH_BASE + bRunInHighFlash*FLASH_HALF_BLEN;		/* 重新确定中断向量表首地址 */
		uint32 * pVtor = (uint32*)(FLASH_BASE + bRunInHighFlash*FLASH_HALF_BLEN);
		__DSB();
	    // 设置主堆栈指针（MSP）
	    asm volatile ("MSR MSP, %0" :: "r" (*pVtor));
	    pVtor++;
	    // 设置复位向量地址（PC）
	    asm volatile ("BX %0" :: "r" (*pVtor));
	    __DSB();
		for(;;) {}
	}

	(*(void(*)())pBootLoaderConf->_c_int00_add)();	/*	_c_int00(); */
}

/***************************************************************************
		任务, 信号量
***************************************************************************/
/*******************************************************************************
 * 创建任务
 * pName:  任务名称
 * func:任务主函数	(void (*os_pthread) (void const *argument))
 * pArgs: 参数(TODO:多于一个参数)
 * prio: 初始优先级
 * u8MaxInstanceNum: 最大实例数量(赋值为0的是instances，看文档说应该是后面会创建几个Thread，但实际弄成0也可以)
 * pStackBuf:堆栈空间
 * u32StackLen:堆栈大小（字节）
 * pCtlBlkContainer: 对于支持静态创建的架构来说，需要通过此参数显式指定此任务控制块的承载空间
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
 *  创建信号量 （uCount为1时，初始值实际为0，暂时没找到修改的地方，如果需要初始值为1，需要先手动post一次）
 * pName:  信号量名称
 * u8Count: 信号量计数
 * pCtlBlkContainer:	  如果此平台支持静态创建，则用于静态创建的数据结构承载，如果不支持则直接传NULL
 * 				静态：需要传入一个地址，承载osStaticSemaphoreDef_t类型
 * 				动态：传入地址为NULL，即从堆中创建
 * 注：用已过时的vSemaphoreCreateBinary()创建的信号量初始值是1，
 * 	   而用新接口xSemaphoreCreateBinary()创建的信号量初始值是0
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
		复位函数, 看门狗, 延时等
***************************************************************************/
/* 断开调试器(通过将OptionByte（OB）的RDP设为1来实现)
 * RDP == 0 ：无保护
 * RDP == 1 ：读保护
 * RDP == 2 ：砖保护
 * 解锁（从RPD1还原回RDP0，可以用stlink-utility等工具修改；RDP2基本无法还原） */
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
			OBInit.RDPLevel = OB_RDP_LEVEL_1;		/* 设置RDP Level 1 */
			if (HAL_FLASHEx_OBProgram(&OBInit) == HAL_OK ) {
			}
			HAL_FLASH_OB_Launch();
			HAL_FLASH_Lock();
		}
	}
#endif
}

/* 如果不是正式运行版本就喂狗（差不多最迟4秒喂狗一次，否则MCU被复位） */
/* 看门狗一旦初始化除非重启否则无法关闭或暂停，所以在main.c里按需开启。其实也可以通过关闭LSI RC时钟方式暂停 */
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

/* 空转延时 */
void Boot_SysCtlDelay(uint32 ui32Count)
{
    for( ; ui32Count != 0; ui32Count--) {
        NOP;
    }
}

/***************************************************************************
        启动，升级，软件版本
***************************************************************************/
/* 从高低16互为补码校验的版本号的4字节变量中解析版本号，如果检测到校验失败返回0，否则返回版本号，如3688 */
uint32 GetVerVar(uint32 u32VersionVar)
{
	if((u32VersionVar&0xFFFF) + (u32VersionVar>>16) == 0x10000UL) {
		return u32VersionVar&0xFFFF;
	} else {
		return 0;
	}
}

/* 获取此地址所在的全局变量的另外半区的变量值 */
uint32 GetCounterpart(uint32 *pVal)
{
	if(CheckExeFlash_0Low_1High()) {
		return *(uint32*)(((uint8*)pVal) - FLASH_HALF_BLEN);			/* 高一半地址的这个全局变量 */
	} else {
		return *(uint32*)(((uint8*)pVal) + FLASH_HALF_BLEN);			/* 低一半地址的这个全局变量 */
	}
}

/* 获取此地址所在的全局变量的另外半区的地址 */
uint32* GetCounterpartAddr(uint32 *pVal)
{
	if(CheckExeFlash_0Low_1High()) {
		return (uint32*)(((uint8*)pVal) - FLASH_HALF_BLEN);			/* 高一半地址的这个全局变量 */
	} else {
		return (uint32*)(((uint8*)pVal) + FLASH_HALF_BLEN);			/* 低一半地址的这个全局变量 */
	}
}

uint32 GetVolatileFromU32Addr(const uint32 *pU32Val)
{
	return (*(volatile uint32*)pU32Val);
}

/* 获得flash里面的软件版本 */
void GetSoftVersion(BOOT_SOFT_VERSION* pBootSoftVer)
{
	pBootSoftVer->u32RunVersion = SOFTWARE_VER;		/* 当前运行的实际软件版本号 */
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

/* 破坏另一个软件版本的完整性标志 */
void InvalidateAnotherSoft(void)
{
	FlashErase2((uint32)GetCounterpartAddr((uint32*)&g_u32RunVer), BOOT_VER_BLEN);			/* 此启动版本号区域在升级时临时会被当成暂存接收到的bootloader的区域，等升级后再恢复成启动版本号区 */
	g_Sys.uTmr_EraseFlash_DistPwr_ms = 4000;	/* 擦除flash过程，可能会引起电源电压不稳，并造成测量异常 */
}

#if (CPU_0ST_1TI == 1)
	uint32 u32SoftIntergrity = 0;
	/* 如果当前运行的是高地址，则写入的就是低地址 */
	EEPROMProgram((uint32_t*)&u32SoftIntergrity,
					EEADD_BOOT_LOADER_CONF + 4 + 4*(!IsMirrorMode()), sizeof(u32SoftIntergrity));
	g_Sys.uTmr_EraseFlash_DistPwr_ms = 1000;	/* 擦除flash过程，可能会引起电源电压不稳，并造成测量异常 */
#endif

/* 交换两个flash bank */
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
		/* 设置BFB2 = !UFB */
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
		/* 如果所有条件都具备，则向FLASH_BOOT_VER的g_u32RunVer写入另一个版本的版本号，重启后便可从另一个版本启动 */
		uint32 u32RunVer = GetVolatileFromU32Addr(GetCounterpartAddr((uint32*)&g_u32SoftVer));
		if(u32RunVer != 0xFFFFFFFF) {			/* 另一半地址第一个4字节不等于0xFFFFFFFF(空的) */
			g_CodeTest.u32Val[12] = GetVolatileFromU32Addr(&g_u32RunVer) & 0xFFFF;
			/* 改写本半区运行版本号 */
			FlashErase2((uint32)((uint8*)&g_u32RunVer), sizeof(g_u32RunVer));
			FlashProgram2(&u32RunVer, (uint32)&g_u32RunVer, 4, TRUE);

			/* 改写另外半区运行版本号 */
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
 * bWhichHalf_0Fst_1Sec要写入哪个半区
 * u32FlashAddr是上半区的相对地址，如需绝对，需要用bWhichHalf_0Fst_1Sec处理
 * pU32FlashData代码片段
 * u32FlashFragBLen代码片长度 */
BOOL FlashUpgradeFrag(BOOL bWhichHalf_0Fst_1Sec, uint32 u32FlashAddr, uint32* pU32FlashData, uint32 u32FlashFragBLen)
{
	/* 真实写入的Flash地址，是需要写入备份区 */
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

	int8 i8TryCnt;		/* 最多重试次数 */
	/* 处理bootloader区域，如果地址属于bootloader区域，则拷到FLASH_BOOT_VER区暂存，等最后代码片全拷贝完整后, 最后会再一起拷贝到bootloader所属的FLASH_BOOT_SEG区, 注意，当前算法只保证起始地址 >= 跳过地址 */
	if((u32FlashAddr >= FLASHADD_IVT_START) && ((u32FlashAddr-FLASHADD_IVT_START) < FLASH_IVT_BLEN)) {
		g_CodeTest.u32Val[80]++;		//00000006
		/* 连续写入两次，为容错 */
		for(i8TryCnt = 2; i8TryCnt > 0; i8TryCnt--) {
			g_CodeTest.u32Val[87] = (uint32)pU32FlashData;	//20012F80
			g_CodeTest.u32Val[88] = BOOT_VER_START + (u32FlashAddr - FLASHADD_IVT_START) + bWhichHalf_0Fst_1Sec*FLASH_HALF_BLEN;		//0808BDE0
			g_CodeTest.u32Val[89] = u32FlashFragBLen - ((((uint8)(u32SrcEnd > u32DstStart)) * (u32SrcEnd - u32DstStart)) - (((uint8)(u32SrcStart > u32DstStart)) * (u32SrcStart - u32DstStart)));		//00000220
			if(FlashProgram2(pU32FlashData
					, BOOT_VER_START + (u32FlashAddr - FLASHADD_IVT_START) + bWhichHalf_0Fst_1Sec*FLASH_HALF_BLEN
					, u32FlashFragBLen - ((((uint8)(u32SrcEnd > u32DstStart)) * (u32SrcEnd - u32DstStart)) - (((uint8)(u32SrcStart > u32DstStart)) * (u32SrcStart - u32DstStart)))
					, TRUE))
			{
				/* 如更新区域完全在bootloader内，则只接算成功 */
				if((u32SrcEnd - FLASHADD_IVT_START) <= FLASH_IVT_BLEN) {
					g_CodeTest.u32Val[81]++;		//00000005
					return TRUE;
				} else {
					g_CodeTest.u32Val[82]++;		//00000001
					/* 如片段中的内容还有bootloader之后的部分，则修正地址留给下面的FlashProgram2统一拷贝或跳过 */
					u32SrcStart = FLASHADD_IVT_START + FLASH_IVT_BLEN;
					u32FlashFragBLen = u32SrcEnd - (FLASHADD_IVT_START + FLASH_IVT_BLEN);
					break;
				}
			}
		}
		if(i8TryCnt == 0) {
			g_CodeTest.u32Val[83]++;		//00000000
			return FALSE;		/* 如果bootloader不能暂存，则直接算升级失败 */
		}
	}

	/* 处理非bootloader区域，为支持跳过某块区域的拷贝，将所有拷贝都分两步，先拷贝跳过区之前的再拷之后的 */
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
	return (FlashProgram2((uint32_t*)pU32FlashData, u32SrcStart, u32FlashFragBLen, TRUE));	/* 写入flash，耗时约58ms，不会影响任务调度, 为兼容STM32，原先u32RealAddr变量改成宏SRC_START */
}
#endif

/* ST ver */
void OnPostLiveUpdate(void)
{
//	extern uint32 cnst_BootLoaderResetVectors[];
	/* 更新bootloader，和另外半区（正升级）的bootloader比较，如果有不同再更新bootloader，当前半区的bootloader不动（意为bootloader随所属主程序版本走） */
	uint32 u32SrcStart = (uint32)GetCounterpartAddr((uint32*)&g_u32RunVer);		/* 升级过程中借用了g_u32RunVer所开头的section*/
	extern uint32 cnst_BootLoaderResetVectors[];
	uint32 u32DstStart = (uint32)GetCounterpartAddr((uint32*)cnst_BootLoaderResetVectors);
//	g_CodeTest.u32Val[30] = u32SrcStart;
//	g_CodeTest.u32Val[31] = u32DstStart;
	uint32 i = 0;
	for(; i < FLASH_IVT_BLEN; i += 4) {
		if((*((uint32*)(u32SrcStart + i))) != (*((uint32*)(u32DstStart + i)))) {		/* 如果发现不同，则更新bootloader */
			g_CodeTest.u32Val[32]++;
			FlashErase2(u32DstStart, FLASH_IVT_BLEN);			/* 清除中断向量表sector */
			/* bootloader拷贝失败，需要立即重试，因为如果拷贝的是低版本则会无法启动， 先尝试更新最新下载的，失败就尝试用另一个半区的恢复 */
			if(FlashProgram2((uint32*)u32SrcStart, u32DstStart, FLASH_IVT_BLEN, TRUE)
					|| FlashProgram2((uint32*)u32SrcStart, u32DstStart, FLASH_IVT_BLEN, TRUE)
					|| FlashProgram2((uint32*)u32SrcStart, u32DstStart, FLASH_IVT_BLEN, TRUE))
			{
				/* 如果三次更新bootloader都失败只能放弃，防止flash写废 */
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
		/* 检查高一半flash、低一半flash的bootloader（如果有）是否相等 */
		uint32* pU32Src = (uint32*)(FLASHADD_BOOTLOADER);
		uint32* pU32Dest = (uint32*)(FLASHADD_BOOTLOADER + FLASH_HALF_BLEN);
		for(i = BOOT_LOADER_BLEN/4; (i > 0) && (*pU32Src++ == *pU32Dest++); i--) {
		}
		if(i) {	/* 高一半flash和低一半flash的bootloader不同，需要复制 */
			FlashErase2(FLASHADD_BOOTLOADER, BOOT_LOADER_BLEN);		/* 擦除BOOT_LOADER区域 */

			/* 把低一半flash的 bootloader所在的flash单元 拷贝到 高一半flash的 必须是一个最小擦除单元 */
			FlashProgram2((uint32_t*)(FLASHADD_BOOTLOADER), (FLASHADD_BOOTLOADER + FLASH_HALF_BLEN), BOOT_LOADER_BLEN, TRUE);
		}
	}

	/* 标记是软件更新所引起的重启 */
	uint32 u32RebootFlag = REBOOT_BY_SOFT_UPDATE;
	EEPROMProgram((uint32_t*)&u32RebootFlag, EEADD_REBOOT_FLAG, sizeof(u32RebootFlag));

	/* 标志软件完整性 */
	uint32 u32SoftVersion = (((uint32)(0 - g_SoftUpdateCtr.uReqSoftVer))<<16) | g_SoftUpdateCtr.uReqSoftVer;
	/* 如果当前运行的是高地址，则写入的就是低地址 */
	EEPROMProgram((uint32_t*)&u32SoftVersion, EEADD_BOOT_LOADER_CONF + 4 + 4*(!IsMirrorMode()), sizeof(u32SoftVersion));	/* 写入低或高版本号 */
	EEPROMProgram((uint32_t*)&u32SoftVersion, EEADD_BOOT_LOADER_CONF, sizeof(u32SoftVersion));		/* 写入运行版本号 */
}
#endif

/* 回滚之后 */
void SoftRollBack(uint32 u32RunVersion)
{
	/* 判断版本, 如果版本和另一个bank的版本号相符则回滚 */
	volatile uint32 u32SoftVerDat = GetVerVar(GetVolatileFromU32Addr(GetCounterpartAddr((uint32*)&g_u32SoftVer)));
	if(u32SoftVerDat && (u32RunVersion == u32SoftVerDat)) {		/* 校验检查, 版本号核对 */
		SwapSoftware();
		g_Sys.tReboot_0Null_nUrgent_pWaitIdle = 1;
	}
}
#if (CPU_0ST_1TI == 1)
void SoftRollBack(uint32 u32RunVersion)
{
	/* 标记是软件更新所引起的重启 */
	SoftRollBack(u32RunVersion);
	uint32 u32RebootFlag = REBOOT_BY_SOFT_UPDATE;
	EEPROMProgram((uint32_t*)&u32RebootFlag, EEADD_REBOOT_FLAG, sizeof(u32RebootFlag));
	/* 更新目标软件版本号 */
	u32RunVersion = ((0-u32RunVersion)<<16) | u32RunVersion;
	EEPROMProgram((uint32_t*)&u32RunVersion, EEADD_BOOT_LOADER_CONF, sizeof(u32RunVersion));
	g_Sys.tReboot_0Null_nUrgent_pWaitIdle = 1;
}
#endif

#if CPU_0ST_1TI == 0
/* 用于计算默认sn */
uint32 CalcUid16B(void)
{
    uint32 u32ChipID[UID_U32LEN];
    CalcUid(u32ChipID);
	return (uint32)CalCRC16ForModbus((uint8*)u32ChipID, UID_U32LEN * 4);
}
#else
/* 用于计算默认sn */
uint32 CalcUid16B(void)
{
	return (((uint32)rand()<<16) + rand())%1000000UL;
}
#endif

/* 用芯片System Control Register 的 UniqueID地址，返回值等于 0x400FEF20，但不应该直接用该立即数地址，因其容易被利用 */
uint32 GetChipIDHash(void)
{
	uint32 u32ChipID[UID_U32LEN];
	CalcUid(u32ChipID);
    /* 对芯片ID先加密，再计数hash值 */
    EncryptWithAES(AUTH_AES_PACK_SERIAL, u32ChipID, 16, 0);
    return CalCRC32ForFileByIni((uint8*)u32ChipID, 16, 0);
}

/***************************************************************************
        片上存储: 用flash模拟eeprom
        目前仅支持配置数据，不支持Msg,Acq，flash单页是
***************************************************************************/
void EEPROMRead(uint32 *pui32Data, uint32 u32Address, uint32 u32ByteLen)
{
	FlashRead2(u32Address, pui32Data, u32ByteLen);
}

void EEPromErase(uint32 ui32Address)
{
	FlashErase2(ui32Address, EEPROM_BLEN);		/* stm32暂时采用整sector（flash模拟eeprom）清除策略 */
}

uint32 EEPROMProgram(uint32 *pui32Data, uint32 ui32Address, uint32 ui32Count)
{
	FlashProgram2(pui32Data, ui32Address, ui32Count, FALSE);
	return 0;		/* 为兼容TI的函数签名 */
}

/* 读取指定地址的半字(16位数据)
* faddr:读地址
* 返回值:对应数据.*/
uint32_t FlashReadU32(uint32_t u32FlashAddr) {
	return *(volatile unsigned long*) u32FlashAddr;
}

/* 从指定地址开始读出指定长度的数据, 之所以名字加个2，是因为避免和TI的重名
* ReadAddr:起始地址
* pBuffer:数据指针
* u32ByteLen:字节数，必须是4的整数 */
void FlashRead2(uint32_t u32ReadAddr, uint32_t *pBuffer, uint32_t u32ByteLen) {
	uint32_t i = 0;
	for(i = 0; i < (u32ByteLen/4); i++) {
		pBuffer[i] = FlashReadU32(u32ReadAddr);	/* 读取4个字节 */
		u32ReadAddr += 4;
	}
}


/* 从指定地址开始写入指定长度的数据, 之所以名字加个2，是因为避免和TI的重名
* u32Dst，flash起始地址, 需要4的整数倍地址
* pSrc:数据指针
* u32BLen:字节数（假设是4的整数倍）
* bValidate 是否验证
* 返回是否写入操作成功，如果操作成功且开启验证，则返回验证结果 */
BOOL FlashProgram2(uint32_t *pSrc, uint32_t u32Dst, uint32_t u32BLen, BOOL bValidate) {
	BOOL bSuccess = FALSE;
	if(u32BLen > 0) {
		const uint32* pOriginSrc = pSrc;
		const uint32* pEndSrc = pSrc + (u32BLen / 4);
		__IO uint32_t* pDst = ((__IO uint32*)u32Dst);

		/* 顺序按指定长度写入内容 */
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
				/* 读取验证内容是否一模一样 */
				pSrc = (uint32*)pOriginSrc;
				pDst =  ((__IO uint32*)u32Dst);
				for(; pSrc < pEndSrc; pSrc += 1) {
					if (*pSrc != *pDst) {
						break;
					} else {
						pDst++;
					}
				}
				/* 如果完全一样，返回TRUE */
				if (pSrc == pEndSrc) {
					bSuccess = TRUE;
				}
			} else {
				bSuccess = TRUE;
			}
		}
	} else if(u32BLen == 0) {		/* 为方便外部统一算法，等于0则也判定成功 */
		bSuccess = TRUE;
	}
	return bSuccess;
}

BOOL IsRebootBySoftUpdate(void)
{
    return TRUE;		/* 从EEPROM区读取 */
}

/* 检查是否断开了调试器（通过检查RDP是否为0实现） */
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
/* 检测软件版本，返回是否是全新flash上运行 */
BOOL CheckSoftVer(void)
{
#if 0
	/* 判断进行了芯片解锁，如果是这样就可以成为发布软件的母板 */
	BOOT_SOFT_VERSION BootSoftVer;
	EEPROMRead((uint32_t*)&BootSoftVer, EEADD_BOOT_LOADER_CONF, sizeof(BOOT_SOFT_VERSION));

	/* flash高一半是被擦除的, 即全新运行的 */
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
/* 检测软件版本，返回是否是全新flash上运行 */
BOOL CheckSoftVer(void)
{
	/* flash高一半是被擦除的, 即全新运行的 */
	if((!CheckExeFlash_0Low_1High())
		&& (pOldBootLoaderConf->_c_int00_add == 0xFFFFFFFFUL)
		&& ((uint32)(pOldBootLoaderConf->pConfProp) == 0xFFFFFFFFUL)
		&& (pOldBootLoaderConf->u32ItemNum == 0xFFFFFFFFUL)
		&& (pOldBootLoaderConf->u32ConfSaveGroupNum == 0xFFFFFFFFUL))
	{
		/* 判断进行了芯片解锁，如果是这样就可以成为发布软件的母板 */
		BOOT_SOFT_VERSION BootSoftVer;
		EEPROMRead((uint32_t*)&BootSoftVer, EEADD_BOOT_LOADER_CONF, sizeof(BOOT_SOFT_VERSION));
		if(SOFT_RUN1_TEST0
			&& (BootSoftVer.u32RunVersion == 0xFFFFFFFFUL)		/* eeprom被擦除--主要是通过解锁-jtag下载过程 */
			&& (BootSoftVer.u32LowFlashVer == 0xFFFFFFFFUL)
			&& (BootSoftVer.u32HighFlashVer == 0xFFFFFFFFUL))
		{
			g_PubSoftAuthCtr.uTmr_IniPubCntNoPwd_ms = 60000;
		}

		/* 设置运行版本 */
		BootSoftVer.u32LowFlashVer = ((((uint32)(0-SOFTWARE_VER))<<16) | SOFTWARE_VER);
		BootSoftVer.u32RunVersion = BootSoftVer.u32LowFlashVer;
		BootSoftVer.u32HighFlashVer = 0;
		EEPROMProgram((uint32_t*)&BootSoftVer, EEADD_BOOT_LOADER_CONF, sizeof(BOOT_SOFT_VERSION));
	}
}
#endif

/* 持久化SAVE_GRP */
void PersistSaveGrp(void)
{
	/* 确定是否有需要保存的SAVE_GRP */
	CONF_SAVE_GROUP ConfAcsGrp;
	for(ConfAcsGrp = SAVE_GRP_BRD; ConfAcsGrp < MAX_SAVE_GRP_NUM; ConfAcsGrp++) {
		if(g_DataAcsIntf.bConfHaveUnsavedChange[ConfAcsGrp] || (g_DataAcsIntf.bConfHaveUnsavedChange[SAVE_GRP_NUL])) {		/* 由于stm32目前存储策略是整擦整写，所以只要有一个SAVE_GRP需要存储就全部擦除，然后重新更新存储所有SAVE_GRP */
			EEPromErase(EEADD_START);						/* 擦除一个BANK的EEPROM模拟区（sector）,TODO:调整为分别擦除,降低中途中断丢数据概率 */
			EEPromErase(EEADD_START + FLASH_HALF_BLEN);		/* 擦除另一个BANK的EEPROM模拟区（sector）,TODO:调整为分别擦除,降低中途中断丢数据概率 */
			/* 保存全部SAVE_GRP */
			for(ConfAcsGrp = SAVE_GRP_BRD; ConfAcsGrp < MAX_SAVE_GRP_NUM; ConfAcsGrp++) {
				g_DataAcsIntf.bConfHaveUnsavedChange[ConfAcsGrp] = FALSE;
				g_DataAcsIntf.bConfFree[ConfAcsGrp] = FALSE;
				/* 存EEPROM模拟区，主 */
				if(pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr != 0) {
					SaveConfToEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr);
					if(!CheckConfFromEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr)) {
						SaveConfToEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr);
					}
				}
				/* 存EEPROM模拟区，备 */
				if(pBootLoaderConf->AcsMedia[ConfAcsGrp].u32BackEEAddr != 0) {
					SaveConfToEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32BackEEAddr);
					if(!CheckConfFromEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32BackEEAddr)) {
						SaveConfToEEProm(ConfAcsGrp, pBootLoaderConf->AcsMedia[ConfAcsGrp].u32BackEEAddr);
					}
				}
#if SUPPORT_FAT_FS
				/* 存SD卡 */
				SaveConfToFile(ConfAcsGrp) ;
				if(ReadOrCheckConfFromFile(ConfAcsGrp, -2) != DATA_ACS_RES_SUC) {
					SaveConfToFile(ConfAcsGrp);
				}
#endif
			}
			g_DataAcsIntf.bConfFree[ConfAcsGrp] = TRUE;
			g_DataAcsIntf.bConfHaveUnsavedChange[SAVE_GRP_NUL] = FALSE;	/* 用于ccs调试状态下使用 */
			break;
		}
	}
}

#if CPU_0ST_1TI == 1
/* 持久化SAVE_GRP */
void PersistSaveGrp(void)
{
	CONF_SAVE_GROUP ConfAcsGrp;
	triST tSaveUrgent_0Idle_pReq_nCmplt = g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt;
	for(ConfAcsGrp = SAVE_GRP_BRD; ConfAcsGrp < MAX_SAVE_GRP_NUM; ConfAcsGrp++) {
		Swi_disable();      /* 避免判断后发生任务切换 */
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
	/* 复位存储 */
	g_DataAcsIntf.bConfHaveUnsavedChange[SAVE_GRP_NUL] = FALSE;	/* 用于ccs调试状态下使用 */
}
#endif
/***************************************************************************
	CPU温度 & CPU Ref & 延时
***************************************************************************/
#if DC_TOTAL_CHN
float32 CalCPUTemperature(uint8 u8DCSigNo)
{
    /* Temperature(in °C) = {(VSENSE – V25)/Avg_Slope} + 25, 其中 V25 = 0.76V, Avg_Slope = 2.5mV/°C     */
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
    RTC模块，需要ioc配置成:
    1. 时钟选择外部32.768KHz晶振(LSE)
    2. Asynchronous Predivider value 31，Synchronous Predivider value 1023
***************************************************************************/
//#include "stm32f4xx_hal_rtc.h"
extern RTC_HandleTypeDef hrtc;
void GetRealTime(REAL_TIME_VAR* pRealTime)
{
    RTC_TimeTypeDef RtcTime;
    RTC_DateTypeDef RtcDate;
    /* 为了保障数据完整性，在读取低位寄存器(如:秒)，硬件会自动锁存高位(如:日期)--读取的时钟需要7倍RTC_Clk(32.768KHz) */
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

    /* 授权验证 */
    uint32 u32Seconds = CalRTCSecondsByDate(pRealTime->u8Year, pRealTime->u8Month, pRealTime->u8Day, 
                                            pRealTime->u8Hour, pRealTime->u8Minute, pRealTime->u8Second);
    if((u32Seconds < 1800)                                  /* 上电半个小时内, 如果时钟没有初始化完成, 不进行授权检查 */
        || (g_Sys.u32License_Seconds == 4102358400UL))      /* 永久授权, 99/12/31     00:00:00 */
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
    /* 读年月日时分秒其实没用，但是如果不这样，下次读取的时候，相应寄存器就会保持在上一次读SSR(毫秒计时器)时刻 */
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

    /* HAL_RTC_SetTime() HAL_RTC_SetDate()执行后，都会清除SSR(毫秒计时器);
    而这段代码执行时间远低于1秒，因此不存在数据完整性的问题，即：进位引起的数据错乱 */
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
 							Uart模块
***************************************************************************/
#include "MdlUARTnModbus.h"
#define MIN_READ_TIMEOUT_ms 2
/* 打开RS485通讯端口，并初始化相应的内存变量 */
extern UART_HandleTypeDef huart1;

///*---------------GPRS接收双缓冲区---------------*/
//#define GPRS_UART_RX_DOUBLE_BUFF_MAX_LEN		128		/* gprs的串口接收双缓冲区buff大小 */
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

/* 初始化双缓冲区 */
void DoubleBuff_Init(UART_DoubleBuff_t *pDoubleBuff)
{
	pDoubleBuff->pU8CurrentBuf = pDoubleBuff->aU8RxBufA;
	pDoubleBuff->pU8NextBuf = pDoubleBuff->aU8RxBufB;
}

//uint8 aU8GPRS_Buff[UART_TR_BUF_BLEN] = {0};
void OpenUartComm(uint8 u8Port, uint32 u32Baud, uint8 u8Parity_0Nul_1Odd_2Even, uint16 uReadTickOut_0Default)
{
    /* 初始化内存 */
	g_UartComm[u8Port].uTimer_WatchDog_ms = 0xFFFF;
	g_UartComm[u8Port].uLastBaud_100bps = u32Baud/100;
	g_UartComm[u8Port].uTime_RxMaxIntv_ms = uReadTickOut_0Default / OS_TICK_KHz;

	/* 硬件初始化 */
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

    if(u8Port == GPRS_UART_PORT) {	/* GPRS通信使用DMA循环接收 */
    	g_GPRSRingBuffer.uHead = g_GPRSRingBuffer.uRear = (RING_BUFFER_MAX_SIZE - __HAL_DMA_GET_COUNTER(g_UartComm[u8Port].Handle->hdmarx)) % RING_BUFFER_MAX_SIZE;
//    	RingBuffer_Init(&g_GPRSRingBuffer);
//    	HAL_UART_DMAStop(g_UartComm[u8Port].Handle);		/* 停止DMA */
    	HAL_UART_Receive_DMA(g_UartComm[u8Port].Handle, g_GPRSRingBuffer.aU8Buffer, RING_BUFFER_MAX_SIZE);	/* 开始循环接收 */
    }

	/* 清除Sem：注，初始化的时候，SEM好像就已经有了，奇怪 */
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
    /* 输入的是huart，需要转成u8Port */
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
	uint16 uMaxLen = UART_TR_BUF_BLEN - g_UartComm[u8Port].uRxFrameIndex;	/* 可以读取的最大字节数 */
	if(u8Port == GPRS_UART_PORT) {	/* 与GPRS通信时才用到环形缓冲区 */
		while(uIniWaitTime_ms--) {
			if((uRxBufPt = RingBuffer_Read(&g_GPRSRingBuffer, pU8Buf, uMaxLen))
			&& (uRxBufPt = StrRemoveHashData(pU8Buf, uRxBufPt)) != 0) {		/* 检查并解析带IRQI的帧 */
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
		HAL_UART_Transmit(g_UartComm[0].Handle, (uint8 *)"【", 2, 300);
		HAL_UART_Transmit(g_UartComm[0].Handle, g_UartComm[u8Port].u8TRBuf, uRxBufPt + g_UartComm[GPRS_UART_PORT].uRxFrameIndex, 300);
		HAL_UART_Transmit(g_UartComm[0].Handle, (uint8 *)"】", 2, 300);
	}
#endif
	return uRxBufPt;
}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	if((HAL_UART_RXEVENT_IDLE == HAL_UARTEx_GetRxEventType(huart)) || (HAL_UART_RXEVENT_TC == HAL_UARTEx_GetRxEventType(huart))) {		/* 当DMA缓冲满时发完，不会触发IDLE，所以两个中断事件都要监听 */
//		if(huart == g_UartComm[GPRS_UART_PORT].Handle) {
//			g_UartComm[GPRS_UART_PORT].uRxBufPt = Size;
//			/* 先启动DMA防止丢失数据 */
//			if(HAL_UARTEx_ReceiveToIdle_DMA(huart, g_GPRS_DoubleBuffer.pU8NextBuf, GPRS_UART_RX_DOUBLE_BUFF_MAX_LEN) == HAL_BUSY) {
//				/* DMA启动失败, 一般不会进来, 但是程序偶尔会有与GPRS通信不上的情况. 怀疑是DMA没启动如果启动失败就再启动一次. */
//				HAL_UARTEx_ReceiveToIdle_DMA(huart, g_GPRS_DoubleBuffer.pU8NextBuf, GPRS_UART_RX_DOUBLE_BUFF_MAX_LEN);
//			}
//			RingBuffer_Write(&g_GPRSRingBuffer, g_GPRS_DoubleBuffer.pU8CurrentBuf, Size);	/* 写入环形缓冲区 */
//			/* 交换指针 */
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
    /* 输入的是huart，需要转成u8Port */
    uint8 u8Port;
    for(u8Port = 0; (u8Port < UART_COMM_NUM) && (g_UartComm[u8Port].Handle != huart); u8Port++) {
    }
	if(u8Port < UART_COMM_NUM) {
		osSemaphoreRelease(g_UartComm[u8Port].Sem_TxCplt);
	}
}

uint16_t ReadFromUart(uint8 u8Port, uint16 uIniWaitTime_ms)
{
    /* 0代表使用默认值: 3.5字符宽度(modbus标准) */
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
		I2C模块
***************************************************************************/
/* I2C写SCL引脚电平 */
void I2C_W_SCL(uint8 u8Port, GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(cnst_I2C_BSP[u8Port].u32SclBank, cnst_I2C_BSP[u8Port].u32SclPin, PinState);
}

/* I2C写SDA引脚电平 */
void I2C_W_SDA(uint8 u8Port, GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(cnst_I2C_BSP[u8Port].u32SdaBank, cnst_I2C_BSP[u8Port].u32SdaPin, PinState);
}

/* I2C读SDA引脚电平 */
uint8_t I2C_R_SDA(uint8 u8Port)
{
	uint8_t BitValue;
	BitValue = (uint8_t)HAL_GPIO_ReadPin(cnst_I2C_BSP[u8Port].u32SdaBank, cnst_I2C_BSP[u8Port].u32SdaPin);
	return BitValue;
}

/* I2C起始 */
void I2C_Start(uint8 u8Port)
{
	I2C_W_SDA(u8Port, GPIO_PIN_SET);
	I2C_W_SCL(u8Port, GPIO_PIN_SET);
	I2C_W_SDA(u8Port, GPIO_PIN_RESET);
	I2C_W_SCL(u8Port, GPIO_PIN_RESET);
}

/* I2C终止 */
void I2C_Stop(uint8 u8Port)
{
	I2C_W_SDA(u8Port, GPIO_PIN_RESET);
	I2C_W_SCL(u8Port, GPIO_PIN_SET);
	I2C_W_SDA(u8Port, GPIO_PIN_SET);
}

/* I2C发送一个字节 */
void I2C_SendByte(uint8 u8Port, uint8_t Byte)
{
	uint8_t i;
	for(i = 0; i < 8; i++) {
		I2C_W_SDA(u8Port, Byte & (0x80 >> i));
		I2C_W_SCL(u8Port, GPIO_PIN_SET);
		I2C_W_SCL(u8Port, GPIO_PIN_RESET);
	}
}

/* I2C接收一个字节 */
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

/* I2C发送应答位 */
void I2C_SendAck(uint8 u8Port, uint8_t AckBit)
{
	I2C_W_SDA(u8Port, AckBit);
	I2C_W_SCL(u8Port, GPIO_PIN_SET);
	I2C_W_SCL(u8Port, GPIO_PIN_RESET);
}

/* I2C接收应答位 */
uint8_t I2C_ReceiveAck(uint8 u8Port)
{
	uint8_t AckBit;
	I2C_W_SDA(u8Port, GPIO_PIN_SET);
	I2C_W_SCL(u8Port, GPIO_PIN_SET);
	AckBit = I2C_R_SDA(u8Port);
	I2C_W_SCL(u8Port, GPIO_PIN_RESET);
	return AckBit;
}

/* I2C写寄存器 */
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

/* I2C读寄存器 */
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
	/* 软件I2C发送函数替换 */
	taskDISABLE_INTERRUPTS();
	I2C_WriteReg(u8Port, u8DevAdd, (uint8*)pUDat, uBLen);
	taskENABLE_INTERRUPTS();
	return TRUE;
}

BOOL ReadFromI2C(uint8 u8Port, uint8 u8DevAdd, uint8* pUDat, uint16 uBLen, uint16 uTimeOut_ms)
{
	/* 软件I2C接收函数替换 */
	taskDISABLE_INTERRUPTS();
	I2C_ReadReg(u8Port, u8DevAdd, pUDat, uBLen);
	taskENABLE_INTERRUPTS();
	return TRUE;
}
/***************************************************************************
		SPI模块
***************************************************************************/
#if SPI_COMM_NUM
void OpenSPIComm(uint8 u8Port)
{
	SPI_COMM *pSPIComm = &g_SPIComm[u8Port];
	/* 创建信号量 */
	osSemaphoreDef(SPIComm0_RxGet);
	pSPIComm->Sem_RxGet = osSemaphoreCreate(osSemaphore(SPIComm0_RxGet),1);
	osSemaphoreWait(pSPIComm->Sem_RxGet, 0);
	osSemaphoreDef(SPIComm0_TxGet);
	pSPIComm->Sem_TxCplt = osSemaphoreCreate(osSemaphore(SPIComm0_TxGet),1);
	osSemaphoreWait(pSPIComm->Sem_TxCplt, 0);
}
/*==========================================================================
| Description	: spi收发
| Author		: Cui				Date	: 2024-01-27
\=========================================================================*/
BOOL WriteToSPI(uint8 u8Port, const uint8 *pU8, uint16 uBLen, uint16 uTimeOut_ms)
{
	HAL_SPI_Transmit_IT(g_SPIComm[u8Port].Handle, (uint8*)pU8, uBLen);
	if(uTimeOut_ms == 0) {		/* 0为根据待发送长度计算 */
		/* 每次通讯位数：uBLen*8；在这个基础上，每B多考虑1b余量；以及除法余数1ms、定时精度1ms */
		uTimeOut_ms = (uBLen*9)/(HAL_RCC_GetPCLK2Freq()/g_SPIComm[u8Port].Handle->Init.BaudRatePrescaler) + 2;
	}
	return Semaphore_pend(g_SPIComm[u8Port].Sem_TxCplt, uTimeOut_ms);
}

BOOL ReadFromSPI(uint8 u8Port, uint8 *pU8, uint16 uBLen, uint16 uTimeOut_ms)
{
	HAL_SPI_Receive_IT(g_SPIComm[u8Port].Handle, pU8, uBLen);
	if(uTimeOut_ms == 0) {		/* 0为根据待发送长度计算 */
		/* 每次通讯位数：uBLen*8；在这个基础上，每B多考虑1b余量；以及除法余数1ms、定时精度1ms */
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
		TF卡存储
***************************************************************************/
void InitSDSPI(void)
{
	MX_FATFS_Init();
}
#endif

#if SUPPORT_NET
/***************************************************************************
		网络
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
   can模块
***************************************************************************/
/*Can FiFo有数据中断，注意can1和can2公用。如果想分开需要在自带库函数改一下*/
#include "MdlCan.h"
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* 获取中断号，传输的地址、数据、长度 */
	uint32 u32CanDat[2] = {0, 0};	/* 因为传输可能非8byte，因此需要预先置零 */
	CAN_RxHeaderTypeDef RxHeader;
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, (uint8*)u32CanDat) != HAL_OK) {
		return ;
	}
    Isr_CanRx(hcan, RxHeader.ExtId, RxHeader.FilterMatchIndex, u32CanDat, RxHeader.DLC);
}
/*Can FiFo满中断*/
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
	//GPIO_toggle(GPIO_INDICATOR_StartNo + 5);
}
/*Can发送mailbox0结束中断*/
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	Isr_CanTx(hcan);
	//GPIO_toggle(GPIO_INDICATOR_StartNo + 5);
}
/*Can发送mailbox1结束中断*/
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	Isr_CanTx(hcan);
	//GPIO_toggle(GPIO_INDICATOR_StartNo + 5);
}
/*Can发送mailbox2结束中断*/
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	Isr_CanTx(hcan);
	//GPIO_toggle(GPIO_INDICATOR_StartNo + 5);
}

/* 初始化can过滤器 */
void InitCanFilter(void* hCan)
{
	CAN_HandleTypeDef* pCan = hCan;
	SET_BIT(pCan->Instance->FMR, CAN_FMR_FINIT);
	WRITE_REG(pCan->Instance->FS1R,0x0FFFFFFF);
	CLEAR_BIT(pCan->Instance->FMR, CAN_FMR_FINIT);
}

/* 配置can过滤器 */
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

/* 通过can总线发送数据 */
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
