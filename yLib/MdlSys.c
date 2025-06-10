/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved
 *
 * File			: main.c
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
 						macro definition
***************************************************************************/
#define _MDL_SYS_C_		/* exclude redefinition */

/***************************************************************************
 						include files
***************************************************************************/
/* 操作系统头文件 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

/* 项目公共头文件 */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"

/* 本级以及下级模块头文件 */
#include "MdlUARTnModbus.h"
#include "MdlDataAccess.h"
#include "MdlNet.h"
#include "BoardSupport.h"
#ifdef CAN_BASE
#include "MdlCan.h"
#endif
#if (DEVICE_TYPE == V5_YBT3)
#include "MdlW5500.h"
#endif


/***************************************************************************
 						global variables definition
***************************************************************************/

/***************************************************************************
						internal functions declaration
***************************************************************************/
void InitSysMdl(void);
void ChkSoftIntegrity(void);
void DiscJtagAndStartWDT(void);
void KickWatchDog(void);
void InitDebugDat(void);
void DrvSoftUpdateTick_1KHz(void);

/***************************************************************************
 							functions definition
***************************************************************************/
/*==========================================================================
| Description	: 复位函数，在初始化PLL后执行
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2017-04-18
\=========================================================================*/
void ResetFunc(void)
{
	/* SRAM分区ZERO_INI初始化，该地址、长度必须和.cmd/ld文件中所分配的ZERO_INI地址、长度对应 */
	InitDataWithZero((uint8*)0x20000000, 0x00018000);
}

/*==========================================================================
| Description	: 模块初始化
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2008-3-4
\=========================================================================*/
void InitSysMdl(void)
{
	/* 初始化配置 */
	ChkAndWaitInitConf(SAVE_GRP_MCONF);	/* 等待配置完成 */

    /* 检查软件完整性 */
    /* 计算结果放在g_PubSoftAuthCtr.u32FlashHash仅仅是为了方便读取，运行时间约132ms
       尽管这里就已经计算出来了，但不立刻做检查，过半个小时做判断     */
#if CPU_0ST_1TI == 1
    uint32 u32Key[2];
    GetAuthKey(AUTH_KEY_FLASH_INTEGRITY, u32Key);
	#if LEGACY
	uint32 u32FlashCrc = CalCRC32ForFileByIni((uint8*)BOOT_LOADER_BLEN + sizeof(BOOT_LOADER_CONF),
                                                FLASH_HALF_BLEN - BOOT_LOADER_BLEN - sizeof(BOOT_LOADER_CONF), 
                                                u32Key[0] ^ u32Key[1]);
	#else
    uint32 u32FlashCrc = CalCRC32ForFileByIni((uint8*)FLASHADD_PROG_START, FLASH_PROG_BLEN, u32Key[0] ^ u32Key[1]);
	#endif
    g_Sys.u32FlashHash_OrigEEAddr = u32FlashCrc ^ u32Key[0];
    g_Sys.u32FlashHash_BackEEAddr = u32FlashCrc ^ u32Key[1];
#elif CPU_0ST_1TI == 0
//    uint32 u32Key[2];
//	GetAuthKey(AUTH_KEY_FLASH_INTEGRITY, u32Key);
//	uint32 u32FlashCrc = CalCRC32ForFileByIni((uint8*)FLASHADD_IVT_START, FLASH_IVT_BLEN, u32Key[0] ^ u32Key[1]);		/* 中断向量表所属sector */
//    u32FlashCrc = CalCRC32ForFileByIni((uint8*)FLASHADD_PROG_START, FLASH_PROG_BLEN, u32Key[0] ^ u32Key[1]);			/* 去除中断向量表、模拟EEPROM区所属sector */
//	g_Sys.u32FlashHash_OrigEEAddr = u32FlashCrc ^ u32Key[0];
//	g_Sys.u32FlashHash_BackEEAddr = u32FlashCrc ^ u32Key[1];
#endif
	/* 初始化GlobalVar.h里面的变量 */
	/* <NULL> */

	/* 初始化输出接口变量 */
    g_Sys.tReboot_0Null_nUrgent_pWaitIdle = 0;

	/* 初始化内部全局变量 */
	/* <NULL> */

	/* 初始化下层模块 */
	/* 启动硬件 */
}

/*==========================================================================
| Description	: 系统工作
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2009-5-31
\=========================================================================*/
void SysTask(void const * pArgs)
{
	volatile int32 i;

	InitSysMdl();
//	if(((GetVerVar(GetVolatileFromU32Addr(&g_u32SoftVer))) == 9000)
//			&& ((GetVerVar(GetVolatileFromU32Addr(GetCounterpartAddr(&g_u32SoftVer)))) == 9000))
//	{
		g_PubSoftAuthCtr.uPubSoftUpdateCnt = 1000;		/* TODO：临时成为母版：升级运行软件计数器 */
		g_PubSoftAuthCtr.uPubSoftInstallCnt = 1000;		/* TODO：临时成为母版：从测试软件到运行软件计数器 */
		g_PubSoftAuthCtr.uPubSnCnt = 1000;				/* TODO：临时成为母版：发布SN计数器 */
		g_PubSoftAuthCtr.uPubLicCnt = 1000;				/* TODO：临时成为母版：发布License计数器 */
//	}
	g_Sys.SerialNo.u32Dat = 11111111;
	for(;;) {
    	ChkSoftIntegrity();
		if(g_DataAcsIntf.bConfHaveUnsavedChange[SAVE_GRP_NUL]) {	/* 调试的时候，手动启动参数存储 */
			g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt = 1;
			Semaphore_post(g_DataAccessBlock.Sem_Op);		//Semaphore_post(SEM_NvMemAcs);
		}
		Task_sleep(OS_TICK_KHz*100);
		/* 测试 */
		if(g_CodeTest.uCodeTestSW) {
			uint32 HwiKey = Hwi_disable();
			
			RunTime_StartCali(0);
			switch(g_CodeTest.uCodeTestSW) {
				case 1:
					break;
				case 2:
					break;
				case 3:
					break;
				case 4:
					break;
				case 5:
					break;
				case 6:
					break;
				case 7:
					break;
				case 8:
					break;
				case 99:
					for(i = 0; i < CODE_TEST_UINT16_NUM; i++) {
						g_CodeTest.uVal[i] = 0;
						g_CodeTest.uVal2[i] = 0;
						g_CodeTest.uVal3[i] = 0;
					}
					for(i = 0; i < CODE_TEST_UINT8_NUM; i++) {
						g_CodeTest.u8Val[i] = 0;
					}
					for(i = 0; i < CODE_TEST_UINT32_NUM; i++) {
						g_CodeTest.u32Val[i] = 0;
						g_CodeTest.i32Val[i] = 0;
					}
					break;
				default:
					break;
			}
			RunTime_EndCali(0);
			
			Hwi_restore(HwiKey);
			g_CodeTest.uCodeTestSW = 0;
		}
	}
}

/*==========================================================================
| Description	: 主控制任务
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2009-5-31
\=========================================================================*/
void CtrTask(const void* argument)
{
	DiscJtagAndStartWDT();
	InitMdlCtr();
	g_Sys.bCtrMdlOK = TRUE;

	for(;;) {
		KickWatchDog();
		Load_update();
    #if (!REMOTE_DEBUG_TERMINAL)
		g_Sys.u8CpuLoad = Load_getCPULoad();
    #endif

		RunMdlCtr();
		Semaphore_pend(g_SysBlock.SEM_Ctr, OS_TICK_KHz*10);			//Semaphore_pend(SEM_CtrTskRun, OS_TICK_KHz*40);

#if 0
		if(g_CodeTest.u32Val[91]) {
			g_CodeTest.u32Val[93] = 0xCC;
			/* 拷贝第一个16K的sector（0x08000000）到下半区的16Ksector（0x08080000） */
			FlashErase2(0x08080000, 16*1204);
			g_CodeTest.u32Val[93] = FlashProgram2((uint32*)0x08000000, 0x08080000, 16*1024, TRUE);
			g_CodeTest.u32Val[91] = 0;
		}
		if(g_CodeTest.u32Val[92]) {
			FlashErase2(0x08004000, 16*1024);
			g_CodeTest.u32Val[92] = 0;
		}
		if(g_CodeTest.u32Val[90]) {
			SwapFlashBank();
			g_CodeTest.u32Val[90] = 0;
		}
		if(g_CodeTest.u32Val[9]) {
			FlashErase2(0x08088000, 512*1024 - 32*1024);
			g_CodeTest.u32Val[9] = 0;
		}
		if(g_CodeTest.u32Val[99]) {
			g_CodeTest.u32Val[98] = g_CodeTest.u32Val[97] = g_CodeTest.u32Val[94] = 0xCC;
//			FLASH_AdvOBProgramInitTypeDef AdvOBInit;
//			AdvOBInit.OptionType = OPTIONBYTE_BOOTCONFIG;
//			HAL_FLASHEx_AdvOBGetConfig(&AdvOBInit);
			FLASH_WaitForLastOperation((uint32_t)50);
			HAL_FLASH_OB_Unlock();
			g_CodeTest.u32Val[98] = FLASH->OPTCR;
			g_CodeTest.u32Val[97] = FLASH->OPTCR & FLASH_OPTCR_BFB2_Msk;
			g_CodeTest.u32Val[94] = FLASH->OPTCR & FLASH_OPTCR_DB1M;
			HAL_FLASH_OB_Launch();
			HAL_FLASH_OB_Lock();
			g_CodeTest.u32Val[99] = 0;

		}
		g_CodeTest.u32Val[95] = READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_UFB_MODE);
#endif
	}
}

/*==========================================================================
| Description	: 主定时器
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-2-1
\=========================================================================*/
void vApplicationTickHook(void)		/* 原来TI void DriveTick_1KHz(void) */
{
	static uint8 s_u8Tmr_Ctr_ms = 0;

	/* 没有必须初始化的变量 */
#if SUPPORT_NET
	DrvNetTick_1KHz();
#endif
	DrvSoftUpdateTick_1KHz();
#if (DEVICE_TYPE == V5_YBT3)
	DrvW5500Tick_1KHz();
#endif

	/* 必须等待模块初始化完成 */
	if(g_Sys.bCtrMdlOK) {
		DrvMdlCtrTick_1KHz();
    #if SUPPORT_SAMPLE
		DrvSampleTick_1KHz();
    #endif
    #if (MAX_RELAY_NUM && SOFT_RUN1_TEST0 && (!REMOTE_DEBUG_TERMINAL))
		static uint8 s_u8Tmr_DOut100Hz_ms = 0;
        if(s_u8Tmr_DOut100Hz_ms) {
            s_u8Tmr_DOut100Hz_ms--;
        } else {
            s_u8Tmr_DOut100Hz_ms = 9;
            //DriveDOutTick_100Hz();
        }
    #endif
	}

	/* 模块初始化在main()中完成，因此无需特殊标志 */
	DrvUartTick_1KHz();

	if(s_u8Tmr_Ctr_ms) {
		s_u8Tmr_Ctr_ms--;
	} else {
		s_u8Tmr_Ctr_ms = PERIOD_CTR_TASK_ms - 1;
		Semaphore_post(g_SysBlock.SEM_Ctr);		//Semaphore_post(SEM_CtrTskRun);
	}
	
#if CPU_0ST_1TI == 0
	/* 由1Hz计时器更新运行时间 */
	if(g_Sys.uTmr_1Hz == 0) {
		g_Sys.u32UpTimer_s++;
		g_Sys.u32RstTimer_s++;
		g_Sys.uTmr_1Hz = 1000 - 1;		/* 0 to 999 == 1000*/
	} else {
		g_Sys.uTmr_1Hz--;
	}
#if SUPPORT_FATFS
	extern void DrvSdspiTick_1Hz(void);
	DrvSdspiTick_1Hz();
#endif
#endif
}

/* 检查软件完整性: 检查sn所绑定芯片ID, 检查软件是否被更改 */
void ChkSoftIntegrity(void)
{
	Swi_disable();
    /* 延时 */
    if(g_Sys.iTimer_nSoftChkWaitTick_pErrCode < 0) {
        g_Sys.iTimer_nSoftChkWaitTick_pErrCode++;
        if(g_Sys.iTimer_nSoftChkWaitTick_pErrCode == 0) {
            /* 仅检查一次 */
            if((g_Sys.u32FlashHash_OrigEEAddr != cnst_BootLoaderConf.AcsMedia[0].u32OrigEEAddr)
                || (g_Sys.u32FlashHash_BackEEAddr != cnst_BootLoaderConf.AcsMedia[0].u32BackEEAddr))
            {
                g_Sys.iTimer_nSoftChkWaitTick_pErrCode |= 4;    /* b2代表flash校验出错 */
            }
        }
    }
    if(g_Sys.iTimer_nSoftChkWaitTick_pErrCode >= 0) {
        if(g_Sys.SerialNo.u32Dat < 10000000UL) {
            g_Sys.iTimer_nSoftChkWaitTick_pErrCode |= 1;    /* 标志b0 */
        } else {
            g_Sys.iTimer_nSoftChkWaitTick_pErrCode &= 6;    /* 清除b0，因为只用了3位，因此这里可以仅用6 */
        }
        if(g_Sys.SerialNo.u32ChipIdHash != GetChipIDHash()) {
            g_Sys.iTimer_nSoftChkWaitTick_pErrCode |= 2;    /* 标志b1 */
        } else {
            g_Sys.iTimer_nSoftChkWaitTick_pErrCode &= 5;    /* 清除b1，因为只用了3位，因此这里可以仅用5 */
        }
    }
	Swi_enable();
}

/*==========================================================================
* 软件授权、升级管理模块, 包括: JTAG与看门狗，软件发布，软件授权三个内容

  测试版与运行版
  	1. 测试版: 对产品硬件进行测试，如DI/DO, AD等，通过JTAG口下载测试软件
  	2. 运行版: 真正实现产品功能的软件, 由测试软件通过升级获取

  发布版本、调试版本与JTAG管理
  	1. 调试版本是指开发阶段，需要接连JTAG口进行调试；
  	2. 发布版本即软件可以被发布，为了防止被读出，需要断开JTAG
	3. JTAG和看门狗连接在一起，启动看门狗即断开JTAG, 喂狗信号必须确认断开JTAG
	   由于调试版本不断开JTAG，也就不会喂狗，插上相应跳线会观察到外部看门狗会不断服务CPU，以此作为标志

  软件授权，有两个角色需要参与:
  	1. 操作员，板子需要和操作员处于同一公网IP下，服务器依据操作员的IP进行判断，并记录授权操作
  	2. 管理员, 给出操作员合法IP，操作时间窗口, 软件份数
  	服务器需要记录操作与授权的全部操作

  软件下载与授权工作场景流程:
	1. 北京办公室下载测试软件阶段，完成: 芯片擦除、下载测试软件，这个过程就配置成不同的产品--产品型号是常数定义。
	2. 运行测试软件后，自动断开JTAG, 测试网口的时候，就向服务器请求序列号--需要操作员
	3. PCB板子发往江西工厂，进行高温老化，老化后在实验台上继续测试，如果异常返回北京，否则储存。
	4. 在板子需要使用的时候，进行首次导入运行软件--需要操作员
	   运行软件由存储软件的母板经过网络加密传输至目标板，服务器仅转发并不存储运行软件。
	5. 对运行软件进行授权，可以配置成试用版(有限期限)或正式版, --需要操作员
  	   对于试用版的使用超期判断，YKF3/YBT3依靠GPS时钟，YBU2依靠服务器时间
  	6. 对于获得授权的软件(即已经过配置)，运行软件的版本升级无需操作员干预，上网就可以自动升级

  安全保障点:
  	1. 运行软件在网络上传输处于加密状态，且一次传输一个密码，截获软件不可解密
  	2. 区分三个服务器: 数据服务器、权限服务器、口令服务器
  		数据服务器: 域名可以配置，控制器日常转发数据的服务器，更新软件也由该服务器进行转发
  		权限服务器: 控制器进行授权(测试版获取序列号，运行版进行期限授权)所登陆的服务器，每个版本软件有一个固定且不可修改的服务器
  					母板发布软件也是经过权限服务器，转发给数据服务器再给控制器
  					一旦某个版本的软件被泄密，停用该版本软件的授权域名，即可以防止该版本被进一步复制
  		口令服务器: 用母板来做，升级软件、进行授权都需要口令。升级软件是控制器向母板提供口令，进行授权是母板向控制器提供口令
		  			口令错误则母板延迟响应外部请求，且向服务器报备，以防止暴力破解
  	   即，
  	   	a. 就算知道加密机制，也不可能模拟待升级控制器，骗取软件
  	   	b. 就算获得了软件，未经授权也无法使用，必须要登陆权限服务器进行授权
  	3. 口令、权限服务器域名在代码中不得显示存储，即不能直接用一个立即数、字符串构成
  	   需要在运行中产生，由函数计算产生。这样，就算破解了加密，也得不到口令、不可能通过改变授权服务器的形式进行授权。
  	   口令生成:
	4. 每一个代码片独立加密，请求地址随机

  软件在线升级(无需人工干预)，包括如下部分:
	1. 加密与解密
		a. RSA用于传递密钥
		b. ASE用于代码段
	2. 软件分片传输: 每个代码片(1KB)分别加密，为了减少母板TCP/IP通讯资源占用，所有通讯都经过服务器转发
	   母板不关注具体控制器，升级单元是代码片，所有授权均基于此；
	   同时多少个控制器在升级由服务器管理; 同时多少个代码片在升级由母板管理。
		a. 单个代码片:
			a1. 母板利用RSA产生N个密钥对，公钥发送给服务器以供待升级控制器订阅，每个RSA密钥对应一个代码片
			a2. 要升级的控制器订阅RSA公钥后，利用RSA加密自身的AES密码、利用AES密码加密升级的请求地址、当前版本、口令
				连同所订阅端RSA密码序号一起发布给母板
			a3. 母板利用RSA私钥获得AES通讯密码与地址，对相应的代码片进行AES加密，发送给服务器，本个代码片发送结束
		b. 控制器查询获得最新版本号与升级许可，需要获得总的升级许可，即可以订阅用于升级的RSA密码对
		c. 安全保障点:
			c1. 母板发布RSA公钥、接收RSA加密结果，只能通过公司服务器(即服务器地址不可配置)。注:控制器无需如此,就用MQTT服务器即可
			c2. 每一个代码片的请求包要有请求地址、当前版本、口令，请求地址随机，口令是时间的函数(YBU没有时间就固定)，每个版本都不一样
				如果口令错误，则母板延迟响应
	3. flash擦除与分段写入: 利用flash mirror功能，Low Region用于运行，High Region用于升级
		a. 擦除高端地址
		b. 订阅RSA公钥，成功后产生: AES密码、并用AES加密当前带升级代码片地址，当前版本，口令
	4. 当前运行版本选择，启动软件运行
		a. 检查当前软件完整性
		b. 检查
	5. 配置数据在不同版本间转换
		a. 所有配置数据都有ID,

  请求序列号操作
  1. 新控制器(无序列号)以临时序列号(低六位随机数，高两位为0)登陆通讯服务器，发布“序列号请求”后，web服务器通过订阅收到该请求。
  2. 操作员通过固定的管理页面用登陆web服务器，根据板子不同--老板子恢复序列号，新板子自动生成序列号
  	 操作员所有的操作(序列号、授权)都要被记录，必要的时候可以关闭权限
  3. web服务器核对新控制器的IP、操作员IP，一致后，接收操作员输入的序列号(或者产生序列号)以及临时序列号，转发给母板
  4. 母板生成加密的“序列号”，发布给临时序列号所对应的主题，新控制器收获该数据后，更新自身的序列号
  
  请求授权操作：
  1. 在运行控制器(有序列号，但有限期限授权)登陆通讯服务器，就提交了授权情况
  2. 操作员通过固定的管理页面用登陆web服务器，产生授权信息(有限期限就是到期时间，或者无限期限)
  	 操作员所有的操作(序列号、授权)都要被记录，必要的时候可以关闭权限
  3. web服务器把序列号、授权信息发送给母板
  4. 母板对序列号、授权信息加密后发生给对应的控制器，控制器接收授权包后，更新自身的授权信息
  	 如果，该台控制器并未联网，则由web服务器订阅该台服务器的授权主题，并把相应的数据做成文件，提交给客户。
  	 客户可以通过文件下载、屏幕输入的办法，把授权信息输入到控制器中

  软件升级，服务器相关操作：
	1. 有新版本发布的时候，注册和新版本相关的权限服务器域名，母板连接权限服务器，权限服务器通知数据服务器升级准备好。
	2. 数据服务器逐一检查控制器的版本，并按照升级策略通知控制器进行升级(有新版本的时候，需要逐步升级)。
	3. 数据服务器在收到控制器“代码片请求码”的时候，转发给权限服务器，权限服务器再转发给母板
	4. 母板收到代码片请求码的时候，生成相应的代码片经过权限服务器-数据服务器转发给控制器

  设备端访问接口包括:
	1. ProcMqttUpdate() : Mqtt升级总接口
	2. IniSoftUpdate()	: 启动升级。初始化升级逻辑，擦除flash
	3. QueryAndPubFlashReq(): 检查升级。发布请求码(包括地址、当前版本、口令，加密)。
	4. UpdateSoft()	 	: 软件升级操作。解密代码片，写入数据
	5. CreateFlashReq()	: 产生代码片请求

  母板端访问接口包括：
  	1. ProcMqttAuth_SN()	: 发布序列号，根据web服务器提交的临时序列号与正式序列号，加密后生成相应的序列号更新码
	2. ProcMqttAuth_LIC()	: 发布软件授权码，根据web服务器提交的序列号与期限，加密后生成相应的授权码
	3. ProcMqttAuth_SOFT() 	: 发布软件：根据提交的请求码(包括地址、当前版本、口令，加密)产生加密的代码片

  通讯软件(MQTT)接口调用逻辑：
	1. 连接服务器的时候，上报本机设备信息(型号、版本号、序列号、授权等)
	2. 连接服务器成功后，运行ChkLicense(), 如果License需要授权，会产生“授权请求码”，将“授权请求码”发生给服务器；
	   服务器回复授权码，调用AuthLicense()处理授权
	3. 运行中，服务器根据第1步获取的机组版本号确定是否要升级。
	   如果要升级，则发出升级通知；控制器端得到升级通知，调用IniSoftUpdate()，生成代码片请求码。
	   服务器回复代码片后，调用UpdateSoft()进行软件升级

  母板通讯端软件相关操作：
	1. 收到权限服务器转发的授权请求，调用PubAuthLic()，生成授权码
	2. 收到权限服务器转发的代码片请求，调用PubSoftware()，生成代码片

  设备端升级其他函数
	1. BootLoader()		: 位于flash开头，一般不参与升级。上电的时候，由此入口，判断该使用哪个代码片。
						  判断依据：首先检查软件完整性标志，如果两份软件都是完整的，那就选择高版本的运行。
						  软件完整性标志：在启动升级的时候破坏，在升级完成检查没问他后打上。
	2. ChkLicense()		: 检查授权

  升级本质是一个复制母版flash中代码到目标板子的过程，为了安全考虑，这个复制过程的起始地址是随机的，好比可以从时钟的任意时间开始复制，12小时后又能回到起点，即全部复制完成。

  BootLoader算法与设备mem分配:

  TI:
  flash: 0x100000Byte, 最小擦除单元0x4000, 代码片长度0x400(1KB)，因此一半flash总共32个擦除单元，512个代码片
  eeprom:0x1800Byte,
  SRAM:  0x40000Byte，从前往后分配为:ZeroIniRam, RawRam, SysStack
  为了保证BootLoader最小更改，因此如此约定：
  1. 第0个flash最小擦除单元用于装载BootLoader，正常程序在第1个flash最小擦除单元之后
     更新软件仅更新第1个flash最小擦除单元及以后的一半flash空间，
     低一半flash第0个最小擦除单元在下载测试软件的时候完成编程
     高一半flash第0个最小擦除单元在运行测试软件的时候完成编程
  2. 上电复位向量指向BootLoader, 完成判断后跳转至_c_int00，_c_int00位置固定在第1个flash最小擦除单元开始位置
  3. 由于堆栈增长的时候，指针是减少的，因此堆栈指针指向SRAM最后

  ST:
  	A, F427:
  flash: 0x100000Byte, 最小擦除单元不固定，需根据地址来查所属sector的尺寸[16K/16K/16K/16K/64K/128K/128K/128K/128K/128K/128K/128K]
  eeprom:从flash中模拟的，尺寸为0x4000Byte（16K）,
  RAM:  0x10000000开头的CCMRAM，总长度64K，从前往后分配为:CCMRAM_TEXT, CCMRAM_DATA， (TODO:还未指定变量、函数)
  	    0x20000000开头的SRAM，总长度192K，从前往后分配为:ZeroIniRam, RawRam, SysStack
  flash结构如下：
  1. ST暂时没用自定义bootloader
  2. 第一个sector(16K)为中断向量表(IVT)
  2. 第二个sector(16K)为模拟EEPROM。
  3. 再之后的所有空间即为可升级部分。

	B, F103:
 flash: 0x100000Byte, 最小擦除单元固定為2KB
  eeprom:从flash中模拟的，尺寸为0x4000Byte（2K x 3）,
  	    0x20000000开头的SRAM，总长度64K，从前往后分配为:ZeroIniRam, RawRam, SysStack
  flash结构如下：
  2. 第1个sector(2K)为中断向量表(IVT)和bootloader
  2. 第2-4个sector(6K)为模拟EEPROM。
  3. 运行版本号，32位，两个16位互补方式实现校验 和 版本号，32位，两个16位互补方式实现校验
  4. 再之后的所有空间即为可升级部分。


  由于堆栈增长的时候，指针是减少的，因此堆栈指针指向SRAM最后
\=========================================================================*/


/*==========================================================================
| Description	: 软件升级所需定时器
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2015-9-27
\=========================================================================*/
void DrvSoftUpdateTick_1KHz(void)
{
	if(g_SoftUpdateCtr.uTmr_ReqWait_ms) {
		g_SoftUpdateCtr.uTmr_ReqWait_ms--;
	}
	if(g_PubSoftAuthCtr.uTmr_IniPubCntNoPwd_ms) {
		g_PubSoftAuthCtr.uTmr_IniPubCntNoPwd_ms--;
	}
	if(g_Sys.uTmr_EraseFlash_DistPwr_ms) {
		g_Sys.uTmr_EraseFlash_DistPwr_ms--;
	}
}

BOOL ProcMqttUpdate(MQTT_COMM* pMqttComm, uint8* pU8Topic, uint8* pU8TopicEnd, uint8* pU8Msg, uint8* pU8MsgEnd)
{
	uint8* pU8SubMsg;
	
	if(CompareTopic(pU8Topic, "SN")) {
		SERIAL_PACK SerialPack;
		pU8SubMsg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 3);
		if(GetHexArray(pU8SubMsg, pU8MsgEnd, (uint8*)(&SerialPack), sizeof(SERIAL_PACK))) {
			DecryptWithAES(AUTH_AES_PACK_SERIAL, (uint32*)(&SerialPack), sizeof(SERIAL_PACK), 0);
			if(SerialPack.u32IniSerialNo == g_Sys.SerialNo.u32Dat) {
				g_Sys.SerialNo.u32Dat = SerialPack.u32NewSerialNo;
				g_Sys.SerialNo.u32ChipIdHash = GetChipIDHash();
				g_Sys.u32License_Seconds = 0;		/* 序列号变化，需要重新授权 */
				g_DataAcsIntf.bConfHaveUnsavedChange[SAVE_GRP_BRD] = TRUE;
				Semaphore_post(g_DataAccessBlock.Sem_Op);		//Semaphore_post(SEM_NvMemAcs);
			}
		}
	} else if(CompareTopic(pU8Topic, "LICENSE")) {
		LICENSE_PACK LicensePack;
		pU8SubMsg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 3);
		if(GetHexArray(pU8SubMsg, pU8MsgEnd, (uint8*)(&LicensePack), sizeof(LICENSE_PACK))) {
			DecryptWithAES(AUTH_AES_PACK_LICENSE, (uint32*)(&LicensePack), sizeof(LICENSE_PACK), 0);
			if(LicensePack.u32DevSerialNo == g_Sys.SerialNo.u32Dat) {
				g_Sys.u32License_Seconds = LicensePack.u32License_Seconds;
				g_DataAcsIntf.bConfHaveUnsavedChange[SAVE_GRP_BRD] = TRUE;
				Semaphore_post(g_DataAccessBlock.Sem_Op);		//Semaphore_post(SEM_NvMemAcs);
			}
		}
	} else if(CompareTopic(pU8Topic, "RUN")) {
		pU8SubMsg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 1);
		if(CompareMsg(pU8SubMsg, "version")) {
			pU8SubMsg = SkipCharInString(pU8SubMsg, pU8MsgEnd, '"', 2);
			uint32 u32RunVersion = (pU8SubMsg[0]-'0')*1000UL + (pU8SubMsg[2]-'0')*100UL + (pU8SubMsg[4]-'0')*10UL + (pU8SubMsg[5]-'0')*1UL;
			/* 标记是软件更新所引起的重启 */
			SoftRollBack(u32RunVersion);
		}
	} else if(CompareTopic(pU8Topic, "REBOOT")) {
		pU8SubMsg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 1);
		if(CompareMsg(pU8SubMsg, "urgent") && CompareMsg(SkipCharInString(pU8SubMsg, pU8MsgEnd, '"', 2), "true")) {
			g_Sys.tReboot_0Null_nUrgent_pWaitIdle = -1;
		} else {
			g_Sys.tReboot_0Null_nUrgent_pWaitIdle = 1;
		}
	} else if(CompareTopic(pU8Topic, "INI")) {
		if(g_Sys.SerialNo.u32Dat >= 10000000UL) {		/* 配置了序列号才能升级 */
			pU8SubMsg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 1);
			if(CompareMsg(pU8SubMsg, "update_ver")) {
				pU8SubMsg = SkipCharInString(pU8SubMsg, pU8MsgEnd, '"', 2);
				uint16 uReqSoftVer = (pU8SubMsg[0]-'0')*1000 + (pU8SubMsg[2]-'0')*100 + (pU8SubMsg[4]-'0')*10 + (pU8SubMsg[5]-'0')*1;	/* 升级目标软件版本号 */
				if(IniSoftUpdate(uReqSoftVer) && (!QueryAndPubFlashReq(pMqttComm, FALSE))) {
					return FALSE;
				}
			}
		}
	/* base64编解码方式 */
	} else if(g_SoftUpdateCtr.uReqSoftVer && (ReadU32(&pU8Topic, pU8TopicEnd) == g_SoftUpdateCtr.uReqSoftVer)) {
		pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 3);
		uint32 u32FlashPackBLen = (SkipCharInString(pU8Msg, pU8MsgEnd, '"', 1) - pU8Msg - 1)*3/4;
		u32FlashPackBLen = DecodeByBase64(pU8Msg, pU8MsgEnd - pU8Msg, pU8Msg, u32FlashPackBLen) - pU8Msg;
		if(UpdateSoft((uint32*)pU8Msg, u32FlashPackBLen)
			&& (!QueryAndPubFlashReq(pMqttComm, FALSE)))
		{
			return FALSE;
		}
	}
	return TRUE;
}

/*==========================================================================
| Description	: 生成升级请求
| G/Out var 	:
| Author		: King			Date	: 2017-11-4
\=========================================================================*/
BOOL CreateFlashReq(FLASH_REQ* pFlashReq, uint32 u32MaxFlashPackBLen)
{
	/* 看请求的代码片是否会超长 */
	uint32 u32FlashFragBLen = 0;
	if(u32MaxFlashPackBLen > 16) {
		u32FlashFragBLen = u32MaxFlashPackBLen - 16;
	}
	/* 计算剩余地址 */
	uint32 u32RemainFlashBLen;
	if(g_SoftUpdateCtr.u32FlashEndAddr <= g_SoftUpdateCtr.u32FlashCurAddr) {
	#if LEGACY 
		u32RemainFlashBLen = FLASH_HALF_BLEN - g_SoftUpdateCtr.u32FlashCurAddr;
	#else
		u32RemainFlashBLen = FLASH_UPDATEABLE_BLEN - (g_SoftUpdateCtr.u32FlashCurAddr - FLASHADD_UPGRADABLE_START);		/* 此时u32RemainFlashBLen的意思是距离为flash物理边界的长度 */
	#endif
	} else {
		u32RemainFlashBLen = g_SoftUpdateCtr.u32FlashEndAddr - g_SoftUpdateCtr.u32FlashCurAddr;			/* 此时u32RemainFlashBLen的意思是整个程序的剩余长度 */
	}
	if(u32RemainFlashBLen < u32FlashFragBLen)	{
		u32FlashFragBLen = u32RemainFlashBLen;		/* 如果发现剩余距离flash物理边界的长度小于请求片段长度，则此次请求使用这个距离边界的长度作为请求长度 */
	}
	u32FlashFragBLen = (u32FlashFragBLen/16)*16;	/* 代码片长度规格化为128bit */

	/* 生成FlashReq */
	pFlashReq->u32FlashFragBLen = u32FlashFragBLen;
	pFlashReq->bWhichHalf_0Fst_1Sec = !CheckExeFlash_0Low_1High();		/* 请求当前程序的另一个版本，注：即便双bank切换方式在写入时也需要绝对地址 */
	pFlashReq->u8Rand = (uint8)rand();
	pFlashReq->bSoft_Run1_Test0 = SOFT_RUN1_TEST0;
	pFlashReq->bIniSoftUpdate = (g_SoftUpdateCtr.u32FlashCurAddr == g_SoftUpdateCtr.u32FlashEndAddr);
	GetAuthKey(AUTH_KEY_FLASH_REQ_KEY, &pFlashReq->u32FlashReqKey);
	pFlashReq->u32FlashAddr = g_SoftUpdateCtr.u32FlashCurAddr;
	EncryptWithAES(AUTH_AES_FLASH_REQ, (uint32*)pFlashReq, FLASH_REQ_ENCRYPT_LEN, 0);	/* 加密代码请求 */

	/* 计算代码片序号与总的代码片数量 */
	if(g_SoftUpdateCtr.u32FlashCurAddr >= g_SoftUpdateCtr.u32FlashEndAddr) {		/* 如果当前地址还在最终地址后面 */
		pFlashReq->uFlashFragNo = (g_SoftUpdateCtr.u32FlashCurAddr - g_SoftUpdateCtr.u32FlashEndAddr) / FLASH_FRAG_DEFAULT_BLEN;
	} else {		/* 如果当前地址已经绕到最终地址前面 */
	#if LEGACY
		pFlashReq->uFlashFragNo = (g_SoftUpdateCtr.u32FlashCurAddr - BOOT_LOADER_BLEN 
									+ FLASH_HALF_BLEN - g_SoftUpdateCtr.u32FlashEndAddr)/FLASH_FRAG_DEFAULT_BLEN;
	#else
		pFlashReq->uFlashFragNo = ((g_SoftUpdateCtr.u32FlashCurAddr - FLASHADD_UPGRADABLE_START) + (FLASH_UPDATEABLE_BLEN - (g_SoftUpdateCtr.u32FlashEndAddr - FLASHADD_UPGRADABLE_START))) / FLASH_FRAG_DEFAULT_BLEN;
	#endif
	}
#if LEGACY
	pFlashReq->uFlashFragNum = (FLASH_HALF_BLEN - BOOT_LOADER_BLEN)/FLASH_FRAG_DEFAULT_BLEN;	
#else
	pFlashReq->uFlashFragNum = FLASH_UPDATEABLE_BLEN / FLASH_FRAG_DEFAULT_BLEN;
#endif
	/* 标志等待定时器 */
	g_SoftUpdateCtr.uTmr_ReqWait_ms = MAX_FLASH_REQ_WAIT_TIME_ms;

	return TRUE;	/* 其实这个函数本来是无需返回的，但是为了编程方便而添加 */
}

BOOL QueryAndPubFlashReq(MQTT_COMM* pMqttComm, BOOL bNeedWait)
{
#if LEGACY
	if((g_SoftUpdateCtr.uReqSoftVer == 0) || (g_SoftUpdateCtr.u32FlashCurAddr < BOOT_LOADER_BLEN) 
#else
	if((g_SoftUpdateCtr.uReqSoftVer == 0) || (g_SoftUpdateCtr.u32FlashCurAddr < MAX_UPDATE_STATUS)
#endif
		|| (g_SoftUpdateCtr.uTmr_ReqWait_ms && bNeedWait))
	{
		return TRUE;
	} else {
		FLASH_REQ FlashReq;
		CreateFlashReq(&FlashReq, MQTT_TR_BASE64_BLEN);	/* 产生代码请求 */
		uint8* pTxBuf = pMqttComm->u8TRBuf + 5; 				/* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
		uint8 u8QoS = 0;			/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
		/* 请求升级:打印Topic */
		PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
		PrintStringNoOvChk(&pTxBuf, "/AUTH/");
		*pTxBuf++ = '0' + (g_SoftUpdateCtr.uReqSoftVer/1000)%10;
		*pTxBuf++ = '0' + (g_SoftUpdateCtr.uReqSoftVer/100)%10;
		*pTxBuf++ = '0' + (g_SoftUpdateCtr.uReqSoftVer/10)%10;
		*pTxBuf++ = '0' + (g_SoftUpdateCtr.uReqSoftVer/1)%10;
		uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
		if(u8QoS) {
			pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
		}
		*pTxBuf++ = '{';	/* Json开始 */
		PrintBase64ToJson(&pTxBuf, "base", (uint8*)&FlashReq, FLASH_REQ_ENCRYPT_LEN); /* base64编码 */
		*pTxBuf++ = ',';
		PrintU32DatToJson(&pTxBuf, "no", FlashReq.uFlashFragNo, 0);			/* 序列号 */
		PrintU32DatToJson(&pTxBuf, "total", FlashReq.uFlashFragNum, 0);		/* 总的代码片数量 */
		pTxBuf--;
		*pTxBuf++ = '}';	/* Json结尾 */
		/* 请求升级结果发布:传输 */
		return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);		/* QoS修改必须在本段代码开始的位置 */
	}
}

BOOL IniSoftUpdate(uint16 uSoftVersion)
{
	Swi_disable();
	if((g_SoftUpdateCtr.uReqSoftVer == 0) && (g_SoftUpdateCtr.u32FlashCurAddr < MAX_UPDATE_STATUS)) {   /* 还没有开始升级 */
		g_SoftUpdateCtr.uReqSoftVer = uSoftVersion;
		Swi_enable();
		/* 破坏软件完整性标志 */
		InvalidateAnotherSoft();
		/* 擦除半片flash的程序部分，TI耗时约30ms,不会影响任务调度 */
		FlashErase2(FLASHADD_PROG_START + (!CheckExeFlash_0Low_1High())*FLASH_HALF_BLEN, FLASH_PROG_BLEN);			/* 除去中断向量表和模拟EEPROM和启动版本之外的区域 */
		g_SoftUpdateCtr.u32FlashEndAddr = (FLASHADD_PROG_START + (((uint32)rand() % FLASH_PROG_BLEN)) + MAX_UPDATE_STATUS) & 0xFFFFFF80;		/* 必须128bit(0xFFFFFF80)对齐, 跳过MAX_UPDATE_STATUS是为避免如果FLASHADD_PROG_START为0时，和MAX_UPDATE_STATUS冲突 */
		g_SoftUpdateCtr.u32FlashCurAddr = g_SoftUpdateCtr.u32FlashEndAddr;
		return TRUE;
	} else {
		Swi_enable();
		return FALSE;
	}
}

//"61":"00000000","62":"00000000","63":"",
//"64":"","65":"00000000","66":"00000000","67":"","68":"","69":"00000000",
//"70":"","71":"00000000","72":"00000000","73":"00000000","74":"00000000","75":"00000000",
//"76":"00000000","77":"00000000","78":"00000000","79":"00000000","dev":
BOOL UpdateSoft(uint32* pU32FlashPack, uint32 u32FlashPackBLen)
{
	/* 判断是否在升级过程 */
	if((g_SoftUpdateCtr.uReqSoftVer == 0) || (g_SoftUpdateCtr.u32FlashCurAddr < MAX_UPDATE_STATUS)) {
		return FALSE;
	}
	
	/* 代码片解密, 数据处理 */
	DecryptWithAES(AUTH_AES_FLASH_DATA, pU32FlashPack, u32FlashPackBLen, g_SoftUpdateCtr.u32FlashCurAddr);
	BOOL bWhichHalf_0Fst_1Sec  = (pU32FlashPack[0] >> 24) == 1;		/* 第三个字节 */
	uint16 u32FlashFragBLen = u32FlashPackBLen - 16;				/* 减去FlashPack包头、包尾部分4个uint32 */
	uint32 u32SerialNo = pU32FlashPack[1 + u32FlashFragBLen/4];
	uint32 u32ReqSoftVer = pU32FlashPack[2 + u32FlashFragBLen/4];
	uint32 u32FlashAddr = pU32FlashPack[3 + u32FlashFragBLen/4];
	uint32* pU32FlashData = pU32FlashPack + 1;

	g_CodeTest.u32Val[40] = u32FlashFragBLen;								//00000C60
	g_CodeTest.u32Val[41] =	u32SerialNo;									//054C5638
	g_CodeTest.u32Val[42] = u32ReqSoftVer;									//00002328
	g_CodeTest.u32Val[43] = u32FlashAddr;									//08000000
	g_CodeTest.u32Val[44] = bWhichHalf_0Fst_1Sec;							//00000001
	g_CodeTest.u32Val[45] = g_SoftUpdateCtr.u32FlashCurAddr;				//08000000
	g_CodeTest.u32Val[46] = (u32FlashAddr - FLASH_BASE) / FLASH_HALF_BLEN;	//00000000
	g_CodeTest.u32Val[47] = CheckExeFlash_0Low_1High();						//00000000
	g_CodeTest.u32Val[48] = g_Sys.SerialNo.u32Dat;							//054C5638
	g_CodeTest.u32Val[49] = g_SoftUpdateCtr.uReqSoftVer;					//00002328

	/* 核对代码是否正确 */
	if((u32FlashAddr == g_SoftUpdateCtr.u32FlashCurAddr) 
		&& ((u32FlashAddr - FLASH_BASE) < FLASH_HALF_BLEN)
		&& (u32ReqSoftVer == g_SoftUpdateCtr.uReqSoftVer)
		&& ((u32SerialNo == 0) || (u32SerialNo == g_Sys.SerialNo.u32Dat)))
	{
	    g_SoftUpdateCtr.u32LastFragAddr = u32FlashAddr;
	    /* 写入代码片 */
	    if(FlashUpgradeFrag(bWhichHalf_0Fst_1Sec, u32FlashAddr, pU32FlashData, u32FlashFragBLen)) {
			/* 完全一致，则修改flash地址 */
    		u32FlashAddr = g_SoftUpdateCtr.u32FlashCurAddr + u32FlashFragBLen;
    		if(u32FlashAddr >= (FLASH_BASE + FLASH_HALF_BLEN)) {		/* 不能回卷到0，0地址是BootLoader */
    			u32FlashAddr = FLASHADD_UPGRADABLE_START;
    		}
    		g_CodeTest.u32Val[67]++;	//0000003A
		} else {
		    g_SoftUpdateCtr.uReqSoftVer = 0;
            g_SoftUpdateCtr.u32FlashCurAddr = UPDATE_FAIL;      /* 升级失败 */
            g_CodeTest.u32Val[68]++;		//00000001
            return FALSE;
		}

		/* 判断是否最后一个代码片: 否，修正地址，继续下一个代码片 */
		if(u32FlashAddr != g_SoftUpdateCtr.u32FlashEndAddr) {
			g_SoftUpdateCtr.u32FlashCurAddr = u32FlashAddr;
		} else {	/* 判断是否最后一个代码片: 是，标志完成：校验写入是否正确，打上完全标志 */
			/* 代码校验 */
			if(0) {
				g_SoftUpdateCtr.u32FlashCurAddr = UPDATE_FAIL;		/* 升级失败 */
			} else {
				OnPostLiveUpdate();
				/* 标志完成升级 */
				/* g_SoftUpdateCtr.uReqSoftVer = 0;   这行代码会让485口升级情况下，不断重启升级 */
				g_SoftUpdateCtr.u32FlashCurAddr = UPDATE_CMPLT;
				g_Sys.tReboot_0Null_nUrgent_pWaitIdle = 1;
			}
			return FALSE;	/* 已经完成了升级，就不需要再产生FlashReq了 */
		}
	} else {
        Rec_DebugChn(0, g_SoftUpdateCtr.u32LastFragAddr);
        Rec_DebugChn(0, g_SoftUpdateCtr.u32FlashCurAddr);
        Rec_DebugChn(0, u32FlashAddr);
        Rec_DebugChn(0, u32ReqSoftVer);
        Rec_DebugChn(0, u32SerialNo);
	}
	return TRUE;
}

/* 母板函数：发布软件 */
uint32 PubSoftware(FLASH_REQ* pFlashReq, uint32* pU32FlashPack, uint32 u32MaxFlashPackBLen, uint32 u32SerialNo)
{
	if(u32MaxFlashPackBLen < 16) {		/* 输入检查 */
		return 0;
	}
	
	/* 解密FlashReq，并确定所用的密码组 */
	uint16 uPubSoftSupportPwdGrpNum = 1;
	if(g_PubSoftAuthCtr.u8PubSoftPwdGrpNum) {
		uPubSoftSupportPwdGrpNum = g_PubSoftAuthCtr.u8PubSoftPwdGrpNum;
	}
	FLASH_REQ FlashReq;
	uint16 uPwdGrpNo = 0;	/* 密码组序号 */
	do {
		FlashReq = *pFlashReq;
		DecryptWithAES((AUTH_TYPE)(AUTH_AES_FLASH_REQ + AUTH_TYPE_NUM*uPwdGrpNo), (uint32*)(&FlashReq), FLASH_REQ_ENCRYPT_LEN, 0);
	} while((!ChkAuthKey((AUTH_TYPE)(AUTH_KEY_FLASH_REQ_KEY + AUTH_TYPE_NUM*uPwdGrpNo), &FlashReq.u32FlashReqKey)) 
			&& (++uPwdGrpNo < uPubSoftSupportPwdGrpNum));

	if((uPwdGrpNo < uPubSoftSupportPwdGrpNum)
	#if LEGACY
		 && ((BOOT_LOADER_BLEN <= FlashReq.u32FlashAddr) && (FlashReq.u32FlashAddr < FLASH_HALF_BLEN))) 	/* 地址范围 */
	 #else
		&& ((FlashReq.u32FlashAddr >= FLASHADD_UPGRADABLE_START) && ((FlashReq.u32FlashAddr - FLASH_BASE) < FLASH_HALF_BLEN))) 
	 #endif
	{
        /* 功能合法性检查 */
		if(FlashReq.bIniSoftUpdate) {
			if(FlashReq.bSoft_Run1_Test0) {
				if(g_PubSoftAuthCtr.uPubSoftUpdateCnt) {
					g_PubSoftAuthCtr.uPubSoftUpdateCnt--;
				} else if(SOFT_RUN1_TEST0) {
					return FALSE;
				}
			} else {
				if(g_PubSoftAuthCtr.uPubSoftInstallCnt) {
					g_PubSoftAuthCtr.uPubSoftInstallCnt--;
                } else if(SOFT_RUN1_TEST0) {
					return FALSE;
				}
			}
		}
		/* 调试:相同的代码片 */
	    Rec_DebugChn(1, FlashReq.u32FlashAddr);
	    Rec_DebugChn(1, RunTime_Interval(1)/120000);		/* TODO：主频不一样 */
		if(pFlashReq->uFlashFragNo && (pFlashReq->uFlashFragNo == g_PubSoftAuthCtr.uFlashFragNo)) {
		    g_CodeTest.uVal3[0]++;
		    g_CodeTest.uVal3[1] = pFlashReq->uFlashFragNo;
		}
		/* 记录升级进度 */
        g_PubSoftAuthCtr.uFlashFragNo = pFlashReq->uFlashFragNo;
        g_PubSoftAuthCtr.uFlashFragNum = pFlashReq->uFlashFragNum;
    
		/* 计算代码片长度 */
		uint32 u32FlashFragBLen = FlashReq.u32FlashFragBLen;
		if((u32FlashFragBLen >= 10000000UL) || (u32FlashFragBLen%16 != 0)) {
			u32FlashFragBLen = FLASH_FRAG_DEFAULT_BLEN;
		}
		u32MaxFlashPackBLen -= 16;	/* 调整为MaxFlashFragBLen, 需要扣除FlashPack包头、包尾部分4个uint32 */
		if(u32FlashFragBLen > u32MaxFlashPackBLen) {
			u32FlashFragBLen = u32MaxFlashPackBLen;
		}
		
		/* 填充、并生成加密的软件包 */
		pU32FlashPack[0] = ((uint32)(((uint32)rand()<<8) + rand()) >> 8) | (FlashReq.bWhichHalf_0Fst_1Sec << 24);
		pU32FlashPack[1 + u32FlashFragBLen/4] = u32SerialNo;
		pU32FlashPack[2 + u32FlashFragBLen/4] = SOFTWARE_VER;
		pU32FlashPack[3 + u32FlashFragBLen/4] = FlashReq.u32FlashAddr;
		memcpy(&pU32FlashPack[1], (const void*)(FlashReq.u32FlashAddr + (FlashReq.bWhichHalf_0Fst_1Sec==1)*FLASH_HALF_BLEN), u32FlashFragBLen);
		u32FlashFragBLen += 16;		/* 调整为FlashPackBLen，需要增加FlashPack包头、包尾部分4个uint32 */
		EncryptWithAES((AUTH_TYPE)(AUTH_AES_FLASH_DATA + AUTH_TYPE_NUM*uPwdGrpNo), pU32FlashPack, u32FlashFragBLen, FlashReq.u32FlashAddr);

//		UpdateSoft((uint32*)pU32FlashPack, 1440);


		return u32FlashFragBLen;
	} else {
	    Rec_DebugChn(0, uPwdGrpNo);
	    Rec_DebugChn(0, FlashReq.u32FlashAddr);
		return 0;
	}
}

BOOL ProcMqttAuth_SOFT(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd, uint32 u32TopicSN)
{
	pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 1);
	BOOL bBase64 = CompareMsg(pU8Msg, "base");		/* base64编码 */
	pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 2);
    uint8* pMsgEnd_FlashReq = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 1) - 1;
    
	FLASH_REQ FlashReq;
	uint16 u32FlashPackBLen;
	uint32* pU32FlashPack;      /* 由于是利用Mqtt的TRBuf作为FlashPack，因此需要从后往前安排 */
	BOOL bFlashReqSuc;
	
    /* 校验请求获得实际的u32FlashPackBLen */
	if(bBase64) {
		pU32FlashPack = (uint32*)(pMqttComm->u8TRBuf + MQTT_TR_BUF_BLEN - MQTT_TR_BASE64_BLEN);
		DecodeByBase64(pU8Msg, pMsgEnd_FlashReq - pU8Msg, (uint8*)&FlashReq, FLASH_REQ_ENCRYPT_LEN);   /* base64转成hex */
		pU8Msg = SkipCharInString(pMsgEnd_FlashReq, pU8MsgEnd, '"', 2);
		FlashReq.uFlashFragNo = ReadU32(&pU8Msg, pU8MsgEnd);
		FlashReq.uFlashFragNum = ReadU32(&pU8Msg, pU8MsgEnd);
		bFlashReqSuc = ((u32FlashPackBLen = PubSoftware(&FlashReq, pU32FlashPack, MQTT_TR_BASE64_BLEN, u32TopicSN)) != 0);
	} else {
		pU32FlashPack = (uint32*)(pMqttComm->u8TRBuf + MQTT_TR_BUF_BLEN - MQTT_FLASH_PACK_ASCII_BLEN);
		bFlashReqSuc = GetHexArray(pU8Msg, pMsgEnd_FlashReq, (uint8*)&FlashReq, FLASH_REQ_ENCRYPT_LEN);      /* ASCII转化为HEX */
		FlashReq.uFlashFragNo = ReadU32(&pU8Msg, pU8MsgEnd);
		FlashReq.uFlashFragNum = ReadU32(&pU8Msg, pU8MsgEnd);
		bFlashReqSuc &= ((u32FlashPackBLen = PubSoftware(&FlashReq, pU32FlashPack, MQTT_FLASH_PACK_ASCII_BLEN, u32TopicSN)) != 0);
	}

	/* 发布代码片 */
	if(bFlashReqSuc)	{
		uint8* pTxBuf = pMqttComm->u8TRBuf + 5;		/* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
		uint8 u8QoS = 0;			/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
		/* 请求升级:打印Topic */
		PrintStringNoOvChk(&pTxBuf, DEV_TYPE_MQTT_TOPIC_String);
		PrintU32(&pTxBuf, pTxBuf+16, u32TopicSN, 0);
		PrintStringNoOvChk(&pTxBuf, "/SUB/UPDATE/");
		*pTxBuf++ = '0' + (SOFTWARE_VER/1000)%10;
		*pTxBuf++ = '0' + (SOFTWARE_VER/100)%10;
		*pTxBuf++ = '0' + (SOFTWARE_VER/10)%10;
		*pTxBuf++ = '0' + (SOFTWARE_VER/1)%10;
		uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
		if(u8QoS) {
			pTxBuf += 2;	/* 根据发布的消息QoS,预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
		}
		*pTxBuf++ = '{';
		if(bBase64) {
			PrintBase64ToJson(&pTxBuf, "base", (uint8*)pU32FlashPack, u32FlashPackBLen);	/* 把母板发布软件代码片第X片转成base64码 */
		} else {
			PrintHexToJson(&pTxBuf, "code", (uint8*)pU32FlashPack, u32FlashPackBLen);	/* 把母板发布软件代码片第X片转成ASCII码 */
		}
		*pTxBuf++ = '}';
		/* 请求升级结果发布:传输 */
		return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS修改必须在本段代码开始的位置 */
	} else {
		return TRUE;
	}
}

/*==========================================================================
| Description	: 版本管理
	GetSoftVersion():获得flash里面的软件版本
	FixSoftVerAndEEConf():通过jtag口下载软件的时候，eeprom中软件版本描述和实际不符的情况
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2017-10-23
\=========================================================================*/
#if CPU_0ST_1TI == 1
void GetSoftVersion(BOOT_SOFT_VERSION* pBootSoftVer)
{
	EEPROMRead((uint32_t*)pBootSoftVer, EEADD_BOOT_LOADER_CONF, sizeof(BOOT_SOFT_VERSION));
	if((pBootSoftVer->u32RunVersion&0xFFFF) + (pBootSoftVer->u32RunVersion>>16) == 0x10000UL) {
		pBootSoftVer->u32RunVersion &= 0xFFFF;
	} else {
		pBootSoftVer->u32RunVersion = 0;
	}
	if((pBootSoftVer->u32LowFlashVer&0xFFFF) + (pBootSoftVer->u32LowFlashVer>>16) == 0x10000UL) {
		pBootSoftVer->u32LowFlashVer &= 0xFFFF;
	} else {
		pBootSoftVer->u32LowFlashVer = 0;
	}
	if((pBootSoftVer->u32HighFlashVer&0xFFFF) + (pBootSoftVer->u32HighFlashVer>>16) == 0x10000UL) {
		pBootSoftVer->u32HighFlashVer &= 0xFFFF;
	} else {
		pBootSoftVer->u32HighFlashVer = 0;
	}
}
#endif

/*==========================================================================
| Description	: 序列号相关
	PubSerialNo_JSON():母板发布序列号，由于仅一个地方调用，不考虑重入性
	UpdateSerialNo_JSON():控制器更新序列号
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2017-10-23
\=========================================================================*/
BOOL ProcMqttAuth_SN(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd)
{
	uint8* pU8SubMsg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 3);
	uint32 u32IniSerialNo = atoi((const char*)pU8SubMsg);
	pU8SubMsg = SkipCharInString(pU8SubMsg, pU8MsgEnd, '"', 4);
	SERIAL_PACK SerialPack;
#if 1
	SerialPack.u32IniSerialNo = u32IniSerialNo;
	SerialPack.u32NewSerialNo = atoi((const char*)pU8SubMsg);
#else
	uint8* pBase64End = SkipCharInString(pU8SubMsg, pU8MsgEnd, '"', 1) - 1;
	memcpy(g_CodeTest.u8Val, (uint8*)pU8SubMsg, pBase64End - pU8SubMsg);
	//DecodeByBase64(pU8SubMsg, pBase64End - pU8SubMsg, &SerialPack, sizeof(SERIAL_PACK));
	GetHexArray(pU8SubMsg, pBase64End, (uint8*)&SerialPack, sizeof(SERIAL_PACK));
	memcpy((uint8*)g_CodeTest.uVal, (uint8*)&SerialPack, sizeof(SERIAL_PACK));
	DecryptWithAES(AUTH_AES_COMM_SN_LIC, (uint32*)(&SerialPack), sizeof(SERIAL_PACK), 0);
	memcpy((uint8*)g_CodeTest.uVal2, (uint8*)&SerialPack, sizeof(SERIAL_PACK));
	/* 关闭母版 */
	return TRUE;
	if((SerialPack.u32IniSerialNo != u32IniSerialNo) || (labs(SerialPack.u32AuthTime - GetRTCSeconds()) > 10)) {
		g_PubSoftAuthCtr.uPubSnCnt = 0;
		//return FALSE;
		return TRUE;
	}
#endif
	SerialPack.u32AuthTime = GetRTCSeconds();
	SerialPack.u32Rand = ((uint32)rand()<<16) + rand();
	EncryptWithAES((AUTH_TYPE)(AUTH_AES_PACK_SERIAL + g_PubSoftAuthCtr.u8PubSnLicPwdGrpNo*AUTH_TYPE_NUM), 
					(uint32*)(&SerialPack), sizeof(SERIAL_PACK), 0);
	g_PubSoftAuthCtr.uPubSnCnt--;		/* 这个地方不做判断，由调用入口进行判断，不考虑重入性 */
	
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5;	/* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
	uint8 u8QoS = 0;			/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
	/* 请求升级:打印Topic */
	PrintStringNoOvChk(&pTxBuf, DEV_TYPE_MQTT_TOPIC_String);
	PrintU32(&pTxBuf, pTxBuf+16, u32IniSerialNo, 0);
	PrintStringNoOvChk(&pTxBuf, "/SUB/UPDATE/SN");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
	}
	*pTxBuf++ = '{';
	PrintHexToJson(&pTxBuf, "code", (uint8*)(&SerialPack), sizeof(SERIAL_PACK));
	*pTxBuf++ = '}';
	
	/* 请求序列号结果发布:传输 */
	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS修改必须在本段代码开始的位置 */
}

/*==========================================================================
| Description	: 授权所需
	PubLicense():母板发布授权，由于仅一个地方调用，不考虑重入性
	UpdateLicense():控制器更新授权
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2017-10-23
\=========================================================================*/
BOOL ProcMqttAuth_LIC(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd)
{
	uint8* pU8SubMsg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 3);
	uint32 u32DevSerialNo = atoi((const char*)pU8SubMsg);
	pU8SubMsg = SkipCharInString(pU8SubMsg, pU8MsgEnd, '"', 4);
	LICENSE_PACK LicensePack;
#if 1
	uint8 u8Year = (pU8SubMsg[0] - '0')*10 + (pU8SubMsg[1] - '0');
	uint8 u8Month = (pU8SubMsg[2] - '0')*10 + (pU8SubMsg[3] - '0');
	uint8 u8Day = (pU8SubMsg[4] - '0')*10 + (pU8SubMsg[5] - '0');
	uint8 u8Hour = (pU8SubMsg[6] - '0')*10 + (pU8SubMsg[7] - '0');
	LicensePack.u32DevSerialNo = u32DevSerialNo;
	LicensePack.u32License_Seconds = CalRTCSecondsByDate(u8Year, u8Month, u8Day, u8Hour, 0, 0);
#else
	uint8* pBase64End = SkipCharInString(pU8SubMsg, pU8MsgEnd, '"', 1) - 1;
	//DecodeByBase64(pU8SubMsg, pBase64End - pU8SubMsg, &LicensePack, sizeof(LICENSE_PACK));
	GetHexArray(pU8SubMsg, pBase64End, (uint8*)&LicensePack, sizeof(LICENSE_PACK));
	DecryptWithAES(AUTH_AES_COMM_SN_LIC, (uint32*)(&LicensePack), sizeof(LICENSE_PACK), 0);
	/* 关闭母版 */
	return TRUE;
	if((LicensePack.u32DevSerialNo != u32DevSerialNo) || (labs(LicensePack.u32AuthTime - GetRTCSeconds()) > 10)) {
		g_PubSoftAuthCtr.uPubLicCnt = 0;
		//return FALSE;
		return TRUE;
	}
#endif
	LicensePack.u32AuthTime = GetRTCSeconds();
	LicensePack.u32Rand = ((uint32)rand()<<16) + rand();
	EncryptWithAES((AUTH_TYPE)(AUTH_AES_PACK_LICENSE + g_PubSoftAuthCtr.u8PubSnLicPwdGrpNo*AUTH_TYPE_NUM), 
					(uint32*)(&LicensePack), sizeof(LICENSE_PACK), 0);
	g_PubSoftAuthCtr.uPubLicCnt--;		/* 这个地方不做判断，由调用入口进行判断，不考虑重入性 */

	uint8* pTxBuf = pMqttComm->u8TRBuf + 5;	/* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
	uint8 u8QoS = 0;			/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
	/* 请求升级:打印Topic */
	PrintStringNoOvChk(&pTxBuf, DEV_TYPE_MQTT_TOPIC_String);
	PrintU32(&pTxBuf, pTxBuf+16, u32DevSerialNo, 0);
	PrintStringNoOvChk(&pTxBuf, "/SUB/UPDATE/LICENSE");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
	}
	*pTxBuf++ = '{';
	PrintHexToJson(&pTxBuf, "code", (uint8*)(&LicensePack), sizeof(LICENSE_PACK));
	*pTxBuf++ = '}';
	
	/* 请求授权结果发布:传输 */
	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS修改必须在本段代码开始的位置 */
}

/*==========================================================================
| Description	: 普通板子<>母板切换所需
	ProcVersionMasterBCD()	: 校验输入口令，把普通板子切换成母板
	ProcMqttAuth_CMD()		: Mqtt母板指令
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2017-10-23
\=========================================================================*/
#define ConvertBCD2uint16(pRxBuf) ((pRxBuf[0]/0x10)*1000 + (pRxBuf[0]%0x10)*100 + (pRxBuf[1]/0x10)*10 + (pRxBuf[1]%0x10))
#define ConvertBCD2uint8(pRxBuf) ((pRxBuf[0]/0x10)*10 + (pRxBuf[0]%0x10))
void ProcVersionMasterBCD(uint8* pRxBuf, uint16 uRegNum)
{
	g_PubSoftAuthCtr.u8PubSoftPwdGrpNum = ConvertBCD2uint8(pRxBuf);
	pRxBuf++;
	g_PubSoftAuthCtr.u8PubSnLicPwdGrpNo = ConvertBCD2uint8(pRxBuf);
	pRxBuf++;
	if(g_PubSoftAuthCtr.uTmr_IniPubCntNoPwd_ms && (uRegNum > 1)) {	/* 只能写入一次 */
		/* 把BCD编码转成普通编码 */
		g_PubSoftAuthCtr.uPubSoftInstallCnt = ConvertBCD2uint16(pRxBuf);
		pRxBuf += 2;
		g_PubSoftAuthCtr.uPubSoftUpdateCnt = ConvertBCD2uint16(pRxBuf);
		pRxBuf += 2;
		g_PubSoftAuthCtr.uPubSnCnt = ConvertBCD2uint16(pRxBuf);
		pRxBuf += 2;
		g_PubSoftAuthCtr.uPubLicCnt = ConvertBCD2uint16(pRxBuf);
		g_PubSoftAuthCtr.uTmr_IniPubCntNoPwd_ms = 0;		/* 仅能写入一次 */
	}
}

BOOL ProcMqttAuth_CMD(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd)
{
	uint8* pU8SubMsg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 3);
	if(CompareMsg(pU8SubMsg, "close")) {
		g_PubSoftAuthCtr.uPubSoftInstallCnt = 0;
		g_PubSoftAuthCtr.uPubSoftUpdateCnt = 0;
		g_PubSoftAuthCtr.uPubSnCnt = 0;
		g_PubSoftAuthCtr.uPubLicCnt = 0;
		g_PubSoftAuthCtr.uTmr_IniPubCntNoPwd_ms = 0;
	}
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5;	/* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
	uint8 u8QoS = 0;			/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
	/* 请求升级:打印Topic */
	PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
	PrintStringNoOvChk(&pTxBuf, "/RES/AUTH");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
	}
	*pTxBuf++ = '{';
	PrintU32DatToJson(&pTxBuf, "soft_ver", SOFTWARE_VER, 0);
	PrintStringNoOvChk(&pTxBuf, "{\"cnt\":{");
	PrintU32DatToJson(&pTxBuf, "install", g_PubSoftAuthCtr.uPubSoftInstallCnt, 0);
	PrintU32DatToJson(&pTxBuf, "upgrade", g_PubSoftAuthCtr.uPubSoftUpdateCnt, 0);
	PrintU32DatToJson(&pTxBuf, "sn", g_PubSoftAuthCtr.uPubSnCnt, 0);
	PrintU32DatToJson(&pTxBuf, "lic", g_PubSoftAuthCtr.uPubLicCnt, 0);
	pTxBuf--;
	*pTxBuf++ = '}';
	*pTxBuf++ = '}';
	
	/* 母板状态发布:传输 */
	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS修改必须在本段代码开始的位置 */
}

///////////////////////////////////////////////////////////////////////////////////
//	以下为各种应用工具
///////////////////////////////////////////////////////////////////////////////////
/* 用于调试,测量运行时间 */
#define CPU_RUN_TIME_CALI_CHN	8					/* CPU运行时间测量通道 */
typedef struct {
	uint32 u32LastCPUTime[CPU_RUN_TIME_CALI_CHN];
	uint32 u32IntervalCnt[CPU_RUN_TIME_CALI_CHN];
}CPU_RUN_TIME_VAR;
CPU_RUN_TIME_VAR g_CpuRunTime;
/*==========================================================================
| Description	: 启动CPU运行时间测量
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2010-1-30
\=========================================================================*/
void RunTime_StartCali(uint8 u8Chn)
{
	if(u8Chn < CPU_RUN_TIME_CALI_CHN) {
		g_CpuRunTime.u32LastCPUTime[u8Chn] = GetCPUTimerCnt();
	}
}

/*==========================================================================
| Description	: 结束当前次CPU运行时间测量
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2010-1-30
\=========================================================================*/
uint32 RunTime_EndCali(uint8 u8Chn)
{
	if(u8Chn < CPU_RUN_TIME_CALI_CHN) {
		g_CpuRunTime.u32IntervalCnt[u8Chn] = GetCPUTimerCnt() - g_CpuRunTime.u32LastCPUTime[u8Chn];
	}
	return g_CpuRunTime.u32IntervalCnt[u8Chn];
}

/*==========================================================================
| Description	: 测量运行间隔时间
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2010-1-30
\=========================================================================*/
uint32 RunTime_Interval(uint8 u8Chn)
{
	if(u8Chn < CPU_RUN_TIME_CALI_CHN) {
		uint32 u32Tmp = GetCPUTimerCnt();
		g_CpuRunTime.u32IntervalCnt[u8Chn] = u32Tmp - g_CpuRunTime.u32LastCPUTime[u8Chn];
		g_CpuRunTime.u32LastCPUTime[u8Chn] = u32Tmp;
	}

	return g_CpuRunTime.u32IntervalCnt[u8Chn];
}

/*==========================================================================
| Description	: 通用调试工具，主要包括如下几种工具：
	1. 少量数据录波, DebugChn，通道少，但存储较深，一般无需其他工具配合，在ccs界面直接观察内存就可以
	2. 大量数据录波, DebugPack,通道多，但存储很浅，需要其他工具配合及时读出(RS485, MQTT)，需要相应软件进行读出、分析
	3. Mqtt打印，把调试信息通过Mqtt打印出来
| Author		: Wang Renfei			Date	: 2020-07-30
\=========================================================================*/
#define DEBUG_CHN_LEN 500
#define DEBUG_CHN_NUM 2
typedef struct {
	uint32 u32RecPt;
	int32 i32Dat[DEBUG_CHN_LEN];
}DEBUG_CHN;
SECTION(".NOT_ZeroInit") DEBUG_CHN g_DebugChn[DEBUG_CHN_NUM];

#define DEBUG_PACK_FDAT_CHN	11
#define DEBUG_PACK_LEN	120		/* mqtt一秒钟传输一次，每次最多传输 约4KB*3/4 = 3KB */
typedef struct {
	uint32 u32Flag;
	float32 fDat[DEBUG_PACK_FDAT_CHN];
}DEBUG_PACK_DATA;
typedef struct{
	BOOL bRecSW;
	uint8 u8RecPt;
	uint8 u8SendPt_RS485;
	uint8 u8SendPt_Mqtt;
	DEBUG_PACK_DATA Data[DEBUG_PACK_LEN];
}DEBUG_PACK;
SECTION(".NOT_ZeroInit") DEBUG_PACK g_DebugPack;

#define MQTT_PRINT_BUF_BLEN		1024
typedef struct {
	/* 通过mqtt发布任务，打印调试信息，每秒钟统一输出一次 */
	uint16 uBufPt;		/* 写入指针 */
	uint8 u8Buf[MQTT_PRINT_BUF_BLEN];	/* 每一条信息格式为 ,"ccc"，以便组装成json c是字符,不得含'"'   */
	uint16 uRsvd;
}MQTT_PRINT;
SECTION(".NOT_ZeroInit") MQTT_PRINT g_MqttPrint;

void InitDebugDat(void)
{
    InitDataWithZero((uint8*)(&g_CodeTest), sizeof(g_CodeTest));
 	InitDataWithZero((uint8*)(&g_DebugChn), sizeof(g_DebugChn));
 	g_DebugPack.bRecSW = TRUE;
 	g_DebugPack.u8RecPt = 0;
 	g_DebugPack.u8SendPt_RS485 = 0;
 	g_DebugPack.u8SendPt_Mqtt = 0;
	g_MqttPrint.uBufPt = 0;
}

/*===========================================================================
 * DebugChn用于记录少量调试数据，由于Buf较深，因此可以直接内存中查看数据
 * a. 通过 Rec_DebugChn()记录数据，
 * b. CopyDebugChnToTxBuf()把数据拷贝到modbus TxBuf中
 *==========================================================================*/
void Rec_DebugChn(uint8 u8RecChn, int32 i32Dat)
{
	DEBUG_CHN* pDebugChn = &g_DebugChn[u8RecChn];
	if((u8RecChn < DEBUG_CHN_NUM) && (pDebugChn->u32RecPt < DEBUG_CHN_LEN)) {
		pDebugChn->i32Dat[pDebugChn->u32RecPt] = i32Dat;
		pDebugChn->u32RecPt++;
	}
}
/* 发送DebugChn数据到Modbus TxBuf, 数据填充顺序 */
void CopyDebugChnToTxBuf(uint8** ppTxBuf, uint16 uRegOff)
{
#if 0
	uRegOff = uRegOff/DEBUG_CHN_LEN;	/* 指向Chn通道 */
	
	if(uRegOff < DEBUG_CHN_NUM) {		/* 输入检查 */
		uint8* pTxBuf = *ppTxBuf;
		DEBUG_CHN* pDebugChn = &g_DebugChn[uRegOff];
		int16 iRecPt = pDebugChn->i32RecPt - 1;		/* pDebugChn->uRecPt指向的是待存位置，iRecPt指向存了数据的位置 */
		if(iRecPt < 0) {
			iRecPt = DEBUG_CHN_LEN - 1;
		}

		/* 拷贝数据进TxBuf */
		int16 i;
		for(i = iRecPt; i >= 0; i--) {
			*pTxBuf++ = pDebugChn->i32Dat[i]/0x100;
			*pTxBuf++ = pDebugChn->i32Dat[i]%0x100;
		}
		for(i = DEBUG_CHN_LEN - 1; i >= iRecPt; i--) {
			*pTxBuf++ = pDebugChn->i32Dat[i]/0x100;
			*pTxBuf++ = pDebugChn->i32Dat[i]%0x100;
		}
		
		*ppTxBuf = pTxBuf;
	}
#endif
}

/* 把DebugChn数据通过mqtt发布 */
BOOL PubDebugChn(MQTT_COMM* pMqttComm, uint8 u8DebugChnNo)
{
	/* 填充Mqtt Buf */
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
	uint8 u8QoS = 1;						/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
	/* 水位发布:打印Topic */
	PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
	PrintStringNoOvChk(&pTxBuf, "/DEBUG/CHN_DATA");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
	}
	/* 打印数据成Json格式 */
	*pTxBuf++ = '{';	/* Json开始 */
	if(u8DebugChnNo >= DEBUG_CHN_NUM) {
		u8DebugChnNo = 0;
	}
	PrintU32DatToJson(&pTxBuf, "chn_no", u8DebugChnNo, 0);
	if(sizeof(g_DebugChn[u8DebugChnNo]) < MQTT_TR_BASE64_BLEN) {	/* 理论上是不会超的，但以防万一 */
		PrintStringNoOvChk(&pTxBuf, "\"base\":\"");
		pTxBuf = EncodeByBase64((uint8*)&g_DebugChn[u8DebugChnNo], sizeof(g_DebugChn[u8DebugChnNo]), pTxBuf);
		*pTxBuf++ = '"';
	}
	*pTxBuf++ = '}';			/* Json结尾 */

	if(PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) {
		InitDataWithZero((uint8*)(&g_DebugChn[u8DebugChnNo]), sizeof(g_DebugChn[u8DebugChnNo]));
		g_DebugChn[u8DebugChnNo].u32RecPt = 0;	/* 复位，可以重新开始采集 */
		return TRUE;
	} else {
		return FALSE;
	}
}

/*===========================================================================
 * DebugPack用于记录大量调试数据，由于Buf很浅(仅存储1.5s左右数据)，因此必须并通过通讯接口(RS485,MQTT)及时访问数据
 * a. 通过Rec_DebugPack()记录数据，
 * b. TryDebugPack()和RunDebugPack()通过485接口发送数据
 * c. PubDebugPack()通过mqtt接口发送数据
 * RS485读取速度：115200bps，即11.52KB，考虑通讯开销，估计能够传输6KB数据，即6K/(12*4) = 125条记录
 * MQTT发送速度: 一秒钟传输一次，每次最多传输约4KB*3/4=3KB，传输记录数：3K/(12*4) = 62.5条
 *==========================================================================*/
void Rec_DebugPack(uint32 u32Flag, float32 fDat0, float32 fDat1, float32 fDat2, 
					float32 fDat3, float32 fDat4, float32 fDat5, float32 fDat6, 
					float32 fDat7, float32 fDat8, float32 fDat9, float32 fDat10)
{
	if(g_DebugPack.bRecSW) {
		uint8 u8RecPt = g_DebugPack.u8RecPt%DEBUG_PACK_LEN;
		g_DebugPack.Data[u8RecPt].u32Flag = u32Flag;
		g_DebugPack.Data[u8RecPt].fDat[0] = fDat0;
		g_DebugPack.Data[u8RecPt].fDat[1] = fDat1;
		g_DebugPack.Data[u8RecPt].fDat[2] = fDat2;
		g_DebugPack.Data[u8RecPt].fDat[3] = fDat3;
		g_DebugPack.Data[u8RecPt].fDat[4] = fDat4;
		g_DebugPack.Data[u8RecPt].fDat[5] = fDat5;
		g_DebugPack.Data[u8RecPt].fDat[6] = fDat6;
		g_DebugPack.Data[u8RecPt].fDat[7] = fDat7;
		g_DebugPack.Data[u8RecPt].fDat[8] = fDat8;
		g_DebugPack.Data[u8RecPt].fDat[9] = fDat9;
		g_DebugPack.Data[u8RecPt].fDat[10] = fDat10;
		g_DebugPack.u8RecPt = (u8RecPt+1)%DEBUG_PACK_LEN;
	}
}
/* 收到指令: bRecSW SendPt
	发送DebugPack数据到Modbus TxBuf, 数据填充顺序:
	GrpNum SendPt Chn Rsvd
	Dat[SendPt + 0][0~11]
	Dat[SendPt + 1][0~11] */
void TryDebugPack(uint8 u8UartPort)
{
	OpenUartComm(u8UartPort, 115200UL, 0, 0);
	RunDebugPack(u8UartPort);
}
void RunDebugPack(uint8 u8UartPort)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	if(ReadFromUart(u8UartPort, 20) && (pUartComm->uRxBufPt == 4)
		&& CheckCrcForModbusRTU(pUartComm->u8TRBuf, pUartComm->uRxBufPt))
	{
		pUartComm->uTimer_WatchDog_ms = 0;		/* 通讯成功 */
		pUartComm->bUartFuncTrySuc = TRUE;

		/* 数据处理 */
		uint32* pU32TxBuf = (uint32*)&pUartComm->u8TRBuf[4];	/* 32b对齐 */
		g_DebugPack.bRecSW = pUartComm->u8TRBuf[0];
		if(g_DebugPack.bRecSW == 0) {
			g_DebugPack.u8RecPt = 0;
			pUartComm->u8TRBuf[0] = 0;						/* 填充实际发送的组数 */
			pUartComm->u8TRBuf[1] = 0;						/* 填充下一个发送指针 */
			pUartComm->u8TRBuf[2] = DEBUG_PACK_FDAT_CHN+1;	/* 填充数据宽度 */
		} else {
			/* 填充数据部分:Modbus TxBuf最长负载255byte */
			uint8 u8SendPt = pUartComm->u8TRBuf[1]%DEBUG_PACK_LEN;
			uint8 u8RecPt = g_DebugPack.u8RecPt;
			int8 i,j;
			uint8 u8MaxTxGrpNum = (255 - 4)/sizeof(DEBUG_PACK_DATA);
			for(i = u8MaxTxGrpNum; (i > 0) && (u8SendPt != u8RecPt); i--) {
				uint32* pU32Dat = (uint32*)&g_DebugPack.Data[u8SendPt];
				for(j = sizeof(DEBUG_PACK_DATA)/4; j > 0; j--) {
					*pU32TxBuf++ = *pU32Dat++;
				}
				u8SendPt = (u8SendPt+1)%DEBUG_PACK_LEN;
			}
			/* 填充包头:备用2bytes */
			pUartComm->u8TRBuf[0] = u8MaxTxGrpNum - i;		/* 填充实际发送的组数 */
			pUartComm->u8TRBuf[1] = u8SendPt;		/* 填充下一个发送指针 */
			pUartComm->u8TRBuf[2] = DEBUG_PACK_FDAT_CHN+1;	/* 填充数据宽度 */
			g_DebugPack.u8SendPt_RS485 = u8SendPt;
		}
		TxDataForModbusRTU(u8UartPort, (uint8*)pU32TxBuf);
	}
}

/* 把DebugPack数据通过mqtt发布 */
BOOL PubDebugPack(MQTT_COMM* pMqttComm)
{
	if(g_DebugPack.u8SendPt_Mqtt == g_DebugPack.u8RecPt) {
		return TRUE;
	} else {
		/* 准备数据 */
		uint8 u8RecPt = g_DebugPack.u8RecPt%DEBUG_PACK_LEN;
		uint8 u8SendPt = g_DebugPack.u8SendPt_Mqtt%DEBUG_PACK_LEN;
		int8 i = MQTT_TR_BASE64_BLEN/sizeof(DEBUG_PACK_DATA);		/* 一次最多发送的数量 */

		/* 填充Mqtt Buf */
		uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
		uint8 u8QoS = 1;			/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
		/* 水位发布:打印Topic */
		PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
		PrintStringNoOvChk(&pTxBuf, "/DEBUG/PACK_DATA");
		uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
		if(u8QoS) {
			pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
		}
		/* 打印数据成Json格式 */
		*pTxBuf++ = '{';	/* Json开始 */
		PrintU32DatToJson(&pTxBuf, "width", DEBUG_PACK_FDAT_CHN + 1, 0);
		PrintStringNoOvChk(&pTxBuf, "\"base\":\"");
		for( ; (i > 0) && (u8SendPt != u8RecPt); i--) {
			pTxBuf = EncodeByBase64((uint8*)&g_DebugPack.Data[u8SendPt], sizeof(DEBUG_PACK_DATA), pTxBuf);
			u8SendPt = (u8SendPt+1)%DEBUG_PACK_LEN;
		}
		*pTxBuf++ = '"';
		*pTxBuf++ = '}';			/* Json结尾 */

		if(PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) {
			g_DebugPack.u8SendPt_Mqtt = u8SendPt;
			return TRUE;
		} else {
			return FALSE;
		}
	}
}

#define HEX_DISP_BUF_BLEN 	90
char g_cHexDispBuf[HEX_DISP_BUF_BLEN];
/*==========================================================================
| 该函数用于把收发Buf内容打印出来，以方便调试。意义在于减少调用DebugPrint()次数，以加快运行速度
| 这个变量仅用于PrintBufByMqtt()函数，以免占用太多stack空间
| pMsg为标题，
| u8MsgSeq为标题序号，只支持0-9, 之所以和pMsg分出来是为方便调用者不用去做字符串连接
| pBuf为待打印buf指针，
| uByteLen为长度
|=========================================================================*/
void PrintBufByMqtt(const uint8* msg, const uint8 u8MsgSeq, const uint8* pU8, uint16 uByteLen) 
{
	char* pDispBuf = g_cHexDispBuf;

	/* 打印消息名 */
	while (*msg) {
		*pDispBuf++ = *msg++;
	}
    if(u8MsgSeq < 10) {
    	*pDispBuf++ = '0' + u8MsgSeq;
    }

	/* 打印长度 */
	*pDispBuf++ = '(';
	PrintU32((uint8**)&pDispBuf, (uint8*)g_cHexDispBuf + HEX_DISP_BUF_BLEN, uByteLen, 0);
	*pDispBuf++ = 'B';
	*pDispBuf++ = ')';
	*pDispBuf++ = ':';

	/* 打印数据 */
	while (uByteLen) {
		/* 计算剩余Buf长度以及能打印的数据长度 */
		int16 iCurPrintDataByteLen = (HEX_DISP_BUF_BLEN
				- (pDispBuf - g_cHexDispBuf)) / 3;
		if (uByteLen < iCurPrintDataByteLen) {
			iCurPrintDataByteLen = uByteLen;
		}
		uByteLen -= iCurPrintDataByteLen;

		/* 打印数据 */
		for (; iCurPrintDataByteLen > 0; iCurPrintDataByteLen--) {
			uint8 u8Data = (*pU8) >> 4;
			if (u8Data < 10) {
				*pDispBuf++ = u8Data + '0';
			} else {
				*pDispBuf++ = u8Data + 'A' - 10;
			}

			u8Data = (*pU8) & 0x0F;
			if (u8Data < 10) {
				*pDispBuf++ = u8Data + '0';
			} else {
				*pDispBuf++ = u8Data + 'A' - 10;
			}
			*pDispBuf++ = ' ';
			pU8++;
		}

		/* 打印到屏幕 */
		pDispBuf--;
		*pDispBuf++ = 0;
		PrintStringByMqtt(g_cHexDispBuf);
		pDispBuf = g_cHexDispBuf; /* 准备生数据打印 */
	}
}

/*==========================================================================
| Description	: Mqtt打印，包括两个部分：
	a. 把待打印内容写进 g_MqttPrint.u8Buf，参考函数 PrintStringByMqtt()
	b. PubMqttPrint() 把g_MqttPrint.u8Buf内容通过mqtt发布出去
| G/Out var		:
| Author		: wangrenfei			Date	: 2020-7-29
\=========================================================================*/
/* 把要输出的内容放在Buf中，形成：,"***" 格式，以便发布程序容易形成json格式，本函数更多用于示例 */
BOOL PrintStringByMqtt(const char* pString)
{
	Swi_disable();
	if(g_MqttPrint.uBufPt >= MQTT_PRINT_BUF_BLEN - 3) {	/* 至少需要3byte */
		Swi_enable();
		return FALSE;
	} else {
		uint8* pBuf = g_MqttPrint.u8Buf + g_MqttPrint.uBufPt;
		*pBuf++ = ',';
		*pBuf++ = '"';
		BOOL bRes = PrintString(&pBuf, &g_MqttPrint.u8Buf[MQTT_PRINT_BUF_BLEN - 1], pString);/* 留一个字节用于最后填充" */
		*pBuf++ = '"';
		g_MqttPrint.uBufPt = pBuf - g_MqttPrint.u8Buf;
		Swi_enable();
		return bRes;
	}
}

/*==========================================================================
| Description	: 传输Mqtt 打印部分
| G/Out var		:
| Author		: wangrenfei			Date	: 2020-7-29
\=========================================================================*/
BOOL PubMqttPrint(MQTT_COMM* pMqttComm)
{
	if(g_MqttPrint.uBufPt == 0) {	/* 检查是否有消息 */
		return TRUE;
	}
	
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
	uint8 u8QoS = 1;			/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
	/* 水位发布:打印Topic */
	PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
	PrintStringNoOvChk(&pTxBuf, "/DEBUG/PRINT");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
	}
	/* 打印数据成Json格式 */
	*pTxBuf++ = '{';	/* Json开始 */
	PrintStringNoOvChk(&pTxBuf, "\"cont\":[");
	Swi_disable();
	memcpy(pTxBuf, &g_MqttPrint.u8Buf[1], g_MqttPrint.uBufPt - 1);	/* 第一个字节是',' */
	pTxBuf += g_MqttPrint.uBufPt - 1;
	g_MqttPrint.uBufPt = 0;
	Swi_enable();
	*pTxBuf++ = ']';			/* 和 "cont":[ 配对 */
	*pTxBuf++ = '}';			/* Json结尾 */

	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);
}

/******************************** FILE END ********************************/
