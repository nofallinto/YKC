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
/* ����ϵͳͷ�ļ� */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

/* ��Ŀ����ͷ�ļ� */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"

/* �����Լ��¼�ģ��ͷ�ļ� */
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
| Description	: ��λ�������ڳ�ʼ��PLL��ִ��
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2017-04-18
\=========================================================================*/
void ResetFunc(void)
{
	/* SRAM����ZERO_INI��ʼ�����õ�ַ�����ȱ����.cmd/ld�ļ����������ZERO_INI��ַ�����ȶ�Ӧ */
	InitDataWithZero((uint8*)0x20000000, 0x00018000);
}

/*==========================================================================
| Description	: ģ���ʼ��
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2008-3-4
\=========================================================================*/
void InitSysMdl(void)
{
	/* ��ʼ������ */
	ChkAndWaitInitConf(SAVE_GRP_MCONF);	/* �ȴ�������� */

    /* ������������ */
    /* ����������g_PubSoftAuthCtr.u32FlashHash������Ϊ�˷����ȡ������ʱ��Լ132ms
       ����������Ѿ���������ˣ�������������飬�����Сʱ���ж�     */
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
//	uint32 u32FlashCrc = CalCRC32ForFileByIni((uint8*)FLASHADD_IVT_START, FLASH_IVT_BLEN, u32Key[0] ^ u32Key[1]);		/* �ж�����������sector */
//    u32FlashCrc = CalCRC32ForFileByIni((uint8*)FLASHADD_PROG_START, FLASH_PROG_BLEN, u32Key[0] ^ u32Key[1]);			/* ȥ���ж�������ģ��EEPROM������sector */
//	g_Sys.u32FlashHash_OrigEEAddr = u32FlashCrc ^ u32Key[0];
//	g_Sys.u32FlashHash_BackEEAddr = u32FlashCrc ^ u32Key[1];
#endif
	/* ��ʼ��GlobalVar.h����ı��� */
	/* <NULL> */

	/* ��ʼ������ӿڱ��� */
    g_Sys.tReboot_0Null_nUrgent_pWaitIdle = 0;

	/* ��ʼ���ڲ�ȫ�ֱ��� */
	/* <NULL> */

	/* ��ʼ���²�ģ�� */
	/* ����Ӳ�� */
}

/*==========================================================================
| Description	: ϵͳ����
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
		g_PubSoftAuthCtr.uPubSoftUpdateCnt = 1000;		/* TODO����ʱ��Ϊĸ�棺����������������� */
		g_PubSoftAuthCtr.uPubSoftInstallCnt = 1000;		/* TODO����ʱ��Ϊĸ�棺�Ӳ��������������������� */
		g_PubSoftAuthCtr.uPubSnCnt = 1000;				/* TODO����ʱ��Ϊĸ�棺����SN������ */
		g_PubSoftAuthCtr.uPubLicCnt = 1000;				/* TODO����ʱ��Ϊĸ�棺����License������ */
//	}
	g_Sys.SerialNo.u32Dat = 11111111;
	for(;;) {
    	ChkSoftIntegrity();
		if(g_DataAcsIntf.bConfHaveUnsavedChange[SAVE_GRP_NUL]) {	/* ���Ե�ʱ���ֶ����������洢 */
			g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt = 1;
			Semaphore_post(g_DataAccessBlock.Sem_Op);		//Semaphore_post(SEM_NvMemAcs);
		}
		Task_sleep(OS_TICK_KHz*100);
		/* ���� */
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
| Description	: ����������
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
		if(g_Sys.tReboot_0Null_nUrgent_pWaitIdle) {
			SysCtlReset();
		}
		RunMdlCtr();
		Semaphore_pend(g_SysBlock.SEM_Ctr, OS_TICK_KHz*10);			//Semaphore_pend(SEM_CtrTskRun, OS_TICK_KHz*40);

#if 0
		if(g_CodeTest.u32Val[91]) {
			g_CodeTest.u32Val[93] = 0xCC;
			/* ������һ��16K��sector��0x08000000�����°�����16Ksector��0x08080000�� */
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
| Description	: ����ʱ��
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-2-1
\=========================================================================*/
void vApplicationTickHook(void)		/* ԭ��TI void DriveTick_1KHz(void) */
{
	static uint8 s_u8Tmr_Ctr_ms = 0;

	/* û�б����ʼ���ı��� */
#if SUPPORT_NET
	DrvNetTick_1KHz();
#endif
	DrvSoftUpdateTick_1KHz();
#if (DEVICE_TYPE == V5_YBT3)
	DrvW5500Tick_1KHz();
#endif

	/* ����ȴ�ģ���ʼ����� */
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

	/* ģ���ʼ����main()����ɣ�������������־ */
	DrvUartTick_1KHz();

	if(s_u8Tmr_Ctr_ms) {
		s_u8Tmr_Ctr_ms--;
	} else {
		s_u8Tmr_Ctr_ms = PERIOD_CTR_TASK_ms - 1;
		Semaphore_post(g_SysBlock.SEM_Ctr);		//Semaphore_post(SEM_CtrTskRun);
	}
	
#if CPU_0ST_1TI == 0
	/* ��1Hz��ʱ����������ʱ�� */
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

/* ������������: ���sn����оƬID, �������Ƿ񱻸��� */
void ChkSoftIntegrity(void)
{
	Swi_disable();
    /* ��ʱ */
    if(g_Sys.iTimer_nSoftChkWaitTick_pErrCode < 0) {
        g_Sys.iTimer_nSoftChkWaitTick_pErrCode++;
        if(g_Sys.iTimer_nSoftChkWaitTick_pErrCode == 0) {
            /* �����һ�� */
            if((g_Sys.u32FlashHash_OrigEEAddr != cnst_BootLoaderConf.AcsMedia[0].u32OrigEEAddr)
                || (g_Sys.u32FlashHash_BackEEAddr != cnst_BootLoaderConf.AcsMedia[0].u32BackEEAddr))
            {
                g_Sys.iTimer_nSoftChkWaitTick_pErrCode |= 4;    /* b2����flashУ����� */
            }
        }
    }
    if(g_Sys.iTimer_nSoftChkWaitTick_pErrCode >= 0) {
        if(g_Sys.SerialNo.u32Dat < 10000000UL) {
            g_Sys.iTimer_nSoftChkWaitTick_pErrCode |= 1;    /* ��־b0 */
        } else {
            g_Sys.iTimer_nSoftChkWaitTick_pErrCode &= 6;    /* ���b0����Ϊֻ����3λ�����������Խ���6 */
        }
        if(g_Sys.SerialNo.u32ChipIdHash != GetChipIDHash()) {
            g_Sys.iTimer_nSoftChkWaitTick_pErrCode |= 2;    /* ��־b1 */
        } else {
            g_Sys.iTimer_nSoftChkWaitTick_pErrCode &= 5;    /* ���b1����Ϊֻ����3λ�����������Խ���5 */
        }
    }
	Swi_enable();
}

/*==========================================================================
* �����Ȩ����������ģ��, ����: JTAG�뿴�Ź�����������������Ȩ��������

  ���԰������а�
  	1. ���԰�: �Բ�ƷӲ�����в��ԣ���DI/DO, AD�ȣ�ͨ��JTAG�����ز������
  	2. ���а�: ����ʵ�ֲ�Ʒ���ܵ����, �ɲ������ͨ��������ȡ

  �����汾�����԰汾��JTAG����
  	1. ���԰汾��ָ�����׶Σ���Ҫ����JTAG�ڽ��е��ԣ�
  	2. �����汾��������Ա�������Ϊ�˷�ֹ����������Ҫ�Ͽ�JTAG
	3. JTAG�Ϳ��Ź�������һ���������Ź����Ͽ�JTAG, ι���źű���ȷ�϶Ͽ�JTAG
	   ���ڵ��԰汾���Ͽ�JTAG��Ҳ�Ͳ���ι����������Ӧ���߻�۲쵽�ⲿ���Ź��᲻�Ϸ���CPU���Դ���Ϊ��־

  �����Ȩ����������ɫ��Ҫ����:
  	1. ����Ա��������Ҫ�Ͳ���Ա����ͬһ����IP�£����������ݲ���Ա��IP�����жϣ�����¼��Ȩ����
  	2. ����Ա, ��������Ա�Ϸ�IP������ʱ�䴰��, �������
  	��������Ҫ��¼��������Ȩ��ȫ������

  �����������Ȩ������������:
	1. �����칫�����ز�������׶Σ����: оƬ���������ز��������������̾����óɲ�ͬ�Ĳ�Ʒ--��Ʒ�ͺ��ǳ������塣
	2. ���в���������Զ��Ͽ�JTAG, �������ڵ�ʱ�򣬾���������������к�--��Ҫ����Ա
	3. PCB���ӷ����������������и����ϻ����ϻ�����ʵ��̨�ϼ������ԣ�����쳣���ر��������򴢴档
	4. �ڰ�����Ҫʹ�õ�ʱ�򣬽����״ε����������--��Ҫ����Ա
	   ��������ɴ洢�����ĸ�徭��������ܴ�����Ŀ��壬��������ת�������洢���������
	5. ���������������Ȩ���������ó����ð�(��������)����ʽ��, --��Ҫ����Ա
  	   �������ð��ʹ�ó����жϣ�YKF3/YBT3����GPSʱ�ӣ�YBU2����������ʱ��
  	6. ���ڻ����Ȩ�����(���Ѿ�������)����������İ汾�����������Ա��Ԥ�������Ϳ����Զ�����

  ��ȫ���ϵ�:
  	1. ��������������ϴ��䴦�ڼ���״̬����һ�δ���һ�����룬�ػ�������ɽ���
  	2. ��������������: ���ݷ�������Ȩ�޷����������������
  		���ݷ�����: �����������ã��������ճ�ת�����ݵķ��������������Ҳ�ɸ÷���������ת��
  		Ȩ�޷�����: ������������Ȩ(���԰��ȡ���кţ����а����������Ȩ)����½�ķ�������ÿ���汾�����һ���̶��Ҳ����޸ĵķ�����
  					ĸ�巢�����Ҳ�Ǿ���Ȩ�޷�������ת�������ݷ������ٸ�������
  					һ��ĳ���汾�������й�ܣ�ͣ�øð汾�������Ȩ�����������Է�ֹ�ð汾����һ������
  		���������: ��ĸ�����������������������Ȩ����Ҫ�����������ǿ�������ĸ���ṩ���������Ȩ��ĸ����������ṩ����
		  			���������ĸ���ӳ���Ӧ�ⲿ��������������������Է�ֹ�����ƽ�
  	   ����
  	   	a. ����֪�����ܻ��ƣ�Ҳ������ģ���������������ƭȡ���
  	   	b. �������������δ����ȨҲ�޷�ʹ�ã�����Ҫ��½Ȩ�޷�����������Ȩ
  	3. ���Ȩ�޷����������ڴ����в�����ʾ�洢��������ֱ����һ�����������ַ�������
  	   ��Ҫ�������в������ɺ�����������������������ƽ��˼��ܣ�Ҳ�ò������������ͨ���ı���Ȩ����������ʽ������Ȩ��
  	   ��������:
	4. ÿһ������Ƭ�������ܣ������ַ���

  �����������(�����˹���Ԥ)���������²���:
	1. ���������
		a. RSA���ڴ�����Կ
		b. ASE���ڴ����
	2. �����Ƭ����: ÿ������Ƭ(1KB)�ֱ���ܣ�Ϊ�˼���ĸ��TCP/IPͨѶ��Դռ�ã�����ͨѶ������������ת��
	   ĸ�岻��ע�����������������Ԫ�Ǵ���Ƭ��������Ȩ�����ڴˣ�
	   ͬʱ���ٸ��������������ɷ���������; ͬʱ���ٸ�����Ƭ��������ĸ�����
		a. ��������Ƭ:
			a1. ĸ������RSA����N����Կ�ԣ���Կ���͸��������Թ����������������ģ�ÿ��RSA��Կ��Ӧһ������Ƭ
			a2. Ҫ�����Ŀ���������RSA��Կ������RSA���������AES���롢����AES������������������ַ����ǰ�汾������
				��ͬ�����Ķ�RSA�������һ�𷢲���ĸ��
			a3. ĸ������RSA˽Կ���AESͨѶ�������ַ������Ӧ�Ĵ���Ƭ����AES���ܣ����͸�����������������Ƭ���ͽ���
		b. ��������ѯ������°汾����������ɣ���Ҫ����ܵ�������ɣ������Զ�������������RSA�����
		c. ��ȫ���ϵ�:
			c1. ĸ�巢��RSA��Կ������RSA���ܽ����ֻ��ͨ����˾������(����������ַ��������)��ע:�������������,����MQTT����������
			c2. ÿһ������Ƭ�������Ҫ�������ַ����ǰ�汾����������ַ�����������ʱ��ĺ���(YBUû��ʱ��͹̶�)��ÿ���汾����һ��
				������������ĸ���ӳ���Ӧ
	3. flash������ֶ�д��: ����flash mirror���ܣ�Low Region�������У�High Region��������
		a. �����߶˵�ַ
		b. ����RSA��Կ���ɹ������: AES���롢����AES���ܵ�ǰ����������Ƭ��ַ����ǰ�汾������
	4. ��ǰ���а汾ѡ�������������
		a. ��鵱ǰ���������
		b. ���
	5. ���������ڲ�ͬ�汾��ת��
		a. �����������ݶ���ID,

  �������кŲ���
  1. �¿�����(�����к�)����ʱ���к�(����λ�����������λΪ0)��½ͨѶ�����������������к����󡱺�web������ͨ�������յ�������
  2. ����Աͨ���̶��Ĺ���ҳ���õ�½web�����������ݰ��Ӳ�ͬ--�ϰ��ӻָ����кţ��°����Զ��������к�
  	 ����Ա���еĲ���(���кš���Ȩ)��Ҫ����¼����Ҫ��ʱ����Թر�Ȩ��
  3. web�������˶��¿�������IP������ԱIP��һ�º󣬽��ղ���Ա��������к�(���߲������к�)�Լ���ʱ���кţ�ת����ĸ��
  4. ĸ�����ɼ��ܵġ����кš�����������ʱ���к�����Ӧ�����⣬�¿������ջ�����ݺ󣬸�����������к�
  
  ������Ȩ������
  1. �����п�����(�����кţ�������������Ȩ)��½ͨѶ�����������ύ����Ȩ���
  2. ����Աͨ���̶��Ĺ���ҳ���õ�½web��������������Ȩ��Ϣ(�������޾��ǵ���ʱ�䣬������������)
  	 ����Ա���еĲ���(���кš���Ȩ)��Ҫ����¼����Ҫ��ʱ����Թر�Ȩ��
  3. web�����������кš���Ȩ��Ϣ���͸�ĸ��
  4. ĸ������кš���Ȩ��Ϣ���ܺ�������Ӧ�Ŀ�������������������Ȩ���󣬸����������Ȩ��Ϣ
  	 �������̨��������δ����������web���������ĸ�̨����������Ȩ���⣬������Ӧ�����������ļ����ύ���ͻ���
  	 �ͻ�����ͨ���ļ����ء���Ļ����İ취������Ȩ��Ϣ���뵽��������

  �����������������ز�����
	1. ���°汾������ʱ��ע����°汾��ص�Ȩ�޷�����������ĸ������Ȩ�޷�������Ȩ�޷�����֪ͨ���ݷ���������׼���á�
	2. ���ݷ�������һ���������İ汾����������������֪ͨ��������������(���°汾��ʱ����Ҫ������)��
	3. ���ݷ��������յ�������������Ƭ�����롱��ʱ��ת����Ȩ�޷�������Ȩ�޷�������ת����ĸ��
	4. ĸ���յ�����Ƭ�������ʱ��������Ӧ�Ĵ���Ƭ����Ȩ�޷�����-���ݷ�����ת����������

  �豸�˷��ʽӿڰ���:
	1. ProcMqttUpdate() : Mqtt�����ܽӿ�
	2. IniSoftUpdate()	: ������������ʼ�������߼�������flash
	3. QueryAndPubFlashReq(): �������������������(������ַ����ǰ�汾���������)��
	4. UpdateSoft()	 	: ����������������ܴ���Ƭ��д������
	5. CreateFlashReq()	: ��������Ƭ����

  ĸ��˷��ʽӿڰ�����
  	1. ProcMqttAuth_SN()	: �������кţ�����web�������ύ����ʱ���к�����ʽ���кţ����ܺ�������Ӧ�����кŸ�����
	2. ProcMqttAuth_LIC()	: ���������Ȩ�룬����web�������ύ�����к������ޣ����ܺ�������Ӧ����Ȩ��
	3. ProcMqttAuth_SOFT() 	: ��������������ύ��������(������ַ����ǰ�汾���������)�������ܵĴ���Ƭ

  ͨѶ���(MQTT)�ӿڵ����߼���
	1. ���ӷ�������ʱ���ϱ������豸��Ϣ(�ͺš��汾�š����кš���Ȩ��)
	2. ���ӷ������ɹ�������ChkLicense(), ���License��Ҫ��Ȩ�����������Ȩ�����롱��������Ȩ�����롱��������������
	   �������ظ���Ȩ�룬����AuthLicense()������Ȩ
	3. �����У����������ݵ�1����ȡ�Ļ���汾��ȷ���Ƿ�Ҫ������
	   ���Ҫ�������򷢳�����֪ͨ���������˵õ�����֪ͨ������IniSoftUpdate()�����ɴ���Ƭ�����롣
	   �������ظ�����Ƭ�󣬵���UpdateSoft()�����������

  ĸ��ͨѶ�������ز�����
	1. �յ�Ȩ�޷�����ת������Ȩ���󣬵���PubAuthLic()��������Ȩ��
	2. �յ�Ȩ�޷�����ת���Ĵ���Ƭ���󣬵���PubSoftware()�����ɴ���Ƭ

  �豸��������������
	1. BootLoader()		: λ��flash��ͷ��һ�㲻�����������ϵ��ʱ���ɴ���ڣ��жϸ�ʹ���ĸ�����Ƭ��
						  �ж����ݣ����ȼ����������Ա�־���������������������ģ��Ǿ�ѡ��߰汾�����С�
						  ��������Ա�־��������������ʱ���ƻ�����������ɼ��û��������ϡ�
	2. ChkLicense()		: �����Ȩ

  ����������һ������ĸ��flash�д��뵽Ŀ����ӵĹ��̣�Ϊ�˰�ȫ���ǣ�������ƹ��̵���ʼ��ַ������ģ��ñȿ��Դ�ʱ�ӵ�����ʱ�俪ʼ���ƣ�12Сʱ�����ܻص���㣬��ȫ��������ɡ�

  BootLoader�㷨���豸mem����:

  TI:
  flash: 0x100000Byte, ��С������Ԫ0x4000, ����Ƭ����0x400(1KB)�����һ��flash�ܹ�32��������Ԫ��512������Ƭ
  eeprom:0x1800Byte,
  SRAM:  0x40000Byte����ǰ�������Ϊ:ZeroIniRam, RawRam, SysStack
  Ϊ�˱�֤BootLoader��С���ģ�������Լ����
  1. ��0��flash��С������Ԫ����װ��BootLoader�����������ڵ�1��flash��С������Ԫ֮��
     ������������µ�1��flash��С������Ԫ���Ժ��һ��flash�ռ䣬
     ��һ��flash��0����С������Ԫ�����ز��������ʱ����ɱ��
     ��һ��flash��0����С������Ԫ�����в��������ʱ����ɱ��
  2. �ϵ縴λ����ָ��BootLoader, ����жϺ���ת��_c_int00��_c_int00λ�ù̶��ڵ�1��flash��С������Ԫ��ʼλ��
  3. ���ڶ�ջ������ʱ��ָ���Ǽ��ٵģ���˶�ջָ��ָ��SRAM���

  ST:
  	A, F427:
  flash: 0x100000Byte, ��С������Ԫ���̶�������ݵ�ַ��������sector�ĳߴ�[16K/16K/16K/16K/64K/128K/128K/128K/128K/128K/128K/128K]
  eeprom:��flash��ģ��ģ��ߴ�Ϊ0x4000Byte��16K��,
  RAM:  0x10000000��ͷ��CCMRAM���ܳ���64K����ǰ�������Ϊ:CCMRAM_TEXT, CCMRAM_DATA�� (TODO:��δָ������������)
  	    0x20000000��ͷ��SRAM���ܳ���192K����ǰ�������Ϊ:ZeroIniRam, RawRam, SysStack
  flash�ṹ���£�
  1. ST��ʱû���Զ���bootloader
  2. ��һ��sector(16K)Ϊ�ж�������(IVT)
  2. �ڶ���sector(16K)Ϊģ��EEPROM��
  3. ��֮������пռ伴Ϊ���������֡�

	B, F103:
 flash: 0x100000Byte, ��С������Ԫ�̶���2KB
  eeprom:��flash��ģ��ģ��ߴ�Ϊ0x4000Byte��2K x 3��,
  	    0x20000000��ͷ��SRAM���ܳ���64K����ǰ�������Ϊ:ZeroIniRam, RawRam, SysStack
  flash�ṹ���£�
  2. ��1��sector(2K)Ϊ�ж�������(IVT)��bootloader
  2. ��2-4��sector(6K)Ϊģ��EEPROM��
  3. ���а汾�ţ�32λ������16λ������ʽʵ��У�� �� �汾�ţ�32λ������16λ������ʽʵ��У��
  4. ��֮������пռ伴Ϊ���������֡�


  ���ڶ�ջ������ʱ��ָ���Ǽ��ٵģ���˶�ջָ��ָ��SRAM���
\=========================================================================*/


/*==========================================================================
| Description	: ����������趨ʱ��
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
	if(g_MqttComm[MQTT_TYPE_PUB].GprsNewAdd.uRecvPingRespCnt) {
		g_MqttComm[MQTT_TYPE_PUB].GprsNewAdd.uRecvPingRespCnt--;
	}
	if(g_MqttComm[MQTT_TYPE_SUB].GprsNewAdd.uRecvPingRespCnt) {
		g_MqttComm[MQTT_TYPE_PUB].GprsNewAdd.uRecvPingRespCnt--;
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
				g_Sys.u32License_Seconds = 0;		/* ���кű仯����Ҫ������Ȩ */
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
			/* ����������������������� */
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
		if(g_Sys.SerialNo.u32Dat >= 10000000UL) {		/* ���������кŲ������� */
			pU8SubMsg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 1);
			if(CompareMsg(pU8SubMsg, "update_ver")) {
				pU8SubMsg = SkipCharInString(pU8SubMsg, pU8MsgEnd, '"', 2);
				uint16 uReqSoftVer = (pU8SubMsg[0]-'0')*1000 + (pU8SubMsg[2]-'0')*100 + (pU8SubMsg[4]-'0')*10 + (pU8SubMsg[5]-'0')*1;	/* ����Ŀ������汾�� */
				if(IniSoftUpdate(uReqSoftVer) && (!QueryAndPubFlashReq(pMqttComm, FALSE))) {
					return FALSE;
				}
			}
		}
	/* base64����뷽ʽ */
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
| Description	: ������������
| G/Out var 	:
| Author		: King			Date	: 2017-11-4
\=========================================================================*/
BOOL CreateFlashReq(FLASH_REQ* pFlashReq, uint32 u32MaxFlashPackBLen)
{
	/* ������Ĵ���Ƭ�Ƿ�ᳬ�� */
	uint32 u32FlashFragBLen = 0;
	if(u32MaxFlashPackBLen > 16) {
		u32FlashFragBLen = u32MaxFlashPackBLen - 16;
	}
	/* ����ʣ���ַ */
	uint32 u32RemainFlashBLen;
	if(g_SoftUpdateCtr.u32FlashEndAddr <= g_SoftUpdateCtr.u32FlashCurAddr) {
	#if LEGACY 
		u32RemainFlashBLen = FLASH_HALF_BLEN - g_SoftUpdateCtr.u32FlashCurAddr;
	#else
		u32RemainFlashBLen = FLASH_UPDATEABLE_BLEN - (g_SoftUpdateCtr.u32FlashCurAddr - FLASHADD_UPGRADABLE_START);		/* ��ʱu32RemainFlashBLen����˼�Ǿ���Ϊflash����߽�ĳ��� */
	#endif
	} else {
		u32RemainFlashBLen = g_SoftUpdateCtr.u32FlashEndAddr - g_SoftUpdateCtr.u32FlashCurAddr;			/* ��ʱu32RemainFlashBLen����˼�����������ʣ�೤�� */
	}
	if(u32RemainFlashBLen < u32FlashFragBLen)	{
		u32FlashFragBLen = u32RemainFlashBLen;		/* �������ʣ�����flash����߽�ĳ���С������Ƭ�γ��ȣ���˴�����ʹ���������߽�ĳ�����Ϊ���󳤶� */
	}
	u32FlashFragBLen = (u32FlashFragBLen/16)*16;	/* ����Ƭ���ȹ��Ϊ128bit */

	/* ����FlashReq */
	pFlashReq->u32FlashFragBLen = u32FlashFragBLen;
	pFlashReq->bWhichHalf_0Fst_1Sec = !CheckExeFlash_0Low_1High();		/* ����ǰ�������һ���汾��ע������˫bank�л���ʽ��д��ʱҲ��Ҫ���Ե�ַ */
	pFlashReq->u8Rand = (uint8)rand();
	pFlashReq->bSoft_Run1_Test0 = SOFT_RUN1_TEST0;
	pFlashReq->bIniSoftUpdate = (g_SoftUpdateCtr.u32FlashCurAddr == g_SoftUpdateCtr.u32FlashEndAddr);
	GetAuthKey(AUTH_KEY_FLASH_REQ_KEY, &pFlashReq->u32FlashReqKey);
	pFlashReq->u32FlashAddr = g_SoftUpdateCtr.u32FlashCurAddr;
	EncryptWithAES(AUTH_AES_FLASH_REQ, (uint32*)pFlashReq, FLASH_REQ_ENCRYPT_LEN, 0);	/* ���ܴ������� */

	/* �������Ƭ������ܵĴ���Ƭ���� */
	if(g_SoftUpdateCtr.u32FlashCurAddr >= g_SoftUpdateCtr.u32FlashEndAddr) {		/* �����ǰ��ַ�������յ�ַ���� */
		pFlashReq->uFlashFragNo = (g_SoftUpdateCtr.u32FlashCurAddr - g_SoftUpdateCtr.u32FlashEndAddr) / FLASH_FRAG_DEFAULT_BLEN;
	} else {		/* �����ǰ��ַ�Ѿ��Ƶ����յ�ַǰ�� */
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
	/* ��־�ȴ���ʱ�� */
	g_SoftUpdateCtr.uTmr_ReqWait_ms = MAX_FLASH_REQ_WAIT_TIME_ms;

	return TRUE;	/* ��ʵ����������������践�صģ�����Ϊ�˱�̷������� */
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
		CreateFlashReq(&FlashReq, MQTT_TR_BASE64_BLEN);	/* ������������ */
		uint8* pTxBuf = pMqttComm->u8TRBuf + 5; 				/* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
		uint8 u8QoS = 0;			/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
		/* ��������:��ӡTopic */
		PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
		PrintStringNoOvChk(&pTxBuf, "/AUTH/");
		*pTxBuf++ = '0' + (g_SoftUpdateCtr.uReqSoftVer/1000)%10;
		*pTxBuf++ = '0' + (g_SoftUpdateCtr.uReqSoftVer/100)%10;
		*pTxBuf++ = '0' + (g_SoftUpdateCtr.uReqSoftVer/10)%10;
		*pTxBuf++ = '0' + (g_SoftUpdateCtr.uReqSoftVer/1)%10;
		uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
		if(u8QoS) {
			pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
		}
		*pTxBuf++ = '{';	/* Json��ʼ */
		PrintBase64ToJson(&pTxBuf, "base", (uint8*)&FlashReq, FLASH_REQ_ENCRYPT_LEN); /* base64���� */
		*pTxBuf++ = ',';
		PrintU32DatToJson(&pTxBuf, "no", FlashReq.uFlashFragNo, 0);			/* ���к� */
		PrintU32DatToJson(&pTxBuf, "total", FlashReq.uFlashFragNum, 0);		/* �ܵĴ���Ƭ���� */
		pTxBuf--;
		*pTxBuf++ = '}';	/* Json��β */
		/* ���������������:���� */
		return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);		/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
	}
}

BOOL IniSoftUpdate(uint16 uSoftVersion)
{
	Swi_disable();
	if((g_SoftUpdateCtr.uReqSoftVer == 0) && (g_SoftUpdateCtr.u32FlashCurAddr < MAX_UPDATE_STATUS)) {   /* ��û�п�ʼ���� */
		g_SoftUpdateCtr.uReqSoftVer = uSoftVersion;
		Swi_enable();
		/* �ƻ���������Ա�־ */
		InvalidateAnotherSoft();
		/* ������Ƭflash�ĳ��򲿷֣�TI��ʱԼ30ms,����Ӱ��������� */
		FlashErase2(FLASHADD_PROG_START + (!CheckExeFlash_0Low_1High())*FLASH_HALF_BLEN, FLASH_PROG_BLEN);			/* ��ȥ�ж��������ģ��EEPROM�������汾֮������� */
		g_SoftUpdateCtr.u32FlashEndAddr = (FLASHADD_PROG_START + (((uint32)rand() % FLASH_PROG_BLEN)) + MAX_UPDATE_STATUS) & 0xFFFFFF80;		/* ����128bit(0xFFFFFF80)����, ����MAX_UPDATE_STATUS��Ϊ�������FLASHADD_PROG_STARTΪ0ʱ����MAX_UPDATE_STATUS��ͻ */
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
	/* �ж��Ƿ����������� */
	if((g_SoftUpdateCtr.uReqSoftVer == 0) || (g_SoftUpdateCtr.u32FlashCurAddr < MAX_UPDATE_STATUS)) {
		return FALSE;
	}
	
	/* ����Ƭ����, ���ݴ��� */
	DecryptWithAES(AUTH_AES_FLASH_DATA, pU32FlashPack, u32FlashPackBLen, g_SoftUpdateCtr.u32FlashCurAddr);
	BOOL bWhichHalf_0Fst_1Sec  = (pU32FlashPack[0] >> 24) == 1;		/* �������ֽ� */
	uint16 u32FlashFragBLen = u32FlashPackBLen - 16;				/* ��ȥFlashPack��ͷ����β����4��uint32 */
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

	/* �˶Դ����Ƿ���ȷ */
	if((u32FlashAddr == g_SoftUpdateCtr.u32FlashCurAddr) 
		&& ((u32FlashAddr - FLASH_BASE) < FLASH_HALF_BLEN)
		&& (u32ReqSoftVer == g_SoftUpdateCtr.uReqSoftVer)
		&& ((u32SerialNo == 0) || (u32SerialNo == g_Sys.SerialNo.u32Dat)))
	{
	    g_SoftUpdateCtr.u32LastFragAddr = u32FlashAddr;
	    /* д�����Ƭ */
	    if(FlashUpgradeFrag(bWhichHalf_0Fst_1Sec, u32FlashAddr, pU32FlashData, u32FlashFragBLen)) {
			/* ��ȫһ�£����޸�flash��ַ */
    		u32FlashAddr = g_SoftUpdateCtr.u32FlashCurAddr + u32FlashFragBLen;
    		if(u32FlashAddr >= (FLASH_BASE + FLASH_HALF_BLEN)) {		/* ���ܻؾ�0��0��ַ��BootLoader */
    			u32FlashAddr = FLASHADD_UPGRADABLE_START;
    		}
    		g_CodeTest.u32Val[67]++;	//0000003A
		} else {
		    g_SoftUpdateCtr.uReqSoftVer = 0;
            g_SoftUpdateCtr.u32FlashCurAddr = UPDATE_FAIL;      /* ����ʧ�� */
            g_CodeTest.u32Val[68]++;		//00000001
            return FALSE;
		}

		/* �ж��Ƿ����һ������Ƭ: ��������ַ��������һ������Ƭ */
		if(u32FlashAddr != g_SoftUpdateCtr.u32FlashEndAddr) {
			g_SoftUpdateCtr.u32FlashCurAddr = u32FlashAddr;
		} else {	/* �ж��Ƿ����һ������Ƭ: �ǣ���־��ɣ�У��д���Ƿ���ȷ��������ȫ��־ */
			/* ����У�� */
			if(0) {
				g_SoftUpdateCtr.u32FlashCurAddr = UPDATE_FAIL;		/* ����ʧ�� */
			} else {
				OnPostLiveUpdate();
				/* ��־������� */
				/* g_SoftUpdateCtr.uReqSoftVer = 0;   ���д������485����������£������������� */
				g_SoftUpdateCtr.u32FlashCurAddr = UPDATE_CMPLT;
				g_Sys.tReboot_0Null_nUrgent_pWaitIdle = 1;
			}
			return FALSE;	/* �Ѿ�������������Ͳ���Ҫ�ٲ���FlashReq�� */
		}
	} else {
        Rec_DebugChn(0, g_SoftUpdateCtr.u32LastFragAddr);
        Rec_DebugChn(0, g_SoftUpdateCtr.u32FlashCurAddr);
        Rec_DebugChn(0, u32FlashAddr);
        Rec_DebugChn(0, u32ReqSoftVer);
        Rec_DebugChn(0, u32SerialNo);
        return FALSE;
	}
	return TRUE;
}

/* ĸ�庯����������� */
uint32 PubSoftware(FLASH_REQ* pFlashReq, uint32* pU32FlashPack, uint32 u32MaxFlashPackBLen, uint32 u32SerialNo)
{
	if(u32MaxFlashPackBLen < 16) {		/* ������ */
		return 0;
	}
	
	/* ����FlashReq����ȷ�����õ������� */
	uint16 uPubSoftSupportPwdGrpNum = 1;
	if(g_PubSoftAuthCtr.u8PubSoftPwdGrpNum) {
		uPubSoftSupportPwdGrpNum = g_PubSoftAuthCtr.u8PubSoftPwdGrpNum;
	}
	FLASH_REQ FlashReq;
	uint16 uPwdGrpNo = 0;	/* ��������� */
	do {
		FlashReq = *pFlashReq;
		DecryptWithAES((AUTH_TYPE)(AUTH_AES_FLASH_REQ + AUTH_TYPE_NUM*uPwdGrpNo), (uint32*)(&FlashReq), FLASH_REQ_ENCRYPT_LEN, 0);
	} while((!ChkAuthKey((AUTH_TYPE)(AUTH_KEY_FLASH_REQ_KEY + AUTH_TYPE_NUM*uPwdGrpNo), &FlashReq.u32FlashReqKey)) 
			&& (++uPwdGrpNo < uPubSoftSupportPwdGrpNum));

	if((uPwdGrpNo < uPubSoftSupportPwdGrpNum)
	#if LEGACY
		 && ((BOOT_LOADER_BLEN <= FlashReq.u32FlashAddr) && (FlashReq.u32FlashAddr < FLASH_HALF_BLEN))) 	/* ��ַ��Χ */
	 #else
		&& ((FlashReq.u32FlashAddr >= FLASHADD_UPGRADABLE_START) && ((FlashReq.u32FlashAddr - FLASH_BASE) < FLASH_HALF_BLEN))) 
	 #endif
	{
        /* ���ܺϷ��Լ�� */
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
		/* ����:��ͬ�Ĵ���Ƭ */
	    Rec_DebugChn(1, FlashReq.u32FlashAddr);
	    Rec_DebugChn(1, RunTime_Interval(1)/120000);		/* TODO����Ƶ��һ�� */
		if(pFlashReq->uFlashFragNo && (pFlashReq->uFlashFragNo == g_PubSoftAuthCtr.uFlashFragNo)) {
		    g_CodeTest.uVal3[0]++;
		    g_CodeTest.uVal3[1] = pFlashReq->uFlashFragNo;
		}
		/* ��¼�������� */
        g_PubSoftAuthCtr.uFlashFragNo = pFlashReq->uFlashFragNo;
        g_PubSoftAuthCtr.uFlashFragNum = pFlashReq->uFlashFragNum;
    
		/* �������Ƭ���� */
		uint32 u32FlashFragBLen = FlashReq.u32FlashFragBLen;
		if((u32FlashFragBLen >= 10000000UL) || (u32FlashFragBLen%16 != 0)) {
			u32FlashFragBLen = FLASH_FRAG_DEFAULT_BLEN;
		}
		u32MaxFlashPackBLen -= 16;	/* ����ΪMaxFlashFragBLen, ��Ҫ�۳�FlashPack��ͷ����β����4��uint32 */
		if(u32FlashFragBLen > u32MaxFlashPackBLen) {
			u32FlashFragBLen = u32MaxFlashPackBLen;
		}
		
		/* ��䡢�����ɼ��ܵ������ */
		pU32FlashPack[0] = ((uint32)(((uint32)rand()<<8) + rand()) >> 8) | (FlashReq.bWhichHalf_0Fst_1Sec << 24);
		pU32FlashPack[1 + u32FlashFragBLen/4] = u32SerialNo;
		pU32FlashPack[2 + u32FlashFragBLen/4] = SOFTWARE_VER;
		pU32FlashPack[3 + u32FlashFragBLen/4] = FlashReq.u32FlashAddr;
		memcpy(&pU32FlashPack[1], (const void*)(FlashReq.u32FlashAddr + (FlashReq.bWhichHalf_0Fst_1Sec==1)*FLASH_HALF_BLEN), u32FlashFragBLen);
//		for(int i = 0; i < u32FlashFragBLen / 4; i++) {
//			pU32FlashPack[1 + i] = 0x11111111;
//		}
		u32FlashFragBLen += 16;		/* ����ΪFlashPackBLen����Ҫ����FlashPack��ͷ����β����4��uint32 */
//		printf("\r\n\r\n��");
//		uint8 *pU8Buffer = (uint8 *)pU32FlashPack;
//		for(int i = 0; i < u32FlashFragBLen; i++) {
//			printf("0x%x ", pU8Buffer[i]);
//		}
//		printf("��\r\n\r\n");
//		HAL_UART_Transmit(g_UartComm[0].Handle, (uint8 *)"\r\n\r\n��", 6, 300);
//		HAL_UART_Transmit(g_UartComm[0].Handle, (uint8 *)pU32FlashPack, u32FlashFragBLen, 1000);
//		HAL_UART_Transmit(g_UartComm[0].Handle, (uint8 *)"��\r\n\r\n", 6, 300);

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
	BOOL bBase64 = CompareMsg(pU8Msg, "base");		/* base64���� */
	pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 2);
    uint8* pMsgEnd_FlashReq = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 1) - 1;
    
	FLASH_REQ FlashReq;
	uint16 u32FlashPackBLen;
	uint32* pU32FlashPack;      /* ����������Mqtt��TRBuf��ΪFlashPack�������Ҫ�Ӻ���ǰ���� */
	BOOL bFlashReqSuc;
	
    /* У��������ʵ�ʵ�u32FlashPackBLen */
	if(bBase64) {
		pU32FlashPack = (uint32*)(pMqttComm->u8TRBuf + MQTT_TR_BUF_BLEN - MQTT_TR_BASE64_BLEN);
		DecodeByBase64(pU8Msg, pMsgEnd_FlashReq - pU8Msg, (uint8*)&FlashReq, FLASH_REQ_ENCRYPT_LEN);   /* base64ת��hex */
		pU8Msg = SkipCharInString(pMsgEnd_FlashReq, pU8MsgEnd, '"', 2);
		FlashReq.uFlashFragNo = ReadU32(&pU8Msg, pU8MsgEnd);
		FlashReq.uFlashFragNum = ReadU32(&pU8Msg, pU8MsgEnd);
		bFlashReqSuc = ((u32FlashPackBLen = PubSoftware(&FlashReq, pU32FlashPack, MQTT_TR_BASE64_BLEN, u32TopicSN)) != 0);
	} else {
		pU32FlashPack = (uint32*)(pMqttComm->u8TRBuf + MQTT_TR_BUF_BLEN - MQTT_FLASH_PACK_ASCII_BLEN);
		bFlashReqSuc = GetHexArray(pU8Msg, pMsgEnd_FlashReq, (uint8*)&FlashReq, FLASH_REQ_ENCRYPT_LEN);      /* ASCIIת��ΪHEX */
		FlashReq.uFlashFragNo = ReadU32(&pU8Msg, pU8MsgEnd);
		FlashReq.uFlashFragNum = ReadU32(&pU8Msg, pU8MsgEnd);
		bFlashReqSuc &= ((u32FlashPackBLen = PubSoftware(&FlashReq, pU32FlashPack, MQTT_FLASH_PACK_ASCII_BLEN, u32TopicSN)) != 0);
	}

	/* ��������Ƭ */
	if(bFlashReqSuc)	{
		uint8* pTxBuf = pMqttComm->u8TRBuf + 5;		/* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
		uint8 u8QoS = 0;			/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
		/* ��������:��ӡTopic */
		PrintStringNoOvChk(&pTxBuf, DEV_TYPE_MQTT_TOPIC_String);
		PrintU32(&pTxBuf, pTxBuf+16, u32TopicSN, 0);
		PrintStringNoOvChk(&pTxBuf, "/SUB/UPDATE/");
		*pTxBuf++ = '0' + (SOFTWARE_VER/1000)%10;
		*pTxBuf++ = '0' + (SOFTWARE_VER/100)%10;
		*pTxBuf++ = '0' + (SOFTWARE_VER/10)%10;
		*pTxBuf++ = '0' + (SOFTWARE_VER/1)%10;
		uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
		if(u8QoS) {
			pTxBuf += 2;	/* ���ݷ�������ϢQoS,Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
		}
		*pTxBuf++ = '{';
		if(bBase64) {
			PrintBase64ToJson(&pTxBuf, "base", (uint8*)pU32FlashPack, u32FlashPackBLen);	/* ��ĸ�巢���������Ƭ��XƬת��base64�� */
		} else {
			PrintHexToJson(&pTxBuf, "code", (uint8*)pU32FlashPack, u32FlashPackBLen);	/* ��ĸ�巢���������Ƭ��XƬת��ASCII�� */
		}
		*pTxBuf++ = '}';
		/* ���������������:���� */
		return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
	} else {
		return TRUE;
	}
}

/*==========================================================================
| Description	: �汾����
	GetSoftVersion():���flash���������汾
	FixSoftVerAndEEConf():ͨ��jtag�����������ʱ��eeprom������汾������ʵ�ʲ��������
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
| Description	: ���к����
	PubSerialNo_JSON():ĸ�巢�����кţ����ڽ�һ���ط����ã�������������
	UpdateSerialNo_JSON():�������������к�
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
	/* �ر�ĸ�� */
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
	g_PubSoftAuthCtr.uPubSnCnt--;		/* ����ط������жϣ��ɵ�����ڽ����жϣ������������� */
	
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5;	/* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
	uint8 u8QoS = 0;			/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
	/* ��������:��ӡTopic */
	PrintStringNoOvChk(&pTxBuf, DEV_TYPE_MQTT_TOPIC_String);
	PrintU32(&pTxBuf, pTxBuf+16, u32IniSerialNo, 0);
	PrintStringNoOvChk(&pTxBuf, "/SUB/UPDATE/SN");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
	}
	*pTxBuf++ = '{';
	PrintHexToJson(&pTxBuf, "code", (uint8*)(&SerialPack), sizeof(SERIAL_PACK));
	*pTxBuf++ = '}';
	
	/* �������кŽ������:���� */
	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
}

/*==========================================================================
| Description	: ��Ȩ����
	PubLicense():ĸ�巢����Ȩ�����ڽ�һ���ط����ã�������������
	UpdateLicense():������������Ȩ
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
	/* �ر�ĸ�� */
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
	g_PubSoftAuthCtr.uPubLicCnt--;		/* ����ط������жϣ��ɵ�����ڽ����жϣ������������� */

	uint8* pTxBuf = pMqttComm->u8TRBuf + 5;	/* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
	uint8 u8QoS = 0;			/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
	/* ��������:��ӡTopic */
	PrintStringNoOvChk(&pTxBuf, DEV_TYPE_MQTT_TOPIC_String);
	PrintU32(&pTxBuf, pTxBuf+16, u32DevSerialNo, 0);
	PrintStringNoOvChk(&pTxBuf, "/SUB/UPDATE/LICENSE");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
	}
	*pTxBuf++ = '{';
	PrintHexToJson(&pTxBuf, "code", (uint8*)(&LicensePack), sizeof(LICENSE_PACK));
	*pTxBuf++ = '}';
	
	/* ������Ȩ�������:���� */
	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
}

/*==========================================================================
| Description	: ��ͨ����<>ĸ���л�����
	ProcVersionMasterBCD()	: У������������ͨ�����л���ĸ��
	ProcMqttAuth_CMD()		: Mqttĸ��ָ��
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
	if(g_PubSoftAuthCtr.uTmr_IniPubCntNoPwd_ms && (uRegNum > 1)) {	/* ֻ��д��һ�� */
		/* ��BCD����ת����ͨ���� */
		g_PubSoftAuthCtr.uPubSoftInstallCnt = ConvertBCD2uint16(pRxBuf);
		pRxBuf += 2;
		g_PubSoftAuthCtr.uPubSoftUpdateCnt = ConvertBCD2uint16(pRxBuf);
		pRxBuf += 2;
		g_PubSoftAuthCtr.uPubSnCnt = ConvertBCD2uint16(pRxBuf);
		pRxBuf += 2;
		g_PubSoftAuthCtr.uPubLicCnt = ConvertBCD2uint16(pRxBuf);
		g_PubSoftAuthCtr.uTmr_IniPubCntNoPwd_ms = 0;		/* ����д��һ�� */
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
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5;	/* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
	uint8 u8QoS = 0;			/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
	/* ��������:��ӡTopic */
	PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
	PrintStringNoOvChk(&pTxBuf, "/RES/AUTH");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
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
	
	/* ĸ��״̬����:���� */
	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
}

///////////////////////////////////////////////////////////////////////////////////
//	����Ϊ����Ӧ�ù���
///////////////////////////////////////////////////////////////////////////////////
/* ���ڵ���,��������ʱ�� */
#define CPU_RUN_TIME_CALI_CHN	8					/* CPU����ʱ�����ͨ�� */
typedef struct {
	uint32 u32LastCPUTime[CPU_RUN_TIME_CALI_CHN];
	uint32 u32IntervalCnt[CPU_RUN_TIME_CALI_CHN];
}CPU_RUN_TIME_VAR;
CPU_RUN_TIME_VAR g_CpuRunTime;
/*==========================================================================
| Description	: ����CPU����ʱ�����
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
| Description	: ������ǰ��CPU����ʱ�����
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
| Description	: �������м��ʱ��
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
| Description	: ͨ�õ��Թ��ߣ���Ҫ�������¼��ֹ��ߣ�
	1. ��������¼��, DebugChn��ͨ���٣����洢���һ����������������ϣ���ccs����ֱ�ӹ۲��ڴ�Ϳ���
	2. ��������¼��, DebugPack,ͨ���࣬���洢��ǳ����Ҫ����������ϼ�ʱ����(RS485, MQTT)����Ҫ��Ӧ������ж���������
	3. Mqtt��ӡ���ѵ�����Ϣͨ��Mqtt��ӡ����
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
#define DEBUG_PACK_LEN	120		/* mqttһ���Ӵ���һ�Σ�ÿ����ഫ�� Լ4KB*3/4 = 3KB */
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
	/* ͨ��mqtt�������񣬴�ӡ������Ϣ��ÿ����ͳһ���һ�� */
	uint16 uBufPt;		/* д��ָ�� */
	uint8 u8Buf[MQTT_PRINT_BUF_BLEN];	/* ÿһ����Ϣ��ʽΪ ,"ccc"���Ա���װ��json c���ַ�,���ú�'"'   */
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
 * DebugChn���ڼ�¼�����������ݣ�����Buf�����˿���ֱ���ڴ��в鿴����
 * a. ͨ�� Rec_DebugChn()��¼���ݣ�
 * b. CopyDebugChnToTxBuf()�����ݿ�����modbus TxBuf��
 *==========================================================================*/
void Rec_DebugChn(uint8 u8RecChn, int32 i32Dat)
{
	DEBUG_CHN* pDebugChn = &g_DebugChn[u8RecChn];
	if((u8RecChn < DEBUG_CHN_NUM) && (pDebugChn->u32RecPt < DEBUG_CHN_LEN)) {
		pDebugChn->i32Dat[pDebugChn->u32RecPt] = i32Dat;
		pDebugChn->u32RecPt++;
	}
}
/* ����DebugChn���ݵ�Modbus TxBuf, �������˳�� */
void CopyDebugChnToTxBuf(uint8** ppTxBuf, uint16 uRegOff)
{
#if 0
	uRegOff = uRegOff/DEBUG_CHN_LEN;	/* ָ��Chnͨ�� */
	
	if(uRegOff < DEBUG_CHN_NUM) {		/* ������ */
		uint8* pTxBuf = *ppTxBuf;
		DEBUG_CHN* pDebugChn = &g_DebugChn[uRegOff];
		int16 iRecPt = pDebugChn->i32RecPt - 1;		/* pDebugChn->uRecPtָ����Ǵ���λ�ã�iRecPtָ��������ݵ�λ�� */
		if(iRecPt < 0) {
			iRecPt = DEBUG_CHN_LEN - 1;
		}

		/* �������ݽ�TxBuf */
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

/* ��DebugChn����ͨ��mqtt���� */
BOOL PubDebugChn(MQTT_COMM* pMqttComm, uint8 u8DebugChnNo)
{
	/* ���Mqtt Buf */
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
	uint8 u8QoS = 1;						/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
	/* ˮλ����:��ӡTopic */
	PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
	PrintStringNoOvChk(&pTxBuf, "/DEBUG/CHN_DATA");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
	}
	/* ��ӡ���ݳ�Json��ʽ */
	*pTxBuf++ = '{';	/* Json��ʼ */
	if(u8DebugChnNo >= DEBUG_CHN_NUM) {
		u8DebugChnNo = 0;
	}
	PrintU32DatToJson(&pTxBuf, "chn_no", u8DebugChnNo, 0);
	if(sizeof(g_DebugChn[u8DebugChnNo]) < MQTT_TR_BASE64_BLEN) {	/* �������ǲ��ᳬ�ģ����Է���һ */
		PrintStringNoOvChk(&pTxBuf, "\"base\":\"");
		pTxBuf = EncodeByBase64((uint8*)&g_DebugChn[u8DebugChnNo], sizeof(g_DebugChn[u8DebugChnNo]), pTxBuf);
		*pTxBuf++ = '"';
	}
	*pTxBuf++ = '}';			/* Json��β */

	if(PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) {
		InitDataWithZero((uint8*)(&g_DebugChn[u8DebugChnNo]), sizeof(g_DebugChn[u8DebugChnNo]));
		g_DebugChn[u8DebugChnNo].u32RecPt = 0;	/* ��λ���������¿�ʼ�ɼ� */
		return TRUE;
	} else {
		return FALSE;
	}
}

/*===========================================================================
 * DebugPack���ڼ�¼�����������ݣ�����Buf��ǳ(���洢1.5s��������)����˱��벢ͨ��ͨѶ�ӿ�(RS485,MQTT)��ʱ��������
 * a. ͨ��Rec_DebugPack()��¼���ݣ�
 * b. TryDebugPack()��RunDebugPack()ͨ��485�ӿڷ�������
 * c. PubDebugPack()ͨ��mqtt�ӿڷ�������
 * RS485��ȡ�ٶȣ�115200bps����11.52KB������ͨѶ�����������ܹ�����6KB���ݣ���6K/(12*4) = 125����¼
 * MQTT�����ٶ�: һ���Ӵ���һ�Σ�ÿ����ഫ��Լ4KB*3/4=3KB�������¼����3K/(12*4) = 62.5��
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
/* �յ�ָ��: bRecSW SendPt
	����DebugPack���ݵ�Modbus TxBuf, �������˳��:
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
		pUartComm->uTimer_WatchDog_ms = 0;		/* ͨѶ�ɹ� */
		pUartComm->bUartFuncTrySuc = TRUE;

		/* ���ݴ��� */
		uint32* pU32TxBuf = (uint32*)&pUartComm->u8TRBuf[4];	/* 32b���� */
		g_DebugPack.bRecSW = pUartComm->u8TRBuf[0];
		if(g_DebugPack.bRecSW == 0) {
			g_DebugPack.u8RecPt = 0;
			pUartComm->u8TRBuf[0] = 0;						/* ���ʵ�ʷ��͵����� */
			pUartComm->u8TRBuf[1] = 0;						/* �����һ������ָ�� */
			pUartComm->u8TRBuf[2] = DEBUG_PACK_FDAT_CHN+1;	/* ������ݿ�� */
		} else {
			/* ������ݲ���:Modbus TxBuf�����255byte */
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
			/* ����ͷ:����2bytes */
			pUartComm->u8TRBuf[0] = u8MaxTxGrpNum - i;		/* ���ʵ�ʷ��͵����� */
			pUartComm->u8TRBuf[1] = u8SendPt;		/* �����һ������ָ�� */
			pUartComm->u8TRBuf[2] = DEBUG_PACK_FDAT_CHN+1;	/* ������ݿ�� */
			g_DebugPack.u8SendPt_RS485 = u8SendPt;
		}
		TxDataForModbusRTU(u8UartPort, (uint8*)pU32TxBuf);
	}
}

/* ��DebugPack����ͨ��mqtt���� */
BOOL PubDebugPack(MQTT_COMM* pMqttComm)
{
	if(g_DebugPack.u8SendPt_Mqtt == g_DebugPack.u8RecPt) {
		return TRUE;
	} else {
		/* ׼������ */
		uint8 u8RecPt = g_DebugPack.u8RecPt%DEBUG_PACK_LEN;
		uint8 u8SendPt = g_DebugPack.u8SendPt_Mqtt%DEBUG_PACK_LEN;
		int8 i = MQTT_TR_BASE64_BLEN/sizeof(DEBUG_PACK_DATA);		/* һ����෢�͵����� */

		/* ���Mqtt Buf */
		uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
		uint8 u8QoS = 1;			/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
		/* ˮλ����:��ӡTopic */
		PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
		PrintStringNoOvChk(&pTxBuf, "/DEBUG/PACK_DATA");
		uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
		if(u8QoS) {
			pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
		}
		/* ��ӡ���ݳ�Json��ʽ */
		*pTxBuf++ = '{';	/* Json��ʼ */
		PrintU32DatToJson(&pTxBuf, "width", DEBUG_PACK_FDAT_CHN + 1, 0);
		PrintStringNoOvChk(&pTxBuf, "\"base\":\"");
		for( ; (i > 0) && (u8SendPt != u8RecPt); i--) {
			pTxBuf = EncodeByBase64((uint8*)&g_DebugPack.Data[u8SendPt], sizeof(DEBUG_PACK_DATA), pTxBuf);
			u8SendPt = (u8SendPt+1)%DEBUG_PACK_LEN;
		}
		*pTxBuf++ = '"';
		*pTxBuf++ = '}';			/* Json��β */

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
| �ú������ڰ��շ�Buf���ݴ�ӡ�������Է�����ԡ��������ڼ��ٵ���DebugPrint()�������Լӿ������ٶ�
| �������������PrintBufByMqtt()����������ռ��̫��stack�ռ�
| pMsgΪ���⣬
| u8MsgSeqΪ������ţ�ֻ֧��0-9, ֮���Ժ�pMsg�ֳ�����Ϊ��������߲���ȥ���ַ�������
| pBufΪ����ӡbufָ�룬
| uByteLenΪ����
|=========================================================================*/
void PrintBufByMqtt(const uint8* msg, const uint8 u8MsgSeq, const uint8* pU8, uint16 uByteLen) 
{
	char* pDispBuf = g_cHexDispBuf;

	/* ��ӡ��Ϣ�� */
	while (*msg) {
		*pDispBuf++ = *msg++;
	}
    if(u8MsgSeq < 10) {
    	*pDispBuf++ = '0' + u8MsgSeq;
    }

	/* ��ӡ���� */
	*pDispBuf++ = '(';
	PrintU32((uint8**)&pDispBuf, (uint8*)g_cHexDispBuf + HEX_DISP_BUF_BLEN, uByteLen, 0);
	*pDispBuf++ = 'B';
	*pDispBuf++ = ')';
	*pDispBuf++ = ':';

	/* ��ӡ���� */
	while (uByteLen) {
		/* ����ʣ��Buf�����Լ��ܴ�ӡ�����ݳ��� */
		int16 iCurPrintDataByteLen = (HEX_DISP_BUF_BLEN
				- (pDispBuf - g_cHexDispBuf)) / 3;
		if (uByteLen < iCurPrintDataByteLen) {
			iCurPrintDataByteLen = uByteLen;
		}
		uByteLen -= iCurPrintDataByteLen;

		/* ��ӡ���� */
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

		/* ��ӡ����Ļ */
		pDispBuf--;
		*pDispBuf++ = 0;
		PrintStringByMqtt(g_cHexDispBuf);
		pDispBuf = g_cHexDispBuf; /* ׼�������ݴ�ӡ */
	}
}

/*==========================================================================
| Description	: Mqtt��ӡ�������������֣�
	a. �Ѵ���ӡ����д�� g_MqttPrint.u8Buf���ο����� PrintStringByMqtt()
	b. PubMqttPrint() ��g_MqttPrint.u8Buf����ͨ��mqtt������ȥ
| G/Out var		:
| Author		: wangrenfei			Date	: 2020-7-29
\=========================================================================*/
/* ��Ҫ��������ݷ���Buf�У��γɣ�,"***" ��ʽ���Ա㷢�����������γ�json��ʽ����������������ʾ�� */
BOOL PrintStringByMqtt(const char* pString)
{
	Swi_disable();
	if(g_MqttPrint.uBufPt >= MQTT_PRINT_BUF_BLEN - 3) {	/* ������Ҫ3byte */
		Swi_enable();
		return FALSE;
	} else {
		uint8* pBuf = g_MqttPrint.u8Buf + g_MqttPrint.uBufPt;
		*pBuf++ = ',';
		*pBuf++ = '"';
		BOOL bRes = PrintString(&pBuf, &g_MqttPrint.u8Buf[MQTT_PRINT_BUF_BLEN - 1], pString);/* ��һ���ֽ�����������" */
		*pBuf++ = '"';
		g_MqttPrint.uBufPt = pBuf - g_MqttPrint.u8Buf;
		Swi_enable();
		return bRes;
	}
}

/*==========================================================================
| Description	: ����Mqtt ��ӡ����
| G/Out var		:
| Author		: wangrenfei			Date	: 2020-7-29
\=========================================================================*/
BOOL PubMqttPrint(MQTT_COMM* pMqttComm)
{
	if(g_MqttPrint.uBufPt == 0) {	/* ����Ƿ�����Ϣ */
		return TRUE;
	}
	
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
	uint8 u8QoS = 1;			/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
	/* ˮλ����:��ӡTopic */
	PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
	PrintStringNoOvChk(&pTxBuf, "/DEBUG/PRINT");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
	}
	/* ��ӡ���ݳ�Json��ʽ */
	*pTxBuf++ = '{';	/* Json��ʼ */
	PrintStringNoOvChk(&pTxBuf, "\"cont\":[");
	Swi_disable();
	memcpy(pTxBuf, &g_MqttPrint.u8Buf[1], g_MqttPrint.uBufPt - 1);	/* ��һ���ֽ���',' */
	pTxBuf += g_MqttPrint.uBufPt - 1;
	g_MqttPrint.uBufPt = 0;
	Swi_enable();
	*pTxBuf++ = ']';			/* �� "cont":[ ��� */
	*pTxBuf++ = '}';			/* Json��β */

	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);
}

/******************************** FILE END ********************************/
