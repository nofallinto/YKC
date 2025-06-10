/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved
 *
 * File			: McuSupport_F103.c
 * Author		: Cui
 * Description	: MCU支持软件
 
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
#define _MSP_F103_C_		/* exclude redefinition */
/***************************************************************************
 						include files
***************************************************************************/
#include "DrvST.h"


/***************************************************************************
 						global variables definition
***************************************************************************/

/***************************************************************************
						internal functions declaration
***************************************************************************/
SECTION(".BootSection") void BootDefaultHandler(void)
{
	for(;;) {
	}
}
/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/
__attribute__((weak, alias("BootDefaultHandler"))) void NMI_Handler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void HardFault_Handler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void MemManage_Handler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void BusFault_Handler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void UsageFault_Handler(void);
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
__attribute__((weak, alias("BootDefaultHandler"))) void SVC_Handler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void DebugMon_Handler(void);
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
__attribute__((weak, alias("BootDefaultHandler"))) void PendSV_Handler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void SysTick_Handler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void WWDG_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void PVD_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TAMP_STAMP_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void RTC_WKUP_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void FLASH_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void RCC_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void EXTI0_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void EXTI1_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void EXTI2_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void EXTI3_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void EXTI4_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void DMA1_Channel1_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void DMA1_Channel2_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void DMA1_Channel3_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void DMA1_Channel4_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void DMA1_Channel5_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void DMA1_Channel6_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void DMA1_Channel7_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void ADC1_2_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void USB_HP_CAN1_TX_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void USB_LP_CAN1_RX0_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void CAN1_RX1_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void CAN1_SCE_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void EXTI9_5_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM1_BRK_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM1_UP_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM1_TRG_COM_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM1_CC_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM2_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM3_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM4_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void I2C1_EV_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void I2C1_ER_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void I2C2_EV_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void I2C2_ER_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void SPI1_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void SPI2_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void USART1_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void USART2_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void USART3_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void EXTI15_10_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void RTC_Alarm_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void USBWakeUp_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM8_BRK_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM8_UP_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM8_TRG_COM_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM8_CC_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void ADC3_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void FSMC_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void SDIO_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM5_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void SPI3_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void UART4_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void UART5_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM6_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void TIM7_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void DMA2_Channel1_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void DMA2_Channel2_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void DMA2_Channel3_IRQHandler(void);
__attribute__((weak, alias("BootDefaultHandler"))) void DMA2_Channel4_5_IRQHandler(void);
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
/* __attribute__((weak, alias("BootDefaultHandler"))) void Default_Handler(void);		 rsvd */
//.word BootRAM	        0xF1E0F85F

/***************************************************************************
						internal functions declaration
***************************************************************************/
/***************************************************************************
		Bootloader
***************************************************************************/
extern SECTION(".BootSection") void Reset_Handler(void);

const SECTION(".BootResetVecs") uint32 cnst_BootLoaderResetVectors[] = {	/* 中断向量表，需要放在0地址开始 */
    SRAM_ADDR_END,						/* stm32:_estack     STACK指向SRAM最后，如果RAM没有这么长，需要修改 */
    (uint32)(&Reset_Handler),				/* stm32:Reset_Handler */
    (uint32)(&NMI_Handler),
    (uint32)(&HardFault_Handler),
    (uint32)(&MemManage_Handler),
    (uint32)(&BusFault_Handler),
    (uint32)(&UsageFault_Handler),
    (uint32)(0),
    (uint32)(0),
    (uint32)(0),
    (uint32)(0),
    (uint32)(&SVC_Handler),
    (uint32)(&DebugMon_Handler),
    (uint32)(0),
    (uint32)(&PendSV_Handler),
    (uint32)(&SysTick_Handler),
 /* External Iterrupts */
    (uint32)(&WWDG_IRQHandler),                         /* Window WatchDog              */
    (uint32)(&PVD_IRQHandler),                          /* PVD through EXTI Line detection */
    (uint32)(&TAMP_STAMP_IRQHandler),                   /* Tamper and TimeStamps through the EXTI line */
    (uint32)(&RTC_WKUP_IRQHandler),                     /* RTC Wakeup through the EXTI line */
    (uint32)(&FLASH_IRQHandler),                        /* FLASH                        */
    (uint32)(&RCC_IRQHandler),                          /* RCC                          */
	(uint32)(&EXTI0_IRQHandler),
	(uint32)(&EXTI1_IRQHandler),
	(uint32)(&EXTI2_IRQHandler),
	(uint32)(&EXTI3_IRQHandler),
	(uint32)(&EXTI4_IRQHandler),
	(uint32)(&DMA1_Channel1_IRQHandler),
	(uint32)(&DMA1_Channel2_IRQHandler),
	(uint32)(&DMA1_Channel3_IRQHandler),
	(uint32)(&DMA1_Channel4_IRQHandler),
	(uint32)(&DMA1_Channel5_IRQHandler),
	(uint32)(&DMA1_Channel6_IRQHandler),
	(uint32)(&DMA1_Channel7_IRQHandler),
	(uint32)(&ADC1_2_IRQHandler),
	(uint32)(&USB_HP_CAN1_TX_IRQHandler),
	(uint32)(&USB_LP_CAN1_RX0_IRQHandler),
	(uint32)(&CAN1_RX1_IRQHandler),
	(uint32)(&CAN1_SCE_IRQHandler),
	(uint32)(&EXTI9_5_IRQHandler),
	(uint32)(&TIM1_BRK_IRQHandler),
	(uint32)(&TIM1_UP_IRQHandler),
	(uint32)(&TIM1_TRG_COM_IRQHandler),
	(uint32)(&TIM1_CC_IRQHandler),
	(uint32)(&TIM2_IRQHandler),
	(uint32)(&TIM3_IRQHandler),
	(uint32)(&TIM4_IRQHandler),
	(uint32)(&I2C1_EV_IRQHandler),
	(uint32)(&I2C1_ER_IRQHandler),
	(uint32)(&I2C2_EV_IRQHandler),
	(uint32)(&I2C2_ER_IRQHandler),
	(uint32)(&SPI1_IRQHandler),
	(uint32)(&SPI2_IRQHandler),
	(uint32)(&USART1_IRQHandler),
	(uint32)(&USART2_IRQHandler),
	(uint32)(&USART3_IRQHandler),
	(uint32)(&EXTI15_10_IRQHandler),
	(uint32)(&RTC_Alarm_IRQHandler),
	(uint32)(&USBWakeUp_IRQHandler),
	(uint32)(&TIM8_BRK_IRQHandler),
	(uint32)(&TIM8_UP_IRQHandler),
	(uint32)(&TIM8_TRG_COM_IRQHandler),
	(uint32)(&TIM8_CC_IRQHandler),
	(uint32)(&ADC3_IRQHandler),
	(uint32)(&FSMC_IRQHandler),
	(uint32)(&SDIO_IRQHandler),
	(uint32)(&TIM5_IRQHandler),
	(uint32)(&SPI3_IRQHandler),
	(uint32)(&UART4_IRQHandler),
	(uint32)(&UART5_IRQHandler),
	(uint32)(&TIM6_IRQHandler),
	(uint32)(&TIM7_IRQHandler),
	(uint32)(&DMA2_Channel1_IRQHandler),
	(uint32)(&DMA2_Channel2_IRQHandler),
	(uint32)(&DMA2_Channel3_IRQHandler),
	(uint32)(&DMA2_Channel4_5_IRQHandler),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0),
	(uint32)(0xF1E0F85F)		/* BootRAM */
};

/***************************************************************************
        片上存储: 用flash模拟eeprom
        目前仅支持配置数据，不支持Msg,Acq，flash单页是
***************************************************************************/

#include <stm32f1xx_hal_flash.h>
#include <stm32f1xx_hal_flash_ex.h>

/* Flash erase，为避免和ti的库函数重名，后缀加2
 * 会擦除以起始地址和长度所在的最小擦除区域
 * u32StartAddr:flash起始地址，
 * 				需要由调用者确定是4的整数倍地址,否则会把上一个邻接page也擦除掉 */
void FlashErase2(uint32 u32StartAddr, uint32 u32BLen)
{
	if(u32BLen > 0) {
		uint32_t flashEraseRet;
		FLASH_EraseInitTypeDef flashEraseInitType;
		flashEraseInitType.TypeErase = FLASH_TYPEERASE_PAGES;
		flashEraseInitType.PageAddress = u32StartAddr;
		flashEraseInitType.NbPages = (u32BLen+2047)/2048;	/* sizeof(page) == 2K */

		HAL_FLASH_Unlock();		/* Unlock the Flash to enable the flash control register access *************/
		if(HAL_FLASHEx_Erase(&flashEraseInitType, &flashEraseRet) == HAL_OK) {		/* HAL_FLASH_GetError() for detailed if needed*/
		}
		HAL_FLASH_Lock();
	}
}

/* 计算UID，返回u32ChipID[UID_U32LEN] */
void CalcUid(uint32 * pU32Uid)
{
    /* 芯片UniqueID地址 UID_BASE，不能memcpy直接用该地址，必须用volatile */
    volatile uint32 u32ChipAdd;
    u32ChipAdd = 0x1F;
    u32ChipAdd = u32ChipAdd*0x100 + 0xFF;
    u32ChipAdd = u32ChipAdd*0x100 + 0xF7;
    u32ChipAdd = u32ChipAdd*0x100 + 0xE8;
    volatile uint32* pU32 = (uint32*)u32ChipAdd;

    /* 4字节凑整填充0，12字节芯片ID */
    int8 i;
    pU32Uid[0] = 0;		/* 后面的EncryptWithAES需要buf长度为16字节的整数倍，所以不足的四字节凑0 */
    for(i = 1; i < UID_U32LEN; i++) {
    	pU32Uid[i] = pU32[i];
    }
}

/******************************** FILE END ********************************/
