/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved
 *				: 
 * File			: BoardSupport.h
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

/***************************************************************************
 				exclude redefinition and c++ environment
***************************************************************************/
#ifndef _BOARD_SUPPORT_H_
#define _BOARD_SUPPORT_H_

#ifdef __cplusplus
extern "C" {
#endif


/***************************************************************************
 							include files
***************************************************************************/
#include "LibMath.h"

/***************************************************************************
				hardware parameter and global definitions
***************************************************************************/


/***************************************************************************
 		use EXT to distinguish stencil.c or others source files
***************************************************************************/
#ifdef _BOARD_SUPPORT_C_
#define EXT
#else
#define EXT extern
#endif


/***************************************************************************
 					variables could be used as extern
***************************************************************************/
#define DEVICE_TYPE_String					"YKC"
#define DEV_TYPE_MQTT_TOPIC_String			"YKC/"

#define FLASH_HALF_BLEN						0x00040000UL							/* 256*1024 */
#define MCU_FLASH_END						(0x08000000 + FLASH_HALF_BLEN*2 - 1)	/* stm32��flash������ַ�겻׼(427��1m��2m����)��������Ҫ�Զ���,YKC�õ�F103ZET6,1M����flash */

#define SRAM_ADDR_END						0x20010000UL			/* 10000 == 64KB, F103 */

#define USE_SELF_DEVELOP_SHELL		TRUE			/* ʹ��������� */
#define USE_SELF_DEVELOP_MOTOR		TRUE			/* ���е�� */
/*===========================================================================
 * ��������
 *==========================================================================*/
#define UART_COMM_NUM						3
#define MAX_UART_NUM       					3
#define ON_INDICATOR_RS485(uRS485Port)	
#define OFF_INDICATOR_RS485(uRS485Port)	

#define MOTOR_RIGHT 						0
#define MOTOR_LEFT  						1

#define I2C_COMM_NUM						1
#define SPI_COMM_NUM						1

/*===========================================================================
 * GPIO����
 *==========================================================================*/
#define MAX_DIN_NUM					14		/* �߼��������� */
#define MAX_RELAY_NUM				2
#define GPIO_KICK_DOG_No			(0)
#define GPIO_DIN_StartNo			(GPIO_KICK_DOG_No + 1)
#define GPIO_RELAY_StartNo			(GPIO_DIN_StartNo + MAX_DIN_NUM)
#define GPIO_INDICATOR_StartNo		(GPIO_RELAY_StartNo + MAX_RELAY_NUM)
#define GPIO_TEST_StartNo			(GPIO_INDICATOR_StartNo + 0)

#define KickExtDog()				GPIO_toggle(GPIO_KICK_DOG_No)
#define OnRelay(uRelayNo)			GPIO_write(uRelayNo + GPIO_RELAY_StartNo, 1);
#define OffRelay(uRelayNo)			GPIO_write(uRelayNo + GPIO_RELAY_StartNo, 0);
#define OnIndicator(uIndicator)		GPIO_write(uIndicator + GPIO_INDICATOR_StartNo, 0)
#define OffIndicator(uIndicator)	GPIO_write(uIndicator + GPIO_INDICATOR_StartNo, 1)
#define	SetTestOut(TestOutNo, bHigh1_Low0)	GPIO_write(TestOutNo + GPIO_TEST_StartNo, bHigh1_Low0)
#define ToggleTestOut(TestNo)		GPIO_toggle(TestNo + GPIO_TEST_StartNo)
#define BurrTestOut(TestNo)			{GPIO_write(TestNo + GPIO_TEST_StartNo, 1); Boot_SysCtlDelay(1000); GPIO_write(TestNo + GPIO_TEST_StartNo, 0);}

#define OFF_INDICATOR_SERVER
#define ON_INDICATOR_SERVER

EXT uint16 g_uDInFilt[MAX_DIN_NUM];

/*===========================================================================
 * ������
 *==========================================================================*/
#define NULL_RelayNo				0xFF
#define TOP_WATER_JET_RelayNo		(GPIO_RELAY_StartNo + 0)
#define BOT_WATER_JET_RelayNo		(GPIO_RELAY_StartNo + 1)
#define BLUE_LED_DOutNo				(GPIO_RELAY_StartNo + 2)
#define RED_LED_DOutNo				(GPIO_RELAY_StartNo + 3)
#define GREEN_LED_DOutNo			(GPIO_RELAY_StartNo + 4)
#define OTHER_DEV_DOutNo			(GPIO_RELAY_StartNo + 5)
#define BMI270_CS(n) 				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, n)			/* IMUģ��Ƭѡ�ܽ� */

/*===========================================================================
 * ������, NULL_DInNo���ⶼ�������GPIO_DIN_StartNo������
 *==========================================================================*/
#define NULL_DInNo					0xFF
#define START_STOP_CMD_DInNo		(1 - 1)
#define LOW_LIQUID_WRN_DInNo		(2 - 1)
#define HIT_SIDE_L_FRONT_DInNo		(3 - 1)
#define HIT_SIDE_L_REAR_DInNo		(4 - 1)
#define HIT_SIDE_R_FRONT_DInNo		(5 - 1)
#define HIT_SIDE_R_REAR_DInNo		(6 - 1)
#define HIT_FRONT_LEFT_DInNo		(7 - 1)
#define HIT_FRONT_RIGHT_DInNo		(8 - 1)
#define HIT_REAR_LEFT_DInNo			(9 - 1)
#define HIT_REAR_RIGHT_DInNo		(10 - 1)
#define CLIFF_FRONT_LEFT_DInNo		(11 - 1)
#define CLIFF_FRONT_RIGHT_DInNo		(12 - 1)
#define CLIFF_REAR_LEFT_DInNo		(13 - 1)
#define CLIFF_REAR_RIGHT_DInNo		(14 - 1)


/*===========================================================================
 * �˷� & AD & Cap
 *==========================================================================*/
#define ADC_BIT 					12U				/* ADλ�� */
#define ADC_RANGE					3.3f				/* ADRng AD����,��λV */
#define ADC_MAX_RESULT				((1<<ADC_BIT)-1)	/* ADMax AD����������Ӧ������ */

#define COEF_k_VAL2DCPWRVOL			((ADC_RANGE*8.52f)/ADC_MAX_RESULT)      /* 110K/10K */
#define COEF_k_VAL2MOTORCUR			(1.08225f*ADC_RANGE/ADC_MAX_RESULT)

/* ֱ���źŶ��� */
#define DCPWR_VOL_DCSigNo       0
#define RIGHT_CUR_DCSigNo       1
#define LEFT_CUR_DCSigNo        2
#define CPU_TEMP_DCSigNo        3
#define VREF_INT_DCSigNo        4
#define DC_TOTAL_CHN        	5		/* ���ֱ���ź����� */

#ifdef _LIB_MATH_C_
const ANALOG_SIG_CORR cnst_DCSigFixCorr[] = {			/* �۳�CPU�¶�ͨ�� */
	{0, COEF_k_VAL2DCPWRVOL},
	{0.2/COEF_k_VAL2MOTORCUR, COEF_k_VAL2MOTORCUR},	
	{0.2/COEF_k_VAL2MOTORCUR, COEF_k_VAL2MOTORCUR},
};
#else
extern const ANALOG_SIG_CORR cnst_DCSigFixCorr[];
#endif

typedef struct {
    uint32 u32Adc1Res[5];
}ADC_DMA_BUF;
EXT ADC_DMA_BUF g_AdcDmaBuf;

/* ģ���źŴ��� */
#define ANALOG_ADSAMP_BUF_LEN		180
typedef struct {
    /* ����5�������������˳���壬���ܷ���DAT_UNI_REC�ṹ�壬ʹ����Ӧ�ĺ������� */
    float32         fSig_avr;           /* �������ݾ�ֵ */
    float32         fSig_rms;           /* �������ݾ�����(rms)ֵ */
    uint16          uSampOkDecCnt;      /* �������������Լ����������ı����Ƶ�ʺ����������������ü�������Ϊ0ʱ������������� */
    uint16          uCurSampPt;         /* ����ָ�� */
    uint16          uSampleBuf[ANALOG_ADSAMP_BUF_LEN]; /* ANALOG_ADSAMP_BUF_LEN ��Ϊ32�������� */
    /* �������� */
}ANALOG_ADSAMP_VAR;
EXT SECTION(".NOT_ZeroInit") ANALOG_ADSAMP_VAR g_DCSigADSamp[DC_TOTAL_CHN];

#define CAP_CLK_FREQ_Hz         1000000.0f  /*  */
/* ��ˢ���������ת��8400�����Ч�ʵ�ת��6889��Լ140Hz��ÿתһȦԼ16�����壬������Ƶ��Լ2.4KHz */
#define BUMP_MOTOR_PulseNo      0
#define RIGHT_MOTOR_PulseNo     1
#define LEFT_MOTOR_PulseNo      2
#define PULSE_SIG_CHN		3
#define PULSE_CYCLE_BUF_LEN		150
typedef struct {
	/* ����7�������������˳���壬���ܷ��� DAT_UNI_REC �ṹ�壬ʹ����Ӧ�ĺ������� */
	float32 		fSig_avr;			/* �������ݾ�ֵ */
	float32 		fSig_rms;			/* �������ݾ�����(rms)ֵ */
	uint16			uSampOkDecCnt;		/* �������������Լ����������ı����Ƶ�ʺ����������������ü�������Ϊ0ʱ������������� */
	uint16			uCurSampPt;			/* ����ָ�� */
	uint16 			uCycleBuf[PULSE_CYCLE_BUF_LEN]; 	/* �ź����ڻ��� */
	uint16 			uPulseEdge;			/* �źű�����Ϣ */
	BOOL 			bPulseTimeOut;		/* �źű��س�ʱ */
	uint8			u8Rsvd;
}PULSE_CAP_VAR;
EXT SECTION(".NOT_ZeroInit") PULSE_CAP_VAR g_PulseCap[PULSE_SIG_CHN];

typedef struct {
	uint32 u32SclBank;
	uint32 u32SclPin;
	uint32 u32SdaBank;
	uint32 u32SdaPin;
}I2C_BSP;		/* ��I2C�ܽŶ���ṹ */

#ifdef _LIB_MATH_C_
const I2C_BSP cnst_I2C_BSP[I2C_COMM_NUM] = {
		{GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9}
};
#else
extern const I2C_BSP cnst_I2C_BSP[I2C_COMM_NUM];
#endif

#define LED_COLOR_RED		(1 << 0)
#define LED_COLOR_GREEN		(1 << 1)
#define LED_COLOR_BLUE		(1 << 2)
#define LED_COLOR_YELLOW	(LED_COLOR_RED | LED_COLOR_GREEN)
#define LED_COLOR_PINK		(LED_COLOR_RED | LED_COLOR_BLUE)
#define LED_COLOR_CRAN 		(LED_COLOR_GREEN | LED_COLOR_BLUE)
/*===========================================================================
 * Conf Var
 *==========================================================================*/
/*---------------------------------------------------------------------------
 * ���ñ���
 *--------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * ���ñ�������
 *--------------------------------------------------------------------------*/
/* <NULL> */

/*---------------------------------------------------------------------------
 * ���õ�½ҳ��
 *--------------------------------------------------------------------------*/
/* <NULL> */

/*---------------------------------------------------------------------------
 * ���ýṹ��
 *--------------------------------------------------------------------------*/
/* <NULL> */

/***************************************************************************
 					functions must be driven by extern
***************************************************************************/
EXT void InitSample(void);
EXT void DrvSampleTick_1KHz(void);

/***************************************************************************
 					functions could be used as extern
***************************************************************************/
EXT void Board_DrvWheel(float32 fRightDuty, float32 LeftDuty);
void Board_DrvPump(float32 fDuty);
EXT uint16 GetCapCurTime_1MHz(void);
EXT uint32 GetCPUTimerCnt(void);

EXT float32 ProcPulseSig(uint8 u8PulseNo);
EXT void UpdateIMU(void);
EXT void UpdateAir(void);
EXT void UpdateIRCtrl(void);
EXT BOOL InitTtsSpeaking(void);
EXT BOOL TtsSpeak(TTS_DIALOG_TYPE DlgType, BOOL bIgnoreDup);
EXT void UpdateTTS(void);
EXT void ControlLed(uint32 u32LedsToWrite);

/*===========================================================================
 * �̵�������
 *==========================================================================*/
#ifdef _LIB_MATH_C_
const RELAY_DIN_ITEM cnst_RelayDInTable[] = {
/* ���� */	    {RELAY_DIN_NULL, 	NULL_DInNo, 		NULL_RelayNo, 		NULL_RelayNo, 		0,	MSG_NULL},
/* �¹� */		{RELAY_DIN_NULL,	NULL_DInNo, 		NULL_RelayNo, 		NULL_RelayNo,		50,	MSG_NULL},
/* ��բ */		{RELAY_DIN_NULL,	NULL_DInNo,	        NULL_RelayNo,		NULL_RelayNo,	100,MSG_NULL},
};
#else
extern const RELAY_DIN_ITEM cnst_RelayDInTable[];
#endif

/************   exclude redefinition and c++ environment   ****************/
#ifdef __cplusplus
}
#endif 					/* extern "C" */

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
/******************************** FILE END ********************************/
