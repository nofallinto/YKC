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
#define MCU_FLASH_END						(0x08000000 + FLASH_HALF_BLEN*2 - 1)	/* stm32的flash截至地址宏不准(427有1m和2m两种)，所以需要自定义,YKC用的F103ZET6,1M内置flash */

#define SRAM_ADDR_END						0x20010000UL			/* 10000 == 64KB, F103 */

#define USE_SELF_DEVELOP_SHELL		TRUE			/* 使用自研外壳 */
#define USE_SELF_DEVELOP_MOTOR		TRUE			/* 自研电机 */
/*===========================================================================
 * 外设数量
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
 * GPIO总体
 *==========================================================================*/
#define MAX_DIN_NUM					14		/* 逻辑开入数量 */
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
 * 开出量
 *==========================================================================*/
#define NULL_RelayNo				0xFF
#define TOP_WATER_JET_RelayNo		(GPIO_RELAY_StartNo + 0)
#define BOT_WATER_JET_RelayNo		(GPIO_RELAY_StartNo + 1)
#define BLUE_LED_DOutNo				(GPIO_RELAY_StartNo + 2)
#define RED_LED_DOutNo				(GPIO_RELAY_StartNo + 3)
#define GREEN_LED_DOutNo			(GPIO_RELAY_StartNo + 4)
#define OTHER_DEV_DOutNo			(GPIO_RELAY_StartNo + 5)
#define BMI270_CS(n) 				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, n)			/* IMU模块片选管脚 */

/*===========================================================================
 * 开入量, NULL_DInNo以外都是相对于GPIO_DIN_StartNo的索引
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
 * 运放 & AD & Cap
 *==========================================================================*/
#define ADC_BIT 					12U				/* AD位数 */
#define ADC_RANGE					3.3f				/* ADRng AD量程,单位V */
#define ADC_MAX_RESULT				((1<<ADC_BIT)-1)	/* ADMax AD满量程所对应的数字 */

#define COEF_k_VAL2DCPWRVOL			((ADC_RANGE*8.52f)/ADC_MAX_RESULT)      /* 110K/10K */
#define COEF_k_VAL2MOTORCUR			(1.08225f*ADC_RANGE/ADC_MAX_RESULT)

/* 直流信号定义 */
#define DCPWR_VOL_DCSigNo       0
#define RIGHT_CUR_DCSigNo       1
#define LEFT_CUR_DCSigNo        2
#define CPU_TEMP_DCSigNo        3
#define VREF_INT_DCSigNo        4
#define DC_TOTAL_CHN        	5		/* 最多直流信号数量 */

#ifdef _LIB_MATH_C_
const ANALOG_SIG_CORR cnst_DCSigFixCorr[] = {			/* 扣除CPU温度通道 */
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

typedef struct {
	uint32 u32Playing_ms;								/* 当前歌曲剩余播放时长 单位：ms */
	uint16 aUSongsRunningTime_s[VOICE_MAX_NO];			/* 每首歌的播放时长 单位：s */
	uint16 uLastPlayNo;									/* 上一次播放的歌曲编号 */
}MP3;
EXT MP3 g_Mp3;

/* 模拟信号处理 */
#define ANALOG_ADSAMP_BUF_LEN		180
typedef struct {
    /* 以下5个变量必须如此顺序定义，才能符合DAT_UNI_REC结构体，使用相应的函数处理 */
    float32         fSig_avr;           /* 采样数据均值 */
    float32         fSig_rms;           /* 采样数据均方根(rms)值 */
    uint16          uSampOkDecCnt;      /* 采样控制完整性计数器，当改变采样频率后置数本计数器，该计数器减为0时，采样结果可用 */
    uint16          uCurSampPt;         /* 采样指针 */
    uint16          uSampleBuf[ANALOG_ADSAMP_BUF_LEN]; /* ANALOG_ADSAMP_BUF_LEN 需为32的整数倍 */
    /* 采样计算 */
}ANALOG_ADSAMP_VAR;
EXT SECTION(".NOT_ZeroInit") ANALOG_ADSAMP_VAR g_DCSigADSamp[DC_TOTAL_CHN];

#define CAP_CLK_FREQ_Hz         1000000.0f  /*  */
/* 有刷电机：空载转速8400，最大效率点转速6889，约140Hz，每转一圈约16个脉冲，即脉冲频率约2.4KHz */
#define BUMP_MOTOR_PulseNo      0
#define RIGHT_MOTOR_PulseNo     1
#define LEFT_MOTOR_PulseNo      2
#define PULSE_SIG_CHN		3
#define PULSE_CYCLE_BUF_LEN		150
typedef struct {
	/* 以下7个变量必须如此顺序定义，才能符合 DAT_UNI_REC 结构体，使用相应的函数处理 */
	float32 		fSig_avr;			/* 采样数据均值 */
	float32 		fSig_rms;			/* 采样数据均方根(rms)值 */
	uint16			uSampOkDecCnt;		/* 采样控制完整性计数器，当改变采样频率后置数本计数器，该计数器减为0时，采样结果可用 */
	uint16			uCurSampPt;			/* 采样指针 */
	uint16 			uCycleBuf[PULSE_CYCLE_BUF_LEN]; 	/* 信号周期缓存 */
	uint16 			uPulseEdge;			/* 信号边沿信息 */
	BOOL 			bPulseTimeOut;		/* 信号边沿超时 */
	uint8			u8Rsvd;
}PULSE_CAP_VAR;
EXT SECTION(".NOT_ZeroInit") PULSE_CAP_VAR g_PulseCap[PULSE_SIG_CHN];

typedef struct {
	uint32 u32SclBank;
	uint32 u32SclPin;
	uint32 u32SdaBank;
	uint32 u32SdaPin;
}I2C_BSP;		/* 软I2C管脚定义结构 */

#ifdef _LIB_MATH_C_
const I2C_BSP cnst_I2C_BSP[I2C_COMM_NUM] = {
		{(uint32)GPIOB, GPIO_PIN_8, (uint32)GPIOB, GPIO_PIN_9}
};
#else
extern const I2C_BSP cnst_I2C_BSP[I2C_COMM_NUM];
#endif

#define LED_COLOR_RED		(1 << 0)
#define LED_COLOR_GREEN		(1 << 1)
#define LED_COLOR_BLUE		(1 << 2)
#define LED_COLOR_YELLOW	(LED_COLOR_RED | LED_COLOR_GREEN)
#define LED_COLOR_PINK		(LED_COLOR_RED | LED_COLOR_BLUE)
#define LED_COLOR_CYAN 		(LED_COLOR_GREEN | LED_COLOR_BLUE)
/*===========================================================================
 * Conf Var
 *==========================================================================*/
/*---------------------------------------------------------------------------
 * 配置变量
 *--------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * 配置变量属性
 *--------------------------------------------------------------------------*/
/* <NULL> */

/*---------------------------------------------------------------------------
 * 配置登陆页面
 *--------------------------------------------------------------------------*/
/* <NULL> */

/*---------------------------------------------------------------------------
 * 配置结构体
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
//EXT BOOL InitTtsSpeaking(void);
//EXT BOOL TtsSpeak(TTS_DIALOG_TYPE DlgType, BOOL bIgnoreDup);
//EXT void UpdateTTS(void);
EXT BOOL InitMp3Speaking(void);	 					/* 初始化MP3模块 */
EXT BOOL Mp3Speak(MP3_DIALOG_TYPE eDialogNo, BOOL bPlay_1Now_0Idle);		/* 立即播放指定音频 */
EXT void ControlLed(uint32 u32LedsToWrite);

/*===========================================================================
 * 继电器动作
 *==========================================================================*/
#ifdef _LIB_MATH_C_
const RELAY_DIN_ITEM cnst_RelayDInTable[] = {
/* 报警 */	    {RELAY_DIN_NULL, 	NULL_DInNo, 		NULL_RelayNo, 		NULL_RelayNo, 		0,	MSG_NULL},
/* 事故 */		{RELAY_DIN_NULL,	NULL_DInNo, 		NULL_RelayNo, 		NULL_RelayNo,		50,	MSG_NULL},
/* 合闸 */		{RELAY_DIN_NULL,	NULL_DInNo,	        NULL_RelayNo,		NULL_RelayNo,	100,MSG_NULL},
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
