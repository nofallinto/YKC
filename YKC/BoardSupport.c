/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved
 *
 * File			: BSP.c
 * Author		: Wang Renfei
 * Description	: 板级支持文件, 涵盖硬件初始化
 *				: 以及SYS/BIOS相关的驱动 : GPIO,SDSPI,UART,以太网,USB,PWM
 *				: 其余硬件，如:ADC, CAP, RTC 由各自模块自行初始化
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
#define _BOARD_SUPPORT_C_		/* exclude redefinition */


/***************************************************************************
 						include files
***************************************************************************/
/* 项目公共头文件 */
#include <string.h>
#include "GlobalVar.h"
#include "LibMath.h"
#include "MdlSys.h"
#include <stdio.h>
/* 本级以及下级模块头文件 */
#include "BoardSupport.h"
#include "MdlUARTnModbus.h"

/***************************************************************************
 						global variables definition
***************************************************************************/

/***************************************************************************
						internal functions declaration
***************************************************************************/
/* <NULL> */


/***************************************************************************
 							functions definition
***************************************************************************/
/*==========================================================================
| Description	: 定时器初始化，总共8个定时器
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2014-11-24
\=========================================================================*/
void InitCap(void);
void Board_initTimers(void)
{
	InitCap();
}

const GPIO_PINCFG cnst_GpioPinCfg[] = {
    /* KickDog */
    {GPIOF, GPIO_PIN_9},

    /* 开入 */
    {GPIOE, GPIO_PIN_8},        //机器人软件开停机控制
    {GPIOD, GPIO_PIN_6},        //水泵管道水位检测

#if USE_SELF_DEVELOP_SHELL
    {GPIOD, GPIO_PIN_3},        //左下碰撞开关
    {GPIOD, GPIO_PIN_4},        //左上碰撞开关
    {GPIOB, GPIO_PIN_0},        //右上碰撞开关
    {GPIOC, GPIO_PIN_5},        //右下碰撞开关

    {GPIOD, GPIO_PIN_2},        //左侧碰撞开关
    {GPIOB, GPIO_PIN_1},        //前右碰撞开关
    {GPIOD, GPIO_PIN_5},        //前左碰撞开关
    {GPIOC, GPIO_PIN_4},		//右侧碰撞开关
#else
	{GPIOD, GPIO_PIN_4},        //侧左前碰撞开关 3
    {GPIOD, GPIO_PIN_3},        //侧左后碰撞开关 4
	{GPIOC, GPIO_PIN_5},        //侧右前碰撞开关 5
    {GPIOB, GPIO_PIN_0},        //侧右后碰撞开关 6

	{GPIOD, GPIO_PIN_5},        //前左碰撞开关 7
	{GPIOC, GPIO_PIN_4},		//前右碰撞开关 8
    {GPIOD, GPIO_PIN_2},        //后左碰撞开关 9
    {GPIOB, GPIO_PIN_1},        //后右碰撞开关 10
#endif
    {GPIOA, GPIO_PIN_15},       //悬崖开关2 左上 11
	{GPIOE, GPIO_PIN_7},        //悬崖开关3 右上 12
	{GPIOE, GPIO_PIN_5},        //悬崖开关1 左下 13
	{GPIOA, GPIO_PIN_3},        //悬崖开关4 右下 14
    /* 继电器 */
    {GPIOD, GPIO_PIN_1}, 	      //左下水泵
    {GPIOC, GPIO_PIN_12},        //左上水泵
    /* 其他开出 */
    {GPIOE, GPIO_PIN_10},       //机器人工作模式三色指示灯控制 蓝色
    {GPIOE, GPIO_PIN_9},       	//机器人工作模式三色指示灯控制 红色
	{GPIOE, GPIO_PIN_11},        //机器人工作模式三色指示灯控制 绿色
	{GPIOE, GPIO_PIN_0},        //其他设备电源

    /* 空余测试点 */
    {GPIOF, GPIO_PIN_14},
    {GPIOF, GPIO_PIN_15},
    {GPIOG, GPIO_PIN_0},
    {GPIOG, GPIO_PIN_1}
};

void Board_initGPIO(void)
{
    /* Enable clock supply for the following peripherals */

	/* Once GPIO_init is called, GPIO_config cannot be changed */
}

/* 驱动履带轮 */
#define BRUSH_MOTOR_PWM_PERIOD  3600       /* 20KHz，科沃斯有刷、无刷都是20KHz */
void Board_DrvWheel(float32 fRightDuty, float32 LeftDuty)
{
    extern TIM_HandleTypeDef htim3;
    /* 右履带：H:高电平，L:低电平，P:PWM输出 */
    if(fRightDuty == 0) {        /* 刹车: HH */
    #if 1
        htim3.Instance->CCR1 = htim3.Instance->ARR + 1;
        htim3.Instance->CCR2 = htim3.Instance->ARR + 1;
    #else
        htim3.Instance->CCR1 = 0;
        htim3.Instance->CCR2 = 0;
    #endif
    } else if(fRightDuty > 0) {  /* 前进: LP, 科沃斯是PH */
        htim3.Instance->CCR1 = F32ToU16(htim3.Instance->ARR*fRightDuty);
        htim3.Instance->CCR2 = 0;
    } else if(fRightDuty < 0) {  /* 后退: PL, 科沃斯是PH */
        htim3.Instance->CCR1 = 0;
        htim3.Instance->CCR2 = F32ToU16(htim3.Instance->ARR*(0-fRightDuty));
    }

    /* 左履带：H:高电平，L:低电平，P:PWM输出 */
    if(LeftDuty == 0) {        /* 刹车: HH */
    #if 1
        htim3.Instance->CCR3 = htim3.Instance->ARR + 1;
        htim3.Instance->CCR4 = htim3.Instance->ARR + 1;
    #else
        htim3.Instance->CCR3 = 0;
        htim3.Instance->CCR4 = 0;
    #endif
    } else if(LeftDuty > 0) {  /* 前进: LP, 科沃斯是PH */
        htim3.Instance->CCR3 = F32ToU16(htim3.Instance->ARR*LeftDuty);
        htim3.Instance->CCR4 = 0;
    } else if(LeftDuty < 0) {  /* 后退: PL, 科沃斯是PH */
        htim3.Instance->CCR3 = 0;
        htim3.Instance->CCR4 = F32ToU16(htim3.Instance->ARR*(0-LeftDuty));
    }
    //htim3.Instance->ARR = BRUSH_MOTOR_PWM_PERIOD;    
}

/* 驱动真空泵 */
void Board_DrvPump(float32 fDuty)
{
    extern TIM_HandleTypeDef htim2;
    if(fDuty <= 0) {        /* 停，科沃斯是两个都是92%占空比 */
       htim2.Instance->CCR3 = 0;
       htim2.Instance->CCR4 = F32ToU16(htim2.Instance->ARR*0.02f);
    } else {
        htim2.Instance->CCR3 = htim2.Instance->ARR + 1;
        htim2.Instance->CCR4 = F32ToU16(htim2.Instance->ARR*fDuty);
    }
    //htim2.Instance->ARR = BRUSH_MOTOR_PWM_PERIOD;
}


/*==========================================================================
| Description	: AD采样模块
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2008-3-4
\=========================================================================*/
void InitAdc(void);
void InitAdc(void) {

}

void InitSample(void)
{
	int8 i;
	
	/* 初始化配置 */
	/* <NULL> */

	/* 初始化输入接口变量 */

	/* 初始化输出接口变量 */
	for(i = 0; i < DC_TOTAL_CHN; i++) {
		g_DCSigADSamp[i].uSampOkDecCnt = ANALOG_ADSAMP_BUF_LEN;
		g_DCSigADSamp[i].uCurSampPt = 0;
	}
	for(i = 0; i < PULSE_SIG_CHN; i++) {
      	InitUDatUniRec((uDAT_UNI_REC*)&g_PulseCap[i], PULSE_CYCLE_BUF_LEN);
    }

	/* 初始化内部全局变量 */
	for(i = 0; i < MAX_DIN_NUM; i++) {
		g_uDInFilt[i] = 0xFFFF;
	}

	/* 初始化下层模块 */
	
	/* 启动硬件 */
	InitAdc();
}

/*==========================================================================
| Description	: 采样Tick
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-4-30
\=========================================================================*/
extern ADC_HandleTypeDef hadc2;
void DrvSampleTick_1KHz(void)
{
	int8 i;
	/* 更新开入 */
	for(i = MAX_DIN_NUM - 1; i >= 0; i--) {
		g_uDInFilt[i] = (g_uDInFilt[i]<<1) + GPIO_read(GPIO_DIN_StartNo + i);
	}
}

/*==========================================================================
| Description	: AD中断响应函数，完成数据存储
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-8-27
\=========================================================================*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    UniRecUDatToBuf((uDAT_UNI_REC*)(&g_DCSigADSamp[DCPWR_VOL_DCSigNo]), ANALOG_ADSAMP_BUF_LEN, g_AdcDmaBuf.u32Adc1Res[0]);
    UniRecUDatToBuf((uDAT_UNI_REC*)(&g_DCSigADSamp[RIGHT_CUR_DCSigNo]), ANALOG_ADSAMP_BUF_LEN, g_AdcDmaBuf.u32Adc1Res[1]);
    UniRecUDatToBuf((uDAT_UNI_REC*)(&g_DCSigADSamp[LEFT_CUR_DCSigNo]), ANALOG_ADSAMP_BUF_LEN, g_AdcDmaBuf.u32Adc1Res[2]);
    UniRecUDatToBuf((uDAT_UNI_REC*)(&g_DCSigADSamp[CPU_TEMP_DCSigNo]), ANALOG_ADSAMP_BUF_LEN, g_AdcDmaBuf.u32Adc1Res[3]);
    UniRecUDatToBuf((uDAT_UNI_REC*)(&g_DCSigADSamp[VREF_INT_DCSigNo]), ANALOG_ADSAMP_BUF_LEN, g_AdcDmaBuf.u32Adc1Res[4]);
}

/*==========================================================================
| Description	: 左侧履带、右侧履带、无刷电机、测速、红外遥控
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-4-30
\=========================================================================*/
uint16 GetCapCurTime_1MHz(void)
{
    extern TIM_HandleTypeDef htim1;
    return htim1.Instance->CNT;
}

uint32 GetCPUTimerCnt(void)
{
    extern TIM_HandleTypeDef htim1;
    return htim1.Instance->CNT;
}

/* 红外遥控器初始化 */
void IrRecvInit(uint8 u8SelectedrIndex)
{
	g_IRCtrl.DevComm[u8SelectedrIndex].bBtnRdyDecode = FALSE;						/* 未捕获到新数据 */
	g_IRCtrl.DevComm[u8SelectedrIndex].bIsIdle = TRUE;								/* 进入非空闲状态 */
	g_IRCtrl.DevComm[u8SelectedrIndex].u8TimerUpdatCount = 0;							/* 定时器溢出清0 */
	g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPulseCount = 0;						/* 捕获到的电位变化计数清0 */
    memset((void*)g_IRCtrl.DevComm[u8SelectedrIndex].rxFrame, 0x00, sizeof(g_IRCtrl.DevComm[u8SelectedrIndex].rxFrame));
}

void IrRecvCapture(uint8 u8SelectedrIndex,TIM_HandleTypeDef *pHtim, uint32 u32Ccr, uint16 uCapChannl)
{
	uint16 uCapTime = 0;
	static uint16 s_uCapTime_last = 0;
	uCapTime = u32Ccr;

	TIM_RESET_CAPTUREPOLARITY(pHtim, uCapChannl);                      /* 复位极性配置 */

	if(g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPol == 0) { 		/* 话说，这个地方干嘛弄得这么复杂？不是可以设置成上升、下降沿都可以触发的么？ 在ioc图形界面里只能配置成一个方向*/
		TIM_SET_CAPTUREPOLARITY(pHtim, uCapChannl, TIM_INPUTCHANNELPOLARITY_RISING); 		/* 改变极性 */
		g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPol = 1;        	/* 极性标志位改为上升沿 */
	} else {
		TIM_SET_CAPTUREPOLARITY(pHtim, uCapChannl, TIM_ICPOLARITY_FALLING);					/* 改变极性 */
		g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPol = 0;
	}
	if(!g_IRCtrl.DevComm[u8SelectedrIndex].bIsIdle) {              /* 如果当前为空闲状态，空闲捕获到的时序，为第一个下降沿 */
		IrRecvInit(u8SelectedrIndex);
	} else {
		if(g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPulseCount < IR_RX_SEQ_BLEN*2) {
			/* 这个代码是有风险的，可能：T5溢出和捕捉同时发生，最好还是恰当设计T5频率，16bit计时器应该是足够的 */
			/* ————查源码确认，如果同时发生，InputCapture()会排在PeriodElapse()之后执行 */
			g_IRCtrl.DevComm[u8SelectedrIndex].rxFrame[g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPulseCount] = g_IRCtrl.DevComm[u8SelectedrIndex].u8TimerUpdatCount*65536 + uCapTime - s_uCapTime_last;		/* 增加g_IRCtrl.u8Tim_udt_count*65536是为支持跨溢出 */
		}
		g_IRCtrl.DevComm[u8SelectedrIndex].u8TimerUpdatCount = 0;
		if(g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPulseCount < 0xFF) {
			g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPulseCount++;
		}
	}
	s_uCapTime_last = uCapTime;
}

void IrRecvDecode(uint8 u8SelectedrIndex)
{
	/* 更新红外遥控器状态 */
	if(g_IRCtrl.DevComm[u8SelectedrIndex].bIsIdle) {							/* 空闲状态，不作任何处理 */
		g_IRCtrl.DevComm[u8SelectedrIndex].u8TimerUpdatCount++;					/* 溢出一次 */
		if(g_IRCtrl.DevComm[u8SelectedrIndex].u8TimerUpdatCount == 2) {			/* 溢出2次 */
			g_IRCtrl.DevComm[u8SelectedrIndex].u8TimerUpdatCount = 0;				/* 溢出次数清零 */
			g_IRCtrl.DevComm[u8SelectedrIndex].bIsIdle = FALSE;					/* 这是为空闲状态 */
			g_IRCtrl.DevComm[u8SelectedrIndex].bBtnRdyDecode = TRUE;			/* 完整的一帧捕获完成，应用那边可以解码了 */
		}
	}
}

/* 记录红外空闲溢出次数，如果非空闲且连续TIM溢出二次则标记为可解码 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *pHtim)
{
    //ProcCap(htim0.Instance->CCR1, 0);
    //ProcCap(htim0.Instance->CCR1, 0);
	if(TIM5 == pHtim->Instance) {							/* 速率100Hz */
		/* 更新红外遥控器状态 */
		IrRecvDecode(0);

		/* 更新另外一边红外遥控器状态 */ /* 这两边的红外要同时跟新没有先后 */
		IrRecvDecode(1);
	}
}
static uint32 g_u32LeftAllCnt = 0;
static uint32 g_u32RightAllCnt = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *pHtim)
{
    if(pHtim->Instance == TIM1) {
        if(pHtim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {         /* 右侧有刷电机反馈 */
            uint16 uCapTime = pHtim->Instance->CCR2;
            UniRecUDatToBuf((uDAT_UNI_REC*)&g_PulseCap[RIGHT_MOTOR_PulseNo], PULSE_CYCLE_BUF_LEN, uCapTime - g_PulseCap[RIGHT_MOTOR_PulseNo].uPulseEdge);
            g_PulseCap[RIGHT_MOTOR_PulseNo].uPulseEdge = uCapTime;
            g_MsrRes.uRightCount++;
            g_u32RightAllCnt++;
        } else if(pHtim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {  /* 无刷电机反馈 */
            uint16 uCapTime = pHtim->Instance->CCR3;
            UniRecUDatToBuf((uDAT_UNI_REC*)&g_PulseCap[BUMP_MOTOR_PulseNo], PULSE_CYCLE_BUF_LEN, uCapTime - g_PulseCap[BUMP_MOTOR_PulseNo].uPulseEdge);
            g_PulseCap[BUMP_MOTOR_PulseNo].uPulseEdge = uCapTime;
        } else if(pHtim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {  /* 左侧有刷电机反馈 */
            uint16 uCapTime = pHtim->Instance->CCR4;
            UniRecUDatToBuf((uDAT_UNI_REC*)&g_PulseCap[LEFT_MOTOR_PulseNo], PULSE_CYCLE_BUF_LEN, uCapTime - g_PulseCap[LEFT_MOTOR_PulseNo].uPulseEdge);
            g_PulseCap[LEFT_MOTOR_PulseNo].uPulseEdge = uCapTime;
            g_MsrRes.uLeftCount++;
            g_u32LeftAllCnt++;
         }
	} else if(TIM5 == pHtim->Instance) { /* 将两个红外捕获 */
		if(pHtim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			IrRecvCapture(0, pHtim, pHtim->Instance->CCR2, TIM_CHANNEL_2);
		}

		if(pHtim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			IrRecvCapture(1, pHtim, pHtim->Instance->CCR3, TIM_CHANNEL_3);
		}
	}
}

/* 对于脉冲捕捉信号进行处理 */
float32 ProcPulseSig(uint8 u8PulseNo)
{
	/* 维护脉冲边沿:由于捕捉定时器仅16bit，为了避免溢出造成脉冲周期测量不准，在没有脉冲的时候，产生一个虚假的"uPulseEdge" */
	uint16 uCurTime = GetCapCurTime_1MHz();
	UInt HwiKey = Hwi_disable();
	if(uCurTime - g_PulseCap[u8PulseNo].uPulseEdge > PERIOD_CTR_TASK_ms*1000) {	/* 如果在转动，一个计算周期内，肯定会有脉冲 */
		g_PulseCap[u8PulseNo].uPulseEdge = uCurTime - PERIOD_CTR_TASK_ms*1000;	/* 避免溢出 */
		InitUDatUniRec((uDAT_UNI_REC*)&g_PulseCap[u8PulseNo], PULSE_CYCLE_BUF_LEN);
	}
	Hwi_restore(HwiKey);

	/* 进行计算 */
	if(ProcUniRecUDatAsWave((uDAT_UNI_REC*)&g_PulseCap[u8PulseNo], PULSE_CYCLE_BUF_LEN, 20)) {
		return CAP_CLK_FREQ_Hz/g_PulseCap[u8PulseNo].fSig_avr;
	} else {
		return 0;
	}
}


float32 CalcForwardAccel(int16_t iAccX, int16_t iAccY, int16_t iAccZ,
                               float q0, float q1, float q2, float q3,
                               float fForwardX, float fForwardY, float fForwardZ)
{

    /* 1. 原始加速度转换为 float */
    float ax = (float)iAccX;
    float ay = (float)iAccY;
    float az = (float)iAccZ;

    /* 2. 估计重力在传感器坐标系下的分量 */
    float gx = 2.0f * (q1*q3 - q0*q2);
    float gy = 2.0f * (q0*q1 + q2*q3);
    float gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    /* 假设传感器输出单位和重力一致，如果不是请按比例调整 */

    /* 3. 去掉重力 */
    float ax_nograv = ax - gx * 9.81f;
    float ay_nograv = ay - gy * 9.81f;
    float az_nograv = az - gz * 9.81f;

    /* 4. 计算前进方向加速度（投影到前进方向） */
    return ax_nograv*fForwardX + ay_nograv*fForwardY + az_nograv*fForwardZ;
}

#include "BMI270.h"		/* IMU */
#if 0
/* 更新IMU六轴和计算姿态数据，陀螺仪内部输出频率可配置范围是0.02-1.6KHz，可开启的低通滤波频率范围是5.5-751Hz*/
void UpdateIMU(void)
{
    /* 根据六轴数据计算姿势角度数据 */
	Bmi270GetAcc();             /* 获取 IMU660RA 的加速度测量数值 */
	Bmi270GetGyro();            /* 获取 IMU660RA 的角速度测量数值 */
	static float32 s_fQ0 = 1, s_fQ1 = 0, s_fQ2 = 0, s_fQ3 = 0;      /* 四元数的元素，代表估计方向 */
	static float32 s_fExInt = 0, s_fEyInt = 0, s_fEzInt = 0;      /* 按比例缩小积分误差 */
	int16 iGyrX = g_Bmi270Comm.iGyrX;
	int16 iGyrY = g_Bmi270Comm.iGyrY;
	int16 iGyrZ = g_Bmi270Comm.iGyrZ;
	int16 iAccX = g_Bmi270Comm.iAccX;
	int16 iAccY = g_Bmi270Comm.iAccY;
	int16 iAccZ = g_Bmi270Comm.iAccZ;
	if(iGyrX && iGyrY && iGyrZ && iAccX && iAccY && iAccZ) {
		float32 fGx = iGyrX / 16384.0;
		float32 fGy = iGyrY / 16384.0;
		float32 fGz = iGyrZ / 16384.0;
		float32 fAx = iAccX;
		float32 fAy = iAccY;
		float32 fAz = iAccZ;

		/* 计算归一化因子 */
		float32 fNorm = sqrtf(fAx * fAx + fAy * fAy + fAz * fAz);

		if(fNorm != 0) {

			fAx = fAx / fNorm;                   /* 归一化 */
			fAy = fAy / fNorm;
			fAz = fAz / fNorm;

			/* 估计方向的重力 */
			float32 fVx = 2 * (s_fQ1 * s_fQ3 - s_fQ0 * s_fQ2);
			float32 fVy = 2 * (s_fQ0 * s_fQ1 + s_fQ2 * s_fQ3);
			float32 fVz = s_fQ0 * s_fQ0 - s_fQ1 * s_fQ1 - s_fQ2 * s_fQ2 + s_fQ3 * s_fQ3;

			/* 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和 */
			float32 fEx = (fAy * fVz - fAz * fVy);
			float32 fEy = (fAz * fVx - fAx * fVz);
			float32 fEz = (fAx * fVy - fAy * fVx);

			/* 积分误差比例积分增益 */
			s_fExInt = s_fExInt + fEx * KI;
			s_fEyInt = s_fEyInt + fEy * KI;
			s_fEzInt = s_fEzInt + fEz * KI;

			/* 调整后的陀螺仪测量 */
			fGx = fGx + KP * fEx + s_fExInt;
			fGy = fGy + KP * fEy + s_fEyInt;
			fGz = fGz + KP * fEz + s_fEzInt;

			/* 整合四元数率和正常化 */
			float32 q0 = s_fQ0;
			float32 q1 = s_fQ1;
			float32 q2 = s_fQ2;
			float32 q3 = s_fQ3;
			s_fQ0 = q0 + (-q1 * fGx - q2 * fGy - q3 * fGz) * HALF_T;
			s_fQ1 = q1 + (q0 * fGx + q2 * fGz - q3 * fGy) * HALF_T;
			s_fQ2 = q2 + (q0 * fGy - q1 * fGz + q3 * fGx) * HALF_T;
			s_fQ3 = q3 + (q0 * fGz + q1 * fGy - q2 * fGx) * HALF_T;

			/* 计算四元数向量长度 */
			fNorm = sqrtf(s_fQ0 * s_fQ0 + s_fQ1 * s_fQ1 + s_fQ2 * s_fQ2 + s_fQ3 * s_fQ3);

			if(fNorm != 0) {
				/* 归一化四元数 */
				s_fQ0 = s_fQ0 / fNorm;
				s_fQ1 = s_fQ1 / fNorm;
				s_fQ2 = s_fQ2 / fNorm;
				s_fQ3 = s_fQ3 / fNorm;
				/* 驱动全局变量 */
				g_MsrRes.iGyroX = g_Bmi270Comm.iGyrX;
				g_MsrRes.iGyroY = g_Bmi270Comm.iGyrY;
				g_MsrRes.iGyroZ = g_Bmi270Comm.iGyrZ;
				g_MsrRes.iAccX = g_Bmi270Comm.iAccX;
				g_MsrRes.iAccY = g_Bmi270Comm.iAccY;
				g_MsrRes.iAccZ = g_Bmi270Comm.iAccZ;

				/* 计算陀螺仪角度： */
				float32 R[3][3];
				R[0][0] = 1 - 2 * s_fQ2 * s_fQ2 - 2 * s_fQ3 * s_fQ3;
			    R[0][1] = 2 * s_fQ1 * s_fQ2 - 2*s_fQ0 * s_fQ3;
			    R[0][2] = 2 * s_fQ1 * q3 + 2 * s_fQ0 * s_fQ2;

			    R[1][0] = 2 * s_fQ1 * s_fQ2 + 2 * s_fQ0 * s_fQ3;
			    R[1][1] = 1 - 2 * s_fQ1 * s_fQ1 - 2 * s_fQ3 * s_fQ3;
			    R[1][2] = 2 * s_fQ2 * s_fQ3 - 2 * s_fQ0 * s_fQ1;

			    R[2][0] = 2 * s_fQ1 * s_fQ3 - 2 * s_fQ0 * s_fQ2;
			    R[2][1] = 2 * s_fQ2 * s_fQ3 + 2 * s_fQ0 * s_fQ1;
			    R[2][2] = 1 - 2 * s_fQ1 * s_fQ1 - 2 * s_fQ2 * s_fQ2;
				float32 ang_rad= atan2(R[1][0], R[0][0]);
				g_MsrRes.fPitch = ang_rad * (180.0f / M_PI);
			    g_MsrRes.u8TryCnt_IMU = 5;
				return;

				g_MsrRes.fPitch  = asinf(-2 * s_fQ1 * s_fQ3 + 2 * s_fQ0 * s_fQ2) * 57.3; /* pitch ,转换为度数 */

				if(g_Bmi270Comm.iAccY > 0) {	/* 板子装反了 */
					if(g_MsrRes.fPitch > 0) {
						g_MsrRes.fPitch = 180 - g_MsrRes.fPitch;
					} else {
						g_MsrRes.fPitch = 0 - 180 - g_MsrRes.fPitch;
					}
				}
				g_MsrRes.fPitch += 90;		/* 正上方是0度 */
			/*	if (g_MsrRes.fPitch < 0) {
				    g_MsrRes.fPitch += 360; // 如果是负数，增加360度
				}*/
				g_MsrRes.fRoll = atan2f(2 * s_fQ2 * s_fQ3 + 2 * s_fQ0 * s_fQ1, -2 * s_fQ1 * s_fQ1 - 2 * s_fQ2 * s_fQ2 + 1) * 57.3; /* roll */
				g_MsrRes.u8TryCnt_IMU = 5;
				return;
			}
		}
	}
	/* 如果到这里就算此次通信失败了 */
	if(g_MsrRes.u8TryCnt_IMU) {
		g_MsrRes.u8TryCnt_IMU--;
	}
}
#else
void UpdateIMU(void)
{
    /* 根据六轴数据计算姿势角度数据 */
	Bmi270GetAcc();             /* 获取 IMU660RA 的加速度测量数值 */
	Bmi270GetGyro();            /* 获取 IMU660RA 的角速度测量数值 */
	static float32 s_fQ0 = 1, s_fQ1 = 0, s_fQ2 = 0, s_fQ3 = 0;      /* 四元数的元素，代表估计方向 */
	static float32 s_fExInt = 0, s_fEyInt = 0, s_fEzInt = 0;      	/* 按比例缩小积分误差 */
	int16 iGyrX = g_Bmi270Comm.iGyrX;
	int16 iGyrY = g_Bmi270Comm.iGyrY;
	int16 iGyrZ = g_Bmi270Comm.iGyrZ;
	int16 iAccX = g_Bmi270Comm.iAccX;
	int16 iAccY = g_Bmi270Comm.iAccY;
	int16 iAccZ = g_Bmi270Comm.iAccZ;
	if(iGyrX && iGyrY && iGyrZ && iAccX && iAccY && iAccZ) {

		 /* ========== 1. 量程缩放 ========== */
		// 陀螺仪 (±2000 dps → 除以 16.4)
		float32 fGx = (float)iGyrX / 16.4f * (M_PI / 180.0f); // 转换成 rad/s
		float32 fGy = (float)iGyrY / 16.4f * (M_PI / 180.0f);
		float32 fGz = (float)iGyrZ / 16.4f * (M_PI / 180.0f);

		// 加速度计 (±2g → 除以 16384)
		float32 fAx = iAccX / 16384.0f;
		float32 fAy = iAccY / 16384.0f;
		float32 fAz = iAccZ / 16384.0f;

		/* ========== 2. 归一化加速度 (用于重力方向) ========== */
		float32 fNorm = sqrtf(fAx * fAx + fAy * fAy + fAz * fAz);		/* 计算归一化因子 */
		if(fNorm != 0) {
			fAx = fAx / fNorm;                   /* 归一化 */
			fAy = fAy / fNorm;
			fAz = fAz / fNorm;
			/* 估计方向的重力 */
			float32 fVx = 2.0f * (s_fQ1 * s_fQ3 - s_fQ0 * s_fQ2);
			float32 fVy = 2.0f * (s_fQ0 * s_fQ1 + s_fQ2 * s_fQ3);
			float32 fVz = s_fQ0 * s_fQ0 - s_fQ1 * s_fQ1 - s_fQ2 * s_fQ2 + s_fQ3 * s_fQ3;

			/* 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和 */
			float32 fEx = (fAy * fVz - fAz * fVy);
			float32 fEy = (fAz * fVx - fAx * fVz);
			float32 fEz = (fAx * fVy - fAy * fVx);

			/* 积分误差比例积分增益 */
			s_fExInt = s_fExInt + fEx * KI;
			s_fEyInt = s_fEyInt + fEy * KI;
			s_fEzInt = s_fEzInt + fEz * KI;

			/* 调整后的陀螺仪测量 */
			fGx = fGx + KP * fEx + s_fExInt;
			fGy = fGy + KP * fEy + s_fEyInt;
			fGz = fGz + KP * fEz + s_fEzInt;

			/* 整合四元数率和正常化 */
			s_fQ0 = s_fQ0 + (-s_fQ1 * fGx - s_fQ2 * fGy - s_fQ3 * fGz) * HALF_T;
			s_fQ1 = s_fQ1 + (s_fQ0 * fGx + s_fQ2 * fGz - s_fQ3 * fGy) * HALF_T;
			s_fQ2 = s_fQ2 + (s_fQ0 * fGy - s_fQ1 * fGz + s_fQ3 * fGx) * HALF_T;
			s_fQ3 = s_fQ3 + (s_fQ0 * fGz + s_fQ1 * fGy - s_fQ2 * fGx) * HALF_T;

			/* 计算四元数向量长度 */
			fNorm = sqrtf(s_fQ0 * s_fQ0 + s_fQ1 * s_fQ1 + s_fQ2 * s_fQ2 + s_fQ3 * s_fQ3);

//			/* 计算角度 */
//			float tx = -fAx;
//			float ty = -fAy;
//			float32 angle_deg;
//			float tmag = sqrtf(tx*tx + ty*ty);
//			if (tmag < 1e-3f) {
//				// 返回上一次角度，或 0，或仅靠陀螺积分（见下节）
//				// 这里先返回 0 作占位
//				g_MsrRes.fPitch = 0.0f;
//			} else {
//				float32 angle_rad = atan2f(tx, ty);  // 注意参数顺序 (x, y)
//				angle_deg = (angle_rad * (180.0f / (float)M_PI)) + 90.0f;
//			}
//			// 角度定义：相对 +Y 轴，正上为 0°
//			static float32 fGyrIntegral = 0;
//			fGyrIntegral += (fGz * 0.01f);
//			angle_deg = (1.0f - 0.05f) * fGyrIntegral + 0.05f * angle_deg;
//
//			// 包到 [-180, 180]
//			if (angle_deg > 180.0f) angle_deg -= 360.0f;
//			if (angle_deg < -180.0f) angle_deg += 360.0f;
//
//			fGyrIntegral = angle_deg;
//
//			g_MsrRes.fPitch = angle_deg;

			if(fNorm != 0) {
				/* 归一化四元数 */
				s_fQ0 = s_fQ0 / fNorm;
				s_fQ1 = s_fQ1 / fNorm;
				s_fQ2 = s_fQ2 / fNorm;
				s_fQ3 = s_fQ3 / fNorm;
				/* 驱动全局变量 */
				g_MsrRes.iGyroX = g_Bmi270Comm.iGyrX;
				g_MsrRes.iGyroY = g_Bmi270Comm.iGyrY;
				g_MsrRes.iGyroZ = g_Bmi270Comm.iGyrZ;
				g_MsrRes.iAccX = g_Bmi270Comm.iAccX;
				g_MsrRes.iAccY = g_Bmi270Comm.iAccY;
				g_MsrRes.iAccZ = g_Bmi270Comm.iAccZ;

				g_MsrRes.fPitch  = -asinf(-2 * s_fQ1 * s_fQ3 + 2 * s_fQ0 * s_fQ2) * 57.3; /* pitch ,转换为度数 */
//				g_MsrRes.fPitch = asinf(2 * (s_fQ1 * s_fQ3 - s_fQ0 * s_fQ2)) * 180.0f / M_PI;

				/* 将擦窗机的方向扩为[-180, 180] 正左为正，正右为负 */
				if(g_Bmi270Comm.iAccY > 0) {	/* 角度偏右 */
					g_MsrRes.fPitch = -90 + g_MsrRes.fPitch;
				} else {	/* 角度偏左 */
					g_MsrRes.fPitch = 90 - g_MsrRes.fPitch;
				}
//				g_MsrRes.fRoll = atan2f(2 * s_fQ2 * s_fQ3 + 2 * s_fQ0 * s_fQ1, -2 * s_fQ1 * s_fQ1 - 2 * s_fQ2 * s_fQ2 + 1) * 57.3; /* roll */
//				g_MsrRes.fRoll = atan2f(2 * (s_fQ2 * s_fQ3 + s_fQ0 * s_fQ1), 1 - 2 * (s_fQ1 * s_fQ1 + s_fQ2 * s_fQ2)) * 180.0f / M_PI;
//				g_MsrRes.fYaw = atan2f(2 * (s_fQ1 * s_fQ2 + s_fQ0 * s_fQ3), 1 - 2 * (s_fQ2 * s_fQ2 + s_fQ3 * s_fQ3)) * 180.0f / M_PI;
				g_MsrRes.u8TryCnt_IMU = 5;
				return;
			}
		}
	}
	/* 如果到这里就算此次通信失败了 */
	if(g_MsrRes.u8TryCnt_IMU) {
		g_MsrRes.u8TryCnt_IMU--;
	}
}
#endif

#include "BMP280.h"		/* 气压 */
/* 更新气压计 */
void UpdateAir(void)
{
	if(Bmp280GetAirAndTemp()) {
		g_MsrRes.fAirTemp = g_Bmp280Comm.fTemperature;
		g_MsrRes.fAirPressure = g_Bmp280Comm.fAirPressure;
		g_MsrRes.u8TryCnt_AirSensor = 5;
	} else if(g_MsrRes.u8TryCnt_AirSensor) {
		g_MsrRes.u8TryCnt_AirSensor--;
	}
}

/* 由于捕获到的时序是有误差的，所以程序不能直接判断值是否相等，所以用此函数判断两个值是否接近，只需要接近标准值即可 */
uint8 IsIrTimeCounterMatch(int16 iTimerCounter1, int16 iTimerCounter2)
{
	return (((iTimerCounter1 - iTimerCounter2) < 0) ? - (iTimerCounter1 - iTimerCounter2) : (iTimerCounter1 - iTimerCounter2)) < 300;	/* 300us */
}

/* 更新HX1838红外遥控通信状态 */
void UpdateIRCtrl(void)
{
	BOOL bSuccess = TRUE;
	uint32 u32Key = Hwi_disable();

	if(g_IRCtrl.DevComm[0].bBtnRdyDecode) { /* 如果有待解码信号  优先采用一号位的红外 */
		g_IRCtrl.u8SelectedrIndex = 0;
	} else if(g_IRCtrl.DevComm[1].bBtnRdyDecode) {
		g_IRCtrl.u8SelectedrIndex = 1;
	}
	if(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].bBtnRdyDecode) {		/* 如果有待解码信号 */
		/* 解码指令帧 */
		memcpy(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData, g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].rxFrame, IR_RX_SEQ_BLEN * 4);
		g_IRCtrl.DevComm[1].bBtnRdyDecode = FALSE;
		g_IRCtrl.DevComm[0].bBtnRdyDecode = FALSE;
		Hwi_restore(u32Key);
		if(IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[0], 9000)
			&& IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[1], 4500))
		{        /* 9ms + 4.5ms检测前导码 */
			uint8 u8BitIdx = 0;
			uint8 i;
			g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].uRepeatCount = 0;
			for(i = 2; i < (IR_RX_SEQ_BLEN * 2); i++) {                                  				/* 检测数据 */
				if(!IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[i], 560)) {									/* 0.56ms */
					bSuccess = FALSE;
					break;
				}
				i++;
				if(IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[i], 1680) || (g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[i] > 10000)) {	/* 1.68ms 或大于10ms  */
					g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].data.u32AllData |= (0x80000000 >> u8BitIdx);                          		/* 第u8BitIdx位置1 */
					u8BitIdx++;
				} else if(IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[i], 560)) {								/* 0.56ms */
					g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].data.u32AllData &= ~(0x80000000 >> u8BitIdx);                         		/* 第u8BitIdx位清0 */
					u8BitIdx++;
				} else {
					bSuccess = FALSE;
					break;
				}
			}
		} else if(g_IRCtrl.u8BtnPressing != IR_BTN_NONE	/* 这里是为了防止误判, 当超过指定延续状态时间时, 再次接收到重复码不应该响应. */
				&& IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[0], 9000)
				&& IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[1], 2250)
				&& IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[2], 560))
		{	/* 9ms + 2.25ms + 0.56ms 为重复码，即按住不松手 */
			g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].uRepeatCount++;
			bSuccess = TRUE;
		} else {
			bSuccess = FALSE;		/* 前导码检测错误 */
		}
	} else {
		/* 暂无待解码指令帧 */
		Hwi_restore(u32Key);
		bSuccess = FALSE;
	}

    if(bSuccess) {
		g_IRCtrl.u8BtnPressing = g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].data.bit.key_val;		/* 使用的时候，直接根据键值进行判断呗 */
    	g_IRCtrl.u8TryCnt = 50;			/* 状态延续500ms */
    } else if(g_IRCtrl.u8TryCnt) {
    	g_IRCtrl.u8TryCnt--;
    } else { /* 当没有延续状态时，将按下状态归零 */
    	g_IRCtrl.u8BtnPressing = IR_BTN_NONE;
    }
}
/*语音模块***************************************************************************************/
#include "queue.h"
#define TTS_UART_NO	1
#define MP3_UART_NO	1
#define MP3_DEFAULT_VOLUME_VALUE			25		/* MP3模块默认音量大小 */

#define QUEUE_SIZE	10
typedef struct {
    uint32 u32Queue[QUEUE_SIZE];  	/* 静态数组作为缓冲区 */
    uint8 u8DeqFront;               /* 队头指针        */
    uint8 u8EnqRear;                /* 队尾指针        */
}CIRCULAR_QUEUE;					/* 用于缓冲uint32类型的环形队列 */

/* 初始化环形队列  [线程安全] */
void CirQueInit(CIRCULAR_QUEUE *queue) {
	Swi_disable();
    queue->u8DeqFront = 0;
    queue->u8EnqRear = 0;
    Swi_enable();
}

/* 入队操作 [线程安全]
 * bIgnoreDup：是否忽略重复，不忽略则查找下是否已经存在，如存在则不再插入 */
BOOL CirQueEnq(CIRCULAR_QUEUE *queue, uint32 u32EnqItem, BOOL bIgnoreDup) {
	BOOL bRes = FALSE;
	Swi_disable();
    uint8 next_rear = (queue->u8EnqRear + 1) % QUEUE_SIZE;
    if (next_rear != queue->u8DeqFront) {  /* 队列未满 */
		uint32 u32QueIndex = 0;
    	if (!bIgnoreDup) {
			/* 检查队列中是否已有该元素 */
			for (u32QueIndex = queue->u8DeqFront; u32QueIndex != queue->u8EnqRear; u32QueIndex = (u32QueIndex + 1) % QUEUE_SIZE) {
				if (queue->u32Queue[u32QueIndex] == u32EnqItem) {
				    bRes = TRUE;		/* 忽略重复，不入队 */
				    break;
				}
			}
    	}
    	if(bIgnoreDup || !bRes) {
			queue->u32Queue[queue->u8EnqRear] = u32EnqItem;
			queue->u8EnqRear = next_rear;
			bRes = TRUE;
    	}
    }
    Swi_enable();
    return bRes;
}

/* 出队操作, pDeqItem由调用者确保不为NULL [线程安全] */
BOOL CirQueDeq(CIRCULAR_QUEUE *pQue, uint32* pDeqItem) {
	BOOL bRes = FALSE;
	Swi_disable();
    if (pQue->u8DeqFront != pQue->u8EnqRear) {  // 队列非空
        uint32 item = pQue->u32Queue[pQue->u8DeqFront];
        pQue->u8DeqFront = (pQue->u8DeqFront + 1) % QUEUE_SIZE;
        *pDeqItem = item;
        bRes = TRUE;
    }
    Swi_enable();
    return bRes;
}

typedef struct {
	CIRCULAR_QUEUE Que_Speaking;
	uint32 u32PlayingTick;
}TTS;
TTS g_TTS;

/* 队列初始化 [线程安全] */
BOOL InitTtsSpeaking(void)
{
	OpenUartComm(TTS_UART_NO, 9600, 0, 100);
	CirQueInit(&g_TTS.Que_Speaking);			/* 语音模块播放队列 */
	g_TTS.u32PlayingTick = 0;
	return TRUE;
}
/* =-==================新增MP3模块相关================== */
/* 发送指令到Mp3，参1：指令类型。参2：数据，参3：响应数据
 * 目前用到的指令发送的数据长度和响应数据长度都是2，因此用一个uint16就够了（后续如果有更长的可以用void*） */
BOOL SendCmdToMp3(MP3_CMD eCmd, uint16 uValue, uint16 *pUResValue)
{
	uint8 *pStart = g_UartComm[MP3_UART_NO].u8TRBuf;
	uint8 *pBuf = pStart;
	*pBuf++ = 0xAA;		/* 固定帧头 */
	*pBuf++ = eCmd;		/* 指令类型 */
	uint8 u8SM = 0xAA + eCmd;	/* SM码(校验码) */
	uint8 uResDataLen = 0;		/* 期望响应的数据长度 */
	pBuf++;						/* 跳过数据长度位 */
	switch(eCmd) {
		case MP3_CMD_QUERY_STATE:			/* 查询播放状态(0~2: 停止/播放/暂停) */
			uResDataLen = 1;
			break;
		case MP3_CMD_PLAY:					/* 播放 */
		case MP3_CMD_PAUSE:					/* 暂停 */
		case MP3_CMD_STOP:					/* 停止 */
		case MP3_CMD_VOLUME_UP:				/* 音量加 */
		case MP3_CMD_VOLUME_DOWN:			/* 音量减 */
			break;
		case MP3_CMD_SELECT_SONG_PLAY:		/* 选择曲目播放 */
			pBuf[0] = uValue >> 8;
			pBuf[1] = uValue;
			u8SM += (pBuf[0] + pBuf[1]);
			pBuf+=2;
			break;
		case MP3_CMD_SET_VOLUME:			/* 设置音量(0~30) */
			pBuf[0] = uValue;
			u8SM += pBuf[0];
			pBuf++;
			break;
		default:
			return FALSE;
	}
	pStart[2] = pBuf - pStart - 3;		/* 数据长度 */
	*pBuf++ = u8SM;
	/* 发送帧 */
	WriteToUart(MP3_UART_NO, pBuf - pStart);
	if(uResDataLen) {	/* 需要响应的，等待接收响应 */
		uint16 uRecvLen = ReadFromUart(MP3_UART_NO, 10);
		if(uRecvLen && (pStart[0] == 0xAA) && (pStart[0] == eCmd) && (pStart[2] == uResDataLen) && pUResValue) {
			u8SM = 0;
			for(uint16 i = 0; i < uRecvLen - 1; i++) {	/* 计算校验和 */
				u8SM += pStart[i];
			}
			if(u8SM != pStart[uRecvLen - 1]) {	/* 校验和不通过 */
				return FALSE;
			}
			if(uResDataLen == 1) {
				*pUResValue = pStart[3];
			} else {
				*pUResValue = (pStart[3] << 8) | pStart[4];
			}
		} else {	/* 验证不通过 */
			return FALSE;
		}
	}
	return TRUE;
}

/* 立即播放指定音频 */
BOOL Mp3Speak(TTS_DIALOG_TYPE eDialogNo)
{
	/* 先停止正在播放的音频再播放. */
	return SendCmdToMp3(MP3_CMD_STOP, 0, NULL) && SendCmdToMp3(MP3_CMD_SELECT_SONG_PLAY, eDialogNo, NULL);
}

/* 初始化MP3模块 */
BOOL InitMp3Speaking(void)
{
	OpenUartComm(MP3_UART_NO, 9600, 0, 100);

	/* 配置MP3音量 */
	SendCmdToMp3(MP3_CMD_SET_VOLUME, MP3_DEFAULT_VOLUME_VALUE, NULL);

	return TRUE;
}
/* =-==================新增MP3模块相关end================== */

/* 如果成功加入播放队列则返回TRUE，如果队列已满则返回FALSE [线程安全] */
BOOL TtsSpeak(TTS_DIALOG_TYPE DlgType, BOOL bIgnoreDup)
{
	return CirQueEnq(&g_TTS.Que_Speaking, DlgType, bIgnoreDup);
}

#include "MdlUARTnModbus.h"
#define TTS_HEADER_LEN	5
/* 更新TTS [线程安全] */
void UpdateTTS(void)
{
	uint32 u32CurrentReqType;
	/* 处理队列中待播放的请求 */
	if(!g_TTS.u32PlayingTick && CirQueDeq(&g_TTS.Que_Speaking, &u32CurrentReqType)) {
		/* v[0~16]:0朗读音量为静音，16朗读音量最大，默认是10
		  m[0~16]:0背景音乐为静音，16背景音乐音量最大，默认是4
		  t[0~5]:0朗读语速最慢，5朗读语速最快，默认是4*/
		uint8 *pBuf = g_UartComm[TTS_UART_NO].u8TRBuf;
		const char* pHcData = cnst_TtsDialog[(TTS_DIALOG_TYPE)u32CurrentReqType];
		uint8 u8HcLength = strlen(pHcData);
		uint8 u8Encode = 1;					/* 文本编码格式，0：GB2312，1：GBK，2：BIG5，3：UNICODE*/
		uint8 u8Music = 0;					/* 选择背景音乐(0：无背景音乐  1-15：背景音乐可选)*/

		/* 固定帧信息 */
		*pBuf++ = 0xFD ; 					/* 构造帧头FD*/
		*pBuf++ = 0x00 ; 					/* 构造数据区长度的高字节*/
		*pBuf++ = u8HcLength + 3; 			/* 构造数据区长度的低字节，加命令字、命令参数和校验的三个字节，数据区长度必须严格一致*/
		*pBuf++ = 0x01 ; 					/* 构造命令字：合成播放命令*/
		*pBuf++ = u8Encode | u8Music << 4 ; /* 构造命令参数：编码格式设定，背景音乐设定*/

		/* 校验码计算 */
		int8 i = 0;
		uint8 u8Ecc = 0;
		for(i = 0; i < TTS_HEADER_LEN; i++) {
			u8Ecc ^= g_UartComm[TTS_UART_NO].u8TRBuf[i];
		}
		for(i = 0; i < u8HcLength; i++) {
			u8Ecc ^= pHcData[i];
		}

		/* 拼接帧 */
		PrintStringNoOvChk(&pBuf, pHcData);
		*pBuf++ = u8Ecc;

		/* 发送帧 */
		WriteToUart(TTS_UART_NO, TTS_HEADER_LEN + u8HcLength + 1);

		/* 等待延时 */
		g_TTS.u32PlayingTick = 100 * 5;		/* 大约等5s的tick */
	}
	if(g_TTS.u32PlayingTick) {
		g_TTS.u32PlayingTick--;
	}
}

/* 控制灯效， u32LedsGpioToWrite：要写入的led,支持与操作以实现混色 */
void ControlLed(uint32 u32LedsToWrite)
{
	GPIO_write(GREEN_LED_DOutNo, ((u32LedsToWrite & LED_COLOR_GREEN) != 0));
	GPIO_write(BLUE_LED_DOutNo, ((u32LedsToWrite & LED_COLOR_BLUE) != 0));
	GPIO_write(RED_LED_DOutNo, ((u32LedsToWrite & LED_COLOR_RED) != 0));
}
/******************************** FILE END ********************************/
