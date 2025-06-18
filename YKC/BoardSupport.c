/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved
 *
 * File			: BSP.c
 * Author		: Wang Renfei
 * Description	: �弶֧���ļ�, ����Ӳ����ʼ��
 *				: �Լ�SYS/BIOS��ص����� : GPIO,SDSPI,UART,��̫��,USB,PWM
 *				: ����Ӳ������:ADC, CAP, RTC �ɸ���ģ�����г�ʼ��
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
/* ��Ŀ����ͷ�ļ� */
#include <string.h>
#include "GlobalVar.h"
#include "LibMath.h"
#include "MdlSys.h"

/* �����Լ��¼�ģ��ͷ�ļ� */
#include "BoardSupport.h"

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
| Description	: ��ʱ����ʼ�����ܹ�8����ʱ��
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

    /* ���� */
    {GPIOE, GPIO_PIN_8},        //�����������ͣ������
    {GPIOD, GPIO_PIN_6},        //ˮ�ùܵ�ˮλ���

#if USE_SELF_DEVELOP_SHELL
    {GPIOD, GPIO_PIN_3},        //������ײ����
    {GPIOD, GPIO_PIN_4},        //������ײ����
    {GPIOB, GPIO_PIN_0},        //������ײ����
    {GPIOC, GPIO_PIN_5},        //������ײ����

    {GPIOD, GPIO_PIN_2},        //�����ײ����
    {GPIOB, GPIO_PIN_1},        //ǰ����ײ����
    {GPIOD, GPIO_PIN_5},        //ǰ����ײ����
    {GPIOC, GPIO_PIN_4},		//�Ҳ���ײ����
#else
	{GPIOD, GPIO_PIN_4},        //����ǰ��ײ���� 3
    {GPIOD, GPIO_PIN_3},        //�������ײ���� 4
	{GPIOC, GPIO_PIN_5},        //����ǰ��ײ���� 5
    {GPIOB, GPIO_PIN_0},        //���Һ���ײ���� 6

	{GPIOD, GPIO_PIN_5},        //ǰ����ײ���� 7
	{GPIOC, GPIO_PIN_4},		//ǰ����ײ���� 8
    {GPIOD, GPIO_PIN_2},        //������ײ���� 9
    {GPIOB, GPIO_PIN_1},        //������ײ���� 10
#endif
    {GPIOA, GPIO_PIN_15},       //���¿���2 ���� 11
	{GPIOE, GPIO_PIN_7},        //���¿���3 ���� 12
	{GPIOE, GPIO_PIN_5},        //���¿���1 ���� 13
	{GPIOA, GPIO_PIN_3},        //���¿���4 ���� 14
    /* �̵��� */
    {GPIOD, GPIO_PIN_1}, 	      //����ˮ��
    {GPIOC, GPIO_PIN_12},        //����ˮ��
    /* �������� */
    {GPIOE, GPIO_PIN_10},       //�����˹���ģʽ��ɫָʾ�ƿ��� ��ɫ
    {GPIOE, GPIO_PIN_9},       	//�����˹���ģʽ��ɫָʾ�ƿ��� ��ɫ
	{GPIOE, GPIO_PIN_11},        //�����˹���ģʽ��ɫָʾ�ƿ��� ��ɫ
	{GPIOE, GPIO_PIN_0},        //�����豸��Դ

    /* ������Ե� */
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

/* �����Ĵ��� */
#define BRUSH_MOTOR_PWM_PERIOD  3600       /* 20KHz������˹��ˢ����ˢ����20KHz */
void Board_DrvWheel(float32 fRightDuty, float32 LeftDuty)
{
    extern TIM_HandleTypeDef htim3;
    /* ���Ĵ���H:�ߵ�ƽ��L:�͵�ƽ��P:PWM��� */
    if(fRightDuty == 0) {        /* ɲ��: HH */
    #if 1
        htim3.Instance->CCR1 = htim3.Instance->ARR + 1;
        htim3.Instance->CCR2 = htim3.Instance->ARR + 1;
    #else
        htim3.Instance->CCR1 = 0;
        htim3.Instance->CCR2 = 0;
    #endif
    } else if(fRightDuty > 0) {  /* ǰ��: LP, ����˹��PH */
        htim3.Instance->CCR1 = F32ToU16(htim3.Instance->ARR*fRightDuty);
        htim3.Instance->CCR2 = 0;
    } else if(fRightDuty < 0) {  /* ����: PL, ����˹��PH */
        htim3.Instance->CCR1 = 0;
        htim3.Instance->CCR2 = F32ToU16(htim3.Instance->ARR*(0-fRightDuty));
    }

    /* ���Ĵ���H:�ߵ�ƽ��L:�͵�ƽ��P:PWM��� */
    if(LeftDuty == 0) {        /* ɲ��: HH */
    #if 1
        htim3.Instance->CCR3 = htim3.Instance->ARR + 1;
        htim3.Instance->CCR4 = htim3.Instance->ARR + 1;
    #else
        htim3.Instance->CCR3 = 0;
        htim3.Instance->CCR4 = 0;
    #endif
    } else if(LeftDuty > 0) {  /* ǰ��: LP, ����˹��PH */
        htim3.Instance->CCR3 = F32ToU16(htim3.Instance->ARR*LeftDuty);
        htim3.Instance->CCR4 = 0;
    } else if(LeftDuty < 0) {  /* ����: PL, ����˹��PH */
        htim3.Instance->CCR3 = 0;
        htim3.Instance->CCR4 = F32ToU16(htim3.Instance->ARR*(0-LeftDuty));
    }
    //htim3.Instance->ARR = BRUSH_MOTOR_PWM_PERIOD;    
}

/* ������ձ� */
void Board_DrvPump(float32 fDuty)
{
    extern TIM_HandleTypeDef htim2;
    if(fDuty <= 0) {        /* ͣ������˹����������92%ռ�ձ� */
       htim2.Instance->CCR3 = 0;
        htim2.Instance->CCR4 = F32ToU16(htim2.Instance->ARR*0.02f);
    } else {
        htim2.Instance->CCR3 = htim2.Instance->ARR + 1;
        htim2.Instance->CCR4 = F32ToU16(htim2.Instance->ARR*fDuty);
    }
    //htim2.Instance->ARR = BRUSH_MOTOR_PWM_PERIOD;
}


/*==========================================================================
| Description	: AD����ģ��
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2008-3-4
\=========================================================================*/
void InitAdc(void);
void InitAdc(void) {

}

void InitSample(void)
{
	int8 i;
	
	/* ��ʼ������ */
	/* <NULL> */

	/* ��ʼ������ӿڱ��� */

	/* ��ʼ������ӿڱ��� */
	for(i = 0; i < DC_TOTAL_CHN; i++) {
		g_DCSigADSamp[i].uSampOkDecCnt = ANALOG_ADSAMP_BUF_LEN;
		g_DCSigADSamp[i].uCurSampPt = 0;
	}
	for(i = 0; i < PULSE_SIG_CHN; i++) {
      	InitUDatUniRec((uDAT_UNI_REC*)&g_PulseCap[i], PULSE_CYCLE_BUF_LEN);
    }

	/* ��ʼ���ڲ�ȫ�ֱ��� */
	for(i = 0; i < MAX_DIN_NUM; i++) {
		g_uDInFilt[i] = 0xFFFF;
	}

	/* ��ʼ���²�ģ�� */
	
	/* ����Ӳ�� */
	InitAdc();
}

/*==========================================================================
| Description	: ����Tick
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-4-30
\=========================================================================*/
extern ADC_HandleTypeDef hadc2;
void DrvSampleTick_1KHz(void)
{
	int8 i;
	/* ���¿��� */
	for(i = MAX_DIN_NUM - 1; i >= 0; i--) {
		g_uDInFilt[i] = (g_uDInFilt[i]<<1) + GPIO_read(GPIO_DIN_StartNo + i);
	}
}

/*==========================================================================
| Description	: AD�ж���Ӧ������������ݴ洢
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
| Description	: ����Ĵ����Ҳ��Ĵ�����ˢ��������١�����ң��
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

/* ����ң������ʼ�� */
void IrRecvInit(uint8 u8SelectedrIndex)
{
	g_IRCtrl.DevComm[u8SelectedrIndex].bBtnRdyDecode = FALSE;						/* δ���������� */
	g_IRCtrl.DevComm[u8SelectedrIndex].bIsIdle = TRUE;								/* ����ǿ���״̬ */
	g_IRCtrl.DevComm[u8SelectedrIndex].u8TimerUpdatCount = 0;							/* ��ʱ�������0 */
	g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPulseCount = 0;						/* ���񵽵ĵ�λ�仯������0 */
    memset((void*)g_IRCtrl.DevComm[u8SelectedrIndex].rxFrame, 0x00, sizeof(g_IRCtrl.DevComm[u8SelectedrIndex].rxFrame));
}

void IrRecvCapture(uint8 u8SelectedrIndex,TIM_HandleTypeDef *pHtim, uint32 u32Ccr, uint16 uCapChannl)
{
	uint16 uCapTime = 0;
	static uint16 s_uCapTime_last = 0;
	uCapTime = u32Ccr;

	TIM_RESET_CAPTUREPOLARITY(pHtim, uCapChannl);                      /* ��λ�������� */

	if(g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPol == 0) { 		/* ��˵������ط�����Ū����ô���ӣ����ǿ������ó��������½��ض����Դ�����ô�� ��iocͼ�ν�����ֻ�����ó�һ������*/
		TIM_SET_CAPTUREPOLARITY(pHtim, uCapChannl, TIM_INPUTCHANNELPOLARITY_RISING); 		/* �ı伫�� */
		g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPol = 1;        	/* ���Ա�־λ��Ϊ������ */
	} else {
		TIM_SET_CAPTUREPOLARITY(pHtim, uCapChannl, TIM_ICPOLARITY_FALLING);					/* �ı伫�� */
		g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPol = 0;
	}
	if(!g_IRCtrl.DevComm[u8SelectedrIndex].bIsIdle) {              /* �����ǰΪ����״̬�����в��񵽵�ʱ��Ϊ��һ���½��� */
		IrRecvInit(u8SelectedrIndex);
	} else {
		if(g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPulseCount < IR_RX_SEQ_BLEN*2) {
			/* ����������з��յģ����ܣ�T5����Ͳ�׽ͬʱ��������û���ǡ�����T5Ƶ�ʣ�16bit��ʱ��Ӧ�����㹻�� */
			/* ����������Դ��ȷ�ϣ����ͬʱ������InputCapture()������PeriodElapse()֮��ִ�� */
			g_IRCtrl.DevComm[u8SelectedrIndex].rxFrame[g_IRCtrl.DevComm[u8SelectedrIndex].u8CapPulseCount] = g_IRCtrl.DevComm[u8SelectedrIndex].u8TimerUpdatCount*65536 + uCapTime - s_uCapTime_last;		/* ����g_IRCtrl.u8Tim_udt_count*65536��Ϊ֧�ֿ���� */
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
	/* ���º���ң����״̬ */
	if(g_IRCtrl.DevComm[u8SelectedrIndex].bIsIdle) {							/* ����״̬�������κδ��� */
		g_IRCtrl.DevComm[u8SelectedrIndex].u8TimerUpdatCount++;					/* ���һ�� */
		if(g_IRCtrl.DevComm[u8SelectedrIndex].u8TimerUpdatCount == 2) {			/* ���2�� */
			g_IRCtrl.DevComm[u8SelectedrIndex].u8TimerUpdatCount = 0;				/* ����������� */
			g_IRCtrl.DevComm[u8SelectedrIndex].bIsIdle = FALSE;					/* ����Ϊ����״̬ */
			g_IRCtrl.DevComm[u8SelectedrIndex].bBtnRdyDecode = TRUE;			/* ������һ֡������ɣ�Ӧ���Ǳ߿��Խ����� */
		}
	}
}

/* ��¼��������������������ǿ���������TIM�����������Ϊ�ɽ��� */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *pHtim)
{
    //ProcCap(htim0.Instance->CCR1, 0);
    //ProcCap(htim0.Instance->CCR1, 0);
	if(TIM5 == pHtim->Instance) {							/* ����100Hz */
		/* ���º���ң����״̬ */
		IrRecvDecode(0);

		/* ��������һ�ߺ���ң����״̬ */ /* �����ߵĺ���Ҫͬʱ����û���Ⱥ� */
		IrRecvDecode(1);
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *pHtim)
{
    if(pHtim->Instance == TIM1) {
        if(pHtim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {         /* �Ҳ���ˢ������� */
            uint16 uCapTime = pHtim->Instance->CCR2;
            UniRecUDatToBuf((uDAT_UNI_REC*)&g_PulseCap[RIGHT_MOTOR_PulseNo], PULSE_CYCLE_BUF_LEN, uCapTime - g_PulseCap[RIGHT_MOTOR_PulseNo].uPulseEdge);
            g_PulseCap[RIGHT_MOTOR_PulseNo].uPulseEdge = uCapTime;
            g_MsrRes.uRightCount++;
        } else if(pHtim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {  /* ��ˢ������� */
            uint16 uCapTime = pHtim->Instance->CCR3;
            UniRecUDatToBuf((uDAT_UNI_REC*)&g_PulseCap[BUMP_MOTOR_PulseNo], PULSE_CYCLE_BUF_LEN, uCapTime - g_PulseCap[BUMP_MOTOR_PulseNo].uPulseEdge);
            g_PulseCap[BUMP_MOTOR_PulseNo].uPulseEdge = uCapTime;
        } else if(pHtim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {  /* �����ˢ������� */
            uint16 uCapTime = pHtim->Instance->CCR4;
            UniRecUDatToBuf((uDAT_UNI_REC*)&g_PulseCap[LEFT_MOTOR_PulseNo], PULSE_CYCLE_BUF_LEN, uCapTime - g_PulseCap[LEFT_MOTOR_PulseNo].uPulseEdge);
            g_PulseCap[LEFT_MOTOR_PulseNo].uPulseEdge = uCapTime;
            g_MsrRes.uLeftCount++;
         }
	} else if(TIM5 == pHtim->Instance) { /* ���������Ⲷ�� */
		if(pHtim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			IrRecvCapture(0, pHtim, pHtim->Instance->CCR2, TIM_CHANNEL_2);
		}

		if(pHtim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			IrRecvCapture(1, pHtim, pHtim->Instance->CCR3, TIM_CHANNEL_3);
		}
	}
}

/* �������岶׽�źŽ��д��� */
float32 ProcPulseSig(uint8 u8PulseNo)
{
	/* ά���������:���ڲ�׽��ʱ����16bit��Ϊ�˱����������������ڲ�����׼����û�������ʱ�򣬲���һ����ٵ�"uPulseEdge" */
	uint16 uCurTime = GetCapCurTime_1MHz();
	UInt HwiKey = Hwi_disable();
	if(uCurTime - g_PulseCap[u8PulseNo].uPulseEdge > PERIOD_CTR_TASK_ms*1000) {	/* �����ת����һ�����������ڣ��϶��������� */
		g_PulseCap[u8PulseNo].uPulseEdge = uCurTime - PERIOD_CTR_TASK_ms*1000;	/* ������� */
		InitUDatUniRec((uDAT_UNI_REC*)&g_PulseCap[u8PulseNo], PULSE_CYCLE_BUF_LEN);
	}
	Hwi_restore(HwiKey);

	/* ���м��� */
	if(ProcUniRecUDatAsWave((uDAT_UNI_REC*)&g_PulseCap[u8PulseNo], PULSE_CYCLE_BUF_LEN, 20)) {
		return CAP_CLK_FREQ_Hz/g_PulseCap[u8PulseNo].fSig_avr;
	} else {
		return 0;
	}
}

#include "BMI270.h"		/* IMU */
/* ����IMU����ͼ�����̬���ݣ��������ڲ����Ƶ�ʿ����÷�Χ��0.02-1.6KHz���ɿ����ĵ�ͨ�˲�Ƶ�ʷ�Χ��5.5-751Hz*/
void UpdateIMU(void)
{
    /* �����������ݼ������ƽǶ����� */
	Bmi270GetAcc();             /* ��ȡ IMU660RA �ļ��ٶȲ�����ֵ */
	Bmi270GetGyro();            /* ��ȡ IMU660RA �Ľ��ٶȲ�����ֵ */
	static float32 s_fQ0 = 1, s_fQ1 = 0, s_fQ2 = 0, s_fQ3 = 0;      /* ��Ԫ����Ԫ�أ�������Ʒ��� */
	static float32 s_fExInt = 0, s_fEyInt = 0, s_fEzInt = 0;      /* ��������С������� */
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

		/* �����һ������ */
		float32 fNorm = sqrtf(fAx * fAx + fAy * fAy + fAz * fAz);

		if(fNorm != 0) {
			fAx = fAx / fNorm;                   /* ��һ�� */
			fAy = fAy / fNorm;
			fAz = fAz / fNorm;

			/* ���Ʒ�������� */
			float32 fVx = 2 * (s_fQ1 * s_fQ3 - s_fQ0 * s_fQ2);
			float32 fVy = 2 * (s_fQ0 * s_fQ1 + s_fQ2 * s_fQ3);
			float32 fVz = s_fQ0 * s_fQ0 - s_fQ1 * s_fQ1 - s_fQ2 * s_fQ2 + s_fQ3 * s_fQ3;

			/* ���������ͷ��򴫸��������ο�����֮��Ľ���˻����ܺ� */
			float32 fEx = (fAy * fVz - fAz * fVy);
			float32 fEy = (fAz * fVx - fAx * fVz);
			float32 fEz = (fAx * fVy - fAy * fVx);

			/* ������������������ */
			s_fExInt = s_fExInt + fEx * KI;
			s_fEyInt = s_fEyInt + fEy * KI;
			s_fEzInt = s_fEzInt + fEz * KI;

			/* ������������ǲ��� */
			fGx = fGx + KP * fEx + s_fExInt;
			fGy = fGy + KP * fEy + s_fEyInt;
			fGz = fGz + KP * fEz + s_fEzInt;

			/* ������Ԫ���ʺ������� */
			s_fQ0 = s_fQ0 + (-s_fQ1 * fGx - s_fQ2 * fGy - s_fQ3 * fGz) * HALF_T;
			s_fQ1 = s_fQ1 + (s_fQ0 * fGx + s_fQ2 * fGz - s_fQ3 * fGy) * HALF_T;
			s_fQ2 = s_fQ2 + (s_fQ0 * fGy - s_fQ1 * fGz + s_fQ3 * fGx) * HALF_T;
			s_fQ3 = s_fQ3 + (s_fQ0 * fGz + s_fQ1 * fGy - s_fQ2 * fGx) * HALF_T;

			/* ������Ԫ���������� */
			fNorm = sqrtf(s_fQ0 * s_fQ0 + s_fQ1 * s_fQ1 + s_fQ2 * s_fQ2 + s_fQ3 * s_fQ3);

			if(fNorm != 0) {
				/* ��һ����Ԫ�� */
				s_fQ0 = s_fQ0 / fNorm;
				s_fQ1 = s_fQ1 / fNorm;
				s_fQ2 = s_fQ2 / fNorm;
				s_fQ3 = s_fQ3 / fNorm;
				/* ����ȫ�ֱ��� */
				g_MsrRes.iGyroX = g_Bmi270Comm.iGyrX;
				g_MsrRes.iGyroY = g_Bmi270Comm.iGyrY;
				g_MsrRes.iGyroZ = g_Bmi270Comm.iGyrZ;
				g_MsrRes.iAccX = g_Bmi270Comm.iAccX;
				g_MsrRes.iAccY = g_Bmi270Comm.iAccY;
				g_MsrRes.iAccZ = g_Bmi270Comm.iAccZ;
				g_MsrRes.fPitch  = asinf(-2 * s_fQ1 * s_fQ3 + 2 * s_fQ0 * s_fQ2) * 57.3; /* pitch ,ת��Ϊ���� */
				if(g_Bmi270Comm.iAccY > 0) {	/* ����װ���� */
					if(g_MsrRes.fPitch > 0) {
						g_MsrRes.fPitch = 180 - g_MsrRes.fPitch;
					} else {
						g_MsrRes.fPitch = 0 - 180 - g_MsrRes.fPitch;
					}
				}
				g_MsrRes.fPitch += 90;		/* ���Ϸ���0�� */
			/*	if (g_MsrRes.fPitch < 0) {
				    g_MsrRes.fPitch += 360; // ����Ǹ���������360��
				}*/
				g_MsrRes.fRoll = atan2f(2 * s_fQ2 * s_fQ3 + 2 * s_fQ0 * s_fQ1, -2 * s_fQ1 * s_fQ1 - 2 * s_fQ2 * s_fQ2 + 1) * 57.3; /* roll */
				g_MsrRes.u8TryCnt_IMU = 5;
				return;
			}
		}
	}
	/* ������������˴�ͨ��ʧ���� */
	if(g_MsrRes.u8TryCnt_IMU) {
		g_MsrRes.u8TryCnt_IMU--;
	}
}

#include "BMP280.h"		/* ��ѹ */
/* ������ѹ�� */
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

/* ���ڲ��񵽵�ʱ���������ģ����Գ�����ֱ���ж�ֵ�Ƿ���ȣ������ô˺����ж�����ֵ�Ƿ�ӽ���ֻ��Ҫ�ӽ���׼ֵ���� */
uint8 IsIrTimeCounterMatch(int16 iTimerCounter1, int16 iTimerCounter2)
{
	return (((iTimerCounter1 - iTimerCounter2) < 0) ? - (iTimerCounter1 - iTimerCounter2) : (iTimerCounter1 - iTimerCounter2)) < 300;	/* 300us */
}

/* ����HX1838����ң��ͨ��״̬ */
void UpdateIRCtrl(void)
{
	BOOL bSuccess = TRUE;
	uint32 u32Key = Hwi_disable();

	if(g_IRCtrl.DevComm[0].bBtnRdyDecode) { /* ����д������ź�  ���Ȳ���һ��λ�ĺ��� */
		g_IRCtrl.u8SelectedrIndex = 0;
	} else if(g_IRCtrl.DevComm[1].bBtnRdyDecode) {
		g_IRCtrl.u8SelectedrIndex = 1;
	}

	if(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].bBtnRdyDecode) {		/* ����д������ź� */
		/* ����ָ��֡ */
		memcpy(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData, g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].rxFrame, IR_RX_SEQ_BLEN * 4);
		g_IRCtrl.DevComm[1].bBtnRdyDecode = FALSE;
		g_IRCtrl.DevComm[0].bBtnRdyDecode = FALSE;
		Hwi_restore(u32Key);
		if(IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[0], 9000)
			&& IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[1], 4500))
		{        /* 9ms + 4.5ms���ǰ���� */
			uint8 u8BitIdx = 0;
			uint8 i;
			g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].uRepeatCount = 0;
			for(i = 2; i < (IR_RX_SEQ_BLEN * 2); i++) {                                  				/* ������� */
				if(!IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[i], 560)) {									/* 0.56ms */
					bSuccess = FALSE;
					break;
				}
				i++;
				if(IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[i], 1680) || (g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[i] > 10000)) {	/* 1.68ms �����10ms  */
					g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].data.u32AllData |= (0x80000000 >> u8BitIdx);                          		/* ��u8BitIdxλ��1 */
					u8BitIdx++;
				} else if(IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[i], 560)) {								/* 0.56ms */
					g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].data.u32AllData &= ~(0x80000000 >> u8BitIdx);                         		/* ��u8BitIdxλ��0 */
					u8BitIdx++;
				} else {
					bSuccess = FALSE;
					break;
				}
			}
		} else if(IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[0], 9000)
				&& IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[1], 2250)
				&& IsIrTimeCounterMatch(g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].srcData[2], 560))
		{	/* 9ms + 2.25ms + 0.56ms Ϊ�ظ��룬����ס������ */
			g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].uRepeatCount++;
			bSuccess = TRUE;
		} else {
			bSuccess = FALSE;		/* ǰ��������� */
		}
	} else {
		/* ���޴�����ָ��֡ */
		Hwi_restore(u32Key);
		bSuccess = FALSE;
	}

    if(bSuccess) {
		g_IRCtrl.u8BtnPressing = g_IRCtrl.DevComm[g_IRCtrl.u8SelectedrIndex].data.bit.key_val;		/* ʹ�õ�ʱ��ֱ�Ӹ��ݼ�ֵ�����ж��� */
    	g_IRCtrl.u8TryCnt = 50;			/* ״̬����500ms */
    } else if(g_IRCtrl.u8TryCnt) {
    	g_IRCtrl.u8TryCnt--;
    } else { /* ��û������״̬ʱ��������״̬���� */
    	g_IRCtrl.u8BtnPressing = IR_BTN_NONE;
    }
}
/*����ģ��***************************************************************************************/
#include "queue.h"
#define TTS_UART_NO	1

#define QUEUE_SIZE	10
typedef struct {
    uint32 u32Queue[QUEUE_SIZE];  	/* ��̬������Ϊ������ */
    uint8 u8DeqFront;               /* ��ͷָ��        */
    uint8 u8EnqRear;                /* ��βָ��        */
}CIRCULAR_QUEUE;					/* ���ڻ���uint32���͵Ļ��ζ��� */

/* ��ʼ�����ζ���  [�̰߳�ȫ] */
void CirQueInit(CIRCULAR_QUEUE *queue) {
	Swi_disable();
    queue->u8DeqFront = 0;
    queue->u8EnqRear = 0;
    Swi_enable();
}

/* ��Ӳ��� [�̰߳�ȫ]
 * bIgnoreDup���Ƿ�����ظ�����������������Ƿ��Ѿ����ڣ���������ٲ��� */
BOOL CirQueEnq(CIRCULAR_QUEUE *queue, uint32 u32EnqItem, BOOL bIgnoreDup) {
	BOOL bRes = FALSE;
	Swi_disable();
    uint8 next_rear = (queue->u8EnqRear + 1) % QUEUE_SIZE;
    if (next_rear != queue->u8DeqFront) {  /* ����δ�� */
		uint32 u32QueIndex = 0;
    	if (!bIgnoreDup) {
			/* ���������Ƿ����и�Ԫ�� */
			for (u32QueIndex = queue->u8DeqFront; u32QueIndex != queue->u8EnqRear; u32QueIndex = (u32QueIndex + 1) % QUEUE_SIZE) {
				if (queue->u32Queue[u32QueIndex] == u32EnqItem) {
				    bRes = TRUE;		/* �����ظ�������� */
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

/* ���Ӳ���, pDeqItem�ɵ�����ȷ����ΪNULL [�̰߳�ȫ] */
BOOL CirQueDeq(CIRCULAR_QUEUE *pQue, uint32* pDeqItem) {
	BOOL bRes = FALSE;
	Swi_disable();
    if (pQue->u8DeqFront != pQue->u8EnqRear) {  // ���зǿ�
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

/* ���г�ʼ�� [�̰߳�ȫ] */
BOOL InitTtsSpeaking(void)
{
	OpenUartComm(TTS_UART_NO, 9600, 0, 100);
	CirQueInit(&g_TTS.Que_Speaking);			/* ����ģ�鲥�Ŷ��� */
	g_TTS.u32PlayingTick = 0;
	return TRUE;
}
/* ����ɹ����벥�Ŷ����򷵻�TRUE��������������򷵻�FALSE [�̰߳�ȫ] */
BOOL TtsSpeak(TTS_DIALOG_TYPE DlgType, BOOL bIgnoreDup)
{
	return CirQueEnq(&g_TTS.Que_Speaking, DlgType, bIgnoreDup);
}

#include "MdlUARTnModbus.h"
#define TTS_HEADER_LEN	5
/* ����TTS [�̰߳�ȫ] */
void UpdateTTS(void)
{
	uint32 u32CurrentReqType;
	/* ��������д����ŵ����� */
	if(!g_TTS.u32PlayingTick && CirQueDeq(&g_TTS.Que_Speaking, &u32CurrentReqType)) {
		/* v[0~16]:0�ʶ�����Ϊ������16�ʶ��������Ĭ����10
		  m[0~16]:0��������Ϊ������16���������������Ĭ����4
		  t[0~5]:0�ʶ�����������5�ʶ�������죬Ĭ����4*/
		uint8 *pBuf = g_UartComm[TTS_UART_NO].u8TRBuf;
		const char* pHcData = cnst_TtsDialog[(TTS_DIALOG_TYPE)u32CurrentReqType];
		uint8 u8HcLength = strlen(pHcData);
		uint8 u8Encode = 1;					/* �ı������ʽ��0��GB2312��1��GBK��2��BIG5��3��UNICODE*/
		uint8 u8Music = 0;					/* ѡ�񱳾�����(0���ޱ�������  1-15���������ֿ�ѡ)*/

		/* �̶�֡��Ϣ */
		*pBuf++ = 0xFD ; 					/* ����֡ͷFD*/
		*pBuf++ = 0x00 ; 					/* �������������ȵĸ��ֽ�*/
		*pBuf++ = u8HcLength + 3; 			/* �������������ȵĵ��ֽڣ��������֡����������У��������ֽڣ����������ȱ����ϸ�һ��*/
		*pBuf++ = 0x01 ; 					/* ���������֣��ϳɲ�������*/
		*pBuf++ = u8Encode | u8Music << 4 ; /* ������������������ʽ�趨�����������趨*/

		/* У������� */
		int8 i = 0;
		uint8 u8Ecc = 0;
		for(i = 0; i < TTS_HEADER_LEN; i++) {
			u8Ecc ^= g_UartComm[TTS_UART_NO].u8TRBuf[i];
		}
		for(i = 0; i < u8HcLength; i++) {
			u8Ecc ^= pHcData[i];
		}

		/* ƴ��֡ */
		PrintStringNoOvChk(&pBuf, pHcData);
		*pBuf++ = u8Ecc;

		/* ����֡ */
		WriteToUart(TTS_UART_NO, TTS_HEADER_LEN + u8HcLength + 1);

		/* �ȴ���ʱ */
		g_TTS.u32PlayingTick = 100 * 5;		/* ��Լ��5s��tick */
	}
	if(g_TTS.u32PlayingTick) {
		g_TTS.u32PlayingTick--;
	}
}

/* ���Ƶ�Ч�� u32LedsGpioToWrite��Ҫд���led,֧���������ʵ�ֻ�ɫ */
void ControlLed(uint32 u32LedsToWrite)
{
	GPIO_write(GREEN_LED_DOutNo, ((u32LedsToWrite & LED_COLOR_GREEN) != 0));
	GPIO_write(BLUE_LED_DOutNo, ((u32LedsToWrite & LED_COLOR_BLUE) != 0));
	GPIO_write(RED_LED_DOutNo, ((u32LedsToWrite & LED_COLOR_RED) != 0));
}
/******************************** FILE END ********************************/
