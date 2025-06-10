#if (!SOFT_RUN1_TEST0)

/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved
 *
 * File			: stencil.c
 * Author		: Wang Renfei
 * Description	: c file's stencil, V1.1 inherit from V1.0
 *				: ģ�黯������ṹ��ÿһ��ģ����ͷ�ļ��ж����ⲿ�ӿڱ���,�ⲿ��������,ģ����÷�����������Դ�ļ��ж���ģ���ڲ�����������
 * Comments		:
 * Date			: 2006-11-27
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 * V0.1 | 2023-12-08  | king | 1. ��ʼ��
 **************************************************************************/

/***************************************************************************
 						macro definition
***************************************************************************/
#define _MDL_YKC_C_		/* exclude redefinition */

/***************************************************************************
 						include files
***************************************************************************/
#include <string.h>
#include <stdio.h>

/* ��Ŀ����ͷ�ļ� */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"

/* �����Լ��¼�ģ��ͷ�ļ� */
#include "MdlNet.h"
#include "MdlDataAccess.h"
#include "BoardSupport.h"
#include "MdlUARTnModbus.h"
#include "MdlGUI.h"
#include "MdlGprs.h"    
#include "BMI270.h"		/* IMU */
#include "BMP280.h"		/* ��ѹ */

#include "MdlYKC.h"

/* ����ģ�� */
void UpdateIMU(void);
void UpdateAir(void);
void UpdateIRCtrl(void);
/***************************************************************************
 						global variables definition
***************************************************************************/
const CONF_n_STAT_PROP cnst_YkcConfProp[] = {                                              /* 1 2 3 4 5 6 7 8 9 A B C D E F 1 2 3 4 5 6 7 8 9 A B C D E F */
    {1001, {ITEM_DAT_TOPIC, 0, 0, SAVE_GRP_NUL}, 3, NULL,                      {"ϵͳ��ͨѶ����", "Default"}},
    {1002, {ITEM_DAT_SOFTVER, 0, 0, SAVE_GRP_NUL}, SOFTWARE_VER, NULL,                {"����汾", "Soft Version"}},
    {1003, {ITEM_DAT_U32_ID_RO, 0, 0, SAVE_GRP_BRD}, 0, (void*)&g_Sys.SerialNo,       {"���к�", "S/N:"}},
    {1004, {ITEM_DAT_T32_RO, 0, 0, SAVE_GRP_BRD}, 0, (void*)&g_Sys.u32License_Seconds,{"��Ȩ����(������ʱ)", "deadline"}},
};

/* EEProm��ַ���� */
/* EEProm��ַ����, �б���TI����λ��������˳��洢��ST���÷�BANK�洢���������� */
#define EEPROM_ADDR_BRD_MAIN		(EEADD_DEV_CONF_START						+ 0*64)
#define EEPROM_ADDR_BRD_BAK			(EEPROM_ADDR_BRD_MAIN+FLASH_HALF_BLEN)
#define EEPROM_ADDR_MCONF_MAIN		(EEPROM_ADDR_BRD_MAIN						+ 5*64)
#define EEPROM_ADDR_MCONF_BAK		(EEPROM_ADDR_MCONF_MAIN+FLASH_HALF_BLEN)
extern void _c_int00(void);
volatile const SECTION(".BootLoaderConf") BOOT_LOADER_CONF cnst_BootLoaderConf = {
	(uint32)&_c_int00, (CONF_n_STAT_PROP*)cnst_YkcConfProp, sizeof(cnst_YkcConfProp)/sizeof(CONF_n_STAT_PROP), MAX_SAVE_GRP_NUM,
	{{0, 0, 0, 0},
	{EEPROM_ADDR_BRD_MAIN, EEPROM_ADDR_BRD_BAK, 0, 0},
	{EEPROM_ADDR_MCONF_MAIN, EEPROM_ADDR_MCONF_BAK, 0, 0},
	{0, 0, 0, 0},
	{0, 0, 0, 0},
	{0, 0, 0, 0}}
};

typedef struct {
	float32 fVacuumDuty;
	float32 fLeftWheelDuty;
	float32 fRightWheelDuty;
	uint16 uTimer_100Hz;
    uint16 uLastPnlCtrBtn;
	int8 i8Tmr_AutoTest_pActRelay_nIntv_tick;
	uint8 u8OnRelayNo;
	triST tMotorTest_0Idle_pInc_nDec;
	BOOL bMainSwitch;						/* �ܿ��� */
	uint8 u8ErrorCnt; 						/* ���ڼ�¼�쳣���� */
	uint8 u8Rsvd;
	uint16 uKeyCnt_Delay; 					/* ������ʱʱ����� */
	uint16 uKeyCnt_Jitter; 					/* ��������ʱ����� */
	uint16 uWaterPumpCnt; 					/* ����ˮ�����δ򿪵�ʱ����� */
}YKC_TEST;
YKC_TEST g_YKCTest;

extern UART_HandleTypeDef huart4;

/***************************************************************************
						internal functions declaration
***************************************************************************/


/***************************************************************************
 							functions definition
***************************************************************************/
/*==========================================================================
| Description	: 
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2023-12-8
\=========================================================================*/
void InitMdlCtr(void)		/* rename from InitYKC() */
{
	/* ��ʼ������ */

	/* ��ʼ������ӿڱ��� */

	/* ��ʼ������ӿڱ��� */
	InitDataWithZero((uint8*)(&g_MsrRes), sizeof(g_MsrRes));
	InitDataWithZero((uint8*)(&g_AnaRes), sizeof(g_AnaRes));
	g_Ctr.u32DinDat = 0;

	/* ��ʼ���ڲ�ȫ�ֱ��� */
	InitDataWithZero((uint8*)(&g_YKCTest), sizeof(g_YKCTest));
	g_YKCTest.bMainSwitch = FALSE;
	g_YKCTest.fVacuumDuty = 0;
	g_YKCTest.fLeftWheelDuty = 0;
	g_YKCTest.fRightWheelDuty = 0;

	/* ��ʼ���²�ģ�� */
	
	/* ����Ӳ�� */
	InitSample();		/* ���ڲ�����Ҫ���buf��Ȼ��źü��㣬����������� */

	GPIO_write(OTHER_DEV_DOutNo, 1);		/* ʹ���ܱ��豸3.3v���� */

	/* ��ʼ�����ᴫ���� */
    if(!Bmi270Init()) {
		/* IMU��ʼ��ʧ��*/
		configASSERT(FALSE);
	}
	/* ��ʼ����ѹ�� */
    if(!Bmp280Init()) {
		/* IMU��ʼ��ʧ��*/
		configASSERT(FALSE);
	}

	/* ��ʼ������ң����  TIM5Ƶ��1MHz, ARR==65535*/
	extern TIM_HandleTypeDef htim5;
	HAL_TIM_Base_Start_IT(&htim5);	 			/* ������ʱ���жϣ�֮����û����main.c�����Ϊ���ڳ�ʼ������֮���ٿ�ʼ */
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);  /* ������ʱ�����벶��֮����û����main.c�����Ϊ���ڳ�ʼ������֮���ٿ�ʼ */
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);

	/* ��ʼ������ģ�� */
	InitTtsSpeaking();

#if (0 && (!SOFT_RUN1_TEST0))
	/* GPRS���� */
	extern UART_HandleTypeDef huart4;
	SOCKET SocketFd; /* �ٶ�����׽��� */
	struct sockaddr_in Socket_Gprs; /* �����õ�ַ�� SOCKADDR_IN*/

	/* GPRS���ӱ��� */
	int8 i8S_Result, i8R_Result, i8C_Result_Close, i8C_Result_Connect = -1;
	uint8 aU8GprsBuffer[500];

	/* ���ӰٶȲ��� (1��) */
	/* �ٶ�ip��ַ���� */
#define BAIDU_IP "39.156.66.10"  			/* �ٶȵ�IP��ַ��ע����ʱ���ܻ�仯 */
#define BAIDU_PORT 80  						/* �ٶ�HTTP�˿ں� */
	memset(&Socket_Gprs, 0, sizeof(Socket_Gprs));
	Socket_Gprs.sin_family = AF_INET; /* ��ַ�����ipv4ͨ�� */
	Socket_Gprs.sin_port = htons(80); /* ���˿ں�תΪ�����ֽ� */
	Socket_Gprs.sin_addr.s_addr = htonl(0x279c420a); /* ����ַתΪ�����ֽ���Ȼ����תΪ�����ֽ��� */
	SocketFd = GprsSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP);  /* ʹ����ʽ�׽������� */

	while(i8C_Result_Connect != GPRS_SUC) { /* �ظ����ӱ�֤������ */
		i8C_Result_Connect = GprsConnect(SocketFd, &Socket_Gprs, sizeof(Socket_Gprs));
	}

	sprintf((uint8*)aU8GprsBuffer, "GET /?st=1 HTTP/1.1\r\nHost:www.baidu.com\r\n\r\n");
	i8S_Result = GprsSend(SocketFd, aU8GprsBuffer, strlen(aU8GprsBuffer), 0);
	while(i8S_Result > 0) { /* ���ɹ���������󣬽��н���(ע��Ҫѭ�����գ���ҳ������Ϣ����һ�η���) */
		i8R_Result = GprsRecv(SocketFd, aU8GprsBuffer, sizeof(aU8GprsBuffer), 0);
		if(i8R_Result > 0) { /* �ɹ����յ����� */
			sprintf(aU8GprsBuffer, "receive:%d byte\r\n", i8R_Result);
			HAL_UART_Transmit(&huart4, (const uint8 *)aU8GprsBuffer, strlen(aU8GprsBuffer), 500);

			/* �����յ��Ļظ����͵������У�������� */
			HAL_UART_Transmit(&huart4, (const uint8 *)aU8GprsBuffer, strlen(aU8GprsBuffer), 500);

			/* �����꽫���ӹر� */
			i8C_Result_Close = GprsClose(SocketFd);
			if(i8C_Result_Close == GPRS_SUC) { /* �رճɹ� */
				sprintf(aU8GprsBuffer, "finish closed\r\n");
				HAL_UART_Transmit(&huart4, (const uint8 *)aU8GprsBuffer, strlen(aU8GprsBuffer), 500);
			}
			break;
		}
	}
#endif
}

/* ����Ƿ񵽴��������� */
BOOL CheckToStart(void)
{
	return TRUE;
}
/*==========================================================================
| Description	: 
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2023-12-8
\=========================================================================*/
void UpdateTTS(void);
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_gpio.h"
/*1�����Գ���ص㣺
MdlTest.c
һ����ִ�еĺ�����InitMdlCtr()
ÿ40ms��ִ��һ�εĺ�����RunMdlCtr()

2�����Բ��裺
�ϵ����ɫ�ư��պ졢�̡���˳�����������1s����ʼִ�в��ԣ�;���������쳣��ͨ���������ʶ�������ȫ���������Խ��������׵ƴ�����Զϵ������
����Ŀ�꣺
1�����������Ĵ����������ת���𲽼ӿ졢ͣ�¡���ת(���в��Դ���)���ò��������⿴���Խ��
2��һ����ˢ���������g_YKCTest.fVacuumDuty���1��Ҳ��������ŷ磬�ò��������⿴���Խ��
3����ѹ�ƣ�g_MsrRes.fAirPressure�Ƿ����9000000��С��1020000����λ��Pa��
4�������ǣ�g_MsrRes.iAccZ����0
5��GPRS�����ٶȷ���GET���󣬲����յ���������
6�����룺8����ײ��⣨��⵽��ײ��������ʾ���ĸ���⵽�˱պϣ�����
	4�����£���⵽��ײ��������ʾ���ĸ���⵽�˱պϣ�����
	һ���ܰ������أ������������Ĵ����ԡ��ŷ���ԡ�ˮ�ò��Թ��ܣ����ػ����Ĵ����ŷ硢ˮ�ã�����
	һ��ˮ�ùܵ���ˮ��⣨ֱ��ˮ�ã�����ǰ�Ⱥ�Ӳ������ҫȷ�ϣ�����
7������������ˮ�ý���򿪣�Ȼ���ȡ�����Ǳ��Ƿ���1���������1˵�����뿪��ȫ��OK�������ʶ��ڼ��������쳣�����졢�̡���LED�����õ����⣩����
8��������(�ʶ������쳣ʱ������)
9����������ң�������񣺲���ͬԭ����
10�������ο���ѹ��g_MsrRes.fVrefintӦ�ô���3.0��С��3.5
11��CPU�¶��Ƿ��쳣��g_MsrRes.fCPUTemperatureӦ��С��50*/

void RunMdlCtr(void)		/* rename from RunYKC */
{
	/* ����ֵ */
	ProcDInFilter();		/* �����˲� */
	CalDCSig(DCPWR_VOL_DCSigNo, 10, 1, &g_MsrRes.fDCPwrVol, NULL);
	CalDCSig(RIGHT_CUR_DCSigNo, 10, 1, &g_MsrRes.fRightCur, NULL);
	CalDCSig(LEFT_CUR_DCSigNo, 10, 1, &g_MsrRes.fLeftCur, NULL);
	g_MsrRes.fCPUTemperature = CalCPUTemperature(CPU_TEMP_DCSigNo);
	g_MsrRes.fVrefint = CalVrefint(VREF_INT_DCSigNo);
	g_MsrRes.fPumpFreq = ProcPulseSig(BUMP_MOTOR_PulseNo);
	g_MsrRes.fRightFreq = ProcPulseSig(RIGHT_MOTOR_PulseNo);
	g_MsrRes.fLeftFreq = ProcPulseSig(LEFT_MOTOR_PulseNo);

	/* ���� */
	UpdateAir();		/* ������ѹ�� */
	UpdateIMU();		/* ����IMU״̬ */
	UpdateIRCtrl();		/* ң���� */

	/* ���ؿ��� */
	if(GPIO_read(GPIO_DIN_StartNo + START_STOP_CMD_DInNo) == FALSE) {
		g_YKCTest.uKeyCnt_Delay++;
	} else { /* �������ɿ�ʱ */
		g_YKCTest.uKeyCnt_Jitter++;
		if((GPIO_read(GPIO_DIN_StartNo + START_STOP_CMD_DInNo) == TRUE) && (g_YKCTest.uKeyCnt_Jitter > 1)) {
			g_YKCTest.uKeyCnt_Jitter = 0;
			if(g_YKCTest.uKeyCnt_Delay > 5) { /* ����ʱ�㹻ʱ */
				if(!g_YKCTest.bMainSwitch) { /* �������ش�ʱ */
					if(CheckToStart()) {
						g_YKCTest.bMainSwitch = TRUE;
						TtsSpeak(VOICE_WELCOM, FALSE);		/* ��ӭʹ�� */
					}
				} else { /* �������عر�ʱ */
					g_YKCTest.bMainSwitch = FALSE;
					g_YKCTest.fLeftWheelDuty = 0;
					g_YKCTest.fRightWheelDuty = 0;
					g_YKCTest.fVacuumDuty = 0;

					TtsSpeak(VOICE_BYEBYE, FALSE);		/* �ټ� */
					/* �ѵƹ��˴���׵� */
					GPIO_write(BLUE_LED_DOutNo, FALSE);
					GPIO_write(RED_LED_DOutNo, FALSE);
					GPIO_write(GREEN_LED_DOutNo, FALSE);
				}
			}
			/* ����ʱ������ */
			g_YKCTest.uKeyCnt_Delay = 0;
		}
	}

	/* ������״̬ */
	if(g_YKCTest.bMainSwitch) {
		if(g_IRCtrl.u8TryCnt) {
			switch(g_IRCtrl.u8BtnPressing) {
			case IR_BTN_0:
				TtsSpeak(VOICE_WELCOM, FALSE);
				TtsSpeak(VOICE_BYEBYE, FALSE);
				g_YKCTest.fVacuumDuty = 0;
				break;
			case IR_BTN_1:
				g_YKCTest.fVacuumDuty = 0.2;
				break;
			case IR_BTN_2:
				g_YKCTest.fVacuumDuty = 0.3;
				break;
			case IR_BTN_3:
				g_YKCTest.fVacuumDuty = 0.4;
				break;
			case IR_BTN_4:
				g_YKCTest.fVacuumDuty = 0.5;
				break;
			case IR_BTN_5:
				g_YKCTest.fVacuumDuty = 0.6;
				break;
			case IR_BTN_6:
				g_YKCTest.fVacuumDuty = 0.7;
				break;
			case IR_BTN_7:
				g_YKCTest.fVacuumDuty = 0.8;
				break;
			case IR_BTN_8:
				g_YKCTest.fVacuumDuty = 0.9;
				break;
			case IR_BTN_9:
				g_YKCTest.fVacuumDuty = 1;
				break;
			case IR_BTN_START:
				/* �����ˮ */
				GPIO_toggle(0 + GPIO_RELAY_StartNo);
				break;
			case IR_BTN_SHARP:
				/* �ұ���ˮ */
				GPIO_toggle(1 + GPIO_RELAY_StartNo);
				break;
			case IR_BTN_UP:
				g_YKCTest.fLeftWheelDuty = 1;
				g_YKCTest.fRightWheelDuty = 1;
				break;
			case IR_BTN_DOWN:
				g_YKCTest.fLeftWheelDuty = -1;
				g_YKCTest.fRightWheelDuty = -1;
				break;
			case IR_BTN_LEFT:
				g_YKCTest.fLeftWheelDuty = 1;
				g_YKCTest.fRightWheelDuty = -1;
				break;
			case IR_BTN_RIGHT:
				g_YKCTest.fLeftWheelDuty = -1;
				g_YKCTest.fRightWheelDuty = 1;
				break;
			case IR_BTN_OK:
				g_YKCTest.fLeftWheelDuty = 0;
				g_YKCTest.fRightWheelDuty = 0;
				break;
			}
		} else {		/* ���û��ң���������¼������Զ����� */
			/* ����ת���𲽼ӿ졢ͣ�¡���ת */
			if(g_YKCTest.tMotorTest_0Idle_pInc_nDec == 0) {
				//g_YKCTest.fDuty = 0;
			} else if(g_YKCTest.tMotorTest_0Idle_pInc_nDec > 0) {
				if(g_YKCTest.fLeftWheelDuty >= 1) {
					g_YKCTest.fLeftWheelDuty = 1;
					g_YKCTest.tMotorTest_0Idle_pInc_nDec = -1;
				} else {
					g_YKCTest.fLeftWheelDuty += 0.001;
				}

				if(g_YKCTest.fRightWheelDuty >= 1) {
					g_YKCTest.fRightWheelDuty = 1;
					g_YKCTest.tMotorTest_0Idle_pInc_nDec = -1;
				} else {
					g_YKCTest.fRightWheelDuty += 0.001;
				}
			} else {
				if(g_YKCTest.fRightWheelDuty <= -1) {
					g_YKCTest.fRightWheelDuty = -1;
					g_YKCTest.tMotorTest_0Idle_pInc_nDec = 1;
				} else {
					g_YKCTest.fRightWheelDuty -= 0.001;
				}
			}
		}

		/* ����ָʾ�� */
		if(g_YKCTest.uTimer_100Hz >= 400) {
			g_YKCTest.uTimer_100Hz = 0;
		} else {
			g_YKCTest.uTimer_100Hz++;
		}
		GPIO_write(BLUE_LED_DOutNo, g_YKCTest.uTimer_100Hz < 100);
		GPIO_write(RED_LED_DOutNo, g_YKCTest.uTimer_100Hz >= 100 && g_YKCTest.uTimer_100Hz < 200);
		GPIO_write(GREEN_LED_DOutNo, g_YKCTest.uTimer_100Hz >= 200 && g_YKCTest.uTimer_100Hz < 300);

		/* ��ײ���� */
		/* �ֶ��ı�ӿ�ģ����ײ */
		if(!GPIO_read(GPIO_DIN_StartNo + HIT_SIDE_L_FRONT_DInNo)) {
			TtsSpeak(VOICE_HIT_SIDE_L_FRONT_DINNO, FALSE);
			g_YKCTest.u8ErrorCnt++;
		}
		if(!GPIO_read(GPIO_DIN_StartNo + HIT_SIDE_L_REAR_DInNo)) {
			TtsSpeak(VOICE_HIT_SIDE_L_REAR_DINNO, FALSE);
			g_YKCTest.u8ErrorCnt++;
		}
		if(!GPIO_read(GPIO_DIN_StartNo + HIT_SIDE_R_FRONT_DInNo)) {
			TtsSpeak(VOICE_HIT_SIDE_R_FRONT_DINNO, FALSE);
			g_YKCTest.u8ErrorCnt++;
		}
		if(!GPIO_read(GPIO_DIN_StartNo + HIT_SIDE_R_REAR_DInNo)) {
			TtsSpeak(VOICE_HIT_SIDE_R_REAR_DINNO, FALSE);
			g_YKCTest.u8ErrorCnt++;
		}
		if(!GPIO_read(GPIO_DIN_StartNo + HIT_FRONT_LEFT_DInNo)) {
			TtsSpeak(VOICE_HIT_FRONT_LEFT_DINNO, FALSE);
			g_YKCTest.u8ErrorCnt++;
		}
		if(!GPIO_read(GPIO_DIN_StartNo + HIT_FRONT_RIGHT_DInNo)) {
			TtsSpeak(VOICE_HIT_FRONT_RIGHT_DINNO, FALSE);
			g_YKCTest.u8ErrorCnt++;
		}
		if(!GPIO_read(GPIO_DIN_StartNo + HIT_REAR_LEFT_DInNo)) {
			TtsSpeak(VOICE_HIT_REAR_LEFT_DINNO, FALSE);
			g_YKCTest.u8ErrorCnt++;
		}
		if(!GPIO_read(GPIO_DIN_StartNo + HIT_REAR_RIGHT_DInNo)){
			TtsSpeak(VOICE_HIT_REAR_RIGHT_DINNO, FALSE);
			g_YKCTest.u8ErrorCnt++;
		}

		/* ���²��� */
		if(GPIO_read(GPIO_DIN_StartNo + CLIFF_FRONT_LEFT_DInNo)) {
			TtsSpeak(VOICE_CLIFF_FRONT_LEFT_DINNO, FALSE);
			g_YKCTest.u8ErrorCnt++;
		}
		if(GPIO_read(GPIO_DIN_StartNo + CLIFF_FRONT_RIGHT_DInNo)) {
			TtsSpeak(VOICE_CLIFF_FRONT_RIGHT_DINNO, FALSE);
			g_YKCTest.u8ErrorCnt++;
		}
		if(GPIO_read(GPIO_DIN_StartNo + CLIFF_REAR_LEFT_DInNo)) {
			TtsSpeak(VOICE_CLIFF_REAR_LEFT_DINNO, FALSE);
			g_YKCTest.u8ErrorCnt++;
		}
		if(GPIO_read(GPIO_DIN_StartNo + CLIFF_REAR_RIGHT_DInNo)) {
			TtsSpeak(VOICE_CLIFF_REAR_RIGHT_DINNO, FALSE);
			g_YKCTest.u8ErrorCnt++;
		}


		/* ��ˮ�ò��� */
//		g_YKCTest.uWaterPumpCnt++;
//		if(g_YKCTest.uWaterPumpCnt < 100) { //���2���ˮ��
//			GPIO_write(0 + GPIO_RELAY_StartNo, TRUE);
//			GPIO_write(1 + GPIO_RELAY_StartNo, FALSE);
//		} else if(g_YKCTest.uWaterPumpCnt < 200) {
//			GPIO_write(1 + GPIO_RELAY_StartNo, TRUE);
//			GPIO_write(0 + GPIO_RELAY_StartNo, FALSE);
//		} else {
//			g_YKCTest.uWaterPumpCnt = 0; /* ��ʱ����� */
//		}

		/* ��ȡ���� */
		if(g_YKCTest.u8ErrorCnt > 0) {
			g_YKCTest.u8ErrorCnt = 0; /* ����쳣 */
		} else {
			/*�򿪰׵���˸���˳�����*/

			/*�ر�ˮ��*/
//			GPIO_write(0 + GPIO_RELAY_StartNo, FALSE);
//			GPIO_write(1 + GPIO_RELAY_StartNo, FALSE);
		}
	}


#if (!SOFT_RUN1_TEST0)
	/* �������ݵ�һЩ����(�ڸ������ݺ���в����ж�) */
	/* ��ѹ����:g_MsrRes.fAirPressure�Ƿ����9000000��С��1020000 */
	if(g_MsrRes.fAirPressure < 9e6 || g_MsrRes.fAirPressure >= 1020000) {
		TtsSpeak(VOICE_AIR_PRES_ABN, FALSE); /* ����ѹ�쳣 */
	}
	/* �����ǲ��� */
	if(g_MsrRes.iAccZ <= 0) {
		TtsSpeak(VOICE_IMU_ABN, FALSE); /* ���������쳣 */
	}
	/* cpu�¶Ȳ��� */
	if(g_MsrRes.fCPUTemperature >= 50) {
		TtsSpeak(VOICE_CPU_TEMPERATURE_ERROR, FALSE);
	}
#endif

	/* ���� */
	UpdateTTS();	/* ��������ģ�� */

	/* ��� */
	Board_DrvPump(fabsf(g_YKCTest.fVacuumDuty));
//	Board_DrvPump(fabsf(1.0f));
	Board_DrvWheel(g_YKCTest.fLeftWheelDuty, g_YKCTest.fRightWheelDuty);
//	Board_DrvWheel(g_CodeTest.fVal[55], g_CodeTest.fVal[55]);

#if 0 	/* �ɲο�����Ƭ�� */
	int8 i;
	/* ����ָʾ�� */
	if(g_YKCTest.uTimer_100Hz >= 400) {
		g_YKCTest.uTimer_100Hz = 0;
	} else {
		g_YKCTest.uTimer_100Hz++;
	}
	GPIO_write(BLUE_LED_DOutNo, g_YKCTest.uTimer_100Hz < 100);
	GPIO_write(RED_LED_DOutNo, g_YKCTest.uTimer_100Hz < 200);
	GPIO_write(GREEN_LED_DOutNo, g_YKCTest.uTimer_100Hz < 300);

//	GPIO_write(BLUE_LED_DOutNo, g_CodeTest.uVal[5]!=0);
//	GPIO_write(RED_LED_DOutNo, g_CodeTest.uVal[6]!=0);
//	GPIO_write(GREEN_LED_DOutNo, g_CodeTest.uVal[7]!=0);
//	g_CodeTest.uVal[8] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8);

#if !TEST
	Board_DrvWheel(g_YKCTest.fDuty, 0 - g_YKCTest.fDuty);
	Board_DrvBump(fabsf(g_YKCTest.fDuty));
#else
	Board_DrvWheel(g_CodeTest.fVal[0], g_CodeTest.fVal[1]);
	Board_DrvBump(g_CodeTest.fVal[2]);
#endif

//	GPIO_write(0 + GPIO_RELAY_StartNo, 1);
//	GPIO_write(1 + GPIO_RELAY_StartNo, 1);
//	OnRelay(g_CodeTest.u32Val[8]);
////	OnRelay(g_CodeTest.u32Val[9]);

	/* �̵������� */
//	int8 i8OnRelayNo = g_YKCTest.u8OnRelayNo;
//	if(g_YKCTest.i8Tmr_AutoTest_pActRelay_nIntv_tick) {
//		if(g_YKCTest.i8Tmr_AutoTest_pActRelay_nIntv_tick > 0) {
//			g_YKCTest.i8Tmr_AutoTest_pActRelay_nIntv_tick--;
//			if(g_YKCTest.i8Tmr_AutoTest_pActRelay_nIntv_tick == 0) {
//				g_YKCTest.i8Tmr_AutoTest_pActRelay_nIntv_tick = 0 - 1000/PERIOD_CTR_TASK_ms;
//			}
//		} else if(g_YKCTest.i8Tmr_AutoTest_pActRelay_nIntv_tick < 0) {
//			g_YKCTest.i8Tmr_AutoTest_pActRelay_nIntv_tick++;
//			if(g_YKCTest.i8Tmr_AutoTest_pActRelay_nIntv_tick == 0) {
//				g_YKCTest.i8Tmr_AutoTest_pActRelay_nIntv_tick = 1000/PERIOD_CTR_TASK_ms;
//				i8OnRelayNo++;
//			}
//		}
//	} else if(g_MiscCommData.u8Tmr_PnlCtrBtn_Valid_ms[CTR_BTN_IncRelay_No]
//			&& ((~g_YKCTest.uLastPnlCtrBtn) & (1<<CTR_BTN_IncRelay_No)))
//	{
//		i8OnRelayNo++;
//	} else if(g_MiscCommData.u8Tmr_PnlCtrBtn_Valid_ms[CTR_BTN_DecRelay_No]
//			&& ((~g_YKCTest.uLastPnlCtrBtn) & (1<<CTR_BTN_DecRelay_No)))
//	{
//		i8OnRelayNo--;
//	}
//	if(i8OnRelayNo < 0) {
//		i8OnRelayNo = MAX_RELAY_NUM;
//	} else if(i8OnRelayNo > MAX_RELAY_NUM) {
//		i8OnRelayNo = 0;
//	}
//	g_YKCTest.u8OnRelayNo = i8OnRelayNo;
//
//	/* �̵������ */
//	uint32 u32RelayDat[(MAX_RELAY_NUM+31)/32];
//	for(i = 0; i < (MAX_RELAY_NUM+31)/32; i++) {
//	    u32RelayDat[i] = 0;
//	}
//	for(i = 0; i < MAX_RELAY_NUM; i++) {
//	    if((g_YKCTest.i8Tmr_AutoTest_pActRelay_nIntv_tick >= 0)
//			&& ((g_YKCTest.u8OnRelayNo == i) || (g_YKCTest.u8OnRelayNo == MAX_RELAY_NUM)))
//		{
//			OnRelay(i);
//			u32RelayDat[i/32] |= (1UL<<(i%32));
//	    } else {
//	        OffRelay(i);
//	    }
//	}
//	for(i = 0; i < (MAX_RELAY_NUM+31)/32; i++) {
//	    g_Ctr.u32RelayDat[i] = u32RelayDat[i];
//	}

#endif

#if (!SOFT_RUN1_TEST0)
	/* ��ʱuart4���Դ��룬��ӡ������������ѹ�ơ�IMU���� */
	if((g_CodeTest.u32Val[66] % 50) == 0) {
		extern UART_HandleTypeDef huart4;
//		sprintf((uint8*)g_CodeTest.fVal2,"reset count:%ld, [try:%d,pitch:%ld roll:%ld], air:%ld"
//				"rFrq:%ld"
//				"lFrq:%ld"
//				"mFrq:%ld"
//				"IR:%d"
//				"IR_CNT:%d"
//				"iAccZ:%d"
//				"fVrefint:%f"
//				"fCPUTemperature:%ld\n",
//				(int32)g_Sys.uRstCount,
//				(int32)g_MsrRes.u8TryCnt_IMU,
//				(int32)(g_MsrRes.fPitch*100),
//				(int32)(g_MsrRes.fRoll*100),
//				(int32)(g_MsrRes.fAirPressure),
//				(int32)(g_MsrRes.fRightFreq*100),
//				(int32)(g_MsrRes.fLeftFreq*100),
//				(int32)(g_MsrRes.fBumpFreq*100),
//				g_IRCtrl.u8BtnPressing,
//				g_IRCtrl.u8TryCnt,
//				(int32)(g_MsrRes.iAccZ),
//				(g_MsrRes.fVrefint),
//				(int32)(g_MsrRes.fCPUTemperature)); //�������Ϊ��������
		HAL_UART_Transmit(&huart4, (const uint8 *)g_CodeTest.fVal2, strlen(g_CodeTest.fVal2), 500);
	}
	g_CodeTest.u32Val[66]++;
#endif
}

/*==========================================================================
| Description	: 
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2012-12-8
\=========================================================================*/
void DrvMdlCtrTick_1KHz(void)		/* rename from DrvYKCTick_1KHz */
{
}


__weak void GetAuthKey(AUTH_TYPE AuthType, uint32* pAuthKey)
{
	switch(AuthType) {
		case AUTH_KEY_FLASH_REQ_KEY:	/* 1�� */
			*pAuthKey++ = 0x01234567;
			break;

		case AUTH_AES_FLASH_REQ:		/* 4�� */
			*pAuthKey++ = 1;
			*pAuthKey++ = 0;
			*pAuthKey++ = 1;
			*pAuthKey++ = 0;
			break;

		case AUTH_AES_FLASH_DATA:		/* 4�� */
			*pAuthKey++ = 0;
			*pAuthKey++ = 1;
			*pAuthKey++ = 0;
			*pAuthKey++ = 1;
			break;

		case AUTH_AES_PACK_SERIAL:		/* 4�� */
			*pAuthKey++ = 1;
			*pAuthKey++ = 1;
			*pAuthKey++ = 1;
			*pAuthKey++ = 1;
			break;

		case AUTH_AES_PACK_LICENSE: 	/* 4�� */
			*pAuthKey++ = 0;
			*pAuthKey++ = 0;
			*pAuthKey++ = 0;
			*pAuthKey++ = 0;
			break;

		case AUTH_KEY_FLASH_INTEGRITY_V1:
			*pAuthKey++ = 1;
			*pAuthKey++ = 1;
			break;

		case AUTH_KEY_FLASH_INTEGRITY_V2:
			*pAuthKey++ = 1;
			*pAuthKey++ = 1;
			*pAuthKey++ = 1;
			*pAuthKey++ = 1;
		default:
			break;
	}
}

__weak BOOL ChkAuthKey(AUTH_TYPE AuthType, uint32* pAuthKey)
{
	switch(AuthType) {
		case AUTH_KEY_FLASH_REQ_KEY:	/* 1���������GetAuthKey()������һ�� */
			if(*pAuthKey == 0x01234567) {	/* �����GetAuthKey()������һ�� */
				return TRUE;
			}
			break;

		default:
			break;
	}
	return FALSE;
}

/*==========================================================================
| Description	: Mqtt���ݷ���
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2023/12/8
\=========================================================================*/
BOOL PubSpecialData(MQTT_COMM* pMqttComm)
{
	/* ������Ϣ�����������ͺš�Ӳ���汾�š�����汾�š�IP�ȣ���QoS1���� */
	/* ������Ϣ����:��ʼ�� */
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5;	/* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
	uint8 u8QoS = 0;	/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
	/* ������Ϣ����:��ӡTopic */
	PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
	PrintStringNoOvChk(&pTxBuf, "/DATA");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
	}
	/* ������Ϣ����:��ӡ���ݳ�Json��ʽ */
	*pTxBuf++ = '{';					/* Json��ʼ */
	PrintStringNoOvChk(&pTxBuf, "\"status\":\"������\"");
	*pTxBuf++ = '}';					/* Json��β */
	/* ������Ϣ����:���� */
	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
}

uint16 ProcLocalCmdInstr(uint8* pRxBuf, uint16 uRegOff, int16 RegNum)
{
	return 0;
}

/*==========================================================================
| Description	: Mqtt����ָ��
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2023-12-8
\=========================================================================*/
BOOL ProcMqttCmdInstr(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd)
{
	return PubSpecialData(pMqttComm);
//	return FALSE;
}
#endif
/******************************** FILE END ********************************/
