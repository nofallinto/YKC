#if (!SOFT_RUN1_TEST0)

/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved
 *
 * File			: stencil.c
 * Author		: Wang Renfei
 * Description	: c file's stencil, V1.1 inherit from V1.0
 *				: 模块化的软件结构，每一个模块在头文件中定义外部接口变量,外部驱动函数,模块调用方法函数，在源文件中定义模块内部变量、函数
 * Comments		:
 * Date			: 2006-11-27
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 * V0.1 | 2023-12-08  | king | 1. 初始化
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

/* 项目公共头文件 */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"

/* 本级以及下级模块头文件 */
#include "MdlNet.h"
#include "MdlDataAccess.h"
#include "BoardSupport.h"
#include "MdlUARTnModbus.h"
#include "MdlGUI.h"
#include "MdlGprs.h"    
#include "BMI270.h"		/* IMU */
#include "BMP280.h"		/* 气压 */

#include "MdlYKC.h"

/* 其他模块 */
void UpdateIMU(void);
void UpdateAir(void);
void UpdateIRCtrl(void);
/***************************************************************************
 						global variables definition
***************************************************************************/
const CONF_n_STAT_PROP cnst_YkcConfProp[] = {                                              /* 1 2 3 4 5 6 7 8 9 A B C D E F 1 2 3 4 5 6 7 8 9 A B C D E F */
    {1001, {ITEM_DAT_TOPIC, 0, 0, SAVE_GRP_NUL}, 3, NULL,                      {"系统与通讯配置", "Default"}},
    {1002, {ITEM_DAT_SOFTVER, 0, 0, SAVE_GRP_NUL}, SOFTWARE_VER, NULL,                {"软件版本", "Soft Version"}},
    {1003, {ITEM_DAT_U32_ID_RO, 0, 0, SAVE_GRP_BRD}, 0, (void*)&g_Sys.SerialNo,       {"序列号", "S/N:"}},
    {1004, {ITEM_DAT_T32_RO, 0, 0, SAVE_GRP_BRD}, 0, (void*)&g_Sys.u32License_Seconds,{"授权期限(年月日时)", "deadline"}},
};

/* EEProm地址分配 */
/* EEProm地址分配, 有别于TI物理位置连续的顺序存储，ST采用分BANK存储主备两个区 */
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
	BOOL bMainSwitch;						/* 总开关 */
	uint8 u8ErrorCnt; 						/* 用于记录异常数量 */
	uint8 u8Rsvd;
	uint16 uKeyCnt_Delay; 					/* 按键延时时间变量 */
	uint16 uKeyCnt_Jitter; 					/* 按键消抖时间变量 */
	uint16 uWaterPumpCnt; 					/* 控制水泵依次打开的时间变量 */
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
	/* 初始化配置 */

	/* 初始化输入接口变量 */

	/* 初始化输出接口变量 */
	InitDataWithZero((uint8*)(&g_MsrRes), sizeof(g_MsrRes));
	InitDataWithZero((uint8*)(&g_AnaRes), sizeof(g_AnaRes));
	g_Ctr.u32DinDat = 0;

	/* 初始化内部全局变量 */
	InitDataWithZero((uint8*)(&g_YKCTest), sizeof(g_YKCTest));
	g_YKCTest.bMainSwitch = FALSE;
	g_YKCTest.fVacuumDuty = 0;
	g_YKCTest.fLeftWheelDuty = 0;
	g_YKCTest.fRightWheelDuty = 0;

	/* 初始化下层模块 */
	
	/* 启动硬件 */
	InitSample();		/* 由于采样需要填充buf，然后才好计算，因此现行启动 */

	GPIO_write(OTHER_DEV_DOutNo, 1);		/* 使能周边设备3.3v供电 */

	/* 初始化六轴传感器 */
    if(!Bmi270Init()) {
		/* IMU初始化失败*/
		configASSERT(FALSE);
	}
	/* 初始化气压计 */
    if(!Bmp280Init()) {
		/* IMU初始化失败*/
		configASSERT(FALSE);
	}

	/* 初始化红外遥控器  TIM5频率1MHz, ARR==65535*/
	extern TIM_HandleTypeDef htim5;
	HAL_TIM_Base_Start_IT(&htim5);	 			/* 启动定时器中断，之所以没放在main.c里，是因为想在初始化代码之后再开始 */
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);  /* 启动定时器输入捕获，之所以没放在main.c里，是因为想在初始化代码之后再开始 */
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);

	/* 初始化语音模块 */
	InitTtsSpeaking();

#if (0 && (!SOFT_RUN1_TEST0))
	/* GPRS测试 */
	extern UART_HandleTypeDef huart4;
	SOCKET SocketFd; /* 百度相关套接字 */
	struct sockaddr_in Socket_Gprs; /* 测试用地址族 SOCKADDR_IN*/

	/* GPRS连接变量 */
	int8 i8S_Result, i8R_Result, i8C_Result_Close, i8C_Result_Connect = -1;
	uint8 aU8GprsBuffer[500];

	/* 连接百度测试 (1次) */
	/* 百度ip地址定义 */
#define BAIDU_IP "39.156.66.10"  			/* 百度的IP地址，注意有时可能会变化 */
#define BAIDU_PORT 80  						/* 百度HTTP端口号 */
	memset(&Socket_Gprs, 0, sizeof(Socket_Gprs));
	Socket_Gprs.sin_family = AF_INET; /* 地址族采用ipv4通信 */
	Socket_Gprs.sin_port = htons(80); /* 将端口号转为网络字节 */
	Socket_Gprs.sin_addr.s_addr = htonl(0x279c420a); /* 将地址转为主机字节序，然后在转为网络字节序 */
	SocketFd = GprsSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP);  /* 使用流式套接字类型 */

	while(i8C_Result_Connect != GPRS_SUC) { /* 重复连接保证连接上 */
		i8C_Result_Connect = GprsConnect(SocketFd, &Socket_Gprs, sizeof(Socket_Gprs));
	}

	sprintf((uint8*)aU8GprsBuffer, "GET /?st=1 HTTP/1.1\r\nHost:www.baidu.com\r\n\r\n");
	i8S_Result = GprsSend(SocketFd, aU8GprsBuffer, strlen(aU8GprsBuffer), 0);
	while(i8S_Result > 0) { /* 当成功发送命令后，进行接收(注意要循环接收，网页发送信息不会一次发完) */
		i8R_Result = GprsRecv(SocketFd, aU8GprsBuffer, sizeof(aU8GprsBuffer), 0);
		if(i8R_Result > 0) { /* 成功接收到数据 */
			sprintf(aU8GprsBuffer, "receive:%d byte\r\n", i8R_Result);
			HAL_UART_Transmit(&huart4, (const uint8 *)aU8GprsBuffer, strlen(aU8GprsBuffer), 500);

			/* 将接收到的回复发送到串口中，方便调试 */
			HAL_UART_Transmit(&huart4, (const uint8 *)aU8GprsBuffer, strlen(aU8GprsBuffer), 500);

			/* 测试完将连接关闭 */
			i8C_Result_Close = GprsClose(SocketFd);
			if(i8C_Result_Close == GPRS_SUC) { /* 关闭成功 */
				sprintf(aU8GprsBuffer, "finish closed\r\n");
				HAL_UART_Transmit(&huart4, (const uint8 *)aU8GprsBuffer, strlen(aU8GprsBuffer), 500);
			}
			break;
		}
	}
#endif
}

/* 检测是否到达启动条件 */
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
/*1，测试程序地点：
MdlTest.c
一次性执行的函数：InitMdlCtr()
每40ms会执行一次的函数：RunMdlCtr()

2，测试步骤：
上电后三色灯按照红、绿、蓝顺序闪动，间隔1s，开始执行测试，途中如遇到异常则通过扬声器朗读，否则全部正常测试结束后，闪白灯代表可以断电结束。
具体目标：
1，左右两个履带电机，轮子转速逐步加快、停下、反转(已有测试代码)，让测试者人肉看测试结果
2，一个无刷排气电机：g_YKCTest.fVacuumDuty设成1，也就是最大排风，让测试者人肉看测试结果
3，气压计：g_MsrRes.fAirPressure是否大于9000000且小于1020000（单位：Pa）
4，陀螺仪：g_MsrRes.iAccZ大于0
5，GPRS：给百度发送GET请求，并能收到返回内容
6，开入：8个碰撞检测（检测到碰撞后语音提示（哪个检测到了闭合））、
	4个悬崖（检测到碰撞后语音提示（哪个检测到了闭合）），
	一个总按键开关（长按开机（履带测试、排风测试、水泵测试功能）、关机（履带、排风、水泵）），
	一个水泵管道有水监测（直连水泵（接线前先和硬件王永耀确认））。
7，开出（两个水泵交替打开（然后读取开入那边是否都是1，如果都是1说明开入开出全部OK，否则朗读第几个开出异常），红、绿、蓝LED（不用单独测）），
8，扬声器(朗读测试异常时的内容)
9，两个红外遥控器捕获：测试同原功能
10，采样参考电压：g_MsrRes.fVrefint应该大于3.0且小于3.5
11，CPU温度是否异常：g_MsrRes.fCPUTemperature应该小于50*/

void RunMdlCtr(void)		/* rename from RunYKC */
{
	/* 计算值 */
	ProcDInFilter();		/* 开入滤波 */
	CalDCSig(DCPWR_VOL_DCSigNo, 10, 1, &g_MsrRes.fDCPwrVol, NULL);
	CalDCSig(RIGHT_CUR_DCSigNo, 10, 1, &g_MsrRes.fRightCur, NULL);
	CalDCSig(LEFT_CUR_DCSigNo, 10, 1, &g_MsrRes.fLeftCur, NULL);
	g_MsrRes.fCPUTemperature = CalCPUTemperature(CPU_TEMP_DCSigNo);
	g_MsrRes.fVrefint = CalVrefint(VREF_INT_DCSigNo);
	g_MsrRes.fPumpFreq = ProcPulseSig(BUMP_MOTOR_PulseNo);
	g_MsrRes.fRightFreq = ProcPulseSig(RIGHT_MOTOR_PulseNo);
	g_MsrRes.fLeftFreq = ProcPulseSig(LEFT_MOTOR_PulseNo);

	/* 外设 */
	UpdateAir();		/* 更新气压计 */
	UpdateIMU();		/* 更新IMU状态 */
	UpdateIRCtrl();		/* 遥控器 */

	/* 开关控制 */
	if(GPIO_read(GPIO_DIN_StartNo + START_STOP_CMD_DInNo) == FALSE) {
		g_YKCTest.uKeyCnt_Delay++;
	} else { /* 当按键松开时 */
		g_YKCTest.uKeyCnt_Jitter++;
		if((GPIO_read(GPIO_DIN_StartNo + START_STOP_CMD_DInNo) == TRUE) && (g_YKCTest.uKeyCnt_Jitter > 1)) {
			g_YKCTest.uKeyCnt_Jitter = 0;
			if(g_YKCTest.uKeyCnt_Delay > 5) { /* 当延时足够时 */
				if(!g_YKCTest.bMainSwitch) { /* 当主开关打开时 */
					if(CheckToStart()) {
						g_YKCTest.bMainSwitch = TRUE;
						TtsSpeak(VOICE_WELCOM, FALSE);		/* 欢迎使用 */
					}
				} else { /* 当主开关关闭时 */
					g_YKCTest.bMainSwitch = FALSE;
					g_YKCTest.fLeftWheelDuty = 0;
					g_YKCTest.fRightWheelDuty = 0;
					g_YKCTest.fVacuumDuty = 0;

					TtsSpeak(VOICE_BYEBYE, FALSE);		/* 再见 */
					/* 把灯关了代替白灯 */
					GPIO_write(BLUE_LED_DOutNo, FALSE);
					GPIO_write(RED_LED_DOutNo, FALSE);
					GPIO_write(GREEN_LED_DOutNo, FALSE);
				}
			}
			/* 将定时器归零 */
			g_YKCTest.uKeyCnt_Delay = 0;
		}
	}

	/* 开机后状态 */
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
				/* 左边喷水 */
				GPIO_toggle(0 + GPIO_RELAY_StartNo);
				break;
			case IR_BTN_SHARP:
				/* 右边喷水 */
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
		} else {		/* 如果没有遥控器按键事件，则自动驱动 */
			/* 轮子转速逐步加快、停下、反转 */
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

		/* 测试指示灯 */
		if(g_YKCTest.uTimer_100Hz >= 400) {
			g_YKCTest.uTimer_100Hz = 0;
		} else {
			g_YKCTest.uTimer_100Hz++;
		}
		GPIO_write(BLUE_LED_DOutNo, g_YKCTest.uTimer_100Hz < 100);
		GPIO_write(RED_LED_DOutNo, g_YKCTest.uTimer_100Hz >= 100 && g_YKCTest.uTimer_100Hz < 200);
		GPIO_write(GREEN_LED_DOutNo, g_YKCTest.uTimer_100Hz >= 200 && g_YKCTest.uTimer_100Hz < 300);

		/* 碰撞测试 */
		/* 手动改变接口模拟碰撞 */
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

		/* 悬崖测试 */
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


		/* 排水泵测试 */
//		g_YKCTest.uWaterPumpCnt++;
//		if(g_YKCTest.uWaterPumpCnt < 100) { //间隔2秒打开水泵
//			GPIO_write(0 + GPIO_RELAY_StartNo, TRUE);
//			GPIO_write(1 + GPIO_RELAY_StartNo, FALSE);
//		} else if(g_YKCTest.uWaterPumpCnt < 200) {
//			GPIO_write(1 + GPIO_RELAY_StartNo, TRUE);
//			GPIO_write(0 + GPIO_RELAY_StartNo, FALSE);
//		} else {
//			g_YKCTest.uWaterPumpCnt = 0; /* 将时间归零 */
//		}

		/* 读取开入 */
		if(g_YKCTest.u8ErrorCnt > 0) {
			g_YKCTest.u8ErrorCnt = 0; /* 清空异常 */
		} else {
			/*打开白灯闪烁，退出调试*/

			/*关闭水泵*/
//			GPIO_write(0 + GPIO_RELAY_StartNo, FALSE);
//			GPIO_write(1 + GPIO_RELAY_StartNo, FALSE);
		}
	}


#if (!SOFT_RUN1_TEST0)
	/* 外设数据的一些测试(在更新数据后进行测试判断) */
	/* 气压测试:g_MsrRes.fAirPressure是否大于9000000且小于1020000 */
	if(g_MsrRes.fAirPressure < 9e6 || g_MsrRes.fAirPressure >= 1020000) {
		TtsSpeak(VOICE_AIR_PRES_ABN, FALSE); /* 报气压异常 */
	}
	/* 陀螺仪测试 */
	if(g_MsrRes.iAccZ <= 0) {
		TtsSpeak(VOICE_IMU_ABN, FALSE); /* 报陀螺仪异常 */
	}
	/* cpu温度测试 */
	if(g_MsrRes.fCPUTemperature >= 50) {
		TtsSpeak(VOICE_CPU_TEMPERATURE_ERROR, FALSE);
	}
#endif

	/* 语音 */
	UpdateTTS();	/* 更新语音模块 */

	/* 电机 */
	Board_DrvPump(fabsf(g_YKCTest.fVacuumDuty));
//	Board_DrvPump(fabsf(1.0f));
	Board_DrvWheel(g_YKCTest.fLeftWheelDuty, g_YKCTest.fRightWheelDuty);
//	Board_DrvWheel(g_CodeTest.fVal[55], g_CodeTest.fVal[55]);

#if 0 	/* 旧参考代码片段 */
	int8 i;
	/* 测试指示灯 */
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

	/* 继电器动作 */
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
//	/* 继电器输出 */
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
	/* 临时uart4测试代码，打印重启次数、气压计、IMU数据 */
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
//				(int32)(g_MsrRes.fCPUTemperature)); //最后三项为测试用例
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
		case AUTH_KEY_FLASH_REQ_KEY:	/* 1个 */
			*pAuthKey++ = 0x01234567;
			break;

		case AUTH_AES_FLASH_REQ:		/* 4个 */
			*pAuthKey++ = 1;
			*pAuthKey++ = 0;
			*pAuthKey++ = 1;
			*pAuthKey++ = 0;
			break;

		case AUTH_AES_FLASH_DATA:		/* 4个 */
			*pAuthKey++ = 0;
			*pAuthKey++ = 1;
			*pAuthKey++ = 0;
			*pAuthKey++ = 1;
			break;

		case AUTH_AES_PACK_SERIAL:		/* 4个 */
			*pAuthKey++ = 1;
			*pAuthKey++ = 1;
			*pAuthKey++ = 1;
			*pAuthKey++ = 1;
			break;

		case AUTH_AES_PACK_LICENSE: 	/* 4个 */
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
		case AUTH_KEY_FLASH_REQ_KEY:	/* 1个，必须和GetAuthKey()产生的一致 */
			if(*pAuthKey == 0x01234567) {	/* 必须和GetAuthKey()产生的一致 */
				return TRUE;
			}
			break;

		default:
			break;
	}
	return FALSE;
}

/*==========================================================================
| Description	: Mqtt数据发布
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2023/12/8
\=========================================================================*/
BOOL PubSpecialData(MQTT_COMM* pMqttComm)
{
	/* 本机信息发布，包括型号、硬件版本号、软件版本号、IP等，以QoS1发布 */
	/* 本机信息发布:初始化 */
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5;	/* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
	uint8 u8QoS = 0;	/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
	/* 本机信息发布:打印Topic */
	PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
	PrintStringNoOvChk(&pTxBuf, "/DATA");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
	}
	/* 本机信息发布:打印数据成Json格式 */
	*pTxBuf++ = '{';					/* Json开始 */
	PrintStringNoOvChk(&pTxBuf, "\"status\":\"调试中\"");
	*pTxBuf++ = '}';					/* Json结尾 */
	/* 本机信息发布:传输 */
	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS修改必须在本段代码开始的位置 */
}

uint16 ProcLocalCmdInstr(uint8* pRxBuf, uint16 uRegOff, int16 RegNum)
{
	return 0;
}

/*==========================================================================
| Description	: Mqtt控制指令
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
