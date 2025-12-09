/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved. 模板V1.3
 *
 * File			: MdlUARTnModbus.c
 * Author		: Wang Renfei
 * Description	: 
 
 支持RS485总线上的多种功能: 一般ModbusRTU从机，ViewTech屏幕，无线水位(ModbusRTU主机 1200bps)
 每一种功能包括: Try**, Run** 两个主要函数；
 Try**	 : 以该应用所特有的通讯参数(如波特率)初始化硬件, 并收发测试数据，以确定当前模式是否恰当。
 Run**	 : 数据成帧，并进行解析

 多种功能自动识别算法:
 每一次成功的通讯(CRC检测正确)，就复位相应的通讯失败(看门狗)定时器；如果看门狗溢出，则启动识别(调用Try**函数)

 依据不同的通讯协议(主要就ModbusRTU, ViewTech两种通讯协议), 还有TxData**, CheckRxBufCrcFor**两个函数
 TxData**  			: 加载该协议的CRC、数据长度等，并完成数据发送
 CheckRxBufCrcFor** : 依据该协议对RxBuf进行CRC检查

 此外，本文将还提供了RS485通讯相关的两个公共函数:
 OpenUartComm			: 封装了UART_open在内的打开RS485的函数
 ReadDataFromRS485	: 自RS485获取数据

 * Date			: 2015-10-22
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 **************************************************************************/

/***************************************************************************
 						macro definition
***************************************************************************/
#define _MDL_RS485_n_MODBUS_C_		/* exclude redefinition */

/***************************************************************************
 						include files
***************************************************************************/
/* 项目公共头文件 */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"

/* 操作系统头文件 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/* 本级以及下级模块头文件 */
#include "MdlUARTnModbus.h"
#include "MdlGUI.h"
#include "BoardSupport.h"

#if SUPPORT_V4_MASTER
#include "MdlV4Master.h"
#endif
#if SUPPORT_IEC101
#include "Mdl101n104.h"
#endif
#if SUPPORT_FLOW_METER
#include "MdlFlow.h"
#endif
#ifdef CAN_BASE
#include "MdlCan.h"
#endif
#if (SUPPORT_TRIPHASE_METER || SUPPORT_PGA460 || SUPPORT_WTRLVL_OPT_TX || SUPPORT_WTRLVL_OPT_RX || SUPPORT_RADIO_SUB || SUPPORT_RADIO_PUB || SUPPORT_GPS || SUPPORT_AIR_COMPRESSOR || SUPPORT_GNL_MODBUS_METER || SUPPORT_PUMP)
#include "MdlExtUartApp.h"
#endif
#ifdef SUPPORT_RAKE_FLOOD
#include "MdlRakeFloodWell.h"
#endif

/***************************************************************************
 						global variables definition
***************************************************************************/
#define RS485_TR_FLASH_PACK_BLEN		(UART_TR_BUF_BLEN - 16)

/* Modbus通讯地址 */
#define MODADD_SIM_XM_START					0			/* 模拟水位仪表等 */
#define MODADD_SIM_XM_END					1000

#define MODADD_MISC_START					1000		/* 各种杂项部分 */
#define MODADD_DEVICE_START					1000		/* 设备信息 */
	#define MODADD_DEVICE_TYPE				1000		/* 设备型号 */
	#define MODADD_DEVICE_CONFNO			1001		/* 设备配置--子型号 */
	#define MODADD_DEVICE_HARD_VER			1002		/* 硬件版本号，现在同设备型号 */
	#define MODADD_DEVICE_SOFT_VER			1003		/* 软件版本号 */
	#define MODADD_DEVICE_SERNOH			1004		/* 序列号高位 */
	#define MODADD_DEVICE_SERNOL			1005		/* 序列号低位 */
	#define MODADD_DEVICE_CPUTYPE			1006		/* CPU型号，现在空 */
#define MODADD_DEVICE_END 					1010
	
#define MODADD_CTR_START				    1010		/* 控制器部分 */
#define MODADD_CTR_END					    1100
	
#define MODADD_MCGS_ITEMPAGE_START			1120		/* MCGS屏幕 conf,msg,acq显示(ItemPage) */
#define MODADD_MCGS_ITEMPAGE_END			1153
	
#define MODADD_SYS_OPR_START				1190		/* 系统工具页面，每一个功能一个32bit地址(即两个modbus地址)，提供密码输入、状态查询 */
#define MODADD_SYS_OPR_END					1200
#define MODADD_MISC_END						1200

#define MODADD_MSG_START                    1200
#define MODADD_MSG_END                      1300

#define MODADD_MSR_RES_START				9200		/* 机组测量数据(g_MsrRes,R) */
#define MODADD_MSR_RES_END					9600

#define MODADD_ANA_RES_START				9600		/* 机组运行数据(g_AnaRes,R) */
#define MODADD_ANA_RES_END					10000

/* 10000~60000区域用于调试，临时分配 */
#define MODADD_DEBUG_CHN_START				0x3000		/* 12288 仅用于调试，地址可能会被修改，实际访问需要关注 */
#define MODADD_DEBUG_CHN_END				0x3800
/* 10000~60000区域用于调试，临时分配 */

#define MODADD_MON_CMD_START				60000		/* 监控地址 */
#define MODADD_MON_KEY_ID					60000		/* 遥控按钮ID */
#define MODADD_MON_CMD_END					61000

#define MODADD_UPDATE_START					61000		/* 软件升级:通过RS485下载软件，主要是运行版下载为测试版 */
#define MODADD_UPDATE_END					61128

#define MODADD_CODE_TEST_START				0xF000		/* CodeTest变量 */
#define MODADD_CODE_TEST_U32_END			0xF100
#define MODADD_CODE_TEST_U16_END			0xF200
#define MODADD_CODE_TEST_END				0xF200

#define MODADD_MASTER_START					0xFF00		/* 生产配置:0xFF00方便输入 */
#define MODADD_MASTER_END					0xFFFF

/***************************************************************************
						internal functions declaration
***************************************************************************/
#if (DEVICE_TYPE == V5_YYT3)
void TrySerialServer(uint8 u8UartPort);		/* 串口服务器，用于支持modbusTCP随机访问 */
void RunSerialServer(uint8 u8UartPort);
#endif

void TryModbusRTUSlave(uint8 u8UartPort, uint32 u32BaudRate, uint16 uMaxDelay_ms);
void RunModbusRTUSlave(uint8 u8UartPort, uint16 uMaxDelay_ms);

void TrySubSoftUpdate(uint8 u8UartPort);
void RunSubSoftUpdate(uint8 u8UartPort);

/***************************************************************************
 							functions definition
***************************************************************************/
/*==========================================================================
| Description	: 
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2017-9-19
\=========================================================================*/
void InitUartComm(void)
{
	int16 i;

	/* 不同的产品RS485 App初始化 */
#if(DEVICE_TYPE == V5_YKF2)
	g_UartComm[0].UartApp = RS485_MCGS_RTU_SLAVE;
	g_UartComm[1].UartApp = UART_VIEW_TECH;
#elif(SOFT_RUN1_TEST0 && ((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4)))
	g_UartComm[0].UartApp = RS485_APP_NULL;
#else
	for(i = 0; i < MAX_UART_NUM; i++) {
		g_UartComm[i].UartApp = RS485_MCGS_RTU_SLAVE;
	}
#endif

	/* RS485通用初始化 */
	for(i = 0; i < MAX_UART_NUM; i++) {
		g_UartComm[i].Handle = NULL;
		g_UartComm[i].bUartFuncTrySuc = FALSE;				/* 初始化，标志尝试失败 */
		g_UartComm[i].uTimer_WatchDog_ms = 0xFFFF;			/* 初始化，使得后面进入断线尝试状态 */
		g_UartComm[i].LastUartApp = RS485_APP_AUTO;
		g_UartComm[i].u8RS485AppAutoTablePt = 0;
		g_UartComm[i].uLastBaud_100bps = 0;
		g_UartComm[i].uHead = 0;
		g_UartComm[i].uRear = 0;
        InitDataWithZero((uint8*)(&g_UartComm[i].ItemPageReq), sizeof(ITEM_PAGE_REQ));
	}
#if SUPPORT_MODBUS_TCP
	for(i = 0; i < MODBUS_TCP_COMM_MAX_NUM; i++) {
		g_ModbusTcpComm[i].bFree = TRUE;
		g_ModbusTcpComm[i].i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP = 0;
		g_ModbusTcpComm[i].uRxBufPt = 0;
		g_ModbusTcpComm[i].uTxBufPt = 0;
	}
#endif
    InitDataWithZero((uint8*)(&g_MiscCommData.ItemPageReq), sizeof(ITEM_PAGE_REQ));
	
	
#if SUPPORT_V4_MASTER
	InitV4Master();
#endif
}

/*==========================================================================
| Description	: 
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2015-9-27
\=========================================================================*/
/* RS485端口功能表 */
#if (DEVICE_TYPE == V5_YBT2)
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {UART_APP_MAX, RS485_APP_AUTO, RS485_YKFR1_XMS_9600};
#elif((DEVICE_TYPE == V5_YBT3) || (DEVICE_TYPE == V5_YBT4))
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {RS485_YKFR1_XMS_9600, RS485_APP_AUTO, RS485_IEC101};
#elif((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3))
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {RS485_APP_AUTO, UART_VIEW_TECH};
#elif(DEVICE_TYPE == V5_YBU4)
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {RS485_APP_NULL, UART_VIEW_TECH, UART_WATERLVL_OPT_TX};
#elif(DEVICE_TYPE == V5_YYT3)
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {RS485_APP_NULL, RS485_APP_NULL, RS485_APP_NULL, UART_WATERLVL_OPT_RX, UART_GPS};
#elif(DEVICE_TYPE == V5_YYT4)		/* for debug */
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {RS485_APP_NULL, RS485_APP_NULL, RS485_APP_NULL, UART_APP_GPRS};
#elif(DEVICE_TYPE == YKC)
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {RS485_MCGS_RTU_SLAVE};
#endif

/* RS485自动配置表 */
#if (!SOFT_RUN1_TEST0)
const UART_APP cnst_RS485AutoTable[] = {RS485_MCGS_RTU_SLAVE, UART_VIEW_TECH};
#elif ((DEVICE_TYPE == V5_YBT2) || (DEVICE_TYPE == V5_YBT3) || (DEVICE_TYPE == V5_YBT4) || (DEVICE_TYPE == V5_YYT3))
const UART_APP cnst_RS485AutoTable[] = {RS485_SOFT_UPDATE, RS485_MCGS_RTU_SLAVE, RS485_YKFR2_38400, RS485_WATERLV_PGA460};
#elif Is_YKF(DEVICE_TYPE)
const UART_APP cnst_RS485AutoTable[] = {RS485_SOFT_UPDATE, RS485_MCGS_RTU_SLAVE, UART_VIEW_TECH, RS485_V5YKF_PERIPHERAL, RS485_DEBUG_PACK};
#elif((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4))
const UART_APP cnst_RS485AutoTable[] = {RS485_SOFT_UPDATE, RS485_TRIPHASE_METER, RS485_MCGS_RTU_SLAVE, RS485_WATERLV_PGA460};
#elif(DEVICE_TYPE == YKC)
const UART_APP cnst_RS485AutoTable[] = {RS485_MCGS_RTU_SLAVE, RS485_APP_NULL};
#else
const UART_APP cnst_RS485AutoTable[] = {RS485_SOFT_UPDATE, RS485_MCGS_RTU_SLAVE, UART_VIEW_TECH};
#endif

void UartCommTask(void const * argument)
{
	uint8 u8UartPort = (uint8)((uint32)argument);
	if(u8UartPort >= MAX_UART_NUM) {	/* 输入检查, 目的是避免YKF2,YKF3 app.cfg不同 */
		return;
	}

	/* 正常运行 */
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	for(;;) {
#if (!SOFT_RUN1_TEST0)	/* 测试软件，通讯可以选择mcgs或者vt屏幕S */
	UART_APP UartAppConf = RS485_APP_AUTO;
#else	/* 根据不同的设备获取当前的 RS485AppConf */
	#if SUPPORT_BEIDOU_HUB
		UART_APP UartAppConf = RS485_BEIDOU_HUB;
	#elif(Is_YKF(DEVICE_TYPE) || (DEVICE_TYPE == V5_YYD) || (DEVICE_TYPE == V5_YYD2) || (DEVICE_TYPE == V5_YYG))
        UART_APP UartAppConf = RS485_APP_AUTO;
	#else
        UART_APP UartAppConf = RS485_APP_NULL;
        if(0) {
      #if UART_APP_CONF_NUM   /* 有可配置项的，按照可配置的来 */
		} else if(u8UartPort < UART_APP_CONF_NUM) {
			UartAppConf = (UART_APP)g_CommConf.u32RS485App[u8UartPort];	
      #endif
        } else {
            UartAppConf = cnst_UartAppConf[u8UartPort];
		}
	#endif
#endif

		/* 根据配置选择RS485App，如果RS485AppConf==RS485_APP_MAX，则不更改APP */
		if(UartAppConf == RS485_APP_AUTO) {
			if(pUartComm->uTimer_WatchDog_ms >= 3000) {	/* 自动情况下，断线3秒即开始尝试--临时性修改，以免mcgs屏幕输入的时候断开 */
				/* 曾经通讯良好，但是现在断线了  ，需要重新开始搜索RS485App	 */
				if(pUartComm->bUartFuncTrySuc) {
					pUartComm->bUartFuncTrySuc = FALSE;
					pUartComm->u8RS485AppAutoTablePt = 0;
				} else {										/* 配置了自动功能 */
					pUartComm->u8RS485AppAutoTablePt++;
					if(pUartComm->u8RS485AppAutoTablePt >= sizeof(cnst_RS485AutoTable)/sizeof(UART_APP)) {
						pUartComm->u8RS485AppAutoTablePt = 0;
					}
				}
				pUartComm->UartApp = cnst_RS485AutoTable[pUartComm->u8RS485AppAutoTablePt];
			}
		} else if(UartAppConf < UART_APP_MAX) {
			pUartComm->UartApp = UartAppConf;
		}

		/* 需要进行初始化，尝试这种App */
		if(pUartComm->UartApp == RS485_APP_NULL) {
			Task_sleep(OS_TICK_KHz*200);
		} else if((pUartComm->uTimer_WatchDog_ms >= 5000)	/* 固定App断线延时5s--如果是自动情况,由前面控制 */
		#if (DEVICE_TYPE == V5_YYT3)						/* 是否App未改变，但是波特率变了 */
				|| ((pUartComm->UartApp == RS485_MODBUS_RTU_SLAVE) 
					&& (g_CommConf.u32RS485Baud_100bps[u8UartPort] != pUartComm->uLastBaud_100bps))
		#endif
				|| (pUartComm->LastUartApp != pUartComm->UartApp))
		{
			OFF_INDICATOR_RS485(u8UartPort);
			switch(pUartComm->UartApp) {
#if SUPPORT_SCREEN
				case UART_VIEW_TECH:
					TryViewTech(u8UartPort);
					break;
#endif
		#if (SUPPORT_V4_MASTER && SOFT_RUN1_TEST0)
				case RS485_YKFR1_XMS_9600:
					TryYkfR1XmsMaster(u8UartPort, UartAppConf);
					break;
				case RS485_YKFR2_38400:
					TryYkfR2Master(u8UartPort);
					break;
		#endif
				case RS485_MCGS_RTU_SLAVE:
		#if SOFT_RUN1_TEST0
					TryModbusRTUSlave(u8UartPort, 38400UL, 300);
		#else
					TryModbusRTUSlave(u8UartPort, 115200UL, 2000);
		#endif
					break;
		#if(DEVICE_TYPE == V5_YYT3)
				case RS485_DATA_RADIO_SUB:
					TryRadioDataSub(u8UartPort);
					break;
				case RS485_SERIAL_SERVER:
					TrySerialServer(u8UartPort);
					break;
				case RS485_MODBUS_RTU_SLAVE:
					TryModbusRTUSlave(u8UartPort, g_CommConf.u32RS485Baud_100bps[u8UartPort]*100UL, 200);
					pUartComm->uLastBaud_100bps = g_CommConf.u32RS485Baud_100bps[u8UartPort];
					break;
				case RS485_AIR_COMPRESSOR:
					TryAirCompressor(u8UartPort);
					break;
		#elif((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4))
				case RS485_MODBUS_RTU_SLAVE:
					TryModbusRTUSlave(u8UartPort, 1200, 200);
					break;

		#elif(Is_YKF(DEVICE_TYPE) && SOFT_RUN1_TEST0)
				case RS485_V5YKF_PERIPHERAL:
					TryV5YkfPeripheral(u8UartPort);
					break;
		#endif
		#if SUPPORT_IEC101
				case RS485_IEC101:
					TryIEC101(u8UartPort);
					break;
		#endif
		#if SUPPORT_FLOW_METER
				case RS485_FLOW_METER:
					TryDrainMeter(u8UartPort);
					break;
				case RS485_BEIDOU_TERM:
				case RS485_BEIDOU_HUB:
					TryBeiDou(u8UartPort);
					break;
		#endif
		#if SOFT_RUN1_TEST0
				case RS485_SOFT_UPDATE:
					TrySubSoftUpdate(u8UartPort);
					break;
		#endif
				case RS485_DEBUG_PACK:
					TryDebugPack(u8UartPort);
					break;
		#if SUPPORT_TRIPHASE_METER
				case RS485_TRIPHASE_METER:
					TryTriPhaseMeter(u8UartPort);
					break;
		#endif
		#if SUPPORT_PGA460
				case RS485_WATERLV_PGA460:
					TryPga460(u8UartPort);
					break;
		#endif
		#if SUPPORT_WTRLVL_OPT_TX
				case UART_WATERLVL_OPT_TX:
					TryWtrLvlOptTx(u8UartPort);
					break;
		#endif
		#if SUPPORT_WTRLVL_OPT_RX
				case UART_WATERLVL_OPT_RX:
					TryWtrLvlOptRx(u8UartPort);
					break;
		#endif
		#if SUPPORT_PUMP
				case RS485_EXT_DIN_RELAY:
//					TryExtDinRelay(u8UartPort);
					break;
		#endif
		#if SUPPORT_GNL_MODBUS_METER
				case RS485_APP_GNLMODBUS_MASTER:
					TryGnlModbusMaster(u8UartPort, g_CommConf.u32RS485Baud_100bps[u8UartPort]*100UL);
					pUartComm->uLastBaud_100bps = g_CommConf.u32RS485Baud_100bps[u8UartPort];
					break;
		#endif
				default:
					Task_sleep(OS_TICK_KHz*20);
					break;
			}
		} else {	/* 以具体某种App形式进行通讯 */
			switch(pUartComm->UartApp) {
#if SUPPORT_SCREEN
				case UART_VIEW_TECH:
					RunViewTech(u8UartPort);
					break;
#endif
		#if SUPPORT_V4_MASTER
				case RS485_YKFR1_XMS_9600:
					RunYkfR1XmsMaster(u8UartPort);
					break;
				case RS485_YKFR2_38400:
					RunYkfR2Master(u8UartPort);
					break;
		#endif
		#if(DEVICE_TYPE == V5_YYT3)
				case RS485_DATA_RADIO_SUB:
					RunRadioDataSub(u8UartPort);
					break;
				case RS485_SERIAL_SERVER:
					RunSerialServer(u8UartPort);
					break;
				case RS485_AIR_COMPRESSOR:
					RunAirCompressor(u8UartPort);
					break;
		#elif(Is_YKF(DEVICE_TYPE) && SOFT_RUN1_TEST0)
				case RS485_V5YKF_PERIPHERAL:
					RunV5YkfPeripheral(u8UartPort);
					break;
		#endif
				case RS485_MCGS_RTU_SLAVE:
				case RS485_MODBUS_RTU_SLAVE:
					RunModbusRTUSlave(u8UartPort, 100);
					break;
		#if SUPPORT_IEC101
				case RS485_IEC101:
					RunIEC101(u8UartPort);
					break;
		#endif
		#if SUPPORT_FLOW_METER
				case RS485_FLOW_METER:
					RunDrainMeter(u8UartPort);
					break;
				case RS485_BEIDOU_TERM:
					RunBeiDouTerm(u8UartPort);
					break;
			#if SUPPORT_BEIDOU_HUB
				case RS485_BEIDOU_HUB:
					RunBeiDouHub(u8UartPort);
					break;
			#endif
		#endif
		#if SOFT_RUN1_TEST0
				case RS485_SOFT_UPDATE:
					RunSubSoftUpdate(u8UartPort);
					break;
		#endif
				case RS485_DEBUG_PACK:
					RunDebugPack(u8UartPort);
					break;
		#if SUPPORT_TRIPHASE_METER
				case RS485_TRIPHASE_METER:
					RunTriPhaseMeter(u8UartPort);
					break;
		#endif
		#if SUPPORT_PGA460
				case RS485_WATERLV_PGA460:
					RunPga460(u8UartPort);
					break;
		#endif
		#if SUPPORT_WTRLVL_OPT_TX
				case UART_WATERLVL_OPT_TX:
					RunWtrLvlOptTx(u8UartPort);
					break;
		#endif
		#if SUPPORT_WTRLVL_OPT_RX
				case UART_WATERLVL_OPT_RX:
					RunWtrLvlOptRx(u8UartPort);
					break;
		#endif
		#if SUPPORT_PUMP
				case RS485_EXT_DIN_RELAY:
					//RunExtDinRelay(u8UartPort);
					break;
		#endif
		#if SUPPORT_GNL_MODBUS_METER
				case RS485_APP_GNLMODBUS_MASTER:
					RunGnlModbusMaster(u8UartPort);
					break;
		#endif
				default:
					pUartComm->uTimer_WatchDog_ms = 0xFFFF;
					pUartComm->bUartFuncTrySuc = FALSE;
					Task_sleep(OS_TICK_KHz*1000);
					break;
			}
		}		
		pUartComm->LastUartApp = pUartComm->UartApp;
	}
}

/*==========================================================================
| Description	: 
| G/Out var		:
| Author		: Wang Renfei			Date	: 2015-9-27
\=========================================================================*/
void DrvUartTick_1KHz(void)
{
	int16 i;

	/* 通讯管理定时器 */
	for(i = 0; i < MAX_UART_NUM; i++) {
		if(g_UartComm[i].uTimer_WatchDog_ms < 0xFFFF) {
			g_UartComm[i].uTimer_WatchDog_ms++;
		}
		if(g_UartComm[i].uTmr_Run_ms) {
			g_UartComm[i].uTmr_Run_ms--;
		}
	}

	/* 维护按钮key */
#if CTR_BTN_TOTAL_NUM
	for(i = 0; i < CTR_BTN_TOTAL_NUM; i++) {
	    if(g_MiscCommData.u8Tmr_PnlCtrBtn_Valid_ms[i]) {
    	    g_MiscCommData.u8Tmr_PnlCtrBtn_Valid_ms[i]--;
        }
	}
#endif

#if SUPPORT_V4_MASTER
	DrvV4MasterTick_1KHz();
#endif
}


/*==========================================================================
| 以下为ModbusRTU通讯部分，含通用 ModbusSlave, 无线水位主机(ModbusMaster), 以及公共函数(TxDataForModbusRTU)
\=========================================================================*/
typedef enum {
	MODBUS_READ_REG 		= 3,
	MODBUS_READ_IN_REG		= 4,
	MODBUS_WRITE_REG		= 6,
	MODBUS_WRITE_REGS		= 16
}MODBUS_COMMAND_VAR;
#define MON_ID_PRDC						0xFFFF
#define MODBUS_NODE_ADD_UI				0xFF

/*==========================================================================
| Description	: ModbusRTU通讯通用函数
	TxDataForModbusRTU()			: 依据ModbusRTU模式添加CRC，并发送数据
	CheckRxBufForModbusRTUMaster()	: 以ModbusRTU主机格式 检查RxBuf是否正确: 包括CRC, 以及头部信息
	RdDataFrModbusRTU()				: ModbusRTU主机 从Modbus总线设备读取数据
	WrDataToModbusRTU()				: ModbusRTU主机 向Modbus总线设备写入数据
	WaitDataFromModbusRTU()			: ModbusRTU从机 等待Modbus总线上来的合格指令(完成CRC校验)
| G/Out var		:
| Author		: Wang Renfei			Date	: 2017-3-25
\=========================================================================*/
void TxDataForModbusRTU(uint8 u8UartPort, uint8* pTxBuf)
{
	uint8* pU8TRBufStart = g_UartComm[u8UartPort].u8TRBuf;	/* 用指针方式更快 */

	/* CRC部分 */
	uint16 uCRC = CalCRC16ForModbus(pU8TRBufStart, pTxBuf - pU8TRBufStart);
	*pTxBuf++ = uCRC/0x100;
	*pTxBuf++ = uCRC&0xFF;

	/* 发送数据 */
	WriteToUart(u8UartPort, pTxBuf - pU8TRBufStart);
}

UART_ACS_RES CheckCrcForModbusRTU(uint8* pRxBufStart, uint16 uRxNum)
{
    if(uRxNum < 3) {
        return UART_NUM_NOT_ENOUGH;
    } else if(CalCRC16ForModbus(pRxBufStart, uRxNum - 2) != pRxBufStart[uRxNum - 2]*0x100 + pRxBufStart[uRxNum - 1]) {
        return UART_CRC_ERR;
    } else {
		return UART_ACS_SUC;
	}
}

UART_ACS_RES CheckRxBufForModbusRTUMaster(UART_COMM* pUartComm, uint8 u8DevAdd, uint8 u8ModCmd, uint16 uReaddRegLen)
{
    UART_ACS_RES AcsRes;
	if((AcsRes = CheckCrcForModbusRTU(pUartComm->u8TRBuf, pUartComm->uRxBufPt)) <= 0) {
	    return AcsRes;
	} else if(pUartComm->u8TRBuf[0] != u8DevAdd) {
	    return UART_MOD_DEVADD_NEQ;
	} else if(pUartComm->u8TRBuf[1] != u8ModCmd) {
	    return UART_MOD_CMD_NEQ;
	} else {
		if((1 <=  u8ModCmd) && (u8ModCmd <= 4)) {
			/* 计算理论上应该返回的字节数量 */
			if(u8ModCmd <= 2) {
				if(uReaddRegLen % 8 == 0) {
					uReaddRegLen = uReaddRegLen/8;
				} else {
					uReaddRegLen = uReaddRegLen/8 + 1;
				}
			} else {
				uReaddRegLen = uReaddRegLen*2;
			}
			
			/* 校验 */
			if((pUartComm->u8TRBuf[2] + 5 == pUartComm->uRxBufPt) && (pUartComm->u8TRBuf[2] == uReaddRegLen)) {
				return UART_ACS_SUC;
			}
		} else if(u8ModCmd == 0x10) {
			if((pUartComm->uRxBufPt == 8) || ((uint16)pUartComm->u8TRBuf[2]*0x100 + pUartComm->u8TRBuf[3] == MODADD_UPDATE_START)) {
				return UART_ACS_SUC;
			}
		} else if(u8ModCmd == 6) {
			if(pUartComm->uRxBufPt == 8) {
				return UART_ACS_SUC;
			}
		}
        return UART_MOD_RD_FAIL;
	}
}

UART_ACS_RES RdDataFrModbusRTU(uint8 u8UartPort, uint16 uMaxDelay_ms, uint8 u8DevAdd, uint16 uRegAdd, uint16 uRegLen, uint16* pU16Dat, MOD_ACS_TYPE ModAcsType)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];

	/* 生成查询帧 */
	uint8* pTxBuf = pUartComm->u8TRBuf;
	*pTxBuf++ = u8DevAdd;
	uint8 u8ModCmd;
	if(ModAcsType == MOD_RD_01B) {
		u8ModCmd = 0x01;
	} else if(ModAcsType == MOD_RD_02B) {
		u8ModCmd = 0x02;
	} else if((ModAcsType == MOD_RD_03B) || (ModAcsType == MOD_RD_03W) || (ModAcsType == MOD_RD_03D) || (ModAcsType == MOD_RD_03D2143)) {
		u8ModCmd = 0x03;
	} else {
		u8ModCmd = 0x04;
	}
	*pTxBuf++ = u8ModCmd;
	*pTxBuf++ = uRegAdd/0x100;
	*pTxBuf++ = uRegAdd&0xFF;
	*pTxBuf++ = uRegLen/0x100;
	*pTxBuf++ = uRegLen&0xFF;

	/* 数据发送 */
	TxDataForModbusRTU(u8UartPort, pTxBuf);

	/* 接收数据并进行处理 */
	UART_ACS_RES AcsRes;
	if(!ReadFromUart(u8UartPort, uMaxDelay_ms)) {
	    return UART_RD_TIME_OUT;
	} else if((AcsRes = CheckRxBufForModbusRTUMaster(pUartComm, u8DevAdd, u8ModCmd, uRegLen)) <= 0) {
	    return AcsRes;
	} else {
		pUartComm->uTimer_WatchDog_ms = 0;		/* 通讯成功 */
		pUartComm->bUartFuncTrySuc = TRUE;

		if(pU16Dat != NULL) {	/* 数据非指向空 */
			/* 数据拷贝 */
			uint8 u8RcvByteLen = CalMinU16(pUartComm->u8TRBuf[2], uRegLen*2);
			uint8* pRxBuf = pUartComm->u8TRBuf + 3;
			uint8* pU8Dat;
			switch(ModAcsType) {
				case MOD_RD_01B:
				case MOD_RD_02B:
					memcpy(pU16Dat, (const void*)pRxBuf, u8RcvByteLen);
					break;
			
				case MOD_RD_03W:
				case MOD_RD_04W:
					for( ; u8RcvByteLen >= 2; u8RcvByteLen -= 2) {
						*pU16Dat = pRxBuf[0]*0x100 + pRxBuf[1];
						pU16Dat++;
						pRxBuf += 2;
					}
					break;
					
				case MOD_RD_03D:
				case MOD_RD_04D:
					for( ; u8RcvByteLen >= 4; u8RcvByteLen -= 4) {
						*((uint32*)pU16Dat) = (((uint32)pRxBuf[0])<<24) + (((uint32)pRxBuf[1])<<16) + (((uint32)pRxBuf[2])<<8) + (((uint32)pRxBuf[3])<<0);
						pU16Dat += 2;
						pRxBuf += 4;
					}
					break;

				case MOD_RD_04D_I2F:	/* 读取的数据是int32，转成float32填充进buf */
					for( ; u8RcvByteLen >= 4; u8RcvByteLen -= 4) {
						int32 i32Dat = (((uint32)pRxBuf[0])<<24) + (((uint32)pRxBuf[1])<<16) + (((uint32)pRxBuf[2])<<8) + (((uint32)pRxBuf[3])<<0);
						 *((float32*)pU16Dat) = i32Dat;
						pU16Dat += 2;
						pRxBuf += 4;
					}
					break;	

				case MOD_RD_03D2143:
				case MOD_RD_04D2143:
					for( ; u8RcvByteLen >= 2; u8RcvByteLen -= 4) {
						*((uint32*)pU16Dat) = (((uint32)pRxBuf[2])<<24) + (((uint32)pRxBuf[3])<<16) + (((uint32)pRxBuf[0])<<8) + (((uint32)pRxBuf[1])<<0);
						pU16Dat += 2;
						pRxBuf += 4;
					}
					break;

				case MOD_RD_03L:
				case MOD_RD_04L:
					pU8Dat = (uint8*)pU16Dat;
					for( ; u8RcvByteLen >= 8; u8RcvByteLen -= 8) {
						pU8Dat[7] = *pRxBuf++;
						pU8Dat[6] = *pRxBuf++;
						pU8Dat[5] = *pRxBuf++;
						pU8Dat[4] = *pRxBuf++;
						pU8Dat[3] = *pRxBuf++;
						pU8Dat[2] = *pRxBuf++;
						pU8Dat[1] = *pRxBuf++;
						pU8Dat[0] = *pRxBuf++;
						pU8Dat += 8;
					}
					break;

				default:
					memcpy(pU16Dat, (const void*)pRxBuf, u8RcvByteLen);
					break;
			}
		}
		
		return UART_ACS_SUC;
	}
}

UART_ACS_RES WrDataToModbusRTU(uint8 u8UartPort, uint16 uMaxDelay_ms, uint8 u8DevAdd, uint16 uRegAdd, uint16 uRegLen, uint16* pU16Dat, MOD_ACS_TYPE ModAcsType)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	uint8* pTxBuf = pUartComm->u8TRBuf;
	uint8 u8ModCmd = 0;

	/* 输入检查 */
	if((uRegLen == 0)
		|| ((ModAcsType == MOD_WR_10W) && (uRegLen > (UART_TR_BUF_BLEN-10)/2))
		|| ((ModAcsType == MOD_WR_10D) && (uRegLen > (UART_TR_BUF_BLEN-10)/4)))
	{
		return UART_ACS_SUC;		/* 虽然超范围，但是不认为通讯失败 */

	/* 生成发送消息帧 */
	} else if(uRegLen == 1) {		/* 如果类型是0x05 0x06 */
		if(ModAcsType == MOD_WR_05B) {          /* 05指令写  写单个线圈值 */
			u8ModCmd = 0x05;
		} else if(ModAcsType == MOD_WR_06W) {   /* 06指令写  写单个寄存器值,16bit宽度,以2-1顺序 */
            u8ModCmd = 0x06;
		} else {
		    return UART_MOD_WR_FAIL;
		}
		*pTxBuf++ = u8DevAdd;
		*pTxBuf++ = u8ModCmd;
		*pTxBuf++ = uRegAdd/0x100;
		*pTxBuf++ = uRegAdd&0xFF;
		*pTxBuf++ = (*pU16Dat)/0x100;
		*pTxBuf++ = (*pU16Dat)&0xFF;
		
	} else {    /* 否则暗示类型可能是0x0F 0x10  */
		switch(ModAcsType) {
			case MOD_WR_0FB:			/* 0F指令写  写多个线圈值 */
				u8ModCmd = 0x0F;
				break;

			case MOD_WR_10B:			/* 10指令写  写多个寄存器值,不考虑字节序,公司内部成对使用 */
			case MOD_WR_10W:			/* 10指令写  写多个寄存器值,16bit宽度,以2-1顺序 */
			case MOD_WR_10D:			/* 10指令写  写多个寄存器值,以4-3-2-1顺序, 即完全逆序 */
			case MOD_WR_10D2143:		/* 10指令写  写多个寄存器值,TODO:以2-1-4-3顺序,暂时不支持 */
				u8ModCmd = 0x10;
				break;

			default:
                return UART_MOD_WR_FAIL;
		}
		/* 则填充0x0F 0x10相应内容 */
		*pTxBuf++ = u8DevAdd;
		*pTxBuf++ = u8ModCmd;
		*pTxBuf++ = uRegAdd/0x100;
		*pTxBuf++ = uRegAdd&0xFF;
		*pTxBuf++ = uRegLen/0x100;
		*pTxBuf++ = uRegLen&0xFF;
		*pTxBuf++ = uRegLen*2;
		if(ModAcsType == MOD_WR_10W) {
			CopyL16DatToTxBuf(&pTxBuf, (uint8*)pU16Dat, uRegLen, uRegLen);	/* 数据溢出保证在前面的输入检查环节 */
		} else if(ModAcsType == MOD_WR_10D) {
			CopyL32DatToTxBuf(&pTxBuf, (uint8*)pU16Dat, uRegLen, uRegLen);	/* 数据溢出保证在前面的输入检查环节 */
		} else {	/* MOD_WR_0FW / MOD_WR_10B / MOD_WR_10D2143*/
			memcpy(pTxBuf, (const void*)pU16Dat, uRegLen*2);
			pTxBuf += uRegLen*2;
		}
	}

	/* 数据发送 */
	TxDataForModbusRTU(u8UartPort, pTxBuf);

	/* 接收数据并进行处理 */
    UART_ACS_RES AcsRes = UART_ACS_NULL;
	if(u8DevAdd == 0) {			/* 广播数据，没有返回数据，发送完需要等待当前帧结束 */
		Task_sleep((4*pUartComm->uTROneByteTime_us*OS_TICK_KHz + 500)/1000UL);
		pUartComm->bUartFuncTrySuc = TRUE;
		return UART_ACS_SUC;
	} else if(!ReadFromUart(u8UartPort, uMaxDelay_ms)) {
		return UART_RD_TIME_OUT;
	} else if((AcsRes = CheckRxBufForModbusRTUMaster(pUartComm, u8DevAdd, u8ModCmd, 0)) <= 0) {
        return AcsRes;
	} else {
		pUartComm->uTimer_WatchDog_ms = 0; 	/* 通讯成功 */
		pUartComm->bUartFuncTrySuc = TRUE;
		return UART_ACS_SUC;
	}
}

UART_ACS_RES WaitDataFromModbusRTU(uint8 u8UartPort, uint16 uMaxDelay_ms)
{
    UART_ACS_RES AcsRes;
	if(!ReadFromUart(u8UartPort, uMaxDelay_ms)) {
	    return UART_RD_TIME_OUT;
	} else if(g_UartComm[u8UartPort].uRxBufPt < 8) {
	    return UART_NUM_NOT_ENOUGH;
	} else if((AcsRes = CheckCrcForModbusRTU(g_UartComm[u8UartPort].u8TRBuf, g_UartComm[u8UartPort].uRxBufPt)) <= 0) {
	    return AcsRes;
	} else {
		g_UartComm[u8UartPort].uTimer_WatchDog_ms = 0;
		g_UartComm[u8UartPort].bUartFuncTrySuc = TRUE;
		
		return UART_ACS_SUC;
	}
}

/*==========================================================================
| 以下为基于 ModbusRTU 具体APP模块函数
\=========================================================================*/
/*==========================================================================
| Description	: modbus通讯从机任务函数, 包括 TryModbusRTUSlave, RunModbusRTUSlave两个函数
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2017-8-12
\=========================================================================*/
void TryModbusRTUSlave(uint8 u8UartPort, uint32 u32BaudRate, uint16 uMaxDelay_ms)
{
	OpenUartComm(u8UartPort, u32BaudRate, 0, 0);

	RunModbusRTUSlave(u8UartPort, uMaxDelay_ms);
}

void RunModbusRTUSlave(uint8 u8UartPort, uint16 uMaxDelay_ms)
{
	/* 接收数据 */
	if(WaitDataFromModbusRTU(u8UartPort, uMaxDelay_ms) > 0) {
		ON_INDICATOR_RS485(u8UartPort);
	#if(Is_YKF(DEVICE_TYPE) || (DEVICE_TYPE == V5_YYB) || (DEVICE_TYPE == V5_YYD) || (DEVICE_TYPE == V5_YYD2) || (DEVICE_TYPE == V5_YYG))
		RespondModbus(u8UartPort, g_CommConf.u32DevAdd);
	#elif ((DEVICE_TYPE == V5_YBT2) || (DEVICE_TYPE == V5_YBT3) || (DEVICE_TYPE == V5_YBT4))
		RespondModbus(u8UartPort, 2);
	#elif(DEVICE_TYPE == V5_YYT3)
		RespondModbus(u8UartPort, g_CommConf.u32RS485Add[u8UartPort]);
	#elif((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4))
		RespondModbus(u8UartPort, g_CommConf.uRS485Add_SendIntv);
	#else
		RespondModbus(u8UartPort, 0xFF);
	#endif
	} else {
		OFF_INDICATOR_RS485(u8UartPort);
	}
}

/*==========================================================================
| Description	: Modbus通讯消息响应函数，响应Modbus RTU/TCP模式 指令
				 不负责校验消息帧(即CRC是否正确),执行本响应前，默认消息正确
| G/Out var 	: uModbusPort < UART端口数(MAX_UART_NUM)，则认为是ModbusRTU，否则认为是ModbusTCP
| Author		: Wang Renfei			Date	: 2015-10-22
\=========================================================================*/
void RespondModbus(uint8 u8ModbusPort, uint8 u8DevAdd)
{
	/* 预处理 */
	UART_COMM* pUartComm;
	uint8* pModBufStart;		/* 指向modbus消息帧开始，即设备地址 */
	uint16 uMsgNodeAddr;
	uint16 uRxNum;
	if(u8ModbusPort < MAX_UART_NUM) { 									/* 认为是ModbusRTU的调用 */
		uMsgNodeAddr = g_UartComm[u8ModbusPort].u8TRBuf[0];
		pUartComm = &g_UartComm[u8ModbusPort];
		uRxNum = pUartComm->uRxBufPt;
		pModBufStart = pUartComm->u8TRBuf;
		if(uRxNum < 6) {		/* 长度检查，设备地址,指令,数据地址,数据内容或者数据长度 */
			return;
		}
#if SUPPORT_MODBUS_TCP
	} else if(u8ModbusPort < MAX_UART_NUM + MODBUS_TCP_COMM_MAX_NUM) {	/* 认为是ModbusTCP的调用 */
		u8ModbusPort -= MAX_UART_NUM;
		uMsgNodeAddr = g_ModbusTcpComm[u8ModbusPort].u8TRBuf[6];
		pUartComm = NULL;
		uRxNum = g_ModbusTcpComm[u8ModbusPort].uRxBufPt;
		g_ModbusTcpComm[u8ModbusPort].uTxBufPt = 0;
		pModBufStart = g_ModbusTcpComm[u8ModbusPort].u8TRBuf + 6;			/* 指向NodeAddress */
		if(uRxNum < 6 + 6) {	/* 长度检查，modbusTCP开始6个byte + 设备地址,指令,数据地址,数据内容或者数据长度 */
			return;
		}
#endif
	} else {
		return;
	}
	
	if(uMsgNodeAddr && (uMsgNodeAddr != u8DevAdd) && (uMsgNodeAddr != MODBUS_NODE_ADD_UI)) {
#if (SUPPORT_V4_MASTER || SUPPORT_V5_MASTER)
		if(u8ModbusPort < MAX_UART_NUM) {
			RespondModbusRTUForOthers(u8ModbusPort);
		}
#endif
		return;
	}
	uint8* pTRBuf = pModBufStart + 1;			/* 指向ModbusCmd */
	uint8* pTxBufEnd = pModBufStart + UART_TR_BUF_BLEN;

	/* 命令处理 */
	int16 iRegNum;
	uint16 uReg;
	uint16 uRegOff;
	uint16 uProcRegNum;
	BOOL bWriteFlashReq = FALSE;
	MODBUS_COMMAND_VAR ModbusCmd = (MODBUS_COMMAND_VAR)pTRBuf[0];
	uint16 uRegAddr = pTRBuf[1]*0x100 + pTRBuf[2];		/* 已经确认了，这么写是可以的 */
	switch(ModbusCmd) {
		case MODBUS_READ_REG:			/* 用于读取配置值 */
		case MODBUS_READ_IN_REG:		/* 用于读取采样测量值 */
			iRegNum = pTRBuf[3]*0x100 + pTRBuf[4];	/* 已经确认了，这么写是可以的 */
			
			if(iRegNum > (pTxBufEnd - pTRBuf - 4)/2) {				/* 请求的数据长度超过Buf容量 */
				*pTRBuf++ |= 0x80;									/* 指出错误 */
			} else {												/* 正常回复 */
				pTRBuf++;											/* 功能码 */
				pTRBuf++;											/* 预留数据长度 */
				
				while(iRegNum > 0) { 	/* 数据内容 */
					uProcRegNum = 0;
					uReg = 0;
					if(uRegAddr < MODADD_SIM_XM_END) {
						if((uRegAddr == 0) && (ModbusCmd == MODBUS_READ_IN_REG)) {
					#if(SUPPORT_NET && ((DEVICE_TYPE == V5_YBT2) || (DEVICE_TYPE == V5_YBT3) || (DEVICE_TYPE == V5_YBT4)))
							if(g_TcpIPComm.RmtSenFromMqtt.fSenData[0] < -0.05) {
								uMsgNodeAddr = 0;		/* 让函数结尾不发送回复，造成“断线” */
							} else {
								uReg = F32ToU16(g_TcpIPComm.RmtSenFromMqtt.fSenData[0]*g_YBTConf.fWaterSenRange*1000);
							}
					#elif((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4))	/* 模拟YBU的AD量程接口 */
							uReg = F32ToU16((g_AnaRes.fLocalSensor + 0.25f) * 24000.0f);
					#elif(DEVICE_TYPE == V5_YYT3)	/* YYT3在模拟水位仪表的时候，需要响应读取配置值 */
							uReg = F32ToU16(g_AnaRes.LogicWtrLvl[LOGIC_WTR_TURB].fWtrLvl * 1000.0f);
						} else if((0 < uRegAddr) && (uRegAddr <= CONF_FOR_YKF_NUM) && (ModbusCmd == MODBUS_READ_REG)) {
							uReg = F32ToU16(g_YYTConf.fConfForYKF[uRegAddr - 1] * 1000.0f);
					#endif
						}
					} else if((MODADD_MISC_START <= uRegAddr) && (uRegAddr < MODADD_MISC_END)) {
						if((MODADD_DEVICE_START <= uRegAddr) && (uRegAddr < MODADD_DEVICE_END)) {
							if(uRegAddr == MODADD_DEVICE_TYPE) {
								uReg = DEVICE_TYPE;
							} else if(uRegAddr == MODADD_DEVICE_HARD_VER) {
								uReg = DEVICE_TYPE;
							} else if(uRegAddr == MODADD_DEVICE_SOFT_VER) {
								uReg = SOFTWARE_VER;
							} else if(uRegAddr == MODADD_DEVICE_SERNOL) {
								uReg = g_Sys.SerialNo.u32Dat&0xFFFF;
							} else if(uRegAddr == MODADD_DEVICE_SERNOH) {
								uReg = g_Sys.SerialNo.u32Dat/0x10000;
							}
						} else if((MODADD_CTR_START <= uRegAddr) && (uRegAddr < MODADD_CTR_END)) {
							uRegOff = uRegAddr - MODADD_CTR_START;
							uProcRegNum = CopyL32DatToTxBuf(&pTRBuf, (uint8*)(&g_Ctr) + uRegOff*2, iRegNum, sizeof(g_Ctr)/2 - uRegOff);
#if SUPPORT_SCREEN
						} else if((MODADD_MCGS_ITEMPAGE_START <= uRegAddr) && (uRegAddr < MODADD_MCGS_ITEMPAGE_END)) {
							if((pUartComm != NULL)	&& (uRegAddr == MODADD_MCGS_ITEMPAGE_START)
								&& (pTRBuf + MCGS_ITEM_PAGE_ACK_MAX_BLEN < pTxBufEnd))
							{
								FillItemPageAck_MCGS(&pUartComm->ItemPageReq, pTRBuf);
								pTRBuf += MCGS_ITEM_PAGE_LINE_WLEN*2;
								uProcRegNum = MCGS_ITEM_PAGE_LINE_WLEN;
							}
#endif
						} else if((MODADD_SYS_OPR_START <= uRegAddr) && (uRegAddr < MODADD_SYS_OPR_END)) {
							uRegOff = uRegAddr - MODADD_SYS_OPR_START;
							//uProcRegNum = CopyL32DatToTxBuf(&pTRBuf, (uint8*)g_DataAcsIntf.i8CheckGlbConf + uRegOff*2, RegNum, MAX_SYS_OPR_NUM);
						}
					} else if((MODADD_DEBUG_CHN_START  <= uRegAddr) && (uRegAddr < MODADD_DEBUG_CHN_END)) {
						CopyDebugChnToTxBuf(&pTRBuf, uRegAddr - MODADD_DEBUG_CHN_START);
						uProcRegNum = iRegNum;
				#if MAX_MSG_TYPE
                    } else if((MODADD_MSG_START <= uRegAddr) && (uRegAddr < MODADD_MSG_END)) {  /* Msg部分 */
                        uProcRegNum = TxMsgForModbus(&pTRBuf, iRegNum);
				#endif
					} else if((MODADD_MSR_RES_START <= uRegAddr) && (uRegAddr < MODADD_MSR_RES_END)) {	/* g_MsrRes部分 */
						uRegOff = uRegAddr - MODADD_MSR_RES_START;
						uProcRegNum = CopyL32DatToTxBuf(&pTRBuf, (uint8*)(&g_MsrRes) + uRegOff*2, iRegNum, sizeof(g_MsrRes)/2 - uRegOff);
					} else if((MODADD_ANA_RES_START <= uRegAddr) && (uRegAddr < MODADD_ANA_RES_END)) { 	/* g_CtrObj部分 */
						uRegOff = uRegAddr - MODADD_ANA_RES_START;
						uProcRegNum = CopyL32DatToTxBuf(&pTRBuf, (uint8*)(&g_AnaRes) + uRegOff*2, iRegNum, sizeof(g_AnaRes)/2 - uRegOff);
					} else if((MODADD_CODE_TEST_START <= uRegAddr) && (uRegAddr < MODADD_CODE_TEST_END)) {
						if(uRegAddr < MODADD_CODE_TEST_U32_END) {
							uProcRegNum = CopyL32DatToTxBuf(&pTRBuf, (uint8*)(&g_CodeTest.u32Val), iRegNum, CODE_TEST_UINT32_NUM*2);
						} else {
							uProcRegNum = CopyL16DatToTxBuf(&pTRBuf, (uint8*)(&g_CodeTest.uVal), iRegNum, CODE_TEST_UINT16_NUM);
						}
					} else if((MODADD_MASTER_START <= uRegAddr) && (uRegAddr < MODADD_MASTER_END)) {
						uProcRegNum = CopyL16DatToTxBuf(&pTRBuf, (uint8*)(&g_PubSoftAuthCtr), iRegNum, sizeof(g_PubSoftAuthCtr)/2);
					}

					if(uProcRegNum) {
						uRegAddr += uProcRegNum;
						iRegNum -= uProcRegNum;
					} else {
						*pTRBuf++ = uReg/0x100;
						*pTRBuf++ = uReg&0xFF;
						uRegAddr++;
						iRegNum--;
					}
				}
				
				pModBufStart[2] = pTRBuf - pModBufStart - 3;	/* 填充数据长度 */
			}
			break;

		case MODBUS_WRITE_REG:
		case MODBUS_WRITE_REGS:
			/* 指令动作 */
			if(ModbusCmd == MODBUS_WRITE_REG) {
				iRegNum = 1;
				pTRBuf += 3;			/* Rx指针指向数据 */
			} else if(ModbusCmd == MODBUS_WRITE_REGS) {
				iRegNum = pTRBuf[3]*0x100 + pTRBuf[4];							/* 已经确认了，这么写是可以的 */
				if((iRegNum*2 != pTRBuf[5]) 	/* 长度错误检测 */
					|| ((pUartComm != NULL) && (iRegNum*2 + 9 != uRxNum))		/* modbusRTU有CRC */
					|| ((pUartComm == NULL) && (iRegNum*2 + 13 != uRxNum)))		/* modbusTCP无CRC */
				{
					iRegNum = 0;
					pTRBuf[0] |= 0x80;
				}
				pTRBuf += 6;			/* Rx指针指向数据 */
			}
	
			while(iRegNum > 0) {
				uReg = pTRBuf[0]*0x100 + pTRBuf[1];
				uProcRegNum = 0;

				if((MODADD_MCGS_ITEMPAGE_START <= uRegAddr) && (uRegAddr < MODADD_MCGS_ITEMPAGE_END)) {
					if((uRegAddr == MODADD_MCGS_ITEMPAGE_START) && (iRegNum >= 3)) { /* 必须在RS485通讯端口调用才行 */
						pTRBuf[iRegNum*2] = 0;	/* 加上字符串结束符 */
						uint32 u32ItemNo = (uint32)pTRBuf[2]*0x1000000UL + (uint32)pTRBuf[3]*0x10000UL
											+ (uint32)pTRBuf[4]*0x100UL + (uint32)pTRBuf[5];
						DATA_PAGE DataPage = (DATA_PAGE)pTRBuf[1];
						pTRBuf += 6;
						if(pUartComm == NULL) {
							ProcItemPageInstr(DATA_USER_MCGS, DataPage, u32ItemNo, &pTRBuf, pTRBuf + (iRegNum-3)*2, &g_MiscCommData.ItemPageReq);
						} else {
							ProcItemPageInstr(DATA_USER_MCGS, DataPage, u32ItemNo, &pTRBuf, pTRBuf + (iRegNum-3)*2, &pUartComm->ItemPageReq);
						}
						uProcRegNum = iRegNum;
					}
				} else if((MODADD_MON_CMD_START <= uRegAddr) && (uRegAddr < MODADD_MON_CMD_END)) {
				    uProcRegNum = WriteMosbusMon(uRegAddr, pTRBuf, iRegNum);
				    pTRBuf += uProcRegNum*2;
			#if SOFT_RUN1_TEST0
				} else if((MODADD_MASTER_START <= uRegAddr) && (uRegAddr < MODADD_MASTER_END)) {
					ProcVersionMasterBCD(pTRBuf, iRegNum);
					uProcRegNum = iRegNum;
			#else
				} else if((MODADD_UPDATE_START <= uRegAddr) && (uRegAddr < MODADD_UPDATE_END)) {
					if(iRegNum*2 == sizeof(FLASH_REQ)) { 	/* 写全了FlashReq */
						FLASH_REQ FlashReq;
						memcpy(&FlashReq, (const void*)pTRBuf, iRegNum*2);		/* 无需把字节顺序调整到modbus格式 */
						pTRBuf = pModBufStart + 8;	/* 进行32bit对齐 */
						pTRBuf += PubSoftware(&FlashReq, (uint32*)pTRBuf, RS485_TR_FLASH_PACK_BLEN, 0);
						uProcRegNum = iRegNum;
						bWriteFlashReq = TRUE;
					}
			#endif
				}

				if(uProcRegNum) {
					uRegAddr += uProcRegNum;
					iRegNum -= uProcRegNum;
				} else {
					uRegAddr++;
					iRegNum--;
					pTRBuf += 2;
				}
			}
			if(!bWriteFlashReq) {	/* 如果不是写FlashReq, 不管是06还是0x10指令，回复数据长度都是6 */
				pTRBuf = pModBufStart + 6;
			}
			break;

		default:
			break;
	}
	
	/* 消息发送控制 */
	if(uMsgNodeAddr) {	/* 地址非0，即不是广播地址 */
		if(pUartComm != NULL) {	/* modbusRTU */
			TxDataForModbusRTU(u8ModbusPort, pTRBuf);
		} else {					/* modbusTCP */
#if SUPPORT_MODBUS_TCP
			g_ModbusTcpComm[u8ModbusPort].uTxBufPt = pTRBuf - g_ModbusTcpComm[u8ModbusPort].u8TRBuf;
			g_ModbusTcpComm[u8ModbusPort].u8TRBuf[5] = g_ModbusTcpComm[u8ModbusPort].uTxBufPt - 6;
#endif
		}
	}
}

extern uint16 ProcLocalCmdInstr(uint8* pRxBuf, uint16 uRegOff, int16 RegNum);
uint16 WriteMosbusMon(uint16 uRegAddr, uint8* pRxBuf, uint16 uRegLen)
{
    uint16 uProcRegNum = 0;
	if((MODADD_MON_CMD_START <= uRegAddr) && (uRegAddr <= MODADD_MON_CMD_END)) {
        uint16 uRegOff = uRegAddr - MODADD_MON_CMD_START;
        if(uRegOff < 4) {           /* 60000~60003是所有产品公共部分 */
        #if CTR_BTN_MODTCP_NUM
            if(uRegOff == 0) {      /* 60000是屏幕按钮 */
				uint16 PnlCtrBtn = pRxBuf[0]*0x100 + pRxBuf[1];
				int8 i;
				for(i = 0; i < CTR_BTN_MODTCP_NUM; i++) {
				    if(PnlCtrBtn & (1<<i)) {
				        g_MiscCommData.u8Tmr_PnlCtrBtn_Valid_ms[i] = KEY_VALID_TIME_ms;
				    }
				}
            }
        #endif
            uProcRegNum = 4 - uRegOff;
            if(uProcRegNum < uRegLen) {
                uProcRegNum = uRegLen;
            }
        } else {                    /* 每个产品需要有自己的ProcLocalCmdInstr() */
            uProcRegNum = ProcLocalCmdInstr(pRxBuf, uRegOff - 4, uRegLen);
        }
    }
    return uProcRegNum;
}

#if (SUPPORT_V4_MASTER || SUPPORT_V5_MASTER)
/*==========================================================================
| 以下是Modbus访问其他设备数据(存在于V4Master, V5Master)的通讯函数，包括如下函数：
RespondModbusTCPForOthers(): ModbusTCP
RespondModbusRTUForOthers(): ModbusRTU
RespondModbusForV4()   : 提供给RespondModbusTCPForV4(), RespondModbusRTUForOthers() 调用，填充设备点表数据
\=========================================================================*/
typedef enum {
	RESPOND_OTHERS_MODBUS_RES_SUC = 0,		/* 成功 */
	RESPOND_OTHERS_MODBUS_RES_V4_NO_DEV,	/* V4异常: 非Buf中的设备，modbusTCP要随机访问, modbusRTU无响应 */
	RESPOND_OTHERS_MODBUS_RES_V4_NO_REG,	/* V4异常: 命中设备，但非ram中的数据，modbusTCP要随机访问, modbusRTU报错 */
	RESPOND_OTHERS_MODBUS_RES_V5_NO_DEV,	/* V5异常: 非Buf中的设备，modbusTCP要随机访问, modbusRTU无响应 */
	RESPOND_OTHERS_MODBUS_RES_V5_NO_REG,	/* V5异常: 命中设备，但非ram中的数据，modbusTCP要随机访问, modbusRTU报错 */
	RESPOND_OTHERS_MODBUS_RES_FAIL,			/* 失败(V4,V5): 命中设备，但异常且modbusTCP无需随机访问，直接报错 */
}RESPOND_OTHERS_MODBUS_RES;
RESPOND_OTHERS_MODBUS_RES ModbusReadFromRamForOthers(uint8** ppModbusRTUBuf);

void RespondModbusTCPForOthers(MODBUS_TCP_COMM* pModbusTcpComm, BOOL bBlock)
{
	if(pModbusTcpComm->uRxBufPt < 6 + 6) {	/* 长度检查，modbusTCP开始6个byte + 设备地址,指令,数据地址,数据内容或者数据长度 */
		pModbusTcpComm->uRxBufPt = 0;		/* 长度错误，不响应 */
	} else {
		pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP = 0;	/* 如果之前有随机访问未结束的，放弃--随机访问所在的RS485任务的优先级更高 */
		uint8* pTRBuf = pModbusTcpComm->u8TRBuf + 6;	/* 指向设备地址，即modbusRTU部分 */
		RESPOND_OTHERS_MODBUS_RES Res = ModbusReadFromRamForOthers(&pTRBuf);

		if(Res == RESPOND_OTHERS_MODBUS_RES_SUC) {	/* ram中命中了数据，补上数据长度，并标志发送指针 */
			pModbusTcpComm->uTxBufPt = pTRBuf - pModbusTcpComm->u8TRBuf;
			pModbusTcpComm->u8TRBuf[5] = pModbusTcpComm->uTxBufPt - 6;
		/* 访问的数据不在ram中 */
		} else if((Res == RESPOND_OTHERS_MODBUS_RES_V4_NO_DEV) || (Res == RESPOND_OTHERS_MODBUS_RES_V4_NO_REG)) {
			pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP = 3;	/* 提交给RS485任务处理 */
			if(!bBlock) {				/* 不阻塞，直接返回 */
				pModbusTcpComm->uTxBufPt = 0;
				return;
			} else {					/* 阻塞，等待结果 */
				int16 i;
				for(i = 10; (i > 0) && (pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP > 0); i--) {
					Task_sleep(OS_TICK_KHz*20);
				}
		
				if(pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP == -1) {
					Res = RESPOND_OTHERS_MODBUS_RES_SUC;	/* 数据指针uTxBufPt, 数据长度u8TRBuf[5]已经在V4Master里面填充完成了 */
				} else {
					Res = RESPOND_OTHERS_MODBUS_RES_FAIL;
				}
			}
	#if SUPPORT_V5_MASTER
		} else if(Res == RESPOND_OTHERS_MODBUS_RES_V5_NO_REG) {
			if(TranModbusByCan(&pTRBuf)) {	/* 补上数据长度，并标志发送指针 */
				pModbusTcpComm->uTxBufPt = pTRBuf - pModbusTcpComm->u8TRBuf;
				pModbusTcpComm->u8TRBuf[5] = pModbusTcpComm->uTxBufPt - 6;
				Res = RESPOND_OTHERS_MODBUS_RES_SUC;
			}
	#endif
		}

		if(Res != RESPOND_OTHERS_MODBUS_RES_SUC) {		/* 如果通讯失败(含随机访问失败)，响应数据 */
			pModbusTcpComm->u8TRBuf[5] = 4; 			/* 理论应该是3，但就是不能正常工作，必须要4 */
			pModbusTcpComm->u8TRBuf[7] |= 0x80;
			pModbusTcpComm->u8TRBuf[8] = 0x0B;
			pModbusTcpComm->uTxBufPt = 9;
		}
	}
}

/* 仅支持03,04指令, 长度检查在调用前已经做了，因此这里没有必要重复 */
void RespondModbusRTUForOthers(uint8 u8ModbusPort)
{
	uint8* pTRBuf = g_UartComm[u8ModbusPort].u8TRBuf;
	uint16 uRes = ModbusReadFromRamForOthers(&pTRBuf);
	if(uRes == RESPOND_OTHERS_MODBUS_RES_SUC) {
		TxDataForModbusRTU(u8ModbusPort, pTRBuf);
	} else if((uRes == RESPOND_OTHERS_MODBUS_RES_V4_NO_REG) || (uRes == RESPOND_OTHERS_MODBUS_RES_FAIL)) {
		g_UartComm[u8ModbusPort].u8TRBuf[1] |= 0x80;
		pTRBuf = &g_UartComm[u8ModbusPort].u8TRBuf[6];
		TxDataForModbusRTU(u8ModbusPort, pTRBuf);
	}
}

/* 搜索设备点表，形成modbus帧 */
RESPOND_OTHERS_MODBUS_RES ModbusReadFromRamForOthers(uint8** ppModbusRTUBuf)
{
	uint8* pTRBufStart = *ppModbusRTUBuf;	/* 指向modbusRTU的数据区，即设备地址位置 */
	uint8* pTxBuf = pTRBufStart + 3;		/* 空出数据长度，指向内容填充区 */
	uint8 u8DevAdd = pTRBufStart[0];
	uint8 u8MchNo = u8DevAdd/10 - 1;		/* 用于数组下标 */
	uint8 u8ModCmd = pTRBufStart[1];
	uint16 uRegAdd = pTRBufStart[2]*0x100 + pTRBufStart[3];
	int16 iRegLen = pTRBufStart[4]*0x100 + pTRBufStart[5];
	uint16 uProcRegNum = 0;

	if(u8DevAdd < 10) {
		if(u8DevAdd == MODDEV_ADD_XMG) {	/* 读XMG水位 */
			if((iRegLen*2 + 10 > UART_TR_BUF_BLEN) || ((u8ModCmd != 0x03) && (u8ModCmd != 0x04))) {
				return RESPOND_OTHERS_MODBUS_RES_FAIL;
		#if(DEVICE_TYPE == V5_YYT3) 	/* YYT3在模拟水位仪表的时候，需要响应读取配置值 */
			} else if(uRegAdd < 2) {
				CopyL32DatToTxBuf(&pTxBuf, (uint8*)&g_AnaRes.LogicWtrLvl[LOGIC_WTR_TURB].fWtrLvl, iRegLen, 2);
			} else if(uRegAdd < (CONF_FOR_YKF_NUM + 1)*2) {
				uRegAdd -= 2;
				CopyL32DatToTxBuf(&pTxBuf, (uint8*)&g_YYTConf.fConfForYKF[uRegAdd/2], iRegLen, CONF_FOR_YKF_NUM*2 - uRegAdd);
		#else
			} else if(uRegAdd < 2) {
				CopyL32DatToTxBuf(&pTxBuf, (uint8*)&g_AnaRes.fTurbWaterLevel, iRegLen, 2);
			} else if(!RespondModbusForReadXmgConf(&pTxBuf, uRegAdd - 2, iRegLen)) {
				return RESPOND_OTHERS_MODBUS_RES_FAIL;
		#endif
			}
		} else {
			return RESPOND_OTHERS_MODBUS_RES_V4_NO_DEV;
		}
	} else if(u8DevAdd%10 == MODDEV_ADDi_YKF) {			/* 读的是YKF控制器,地址编码10,20,30... */
		if(0) {
	#if SUPPORT_V4_MASTER
		} else if((u8MchNo < MAX_YKF_NUM_V4) && g_YKFnXMSData[u8MchNo].u8YkfR2TryCnt_0Fail) { 	/* V4_YKF控制器在线 */
			if((u8ModCmd != 0x03) && (u8ModCmd != 0x04)) {
				return RESPOND_OTHERS_MODBUS_RES_V4_NO_REG;
			} else if(iRegLen*2 + 10 > UART_TR_BUF_BLEN) {	/* buf溢出 */
				return RESPOND_OTHERS_MODBUS_RES_FAIL;
			} else {
				BOOL bFindDataInRam = FALSE;
				while(iRegLen > 0) {
					if((1000 <= uRegAdd) && (uRegAdd < 1000 + YKF_REG_LEN_1000)) {
						uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, (uint8*)(&g_YKFnXMSData[u8MchNo].uYKFReg1000[uRegAdd - 1000]), iRegLen, 1000 + YKF_REG_LEN_1000 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((1140 <= uRegAdd) && (uRegAdd < 1140 + YKF_REG_LEN_1140)) {
						uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, (uint8*)(&g_YKFnXMSData[u8MchNo].uYKFReg1140[uRegAdd - 1140]), iRegLen, 1140 + YKF_REG_LEN_1140 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((9800 <= uRegAdd) && (uRegAdd < 9800 + YKF_REG_LEN_9800)) {
						uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, (uint8*)(&g_YKFnXMSData[u8MchNo].uYKFReg9800[uRegAdd - 9800]), iRegLen, 9800 + YKF_REG_LEN_9800 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((9800 + YKF_REG_LEN_9800 <= uRegAdd) && (uRegAdd < 9900)) {
						if(iRegLen > 100 - YKF_REG_LEN_9800) {
							uProcRegNum = 100 - YKF_REG_LEN_9800;
						} else {
							uProcRegNum = iRegLen;
						}
						memset(pTxBuf, 0, uProcRegNum*2);
						pTxBuf += uProcRegNum*2;
					} else if((9900 <= uRegAdd) && (uRegAdd < 9900 + YKF_REG_LEN_9900)) {
						uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, (uint8*)(&g_YKFnXMSData[u8MchNo].uYKFReg9900[uRegAdd - 9900]), iRegLen, 9900 + YKF_REG_LEN_9900 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if(bFindDataInRam) {
						*pTxBuf++ = 0;
						*pTxBuf++ = 0;
						uProcRegNum = 1;
					} else {
						return RESPOND_OTHERS_MODBUS_RES_V4_NO_REG;
					}
					uRegAdd += uProcRegNum;
					iRegLen -= uProcRegNum;
				}
			}
	#endif
	#if SUPPORT_V5_MASTER
		} else if((u8MchNo < MAX_YKF_NUM) && g_V5YKFData[u8MchNo].u8TryCnt_0Fail) {
			if((u8ModCmd != 0x03) && (u8ModCmd != 0x04)) {
				return RESPOND_OTHERS_MODBUS_RES_V5_NO_REG;
			} else if(iRegLen*2 + 10 > UART_TR_BUF_BLEN) {	/* buf溢出 */
				return RESPOND_OTHERS_MODBUS_RES_FAIL;
			} else {
				BOOL bFindDataInRam = FALSE;
				while(iRegLen > 0) {
					if((1010 <= uRegAdd) && (uRegAdd < 1010 + V5_CTR_LEN_1010)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)(&g_V5YKFData[u8MchNo].u32Ctr1010[(uRegAdd - 1010)/2]), iRegLen, 1010 + V5_CTR_LEN_1010*2 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((9200 <= uRegAdd) && (uRegAdd < 9200 + YKF_MSR_LEN_9200)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)(&g_V5YKFData[u8MchNo].u32MsrRes9200[(uRegAdd - 9200)/2]), iRegLen, 9200 + YKF_MSR_LEN_9200*2 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((9600 <= uRegAdd) && (uRegAdd < 9600 + YKF_ANA_LEN_9600)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)(&g_V5YKFData[u8MchNo].u32AnaRes9600[(uRegAdd - 9600)/2]), iRegLen, 9600 + YKF_ANA_LEN_9600*2 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if(bFindDataInRam) {
						*pTxBuf++ = 0;
						*pTxBuf++ = 0;
						uProcRegNum = 1;
					} else {
						return RESPOND_OTHERS_MODBUS_RES_V5_NO_REG;
					}
					uRegAdd += uProcRegNum;
					iRegLen -= uProcRegNum;
				}
			}
	#endif
		} else {
			return RESPOND_OTHERS_MODBUS_RES_FAIL;
		}
	} else if(u8DevAdd%10 == MODDEV_ADDi_XMS) { 				/* XMS温度巡检仪, 地址编码13,23,33... */
		if(((u8ModCmd != 0x03) && (u8ModCmd != 0x04))		/* 不支持的指令(仅支持03,04指令) */
			|| (iRegLen*2 + 10 > UART_TR_BUF_BLEN))			/* Buf溢出检查 */
		{
			return RESPOND_OTHERS_MODBUS_RES_FAIL;
	#if SUPPORT_V4_MASTER
		} else if((u8MchNo < MAX_YKF_NUM_V4) && (uRegAdd < g_YKFnXMSData[u8MchNo].uXMSSenNum)) {	/* 该温度传感器断线 */
			CopyL16DatToTxBuf(&pTxBuf, (uint8*)(&g_YKFnXMSData[u8MchNo].uXMSData[uRegAdd]), iRegLen, g_YKFnXMSData[u8MchNo].uXMSSenNum - uRegAdd);
	#endif
		} else {
			return RESPOND_OTHERS_MODBUS_RES_FAIL;
		}
	} else if(u8DevAdd%10 == MODDEV_ADDi_YYB) {				/* 读的是YYB控制器,地址编码17,27,37... */
		if(0) {
	#if SUPPORT_V4_MASTER
		} else if((u8MchNo < MAX_RCX_NUM) && g_RcxData[u8MchNo].uRcxTryCnt_0Fail) {
			if((u8ModCmd != 0x03) && (u8ModCmd != 0x04)) {
				return RESPOND_OTHERS_MODBUS_RES_V4_NO_REG;
			} else if(iRegLen*2 + 10 > UART_TR_BUF_BLEN) {	/* buf溢出 */
				return RESPOND_OTHERS_MODBUS_RES_FAIL;
			} else {
				while(iRegLen > 0) {
					if(uRegAdd == 0x800) {							/* 开入 */
						uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, (uint8*)&g_RcxData[u8MchNo].uDIn0x800, iRegLen, 1);
					} else if(uRegAdd == 0xB00) { 					/* 开出 */
						uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, (uint8*)&g_RcxData[u8MchNo].uRelay0xB00, iRegLen, 1);
					} else if((uRegAdd == 0xA00) || (uRegAdd == 0xA01)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)&g_RcxData[u8MchNo].u32EngP_0x0A00_KWH, iRegLen, 2);
					} else if((uRegAdd == 0xA04) || (uRegAdd == 0xA05)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)&g_RcxData[u8MchNo].u32EngQ_0x0A04_KVarH, iRegLen, 2);
					} else if((uRegAdd == 0xA02) || (uRegAdd == 0xA03)) {
						*pTxBuf++ = 0;
						*pTxBuf++ = 0;
						uProcRegNum = 1;
					} else if((0x600 <= uRegAdd) && (uRegAdd < 0x600 + RCX_REG_LEN_0x600)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)&g_RcxData[u8MchNo].fReg0x600[(uRegAdd-0x600)/2], iRegLen, 0x600 + RCX_REG_LEN_0x600 - uRegAdd);
					} else {
						return RESPOND_OTHERS_MODBUS_RES_FAIL;
					}
					uRegAdd += uProcRegNum;
					iRegLen -= uProcRegNum;
				}
			}
	#endif
	#if SUPPORT_V5_MASTER
		} else if((u8MchNo < MAX_YYB_NUM) && g_YYBData[u8MchNo].u8TryCnt_0Fail) {
			if((u8ModCmd != 0x03) && (u8ModCmd != 0x04)) {
				return RESPOND_OTHERS_MODBUS_RES_V5_NO_REG;
			} else if(iRegLen*2 + 10 > UART_TR_BUF_BLEN) {	/* buf溢出 */
				return RESPOND_OTHERS_MODBUS_RES_FAIL;
			} else {
				BOOL bFindDataInRam = FALSE;
				while(iRegLen > 0) {
					if((1010 <= uRegAdd) && (uRegAdd < 1010 + V5_CTR_LEN_1010)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)(&g_YYBData[u8MchNo].u32Ctr1010[(uRegAdd - 1010)/2]), iRegLen, 1010 + V5_CTR_LEN_1010*2 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((9200 <= uRegAdd) && (uRegAdd < 9200 + YYB_MSR_LEN_9200)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)(&g_YYBData[u8MchNo].u32MsrRes9200[(uRegAdd - 9200)/2]), iRegLen, 9200 + YYB_MSR_LEN_9200*2 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((9600 <= uRegAdd) && (uRegAdd < 9600 + YYB_ANA_LEN_9600)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)(&g_YYBData[u8MchNo].u32AnaRes9600[(uRegAdd - 9600)/2]), iRegLen, 9600 + YYB_ANA_LEN_9600*2 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if(bFindDataInRam) {
						*pTxBuf++ = 0;
						*pTxBuf++ = 0;
						uProcRegNum = 1;
					} else {
						return RESPOND_OTHERS_MODBUS_RES_V5_NO_REG;
					}
					uRegAdd += uProcRegNum;
					iRegLen -= uProcRegNum;
				}
			}
	#endif
		} else {
			return RESPOND_OTHERS_MODBUS_RES_FAIL;
		}
#if SUPPORT_TRIPHASE_METER		/* 多功能三相电表 */
	} else if(u8DevAdd%10 == MODDEV_ADDi_TPM) {
		if((0x03 == u8ModCmd || 0x04 == u8ModCmd) && (iRegLen*2 + 10 <= UART_TR_BUF_BLEN) && u8MchNo < MAX_TRIPHA_METER_NUM) {			/* 处理读指令 */
			BOOL bFindDataInRam = FALSE;
			while(iRegLen > 0) {
				if(TRIPHA_MTR_REPUB_REG_DIO == uRegAdd) {		/* 开入开出 */
					if(g_TriPhasMeter[u8MchNo].uTryCnt_0Fail) {		
						*(uint32*)pTxBuf = g_TriPhasMeter[u8MchNo].uDIO;
						pTxBuf += 4;
						uProcRegNum = 2;
						bFindDataInRam = TRUE;
					} else {
						return RESPOND_OTHERS_MODBUS_RES_FAIL;
					}
#if SUPPORT_PUMP
				} else if((uRegAdd >= PUMP_CTL_STATE) && (uRegAdd <= PUMP_RELAY)) {			/* 泵站运行状态、通讯状态、开入、开出、 */
					uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, ((uint8*)&g_PumpCtr[u8MchNo])+((uRegAdd-PUMP_CTL_STATE)*2), 2, 2);
				} else if((uRegAdd >= PUMP_RUN_MODE) && (uRegAdd <= PUMP_READY_STOP_COUNTDOWN)) {		/* 泵站控制状态2 */
					uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, ((uint8*)&g_PumpCtr[u8MchNo])+((uRegAdd-PUMP_CTL_STATE)*2), 1, 1);
				} else if((uRegAdd >= PUMP_ACT_WATER_LEVEL) && (uRegAdd <= PUMP_STOP_WATER_LEVEL)) {		/* 智能启\停机预设水位 */
					uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, ((uint8*)&g_PumpCommConf)+((uRegAdd-PUMP_ACT_WATER_LEVEL)*2), 2, 2);
				} else if((uRegAdd >= PUMP_BTN_ECHO) && (uRegAdd <= PUMP_BTN_LOCK)) {
					uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, ((uint8*)&g_PumpCtr[u8MchNo].uPnlCtrBtnEcho)+((uRegAdd-PUMP_BTN_ECHO)*2), 2, 2);
#endif
				} else if((uRegAdd >= TRIPHA_MTR_REPUB_REG_UA) && (uRegAdd <= TRIPHA_MTR_REPUB_REG_FRQ)) {
					float32 fValue;
					if(GetValFromTriPhasMtr_RepubAddr(u8MchNo, uRegAdd, &fValue)) {			/* 读取电压\电流\功率\频率\因数测量值 */
						*(uint32*)pTxBuf = __rev(*((uint32*)&fValue));
						pTxBuf += 4;
						uProcRegNum = 2;
						bFindDataInRam = TRUE;
					} else {
						return RESPOND_OTHERS_MODBUS_RES_FAIL;
					}
				} else if((uRegAdd >= TRIPHA_MTR_REPUB_REG_KWH) && (uRegAdd <= TRIPHA_MTR_REPUB_REG_RVAR)) {
					uint32 u32Value;
					if(GetWattFromTriPhasMtr_RepubAddr(u8MchNo, uRegAdd, &u32Value)) {			/* 读取电度值 */
						*(uint32*)pTxBuf = __rev(u32Value);
						pTxBuf += 4;
						uProcRegNum = 2;
						bFindDataInRam = TRUE;
					} else {
						return RESPOND_OTHERS_MODBUS_RES_FAIL;
					}
				} else if(bFindDataInRam) {
					*pTxBuf++ = 0;
					*pTxBuf++ = 0;
					uProcRegNum = 1;
				} else {
					return RESPOND_OTHERS_MODBUS_RES_FAIL;
				}
				uRegAdd += uProcRegNum;
				iRegLen -= uProcRegNum;
			}
		} else if((0x06 == u8ModCmd) || (0x10 == u8ModCmd)) {		/* 06 10 指令 */
			uint8 u8DevNo = u8DevAdd / 10;
			if((uRegAdd == PUMP_CTR_BTN) && (g_PumpCtr[u8DevNo / 10].u8CtrMode > PUMP_RUN_MODE_HAND)) {
				if((iRegLen & (1<<PUMP_CTR_BTN_AUTO_BitNo)) && (g_PumpCtr[u8DevNo].u8CtrMode == PUMP_RUN_MODE_MON_LOCAL)) {
					g_PumpCtr[u8DevNo].u8CtrMode = PUMP_RUN_MODE_UNKNOWN;
				} else if(iRegLen & (1<<PUMP_CTR_BTN_START_BitNo)) {
					g_PumpCtr[u8DevNo].u8CtrMode = PUMP_RUN_MODE_MON_LOCAL;
					ProcPumpCtrKey(PUMP_CTR_BTN_START_BitNo,u8DevNo);  			/* 此处是否需要判断操作成功？	需要不成功的信息的话，应该要有ModBus回应 */
				} else if(iRegLen &(1<<PUMP_CTR_BTN_STOP_BitNo)) {					/* 停止泵，写入自动获取控制权限 */
					g_PumpCtr[u8DevNo].u8CtrMode = PUMP_RUN_MODE_MON_LOCAL;
					ProcPumpCtrKey(PUMP_CTR_BTN_STOP_BitNo,u8DevNo);
				} else if(iRegLen &(1<<PUMP_CTR_BTN_CANCEL_BitNo)) {				/* 取消 */
					ProcPumpCtrKey(PUMP_CTR_BTN_CANCEL_BitNo,u8DevNo);
				} else if(iRegLen &(1<<PUMP_CTR_BTN_OK_BitNo)) {					/* 确认 */
					ProcPumpCtrKey(PUMP_CTR_BTN_OK_BitNo,u8DevNo);
				}
			}
		} else {
			return RESPOND_OTHERS_MODBUS_RES_FAIL;		/* 仅支持03,04,06,0x10指令 */
		}
#endif
	} else {
		return RESPOND_OTHERS_MODBUS_RES_V4_NO_DEV;
	}

	pTRBufStart[2] = pTxBuf - pTRBufStart - 3;	/* 填充数据长度 */
	*ppModbusRTUBuf = pTxBuf;
	return RESPOND_OTHERS_MODBUS_RES_SUC;
}
#endif

#if(DEVICE_TYPE == V5_YYT3)
/*==========================================================================
| Description	: 串口服务器，modbusTCP转modubsRTU通讯任务
	TrySerialServer(): 尝试成为串口服务器
	RunSerialServer(): 串口服务器主机任务
| G/Out var		:
| Author		: Wang Renfei			Date	: 2015-9-27
\=========================================================================*/
void TrySerialServer(uint8 u8UartPort)
{
	OFF_INDICATOR_RS485(u8UartPort);
	OpenUartComm(u8UartPort, g_CommConf.u32RS485Baud_100bps[u8UartPort]*100, 0, OS_TICK_KHz*7);	/* 5byte时间，这个位置曾经用2Byte数据长度，但是1200bps下工作不好 (20220703丰都白水河电站需要至少7ms)*/
	g_UartComm[u8UartPort].uTimer_WatchDog_ms = 0;
	g_UartComm[u8UartPort].bUartFuncTrySuc = TRUE;
}

/* 处理所有非YKF的modbusTCP通信请求 */
void RunSerialServer(uint8 u8UartPort)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	MODBUS_TCP_COMM* pModbusTcpComm = g_ModbusTcpComm;
	pUartComm->uTmr_Run_ms = 10;

	int8 i;
	for(i = 0; i < MODBUS_TCP_COMM_MAX_NUM; i++) {
		/* 由于ModbusTCP是局域网内通讯，且是阻塞状态等待回复，因此本次通讯如果失败就立即再次尝试，且不处理YKF的读写请求 */
		while((pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP > 0) && (pModbusTcpComm->u8TRBuf[6]%10 != MODDEV_ADDi_YKF)) {
			if(pModbusTcpComm->uRxBufPt <= 6) {
				pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP = -2;
				OFF_INDICATOR_RS485(u8UartPort);
			} else {
				uint16 uReadRegLen = (uint16)pModbusTcpComm->u8TRBuf[10]*0x100 + (uint16)pModbusTcpComm->u8TRBuf[11];	/* 如果读指令，读取的寄存器长度 */
				uint16 uCopyLen = pModbusTcpComm->uRxBufPt - 6;
				memcpy(pUartComm->u8TRBuf, &pModbusTcpComm->u8TRBuf[6], uCopyLen); /* 抛弃modbusTCP信息帧前面6个byte */
				TxDataForModbusRTU(u8UartPort, &(pUartComm->u8TRBuf[uCopyLen]));
				if(ReadFromUart(u8UartPort, 100)
					&& (CheckRxBufForModbusRTUMaster(pUartComm, pModbusTcpComm->u8TRBuf[6], pModbusTcpComm->u8TRBuf[7], uReadRegLen) > 0))	/*>*/
				{
					ON_INDICATOR_RS485(u8UartPort);
					pUartComm->uTimer_WatchDog_ms = 0; 		/* 通讯成功 */
					pUartComm->bUartFuncTrySuc = TRUE;

					/* 数据拷贝 */
					Swi_disable();	/* 防止运行中i8RandAcs_Idle0_SucN1_FailN2_MaxTryCntP被更改 */
					if(pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP > 0) {
						uCopyLen = pUartComm->uRxBufPt - 2;		/* 不拷贝最后的CRC部分 */
						pModbusTcpComm->u8TRBuf[5] = uCopyLen;		/* 数据长度 */
						memcpy(&pModbusTcpComm->u8TRBuf[6], pUartComm->u8TRBuf, uCopyLen);
						pModbusTcpComm->uTxBufPt = uCopyLen + 6;	/* uTRBufPt非0代表数据通讯成功 */
						pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP = -1;
					}
					Swi_enable();
				} else {
					OFF_INDICATOR_RS485(u8UartPort);
					Swi_disable();
					if(pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP == 1) {
						pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP = -2;
					} else if(pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP > 1) {
						pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP--;
					}
					Swi_enable();
				}
			}
			Task_sleep(OS_TICK_KHz*20);		/* TODO 104调试好关闭这个，单纯组态去掉这行没问题  */
		}
		pModbusTcpComm++;
	}
	Task_sleep(OS_TICK_KHz*pUartComm->uTmr_Run_ms);
}
#endif

/*==========================================================================
| Description	: 软件通过RS485口升级: 发布端是从机，订阅端是主机
| G/Out var		: 
| Author		: Wang Renfei			Date	: 2017-08-28
\=========================================================================*/
void TrySubSoftUpdate(uint8 u8UartPort)
{
	uint16 uDeviceType, uSoftVersion;
	
	OpenUartComm(u8UartPort, 115200UL, 0, 0);
	if((RdDataFrModbusRTU(u8UartPort, 10, 0xFF, MODADD_DEVICE_TYPE, 1, &uDeviceType, MOD_RD_04W) > 0)
	    && (uDeviceType == DEVICE_TYPE)
		&& (RdDataFrModbusRTU(u8UartPort, 10, 0xFF, MODADD_DEVICE_SOFT_VER, 1, &uSoftVersion, MOD_RD_04W) > 0))
	{
		IniSoftUpdate(uSoftVersion);
	} else {
		g_UartComm[u8UartPort].uTimer_WatchDog_ms = 0xFFFF;
		g_UartComm[u8UartPort].bUartFuncTrySuc = FALSE;
	}
}

void RunSubSoftUpdate(uint8 u8UartPort)
{
	FLASH_REQ FlashReq;
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];

	/* 在写 FlashReq 的回复中传递 FlashPack */
	CreateFlashReq(&FlashReq, RS485_TR_FLASH_PACK_BLEN);
	if(WrDataToModbusRTU(u8UartPort, 10, 0xFF, MODADD_UPDATE_START, sizeof(FLASH_REQ)/2, (uint16*)(&FlashReq), MOD_WR_10B) > 0) {
		if(!UpdateSoft((uint32*)&pUartComm->u8TRBuf[8], pUartComm->uRxBufPt - 10)) {	
			pUartComm->UartApp = RS485_MCGS_RTU_SLAVE;
		}
	}
}

/******************************** FILE END ********************************/
