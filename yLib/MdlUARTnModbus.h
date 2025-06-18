/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved.	模板V1.3
 *				: 
 * File			: stencil.h
 * Author		: Wang Renfei
 * Description	: 
 * Date			: 2009-5-31
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 **************************************************************************/

/***************************************************************************
 				exclude redefinition and c++ environment
***************************************************************************/
#ifndef _MDL_RS485_n_MODBUS_H_
#define _MDL_RS485_n_MODBUS_H_

/***************************************************************************
 							include files
***************************************************************************/
#include "BoardSupport.h"

#include "GlobalVar.h"
#include "MdlDataAccess.h"
#include "MdlGUI.h"


/***************************************************************************
				hardware parameter and global definitions
***************************************************************************/
/* <NULL> */


/***************************************************************************
 		use EXT to distinguish stencil.c or others source files
***************************************************************************/
#ifdef _MDL_RS485_n_MODBUS_C_
#define EXT
#else
#define EXT extern
#endif


/***************************************************************************
 					variables could be used as extern
***************************************************************************/
/*===========================================================================
 * RS485通讯部分
 *==========================================================================*/
typedef enum {
	RS485_APP_NULL 			= 0,			/* 无 */
	RS485_APP_AUTO			= 1,			/* 控制器自动按照每一种APP功能进行尝试，如果尝试成功则保持这种功能方式 */
	UART_VIEW_TECH			= 2,			/* ViewTech屏幕驱动 */
	RS485_MCGS_RTU_SLAVE	= 3,			/* MCGS屏幕接口(modbus从机，运行版38.4kbps，测试版115.2kbps) */
	RS485_YKFR1_XMS_9600	= 4,			/* 用于连接YKF控制器R1, XMG, XMS, ELE(直供电), RCX(线路与变压器保护) */
	RS485_YKFR2_38400		= 5,			/* 用于连接YKF控制器R2 */
	RS485_SOFT_UPDATE		= 6,			/* 用于从运行版换回测试版，不必远程升级，通过RS485线对拷即可 */
	RS485_V5YKF_PERIPHERAL	= 7,			/* YKF23外设：(温度采集模块\压力传感器)，通讯波特率9600 */

	RS485_MODBUS_RTU_SLAVE	= 21,			/* 标准ModbusRTU从机波特率、地址待定, 从机排在最后 */
	RS485_DATA_RADIO_SUB	= 22,			/* 数传电台站房端 */
	RS485_IEC101			= 23,			/* IEC101规约 */
	RS485_FLOW_METER		= 24,			/* 流量计接口 */
	RS485_BEIDOU_TERM		= 25,			/* 北斗终端 */
	RS485_BEIDOU_HUB		= 26,			/* 北斗Hub */
	RS485_DEBUG_PACK		= 27,			/* 传输DebugPack数据 */
	RS485_TRIPHASE_METER	= 28,			/* 智能三相电能表 */
	/* 串口服务器，用于modbusTCP转modbusRTU；从设备地址不得用: 2(水位仪表地址),10*N(YKF地址),10*N+3(温度巡检仪地址),10*N+7(变压器保护装置地址) */
	RS485_SERIAL_SERVER		= 29,
	RS485_APP_GNLMODBUS_MASTER = 30,		/* 通用MODBUS数据源设备(不适合放入自动) */
	RS485_WATERLV_PGA460 	= 32,			/* PGA460超声波水位计 */
	RS485_EXT_DIN_RELAY		= 33,			/* 扩展开入开出模块*/
	RS485_AIR_COMPRESSOR	= 34,			/* 空压机 */
	UART_GPS				= 35,			/* GPS */
	UART_WATERLVL_OPT_TX	= 36,			/* 光纤水位发射*/
	UART_WATERLVL_OPT_RX	= 37,			/* 光纤水位接收*/
	UART_APP_MAX
}UART_APP;

#define UART_TR_BUF_BLEN	(255 + 9)		/* 最长的是0x10指令 头(7byte)+数据(255byte)+CRC(2byte) */
typedef struct {
    UART_Handle Handle;
    void* Sem_TxCplt;
    void* Sem_RxGet;
    
	UART_APP UartApp;						/* RS485应用 */
	UART_APP LastUartApp;					/* 上一次打开RS485时的应用 */
	BOOL bUartFuncTrySuc;					/* 尝试成功 */
	uint8 u8RS485AppAutoTablePt;			/* RS485自动应用在 cnst_RS485AppAutoTable[] 搜索指针 */
	uint16 uLastBaud_100bps;				/* 上一次打开RS485时的通讯波特率 */
	uint16 uTROneByteTime_us;				/* 发送一个字节所需的时间,用于计算uMaxTransferTmr_ms */

	/* 通讯管理 */
	uint16 uTimer_WatchDog_ms;				/* 通讯看门狗定时器，如果是主机则无从机响应；如果是从机则无CRC正确的主机指令(不一定发往本机) */
	uint16 uTmr_Run_ms;						/* 单次通讯任务定时器，使得通讯任务保持固定的频率 */
    uint16 uTime_RxMaxIntv_ms;				/* 帧内字节间最大间隔时间 */
	uint16 uRxBufPt;
	uint16 uRxFrameIndex;						/* 当一帧数据没接收完整时, DMA再次接收数据时会从这个索引处继续接受, 所以用完应该立马置0 */

	/* 通讯Buf部分:32b对齐 */
	uint8 u8TRBuf[UART_TR_BUF_BLEN];

	/* ItemPage请求，兼容mcgs,viewtech 用于处理conf,msg,acq */
	ITEM_PAGE_REQ ItemPageReq;
}UART_COMM;
EXT SECTION(".NOT_ZeroInit") UART_COMM g_UartComm[MAX_UART_NUM];

typedef enum {
	MOD_WR_05B,			/* 05指令写  写单个线圈值 */
	MOD_WR_06W,			/* 06指令写  写单个寄存器值,16bit宽度,以2-1顺序 */
	MOD_WR_0FB,			/* 0F指令写  写多个线圈值 */
	MOD_WR_10B,			/* 10指令写  写多个寄存器值,不关注字节顺序，公司内部成对使用 */
	MOD_WR_10W,			/* 10指令写  写多个寄存器值,16bit宽度,以2-1顺序 */
	MOD_WR_10D,			/* 10指令写  写多个寄存器值,以4-3-2-1顺序, 即完全逆序 */
	MOD_WR_10D2143,		/* 10指令写  写多个寄存器值,TODO:以2-1-4-3顺序,暂时不支持 */

	MOD_RD_01B, 		/* 01指令读, 1bit宽度 */
	MOD_RD_02B,			/* 02指令读, 1bit宽度 */
	MOD_RD_03B, 		/* 03指令读, 不考虑字节顺序 */
	MOD_RD_03W,			/* 03指令读, 16bit宽度,以2-1顺序 */
	MOD_RD_03D,			/* 03指令读, 32bit宽度,以4-3-2-1顺序, 即完全逆序 */
	MOD_RD_03L, 		/* 03指令读, 64bit宽度,以8-7-6-5-4-3-2-1顺序, 即完全逆序 */
	MOD_RD_03D2143, 	/* 03指令读, 32bit宽度,以2-1-4-3顺序 */
	MOD_RD_04B,			/* 04指令读, 不考虑字节顺序 */
	MOD_RD_04W,			/* 04指令读, 16bit宽度,以2-1顺序 */
	MOD_RD_04D,			/* 04指令读, 32bit宽度,以4-3-2-1顺序, 即完全逆序 */
	MOD_RD_04D2143,		/* 04指令读, 32bit宽度,以2-1-4-3顺序 */
	MOD_RD_04D_I2F,		/* 04指令读, 32bit宽度,以4-3-2-1顺序, 即完全逆序; 读取的数据是I32,转成F32写入Buf */
	MOD_RD_04L 			/* 04指令读, 64bit宽度,以8-7-6-5-4-3-2-1顺序, 即完全逆序 */
}MOD_ACS_TYPE;

/* RS485及基于这之上的通讯结果代码     <=0:故障代码 >0:通讯成功 */
typedef enum {
    UART_ACS_NULL           = 0,    /* 保持在初始化状态，并没有执行相应的读写操作 */
    UART_RD_TIME_OUT        = -2,   /* 读取超时 */
    UART_NUM_NOT_ENOUGH     = -3,   /* 读取的字节数过少 */
    UART_CRC_ERR            = -4,   /* 通讯crc错误 */
    UART_MOD_DEVADD_NEQ     = -5,   /* Modbus通讯地址与发出去的地址不一致 */
    UART_MOD_CMD_NEQ        = -6,   /* Modbus通讯指令与发出去的指令不一致 */
    UART_MOD_RD_FAIL        = -7,   /* Modbus读取失败，原因未知 */
	UART_MOD_WR_FAIL        = -8,   /* Modbus写入失败，原因未知 */
    UART_ACS_SUC            = 1     /* 通讯成功 */
}UART_ACS_RES;

/* modbus总线设备，分为两类：
	a. 仅一台设备，地址宏定义为 MODDEV_ADD_***
	b. 多台设备，地址为    10*N + ADDi，其中ADDi为尾数部分 */
#define MODDEV_ADD_XMG			2
#define MODDEV_ADD_YYT			2
#define MODDEV_ADD_ELE			3
#define	MODDEV_ADD_RAIN			5		/* 雨量计 */
#define MODDEV_ADDi_YKF			0
#define MODDEV_ADDi_XMS			3
#define	MODDEV_ADDi_GOV			6		/* 调速器 */
#define	MODDEV_ADDi_YYB			7
#define	MODDEV_ADDi_DLT			8		/* DLT645电能表 */
#define	MODDEV_ADDi_TPM			9		/* 多功能三相电表 */

/*===========================================================================
 * ModbusTCP通讯部分--为了保证通过ModbusRTU和ModbusTCP拥有完全一样的数据访问特性，采取共用RespondModbus()等特性
 * 不管原生网口还是扩展网口，每一个连接都需要一个连接任务块
 * 0 ~ (MODBUS_TCP_INT_TASK_NUM-1) 提供给原生网口， MODBUS_TCK_INT_TASK_NUM之外提供给扩展网口
 *==========================================================================*/
#if SUPPORT_MODBUS_TCP
#if (DEVICE_TYPE == V5_YYT3)
#define MODBUS_TCP_INT_TASK_NUM			8		/* 基于原生网口的ModbusTCP连接数量 */
#else
#define MODBUS_TCP_INT_TASK_NUM			4		/* 基于原生网口的ModbusTCP连接数量 */
#endif
#if (DEVICE_TYPE == V5_YBT3)
	#define MODBUS_TCP_EXT_COMM_NUM		4		/* 基于扩展网口的ModbusTCP连接数量 */
#else
	#define MODBUS_TCP_EXT_COMM_NUM 	0
#endif
#define MODBUS_TCP_COMM_MAX_NUM 	(MODBUS_TCP_INT_TASK_NUM + MODBUS_TCP_EXT_COMM_NUM)

typedef struct {
	/* 通讯管理 */
	BOOL bFree;									/* 仅用于原生网口，代表该任务块以及对应的任务堆栈未被使用 */
	int8 i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP;/* 通过RS485口进行modbusRTU随机访问   --0:空闲;-1:成功;-2:失败;>0:最大努力次数 */
	uint16 uRxBufPt;
	uint16 uTxBufPt;
	
	/* 通讯Buf部分 */
	uint8 u8TRBuf[UART_TR_BUF_BLEN];
	osStaticThreadDef_t ModbusTCPSessionStackCtlBlk;	/* 静态创建 */
	uint32 ModbusTcpSessionStack[MODBUS_TCP_TASK_STACK/4];
}MODBUS_TCP_COMM;
EXT SECTION(".NOT_ZeroInit") MODBUS_TCP_COMM g_ModbusTcpComm[MODBUS_TCP_COMM_MAX_NUM];
#endif
/*===========================================================================
 * 通讯确认
 *==========================================================================*/
#define KEY_VALID_TIME_ms 		250		        /* GUI按键有效时间（毫秒） */
/*  为了统一 VT屏幕,modbusRTU(MCGS),modbusTCP(组态),web前端(mqtt/json)访问，控制按钮(CtrBtn)又可以分为：
    	B1. 所有接口都可以访问的按钮，主要是modbusTCP(组态)
    	B2. 仅现场操作屏幕(VT屏幕,MCGS/485/modbusRTU)可以访问的按钮
    为了支持多源写入，每个按钮单独设置一个定时器:     写入该按钮的时候，定时器保持250ms
    具体按钮行为，由各个产品自行定义；即使是通用按钮，不同产品对按钮产生的效果也可以不一样。

    如需要使用控制按钮，需要在产品头文件中定义，如下两个变量:
    #define CTR_BTN_MODTCP_NUM     4      //modbusTcp可以访问的按钮数量
    #define CTR_BTN_TOTAL_NUM      4      //总的控制按钮数量
*/ 

/* modbus通讯状态 */
typedef struct {
	COMM_DEV_STATUS_VAR CommDevStatus;			/* 总线设备通讯情况，不必考虑重入性，偶尔错了也无所谓 */

    /* 面板按钮 */
#ifdef CTR_BTN_TOTAL_NUM
    uint8 u8Tmr_PnlCtrBtn_Valid_ms[CTR_BTN_TOTAL_NUM];  /* 面板按钮有效性标志，写入面板按钮，就开始相应的倒计时 */
#endif
    uint16 uPnlCtrBtnEcho;                      /* 输入,面板按钮响应 */
    uint16 uPnlCtrBtnLock;                      /* 输入,面板按钮锁,即要产生"不可按"的效果 */

    ITEM_PAGE_REQ ItemPageReq;                  /* ModbusTCP访问ItemPage所用，所有ModbusTCP共用一个 */
}MISC_COMM_DATA;
EXT MISC_COMM_DATA g_MiscCommData;

/*===========================================================================
 * Input Var
 *==========================================================================*/
/* <NULL> */

/*===========================================================================
 * Output Var
 *==========================================================================*/
/* <NULL> */

/*===========================================================================
 * Conf Var
 *==========================================================================*/
/*---------------------------------------------------------------------------
 * 配置变量
 *--------------------------------------------------------------------------*/
/* <NULL> */

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
EXT void InitUartComm(void);
EXT void UartCommTask(void const * argument);
EXT void DrvUartTick_1KHz(void);

/***************************************************************************
 					functions could be used as extern
***************************************************************************/
/* Modbus通讯相关函数 */
EXT void TxDataForModbusRTU(uint8 u8UartPort, uint8* pTxBuf);
EXT UART_ACS_RES CheckCrcForModbusRTU(uint8* pRxBufStart, uint16 uRxNum);
EXT UART_ACS_RES CheckRxBufForModbusRTUMaster(UART_COMM* pUartComm, uint8 u8DevAdd, uint8 u8ModCmd, uint16 uReaddRegLen);
EXT UART_ACS_RES RdDataFrModbusRTU(uint8 u8UartPort, uint16 uMaxDelay_ms, uint8 u8DevAdd, uint16 uRegAdd, uint16 uRegLen, uint16* pU16Dat, MOD_ACS_TYPE ModAcsType);
EXT UART_ACS_RES WrDataToModbusRTU(uint8 u8UartPort, uint16 uMaxDelay_ms, uint8 u8DevAdd, uint16 uRegAdd, uint16 uRegLen, uint16* pU16Dat, MOD_ACS_TYPE ModAcsType);
EXT UART_ACS_RES WaitDataFromModbusRTU(uint8 u8UartPort, uint16 uMaxDelay_ms);
/* 本机数据modubs响应函数 uModbusPort < RS485端口数(MAX_UART_NUM)，则认为是ModbusRTU，否则认为是ModbusTCP */
EXT void RespondModbus(uint8 u8ModbusPort, uint8 u8DevAdd);
EXT uint16 WriteMosbusMon(uint16 uRegAddr, uint8* pRxBuf, uint16 uRegLen);

/* can通讯所需以modbus寻址方式访问设备数据 */
EXT BOOL ReadModbus64bitsForCan(uint16 uRegAddr, uint32* pU32Dat);
EXT BOOL WriteModbusForCan(uint16 uRegAddr, uint8* pRxBuf, uint16 uRegLen);

/* 非本机数据Modbus响应函数，用于YBTn, YYTn等通讯规约 */
#if SUPPORT_MODBUS_TCP
EXT void RespondModbusTCPForOthers(MODBUS_TCP_COMM* pModbusTcpComm, BOOL bBlock);
EXT void RespondModbusRTUForOthers(uint8 u8ModbusPort);
#endif
/************   exclude redefinition and c++ environment   ****************/

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
/******************************** FILE END ********************************/
