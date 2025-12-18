/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved
 *				: 
 * File			: stencil.h
 * Author		: Wang Renfei
 * Description	: head file's stencil, V1.2 inherit from V1.1
 *				: 模块化的软件结构，每一个模块在头文件中定义外部接口变量,外部驱动函数,模块调用方法函数，在源文件中定义模块内部变量、函数
 *				: 
 * Comments		:
 * Date			: 2009-5-31
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 * V0.1	| 2023-12-8   |	king | 1. Create
 **************************************************************************/

/***************************************************************************
 				exclude redefinition and c++ environment
***************************************************************************/
#ifndef _MDL_YKC_H_
#define _MDL_YKC_H_

#ifdef __cplusplus
extern "C" {
#endif


/***************************************************************************
 							include files
***************************************************************************/
#include "LibMath.h"
#include "GlobalVar.h"

/***************************************************************************
				hardware parameter and global definitions
***************************************************************************/
#define OS_TICK_KHz				1				/* 操作系统tick频率 */
#define PERIOD_CTR_TASK_ms		10

#define MAX_INTERNET_ACCESS_NUM	1
#define SUPPORT_GPRS			TRUE
#define SUPPORT_ETHERNET		FALSE
#define SUPPORT_MODBUS_TCP		FALSE
#define SUPPORT_SCREEN			FALSE

#define MAX_MSG_TYPE			0x0060			    /* 该项非零，代表需要MSG模块 */

/***************************************************************************
 		use EXT to distinguish stencil.c or others source files
***************************************************************************/
#ifdef _MDL_YKC_C_
#define EXT
#else
#define EXT extern
#endif

/***************************************************************************
 					variables could be used as extern
***************************************************************************/
/*===========================================================================
 * 控制系统定义
 *==========================================================================*/
#define MOTOR_HISTORY_CUR_MAX_CNT		15		/* 履带电机历史数据数量 */
typedef struct {								/* modbus通讯地址: 9200，注意地址步进是2 */
	float32 fCPUTemperature;					/* CPU温度，单位:摄氏度 */
	float32 fVrefint;			/* 内部参考电源电压，应该1.2V左右 */
	float32 fDCPwrVol;			/* 电源电压 */
	float32 fRightCur; 			/* 右侧电机(履带)电流 */
	float32 fLeftCur;			/* 左侧电机(履带)电流 */
	float32 fLastRightCur;		/* 上次右侧电机(履带)电流 */
	float32 fLastLeftCur;		/* 上次左侧电机(履带)电流 */

	F32_QUEUE_t qRightMotorCurrHistory;		/* 右履带电流历史数据 */
	F32_QUEUE_t qLeftMotorCurrHistory;		/* 左履带电流历史数据 */

	float32 fRightMaxI;
	float32 fLeftMaxI;

	/* 气压计 */
	float32 fAirPressure;		/* 真空度 */
	float32 fAirTemp;			/* 气压计温度 */
	uint8 u8TryCnt_AirSensor;	/* 气压计传感器通讯指示 */


	/* IMU数据 */
	uint8 u8TryCnt_IMU;
	uint16 fGyrZ;				/* Z轴方向的角速度 */
	float32 fPitch;				/* ° */
	float32 fPitchDelta_rad;	/* Pitch的变化量，rad */
	float32 fRoll;				/* ° */
	float32 fYaw;

	/* 转速计 */
	float32 fPumpFreq;
	float32 fRightFreq;		/* 前进为正频率，后退为负频率 */
	float32 fLeftFreq;		/* 前进为正频率，后退为负频率 */
	uint16 uRightCount;
	uint16 uLeftCount;
}MSR_RES;
EXT SECTION(".NOT_ZeroInit") MSR_RES g_MsrRes;

/* 调试相关 */
typedef struct {
	float32 fAimed_Air_P;			/* 期望抽的压差 */
	uint32 u8GprsDebugSwitch;		/* GPRS串口调试信息开关 */
	uint32 u8CliffSwitch;			/* 是否启用悬崖开关 */
	uint32 u8GyroSwitch;			/* 陀螺仪中断判断堵转开关 */
	float32 fFilterFactor;			/* 滤波因子 */
}DebugConf;
EXT SECTION(".NOT_ZeroInit") DebugConf g_DebugConf;

/*===========================================================================
 * 红外遥控器
 *==========================================================================*/
#define IR_BTN_NONE		0
#define	IR_BTN_0		152
#define IR_BTN_1		162
#define	IR_BTN_2		98
#define	IR_BTN_3		226
#define	IR_BTN_4		34
#define	IR_BTN_5		2
#define	IR_BTN_6		194
#define	IR_BTN_7		224
#define	IR_BTN_8		168
#define	IR_BTN_9		144
#define	IR_BTN_STAR		104		/* 星号键 */
#define	IR_BTN_SHARP	176		/* #号键 */
#define	IR_BTN_UP		24
#define	IR_BTN_DOWN		74
#define	IR_BTN_LEFT		16
#define	IR_BTN_RIGHT	90
#define	IR_BTN_OK		56
#define IR_RX_SEQ_BLEN	33

typedef struct {
	uint8 u8BtnPressing;							/* 当前按下的按键 */
    uint8 u8TryCnt;									/* 按键有效性 */
    uint8 u8SelectedrIndex;							/* 当前指定的红外标志 */
    uint8 u8Rsvd;									/* 占位符 */
    struct {
    	BOOL bBtnRdyDecode;							/* 有待解码的数据帧 */
    	uint8 u8TimerUpdatCount;					/* timer溢出次数 */
    	uint16 uCanlCnt;							/* 通道有效性 */

    	int32 i32Wait;
    	uint8 u8CapPol;								/* 捕获极性 */
    	uint8 u8CapPulseCount;						/* 捕获脉冲次数 */
    	uint16 uRepeatCount;						/* 连续重复码次数(按住不松手) */
    	uint8 bIsIdle;								/* 空闲状态 */
        uint8 u8Rsvd2;								/* 占位符2 */

    	uint16 rxFrame[IR_RX_SEQ_BLEN * 2];			/* 数据帧（驱动侧） */
    	uint16 srcData[IR_RX_SEQ_BLEN * 2];			/* 数据帧（应用侧） */


    	union{
    	    uint32 u32AllData;
    	    struct {
    	        BITBOOL key_val_n:8;
    	        BITBOOL key_val  :8;
    	        BITBOOL addr_n   :8;
    	        BITBOOL addr     :8;
    	    } bit;
    	} data;
    } DevComm[2];
}IR_CTRL;

EXT IR_CTRL g_IRCtrl;		/* 红外遥控器主全局变量 */

/* 语音类型 */
typedef enum {
	SILENCE			= 0,
	VOICE_WELCOM,							/* 欢迎使用远索擦窗机器人 */
	VOICE_AIR_PRES_LOW,						/* 气压过低 */
	VOICE_AIR_PRES_ABN,	 					/* 气压异常 */
	VOICE_IMU_ABN, 							/* 陀螺仪异常 */
	VOICE_LIQUID_LOW, 						/* 水箱水位低 */
	VOICE_STARTCLEAN, 						/* 清扫开始 */
	VOICE_FINISHCLEAN, 						/* 清扫完毕 */
	VOICE_AC_UNPLUGGED,						/* 请接上交流电源 */
	VOICE_CLIFF_DETECTED,					/* 有悬崖开关处于悬空状态 */
	VOICE_VOLTAGE_ERROR, 					/* 电压异常 */
	VOICE_CPU_TEMPERATURE_ERROR, 			/* CPU温度异常 */
	VOICE_BYEBYE, 							/* 成功退出 */
	VOICE_MAX_NO,							/* 歌曲最大编号 */
#if (!SOFT_RUN1_TEST0)
	VOICE_HIT_SIDE_L_FRONT_DINNO, 			/* 左上检测到碰撞 */
	VOICE_HIT_SIDE_L_REAR_DINNO, 			/* 左下检测到碰撞 */
	VOICE_HIT_SIDE_R_FRONT_DINNO, 			/* 右上检测到碰撞 */
	VOICE_HIT_SIDE_R_REAR_DINNO, 			/* 右下检测到碰撞 */
	VOICE_HIT_FRONT_LEFT_DINNO, 			/* 前左检测到碰撞 */
	VOICE_HIT_FRONT_RIGHT_DINNO, 			/* 前右检测到碰撞 */
	VOICE_HIT_REAR_LEFT_DINNO, 				/* 左侧检测到碰撞 */
	VOICE_HIT_REAR_RIGHT_DINNO, 			/* 右侧检测到碰撞 */
	VOICE_CLIFF_FRONT_LEFT_DINNO, 			/* 悬崖左上检测到碰撞 */
	VOICE_CLIFF_FRONT_RIGHT_DINNO, 			/* 悬崖右上检测到碰撞 */
	VOICE_CLIFF_REAR_LEFT_DINNO, 			/* 悬崖左下检测到碰撞 */
	VOICE_CLIFF_REAR_RIGHT_DINNO 			/* 悬崖右下检测到碰撞 */
#endif
}MP3_DIALOG_TYPE;

typedef enum {
	MP3_CMD_QUERY_STATE = 0x01,			/* 查询播放状态(0~2: 停止/播放/暂停) */
	MP3_CMD_PLAY = 0x02,				/* 播放 */
	MP3_CMD_PAUSE = 0x03,				/* 暂停 */
	MP3_CMD_STOP = 0x04,				/* 停止 */
	MP3_CMD_SELECT_SONG_PLAY = 0x07,	/* 选择曲目播放 */
	MP3_CMD_QUERY_DRIVE = 0x09,			/* 查看当前盘符 */
	MP3_CMD_QUERY_SONGS_NUMBER = 0x0C,	/* 查询总曲目数 */
	MP3_CMD_SET_VOLUME = 0x13,			/* 设置音量(0~30) */
	MP3_CMD_VOLUME_UP = 0x14,			/* 音量加 */
	MP3_CMD_VOLUME_DOWN = 0x15,			/* 音量减 */
	MP3_CMD_SELECT_SONG = 0x1F,			/* 选曲不播放 */
	MP3_CMD_GET_RUNING_TIME = 0x24,		/* 获取当前曲目总时间 */
}MP3_CMD;	/* MP3模块指令 */

EXT const char* cnst_TtsDialog[]
#ifndef _MDL_YKC_C_
;
#else
= {
		"[v16][m0][t5]",
		"[v16][m0][t5]欢迎使用远索擦窗机器人",
		"[v16][m0][t5]气压过低",
		"[v16][m0][t5]气压异常",
		"[v16][m0][t5]陀螺仪异常",
		"[v16][m0][t5]水箱水位低",
		"[v16][m0][t5]清扫开始",
		"[v16][m0][t5]清扫完毕",
		"[v16][m0][t5]请接上交流电源",
		"[v16][m0][t5]请避免四角悬空",
		"[v16][m0][t5]内部采样参考电压异常",
		"[v16][m0][t5]CPU温度异常",
		"[v16][m0][t5]成功退出",
		"[v16][m0][t5]左上检测到碰撞",
		"[v16][m0][t5]左下检测到碰撞",
		"[v16][m0][t5]右上检测到碰撞",
		"[v16][m0][t5]右下检测到碰撞",
		"[v16][m0][t5]前左检测到碰撞",
		"[v16][m0][t5]前右检测到碰撞",
		"[v16][m0][t5]左侧检测到碰撞",
		"[v16][m0][t5]右侧检测到碰撞",
		"[v16][m0][t5]悬崖左上检测到碰撞",
		"[v16][m0][t5]悬崖右上检测到碰撞",
		"[v16][m0][t5]悬崖左下检测到碰撞",
		"[v16][m0][t5]悬崖右下检测到碰撞"
};
#endif

//EXT BOOL InitTtsSpeaking();
//EXT BOOL TtsSpeak(TTS_DIALOG_TYPE DlgType, BOOL bIgnoreDup);		/* 如果成功加入播放队列则返回TRUE，如果队列已满则返回FALSE */


/* 异常序号，异常存放在 g_AnaRes.u32Abnormal */
#define ABN_TIME_INVERSE_START_No       0  	/* 以下是反时限异常 */
#define ABN_T_INV_MOTOR_RIGHT_No		0	/* 机组A相电流反时限, YKF2/3/YYD/YYG支持, 动作:跳闸关机 */
#define ABN_T_INV_MOTOR_LEFT_No			1	/* 机组B相电流反时限, YKF2/3/YYD/YYG支持, 动作:跳闸关机 */
#define TOTAL_ABN_NUM                   2	/* 总异常数量 */
#define ABNORMAL_BUF_b32LEN             ((TOTAL_ABN_NUM + 31)/32)

/* 有控制意义的单元, 提供给控制模块，或者工程人员调试时使用 */
typedef struct {							/* modbus通讯地址: 9600，注意地址步进是2 */
	float32 fLeftSpeed;
	float32 fRightSpeed;
	uint32 u32Abnormal[ABNORMAL_BUF_b32LEN];
}ANA_RES_VAR;
EXT SECTION(".NOT_ZeroInit") ANA_RES_VAR g_AnaRes;

/* 统计变量，必须先放置STAT_CUM_ITEM */
typedef struct {
}STAT_CUM_DATA;
EXT SECTION(".NOT_ZeroInit") STAT_CUM_DATA g_StatCum;

typedef struct {
}STAT_COUNT_DATA;
EXT SECTION(".NOT_ZeroInit") STAT_COUNT_DATA g_StatCount;

#define COMM_DEV_STATUS_VAR	uint32
/*===========================================================================
*  远程与现场控制接口
*==========================================================================*/
#define CTR_BTN_IncRelay_No         0           /* 增励 */
#define CTR_BTN_DecRelay_No         1           /* 减励 */
#define CTR_BTN_TOTAL_NUM       2           /* 总的控制按钮数量 */

/*===========================================================================
*  6. g_Ctr: 主控制器状态
*==========================================================================*/
typedef enum {				/* 运行模式 */
	RUN_MODE_ABN_OCCUR						/* 异常动作,并不是一种运行模式, 用于产生关机消息，仅是编程方便 */
}RUN_MODE_STATE;

/*===========================================================================
 * MSG部分
 *==========================================================================*/
typedef enum {
	MSG_NULL						= 0x0000,		/* 空 */
	MSG_HuanYinShiYong				= 0x0001,		/* 欢迎使用--这条消息仅用于开机显示，不应该用AddMsg添加(存储) */
	
	/* 运行异常消息起始 */
	MSG_MqttDisc    				= 0x0050,		/* MQTT服务器连接断线 */
	MSG_LicenseTimeOut				= 0x0051,		/* 控制器许可证到期,请联系我司 */
	MSG_LicenseRenewAlert			= 0x0052,		/* 控制器试用即将到期,还剩天,请及时缴费 */
    MSG_SoftIntegrityAbn            = 0x0053,       /* 控制器软件异常,请联系我司,异常代码: */
    MSG_SoftUpgradeFail             = 0x0054,       /* 软件升级失败,故障代码: */
    MSG_Rsvd55                      = 0x0055,       /* 备用 */
    MSG_Rsvd56                      = 0x0056,       /* 备用 */
    MSG_Rsvd57                      = 0x0057,       /* 备用 */
	MSG_Rsvd58				        = 0x0058,		/* 备用 */
	MSG_Rsvd59			            = 0x0059,		/* 备用 */
	MSG_Rsvd5A 		                = 0x005A,		/* 备用 */
	MSG_Rsvd5B 		                = 0x005B,		/* 备用 */
	MSG_Rsvd5C						= 0x005C,		/* 备用 */
	MSG_Rsvd5D						= 0x005D,		/* 备用 */
    MSG_Rsvd5E                      = 0x005E,       /* 备用 */
    MSG_Rsvd5F                      = 0x005F,       /* 备用 */

}MSG_ID;

EXT const MSG_PROP cnst_MsgProp[MAX_MSG_TYPE]	/* 为了便于识别,有附加值的消息正文部分不可以带数字 */
#ifndef _MDL_YKC_C_
;
#else
= {	{-1, F32_DEFAULT,	{"", ""}},
	{-1, F32_DEFAULT,	{"欢迎使用YKC", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},

	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},

	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},

	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},

	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	
	{-1, F32_DEFAULT,	{"连接服务器断线", "Reserved"}},
	{-1, F32_DEFAULT,	{"试用到期,请联系我司www.yoplore.com", "Reserved"}},
	{9,  F32_INT,		{"试用即将到期,还剩天,请及时缴费", "Reserved"}},
	{19, F32_INT,	    {"控制器软件异常,请联系我司,异常代码:", "Reserved"}},
	{12, F32_INT,		{"软件升级失败,故障代码:", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}},
	{-1, F32_DEFAULT,	{"备用", "Reserved"}}
};
#endif

/*===========================================================================
 * 配置变量
 *==========================================================================*/
/* 通讯配置数据, 数值为0说明未被初始化 */
typedef struct {
	/* 以下是所有产品必须 */
	uint32 u32MqttPubIntv_ms;					/* Mqtt发布间隔 */
	uint32 u32GuiIdleTimeOut;					/* 主屏GUI跳转默认页超时(s) */

}COMM_CONF;
EXT SECTION(".NOT_ZeroInit") COMM_CONF g_CommConf;

#if CPU_0ST_1TI
/*---------------------------------------------------------------------------
 * 采集与消息eeprom存储地址
 *--------------------------------------------------------------------------*/
/* EEProm地址分配, 以64byte为单元，总共96片
	0		: BootLoader
	1~43	: 配置存储部分
	44~95	: Acq, Msg部分 */
#define EEPROM_ADDR_MSG				((96-20)*64)			/* 64*5*32bit, Msg每条占用20byte */
#define EEPROM_ADDR_ACQ				((96-20-32)*64)			/* 64*8*32bit, Acq每条占用32byte */
#define EEPROM_MSG_ITEM_NUM			64						/* 根据前面分配的空间，最多存储64条消息 */
#define EEPROM_ACQ_ITEM_NUM			64						/* 根据前面分配的空间，最多存储64条Acq */
#endif
/***************************************************************************
 					functions must be driven by extern
***************************************************************************/
EXT void InitMdlCtr(void);
EXT void RunMdlCtr(void);
EXT void DrvMdlCtrTick_1KHz(void);

/***************************************************************************
 					functions could be used as extern
***************************************************************************/

/************   exclude redefinition and c++ environment   ****************/
#ifdef __cplusplus
}
#endif 					/* extern "C" */

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
/******************************** FILE END ********************************/

