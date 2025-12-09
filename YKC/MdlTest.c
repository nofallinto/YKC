#if !SOFT_RUN1_TEST0
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
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/* 项目公共头文件 */
#include "GlobalVar.h"
#include "MdlSys.h"
//#include "LibMath.h"

/* 本级以及下级模块头文件 */
#include "MdlNet.h"
#include "MdlDataAccess.h"
#include "BoardSupport.h"
#include "MdlUARTnModbus.h"
#include "MdlGUI.h"
#include "BMI270.h"		/* IMU */
#include "BMP280.h"		/* 气压 */

#include "MdlYKC.h"

/* 其他模块 */
/***************************************************************************
 						global variables definition
***************************************************************************/
typedef enum {
	CLN_ST_IDLE				= 0,		/* 空闲:真空泵断电 */
	CLN_ST_STOP				= 1,		/* 暂停 */
	CLN_ST_HANG				= 2,		/* 悬空:真空泵上电，但是还没有贴到窗台上 */
	CLN_ST_END				= 3,		/* 清洁完成 */
	CLN_ST_JUMP				= 4,		/* 循环跳转 */

	CLN_ST_DRIVE			= 5,		/* 直线+旋转 */
	/* 以下为单独动作 */
	CLN_ST_JTURN			= 6,		/* J弯 (直角转弯) */
	CLN_ST_CTURN			= 7,		/* S弯 (倒U型弯) */
	/* 探索边沿，边沿传感器需要被不断触发 */
	CLN_ST_EXP_FORWARD_L	= 8,		/* 沿左边前进 */
	CLN_ST_EXP_FORWARD_R	= 9,		/* 沿右边前进 */
	CLN_ST_EXP_BACK_L		= 10,		/* 沿左边后退 */
	CLN_ST_EXP_BACK_R		= 11,		/* 沿右边后退 */
	CLN_ST_EXP_UP_FORWARD_R = 12,		/* 靠左前进探索上边沿 */
	CLN_ST_EXP_UP_BACK_R	= 13,		/* 靠左后退探索上边沿 */
	CLN_ST_BLANK			= 14,		/* 空白 */

	CLN_ST_TEST
}WIN_CLN_STATE;

typedef struct {
	/* 机器人位置，以启动位置为原点 */
	int32 i32X;			/* 以0.01脉冲为单位 */
	int32 i32Y;
	float32 fAngle;		/* 机器人朝向，12点方向为0度（0°为12点，-45°为3点，+45°为9点） */

	/* 当前次运动距离 */
	int32 i32x;
	int32 i32y;

	int32 i32TripCntForSpray;			/* 一个清扫脚本状态内运行的履带行程累积（ 为计算喷水时机而统计） */
	/* 当前状态 */
	WIN_CLN_STATE WinClnST, WinClnST_beforeSTOP;
	uint8 u8ClnSeqNo, u8ClnSeqNo_last;			/*  */
	uint16 uTimer_StrStpOn_10ms;		/* 启停键按下持续时间 */
	uint16 uTimer_WinClnST;				/* 在某个状态持续的时间 */
	uint16 uTimer_DInSig_10ms[MAX_DIN_NUM];
	uint32 u32DinDat_last;

	/* 运动目标 */
	int32 i32Forward;					/* 前进距离: >0前进 <0后退 */
	float32 fAimedAngle;				/* 擦窗机器人要转到的目标角度（0°为12点，-45°为3点，+45°为9点） */

	/* 履带驱动 */
	float32 fDragConfR2L;				/* 左右轮的阻力差异 */
	float32 fVr2lAimErr_last;

	/* 控制量 */
	float32 fRightDuty;
	float32 fLeftDuty;

	/* 左右履带电机堵转识别与过电流保护 */
	/* 履带电机保护延时器，电机动作一定时间后，堵转判断才启动；前进计时器为正，后退计时器为负 */
	int8 i8Timer_RightAct;
	int8 i8Timer_LeftAct;
	BOOL bRightStall;					/* 右侧履带停滞卡住 */
	BOOL bLeftStall;					/* 右侧履带停滞卡住 */

	/* 真空泵 */
	float32 fAirPressure_PumpIdle;		/* 真空泵停时的空气压力 */
	float32 fVacuumAimErr_last;			/* 越接近0说明排风吸力越好 */
	float32 fPumpDuty;					/* 真空泵PWM占空比 */
	uint8 u8Tmr_PumpFullSpeed_10ms;		/* 判定真空泵是已有足够时间启动至符合启动要求的全速，0表示时间已足够泵到达全速 */
	uint8 u8Tmr_VacummSuff_10ms;		/* 真空泵吸力不足计时器，用于忽略瞬时的气压不足 */
	int8 i8Tmr_Spray_Front_10ms;		/* 喷清洗液(前)计时器, -1代表此计时器空闲未使用 */
	int8 i8Tmr_Spray_Back_10ms;			/* 喷清洗液(后)计时器, -1代表此计时器空闲未使用 */


	/* 电机电流 */
	float32 fRightHitCurrent;			/* 右电机碰撞时电流 正常行驶的电流就不记录了, 因为擦窗机朝向不同, 运行时电流不同. */
	float32 fLeftHitCurrent;			/* 左电机碰撞时电流 */

	float32 fGyrZAlpha;					/* Z轴方向上角速度的平滑系数 ((0.0f, 1.0f), 越小越平滑) */
	float32 fGyrZJumpThr;				/* Z轴方向上角速度跳变阈值  (越小越敏感) */
	float32 fGyrZDecayFactor;			/* Z轴方向上角速度跳变积分衰减因子  ((0.0f, 1.0f), 越小衰减越快) */
	float32 fGyrZMean;					/* Z轴方向上角速度均值 */
	float32 fGyrZVar;					/* Z轴方向上角速度方差 */
	float32 fGyrZIntegral;				/* Z轴方向上角速度积分累加 + 衰减 */
	BOOL bGyrZIsStall;					/* Z轴方向上角速度判断堵转 */
	uint8 u8Tmr_GyrZStall_10ms;			/* Z轴方向上角速度堵转计时器 */

	uint8 u8Tmr_MotorStall_10ms;		/* 电机堵转计时器 */
	uint8 u8Tmr_LeftOverCurrent_10ms;	/* 左电机过流计时器 */

	uint8 u8DebugNo;					/* 用于调试 */

	BOOL bDisableCliff;
}YKC_CTR;

YKC_CTR g_YKCCtr;

typedef struct {

}WIN_CLN_RUN_DATA;

const CONF_n_STAT_PROP cnst_YkcConfProp[] = {                                              /* 1 2 3 4 5 6 7 8 9 A B C D E F 1 2 3 4 5 6 7 8 9 A B C D E F */
    {1001, {ITEM_DAT_TOPIC, 0, 0, SAVE_GRP_NUL}, 3, NULL,                      			{"系统与通讯配置", "Default"}},
    {1002, {ITEM_DAT_SOFTVER, 0, 0, SAVE_GRP_NUL}, SOFTWARE_VER, NULL,                	{"软件版本", "Soft Version"}},
    {1003, {ITEM_DAT_U32_ID_RO, 0, 0, SAVE_GRP_BRD}, 0, (void*)&g_Sys.SerialNo,       	{"序列号", "S/N:"}},
    {1004, {ITEM_DAT_T32_RO, 0, 0, SAVE_GRP_BRD}, 0, (void*)&g_Sys.u32License_Seconds,	{"授权期限(年月日时)", "deadline"}},
	{1005, {ITEM_DAT_F32, 0, 0, SAVE_GRP_MCONF}, 0, (void *)&g_YKCCtr.fRightHitCurrent, {"右电机堵转时电流", "Soft Version"}},
	{1006, {ITEM_DAT_F32, 0, 0, SAVE_GRP_MCONF}, 0, (void *)&g_YKCCtr.fLeftHitCurrent, 	{"左电机堵转时电流", "Soft Version"}},
	{1007, {ITEM_DAT_F32, 0, 0, SAVE_GRP_MCONF}, 0, (void *)&g_YKCCtr.fGyrZAlpha, 		{"Z轴方向上角速度的平滑系数 ((0.0f, 1.0f), 越小越平滑)", "Soft Version"}},
	{1008, {ITEM_DAT_F32, 0, 0, SAVE_GRP_MCONF}, 0, (void *)&g_YKCCtr.fGyrZJumpThr, 	{"Z轴方向上角速度跳变阈值  (越小越敏感)", "Soft Version"}},
	{1009, {ITEM_DAT_F32, 0, 0, SAVE_GRP_MCONF}, 0, (void *)&g_YKCCtr.fGyrZDecayFactor, {"Z轴方向上角速度跳变积分衰减因子  ((0.0f, 1.0f), 越小衰减越快)", "Soft Version"}},
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

/* 由一个个单独动作编排出来的动作序列 */
typedef struct {
	WIN_CLN_STATE WinClnST;
	int8 i8Angle;				/* 目标角度（0°为0点，-45°为3点，+45°为9点）仅限[-120°,120°] */
	int16 iForward;				/* 前进或者后退，限制见下文 */
	int8 i8Spray;				/* 喷水，0不喷，1向前喷，-1向后喷 */
	uint8	u8Rsvd;
	uint16	uRsvd;
}CLN_ST_SEQ;
/* 以下是 CLN_ST_SEQ.iForward 特殊指令，除此之外都是合法有效的距离指令 */
#define FORWORD_INSTR_ReturnX	32001	/* 返回X原点 */
#define FORWORD_INSTR_ReturnY	32002		/* 返回Y原点 */
#define FORWORD_INSTR_INF		32000	/* 朝前走到头 */
#define FORWORD_INSTR_nINF		-32000	/* 后退到头 */
#define VALID_FORWORD_DISTANCE(x) ((FORWORD_INSTR_nINF < x) && (x < FORWORD_INSTR_INF))
/* 以下是 CLN_ST_SEQ.iSpray 指令 */
#define SPRAY_INSTR_SPRAY_NONE		0
#define SPRAY_INSTR_SPRAY_FRONT		1
#define SPRAY_INSTR_SPRAY_BACK		(-1)

const CLN_ST_SEQ cnst_ClnSTSeq[] = {
//	{CLN_ST_END, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},
//	{CLN_ST_DRIVE,			45, 	0,						SPRAY_INSTR_SPRAY_NONE},	/* 1. 校准:左转45度 */
//	{CLN_ST_DRIVE, 			-45,	0,						SPRAY_INSTR_SPRAY_NONE},	/* 2. 校准:右转45度 */
//	{CLN_ST_DRIVE, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 3. 校准:回到0度 */
//	{CLN_ST_DRIVE, 			0, 		FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_FRONT},	/* 4. 运动到边 */
////	{CLN_ST_DRIVE, 			0, 		-1800,					SPRAY_INSTR_SPRAY_NONE},	/* 5. 后退 */
//	{CLN_ST_DRIVE, 			0, 		-500,					SPRAY_INSTR_SPRAY_NONE},	/* 5. 后退 */
//	{CLN_ST_DRIVE, 			50, 	0,						SPRAY_INSTR_SPRAY_NONE},	/* 6. 旋转50度 */
//	{CLN_ST_JTURN, 			90, 	1,						SPRAY_INSTR_SPRAY_FRONT},	/* 7. 前进J弯:减少角度 */
//	{CLN_ST_EXP_FORWARD_R, 	90, 	FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_NONE},	/* 8. 沿着上沿前进 */
//	{CLN_ST_EXP_BACK_R, 	90,	 	FORWORD_INSTR_nINF,		SPRAY_INSTR_SPRAY_BACK},	/* 9. 沿着上沿后退 */
//	{CLN_ST_EXP_FORWARD_R, 	90, 	FORWORD_INSTR_INF, 		SPRAY_INSTR_SPRAY_NONE},	/* 10. 再次沿着上沿前进（为防止(顶部倒车时没覆盖到的区域)和(宽度较窄时，左上角的三角区域)没擦到） */

	{CLN_ST_END, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},
	{CLN_ST_DRIVE,			45, 	0,						SPRAY_INSTR_SPRAY_NONE},	/* 1. 校准:左转45度 */
	{CLN_ST_DRIVE, 			-45,	0,						SPRAY_INSTR_SPRAY_NONE},	/* 2. 校准:右转45度 */
	{CLN_ST_DRIVE, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 3. 校准:回到0度 */
	{CLN_ST_DRIVE, 			0, 		FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_FRONT},	/* 4. 运动到边 */
	{CLN_ST_DRIVE, 			0, 		-500,					SPRAY_INSTR_SPRAY_NONE},	/* 5. 后退 */
	{CLN_ST_DRIVE, 			50, 	0,						SPRAY_INSTR_SPRAY_NONE},	/* 6. 旋转50度 */
	{CLN_ST_JTURN, 			90, 	1,						SPRAY_INSTR_SPRAY_FRONT},	/* 7. 前进J弯:减少角度 */
	{CLN_ST_EXP_UP_FORWARD_R,90, 	FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_NONE},	/* 8. 沿着上沿前进 (探索上边沿) */
	{CLN_ST_EXP_UP_BACK_R, 	90,	 	FORWORD_INSTR_nINF,		SPRAY_INSTR_SPRAY_BACK},	/* 9. 沿着上沿后退 (探索上边沿)*/
	{CLN_ST_JUMP, 			0, 		-2,						SPRAY_INSTR_SPRAY_NONE},	/* 10. 跳转到探索上边沿开头 */
	{CLN_ST_EXP_FORWARD_R, 	90, 	FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_NONE},	/* 11. 清洁上沿前进 (从右往左清洁上边沿的开始No) */
	{CLN_ST_EXP_BACK_R, 	90,	 	FORWORD_INSTR_nINF,		SPRAY_INSTR_SPRAY_BACK},	/* 12. 清洁上沿后退 (从左往右清洁上边沿的开始No) */
	{CLN_ST_EXP_FORWARD_R, 	90, 	FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_NONE},	/* 13. 清洁上沿前进 */

	/* 以下是Z清洁 */
	{CLN_ST_JTURN, 			60, 	-1,						SPRAY_INSTR_SPRAY_NONE},	/* 14. 后退J弯:加大角度 */
	{CLN_ST_CTURN, 			90, 	-4000,					SPRAY_INSTR_SPRAY_NONE},	/* 15. 后退S弯:减少角度 */
	{CLN_ST_DRIVE, 			90, 	FORWORD_INSTR_nINF,		SPRAY_INSTR_SPRAY_BACK},	/* 16. 后退到头 */
	{CLN_ST_CTURN, 			120, 	4000,					SPRAY_INSTR_SPRAY_NONE},	/* 17. 前进S弯:加大角度 */
	{CLN_ST_CTURN, 			90, 	4000,					SPRAY_INSTR_SPRAY_NONE},	/* 18. 前进S弯:减少角度 */
	{CLN_ST_DRIVE, 			90, 	FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_FRONT},	/* 19. 前进到头 */
	{CLN_ST_CTURN, 			60, 	-4000,					SPRAY_INSTR_SPRAY_NONE},	/* 20. 后退S弯:加大角度 */
	{CLN_ST_CTURN, 			90, 	-4000,					SPRAY_INSTR_SPRAY_NONE},	/* 21. 后退S弯:减少角度 */
	{CLN_ST_JUMP, 			0, 		-6,						SPRAY_INSTR_SPRAY_NONE},	/* 22. 跳转到Z清洁开头 */


	/* 底部清洁:自右向左(前进)S弯碰边后：先前进，再后退 */
	{CLN_ST_JTURN, 			90, 	1,						SPRAY_INSTR_SPRAY_NONE},	/* 23. 前进J弯:减少角度 */
	{CLN_ST_EXP_FORWARD_L, 	90, 	FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_FRONT},	/* 24. 沿着下沿前进 */
	{CLN_ST_EXP_BACK_L, 	90, 	FORWORD_INSTR_nINF,		SPRAY_INSTR_SPRAY_NONE},	/* 25. 沿着下沿后退 */
	{CLN_ST_JTURN, 			60, 	1,						SPRAY_INSTR_SPRAY_NONE},	/* 26. 前进J弯:加大角度 */
	{CLN_ST_DRIVE, 			90, 	4000,					SPRAY_INSTR_SPRAY_NONE},	/* 27. 前进J弯:减少角度 */
	{CLN_ST_DRIVE, 			90, 	FORWORD_INSTR_ReturnX,	SPRAY_INSTR_SPRAY_NONE},	/* 28. X轴返回原点 */
	{CLN_ST_DRIVE,	 		0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 29. 旋转到朝上 */
	{CLN_ST_DRIVE, 			0, 		FORWORD_INSTR_ReturnY,	SPRAY_INSTR_SPRAY_NONE},	/* 30. Y轴返回原点 */
	{CLN_ST_END, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 31. 结束清洁 */

	/* 底部清洁:自左向右(后退)S弯碰边后：先后退，再前进 */
	{CLN_ST_JTURN, 			90,	 	-1, 					SPRAY_INSTR_SPRAY_NONE},	/* 32. 后退J弯:减少角度 */
	{CLN_ST_EXP_BACK_L, 	90, 	FORWORD_INSTR_nINF, 	SPRAY_INSTR_SPRAY_BACK},	/* 33. 沿着下沿后退 */
	{CLN_ST_EXP_FORWARD_L, 	90, 	FORWORD_INSTR_INF, 		SPRAY_INSTR_SPRAY_NONE},	/* 34. 沿着下沿前进 */
	{CLN_ST_JTURN, 			120, 	-1, 					SPRAY_INSTR_SPRAY_NONE},	/* 35. 后退J弯:加大角度 */
	{CLN_ST_DRIVE, 			90, 	-4000, 					SPRAY_INSTR_SPRAY_NONE},	/* 36. 后退S弯:减少角度 */
	{CLN_ST_DRIVE, 			90, 	FORWORD_INSTR_ReturnX, 	SPRAY_INSTR_SPRAY_NONE},	/* 37. X轴返回原点 */
	{CLN_ST_DRIVE, 			0, 		0, 						SPRAY_INSTR_SPRAY_NONE},	/* 38. 旋转到朝上 */
	{CLN_ST_DRIVE, 			0, 		FORWORD_INSTR_ReturnY, 	SPRAY_INSTR_SPRAY_NONE},	/* 39. Y轴返回原点 */
	{CLN_ST_END, 			0, 		0, 						SPRAY_INSTR_SPRAY_NONE},	/* 40. 结束清洁 */

	/* 测试1 */
	{CLN_ST_DRIVE, 			90, 	0,						SPRAY_INSTR_SPRAY_NONE},	/* 41. 旋转45度 */
	{CLN_ST_CTURN, 			120, 	2000,					SPRAY_INSTR_SPRAY_NONE},	/* 42. 前进S弯:加大角度 */
	{CLN_ST_END, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 43. 结束清洁 */

	/* 测试2 */
	{CLN_ST_EXP_FORWARD_R, 	90, 	FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_NONE},	/* 44. 沿着上沿前进 */
	{CLN_ST_EXP_BACK_R, 	90,	 	FORWORD_INSTR_nINF,		SPRAY_INSTR_SPRAY_BACK},	/* 45. 沿着上沿后退 */
	{CLN_ST_EXP_FORWARD_R, 	90, 	FORWORD_INSTR_INF, 		SPRAY_INSTR_SPRAY_NONE},	/* 46. 再次沿着上沿前进（为防止(顶部倒车时没覆盖到的区域)和(宽度较窄时，左上角的三角区域)没擦到） */
	{CLN_ST_END, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 47. 结束清洁 */

	/* 测试3 */
	{CLN_ST_DRIVE,			90, 	0,						SPRAY_INSTR_SPRAY_NONE},	/* 48. 校准:左转90度 */
	{CLN_ST_DRIVE, 			90, 	FORWORD_INSTR_nINF,		SPRAY_INSTR_SPRAY_NONE},	/* 49. 后退到头 */
	{CLN_ST_DRIVE, 			90, 	FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_NONE},	/* 50. 前进到头 */
	{CLN_ST_JUMP, 			0, 		-2,						SPRAY_INSTR_SPRAY_NONE},	/* 51. 跳转到Z清洁开头 */
	{CLN_ST_END, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 52. 结束清洁 */

	/* 测试4 */

};
#define CLN_SEQ_START_No		1
#define CLN_SEQ_Z_BEGIN_No		11
#define CLN_SEQ_WIN_BTM_R2L_No	23			/* 底部清洁，自右向左 */
#define CLN_SEQ_WIN_BTM_L2R_No	32			/* 底部清洁，自左向右 */
#define CLN_SEQ_TEST_No			38
#define CLN_SEQ_TEST2_No		39
#define CLN_SEQ_TEST3_No		40

/* 调试用 */
#define CLN_SEQ_TEST1_START_No	41
#define CLN_SEQ_TEST2_START_No	44
#define CLN_SEQ_TEST3_START_No	48

/*---------------------测试用开关---------------------*/
#define CLIFF_DEACTIVE 	FALSE			/* 临时关闭悬崖开关（常为开） */
#define PUMP_DEACTIVE	FALSE			/* 关闭真空泵 */
#define LOW_AIR_P		FALSE			/* 允许低真空水平至2300Pa，而不是标准的2800Pa */
#define DEBUG_PRINT_CLN_STATE	TRUE 	/* 打印清污机状态 */
#define LOW_AIR_CLOSE_CLN	FALSE		/* 低气压时停止擦窗机 */
#define CLOSE_SPRAY_WATER	FALSE		/* 关闭喷水 */
#define MAX_DUTY_RATIO		0.85f		/* 履带最大占空比, 用于控制擦窗机最大速度 */
/*----------------------------end----------------------------*/

/* 1000个履带轮脉冲，行走约37mm; 27脉冲/mm; 履带间距约230mm, 擦窗机为265mm*265mm正方形 */
#define MCH_LENGTH_mm				265.0f		/* 机身长度 */
#define TRACK_DISTANCE_mm			230.0f		/* 履带间距 */
#define PULSE_FORWARD_mm			0.037f		/* 每脉冲前进距离 */
#define TRACK_DISTANCE_pulse		(TRACK_DISTANCE_mm/PULSE_FORWARD_mm) /* 履带间距，以履带行进脉冲为单位，约6216个脉冲 */
#define PI							3.1415926f
#define VACUUM_AIM_ERR_TOLERANCE	0.15f
#define PUMP_TICK_TO_FULL_SPEED		200			/* 真空泵开到全速所需时长（单位10ms） */
#define PUMP_TICK_VACUMM_SUFF		100			/* 气压不足判定时长（单位10ms）  */
#define PUMP_IDEL_DUTY				0.0f		/* IDEL时维持很低的吸力，防止掉落 */
#define SPRAY_TIMEOUT_10ms			100			/* 喷水超时 */
#if LOW_AIR_P
			#define AIMED_AIR_P	2100
#else
			#define AIMED_AIR_P	2800
#endif

/***************************************************************************
						internal functions declaration
***************************************************************************/
void NextWinClnSTBySeq(uint8 u8NewSeqNo_0Auto);

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
	InitDataWithZero((uint8*)(&g_MsrRes), sizeof(g_MsrRes));
	InitDataWithZero((uint8*)(&g_AnaRes), sizeof(g_AnaRes));
	InitDataWithZero((uint8*)(&g_YKCCtr), sizeof(g_YKCCtr));
	InitDataWithZero((uint8*)(&g_Ctr), sizeof(g_Ctr));

	/* 初始化配置 */
	g_YKCCtr.fGyrZAlpha = 0.05f;
	g_YKCCtr.fGyrZJumpThr = 10.0f;
	g_YKCCtr.fGyrZDecayFactor = 0.9f;

	/* 初始化队列 */
	InitF32Queue(&g_MsrRes.qLeftMotorCurrHistory, 15);
	InitF32Queue(&g_MsrRes.qRightMotorCurrHistory, 15);

	/* 初始化输入接口变量 */

	/* 初始化输出接口变量 */
	g_YKCCtr.fDragConfR2L = 0.88;
	g_YKCCtr.u8Tmr_PumpFullSpeed_10ms = PUMP_TICK_TO_FULL_SPEED;
	g_YKCCtr.u8Tmr_VacummSuff_10ms = PUMP_TICK_VACUMM_SUFF;
	g_YKCCtr.i8Tmr_Spray_Front_10ms = -1;
	g_YKCCtr.i8Tmr_Spray_Back_10ms = -1;

	g_YKCCtr.fPumpDuty = 0;
	g_YKCCtr.i32TripCntForSpray = 0;
	/* 初始化下层模块 */

	/* 启动硬件 */
	InitSample();		/* 由于采样需要填充buf，然后才好计算，因此现行启动 */

	GPIO_write(OTHER_DEV_DOutNo, 1);		/* 使能周边设备3.3v供电 */

	printf("---------------正在测试硬件驱动---------------\n");

	/* 测试语音模块 */
	printf("\n初始化MP3模块...\n");
	if(!InitMp3Speaking()) {
		printf("MP3模块初始化失败...\n");
	}
	printf("MP3模块初始化成功...\n");


	printf("\n初始化陀螺仪模块...\n");
	/* 测试六轴传感器 */
    if(!Bmi270Init()) {
    	printf("陀螺仪模块初始化失败...\n");
	}
    printf("陀螺仪模块初始化成功...\n");


    printf("\n初始化气压计模块...\n");
	/* 初始化气压计 */
    if(!Bmp280Init()) {
    	printf("气压计模块初始化失败...\n");
	}
    printf("气压计模块初始化成功...\n");

	printf("\n开始流水灯\n");
	printf("红...\n");
	ControlLed(LED_COLOR_RED);
	Task_sleep(2000);
	printf("绿...\n");
	ControlLed(LED_COLOR_GREEN);
	Task_sleep(2000);
	printf("蓝...\n");
	ControlLed(LED_COLOR_BLUE);
	Task_sleep(2000);
	printf("黄...\n");
	ControlLed(LED_COLOR_YELLOW);
	Task_sleep(2000);
	printf("粉...\n");
	ControlLed(LED_COLOR_PINK);
	Task_sleep(2000);
	printf("青...\n");
	ControlLed(LED_COLOR_CYAN);
	Task_sleep(2000);

	ControlLed(0);

	printf("\n已启动红外捕捉, 可使用红外遥控测试\n");
	/* 初始化红外遥控器  TIM5频率1MHz, ARR==65535 */
	extern TIM_HandleTypeDef htim5;
	HAL_TIM_Base_Start_IT(&htim5);	 				/* 启动定时器中断，之所以没放在main.c里，是因为想在初始化代码之后再开始 */
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);  	/* 启动定时器输入捕获，之所以没放在main.c里，是因为想在初始化代码之后再开始 */
}

/*==========================================================================
| Description	:
	运动控制分为：运动决策、运动执行与观测

	运动分为基本动作、高级动作(由多个基本动作组合)
	基本动作：直行+转角

	擦窗机位置：x,y,a——x,y 是相对起点，每一次运动后根据履带运动距离、运动方向更新；a是擦窗机角度，通过imu读出

	运动决策，具体动作由{}标记，运动目标用[]标记：
	运动到顶点：
		启动后，{原地旋转}，调整到[朝上，垂直于地面]，
		然后{直行(距离无限制)}，直至触碰
		然后{到头S转弯}，
		然后{贴边}，直至到头；再{贴边}，直至到头；
		再N弯-直行-N弯-直行，直至N弯右前碰到边

待删：
		直行：两条履带同向运动，保证走过相同的长度
			包括：垂直、水平、斜行三种运动轨迹，运动中需要保持G方向不变
			效果：角度不变，中心运动
		原地旋转：两条履带反向运动，保证走过相同的长度
			效果：中心不动，角度发生改变
		单边转弯：一条履带静止，另外一条履带运动
			效果：以一条履带中心为原点进行转动，中心发生旋转运动
		双边转弯：两条履带以不同速度运行，应该是指定
			效果：转弯半径较单边转弯更大
	组合动作：
		到头转弯：直行(退后**)+原地旋转
		贴边，尽量靠近边沿：单边转弯以获取小角度朝着边沿行驶，使得限位(或悬崖)处于不断触发状态
		探边，探索边沿：这种发生在一个角触边的情况
		N/Z弯：直行脱离边沿-单边转弯-直行(其实是斜行)-单边转弯，这个动作的单边转弯应该是双边转弯


| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2023-12-8
\=========================================================================*/
/* 履带过流判断值 */
BOOL IsMeetWheelStallCur(float32 fCurrent)
{
//	float32 fVacummRate = (g_YKCCtr.fAirPressure_PumpIdle - g_MsrRes.fAirPressure) / AIMED_AIR_P;
//	/* 2107Pa时为0.76A, 2800Pa时为1A, 用真空压力为AIMED_AIR_P时的履带卡住电流当成最大判定电流 */
//	if((fVacummRate < 1.0f) && (fVacummRate > 0.5f)) {		/* >0.5是防止真空度偶尔过低一下，不至于被误判成卡住 */
//		return fCurrent > fVacummRate;
//	} else {
//		return fCurrent > 1.0f;
//	}
#if USE_SELF_DEVELOP_MOTOR
	return fCurrent > 1.5f;		/* 如果限速了 推荐过流电流是1.0~1.2, 没限速推荐过流电流是1.2~1.5 */
#else
	return fCurrent > 0.5f;
#endif
}
BOOL WinClnSectionHang(void);
BOOL WinClnCompleteHang(void);
void WinClnDebug(void);
/* 判断擦窗机是否部分悬空 即：至少有一个悬空 */
BOOL WinClnSectionHang(void)
{
	return (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]			/* 四个角贴上 */
										|| g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo]
										|| g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]
										|| g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo]);
}

/* 判断擦窗机是否完全悬空 即：四个角全部悬空 */
BOOL WinClnCompleteHang(void)
{
	return (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]			/* 四个角贴上 */
										&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo]
										&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]
										&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo]);
}
/* Debug 调试信息 */
const char *aChWinClnSTStr[] = {
		"空闲",
		"暂停",
		"悬空",
		"清洁完成",
		"循环跳转",
		"直线+旋转",
		"J弯(指教转弯)",
		"S弯(倒U型弯)",
		"沿左边前进",
		"沿右边前进",
		"沿左边后退",
		"沿右边后退",
		"前进探索上沿",
		"后退探索上沿",
		"空白"
};
const char *aChWinClnDinStr[] = {
		"开关按下",
		"水箱有水",
		"侧左前碰撞",
		"侧左后碰撞",
		"侧右前碰撞",
		"侧右后碰撞",
		"前左碰撞",
		"前右碰撞",
		"后左碰撞",
		"后右碰撞",
		"前左悬空",
		"前右悬空",
		"后左悬空",
		"后右悬空"
};
#if DEBUG_PRINT_CLN_STATE
#include <stdio.h>
#define DEBUG_PRINTF_FREQUENCY_10MS			100		/* Debug信息打印频率 单位10ms */
void WinClnDebug(void)
{
	static int i32DebugCnt = DEBUG_PRINTF_FREQUENCY_10MS;
	if(i32DebugCnt--) {
		return ;
	} else {
		i32DebugCnt = DEBUG_PRINTF_FREQUENCY_10MS;
	}
//	printf("擦窗机运动状态: %s 角度: %.2f 目标角度: %.2f\n", aChWinClnSTStr[g_YKCCtr.WinClnST], g_YKCCtr.fAngle, g_YKCCtr.fAimedAngle);
//	printf("\n擦窗机运动状态: %s 左轮电流：%.4f, 右轮电流：%.4f\n", aChWinClnSTStr[g_YKCCtr.WinClnST], g_MsrRes.fLeftCur, g_MsrRes.fRightCur);
//	printf("g_MsrRes.fAccelerateX = %.5f\n", g_MsrRes.fAccelerateX);
	printf("\n气压差：%.2f\n", g_YKCCtr.fAirPressure_PumpIdle - g_MsrRes.fAirPressure);
	/* 打印开出状态 */
//	printf("擦窗机碰撞悬空状态: ");
//	for(int i = 0; i <= CLIFF_REAR_RIGHT_DInNo; i++) {
//		if(g_YKCCtr.uTimer_DInSig_10ms[i]) {
//			printf("%s ", aChWinClnDinStr[i]);
//		}
//	}
//	printf("\n\n\n");
}
#endif

#define MOTOR_INC_TOLERANCE_A		0.01f			/* 履带电机最小上涨步长 */
#define MOTOR_INC_REQUIRED_COUNT    10				/* 电机电流连续上涨次数到达该值认为堵转的爬升 */
#define MOTOR_SLOPE_THRESHOLD		3.0f			/* 履带电机堵转K阈值 */

/* 更新擦窗机Z轴角速度数据 */
void UpdateWinClnGyrZ(void)
{
//	if((fabsf(g_YKCCtr.fAimedAngle - g_YKCCtr.fAngle) < 5.0f)		/* 确保不是在转弯，否则角速度会异常增大判断会失效 */
//	&& (((abs(g_YKCCtr.i8Timer_RightAct) > 10)
//			&& (fabs(g_YKCCtr.fRightDuty) > MAX_DUTY_RATIO - 0.03f))
//		|| ((abs(g_YKCCtr.i8Timer_LeftAct) > 10)
//			&& (fabs(g_YKCCtr.fLeftDuty) > MAX_DUTY_RATIO - 0.03f)))) {	/* 擦窗机正在稳定运行, 并且占空比为满（之所以满占空比是因为擦窗机运行时至少有一个电机是满占空比） */
	if(((abs(g_YKCCtr.i8Timer_RightAct) > 10)
			&& (fabs(g_YKCCtr.fRightDuty) > MAX_DUTY_RATIO - 0.03f))
		|| ((abs(g_YKCCtr.i8Timer_LeftAct) > 10)
			&& (fabs(g_YKCCtr.fLeftDuty) > MAX_DUTY_RATIO - 0.03f))) {	/* 擦窗机正在稳定运行, 并且占空比为满（之所以满占空比是因为擦窗机运行时至少有一个电机是满占空比） */
		float32 fGyrZ = (float32)abs(g_MsrRes.iGyroZ);
		float32 fAim_R2L = g_YKCCtr.fRightDuty / g_YKCCtr.fLeftDuty;
		if(g_YKCCtr.fGyrZMean == 0.0f) {
			g_YKCCtr.fGyrZMean = fGyrZ;
			g_YKCCtr.fGyrZVar = 1e-6f;
		} else {
			float32 fDiff = fGyrZ - g_YKCCtr.fGyrZMean;
			g_YKCCtr.fGyrZMean = (g_YKCCtr.fGyrZAlpha * fGyrZ) + (1 - g_YKCCtr.fGyrZAlpha) * g_YKCCtr.fGyrZMean;
			g_YKCCtr.fGyrZVar = (g_YKCCtr.fGyrZAlpha * fDiff * fDiff) + (1 - g_YKCCtr.fGyrZAlpha) * g_YKCCtr.fGyrZVar;
		}
		float32 fStdDev = sqrtf(g_YKCCtr.fGyrZVar + 1e-6f);				/* 标准差 */
		float32 fJumpZ = (fGyrZ - g_YKCCtr.fGyrZMean) / fStdDev;		/* 跳变异常值 */

		/* 积分累加 + 衰减 */
		g_YKCCtr.fGyrZIntegral = g_YKCCtr.fGyrZIntegral * g_YKCCtr.fGyrZDecayFactor + fJumpZ;
		g_YKCCtr.bGyrZIsStall = g_YKCCtr.fGyrZIntegral > g_YKCCtr.fGyrZJumpThr;

		/* 查看是否发生了堵转 */
		if(fGyrZ < 4.0f) {
			if(g_YKCCtr.u8Tmr_GyrZStall_10ms < 0xFF) {
				g_YKCCtr.u8Tmr_GyrZStall_10ms++;
			}
		} else {
			g_YKCCtr.u8Tmr_GyrZStall_10ms = 0;
		}
#if DEBUG_PRINT_CLN_STATE
		printf("fGyrZ = %f, fJumpZ = %.5f, g_YKCCtr.fGyrZIntegral = %.5f DutyAim_R2L = %.5f\n", fGyrZ, fJumpZ, g_YKCCtr.fGyrZIntegral, fAim_R2L);
		printf("ac = %.5f, ex = %.5f\n\n", g_YKCCtr.fAngle, g_YKCCtr.fAimedAngle);
//		printf("RightDuty = %f, LeftDuty = %.5f, RightI = %.5f LeftI = %.5f, ac = %.5f, ex = %.5f\n", g_YKCCtr.fRightDuty, g_YKCCtr.fLeftDuty, g_MsrRes.fRightCur, g_MsrRes.fLeftCur, g_YKCCtr.fAngle, g_YKCCtr.fAimedAngle);
#endif
	} else {	/* 停下或者行进状态改变了, 重新计算 */
		g_YKCCtr.bGyrZIsStall = FALSE;
		g_YKCCtr.fGyrZIntegral = 0.0f;
		g_YKCCtr.fGyrZMean = 0.0f;
		g_YKCCtr.u8Tmr_GyrZStall_10ms = 0;
	}
}

/* 更新擦窗机电流数据 */
void UpdateWinClnCurr(void)
{
//	/* 右电机过流判断与运行电流记录 */
//	if(abs(g_YKCCtr.i8Timer_RightAct) > 10) {	/* 电机运动稳定了再进行过流判断并保存正常行驶的电流 */
//		if((fabs(g_YKCCtr.fRightDuty) > MAX_DUTY_RATIO - 0.03f)) {	/* 记录满占空比时的电流 */
//			F32QueueEnter(&g_MsrRes.qRightMotorCurrHistory, g_MsrRes.fRightCur);		/* 电流入队 */
//			float32 fK = GetF32QueueSlope(&g_MsrRes.qRightMotorCurrHistory, PERIOD_CTR_TASK_ms / 1000.0f, 10);
//#if DEBUG_PRINT_CLN_STATE
//			printf("right K = %.4f\n", fK);
//#endif
//			if(fK > MOTOR_SLOPE_THRESHOLD) {		/* 疑似堵转, 记录当前的最大电流值 */
//				g_YKCCtr.fRightHitCurrent = F32QueueGetMaxElement(&g_MsrRes.qRightMotorCurrHistory);
//				g_YKCCtr.u8Tmr_RightOverCurrent_10ms++;
//			}
//		}
//	} else {	/* 可能是换方向了, 将当前记录的电流值清除, 重新记录 */
//		F32QueueClear(&g_MsrRes.qRightMotorCurrHistory);
//		g_YKCCtr.u8Tmr_RightOverCurrent_10ms = 0;
//	}
//
//	if(abs(g_YKCCtr.i8Timer_LeftAct) > 10) {
//		/* 电流入队 */
//		if((fabs(g_YKCCtr.fLeftDuty) > MAX_DUTY_RATIO - 0.03f)) {	/* 记录满占空比时的电流 */
//			F32QueueEnter(&g_MsrRes.qLeftMotorCurrHistory, g_MsrRes.fLeftCur);
//			float32 fK = GetF32QueueSlope(&g_MsrRes.qLeftMotorCurrHistory, PERIOD_CTR_TASK_ms / 1000.0f, 10);
//#if DEBUG_PRINT_CLN_STATE
//			printf("left K = %.4f\n", fK);
//#endif
//			if(fK > MOTOR_SLOPE_THRESHOLD) {
//				g_YKCCtr.fLeftHitCurrent = F32QueueGetMaxElement(&g_MsrRes.qLeftMotorCurrHistory);
//				g_YKCCtr.u8Tmr_LeftOverCurrent_10ms++;
//			}
//		}
//	} else {
//		F32QueueClear(&g_MsrRes.qLeftMotorCurrHistory);
//		g_YKCCtr.u8Tmr_LeftOverCurrent_10ms = 0;
//	}
}

/* 获取擦窗机堵转状态 */
void GetMotorStallStatus(void)
{
	g_YKCCtr.bLeftStall = FALSE;
	g_YKCCtr.bRightStall = FALSE;
	if(g_Bmi270Comm.bIsStop && ((fabsf(g_YKCCtr.fRightDuty) > 0.5f) || (fabsf(g_YKCCtr.fRightDuty) > 0.5f))) {
		/* 陀螺仪中断判断堵转 */
		if(g_YKCCtr.u8Tmr_MotorStall_10ms > 0) {	/* 堵转超过nms后才认为是堵转 */
			g_YKCCtr.u8Tmr_MotorStall_10ms--;
		} else {	/* 确定是堵转 */
			g_YKCCtr.bLeftStall = TRUE;
			g_YKCCtr.bRightStall = TRUE;
			g_YKCCtr.u8Tmr_MotorStall_10ms = 5;
			printf("【中断】\n");
		}
	} else if(g_YKCCtr.u8Tmr_GyrZStall_10ms > 10) {	/* 陀螺仪Z轴方向上角速度不变判断堵转 */
		g_YKCCtr.bLeftStall = TRUE;
		g_YKCCtr.bRightStall = TRUE;
		printf("【角速度不变】\n");
	} else if(g_YKCCtr.bGyrZIsStall) {				/* 陀螺仪Z轴方向的角速度变化过快 堵转(不稳定，尤其是转弯时角速度一定会变化特别大，暂时不使用) */
//		g_YKCCtr.bLeftStall = TRUE;
//		g_YKCCtr.bRightStall = TRUE;
		printf("【角速度瞬变】\n");
	}
	if(g_MsrRes.fRightCur > 1.5f) {		/* 电流判断堵转 */
		g_YKCCtr.bRightStall = TRUE;
		printf("【电流】\n");
	}
	if(g_MsrRes.fLeftCur > 1.5f) {
		g_YKCCtr.bLeftStall = TRUE;
		printf("【电流】\n");
	}
}

static uint16 uRunTmr_10ms = 0;
char *ppChGprsStateStr[5] = {"正常", "未初始化", "没有SIM卡", "无法上网", "未知状态"};

void RunMdlCtr(void)		/* rename from RunYKC */
{
	uRunTmr_10ms++;

	UpdateAir();		/* 更新气压计 */
	UpdateIMU();		/* 更新IMU状态 */

	if(uRunTmr_10ms > 100) {
		uRunTmr_10ms = 0;
		uint8 uStateIndex = abs(g_GprsComm.GprsStatus) > 3 ? 4 : abs(g_GprsComm.GprsStatus);
		printf("卡状态： %s\n", ppChGprsStateStr[uStateIndex]);
		printf("当前气压： %.3f\n", g_MsrRes.fAirPressure);
		printf("当前角度： %.3f\n", g_YKCCtr.fAngle);
	}
	int8 i;

	UInt HwiKey = Hwi_disable();
	int16 iRightCount = g_MsrRes.uRightCount;
	int16 iLeftCount = g_MsrRes.uLeftCount;
	g_MsrRes.uRightCount = 0;
	g_MsrRes.uLeftCount = 0;
    Hwi_restore(HwiKey);
	if(g_YKCCtr.fRightDuty < 0) {
		iRightCount = 0 - iRightCount;		/* 反转时，用负数计数 */
	}
	if(g_YKCCtr.fLeftDuty < 0) {
		iLeftCount = 0 - iLeftCount;		/* 反转时，用负数计数 */
	}

	/* 解算当前角度 */
	float32 fAngle = g_MsrRes.fPitch;
	float32 fAngleAvr = ((g_YKCCtr.fAngle + fAngle)/2)*(PI/180);		/* 两次平均角度（弧度） */

	/* 刷新当前位置 */
	float32 fTrip = (iRightCount + iLeftCount)*0.5f;
	g_YKCCtr.i32X += F32ToI32(fTrip*sinf(fAngleAvr));
	g_YKCCtr.i32Y += F32ToI32(fTrip*cosf(fAngleAvr));

	/* 响应遥控器干预 */
	UpdateIRCtrl();
	if(g_IRCtrl.u8TryCnt) {
		switch(g_IRCtrl.u8BtnPressing) {
		case IR_BTN_0:
			printf("\n按下0\n");
			break;
		case IR_BTN_1:
			printf("\n按下1\n");
			break;
		case IR_BTN_2:
			printf("\n按下2\n");
			break;
		case IR_BTN_3:
			printf("\n按下3\n");
			break;
		case IR_BTN_4:
			printf("\n按下4\n");
			break;
		case IR_BTN_5:
			printf("\n按下5\n");
			break;
		case IR_BTN_6:
			printf("\n按下6\n");
			break;
		case IR_BTN_7:
			printf("\n按下7\n");
			break;
		case IR_BTN_8:
			printf("\n按下8\n");
			break;
		case IR_BTN_9:
			printf("\n按下9\n");
			break;
		case IR_BTN_START:
			printf("\n按下左喷水\n");
			break;
		case IR_BTN_SHARP:
			printf("\n按下右喷水\n");
			break;
		case IR_BTN_UP:
			printf("\n按下前进\n");
			break;
		case IR_BTN_DOWN:
			printf("\n按下后退\n");
			break;
		case IR_BTN_LEFT:
			printf("\n按下左转\n");
			break;
		case IR_BTN_RIGHT:
			printf("\n按下右转\n");
			break;
		case IR_BTN_OK:
			printf("\n按下OK\n");
			break;
		}
	}
}

/* 依据序列进行动作
	0	: 按照序列进行
	非0	: 指定序号 */
void NextWinClnSTBySeq(uint8 u8NewSeqNo_0Auto)
{
	if((u8NewSeqNo_0Auto != 0)	/* 指定新的序列，不在乎之前的序号 */
		/* 当前是否在某个动作序列中 */
		|| ((g_YKCCtr.u8ClnSeqNo < sizeof(cnst_ClnSTSeq)/sizeof(CLN_ST_SEQ))
			&& (cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].WinClnST != CLN_ST_END)))
	{
		if(u8NewSeqNo_0Auto != 0) {		/* 非0，按照指定的进行 */
			g_YKCCtr.u8ClnSeqNo = u8NewSeqNo_0Auto;
		} else {						/* 0，自增 */
			g_YKCCtr.u8ClnSeqNo++;
		}
		/* 下一个动作如果是当前动作序列结尾，则返回 */
		if((g_YKCCtr.u8ClnSeqNo >= sizeof(cnst_ClnSTSeq)/sizeof(CLN_ST_SEQ))
			|| (cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].WinClnST == CLN_ST_END))
		{
			g_YKCCtr.i32Forward = 0;
			g_YKCCtr.fAimedAngle = g_YKCCtr.fAngle;
			g_YKCCtr.WinClnST = CLN_ST_END;
		} else if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].WinClnST == CLN_ST_BLANK) {
			g_YKCCtr.i32Forward = 0;
			g_YKCCtr.fAimedAngle = g_YKCCtr.fAngle;
			g_YKCCtr.WinClnST = CLN_ST_BLANK;
		} else {	/* 否则用新的动作序列初始化 */
			if(VALID_FORWORD_DISTANCE(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward)) {
				g_YKCCtr.i32Forward = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward;
			} else if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward == FORWORD_INSTR_INF) {
				g_YKCCtr.i32Forward = 1000000000;	//约37KM
			} else if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward == FORWORD_INSTR_nINF) {
				g_YKCCtr.i32Forward = -1000000000;	//约37KM
			} else if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward == FORWORD_INSTR_ReturnX) {
				g_YKCCtr.i32Forward = 0 - g_YKCCtr.i32X;
			} else if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward == FORWORD_INSTR_ReturnY) {
				g_YKCCtr.i32Forward = 0 - g_YKCCtr.i32Y;
			} else {
				g_YKCCtr.i32Forward = 0;
			}
			g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle;
			g_YKCCtr.WinClnST = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].WinClnST;
		}
	}
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
	/* 存在多组密码, 计算密码组数 */
	uint16 uAuthGrpNo = AuthType/AUTH_TYPE_NUM;
	AuthType = (AUTH_TYPE)(AuthType%AUTH_TYPE_NUM);

	/* 填充密码 */
	if(uAuthGrpNo == 0) {					/* 第0组密码 */
		switch(AuthType) {
			case AUTH_KEY_FLASH_REQ_KEY:	/* 1个 */
				*pAuthKey++ = 0x767C2584;
				break;

			case AUTH_AES_FLASH_REQ:		/* 4个 */
				*pAuthKey++ = 0x266946E7;
				*pAuthKey++ = 0x571E4325;
				*pAuthKey++ = 0x9857C686;
				*pAuthKey++ = 0x7A26B3C7;
				break;

			case AUTH_AES_FLASH_DATA:		/* 4个 */
				*pAuthKey++ = 0x295692F5;
				*pAuthKey++ = 0x3A2B4896;
				*pAuthKey++ = 0x8284411E;
				*pAuthKey++ = 0x7F525561;
				break;

			case AUTH_AES_PACK_SERIAL:		/* 4个 */
				*pAuthKey++ = 0x533A4B17;
				*pAuthKey++ = 0x7B992D81;
				*pAuthKey++ = 0x47E09593;
				*pAuthKey++ = 0x562F8395;
				break;

			case AUTH_AES_PACK_LICENSE: 	/* 4个 */
				*pAuthKey++ = 0x577A3339;
				*pAuthKey++ = 0x11B75C26;
				*pAuthKey++ = 0x82536B22;
				*pAuthKey++ = 0x8A2B931B;
				break;

			case AUTH_KEY_FLASH_INTEGRITY_V1:
			case AUTH_KEY_FLASH_INTEGRITY_V2:
				*pAuthKey++ = 1;
				*pAuthKey++ = 1;
				break;
			default:
				break;
		}
	} else if(uAuthGrpNo == 1) {			/* 第1组密码 */
		switch(AuthType) {
			case AUTH_KEY_FLASH_REQ_KEY:	/* 1个 */
				*pAuthKey++ = 4;
				break;

			case AUTH_AES_FLASH_REQ:		/* 4个 */
				*pAuthKey++ = 4;
				*pAuthKey++ = 3;
				*pAuthKey++ = 2;
				*pAuthKey++ = 1;
				break;

			case AUTH_AES_FLASH_DATA:		/* 4个 */
				*pAuthKey++ = 1;
				*pAuthKey++ = 2;
				*pAuthKey++ = 3;
				*pAuthKey++ = 4;
				break;

			case AUTH_AES_PACK_SERIAL:		/* 4个 */
				*pAuthKey++ = 5;
				*pAuthKey++ = 6;
				*pAuthKey++ = 7;
				*pAuthKey++ = 8;
				break;

			case AUTH_AES_PACK_LICENSE: 	/* 4个 */
				*pAuthKey++ = 4;
				*pAuthKey++ = 3;
				*pAuthKey++ = 2;
				*pAuthKey++ = 1;
				break;

			case AUTH_KEY_FLASH_INTEGRITY_V1:
			case AUTH_KEY_FLASH_INTEGRITY_V2:
				*pAuthKey++ = 1;
				*pAuthKey++ = 1;
				break;
			default:
				break;
		}
	}
}

__weak BOOL ChkAuthKey(AUTH_TYPE AuthType, uint32* pAuthKey)
{
	/* 存在多组密码, 计算密码组数 */
	uint16 uAuthGrpNo = AuthType/AUTH_TYPE_NUM;
	AuthType = (AUTH_TYPE)(AuthType%AUTH_TYPE_NUM);

	/* 比较密码 */
	if(AuthType == AUTH_KEY_FLASH_REQ_KEY) {	/* 1个,必须和GetAuthKey()产生的一致 */
		return ((uAuthGrpNo == 0) && (pAuthKey[0] == 0x767C2584)) || ((uAuthGrpNo == 1) && (pAuthKey[0] == 4));
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
	PrintStringToJson(&pTxBuf, "Status", aChWinClnSTStr[g_YKCCtr.WinClnST]);
	PrintF32DatToJson(&pTxBuf, "气压差", g_YKCCtr.fAirPressure_PumpIdle - g_MsrRes.fAirPressure, 4);
	PrintF32DatToJson(&pTxBuf, "Idle", g_YKCCtr.fAirPressure_PumpIdle, 2);
	PrintF32DatToJson(&pTxBuf, "Now", g_MsrRes.fAirPressure, 2);
	PrintF32DatToJson(&pTxBuf, "Pump", g_YKCCtr.fPumpDuty, 2);
	PrintU32DatToJson(&pTxBuf, "din", g_Ctr.u32DinDat, 0);
	PrintU32DatToJson(&pTxBuf, "RightCnt", g_MsrRes.uRightCount, 0);
	PrintU32DatToJson(&pTxBuf, "LeftCnt", g_MsrRes.uLeftCount, 0);
	PrintF32DatToJson(&pTxBuf, "RightI", g_MsrRes.fRightCur, 2);
	PrintF32DatToJson(&pTxBuf, "LeftI", g_MsrRes.fLeftCur, 2);
	PrintF32DatToJson(&pTxBuf, "RightMaxI", g_MsrRes.fRightMaxI, 2);
	PrintF32DatToJson(&pTxBuf, "LeftMaxI", g_MsrRes.fLeftMaxI, 2);
	PrintF32DatToJson(&pTxBuf, "当前角度：", g_YKCCtr.fAngle, 2);
	PrintF32DatToJson(&pTxBuf, "目标角度：：", g_YKCCtr.fAimedAngle, 2);
	PrintStringNoOvChk(&pTxBuf, "\"Hit:\"");
	for(int i = 0; i <= CLIFF_REAR_RIGHT_DInNo; i++) {
		if(g_YKCCtr.uTimer_DInSig_10ms[i]) {
//			printf("%s ", aChWinClnDinStr[i]);
			PrintStringNoOvChk(&pTxBuf, aChWinClnDinStr[i]);
			*pTxBuf++ = ',';
		}
	}
//	*pTxBuf++ = ',';
	pTxBuf--;
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
	return FALSE;
}
#endif
/******************************** FILE END ********************************/
