//下载程序时：没连AC插头时，风机较弱，真空度不够；接上AC插头时，下载器提示没接板子，但测量stlink处的3.3正常（3.28v），不接AC插头3.29v，所以原因还不知道，这条我反馈给陈工。
//调查出了拒触的原因：
//    A,  主白色塑料材料有点涩、酥、摩擦阻力大，导致有时在有压力时，碰撞开关会按不完全导致不起作用，且软件里边缘、前后同时探索时需要同面两个碰撞开关同时检测到碰撞才算检测到边，这成功出现的概率就更小。
//    B，检测条件除了碰撞还有stall，可是真卡住时有的机器电流没有程序里用的1A那么大，我放宽到了0.5A，可能是由于组装的原因和密封材料不同的原因，有的机器有的没到2800Pa，造成履带电机卡住时电流没那么大，从而没成功检测到stall。另外气压不足和主密封条有显著关系，和安装过程中各个空隙密闭不严也有关系。




#if SOFT_RUN1_TEST0
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

	uint8 u8Tmr_RightOverCurrent_10ms;	/* 右电机过流计时器 */
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
//	{CLN_ST_DRIVE, 			0, 		-1800,					SPRAY_INSTR_SPRAY_NONE},	/* 5. 后退 */
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
	{CLN_ST_DRIVE, 			90, 	0,						SPRAY_INSTR_SPRAY_NONE},	/* 38. 旋转45度 */
	{CLN_ST_CTURN, 			120, 	2000,					SPRAY_INSTR_SPRAY_NONE},	/* 14. 前进S弯:加大角度 */
	{CLN_ST_END, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 40. 结束清洁 */

	/* 测试2 */
	{CLN_ST_EXP_FORWARD_R, 	90, 	FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_NONE},	/* 41. 沿着上沿前进 */
	{CLN_ST_EXP_BACK_R, 	90,	 	FORWORD_INSTR_nINF,		SPRAY_INSTR_SPRAY_BACK},	/* 42. 沿着上沿后退 */
	{CLN_ST_EXP_FORWARD_R, 	90, 	FORWORD_INSTR_INF, 		SPRAY_INSTR_SPRAY_NONE},	/* 43. 再次沿着上沿前进（为防止(顶部倒车时没覆盖到的区域)和(宽度较窄时，左上角的三角区域)没擦到） */
	{CLN_ST_END, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 44. 结束清洁 */

	/* 测试3 */
//	{CLN_ST_DRIVE, 			90, 	FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_FRONT},	/* 45. 运动到边 */
	{CLN_ST_DRIVE, 			-90,	0,						SPRAY_INSTR_SPRAY_NONE},	/* 2. 校准:右转90度 */
	{CLN_ST_JTURN, 			-60, 	-1,						SPRAY_INSTR_SPRAY_NONE},	/* 11. 后退J弯:加大角度 */
	{CLN_ST_END, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 46. 结束清洁 */

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
#define CLN_SEQ_TEST2_START_No	41
#define CLN_SEQ_TEST3_START_No	45

/*---------------------测试用开关---------------------*/
#define CLIFF_DEACTIVE 	FALSE			/* 临时关闭悬崖开关（常为开） */
#define PUMP_DEACTIVE	FALSE			/* 关闭真空泵 */
#define LOW_AIR_P		TRUE			/* 允许低真空水平至2300Pa，而不是标准的2800Pa */
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
	g_YKCCtr.fGyrZJumpThr = 150.0f;
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

	/* 初始化语音模块 */
	InitTtsSpeaking();
	
	GPIO_write(OTHER_DEV_DOutNo, 1);		/* 使能周边设备3.3v供电 */
	/* 初始化六轴传感器 */
    if(!Bmi270Init()) {
		/* IMU初始化失败*/
    	TtsSpeak(VOICE_IMU_ABN, FALSE);
		configASSERT(FALSE);
	}
	/* 初始化气压计 */
    if(!Bmp280Init()) {
		/* IMU初始化失败*/
    	TtsSpeak(VOICE_AIR_PRES_ABN, FALSE);
		configASSERT(FALSE);
	}

	/* 初始化红外遥控器  TIM5频率1MHz, ARR==65535 */
	extern TIM_HandleTypeDef htim5;
	HAL_TIM_Base_Start_IT(&htim5);	 			/* 启动定时器中断，之所以没放在main.c里，是因为想在初始化代码之后再开始 */
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);  /* 启动定时器输入捕获，之所以没放在main.c里，是因为想在初始化代码之后再开始 */
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
	return fCurrent > 2.0f;		/* 如果限速了 推荐过流电流是1.0~1.2, 没限速推荐过流电流是1.2~1.5 */
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
#define DEBUG_PRINTF_FREQUENCY_10MS			5		/* Debug信息打印频率 单位10ms */
void WinClnDebug(void)
{
	static int i32DebugCnt = DEBUG_PRINTF_FREQUENCY_10MS;
	if(i32DebugCnt--) {
		return ;
	} else {
		i32DebugCnt = DEBUG_PRINTF_FREQUENCY_10MS;
	}
	printf("擦窗机运动状态: %s 角度: %.2f 目标角度: %.2f\n", aChWinClnSTStr[g_YKCCtr.WinClnST], g_YKCCtr.fAngle, g_YKCCtr.fAimedAngle);
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
	if((abs(g_YKCCtr.i8Timer_RightAct) > 10
		&& (fabs(g_YKCCtr.fRightDuty) > MAX_DUTY_RATIO - 0.03f))
	|| (abs(g_YKCCtr.i8Timer_LeftAct) > 10
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
//		printf("fGyrZ = %f, fJumpZ = %.5f, g_YKCCtr.fGyrZIntegral = %.5f g_YKCCtr.fMotorDutyAim_R2L = %.5f\n", fGyrZ, fJumpZ, g_YKCCtr.fGyrZIntegral, fAim_R2L);
//		printf("fGyrZ = %f, fJumpZ = %.5f, g_YKCCtr.fGyrZIntegral = %.5f ac = %.5f, ex = %.5f, fAim_R2L = %.5f\n", fGyrZ, fJumpZ, g_YKCCtr.fGyrZIntegral, g_YKCCtr.fAngle, g_YKCCtr.fAimedAngle, fAim_R2L);
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
	/* 右电机过流判断与运行电流记录 */
	if(abs(g_YKCCtr.i8Timer_RightAct) > 10) {	/* 电机运动稳定了再进行过流判断并保存正常行驶的电流 */
		if((fabs(g_YKCCtr.fRightDuty) > MAX_DUTY_RATIO - 0.03f)) {	/* 记录满占空比时的电流 */
			F32QueueEnter(&g_MsrRes.qRightMotorCurrHistory, g_MsrRes.fRightCur);		/* 电流入队 */
			float32 fK = GetF32QueueSlope(&g_MsrRes.qRightMotorCurrHistory, PERIOD_CTR_TASK_ms / 1000.0f, 10);
#if DEBUG_PRINT_CLN_STATE
			printf("right K = %.4f\n", fK);
#endif
			if(fK > MOTOR_SLOPE_THRESHOLD) {		/* 疑似堵转, 记录当前的最大电流值 */
				g_YKCCtr.fRightHitCurrent = F32QueueGetMaxElement(&g_MsrRes.qRightMotorCurrHistory);
				g_YKCCtr.u8Tmr_RightOverCurrent_10ms++;
			}
		}
	} else {	/* 可能是换方向了, 将当前记录的电流值清除, 重新记录 */
		F32QueueClear(&g_MsrRes.qRightMotorCurrHistory);
		g_YKCCtr.u8Tmr_RightOverCurrent_10ms = 0;
	}

	if(abs(g_YKCCtr.i8Timer_LeftAct) > 10) {
		/* 电流入队 */
		if((fabs(g_YKCCtr.fLeftDuty) > MAX_DUTY_RATIO - 0.03f)) {	/* 记录满占空比时的电流 */
			F32QueueEnter(&g_MsrRes.qLeftMotorCurrHistory, g_MsrRes.fLeftCur);
			float32 fK = GetF32QueueSlope(&g_MsrRes.qLeftMotorCurrHistory, PERIOD_CTR_TASK_ms / 1000.0f, 10);
#if DEBUG_PRINT_CLN_STATE
			printf("left K = %.4f\n", fK);
#endif
			if(fK > MOTOR_SLOPE_THRESHOLD) {
				g_YKCCtr.fLeftHitCurrent = F32QueueGetMaxElement(&g_MsrRes.qLeftMotorCurrHistory);
				g_YKCCtr.u8Tmr_LeftOverCurrent_10ms++;
			}
		}
	} else {
		F32QueueClear(&g_MsrRes.qLeftMotorCurrHistory);
		g_YKCCtr.u8Tmr_LeftOverCurrent_10ms = 0;
	}
}

void RunMdlCtr(void)		/* rename from RunYKC */
{
//	g_YKCCtr.bDisableCliff = FALSE;

//	ControlLed(0);
	int8 i;

	/* 计算值 */
	ProcDInFilter();		/* 开入滤波 */

//#if CLIFF_DEACTIVE
	if(g_YKCCtr.bDisableCliff) {
		g_Ctr.u32DinDat |= (1 << CLIFF_FRONT_LEFT_DInNo) | (1 << CLIFF_FRONT_RIGHT_DInNo) | (1 << CLIFF_REAR_LEFT_DInNo) | (1 << CLIFF_REAR_RIGHT_DInNo);
	}
//#endif
	g_MsrRes.fLastRightCur = g_MsrRes.fRightCur;
	g_MsrRes.fLastLeftCur = g_MsrRes.fLeftCur;
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
	/* 保护: 需要躲过刚启动、正反转切换的时候，另：履带电机堵转>=1.2A，正常工作电流0.5A */
	if(g_YKCCtr.fRightDuty > 0) {
		if(g_YKCCtr.i8Timer_RightAct < 0) {
			g_YKCCtr.i8Timer_RightAct = 1;
		} else if(g_YKCCtr.i8Timer_RightAct < 100) {
			g_YKCCtr.i8Timer_RightAct++;
		}
	} else if(g_YKCCtr.fRightDuty < 0) {
		if(g_YKCCtr.i8Timer_RightAct > 0) {
			g_YKCCtr.i8Timer_RightAct = -1;
		} else if(g_YKCCtr.i8Timer_RightAct > -100) {
			g_YKCCtr.i8Timer_RightAct--;
		}
	} else {
		g_YKCCtr.i8Timer_RightAct = 0;
	}
	if(g_YKCCtr.fLeftDuty > 0) {
		if(g_YKCCtr.i8Timer_LeftAct < 0) {
			g_YKCCtr.i8Timer_LeftAct = 1;
		} else if(g_YKCCtr.i8Timer_LeftAct < 100) {
			g_YKCCtr.i8Timer_LeftAct++;
		}
	} else if(g_YKCCtr.fLeftDuty < 0) {
		if(g_YKCCtr.i8Timer_LeftAct > 0) {
			g_YKCCtr.i8Timer_LeftAct = -1;
		} else if(g_YKCCtr.i8Timer_LeftAct > -100) {
			g_YKCCtr.i8Timer_LeftAct--;
		}
	} else {
		g_YKCCtr.i8Timer_LeftAct = 0;
	}

	/* 记录运行时最大电流 */
	if(g_MsrRes.fRightCur > g_MsrRes.fRightMaxI) {
		g_MsrRes.fRightMaxI = g_MsrRes.fRightCur;
	}
	if(g_MsrRes.fLeftCur > g_MsrRes.fLeftMaxI) {
		g_MsrRes.fLeftMaxI = g_MsrRes.fLeftCur;
	}

	UpdateWinClnCurr();
	UpdateWinClnGyrZ();

//	g_YKCCtr.bRightStall = ((g_YKCCtr.u8Tmr_GyrZStall_10ms > 10) || (g_YKCCtr.u8Tmr_RightOverCurrent_10ms > 3) || (IsMeetWheelStallCur(g_MsrRes.fRightCur)));
//	g_YKCCtr.bLeftStall = ((g_YKCCtr.u8Tmr_GyrZStall_10ms > 10) || (g_YKCCtr.u8Tmr_LeftOverCurrent_10ms > 3) || (IsMeetWheelStallCur(g_MsrRes.fLeftCur)));

//	g_YKCCtr.bRightStall = ((g_YKCCtr.u8Tmr_GyrZStall_10ms > 10) || (IsMeetWheelStallCur(g_MsrRes.fRightCur)));
//	g_YKCCtr.bLeftStall = ((g_YKCCtr.u8Tmr_GyrZStall_10ms > 10) || (IsMeetWheelStallCur(g_MsrRes.fLeftCur)));
	g_YKCCtr.bRightStall = FALSE;
	g_YKCCtr.bLeftStall = FALSE;

	/* 角速度瞬时碰撞判断堵转暂时不用，陀螺仪还是有点问题 */
//	g_YKCCtr.bRightStall = (g_YKCCtr.bGyrZIsStall || (g_YKCCtr.u8Tmr_GyrZStall_10ms > 10) || (g_YKCCtr.u8Tmr_RightOverCurrent_10ms > 2));
//	g_YKCCtr.bLeftStall = (g_YKCCtr.bGyrZIsStall || (g_YKCCtr.u8Tmr_GyrZStall_10ms > 10) || (g_YKCCtr.u8Tmr_LeftOverCurrent_10ms > 2));

//	BOOL bCurOv = InverseTimeVerify(ABN_T_INV_MOTOR_RIGHT_No, 0, 3, 0.05, g_MsrRes.fRightCur, 1, MSG_NULL)
//				|| InverseTimeVerify(ABN_T_INV_MOTOR_LEFT_No, 0, 3, 0.05, g_MsrRes.fLeftCur, 1, MSG_NULL);
	
	
	/* 开入定时：悬崖开关，释放认为是信号 */
	for(i = MAX_DIN_NUM - 1; i > HIT_REAR_RIGHT_DInNo; i--) {
		if(g_Ctr.u32DinDat & (1<<i)) {		/* 悬崖开关处于压缩状态 */
			g_YKCCtr.uTimer_DInSig_10ms[i] = 0;
		} else if(g_YKCCtr.uTimer_DInSig_10ms[i] < 0xFFFF) {	/* 悬崖开关处于释放状态 */
			g_YKCCtr.uTimer_DInSig_10ms[i]++;
		}
	}
	/* 开入定时：其他开关，按下认为是信号 */
	for(i = HIT_REAR_RIGHT_DInNo; i >= 0; i--) {
		if((g_Ctr.u32DinDat & (1<<i)) == 0) {
			g_YKCCtr.uTimer_DInSig_10ms[i] = 0;
		} else if(g_YKCCtr.uTimer_DInSig_10ms[i] < 0xFFFF) {
			g_YKCCtr.uTimer_DInSig_10ms[i]++;
		}
	}
#if DEBUG_PRINT_CLN_STATE
	WinClnDebug();
#endif
#define SINGLE_SIG_VALID_TIME_10ms 10
	/* 刷新行程 */
//	printf("\n左：Duty = %.3f, Cnt = %d\n右：Duty = %.3f, Cnt = %d\n\n", g_YKCCtr.fLeftDuty, g_MsrRes.uLeftCount, g_YKCCtr.fRightDuty, g_MsrRes.uRightCount);
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

//	printf("fAngle = %.5f\n", fAngle);

	/* 观察运动目标完成情况：行进 */
	int32 i32Forward = g_YKCCtr.i32Forward;
	/* 行进目标为0(CLN_ST_J也是没有明确的行进目标)，则看角度是否扫过目标--这个地方，逻辑尚需进一步完善 */
	if((i32Forward == 0) || (g_YKCCtr.WinClnST == CLN_ST_JTURN)) {
		if(((g_YKCCtr.fAngle < g_YKCCtr.fAimedAngle) && (g_YKCCtr.fAimedAngle < fAngle))
			|| ((g_YKCCtr.fAngle > g_YKCCtr.fAimedAngle) && (g_YKCCtr.fAimedAngle > fAngle)))
		{
			g_YKCCtr.fAimedAngle = fAngle;
		}
	} else {
		i32Forward -= F32ToI32(fTrip);
		if(((g_YKCCtr.i32Forward > 0) && (i32Forward < 0))	/* 行进过头--不能恰好到0 */
			|| ((g_YKCCtr.i32Forward < 0) && (i32Forward > 0)))
		{
			i32Forward = 0;
			g_YKCCtr.fAimedAngle = fAngle;	/* 行进到位，不必管角度(哪怕角度略有偏差)，都认为完成当前任务 */
		} else if((g_YKCCtr.WinClnST == CLN_ST_CTURN)	/* CTurn转角到位也是可以的 */
			&& (((g_YKCCtr.fAngle < g_YKCCtr.fAimedAngle) && (g_YKCCtr.fAimedAngle < fAngle))
				|| ((g_YKCCtr.fAngle > g_YKCCtr.fAimedAngle) && (g_YKCCtr.fAimedAngle > fAngle))))
		{
			i32Forward = 0;
			g_YKCCtr.fAimedAngle = fAngle;
		}
	}
	g_YKCCtr.fAngle = fAngle;
	g_YKCCtr.i32Forward = i32Forward;

	/* 维护定时器 */
	if(g_YKCCtr.uTimer_WinClnST < 0xFFFF) {
		g_YKCCtr.uTimer_WinClnST++;
	}

	/* 启动、停止 */
	if(g_YKCCtr.uTimer_DInSig_10ms[START_STOP_CMD_DInNo] == 1) {	/* 按钮被按下 */
		if(g_YKCCtr.WinClnST == CLN_ST_IDLE) {
			g_YKCCtr.u8DebugNo++;
			g_YKCCtr.WinClnST = CLN_ST_HANG;
			TtsSpeak(VOICE_WELCOM, FALSE);		/* 欢迎使用 */
			//g_YKCCtr.WinClnST = CLN_ST_TEST;
		} else if(g_YKCCtr.WinClnST == CLN_ST_STOP) {
			g_YKCCtr.WinClnST = g_YKCCtr.WinClnST_beforeSTOP;
		} else {
			g_YKCCtr.WinClnST_beforeSTOP = g_YKCCtr.WinClnST;
			g_YKCCtr.WinClnST = CLN_ST_STOP;
		}
	} else if(g_YKCCtr.uTimer_DInSig_10ms[START_STOP_CMD_DInNo] > 250) {	/* 按钮持续了2.5秒 */
		if(g_YKCCtr.WinClnST != CLN_ST_IDLE) {
			TtsSpeak(VOICE_BYEBYE, FALSE);
			g_YKCCtr.WinClnST = CLN_ST_IDLE;
			g_YKCCtr.u8Tmr_PumpFullSpeed_10ms = PUMP_TICK_TO_FULL_SPEED;
		}
	}

	if(WinClnCompleteHang() && g_YKCCtr.WinClnST > CLN_ST_END) {	/* 擦窗机完全悬空 可能意外掉落 */
		g_YKCCtr.WinClnST = CLN_ST_IDLE;
	}

	/* 运动决策 */
	switch(g_YKCCtr.WinClnST) {
		case CLN_ST_IDLE:			/* 空闲:真空泵断电 */
			ControlLed(LED_COLOR_BLUE);
			g_YKCCtr.u8ClnSeqNo = 0;
			break;

		case CLN_ST_STOP:			/* 暂停:真空泵带电，需要保持吸力 */
			ControlLed(LED_COLOR_YELLOW);
			break;

		case CLN_ST_END:			/* 清洁结束:真空泵带电，需要保持吸力 */
//			ControlLed(LED_COLOR_GREEN);
			TtsSpeak(VOICE_FINISHCLEAN, FALSE);
			g_YKCCtr.u8ClnSeqNo = 0;
			//g_YKCCtr.fAimedAngle = 0;
			//g_YKCCtr.i32Forward = 0;
			break;

		case CLN_ST_HANG:			/* 悬空:真空泵上电，但是还没有贴到窗台上 */
			ControlLed(LED_COLOR_RED);
			if((g_YKCCtr.uTimer_DInSig_10ms[START_STOP_CMD_DInNo] == 0) && (g_YKCCtr.u8Tmr_PumpFullSpeed_10ms == 0)) { 		/* 放开开关按钮 */
//				if(g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]			/* 四个角贴上 */
//					&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo]
//					&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]
//					&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo])
				if(WinClnSectionHang()) {		/* 至少有一个脚悬空就继续等待 */
					TtsSpeak(VOICE_CLIFF_DETECTED, FALSE);
				} else {
					NextWinClnSTBySeq(CLN_SEQ_START_No);
//					NextWinClnSTBySeq(CLN_SEQ_WIN_BTM_R2L_No);
				}
			}
			break;
		
		case CLN_ST_JUMP:		/* 循环跳转 */
			NextWinClnSTBySeq(g_YKCCtr.u8ClnSeqNo + cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward);
			break;

		case CLN_ST_BLANK:		/* 空白 */
			ControlLed(LED_COLOR_CRAN);
			NextWinClnSTBySeq(0);
			break;

		case CLN_ST_TEST:
			//NextWinClnSTBySeq(CLN_SEQ_TEST_No);
			break;

		case CLN_ST_DRIVE:		/* 直线+旋转 */
			ControlLed(LED_COLOR_CRAN);
			if(((g_YKCCtr.i32Forward == 0) && (g_YKCCtr.fAimedAngle == g_YKCCtr.fAngle))
				|| ((g_YKCCtr.i32Forward > 0)
					&& ((g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| (g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| ((g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo] || g_YKCCtr.bLeftStall)
							&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo] || g_YKCCtr.bRightStall))
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]
							&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo])))
				|| ((g_YKCCtr.i32Forward < 0)
					&& ((g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| (g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| ((g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo] || g_YKCCtr.bLeftStall)
							&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo] || g_YKCCtr.bRightStall))
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]
							&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo]))))
			{
				NextWinClnSTBySeq(0);
			}
			break;

		case CLN_ST_JTURN:		/* J弯 (直角转弯) */
			ControlLed(LED_COLOR_CRAN);
			if((fabsf(g_YKCCtr.fAimedAngle - g_YKCCtr.fAngle) < 5)
				|| ((cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward > 0)
					&& (((g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo] || g_YKCCtr.bLeftStall)
							&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo] || g_YKCCtr.bRightStall))
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]
							&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo])))
				|| ((cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward < 0)
					&& (((g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo] || g_YKCCtr.bLeftStall)
							&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo] || g_YKCCtr.bRightStall))
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]
							&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo]))))
			{
				NextWinClnSTBySeq(0);
			}
			break;

		case CLN_ST_CTURN:		/* S弯 (倒U型弯) */
			ControlLed(LED_COLOR_CRAN);
			/* 到底了：前进的话，左前触碰到边沿；后退的话，左后触碰到边沿 */
			if((cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward > 0)
				&& (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]
					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_FRONT_DInNo]))
			{
				NextWinClnSTBySeq(CLN_SEQ_WIN_BTM_R2L_No);
			} else if((cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward < 0)
				&& (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]
					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_REAR_DInNo]))
			{
				NextWinClnSTBySeq(CLN_SEQ_WIN_BTM_L2R_No);
			} else if(g_YKCCtr.fAimedAngle == g_YKCCtr.fAngle) {
				NextWinClnSTBySeq(0);
			}
			break;

		case CLN_ST_EXP_UP_FORWARD_R:	/* 探索上沿靠右前进 */
			if(g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]		/* 触碰到边沿 */
				|| g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo]
				|| ((g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo] || g_YKCCtr.bLeftStall)
					&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo] || g_YKCCtr.bRightStall)))
			{
				NextWinClnSTBySeq(0);
			} else if(g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_FRONT_DInNo] > 5) { /* 查看侧右前是否发生碰撞，说明接近靠近了上沿，开始顶部清洁. */
				NextWinClnSTBySeq(11);
			} else {	/* 没有碰到边沿，并且没有靠近边沿，并且角度没有偏离太狠就加大角度继续前进 */
				g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle - 10;
			}
			break;
		case CLN_ST_EXP_UP_BACK_R:	/* 探索上沿靠右后退 */
			if(g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]	/* 触碰到边沿 */
				|| g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo]
				|| ((g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo] || g_YKCCtr.bLeftStall)
					&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo] || g_YKCCtr.bRightStall)))
			{
				NextWinClnSTBySeq(0);
			}  else if(g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_REAR_DInNo] > 5) { /* 查看侧右后是否发生碰撞，说明接近靠近了上沿，开始顶部清洁. */
				NextWinClnSTBySeq(12);
			} else {	/* 没有碰到边沿，并且没有靠近边沿，加大角度继续前进 */
				g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle + 10;
			}
			break;

		case CLN_ST_EXP_FORWARD_L:	/* 靠左前行，左边需要被不断触发 */
		case CLN_ST_EXP_FORWARD_R:	/* 靠右前行，右边需要被不断触发 */
			ControlLed(LED_COLOR_CRAN);
			/* 保护性编程，不应该走到这步 */
			if(g_YKCCtr.u8ClnSeqNo >= sizeof(cnst_ClnSTSeq)/sizeof(CLN_ST_SEQ)) {
				/* 到头: 悬崖传感器都悬空，或者前面碰撞传感器都碰到 */
			} else if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]	/* 触碰到边沿 */
						&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo])
					|| ((g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo] || g_YKCCtr.bLeftStall)
						&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo] || g_YKCCtr.bRightStall)))
					//|| (g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
					//|| (g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms))
			{
				NextWinClnSTBySeq(0);
			} else if(g_YKCCtr.WinClnST == CLN_ST_EXP_FORWARD_L) {	/* 靠左前行，左边需要被不断触发 */
				/* 左前碰到，朝右拐 */
//				if(g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]
//					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_FRONT_DInNo])
//				{
//					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle - 5;
////					g_YKCCtr.i32Forward = 1000;
//				/* 后部也没有碰到，再左拐 */
//				} else if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo] == 0)
//						&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_REAR_DInNo] == 0))
//				{
//					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle + 5;
////					g_YKCCtr.i32Forward = 1000;
//				} else {
////					g_YKCCtr.i32Forward = 1000;
//				}
				/* 左前碰到，朝右拐 */
				if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo] > 10)
					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_FRONT_DInNo])
				{
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle - 5;
//					g_YKCCtr.i32Forward = 1000;
				/*  */
				} else {	/* 左前没碰到，朝左拐 */
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle + 5;
				}
			} else {	/* 靠右前行，右边需要被不断触发 */
//				/* 右前碰到，朝左拐 */
//				if(g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo]
//					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_FRONT_DInNo])
//				{
//					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle + 5;
////					g_YKCCtr.i32Forward = 1000;
//				/* 后部也没有碰到，再右拐 */
//				} else if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo] == 0)
//						&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_REAR_DInNo] == 0))
//				{
//					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle - 5;
////					g_YKCCtr.i32Forward = 1000;
//				} else {
////					g_YKCCtr.i32Forward = 1000;
//				}
				/* 右前碰到，朝左拐 */
				if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo] > 10)
					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_FRONT_DInNo])
				{
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle + 5;
//					g_YKCCtr.i32Forward = 1000;
					/* 右前没碰到，朝右拐 */
				}  else {
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle - 5;
				}
			}
			break;

		case CLN_ST_EXP_BACK_L:	/* 靠左退行，左边需要被不断触发 */
		case CLN_ST_EXP_BACK_R:	/* 靠右退行，右边需要被不断触发 */
			ControlLed(LED_COLOR_CRAN);
			/* 保护性编程，不应该走到这步 */
			if(g_YKCCtr.u8ClnSeqNo >= sizeof(cnst_ClnSTSeq)/sizeof(CLN_ST_SEQ)) {
				/* 到头: 悬崖传感器都悬空，或者前面碰撞传感器都碰到 */
			} else if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]	/* 触碰到边沿 */
						&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo])
					|| ((g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo] || g_YKCCtr.bLeftStall)
						&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo] || g_YKCCtr.bRightStall)))
					//|| (g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
					//|| (g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms))
			{
				NextWinClnSTBySeq(0);
			} else if(g_YKCCtr.WinClnST == CLN_ST_EXP_BACK_L) {/* 靠左退行，左边需要被不断触发 */
				/* 左后碰到，朝右拐 */
//				if(g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]	/* 左后悬空或者侧左后碰撞 */
//					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_REAR_DInNo])
//				{
//					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle + 5;
////					g_YKCCtr.i32Forward = -1000;
//				/* 前部也没有碰到，再左拐 */
//				} else if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo] == 0)	/* 左前不悬空并且侧左前不碰撞 */
//						&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_FRONT_DInNo] == 0))
//				{
//					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle - 5;
////					g_YKCCtr.i32Forward = -1000;
//				} else {	/* 否则，朝左拐 */
////					g_YKCCtr.i32Forward = -1000;
//				}
				if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo] > 10)	/* 左后悬空或者侧左后碰撞 */
					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_REAR_DInNo])
				{
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle + 5;
//					g_YKCCtr.i32Forward = -1000;
				/* 前部也没有碰到，再左拐 */
				}  else {	/* 否则，朝左拐 */
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle - 5;
				}
			} else {	/* 靠右退行，右边需要被不断触发 */
//				/* 右后碰到，朝左拐 */
//				if(g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo]
//					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_REAR_DInNo])
//				{
//					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle - 5;
////					g_YKCCtr.i32Forward = -1000;
//				/* 前部也没有碰到，再右拐 */
//				} else if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo] == 0)
//						&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_FRONT_DInNo] == 0))
//				{
//					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle + 5;
////					g_YKCCtr.i32Forward = -1000;
//				} else {
////					g_YKCCtr.i32Forward = -1000;
//				}
				/* 右后碰到，朝左拐 */
				if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo] > 10)
					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_REAR_DInNo])
				{
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle - 5;
//					g_YKCCtr.i32Forward = -1000;
				/* 前部也没有碰到，再右拐 */
				}  else {
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle + 5;
				}
			}
			break;

		default:
			g_YKCCtr.WinClnST = CLN_ST_STOP;
			break;
	}

	/* 临时性保护代码，避免到了边沿还继续 */
	if(((g_Ctr.u32DinDat & (1<<CLIFF_FRONT_LEFT_DInNo)) == 0)	/* 四面全悬空，应该是反过来了 */
		&& ((g_Ctr.u32DinDat & (1<<CLIFF_FRONT_RIGHT_DInNo)) == 0)
		&& ((g_Ctr.u32DinDat & (1<<CLIFF_REAR_LEFT_DInNo)) == 0)
		&& ((g_Ctr.u32DinDat & (1<<CLIFF_REAR_RIGHT_DInNo)) == 0))
	{		
	} else if(((g_YKCCtr.i32Forward > 0)	/* 前进 */
		&& ((((g_Ctr.u32DinDat & (1<<CLIFF_FRONT_LEFT_DInNo)) == 0)	/* 触碰到边沿 */
				&& ((g_Ctr.u32DinDat & (1<<CLIFF_FRONT_RIGHT_DInNo)) == 0))
			|| ((g_Ctr.u32DinDat & (1<<HIT_FRONT_LEFT_DInNo))
				&& (g_Ctr.u32DinDat & (1<<HIT_FRONT_RIGHT_DInNo)))))
	|| ((g_YKCCtr.i32Forward < 0)	/* 后退 */
		&& ((((g_Ctr.u32DinDat & (1<<CLIFF_REAR_LEFT_DInNo)) == 0)	/* 触碰到边沿 */
				&& ((g_Ctr.u32DinDat & (1<<CLIFF_REAR_RIGHT_DInNo)) == 0))
			|| ((g_Ctr.u32DinDat & (1<<HIT_REAR_LEFT_DInNo))
				&& (g_Ctr.u32DinDat & (1<<HIT_REAR_RIGHT_DInNo))))))
	{
//		g_YKCCtr.i32Forward = 0;
		//g_YKCCtr.WinClnST = CLN_ST_STOP;
	}

	/* 运动执行:依据运动目标，解算当前步骤所需要的履带动作 */
	float32 fRightTrip = 0;
	float32 fLeftTrip = 0;
	float32 fd = 0;
	if(g_YKCCtr.WinClnST < CLN_ST_DRIVE) {
		g_YKCCtr.fRightDuty = 0;
		g_YKCCtr.fLeftDuty = 0;
	} else {
		float32 fForward;
		if(g_YKCCtr.WinClnST == CLN_ST_JTURN) {
			/* d/D = 2*tan(A)*H/(L - tan(A)*(L-H))
			   d: 两条履带差速
			   D: 外侧履带速度(全速)
			   A: 机器人与基线(远离/靠近)的夹角
			   H: 履带间距
			   L: 车身长度		 */
			/* 求机身与基线线夹角 */
			float32 fAngle = fabsf(g_YKCCtr.fAngle - 90);	/* 基准线是水平方向(90度) */
			/* 前进方向侧面限位开关动作需要特别处理：要不然会把前进方向的行程开关往后拉，造成前面两个碰撞开关虚假信号 */
			if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward > 0) {
				if(g_YKCCtr.fAimedAngle > g_YKCCtr.fAngle) {/* 前进左弯:注意右前不要发生剐蹭，如果发生，需要加大转弯角度 */
					if(g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_FRONT_DInNo]) {
						fAngle += 10;
					}
				} else {	/* 前进右弯:注意左前不要发生剐蹭，如果发生，需要加大转弯角度 */
					if(g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_FRONT_DInNo]) {
						fAngle += 10;
					}
				}
			} else {
				if(g_YKCCtr.fAimedAngle < g_YKCCtr.fAngle) {/* 后退左弯:注意右后不要发生剐蹭，如果发生，需要加大转弯角度 */
					if(g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_REAR_DInNo]) {
						fAngle += 10;
					}
				} else {	/* 后退右弯:注意左后不要发生剐蹭，如果发生，需要加大转弯角度 */
					if(g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_REAR_DInNo]) {
						fAngle += 10;
					}
				}
			}

			/* 计算两轮差速 */
			float32 fTanA = tanf(fAngle*PI/180);
			fd = 2*fTanA*TRACK_DISTANCE_mm/(MCH_LENGTH_mm - fTanA*(MCH_LENGTH_mm - TRACK_DISTANCE_mm));
			g_YKCCtr.u8DebugNo = 2;
			if(fd > 1) {
				fd = 1;
			} else if(fd < 0.1f*g_YKCCtr.u8DebugNo) {
				fd = 0.1f*g_YKCCtr.u8DebugNo;
			}

			/* 前进左弯 */
			fForward = 2000;	/* 1秒约2K个脉冲，按照0.1秒预估 */
			if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward < 0) {
				fForward = 0 - fForward;
			}
			/* 左弯 */
			if((cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward > 0) ^ (g_YKCCtr.fAimedAngle < g_YKCCtr.fAngle)) {
				fRightTrip = fForward;
				fLeftTrip = fForward*(1 - fd);
			} else {	/* 右弯 */
				fRightTrip = fForward*(1 - fd);
				fLeftTrip = fForward;
			}
		} else {
			float32 fTurnAngle = g_YKCCtr.fAimedAngle - g_YKCCtr.fAngle;
			if(fTurnAngle > 180) {
				fTurnAngle -= 360;
			} else if(fTurnAngle < -180) {
				fTurnAngle += 360;
			}
			/* 计算左右行程，举例：30度对应是1627个计数 */
			float32 fTripErr = (TRACK_DISTANCE_pulse/2)*fTurnAngle*(PI/180.0f);

			fForward = fabsf(g_YKCCtr.i32Forward);
			/* 防止转角过大、过小：转弯一般是S弯，4000转30度(1627计数)，fabsf(fForward/fTripErr) = 2.5 */
			if(fForward > 10*fabsf(fTripErr)) {	/* 因为g_YKCCtr.i32Forward可能是1e10，导致角度过大 */
				fForward = 10*fabsf(fTripErr);
			} else if(g_YKCCtr.i32Forward && (fForward < 1.25*fabsf(fTripErr))) {	
				fForward = 1.25*fabsf(fTripErr);
			}
			if(g_YKCCtr.i32Forward < 0) {	/* 恢复符号 */
				fForward = 0 - fForward;
			}
			fRightTrip = fForward + fTripErr;
			fLeftTrip = fForward - fTripErr;
		}

		/* 计算速度:速度比值 */
		float32 fRightDuty, fLeftDuty;
		/* 某一个履带和总体方向不一致，大概率是行程最后一点点，测量角度发生了抖动，为了避免机器人发生抖动，避免与前进方向不一致的履带方向 */
		if((fabsf(fRightTrip) < fabsf(fLeftTrip)*0.01)
			|| ((fRightTrip > 0) && (fForward < 0))
			|| ((fRightTrip < 0) && (fForward > 0))
			|| ((g_YKCCtr.WinClnST != CLN_ST_JTURN)
				&& (((g_YKCCtr.i32Forward > 0) && g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo])
					|| ((g_YKCCtr.i32Forward < 0) && g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo]))))
		{
			fRightDuty = 0;
			fLeftDuty = 1;
		} else if((fabsf(fLeftTrip) < fabsf(fRightTrip)*0.01)
			|| ((fLeftTrip > 0) && (fForward < 0))
			|| ((fLeftTrip < 0) && (fForward > 0))
			|| ((g_YKCCtr.WinClnST != CLN_ST_JTURN)
				&& (((g_YKCCtr.i32Forward > 0) && g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo])
					|| ((g_YKCCtr.i32Forward < 0) && g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo]))))
		{
			fRightDuty = 1;
			fLeftDuty = 0;
		} else {	/* 计算比值 */
			float32 fAim_R2L = fabsf(fRightTrip/fLeftTrip);
			float32 fPID_P = 0;
			float32 fPID_I = 0;
			if((g_YKCCtr.u8ClnSeqNo != g_YKCCtr.u8ClnSeqNo_last)
				|| (g_MsrRes.fLeftFreq == 0) || (g_MsrRes.fRightFreq == 0)
				|| (g_YKCCtr.fLeftDuty == 0) || (g_YKCCtr.fRightDuty == 0))
			{
				fAim_R2L *= g_YKCCtr.fDragConfR2L;
				g_YKCCtr.fVr2lAimErr_last = 0;
			} else {
				float32 fVr2lAimErr = fAim_R2L - g_MsrRes.fRightFreq/g_MsrRes.fLeftFreq;
				fPID_I = fVr2lAimErr*0.1;
				fPID_P = (fVr2lAimErr - g_YKCCtr.fVr2lAimErr_last) * VACUUM_AIM_ERR_TOLERANCE;
				fAim_R2L = fabsf(g_YKCCtr.fRightDuty/g_YKCCtr.fLeftDuty) + fPID_P + fPID_I;
				g_YKCCtr.fVr2lAimErr_last = fVr2lAimErr;
			}
			if(fAim_R2L > 10) {	/* 限幅 */
				fAim_R2L = 10;
			} else if(fAim_R2L < 0) {
				fAim_R2L = 0;
			}
			if(fAim_R2L > 1) {
//				fRightDuty = 1;
//				fLeftDuty = 1/fAim_R2L;
				fRightDuty = MAX_DUTY_RATIO;
				fLeftDuty = MAX_DUTY_RATIO / fAim_R2L;
			} else {
//				fRightDuty = fAim_R2L;
//				fLeftDuty = 1;
				fLeftDuty = MAX_DUTY_RATIO;
				fRightDuty = fAim_R2L * MAX_DUTY_RATIO;
			}
		}
		/* 计算速度:调整符号 */
		if(fRightTrip < 0) {
			fRightDuty = 0 - fRightDuty;
		}
		if(fLeftTrip < 0) {
			fLeftDuty = 0 - fLeftDuty;
		}

		/* ========== 新增占空比限幅处理 ========== */
		// 保持转向比例关系的同时限制最大幅值
//		float32 maxDuty = fmaxf(fabsf(fRightDuty), fabsf(fLeftDuty));
//		if(maxDuty > MAX_DUTY_RATIO) {
//		    float32 scale = MAX_DUTY_RATIO / maxDuty;
//		    fRightDuty *= scale;
//		    fLeftDuty *= scale;
//		}
		/* ========== 限幅结束 ========== */

		g_YKCCtr.fRightDuty = fRightDuty;
		g_YKCCtr.fLeftDuty = fLeftDuty;
	}
	
	/* 响应遥控器干预 */
	UpdateIRCtrl();
	if(g_IRCtrl.u8TryCnt) {
		switch(g_IRCtrl.u8BtnPressing) {
		case IR_BTN_0:
//			Board_DrvPump(0.1f);
			NextWinClnSTBySeq(CLN_SEQ_START_No);
			break;
		case IR_BTN_1:
			NextWinClnSTBySeq(CLN_SEQ_WIN_BTM_R2L_No);
//			Board_DrvPump(0.3f);
			break;
		case IR_BTN_2:
//			Board_DrvPump(0.5f);
			NextWinClnSTBySeq(CLN_SEQ_WIN_BTM_L2R_No);
			break;
		case IR_BTN_3:
//			Board_DrvPump(1.0f);
			NextWinClnSTBySeq(CLN_SEQ_TEST_No);
			break;
		case IR_BTN_4:
			NextWinClnSTBySeq(CLN_SEQ_TEST3_START_No);
			break;
		case IR_BTN_5:
			break;
		case IR_BTN_6:
			break;
		case IR_BTN_7:
			NextWinClnSTBySeq(4);
			break;
		case IR_BTN_8:
			g_YKCCtr.bDisableCliff = FALSE;
			break;
		case IR_BTN_9:
			g_YKCCtr.bDisableCliff = TRUE;
			break;
		case IR_BTN_START:
			/* 左边喷水 */
			GPIO_write(TOP_WATER_JET_RelayNo, 1);
			if(g_YKCCtr.i8Tmr_Spray_Front_10ms == -1) {
				g_YKCCtr.i8Tmr_Spray_Front_10ms = SPRAY_TIMEOUT_10ms;
			}
			break;
		case IR_BTN_SHARP:
			/* 右边喷水 */
			GPIO_write(BOT_WATER_JET_RelayNo, 1);
			if(g_YKCCtr.i8Tmr_Spray_Back_10ms == -1) {
				g_YKCCtr.i8Tmr_Spray_Back_10ms = SPRAY_TIMEOUT_10ms;
			}
			break;
		case IR_BTN_UP:
			g_YKCCtr.fLeftDuty = MAX_DUTY_RATIO;
			g_YKCCtr.fRightDuty = MAX_DUTY_RATIO;
			break;
		case IR_BTN_DOWN:
			g_YKCCtr.fLeftDuty = -MAX_DUTY_RATIO;
			g_YKCCtr.fRightDuty = -MAX_DUTY_RATIO;
			break;
		case IR_BTN_LEFT:
//			g_YKCCtr.fLeftDuty = 1;
//			g_YKCCtr.fRightDuty = -1;
			g_YKCCtr.fLeftDuty = -MAX_DUTY_RATIO;
			g_YKCCtr.fRightDuty = MAX_DUTY_RATIO;
			break;
		case IR_BTN_RIGHT:
//			g_YKCCtr.fLeftDuty = -1;
//			g_YKCCtr.fRightDuty = 1;
			g_YKCCtr.fLeftDuty = MAX_DUTY_RATIO;
			g_YKCCtr.fRightDuty = -MAX_DUTY_RATIO;
			break;
		case IR_BTN_OK:
			g_YKCCtr.fLeftDuty = 0;
			g_YKCCtr.fRightDuty = 0;
			g_YKCCtr.WinClnST = CLN_ST_IDLE;
			break;
		}
	}

	//测试用：如果上碰到：黄色
	//测试用：如果下碰到：绿色
	//测试用：如果左碰到：蓝色
	//测试用：如果右碰到：青色
	//测试用：如果左卡住：粉色
	//测试用：如果右卡住：红色
#if 0
	if((g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
			&& g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
	{
		ControlLed(LED_COLOR_YELLOW);
	} else if((g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
			&& g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
	{
		ControlLed(LED_COLOR_GREEN);
	} else if((g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_FRONT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
			&& g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_REAR_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
	{
		ControlLed(LED_COLOR_BLUE);
	} else if((g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_FRONT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
			&& g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_REAR_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
	{
		ControlLed(LED_COLOR_CRAN);
	} else if(g_YKCCtr.bLeftStall) {
		ControlLed(LED_COLOR_PINK);
	} else if(g_YKCCtr.bRightStall) {
		ControlLed(LED_COLOR_RED);
	}
#endif
	Board_DrvWheel(g_YKCCtr.fRightDuty, g_YKCCtr.fLeftDuty);	/* 执行 */

	/* 真空泵 */
	float32 fPumpPID_P, fPumpPID_I;
	if(g_YKCCtr.WinClnST == CLN_ST_IDLE) {
//		if(g_IRCtrl.u8TryCnt) {
//			if(g_IRCtrl.u8BtnPressing == IR_BTN_5) {
//				g_YKCCtr.fPumpDuty += 0.1f;
//				if(g_YKCCtr.fPumpDuty > 1.0f) {
//					g_YKCCtr.fPumpDuty = 1.0f;
//				}
//				g_IRCtrl.u8TryCnt = 0;
//			} else if(g_IRCtrl.u8BtnPressing == IR_BTN_6) {
//				g_YKCCtr.fPumpDuty -= 0.1f;
//				if(g_YKCCtr.fPumpDuty < 0.0f) {
//					g_YKCCtr.fPumpDuty = 0.0f;
//				}
//				g_IRCtrl.u8TryCnt = 0;
//			}
//		} else {
//			g_YKCCtr.fPumpDuty = PUMP_IDEL_DUTY;
//		}
		g_YKCCtr.fPumpDuty = PUMP_IDEL_DUTY;
		g_YKCCtr.fAirPressure_PumpIdle = g_MsrRes.fAirPressure;
	} else {
		if(g_YKCCtr.fPumpDuty == 0) {	/* 刚启动 */
			g_YKCCtr.fPumpDuty = 1;
		} else {
			float32 fVacuumAimErr = (g_MsrRes.fAirPressure + AIMED_AIR_P - g_YKCCtr.fAirPressure_PumpIdle)/AIMED_AIR_P;
			fPumpPID_P = fVacuumAimErr*0.02;
			fPumpPID_I = (fVacuumAimErr - g_YKCCtr.fVacuumAimErr_last)*0.01;
			g_YKCCtr.fPumpDuty += fPumpPID_P + fPumpPID_I;
			g_YKCCtr.fVacuumAimErr_last = fVacuumAimErr;
			if(g_YKCCtr.fPumpDuty > 1) {
				g_YKCCtr.fPumpDuty = 1;
			} else if(g_YKCCtr.fPumpDuty < 0) {
				g_YKCCtr.fPumpDuty = 0;
			}
			/* 如果吸力不足则进入CLN_ST_STOP */
			if((g_YKCCtr.u8Tmr_PumpFullSpeed_10ms == 0) && (g_YKCCtr.fVacuumAimErr_last >= VACUUM_AIM_ERR_TOLERANCE)) {
				if(g_YKCCtr.u8Tmr_VacummSuff_10ms > 0) {		/* 忽略偶尔的气压偏低 */
					g_YKCCtr.u8Tmr_VacummSuff_10ms--;
				} else {		/* 气压持续偏低 */
					if(g_YKCCtr.WinClnST != CLN_ST_STOP) {
					#if LOW_AIR_CLOSE_CLN
						g_YKCCtr.WinClnST_beforeSTOP = g_YKCCtr.WinClnST;
						g_YKCCtr.WinClnST = CLN_ST_STOP;
					#endif
					}
					TtsSpeak(VOICE_AIR_PRES_LOW, FALSE);
				}
			} else {
				g_YKCCtr.u8Tmr_VacummSuff_10ms = PUMP_TICK_VACUMM_SUFF;
			}
		}
		if(g_YKCCtr.u8Tmr_PumpFullSpeed_10ms) {
			g_YKCCtr.u8Tmr_PumpFullSpeed_10ms--;
		}
//		g_YKCCtr.fPumpDuty = 1.0f;
	}

#if PUMP_DEACTIVE
	Board_DrvPump(0);		/* 执行 */
#else
	Board_DrvPump(g_YKCCtr.fPumpDuty);		/* 执行 */
#endif

	uint32 u32Flag = g_Ctr.u32DinDat + (g_YKCCtr.bRightStall<<14) + (g_YKCCtr.bLeftStall<<15)
					 + (g_YKCCtr.WinClnST<<16) + (g_YKCCtr.u8ClnSeqNo<<24);
#if 0
	Rec_DebugPack(u32Flag, g_YKCCtr.fRightDuty, g_YKCCtr.fLeftDuty, g_YKCCtr.i32Forward, g_YKCCtr.fAimedAngle, 
				g_YKCCtr.fAngle, fRightTrip, fLeftTrip, g_MsrRes.fRightFreq, g_MsrRes.fLeftFreq, g_MsrRes.fPitch, fd);
#elif 1
	Rec_DebugPack(u32Flag, g_YKCCtr.fRightDuty, g_YKCCtr.fLeftDuty, g_YKCCtr.i32Forward, g_YKCCtr.fAimedAngle, 
				g_YKCCtr.fAngle, fRightTrip, fLeftTrip, g_MsrRes.fRightFreq, g_MsrRes.fLeftFreq, g_MsrRes.fRightCur, g_MsrRes.fLeftCur);

#elif 0
	Rec_DebugPack(u32Flag, g_MsrRes.fAirPressure, fPumpPID_P, fPumpPID_I, g_YKCCtr.fPumpDuty, g_MsrRes.fPumpFreq, 0, 0, 0, 0, 0, 0);
#else
	Rec_DebugPack(u32Flag, g_MsrRes.iAccX, g_MsrRes.iAccY, g_MsrRes.iAccZ, g_YKCCtr.fAngle, g_MsrRes.fPitch, g_MsrRes.fRoll, 0, 0, 0, 0, 0);
#endif

	/* 为计算喷水时机而统计 */
	if((cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Spray != SPRAY_INSTR_SPRAY_NONE)) {	/* 该脚本序号有喷水需求 */
		/* 刷新用于喷水间隔的行程累积量 */
		g_YKCCtr.i32TripCntForSpray -= ((abs(iLeftCount) + abs(iRightCount)) / 2);		/* 两轮步数均值 */
		if((g_YKCCtr.u8ClnSeqNo != g_YKCCtr.u8ClnSeqNo_last) || (g_YKCCtr.i32TripCntForSpray <= 0)) {		/* 刚切换了脚本序号或累积到了一定步数 */
			if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Spray == SPRAY_INSTR_SPRAY_FRONT) {
#if !CLOSE_SPRAY_WATER
				GPIO_write(TOP_WATER_JET_RelayNo, 1);
#endif
			} else if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Spray == SPRAY_INSTR_SPRAY_BACK) {
#if !CLOSE_SPRAY_WATER
				GPIO_write(BOT_WATER_JET_RelayNo, 1);
#endif
			}
			if(g_YKCCtr.i8Tmr_Spray_Front_10ms == -1) {
				g_YKCCtr.i8Tmr_Spray_Front_10ms = SPRAY_TIMEOUT_10ms;
			}
			if(g_YKCCtr.i8Tmr_Spray_Back_10ms == -1) {
				g_YKCCtr.i8Tmr_Spray_Back_10ms = SPRAY_TIMEOUT_10ms;
			}
			g_YKCCtr.i32TripCntForSpray = 15000;		/* 两次喷水间隔步数 */
		}
	}

	/* 更新前喷水开出状态， -1 未使用 / 0 该停了/ n 开始计时 */
	if(g_YKCCtr.i8Tmr_Spray_Front_10ms == 0) {
		GPIO_write(TOP_WATER_JET_RelayNo, 0);
		if(!GPIO_read(LOW_LIQUID_WRN_DInNo)) {
			TtsSpeak(VOICE_LIQUID_LOW, TRUE);
		}
		g_YKCCtr.i8Tmr_Spray_Front_10ms = -1;
	} else if(g_YKCCtr.i8Tmr_Spray_Front_10ms > 0) {
		g_YKCCtr.i8Tmr_Spray_Front_10ms--;
	}
	/* 更新后喷水电机状态， -1 未使用 / 0 该停了/ n 开始计时 */
	if(g_YKCCtr.i8Tmr_Spray_Back_10ms == 0) {
		GPIO_write(BOT_WATER_JET_RelayNo, 0);
		if(!GPIO_read(LOW_LIQUID_WRN_DInNo)) {
			TtsSpeak(VOICE_LIQUID_LOW, TRUE);
		}
		g_YKCCtr.i8Tmr_Spray_Back_10ms = -1;
	} else if(g_YKCCtr.i8Tmr_Spray_Back_10ms > 0) {
		g_YKCCtr.i8Tmr_Spray_Back_10ms--;
	}

	UpdateTTS();	/* 更新语音模块 */

	g_YKCCtr.u32DinDat_last = g_Ctr.u32DinDat;
	g_YKCCtr.u8ClnSeqNo_last = g_YKCCtr.u8ClnSeqNo;

	g_CodeTest.fVal[69] = g_YKCCtr.fAirPressure_PumpIdle - g_MsrRes.fAirPressure;
	g_CodeTest.fVal[70] = GPIO_read(TOP_WATER_JET_RelayNo);
	g_CodeTest.fVal[71] = GPIO_read(BOT_WATER_JET_RelayNo);
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
