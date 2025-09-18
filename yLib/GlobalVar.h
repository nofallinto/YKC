
/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved
 *				: 
 * File			: GlobalVar.h
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
/*===========================================================================
 * 产品型号与软件版本号定义，放在开始便于不同产品修改
 Device Type Code Definition:	产品代(4bit) + 主机(4bit) + 辅助功能机(8bit)
开发代
	0x0		Reserved
	0x1		CH based Controller
	0x2		YKF based Controller
	0x5		V5 产品线
*==========================================================================*/
#define V5_YKF2 				0x5102
#define V5_YKF3 				0x5103
#define V5_YKF4                 0x5104
#define V5_YKF5                 0x5105
#define V5_YKF6					0x5106
#define Is_YKF(DEVICE_TYPE)		((DEVICE_TYPE == V5_YKF2) || (DEVICE_TYPE == V5_YKF3) || (DEVICE_TYPE == V5_YKF4) || (DEVICE_TYPE == V5_YKF5) || (DEVICE_TYPE == V5_YKF6))
#define V5_YYT3 				0x5203
#define V5_YYT4 				0x5204
#define V5_YBU2 				0x5300
#define V5_YBU3 				0x5303
#define V5_YBU4 				0x5304
#define V5_YBT2 				0x5420
#define V5_YBT3 				0x5423
#define V5_YBT4 				0x5424
#define V5_YYB					0x5500
#define V5_YYG					0x5510
#define V5_YYD					0x5520
#define V5_YYD2					0x5522
#define YKC						0x7001

/***************************************************************************
 				exclude redefinition and c++ environment
***************************************************************************/
#ifndef _GLOBAL_VAR_H_
#define _GLOBAL_VAR_H_

/***************************************************************************
				hardware parameter and global definitions
***************************************************************************/
/*===========================================================================
 * 基本类型变量
 *==========================================================================*/
#ifndef CPU_DATA_TYPES
#define CPU_DATA_TYPES
	typedef unsigned char		BOOL;
	typedef signed char			triST;		/* 三态量，即有:负、零、正三种状态 */
	typedef signed char			int8;
	typedef unsigned char		uint8;
	typedef short 				int16;
	typedef unsigned short		uint16;
	typedef long				int32;
	typedef unsigned long		uint32;
	typedef long long			int64;
	typedef unsigned long long	uint64;
	typedef float				float32;
	typedef long double 		float64;
	typedef unsigned int		BITBOOL;
#endif

#ifndef TRUE
	#define TRUE 1
#endif
#ifndef FALSE
	#define FALSE 0
#endif

/*===========================================================================
 * 指令
 *==========================================================================*/
#define	 NOP			        do{asm (" nop");	}	while(0)
#define SECTION(sectionName) 	__attribute__((section(sectionName)))   /* 可用于指定全局变量和函数的段(section) */
#define __weak                  __attribute__((weak))					/* 晚绑定虚函数 */

/***************************************************************************
 		use EXT to distinguish stencil.c or others source files
***************************************************************************/
#ifdef _MDL_SYS_C_
#define EXT
#else
#define EXT extern
#endif

/***************************************************************************
 					公共变量
***************************************************************************/
/*===========================================================================
 * 常用结构体定义
 *==========================================================================*/
typedef struct {		/* 时间定义 */
	uint8 u8Year;								/* 1-99，如果该位为0则代表整个时间无效 */
	uint8 u8Month;								/* 1-12 */
	uint8 u8Day;								/* 1-31 */
	uint8 u8Hour;								/* 0-23 */
	uint8 u8Minute;								/* 0-59 */
	uint8 u8Second;								/* 0-59 */
	uint16 uMilliSec;							/* 0-999 */
}REAL_TIME_VAR;

typedef struct {		/* 温度传感器数据 */
	int16 iDegree_100mC;						/* 温度数据，单位0.1摄氏度 */
	uint8 u8PlaceNo;							/* 同一个位置的编码, 0:代表只有一个，无需序号; >0:序号 */
	int8 i8Placement;							/* 温度传感器位置，0:无温度传感器; >0:温度有效; <0:传感器断线    */
}TEMPERATURE_DATA;

/* 积量统计 */
typedef struct {
	int64 i64TotalCumulant;
	float32 fFragCumulant;			/* 一个时间片内的积量，用于计算平均值 */
	float32 fCumulantRem;			/* 积量的余量，以防止一个时间片的数据不够最低积量标准 */
}STAT_CUM_ITEM;

/* 计数统计 */
typedef struct {
	uint32 u32StatThr;
	uint32 u32StatVal;
}STAT_COUNT_ITEM;

typedef enum {	/* F32数据物理意义，用于表达不同的物理量，期望不同的显示形式 */
	F32_DEFAULT			= 0,	/* 采用默认的数据显示方式 */
	F32_SgnDigits_1		= 1,	/* 1~10为指定有效位 */
	F32_SgnDigits_10 	= 10,	/* 1~10为指定有效位 */
	F32_INT,	/* 整数, 无小数点. [-999,9999]:sabc,s是符号,    <-999:-999, >9999:9999 */
	F32_CENT,	/* 百分比, [0,100):ab.c%, [100,999]:abc%,    >999:999% */
	F32_TEMP,	/* 温度: [0,100):ab.c℃, [100,999):abc℃, >=999:999℃ */
	F32_WATER, 	/* 水位: [0,9.999]:a.bcdm, (9.999,99.99):ab.cdm, >99.99:99.99m */
    F32_MPa,    /* 压力: [0,9.999]:a.bcdMPa, (9.999,99.99]:ab.cdMPa, >99.99:99.99Mpa */
	F32_FREQ,	/* 频率:	 [0,99.99]:ab.cdHz, [100,999.9):abc.dHz, >=999.9:999.9Hz */
	F32_VOL,	/* 电压: [0,10):a.bcV, [10,100):ab.cV, [100,8e3):  abcdV, [8e3,1e4):a.bcKV, [1e4, 1e5):ab.cKV, >=1e5:abcKV */
	F32_CUR,	/* 电流: [0,10):a.bcA, [10,100):ab.cA, [100,8e3):  abcdA, [8e3,1e4):a.bcKA, [1e4, 1e5):ab.cKA, >=1e5:abcKA */
	F32_PwrP,	/* 有功功率:  [0,10):a.bcKW, [10,100):ab.cKW, [100,1000):  abcKW, [1000, 1e4):a.bcMW, [1e4, 1e5):ab.cMW, [1e5, 1e6):abcMW */
	F32_PwrQ,	/* 无功功率: 同有功功率，只是单位KVar, MVar */
	F32_COS,	/* 功率因数:[-1.1,1.1]:a.bcd, 除此之外，仅显示整数部分 */
/* 有/无功电度(中):[0,10):a.bc度, [10,1e3):abc.d度, [1e3,1e4):abcd度, [1e4,1e5)a.bc万度, [1e5,1e7):ab.c万度, [1e7,1e8):abcd万度, 最大9999亿度
   有功电度(英):[0,100):ab.cdKWH, [100,1e3):abc.dKWH, [1e3,1e5):ab.cdMWH, [1e5,1e6)abc.dMWH, 最大999.9TWH
   无功电度(英):[0,100):ab.cdKVarH, [100,1e3):abc.dKVarH, [1e3,1e5):ab.cdMVarH, [1e5,1e6)abc.dMVarH, 最大999.9TVarH */
	F32_EngP,
	F32_EngQ,	/* 无功电度:同无功电度 */
	F32_BatAH,	/* 蓄电池:[0,99.99]:ab.cdAH, [100,999.9):abc.dAH, >=999.9:999.9AH  */
/* 阻抗: [0,10):a.bcOhm, [10,100):ab.cOhm, [100,1e3):  abcOhm, [1e3,1e4):a.bcKOhm, [1e4, 1e5):ab.cKOhm, [1e5, 1e6):abcKOhm,
        [1e6,1e7):a.bcMOhm, [1e7,1e8):ab.cMOhm, [1e8,1e9):abcMOhm, >=1e9:999MOhm   */
    F32_OHM,
	F32_FlowRate,/* 流量: (-inf, 10]显示为 -ab.cdem3/s; (-10, 0)显示为-a.bcdem3/s; [0, 10)显示为a.bcdem3/s; [10, inf)显示为 ab.cdem3/s */
	F32_FlowVlct,/* 流速: (-inf, 10]显示为 -ab.cdem/s; (-10, 0)显示为-a.bcdem/s; [0, 10)显示为a.bcdem/s; [10, inf)显示为 ab.cdem/s */
	F32_TIME,	 /* 时间：(100H,1H]显示为 aH; (60min,1min]显示为 aMin; (60s,1s]显示为 as;*/
/* 电压变化率： [0,1mV):0.abcmV/m, [1mV,10mV):a.bcmV/m, [10mV,100mV):ab.cmV/m, [100mV,1V):abcmV/m, [1V,10V):a.bcV/m,
		[10V,100V):ab.cV/m, [100,1000):abcV/m, [10,inf):999V/m */
	F32_VpM
}F32_MEANING;

/*===========================================================================
 * 2. 数据显示、配置、事件消息、采集参数的数据格式定义
 *==========================================================================*/
typedef enum {			/* UI界面的页面 */
    /* 这几个页面，不同产品应该是一样的 */
	DATA_PAGE_NULL				= 0x00,             /* 空页面 */
	DATA_PAGE_CONF				= 0x01,				/* 配置页面 */
	DATA_PAGE_MSG				= 0x02,				/* 消息页面 */
	DATA_PAGE_ACQ				= 0x03,				/* 采集页面 */
    /* 以下页面，不同产品不一样 */
	DATA_PAGE_WELCOME			= 0x04, 			/* 欢迎页面 */
	DATA_PAGE_MAIN				= 0x05, 			/* 主页面 */
	DATA_PAGE_ASSIT 			= 0x06, 			/* 辅助页面 */
	DATA_PAGE_SYS				= 0x07, 			/* 系统工具页面 */
	DATA_PAGE_DEBUG				= 0x08,
	DATA_PAGE_TEMP_SINGLE       = 0x09,             /* 扩展屏幕(温度), 单通道巡检 */
	DATA_PAGE_TEMP_MULTI        = 0x0A,              /* 扩展屏幕(温度), 多通道巡检 */
	MAX_DATA_PAGE
}DATA_PAGE;

typedef enum {
	ITEM_DAT_NUL				= 0x00,
	/* 以下7种类型为UI屏幕显示、输入抽象化 */
	ITEM_UI_TOPIC				= 0x01, 			/* 标题,可以操作返回、刷新、上下滚动等 */
	ITEM_UI_TEXT				= 0x02, 			/* 纯文本,无输入框,  */
	ITEM_UI_CLICK				= 0x03, 			/* 可点击对象，无输入框, 如BOOL,ECONT,S32,无密码LINK都映射到这里 */
	ITEM_UI_LOGIN				= 0x04, 			/* 一个密码输入框, 由LINK带密码的构成 */
	ITEM_UI_ONEBOX				= 0x05, 			/* 一个数字输入框 */
	ITEM_UI_TWOBOX				= 0x06, 			/* 两个数字输入框 */
	ITEM_UI_ONEDBOX				= 0x07, 			/* 一个两倍宽度数字输入框 */
	ITEM_UI_ANSI				= 0x08, 			/* 一个32字符ANSI文本输入(目前支持ASCII就够) */

	/* 以下为实际配置使用的类型，右移4位得到UI类型，例外：LINK */
	ITEM_DAT_TOPIC				= 0x10, 			/* 标题, LkId_Val_ItN代表列表数量; 如果有变量则作为页面变量, 从后续的位变量配置表中逐个位初始化变量 */
	ITEM_DAT_TXT				= 0x20, 			/* 纯文本,无输入框,  */
	ITEM_DAT_SOFTVER			= 0x21, 			/* 软件版本号,在LkId_Val_ItN存储，无变量 */
	ITEM_DAT_ENUM				= 0x22, 			/* 枚举 */
	ITEM_DAT_T64_RO 			= 0x23, 			/* 64位时间(只读),  REAL_TIME_VAR格式 */
	ITEM_DAT_U32_RO 			= 0x24, 			/* 只读的U32位变量 */
	ITEM_DAT_ANSI_RO			= 0x25,				/* 硬件版本号，其实是只读的ANSI字符串 */
	ITEM_DAT_T32_RO 			= 0x26, 			/* 32位时间(只读),  seconds格式存储，年/月/日 时:分:秒 形式显示 */
    ITEM_DAT_I64_RO             = 0x27,             /* 只读的I64位变量 */
    ITEM_DAT_U32_ID_RO          = 0x28,             /* 用芯片ID(Hash值)加持的变量, 只显示变量值，修改的时候需要记录芯片ID，用于sn,license */
	ITEM_DAT_S32				= 0x30, 			/* 32位结构体, 存取数据在链接页面的TOPIC，所指向的数据仅用于显示，便于核验 */
	ITEM_DAT_BOOL				= 0x31, 			/* bool变量 */
	ITEM_DAT_ECONT				= 0x32, 			/* 枚举内容, LkId_Val_ItN 代表枚举值(属性值) */
	ITEM_DAT_LINK				= 0x40, 			/* 链接，链接的变量是密码,LkId_Val_ItN代表链接地址 */
	ITEM_DAT_FUNC               = 0x41,             /* 特定功能，输入密码，执行某种动作(如：清空统计数据)，LkId_Val_ItN值见 SYS_FUNCTION */
	ITEM_DAT_U32 				= 0x50, 			/* 32位无符号数 */
	ITEM_DAT_F32 				= 0x51, 			/* 32位浮点数 */
	ITEM_DAT_TEMPSENCONF		= 0x52, 			/* 用于温度传感器配置,对于触摸屏软件和ITEM_DAT_U32一样处理 */
	ITEM_DAT_ACQ_U32			= 0x53, 			/* 带采集功能的32位无符号数(采集的部分是第二个数，为浮点数) */
	ITEM_DAT_ACQ_F32			= 0x54,				/* 带采集功能的32位浮点数(采集的部分是第二个数，为浮点数) */
	ITEM_DAT_D2U32				= 0x60, 			/* 2个32位无符合数--用于统计页面 */
	ITEM_DAT_D2F32				= 0x61, 			/* 2个32位无符合数--用于统计页面 */
	ITEM_DAT_SUMM				= 0x62, 			/* 用于带自动采集功能变量的配置，对于触摸屏软件,Data1是float32, data2是int32 */
	ITEM_DAT_pTIME				= 0x63,				/* 峰谷电时间起点--显示为p#hh:mm; 输入#:可以用.代替,也可无; 存储忽略#. */
	ITEM_DAT_I64				= 0x70, 			/* 64位符合数--用于统计页面 */
	ITEM_DAT_IPv4				= 0x71, 			/* IPv4地址，显示的时候需要显示为 abc.abc.abc.abc */
	ITEM_DAT_RMTS				= 0x72,				/* 远程传感器，仅配置序列号部分 */
	ITEM_DAT_DIGITSTR			= 0x73, 			/* 由数字构成的字符串，用于编码、密码 */
	ITEM_DAT_TELE_ITEM			= 0x74,				/* 101,104的点表配置项 */
	ITEM_DAT_ANSI 				= 0x80, 			/* ANSI输入 */
}ITEM_DATA_TYPE;
#define CONF_ANSI_BYTE_LEN	32						/* ITEM_DAT_ANSI格式数据长度，需要为4的倍数以方便对齐 */

typedef enum {										/* 配置/统计数据存储进行分类存储，每一个数据将存储到对应的类中 */
	SAVE_GRP_NUL				= 0,
	SAVE_GRP_BRD				= 1,				/* 板子相关类，两个eeprom备份 */
	SAVE_GRP_MCONF				= 2,				/* 主要配置数据类 */
	SAVE_GRP_TSEN				= 3,				/* 温度传感器配置类 */
	SAVE_GRP_RSVD				= 4,				/* 备用 */
	SAVE_GRP_STAT 				= 5,				/* 统计数据配置类 */
MAX_SAVE_GRP_NUM	= 6
}CONF_SAVE_GROUP;

typedef struct {
	ITEM_DATA_TYPE DataType;						/* 数据类型 ITEM_DATA_TYPE */
	uint8 uBcdQ;									/* 小数点位置 */
	/* 如果是浮点类型，则为显示时的浮点数有效位数(0为采用默认值4)；否则为位宽度, 如果有设置则该变量在页面变量中(Topic所对应) */
	int8 i8SgnDigits_u8BitNum;
	uint8 SaveGrp;									/* 数据存取分组, 仅存储系统使用, 定义见 CONF_SAVE_GROUP */
}ITEM_TYPE;

typedef enum {
	CHINESE			= 0,
	ENGLISH			= 1,
#if (DEVICE_TYPE == V5_YYT3 || DEVICE_TYPE == V5_YYT4)
	NUMBERIC		= 2,							/* 数码管 */
#endif
	MAX_LANG_TYPE
}LANG_TYPE;
typedef struct {									/* Conf,Stat 数据属性描述 */
	uint32 u32ItemID;								/* 变量ID */
	ITEM_TYPE ItemType;
	uint32 LkId_Val_ItN;							/* 链接ID/设定默认值/属性值/列表数量 */
	void* pData;
	char* pName[MAX_LANG_TYPE];
}CONF_n_STAT_PROP;
#define CONF_PAGE_LVL_NUM				4			/* 配置页面层级数量, 要比真实层级多1，方便寻址 */

/* 几个复杂配置结构体 */
typedef struct {									/* ITEM_DAT_ACQ_F32 */
	float32 fSetVal;								/* 人工设定参数--为了简化UI软件，方便访问的第一个变量 */
	float32 fCaliVal;								/* 测量值 */
}ACQ_F32;

typedef struct {									/* ITEM_DAT_ACQ_U32 */
	uint32 u32SetVal;								/* 人工设定参数--为了简化UI软件，方便访问的第一个变量 */
	float32 fCaliVal;								/* 测量值 */
}ACQ_U32;

typedef struct {	/* 用于温度传感器配置,表达形式: 1#温度传感器序列号HHHHHHHHHHHHHH, 当前温度: 27.8 时间: 99999 放置位置:	*/
	uint32 u32TempSenPosition;						/* 传感器放置位置--为了简化UI软件，方便访问的第一个变量 */
	uint32 u32Count_TempSen_Change;					/* 传感器通讯断线计数器，以温度传感器变化(接入或者断开)作为计数单位 */
	uint64 u64TempSenRom;
}TEMP_SEN_CONF;

typedef struct {				/* ITEM_DAT_SUMM 相关参数,以下变量顺序不得更改 */
	int32 i32SummaryPlanNum;    /* 0:综合的时候有几组数据，用几组数据; 非0:取绝对值，综合的时候所用的组数; >0使用综合值; <0仅综合,不使用 */
	float32 fSetVal;			/* 人工设定参数--为了简化UI软件，方便访问的第一个变量 */
	float32 fAcqSummary;		/* 采集综合值--为了简化数据存储，方便访问的第二个变量 */
	uint32 u32SummaryRealNum;	/* 综合值真实数据组数 */
	float32 fCurAcq;			/* 测量值 */
}ACQ_SUMM;
#define ACQ_SUMM_USE_SET(i32SummaryPlanNum)		(i32SummaryPlanNum <= 0)
#define ACQ_SUMM_USE_SUMM(i32SummaryPlanNum)	(i32SummaryPlanNum > 1)
#define ACQ_SUMM_USE_CUR(i32SummaryPlanNum)		(i32SummaryPlanNum == 1)

/*===========================================================================
* 3. 存储系统的访问接口: 配置、消息、采集 以及系统工具: 数据初始化、检查
*==========================================================================*/
/* 数据操作，分为: IDLE-MISSION-(RES)-END-IDLE，如果处理超时，等不到结果请求方可以置为END
	IDLE	: 可以接受数据操作请求
	MISSION : 有操作任务,
	RES		: 有操作结果
	END		: 数据请求方收到操作结果，或者超时放弃等待 */
typedef enum {
	DATA_ACS_PROC				= 1,
		
	DATA_ACS_IDLE				= 0,
	DATA_ACS_RES_SUC			= -1,		/* 0xFF */
	DATA_ACS_RES_FAIL			= -2,		/* 0xFE */
	DATA_ACS_RES_FULL			= -3,		/* 0xFD */
	DATA_ACS_RES_EMPTY			= -4,		/* 0xFC */
	DATA_ACS_RES_CRC_ERR		= -5,		/* 0xFB */		
	DATA_ACS_RES_NOT_CMPLT		= -6,		/* 0xFA */
	DATA_ACS_RES_SUC_BUT_OLD	= -7,
	DATA_ACS_RES_OPEN_FILE_FAIL	= -8,
	DATA_ACS_RES_URGENT_QUIT	= -9,		/* 有紧急存储任务，退出当前操作 */
	DATA_ACS_RES_PWD_ERR		= -10,		/* 输入密码错误 */
	DATA_ACS_RES_BUSY			= -11,		/* 系统忙 */
	DATA_ACS_RES_TIMEOUT		= -12,		/* 超时 */
	DATA_ACS_RES_CRC_CHG		= -13,		/* CRC状态发生变化：从crc好到crc不好，或者反过来 */
    DATA_ACS_RES_INPUT_ERR      = -14,      /* 输入错误 */
	DATA_ACS_RES_MAX			= -15,
	
	DATA_ACS_END				= -20,		/* 标识数据请求方已经消费完数据 */
	
	DATA_ACS_READ_DATA_PAGE		= -40,
	DATA_ACS_READ_MSG			= DATA_ACS_READ_DATA_PAGE - DATA_PAGE_MSG,
	DATA_ACS_READ_ACQ			= DATA_ACS_READ_DATA_PAGE - DATA_PAGE_ACQ,

	DATA_ACS_RST_MSG 			= -59,		/* 系统操作 */
	DATA_ACS_REQ				= -60,
	DATA_ACS_CHK_GLB_CONF		= -61,
	DATA_ACS_PUSH_GLB_CONF		= -62,
	DATA_ACS_RST_ACQ 			= -63,
	DATA_ACS_RST_STAT			= -64
}DATA_ACS_OPR;
#define DATA_ACS_RES(State)			((DATA_ACS_END <= State) && (State < 0))
#define DATA_ACS_MISSION(State)		(State <= DATA_ACS_READ_DATA_PAGE)
EXT const char* cnst_DataAcsOprRes[][MAX_LANG_TYPE]
#ifndef _MDL_SYS_C_
;
#else
= {	/* 数据处理结果 */
    {"******", "******"},
	{"成功", "success"},
	{"失败", "fail"},
	{"存储容量满", "storage full"},
	{"数据源空", "empty"},
	{"crc校验失败", "crc error"},
	{"部分完成", "NOT complete "},
	{"成功但版本过老", "success but version old"},
	{"打开文件失败", "open file error"},
	{"紧急任务,退出操作", "urgent quit"},
	{"密码错误", "password error"},
	{"系统忙", "sys busy, abort"},
	{"超时", "please wait"},
    {"CRC状态发生变化", "please wait"},
	{"输入错误", "please wait"},
	{"处理中,请稍候", "please wait"},         /* DATA_ACS_RES_MAX所对应的值 */
};
#endif

typedef enum {		/* 数据用户，不同的用户需要不同的格式、数据段 */
	DATA_USER_NULL				= 0,
	DATA_USER_MCGS,
	DATA_USER_VIEWTECH,
	DATA_USER_NET,
	DATA_USER_FILE
}DATA_USER;

typedef enum {      /* ITEM_DAT_FUNC LkId_Val_ItN数值所对应的功能 */
    SYS_FUNC_RST_STAT   = 0,
    SYS_FUNC_RST_ACQ    = 1,
    SYS_FUNC_ENG_DEBUG  = 2,
    MAX_SYS_FUNC
}SYS_FUNCTION;
typedef struct {									/* 数据存取任务对外接口 */
    int8 i8SysFunc[MAX_SYS_FUNC];

	/* conf存储相关 */
	BOOL bConfHaveUnsavedChange[MAX_SAVE_GRP_NUM];	/* 配置有未保存的修改 */
	BOOL bConfFree[MAX_SAVE_GRP_NUM];				/* 由于存储任务的优先级最低，因此其他任务只需判断该标志，无需加锁 */
	DATA_ACS_OPR eConfInitRes[MAX_SAVE_GRP_NUM];	/* 上电初始化成功 */

	triST tSaveUrgent_0Idle_pReq_nCmplt;			/* 紧急存储，主要用于即将重启或者即将断电的情况 0:无操作 >0:请求存储 <0:完成存储 */
	BOOL bTestRes_TFCard;
	uint8 u8Rsvd;
}DATA_ACS_INTF;
EXT SECTION(".NOT_ZeroInit") DATA_ACS_INTF g_DataAcsIntf;

/* 消息解码所需要的结构 */
typedef struct {
	int8 i8DataPosition;				/* 消息数值插入位置，-1代表无数据 */
	F32_MEANING F32Meaning;
	char* pText[MAX_LANG_TYPE];		/* 消息文本 */
}MSG_PROP;

/*===========================================================================
* 4. 软件相关
	升级与授权:为了安全，密码不得存储在代码中,只能在运行中产生, 					不同的产品采用不一样的密码
	GetAuthKey()用于产品密码，ChkAuthKey()用于校验密码，不同的产品需要修改，以产生不一样的密码
*==========================================================================*/
typedef struct {									/* 配置变量存储地址 */
	uint32 u32OrigEEAddr;							/* 正本EEProm地址(Byte为基本单元) */
	uint32 u32BackEEAddr;							/* 副本EEProm地址(Byte为基本单元) */
	const uint8* pOrigPath;							/* 已经不再使用了，留着为升级的时候可以兼容老版本 */
	const uint8* pBackPath;							/* 已经不再使用了，留着为升级的时候可以兼容老版本 */
}CONF_ACS_MEDIA;
typedef struct {									/* 软件配置表，本表必须固定放在0x4000位置 */
	uint32 _c_int00_add;							/* _c_int00()函数的地址，BootLoader通过该变量跳转到运行软件 */
	CONF_n_STAT_PROP* pConfProp;					/* 配置变量属性表地址 */
	uint32 u32ItemNum;								/* 配置变量数量 */
	uint32 u32ConfSaveGroupNum; 					/*	*/
	CONF_ACS_MEDIA AcsMedia[MAX_SAVE_GRP_NUM];
}BOOT_LOADER_CONF;

typedef enum {			/* 请求授权类型，AUTH_AES_*** 为4个32bit变量， */
	AUTH_KEY_FLASH_REQ_KEY = 0,
	AUTH_AES_FLASH_REQ,
	AUTH_AES_FLASH_DATA,
	AUTH_AES_PACK_SERIAL,
	AUTH_AES_PACK_LICENSE,
	AUTH_KEY_FLASH_INTEGRITY_V1,/* 仅YKF2/3,YYD需要，且需要保持不变 */
	AUTH_KEY_FLASH_INTEGRITY_V2,/* 软件升级，该值可以更改 */
	AUTH_AES_COMM_SN_LIC,		/* 升级软件给母版传递 sn_new\lic 信息 */
	AUTH_TYPE_NUM
}AUTH_TYPE;
/* 为了支持密码更改，需要存储多组   密码，后续请求的AuthType = AUTH_*** + AUTH_TYPE_NUM*GrpNo
   最新的GrpNo=0，次新的GrpNo=1... */
EXT void GetAuthKey(AUTH_TYPE AuthType, uint32* pAuthKey);
EXT BOOL ChkAuthKey(AUTH_TYPE AuthType, uint32* pAuthKey);

typedef struct {				/* 这个变量存储在eeprom EEADD_BOOT_SOFT_VERSION 中 */
	uint32 u32RunVersion;		/* 指定在运行的版本 */
	uint32 u32LowFlashVer;		/* 低地址空间的版本, 采取高低16位互补的形式校验 */
	uint32 u32HighFlashVer;		/* 高地址空间的版本, 采取高低16位互补的形式校验 */
}BOOT_SOFT_VERSION;
/*===========================================================================
 * 5. 机组/站房/远端 数据交换/控制指令, 可以接收处理CAN/RS485/以太网来的数据，因此不列在内
 *==========================================================================*/
#define MAX_YKF_NUM						12		/* 一个电站，最多机组数量 */
#define MAX_YYB_NUM						6		/* 一个电站，最多变压器数量 */

/* 远程传感器，包括来自mqtt(网络)，光纤、无线数传电台(UART) */
#define MAX_RMT_SEN_NUM	2
#define MAX_TRY_CNT_EXT_RMT_SEN_FROM_MQTT	10		/* 断线延时(秒) */
typedef struct {
	uint16 uNum_DataValid;				/* 标记fSenData中，有几个数据是有效的 */
	uint16 uRsvd;
	uint32 u32RcvTime_RTCSeconds;		/* 标记收到数据的时间，以RTC          32位秒计数器为基准 */
	float32	fDCPwrVol;					/* 采样板电源电压 */
	float32 fSenData[MAX_RMT_SEN_NUM];	/* 传感器数据 */
}RMT_SEN_DAT;

/*===========================================================================
 * 6. 队列相关
 *==========================================================================*/
/* f32环形队列 */
typedef struct {
	float32 *pFData;
	uint16 uStart;
	uint16 uEnd;
	uint16 uMaxLen;		/* 队列最大长度 */
}F32_QUEUE_t;
/***************************************************************************
 					functions must be driven by extern
***************************************************************************/
/* <NULL> */


/***************************************************************************
 					functions could be used as extern
***************************************************************************/
//__weak void InitMdlCtr(void);			from InitYxx()
//__weak void RunMdlCtr(void);			from RunYxx()
//__weak void DrvMdlCtrTick_1KHz(void);	from DrvYxxTick_1KHz()

/************   exclude redefinition and c++ environment   ****************/
#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */

/*===========================================================================
 * 最后: 不同的设备在这里做映射
 *==========================================================================*/
#if CPU_0ST_1TI
    #include "DrvTI.h"
#else
    #include "DrvST.h"
	#include "BoardSupport.h"
#endif
#if ((DEVICE_TYPE == V5_YBT2) || (DEVICE_TYPE == V5_YBT3) || (DEVICE_TYPE == V5_YBT4))
	#include "MdlYBT.h"
#elif (DEVICE_TYPE == V5_YYT3) || (DEVICE_TYPE == V5_YYT4)
	#include "MdlYYT.h"
#elif ((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4))
	#include "MdlYBU.h"
#elif (Is_YKF(DEVICE_TYPE) || (DEVICE_TYPE == V5_YYD) || (DEVICE_TYPE == V5_YYD2) || (DEVICE_TYPE == V5_YYG))
	#include "MdlYKF.h"
#elif (DEVICE_TYPE == V5_YYB)
	#include "MdlYYB.h"
#elif (DEVICE_TYPE == YKC)
	#include "MdlYKC.h"
#endif

/******************************** FILE END ********************************/
