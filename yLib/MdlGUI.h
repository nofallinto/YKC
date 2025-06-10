/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved.	模板V1.4
 *				: 
 * File			: 
 * Author		: Wang Renfei
 * Description	: 

2017.3.24 未完成事项
1. 输入，这个需要等屏幕厂家修补bug才可以进一步往下工作
2. 其他页面仅完成主页面8个数值变量的显示，没有机组状态等文本枚举型变量的显示
3. 待完成页面有: 辅助、系统、高级参数、调试页面
4. 初步完成ItemPage页面的显示，但是由于缺乏输入，配置页面缺乏跳转功能，尚需进一步完善
5. 缺乏CRC部分
6. ViewTech屏幕所有显示控件地址的规划

 * Date			: 
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * V1.0	| 2017-3-24	  |	king | 
 **************************************************************************/

/***************************************************************************
 				exclude redefinition and c++ environment
***************************************************************************/
#ifndef _MDL_GUI_H_
#define _MDL_GUI_H_

/***************************************************************************
 							include files
***************************************************************************/
#include "MdlNet.h"


/***************************************************************************
				hardware parameter and global definitions
***************************************************************************/
/* <NULL> */


/***************************************************************************
 		use EXT to distinguish stencil.c or others source files
***************************************************************************/
#ifdef _MDL_GUI_C_
#define EXT
#else
#define EXT extern
#endif


/***************************************************************************
 					variables could be used as extern
***************************************************************************/
#if SUPPORT_SCREEN
/* VT屏幕通讯接口 */
typedef enum{
	/* 旧通讯接口 */
	VT_COMM_VER_DEFAULT			= 0,
	VT_COMM_VER_OLD1		    = 1,			/* YKF2\3 16路 */
	VT_COMM_VER_OLD2		    = 2,			/* YKF2\3 16路 */
	VT_COMM_VER_OLD3		    = 3,			/* YKF2\3 16路, YBU3, YYT3 */
	VT_VAL_DEV_SDW043			= 4,			/* ViewTech YYD4.3寸屏 */
	VT_COMM_VER_MERGE			= 5,			/* ViewTech VT合并屏幕、YKF4\5 10寸屏 */
	MAX_VT_COMM_VER
}VT_COMM_VER;
#endif

typedef struct {
	/* 以下是配置页面描述与维护 */
	DATA_PAGE DataPage;							/* 该变量用于MCGS、ViewTech 页面 */
	LANG_TYPE Language;							/* 该变量用于MQTT配置页面 */
	/* 配置页面 */
	uint16 uConfPageTopicItemNo;				/* 配置页面首行ItemNo */
	uint16 uConfWinFirstLineItemNo;				/* 当前显示窗口第一行ItemNo */

#if SUPPORT_SCREEN
	/* ViewTech屏幕 */
	uint8 u8PageRefresh1Hz_100ms;				/* 页面1Hz刷新定时器 */
	uint8 u8SlowRefresh_1Hz;					/* 慢速新刷定时器*/
	VT_COMM_VER VTCommVer;						/* 主VT屏通讯型号*/
	DATA_PAGE ExtPage;                          /* 扩展屏幕页面 */
	uint8 u8ExtItemNo;							/* 扩展屏幕项目序号(目前主要用于温度序号) */
#endif

	/* Mcgs屏幕当前在显示的行; VT屏幕配置页面正在修改的行; VT屏幕其他页面项目(例如温度序号);  */
	BOOL bMsgAcqPageWinDown;					/* Msg,Acq页面向后翻页，即序号减少 */
	uint32 u32ItemNo;							/* 页面初始化时，会将该变量初始化成0xFFFFFFFFUL */

	/*正在输入状态*/
	uint8 u8InputingType;						/* 输入位置，是否双输入框后一个输入 */
	BOOL bUnprocessedInput;

	/* 屏保 */
	uint8 u8Tmr_OffScn_1s;						/* 主屏熄屏 */
	uint8 u8Tmr_ExtOffScn_1s;					/* 扩展屏幕熄屏 */
}ITEM_PAGE_REQ;

#if SUPPORT_SCREEN
/* Viewtech宏前缀命名规则  *
 * VT_DEV代表屏幕设备地址
 * VT_REG代表寄存器(REG)地址  *
 * VT_VAR代表变量(VAR)地址  *
 * VT_VAL代表取值
 * VT_LEN代表字节长度
 * VT_BUF代表数据在通讯Buf中的位置
 * VT_PAGE是VT屏幕页面编号
 */

/* 设备型号 */
#define VT_VAR_DEVICE_TYPE						0x0000		/* ViewTech屏幕型号 */

#define VT_DEV_MON 	                            0xA55A      /* 主屏幕地址 */
#define VT_DEV_EXT	                            0x5AA5      /* 扩展屏幕地址 */
#define VT_REG_PIC_ID							0x03		/* VIEWTECH 控制当前显示页的寄存器 */
#define VT_REG_LED_CTL							0x01		/* VIEWTECH 亮度控制寄存器 */
#define VT_REG_TOUCH_STATUS						0x06		/* VIEWTECH 读取触摸状态寄存器 */

#define VT_VAL_PNLKEY_MAX_PAGE_VAL				0x3000		/* 屏幕侧以PNLKEY形式传来的页码的最大可能+1 */
#define VT_VAL_PNLKEY_STATUS_BTN_MIN_VAL		0x3001		/* 屏幕侧以PNLKEY形式传来的状态按钮值的最小可能（含） */
#define VT_VAL_PNLKEY_STATUS_BTN_MAX_VAL		0x301F		/* 屏幕侧以PNLKEY形式传来的状态按钮值的最大可能（含） */

#define VT_BUF_RDREG							0x06		/* VT读取屏幕寄存器指令返回内容位置 */
#define VT_BUF_RDVAR							0x07		/* VT读取屏幕内存指令返回内容位置 */

#define VT_REG_INPUT_STATUS						0xE9		/* VT是否处于输入状态（VT叫用户键码） */
#define VT_REG_KEYBOARD_CTL						0x4F		/* 弹出键盘呼叫地址 */

/* 屏幕按钮 */
#define VT_VAR_PNLKEY							0xF100		/* 命令输入   */
#define VT_VAR_KEYBOARD_STATUS					0xF000		/* VT键盘输入状态，高字节值：0x5A：（输入结束标记），表示输入结束。0x00：表示此事液晶屏处于输入过程状态中。 */
#define VT_VAR_KEYBOARD_INPUT					0xF001		/* VT键盘输入内容  */
#define VT_VAL_KEYBOARD_ASCII					20			/* VT弹出英文键盘呼叫编号 */
#define VT_VAL_KEYBOARD_NUM						10			/* VT弹出数字键盘呼叫编号 */
#define VT_VAL_KEYBOARD_PWD						11			/* VT弹出密码键盘呼叫编号 */

/* 不同产品逻辑页面(DataPage)与VT屏幕Page之间的映射关系。页面切换按钮的值是待切换页面ID */
#define VT_PAGE_DEFAULT							00			/* 通用页面 : 默认页面  */
#define VT_PAGE_CONFIG							10			/* 通用页面 : 配置  */
#define VT_PAGE_MSGoACQ							12			/* 通用页面 : 消息  */
#define VT_PAGE_BTN_ACQ 						14			/* 注意：这个是采集页面按钮，显示的时候用VT_PAGE_MSGoACQ */
#ifdef _MDL_GUI_C_
const uint16 cnst_uDataPage2VTPageBtn[MAX_DATA_PAGE]
 = {00, 10, 12, 14,                                         /* 欢迎页面、配置页面、消息页面、采集页面 */
     01, 50, 52, 56,                                        /* 欢迎页面、主页面、辅助页面、系统页面 */
     54, 58, 60};                                           /* 调试页面、单通道温度、多通道温度 */
#endif
EXT const uint16 cnst_uDataPage2VTPage[][MAX_DATA_PAGE]       /* 以下数字所标的，是非通用页面，需要与VT项目保持一致 */
#ifndef _MDL_GUI_C_
;
#elif Is_YKF(DEVICE_TYPE)
= {{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* 欢迎页面、配置页面、消息页面、采集页面 */
		01, 50, 52, 56,                                                     /* 欢迎页面、主页面、辅助页面、系统页面 */
		54, 58, 60},                                                      	/* 调试页面、单通道温度、多通道温度 */
	/* V5_YKF2 温度24路 */
	{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ, 	/* 欢迎页面、配置页面、消息页面、采集页面 */
		01, 50, 90, 92,                                                     /* 欢迎页面、主页面、辅助页面、系统页面 */
		54, 58, 60}                                                        	/* 调试页面、单通道温度、多通道温度 */
};
#elif((DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4))
= {{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* 空页面、配置页面、消息页面、采集页面 */
		01, 50, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, 							/* 欢迎页面、主页面、辅助页面、系统页面 */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT},                 /* 调试页面、单通道温度、多通道温度 */
	/* V5_YBU合并屏幕后的页码 */
	{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ, 				/* 空页面、配置页面、消息页面、采集页面 */
		01, 80, 80, VT_PAGE_DEFAULT, 							/* 欢迎页面、主页面、辅助页面、系统页面 */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT}                  /* 调试页面、单通道温度、多通道温度 */
};
#elif(DEVICE_TYPE == V5_YYG)
= {{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* 欢迎页面、配置页面、消息页面、采集页面 */
		01, 50, 52, 56,                                                     /* 欢迎页面、主页面、辅助页面、系统页面 */
		54, 58, 60},                                                      	/* 调试页面、单通道温度、多通道温度 */
		/* V5_YYG */
	{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* 空页面、配置页面、消息页面、采集页面 */
		01, 62, 64, 56, 													/* 欢迎页面、主页面、辅助页面、系统页面 */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT}            		/* 调试页面、单通道温度、多通道温度 */
};
#elif((DEVICE_TYPE == V5_YYD) || (DEVICE_TYPE == V5_YYD2))
= {{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,  	/* 欢迎页面、配置页面、消息页面、采集页面 */
		01, 50, 52, VT_PAGE_DEFAULT,                                        /* 欢迎页面、主页面、辅助页面、系统页面 */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT},                 /* 调试页面、单通道温度、多通道温度 */
	/* V5_YYD 7寸 */
	{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ, 	/* 空页面、配置页面、消息页面、采集页面 */
		01, 70, 72, VT_PAGE_DEFAULT, 										/* 欢迎页面、主页面、辅助页面、系统页面 */
		54, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT}               				/* 调试页面、单通道温度、多通道温度 */
};
#elif(DEVICE_TYPE == V5_YYT3) || (DEVICE_TYPE == V5_YYT4)
= {{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* 空页面、配置页面、消息页面、采集页面 */
		01, 50, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, 							/* 欢迎页面、主页面、辅助页面、系统页面 */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT},                 /* 调试页面、单通道温度、多通道温度 */
	{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* 空页面、配置页面、消息页面、采集页面 */
		01, 50, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, 							/* 欢迎页面、主页面、辅助页面、系统页面 */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT}                  /* 调试页面、单通道温度、多通道温度 */
};
#else
= {{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* 空页面、配置页面、消息页面、采集页面 */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT,	/* 欢迎页面、主页面、辅助页面、系统页面 */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT},	                /* 调试页面、单通道温度、多通道温度 */
	{VT_PAGE_DEFAULT, VT_PAGE_CONFIG, VT_PAGE_MSGoACQ, VT_PAGE_MSGoACQ,   	/* 空页面、配置页面、消息页面、采集页面 */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT,	/* 欢迎页面、主页面、辅助页面、系统页面 */
		VT_PAGE_DEFAULT, VT_PAGE_DEFAULT, VT_PAGE_DEFAULT}	                /* 调试页面、单通道温度、多通道温度 */
};
#endif

/*VIEW_TECH寄存器操作动作编码*/
typedef enum {
	VIEW_TECH_WRITE_REG		= 0x80,					/*用于写地址范围在0-0xFF的寄存器 */
	VIEW_TECH_READ_REG		= 0x81,					/*用于读地址范围在0-0xFF的寄存器 */
	VIEW_TECH_WRITE_VAR		= 0x82,					/*用于写地址范围在0-0xFFFF的变量 */
	VIEW_TECH_READ_VAR		= 0x83,					/*用于读地址范围在0-0xFFFF的变量 */
}VIEW_TECH_CMD;

#define VT_LEN_ITEM_CHAR				62			/* ViewTech屏幕，ItemPage单行字符(英文)长度，必须是偶数 */
#define VT_LEN_ITEM_BYTE	(VT_LEN_ITEM_CHAR + 2)	/* ViewTech屏幕，ItemPage单行buf长度还需加2个byte    用于MsgID或AcqID */
#define MCGS_LEN_ITEM_CHAR				60			/* MCGS屏幕，ItemPage单行字符(英文)长度，必须是偶数 */

#define MCGS_ITEM_PAGE_LINE_WLEN		(MCGS_LEN_ITEM_CHAR + 6)/2
#define MCGS_ITEM_PAGE_ADDI_BLEN		10				/* 由于屏幕软件要求汉字编码必须以16bit为单位，因此一开始生成的内容必须经过整理以满足屏幕软件处理要求，需要额外的ram进行整理 */
#define MCGS_ITEM_PAGE_ACK_MAX_BLEN		(MCGS_LEN_ITEM_CHAR + 6 + MCGS_ITEM_PAGE_ADDI_BLEN)
#define MCGS_ITEM_PAGE_TYPE_BSTART		0
#define MCGS_ITEM_PAGE_PAYLOAD_BSTART	2
#define MCGS_ITEM_PAGE_ADDIPAYLOAD_BSTART	(MCGS_ITEM_PAGE_PAYLOAD_BSTART + MCGS_ITEM_PAGE_ADDI_BLEN)
#define MCGS_ITEM_PAGE_ITEMNO_BSTART	(2 + MCGS_LEN_ITEM_CHAR)
typedef struct {									/* MCGS页面下 ItemPageAck 基本Item数据结构, 不同类型下的细致结构参见相应文档 */
	uint16 uItemType;								/* 指出后面的PayLoad所采用的具体页面Item结构,该位在初始化 */
	uint16 uPayLoad[MCGS_LEN_ITEM_CHAR/2]; 			/* 数据结构之负载部分，用于存储具体需要显示的数据 */
	uint32 u32ItemNo; 								/* 在Page定位的主要依据 */
}MCGS_ITEM_PAGE_ACK;

typedef struct {									/* ViewTech屏幕 MsgAcq页面一行的数据结构 */
	uint8 u8Text[VT_LEN_ITEM_CHAR];
	uint16 uMsgAcqID;
}VIEW_TECH_MSG_ACQ_ITEM;

#define VT_VAL_COLOR_RED										0xF800
#define VT_VAL_COLOR_YELLOW										0xFFE0

#define SetMsgAcqLineColor(u8UartPort, uLineColorAddr, uColor) do{\
	uint8 *pTRBuf = &g_UartComm[u8UartPort].u8TRBuf[4];	\
	*pTRBuf++ = uLineColorAddr/0x100;				\
	*pTRBuf++ = uLineColorAddr&0xFF;				\
	*pTRBuf++ = uColor/0x100;                       \
	*pTRBuf++ = uColor%0x100;                       \
	SendData_VT(u8UartPort, pTRBuf);				\
}while(0);
#endif

/* MQTT conf/msg/acq操作 */
#define MQTT_ITEM_PROC_BUF_BLEN     (MQTT_TR_BUF_BLEN - 100)
typedef struct {
	/* 配置同步(备份、导入)操作 */
	uint16 uStartConfID;							 /* 起始配置ID */
	uint16 uEndConfID;								 /* 结束配置ID */
	int8 i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP;	 /* 备份、导入状态--0:空闲;-1:成功;-2:失败;>0:最大努力次数 */

	/* 配置页面操作 */
	int8 i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP;	/* 写数据状态--0:空闲;-1:成功;-2:失败;>0:最大努力次数 */
	uint16 uPageOpItemNo;							/* 0xFFFF代表初始化页面，真实行号取0 */
	ITEM_PAGE_REQ ItemPageReq;		/* ProcItemPageInstr()调用所用的标准结构体 */

    /* 已经发布给数据库的MsgNo */
    uint32 u32MsgNo_HavePubedForDB;

    /* msg、acq页面操作 */
	uint32 u32ItemNo_InStart;
	uint32 u32ItemNo_OutEnd;
	int16 iItemNum_InNeed;
	int16 iItemNum_OutSuc;
    int8 i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage;
    DATA_ACS_OPR DataAcsRes;
    uint16 uContLen;
	uint8 u8Buf[MQTT_ITEM_PROC_BUF_BLEN];                  /* DataAccess任务先写进该Buf */
}MQTT_ITEM_PROC;
EXT MQTT_ITEM_PROC g_MqttItemProc;

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
#if SUPPORT_SCREEN
EXT	void TryViewTech(uint8 u8UartPort);
EXT	void RunViewTech(uint8 u8UartPort);

/***************************************************************************
 					functions could be used as extern
***************************************************************************/
/* MCGS屏幕所需 */
EXT void FillItemPageAck_MCGS(ITEM_PAGE_REQ* pItemPageReq, uint8* pMCGSItemPageAck);

/* ViewTech屏幕所需 */
EXT BOOL ReadData_VT(uint8 u8UartPort, uint16 uDevAddr, VIEW_TECH_CMD ViewTechCmd, uint16 uDataAddr, uint16 uWordLen);
EXT void WriteVar_VT(uint8 u8UartPort, uint16 uDevAddr, uint16 uDataAddr, uint8 *pValue, uint8 u8WordLen);
EXT void WriteReg_VT(uint8 u8UartPort, uint16 uDevAddr, uint16 uDataAddr, uint16 uValue, uint8 u8ByteLen);
EXT void ClearVar_VT(uint8 u8UartPort, uint16 uDevAddr, uint16 uDataAddr, uint8 u8WordLenToClear);
EXT void SendData_VT(uint8 u8UartPort, uint8* pTxBuf);
EXT uint16 DataPage2VTPage(VT_COMM_VER VTCommVer, DATA_PAGE DataPage);
EXT void ChangePage_VT(ITEM_PAGE_REQ *pItemPageReq, uint16 uVTPage);
EXT uint8 PrintStringFromText_VT(uint8 *pDstBuf, const char *pSrcText, uint8 u8MaxLen);
EXT uint8* PrintTemperatureName(uint8* pText, TEMPERATURE_DATA* pTempData, char TailChar);
EXT uint8* PrintTemperatureValue(uint8* pText, TEMPERATURE_DATA* pTempData, const char* pDiscText);
#endif

/* mqtt配置 CONF_n_STAT_PROP格式 接口 */
EXT void ProcMqttConfInstr(uint8* pU8Msg, uint8* pU8MsgEnd);
EXT void ProcMqttConfSync(uint8* pU8Msg, uint8* pU8MsgEnd);
EXT BOOL QueryAndPubMqttConfResAndPage(MQTT_COMM* pMqttComm);

/* 打印CONF_n_STAT_PROP格式配置变量，处理ItemPage指令 */
EXT uint8* PrintLocalConfItem(DATA_USER DataUser, CONF_n_STAT_PROP* pDataAcsProp, uint8* pBuf, uint16 uTextByteLen);
EXT int8 ProcItemPageInstr(DATA_USER DataUser, DATA_PAGE DataPage, uint32 u32ItemNo, uint8** ppU8Instr, uint8* pU8InstrEnd, ITEM_PAGE_REQ* pItemPageReq);

/************   exclude redefinition and c++ environment   ****************/

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
/******************************** FILE END ********************************/
