	/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved.	模板V1.3
 *				: 
 * File			: stencil.h
 * Author		: Wang Renfei
 * Description	: 包括三个模块：配置、消息、采集的数据存取
 * Date			: 2009-5-31
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 * V1.3	| 2015-9-7	  |	king | 1. 文件头部增加模板版本号并增加模块调用约定
 * 		|			  |		 | 2. 删除Comments栏目
 * 		| 2015-9-27	  |		 | 3. 添加RunMdl任务与DrvMdl_100Hz定时器驱动
 * V1.2	| 2009-5-31	  |	king | 1. 修改为适应模块化,增加需要外部驱动的函数
 * V1.1 | 2006-11-27  | king | 1. Add compatiblility with c++
 * 		| 			  | king | 2. Change Resivision control table
 * V1.0	| 2005-2-28   |	king | 1. Create
 **************************************************************************/

/***************************************************************************
 				exclude redefinition and c++ environment
***************************************************************************/
#ifndef _MDL_DATA_ACCESS_H_
#define _MDL_DATA_ACCESS_H_

#ifdef __cplusplus
extern "C" {
#endif


/***************************************************************************
 							include files
***************************************************************************/
#include "GlobalVar.h"
#include "MdlNet.h"

/***************************************************************************
				hardware parameter and global definitions
***************************************************************************/

/***************************************************************************
 		use EXT to distinguish stencil.c or others source files
***************************************************************************/
#ifdef _MDL_DATA_ACCESS_C_
#define EXT
#else
#define EXT extern
#endif


/***************************************************************************
 					variables could be used as extern
***************************************************************************/
//extern SEM_Handle SEM_NvMemAcs;
/*===========================================================================
 * Msg部分，使用MSG模块，需要做如下配置：
1. #define MAX_MSG_TYPE			0x0040				// 总的消息类型数量，最好是0x20的整数倍，必须有
   #define MSG_ABORNMAL_START	0x0020				// 异常消息起始位置，这一条定义可以没有

2. 消息部分
 	typedef enum {					// MSG_***要从 0x0002开始
		MSG_NULL					= 0x0000,		// 空，这条可以没有
		MSG_HuanYinShiYong 			= 0x0001,		// 欢迎使用，这条可以没有
		//以下是正式消息部分
		MSG_***						= 0x0002,
		 ...
	} MSG_ID;

3. 消息解码部分，必须有	MAX_MSG_TYPE 条
	const MSG_PROP cnst_MsgProp[MAX_MSG_TYPE]		// 为了便于识别，有附加值的消息正文部分不可以带数字
	= {	{-1,	"", ""},
		{-1,	"欢迎使用***", "Reserved"},
		...
	};
 *==========================================================================*/


/*===========================================================================
 * ACQ模块：对采集数据进行 写入、读取、综合成AcqSumm；如果使用需要做如下配置：
 1. #define ACQ_ITEM_DAT_NUM 5					//单条ACQ采集所记录的数据量
 2. 
 *==========================================================================*/
/* AcqID说明 */
typedef struct {
	BITBOOL	ID		: 8;	/* 主ID部分 */
	BITBOOL SubID	: 8;	/* 子ID部分，如果搜索的时候该部分有值即为精确匹配，否则为不匹配该字段 */
}STRUCT_ACQ_ID;

#if ACQ_ITEM_DAT_NUM
typedef struct {								/* 读取的数据项 */
	uint16 uAcqId;								/* 用于处理函数进一步验证 */
	uint16 uDaysFromOrigin;						/* 从2000/01/01开始的天数，以便判断数据的新鲜程度 */
	float32 fD[ACQ_ITEM_DAT_NUM];
}ACQ_READ_ITEM;

typedef struct {								/* 把AcqData、AcqSumm、处理函数联系起来 */
	uint16 uAcqId;
	uint16 uRsvd;
	void (*AcqSummProc)(ACQ_READ_ITEM* pAcqDataStart, uint16 uItemNum);	/* 处理函数 */
	ACQ_SUMM* pAcqSumm[ACQ_ITEM_DAT_NUM];		/* 指向相关的AcqSumm，以便在AcqSumm发生更改的时候，查找相应的处理单 */
}ACQ_SUMM_MAP;
#endif

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
EXT void DataAccessTask(void const * argument);

/***************************************************************************
 					functions could be used as extern
***************************************************************************/
/* 数据存储、清除接口 */
EXT void SaveStatMsgAcq(void);			/* 启动Stat、Msg、Acq数据进行存储 */
EXT void UrgentSaveNvData(void);		/* 板上电源监测: 低电压紧急启动数据存储，需要调高存储任务优先级 */
EXT BOOL ChkAndWaitInitConf(CONF_SAVE_GROUP ConfStatSaveMedia);				/* 初始化配置与统计 */
EXT BOOL RegisterSysFunc(SYS_FUNCTION SysFunc, BOOL bUnLock);
EXT void RegisterPageReq(DATA_PAGE DataPage, uint8* pU8Msg, uint8* pU8MsgEnd);  /* MQTT通讯 */
EXT DATA_ACS_OPR ReadConfFromEEProm(CONF_SAVE_GROUP AcsGrp); 
EXT BOOL CheckConfFromEEProm(CONF_SAVE_GROUP AcsGrp, uint32 u32EEPromByteAddr); 	/* 提到.h理由：为DrvXX能看到 */
EXT void SaveConfToEEProm(CONF_SAVE_GROUP AcsGrp, uint32 u32EEPromByteAddr);

/* 消息添加工具 */
EXT void AddMsgA_WithVal(uint16 uMsgId, float32 fMsgVal);		/* 仅该函数支持重入，消息可以重复添加 */
EXT void AddMsgB_WithVal(uint16 uMsgId, float32 fMsgVal);		/* 消息仅能添加一次 */
EXT void UpdateMsgC_WithVal(uint16 uMsgId, BOOL bMax1OrMin0, float32 fMsgVal);
EXT void ConfirmMsgC(uint16 uMsgId);
EXT void FinishMsgC(uint16 uMsgId, BOOL bNeedBlock);
EXT void ClearAllMsgShieldFlag(void);
EXT void ClearSingleMsgShieldFlag(uint16 uMsgId);

/* 采集存取工具 */
EXT void AddAcqItem(uint16 uAcqID, float32 fD0, float32 fD1, float32 fD2, float32 fD3, float32 fD4);
EXT void InformAcqSummConfChange(void* pAcqSumm);

/*	MSG, ACQ 文本访问接口
	DataUser	: 数据用户, DATA_USER_NET数据长度不固定，一个Item结束立刻放置下一个Item;
							DATA_USER_MCGS，DATA_USER_VIEWTECH则数据长度固定为N，进行填充，扔掉ItemNo与CRC部分;
							DATA_USER_VIEWTECH最后两个字节为MsgID或AcqID
								不满会以空字符串按照期望读的条数进行填充(不超过pBufEnd)，但不会修改返回的Itemo, ItemNum
	DataPage					: 目前仅支持DATA_PAGE_MSG, DATA_PAGE_ACQ
	bNetAbortUnCmpltMsg			: 网络发送数据，遇到不完整的消息中止，仅发送完整的消息--用于数据库接收
	pU32ItemNo_InStart_OutEnd 	: 开始ItemNo(不含), 返回实际读取的最后一条ItemNo--可能指定的ItemNo读不到数据, 0xFFFFFFFFUL代表读取Topic
	pI32ItemNum_InNeed_OutSuc 	: 读取的Item数量(正:ItemNo增加，即往后读;负:ItemNo减少，即往前读)，返回实际读取的Item数量(带符号，仅指向前、向后成功读取)
	ppBuf						: Buf起点，可以写入，返回访问完成后的指针
	pBufEnd						: Buf终点，不可写入	*/
EXT DATA_ACS_OPR GetTextForMsgOrAcq(DATA_USER DataUser, DATA_PAGE DataPage, uint32* pU32ItemNo_InStart_OutEnd, 
					                int32* pI32ItemNum_InNeed_OutSuc, uint8** ppBuf, uint8* pBufEnd);
EXT BOOL PubMsgAndAcq(MQTT_COMM* pMqttComm);	/* 发布Msg，Acq */
EXT uint16 ReadLastAddMsgForVT(uint8* pBuf, uint8* pBufEnd, BOOL bTimeShort);

/* 把ram中的Msg通过modbus发送，一条消息占据6个modbus16bit寄存器 */
typedef struct {    /* 这个结构体并没有实际应用，仅作为modbus接口传输的示例 */
	uint8 u8Year;					/* 年，1-99，如果该位为0则代表整个时间无效 */
	uint8 u8Month;					/* 月，1-12 */
	uint8 u8Day;					/* 日，1-31 */
	uint8 u8Hour;					/* 时，0-23 */
	uint8 u8Minute;					/* 分，0-59 */
	uint8 u8Second;					/* 秒，0-59 */
	uint16 uMsgId;                  /* MsgId，需要查阅MSG_ID以便解码 */
	float32 fMsgVal;                /* 消息值，注意是浮点数 */
}MSG_FOR_MODBUS;
EXT uint16 TxMsgForModbus(uint8** ppBuf, uint16 uRegNum);

/************   exclude redefinition and c++ environment   ****************/
#ifdef __cplusplus
}
#endif 					/* extern "C" */

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
/******************************** FILE END ********************************/
