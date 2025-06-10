/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved. 模板V1.3
 *
 * File			: MdlDataAccess.c
 * Author		: Wang Renfei
 * Description	: 主要功能: 数据存取、备份(TF卡,U盘,EEPROM)，支持通讯/显示访问接口(主要是ItemPage)，满足四种数据类型(配置、统计、记录、采集)的需求

 	由于文件系统所消耗的内存较大，而需要访问文件的task较多，为了避免每个task都为文件系统分配内存，因此把文件访问集中在一个task中实现
 	其他task访问文件，通过提交请求实现，实现函数主要分为三大类:
 	1. 外部接口的注册任务(如读写数据)、检查任务完成情况
 	2. 具体任务实现--通过主任务调用实现
 	3. 工具类函数，各种任务都会使用
 
	外部访问模型:
 	1. 数据存取以项目(Item)为单元，一个项目为一行
	2. 配置与统计，通过 CONF_n_STAT_PROP 描述项目行 格式/提示符/数据/文本/项目属性值，作为ram 和 文件系统交互 的接口
	3. 记录，通过 MSG 描述每一条消息
	4. 采集，通过 ** 描述

 * Date			: 2016-6-6
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 **************************************************************************/

/***************************************************************************
 						macro definition
***************************************************************************/
#define _MDL_DATA_ACCESS_C_			/* exclude redefinition */


/***************************************************************************
 						include files
***************************************************************************/
/* 操作系统头文件 */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

/* 项目公共头文件 */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"
#include "MdlNet.h"

/* 本级以及下级模块头文件 */
#include "MdlDataAccess.h"
#include "BoardSupport.h"

/* 其他模块 */
#ifdef BOARD_SDSPI
	#include "fatfs/ff.h"
	#define SUPPORT_FAT_FS	TRUE
#else
	#define SUPPORT_FAT_FS	FALSE
#endif
#include "MdlGUI.h"

/***************************************************************************
 						global variables definition
***************************************************************************/
#define MAX_READ_MSG_ACQ_NUM		4
typedef struct {
	DATA_ACS_OPR DataAcsOpr;
	DATA_USER DataUser;
	uint16 uRsvd;
	uint8* pBuf;
	uint8* pBufEnd;
	uint32 u32ItemNo_InStart_OutEnd;		/* 访问起始行号(含)，返回最后一条读出记录的行号 */
	int32 i32ItemNum_InNeed_OutRemain;		/* 正:增行号读，负:减行号读; 返回剩余的项目数量 */
}READ_MSG_ACQ_TXT;

/* msg,acq存取方式 */
typedef enum {
    MSG_ACQ_ACS_V1 = 0,                   /* 每天一个文件，按照/年/月/日.txt进行组织 */
    MSG_ACQ_ACS_V2 = 1,                   /* 每10万条消息一个文件，按照/sn.txt进行组织 */
}MSG_ACQ_ACS_VER;

#define FILE_CONT_BUF_LEN			10240   /* 不得少于2个扇区 1024byte */
#define FILE_PATH_BUF_LEN			80
#define FILE_NAME_BYTE_LEN			13				/* 8.3格式 */

#define MSG_ACQ_FILE_PATH_DEPTH		3
#define MAX_FILE_PATH_DEPTH			(MSG_ACQ_FILE_PATH_DEPTH + 1)

typedef struct {
	/* 访问接口 */
	READ_MSG_ACQ_TXT ReadMsgOrAcqTxt[MAX_READ_MSG_ACQ_NUM];
	BOOL bNeedSaveAcq;
	BOOL bNeedSaveMsg;
	uint16 uRsvd;
	
	/* SPI句柄--这个位置必须放在 g_NvMemAcsCtr.u8FilePath */
    SDSPI_Handle sdspiHandle;	/* 必须放在这个位置，这之前(不含)的需要被初始化为0 */

    /* Buf无需初始化为0的 */
	uint8 u8FilePath[FILE_PATH_BUF_LEN];	/* 文件目录，由于文件目录操作没有做溢出检查，因此必须放在文件Buf之前，万一溢出影响不大 */
	uint8 u8FileCont[FILE_CONT_BUF_LEN];	/* 文件内容 */
#if SUPPORT_FAT_FS
	uint8 fatfsTempSpace[_MAX_SS];			/* fatfs临时操作空间 */
#endif
}NVMEM_ACS_CTR;
SECTION(".NOT_ZeroInit") NVMEM_ACS_CTR g_NvMemAcsCtr;

#define CONF_SAVE_NAME_BLEN		(28*2)

/* 判断EEPROM地址是否在合法的SAVE_GRP存储范围内 */
#if CPU_0ST_1TI == 0
#define WITHIN_SAVE_GRP_EEPROM_ADDR(eepromAddr) (((eepromAddr >= (EEADD_START + BOOT_LOADER_CONF_BLEN)) && (eepromAddr < (EEADD_START + EEPROM_BLEN))) || ((eepromAddr >= (EEADD_START+FLASH_HALF_BLEN + BOOT_LOADER_CONF_BLEN)) && (eepromAddr < (EEADD_START+FLASH_HALF_BLEN + EEPROM_BLEN))))
#elif CPU_0ST_1TI == 1
#define WITHIN_SAVE_GRP_EEPROM_ADDR(eepromAddr) (((eepromAddr >= (EEADD_START + BOOT_LOADER_CONF_BLEN)) && (eepromAddr < (EEADD_START + EEPROM_BLEN))))
#endif

//BOOL WITHIN_SAVE_GRP_EEPROM_ADDR(uint32 eepromAddr)
//{
//	g_CodeTest.u32Val[30] = ((eepromAddr >= (EEADD_START + BOOT_LOADER_CONF_BLEN)) && (eepromAddr < (EEADD_START + EEPROM_BLEN)));
//	g_CodeTest.u32Val[31] = (eepromAddr >= (EEADD_START + BOOT_LOADER_CONF_BLEN));
//	g_CodeTest.u32Val[32] = (eepromAddr < (EEADD_START + EEPROM_BLEN));
//
//	g_CodeTest.u32Val[34] = (((eepromAddr+FLASH_HALF_BLEN) >= (EEADD_START+FLASH_HALF_BLEN + BOOT_LOADER_CONF_BLEN)) && ((eepromAddr+FLASH_HALF_BLEN) < (EEADD_START+FLASH_HALF_BLEN + EEPROM_BLEN)));
//	g_CodeTest.u32Val[35] = (((eepromAddr+FLASH_HALF_BLEN) >= (EEADD_START+FLASH_HALF_BLEN + BOOT_LOADER_CONF_BLEN)));
//	g_CodeTest.u32Val[36] = ((((eepromAddr+FLASH_HALF_BLEN) < (EEADD_START+FLASH_HALF_BLEN + EEPROM_BLEN))));
//}
/***************************************************************************
						internal functions declaration
***************************************************************************/
/* 初始化 */
void InitDataAccess(void);
void InitPowerOnData(void);
void FixSoftVerAndConf(void);
#ifdef BOARD_SDSPI
BOOL TestOfTFCard(void);        /* tf卡测试，仅测试软件使用 */
#endif

/* 系统操作:复位Msg,Acq,Stat数据 */
void ResetMsgData(void);
void ResetAcqData(void);
void ResetStatData(void);

/* 文件名生成、目录检查函数 */
#if SUPPORT_FAT_FS
void ChkAndMkDir(void);
void EmptyFold(uint8* pFilePathIni);
#endif

/* Conf处理函数 */
void SaveConfToFile(CONF_SAVE_GROUP ConfAcsGrp);
DATA_ACS_OPR ReadOrCheckConfFromFile(CONF_SAVE_GROUP ConfAcsGrp, int16 iTryCnt_pRd_nChk);
void InitConfWithDefault(void);
uint8 CalDataBLenFromItemDataType(ITEM_DATA_TYPE DataType);
void SaveConfToEEProm(CONF_SAVE_GROUP AcsGrp, uint32 u32EEPromByteAddr);
BOOL CheckConfFromEEProm(CONF_SAVE_GROUP AcsGrp, uint32 u32EEPromByteAddr);
DATA_ACS_OPR ReadConfFromEEProm(CONF_SAVE_GROUP AcsGrp);

/* Msg模块函数 */
void InitMsgMdl(void);
void SaveMsgToNvMem(void);

/* Acq模块函数 */
void InitAcqMdl(void);
void SaveAcqToNvMem(void);
void RunAcqSummTask(void);

/* Msg,Acq模块公共函数 */
#if SUPPORT_FAT_FS
uint32 ReadLastItemNoFromFile(DATA_PAGE DataPage, MSG_ACQ_ACS_VER eReadItemVer);
#endif
void ReadItemNoAndEEPromNo(DATA_PAGE DataPage);
void ReadTextForMsgOrAcq(READ_MSG_ACQ_TXT* pReadMsgOrAcqCtr);

/* 以下队列中(包括本文件内)所有pTextEnd指向队尾后一个(不可存储)的地址 */
uint32 ReadH32FixLen(uint8* pText);
uint32 ReadU32FixLen(uint8* pText);
float32 ConvTextToF32_RightStart(uint8** ppText, uint8* pTextStart);

/***************************************************************************
 							functions definition
***************************************************************************/
/*==========================================================================
| Description	: 模块初始化
| G/Out var		:
| Author		: Wang Renfei			Date	: 2008-3-4
\=========================================================================*/
void InitDataAccess(void)
{
	/* 初始化配置 */
	/* <NULL> */

	/* 初始化输入接口变量 */
	/* <NULL> */

	/* 初始化输出接口变量 */
	/* <NULL> */

	/* 初始化内部全局变量 */
	InitDataWithZero((uint8*)(&g_NvMemAcsCtr), (uint8*)(&g_NvMemAcsCtr.sdspiHandle) - (uint8*)(&g_NvMemAcsCtr));

	/* 启动硬件 */
	/* 打开TF卡 */

#ifdef BOARD_SDSPI
	InitSDSPI();
//TI::InitSDSPI()
//	SDSPI_Params sdspiParams;
//	sdspiParams.bitRate = 12000000UL;
//	sdspiParams.custom = NULL;
//	g_NvMemAcsCtr.sdspiHandle = SDSPI_open(BOARD_SDSPI, 0, &sdspiParams);
#endif
    /* TF卡测试 */
#if SUPPORT_FAT_FS && (!SOFT_RUN1_TEST0)
    g_DataAcsIntf.bTestRes_TFCard = TestOfTFCard();
#endif

	/* 初始化下层模块 */
#if MAX_MSG_TYPE
	InitMsgMdl();
#endif
#if ACQ_ITEM_DAT_NUM
	InitAcqMdl();
#endif
}

/*==========================================================================
| Description	: 上电初始化
	1. 检查目录
	2. 获取并检查License是否超期
	3. 读取配置数据(配置与统计信息初始化)
	3.1 静态配置数据
	3.2 温度传感器通讯断线计数器部分??
	3.3 Acq已经总结存储的部分，用于配置中采集变量
	4 读取统计数据

	除了License，对于其他数据(配置、统计类)读取步骤:
	1. 搜索数据文件正本，如果找到则读取，并进行数据校验
	2. 如果数据校验错误，则再读一次正本，进行数据校验
	3. 如果未能搜索到文件正本，或者连续两次数据校验错误，则搜索BackUp文件夹最近的备份
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-06-21
\=========================================================================*/
#define MAX_READ_BACKUP_TRY_COUNT	5
void InitPowerOnData(void)
{
	/* 0. 初始化到默认变量 */
	InitConfWithDefault();
	g_DataAcsIntf.eConfInitRes[SAVE_GRP_NUL] = DATA_ACS_IDLE;			/* 为了访问方便，前面设置了第0个空的访问标志 */
	g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt = 0;					/* 初始化 */

	CONF_SAVE_GROUP ConfAcsGrp;
#if SUPPORT_FAT_FS
	/* 1. 检查目录结构的存在 */
	ChkAndMkDir();

	/* 2. 读取仅存储在eeprom中的配置与统计数据 */
	g_DataAcsIntf.eConfInitRes[SAVE_GRP_BRD] = ReadConfFromEEProm(SAVE_GRP_BRD);

	/* 3. 读取存储在(文件+eeprom)中的配置与统计数据 */
	for(ConfAcsGrp = SAVE_GRP_MCONF; ConfAcsGrp < MAX_SAVE_GRP_NUM; ConfAcsGrp++) {
		/* 优先搜索数据正本 */
		DATA_ACS_OPR eInitRes = ReadOrCheckConfFromFile(ConfAcsGrp, 2);
		if((eInitRes == DATA_ACS_RES_SUC) || (eInitRes == DATA_ACS_RES_NOT_CMPLT)) {
		} else {
			eInitRes = ReadConfFromEEProm(ConfAcsGrp);			
		}
		g_DataAcsIntf.eConfInitRes[ConfAcsGrp] = eInitRes;
	}
#else
	for(ConfAcsGrp = SAVE_GRP_BRD; ConfAcsGrp < MAX_SAVE_GRP_NUM; ConfAcsGrp++) {
		g_DataAcsIntf.eConfInitRes[ConfAcsGrp] = ReadConfFromEEProm(ConfAcsGrp);
	}
#endif
	
	/* 4. 特殊配置变量初始化 */
	if(g_Sys.SerialNo.u32Dat < 10000000UL) {
		g_Sys.SerialNo.u32Dat = CalcUid16B();
	}

	/* 5. 标记配置可以访问 */
	for(ConfAcsGrp = SAVE_GRP_BRD; ConfAcsGrp < MAX_SAVE_GRP_NUM; ConfAcsGrp++) {
		g_DataAcsIntf.bConfFree[ConfAcsGrp] = TRUE;

	#if 0	/* 统计eeprom占用，新版本如果添加了变量，可以打开编译开关，平时关闭 */
		/* 统计该存储类别使用的eeprom大小 */
		int16 i;
		CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
		for(i = pBootLoaderConf->u32ItemNum; i > 0; i--) {
			if(pDataAcsProp->ItemType.SaveGrp == ConfAcsGrp) {
				g_CodeTest.uVal3[ConfAcsGrp*2 + 0] += CalDataBLenFromItemDataType(pDataAcsProp->ItemType.DataType);
			}
			pDataAcsProp++;
		}
		if(g_CodeTest.uVal3[ConfAcsGrp*2 + 0]) {			/* 有数据 */
			g_CodeTest.uVal3[ConfAcsGrp*2 + 0] += 4;		/* 最后CRC用4byte */
			/* 计算该存储类别分配的eeprom大小 */
			if(ConfAcsGrp < MAX_SAVE_GRP_NUM-1) {
				if(pBootLoaderConf->AcsMedia[ConfAcsGrp+1].u32OrigEEAddr == 0) {
					g_CodeTest.uVal3[ConfAcsGrp*2 + 1] = pBootLoaderConf->AcsMedia[ConfAcsGrp+2].u32OrigEEAddr - pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr;
				} else {
					g_CodeTest.uVal3[ConfAcsGrp*2 + 1] = pBootLoaderConf->AcsMedia[ConfAcsGrp+1].u32OrigEEAddr - pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr;
				}
			} else {
				#ifdef EEPROM_ADDR_ACQ
					g_CodeTest.uVal3[ConfAcsGrp*2 + 1] = EEPROM_ADDR_ACQ - pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr;
				#else
					g_CodeTest.uVal3[ConfAcsGrp*2 + 1] = EEPROM_ADDR_MSG - pBootLoaderConf->AcsMedia[ConfAcsGrp].u32OrigEEAddr;
				#endif
			}
		}
	#endif
	}
}

/*==========================================================================
| Description	: 数据存取任务

	数据存储包括两个模式: 文件(tf卡)、eeprom, 各有侧重

	存取时机	断电存储优先级		其他存储			上电读取顺序		通讯读取
		conf		1(如果有)	修改配置的时候				1			无需访问文件，直接组装
		Summ		2			修改综合的时候				2			无需访问文件，直接组装
		acq			4			Buf超过一半容量的时候		3			文件中的文本或者内存中编码信息
		msg			3			Buf超过一半容量的时候					文件中的文本或者内存中编码信息

	存储位置				文件(tf卡)												eeprom
	conf,stat		一主一备，备份文件打上时间戳							起到备份作用，conf里面主数据记录
	conf_温度传感器	同上													同上
	conf_矫正,SN等	主要起到数据导出方便看的作用，即存储的时候还是要存的	两份数据，一主一备，要加CRC32校验
	acq,msg			仅保留一份数据，但是每天一个文件以降低损坏概率			不保留
	acq,msg_ItemNo	以记录中包含为主，但如果一个文件中都读不到，则放弃		形成一个ring，每次刷掉最小ItemNo，读取的时候取最大值
	acq_总结		一主一备，定时(比如一个月?)整理形成						起到备份作用
	如果文件和eeprom里面的数据不一致，优先校验正确数据，如果校验都正确，则依据主/备原则，修改备份数据以保持一致，

配置、统计数据存取
	对于配置、统计数据存储(新建一个文件，把所有数据重新写入)步骤:
		1. 新建空白文件，对配置/统计数据进行存储，并在文件后加上校验(crc32)，保存两份，一份在BackUp文件夹中，并打上时间戳(yymmddhh)
		2. 把保存的文件和原始数据做对比，如果一致则认为存储成功，否则删除文件重复1
	对于配置、统计类数据读取步骤:
		1. 搜索数据文件正本，如果找到则读取，并进行数据校验
		2. 如果数据校验错误，则再读一次正本，进行数据校验
		3. 如果未能搜索到文件正本，或者连续两次数据校验错误，则搜索BackUp文件夹最近的备份

Msg,Acq数据存取
	对于Msg, Acq数据存储(定期截断，形成新文件；数据是添加在文件末尾)步骤:
		1. 形成每天一个文件，以降低文件损坏概率,并按照年/月进行文件夹整理，文件命名格式:ddnnnnnn，月文件夹mmnnnnnn, 年文件夹yynnnnnn
		2. 每项记录一个校验
	对于Msg, Acq数据读取(定期截断，形成新文件；数据是添加在文件末尾)步骤:
		1. 如果访问形式是ItemNo, 则根据提交的访问形式，查找文件夹，确定文件，再访问文件以获得具体的文本
		2. 如果访问形式是时间，则直接查找文件夹/文件名即可

数据备份、恢复
	备份 : 把数据从tf卡拷贝到U盘，放在U盘 Yoplore\NNNNNNNN\YYMMDDHH 文件夹中，其中NNNNNNNN是序列号, YYMMDDHH是备份日期
		1. 简单备份: conf/stat数据不拷贝BackUp文件夹，Acq/Msg数据拷贝全部
		2. 全部备份: conf/stat数据拷贝BackUp文件夹，Acq/Msg数据拷贝全部
	操作步骤 : 输入备份密码即可操作，把TF卡上的内容备份到U盘上

	恢复 : 把数据从U盘拷贝到tf卡中
		1. 简单恢复: conf/stat数据不拷贝BackUp文件夹，Acq/Msg数据拷贝全部
		2. 全部备份: conf/stat数据拷贝BackUp文件夹，Acq/Msg数据拷贝全部
	操作步骤 : 输入控制器序列号、备份时间，如果没有输入，则选择最新的备份



重要参数
	1. Msg, Acq的最后一个ItemNo
	2. Conf, Stat数值
	
	
| G/Out var		:
| Author		: Wang Renfei			Date	: 2015-9-27
\=========================================================================*/
void DataAccessTask(void const * argument)
{
	InitDataAccess();
	if(g_Sys.uRstCount == 0) {	/* 上电复位，才进行此操作 */
		InitPowerOnData();
		FixSoftVerAndConf();
	}

	for(;;) {
		Task_setPri(g_DataAccessBlock.DataAcsTSK, TASK_LOWEST_PRIORITY); /* 刚上电度时候，优先级处于最高，以便尽快初始化配置变量；紧急存储也会调高优先级，完成后降低为2 */
		if(Semaphore_pend(g_DataAccessBlock.Sem_Op, BIOS_WAIT_FOREVER)){
		}
		PersistSaveGrp();		/* 配置或者统计数据，保存在(文件+EEPROM)的数据 */
		/* 如果是来自远程的配置存储请求，则 */
#if CPU_0ST_1TI == 0
		CONF_SAVE_GROUP ConfAcsGrp;
#endif
	    Swi_disable();
		for(ConfAcsGrp = SAVE_GRP_BRD; (ConfAcsGrp < MAX_SAVE_GRP_NUM) && (!g_DataAcsIntf.bConfHaveUnsavedChange[ConfAcsGrp]); ConfAcsGrp++) {
		}
		BOOL bMqttConf = (g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP > 0)
		                || (g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP > 0);
		if(ConfAcsGrp < MAX_SAVE_GRP_NUM) {		/* 有未保存的数据，则再来一次 */
		    if(bMqttConf) {
                Semaphore_post(g_DataAccessBlock.Sem_Op);
		    }
		} else if(bMqttConf) {
		    if(g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP > 0) {
		        g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP = -1;
		    }
		    if(g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP > 0) {
		        g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = -1;
		    }
			Semaphore_post(SEM_MqttPubReq);
	    }
	    Swi_enable();

	#if MAX_MSG_TYPE		/* Msg数据保存 */
		if(g_NvMemAcsCtr.bNeedSaveMsg || (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0)) {
			SaveMsgToNvMem();
			g_NvMemAcsCtr.bNeedSaveMsg = FALSE;
		}
	#endif

	#if ACQ_ITEM_DAT_NUM	/* Acq数据存取 */
		RunAcqSummTask();				/* Acq数据读取与综合处理 */
		if(g_NvMemAcsCtr.bNeedSaveAcq || (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0)) {
			SaveAcqToNvMem();
			g_NvMemAcsCtr.bNeedSaveAcq = FALSE;
		}
	#endif

		/* Msg, Acq文本读取 */
		int8 i;
		for(i = 0; (i < MAX_READ_MSG_ACQ_NUM) && (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt <= 0); i++) {
			if(DATA_ACS_MISSION(g_NvMemAcsCtr.ReadMsgOrAcqTxt[i].DataAcsOpr)) {
				ReadTextForMsgOrAcq(&g_NvMemAcsCtr.ReadMsgOrAcqTxt[i]);
			} else if(g_NvMemAcsCtr.ReadMsgOrAcqTxt[i].DataAcsOpr == DATA_ACS_END) {	/* 数据请求方已经消费完数据 */
				g_NvMemAcsCtr.ReadMsgOrAcqTxt[i].DataAcsOpr = DATA_ACS_IDLE;
			}
		}
	    ReadTextForMsgOrAcq(NULL);  /* NULL指的是向数据库发布消息的任务 */
	
		/* 系统操作 */
	#if ACQ_ITEM_DAT_NUM
		if(DATA_ACS_MISSION(g_DataAcsIntf.i8SysFunc[SYS_FUNC_RST_ACQ])) {
			ResetAcqData();
			g_DataAcsIntf.i8SysFunc[SYS_FUNC_RST_ACQ] = DATA_ACS_RES_SUC;
		}
	#endif
		if(DATA_ACS_MISSION(g_DataAcsIntf.i8SysFunc[SYS_FUNC_RST_STAT])) {
			g_DataAcsIntf.bConfFree[SAVE_GRP_STAT] = FALSE;
			ResetStatData();
			g_DataAcsIntf.bConfFree[SAVE_GRP_STAT] = TRUE;
			g_DataAcsIntf.i8SysFunc[SYS_FUNC_RST_STAT] = DATA_ACS_RES_SUC;
		}
	#ifdef GLOBAL_CONF_ID_PREFIX
		//RunConfPushAndCheck();
	#endif

		if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
			g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt = -1;
            Semaphore_post(g_DataAccessBlock.Sem_Op);   // Semaphore_post(SEM_NvMemAcs); 	/* 可能有任务因为紧急存储而放弃，因此这里重启一遍 */
		}
	}
}

#ifdef BOARD_SDSPI
/* TF卡读写测试 */
const char cnst_TFCardTestText[] = \
"***********************************************************************\n"
"A		   1		 2		   3		 4		   5		 6		   7\n"
"01234567890123456789012345678901234567890123456789012345678901234567890\n"
"0		   1		 2		   3		 4		   5		 6		   7\n"
"01234567890123456789012345678901234567890123456789012345678901234567890\n"
"0		   1		 2		   3		 4		   5		 6		   7\n"
"01234567890123456789012345678901234567890123456789012345678901234567890\n"
"0		   1		 2		   3		 4		   5		 6		   7\n"
"01234567890123456789012345678901234567890123456789012345678901234567890\n"
"0		   1		 2		   3		 4		   5		 6		   7\n"
"01234567890123456789012345678901234567890123456789012345678901234567890\n"
"0		   1		 2		   3		 4		   5		 6		   7\n"
"01234567890123456789012345678901234567890123456789012345678901234567890\n"
"0		   1		 2		   3		 4		   5		 6		   7\n"
"01234567890123456789012345678901234567890123456789012345678901234567890\n"
"0		   1		 2		   3		 4		   5		 6		   7\n"
"01234567890123456789012345678901234567890123456789012345678901234567890\n"
"0		   1		 2		   3		 4		   5		 6		   7\n"
"01234567890123456789012345678901234567890123456789012345678901234567890\n"
"***********************************************************************\n";
BOOL TestOfTFCard(void)
{
	FIL File;
	UINT u32ByteNum;
	int16 iStrLen = sizeof(cnst_TFCardTestText);
	if(f_open(&File, "0:/TFTest.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
        f_write(&File, cnst_TFCardTestText, iStrLen, &u32ByteNum);
		f_close(&File);
	}
	if(f_open(&File, "0:/TFTest.txt", FA_OPEN_EXISTING | FA_READ) == FR_OK) {
		f_read(&File, g_NvMemAcsCtr.u8FileCont, FILE_CONT_BUF_LEN, &u32ByteNum);
		f_close(&File);
		if(iStrLen == u32ByteNum) {
			uint8* pU8Read = g_NvMemAcsCtr.u8FileCont;
			const char* pU8Text = cnst_TFCardTestText;
			for(; (iStrLen > 0) && (*pU8Read++ == *pU8Text++); iStrLen--) {
			}
		}
	}	
	return (iStrLen == 0);
}
#endif

/*==========================================================================
 * 外部工具
 1. SaveStatMsgAcq() 在停机时，持久化当前ram中的数据
 2. UrgentSaveNvData() 设备重启时，保存当前数据
 3. ChkAndWaitInitConf() 提供给其他任务，看配置数据是否初始化完成
 4. RegisterDataOperation() 提供系统页面操作: 同步配置、清除数据等
 *========================================================================*/
void SaveStatMsgAcq(void)
{
	g_NvMemAcsCtr.bNeedSaveAcq = TRUE;
	g_NvMemAcsCtr.bNeedSaveMsg = TRUE;
	g_DataAcsIntf.bConfHaveUnsavedChange[SAVE_GRP_STAT] = TRUE;
	Semaphore_post(g_DataAccessBlock.Sem_Op);		//	Semaphore_post(SEM_NvMemAcs);
}	

void UrgentSaveNvData(void)
{
	/* 启动存储 */
	g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt = 1;
	Semaphore_post(g_DataAccessBlock.Sem_Op);				//Semaphore_post(SEM_NvMemAcs);
	Task_setPri(g_DataAccessBlock.DataAcsTSK, 15);			//	Task_setPri(TSK_DataAcs, 15);			/* 调高存储任务优先级至最高 */

	/* 等待完成 */
	while(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
		Task_sleep(OS_TICK_KHz*1);
	}
}

BOOL ChkAndWaitInitConf(CONF_SAVE_GROUP AcsGrp)
{
	int8 i;
	for(i = 100; (i > 0) && (g_DataAcsIntf.eConfInitRes[AcsGrp] == DATA_ACS_IDLE); i--) {
		Task_sleep(OS_TICK_KHz*10);
	}
	return (i != 0);
}

BOOL RegisterSysFunc(SYS_FUNCTION SysFunc, BOOL bUnLock)
{
    if(SysFunc >= MAX_SYS_FUNC) {
        return FALSE;
    } else if(SysFunc == SYS_FUNC_ENG_DEBUG) {  /* 工程调试 */
        if(bUnLock) {  
            g_Ctr.bEngDebug = TRUE;
            g_DataAcsIntf.i8SysFunc[SysFunc] = DATA_ACS_RES_SUC;
        } else {
            g_Ctr.bEngDebug = FALSE;
            g_DataAcsIntf.i8SysFunc[SysFunc] = DATA_ACS_RES_PWD_ERR;
        }
    } else if(bUnLock) {
        g_DataAcsIntf.i8SysFunc[SysFunc] = DATA_ACS_REQ;
		Semaphore_post(g_DataAccessBlock.Sem_Op);		//		Semaphore_post(SEM_NvMemAcs);
    } else {    
        g_DataAcsIntf.i8SysFunc[SysFunc] = DATA_ACS_RES_PWD_ERR;
    }
    
    return (g_DataAcsIntf.i8SysFunc[SysFunc] != DATA_ACS_RES_PWD_ERR);
}

/*==========================================================================
 * 锁存当前时间
 *========================================================================*/
/* 初始化配置至默认值 */
void InitConfWithDefault(void)
{
	CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
	CONF_n_STAT_PROP* pPageDataAcsProp;
	uint32 u32PageData;
	int32 i, j;
	uint16 uBitStart;

	for(i = pBootLoaderConf->u32ItemNum; i > 0; i--) {
		if(pDataAcsProp->pData != NULL) {
			switch(pDataAcsProp->ItemType.DataType) {
				case ITEM_DAT_TOPIC:
					u32PageData = 0;
					uBitStart = 0;
					pPageDataAcsProp = pDataAcsProp + 1;
					for(j = pDataAcsProp->LkId_Val_ItN; j > 0; j--) {
						if((pPageDataAcsProp->pData == NULL) 	/* 确定位变量 */
							&& pPageDataAcsProp->ItemType.i8SgnDigits_u8BitNum
							&& ((pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL)
								|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM)
								|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32)))
						{
							u32PageData += pPageDataAcsProp->LkId_Val_ItN<<uBitStart;
							uBitStart += pPageDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
						}
						pPageDataAcsProp++;
					}
					*((uint32*)pDataAcsProp->pData) = u32PageData;
					break;
			
                case ITEM_DAT_D2U32:
                case ITEM_DAT_U32_ID_RO:
				case ITEM_DAT_ACQ_U32:
				case ITEM_DAT_pTIME:
					*((uint32*)pDataAcsProp->pData + 1) = 0;    /* ITEM_DAT_ACQ_U32这里应该是浮点数，但因为浮点数和整数都是0 */
				case ITEM_DAT_U32:
				case ITEM_DAT_U32_RO:
				case ITEM_DAT_T32_RO:
				case ITEM_DAT_IPv4:
				case ITEM_DAT_BOOL:
				case ITEM_DAT_ENUM:
				case ITEM_DAT_TELE_ITEM:
					*((uint32*)pDataAcsProp->pData) = pDataAcsProp->LkId_Val_ItN;
					break;
				
				case ITEM_DAT_ACQ_F32:
					*((float32*)pDataAcsProp->pData + 1) = 0;
				case ITEM_DAT_F32:
					*((float32*)pDataAcsProp->pData) = ((float32)pDataAcsProp->LkId_Val_ItN)*0.001f;
					break;
				
				case ITEM_DAT_I64:
                case ITEM_DAT_I64_RO:
					Swi_disable();
					*((int64*)pDataAcsProp->pData) = (int64)pDataAcsProp->LkId_Val_ItN;
					Swi_enable();
					break;
					
				case ITEM_DAT_T64_RO:					
					Swi_disable();
					*((uint64*)pDataAcsProp->pData) = pDataAcsProp->LkId_Val_ItN;
					Swi_enable();
					break;
				
				case ITEM_DAT_D2F32:
					*((float32*)pDataAcsProp->pData) = 0;
					*((float32*)pDataAcsProp->pData + 1) = ((float32)pDataAcsProp->LkId_Val_ItN)*0.001f;
					break;

				case ITEM_DAT_RMTS:
				case ITEM_DAT_DIGITSTR:
				case ITEM_DAT_ANSI:
				case ITEM_DAT_ANSI_RO:
					if(pDataAcsProp->LkId_Val_ItN == 0) {
						*(char*)pDataAcsProp->pData = 0;
					} else {
						strcpy((char*)pDataAcsProp->pData, (char*)pDataAcsProp->LkId_Val_ItN);
					}
					break;
				
				case ITEM_DAT_TEMPSENCONF:
					*((uint32*)pDataAcsProp->pData + 0) = 0;
					*((uint32*)pDataAcsProp->pData + 1) = 0;
					*((uint32*)pDataAcsProp->pData + 2) = 0;
					*((uint32*)pDataAcsProp->pData + 3) = 0;
					break;
					
				case ITEM_DAT_SUMM:
					*((int32*)pDataAcsProp->pData + 0) = 0;
					*((float32*)pDataAcsProp->pData + 1) = ((float32)pDataAcsProp->LkId_Val_ItN)*0.001f;
					*((float32*)pDataAcsProp->pData + 2) = *((float32*)pDataAcsProp->pData + 1);
					*((uint32*)pDataAcsProp->pData + 3) = 0;
					*((float32*)pDataAcsProp->pData + 4) = 0;
					break;

				default:
					break;
			}
		}
		pDataAcsProp++;
	}
}

/* 修复软件版本号和配置缺失 */
void FixSoftVerAndConf(void)
{
	/* 检查版本 */
	if(CheckSoftVer()) {
		g_PubSoftAuthCtr.uTmr_IniPubCntNoPwd_ms = 60000;	/* 可成为为母版 */
	}
	/* 把初始化失败的变量按照默认值或者部分初始化成功的值进行存储 */
	CONF_SAVE_GROUP AcsGrp;
	for(AcsGrp = SAVE_GRP_BRD; AcsGrp < MAX_SAVE_GRP_NUM; AcsGrp++) {
		g_DataAcsIntf.bConfHaveUnsavedChange[AcsGrp] = (g_DataAcsIntf.eConfInitRes[AcsGrp] != DATA_ACS_RES_SUC);
	}
}

/*==========================================================================
| Description	: 计算配置项目数据长度
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2016-10-31
\=========================================================================*/
uint8 CalDataBLenFromItemDataType(ITEM_DATA_TYPE DataType)
{
	switch(DataType) {
		case ITEM_DAT_TOPIC:
		case ITEM_DAT_U32_RO:
		case ITEM_DAT_T32_RO:
		case ITEM_DAT_U32:
		case ITEM_DAT_F32:
		case ITEM_DAT_IPv4:
		case ITEM_DAT_ENUM:
		case ITEM_DAT_BOOL:
		case ITEM_DAT_TELE_ITEM:
		case ITEM_DAT_ACQ_U32:
		case ITEM_DAT_ACQ_F32:
			return 4;

		case ITEM_DAT_I64:
        case ITEM_DAT_I64_RO:
		case ITEM_DAT_T64_RO:
		case ITEM_DAT_D2U32:
		case ITEM_DAT_D2F32:
		case ITEM_DAT_pTIME:
		case ITEM_DAT_U32_ID_RO:
			return 8;

		case ITEM_DAT_SUMM:
		case ITEM_DAT_TEMPSENCONF:
			return 16;
		
		case ITEM_DAT_ANSI:
		case ITEM_DAT_ANSI_RO:
		case ITEM_DAT_RMTS:
		case ITEM_DAT_DIGITSTR:
			return CONF_ANSI_BYTE_LEN;

		default:
			return 0;
	}
}

/*==========================================================================
| Description	: 清除Stat数据
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-11-04
\=========================================================================*/
void ResetStatData(void)
{
#if (SUPPORT_FAT_FS || CPU_0ST_1TI)
	CONF_n_STAT_PROP* pDataAcsProp;
	int16 ItemNum = pBootLoaderConf->u32ItemNum;
	int16 ItemNo;
#endif
#if CPU_0ST_1TI == 1
	/* 为了保持兼容性，之前版本 u32CrcForEEProm值是0xFFFFFFFF，AcsMedia[0].u32OrigEEAddr是0；因此这个地方初始化用u32OrigEEAddr-1 */
	uint32 u32CrcForEEProm = pBootLoaderConf->AcsMedia[0].u32OrigEEAddr - 1;
	uint32 u32EEPromByteAddr = pBootLoaderConf->AcsMedia[SAVE_GRP_STAT].u32OrigEEAddr;
	int16 ItemNum = pBootLoaderConf->u32ItemNum;
	int16 ItemNo;

    EEPromErase(u32EEPromByteAddr);
	/* 初始化变量并写入EEPROM */
	pDataAcsProp = pBootLoaderConf->pConfProp;
	for(ItemNo = ItemNum; ItemNo > 0; ItemNo--) {
		if((pDataAcsProp->ItemType.SaveGrp == SAVE_GRP_STAT) && (pDataAcsProp->pData != NULL)) {
			switch(pDataAcsProp->ItemType.DataType) {
				case ITEM_DAT_I64:
                case ITEM_DAT_I64_RO:
					Swi_disable();
					*((int64*)pDataAcsProp->pData) = 0;
					Swi_enable();
					break;
				
				case ITEM_DAT_D2U32:
					*(((uint32*)pDataAcsProp->pData) + 1) = 0;
					break;

				default:
					break;
			}
			EEPROMProgram((uint32_t*)(pDataAcsProp->pData), u32EEPromByteAddr, 8);
			u32CrcForEEProm = CalCRC32ForFileByIni((uint8*)(pDataAcsProp->pData), 8, u32CrcForEEProm);
			u32EEPromByteAddr += 8;
		}
		pDataAcsProp++;
	}
	/* EEPROM Crc部分 */
	EEPROMProgram((uint32_t*)(&u32CrcForEEProm), u32EEPromByteAddr, 4);
#endif

#if SUPPORT_FAT_FS
	/* 生成配置目录 */
	uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/CONF/STAT.txt");
	*pFilePath = 0;

	/* 写入文件 */
	FIL File;
	pDataAcsProp = pBootLoaderConf->pConfProp;
	if(f_open(&File, (const TCHAR*)g_NvMemAcsCtr.u8FilePath, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)	{
		return;
	}

	UINT u32SavedByteNum, u32ByteNumToBeSaved;			/* 成功保存的字节数 */
	uint32 u32CrcForFile = 0xFFFFFFFF;
	uint8* pBuf;
	for(ItemNo = ItemNum; ItemNo > 0; ItemNo--) {
		if((pDataAcsProp->ItemType.SaveGrp == SAVE_GRP_STAT) && (pDataAcsProp->pData != NULL)) {
			pBuf = g_NvMemAcsCtr.u8FileCont;

			/* 0. 写入ItemID */
			PrintU32(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], pDataAcsProp->u32ItemID, 0);
		
			/* 1. 写入提示符部分 */
			if(g_Sys.u32LocalLang >= MAX_LANG_TYPE) {
				g_Sys.u32LocalLang = CHINESE;
			}
			PrintString(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], pDataAcsProp->pName[g_Sys.u32LocalLang]);
			for( ; pBuf < &g_NvMemAcsCtr.u8FileCont[CONF_SAVE_NAME_BLEN]; pBuf++) {
				*pBuf = ' ';
			}

			/* 2. 如果有数据项，则写入数据 */
			switch(pDataAcsProp->ItemType.DataType) {
				case ITEM_DAT_I64:
                case ITEM_DAT_I64_RO:
					PrintI64(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], *((int64*)pDataAcsProp->pData), pDataAcsProp->ItemType.uBcdQ);
					break;

				case ITEM_DAT_D2U32:
					PrintU32(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], *((uint32*)pDataAcsProp->pData), pDataAcsProp->ItemType.uBcdQ);
					*pBuf++ = ' ';
					PrintU32(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], *(((uint32*)pDataAcsProp->pData) + 1), pDataAcsProp->ItemType.uBcdQ);
					break;
			}

			/* 3. 回车换行 */
			*pBuf = 0x0D;
			*(pBuf + 1) = 0x0A;
			pBuf += 2;

			/* 4. 计算CRC并保存当前行 */
			u32ByteNumToBeSaved = pBuf - g_NvMemAcsCtr.u8FileCont;
			u32CrcForFile = CalCRC32ForFileByIni(g_NvMemAcsCtr.u8FileCont, u32ByteNumToBeSaved, u32CrcForFile);
			f_write(&File, g_NvMemAcsCtr.u8FileCont, u32ByteNumToBeSaved, &u32SavedByteNum);
		}

		/* 6. 准备下一行 */
		pDataAcsProp++;
	}

	/* 添加校验，并保存、关闭文件 */
	pBuf = g_NvMemAcsCtr.u8FileCont;
	PrintString(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], "校验码(如果手动修改本文件,需把冒号后的校验码改为12345678):");
	u32CrcForFile = CalCRC32ForFileByIni(g_NvMemAcsCtr.u8FileCont, pBuf - g_NvMemAcsCtr.u8FileCont, u32CrcForFile);
	PrintH32(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], u32CrcForFile);
	*pBuf = 0x0D;
	*(pBuf + 1) = 0x0A;
	pBuf += 2;
	f_write(&File, g_NvMemAcsCtr.u8FileCont, pBuf - g_NvMemAcsCtr.u8FileCont, &u32SavedByteNum);
	f_close(&File);
#endif
}

/*==========================================================================
* EEProm存取配置数据，主要包括如下函数：
* 1. SaveConfToEEProm() 把配置数据写入EEProm
* 2. CheckConfFromEEProm() 检查EEProm中的数据和内存中的是否一致
* 3. ReadConfFromEEProm() 从EEProm中读取配置数据
* 4. CheckConfValidFromEEProm() 检查EEProm中的配置数据完整性，供ReadConfFromEEProm()调用判断配置数据完整性
\=========================================================================*/
/*==========================================================================
| Description	: 把配置数值部分以二进制的形式存储在eeprom中，并进行crc32校验
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
#define CONF_ITEM_MAX_DATA_U32LEN 	(CONF_ANSI_BYTE_LEN/4)	/* 必须以最长的配置项目为准 */
#define CONF_EEDATA_BY_HIGH_FLASH	0xAAAAAAAAUL	/* eeprom中存储的配置数据是高一半FLASH软件所写入 */
#define CONF_EEDATA_BY_LOW_FLASH	0x55555555UL	/* eeprom中存储的配置数据是低一半FLASH软件所写入 */
void SaveConfToEEProm(CONF_SAVE_GROUP AcsGrp, uint32 u32EEPromByteAddr)
{
	CONF_n_STAT_PROP* pDataAcsProp;
	uint32_t u32DataBuf[CONF_ITEM_MAX_DATA_U32LEN];
	uint32 u32ByteLen, u32ItemNum, u32FlashBankFlag;
	/* 为了保持兼容性，之前版本 u32CrcForEEProm值是0xFFFFFFFF，AcsMedia[0].u32OrigEEAddr是0；因此这个地方初始化用u32OrigEEAddr-1 */
	uint32 u32CrcForEEProm = pBootLoaderConf->AcsMedia[0].u32OrigEEAddr - 1;
	int16 i;

#if LEGACY
	if((u32EEPromByteAddr == 0) || (u32EEPromByteAddr >= 96*64)) {			/* 输入检查 */
		return;
	}
#else
	/* 输入地址范围检查 */
	if(!WITHIN_SAVE_GRP_EEPROM_ADDR(u32EEPromByteAddr)) {
		return;
	}
#endif

#if CPU_0ST_1TI == 1
    EEPromErase(u32EEPromByteAddr);		/* TI内置的EEPROM支持分区擦除 */
#endif

	/* 写入:eeprom数据所对应的配置描述--如果是用高一半flash，则配置表描述为0xAAAAAAAAUL, 否则0x55555555UL */
	if(CheckExeFlash_0Low_1High()) {
		u32FlashBankFlag = CONF_EEDATA_BY_HIGH_FLASH;
	} else {
		u32FlashBankFlag = CONF_EEDATA_BY_LOW_FLASH;
	}
	u32ByteLen = 4;
	EEPROMRead((uint32_t*)u32DataBuf, u32EEPromByteAddr, u32ByteLen);
	if(u32DataBuf[0] != u32FlashBankFlag) {
		EEPROMProgram((uint32_t*)(&u32FlashBankFlag), u32EEPromByteAddr, u32ByteLen);
	}
	u32CrcForEEProm = CalCRC32ForFileByIni((uint8*)&u32FlashBankFlag, u32ByteLen, u32CrcForEEProm);
	u32EEPromByteAddr += u32ByteLen;

	/* 写入:数据部分 */
	pDataAcsProp = pBootLoaderConf->pConfProp;
	u32ItemNum = pBootLoaderConf->u32ItemNum;
	for( ; u32ItemNum > 0; u32ItemNum--) {
		if((pDataAcsProp->ItemType.SaveGrp == AcsGrp) && (pDataAcsProp->pData != NULL)) {
			u32ByteLen = CalDataBLenFromItemDataType(pDataAcsProp->ItemType.DataType);
			if(u32ByteLen && (u32ByteLen <= CONF_ITEM_MAX_DATA_U32LEN*4)) {
				EEPROMRead((uint32_t*)u32DataBuf, u32EEPromByteAddr, u32ByteLen);
				for(i = u32ByteLen/4 - 1; i >= 0; i--) {
					if(u32DataBuf[i] != *((uint32*)(pDataAcsProp->pData) + i)) {
						break;
					}
				}
				if(i >= 0) {
					Swi_disable();	/* 锁存配置数据，以免存储中被更改 */
					for(i = u32ByteLen/4 - 1; i >= 0; i--) {
						u32DataBuf[i] = *((uint32*)(pDataAcsProp->pData) + i);
					}
					Swi_enable();
					EEPROMProgram(u32DataBuf, u32EEPromByteAddr, u32ByteLen);
#if 0
uint8 printfBuf[80] = {0};
sprintf(printfBuf, "Saving, Group:%d, EEaddr:%8X, Val:%d", pDataAcsProp->ItemType.SaveGrp, u32EEPromByteAddr, *((uint32*)u32DataBuf));
HAL_UART_Transmit(&huart2, printfBuf, 40, 1000);
#endif
				}
				u32CrcForEEProm = CalCRC32ForFileByIni((uint8*)u32DataBuf, u32ByteLen, u32CrcForEEProm);
				u32EEPromByteAddr += u32ByteLen;
			}
		}
		pDataAcsProp++;
	}

	/* 写入:CRC部分 */
	EEPROMRead((uint32_t*)u32DataBuf, u32EEPromByteAddr, 4);
	if(u32DataBuf[0] != u32CrcForEEProm) {
		EEPROMProgram((uint32_t*)(&u32CrcForEEProm), u32EEPromByteAddr, 4);
	}
}

/*==========================================================================
| Description	: 检查eeprom中的配置数据
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
BOOL CheckConfFromEEProm(CONF_SAVE_GROUP AcsGrp, uint32 u32EEPromByteAddr)
{
#if LEGACY
	if((u32EEPromByteAddr == 0) || (u32EEPromByteAddr >= 96*64)) {	/* 输入检查 */
#else 
	if(!WITHIN_SAVE_GRP_EEPROM_ADDR(u32EEPromByteAddr)) {	/* 输入检查 */
#endif
		return FALSE;
	}
	/* 配置表检查--如果是用高一半flash，则配置表描述为0xAAAAAAAAUL, 否则0x55555555UL */
	uint32 u32ByteLen = 4;
	uint32 u32DataBuf[CONF_ITEM_MAX_DATA_U32LEN];
	EEPROMRead((uint32_t*)u32DataBuf, u32EEPromByteAddr, u32ByteLen);
	if((CheckExeFlash_0Low_1High() && (u32DataBuf[0] != CONF_EEDATA_BY_HIGH_FLASH))
		|| ((!CheckExeFlash_0Low_1High()) && (u32DataBuf[0] != CONF_EEDATA_BY_LOW_FLASH))) 
	{
		return FALSE;
	}
    /* 为了保持兼容性，之前版本 u32CrcForEEProm值是0xFFFFFFFF，AcsMedia[0].u32OrigEEAddr是0；因此这个地方初始化用u32OrigEEAddr-1 */
	uint32 u32CrcForEEProm = pBootLoaderConf->AcsMedia[0].u32OrigEEAddr - 1;
	u32CrcForEEProm = CalCRC32ForFileByIni((uint8*)u32DataBuf, u32ByteLen, u32CrcForEEProm);
	u32EEPromByteAddr += u32ByteLen;

	/* 数据检查 */
	CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
	uint32 u32ItemNum = pBootLoaderConf->u32ItemNum;
	for( ; u32ItemNum > 0; u32ItemNum--) {
		if((pDataAcsProp->ItemType.SaveGrp == AcsGrp) && (pDataAcsProp->pData != NULL)) {
			u32ByteLen = CalDataBLenFromItemDataType(pDataAcsProp->ItemType.DataType);
			if(u32ByteLen && (u32ByteLen <= CONF_ITEM_MAX_DATA_U32LEN*4)) {
				EEPROMRead((uint32_t*)u32DataBuf, u32EEPromByteAddr, u32ByteLen);
				int16 i;
				for(i = u32ByteLen/4 - 1; i >= 0; i--) {
					if(u32DataBuf[i] != *((uint32*)(pDataAcsProp->pData) + i)) {
						return FALSE;
					}
				}
				u32CrcForEEProm = CalCRC32ForFileByIni((uint8*)u32DataBuf, u32ByteLen, u32CrcForEEProm);
				u32EEPromByteAddr += u32ByteLen;
			}
		}
		pDataAcsProp++;
	}
	
	/* CRC检查 */
	EEPROMRead((uint32_t*)u32DataBuf, u32EEPromByteAddr, 4);
	if(u32DataBuf[0] == u32CrcForEEProm) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/*==========================================================================
| Description	: 从eeprom中读取配置数据
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
uint32 CheckConfValidFromEEProm(CONF_SAVE_GROUP AcsGrp, BOOL bConfAcsDscp_New0_Old1, uint32 u32EEPromByteAddr);
DATA_ACS_OPR ReadConfFromEEProm(CONF_SAVE_GROUP AcsGrp)
{
	uint32 u32EEPromByteAddr;

#if 0
	if(AcsGrp == SAVE_GRP_MCONF) {
		g_CodeTest.u32Val[35]= CheckConfValidFromEEProm(AcsGrp, 0, pBootLoaderConf->AcsMedia[AcsGrp].u32OrigEEAddr);		//00000000
		g_CodeTest.u32Val[36]= CheckConfValidFromEEProm(AcsGrp, 0, pBootLoaderConf->AcsMedia[AcsGrp].u32BackEEAddr);		//00000000
		g_CodeTest.u32Val[37]= (AcsGrp < pOldBootLoaderConf->u32ConfSaveGroupNum);											//00000001
		g_CodeTest.u32Val[38]= pOldBootLoaderConf->u32ConfSaveGroupNum;														//00000006
		g_CodeTest.u32Val[39]= CheckConfValidFromEEProm(AcsGrp, 1, pOldBootLoaderConf->AcsMedia[AcsGrp].u32OrigEEAddr);		//00000000
		g_CodeTest.u32Val[34]= CheckConfValidFromEEProm(AcsGrp, 1, pOldBootLoaderConf->AcsMedia[AcsGrp].u32BackEEAddr);		//00000000
	}
#endif

	/* 检查eeprom是否当前配置表 */
	if(pBootLoaderConf->AcsMedia[AcsGrp].u32OrigEEAddr == 0)	{	/* 未配置保存的,给予成功标志,免得后面维护EEConf的时候存储 */
		return DATA_ACS_RES_SUC;
	} else if(((u32EEPromByteAddr = CheckConfValidFromEEProm(AcsGrp, 0, pBootLoaderConf->AcsMedia[AcsGrp].u32OrigEEAddr)) != 0)
		|| ((u32EEPromByteAddr = CheckConfValidFromEEProm(AcsGrp, 0, pBootLoaderConf->AcsMedia[AcsGrp].u32BackEEAddr)) != 0))
	{
		/* 读取数据 */
		CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
		uint32 u32ItemNum = pBootLoaderConf->u32ItemNum;
		for( ; u32ItemNum > 0; u32ItemNum--) {
			if((pDataAcsProp->ItemType.SaveGrp == AcsGrp) && (pDataAcsProp->pData != NULL)) {
				uint32 u32ByteLen = CalDataBLenFromItemDataType(pDataAcsProp->ItemType.DataType);
				if(u32ByteLen) {
					Swi_disable();
					EEPROMRead((uint32_t*)(pDataAcsProp->pData), u32EEPromByteAddr, u32ByteLen);
//					#include "usart.h"
//					extern UART_HandleTypeDef huart2;
//					uint8 printfBuf[80] = {0};
//					sprintf(printfBuf, "Reading,Group:%d, EEaddr:%8X, val:%d\r\n", pDataAcsProp->ItemType.SaveGrp, u32EEPromByteAddr, *((uint32_t*)(pDataAcsProp->pData)));
//					HAL_UART_Transmit(&huart2, printfBuf, 40, 1000);
					Swi_enable();
					u32EEPromByteAddr += u32ByteLen;
				}
			}
			pDataAcsProp++;
		}
		return DATA_ACS_RES_SUC;
	} else if((AcsGrp < pOldBootLoaderConf->u32ConfSaveGroupNum)
			&& (((u32EEPromByteAddr = CheckConfValidFromEEProm(AcsGrp, 1, pOldBootLoaderConf->AcsMedia[AcsGrp].u32OrigEEAddr)) != 0)
				|| ((u32EEPromByteAddr = CheckConfValidFromEEProm(AcsGrp, 1, pOldBootLoaderConf->AcsMedia[AcsGrp].u32BackEEAddr)) != 0)))
	{
		/* 统计需要初始化多少项目 */
		uint32 u32ItemNum = 0;
		CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
		int16 i;
		for(i = pBootLoaderConf->u32ItemNum; i > 0; i--) {
			if(pDataAcsProp->ItemType.SaveGrp == AcsGrp) {
				u32ItemNum++;
			}
			pDataAcsProp++;
		}
		
		/* 读取数据 */
		uint32 u32OldItemNum;
		CONF_n_STAT_PROP* pOldDataAcsProp = (CONF_n_STAT_PROP*)((uint32)pOldBootLoaderConf->pConfProp + FLASH_HALF_BLEN);
		for(u32OldItemNum = pOldBootLoaderConf->u32ItemNum; u32OldItemNum > 0; u32OldItemNum--) {
			if((pOldDataAcsProp->ItemType.SaveGrp == AcsGrp) && (pOldDataAcsProp->pData != NULL)) {	/* 在老配置表中搜索符合的存储定义 */
				uint32 u32ByteLen = CalDataBLenFromItemDataType(pOldDataAcsProp->ItemType.DataType);
				if(u32ByteLen) {	/* 确有数据存储项目--有类型、有数据 */
					/* 在新的配置表中搜索相同ID的配置项 */
					pDataAcsProp = pBootLoaderConf->pConfProp;
					for(i = pBootLoaderConf->u32ItemNum; i > 0; i--) {
						if((pDataAcsProp->u32ItemID == pOldDataAcsProp->u32ItemID) /* 找到同样的配置ID，则把数据写入 */
							&& (pDataAcsProp->ItemType.SaveGrp == AcsGrp))
						{
						    if((pDataAcsProp->ItemType.DataType == pOldDataAcsProp->ItemType.DataType) 
    						    || ((pDataAcsProp->ItemType.DataType == ITEM_DAT_I64) 
                                    && (pOldDataAcsProp->ItemType.DataType == ITEM_DAT_I64_RO))
                                || ((pDataAcsProp->ItemType.DataType == ITEM_DAT_I64_RO) 
                                    && (pOldDataAcsProp->ItemType.DataType == ITEM_DAT_I64))
                                || ((pDataAcsProp->ItemType.DataType == ITEM_DAT_ACQ_F32) 
                                    && (pOldDataAcsProp->ItemType.DataType == ITEM_DAT_F32))
                                || ((pDataAcsProp->ItemType.DataType == ITEM_DAT_F32) 
                                    && (pOldDataAcsProp->ItemType.DataType == ITEM_DAT_ACQ_F32))
                                || ((pDataAcsProp->ItemType.DataType == ITEM_DAT_ACQ_U32) 
                                    && (pOldDataAcsProp->ItemType.DataType == ITEM_DAT_U32))
                                || ((pDataAcsProp->ItemType.DataType == ITEM_DAT_U32) 
                                    && (pOldDataAcsProp->ItemType.DataType == ITEM_DAT_ACQ_U32))
                                || ((pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_ID_RO) 
                                    && (pOldDataAcsProp->ItemType.DataType == ITEM_DAT_U32_RO))
								|| ((pDataAcsProp->ItemType.DataType == ITEM_DAT_pTIME) 
                                    && (pOldDataAcsProp->ItemType.DataType == ITEM_DAT_D2U32))
								|| ((pDataAcsProp->ItemType.DataType == ITEM_DAT_D2U32) 
                                    && (pOldDataAcsProp->ItemType.DataType == ITEM_DAT_pTIME)))
						    {
    							Swi_disable();
    							EEPROMRead((uint32_t*)(pDataAcsProp->pData), u32EEPromByteAddr, u32ByteLen);
    							Swi_enable();
    							u32ItemNum--;
    							break;
    						} else if(((pDataAcsProp->ItemType.DataType == ITEM_DAT_U32) 
                                        && (pOldDataAcsProp->ItemType.DataType == ITEM_DAT_TEMPSENCONF))
                                    || ((pDataAcsProp->ItemType.DataType == ITEM_DAT_TEMPSENCONF) 
                                        && (pOldDataAcsProp->ItemType.DataType == ITEM_DAT_U32)))
                            {
    							Swi_disable();
    							EEPROMRead((uint32_t*)(pDataAcsProp->pData), u32EEPromByteAddr, CalDataBLenFromItemDataType(ITEM_DAT_U32));
    							Swi_enable();
    							u32ItemNum--;
    							break;
							} else if((pDataAcsProp->ItemType.DataType == ITEM_DAT_F32) 
							        && (pOldDataAcsProp->ItemType.DataType == ITEM_DAT_U32))
						    {
    							Swi_disable();
    							EEPROMRead((uint32_t*)(pDataAcsProp->pData), u32EEPromByteAddr, u32ByteLen);
    							*(float32*)pDataAcsProp->pData = ((float32)(*(uint32*)pDataAcsProp->pData))
    							                                  /cnst_fBcdQFactor[pOldDataAcsProp->ItemType.uBcdQ];
    							Swi_enable();
    							u32ItemNum--;
    							break;
                            } else if((pDataAcsProp->ItemType.DataType == ITEM_DAT_U32) 
                                    && (pOldDataAcsProp->ItemType.DataType == ITEM_DAT_F32))
                            {
    							Swi_disable();
    							EEPROMRead((uint32_t*)(pDataAcsProp->pData), u32EEPromByteAddr, u32ByteLen);
    							*(uint32*)pDataAcsProp->pData = F32ToU32((*(float32*)pDataAcsProp->pData)
    							                                  *cnst_fBcdQFactor[pDataAcsProp->ItemType.uBcdQ]);
    							Swi_enable();
    							u32ItemNum--;
    							break;
                            }
						}
						pDataAcsProp++;
					}
					
					u32EEPromByteAddr += u32ByteLen;			/* 修正EEProm   */
				}
			}
			pOldDataAcsProp++;
		}

		if(u32ItemNum) {
			return DATA_ACS_RES_NOT_CMPLT;
		} else {
			return DATA_ACS_RES_SUC_BUT_OLD;
		}
	}
	
	return DATA_ACS_RES_FAIL;
}

/* 检查eeprom配置数据完整性，返回配置数据在EEProm中的地址 */
uint32 CheckConfValidFromEEProm(CONF_SAVE_GROUP AcsGrp, BOOL bConfAcsDscp_New0_Old1, uint32 u32EEPromByteAddr)
{
	uint32 reslt = 0;
	uint32 u32DataBuf[CONF_ITEM_MAX_DATA_U32LEN];
	uint32 u32EEPromByteAddrStart = u32EEPromByteAddr;

	/* 输入检查 */
	if(!WITHIN_SAVE_GRP_EEPROM_ADDR(u32EEPromByteAddr)) {
//		return 0;
			if(AcsGrp == SAVE_GRP_MCONF) {
				reslt = 1;
				goto end;
			} else {
				return 0;
			}

	}
	g_CodeTest.u32Val[32]++;		//0000000C

	CONF_n_STAT_PROP* pDataAcsProp;
	uint32 u32ItemNum, u32CrcForEEProm;
	/* 为了保持兼容性，之前版本 u32CrcForEEProm值是0xFFFFFFFF，AcsMedia[0].u32OrigEEAddr是0；因此这个地方初始化用u32OrigEEAddr-1 */
	if(bConfAcsDscp_New0_Old1) {
		pDataAcsProp = (CONF_n_STAT_PROP*)((uint32)pOldBootLoaderConf->pConfProp + FLASH_HALF_BLEN);
		u32ItemNum = pOldBootLoaderConf->u32ItemNum;
        u32CrcForEEProm = pOldBootLoaderConf->AcsMedia[0].u32OrigEEAddr - 1;
	} else {
		pDataAcsProp = pBootLoaderConf->pConfProp;
		u32ItemNum = pBootLoaderConf->u32ItemNum;
		u32CrcForEEProm = pBootLoaderConf->AcsMedia[0].u32OrigEEAddr - 1;
	}

	/* 头部标志检查 */
	uint32 u32ByteLen = 4;
	EEPROMRead((uint32_t*)u32DataBuf, u32EEPromByteAddr, u32ByteLen);
	if(bConfAcsDscp_New0_Old1 ^ CheckExeFlash_0Low_1High()) {
		if(u32DataBuf[0] != CONF_EEDATA_BY_HIGH_FLASH) {
//			return 0;
			if(AcsGrp == SAVE_GRP_MCONF) {
				reslt = 2;
				goto end;
			} else {
				return 0;
			}

		}
	} else {
		if(u32DataBuf[0] != CONF_EEDATA_BY_LOW_FLASH) {
//			return 0;
			if(AcsGrp == SAVE_GRP_MCONF) {
				reslt = 3;
				goto end;
			} else {
				return 0;
			}
		}
	}
	u32CrcForEEProm = CalCRC32ForFileByIni((uint8*)u32DataBuf, u32ByteLen, u32CrcForEEProm);
	u32EEPromByteAddr += u32ByteLen;

	/* CRC计算 */
	for( ; u32ItemNum > 0; u32ItemNum--) {
		if((pDataAcsProp->ItemType.SaveGrp == AcsGrp) && (pDataAcsProp->pData != NULL)) {
			u32ByteLen = CalDataBLenFromItemDataType(pDataAcsProp->ItemType.DataType);
			if(u32ByteLen && (u32ByteLen <= CONF_ITEM_MAX_DATA_U32LEN*4)) {
				EEPROMRead((uint32_t*)u32DataBuf, u32EEPromByteAddr, u32ByteLen);
				u32CrcForEEProm = CalCRC32ForFileByIni((uint8*)u32DataBuf, u32ByteLen, u32CrcForEEProm);
				u32EEPromByteAddr += u32ByteLen;
			}
		}
		pDataAcsProp++;
	}


	/* CRC检查 */
//	return u32EEPromByteAddrStart + 4;	/* 需要跳过起始的高/低一半flash标志部分 */
	EEPROMRead((uint32_t*)u32DataBuf, u32EEPromByteAddr, 4);
	if(u32DataBuf[0] == u32CrcForEEProm) {
//		return u32EEPromByteAddrStart + 4;	/* 需要跳过起始的高/低一半flash标志部分 */
		if(AcsGrp == SAVE_GRP_MCONF) {
			reslt = u32EEPromByteAddrStart + 4;
			goto end;
		} else {
			return u32EEPromByteAddrStart + 4;	/* 需要跳过起始的高/低一半flash标志部分 */
		}
	} else {
		if(AcsGrp == SAVE_GRP_MCONF) {
			reslt = 0;
			goto end;
		} else {
			return 0;
		}
	}
end:

	if(AcsGrp == SAVE_GRP_MCONF) {
		if(reslt == 0) {
//			g_CodeTest.uVal[u8DebugStep] = 10;
//			g_CodeTest.uVal[10+u8DebugStep]++;
			g_CodeTest.u32Val[20]++;
			reslt = 0;
		} else if(reslt == 1) {
//			g_CodeTest.uVal[u8DebugStep] = 11;
			g_CodeTest.u32Val[21]++;
//			g_CodeTest.uVal[10+u8DebugStep]++;
			reslt = 0;
		} else if(reslt == 2) {
//			g_CodeTest.uVal[u8DebugStep] = 12;									//"u1":"12", "u2":"12", "u3":"10", "u4":"10",     "u11":"1", "u12":"1", "u13":"1", "u14":"1
//			g_CodeTest.uVal[10+u8DebugStep]++;
			g_CodeTest.u32Val[22]++;
			reslt = 0;
//			if(bConfAcsDscp_New0_Old1 == 0) {
//				g_CodeTest.u32Val[20] =AcsGrp;		//00000002
//				g_CodeTest.u32Val[21] =bConfAcsDscp_New0_Old1;	//00000000
//				g_CodeTest.u32Val[22] =u32EEPromByteAddr;//08084140
//				g_CodeTest.u32Val[23] =reslt;//00000000
//				g_CodeTest.u32Val[24]++;//00000002
//			} else {
//				g_CodeTest.u32Val[25] =AcsGrp;//00000002
//				g_CodeTest.u32Val[26] =bConfAcsDscp_New0_Old1;//00000001
//				g_CodeTest.u32Val[27] =u32EEPromByteAddr;//08084144
//				g_CodeTest.u32Val[28] =reslt;//00000000
//				g_CodeTest.u32Val[29]++;//00000002
//			}
		} else if(reslt == 3) {
//			g_CodeTest.uVal[u8DebugStep] = 13;
//			g_CodeTest.uVal[10+u8DebugStep]++;
			g_CodeTest.u32Val[23]++;
			reslt = 0;
	    } else {
//			g_CodeTest.u32Val[25]++;		//00000002
			if(bConfAcsDscp_New0_Old1 == 0) {
//				g_CodeTest.uVal[u8DebugStep] = 13;
//				g_CodeTest.uVal[10+u8DebugStep]++;
				g_CodeTest.u32Val[23]++;
				g_CodeTest.u32Val[30] = u32EEPromByteAddr;//00000000
			} else {
//				g_CodeTest.uVal[u8DebugStep] = 14;
//				g_CodeTest.uVal[10+u8DebugStep]++;
				g_CodeTest.u32Val[24]++;
				g_CodeTest.u32Val[31] = u32EEPromByteAddr;//00000000
			}
	    }
	}


	return reslt;
}

#if SUPPORT_FAT_FS
/*==========================================================================
| Description	: 通过文件系统存储conf、msg、acq
	各种文件名/目录操作: 生成文件名、打开文件、关闭文件、检查并产生目录
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-06-24
\=========================================================================*/
void ChkAndMkDir(void)
{
	/* 检查文件系统:如果文件系统不存在需要按fat32格式格式化TF卡 */
	if(f_stat("0:", NULL) == FR_NO_FILESYSTEM) {
		f_mkfs(0, FM_FAT32, 0, g_NvMemAcsCtr.fatfsTempSpace, _MAX_SS);
	}
	
	/* 检查根目录 */
	if((f_stat((const TCHAR*)"0:/Yoplore", NULL)) != FR_OK) {
		f_mkdir((const TCHAR*)"0:/Yoplore");
	}

	/* 检查 设备 目录 */
	uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
	*pFilePath = 0;
	if((f_stat((const TCHAR*)g_NvMemAcsCtr.u8FilePath, NULL)) != FR_OK) {
		f_mkdir((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
	}

	/* 检查 /CONF 目录 */
	uint8* pFilePathBak = pFilePath;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Conf");
	*pFilePath = 0;
	if(f_stat((const TCHAR*)g_NvMemAcsCtr.u8FilePath, NULL) != FR_OK) {
		f_mkdir((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
	}

	/* 检查 /MSG2 目录 */
	pFilePath = pFilePathBak;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Msg2");
	*pFilePath = 0;
	/* f_stat执行时间约2.7ms */
	if(f_stat((const TCHAR*)g_NvMemAcsCtr.u8FilePath, NULL) != FR_OK) {
		f_mkdir((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
	}
	
	/* 检查 /MSG 目录 */
#if SUPPORT_MsgV1RW_AcqV1R
	pFilePath = pFilePathBak;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Msg");
	*pFilePath = 0;
	if(f_stat((const TCHAR*)g_NvMemAcsCtr.u8FilePath, NULL) != FR_OK) {
		f_mkdir((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
	}
#endif
	
	/* 检查 /ACQ 目录 */
	pFilePath = pFilePathBak;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Acq2");
	*pFilePath = 0;
	if(f_stat((const TCHAR*)g_NvMemAcsCtr.u8FilePath, NULL) != FR_OK) {
		f_mkdir((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
	}
}

void EmptyFold(uint8* pFilePathIni)
{
	DIR dir;
	uint8* pFilePath = pFilePathIni;
	*pFilePath = 0;
	while(f_opendir(&dir, (const TCHAR*)g_NvMemAcsCtr.u8FilePath) == FR_OK) {
		*pFilePath++ = '/';
		FILINFO fno;
		BOOL findFold = FALSE;
		while((f_readdir(&dir, &fno) == FR_OK) && fno.fname[0]) {		/* 成功读取文件或文件夹名 */
			int8 i;
			for(i = 0; (i < FILE_NAME_BYTE_LEN) && fno.fname[i]; i++) {	/* 拷贝文件或文件夹名 */
				pFilePath[i] = fno.fname[i];
			}
			if((fno.fattrib & AM_DIR) == 0) {	/* 文件，直接删除 */
				pFilePath[i] = 0;
				f_unlink((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
			} else {				/* 文件夹，则进入下一层级 */
				findFold = TRUE;
				pFilePath += i;
				*pFilePath = 0;		/* 为下一个循环打开目录做准备 */
				break;
			}
		}
		/* 无下级文件夹，则说明本文件夹已经清空 */
		if(!findFold) {
			pFilePath--;	/* 进入本级目录的时候，进行了一次 *pFilePath++ = '/';  */
			if(pFilePath <= pFilePathIni) {		/* 已经到达根目录，且已经清空了，完成任务 */
				break;
			} else {		/* 非根目录，则删除本目录 */
				*pFilePath = 0;
				f_unlink((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
				for(; (pFilePath >= pFilePathIni) && (*pFilePath != '/'); pFilePath--) {	/* 搜索上一个分断符 */
				}
				*pFilePath = 0;	/* 为下一个循环打开目录做准备 */
			}
		}
	}
}

/*==========================================================================
 *	以下为 配置 文件操作函数
 *=========================================================================*/
uint32 GetConfIDInText(uint8* pLineStart);
BOOL ConvTextForNormalConf(uint8* pText, uint8* pTextStart, CONF_n_STAT_PROP* pDataAcsProp, BOOL bRead1_Check0);
BOOL CheckTextWithCrc32ForConf(uint8* pLineStart, uint8* pLineEnd, uint32 u32IniCrc);
BOOL ConvTextForTempSenConf(uint8* pText, uint8* pTextStart, TEMP_SEN_CONF* pData, BOOL bRead1_Check0);
BOOL ConvTextForIPv4(uint8* pText, uint8* pTextStart, uint32* pU32Data, BOOL bRead1_Check0);
BOOL ReadConfStringInText(uint8* pText, uint8* pTextStart, uint8* pU8Data, BOOL bRead1_Check0);
BOOL ConvTextForAcqSumm(uint8* pText, uint8* pTextEnd, ACQ_SUMM* pData, BOOL bRead1_Check0);
BOOL ReadT64InText(uint8* pText, uint8* pTextEnd, REAL_TIME_VAR* pRealTime, BOOL bRead1_Check0);
BOOL ReadPTimeInText(uint8* pText, uint8* pTextEnd, uint32* pU32, BOOL bRead1_Check0);
/*==========================================================================
| Description	: 把配置数据以文本的形式存储在文件中，含提示符部分
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
void SaveConfToFile(CONF_SAVE_GROUP ConfAcsGrp)
{
	/* 生成配置目录 */
	uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
	if(ConfAcsGrp == SAVE_GRP_MCONF) {
    	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/CONF/MAIN.txt");
    } else if(ConfAcsGrp == SAVE_GRP_TSEN) {
    	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/CONF/TSEN.txt");
    } else if(ConfAcsGrp == SAVE_GRP_STAT) {
    	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/CONF/STAT.txt");
    } else {
        return;
    }
	*pFilePath = 0;

	/* 打开文件 */
	FIL File;
	UINT u32SavedByteNum;			/* 成功保存的字节数 */
	if(f_open(&File, (const TCHAR*)g_NvMemAcsCtr.u8FilePath, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)	{
		return;
 	} 

	/* 打印序列号与版本号 */
	uint32 u32CrcForFile = 0xFFFFFFFF;
	uint8* pBuf = g_NvMemAcsCtr.u8FileCont;
	PrintString(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], "0000  ");	/* 避免读取ConfID时候出错 */
	PrintString(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], DEV_TYPE_MQTT_TOPIC_String);
	PrintU32(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], g_Sys.SerialNo.u32Dat, 0);
	*pBuf++ = ' ';
	PrintSoftVer(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], SOFTWARE_VER);
	*pBuf++ = 0x0D;
	*pBuf++ = 0x0A;
	if(pBuf + 200 > &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN]) {	/* 前面应该数据量很少，这个做个判断以防万一 */
		return;
	}
	
	/* 打印配置项 */
	CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
	int16 ItemNum = pBootLoaderConf->u32ItemNum;
	for(; ItemNum > 0; ItemNum--) {
		if((pDataAcsProp->ItemType.SaveGrp == ConfAcsGrp) && (pDataAcsProp->pData != NULL)) {
			pBuf = PrintLocalConfItem(DATA_USER_FILE, pDataAcsProp, pBuf, 100);

			/* 计算CRC并保存当前行 */
			UINT u32ByteNumToBeSaved = pBuf - g_NvMemAcsCtr.u8FileCont;
			if(u32ByteNumToBeSaved + 120 > FILE_CONT_BUF_LEN) {
				u32CrcForFile = CalCRC32ForFileByIni(g_NvMemAcsCtr.u8FileCont, u32ByteNumToBeSaved, u32CrcForFile);
				f_write(&File, g_NvMemAcsCtr.u8FileCont, u32ByteNumToBeSaved, &u32SavedByteNum);
				pBuf = g_NvMemAcsCtr.u8FileCont;
			}
		}

		/* 6. 准备下一行 */
		pDataAcsProp++;
	}

	/* 添加校验，并保存、关闭文件: 后面校验码行不会超长 */
	PrintString(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], "9999  ");
	u32CrcForFile = CalCRC32ForFileByIni(g_NvMemAcsCtr.u8FileCont, pBuf - g_NvMemAcsCtr.u8FileCont, u32CrcForFile);
	PrintH32(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], u32CrcForFile);
	*pBuf = 0x0D;
	*(pBuf + 1) = 0x0A;
	pBuf += 2;
	f_write(&File, g_NvMemAcsCtr.u8FileCont, pBuf - g_NvMemAcsCtr.u8FileCont, &u32SavedByteNum);
	f_close(&File);
}

/*==========================================================================
| Description	: 读出/检查文件中存储的文本形式统计/配置数据，并进行CRC检查
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
DATA_ACS_OPR ReadOrCheckConfFromFile(CONF_SAVE_GROUP ConfAcsGrp, int16 iTryCnt_pRd_nChk)
{
	FIL File;
	uint8* pText;
	uint8* pLineStart;
	uint8* pLineEnd;
	DATA_ACS_OPR DataAcsRes;

	/* 输入检查 */
	if(iTryCnt_pRd_nChk == 0) {
		return DATA_ACS_RES_INPUT_ERR;
	}

	BOOL bRead1_Check0 = (iTryCnt_pRd_nChk > 0);
	if(!bRead1_Check0) {
		iTryCnt_pRd_nChk = 0 - iTryCnt_pRd_nChk;
	}
	
	/* 生成配置目录 */
	uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
	if(ConfAcsGrp == SAVE_GRP_MCONF) {
    	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/CONF/MAIN.txt");
    } else if(ConfAcsGrp == SAVE_GRP_TSEN) {
    	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/CONF/TSEN.txt");
    } else if(ConfAcsGrp == SAVE_GRP_STAT) {
    	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/CONF/STAT.txt");
    } else {
        return DATA_ACS_RES_INPUT_ERR;
    }
	*pFilePath = 0;

	for( ; iTryCnt_pRd_nChk > 0; iTryCnt_pRd_nChk--) {
		DataAcsRes = DATA_ACS_IDLE;
		if(f_open(&File, (const TCHAR*)g_NvMemAcsCtr.u8FilePath, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
			DataAcsRes = DATA_ACS_RES_OPEN_FILE_FAIL;
			continue;
		}
		/* 文件CRC检查 */
		uint32 u32DataFilePt = 0;
		uint32 u32CrcForFile = 0xFFFFFFFF;
		UINT u32ByteNum;
		do {
			f_lseek(&File, u32DataFilePt);
			f_read(&File, g_NvMemAcsCtr.u8FileCont, FILE_CONT_BUF_LEN, &u32ByteNum);
			/* 搜索文件，寻找回车换行 */
			pText = g_NvMemAcsCtr.u8FileCont;
			pLineStart = pText;
			pLineEnd = pLineStart;
			for( ; pText < &g_NvMemAcsCtr.u8FileCont[u32ByteNum]; pText++) {
				/* 找到回车换行符号, 说明一个有效的搜索 */
				if(*pText == 0x0A) {
					pLineStart = pLineEnd;
					pLineEnd = pText+1;
				}
			}
			u32DataFilePt += pLineStart - g_NvMemAcsCtr.u8FileCont;
			u32CrcForFile = CalCRC32ForFileByIni(g_NvMemAcsCtr.u8FileCont, pLineStart - g_NvMemAcsCtr.u8FileCont, u32CrcForFile);
		} while(u32ByteNum == FILE_CONT_BUF_LEN);
		if(!CheckTextWithCrc32ForConf(pLineStart, &g_NvMemAcsCtr.u8FileCont[u32ByteNum], u32CrcForFile)) {
			DataAcsRes = DATA_ACS_RES_CRC_ERR;
			continue;
		}

		/* 统计需要初始化多少项目 */
		int16 ItemNum = 0;
		CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
		int16 i;
		for(i = pBootLoaderConf->u32ItemNum; i > 0; i--) {
			if((pDataAcsProp->ItemType.SaveGrp == ConfAcsGrp) && (pDataAcsProp->pData != NULL)) {
				ItemNum++;
			}
			pDataAcsProp++;
		}

		/* 读取项目 */
		u32CrcForFile = 0xFFFFFFFF;
		u32DataFilePt = 0;								/* 初始化文件读取指针 */
		u32ByteNum = 0;
		pText = g_NvMemAcsCtr.u8FileCont;
		pLineEnd = g_NvMemAcsCtr.u8FileCont;
		while(ItemNum > 0) {
			if(pText >= &g_NvMemAcsCtr.u8FileCont[u32ByteNum]) {
				u32DataFilePt += pLineEnd - g_NvMemAcsCtr.u8FileCont;		/* 修改文件读取指针 */
				f_lseek(&File, u32DataFilePt);
				f_read(&File, g_NvMemAcsCtr.u8FileCont, FILE_CONT_BUF_LEN, &u32ByteNum);
				if(u32ByteNum == 0) {					/* 已经读不出数据 */
					break;
				} else {								/* 否则初始化行搜索指针 */
					pText = g_NvMemAcsCtr.u8FileCont;
				}
			}

			pLineStart = pText;
			for( ; pText < &g_NvMemAcsCtr.u8FileCont[u32ByteNum]; pText++) {
				/* 找到回车换行符号, 说明一个有效的搜索 */
				if(*pText == 0x0A) {
					/* 获得该行ID */
					uint32 u32ItemID = GetConfIDInText(pLineStart);
					if(u32ItemID != 0) {
						/* 搜寻该行ID所对应的pDataAcsProp */
						pDataAcsProp = pBootLoaderConf->pConfProp;
						for(i = pBootLoaderConf->u32ItemNum; i > 0; i--) {
							if((pDataAcsProp->u32ItemID != u32ItemID) 
								|| (pDataAcsProp->ItemType.SaveGrp != ConfAcsGrp) 
								|| (pDataAcsProp->pData == NULL))
							{
								pDataAcsProp++;
							} else {	/* 找到对应项, 获取配置数据 */
								/* 跳过前面的 ConfID 以及一个空格 */
								pLineStart += 5;
								switch(pDataAcsProp->ItemType.DataType) {
									case ITEM_DAT_TEMPSENCONF:
										if(!ConvTextForTempSenConf(pText, pLineStart, (TEMP_SEN_CONF*)(pDataAcsProp->pData), bRead1_Check0)) {
											goto READ_ITEM_FAIL;
										}
										break;
								
									case ITEM_DAT_IPv4:
										if(!ConvTextForIPv4(pText, pLineStart, (uint32*)(pDataAcsProp->pData), bRead1_Check0)) {
											goto READ_ITEM_FAIL;
										}
										break;
								
									case ITEM_DAT_RMTS:
									case ITEM_DAT_DIGITSTR:
									case ITEM_DAT_ANSI:
									case ITEM_DAT_ANSI_RO:
										if(!ReadConfStringInText(pText, pLineStart, (uint8*)(pDataAcsProp->pData), bRead1_Check0)) {
											goto READ_ITEM_FAIL;
										}
										break;
								
									case ITEM_DAT_SUMM:
										if(!ConvTextForAcqSumm(pLineStart, pText, (ACQ_SUMM*)pDataAcsProp->pData, bRead1_Check0)) {
											goto READ_ITEM_FAIL;
										}
										break;

									case ITEM_DAT_T64_RO:
										if(!ReadT64InText(pLineStart, pText, (REAL_TIME_VAR*)pDataAcsProp->pData, bRead1_Check0)) {
											goto READ_ITEM_FAIL;
										}
										break;
									
									case ITEM_DAT_pTIME:
										if(!ReadPTimeInText(pLineStart, pText, (uint32*)pDataAcsProp->pData, bRead1_Check0)) {
											goto READ_ITEM_FAIL;
										}
										break;
										
									case ITEM_DAT_TOPIC:
									case ITEM_DAT_BOOL:
									case ITEM_DAT_ENUM:
									case ITEM_DAT_U32:
									case ITEM_DAT_T32_RO:
									case ITEM_DAT_U32_RO:
									case ITEM_DAT_F32:
									case ITEM_DAT_ACQ_U32:
									case ITEM_DAT_ACQ_F32:
									case ITEM_DAT_I64:
                                    case ITEM_DAT_I64_RO:
									case ITEM_DAT_D2U32:
									case ITEM_DAT_D2F32:
									case ITEM_DAT_U32_ID_RO:
										if(!ConvTextForNormalConf(pText, pLineStart, pDataAcsProp, bRead1_Check0)) {
											goto READ_ITEM_FAIL;
										}
										break;
								}
								ItemNum--;
								break;
							}
						}
					}
					
					pText++;
					pLineEnd = pText;
					break;		/* 结束当前行搜索 */
				}
			}

			/* 已经完成当前文件搜索 */
			if((u32ByteNum < FILE_CONT_BUF_LEN) && (pText >= &g_NvMemAcsCtr.u8FileCont[u32ByteNum])) {
				break;
			}
		}

		/* 数据完整性校验 */
		if(ItemNum == 0) {				/* 未读完 */
			return DATA_ACS_RES_SUC;
		} else {
			DataAcsRes = DATA_ACS_RES_NOT_CMPLT;
			continue;
		}

READ_ITEM_FAIL:
		DataAcsRes = DATA_ACS_RES_FAIL;
	}
	
	return DataAcsRes;
}

/*==========================================================================
| Function name	: BOOL ConvTextForNormalConf(uint8* pText, uint8* pTextStart, CONF_n_STAT_PROP* pDataAcsProp, BOOL bRead1_Check0)
| Description	: 从一具体行配置/统计文本中提取具体的配置/统计数据，写入相应的数据位置或者仅做检查，注意是从行尾开始
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
BOOL ConvTextForNormalConf(uint8* pText, uint8* pTextStart, CONF_n_STAT_PROP* pDataAcsProp, BOOL bRead1_Check0)
{
	/* 0. 输入检查 */
	if(pDataAcsProp->pData == NULL) {
		return FALSE;
	}
	float32 fData;

	/* 1. 把第一个数据读取出来 */
	uint16 uPointPlace = 0;
	int16 iExp = 0;
	int64 i64Data = 0;
	uint16 uDigitNo = 0;
	BOOL bFindData = FALSE;
	BOOL bFindNeg = FALSE;
	BOOL bFindExp = FALSE;
	BOOL bFindPoint = FALSE;
	for( ; pText >= pTextStart; pText--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			i64Data += ((int64)(*pText - '0'))*cnst_u64BcdQFactor[uDigitNo];
			uDigitNo++;
		} else if((*pText == '.') && (!bFindPoint)) {
			uPointPlace = uDigitNo;
			bFindPoint = bFindData;
		} else if((*pText == '-') && (!bFindNeg))  {
			bFindNeg = bFindData;
		} else if((*pText == 'e') && (!bFindExp)) {
			iExp = i64Data;
			if(bFindNeg) {
				iExp = 0 - iExp;
				bFindNeg = FALSE;
			}
			i64Data = 0;
			uDigitNo = 0;
			uPointPlace = 0;
			bFindExp = bFindData;
		} else if(bFindData) {
			pText--;
			break;
		}
	}
	iExp -= uPointPlace;

	/* 2. 根据具体的保存数据格式进行存储 */
	if(!bFindData) {
		return FALSE;
	} else {
		switch(pDataAcsProp->ItemType.DataType) {
			case ITEM_DAT_U32:
			case ITEM_DAT_ACQ_U32:
			case ITEM_DAT_U32_RO:
			case ITEM_DAT_D2U32:
			case ITEM_DAT_U32_ID_RO:
				iExp += pDataAcsProp->ItemType.uBcdQ;
				if(iExp > 0) {		/* 小数点位置和输入不一致处理 */
					i64Data *= cnst_u64BcdQFactor[iExp];
				} else if(iExp < 0) {
					iExp = 0 - iExp;
					i64Data = (i64Data + (cnst_u64BcdQFactor[iExp]>>1))/cnst_u64BcdQFactor[iExp];
				}
			case ITEM_DAT_TOPIC:	/* 忽略一切小数点、指数项 */
			case ITEM_DAT_BOOL:
			case ITEM_DAT_ENUM:
				if(bRead1_Check0) {
					if((pDataAcsProp->ItemType.DataType == ITEM_DAT_D2U32)
					    || (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_ID_RO))
					{
						*((uint32*)pDataAcsProp->pData + 1) = (uint32)i64Data;
					} else {
						*((uint32*)pDataAcsProp->pData) = (uint32)i64Data;
					}
				} else if((pDataAcsProp->ItemType.DataType == ITEM_DAT_D2U32)
					    || (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_ID_RO))
			    {
					if(*((uint32*)pDataAcsProp->pData + 1) != (uint32)i64Data) {
						return FALSE;
					}
				} else if(*((uint32*)(pDataAcsProp->pData)) != (uint32)i64Data) {
					return FALSE;
				}
				break;
				
            case ITEM_DAT_I64_RO:
			case ITEM_DAT_I64:
				iExp += pDataAcsProp->ItemType.uBcdQ;
				if(iExp > 0) {		/* 小数点位置和输入不一致处理 */
					i64Data *= cnst_u64BcdQFactor[iExp];
				} else if(iExp < 0) {
					iExp = 0 - iExp;
					i64Data = (i64Data + (cnst_u64BcdQFactor[iExp]>>1))/cnst_u64BcdQFactor[iExp];
				}
				if(bFindNeg) {
					i64Data = 0 - i64Data;
				}
				Swi_disable();
				if(bRead1_Check0) {
					*((int64*)pDataAcsProp->pData) = i64Data;
				} else if(*((int64*)pDataAcsProp->pData) != i64Data) {
					Swi_enable();
					return FALSE;
				}
				Swi_enable();
				break;
		
			case ITEM_DAT_F32:
			case ITEM_DAT_ACQ_F32:
			case ITEM_DAT_D2F32:
				if(iExp > 0) {
					fData = ((float32)i64Data)*cnst_fBcdQFactor[iExp];
				} else if(iExp < 0) {
					iExp = 0 - iExp;
					fData = ((float32)i64Data)/cnst_fBcdQFactor[iExp];
				} else {
					fData = (float32)i64Data;
				}
				if(bFindNeg) {
					fData = 0 - fData;
				}
				if(bRead1_Check0) {					
					if(pDataAcsProp->ItemType.DataType == ITEM_DAT_D2F32) {
						*((float32*)pDataAcsProp->pData + 1) = fData;
					} else {
						*((float32*)pDataAcsProp->pData) = fData;
					}
				} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_D2F32) {
					if(fabsf(*((float32*)pDataAcsProp->pData + 1) - fData) > fabsf(fData)*0.001f) {
						return FALSE;
					}
				} else if(fabsf(*((float32*)pDataAcsProp->pData) - fData) > fabsf(fData)*0.001f) {
					return FALSE;
				}
				break;
		}
	}
	
	/* 3. 如果有第2个数 */
	if((pDataAcsProp->ItemType.DataType == ITEM_DAT_D2U32)
	    || (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_ID_RO)
		|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_D2F32))
	{
		uPointPlace = 0;
		iExp = 0;
		i64Data = 0;
		uDigitNo = 0;
		bFindData = FALSE;
		bFindNeg = FALSE;
		bFindExp = FALSE;
		bFindPoint = FALSE;
		for( ; pText >= pTextStart; pText--) {
			if(('0' <= *pText) && (*pText <= '9')) {
				bFindData = TRUE;
				i64Data += ((uint64)(*pText - '0'))*cnst_u64BcdQFactor[uDigitNo];
				uDigitNo++;
			} else if((*pText == '.') && (!bFindPoint)) {
				uPointPlace = uDigitNo;
				bFindPoint = bFindData;
			} else if((*pText == '-') && (!bFindNeg)) {
				bFindNeg = bFindData;
			} else if((*pText == 'e') && (!bFindExp)) {
				iExp = i64Data;
				if(bFindNeg) {
					iExp = 0 - iExp;
					bFindNeg = FALSE;
				}
				i64Data = 0;
				uDigitNo = 0;
				uPointPlace = 0;
				bFindExp = bFindData;
			} else if(bFindData) {
				pText--;
				break;
			}
		}
		iExp -= uPointPlace;

	/* 4. 根据具体的保存数据格式进行存储 */
		if(!bFindData) {
			return FALSE;
		} else if((pDataAcsProp->ItemType.DataType == ITEM_DAT_D2U32)
    	    || (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_ID_RO))
		{
			iExp += pDataAcsProp->ItemType.uBcdQ;
			if(iExp > 0) {		/* 小数点位置和输入不一致处理 */
				i64Data *= cnst_u64BcdQFactor[iExp];
			} else if(iExp < 0) {
				iExp = 0 - iExp;
				i64Data = (i64Data + (cnst_u64BcdQFactor[iExp]>>1))/cnst_u64BcdQFactor[iExp];
			}
			if(bRead1_Check0) {
				*((uint32*)pDataAcsProp->pData) = i64Data;
			} else if(*((uint32*)pDataAcsProp->pData) != (uint32)i64Data) {
				return FALSE;
			}
		} else {
			if(iExp > 0) {
				fData = ((float32)i64Data)*cnst_fBcdQFactor[iExp];
			} else if(iExp < 0) {
				iExp = 0 - iExp;
				fData = ((float32)i64Data)/cnst_fBcdQFactor[iExp];
			} else {
				fData = (float32)i64Data;
			}
			if(bFindNeg) {
				fData = 0 - fData;
			}
			if(bRead1_Check0) {
				*((float32*)pDataAcsProp->pData) = fData;
			} else if(fabsf(*((float32*)pDataAcsProp->pData) - fData) > fabsf(fData)*0.001f) {
				return FALSE;
			}
		}
	}

	return TRUE;
}

/*==========================================================================
| Description	: 从一具体文本行中提取具体的CRC32数据，进行CRC32校验，注意是从行尾开始
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
BOOL CheckTextWithCrc32ForConf(uint8* pLineStart, uint8* pLineEnd, uint32 u32IniCrc)
{
	uint8* pText;
	uint32 u32Hex = 0;
	uint8 u8DigitNo = 0;
	BOOL bFindData = FALSE;
	
	for(pText = pLineEnd - 1; (pText >= pLineStart) && (u8DigitNo < 32); pText--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u32Hex += (((uint32)(*pText - '0'))<<u8DigitNo);
			u8DigitNo += 4;
		} else if(('A' <= *pText) && (*pText <= 'F')) {
			bFindData = TRUE;
			u32Hex += (((uint32)(*pText - 'A' + 10))<<u8DigitNo);
			u8DigitNo += 4;
		} else if(('a' <= *pText) && (*pText <= 'f')) {
			bFindData = TRUE;
			u32Hex += (((uint32)(*pText - 'a' + 10))<<u8DigitNo);
			u8DigitNo += 4;
		} else if(bFindData) {
			break;
		}
	}
	
	if(u32Hex == CalCRC32ForFileByIni(pLineStart, pText - pLineStart + 1, u32IniCrc)) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/*==========================================================================
| Description	: 从配置行中取得配置项ID(在最前面)
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2017-10-13
\=========================================================================*/
uint32 GetConfIDInText(uint8* pLineStart)
{
	uint32 u32Data = 0;
	BOOL bFindData = FALSE;
	int8 i;
	
	for(i = 10; i > 0; i--) {	/* 最多也就10位数 */
		if(('0' <= *pLineStart) && (*pLineStart <= '9')) {
			bFindData = TRUE;
			u32Data = u32Data*10 + ((uint32)(*pLineStart - '0'));
		} else if(bFindData) {
			break;
		}
		pLineStart++;
	}
	return u32Data;
}

/*==========================================================================
| Function name	: BOOL ConvTextForTempSenConf(uint8* pText, uint8* pTextStart, TEMP_SEN_CONF* pData, BOOL bRead1_Check0)
| Description	: 把温度配置的文本信息转化为二进制的形式
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
BOOL ConvTextForTempSenConf(uint8* pText, uint8* pTextStart, TEMP_SEN_CONF* pData, BOOL bRead1_Check0)
{
	uint64 u64Hex;
	uint32 u32Data;
	uint16 uDigitNo;
	BOOL bFindData;

	/* 1. 取出u32TempSenPosition */
	u32Data = 0;
	uDigitNo = 0;
	bFindData = FALSE;
	for( ; pText >= pTextStart; pText--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u32Data += ((uint32)(*pText - '0'))*cnst_u32BcdQFactor[uDigitNo];
			uDigitNo++;
		} else if(bFindData) {
			pText--;
			break;
		}
	}
	if(!bFindData) {
		return FALSE;
	} else if(bRead1_Check0) {
		pData->u32TempSenPosition = u32Data;
	} else if(pData->u32TempSenPosition != u32Data) {
		return FALSE;
	}

	/* 2. 取出u32TempSenChangeCount */
	u32Data = 0;
	uDigitNo = 0;
	bFindData = FALSE;
	for( ; pText >= pTextStart; pText--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u32Data += ((uint32)(*pText - '0'))*cnst_u32BcdQFactor[uDigitNo];
			uDigitNo++;
		} else if(bFindData) {
			pText--;
			break;
		}
	}
	if(!bFindData) {
		return FALSE;
	} else if(bRead1_Check0) {
		pData->u32Count_TempSen_Change = u32Data;
	} else if(pData->u32Count_TempSen_Change != u32Data) {
		return FALSE;
	}

	/* 3. 取出u64TempSenRom */
	u64Hex = 0;
	uDigitNo = 0;
	bFindData = FALSE;
	for( ; pText >= pTextStart; pText--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u64Hex += ((uint64)(*pText - '0'))<<(uDigitNo*4);
			uDigitNo++;
		} else if(('A' <= *pText) && (*pText <= 'F')) {
			bFindData = TRUE;
			u64Hex += ((uint64)(*pText - 'A' + 10))<<(uDigitNo*4);
			uDigitNo++;
		} else if(('a' <= *pText) && (*pText <= 'f')) {
			bFindData = TRUE;
			u64Hex += ((uint64)(*pText - 'a' + 10))<<(uDigitNo*4);
			uDigitNo++;
		} else if(bFindData) {
			break;
		}
	}
	if(!bFindData) {
		return FALSE;
	} else if(bRead1_Check0) {
		pData->u64TempSenRom = u64Hex;
	} else if(pData->u64TempSenRom != u64Hex) {
		return FALSE;
	}

	return TRUE;
}

/*==========================================================================
| Description	: 把文本转化为IPv4地址
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
BOOL ConvTextForIPv4(uint8* pText, uint8* pTextStart, uint32* pU32Data, BOOL bRead1_Check0)
{
	uint32 u32Data, u32Seg;
	uint16 uDigitNo;
	int16 i;
	BOOL bFindData;

	u32Data = 0;
	for(i = 0; i < 4; i++) {
		u32Seg = 0;
		uDigitNo = 0;
		bFindData = FALSE;
		for( ; (pText >= pTextStart) && (uDigitNo < 3); pText--) {
			if(('0' <= *pText) && (*pText <= '9')) {
				bFindData = TRUE;
				u32Seg += ((uint32)(*pText - '0'))*cnst_u32BcdQFactor[uDigitNo];
				uDigitNo++;
			} else if(bFindData) {
				pText--;
				break;
			}
		}
		u32Data += cnst_u32IPv4Factor[i]*u32Seg;
	}
	
	if(!bFindData) {
		return FALSE;
	} else if(bRead1_Check0) {
		*pU32Data = u32Data;
	} else if(*pU32Data != u32Data) {
		return FALSE;
	}
	return TRUE;
}

/*==========================================================================
| Description	: 在文本中搜索字符串，0~9,a~z,A~Z,.
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
BOOL ReadConfStringInText(uint8* pText, uint8* pTextStart, uint8* pU8Data, BOOL bRead1_Check0)
{
	BOOL bFindData = FALSE;
	int16 iByteCnt = 0;
	for( ; pText >= pTextStart; pText--) {			/* 搜索字符串长度 */
		if((*pText != 0x00) && (*pText != 0x0A) && (*pText != 0x0D) && (*pText != ' '))	{
			bFindData = TRUE;
			iByteCnt++;
		} else if(bFindData) {
			pText++;
			break;
		}
	}
	if(iByteCnt > CONF_ANSI_BYTE_LEN - 1) {
		iByteCnt = CONF_ANSI_BYTE_LEN - 1;
	}
	
	if(!bFindData) {
		return FALSE;
	} else if(bRead1_Check0) {
		pU8Data[iByteCnt--] = 0;
		for( ; iByteCnt >= 0; iByteCnt--) {
			pU8Data[iByteCnt] = pText[iByteCnt];
		}
	} else {
		iByteCnt--;
		for( ; iByteCnt >= 0; iByteCnt--) {
			if(pU8Data[iByteCnt] != pText[iByteCnt]) {
				return FALSE;
			}
		}
	}
	return TRUE;
}

/*==========================================================================
| Description	: 把具有自动采集功能的文本转化为配置, 左起
	显示在文本上: 导前时间(s);  0.0987 20 0.1
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-10-31
\=========================================================================*/
BOOL ConvTextForAcqSumm(uint8* pText, uint8* pTextEnd, ACQ_SUMM* pData, BOOL bRead1_Check0)
{
	/* 1. 取出fAcqSummary */
	float32 fData;
	if(!GetF32(&pText, pTextEnd, &fData)) {
		return FALSE;
	} else if(bRead1_Check0) {
		pData->fAcqSummary = fData;
	} else if(fabsf(pData->fAcqSummary - fData) > fabsf(fData)*0.001f) {
		return FALSE;
	}
	
	/* 2. 取出u32SummaryRealNum */
	uint32 u32Data = 0;
	BOOL bFindData = FALSE;
	for( ; pText < pTextEnd; pText++) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u32Data = u32Data*10UL + (uint32)(*pText - '0');
		} else if(bFindData) {
			pText++;
			break;
		}
	}
	if(!bFindData) {
		return FALSE;
	} else if(bRead1_Check0) {
		pData->u32SummaryRealNum = u32Data;
	} else if(pData->u32SummaryRealNum != u32Data) {
		return FALSE;
	}
	
	/* 3. 取出i32SummaryPlanNum */
	int32 i32Data;
	if(!GetI32(&pText, pTextEnd, &i32Data, 0)) {
		return FALSE;
	} else if(bRead1_Check0) {
		pData->i32SummaryPlanNum = i32Data;
	} else if(pData->i32SummaryPlanNum != i32Data) {
		return FALSE;
	}

	/* 4. 取出fSetVal */
	if(!GetF32(&pText, pTextEnd, &fData)) {
		return FALSE;
	} else if(bRead1_Check0) {
		pData->fSetVal = fData;
	} else if(fabsf(pData->fSetVal - fData) > fabsf(fData)*0.001f) {
		return FALSE;
	}
	
	return TRUE;
}

BOOL ReadT64InText(uint8* pText, uint8* pTextEnd, REAL_TIME_VAR* pRealTime, BOOL bRead1_Check0)
{
	if(bRead1_Check0) {
		pRealTime->u8Year = ReadU32(&pText, pTextEnd);
		pRealTime->u8Month = ReadU32(&pText, pTextEnd);
		pRealTime->u8Day = ReadU32(&pText, pTextEnd);
		pRealTime->u8Hour = ReadU32(&pText, pTextEnd);
		pRealTime->u8Minute = ReadU32(&pText, pTextEnd);
		pRealTime->u8Second = ReadU32(&pText, pTextEnd);
		pRealTime->uMilliSec = ReadU32(&pText, pTextEnd);
	} else if((pRealTime->u8Year != ReadU32(&pText, pTextEnd))
			|| (pRealTime->u8Month != ReadU32(&pText, pTextEnd))
			|| (pRealTime->u8Day != ReadU32(&pText, pTextEnd))
			|| (pRealTime->u8Hour != ReadU32(&pText, pTextEnd))
			|| (pRealTime->u8Minute != ReadU32(&pText, pTextEnd))
			|| (pRealTime->u8Second != ReadU32(&pText, pTextEnd))
			|| (pRealTime->uMilliSec != ReadU32(&pText, pTextEnd)))
	{
		return FALSE;
	}
	return TRUE;
}

/* 峰谷电的时间，老版本存储格式为hh.mm，后来为s#hh:mm */
BOOL ReadPTimeInText(uint8* pText, uint8* pTextEnd, uint32* pU32, BOOL bRead1_Check0)
{
	BOOL bFindData = FALSE;
	uint32 u32Data[2] = {0, 0};
	int8 i = 0;		/* 数字 */
	for( ; (pText < pTextEnd) && (i < 2); pText++) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u32Data[i] = u32Data[i]*10 + (*pText - '0');
		} else if((*pText == '#') || (*pText == ':') || (*pText == '.')) {	/* 时间可能呈现为 1#12:00 */
		} else if(bFindData) {
			i++;	/* 指向下一个数据 */
			bFindData = FALSE;
		}
	}
	if(bFindData && (pText >= pTextEnd)) {/* Buf最后一个依然是有效数字，就不会触发i++ */
		i++;
	}

	if(i < 2) {	/* 没有读完 */
		return FALSE;
	} else if(bRead1_Check0) {
		pU32[0] = u32Data[0];
		pU32[1] = u32Data[1];
		return TRUE;
	} else {
		return (pU32[0] == u32Data[0]) && (pU32[1] == u32Data[1]);
	}
}
#endif

/*==========================================================================
	以下为 Msg & Acq 部分
  Msg, Acq  数据完整性的保证
  1. 往ram里面添加数据只能在一个任务中进行，msg数据完整性标识是MsgNo, Acq数据完整性标识是
	 添加完数据后，才修改ItemNo_forAdd，这样保证ItemNo_forAdd指针的有效性
  2. 可添加的标识还需要增加MsgId，AcqId
  3. 存储ram数据到NvMem任务优先级最低，存储完后，先修改NvMem指针，然后再销毁ram数据完整性标识
	 值得注意的是：由于数据需要存储file、eeprom中，因此销毁数据完整性标识以最差的那份NvMem数据为准(一般是file)
	 由于tf卡可能失效，造成file中永远写不进去数据，因此在RamBuf比较满的时候，需要放弃考虑file
  =========================================================================*/
typedef enum {
	CONV_MSG_ACQ_CODE_SUC		= 0,
	CONV_MSG_ACQ_CODE_ERR_FULL	= 1,
	CONV_MSG_ACQ_CODE_ERR_DATA	= 2
}CONV_MSG_ACQ_CODE_RES;

#if SUPPORT_FAT_FS
BOOL SaveMsgOrAcq_V1(DATA_PAGE DataPage, uint8* pFileContBuf, uint8 u8Year, uint8 u8Month, uint8 u8Day, uint32 u32ItemNo)
{
    uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
    PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
    if(DataPage == DATA_PAGE_MSG) {
        PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Msg");
    } else if(DataPage == DATA_PAGE_ACQ) {
        PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Acq");
    } else {
        return FALSE;
    }
    
    uint8 u8Time[3];
    u8Time[2] = u8Year;
    u8Time[1] = u8Month;
    u8Time[0] = u8Day;
    int8 i8FilePathDepth;
    for(i8FilePathDepth = 2; i8FilePathDepth >= 0; i8FilePathDepth--) {
        *pFilePath = 0;
        DIR dir;
        if(f_opendir(&dir, (const TCHAR*)g_NvMemAcsCtr.u8FilePath) != FR_OK) {
            return FALSE;
        } else {
            *pFilePath++ = '/';
            pFilePath[1] = u8Time[i8FilePathDepth]%10 + '0';
            u8Time[i8FilePathDepth] = u8Time[i8FilePathDepth]/10;
            pFilePath[0] = u8Time[i8FilePathDepth]%10 + '0';
            
            /* 搜索当前目录中，指定日期的文件或文件夹，如果有多个，选择ItemNo最大的 */
            uint32 u32MaxDirOrFileItemNo = 0;
            uint32 u32CurDirOrFileItemNo;
            FILINFO fno;
            int8 i;
            while((f_readdir(&dir, &fno) == FR_OK) && fno.fname[0]) {
                if((fno.fname[0] == pFilePath[0]) && (fno.fname[1] == pFilePath[1]) 
                    && ((i8FilePathDepth != 0) ^ ((fno.fattrib & AM_DIR) == 0)))
                {
                    u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname + 2);
                    if(u32CurDirOrFileItemNo > u32MaxDirOrFileItemNo) {
                        u32MaxDirOrFileItemNo = u32CurDirOrFileItemNo;
                        for(i = 2; (i < FILE_NAME_BYTE_LEN) && fno.fname[i]; i++) {
                            pFilePath[i] = fno.fname[i];
                        }
                    }
                }
            }
    
            if(u32MaxDirOrFileItemNo) {     /* 找到文件或文件夹 */
                pFilePath += i;
            } else {    /* 没有找到，就需要创建文件夹或者文件 */
                /* 生成文件夹、文件名 */
                pFilePath += 2;
                uint32 u32ItemNoTmp = u32ItemNo;
                for(i = 5; i >= 0; i--) {
                    pFilePath[i] = u32ItemNoTmp%10 + '0';
                    u32ItemNoTmp = u32ItemNoTmp/10;
                }
                pFilePath += 6;
                if(i8FilePathDepth) {   /* 创建文件夹 */
                    pFilePath[0] = 0;
                    if(f_mkdir((const TCHAR*)g_NvMemAcsCtr.u8FilePath) == FR_EXIST) {
                        f_unlink((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
                        f_mkdir((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
                    }
                } else {
                    pFilePath[0] = '.';
                    pFilePath[1] = 't';
                    pFilePath[2] = 'x';
                    pFilePath[3] = 't';
                    pFilePath[4] = 0;
                    pFilePath += 4;
                }
            }
        }
    }

    /* 存进文件 */
    FIL File;
    if(f_open(&File, (const TCHAR*)g_NvMemAcsCtr.u8FilePath, FA_OPEN_ALWAYS | FA_WRITE) != FR_OK) {
        return FALSE;
    } else {
        UINT u32ByteNum = pFileContBuf - g_NvMemAcsCtr.u8FileCont;
        f_lseek(&File, f_size(&File));
        f_write(&File, &g_NvMemAcsCtr.u8FileCont, u32ByteNum, &u32ByteNum);
        f_close(&File);
        return TRUE;
    }
}

/* 打开msg2,acq2文件 */
BOOL SaveMsgOrAcq_V2(DATA_PAGE DataPage, uint8* pFileContBuf, uint32 u32ItemNo)
{
	uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
    PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
    if(DataPage == DATA_PAGE_MSG) {
        PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Msg2");
    } else if(DataPage == DATA_PAGE_ACQ) {
        PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Acq2");
    } else {
        return FALSE;
    }
    
    /* 搜索文件夹中，最后一个文件 */
    *pFilePath = 0;
    DIR dir;
    if(f_opendir(&dir, (const TCHAR*)g_NvMemAcsCtr.u8FilePath) != FR_OK) {
        return FALSE;
    } else {
        *pFilePath++ = '/';
        /* 搜索目录中是否存在比期望的ItemNo更大的文件 */
        uint32 u32MaxDirOrFileItemNo = (u32ItemNo/100000)*100000;        
        uint32 u32CurDirOrFileItemNo;
        FILINFO fno;
        int8 i;
        uint8 u8FileNameLen = 0;    /* 文件名长度,0代表没有找到 */
        while((f_readdir(&dir, &fno) == FR_OK) && fno.fname[0] && ((fno.fattrib & AM_DIR) == 0)) {
            u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname);            
            /* 之前软件bug，会产生000000.txt，实际上里面存的第一条序号是1 */
            if(u32CurDirOrFileItemNo == 0) {
                u32CurDirOrFileItemNo = 1;
            }
            if(u32CurDirOrFileItemNo > u32MaxDirOrFileItemNo) {
                u32MaxDirOrFileItemNo = u32CurDirOrFileItemNo;
                for(i = 0; (i < FILE_NAME_BYTE_LEN) && fno.fname[i]; i++) {
                    pFilePath[i] = fno.fname[i];
                }
                u8FileNameLen = i;
            }
        }
        
        /* 形成完整目录   */
        if(u8FileNameLen) {
            pFilePath += u8FileNameLen;
        } else {    /* 没有找到对应的文件，  */
            PrintU32WithLen(&pFilePath, u32ItemNo, 6);
            PrintStringNoOvChk(&pFilePath, ".txt");
        }
        *pFilePath = 0;

        /* 写进文件 */
        FIL File;
        if(f_open(&File, (const TCHAR*)g_NvMemAcsCtr.u8FilePath, FA_OPEN_ALWAYS | FA_WRITE) != FR_OK) {
            return FALSE;
        } else {
			UINT u32ByteNum = pFileContBuf - g_NvMemAcsCtr.u8FileCont;
            f_lseek(&File, f_size(&File));
            f_write(&File, &g_NvMemAcsCtr.u8FileCont, u32ByteNum, &u32ByteNum);
            f_close(&File);
            return TRUE;
        }
    }
}

BOOL CheckTextWithCrc32ForMsgAndAcq(uint8* pLineStart, uint8* pLineEnd);
uint32 ReadLastItemNoFromFile(DATA_PAGE DataPage, MSG_ACQ_ACS_VER eReadItemVer)
{
	uint32 u32LastItemNoFromFile = 0;
	
	/* 搜索文件 */
	uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
	if(DataPage == DATA_PAGE_MSG) {
		PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Msg");
	} else if(DataPage == DATA_PAGE_ACQ) {
		PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Acq");
	} else {
		return 0;
	}
	if(eReadItemVer == MSG_ACQ_ACS_V2) {               /* 读取Msg2或者Acq2 */
	    *pFilePath++ = '2';
	}

	/* 搜索年、月、日文件夹中ItemNo最大的 */
	int16 i, iFilePathDepth;
	if(eReadItemVer == MSG_ACQ_ACS_V2) {
	    iFilePathDepth = 0;
	} else {
	    iFilePathDepth = 2;
	}
	for(; iFilePathDepth >= 0; iFilePathDepth--) {
		*pFilePath = 0;
		DIR dir;
		if(f_opendir(&dir, (const TCHAR*)g_NvMemAcsCtr.u8FilePath) == FR_OK) {
			*pFilePath++ = '/';
			uint32 u32CurSeekMaxItemNo = 0; 		/* 当前在读的行之行号，每成功获取一条则增一 */
			i = 0;
			FILINFO fno;
			while((f_readdir(&dir, &fno) == FR_OK) && fno.fname[0]) {
				if((iFilePathDepth != 0) ^ ((fno.fattrib & AM_DIR) == 0)) {
			    	uint32 u32CurDirOrFileItemNo;
				    if(eReadItemVer == MSG_ACQ_ACS_V2) {
    					u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname); /* 以十六进制表达的当前在读的行之行号，方便文件夹/文件搜索 */
				    } else {
    					u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname + 2); /* 以十六进制表达的当前在读的行之行号，方便文件夹/文件搜索 */
    				}
    				/* 之前软件bug，会产生000000.txt，实际上里面存的第一条序号是1 */
    				if(u32CurDirOrFileItemNo == 0) {
    				    u32CurDirOrFileItemNo = 1;
    				}
					if(u32CurSeekMaxItemNo < u32CurDirOrFileItemNo) {
						for(i = 0; (i < FILE_NAME_BYTE_LEN) && fno.fname[i]; i++) {
							pFilePath[i] = fno.fname[i];
						}
						u32CurSeekMaxItemNo = u32CurDirOrFileItemNo;
						u32LastItemNoFromFile = u32CurDirOrFileItemNo;	/* 以文件夹、文件为准 */
					}
				}
			}
			pFilePath += i;
		} else {
			break;
		}
	}

	/* 打开文件 */
	*pFilePath = 0;
	FIL File; 
	if((iFilePathDepth < 0) 	/* 前面打开文件成功 */
		&& (f_open(&File, (const TCHAR*)g_NvMemAcsCtr.u8FilePath, FA_READ) == FR_OK))
	{
		uint32 u32DataFilePt = f_size(&File);		/* 初始化文件访问指针 */
		int32 i32LineStartPt = 0;
		uint32 u32SkipItem = 0; 				/* 如果CRC校验失败，会跳过一行，因此相应读出来的ItemNo需要加上这一行 */
		do {
			/* 获取数据 */
			UINT u32BufByteNum;
			if(u32DataFilePt + i32LineStartPt > FILE_CONT_BUF_LEN) {
				u32DataFilePt -= FILE_CONT_BUF_LEN - i32LineStartPt;
				u32BufByteNum = FILE_CONT_BUF_LEN;
			} else {
				u32BufByteNum = u32DataFilePt + i32LineStartPt;
				u32DataFilePt = 0;
			}
			f_lseek(&File, u32DataFilePt);
			f_read(&File, g_NvMemAcsCtr.u8FileCont, u32BufByteNum, &u32BufByteNum);
			int32 i32LineSeekPt = u32BufByteNum - 2;	/* 跳过行尾的回车换行 */
			if(i32LineSeekPt < 0) {
				break;
			}

			/* 行搜索: 找到回车换行符号, 说明一个有效的搜索 */
			int32 i32LineEndPt = u32BufByteNum;
			for( ; i32LineSeekPt >= 0; i32LineSeekPt--) {
				/* 找到回车换行符号, 说明一个有效的搜索 */
				if((g_NvMemAcsCtr.u8FileCont[i32LineSeekPt] == 0x0A) || ((i32LineSeekPt == 0) && (u32DataFilePt == 0))) {
					if(i32LineEndPt - i32LineSeekPt > 10) {
						i32LineStartPt = i32LineSeekPt;
						if(g_NvMemAcsCtr.u8FileCont[i32LineSeekPt] == 0x0A) {
							i32LineStartPt++;
						}
						if(CheckTextWithCrc32ForMsgAndAcq(&g_NvMemAcsCtr.u8FileCont[i32LineStartPt], &g_NvMemAcsCtr.u8FileCont[i32LineEndPt])) {		/* 一个有效的行 */
							u32LastItemNoFromFile = ReadU32FixLen(&g_NvMemAcsCtr.u8FileCont[i32LineStartPt]) + u32SkipItem;
							u32DataFilePt = 0;	/* 修改文件指针以退出最外面的循环 */
							break;
						} else {						
							i32LineEndPt = i32LineStartPt;
							u32SkipItem++;
						}
					}
				}
			}
		} while(u32DataFilePt);
	}
		
	return u32LastItemNoFromFile;
}
#endif

BOOL CheckTextWithCrc32ForMsgAndAcq(uint8* pLineStart, uint8* pLineEnd)
{
	uint8* pText;
	uint32 u32Hex = 0;
	uint8 u8DigitNo = 0;
	BOOL bFindData = FALSE;
	
	for(pText = pLineEnd - 1; (pText >= pLineStart) && (u8DigitNo < 32); pText--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u32Hex += (((uint32)(*pText - '0'))<<u8DigitNo);
			u8DigitNo += 4;
		} else if(('A' <= *pText) && (*pText <= 'F')) {
			bFindData = TRUE;
			u32Hex += (((uint32)(*pText - 'A' + 10))<<u8DigitNo);
			u8DigitNo += 4;
		} else if(('a' <= *pText) && (*pText <= 'f')) {
			bFindData = TRUE;
			u32Hex += (((uint32)(*pText - 'a' + 10))<<u8DigitNo);
			u8DigitNo += 4;
		} else if(bFindData) {
			break;
		}
	}
	
	if(u32Hex == CalCRC32ForFileByIni(pLineStart, pText - pLineStart + 1, 0xFFFFFFFF)) {	/* CRC默认初始化为 0xFFFFFFFF */
		return TRUE;
	} else {
		return FALSE;
	}
}


/*==========================================================================
| Function name	: 
| Description	: 事件记录机制，事件分三大类:
	A. 简单事件	: 仅需要在事件发生的时候，记录下当前值与时间，消息被添加的时候就完成，例如发电度;
	B. 阻塞事件	: 基本和A类似，只是添加消息后，除非再次允许否则不得再次添加，例如动作次数超限，免得大量出现类似事件
	C. 保护事件	: 事件需要记录发生时刻、极值，还需要确认与完成，如过压保护;
	总的功能包括:
		初始化		: 初始化g_MsgStrCtr(消息控制器)
		添加		: 添加新的记录，并进行确认/取消(C类) 或者 解锁(B类)
		解码与存取	: 把g_MsgStrCtr中的已经完成记录的消息编码翻译成文本，插入文件系统，并清除g_MsgStrCtr中相应的消息
					: 对于A类事件，添加消息可以立即存储
					: 对于B类事件，存储的时候不得复位阻塞(复位一般是关机到位后)，但标识该消息已经存储
					: 对于C类事件，完成记录后，就可以存储
					: 为了防止发生时间早的反而存储在后面，存储的时候要考虑Ram中未确认的C类事件，即仅存储时间点在"最早未确认C类事件之前"的事件
		读取与通讯	: 依据编号(ItemNo)、时间进行从存储介质(Ram/文件)查找，并提交给通讯系统(包括显示访问)
		ItemNo分配	: Ram里面的Msg在分配ItemNo的时候，以消息确认的时间先后分配ItemNo，即每确认一条消息的时候才分配ItemNo
					: 但是当把Ram里面的编码消息译成文本存入文件的时候，需要按照事件消息时间先后顺序重新分配ItemNo
					: 为了防止写入过程ItemNo重新分配引起的通讯/显示混乱，只有当通讯/显示未访问Msg的时候才可以进行存储操作

	添加消息的函数包括:
		A类操作函数包括	:
			添加消息(非阻塞): 添加消息，不设置阻塞，即同样的消息还可以再次被添加
		B类操作函数包括	:
			添加消息(阻塞)	: 添加消息，并设置阻塞，即同样的消息不可以再次被添加
			取消阻塞		: 取消阻塞的设置
		C类操作函数包括	:
			建立事件、更新值: 如果没有可以更新值的记录，就建立记录(主要是记录发生的时间)；否则仅更新值
			确认事件		: 对于保护类事件，往往需要一定的延时才能确认事件(以防干扰)
			完成记录		: 值恢复正常以后，才认为一个事件记录完成--如果事件不曾确认，则放弃该事件；否则标识该事件已经完整。
	注意:添加消息类函数不可同时调用
			
	MsgBuf标志: 侦测到事件的时候，赋予MsgId; 确认事件的时候，赋予ItemNo; 事件完成的时候，赋值CmpltFlag; 存储的时候，赋值uEEPromNo

| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-10-3
\=========================================================================*/
#if MAX_MSG_TYPE
/* 这两条MsgID是默认的 */
#define MSG_NULL 				0x0000			/* 空 */
#define MSG_HuanYinShiYong		0x0001			/* 欢迎使用--这条消息仅用于开机显示，不应该用AddMsg添加(存储) */

typedef struct {								/* 160bit, 20byte */
	REAL_TIME_VAR MsgTimeStamp; 				/* 事件时间戳 */
	uint32 u32ItemNo;							/* 在Confirm时候，就赋值32ItemNo，并不代表事件是完整的 */
	uint16 uMsgId;								/* 事件ID */
	uint16 uEEPromNo;							/* 存储在EEProm中的序号 */
	float32 fMsgVal;							/* 事件值 */
}MSG_ITEM;

/*	事件消息记录: 添加消息在非MSG_NUL事件对MSG_NUL覆盖	*/
#define MSG_ITEM_BUF_LEN		16
#define MSG_SHIELD_FLAG_LEN		((MAX_MSG_TYPE+31)/32)
#define MSG_SAVED_BUF_LEN       8
typedef struct {
	uint32 u32CmpltFlag;						/* Msg完成标识 */
	uint32 u32ShieldFlag[MSG_SHIELD_FLAG_LEN];	/* 事件屏蔽标志，防止反复添加事件 */
	MSG_ITEM MsgBuf[MSG_ITEM_BUF_LEN]; 			/* 消息缓冲区--添加消息是添加在这其中 */

	/* 一些供快速访问消息的Buf */
	MSG_ITEM* pLastAddMsg;
	MSG_ITEM MsgSaved[MSG_SAVED_BUF_LEN];       /* 已经持久化的消息 */
	int8 i8Pt_MsgSaved;                         /* 指向存储的数据 */
	uint8 u8Rsvd;

	/* 存取相关的指针数据 */
	uint16 uEEPromNo_ForSave;					/* 用于存储下一条消息的存储在EEProm中的序号 */
	uint32 u32MsgNo_ram;                        /* ram存储的MsgNo起始(不含) */
	/* 这4个变量在 ReadItemNoAndEEPromNo 中初始化 */
	uint32 u32MsgNo_FileSaved;					/* 文件中已经存储的MsgNo */
	uint32 u32MsgNo_EESaved;					/* EEProm已经存储的MsgNo */
	uint32 u32MsgNo_ForAdd;						/* 用于添加下一条消息的MsgNo */
	uint32 u32MsgNo_Cmplt;                      /* 已经完成了的消息MsgNo */
}MSG_CTR;
SECTION(".NOT_ZeroInit") MSG_CTR g_MsgCtr;
extern const MSG_PROP cnst_MsgProp[];			/* Msg解码所需的，把MsgID翻译成文本 */

void InitMsgMdl(void)
{
	if(g_Sys.uRstCount) {	/* 仅上电复位才初始化 */
		return;
	}
	
	int16 i;
	for(i = 0; i < MSG_SHIELD_FLAG_LEN; i++) {
		g_MsgCtr.u32ShieldFlag[i] = 0;
	}
	g_MsgCtr.u32CmpltFlag = 0;
	for(i = MSG_ITEM_BUF_LEN-1; i >= 0; i--) {
		g_MsgCtr.MsgBuf[i].u32ItemNo = 0;
		g_MsgCtr.MsgBuf[i].uMsgId = MSG_NULL;
		g_MsgCtr.MsgBuf[i].fMsgVal = 0;
	}
	for(i = MSG_SAVED_BUF_LEN-1; i >= 0; i--) {
		g_MsgCtr.MsgSaved[i].u32ItemNo = 0;
		g_MsgCtr.MsgSaved[i].uMsgId = MSG_NULL;
		g_MsgCtr.MsgSaved[i].fMsgVal = 0;
	}
	/* 初始化Mqtt访问接口 */
	g_MqttItemProc.u32MsgNo_HavePubedForDB = 0xFFFFFFFFUL;

	/* 读取最后一行的行号:首先是文件中，如果文件中读取失败则从eeprom中 */
#if SUPPORT_FAT_FS
	uint32 u32LastItemNoFromFile = 0;
	/* ReadLastItemNoFromFile()返回0说明读失败--遇到过一次可能是读取失败，因此改成连读5次 */
	for(i = 5; (i > 0) && ((u32LastItemNoFromFile = ReadLastItemNoFromFile(DATA_PAGE_MSG, MSG_ACQ_ACS_V2)) == 0); i--) {
	}
	#if 0 // 删除对msgV1 ItemNo的支持,但是这段代码保留，以备哪天需要远程读取使用 SUPPORT_MsgV1RW_AcqV1R
		uint32 u32LastItemNoFromFileV1 = ReadLastItemNoFromFile(DATA_PAGE_MSG, MSG_ACQ_ACS_V1);
		if(u32LastItemNoFromFile < u32LastItemNoFromFileV1) {
			u32LastItemNoFromFile = u32LastItemNoFromFileV1;
		}
	#endif
#else
	uint32 u32LastItemNoFromFile = 0;
#endif

	/* 在eeprom中搜索ItemNo，EEPromNo */
#if CPU_0ST_1TI
	uint32 u32EEPromAddr = EEPROM_ADDR_MSG;
	uint16 uRemainItemInEEProm = EEPROM_MSG_ITEM_NUM;	/* 计算总的读取长度 */
	uint32 u32LastItemNoFromEEProm = 0; 				/* 来自EEProm中所存储的最后一个ItemNo */
	uint16 uExpectEEPromNo = 0; 						/* 期望的EEPromNo */
	/* 搜索EEPromNo发生跳变的, 取其所保存的ItemNo, EEPromNo;
	   如果一直没有跳变，则说明数据存储位置正好回到开始，取末位EEPromNo增一作为EEPromNo */
	do {
		/* 从EEProm中读出数据 */
		uint16 uCurReadItemNum = FILE_CONT_BUF_LEN/sizeof(MSG_ITEM);	/* 当前次读取的条数，保证读出整数条 */
		if(uCurReadItemNum > uRemainItemInEEProm) { 				/* 确保不会超出范围 */
			uCurReadItemNum = uRemainItemInEEProm;
		}
		EEPROMRead((uint32_t*)g_NvMemAcsCtr.u8FileCont, u32EEPromAddr, uCurReadItemNum*sizeof(MSG_ITEM));

		/* 搜索数据 */
		MSG_ITEM* pSeekMsgBuf = (MSG_ITEM*)g_NvMemAcsCtr.u8FileCont;
		if(uExpectEEPromNo == 0) {	/* 初始化uExpectEEPromNo */
			/* 新擦除的芯片 */
			if(((pSeekMsgBuf->u32ItemNo == 0xFFFFFFFFUL) && (pSeekMsgBuf->uMsgId == 0xFFFFUL) && (pSeekMsgBuf->uEEPromNo == 0xFFFFUL))
				|| (pSeekMsgBuf->uEEPromNo%EEPROM_MSG_ITEM_NUM))	/* 第一个数据EEPromNo应该是EEPROM_MSG_ITEM_NUM整数倍，否则可能数据破坏 */
			{
				u32LastItemNoFromEEProm = 0;
				break;
			} else {
				uExpectEEPromNo = pSeekMsgBuf->uEEPromNo;
			}
		}
		/* EEPromNo和预想的不一样(即自增一)则是插入点 */
		for(i = uCurReadItemNum; (i > 0) && (pSeekMsgBuf->uEEPromNo == uExpectEEPromNo); i--) {
			u32LastItemNoFromEEProm = pSeekMsgBuf->u32ItemNo;
			uExpectEEPromNo++;
			pSeekMsgBuf++;
		}

		/* 有结果，跳出循环 */
		if(i) {
			break;
		} else {	/* 无结果，准备下一次循环 */
			uRemainItemInEEProm -= uCurReadItemNum;
			u32EEPromAddr += uCurReadItemNum*sizeof(MSG_ITEM);
		}
	} while(uRemainItemInEEProm);
	g_MsgCtr.uEEPromNo_ForSave = uExpectEEPromNo;
#endif
	
	/* 存入 u32MsgNo_Saved */
	if(u32LastItemNoFromFile) {
		g_MsgCtr.u32MsgNo_FileSaved = u32LastItemNoFromFile;
		g_MsgCtr.u32MsgNo_EESaved = u32LastItemNoFromFile;
	} else {
	#if CPU_0ST_1TI
		g_MsgCtr.u32MsgNo_FileSaved = u32LastItemNoFromEEProm;
		g_MsgCtr.u32MsgNo_EESaved = u32LastItemNoFromEEProm;
	#else
	    g_MsgCtr.u32MsgNo_FileSaved = 0;
	    g_MsgCtr.u32MsgNo_EESaved = 0;
	#endif
	}
	g_MsgCtr.u32MsgNo_ForAdd = g_MsgCtr.u32MsgNo_FileSaved + 1;
	g_MsgCtr.u32MsgNo_Cmplt = g_MsgCtr.u32MsgNo_FileSaved;
	g_MsgCtr.u32MsgNo_ram = g_MsgCtr.u32MsgNo_FileSaved;

	/* 初始化g_MsgCtr.MsgSaved[0] */
    GetRealTime(&g_MsgCtr.MsgSaved[0].MsgTimeStamp);
    g_MsgCtr.MsgSaved[0].fMsgVal = 0;
    g_MsgCtr.MsgSaved[0].uMsgId = MSG_HuanYinShiYong;
    g_MsgCtr.MsgSaved[0].u32ItemNo = g_MsgCtr.u32MsgNo_ForAdd;
    g_MsgCtr.pLastAddMsg = &g_MsgCtr.MsgSaved[0];
	g_MsgCtr.i8Pt_MsgSaved = 0;
}

/*==========================================================================
| Description	: 清除Msg数据
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-11-04
\=========================================================================*/
void ResetMsgData(void)
{
	/* 文件系统操作 */
	Swi_disable();
	InitDataWithZero((uint8*)(&g_MsgCtr), sizeof(g_MsgCtr));
	Swi_enable();
#if SUPPORT_FAT_FS
	uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Msg");
	#if SUPPORT_MsgV1RW_AcqV1R
		EmptyFold(pFilePath);
	#endif
	*pFilePath++ = '2';		/* 删除 /msg2 */
	EmptyFold(pFilePath);
#endif

#if CPU_0ST_1TI
	/* EEPROM操作 */
	uint32 u32EEPromAddr = EEPROM_ADDR_MSG;
	uint32 u32EEPromLen = EEPROM_MSG_ITEM_NUM*sizeof(MSG_ITEM);
	InitDataWithZero(g_NvMemAcsCtr.u8FileCont, FILE_PATH_BUF_LEN);
	do {
		if(u32EEPromLen > FILE_PATH_BUF_LEN) {
			EEPROMProgram((uint32_t*)g_NvMemAcsCtr.u8FileCont, u32EEPromAddr, FILE_PATH_BUF_LEN);
			u32EEPromLen -= FILE_PATH_BUF_LEN;
			u32EEPromAddr += FILE_PATH_BUF_LEN;
		} else {
			EEPROMProgram((uint32_t*)g_NvMemAcsCtr.u8FileCont, u32EEPromAddr, u32EEPromLen);
			u32EEPromLen = 0;
		}
	} while(u32EEPromLen);
#endif
}

/* 把ram中的Msg通过modbus发送，一条消息占据6个modbus16bit寄存器 */
void ConvertMsgItem2MsgForModbus(uint8** ppBuf, MSG_ITEM* pMsg);
uint16 TxMsgForModbus(uint8** ppBuf, uint16 uRegNum)
{
    int8 i, iMsgNum;
	uint32 u32TxMsgNo = 0xFFFFFFFFUL;
	uint16 uTxMsgNum = uRegNum*2/sizeof(MSG_FOR_MODBUS);    /* 要读取的消息数量 */
	if(uTxMsgNum > MSG_SAVED_BUF_LEN) {
	    uTxMsgNum = MSG_SAVED_BUF_LEN;
	}
	
	Swi_disable();
    /* 尚未持久化的消息 */
    for(iMsgNum = uTxMsgNum; iMsgNum > 0; iMsgNum--) {   /* 最多16条，以免溢出 */
        /* 搜索ram中的MsgNo最大的 */
        MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
        uint32 u32MaxMsgNo = g_MsgCtr.u32MsgNo_EESaved;
        int8 i8MsgPt = -1;          /* 用于存储的MsgBuf下标 */
        for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
            /* 隐含了 u32ItemNo 非零，代表消息确认 */
            if((u32MaxMsgNo < pMsg->u32ItemNo) && (pMsg->u32ItemNo < u32TxMsgNo)) {
                u32MaxMsgNo = pMsg->u32ItemNo;
                i8MsgPt = i;
            }
            pMsg++;
        }
    
        if(i8MsgPt < 0) {   /* 已经找完了 */
            break;
        } else {
            pMsg = &g_MsgCtr.MsgBuf[i8MsgPt];
            u32TxMsgNo = pMsg->u32ItemNo;
            ConvertMsgItem2MsgForModbus(ppBuf, pMsg);
        }
    }

    /* 曾经在ram中，但现在已经持久化的消息 */
    int8 i8Pt = g_MsgCtr.i8Pt_MsgSaved;
    if((i8Pt >= 0) && (i8Pt <= MSG_SAVED_BUF_LEN - 1)) {    /* 确保指针没出错 */
        for( ; (iMsgNum > 0) && g_MsgCtr.MsgSaved[i8Pt].uMsgId; iMsgNum--) {
            ConvertMsgItem2MsgForModbus(ppBuf, &g_MsgCtr.MsgSaved[i8Pt]);
            i8Pt--;
            if(i8Pt < 0) {
                i8Pt = MSG_SAVED_BUF_LEN - 1;
            }
        }
    }
	Swi_enable();

    /* 剩余部分填0 */
	if(iMsgNum != 0) {
        uint8* pBuf = *ppBuf;
        InitDataWithZero(pBuf, iMsgNum*sizeof(MSG_FOR_MODBUS));
        pBuf += iMsgNum*sizeof(MSG_FOR_MODBUS);
        *ppBuf = pBuf;
    }
	
	return uRegNum;     /* 这个地方，不能用实际寄存器数，要不然调用处会继续调用 */
}

void ConvertMsgItem2MsgForModbus(uint8** ppBuf, MSG_ITEM* pMsg)
{
	uint8* pBuf = *ppBuf;
	
    *pBuf++ = pMsg->MsgTimeStamp.u8Year;
    *pBuf++ = pMsg->MsgTimeStamp.u8Month;
    *pBuf++ = pMsg->MsgTimeStamp.u8Day;
    *pBuf++ = pMsg->MsgTimeStamp.u8Hour;
    *pBuf++ = pMsg->MsgTimeStamp.u8Minute;
    *pBuf++ = pMsg->MsgTimeStamp.u8Second;
    *pBuf++ = pMsg->uMsgId/0x100;
    *pBuf++ = pMsg->uMsgId%0x100;
    *((uint32*)pBuf) = __rev(*((uint32*)&pMsg->fMsgVal));
    pBuf += 4;
    
	*ppBuf = pBuf;
}


/*==========================================================================
| Description	: 存储Msg消息(把Ram里面的可以存储的编码Msg解码成文本Msg，写入文件)
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2009-10-15
\=========================================================================*/
BOOL PrintMsgCode(DATA_USER DataUser, uint8** ppBuf, uint8* pBufEnd, MSG_ITEM* pMsg, BOOL bMsgCmplt, BOOL bTimeShort);
void SaveMsgToNvMem(void)
{
	MSG_ITEM* pMsg;
	uint32 u32MsgNo_ForAdd = g_MsgCtr.u32MsgNo_ForAdd;	/* 锁存，免得该函数运行期间发生消息添加引起不确定的运行结果 */
	uint32 u32MinMsgNo;		/* 超过已存储的最小MsgNo */
	int8 i;

#if SUPPORT_FAT_FS
	/* 把消息存储进文件中 */
	uint8* pBuf = g_NvMemAcsCtr.u8FileCont;
	BOOL bQuitFileSave = FALSE;
	int8 i8SearchCnt = MSG_ITEM_BUF_LEN;
	uint8 u8Year = 0;
	uint8 u8Month = 0;
	uint8 u8Day = 0;
	uint32 u32FileBufFirstItemNo = 0;
	uint32 u32MsgNo_FileSaved = g_MsgCtr.u32MsgNo_FileSaved;    /* 需要存进文件才能真正修改g_MsgCtr.u32MsgNo_FileSaved */
	do {
		/* 搜索高于已经存储的MsgNo */
		pMsg = g_MsgCtr.MsgBuf;
		u32MinMsgNo = 0xFFFFFFFFUL;
		int8 i8MsgPt = -1;
		for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
			if(pMsg->uMsgId && pMsg->u32ItemNo) {
				if((u32MsgNo_FileSaved < pMsg->u32ItemNo) && (pMsg->u32ItemNo < u32MinMsgNo)) {
					i8MsgPt = i;
					u32MinMsgNo = pMsg->u32ItemNo;
				}
			}
			pMsg++;
		}
		
		/* 最多搜索MSG_ITEM_BUF_LEN次，是否搜索完成 */
		i8SearchCnt--;
		bQuitFileSave = (i8SearchCnt <= 0);
		
		/* 打印成文本 */
		BOOL bPrintBreak = FALSE;       /* 新的消息，或者消息文件超了，需要形成新的文件 */
		if(i8MsgPt < 0) {   /* 已经找完了 */
			bQuitFileSave = TRUE;
		} else if(((g_MsgCtr.u32CmpltFlag & (1<<i8MsgPt)) != 0) || (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) 
					|| (u32MinMsgNo + MSG_ITEM_BUF_LEN*3/4 <= u32MsgNo_ForAdd))	/* MsgBuf已经有MSG_ITEM_BUF_LEN*3/4条 */
		{
			pMsg = &g_MsgCtr.MsgBuf[i8MsgPt];
			/* 新一天的消息，或者消息超了 */
			if((SUPPORT_MsgV1RW_AcqV1R && u8Day && ((u8Year != pMsg->MsgTimeStamp.u8Year) 
													|| (u8Month != pMsg->MsgTimeStamp.u8Month)
													|| (u8Day != pMsg->MsgTimeStamp.u8Day)))
		        || (u32FileBufFirstItemNo && (u32FileBufFirstItemNo/100000 != pMsg->u32ItemNo/100000)))	/* 每十万条一个文件 */
			{
			    bPrintBreak = TRUE;
			} else if(PrintMsgCode(DATA_USER_FILE, &pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], /* 解码 */
				                        pMsg, (g_MsgCtr.u32CmpltFlag & (1<<i8MsgPt)) != 0, FALSE))
			{
				u32MsgNo_FileSaved = pMsg->u32ItemNo;
				/* 仅记录第一次成功打印的ItemNo，要不然存储函数是后面Msg的ItemNo，造成文件名bug(应该是第一条，但实际不是) */
				if(u32FileBufFirstItemNo == 0) {
                    u32FileBufFirstItemNo = pMsg->u32ItemNo;
                }
                u8Year = pMsg->MsgTimeStamp.u8Year;
                u8Month = pMsg->MsgTimeStamp.u8Month;
                u8Day = pMsg->MsgTimeStamp.u8Day;
			} else {
				bQuitFileSave = TRUE;
			}
		} else {
			bQuitFileSave = TRUE;
		}

		/* 写入文件 */
		if(bPrintBreak
		    || (bQuitFileSave && (pBuf != g_NvMemAcsCtr.u8FileCont))
			|| (pBuf - g_NvMemAcsCtr.u8FileCont + 120 > FILE_CONT_BUF_LEN))		/* 单条MsgItem不超过120byte */
		{
            /* 保存进/msg2/sn.txt */
            BOOL bSaveRes = SaveMsgOrAcq_V2(DATA_PAGE_MSG, pBuf, u32FileBufFirstItemNo);
		#if SUPPORT_MsgV1RW_AcqV1R	/* 保存进/msg/年/月/日.txt */
            bSaveRes |= SaveMsgOrAcq_V1(DATA_PAGE_MSG, pBuf, u8Year, u8Month, u8Day, u32FileBufFirstItemNo);
		#endif
            if(bSaveRes) {  /* 为下一次打印初始化 */
                g_MsgCtr.u32MsgNo_FileSaved = u32MsgNo_FileSaved;
                pBuf = g_NvMemAcsCtr.u8FileCont;
                u32FileBufFirstItemNo = 0;
            } else {
                bQuitFileSave = TRUE;
            }
		}
	} while(!bQuitFileSave);
#endif

	/* 把消息存储进eeprom中 */
	for(i = MSG_ITEM_BUF_LEN; i > 0; i--) {
		/* 搜索高于已经存储的MsgNo */
		MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
		u32MinMsgNo = 0xFFFFFFFFUL;		/* 超过已存储的最小MsgNo */
		int8 i8MsgPt = -1;
		int8 j;
		for(j = 0; j < MSG_ITEM_BUF_LEN; j++) {
			if((g_MsgCtr.u32MsgNo_EESaved < pMsg->u32ItemNo) && (pMsg->u32ItemNo < u32MinMsgNo)) {
				u32MinMsgNo = pMsg->u32ItemNo;
				i8MsgPt = j;
			}
			pMsg++;
		}

		/* 存储 */
		if(i8MsgPt < 0) {
			break;		
		} else if(((g_MsgCtr.u32CmpltFlag & (1<<i8MsgPt)) != 0) || (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) 
					|| (u32MinMsgNo + MSG_ITEM_BUF_LEN*3/4 <= u32MsgNo_ForAdd))	/* MsgBuf已经有MSG_ITEM_BUF_LEN*3/4条 */
		{
			/* 把消息存储进eeprom中 */
			pMsg = &g_MsgCtr.MsgBuf[i8MsgPt];
			pMsg->uEEPromNo = g_MsgCtr.uEEPromNo_ForSave++;
		#if CPU_0ST_1TI
			EEPROMProgram((uint32_t*)pMsg, EEPROM_ADDR_MSG + (pMsg->uEEPromNo%EEPROM_MSG_ITEM_NUM)*sizeof(MSG_ITEM), sizeof(MSG_ITEM));
		#endif
			g_MsgCtr.u32MsgNo_EESaved = pMsg->u32ItemNo;
            /* 把已经持久化的消息迁移到MsgSaved，以备快速访问 */
            Swi_disable();
			/* 避免软件调试的时候，在MsgSaved有两个一样的ItemNo */
            if(g_MsgCtr.MsgSaved[g_MsgCtr.i8Pt_MsgSaved].uMsgId != MSG_HuanYinShiYong) {
    			g_MsgCtr.i8Pt_MsgSaved++;
    			if(g_MsgCtr.i8Pt_MsgSaved >= MSG_SAVED_BUF_LEN) {
    			    g_MsgCtr.i8Pt_MsgSaved = 0;
    			}
			}
			g_MsgCtr.MsgSaved[g_MsgCtr.i8Pt_MsgSaved] = *pMsg;
            Swi_enable();
		} else {
			break;
		}
	}
#if !SUPPORT_FAT_FS	/* 如果没有tf卡存储，则这一项跟随eeprom，否则后面清除消息运行容易出错 */
	g_MsgCtr.u32MsgNo_FileSaved = g_MsgCtr.u32MsgNo_EESaved;
#endif

	/* 清空已经存储的MsgBuf */
	/* MsgBuf中实际条数是 u32MsgNo_ForAdd - u32MsgNo_FileSaved - 1, 因此消息数量 >= MSG_ITEM_BUF_LEN*3/4触发该条款
	前面存储的时候，在消息条数MSG_ITEM_BUF_LEN*3/4的时候，就会不等完成就存储，以减少MsgBuf中消息条数，因此除非tf卡损坏，要不然不应该触发该条件 */
	if(u32MsgNo_ForAdd - g_MsgCtr.u32MsgNo_FileSaved > MSG_ITEM_BUF_LEN*3/4) {
		u32MinMsgNo = u32MsgNo_ForAdd - MSG_ITEM_BUF_LEN*3/4;
	} else {
		u32MinMsgNo = g_MsgCtr.u32MsgNo_FileSaved;
	}
	g_MsgCtr.u32MsgNo_ram = u32MinMsgNo;    /* 该MsgNo以上的Msg还存储在ram中，方便读取 */
	pMsg = g_MsgCtr.MsgBuf;
	Swi_disable();
	/* 可能没有完成的消息也进行了存储、清除，在这里"被逼完成"，因此需要更新u32MsgNo_Cmplt */
    if(g_MsgCtr.u32MsgNo_Cmplt < g_MsgCtr.u32MsgNo_ram) {
        g_MsgCtr.u32MsgNo_Cmplt = g_MsgCtr.u32MsgNo_ram;
    }
    /* 由于不是从最小的ItemNo开始清理，因此需要关任务调度 */
	for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
		if(pMsg->u32ItemNo <= u32MinMsgNo) {
			pMsg->uMsgId = 0;
			pMsg->u32ItemNo = 0;
			g_MsgCtr.u32CmpltFlag &= ~(1<<i);
		}
		pMsg++;
	}
	/* 最近添加消息指针指向的消息已经不在了，指向存储消息里面最新的 */
    if(g_MsgCtr.pLastAddMsg->u32ItemNo == 0) {
        g_MsgCtr.pLastAddMsg = &g_MsgCtr.MsgSaved[g_MsgCtr.i8Pt_MsgSaved];
    }
	Swi_enable();

	/* 该函数运行期间，有新的消息加入，需要再次启动存储 */
	if(u32MsgNo_ForAdd != g_MsgCtr.u32MsgNo_ForAdd) {
    	g_NvMemAcsCtr.bNeedSaveMsg = TRUE;
        Semaphore_post(g_DataAccessBlock.Sem_Op);   //Semaphore_post(SEM_NvMemAcs); /* 启动存储任务，把新产生的消息打印出来，发送给DB */
	}
}

/*==========================================================================
| Description	: 把Msg的二进制编码转化为文本形式
mcgs最长编码 60byte, 其中时间部分为22byte,消息部分不得超过38byte--即必须妥善编码，使得文本小于60byte
file = mcgs +23byte(含回车换行)
net = mcgs + 52byte(完整的json字符串)
bTimeShort 短时间格式，仅含：时分秒，主要用于VT屏幕，编码的时候也仅考虑了VT屏幕
| In/Out/G var	: Buf满，返回False；否则返回TRUE
| Author		: Wang Renfei			Date	: 2016-07-16
\=========================================================================*/
BOOL PrintMsgCode(DATA_USER DataUser, uint8** ppBuf, uint8* pBufEnd, MSG_ITEM* pMsg, BOOL bMsgCmplt, BOOL bTimeShort)
{
	uint8* pBufStart = *ppBuf;
	uint8* pBuf = pBufStart;
	char* pPropText;

	/* ItemNo部分: C1~C6 */
	uint32 u32ItemNo = pMsg->u32ItemNo;
	if(pMsg->uMsgId >= MAX_MSG_TYPE) {
		return FALSE;
	} else if(DataUser == DATA_USER_FILE) {
		if(pBuf + 84 > pBufEnd) {				/* Buf长度检查:保证了前面确定性的结果 */
			return FALSE;
		}
		PrintU32WithLen(&pBuf, u32ItemNo, 6); 	/* ItemNo部分:形成6位格式 */
		*pBuf++ = ' ';
	} else if(DataUser == DATA_USER_NET) {		/* Buf长度检查:保证了前面确定性的结果 */
		if(pBuf + 142 > pBufEnd) {
			return FALSE;
		}
		*pBuf++ = '{';
		PrintU32DatToJson(&pBuf, "no", u32ItemNo, 0);
		PrintStringNoOvChk(&pBuf, "\"time\":\"");
#if SUPPORT_SCREEN
	} else if(DataUser == DATA_USER_MCGS) {
		if(pBuf + MCGS_LEN_ITEM_CHAR > pBufEnd) {		/* Buf长度检查 */
			return FALSE;
		} else {
			pBufEnd = pBufStart + MCGS_LEN_ITEM_CHAR;	/* 对于 MCGS 屏幕，需要把数据限制在当前行内 */
		}
	} else if(DataUser == DATA_USER_VIEWTECH) {
		if(pBuf + VT_LEN_ITEM_BYTE > pBufEnd) {		/* Buf长度检查 */
			return FALSE;
		} else {
			pBufEnd = pBufStart + VT_LEN_ITEM_CHAR;	/* 对于ViewTech屏幕，需要把数据限制在当前行内 */
		}
	}
#endif

	if(pMsg->uMsgId != MSG_NULL) {	/* 如果空消息，则不打印时间，因为时间很可能是错的 */
	    if(!bTimeShort) {
    		PrintT64(DataUser, &pBuf, pBufEnd, &pMsg->MsgTimeStamp);		/* 时间部分 */
    	} else {
            uint8 u8Data = pMsg->MsgTimeStamp.u8Second;
            *(pBuf + 8) = ' ';
            *(pBuf + 7) = u8Data%10 + '0';
            *(pBuf + 6) = (u8Data/10)%10 + '0';
            *(pBuf + 5) = ':';
            u8Data = pMsg->MsgTimeStamp.u8Minute;
            *(pBuf + 4) = u8Data%10 + '0';
            *(pBuf + 3) = (u8Data/10)%10 + '0';
            *(pBuf + 2) = ':';
            u8Data = pMsg->MsgTimeStamp.u8Hour;
            *(pBuf + 1) = u8Data%10 + '0';
            *(pBuf + 0) = (u8Data/10)%10 + '0';
            pBuf += 9;
    	}
	}
	if(DataUser == DATA_USER_NET) {
		PrintStringNoOvChk(&pBuf, "\",\"text\":\"");
	} else {	
		*pBuf++ = ' ';
	}
	/* 之上的Buf溢出检测在入口处做的，已经考虑了最坏的情况 */
	
	/* MsgProp + fMsgVal部分, 由于可能有插入点，因此需要分断处理字符串 */
	if(g_Sys.u32LocalLang >= MAX_LANG_TYPE) {
		g_Sys.u32LocalLang = CHINESE;
	}
	pPropText = cnst_MsgProp[pMsg->uMsgId].pText[g_Sys.u32LocalLang];
	if(((cnst_MsgProp[pMsg->uMsgId].i8DataPosition >= 0)				/* 插入点之前的PropText处理 */
			&& ((!PrintStringWithCharNum(&pBuf, pBufEnd, &pPropText, cnst_MsgProp[pMsg->uMsgId].i8DataPosition))
				|| (!PrintFM32(&pBuf, pBufEnd, pMsg->fMsgVal, cnst_MsgProp[pMsg->uMsgId].F32Meaning))))
		|| (!PrintString(&pBuf, pBufEnd, pPropText)))	/* 插入点之后的PropText处理 */
	{
		if((DataUser == DATA_USER_MCGS) || (DataUser == DATA_USER_VIEWTECH)) {		/* 对于屏幕，接受不完整的一条 */
		} else {
			return FALSE;
		}
	}

#if SUPPORT_SCREEN
	if(DataUser == DATA_USER_MCGS) {
		/* 如果Buf未满需要添加0作为字符串的结尾, 满了可以不用 */
		if(pBuf < pBufEnd) {
			*pBuf++ = 0;
			pBuf = pBufStart + MCGS_LEN_ITEM_CHAR;
		}
	} else if(DataUser == DATA_USER_VIEWTECH) {
		/* 如果Buf未满需要添加0作为字符串的结尾, 满了可以不用 */
		if(pBuf < pBufEnd) {
			*pBuf++ = 0;
			pBuf = pBufStart + VT_LEN_ITEM_CHAR;
		}
		*((uint16*)pBuf) = pMsg->uMsgId;	/* MsgID字段 */
		pBuf += 2;
#endif
	} else if(DataUser == DATA_USER_FILE) {
		if(pBuf + 17 < pBufEnd) {		/* 后续总长16byte */
			/* 添加 msg_id */
			*pBuf++ = ' ';
			PrintH16(&pBuf, pBufEnd, pMsg->uMsgId);
			if(!bMsgCmplt) {				/* ID后添加*代表消息写入时并未完成 */
				*pBuf++ = '*';
			}
			
			/* 添加msg_val */
			*pBuf++ = ' ';
			PrintF32(&pBuf, pBufEnd, pMsg->fMsgVal, 4);

			/* 添加CRC、回车换行 */
			*pBuf++ = ' ';
			PrintH32(&pBuf, pBufEnd, CalCRC32ForFileByIni(pBufStart, pBuf - pBufStart, 0xFFFFFFFF));
			*pBuf = 0x0D;
			*(pBuf + 1) = 0x0A;
			pBuf += 2;
		} else {
			return FALSE;
		}
	} else if(DataUser == DATA_USER_NET) {	
		if(pBuf + 50 < pBufEnd) {		/* 后续总长最多42byte，多8byte */
			if(bMsgCmplt) {			/* 添加id */
				*pBuf++ = '"';
				*pBuf++ = ',';
				PrintH16DatToJson(&pBuf, "msg_id", pMsg->uMsgId);
				PrintF32DatToJson(&pBuf, "msg_val", pMsg->fMsgVal, 4);	/* 值最多10个字符 */
				pBuf--;
				*pBuf++ = '}';
				*pBuf++ = ',';
			} else {				/* 消息不完整，就不添加id */
				*pBuf++ = '"';
				*pBuf++ = '}';
				*pBuf++ = ',';
			}
		} else {
			return FALSE;
		}
	}

	*ppBuf = pBuf;
	return TRUE;
}

/*==========================================================================
| Description	: 在ConvertMsgCode2Text基础上，针对EEProm里面的数据进行了输入检查
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2018-7-24
\=========================================================================*/
CONV_MSG_ACQ_CODE_RES ConvertMsgEECode2Text(DATA_USER DataUser, uint8** ppBuf, uint8* pBufEnd, MSG_ITEM* pMsgEEItem)
{
	/* 输入检查 */
	if((!isfinite(pMsgEEItem->fMsgVal)) || (pMsgEEItem->uMsgId == MSG_NULL)	|| (pMsgEEItem->uMsgId >= MAX_MSG_TYPE)) {
		return CONV_MSG_ACQ_CODE_ERR_DATA;
	} else if(PrintMsgCode(DataUser, ppBuf, pBufEnd, pMsgEEItem, TRUE, FALSE)) {
		return CONV_MSG_ACQ_CODE_SUC;
	} else {
		return CONV_MSG_ACQ_CODE_ERR_FULL;
	}
}

/* 搜索ram中连续的最后一条 完成消息 */
uint32 SeekCmpltMsgNo(void)
{
    uint32 u32CmpltMsgNo = g_MsgCtr.u32MsgNo_EESaved;
    int8 i, j;
    for(j = MSG_ITEM_BUF_LEN; j > 0; j--) {
        MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
        /* 搜索超过u32CmpltMsgNo最小的MsgNo   */
        uint32 u32MinCmpltMsgNoInRam = 0xFFFFFFFFUL;
        int8 i8MsgPt = -1;          /* 用于存储的MsgBuf下标 */
        for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
            if((u32MinCmpltMsgNoInRam > pMsg->u32ItemNo) && (pMsg->u32ItemNo > u32CmpltMsgNo)) {
                u32MinCmpltMsgNoInRam = pMsg->u32ItemNo;
                i8MsgPt = i;
            }
            pMsg++;
        }
        if((i8MsgPt >= 0) && ((g_MsgCtr.u32CmpltFlag & (1<<i8MsgPt)) != 0)) {
            u32CmpltMsgNo = u32MinCmpltMsgNoInRam;
        } else {
            break;
        }
    }
    return u32CmpltMsgNo;
}

/*==========================================================================
| Description	: A类事件: 添加事件消息，虽然设置阻塞，但是不被阻塞, 仅考虑该函数的重入性
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2012-1-2
\=========================================================================*/
void AddMsgA_WithVal(uint16 uMsgId, float32 fMsgVal)
{
	if((!isfinite(fMsgVal)) || (uMsgId == MSG_NULL) || (uMsgId >= MAX_MSG_TYPE)) {	/* 输入检查 */
		return;
#ifdef MSG_ABORNMAL_START
	} else if(uMsgId >= MSG_ABORNMAL_START) {	/* 异常消息报警 */
		g_Ctr.bBeepAbornmal = TRUE;
#endif
	}
	
	MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
	int8 i;
	BOOL bCmpltMsg = FALSE;
	Swi_disable();
	for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
		if(pMsg->uMsgId == MSG_NULL) {				/* 寻找空的存储空间 */
			g_MsgCtr.u32ShieldFlag[uMsgId/32] |= 1UL<<(uMsgId%32);
			GetRealTime(&pMsg->MsgTimeStamp);
			pMsg->fMsgVal = fMsgVal;
			pMsg->uMsgId = uMsgId;
			pMsg->u32ItemNo = g_MsgCtr.u32MsgNo_ForAdd++;
			g_MsgCtr.u32CmpltFlag |= 1<<i;
            bCmpltMsg = TRUE;
			g_MsgCtr.pLastAddMsg = pMsg;
			break;
		}
		pMsg++;
	}
	if(bCmpltMsg) {
    	g_MsgCtr.u32MsgNo_Cmplt = SeekCmpltMsgNo();
    	g_NvMemAcsCtr.bNeedSaveMsg = TRUE;
        Semaphore_post(g_DataAccessBlock.Sem_Op);   //Semaphore_post(SEM_NvMemAcs);	/* 启动存储任务，把新产生的消息打印出来，发送给DB */
    }
	Swi_enable();
}

/*==========================================================================
| Description	: B类事件: 添加事件消息(有数据)，且进行阻塞
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-3-1
\=========================================================================*/
void AddMsgB_WithVal(uint16 uMsgId, float32 fMsgVal)
{
	if((!isfinite(fMsgVal)) || (uMsgId == MSG_NULL) || (uMsgId >= MAX_MSG_TYPE)) {	/* 输入检查 */
		return;
#ifdef MSG_ABORNMAL_START
	} else if(uMsgId >= MSG_ABORNMAL_START) {	/* 异常消息报警，即使被阻塞，报警仍然要有 */
		g_Ctr.bBeepAbornmal = TRUE;
#endif
	}
	if(g_MsgCtr.u32ShieldFlag[uMsgId/32] & (1UL<<(uMsgId%32))) { 		/* 阻塞检查 */
		return;
	}
	
	MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
	int8 i;
	BOOL bCmpltMsg = FALSE;
	Swi_disable();
	for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
		if(pMsg->uMsgId == MSG_NULL) {				/* 寻找空的存储空间 */
			g_MsgCtr.u32ShieldFlag[uMsgId/32] |= 1UL<<(uMsgId%32);
			GetRealTime(&pMsg->MsgTimeStamp);
			pMsg->fMsgVal = fMsgVal;
			pMsg->uMsgId = uMsgId;
			pMsg->u32ItemNo = g_MsgCtr.u32MsgNo_ForAdd++;
			g_MsgCtr.u32CmpltFlag |= 1<<i;			
            bCmpltMsg = TRUE;
			g_MsgCtr.pLastAddMsg = pMsg;
			break;
		}
		pMsg++;
	}
	if(bCmpltMsg) {
    	g_MsgCtr.u32MsgNo_Cmplt = SeekCmpltMsgNo();
    	g_NvMemAcsCtr.bNeedSaveMsg = TRUE;
        Semaphore_post(g_DataAccessBlock.Sem_Op);   //Semaphore_post(SEM_NvMemAcs); 	/* 启动存储任务，把新产生的消息打印出来，发送给DB */
    }
	Swi_enable();
}

/*==========================================================================
| Description	: C类事件:更新事件数值
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-10-3
\=========================================================================*/
void UpdateMsgC_WithVal(uint16 uMsgId, BOOL bMax1OrMin0, float32 fMsgVal)
{
	if((!isfinite(fMsgVal)) || (uMsgId == MSG_NULL)	|| (uMsgId >= MAX_MSG_TYPE)		/* 输入检查 */
		|| (g_MsgCtr.u32ShieldFlag[uMsgId/32] & (1UL<<(uMsgId%32))))			/* 阻塞检查 */
	{
		return;
	}

	/* 搜索，看是否已经存在未确认的Msg--即确定是更新值还是添加Msg */
	MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
	int8 i;
	BOOL bNeedAddNewMsg = TRUE;
	for(i = 0; bNeedAddNewMsg && (i < MSG_ITEM_BUF_LEN); i++) {
		if((pMsg->uMsgId == uMsgId) && ((g_MsgCtr.u32CmpltFlag & (1UL<<i)) == 0)) {
			bNeedAddNewMsg = FALSE;
		} else {
			pMsg++;
		}
	}

	/* 添加Msg */
	if(bNeedAddNewMsg) {
		pMsg = g_MsgCtr.MsgBuf;
		Swi_disable();
		for(i = MSG_ITEM_BUF_LEN - 1; i >= 0; i--) {
			if(pMsg->uMsgId == 0) {						/* 寻找空的存储空间 */
				GetRealTime(&pMsg->MsgTimeStamp);
				pMsg->fMsgVal = fMsgVal;
				pMsg->uMsgId = uMsgId;
				break;
			}
			pMsg++;
		}
		Swi_enable();
	} else if((bMax1OrMin0 && (fMsgVal > pMsg->fMsgVal))
			|| ((!bMax1OrMin0) && (fMsgVal < pMsg->fMsgVal)))
	{
		pMsg->fMsgVal = fMsgVal;
	}
}

/*==========================================================================
| Description	: C类事件:确认事件消息
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-10-8
\=========================================================================*/
void ConfirmMsgC(uint16 uMsgId)
{
	if((uMsgId == MSG_NULL) || (uMsgId >= MAX_MSG_TYPE)) {	/* 输入检查 */
		return;		
#ifdef MSG_ABORNMAL_START
	} else if(uMsgId >= MSG_ABORNMAL_START) {				/* 异常消息报警，即使被阻塞，报警仍然要有 */
		g_Ctr.bBeepAbornmal = TRUE;
#endif
	}
	if(g_MsgCtr.u32ShieldFlag[uMsgId/32] & (1UL<<(uMsgId%32))) {	/* 阻塞检查 */
		return;
	}

	MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
	int8 i;
	Swi_disable();
	for(i = MSG_ITEM_BUF_LEN - 1; i >= 0; i--) {
		if((pMsg->uMsgId == uMsgId) && (pMsg->u32ItemNo == 0)) {
			pMsg->u32ItemNo = g_MsgCtr.u32MsgNo_ForAdd++;			
            g_MsgCtr.pLastAddMsg = pMsg;
			break;
		}
		pMsg++;
	}
	Swi_enable();
}

/*==========================================================================
| Description	: C类事件:完成消息:对于未确认的消息，放弃; 对于确认的消息，保存
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-2-27
\=========================================================================*/
void FinishMsgC(uint16 uMsgId, BOOL bNeedBlock)
{
	/* 阻塞检查 */
	if((uMsgId == MSG_NULL) || (uMsgId >= MAX_MSG_TYPE)) {
		return;
	}

	MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
	int8 i;
	BOOL bCmpltMsg = FALSE;
	Swi_disable();
	for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
		if((pMsg->uMsgId == uMsgId) && ((g_MsgCtr.u32CmpltFlag & (1UL<<i)) == 0)) {
			if(pMsg->u32ItemNo) {
				if(bNeedBlock) {	/* 阻塞 */
					g_MsgCtr.u32ShieldFlag[uMsgId/32] |= 1UL<<(uMsgId%32);
				}
				g_MsgCtr.u32CmpltFlag |= 1<<i;
				bCmpltMsg = TRUE;
			} else {
				pMsg->uMsgId = 0;
			}
			break;
		}
		pMsg++;
	}
	if(bCmpltMsg) {
    	g_MsgCtr.u32MsgNo_Cmplt = SeekCmpltMsgNo();
    	g_NvMemAcsCtr.bNeedSaveMsg = TRUE;
        Semaphore_post(g_DataAccessBlock.Sem_Op);   	//Semaphore_post(SEM_NvMemAcs);	/* 启动存储任务，把新产生的消息打印出来，发送给DB */
	}
	Swi_enable();
}

void ClearAllMsgShieldFlag(void)
{
	int8 i;
	for(i = 0; i < MSG_SHIELD_FLAG_LEN; i++) {
		g_MsgCtr.u32ShieldFlag[i] = 0;
	}
}

void ClearSingleMsgShieldFlag(uint16 uMsgId)
{
	if((uMsgId != MSG_NULL) && (uMsgId < MAX_MSG_TYPE)) {
		g_MsgCtr.u32ShieldFlag[uMsgId/32] &= ~(1UL<<(uMsgId%32));
	}
}
#endif

/*==========================================================================
| Function name	: void Acq(void)
| Description	: 采集类
	采集变量在文件中的记录格式为: AcqID + RecTime + Data(0~n) + LRC

	数据存储过程:
		1. 通过接口函数，数据保存在Buf中，接口输入:"AcqID(通过查找关联有效数据数量) + 数据"，加上时间戳、ItemNo
		2. 把数据从Buf中加上"时间戳、ItemNo、校验数据"，转化为文本，写入文件(所有数据在一个文件中)

	控制部分读取数据过程:
		1. 依据(ACQID + 数据指针 + 起始ItemNo + 读取Item数量)从文件系统获得具体采集项目数据(二进制)，写入RAM
	
	通讯/显示访问数据以文本形式(从文件中读取即可)，包括:
		1. 依据(ACQID + 起始ItemNo + 读取Item数量)从文件系统获得具体采集项目数据(文本)，提交给通讯部分

| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-6-6
\=========================================================================*/
#if ACQ_ITEM_DAT_NUM
BOOL ChkAcqIDWithReadAcqData(uint16 uReadAcqId, uint16 uSeekAcqId);
#define ACQ_WRITE_BUF_LEN			4			/* Acq写Buf大小 */
#define ACQ_READ_BUF_LEN 			200			/* Acq读Buf大小 */
#define ACQID_NULL					0			/* 空的AcqID */
typedef struct {								/* 单条数据 */
	REAL_TIME_VAR AcqTimeStamp; 				/* 事件时间戳 */
	uint32 u32ItemNo;							/* 赋予ItemNo代表事件是完整的 */
	uint16 uAcqId;
	uint16 uEEPromNo;							/* 存储在EEProm中的序号 */
	float32 fD[ACQ_ITEM_DAT_NUM];
}ACQ_WRITE_ITEM;
typedef struct {								/* 在EEProm中存储的数据格式 */
	uint32 u32AcqTimeStamp;						/* 仅存储:年、月、日、时 */
	uint32 u32ItemNo;
	uint16 uAcqId;
	uint16 uEEPromNo;							/* 存储在EEProm中的序号 */
	float32 fD[ACQ_ITEM_DAT_NUM];
}ACQ_EE_ITEM_VAR;
typedef struct {
	/* AcqNo用于记录Acq的序号 */
	uint32 u32AcqNo_ForAdd;						/* 下一条添加的AcqNo */
	uint32 u32AcqNo_FileSaved;					/* 文件中已经存储的AcqNo */
	uint32 u32AcqNo_EESaved;					/* EEProm已经存储的AcqNo */
	uint32 u32AcqNo_ram;                        /* ram存储的MsgNo起始(不含) */
	uint16 uEEPromNo_ForSave;					/* 用于存储下一条的EEPromNo */
	
	/* 添加Acq数据进入Buf */
	uint16 uBufPt_ForAdd;						/* 缓冲区写入指针，指向最后一条新写入Ram的数据 */
	uint16 uBufPt_ForFileSave;					/* 缓冲区写入文件指针, 指向最后一条已存入文件的数据 */
	uint16 uBufPt_ForEEPromSave;				/* 缓冲区写入EEProm指针, 指向最后一条已存入EEProm的数据 */
	ACQ_WRITE_ITEM AcqWriteBuf[ACQ_WRITE_BUF_LEN];/* 写入缓冲区 */

	/* 从NvMem中读取数据 */
	uint32 u32AcqSummNeedProc;					/* 一个位代表 cnst_Acq_Summ_Map_Table 中对应的请求 ，由于该表很短，32bit足够表达了 */
	ACQ_READ_ITEM AcqReadBuf[ACQ_READ_BUF_LEN];
}ACQ_CTR;
SECTION(".NOT_ZeroInit") ACQ_CTR g_AcqCtr;
void InitAcqMdl(void)
{
	if(g_Sys.uRstCount) {	/* 仅上电复位才初始化 */
		return;
	}
	
	g_AcqCtr.uBufPt_ForAdd = 0;
	g_AcqCtr.uBufPt_ForFileSave = 0;
	g_AcqCtr.uBufPt_ForEEPromSave = 0;
	int16 i;
	for(i = ACQ_WRITE_BUF_LEN-1; i >= 0; i--) {
		g_AcqCtr.AcqWriteBuf[i].u32ItemNo = 0;
		g_AcqCtr.AcqWriteBuf[i].uAcqId = ACQID_NULL;
	}

	g_AcqCtr.u32AcqSummNeedProc = 0;
	
	/* 读取最后一行的行号:首先是文件中，如果文件中读取失败则从eeprom中 */
#if SUPPORT_FAT_FS		/* 搜索文件, 获得ItemNo */
	uint32 u32LastItemNoFromFile = 0;
	/* ReadLastItemNoFromFile()返回0说明读失败 */
	for(i = 5; (i > 0) && ((u32LastItemNoFromFile = ReadLastItemNoFromFile(DATA_PAGE_ACQ, MSG_ACQ_ACS_V2)) == 0); i--) {
	}
	#if 0 // 删除对acqV1 ItemNo的支持,但是这段代码保留，以备哪天需要远程读取使用 SUPPORT_MsgV1RW_AcqV1R
		uint32 u32LastItemNoFromFileV1 = ReadLastItemNoFromFile(DATA_PAGE_ACQ, MSG_ACQ_ACS_V1);
		if(u32LastItemNoFromFile < u32LastItemNoFromFileV1) {
			u32LastItemNoFromFile = u32LastItemNoFromFileV1;
		}
	#endif
#else
	uint32 u32LastItemNoFromFile = 0;
#endif

#if CPU_0ST_1TI
	uint32 u32EEPromAddr = EEPROM_ADDR_ACQ;
	uint16 uRemainItemInEEProm = EEPROM_ACQ_ITEM_NUM;	/* 计算总的读取长度 */
	uint32 u32LastItemNoFromEEProm = 0; 				/* 来自EEProm中所存储的最后一个ItemNo */
	uint16 uExpectEEPromNo = 0; 						/* 期望的EEPromNo */
	/* 搜索EEPromNo发生跳变的, 取其所保存的ItemNo, EEPromNo;
	   如果一直没有跳变，则说明数据存储位置正好回到开始，取末位EEPromNo增一作为EEPromNo */
	do {
		/* 从EEProm中读出数据 */
		uint16 uCurReadItemNum = FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR); /* 保证读出整数条 */
		if(uCurReadItemNum > uRemainItemInEEProm) { 	/* 确保不会超出范围 */
			uCurReadItemNum = uRemainItemInEEProm;
		}
		EEPROMRead((uint32_t*)g_NvMemAcsCtr.u8FileCont, u32EEPromAddr, uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR));
	
		/* 搜索数据 */
		ACQ_EE_ITEM_VAR* pSeekAcqBuf = (ACQ_EE_ITEM_VAR*)g_NvMemAcsCtr.u8FileCont;
		if(uExpectEEPromNo == 0) {	/* 初始化uExpectEEPromNo */
			/* 新擦除的芯片 */
			if(((pSeekAcqBuf->u32ItemNo == 0xFFFFFFFFUL) && (pSeekAcqBuf->uAcqId == 0xFFFFUL) && (pSeekAcqBuf->uEEPromNo == 0xFFFFUL))
				|| (pSeekAcqBuf->uEEPromNo%EEPROM_ACQ_ITEM_NUM))	/* 第一个数据EEPromNo应该是EEPROM_ACQ_ITEM_NUM整数倍，否则可能数据破坏 */
			{
				u32LastItemNoFromEEProm = 0;
				break;
			} else {
				uExpectEEPromNo = pSeekAcqBuf->uEEPromNo;
			}
		}
		/* EEPromNo和预想的不一样(即自增一), 则说明找到插入点 */
		for(i = uCurReadItemNum; (i > 0) && (pSeekAcqBuf->uEEPromNo == uExpectEEPromNo); i--) {
			u32LastItemNoFromEEProm = pSeekAcqBuf->u32ItemNo;
			uExpectEEPromNo++;
			pSeekAcqBuf++;
		}
	
		/* 有结果，跳出循环 */
		if(i) {
			break;
		} else {	/* 无结果，准备下一次循环 */
			uRemainItemInEEProm -= uCurReadItemNum;
			u32EEPromAddr += uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR);
		}
	} while(uRemainItemInEEProm);
	g_AcqCtr.uEEPromNo_ForSave = uExpectEEPromNo;
#endif

	/* 存入 u32AcqNo_Saved */
	if(u32LastItemNoFromFile) {
		g_AcqCtr.u32AcqNo_FileSaved = u32LastItemNoFromFile;
		g_AcqCtr.u32AcqNo_EESaved = u32LastItemNoFromFile;
	} else {
    #if CPU_0ST_1TI
		g_AcqCtr.u32AcqNo_FileSaved = u32LastItemNoFromEEProm;
		g_AcqCtr.u32AcqNo_EESaved = u32LastItemNoFromEEProm;
	#else
	    g_AcqCtr.u32AcqNo_FileSaved = 0;
	    g_AcqCtr.u32AcqNo_EESaved = 0;
	#endif
	}
	g_AcqCtr.u32AcqNo_ForAdd = g_AcqCtr.u32AcqNo_FileSaved + 1;
	g_AcqCtr.u32AcqNo_ram = g_AcqCtr.u32AcqNo_FileSaved;
}

void RandAcqData(void)
{
	int16 i, j;
	for(i = 0; i < ACQ_READ_BUF_LEN; i++) {
		g_AcqCtr.AcqReadBuf[i].uDaysFromOrigin = i;
		g_AcqCtr.AcqReadBuf[i].uAcqId = 0x1000;
		for(j = 0; j < ACQ_ITEM_DAT_NUM; j++) {
			g_AcqCtr.AcqReadBuf[i].fD[j] = rand();
		}
	}
}

/*==========================================================================
| Description	: 清除Acq数据
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-11-04
\=========================================================================*/
void ResetAcqData(void)
{
	/* 文件系统操作 */
	Swi_disable();
	InitDataWithZero((uint8*)(&g_AcqCtr), sizeof(g_AcqCtr));
	Swi_enable();
#if SUPPORT_FAT_FS
	uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Acq");
	#if SUPPORT_MsgV1RW_AcqV1R	/* 删除/acq目录 */
		EmptyFold(pFilePath);
	#endif
	*pFilePath++ = '2';		/* 删除 /acq2目录 */
	EmptyFold(pFilePath);
#endif

	/* EEPROM操作 */
#if CPU_0ST_1TI
	uint32 u32EEPromAddr = EEPROM_ADDR_ACQ;
	uint32 u32EEPromLen = EEPROM_ACQ_ITEM_NUM*sizeof(ACQ_EE_ITEM_VAR);
	InitDataWithZero(g_NvMemAcsCtr.u8FileCont, FILE_PATH_BUF_LEN);
	do {
		if(u32EEPromLen > FILE_PATH_BUF_LEN) {
			EEPROMProgram((uint32_t*)g_NvMemAcsCtr.u8FileCont, u32EEPromAddr, FILE_PATH_BUF_LEN);
			u32EEPromLen -= FILE_PATH_BUF_LEN;
			u32EEPromAddr += FILE_PATH_BUF_LEN;
		} else {
			EEPROMProgram((uint32_t*)g_NvMemAcsCtr.u8FileCont, u32EEPromAddr, u32EEPromLen);
			u32EEPromLen = 0;
		}
	} while(u32EEPromLen);
#endif
}

/*==========================================================================
| Description	: 添加采集到的数值，如果空余的AcqItemBuf小于等于SAVE_ACQ_BUF_MAX_FREE_ITEM_NUM，就启动写入
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-7-16
\=========================================================================*/
void AddAcqItem(uint16 uAcqId, float32 fD0, float32 fD1, float32 fD2, float32 fD3, float32 fD4)
{
	if(g_AcqCtr.uBufPt_ForAdd >= ACQ_WRITE_BUF_LEN) {
		g_AcqCtr.uBufPt_ForAdd = 0;
	}
	
	ACQ_WRITE_ITEM* pAcqItem = &g_AcqCtr.AcqWriteBuf[g_AcqCtr.uBufPt_ForAdd];
	if((pAcqItem->uAcqId == ACQID_NULL) && isfinite(fD0) && isfinite(fD1) && isfinite(fD2) 
		&& isfinite(fD3) && isfinite(fD4)) 
	{
		GetRealTime(&pAcqItem->AcqTimeStamp);
		if(pAcqItem->AcqTimeStamp.u8Year > 19) {	/* RTC时间有效 */
			pAcqItem->fD[0] = fD0;
			pAcqItem->fD[1] = fD1;
			pAcqItem->fD[2] = fD2;
			pAcqItem->fD[3] = fD3;
			pAcqItem->fD[4] = fD4;

			pAcqItem->uAcqId = uAcqId;
			pAcqItem->u32ItemNo = g_AcqCtr.u32AcqNo_ForAdd++;
			g_AcqCtr.uBufPt_ForAdd++;

			extern const ACQ_SUMM_MAP cnst_Acq_Summ_Map_Table[];
			int8 i;
			for(i = 0; cnst_Acq_Summ_Map_Table[i].uAcqId; i++) {
				if(ChkAcqIDWithReadAcqData(uAcqId, cnst_Acq_Summ_Map_Table[i].uAcqId)) {
					g_AcqCtr.u32AcqSummNeedProc |= (1<<i);
					break;
				}
			}
		}
	}

    g_NvMemAcsCtr.bNeedSaveAcq = TRUE;
	Semaphore_post(g_DataAccessBlock.Sem_Op);	//Semaphore_post(SEM_NvMemAcs);	/* 需要启动AcqSumm计算 */
}

/*==========================================================================
| Description	: 把Acq的二进制编码转化为文本形式
mcgs最长编码 60byte, 其中时间+id部分为20byte,数值部分不得超过40byte--最长可能会达到74个，即必须妥善编码，使得文本小于60byte
file = mcgs +28byte(含回车换行)
net = mcgs + 49byte(完整的json字符串)
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-07-16
\=========================================================================*/
BOOL ConvertAcqCode2Text(DATA_USER DataUser, uint8** ppBuf, uint8* pBufEnd, ACQ_WRITE_ITEM* pAcqItem)
{
	uint8* pBufStart = *ppBuf;
	uint8* pBuf = pBufStart;
	int16 i;

	if(DataUser == DATA_USER_NET) {
		if(pBuf + 123 > pBufEnd) {	/* Buf长度检查 */
			return FALSE;
		}
		
		/* ItemNo部分: C1~C6 */
		*pBuf++ = '{';
		PrintU32DatToJson(&pBuf, "no", pAcqItem->u32ItemNo, 0);
		PrintStringNoOvChk(&pBuf, "\"time\":\"");

		/* 时间部分: */
		*(pBuf + 18) = ',';
		*(pBuf + 17) = '"';
		*(pBuf + 16) = '0';
		*(pBuf + 15) = '0';
		*(pBuf + 14) = ':';
	} else if(DataUser == DATA_USER_FILE) {
		if(pBuf + 102 > pBufEnd) {	/* Buf长度检查 */
			return FALSE;
		}
		
		PrintU32WithLen(&pBuf, pAcqItem->u32ItemNo, 6); 	/* ItemNo部分:形成6位格式 */
		*pBuf++ = ' ';
		
	} else if(DataUser == DATA_USER_MCGS) {
		if(pBuf + MCGS_LEN_ITEM_CHAR > pBufEnd) {		/* Buf长度检查 */
			return FALSE;
		} else {
			pBufEnd = pBufStart + MCGS_LEN_ITEM_CHAR;	/* 对于屏幕，需要把数据限制在当前行内 */
		}
	} else if(DataUser == DATA_USER_VIEWTECH) {
		if(pBuf + VT_LEN_ITEM_BYTE > pBufEnd) {		/* Buf长度检查 */
			return FALSE;
		} else {
			pBufEnd = pBufStart + VT_LEN_ITEM_CHAR;	/* 对于屏幕，需要把数据限制在当前行内 */
		}
		
		/* 时间部分: */
	}
	*(pBuf + 13) = pAcqItem->AcqTimeStamp.u8Minute%10 + '0';
	*(pBuf + 12) = pAcqItem->AcqTimeStamp.u8Minute/10 + '0';
	*(pBuf + 11) = ':';
	*(pBuf + 10) = pAcqItem->AcqTimeStamp.u8Hour%10 + '0';
	*(pBuf + 9) = pAcqItem->AcqTimeStamp.u8Hour/10 + '0';
	*(pBuf + 8) = ' ';
	*(pBuf + 7) = pAcqItem->AcqTimeStamp.u8Day%10 + '0';
	*(pBuf + 6) = pAcqItem->AcqTimeStamp.u8Day/10 + '0';
	if(DataUser == DATA_USER_NET) {
		*(pBuf + 5) = pAcqItem->AcqTimeStamp.u8Month%10 + '0';
		*(pBuf + 4) = pAcqItem->AcqTimeStamp.u8Month/10 + '0';
		*(pBuf + 3) = pAcqItem->AcqTimeStamp.u8Year%10 + '0';
		*(pBuf + 2) = pAcqItem->AcqTimeStamp.u8Year/10 + '0';
		*(pBuf + 1) = '0';
		*(pBuf + 0) = '2';
		pBuf += 19;
		
		/* AcqID部分 */
		PrintH16DatToJson(&pBuf, "acq_id", pAcqItem->uAcqId);
		/* 之上的Buf溢出检测在入口处做的，已经考虑了最坏的情况 */
		
		/* 数据部分: */
		PrintStringNoOvChk(&pBuf, "\"data\":[");
		for(i = 0; i < ACQ_ITEM_DAT_NUM; i++) {
			*pBuf++ = '"';
			if(PrintF32(&pBuf, pBufEnd, pAcqItem->fD[i], -4) && (pBuf + 9 <= pBufEnd)) {	/* 后续json结尾需要5个字符，额外多4个字符 */
				*pBuf++ = '"';
				*pBuf++ = ',';
			} else {
				return FALSE;
			}
		}
		pBuf--;
		*pBuf++ = ']';
		*pBuf++ = '}';
		*pBuf++ = ',';
	} else {
		*(pBuf + 5) = '/';
		*(pBuf + 4) = pAcqItem->AcqTimeStamp.u8Month%10 + '0';
		*(pBuf + 3) = pAcqItem->AcqTimeStamp.u8Month/10 + '0';
		*(pBuf + 2) = '/';
		*(pBuf + 1) = pAcqItem->AcqTimeStamp.u8Year%10 + '0';
		*(pBuf + 0) = pAcqItem->AcqTimeStamp.u8Year/10 + '0';
		pBuf += 14;
		*pBuf++ = ' ';
		
		/* AcqID部分 */
		PrintH16(&pBuf, pBufEnd, pAcqItem->uAcqId);
		*pBuf++ = ' ';
		/* 之上的Buf溢出检测在入口处做的，已经考虑了最坏的情况 */

		/* 数据部分: */
		for(i = 0; i < ACQ_ITEM_DAT_NUM; i++) {
			if(PrintF32(&pBuf, pBufEnd, pAcqItem->fD[i], -4) && (pBuf + 1 <= pBufEnd)) {
				*pBuf++ = ' ';
			} else {
				if((DataUser == DATA_USER_MCGS) || (DataUser == DATA_USER_VIEWTECH)) {	/* 对于屏幕，接受不完整的一条 */
					break;
				} else {
					return FALSE;
				}
			}
		}
	}

	if(DataUser == DATA_USER_MCGS) {
		/* 如果Buf未满需要添加0作为字符串的结尾, 满了可以不用 */
		if(pBuf < pBufEnd) {
			*pBuf++ = 0;
			pBuf = pBufStart + MCGS_LEN_ITEM_CHAR;
		}
	} else if(DataUser == DATA_USER_VIEWTECH) {
		/* 如果Buf未满需要添加0作为字符串的结尾, 满了可以不用 */
		if(pBuf < pBufEnd) {
			*pBuf++ = 0;
			pBuf = pBufStart + VT_LEN_ITEM_CHAR;
		}
		*((uint16*)pBuf) = pAcqItem->uAcqId;	/* AcqID字段 */
		pBuf += 2;
	} else if(DataUser == DATA_USER_FILE) {
		/* 添加CRC、回车换行 */
		if(pBuf + 10 <= pBufEnd) {	/* 后续总长10 */
			PrintH32(&pBuf, pBufEnd, CalCRC32ForFileByIni(pBufStart, pBuf - pBufStart, 0xFFFFFFFF));
			*pBuf = 0x0D;
			*(pBuf + 1) = 0x0A;
			pBuf += 2;
		} else {
			return FALSE;
		}
	}

	*ppBuf = pBuf;
	return TRUE;
}

CONV_MSG_ACQ_CODE_RES ConvertAcqEECode2Text(DATA_USER DataUser, uint8** ppBuf, uint8* pBufEnd, ACQ_EE_ITEM_VAR* pAcqEE)
{
	ACQ_WRITE_ITEM AcqItem;
	uint32 u32TimeOfAcqEEItem = pAcqEE->u32AcqTimeStamp;
	int16 i;

	/* 输入检查 */
	if(!(isfinite(pAcqEE->fD[0]) && isfinite(pAcqEE->fD[1]) && isfinite(pAcqEE->fD[2]) 
		&& isfinite(pAcqEE->fD[3]) && isfinite(pAcqEE->fD[4])))
	{
		return CONV_MSG_ACQ_CODE_ERR_DATA;
	}

	/* 把ACQ_EE_ITEM_VAR转化成ACQ_ITEM_VAR */
	AcqItem.AcqTimeStamp.u8Year = u32TimeOfAcqEEItem/0x1000000UL;
	AcqItem.AcqTimeStamp.u8Month = (u32TimeOfAcqEEItem/0x10000UL)%0x100;
	AcqItem.AcqTimeStamp.u8Day = (u32TimeOfAcqEEItem/0x100UL)%0x100;
	AcqItem.AcqTimeStamp.u8Hour = u32TimeOfAcqEEItem%0x100UL;
	AcqItem.AcqTimeStamp.u8Minute = 0;
	AcqItem.AcqTimeStamp.u8Second = 0;
	AcqItem.AcqTimeStamp.uMilliSec = 0;
	for(i = ACQ_ITEM_DAT_NUM-1; i >= 0; i--) {
		AcqItem.fD[i] = pAcqEE->fD[i];
	}
	AcqItem.uEEPromNo = pAcqEE->uEEPromNo;
	AcqItem.uAcqId = pAcqEE->uAcqId;
	AcqItem.u32ItemNo = pAcqEE->u32ItemNo;

	if(ConvertAcqCode2Text(DataUser, ppBuf, pBufEnd, &AcqItem)) {
		return CONV_MSG_ACQ_CODE_SUC;
	} else {
		return CONV_MSG_ACQ_CODE_ERR_FULL;
	}
}

/*==========================================================================
| Description	: 把AcqBuf里面的二进制数值存储进文件
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-7-16
\=========================================================================*/
void SaveAcqToNvMem(void)
{
	ACQ_WRITE_ITEM* pAcqItem;
	int16 i;

#if SUPPORT_FAT_FS
	/* 把acq存储进文件中 */
	uint8* pBuf = g_NvMemAcsCtr.u8FileCont;
	BOOL bQuitFileSave = FALSE;	/* 继续文件保存 */
	uint32 u32FileBufFirstItemNo = 0;
	uint32 u32AcqNo_FileSaved = g_AcqCtr.u32AcqNo_FileSaved;
	do {
		if(g_AcqCtr.uBufPt_ForFileSave >= ACQ_WRITE_BUF_LEN) {/* 指针范围检查，必须放在这个位置，和AddAcqItem()保持一致 */
			g_AcqCtr.uBufPt_ForFileSave = 0;
		}
		
		/* 搜索是否有新的可存储数据，如果有则解码存储 */
		BOOL bPrintBreak = FALSE;       /* 新的消息，或者消息文件超了，需要形成新的文件 */
		pAcqItem = &g_AcqCtr.AcqWriteBuf[g_AcqCtr.uBufPt_ForFileSave];
        if(u32FileBufFirstItemNo && (u32FileBufFirstItemNo/100000 != pAcqItem->u32ItemNo/100000)) {	/* 每十万条一个文件 */
            bPrintBreak = TRUE;
		} else if((pAcqItem->u32ItemNo > g_AcqCtr.u32AcqNo_FileSaved)
			&& ConvertAcqCode2Text(DATA_USER_FILE, &pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], pAcqItem))
		{
			u32AcqNo_FileSaved = pAcqItem->u32ItemNo;
			g_AcqCtr.uBufPt_ForFileSave++;
			bQuitFileSave = (g_AcqCtr.uBufPt_ForFileSave%ACQ_WRITE_BUF_LEN == g_AcqCtr.uBufPt_ForAdd%ACQ_WRITE_BUF_LEN);/* 是否完成搜索了 */
            /* 仅记录第一次成功打印的ItemNo，要不然存储函数是后面Msg的ItemNo，造成文件名bug(应该是第一条，但实际不是) */
			if(u32FileBufFirstItemNo == 0) {
    			u32FileBufFirstItemNo = pAcqItem->u32ItemNo;
    		}
		} else {
			bQuitFileSave = TRUE;
		}

		/* 写入文件 */
		if(bPrintBreak
		    || (bQuitFileSave && (pBuf != g_NvMemAcsCtr.u8FileCont))
			|| (pBuf - g_NvMemAcsCtr.u8FileCont + 100 > FILE_CONT_BUF_LEN))	/* 单条AcqItem不超过100byte:最少47byte */
		{
            /* 保存进/acq2/sn.txt */
            /* 没有保存进/acq/年/月/日.txt: acq数据丢了影响不大，正好用来测试一下单文件存储是否靠谱 */
            if(SaveMsgOrAcq_V2(DATA_PAGE_ACQ, pBuf, u32FileBufFirstItemNo)) {
                /* 为下一次打印初始化 */
                g_AcqCtr.u32AcqNo_FileSaved = u32AcqNo_FileSaved;
                pBuf = g_NvMemAcsCtr.u8FileCont;
                u32FileBufFirstItemNo = 0;
            } else {
                bQuitFileSave = TRUE;
            }
		}
	} while(!bQuitFileSave);
#endif

	/* 把消息存储进eeprom中 */
#if CPU_0ST_1TI
    ACQ_EE_ITEM_VAR AcqEEItem;
	do {
		if(g_AcqCtr.uBufPt_ForEEPromSave >= ACQ_WRITE_BUF_LEN) {/* 指针范围检查，必须放在这个位置，和AddAcqItem()保持一致 */
			g_AcqCtr.uBufPt_ForEEPromSave = 0;
		}
		/* 搜索是否有新的可存储数据，如果有则存储 */
		pAcqItem = &g_AcqCtr.AcqWriteBuf[g_AcqCtr.uBufPt_ForEEPromSave];
		if(pAcqItem->u32ItemNo > g_AcqCtr.u32AcqNo_EESaved) {
			AcqEEItem.u32AcqTimeStamp = (uint32)pAcqItem->AcqTimeStamp.u8Year*0x1000000UL
										+ (uint32)pAcqItem->AcqTimeStamp.u8Month*0x10000UL
										+ (uint32)pAcqItem->AcqTimeStamp.u8Day*0x100UL
										+ (uint32)pAcqItem->AcqTimeStamp.u8Hour*0x1UL;
			AcqEEItem.uEEPromNo = g_AcqCtr.uEEPromNo_ForSave++;
			for(i = ACQ_ITEM_DAT_NUM-1; i >= 0; i--) {
				AcqEEItem.fD[i] = pAcqItem->fD[i];
			}
			AcqEEItem.uAcqId = pAcqItem->uAcqId;
			AcqEEItem.u32ItemNo = pAcqItem->u32ItemNo;
			EEPROMProgram((uint32_t*)(&AcqEEItem), EEPROM_ADDR_ACQ + (AcqEEItem.uEEPromNo%EEPROM_ACQ_ITEM_NUM)*sizeof(ACQ_EE_ITEM_VAR), sizeof(ACQ_EE_ITEM_VAR));
			g_AcqCtr.u32AcqNo_EESaved = pAcqItem->u32ItemNo;
		}
		
		/* 修正指针 */
		g_AcqCtr.uBufPt_ForEEPromSave++;
	} while(g_AcqCtr.uBufPt_ForEEPromSave%ACQ_WRITE_BUF_LEN != g_AcqCtr.uBufPt_ForAdd%ACQ_WRITE_BUF_LEN);
#endif

	/* 清空已经存储的AcqBuf */
	/* 看Buf容量，由于file可能存储失败，因此当u32AcqNo_FileSaved落后太多的时候，抛弃这个部分数据 */
	uint32 u32MinAcqNo;
	if(g_AcqCtr.u32AcqNo_ForAdd - g_AcqCtr.u32AcqNo_FileSaved > ACQ_WRITE_BUF_LEN*3/4) {
		u32MinAcqNo = g_AcqCtr.u32AcqNo_ForAdd - ACQ_WRITE_BUF_LEN*2/3;
	} else {
		u32MinAcqNo = g_AcqCtr.u32AcqNo_FileSaved;
	}
	g_AcqCtr.u32AcqNo_ram = u32MinAcqNo;
	pAcqItem = g_AcqCtr.AcqWriteBuf;
	Swi_disable();				/* 由于不是从最小的ItemNo开始清理，因此需要关任务调度 */
	for(i = ACQ_WRITE_BUF_LEN; i > 0; i--) {
		if(pAcqItem->u32ItemNo <= u32MinAcqNo) {
			pAcqItem->u32ItemNo = 0;		/* 必须先破坏数据完整性标识 */
			pAcqItem->uAcqId = ACQID_NULL;
		}
		pAcqItem++;
	}
	Swi_enable();
}

/*==========================================================================
| Description	: 从依据Acq-Summ映射关系表(cnst_Acq_Summ_Map_Table)，获取数据并进行处理
		触发处理事件有两个: 添加新的Acq数据(造成处理指针与添加指针不一致), 修改配置
		需要依据产品编制Acq-Summ映射关系表(cnst_Acq_Summ_Map_Table)
		入口函数是RunAcqSummTask()，在DataAccessTask中运行
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-7-16
\=========================================================================*/
void ProcAcqSummByMapItem(const ACQ_SUMM_MAP* pCnstAcqSummProc);
void RunAcqSummTask(void)
{
	extern const ACQ_SUMM_MAP cnst_Acq_Summ_Map_Table[];
	if(g_AcqCtr.u32AcqSummNeedProc) {
		int8 i;
		for(i = 0; cnst_Acq_Summ_Map_Table[i].uAcqId; i++) {
			if(g_AcqCtr.u32AcqSummNeedProc & (1UL<<i)) {
				Swi_disable();				/* g_AcqCtr.u32AcqSummNeedProc可能 */
				g_AcqCtr.u32AcqSummNeedProc &= ~(1UL<<i);
				Swi_enable();
				ProcAcqSummByMapItem(&cnst_Acq_Summ_Map_Table[i]);
			}
		}
	}
}

/* 依据Acq-Summ映射项，由AcqId、数据计算获得AcqSumm */
int32 GetDataForAcq(uint16 uSeekAcqId, int32 i32ItemNum_InNeed_OutRemain);
void ProcAcqSummByMapItem(const ACQ_SUMM_MAP* pAcqSummMapItem)
{
	int32 i32ItemNum_NeedRead = 0;
	int16 i;

	/* 获得计算长度: 由最长的那个计算需求获得 */
	for(i = 0; i < ACQ_ITEM_DAT_NUM; i++) {
		if(pAcqSummMapItem->pAcqSumm[i] == NULL) {
		} else if(pAcqSummMapItem->pAcqSumm[i]->i32SummaryPlanNum == 0) {
			i32ItemNum_NeedRead = ACQ_READ_BUF_LEN;
		} else if(i32ItemNum_NeedRead < labs(pAcqSummMapItem->pAcqSumm[i]->i32SummaryPlanNum)) {
			i32ItemNum_NeedRead = labs(pAcqSummMapItem->pAcqSumm[i]->i32SummaryPlanNum);
		}
	}
	if(i32ItemNum_NeedRead > ACQ_READ_BUF_LEN) {
		i32ItemNum_NeedRead = ACQ_READ_BUF_LEN;
	}
	
	/* 需要处理的数据组数非零，即说明需要进行处理 */
	if(i32ItemNum_NeedRead && (pAcqSummMapItem->AcqSummProc != NULL)) {
		i32ItemNum_NeedRead = 0 - i32ItemNum_NeedRead;	/* 超前读，需要修改为负 */
		uint16 uItemNum_SucRead = GetDataForAcq(pAcqSummMapItem->uAcqId, i32ItemNum_NeedRead) - i32ItemNum_NeedRead;	/* 读取数据 */
		pAcqSummMapItem->AcqSummProc(g_AcqCtr.AcqReadBuf, uItemNum_SucRead);	/* 处理数据 */
	}
}

/*==========================================================================
| Description	: 从ram，NvMem(文件系统,eeprom)获得指定AcqId以及长度的Acq数据
| In/Out/G var	: /返回：剩余长度/
| Author		: Wang Renfei			Date	: 2019-1-4
\=========================================================================*/
int32 GetDataForAcq(uint16 uSeekAcqId, int32 i32ItemNum_InNeed_OutRemain)
{
	uint32 u32ItemNo_InStart_OutEnd = 0xFFFFFFFFUL;
	ACQ_READ_ITEM* pAcqDataRead = g_AcqCtr.AcqReadBuf;
	int32 i, j;

	/* 输入检查 */
	if((i32ItemNum_InNeed_OutRemain >= 0) || (i32ItemNum_InNeed_OutRemain < (int16)(0 - ACQ_READ_BUF_LEN))
		|| (u32ItemNo_InStart_OutEnd <= 1))
	{
		return 0;
	}

	/* 先从ram中读取数据 */
	int16 iAcqPt = g_AcqCtr.uBufPt_ForAdd;
	for(i = ACQ_WRITE_BUF_LEN; (i > 0) && i32ItemNum_InNeed_OutRemain; i--) {
		iAcqPt--;
		if(iAcqPt < 0) {
			iAcqPt = ACQ_WRITE_BUF_LEN-1;
		}
		ACQ_WRITE_ITEM* pAcqItem = &g_AcqCtr.AcqWriteBuf[iAcqPt];
		if(!pAcqItem->u32ItemNo) {		/* 已经找完了 */
			break;
		} else if((pAcqItem->u32ItemNo < u32ItemNo_InStart_OutEnd) && ChkAcqIDWithReadAcqData(pAcqItem->uAcqId, uSeekAcqId)) {
			pAcqDataRead->uAcqId = pAcqItem->uAcqId;
			pAcqDataRead->uDaysFromOrigin = CalDaysFromOrigin(pAcqItem->AcqTimeStamp.u8Year,
																pAcqItem->AcqTimeStamp.u8Month,
																pAcqItem->AcqTimeStamp.u8Day);
			for(j = ACQ_ITEM_DAT_NUM-1; j >= 0; j--) {
				pAcqDataRead->fD[j] = pAcqItem->fD[j];
			}
			u32ItemNo_InStart_OutEnd = pAcqItem->u32ItemNo;
			i32ItemNum_InNeed_OutRemain++;
			pAcqDataRead++;
		}
	}

	/* 读取NvMem前的检查 */
	if((i32ItemNum_InNeed_OutRemain == 0) || (u32ItemNo_InStart_OutEnd <= 1) 
	    || (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0)) 
	{
	    return i32ItemNum_InNeed_OutRemain;
	}
#if CPU_0ST_1TI	//ST的这段代码还不完善，先这样以保持兼容
    int32 i32ItemNum_BeforeFileRead = i32ItemNum_InNeed_OutRemain;  /* 用于检查文件读取是否成功 */
#endif
#if SUPPORT_FAT_FS
	if(f_stat(DEVICE_PATH_String, NULL) == FR_OK) {				/* 检查TF卡: TF卡工作正常 */
    /* 支持新旧版本acq读取
       先读取 DEVICE_PATH_String/acq2 如果没有足够的数据，再读取 DEVICE_PATH_String/acq
       acq2是单级目录(/序号.txt)；acq是三级目录(/年/月/日.txt)
       为了代码兼容，acq2 模拟 acq最后一级目录，即i8SeekFilePathDepth=0的情形     */
		FIL File;
		DIR dir;
		FILINFO fno;
		uint8* pFilePathRecBuf[MSG_ACQ_FILE_PATH_DEPTH];		/* 文件路径指针，用于分断年文件夹(2)、月文件夹(1)、日文件(0) */
		uint8* pText;
		uint8* pLineStart;
		uint8* pLineEnd;										/* 文件中的数据指针，当前文本、行开始、行结尾 */
		uint32 u32LastOpenDirOrFileItemNo[MSG_ACQ_FILE_PATH_DEPTH];	/* 上一轮搜索的ItemNo */
		uint32 u32CurDirOrFileItemNo, u32CurSeekCloseItemNo;	/* 当前次文件搜索文件所附带的ItemNo, 和搜索目标最接近的文件ItemNo */
		uint32 u32DataFilePt;
		uint32 u32ItemNo_ForSelectFileOrDir;					/* 需要读取的ItemNo，用于搜索文件或者目录,需要调整到"含" */
		int16 iSeekFilePathDepth;								/* 正在搜索的路径深度 */
		BOOL bFindFile, bCurDirSeekHaveResult;
		DATA_ACS_OPR DataAcsOpr = DATA_ACS_READ_ACQ;
        MSG_ACQ_ACS_VER eReadItemVer = MSG_ACQ_ACS_V2;
		
		/* 初始化工作:定位相应的入门路径 */
		uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
		PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
		PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Acq2");
		pFilePathRecBuf[MSG_ACQ_FILE_PATH_DEPTH - 1] = pFilePath-1; /* 指向/Acq 以便 MSG_ACQ_ACS_V1 时候使用 */
		pFilePathRecBuf[0] = pFilePath;                             /* 指向/Acq2 */
		for(i = MSG_ACQ_FILE_PATH_DEPTH - 1; i >= 0; i--) {			/* 清除下一级目录 */
			u32LastOpenDirOrFileItemNo[i] = 0;
		}
		iSeekFilePathDepth = 0;
		u32ItemNo_ForSelectFileOrDir = u32ItemNo_InStart_OutEnd - 1;/* 搜索文件或者目录，需要的是"含"该ItemNo的目录或者文件 */

		while(i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr)) {
		/* 搜索目录，定位当前所想读取的文件 */
		/* 搜索算法: 如果当前目录没有搜索到目标文件/目录，则退回上一级目录进行搜索；如果找到则往下一级目录中搜索
			为了防止ItemNo不连续引起的读取失败，进行如下搜索, 获取的内容其ItemNo一定要小于等于要读取的ItemNo:
				1. 首次搜索该目录(通过判断u32LastOpenDirOrFileItemNo[] 为零)，
					则搜索小于等于要读取的ItemNo中最大的，搜索结果存储于u32LastOpenDirOrFileItemNo[]
				2. 二次搜索该目录(通过判断u32LastOpenDirOrFileItemNo[] 非零)，
					则搜索比上次小的结果中最大的
			如果搜索失败，则退回上一级目录进行搜索
			目录级别: 0:最后一级文件, 1:月文件夹, 2:年文件夹		*/
			bFindFile = FALSE;
			do {
				/* 根据当前的深度进行搜索: 搜索相应的年、月目录以及日期文件 */
				pFilePath = pFilePathRecBuf[iSeekFilePathDepth];
				*pFilePath = 0;
				if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
					DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
					break;
				} else if(f_opendir(&dir, (const TCHAR*)g_NvMemAcsCtr.u8FilePath) != FR_OK) {
					DataAcsOpr = DATA_ACS_RES_OPEN_FILE_FAIL;
					break;
				} else {
					*pFilePath++ = '/';
					u32CurSeekCloseItemNo = 0;
					bCurDirSeekHaveResult = FALSE;
					while((g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt <= 0) && (f_readdir(&dir, &fno) == FR_OK) && fno.fname[0]) {
						if((iSeekFilePathDepth != 0) ^ ((fno.fattrib & AM_DIR) == 0)) {		/* 最后一级必须是文件，否则必须是目录 */
						    if(eReadItemVer == MSG_ACQ_ACS_V2) {
    							u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname);
						    } else if(eReadItemVer == MSG_ACQ_ACS_V1) {
    							u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname + 2);
    						}
                            /* 之前软件bug，会产生000000.txt，实际上里面存的第一条序号是1 */
                            if(u32CurDirOrFileItemNo == 0) {
                                u32CurDirOrFileItemNo = 1;
                            }
							
							if(((u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] == 0) && (u32CurSeekCloseItemNo < u32CurDirOrFileItemNo)
									&& (u32CurDirOrFileItemNo <= u32ItemNo_ForSelectFileOrDir))
								|| (u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] && (u32CurSeekCloseItemNo < u32CurDirOrFileItemNo)
									&& (u32CurDirOrFileItemNo < u32LastOpenDirOrFileItemNo[iSeekFilePathDepth])))
							{
								for(i = 0; (i < FILE_NAME_BYTE_LEN) && fno.fname[i]; i++) {
									pFilePath[i] = fno.fname[i];
								}
								u32CurSeekCloseItemNo = u32CurDirOrFileItemNo;
								bCurDirSeekHaveResult = TRUE;
							}
						}
					}

					if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
						DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
						break;
					/* 如果该层次目录进行了成功的搜索 */
					} else if(bCurDirSeekHaveResult) {					/* 搜索到目录/文件 */
						u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] = u32CurSeekCloseItemNo;
						pFilePath += i;
						if(iSeekFilePathDepth) {					/* 进入下一级目录 */
							iSeekFilePathDepth--;
							pFilePathRecBuf[iSeekFilePathDepth] = pFilePath;
						} else {
							bFindFile = TRUE;
						}
                    } else if(eReadItemVer == MSG_ACQ_ACS_V2) {    /* acq2目录已经读完，切换到acq */
                        iSeekFilePathDepth = MSG_ACQ_FILE_PATH_DEPTH - 1;
                        u32LastOpenDirOrFileItemNo[0] = 0;  /* 访问\acq2的时候，用了这个 */
                        eReadItemVer = MSG_ACQ_ACS_V1;
					} else if(iSeekFilePathDepth < MSG_ACQ_FILE_PATH_DEPTH - 1) {	/* 则停止本次搜索，以回到外面打大循环，搜索上一级目录 */
						for(i = iSeekFilePathDepth; i >= 0; i--) {		/* 清除下一级目录 */
							u32LastOpenDirOrFileItemNo[i] = 0;
						}
						iSeekFilePathDepth++;
					} else {			/* 不管怎么样都搜不到 */
						DataAcsOpr = DATA_ACS_RES_EMPTY;
						break;
					}
				}
			} while(!bFindFile);
			if(DATA_ACS_RES(DataAcsOpr)) {
				break;
			}

			/* 读取文件:获取数据 */
			*pFilePath = 0;
			if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
				DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
				break;
			} else if(f_open(&File, (const TCHAR*)g_NvMemAcsCtr.u8FilePath, FA_READ) != FR_OK) {	/* 文件打开失败 */
				break;												/* 结束搜索，转到本函数最后处理 */
			} else {
				u32DataFilePt = f_size(&File);						/* 初始化文件访问指针 */
				pLineStart = g_NvMemAcsCtr.u8FileCont;
				do {
					/* 读取文件 */
					UINT u32BufByteNum;
					if(u32DataFilePt + (pLineStart - g_NvMemAcsCtr.u8FileCont) > FILE_CONT_BUF_LEN) {
						u32DataFilePt -= FILE_CONT_BUF_LEN - (pLineStart - g_NvMemAcsCtr.u8FileCont);
						u32BufByteNum = FILE_CONT_BUF_LEN;
					} else {
						u32BufByteNum = u32DataFilePt + (pLineStart - g_NvMemAcsCtr.u8FileCont);
						u32DataFilePt = 0;
					}
					if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
						DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
						break;
					}
					f_lseek(&File, u32DataFilePt);
					f_read(&File, g_NvMemAcsCtr.u8FileCont, u32BufByteNum, &u32BufByteNum);
					if(u32BufByteNum >= 2) {
						pText = &g_NvMemAcsCtr.u8FileCont[u32BufByteNum - 2];
						pLineEnd = &g_NvMemAcsCtr.u8FileCont[u32BufByteNum];
					} else {
						break;
					}

					/* 在本文件中进行搜索 */
					for( ; i32ItemNum_InNeed_OutRemain && (pText >= g_NvMemAcsCtr.u8FileCont); pText--) {
						/* 行结尾 或者 文件开头 */
						if((*pText == 0x0A) || ((u32DataFilePt == 0) && (pText == g_NvMemAcsCtr.u8FileCont))) {
							pLineStart = pText;
							if(*pText == 0x0A) {
								pLineStart++;
							}

							/* 匹配搜索: ItemNo, AcqID, 并进行CRC校验 */
							if(pLineStart + 10 < pLineEnd) {				/* 滤除空行，或者仅几个字的行 */
								uint32 u32CurLineItemNo = ReadU32(&pText, pLineEnd);
								uint16 uDaysFromOrigin = CalDaysFromOrigin(ReadU32(&pText, pLineEnd),		/* 年 */
																			ReadU32(&pText, pLineEnd),	/* 月 */
																			ReadU32(&pText, pLineEnd));	/* 日 */
								pText += 6;	/* 跳过 时:分 */
								uint16 uReadAcqId = ReadH32(&pText, pLineEnd);
								if((u32CurLineItemNo < u32ItemNo_InStart_OutEnd)		/* 有效的行 */
									&& ChkAcqIDWithReadAcqData(uReadAcqId, uSeekAcqId)
									&& (CheckTextWithCrc32ForMsgAndAcq(pLineStart, pLineEnd)))
								{
									for(i = 0; (i < ACQ_ITEM_DAT_NUM) && GetF32(&pText, pLineEnd, &pAcqDataRead->fD[i]); i++) {
									}
									if(i == ACQ_ITEM_DAT_NUM) {
										pAcqDataRead->uAcqId = uReadAcqId;
										pAcqDataRead->uDaysFromOrigin = uDaysFromOrigin;
										i32ItemNum_InNeed_OutRemain++;
										u32ItemNo_InStart_OutEnd = u32CurLineItemNo;
										pAcqDataRead++;
									}
								}
							}
							pLineEnd = pLineStart;
							pText = pLineStart - 2;		/* 跳过回车换行符 */
						}
					}
				} while(i32ItemNum_InNeed_OutRemain && u32DataFilePt);
			}
		}
	}
#endif

#if CPU_0ST_1TI /* 如果文件读出失败，则从EEProm中读取 */
	if((i32ItemNum_BeforeFileRead == i32ItemNum_InNeed_OutRemain)
	    && (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt <= 0))
    {
		/* 从最近存储的位置开始查找 */
		uint32 u32LastItemNo = 0xFFFFFFFFUL;
		uint16 uCurReadEEPromItemPt = g_AcqCtr.uEEPromNo_ForSave%EEPROM_ACQ_ITEM_NUM;		/* 利用最大EEPromNo和ITEM_NUM整数倍的关系 */;
		uint16 uRemainItemInEEProm = EEPROM_ACQ_ITEM_NUM;	/* 初始化总的Item数量 */
		uint16 uCurReadItemNum;
		while(uRemainItemInEEProm && i32ItemNum_InNeed_OutRemain) {
			/* 计算读取EEProm位置 */
			if(uCurReadEEPromItemPt == 0) {
				uCurReadEEPromItemPt = EEPROM_ACQ_ITEM_NUM;
			}
			if(uCurReadEEPromItemPt > FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR)) {			/* 保证读出整数条 */
				uCurReadItemNum = FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR);
			} else {
				uCurReadItemNum = uCurReadEEPromItemPt;
			}
			if(uCurReadItemNum > uRemainItemInEEProm) {
				uCurReadItemNum = uRemainItemInEEProm;
			}
			uCurReadEEPromItemPt -= uCurReadItemNum;
			EEPROMRead((uint32_t*)g_NvMemAcsCtr.u8FileCont, EEPROM_ADDR_ACQ + uCurReadEEPromItemPt*sizeof(ACQ_EE_ITEM_VAR),
						uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR));
			uRemainItemInEEProm -= uCurReadItemNum;

			/* 搜索数据 */
			ACQ_EE_ITEM_VAR* pAcqEE = (ACQ_EE_ITEM_VAR*)(&g_NvMemAcsCtr.u8FileCont[uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR)]);
			for(i = uCurReadItemNum; (i > 0) && i32ItemNum_InNeed_OutRemain; i--) {
				pAcqEE--;
				/* ItemNo本来应该是递减的，但发生逆转，说明数据可能不是这台机组的，需要结束读取 */
				if((pAcqEE->u32ItemNo >= u32LastItemNo)
					|| (pAcqEE->u32ItemNo == 0))			/* 无效数据 */
				{
					uRemainItemInEEProm = 0;
					break;
				/* 是所要读取的数据,进行解码 */
				} else if((pAcqEE->u32ItemNo < u32ItemNo_InStart_OutEnd) && ChkAcqIDWithReadAcqData(pAcqEE->uAcqId, uSeekAcqId)) {
					if(isfinite(pAcqEE->fD[0]) && isfinite(pAcqEE->fD[1]) && isfinite(pAcqEE->fD[2])
						&& isfinite(pAcqEE->fD[3]) && isfinite(pAcqEE->fD[4])) 
					{
						pAcqDataRead->uAcqId = pAcqEE->uAcqId;
						pAcqDataRead->uDaysFromOrigin = CalDaysFromOrigin(pAcqEE->u32AcqTimeStamp/0x1000000UL,		/* 年 */
																			(pAcqEE->u32AcqTimeStamp/0x10000UL)%0x100,	/* 月 */
																			(pAcqEE->u32AcqTimeStamp/0x100UL)%0x100);	/* 日 */
						for(j = ACQ_ITEM_DAT_NUM - 1; j >= 0; j--) {
							pAcqDataRead->fD[j] = pAcqEE->fD[j];
						}
						pAcqDataRead++;
						i32ItemNum_InNeed_OutRemain++;
					}
					u32ItemNo_InStart_OutEnd = pAcqEE->u32ItemNo;
				}
				u32LastItemNo = pAcqEE->u32ItemNo;
			}
		}
	}
#endif

	/* 成功读取 */
	return i32ItemNum_InNeed_OutRemain;
}

/* 检查读取的AcqID是否是读取任务所期望的 */
BOOL ChkAcqIDWithReadAcqData(uint16 uReadAcqId, uint16 uSeekAcqId)
{
	if((uSeekAcqId & 0x0FFF) == 0) {
		if(uSeekAcqId == (uReadAcqId & 0xF000)) {
			return TRUE;
		} else {
			return FALSE;
		}
	} else if(uSeekAcqId == uReadAcqId) {	
		return TRUE;
	} else {
		return FALSE;
	}
}
#endif

void InformAcqSummConfChange(void* pAcqSumm)
{
#if ACQ_ITEM_DAT_NUM
	extern const ACQ_SUMM_MAP cnst_Acq_Summ_Map_Table[];
	/* 有修改了AcqSumm配置项，需要重新计算 */
	if(pAcqSumm != NULL) {
		/* 在处理表中搜索存在 pAcqSumm 的处理项 */
		int8 i, j;
		for(i = 0; cnst_Acq_Summ_Map_Table[i].uAcqId; i++) {
			for(j = ACQ_ITEM_DAT_NUM-1; (j >= 0) && (cnst_Acq_Summ_Map_Table[i].pAcqSumm[j] != pAcqSumm); j--) {
			}
			if(j >= 0) {	/* 找到相应的pCnstAcqSummProc处理项 */
				g_AcqCtr.u32AcqSummNeedProc |= (1<<i);
				Semaphore_post(SEM_NvMemAcs);
				break;
			}
		}
	}
#endif
}

/*==========================================================================
| 以下为UI和通讯部分
\=========================================================================*/
/*==========================================================================
| Description	: 文本访问接口，主要用于以太网通讯/UI接口, 阻塞实现
	这个功能分两个部分完成: 外部接口注册相应的需求，内部实现完成后给返回
	为了简化内部处理逻辑，避免了ram中二进制格式编码转成文本的复杂，仅读取文件系统中的文本，因此内部处理需要分两步:
	1. 把相应MsgBuf/AcqBuf中二进制编码存储进文件系统；
	2. 从文件系统中读取相应的数据
| In/Out/G var	: 
	DataUser	: 数据用户, DATA_USER_NET数据长度不固定，一个Item结束立刻放置下一个Item;
							DATA_USER_MCGS，DATA_USER_VIEWTECH则数据长度固定为N，进行填充，扔掉ItemNo与CRC部分;
								不满会以空字符串按照期望读的条数进行填充(不超过pBufEnd)，但不会修改返回的Itemo, ItemNum
	DataPage					: 目前仅支持DATA_PAGE_MSG, DATA_PAGE_ACQ
	pU32ItemNo_InStart_OutEnd 	: 开始ItemNo(不含), 返回实际读取的最后一条ItemNo--可能指定的ItemNo读不到数据, 0xFFFFFFFFUL代表读取Topic
	pI32ItemNum_InNeed_OutSuc 	: 读取的Item数量(正:ItemNo增加，即往后读;负:ItemNo减少，即往前读)，返回实际读取的Item数量(带符号，仅指向前、向后成功读取)
	ppBuf						: Buf起点，可以写入，返回访问完成后的指针
	pBufEnd						: Buf终点，不可写入
| Author		: Wang Renfei			Date	: 2016-07-16
\=========================================================================*/
DATA_ACS_OPR GetTextForMsgOrAcq(DATA_USER DataUser, DATA_PAGE DataPage, 
		uint32* pU32ItemNo_InStart_OutEnd, int32* pI32ItemNum_InNeed_OutSuc, uint8** ppBuf, uint8* pBufEnd)
{
	DATA_ACS_OPR DataAcsOpr = (DATA_ACS_OPR)(DATA_ACS_READ_DATA_PAGE - DataPage);
	int32 i32ItemNum_InNeed_OutRemain = *pI32ItemNum_InNeed_OutSuc;
	
	/* 是否需要读NvMem, 且Buf有空 */
	int8 i8ReadTaskCtrNo;
	Swi_disable();
	for(i8ReadTaskCtrNo = MAX_READ_MSG_ACQ_NUM - 1; i8ReadTaskCtrNo >= 0; i8ReadTaskCtrNo--) {
		if(g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr == DATA_ACS_IDLE) {
			g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].u32ItemNo_InStart_OutEnd = *pU32ItemNo_InStart_OutEnd;
			g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].i32ItemNum_InNeed_OutRemain = i32ItemNum_InNeed_OutRemain;
			g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].pBuf = *ppBuf;
			g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].pBufEnd = pBufEnd;
			g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataUser = DataUser;
			g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr = DataAcsOpr;
            Semaphore_post(g_DataAccessBlock.Sem_Op);		//Semaphore_post(SEM_NvMemAcs);
			break;
		}
	}
	Swi_enable();

	if(i8ReadTaskCtrNo >= 0) {
	    /* 等待读取完成 */
	    int8 i8TryCnt = 100;
		for(; (i8TryCnt > 0) && (!DATA_ACS_RES(g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr)); i8TryCnt--) {
			Task_sleep(OS_TICK_KHz*5);
		}

        /* 正常完成读取 */
        if(DATA_ACS_RES(g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr)) {
			DataAcsOpr = g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr;
        /* 如果超时了，也算一种结果吧，等待读取任务退出，能有多少数据就算多少吧 */
		} else {
			g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr = DATA_ACS_RES_TIMEOUT;	/* 放弃读取 */
			Task_sleep(OS_TICK_KHz*100);	/* 等待 ReadTextForMsgOrAcq() 退出，产生部分结果。观察到实际需要的时间7~14ms */
			DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
		}

		/* 数据存储 */
		Swi_disable();
	    /* 超时且未读出，则修改u32ItemNo_InStart_OutEnd指向ram中的第一条数据，免得逻辑卡死 */
		if((g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr == DATA_ACS_RES_TIMEOUT)
		    && (i32ItemNum_InNeed_OutRemain == g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].i32ItemNum_InNeed_OutRemain))
	    {
		    if(0) {
        #if MAX_MSG_TYPE
		    } else if(DataPage == DATA_PAGE_MSG) { 
		        *pU32ItemNo_InStart_OutEnd = g_MsgCtr.u32MsgNo_ram;
		#endif
		#if ACQ_ITEM_DAT_NUM
		    } else {
		        *pU32ItemNo_InStart_OutEnd = g_AcqCtr.u32AcqNo_ram;
		#endif
		    }
		} else {
			*pU32ItemNo_InStart_OutEnd = g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].u32ItemNo_InStart_OutEnd;
		}
        i32ItemNum_InNeed_OutRemain -= g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].i32ItemNum_InNeed_OutRemain;
        *pI32ItemNum_InNeed_OutSuc = i32ItemNum_InNeed_OutRemain;
        *ppBuf = g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].pBuf;
		Swi_enable();
		g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr = DATA_ACS_END;	/* 释放 ReadMsgOrAcqTxt 控制块 */
	}
	
	/* 返回 */
	if(DataAcsOpr == DATA_ACS_RES_FULL) {
		return DATA_ACS_RES_FULL;
	} else if(i32ItemNum_InNeed_OutRemain == 0) {
		return DATA_ACS_RES_SUC;
	} else {
		return DATA_ACS_RES_EMPTY;
	}
}

/* VT屏幕主页面仅读取一行，显示LastAddMsg内容 */
uint16 ReadLastAddMsgForVT(uint8* pBuf, uint8* pBufEnd, BOOL bTimeShort)
{
#if MAX_MSG_TYPE
	PrintMsgCode(DATA_USER_VIEWTECH, &pBuf, pBufEnd, g_MsgCtr.pLastAddMsg, TRUE, bTimeShort);
    return g_MsgCtr.pLastAddMsg->uMsgId;
#else
    PrintString(&pBuf, pBufEnd, "00:00:00 欢迎使用");
    *pBuf = 0;
    return 0;
#endif

}

/*==========================================================================
| Description	: 如果往前读(i32ItemNum_InNeed_OutRemain < 0)，则读取u32ItemNo_InStart_OutEnd(不含)之前的若干行的数据
				  如果往后读(i32ItemNum_InNeed_OutRemain > 0)，则读取u32ItemNo_InStart_OutEnd(不含)之后的若干行的数据
				如果是显示读取，则需要抛弃前面的ItemNo, 后面的CRC与回车换行
				如果是通讯读取，则需要抛弃后面的回车换行，改成0
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-07-18
\=========================================================================*/
#define READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)  ((pReadMsgOrAcqCtr == NULL) || DATA_ACS_MISSION(pReadMsgOrAcqCtr->DataAcsOpr))
BOOL ReadTextFromLineForMsgOrAcq_MustSwiDisable(DATA_USER DataUser, DATA_ACS_OPR DataAcsOpr, uint8** ppBuf, uint8* pBufEnd, uint8* pLineStart, uint8* pLineEnd, BOOL bCrcOK);	/* 把一行的文件数据填进相应的Buf */
void ReadTextForMsgOrAcq(READ_MSG_ACQ_TXT* pReadMsgOrAcqCtr)
{
    /* 这些变量由后面代码初始化 */
	uint8* pBuf;
	uint8* pBufEnd;
	uint32 u32ItemNo_InStart_OutEnd;
	int32 i32ItemNum_InNeed_OutRemain;
	DATA_USER DataUser;
	DATA_ACS_OPR DataAcsOpr;
	int16 i;

    /* 初始化 */
    /* pReadMsgOrAcqCtr == NULL 代表发布消息到数据库 */
    if(pReadMsgOrAcqCtr == NULL) {
    #if (!MAX_MSG_TYPE)
        return;
    #else
        if(g_MsgCtr.u32MsgNo_Cmplt <= g_MqttItemProc.u32MsgNo_HavePubedForDB) {  /* 没有新增的，不必退出 */
            return;
        } else {
            u32ItemNo_InStart_OutEnd = g_MqttItemProc.u32MsgNo_HavePubedForDB;
            i32ItemNum_InNeed_OutRemain = g_MsgCtr.u32MsgNo_Cmplt - u32ItemNo_InStart_OutEnd;
        }
        DataUser = DATA_USER_NET;
        DataAcsOpr = (DATA_ACS_OPR)(DATA_ACS_READ_DATA_PAGE - DATA_PAGE_MSG);
    #endif
    } else {
        u32ItemNo_InStart_OutEnd = pReadMsgOrAcqCtr->u32ItemNo_InStart_OutEnd;
        i32ItemNum_InNeed_OutRemain = pReadMsgOrAcqCtr->i32ItemNum_InNeed_OutRemain;
        DataUser = pReadMsgOrAcqCtr->DataUser;
        DataAcsOpr = pReadMsgOrAcqCtr->DataAcsOpr;
    }
	if((pReadMsgOrAcqCtr == NULL) || (pReadMsgOrAcqCtr->DataUser == DATA_USER_NET)) {
    	/* 网络读取，如果没有空余资源，则返回 */
	    if(g_MqttItemProc.i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage > 0) {
    		return ;
	    } else {
	        g_MqttItemProc.i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage = -1;
    	    pBuf = g_MqttItemProc.u8Buf;
    	    pBufEnd = pBuf + sizeof(g_MqttItemProc.u8Buf);
    	}
	} else {
	    pBuf = pReadMsgOrAcqCtr->pBuf;
	    pBufEnd = pReadMsgOrAcqCtr->pBufEnd;
	}
	
	/* 输入检查 */
	if((i32ItemNum_InNeed_OutRemain == 0)
		|| ((i32ItemNum_InNeed_OutRemain < 0) && (u32ItemNo_InStart_OutEnd <= 1)))
	{
		DataAcsOpr = DATA_ACS_RES_INPUT_ERR;
	}

	/* 头部处理 */
    Swi_disable();      /* 需要防止该任务已经被取消了，以下都是ram中操作，很快 */
    if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
        DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
    } else if((!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) || (!DATA_ACS_MISSION(DataAcsOpr))) {
        DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
	} else if(DataUser == DATA_USER_NET) {		/* 不考虑Buf空间连topic都不够的情况 */
		*pBuf++ = '{';
		/* 复位处理: Topic行 */
		if((pReadMsgOrAcqCtr != NULL) && (u32ItemNo_InStart_OutEnd == 0xFFFFFFFFUL)) {
			if(0) {
		#if MAX_MSG_TYPE
			} else if(DataAcsOpr == DATA_ACS_READ_MSG) {
				u32ItemNo_InStart_OutEnd = g_MsgCtr.u32MsgNo_ForAdd;
				PrintU32DatToJson(&pBuf, "no", u32ItemNo_InStart_OutEnd, 0);
				PrintStringToJson(&pBuf, "title", "机组消息");
		#endif
		#if ACQ_ITEM_DAT_NUM
			} else if(DataAcsOpr == DATA_ACS_READ_ACQ) {
				u32ItemNo_InStart_OutEnd = g_AcqCtr.u32AcqNo_ForAdd;
				PrintU32DatToJson(&pBuf, "no", u32ItemNo_InStart_OutEnd, 0);
				PrintStringToJson(&pBuf, "title", "采集参数");
		#endif
			} else {
				DataAcsOpr = DATA_ACS_RES_INPUT_ERR;
			}
		}
		PrintStringNoOvChk(&pBuf, "\"cont\":[");
		pBufEnd -= 3;		/* 需要为最后的json结尾预留3个byte */
	} else if((DataUser == DATA_USER_MCGS) || (DataUser == DATA_USER_VIEWTECH)) {
		/* 复位处理: Topic行 */
		if(u32ItemNo_InStart_OutEnd == 0xFFFFFFFFUL) {
			if(0) {
		#if MAX_MSG_TYPE
			} else if(DataAcsOpr == DATA_ACS_READ_MSG) {
				u32ItemNo_InStart_OutEnd = g_MsgCtr.u32MsgNo_ForAdd;
		#endif
		#if ACQ_ITEM_DAT_NUM
			} else if(DataAcsOpr == DATA_ACS_READ_ACQ) {
				u32ItemNo_InStart_OutEnd = g_AcqCtr.u32AcqNo_ForAdd;
		#endif
			} else {
				DataAcsOpr = DATA_ACS_RES_INPUT_ERR;
			}
		}
	} else {
		DataAcsOpr = DATA_ACS_RES_INPUT_ERR;
	}

	/* 正文处理 */
	/* 如果是从大读到小，则需要先检查数据是在ram中还是文件中: 如果是ram中，就调用ram解码；如果是文件中，就访问文件 */
	if(i32ItemNum_InNeed_OutRemain < 0) {
		if(0) {
	#if MAX_MSG_TYPE
		} else if(DataAcsOpr == DATA_ACS_READ_MSG) {
            /* 在ram中，尚未持久化的消息 */
			do {
				/* 搜索ram中的MsgNo最大的 */
				MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
				uint32 u32MaxMsgNo = 0;
				int8 i8MsgPt = -1;			/* 用于存储的MsgBuf下标 */
				for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
					/* 隐含了 u32ItemNo 非零，代表消息确认 */
					if((u32MaxMsgNo < pMsg->u32ItemNo) && (pMsg->u32ItemNo < u32ItemNo_InStart_OutEnd)) {
						u32MaxMsgNo = pMsg->u32ItemNo;
						i8MsgPt = i;
					}
					pMsg++;
				}

				if(i8MsgPt < 0) { 	/* 已经找完了 */
					break;
				} else {
					BOOL bMsgCmplt = ((g_MsgCtr.u32CmpltFlag & (1<<i8MsgPt)) != 0);
					if(PrintMsgCode(DataUser, &pBuf, pBufEnd, &g_MsgCtr.MsgBuf[i8MsgPt], bMsgCmplt, FALSE)) {
						u32ItemNo_InStart_OutEnd = g_MsgCtr.MsgBuf[i8MsgPt].u32ItemNo;
						i32ItemNum_InNeed_OutRemain++;
					} else {
						DataAcsOpr = DATA_ACS_RES_FULL;
						break;
					}
				}
			} while(i32ItemNum_InNeed_OutRemain);

            /* 曾经在MsgBuf中，但现在已经持久化的消息，存在 MsgSaved 部分。
            注: g_MsgCtr.MsgSaved存储的消息ItemNo是连续的，且g_MsgCtr.i8Pt_MsgSaved指向最后一条存储消息，仅需顺序查找即可 */
            int8 i8Pt = g_MsgCtr.i8Pt_MsgSaved;
            int8 i;
            if((i8Pt >= 0) && (i8Pt <= MSG_SAVED_BUF_LEN - 1)) {    /* 确保指针没出错 */
                for(i = MSG_SAVED_BUF_LEN; i32ItemNum_InNeed_OutRemain && (i > 0) && g_MsgCtr.MsgSaved[i8Pt].uMsgId; i--) {
					/* 这一条是上电初始化的，有新消息会把它覆盖掉，读到它说明MsgSaved是空的 */
					if((i8Pt == 0) && (g_MsgCtr.MsgSaved[i8Pt].uMsgId == MSG_HuanYinShiYong)) {
						break;
                    /* 从前往后读，MsgBuf已经搜索过了，MsgSaved是最新的了
                       且从前往后搜索MsgSaved，找到第一条小于u32ItemNo_InStart_OutEnd就是可用的 */
                    } else if(g_MsgCtr.MsgSaved[i8Pt].u32ItemNo >= u32ItemNo_InStart_OutEnd) {
                    } else if(!PrintMsgCode(DataUser, &pBuf, pBufEnd, &g_MsgCtr.MsgSaved[i8Pt], TRUE, FALSE)) {
						DataAcsOpr = DATA_ACS_RES_FULL;
						break;
                    } else {
						u32ItemNo_InStart_OutEnd = g_MsgCtr.MsgSaved[i8Pt].u32ItemNo;
                        i32ItemNum_InNeed_OutRemain++;
                    }                    
                    i8Pt--;
                    if(i8Pt < 0) {
                        i8Pt = MSG_SAVED_BUF_LEN - 1;
                    }
                }
            }
	#endif
	#if ACQ_ITEM_DAT_NUM
		} else if(DataAcsOpr == DATA_ACS_READ_ACQ) {
			int16 iAcqPt = g_AcqCtr.uBufPt_ForAdd;
			for(i = ACQ_WRITE_BUF_LEN; (i > 0) && i32ItemNum_InNeed_OutRemain; i--) {
				iAcqPt--;
				if(iAcqPt < 0) {
					iAcqPt = ACQ_WRITE_BUF_LEN-1;
				}
				if(!g_AcqCtr.AcqWriteBuf[iAcqPt].u32ItemNo) {			/* 已经找完了 */
					break;
				} else if(g_AcqCtr.AcqWriteBuf[iAcqPt].u32ItemNo < u32ItemNo_InStart_OutEnd) {
					if(ConvertAcqCode2Text(DataUser, &pBuf, pBufEnd, &g_AcqCtr.AcqWriteBuf[iAcqPt])) {
						u32ItemNo_InStart_OutEnd = g_AcqCtr.AcqWriteBuf[iAcqPt].u32ItemNo;
						i32ItemNum_InNeed_OutRemain++;
					} else {
						DataAcsOpr = DATA_ACS_RES_FULL;
						break;
					}
				}
			}
	#endif
		}
		
    #if MAX_MSG_TYPE
	} else if(DataAcsOpr == DATA_ACS_READ_MSG) {    /* 隐含(i32ItemNum_InNeed_OutRemain > 0) */
        /* 曾经在MsgBuf中，但现在已经持久化的消息，存在 MsgSaved 部分
        注: g_MsgCtr.MsgSaved存储的消息ItemNo是连续的，且g_MsgCtr.i8Pt_MsgSaved指向最后一条存储消息，仅需顺序查找即可 */
        int8 i8Pt = g_MsgCtr.i8Pt_MsgSaved;
        int8 i;
        if((i8Pt >= 0) && (i8Pt <= MSG_SAVED_BUF_LEN - 1)) {    /* 确保指针没出错 */
            for(i = MSG_SAVED_BUF_LEN; i32ItemNum_InNeed_OutRemain && (i > 0); i--) {
                i8Pt++; /* 指向第一条存储消息 */
                if(i8Pt >= MSG_SAVED_BUF_LEN) {
                    i8Pt = 0;
                }
				/* 这一条是上电初始化的，有新消息会把它覆盖掉，读到它说明MsgSaved是空的 */
				if((i8Pt == 0) && (g_MsgCtr.MsgSaved[i8Pt].uMsgId == MSG_HuanYinShiYong)) {
					break;
                } else if(!g_MsgCtr.MsgSaved[i8Pt].uMsgId) {   /* 还没有存储消息 */
                /* MsgSaved从后往前搜索，如果后面的ItemNo就已经超过了想要读取的，就没有必要继续了 */
                } else if(g_MsgCtr.MsgSaved[i8Pt].u32ItemNo > u32ItemNo_InStart_OutEnd + 1) {
                    break;
                /* 是从后往前读，且MsgSaved也是从后往前搜索，需要限制在恰好u32ItemNo_InStart_OutEnd + 1才是有效的 */
                } else if(g_MsgCtr.MsgSaved[i8Pt].u32ItemNo != u32ItemNo_InStart_OutEnd + 1) {
                } else if(!PrintMsgCode(DataUser, &pBuf, pBufEnd, &g_MsgCtr.MsgSaved[i8Pt], TRUE, FALSE)) {
                    DataAcsOpr = DATA_ACS_RES_FULL;
                    break;
                } else {
                    u32ItemNo_InStart_OutEnd = g_MsgCtr.MsgSaved[i8Pt].u32ItemNo;
                    i32ItemNum_InNeed_OutRemain--;
                }
            }
        }
	#endif
	}
    Swi_enable();

#if (SUPPORT_FAT_FS || CPU_0ST_1TI)
    BOOL bNeedReadNvMem = DATA_ACS_MISSION(DataAcsOpr)
                && ((i32ItemNum_InNeed_OutRemain < 0)    /* 从前往后读，前面代码已经ram中读取了，后续只能NvMem中读取 */
                /* 从后往前读，且读取的内容在ram中还有，就没有必要去NvMem中读取 */
                #if MAX_MSG_TYPE        /* g_MsgCtr.u32MsgNo_ram 是 ram存储的MsgNo起始(不含) */
                    || ((DataAcsOpr == DATA_ACS_READ_MSG) && (u32ItemNo_InStart_OutEnd < g_MsgCtr.u32MsgNo_ram))
                #endif
                #if ACQ_ITEM_DAT_NUM    /* g_AcqCtr.u32AcqNo_ram 是 ram存储的AcqNo起始(不含) */
                    || ((DataAcsOpr == DATA_ACS_READ_ACQ) && (u32ItemNo_InStart_OutEnd < g_AcqCtr.u32AcqNo_ram))
                #endif
                    || 0);
    int32 i32ItemNum_BeforeFileRead = i32ItemNum_InNeed_OutRemain;
#endif
    /* 从NvMem中读取:tf卡 */
#if SUPPORT_FAT_FS
	if(bNeedReadNvMem && (f_stat(DEVICE_PATH_String, NULL) == FR_OK)) {
    /* 支持新旧版本msg(包括acq,后面不再列举)读取
       从新往旧读(i32ItemNum_InNeed_OutRemain<0)，先读取 DEVICE_PATH_String/Msg2 ，再读取 DEVICE_PATH_String/Msg
       从旧往新读(i32ItemNum_InNeed_OutRemain>0)，先读取 DEVICE_PATH_String/Msg ，再读取 DEVICE_PATH_String/Msg2
       msg2是单级目录(/序号.txt)；msg是三级目录(/年/月/日.txt)
       为了代码兼容，msg2 模拟 msg,acq最后一级目录，即i8SeekFilePathDepth=0的情形    */
		FIL File;
		DIR dir;
		FILINFO fno;
		uint8* pFilePathRecBuf[MSG_ACQ_FILE_PATH_DEPTH];	/* 文件路径指针，用于分断年文件夹、月文件夹、日文件 */
		uint8* pText;
		uint8* pLineStart;									/* 文件中的数据指针，当前文本、行开始、行结尾 */
		uint8* pLineEnd;									
		uint32 u32LastOpenDirOrFileItemNo[MSG_ACQ_FILE_PATH_DEPTH]; /* 上一轮搜索的ItemNo */
		uint32 u32CurDirOrFileItemNo;/* 当前次文件搜索文件所附带的ItemNo, 和搜索目标最接近的文件ItemNo */
		uint32 u32MaybeCurLineItemNo;						/* 当前行号推断值，当行CRC校验错误的时候，就用推断值 */
		uint32 u32MaybeFileLastItemNo;                      /* 文件最后一条ItemNo，取超尾值，即+1 */
		uint32 u32MaybeFileFirstItemNo;                     /* 文件第一条ItemNo，来自于打开的文件名 */
		uint32 u32FileSaveItemNo;                           /* 文件所存储的最后一条ItemNo */
		int16 i;
		int16 iSeekFilePathDepth;							/* 正在搜索的路径深度:年2,月1,日0 */
		BOOL bFindFile, bCurDirSeekHaveResult;				/* 访问类型，定位相应的入门路径 */
        triST tCRC_nF_0N_pS = 0;                            /* 上一条消息CRC状态，<0:crc错误 0:初始化 >0:crc成功 */
        MSG_ACQ_ACS_VER eReadItemVer;

		/* 初始化文件目录 */
		uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
		PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
		if(0) {
	#if MAX_MSG_TYPE
		} else if(DataAcsOpr == DATA_ACS_READ_MSG) {
			PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Msg");
			u32FileSaveItemNo = g_MsgCtr.u32MsgNo_FileSaved;
	#endif
	#if ACQ_ITEM_DAT_NUM
		} else if(DataAcsOpr == DATA_ACS_READ_ACQ) {
			PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Acq");
			u32FileSaveItemNo = g_AcqCtr.u32AcqNo_FileSaved;
	#endif
		} else {
    		u32FileSaveItemNo = 0;
			pReadMsgOrAcqCtr->DataAcsOpr = DATA_ACS_RES_INPUT_ERR;
			return;
		}
        u32MaybeCurLineItemNo = u32FileSaveItemNo + 1;              /* 后面会减1 */
		pFilePathRecBuf[MSG_ACQ_FILE_PATH_DEPTH-1] = pFilePath;     /* 指向/Msg */
		for(i = MSG_ACQ_FILE_PATH_DEPTH - 1; i >= 0; i--) {			/* 清除下一级目录 */
			u32LastOpenDirOrFileItemNo[i] = 0;
		}
        *pFilePath = 0;
		if(0) {
	#if SUPPORT_MsgV1RW_AcqV1R
		} else if((i32ItemNum_InNeed_OutRemain > 0)	/* 从旧往新读，先读msg，再读msg2 */
				&& (f_opendir(&dir, (const TCHAR*)g_NvMemAcsCtr.u8FilePath) == FR_OK))
		{
		    eReadItemVer = MSG_ACQ_ACS_V1;
            iSeekFilePathDepth = 2;
	#endif
		} else {
		    eReadItemVer = MSG_ACQ_ACS_V2;
		    *pFilePath++ = '2';                                     /* 单层目录，产生/msg2 */
            pFilePathRecBuf[0] = pFilePath;                         /* 指向/Msg2 */
            iSeekFilePathDepth = 0;
		}

		/* 还有需要读的数据 */
		while(i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr)) {
            uint32 u32MachItemNo;       /* 要搜寻的行号，用于搜索文件以及在文件中定位消息 */
            if(i32ItemNum_InNeed_OutRemain > 0) {
                u32MachItemNo = u32ItemNo_InStart_OutEnd + 1;
            } else {
                u32MachItemNo = u32ItemNo_InStart_OutEnd - 1;
            }
            
			/* 搜索目录，定位当前所想读取的文件 */
			/* 搜索算法: 如果当前目录没有搜索到目标文件/目录，则退回上一级目录进行搜索；如果找到则往下一级目录中搜索
				目录级别: 0:最后一级文件, 1:月文件夹, 2:年文件夹		*/
			bFindFile = FALSE;
			do {
				/* 根据当前的深度进行搜索: 搜索相应的年、月目录以及日期文件 */
				pFilePath = pFilePathRecBuf[iSeekFilePathDepth];
				*pFilePath = 0;
                if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
                    DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
                } else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {
                    DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
                } else if(f_opendir(&dir, (const TCHAR*)g_NvMemAcsCtr.u8FilePath) != FR_OK) {
                    if((eReadItemVer == MSG_ACQ_ACS_V1) && (iSeekFilePathDepth == 2)) { /* 可能没有/msg 或者 /msg2目录 */
    					DataAcsOpr = DATA_ACS_RES_EMPTY;
                    } else {
    					DataAcsOpr = DATA_ACS_RES_OPEN_FILE_FAIL;
    				}
				} else {
				    /* 在当前层目录进行搜索 */
					*pFilePath++ = '/';
					if(u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] && (i32ItemNum_InNeed_OutRemain > 0)) { /* 二次搜索，且读u32ItemNo_InStart_OutEnd之后的 */
						u32MaybeFileFirstItemNo = 0xFFFFFF;
					} else {
						u32MaybeFileFirstItemNo = 0;
					}
					u32MaybeFileLastItemNo = u32FileSaveItemNo + 1;   /* 超尾 */
					bCurDirSeekHaveResult = FALSE;
					while((f_readdir(&dir, &fno) == FR_OK) && fno.fname[0]) {
						if((iSeekFilePathDepth != 0) ^ ((fno.fattrib & AM_DIR) == 0)) {
						    if(eReadItemVer == MSG_ACQ_ACS_V2) {
    							u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname);
						    } else if(eReadItemVer == MSG_ACQ_ACS_V1) {
    							u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname + 2);
    						}
                            /* 之前软件bug，会产生000000.txt，实际上里面存的第一条序号是1 */
    						if(u32CurDirOrFileItemNo == 0) {
    						    u32CurDirOrFileItemNo = 1;
    						}

							/* 文件可能的最大行号(超尾) */
							if((u32MachItemNo < u32CurDirOrFileItemNo) && (u32CurDirOrFileItemNo < u32MaybeFileLastItemNo)) {
								u32MaybeFileLastItemNo = u32CurDirOrFileItemNo;
							}

							/* 	搜索包含数据的目录或者文件--为了防止ItemNo不连续引起的读取失败，进行如下搜索:
								1. 首次搜索该目录(通过判断u32LastOpenDirOrFileItemNo[] 为零)，
									则搜索小于等于要读取的ItemNo中最大的，搜索结果存储于u32LastOpenDirOrFileItemNo[]
									如果搜索没有结果，且(i32ItemNum_InNeed_OutRemain > 0)，则把u32LastOpenDirOrFileItemNo[]置位1，以便二次搜索
								2. 二次搜索该目录(通过判断u32LastOpenDirOrFileItemNo[] 非零)，
									如果(i32ItemNum_InNeed_OutRemain < 0)则搜索比上次小的结果中最大的
									如果(i32ItemNum_InNeed_OutRemain > 0)则搜索比上次大的结果中最小的
							因为要搜索当前行号推断值u32MaybeCurLineItemNo，因此就算搜到恰好的文件也不退出搜索 */
							if(((u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] == 0) && (u32MaybeFileFirstItemNo < u32CurDirOrFileItemNo)
							        && (u32CurDirOrFileItemNo <= u32MachItemNo))
								|| (u32LastOpenDirOrFileItemNo[iSeekFilePathDepth]		/* 二次搜索 */
									&& (((i32ItemNum_InNeed_OutRemain < 0) && (u32MaybeFileFirstItemNo < u32CurDirOrFileItemNo)
											&& (u32CurDirOrFileItemNo < u32LastOpenDirOrFileItemNo[iSeekFilePathDepth]))
										|| ((i32ItemNum_InNeed_OutRemain > 0) && (u32CurDirOrFileItemNo < u32MaybeFileFirstItemNo)
											&& (u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] < u32CurDirOrFileItemNo)))))
							{
								for(i = 0; (i < FILE_NAME_BYTE_LEN) && fno.fname[i]; i++) {
									pFilePath[i] = fno.fname[i];
								}
								u32MaybeFileFirstItemNo = u32CurDirOrFileItemNo;
								bCurDirSeekHaveResult = TRUE;
							}
						}
					}

					/* 如果该层次目录进行了成功的搜索 */
					if(bCurDirSeekHaveResult) { 		/* 搜索到目录/文件 */
						u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] = u32MaybeFileFirstItemNo;
						pFilePath += i;
						if(iSeekFilePathDepth) {
							iSeekFilePathDepth--;										/* 进行下一级目录搜索 */
							pFilePathRecBuf[iSeekFilePathDepth] = pFilePath;
						} else {
							bFindFile = TRUE;
							/* 获得当前行号推断值: 如果往后读，则用当前的目录/文件ItemNo减1,因为后面会增1；往前读，则不能如此 */
							if(i32ItemNum_InNeed_OutRemain > 0) {
								u32MaybeCurLineItemNo = u32MaybeFileFirstItemNo - 1;	
							} else {
								u32MaybeCurLineItemNo = u32MaybeFileLastItemNo;	    /* 这是个超尾值，后面会减一，正合适 */
							}
						}
					/* 如果(i32ItemNum_InNeed_OutRemain > 0), 且该目录当前次搜索属于首次搜索，则置1，以方便进行二次搜索 */
					} else if((i32ItemNum_InNeed_OutRemain > 0) && (u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] == 0)) {
						u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] = 1;
					} else if(eReadItemVer == MSG_ACQ_ACS_V2) {   /* msg2目录已经读完，切换到msg */
						if(0) {
					#if SUPPORT_MsgV1RW_AcqV1R
						} else if(i32ItemNum_InNeed_OutRemain < 0) {	/* 从新往旧读，则还需要检索msg目录 */
    					    iSeekFilePathDepth = MSG_ACQ_FILE_PATH_DEPTH - 1;
    					    u32LastOpenDirOrFileItemNo[0] = 0;  /* 访问\msg2的时候，用了这个 */
    					    eReadItemVer = MSG_ACQ_ACS_V1;
					#endif						
						} else {	/* 从旧往新读，是先msg目录再msg2目录，因此算读完了 */
                        	DataAcsOpr = DATA_ACS_RES_EMPTY;
						}
					} else if(iSeekFilePathDepth < MSG_ACQ_FILE_PATH_DEPTH - 1) {	/* 则停止本目录搜索，搜索上一级目录 */
						for(i = iSeekFilePathDepth; i >= 0; i--) {					/* 清除下一级目录 */
							u32LastOpenDirOrFileItemNo[i] = 0;
						}
						iSeekFilePathDepth++;
					} else {							/* 已经到了最外面的目录，还是搜不到, 则放弃停止本次搜索 */
					    if(i32ItemNum_InNeed_OutRemain < 0) {       /* 从前往后读，是先msg2目录再msg目录，因此算读完了 */
                            DataAcsOpr = DATA_ACS_RES_EMPTY;
					    } else {    /* 从后往钱读，则还需要检索msg2目录 */
    					    iSeekFilePathDepth = 0;
    					    pFilePath = pFilePathRecBuf[MSG_ACQ_FILE_PATH_DEPTH-1]; /* 指向/msg */
                            *pFilePath++ = '2';                                     /* 单层目录，产生/msg2 */
                            pFilePathRecBuf[0] = pFilePath;                         /* 指向/Msg2 */
    					    u32LastOpenDirOrFileItemNo[0] = 0;  /* 访问\msg的时候，用了这个 */
    					    eReadItemVer = MSG_ACQ_ACS_V2;
					    }
					}
				}
			} while((!bFindFile) && DATA_ACS_MISSION(DataAcsOpr));

			/* 读取文件: 定位相应的行号，为了增强抗干扰性(防止乱码引起的读取失败)，每读取一行都和u32CurReadItemNo匹配 */
			*pFilePath = 0;
			if(DATA_ACS_RES(DataAcsOpr)) {				/* 前面搜索已经有结果了 */
			} else if((!bFindFile) || (f_open(&File, (const TCHAR*)g_NvMemAcsCtr.u8FilePath, FA_READ) != FR_OK)) {	/* 文件打开失败 */
				DataAcsOpr = DATA_ACS_RES_OPEN_FILE_FAIL;           /* 结束搜索，转到本函数最后处理 */
			} else {
                int32 i32DataFilePt = 0;
                UINT u32BufByteNum = 0;
                BOOL bFileCmplt = FALSE;                    /* 文件已经全部读进Buf */
                BOOL bNextFile = FALSE;                     /* 需要访问下一个文件 */

                /* 依据文件大小，决定是一次性全部读出；还是先定位后读取 */
                if(f_size(&File) < FILE_CONT_BUF_LEN) {     /* 文件较小，就一次性全部读出 */
                    f_read(&File, g_NvMemAcsCtr.u8FileCont, FILE_CONT_BUF_LEN, &u32BufByteNum);
                    bFileCmplt = TRUE;
                    if(u32BufByteNum == 0) {
                        bNextFile = TRUE;
                    } else if(i32ItemNum_InNeed_OutRemain > 0) {
                        pText = g_NvMemAcsCtr.u8FileCont;
                        pLineStart = pText;
                    } else {
                        pText = &g_NvMemAcsCtr.u8FileCont[u32BufByteNum - 1];
                        pLineEnd = pText;
                    }
                } else {
                    /* 如果文件较大，需要先快速查找的办法定位消息所在的位置，每条中文消息长度大约83.7Byte
                    循环迭代，主要更新三个关键变量：预估的搜索范围Zone, 该预估范围的起始StartNo, 该预估范围的超尾EndNo
                    1. 以全部文件范围初始化Zone
                    2. 每次读取两个Sector(512B*2), Buf中间(512B)在文件中的位置由下决定：
                        HalfPt = (ReadNo - StartNo)*Zone_len/(EndNo - StartNo)
                    3. 如果想要读取的ItemNo在该Sector中，则停止搜索
                        如果想要读取的ItemNo在该Sector后，则以该Sector后作为新的搜索范围，重复23
                        如果想要读取的ItemNo在该Sector前，则以该Sector前作为新的搜索范围，重复23                  */
                    /* 初始化， */
                    uint16 uLineBLen;
                    if(u32MaybeFileLastItemNo < u32MaybeFileFirstItemNo) {
                        uLineBLen = 84; /* 每条中文消息长度大约83.7Byte */
                    } else {
                        uint32 u32FileItemNum = u32MaybeFileLastItemNo - u32MaybeFileFirstItemNo;
                        uLineBLen = (f_size(&File) + u32FileItemNum/2)/u32FileItemNum;
                    }
                    i32DataFilePt = (u32MachItemNo - u32MaybeFileFirstItemNo)*uLineBLen;
                    triST tItemPlace_nBeforeSecotr_0InSector_pAfterSector;
                    do {
                        /* 一次读取2个Sector */
                        i32DataFilePt -= 256;   /* 如果要读取的在Sector前一半就连带读取前一个Sector;反之读取后一个Sector */
                        if(i32DataFilePt < 0) {
                            i32DataFilePt = 0;
                        } else if(i32DataFilePt > f_size(&File)) {
                            i32DataFilePt = f_size(&File) - 512;
                        }
                        i32DataFilePt = (i32DataFilePt/512)*512;    /* 指向一个完整的Sector */
                        f_lseek(&File, i32DataFilePt);
                        if(FILE_CONT_BUF_LEN > 1024) {
                            f_read(&File, g_NvMemAcsCtr.u8FileCont, 1024, &u32BufByteNum);  /* 一次读两个Sector */
                        } else {    /* 本想一次读两个Sector，奈何Buf太小 */
                            f_read(&File, g_NvMemAcsCtr.u8FileCont, FILE_CONT_BUF_LEN, &u32BufByteNum);
                        }
                        if(u32BufByteNum == 0) {
                            break;
                        } else {
                            pText = g_NvMemAcsCtr.u8FileCont;
                            if(i32DataFilePt == 0) {    /* 如果是从文件开头读取，则第一个字节是行头 */
                                pLineStart = pText;
                            } else {    /* 否则，需要先找到0x0A作为新一行的开始 */
                                pLineStart = NULL;
                            }
                        }

                        /* Sector是否包含了要读取的ItemNo */
                        uint32 u32LineItemNo = u32MaybeCurLineItemNo;
                        tItemPlace_nBeforeSecotr_0InSector_pAfterSector = 1;
                        BOOL bBufEnd = FALSE;
                        BOOL bLineItemNoLsMatchItemNo = FALSE;
                        do {
                            bBufEnd = (pText >= &g_NvMemAcsCtr.u8FileCont[u32BufByteNum - 1]);
                            if((*pText == 0x0A)     /* 搜索到行结尾 或者 文件结尾 */
                                || (bBufEnd && (i32DataFilePt + u32BufByteNum > f_size(&File))))
                            {
                                pLineEnd = pText + 1;
                                /* 找到了一行，开始读取；需要滤除空行，或者仅几个字的行 */
                                if((pLineStart != NULL) && (pLineEnd - pLineStart > 10)) {
    								/* 对这一行内容进行CRC校验，并获取相应的行编号 */
    								if(CheckTextWithCrc32ForMsgAndAcq(pLineStart, pLineEnd)) {
        								u32LineItemNo = ReadU32FixLen(pLineStart);
                                        uLineBLen = pLineEnd - pLineStart;   /* 根据实际消息长度进行修正 */
        								/* 如果当前Sector内容已经超出了所要读取的，就没有必要继续往后搜索了 */
        								if(u32MachItemNo < u32LineItemNo) {
        							/* 消息可能会出现空洞，之前是u32MachItemNo > u32LineItemNo，变成了u32MachItemNo < u32LineItemNo */
        								    if(bLineItemNoLsMatchItemNo) {
            								    tItemPlace_nBeforeSecotr_0InSector_pAfterSector = 0;
        								    } else {
            								    tItemPlace_nBeforeSecotr_0InSector_pAfterSector = -1;
            								}
        								    break;
        								}
    								} else {
    								    u32LineItemNo++;
    								}
    								/* 当前行号在匹配的ItemNo之前，先做好标识，以备消息空洞 */
    								if(u32LineItemNo < u32MachItemNo) {
    								    bLineItemNoLsMatchItemNo = TRUE;
    								    
    								/* 找到了对应的行 */
    								} else if(u32MachItemNo == u32LineItemNo) {
        								tItemPlace_nBeforeSecotr_0InSector_pAfterSector = 0;
    								    break;
    								}
    							}
                                pLineStart = pLineEnd;
                            }
                            pText++;
                        } while(!bBufEnd);

                        /* 需要放弃读取 */
                        if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
                            DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
                        } else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {
                            DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;

                        /* 要读取的数据在Sector之前 */
                        } else if(tItemPlace_nBeforeSecotr_0InSector_pAfterSector < 0) {
                            if(i32DataFilePt == 0) {    /* 已经是文件的开头，本文件已经找遍了，需要寻找其他文件 */
                                bNextFile = TRUE;
                            } else {
                                i32DataFilePt += pLineStart - g_NvMemAcsCtr.u8FileCont; /* 行开头在文件中的位置 */
                                i32DataFilePt -= (u32LineItemNo - u32MachItemNo)*uLineBLen;    /* 期望的行在文件中的位置 */
                            }

                        /* 拥有匹配的选择 */
                        } else if(tItemPlace_nBeforeSecotr_0InSector_pAfterSector == 0) {
                            /* 如果是往前读，为了对接后面的读的代码，需要调整pText */
                            if(i32ItemNum_InNeed_OutRemain < 0) {
                                pText = pLineStart - 1; /* 指向上一行的结尾:0x0A */
                            }

                        /* 要读取的数据在Sector之后 */
                        } else if(tItemPlace_nBeforeSecotr_0InSector_pAfterSector > 0) {
                            if(i32DataFilePt + u32BufByteNum >= f_size(&File)) {  /* 已经是文件的结尾，本文件找遍了，需要寻找其他文件 */
                                bNextFile = TRUE;
                            } else {
                                i32DataFilePt += pLineStart - g_NvMemAcsCtr.u8FileCont; /* 行开头在文件中的位置 */
                                /* 期望的行在文件中的位置，由于pLineStart实际指向下一行的行首 */
                                i32DataFilePt += (u32MachItemNo - u32LineItemNo - 1)*uLineBLen; 
                            }
                        }
                    } while(tItemPlace_nBeforeSecotr_0InSector_pAfterSector && (!bNextFile) && DATA_ACS_MISSION(DataAcsOpr));
                }

                /* 本文件中没有，要去读下一个文件; 或者需要放弃 */
                if(bNextFile || DATA_ACS_RES(DataAcsOpr)) {
                
                /* 从文件中读取 */
    			} else if(i32ItemNum_InNeed_OutRemain > 0) {
    				do {
    					/* 在读取的文件Buf中进行具体的一行搜索 */
    					BOOL bBufEnd = FALSE;
    					do {
    					    bBufEnd = (pText >= &g_NvMemAcsCtr.u8FileCont[u32BufByteNum-1]);
                            /* 行结尾 或者 文件结尾 */
                            if((*pText == 0x0A) || (bBufEnd && bFileCmplt)) {
    							pLineEnd = pText + 1;
                                /* 需要放弃读取 */
                                if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {   
                                    DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
                                    break;
                                } else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {
                                    DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
                                    break;
                                    
                                /* 找到了一行，开始读取；需要滤除空行，或者仅几个字的行 */
    							} else if((pLineStart != NULL) && (pLineStart + 10 < pLineEnd)) {
    								/* 对这一行内容进行CRC校验，并查看是否是所要搜索的行? */
    								BOOL bCrcOK = CheckTextWithCrc32ForMsgAndAcq(pLineStart, pLineEnd);
    								if(bCrcOK) {
    									u32MaybeCurLineItemNo = ReadU32FixLen(pLineStart);
    								} else {	/* 如果CRC校验失败，则用前一行的ItemNo加1实现 */
    									u32MaybeCurLineItemNo++;
    								}

    								if(u32MaybeCurLineItemNo > u32ItemNo_InStart_OutEnd) {			/* 是所要读取的行 */
    									Swi_disable();		/* 需要防止该任务已经被取消了 */
    									if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {
    										Swi_enable();
                                            DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
    										break;
                                        /* 网络通讯，被破坏的消息发送出来，json解码也可能会有问题。
                                            因此  如果CRC状态发生变化，就完结json消息帧，以使得好坏json分离 */
                                        } else if((DataUser == DATA_USER_NET)
                                                && (((tCRC_nF_0N_pS < 0) && bCrcOK) || ((tCRC_nF_0N_pS > 0) && (!bCrcOK))))
                                        {
    										Swi_enable();
                                            DataAcsOpr = DATA_ACS_RES_CRC_CHG;
    										break;
    									} else if(ReadTextFromLineForMsgOrAcq_MustSwiDisable(DataUser, DataAcsOpr, &pBuf, pBufEnd, pLineStart, pLineEnd, bCrcOK)) {
    										Swi_enable();
                                            tCRC_nF_0N_pS = bCrcOK? 1 : -1;
    										u32ItemNo_InStart_OutEnd = u32MaybeCurLineItemNo;
    										i32ItemNum_InNeed_OutRemain--;
    										if(i32ItemNum_InNeed_OutRemain == 0) {  /* 已经完成读取 */
    										    break;
    										}
    									} else {
    										Swi_enable();
    										DataAcsOpr = DATA_ACS_RES_FULL;
    										break;
    									}
    								}
    							}
    							pLineStart = pLineEnd;
    						}
    						pText++;
    					} while(!bBufEnd);

    					/* 文件已经读完 */
    					if(bFileCmplt) {
    					    break;
    					} else {    /* 继续读取文件 */
        					i32DataFilePt += pLineEnd - g_NvMemAcsCtr.u8FileCont;		/* 修改文件读取指针 */
        					f_lseek(&File, i32DataFilePt);
        					f_read(&File, g_NvMemAcsCtr.u8FileCont, FILE_CONT_BUF_LEN, &u32BufByteNum);
                            bFileCmplt = (u32BufByteNum < FILE_CONT_BUF_LEN);
        					if(u32BufByteNum) {
        						pText = g_NvMemAcsCtr.u8FileCont;
        						pLineStart = g_NvMemAcsCtr.u8FileCont;
        					} else {		/* 已经读不出数据，跳出本文件，开始搜索下一个文件 */
        						break;
        					}
        				}
    				} while(i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr));
    			} else {									/* i32ItemNum_InNeed_OutRemain < 0, 则从文件末尾开始读 */
    				do {
    					/* 在读取的文件Buf中进行具体的一行搜索 */
    					BOOL bBufEnd = FALSE;
    					do {
    						/* 行结尾 或者 文件开头 */
                            bBufEnd = (pText <= g_NvMemAcsCtr.u8FileCont);
                            if((*pText == 0x0A) || (bBufEnd && bFileCmplt)) {
    							pLineStart = pText;
    							if(*pText == 0x0A) {
    								pLineStart++;
    							}

                                /* 异常需要退出的情况 */
                                if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
                                    DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
                                    break;
                                } else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {
                                    DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
                                    break;
                                    
                                 /* 找到了一行，开始读取；需要滤除空行，或者仅几个字的行 */
    							} else if((pLineEnd != NULL) && (pLineStart + 10 < pLineEnd)) {
    								/* 对这一行内容进行CRC校验，并查看是否是所要搜索的行? */
                                    BOOL bCrcOK = CheckTextWithCrc32ForMsgAndAcq(pLineStart, pLineEnd);
                                    if(bCrcOK) {
    									u32MaybeCurLineItemNo = ReadU32FixLen(pLineStart);
    								} else {	/* 如果CRC校验失败，则用前一行的ItemNo减1实现 */
    									u32MaybeCurLineItemNo--;
    								}

    								if(u32MaybeCurLineItemNo < u32ItemNo_InStart_OutEnd) {			/* 是所要读取的行 */
    									Swi_disable();		/* 需要防止该任务已经被取消了 */
    									if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {									
    										Swi_enable();
                                            DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
                                            break;
                                        /* 网络通讯，被破坏的消息发送出来，json解码也可能会有问题。
                                            因此  如果CRC状态发生变化，就完结json消息帧，以使得好坏json分离 */
                                        } else if((DataUser == DATA_USER_NET)
                                                && (((tCRC_nF_0N_pS < 0) && bCrcOK) || ((tCRC_nF_0N_pS > 0) && (!bCrcOK))))
                                        {
                                            Swi_enable();
                                            DataAcsOpr = DATA_ACS_RES_CRC_CHG;
                                            break;
    									} else if(ReadTextFromLineForMsgOrAcq_MustSwiDisable(DataUser, DataAcsOpr, &pBuf, pBufEnd, pLineStart, pLineEnd, bCrcOK)) {
    										Swi_enable();
                                            tCRC_nF_0N_pS = bCrcOK? 1 : -1;
    										u32ItemNo_InStart_OutEnd = u32MaybeCurLineItemNo;
    										i32ItemNum_InNeed_OutRemain++;
    										if(i32ItemNum_InNeed_OutRemain == 0) {
    										    break;
    										}
    									} else {
    										Swi_enable();
    										DataAcsOpr = DATA_ACS_RES_FULL;    										
                                            break;
    									}
    								}
    							}
    							pLineEnd = pLineStart;
    						}
    						pText--;
    					} while(!bBufEnd);

                        /* 读取文件 */
                        if(bFileCmplt) {     /* 文件已经读完 */
                            break;
                        } else {            /* 继续读取文件 */
                            u32BufByteNum = i32DataFilePt + (pLineStart - g_NvMemAcsCtr.u8FileCont);    /* 计算还剩多少数据未读取 */
                            bFileCmplt = (u32BufByteNum <= FILE_CONT_BUF_LEN);
                            if(bFileCmplt) {
                                i32DataFilePt = 0;
                            } else {
                                i32DataFilePt -= FILE_CONT_BUF_LEN - (pLineStart - g_NvMemAcsCtr.u8FileCont);
                                u32BufByteNum = FILE_CONT_BUF_LEN;
                            }
        					f_lseek(&File, i32DataFilePt);
        					f_read(&File, g_NvMemAcsCtr.u8FileCont, u32BufByteNum, &u32BufByteNum);
        					if(u32BufByteNum >= 2) {
        						pText = &g_NvMemAcsCtr.u8FileCont[u32BufByteNum - 2];
        						pLineEnd = &g_NvMemAcsCtr.u8FileCont[u32BufByteNum - 1];
        					} else {
        						break;
        					}
        				}
    				} while(i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr));
    			}
    		}
		}
	} 
#endif

#if CPU_0ST_1TI
	/* 从NvMem中读取: eeprom：前面文件中没有读取，则再进行eeprom读取 */
    /* bNeedReadNvMem 代表读文件前，是否要读这个地方，本来应该更新bNeedReadNvMem的，但是因为读eeprom前提是读文件没有获得数据，那也就没有必要更新了 */
	if(bNeedReadNvMem && (i32ItemNum_BeforeFileRead == i32ItemNum_InNeed_OutRemain)) {
        /* 前面文件没有获取任何数据，进入eeprom读取需要再次初始化；
            否则DataAcsOpr可能是 DATA_ACS_RES_EMPTY, 或者DATA_ACS_RES_OPEN_FILE_FAIL */
        if(pReadMsgOrAcqCtr == NULL) {
            DataAcsOpr = DATA_ACS_READ_MSG;
        } else {
            DataAcsOpr = pReadMsgOrAcqCtr->DataAcsOpr;
        }
        
		/* 从最近存储的位置开始查找 */
		if(0) {
	#if MAX_MSG_TYPE
		} else if(DataAcsOpr == DATA_ACS_READ_MSG) {
			uint32 u32LastItemNo = 0xFFFFFFFFUL;
			uint16 uCurReadEEPromItemPt = g_MsgCtr.uEEPromNo_ForSave%EEPROM_MSG_ITEM_NUM;		/* 利用最大EEPromNo和ITEM_NUM整数倍的关系 */
			uint16 uRemainItemInEEProm = EEPROM_MSG_ITEM_NUM;		/* 初始化总的Item数量 */
			uint16 uCurReadItemNum;
			int16 i;
			if(i32ItemNum_InNeed_OutRemain > 0) {
				/* 从最新的数据往前先搜索距离所要读的ItemNo最近的Item */
				do {
					/* 计算读取EEProm位置、读取长度 */
					if(uCurReadEEPromItemPt == 0) {
						uCurReadEEPromItemPt = EEPROM_MSG_ITEM_NUM;
					}
					if(uCurReadEEPromItemPt > FILE_CONT_BUF_LEN/sizeof(MSG_ITEM)) {			/* 保证读出整数条 */
						uCurReadItemNum = FILE_CONT_BUF_LEN/sizeof(MSG_ITEM);
					} else {
						uCurReadItemNum = uCurReadEEPromItemPt;
					}
					if(uCurReadItemNum > uRemainItemInEEProm) {
						uCurReadItemNum = uRemainItemInEEProm;
					}
					uCurReadEEPromItemPt -= uCurReadItemNum;
					EEPROMRead((uint32_t*)g_NvMemAcsCtr.u8FileCont, EEPROM_ADDR_MSG + uCurReadEEPromItemPt*sizeof(MSG_ITEM),
								uCurReadItemNum*sizeof(MSG_ITEM));
					
					/* 搜索数据 */
					MSG_ITEM* pMsgEE = (MSG_ITEM*)(&g_NvMemAcsCtr.u8FileCont[uCurReadItemNum*sizeof(MSG_ITEM)]);
					for(i = uCurReadItemNum; i > 0; i--) {
						pMsgEE--;
						/* ItemNo本来应该是递增的，但发生逆转，说明再往前的数据可能不是这台机组的 */
						if((pMsgEE->u32ItemNo >= u32LastItemNo)
							|| (pMsgEE->u32ItemNo <= u32ItemNo_InStart_OutEnd))	/* ItemNo已经到了读出点 */
						{
							break;
						}
						u32LastItemNo = pMsgEE->u32ItemNo;
					}

					/* 有结果，跳出循环 */
					if(i) {
						uRemainItemInEEProm -= (uCurReadItemNum - i);
						uCurReadEEPromItemPt += i;
						break;
					} else {
						uRemainItemInEEProm -= uCurReadItemNum;
					}
				} while(uRemainItemInEEProm);

				/* 已经找到起点开始读取 */
				uRemainItemInEEProm = EEPROM_MSG_ITEM_NUM - uRemainItemInEEProm;	/* 前面是剩余未搜索 */
				while(uRemainItemInEEProm && i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr)) {
					/* 计算读取EEProm位置 */
					if(uCurReadEEPromItemPt >= EEPROM_MSG_ITEM_NUM) {
						uCurReadEEPromItemPt = 0;
					}
					uCurReadItemNum = EEPROM_MSG_ITEM_NUM - uCurReadEEPromItemPt;
					if(uCurReadItemNum > FILE_CONT_BUF_LEN/sizeof(MSG_ITEM)) {	/* 保证读出整数条 */
						uCurReadItemNum = FILE_CONT_BUF_LEN/sizeof(MSG_ITEM);
					}
					if(uCurReadItemNum > uRemainItemInEEProm) {
						uCurReadItemNum = uRemainItemInEEProm;
					}
					EEPROMRead((uint32_t*)g_NvMemAcsCtr.u8FileCont, EEPROM_ADDR_MSG + uCurReadEEPromItemPt*sizeof(MSG_ITEM),
								uCurReadItemNum*sizeof(MSG_ITEM));
					uCurReadEEPromItemPt += uCurReadItemNum;
					uRemainItemInEEProm -= uCurReadItemNum;
					
					/* 搜索数据 */
					MSG_ITEM* pMsgEE = (MSG_ITEM*)(g_NvMemAcsCtr.u8FileCont);
					Swi_disable();		/* 需要防止该任务已经被取消了 */
                    if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
                        DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
					} else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {					
                        DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
					} else {
						for(i = uCurReadItemNum; (i > 0) && i32ItemNum_InNeed_OutRemain; i--) {
							if(pMsgEE->u32ItemNo > u32ItemNo_InStart_OutEnd) {
								CONV_MSG_ACQ_CODE_RES ConvCodeRes = ConvertMsgEECode2Text(DataUser, &pBuf, pBufEnd, pMsgEE);
								if(ConvCodeRes == CONV_MSG_ACQ_CODE_SUC) {
									u32ItemNo_InStart_OutEnd = pMsgEE->u32ItemNo;
									i32ItemNum_InNeed_OutRemain--;
								} else if(ConvCodeRes == CONV_MSG_ACQ_CODE_ERR_DATA) {
									u32ItemNo_InStart_OutEnd = pMsgEE->u32ItemNo;
								} else {
									DataAcsOpr = DATA_ACS_RES_FULL;
									break;
								}
							}
							pMsgEE++;
						}
					}
                    Swi_enable();
				};
			} else {	/* 从最新的数据往前读即可 */
				while(uRemainItemInEEProm && i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr)) {
					/* 计算读取EEProm位置 */
					if(uCurReadEEPromItemPt == 0) {
						uCurReadEEPromItemPt = EEPROM_MSG_ITEM_NUM;
					}
					if(uCurReadEEPromItemPt > FILE_CONT_BUF_LEN/sizeof(MSG_ITEM)) {			/* 保证读出整数条 */
						uCurReadItemNum = FILE_CONT_BUF_LEN/sizeof(MSG_ITEM);
					} else {
						uCurReadItemNum = uCurReadEEPromItemPt;
					}
					if(uCurReadItemNum > uRemainItemInEEProm) {
						uCurReadItemNum = uRemainItemInEEProm;
					}
					uCurReadEEPromItemPt -= uCurReadItemNum;
					EEPROMRead((uint32_t*)g_NvMemAcsCtr.u8FileCont, EEPROM_ADDR_MSG + uCurReadEEPromItemPt*sizeof(MSG_ITEM), 
								uCurReadItemNum*sizeof(MSG_ITEM));
					uRemainItemInEEProm -= uCurReadItemNum;

					/* 搜索数据 */
					MSG_ITEM* pMsgEE = (MSG_ITEM*)(&g_NvMemAcsCtr.u8FileCont[uCurReadItemNum*sizeof(MSG_ITEM)]);
					Swi_disable();		/* 需要防止该任务已经被取消了 */
                    if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
                        DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
					} else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {					    
                        DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
					} else {
						for(i = uCurReadItemNum; (i > 0) && i32ItemNum_InNeed_OutRemain; i--) {
							pMsgEE--;
							/* ItemNo本来应该是递增的，但发生逆转，说明数据可能不是这台机组的，需要结束读取 */
							if((pMsgEE->u32ItemNo >= u32LastItemNo)
								|| (pMsgEE->u32ItemNo == 0))	/* 无效数据 */
							{
								uRemainItemInEEProm = 0;
								break;
							/* 是所要读取的数据,进行解码 */
							} else if(pMsgEE->u32ItemNo < u32ItemNo_InStart_OutEnd) {
								CONV_MSG_ACQ_CODE_RES ConvCodeRes = ConvertMsgEECode2Text(DataUser, &pBuf, pBufEnd, pMsgEE);
								if(ConvCodeRes == CONV_MSG_ACQ_CODE_SUC) {
									u32ItemNo_InStart_OutEnd = pMsgEE->u32ItemNo;
									i32ItemNum_InNeed_OutRemain++;
								} else if(ConvCodeRes == CONV_MSG_ACQ_CODE_ERR_DATA) {
									u32ItemNo_InStart_OutEnd = pMsgEE->u32ItemNo;
								} else {
									DataAcsOpr = DATA_ACS_RES_FULL;
									break;
								}
							}
							u32LastItemNo = pMsgEE->u32ItemNo;
						}
					}
                    Swi_enable();
				};
			}
	#endif
	#if ACQ_ITEM_DAT_NUM
		} else if(DataAcsOpr == DATA_ACS_READ_ACQ) {
			uint32 u32LastItemNo = 0xFFFFFFFFUL;
			uint16 uCurReadEEPromItemPt = g_AcqCtr.uEEPromNo_ForSave%EEPROM_ACQ_ITEM_NUM;		/* 利用最大EEPromNo和ITEM_NUM整数倍的关系 */
			uint16 uRemainItemInEEProm = EEPROM_ACQ_ITEM_NUM;		/* 初始化总的Item数量 */
			uint16 uCurReadItemNum;
			int16 i;
			if(i32ItemNum_InNeed_OutRemain > 0) {
				/* 从最新的数据往前先搜索距离所要读的ItemNo最近的Item */
				do {
					/* 计算读取EEProm位置、读取长度 */
					if(uCurReadEEPromItemPt == 0) {
						uCurReadEEPromItemPt = EEPROM_ACQ_ITEM_NUM;
					}
					if(uCurReadEEPromItemPt > FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR)) {			/* 保证读出整数条 */
						uCurReadItemNum = FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR);
					} else {
						uCurReadItemNum = uCurReadEEPromItemPt;
					}
					if(uCurReadItemNum > uRemainItemInEEProm) {
						uCurReadItemNum = uRemainItemInEEProm;
					}
					uCurReadEEPromItemPt -= uCurReadItemNum;
					EEPROMRead((uint32_t*)g_NvMemAcsCtr.u8FileCont, EEPROM_ADDR_ACQ + uCurReadEEPromItemPt*sizeof(ACQ_EE_ITEM_VAR),
								uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR));
					
					/* 搜索数据 */
					ACQ_EE_ITEM_VAR* pAcqEE = (ACQ_EE_ITEM_VAR*)(&g_NvMemAcsCtr.u8FileCont[uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR)]);
					for(i = uCurReadItemNum; i > 0; i--) {
						pAcqEE--;
						/* ItemNo本来应该是递增的，但发生逆转，说明再往前的数据可能不是这台机组的 */
						if((pAcqEE->u32ItemNo >= u32LastItemNo)
							|| (pAcqEE->u32ItemNo <= u32ItemNo_InStart_OutEnd))		/* ItemNo已经到了读出点 */
						{
							break;
						}
						u32LastItemNo = pAcqEE->u32ItemNo;
					}
					
					/* 有结果，跳出循环 */
					if(i) {
						uRemainItemInEEProm -= (uCurReadItemNum - i);
						uCurReadEEPromItemPt += i;
						break;
					} else {
						uRemainItemInEEProm -= uCurReadItemNum;
					}
				} while(uRemainItemInEEProm);

				/* 已经找到起点开始读取 */
				uRemainItemInEEProm = EEPROM_ACQ_ITEM_NUM - uRemainItemInEEProm;	/* 前面是剩余未搜索 */
				while(uRemainItemInEEProm && i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr)) {
					/* 计算读取EEProm位置 */
					if(uCurReadEEPromItemPt >= EEPROM_ACQ_ITEM_NUM) {
						uCurReadEEPromItemPt = 0;
					}
					uCurReadItemNum = EEPROM_ACQ_ITEM_NUM - uCurReadEEPromItemPt;
					if(uCurReadItemNum > FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR)) {		/* 保证读出整数条 */
						uCurReadItemNum = FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR);
					}
					if(uCurReadItemNum > uRemainItemInEEProm) {
						uCurReadItemNum = uRemainItemInEEProm;
					}
					EEPROMRead((uint32_t*)g_NvMemAcsCtr.u8FileCont, EEPROM_ADDR_ACQ + uCurReadEEPromItemPt*sizeof(ACQ_EE_ITEM_VAR),
								uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR));
					uCurReadEEPromItemPt += uCurReadItemNum;
					uRemainItemInEEProm -= uCurReadItemNum;
					
					/* 搜索数据 */
					ACQ_EE_ITEM_VAR* pAcqEE = (ACQ_EE_ITEM_VAR*)(g_NvMemAcsCtr.u8FileCont);
					Swi_disable();		/* 需要防止该任务已经被取消了 */
					if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
                        DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
					} else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {					
                        DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
					} else {
						for(i = uCurReadItemNum; (i > 0) && i32ItemNum_InNeed_OutRemain; i--) {
							if(pAcqEE->u32ItemNo > u32ItemNo_InStart_OutEnd) {
								CONV_MSG_ACQ_CODE_RES ConvCodeRes = ConvertAcqEECode2Text(DataUser, &pBuf, pBufEnd, pAcqEE);
								if(ConvCodeRes == CONV_MSG_ACQ_CODE_SUC) {
									u32ItemNo_InStart_OutEnd = pAcqEE->u32ItemNo;
									i32ItemNum_InNeed_OutRemain--;
								} else if(ConvCodeRes == CONV_MSG_ACQ_CODE_ERR_DATA) {
									u32ItemNo_InStart_OutEnd = pAcqEE->u32ItemNo;
								} else {
									DataAcsOpr = DATA_ACS_RES_FULL;
									break;
								}
							}
							pAcqEE++;
						}
					}
                    Swi_enable();
				};
			} else {	/* 从最新的数据往前读即可 */
				while(uRemainItemInEEProm && i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr)) {
					/* 计算读取EEProm位置 */
					if(uCurReadEEPromItemPt == 0) {
						uCurReadEEPromItemPt = EEPROM_ACQ_ITEM_NUM;
					}
					if(uCurReadEEPromItemPt > FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR)) {			/* 保证读出整数条 */
						uCurReadItemNum = FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR);
					} else {
						uCurReadItemNum = uCurReadEEPromItemPt;
					}
					if(uCurReadItemNum > uRemainItemInEEProm) {
						uCurReadItemNum = uRemainItemInEEProm;
					}
					uCurReadEEPromItemPt -= uCurReadItemNum;
					EEPROMRead((uint32_t*)g_NvMemAcsCtr.u8FileCont, EEPROM_ADDR_ACQ + uCurReadEEPromItemPt*sizeof(ACQ_EE_ITEM_VAR),
								uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR));
					uRemainItemInEEProm -= uCurReadItemNum;

					/* 搜索数据 */
					ACQ_EE_ITEM_VAR* pAcqEE = (ACQ_EE_ITEM_VAR*)(&g_NvMemAcsCtr.u8FileCont[uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR)]);
					Swi_disable();		/* 需要防止该任务已经被取消了 */
					if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
                        DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
					} else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {					    
                        DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
					} else {
						for(i = uCurReadItemNum; (i > 0) && i32ItemNum_InNeed_OutRemain; i--) {
							pAcqEE--;
							/* ItemNo本来应该是递减的，但发生逆转，说明数据可能不是这台机组的，需要结束读取 */
							if((pAcqEE->u32ItemNo >= u32LastItemNo)
								|| (pAcqEE->u32ItemNo == 0))			/* 无效数据 */
							{
								uRemainItemInEEProm = 0;
								break;
							/* 是所要读取的数据,进行解码 */
							} else if(pAcqEE->u32ItemNo < u32ItemNo_InStart_OutEnd) {
								CONV_MSG_ACQ_CODE_RES ConvCodeRes = ConvertAcqEECode2Text(DataUser, &pBuf, pBufEnd, pAcqEE);
								if(ConvCodeRes == CONV_MSG_ACQ_CODE_SUC) {
									u32ItemNo_InStart_OutEnd = pAcqEE->u32ItemNo;
									i32ItemNum_InNeed_OutRemain++;
								} else if(ConvCodeRes == CONV_MSG_ACQ_CODE_ERR_DATA) {
									u32ItemNo_InStart_OutEnd = pAcqEE->u32ItemNo;
								} else {
									DataAcsOpr = DATA_ACS_RES_FULL;
									break;
								}
							}
							u32LastItemNo = pAcqEE->u32ItemNo;
						}
					}
                    Swi_enable();
				};
			}
	#endif
		}
	}
#endif

	Swi_disable();
    if(READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {
        /* 前面读NvMem如果读空，会把DataAcsOpr修改成DATA_ACS_RES_EMPTY，这种情况下，后面应该继续读 */
        if(DataAcsOpr == DATA_ACS_RES_EMPTY) {
            if(pReadMsgOrAcqCtr == NULL) {
                DataAcsOpr = DATA_ACS_READ_MSG;
            } else {
                DataAcsOpr = pReadMsgOrAcqCtr->DataAcsOpr;
            }
        }
        
        /* 如果是从小读到大，则需要后检查ram中的数据 */
        if((i32ItemNum_InNeed_OutRemain > 0) && DATA_ACS_MISSION(DataAcsOpr)) {
            if(0) {
        #if MAX_MSG_TYPE
            } else if(DataAcsOpr == DATA_ACS_READ_MSG) {
                do {
                    /* 搜索ram中的MsgNo最小的 */
                    MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
                    uint32 u32MinMsgNo = 0xFFFFFFFF;
                    int8 i8MsgPt = -1;          /* 用于存储的MsgBuf下标 */
                    for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
                        if((u32MinMsgNo > pMsg->u32ItemNo) && (pMsg->u32ItemNo > u32ItemNo_InStart_OutEnd)) {
                            u32MinMsgNo = pMsg->u32ItemNo;
                            i8MsgPt = i;
                        }
                        pMsg++;
                    }
                    if(i8MsgPt < 0) { /* 已经找完了 */
                        break;
                    } else {                    
                        BOOL bMsgCmplt = ((g_MsgCtr.u32CmpltFlag & (1<<i8MsgPt)) != 0);
                        if((pReadMsgOrAcqCtr == NULL) && (!bMsgCmplt)) {
                            DataAcsOpr = DATA_ACS_RES_EMPTY;
                            break;
                        } else if(PrintMsgCode(DataUser, &pBuf, pBufEnd, &g_MsgCtr.MsgBuf[i8MsgPt], bMsgCmplt, FALSE)) {
                            u32ItemNo_InStart_OutEnd = g_MsgCtr.MsgBuf[i8MsgPt].u32ItemNo;
                            i32ItemNum_InNeed_OutRemain--;
                        } else {
                            DataAcsOpr = DATA_ACS_RES_FULL;
                            break;
                        }
                    }
                } while(i32ItemNum_InNeed_OutRemain);
        #endif
        #if ACQ_ITEM_DAT_NUM
            } else if(DataAcsOpr == DATA_ACS_READ_ACQ) {
                /* 搜索ram中的AcqNo最小的 */
                int16 iAcqPt;
                uint32 u32MinAcqNo = 0xFFFFFFFFUL;
                ACQ_WRITE_ITEM* pAcq = g_AcqCtr.AcqWriteBuf;
                for(i = ACQ_WRITE_BUF_LEN-1; i >= 0; i--) {
                    if((u32MinAcqNo > pAcq->u32ItemNo) && (pAcq->u32ItemNo > u32ItemNo_InStart_OutEnd)) {
                        u32MinAcqNo = pAcq->u32ItemNo;
                        iAcqPt = i;
                    }
                    pAcq++;
                }
                
                /* 提取数据 */
                if(u32MinAcqNo != 0xFFFFFFFFUL) {   /* 前面的搜索有结果 */
                    for(i = ACQ_WRITE_BUF_LEN; (i > 0) && i32ItemNum_InNeed_OutRemain; i--) {
                        if(g_AcqCtr.AcqWriteBuf[iAcqPt].u32ItemNo <= u32ItemNo_InStart_OutEnd) {    /* 已经找完了 */
                            break;
                        } else if(ConvertAcqCode2Text(DataUser, &pBuf, pBufEnd, &g_AcqCtr.AcqWriteBuf[iAcqPt])) {
                            u32ItemNo_InStart_OutEnd++;
                            i32ItemNum_InNeed_OutRemain--;
                            iAcqPt++;
                            if(iAcqPt >= ACQ_WRITE_BUF_LEN) {
                                iAcqPt = 0;
                            }
                        } else {
                            DataAcsOpr = DATA_ACS_RES_FULL;
                            break;
                        }
                    }
                }
        #endif
            }
        }

        /* 加上结尾 */
        if(DataUser == DATA_USER_NET) { /* 这里大约会需要3个byte，需要在一开始的时候预留出来 */
            pBuf--;
            if(*pBuf == '[') {          /* 说明内容是空的，为了避免'['被冲掉 */
                pBuf++;
            }
            *pBuf++ = ']';
            *pBuf++ = '}';
        
        /* 已经读完，但没完成期望条数，填充Gui剩余显示Buf */
#if SUPPORT_SCREEN
        } else if(DataUser == DATA_USER_MCGS) {
            for(i = labs(i32ItemNum_InNeed_OutRemain); (i != 0) && (pBuf + MCGS_LEN_ITEM_CHAR <= pBufEnd); i--) {
                *pBuf = 0;
                pBuf += MCGS_LEN_ITEM_CHAR;
            }
        } else if(DataUser == DATA_USER_VIEWTECH) {
            for(i = labs(i32ItemNum_InNeed_OutRemain); (i != 0) && (pBuf + VT_LEN_ITEM_CHAR <= pBufEnd); i--) {
                *pBuf = 0;
                pBuf += VT_LEN_ITEM_CHAR;
                *((uint16*)pBuf) = 0;   /* MsgID 或 AcqID字段 */
                pBuf += 2;
            }
#endif
        }
    }
    
    if(!DATA_ACS_RES(DataAcsOpr)) {
        if(i32ItemNum_InNeed_OutRemain) {
            DataAcsOpr = DATA_ACS_RES_EMPTY;
        } else {
            DataAcsOpr = DATA_ACS_RES_SUC;
        }
    }

    /* 网络任务，直接就在这里发布 */
    if(pReadMsgOrAcqCtr == NULL) {      /* 网络发布给数据库 */
	    g_MqttItemProc.u32ItemNo_OutEnd = u32ItemNo_InStart_OutEnd;
	    g_MqttItemProc.uContLen = pBuf - g_MqttItemProc.u8Buf;
        g_MqttItemProc.i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage = 1;
        Semaphore_post(SEM_MqttPubReq);

	} else if(pReadMsgOrAcqCtr->DataUser == DATA_USER_NET) {
	    g_MqttItemProc.u32ItemNo_InStart = pReadMsgOrAcqCtr->u32ItemNo_InStart_OutEnd;
	    g_MqttItemProc.u32ItemNo_OutEnd = u32ItemNo_InStart_OutEnd;
        g_MqttItemProc.iItemNum_InNeed = pReadMsgOrAcqCtr->i32ItemNum_InNeed_OutRemain;
        g_MqttItemProc.iItemNum_OutSuc = pReadMsgOrAcqCtr->i32ItemNum_InNeed_OutRemain - i32ItemNum_InNeed_OutRemain;
        g_MqttItemProc.DataAcsRes = DataAcsOpr;
	    g_MqttItemProc.uContLen = pBuf - g_MqttItemProc.u8Buf;
	    if(pReadMsgOrAcqCtr->DataAcsOpr == DATA_ACS_READ_MSG) {
            g_MqttItemProc.i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage = 2;
        } else if(pReadMsgOrAcqCtr->DataAcsOpr == DATA_ACS_READ_ACQ) {            
            g_MqttItemProc.i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage = 3;
        }
        pReadMsgOrAcqCtr->DataAcsOpr = DATA_ACS_IDLE;
	    Semaphore_post(SEM_MqttPubReq);

    /* 通过GetTextForMsgOrAcq()添加的任务，防止该任务已经被取消了 */
	} else if(READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr) || (pReadMsgOrAcqCtr->DataAcsOpr == DATA_ACS_RES_TIMEOUT)) {
        pReadMsgOrAcqCtr->pBuf = pBuf;
        pReadMsgOrAcqCtr->u32ItemNo_InStart_OutEnd = u32ItemNo_InStart_OutEnd;
        pReadMsgOrAcqCtr->i32ItemNum_InNeed_OutRemain = i32ItemNum_InNeed_OutRemain;
        pReadMsgOrAcqCtr->DataAcsOpr = DataAcsOpr;
    }
	Swi_enable();
}

/* 把一行的文件数据填进相应的Buf
Msg文本A   : 000035 18/06/20 11:29:46.230 电网相欠周,频率0.02794Hz 0041 7F1B7044
Msg文本B	 : 000035 18/06/20 11:29:46.230 电网相欠周,频率0.02794Hz 0041 0.02794 7F1B7044
Acq文本	 : 000065 18/06/20 11:29 6000 1.00 4.00 1.80 8.00 0.100 7F1B7044 */
BOOL ReadTextFromLineForMsgOrAcq_MustSwiDisable(DATA_USER DataUser, DATA_ACS_OPR DataAcsOpr, uint8** ppBuf, uint8* pBufEnd, uint8* pLineStart, uint8* pLineEnd, BOOL bCrcOK)
{
	uint8* pBuf = *ppBuf;
	int16 i;
	pLineEnd -= 2;	/* 去除最后的回车换行 */

	/* 2024.3.24注释，这段代码没用，已经测试过，但以防万一，还是留在这里 */
	/* Msg文本格式发生过升级，添加了msg_val字段，为了兼容新老版本，需要识别 */
	/* volatile BOOL bHaveMsgVal = (SkipCharInString(pLineStart, pLineEnd, ' ', 6) != pLineEnd); */
	
	/* 如果是UI读取，则需要抛弃前面的ItemNo, 后面的MsgID(如果有的话),CRC与回车换行,并添加0以表示字符的结尾 */
	if(0) {
#if SUPPORT_SCREEN
	} else if((DataUser == DATA_USER_MCGS) || (DataUser == DATA_USER_VIEWTECH)) {
		/* 获得行字符(英文)长度 */
		uint16 uItemLineByteLen = MCGS_LEN_ITEM_CHAR;
		if(DataUser == DATA_USER_VIEWTECH) {
			uItemLineByteLen = VT_LEN_ITEM_BYTE;
		}
		/* Buf空余长度判断 */
		if(pBufEnd - pBuf < uItemLineByteLen) {
			return FALSE;
		}
		pLineStart += 7;	/* 跳过MsgNo,AcqNo部分 */
		int32 i32LineReadByteNum;
		if(DataAcsOpr == DATA_ACS_READ_MSG) {
			/* 为了兼容新老版本Msg格式，不能用Acq的办法(有固定的尾部长度)，需要：跳过3个' ', 到MsgID然后退回一个' ' */
			i32LineReadByteNum = SkipCharInString(pLineStart, pLineEnd, ' ', 3) - pLineStart - 1;
		} else {	/* Acq只需要扔掉最后的CRC部分 */
			i32LineReadByteNum = pLineEnd - pLineStart - 9;		/* 9:8位CRC+一个空格 */
		}
		if(i32LineReadByteNum >= uItemLineByteLen) { 			/* 最多 uItemLineByteLen 长度 */
			i32LineReadByteNum = uItemLineByteLen;
			/* 对于UI相对简单，直接拷贝即可 */
			for(i = i32LineReadByteNum-1; i >= 0; i--) {		/* 抛弃最后的0 */
				pBuf[i] = pLineStart[i];
			}
		} else {
			/* 对于UI相对简单，直接拷贝即可 */
			for(i = i32LineReadByteNum-1; i >= 0; i--) {
				pBuf[i] = pLineStart[i];
			}
			pBuf[i32LineReadByteNum] = 0;
		}
		if(DataUser == DATA_USER_VIEWTECH) {
			if(DataAcsOpr == DATA_ACS_READ_MSG) {
				pLineStart = SkipCharInString(pLineStart, pLineEnd, ' ', 3);	/* 指向MsgID */
			} else if(DataAcsOpr == DATA_ACS_READ_ACQ) {
				pLineStart += 15;				/* 指向AcqID */
			}
			*((uint16*)(pBuf + VT_LEN_ITEM_CHAR)) = ReadH32(&pLineStart, pLineEnd);
		}
		pBuf += uItemLineByteLen;							/* 触摸屏数据，每个Item占用的长度是固定的 */
#endif
	/* 如果是以太网通讯读取，则需要进一步处理成json格式 */
	} else if(DataUser == DATA_USER_NET) {
		if(DataAcsOpr == DATA_ACS_READ_MSG) {
			/* 溢出检测：Msg从文本转成json需要补充额外的字符数:50+17(17是未完成消息的 ,"unfinish":"true")，多增加10个以备安全 */
			if(pBufEnd - pBuf < pLineEnd - pLineStart + 77) {
				return FALSE;
			}
	/*	Msg文本A	 : 000035 18/06/20 11:29:46.230 电网相欠周,频率0.02794Hz 0041 7F1B7044
		Msg文本B	 : 000035 18/06/20 11:29:46.230 电网相欠周,频率0.02794Hz 0041 0.02794 7F1B7044
	转成json格式
		{"no":"66","time":"20180620 11:29:48.750","text":"电网相欠周,频率0.02794Hz","msg_id":"0041","msg_val":"0.02794","cmplt":"false"},		*/
			/* no部分 */
			*pBuf++ = '{';
            for( ; (pLineStart < pLineEnd) && ((*pLineStart <= '0') || (*pLineStart > '9')); pLineStart++) {/* 跳过前面0以及非数字的部分 */
            }
            if(bCrcOK) {
    			PrintStringNoOvChk(&pBuf, "\"no\":\"");
    			while((pLineStart < pLineEnd) && ('0' <= *pLineStart) && (*pLineStart <= '9')) {
    				*pBuf++ = *pLineStart++;
    			}
                *pBuf++ = '"';
                *pBuf++ = ',';
			} else {
                while((pLineStart < pLineEnd) && ('0' <= *pLineStart) && (*pLineStart <= '9')) {
                    pLineStart++;
                }
			}
			pLineStart++;					/* 跳过空格部分 */
			/* time部分 */
			PrintStringNoOvChk(&pBuf, "\"time\":\"20");
			*pBuf++ = *pLineStart++;	/* 年 */
			*pBuf++ = *pLineStart++;
			pLineStart++;					/* 跳过\部分 */
			*pBuf++ = *pLineStart++;	/* 月 */
			*pBuf++ = *pLineStart++;
			pLineStart++;					/* 跳过\部分 */
			for(i = 15; i > 0; i--) {		/* 日 时:分:秒 */
				*pBuf++ = *pLineStart++;
			}
			pLineStart++;					/* 跳过空格部分 */
			uint8* pMsgVal = pLineStart;	/* 保留指向Msg开头的部分，用于后续可能的获取MsgVal */
			/* msg正文 */
			PrintStringNoOvChk(&pBuf, "\",\"text\":\"");
			while((pLineStart < pLineEnd) && (*pLineStart != ' ')) {
				*pBuf++ = *pLineStart++;
			}
			uint8* pMsgEnd = pLineStart;	/* 保留指向Msg结尾的部分，用于后续可能的获取MsgVal */
			pLineStart++;					/* 跳过空格部分 */
			/* msg_id部分 */
			PrintStringNoOvChk(&pBuf, "\",\"msg_id\":\"");
			while((pLineStart < pLineEnd) 		/* 十六进制表达 */
					&& ((('0' <= *pLineStart) && (*pLineStart <= '9')) || (('A' <= *pLineStart) && (*pLineStart <= 'F'))))
			{
				*pBuf++ = *pLineStart++;
			}
			BOOL bUnFinish = (*pLineStart == '*');	/* MsgID带*，说明不完整的消息 */
			if(bUnFinish) {	/* 跳过* */
				pLineStart++;
			}
			/* msg_val部分 */
			PrintStringNoOvChk(&pBuf, "\",\"msg_val\":\"");
			if(pLineEnd - pLineStart > 9) {	/* 判断为Msg文本B，msg_id后就是msg_val字段*/
				pLineStart++;				/* 跳过msg_val前面的那个 ' ' */
				while((pLineStart < pLineEnd) && (pBuf < pBufEnd) && (*pLineStart != ' ')) {/* 拷贝MsgVal */
					*pBuf++ = *pLineStart++;
				}
			} else {	/* Msg文本A,需要在消息文本中搜索MsgVal */
				while((pMsgVal < pMsgEnd)								/* 搜索MsgVal */
					&& (!((('0' <= *pMsgVal) && (*pMsgVal <= '9'))		/* 值 */
							|| ((*pMsgVal == '-') && (('0' <= pMsgVal[1]) && (pMsgVal[1] <= '9'))))))	/* 负号 */
				{
					pMsgVal++;
				}
				BOOL bFindNeg = FALSE;
				BOOL bFindExp = FALSE;
				BOOL bFindPoint = FALSE;
				while((pMsgVal < pMsgEnd) && (pBuf < pBufEnd)) {	/* 拷贝MsgVal */
					if(('0' <= *pMsgVal) && (*pMsgVal <= '9')) {
						*pBuf++ = *pMsgVal++;
					} else if((!bFindPoint) && (*pMsgVal == '.') && ('0' <= pMsgVal[1]) && (pMsgVal[1] <= '9')) {
						*pBuf++ = *pMsgVal++;
						bFindPoint = TRUE;
					} else if((!bFindNeg) && (*pMsgVal == '-') && ('0' <= pMsgVal[1]) && (pMsgVal[1] <= '9')) {
						*pBuf++ = *pMsgVal++;
						bFindNeg = TRUE;
					} else if((!bFindExp) && (*pMsgVal == 'e')) {
						bFindExp = TRUE;					
						bFindPoint = TRUE;	/* e指数后面不应该再有小数点 */
						if(('0' <= pMsgVal[1]) && (pMsgVal[1] <= '9')) {
							*pBuf++ = *pMsgVal++;
						} else if((pMsgVal[1] == '-') && ('0' <= pMsgVal[2]) && (pMsgVal[2] <= '9')) {
							bFindNeg = FALSE;
							*pBuf++ = *pMsgVal++;
						} else {
							break;
						}
					} else {
						break;
					}
				}				
			}
			if(bUnFinish) {
				PrintStringNoOvChk(&pBuf, "\",\"unfinish\":\"true");
			}
			if(pBuf + 3 > pBufEnd) {	/* 溢出检查 */
				return FALSE;
			} else {
				*pBuf++ = '"';
				*pBuf++ = '}';
				*pBuf++ = ',';
			}
		} else if(DataAcsOpr == DATA_ACS_READ_ACQ) {
			/* 溢出检测：Acq从文本转成json需要补充额外的31字符，多增加4个以备安全 */
			if(pBufEnd - pBuf < pLineEnd - pLineStart + 35) {
				return FALSE;
			}
			/* 把 000065 18/06/20 11:29 6000 1.00 4.00 1.80 8.00 0.100 7F1B7044 转化成json格式
				{"no":"65", "time":"20180703 00:07:00", "acq_id":"6000", "data":[1.00,4.00,1.80,8.00,0.100]},	*/
			/* no部分 */
			PrintStringNoOvChk(&pBuf, "{\"no\":\"");
			for( ; (pLineStart < pLineEnd) && (*pLineStart == '0'); pLineStart++) {
			}
			while((pLineStart < pLineEnd) && ('0' <= *pLineStart) && (*pLineStart <= '9')) {
				*pBuf++ = *pLineStart++;
			}
			pLineStart++;					/* 跳过空格部分 */
			/* time部分 */
			PrintStringNoOvChk(&pBuf, "\",\"time\":\"20");
			*pBuf++ = *pLineStart++;		/* 年 */
			*pBuf++ = *pLineStart++;
			pLineStart++;						/* 跳过\部分 */
			*pBuf++ = *pLineStart++;		/* 月 */
			*pBuf++ = *pLineStart++;
			pLineStart++;						/* 跳过\部分 */
			for(i = 8; i > 0; i--) {			/* 日 时:分 */
				*pBuf++ = *pLineStart++;
			}
			pLineStart++;						/* 跳过空格部分 */
			/* time秒部分+acq_id */
			PrintStringNoOvChk(&pBuf, ":00\",\"acq_id\":\"");
			while((pLineStart < pLineEnd) && (*pLineStart != ' ')) {
				*pBuf++ = *pLineStart++;
			}
			/* data部分 */
			PrintStringNoOvChk(&pBuf, "\",\"data\":[");
			for(i = 5; i > 0; i--) {
				pLineStart++;					/* 跳过空格部分 */
				*pBuf++ = '"';
				while((pLineStart < pLineEnd) && (*pLineStart != ' ')) {
					*pBuf++ = *pLineStart++;
				}
				*pBuf++ = '"';
				*pBuf++ = ',';
			}
			pBuf--;
			*pBuf++ = ']';
			*pBuf++ = '}';
			*pBuf++ = ',';
		}
	}
	
	*ppBuf = pBuf;
	return TRUE;
}

/* Mqtt访问Msg, Acq */
void RegisterPageReq(DATA_PAGE DataPage, uint8* pU8Msg, uint8* pU8MsgEnd)
{
    /* 数据库消息指针初始化 */
    if((DataPage == DATA_PAGE_MSG) && CompareMsg(pU8Msg+2, "msgpt")) {  /* 指令应该是{"msgpt":"**"} */
        g_MqttItemProc.u32MsgNo_HavePubedForDB = ReadU32(&pU8Msg, pU8MsgEnd);
        Semaphore_post(g_DataAccessBlock.Sem_Op);		//        Semaphore_post(SEM_NvMemAcs);
        return;
    }

    /* 页面访问：接口处理 */
    pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 3);
    uint32 u32ItemNo = 0xFFFFFFFFUL;    /* 对于接口函数来说, 0xFFFFFFFFUL代表初始化 */
    int32 i32ItemNum = 0;
    if(!CompareMsg(pU8Msg, "ini")) {
        u32ItemNo = ReadU32(&pU8Msg, pU8MsgEnd);
    }
    GetI32(&pU8Msg, pU8MsgEnd, &i32ItemNum, 0);

    /* 页面访问：请求注册 */
	if(i32ItemNum) {
        int8 i8ReadTaskCtrNo;
        Swi_disable();
        for(i8ReadTaskCtrNo = MAX_READ_MSG_ACQ_NUM - 1; i8ReadTaskCtrNo >= 0; i8ReadTaskCtrNo--) {
            if(g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr == DATA_ACS_IDLE) {
                g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].u32ItemNo_InStart_OutEnd = u32ItemNo;
                g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].pBuf = NULL;
                g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].pBufEnd = NULL;
                g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].i32ItemNum_InNeed_OutRemain = i32ItemNum;
                g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataUser = DATA_USER_NET;
                g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr = (DATA_ACS_OPR)(DATA_ACS_READ_DATA_PAGE - DataPage);
                Semaphore_post(g_DataAccessBlock.Sem_Op);		//                Semaphore_post(SEM_NvMemAcs);
                break;
            }
        }
        Swi_enable();
    }
}

/* 发布Msg,Acq */
void PrintMsgAcqPageRes(uint8** ppTxBuf);
BOOL PubMsgAndAcq(MQTT_COMM* pMqttComm)
{
    /* 锁存 */
    Swi_disable();
	int8 i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage = g_MqttItemProc.i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage;
	uint16 uContLen = g_MqttItemProc.uContLen;
    Swi_enable();
    
	if(uContLen > MQTT_ITEM_PROC_BUF_BLEN) { /* 避免Buf操作溢出数据超长 */
	    g_MqttItemProc.uContLen = 0;
        g_MqttItemProc.i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage = 0;
	} else if(i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage > 0) {
		/* 内容 */
		uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
		uint8 u8QoS = 0;			/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
		/* 打印Topic */
		PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
		if(i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage == 1) {
            PrintStringNoOvChk(&pTxBuf, "/MSG");
	    } else if(i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage == 2) {
            PrintStringNoOvChk(&pTxBuf, "/PAGE/MSG");
	    } else if(i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage == 3) {
            PrintStringNoOvChk(&pTxBuf, "/PAGE/ACQ");
	    }
		uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
		if(u8QoS) {
			pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
		}
		memcpy(pTxBuf, g_MqttItemProc.u8Buf, uContLen);
		pTxBuf += uContLen;
		/* 配置页面发布:传输 */
		if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) {	/* QoS修改必须在本段代码开始的位置 */
			return FALSE;
		}
		
		/* 页面：结果 */
		if(i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage == 1) {
			g_MqttItemProc.u32MsgNo_HavePubedForDB = g_MqttItemProc.u32ItemNo_OutEnd;
		} else { /* 只有页面操作才有结果 */
    		pTxBuf = pMqttComm->u8TRBuf + 5; /* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
    		u8QoS = 0;			/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
    		/* Msg结果发布:打印Topic */
    		PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
    		if(i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage == 2) {
        		PrintStringNoOvChk(&pTxBuf, "/RES/MSG");
    		} else {
        		PrintStringNoOvChk(&pTxBuf, "/RES/ACQ");
        	}
    		uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
    		if(u8QoS) {
    			pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
    		}
            PrintMsgAcqPageRes(&pTxBuf);
    		/* Msg页面结果发布:传输 */
    		if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) {	/* QoS修改必须在本段代码开始的位置 */
    			return FALSE;
    		}
		}

        g_MqttItemProc.i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage = 0;
#if MAX_MSG_TYPE
        if(g_MqttItemProc.u32MsgNo_HavePubedForDB < g_MsgCtr.u32MsgNo_Cmplt) {
            Semaphore_post(g_DataAccessBlock.Sem_Op);		//            Semaphore_post(SEM_NvMemAcs);
        }
    } else if(g_MqttItemProc.u32MsgNo_HavePubedForDB == 0xFFFFFFFFUL) {
        /* 内容 */
        uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
        uint8 u8QoS = 0;            /* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
        /* 打印Topic */
        PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
        PrintStringNoOvChk(&pTxBuf, "/REQ/MSGPT");
        uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
        if(u8QoS) {
            pTxBuf += 2;    /* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
        }
        PrintStringNoOvChk(&pTxBuf, "{\"req\":\"msgpt\"}");
        /* 配置页面发布:传输 */
        if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) {  /* QoS修改必须在本段代码开始的位置 */
            return FALSE;
        }
#endif
	}

	return TRUE;
}

void PrintMsgAcqPageRes(uint8** ppTxBuf)
{
    uint8* pTxBuf = *ppTxBuf;
    *pTxBuf++ = '{';    /* Json开始 */
    if(g_MqttItemProc.u32ItemNo_InStart == 0xFFFFFFFFUL) {
        PrintStringToJson(&pTxBuf, "no", "ini");
    } else {
        PrintU32DatToJson(&pTxBuf, "no", g_MqttItemProc.u32ItemNo_InStart, 0);
    }
    if(g_MqttItemProc.DataAcsRes == DATA_ACS_RES_SUC) {
        PrintStringToJson(&pTxBuf, "res", "success");
    } else if(g_MqttItemProc.DataAcsRes == DATA_ACS_RES_FULL) {
        PrintStringToJson(&pTxBuf, "res", "full");
    } else if(g_MqttItemProc.DataAcsRes == DATA_ACS_RES_EMPTY) {
        PrintStringToJson(&pTxBuf, "res", "empty");
    } else {
        PrintStringToJson(&pTxBuf, "res", "fail");
    }
    PrintI32DatToJson(&pTxBuf, "req", g_MqttItemProc.iItemNum_InNeed, 0);
    PrintI32DatToJson(&pTxBuf, "get", g_MqttItemProc.iItemNum_OutSuc, 0);
    PrintU32DatToJson(&pTxBuf, "end", g_MqttItemProc.u32ItemNo_OutEnd, 0);
    pTxBuf--;
    *pTxBuf++ = '}';    /* Json结尾 */

    *ppTxBuf = pTxBuf;
}
/******************************** FILE END ********************************/
