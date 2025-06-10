/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved. ģ��V1.3
 *
 * File			: MdlDataAccess.c
 * Author		: Wang Renfei
 * Description	: ��Ҫ����: ���ݴ�ȡ������(TF��,U��,EEPROM)��֧��ͨѶ/��ʾ���ʽӿ�(��Ҫ��ItemPage)������������������(���á�ͳ�ơ���¼���ɼ�)������

 	�����ļ�ϵͳ�����ĵ��ڴ�ϴ󣬶���Ҫ�����ļ���task�϶࣬Ϊ�˱���ÿ��task��Ϊ�ļ�ϵͳ�����ڴ棬��˰��ļ����ʼ�����һ��task��ʵ��
 	����task�����ļ���ͨ���ύ����ʵ�֣�ʵ�ֺ�����Ҫ��Ϊ������:
 	1. �ⲿ�ӿڵ�ע������(���д����)���������������
 	2. ��������ʵ��--ͨ�����������ʵ��
 	3. �����ຯ�����������񶼻�ʹ��
 
	�ⲿ����ģ��:
 	1. ���ݴ�ȡ����Ŀ(Item)Ϊ��Ԫ��һ����ĿΪһ��
	2. ������ͳ�ƣ�ͨ�� CONF_n_STAT_PROP ������Ŀ�� ��ʽ/��ʾ��/����/�ı�/��Ŀ����ֵ����Ϊram �� �ļ�ϵͳ���� �Ľӿ�
	3. ��¼��ͨ�� MSG ����ÿһ����Ϣ
	4. �ɼ���ͨ�� ** ����

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
/* ����ϵͳͷ�ļ� */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

/* ��Ŀ����ͷ�ļ� */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"
#include "MdlNet.h"

/* �����Լ��¼�ģ��ͷ�ļ� */
#include "MdlDataAccess.h"
#include "BoardSupport.h"

/* ����ģ�� */
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
	uint32 u32ItemNo_InStart_OutEnd;		/* ������ʼ�к�(��)���������һ��������¼���к� */
	int32 i32ItemNum_InNeed_OutRemain;		/* ��:���кŶ�����:���кŶ�; ����ʣ�����Ŀ���� */
}READ_MSG_ACQ_TXT;

/* msg,acq��ȡ��ʽ */
typedef enum {
    MSG_ACQ_ACS_V1 = 0,                   /* ÿ��һ���ļ�������/��/��/��.txt������֯ */
    MSG_ACQ_ACS_V2 = 1,                   /* ÿ10������Ϣһ���ļ�������/sn.txt������֯ */
}MSG_ACQ_ACS_VER;

#define FILE_CONT_BUF_LEN			10240   /* ��������2������ 1024byte */
#define FILE_PATH_BUF_LEN			80
#define FILE_NAME_BYTE_LEN			13				/* 8.3��ʽ */

#define MSG_ACQ_FILE_PATH_DEPTH		3
#define MAX_FILE_PATH_DEPTH			(MSG_ACQ_FILE_PATH_DEPTH + 1)

typedef struct {
	/* ���ʽӿ� */
	READ_MSG_ACQ_TXT ReadMsgOrAcqTxt[MAX_READ_MSG_ACQ_NUM];
	BOOL bNeedSaveAcq;
	BOOL bNeedSaveMsg;
	uint16 uRsvd;
	
	/* SPI���--���λ�ñ������ g_NvMemAcsCtr.u8FilePath */
    SDSPI_Handle sdspiHandle;	/* ����������λ�ã���֮ǰ(����)����Ҫ����ʼ��Ϊ0 */

    /* Buf�����ʼ��Ϊ0�� */
	uint8 u8FilePath[FILE_PATH_BUF_LEN];	/* �ļ�Ŀ¼�������ļ�Ŀ¼����û���������飬��˱�������ļ�Buf֮ǰ����һ���Ӱ�첻�� */
	uint8 u8FileCont[FILE_CONT_BUF_LEN];	/* �ļ����� */
#if SUPPORT_FAT_FS
	uint8 fatfsTempSpace[_MAX_SS];			/* fatfs��ʱ�����ռ� */
#endif
}NVMEM_ACS_CTR;
SECTION(".NOT_ZeroInit") NVMEM_ACS_CTR g_NvMemAcsCtr;

#define CONF_SAVE_NAME_BLEN		(28*2)

/* �ж�EEPROM��ַ�Ƿ��ںϷ���SAVE_GRP�洢��Χ�� */
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
/* ��ʼ�� */
void InitDataAccess(void);
void InitPowerOnData(void);
void FixSoftVerAndConf(void);
#ifdef BOARD_SDSPI
BOOL TestOfTFCard(void);        /* tf�����ԣ����������ʹ�� */
#endif

/* ϵͳ����:��λMsg,Acq,Stat���� */
void ResetMsgData(void);
void ResetAcqData(void);
void ResetStatData(void);

/* �ļ������ɡ�Ŀ¼��麯�� */
#if SUPPORT_FAT_FS
void ChkAndMkDir(void);
void EmptyFold(uint8* pFilePathIni);
#endif

/* Conf������ */
void SaveConfToFile(CONF_SAVE_GROUP ConfAcsGrp);
DATA_ACS_OPR ReadOrCheckConfFromFile(CONF_SAVE_GROUP ConfAcsGrp, int16 iTryCnt_pRd_nChk);
void InitConfWithDefault(void);
uint8 CalDataBLenFromItemDataType(ITEM_DATA_TYPE DataType);
void SaveConfToEEProm(CONF_SAVE_GROUP AcsGrp, uint32 u32EEPromByteAddr);
BOOL CheckConfFromEEProm(CONF_SAVE_GROUP AcsGrp, uint32 u32EEPromByteAddr);
DATA_ACS_OPR ReadConfFromEEProm(CONF_SAVE_GROUP AcsGrp);

/* Msgģ�麯�� */
void InitMsgMdl(void);
void SaveMsgToNvMem(void);

/* Acqģ�麯�� */
void InitAcqMdl(void);
void SaveAcqToNvMem(void);
void RunAcqSummTask(void);

/* Msg,Acqģ�鹫������ */
#if SUPPORT_FAT_FS
uint32 ReadLastItemNoFromFile(DATA_PAGE DataPage, MSG_ACQ_ACS_VER eReadItemVer);
#endif
void ReadItemNoAndEEPromNo(DATA_PAGE DataPage);
void ReadTextForMsgOrAcq(READ_MSG_ACQ_TXT* pReadMsgOrAcqCtr);

/* ���¶�����(�������ļ���)����pTextEndָ���β��һ��(���ɴ洢)�ĵ�ַ */
uint32 ReadH32FixLen(uint8* pText);
uint32 ReadU32FixLen(uint8* pText);
float32 ConvTextToF32_RightStart(uint8** ppText, uint8* pTextStart);

/***************************************************************************
 							functions definition
***************************************************************************/
/*==========================================================================
| Description	: ģ���ʼ��
| G/Out var		:
| Author		: Wang Renfei			Date	: 2008-3-4
\=========================================================================*/
void InitDataAccess(void)
{
	/* ��ʼ������ */
	/* <NULL> */

	/* ��ʼ������ӿڱ��� */
	/* <NULL> */

	/* ��ʼ������ӿڱ��� */
	/* <NULL> */

	/* ��ʼ���ڲ�ȫ�ֱ��� */
	InitDataWithZero((uint8*)(&g_NvMemAcsCtr), (uint8*)(&g_NvMemAcsCtr.sdspiHandle) - (uint8*)(&g_NvMemAcsCtr));

	/* ����Ӳ�� */
	/* ��TF�� */

#ifdef BOARD_SDSPI
	InitSDSPI();
//TI::InitSDSPI()
//	SDSPI_Params sdspiParams;
//	sdspiParams.bitRate = 12000000UL;
//	sdspiParams.custom = NULL;
//	g_NvMemAcsCtr.sdspiHandle = SDSPI_open(BOARD_SDSPI, 0, &sdspiParams);
#endif
    /* TF������ */
#if SUPPORT_FAT_FS && (!SOFT_RUN1_TEST0)
    g_DataAcsIntf.bTestRes_TFCard = TestOfTFCard();
#endif

	/* ��ʼ���²�ģ�� */
#if MAX_MSG_TYPE
	InitMsgMdl();
#endif
#if ACQ_ITEM_DAT_NUM
	InitAcqMdl();
#endif
}

/*==========================================================================
| Description	: �ϵ��ʼ��
	1. ���Ŀ¼
	2. ��ȡ�����License�Ƿ���
	3. ��ȡ��������(������ͳ����Ϣ��ʼ��)
	3.1 ��̬��������
	3.2 �¶ȴ�����ͨѶ���߼���������??
	3.3 Acq�Ѿ��ܽ�洢�Ĳ��֣����������вɼ�����
	4 ��ȡͳ������

	����License��������������(���á�ͳ����)��ȡ����:
	1. ���������ļ�����������ҵ����ȡ������������У��
	2. �������У��������ٶ�һ����������������У��
	3. ���δ���������ļ�����������������������У�����������BackUp�ļ�������ı���
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-06-21
\=========================================================================*/
#define MAX_READ_BACKUP_TRY_COUNT	5
void InitPowerOnData(void)
{
	/* 0. ��ʼ����Ĭ�ϱ��� */
	InitConfWithDefault();
	g_DataAcsIntf.eConfInitRes[SAVE_GRP_NUL] = DATA_ACS_IDLE;			/* Ϊ�˷��ʷ��㣬ǰ�������˵�0���յķ��ʱ�־ */
	g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt = 0;					/* ��ʼ�� */

	CONF_SAVE_GROUP ConfAcsGrp;
#if SUPPORT_FAT_FS
	/* 1. ���Ŀ¼�ṹ�Ĵ��� */
	ChkAndMkDir();

	/* 2. ��ȡ���洢��eeprom�е�������ͳ������ */
	g_DataAcsIntf.eConfInitRes[SAVE_GRP_BRD] = ReadConfFromEEProm(SAVE_GRP_BRD);

	/* 3. ��ȡ�洢��(�ļ�+eeprom)�е�������ͳ������ */
	for(ConfAcsGrp = SAVE_GRP_MCONF; ConfAcsGrp < MAX_SAVE_GRP_NUM; ConfAcsGrp++) {
		/* ���������������� */
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
	
	/* 4. �������ñ�����ʼ�� */
	if(g_Sys.SerialNo.u32Dat < 10000000UL) {
		g_Sys.SerialNo.u32Dat = CalcUid16B();
	}

	/* 5. ������ÿ��Է��� */
	for(ConfAcsGrp = SAVE_GRP_BRD; ConfAcsGrp < MAX_SAVE_GRP_NUM; ConfAcsGrp++) {
		g_DataAcsIntf.bConfFree[ConfAcsGrp] = TRUE;

	#if 0	/* ͳ��eepromռ�ã��°汾�������˱��������Դ򿪱��뿪�أ�ƽʱ�ر� */
		/* ͳ�Ƹô洢���ʹ�õ�eeprom��С */
		int16 i;
		CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
		for(i = pBootLoaderConf->u32ItemNum; i > 0; i--) {
			if(pDataAcsProp->ItemType.SaveGrp == ConfAcsGrp) {
				g_CodeTest.uVal3[ConfAcsGrp*2 + 0] += CalDataBLenFromItemDataType(pDataAcsProp->ItemType.DataType);
			}
			pDataAcsProp++;
		}
		if(g_CodeTest.uVal3[ConfAcsGrp*2 + 0]) {			/* ������ */
			g_CodeTest.uVal3[ConfAcsGrp*2 + 0] += 4;		/* ���CRC��4byte */
			/* ����ô洢�������eeprom��С */
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
| Description	: ���ݴ�ȡ����

	���ݴ洢��������ģʽ: �ļ�(tf��)��eeprom, ���в���

	��ȡʱ��	�ϵ�洢���ȼ�		�����洢			�ϵ��ȡ˳��		ͨѶ��ȡ
		conf		1(�����)	�޸����õ�ʱ��				1			��������ļ���ֱ����װ
		Summ		2			�޸��ۺϵ�ʱ��				2			��������ļ���ֱ����װ
		acq			4			Buf����һ��������ʱ��		3			�ļ��е��ı������ڴ��б�����Ϣ
		msg			3			Buf����һ��������ʱ��					�ļ��е��ı������ڴ��б�����Ϣ

	�洢λ��				�ļ�(tf��)												eeprom
	conf,stat		һ��һ���������ļ�����ʱ���							�𵽱������ã�conf���������ݼ�¼
	conf_�¶ȴ�����	ͬ��													ͬ��
	conf_����,SN��	��Ҫ�����ݵ������㿴�����ã����洢��ʱ����Ҫ���	�������ݣ�һ��һ����Ҫ��CRC32У��
	acq,msg			������һ�����ݣ�����ÿ��һ���ļ��Խ����𻵸���			������
	acq,msg_ItemNo	�Լ�¼�а���Ϊ���������һ���ļ��ж��������������		�γ�һ��ring��ÿ��ˢ����СItemNo����ȡ��ʱ��ȡ���ֵ
	acq_�ܽ�		һ��һ������ʱ(����һ����?)�����γ�						�𵽱�������
	����ļ���eeprom��������ݲ�һ�£�����У����ȷ���ݣ����У�鶼��ȷ����������/��ԭ���޸ı��������Ա���һ�£�

���á�ͳ�����ݴ�ȡ
	�������á�ͳ�����ݴ洢(�½�һ���ļ�����������������д��)����:
		1. �½��հ��ļ���������/ͳ�����ݽ��д洢�������ļ������У��(crc32)���������ݣ�һ����BackUp�ļ����У�������ʱ���(yymmddhh)
		2. �ѱ�����ļ���ԭʼ�������Աȣ����һ������Ϊ�洢�ɹ�������ɾ���ļ��ظ�1
	�������á�ͳ�������ݶ�ȡ����:
		1. ���������ļ�����������ҵ����ȡ������������У��
		2. �������У��������ٶ�һ����������������У��
		3. ���δ���������ļ�����������������������У�����������BackUp�ļ�������ı���

Msg,Acq���ݴ�ȡ
	����Msg, Acq���ݴ洢(���ڽضϣ��γ����ļ���������������ļ�ĩβ)����:
		1. �γ�ÿ��һ���ļ����Խ����ļ��𻵸���,��������/�½����ļ��������ļ�������ʽ:ddnnnnnn�����ļ���mmnnnnnn, ���ļ���yynnnnnn
		2. ÿ���¼һ��У��
	����Msg, Acq���ݶ�ȡ(���ڽضϣ��γ����ļ���������������ļ�ĩβ)����:
		1. ���������ʽ��ItemNo, ������ύ�ķ�����ʽ�������ļ��У�ȷ���ļ����ٷ����ļ��Ի�þ�����ı�
		2. ���������ʽ��ʱ�䣬��ֱ�Ӳ����ļ���/�ļ�������

���ݱ��ݡ��ָ�
	���� : �����ݴ�tf��������U�̣�����U�� Yoplore\NNNNNNNN\YYMMDDHH �ļ����У�����NNNNNNNN�����к�, YYMMDDHH�Ǳ�������
		1. �򵥱���: conf/stat���ݲ�����BackUp�ļ��У�Acq/Msg���ݿ���ȫ��
		2. ȫ������: conf/stat���ݿ���BackUp�ļ��У�Acq/Msg���ݿ���ȫ��
	�������� : ���뱸�����뼴�ɲ�������TF���ϵ����ݱ��ݵ�U����

	�ָ� : �����ݴ�U�̿�����tf����
		1. �򵥻ָ�: conf/stat���ݲ�����BackUp�ļ��У�Acq/Msg���ݿ���ȫ��
		2. ȫ������: conf/stat���ݿ���BackUp�ļ��У�Acq/Msg���ݿ���ȫ��
	�������� : ������������кš�����ʱ�䣬���û�����룬��ѡ�����µı���



��Ҫ����
	1. Msg, Acq�����һ��ItemNo
	2. Conf, Stat��ֵ
	
	
| G/Out var		:
| Author		: Wang Renfei			Date	: 2015-9-27
\=========================================================================*/
void DataAccessTask(void const * argument)
{
	InitDataAccess();
	if(g_Sys.uRstCount == 0) {	/* �ϵ縴λ���Ž��д˲��� */
		InitPowerOnData();
		FixSoftVerAndConf();
	}

	for(;;) {
		Task_setPri(g_DataAccessBlock.DataAcsTSK, TASK_LOWEST_PRIORITY); /* ���ϵ��ʱ�����ȼ�������ߣ��Ա㾡���ʼ�����ñ����������洢Ҳ��������ȼ�����ɺ󽵵�Ϊ2 */
		if(Semaphore_pend(g_DataAccessBlock.Sem_Op, BIOS_WAIT_FOREVER)){
		}
		PersistSaveGrp();		/* ���û���ͳ�����ݣ�������(�ļ�+EEPROM)������ */
		/* ���������Զ�̵����ô洢������ */
#if CPU_0ST_1TI == 0
		CONF_SAVE_GROUP ConfAcsGrp;
#endif
	    Swi_disable();
		for(ConfAcsGrp = SAVE_GRP_BRD; (ConfAcsGrp < MAX_SAVE_GRP_NUM) && (!g_DataAcsIntf.bConfHaveUnsavedChange[ConfAcsGrp]); ConfAcsGrp++) {
		}
		BOOL bMqttConf = (g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP > 0)
		                || (g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP > 0);
		if(ConfAcsGrp < MAX_SAVE_GRP_NUM) {		/* ��δ��������ݣ�������һ�� */
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

	#if MAX_MSG_TYPE		/* Msg���ݱ��� */
		if(g_NvMemAcsCtr.bNeedSaveMsg || (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0)) {
			SaveMsgToNvMem();
			g_NvMemAcsCtr.bNeedSaveMsg = FALSE;
		}
	#endif

	#if ACQ_ITEM_DAT_NUM	/* Acq���ݴ�ȡ */
		RunAcqSummTask();				/* Acq���ݶ�ȡ���ۺϴ��� */
		if(g_NvMemAcsCtr.bNeedSaveAcq || (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0)) {
			SaveAcqToNvMem();
			g_NvMemAcsCtr.bNeedSaveAcq = FALSE;
		}
	#endif

		/* Msg, Acq�ı���ȡ */
		int8 i;
		for(i = 0; (i < MAX_READ_MSG_ACQ_NUM) && (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt <= 0); i++) {
			if(DATA_ACS_MISSION(g_NvMemAcsCtr.ReadMsgOrAcqTxt[i].DataAcsOpr)) {
				ReadTextForMsgOrAcq(&g_NvMemAcsCtr.ReadMsgOrAcqTxt[i]);
			} else if(g_NvMemAcsCtr.ReadMsgOrAcqTxt[i].DataAcsOpr == DATA_ACS_END) {	/* ���������Ѿ����������� */
				g_NvMemAcsCtr.ReadMsgOrAcqTxt[i].DataAcsOpr = DATA_ACS_IDLE;
			}
		}
	    ReadTextForMsgOrAcq(NULL);  /* NULLָ���������ݿⷢ����Ϣ������ */
	
		/* ϵͳ���� */
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
            Semaphore_post(g_DataAccessBlock.Sem_Op);   // Semaphore_post(SEM_NvMemAcs); 	/* ������������Ϊ�����洢�������������������һ�� */
		}
	}
}

#ifdef BOARD_SDSPI
/* TF����д���� */
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
 * �ⲿ����
 1. SaveStatMsgAcq() ��ͣ��ʱ���־û���ǰram�е�����
 2. UrgentSaveNvData() �豸����ʱ�����浱ǰ����
 3. ChkAndWaitInitConf() �ṩ���������񣬿����������Ƿ��ʼ�����
 4. RegisterDataOperation() �ṩϵͳҳ�����: ͬ�����á�������ݵ�
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
	/* �����洢 */
	g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt = 1;
	Semaphore_post(g_DataAccessBlock.Sem_Op);				//Semaphore_post(SEM_NvMemAcs);
	Task_setPri(g_DataAccessBlock.DataAcsTSK, 15);			//	Task_setPri(TSK_DataAcs, 15);			/* ���ߴ洢�������ȼ������ */

	/* �ȴ���� */
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
    } else if(SysFunc == SYS_FUNC_ENG_DEBUG) {  /* ���̵��� */
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
 * ���浱ǰʱ��
 *========================================================================*/
/* ��ʼ��������Ĭ��ֵ */
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
						if((pPageDataAcsProp->pData == NULL) 	/* ȷ��λ���� */
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
					*((uint32*)pDataAcsProp->pData + 1) = 0;    /* ITEM_DAT_ACQ_U32����Ӧ���Ǹ�����������Ϊ����������������0 */
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

/* �޸�����汾�ź�����ȱʧ */
void FixSoftVerAndConf(void)
{
	/* ���汾 */
	if(CheckSoftVer()) {
		g_PubSoftAuthCtr.uTmr_IniPubCntNoPwd_ms = 60000;	/* �ɳ�ΪΪĸ�� */
	}
	/* �ѳ�ʼ��ʧ�ܵı�������Ĭ��ֵ���߲��ֳ�ʼ���ɹ���ֵ���д洢 */
	CONF_SAVE_GROUP AcsGrp;
	for(AcsGrp = SAVE_GRP_BRD; AcsGrp < MAX_SAVE_GRP_NUM; AcsGrp++) {
		g_DataAcsIntf.bConfHaveUnsavedChange[AcsGrp] = (g_DataAcsIntf.eConfInitRes[AcsGrp] != DATA_ACS_RES_SUC);
	}
}

/*==========================================================================
| Description	: ����������Ŀ���ݳ���
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
| Description	: ���Stat����
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
	/* Ϊ�˱��ּ����ԣ�֮ǰ�汾 u32CrcForEEPromֵ��0xFFFFFFFF��AcsMedia[0].u32OrigEEAddr��0���������ط���ʼ����u32OrigEEAddr-1 */
	uint32 u32CrcForEEProm = pBootLoaderConf->AcsMedia[0].u32OrigEEAddr - 1;
	uint32 u32EEPromByteAddr = pBootLoaderConf->AcsMedia[SAVE_GRP_STAT].u32OrigEEAddr;
	int16 ItemNum = pBootLoaderConf->u32ItemNum;
	int16 ItemNo;

    EEPromErase(u32EEPromByteAddr);
	/* ��ʼ��������д��EEPROM */
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
	/* EEPROM Crc���� */
	EEPROMProgram((uint32_t*)(&u32CrcForEEProm), u32EEPromByteAddr, 4);
#endif

#if SUPPORT_FAT_FS
	/* ��������Ŀ¼ */
	uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/CONF/STAT.txt");
	*pFilePath = 0;

	/* д���ļ� */
	FIL File;
	pDataAcsProp = pBootLoaderConf->pConfProp;
	if(f_open(&File, (const TCHAR*)g_NvMemAcsCtr.u8FilePath, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)	{
		return;
	}

	UINT u32SavedByteNum, u32ByteNumToBeSaved;			/* �ɹ�������ֽ��� */
	uint32 u32CrcForFile = 0xFFFFFFFF;
	uint8* pBuf;
	for(ItemNo = ItemNum; ItemNo > 0; ItemNo--) {
		if((pDataAcsProp->ItemType.SaveGrp == SAVE_GRP_STAT) && (pDataAcsProp->pData != NULL)) {
			pBuf = g_NvMemAcsCtr.u8FileCont;

			/* 0. д��ItemID */
			PrintU32(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], pDataAcsProp->u32ItemID, 0);
		
			/* 1. д����ʾ������ */
			if(g_Sys.u32LocalLang >= MAX_LANG_TYPE) {
				g_Sys.u32LocalLang = CHINESE;
			}
			PrintString(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], pDataAcsProp->pName[g_Sys.u32LocalLang]);
			for( ; pBuf < &g_NvMemAcsCtr.u8FileCont[CONF_SAVE_NAME_BLEN]; pBuf++) {
				*pBuf = ' ';
			}

			/* 2. ������������д������ */
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

			/* 3. �س����� */
			*pBuf = 0x0D;
			*(pBuf + 1) = 0x0A;
			pBuf += 2;

			/* 4. ����CRC�����浱ǰ�� */
			u32ByteNumToBeSaved = pBuf - g_NvMemAcsCtr.u8FileCont;
			u32CrcForFile = CalCRC32ForFileByIni(g_NvMemAcsCtr.u8FileCont, u32ByteNumToBeSaved, u32CrcForFile);
			f_write(&File, g_NvMemAcsCtr.u8FileCont, u32ByteNumToBeSaved, &u32SavedByteNum);
		}

		/* 6. ׼����һ�� */
		pDataAcsProp++;
	}

	/* ���У�飬�����桢�ر��ļ� */
	pBuf = g_NvMemAcsCtr.u8FileCont;
	PrintString(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], "У����(����ֶ��޸ı��ļ�,���ð�ź��У�����Ϊ12345678):");
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
* EEProm��ȡ�������ݣ���Ҫ�������º�����
* 1. SaveConfToEEProm() ����������д��EEProm
* 2. CheckConfFromEEProm() ���EEProm�е����ݺ��ڴ��е��Ƿ�һ��
* 3. ReadConfFromEEProm() ��EEProm�ж�ȡ��������
* 4. CheckConfValidFromEEProm() ���EEProm�е��������������ԣ���ReadConfFromEEProm()�����ж���������������
\=========================================================================*/
/*==========================================================================
| Description	: ��������ֵ�����Զ����Ƶ���ʽ�洢��eeprom�У�������crc32У��
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
#define CONF_ITEM_MAX_DATA_U32LEN 	(CONF_ANSI_BYTE_LEN/4)	/* ���������������ĿΪ׼ */
#define CONF_EEDATA_BY_HIGH_FLASH	0xAAAAAAAAUL	/* eeprom�д洢�����������Ǹ�һ��FLASH�����д�� */
#define CONF_EEDATA_BY_LOW_FLASH	0x55555555UL	/* eeprom�д洢�����������ǵ�һ��FLASH�����д�� */
void SaveConfToEEProm(CONF_SAVE_GROUP AcsGrp, uint32 u32EEPromByteAddr)
{
	CONF_n_STAT_PROP* pDataAcsProp;
	uint32_t u32DataBuf[CONF_ITEM_MAX_DATA_U32LEN];
	uint32 u32ByteLen, u32ItemNum, u32FlashBankFlag;
	/* Ϊ�˱��ּ����ԣ�֮ǰ�汾 u32CrcForEEPromֵ��0xFFFFFFFF��AcsMedia[0].u32OrigEEAddr��0���������ط���ʼ����u32OrigEEAddr-1 */
	uint32 u32CrcForEEProm = pBootLoaderConf->AcsMedia[0].u32OrigEEAddr - 1;
	int16 i;

#if LEGACY
	if((u32EEPromByteAddr == 0) || (u32EEPromByteAddr >= 96*64)) {			/* ������ */
		return;
	}
#else
	/* �����ַ��Χ��� */
	if(!WITHIN_SAVE_GRP_EEPROM_ADDR(u32EEPromByteAddr)) {
		return;
	}
#endif

#if CPU_0ST_1TI == 1
    EEPromErase(u32EEPromByteAddr);		/* TI���õ�EEPROM֧�ַ������� */
#endif

	/* д��:eeprom��������Ӧ����������--������ø�һ��flash�������ñ�����Ϊ0xAAAAAAAAUL, ����0x55555555UL */
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

	/* д��:���ݲ��� */
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
					Swi_disable();	/* �����������ݣ�����洢�б����� */
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

	/* д��:CRC���� */
	EEPROMRead((uint32_t*)u32DataBuf, u32EEPromByteAddr, 4);
	if(u32DataBuf[0] != u32CrcForEEProm) {
		EEPROMProgram((uint32_t*)(&u32CrcForEEProm), u32EEPromByteAddr, 4);
	}
}

/*==========================================================================
| Description	: ���eeprom�е���������
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
BOOL CheckConfFromEEProm(CONF_SAVE_GROUP AcsGrp, uint32 u32EEPromByteAddr)
{
#if LEGACY
	if((u32EEPromByteAddr == 0) || (u32EEPromByteAddr >= 96*64)) {	/* ������ */
#else 
	if(!WITHIN_SAVE_GRP_EEPROM_ADDR(u32EEPromByteAddr)) {	/* ������ */
#endif
		return FALSE;
	}
	/* ���ñ���--������ø�һ��flash�������ñ�����Ϊ0xAAAAAAAAUL, ����0x55555555UL */
	uint32 u32ByteLen = 4;
	uint32 u32DataBuf[CONF_ITEM_MAX_DATA_U32LEN];
	EEPROMRead((uint32_t*)u32DataBuf, u32EEPromByteAddr, u32ByteLen);
	if((CheckExeFlash_0Low_1High() && (u32DataBuf[0] != CONF_EEDATA_BY_HIGH_FLASH))
		|| ((!CheckExeFlash_0Low_1High()) && (u32DataBuf[0] != CONF_EEDATA_BY_LOW_FLASH))) 
	{
		return FALSE;
	}
    /* Ϊ�˱��ּ����ԣ�֮ǰ�汾 u32CrcForEEPromֵ��0xFFFFFFFF��AcsMedia[0].u32OrigEEAddr��0���������ط���ʼ����u32OrigEEAddr-1 */
	uint32 u32CrcForEEProm = pBootLoaderConf->AcsMedia[0].u32OrigEEAddr - 1;
	u32CrcForEEProm = CalCRC32ForFileByIni((uint8*)u32DataBuf, u32ByteLen, u32CrcForEEProm);
	u32EEPromByteAddr += u32ByteLen;

	/* ���ݼ�� */
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
	
	/* CRC��� */
	EEPROMRead((uint32_t*)u32DataBuf, u32EEPromByteAddr, 4);
	if(u32DataBuf[0] == u32CrcForEEProm) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/*==========================================================================
| Description	: ��eeprom�ж�ȡ��������
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

	/* ���eeprom�Ƿ�ǰ���ñ� */
	if(pBootLoaderConf->AcsMedia[AcsGrp].u32OrigEEAddr == 0)	{	/* δ���ñ����,����ɹ���־,��ú���ά��EEConf��ʱ��洢 */
		return DATA_ACS_RES_SUC;
	} else if(((u32EEPromByteAddr = CheckConfValidFromEEProm(AcsGrp, 0, pBootLoaderConf->AcsMedia[AcsGrp].u32OrigEEAddr)) != 0)
		|| ((u32EEPromByteAddr = CheckConfValidFromEEProm(AcsGrp, 0, pBootLoaderConf->AcsMedia[AcsGrp].u32BackEEAddr)) != 0))
	{
		/* ��ȡ���� */
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
		/* ͳ����Ҫ��ʼ��������Ŀ */
		uint32 u32ItemNum = 0;
		CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
		int16 i;
		for(i = pBootLoaderConf->u32ItemNum; i > 0; i--) {
			if(pDataAcsProp->ItemType.SaveGrp == AcsGrp) {
				u32ItemNum++;
			}
			pDataAcsProp++;
		}
		
		/* ��ȡ���� */
		uint32 u32OldItemNum;
		CONF_n_STAT_PROP* pOldDataAcsProp = (CONF_n_STAT_PROP*)((uint32)pOldBootLoaderConf->pConfProp + FLASH_HALF_BLEN);
		for(u32OldItemNum = pOldBootLoaderConf->u32ItemNum; u32OldItemNum > 0; u32OldItemNum--) {
			if((pOldDataAcsProp->ItemType.SaveGrp == AcsGrp) && (pOldDataAcsProp->pData != NULL)) {	/* �������ñ����������ϵĴ洢���� */
				uint32 u32ByteLen = CalDataBLenFromItemDataType(pOldDataAcsProp->ItemType.DataType);
				if(u32ByteLen) {	/* ȷ�����ݴ洢��Ŀ--�����͡������� */
					/* ���µ����ñ���������ͬID�������� */
					pDataAcsProp = pBootLoaderConf->pConfProp;
					for(i = pBootLoaderConf->u32ItemNum; i > 0; i--) {
						if((pDataAcsProp->u32ItemID == pOldDataAcsProp->u32ItemID) /* �ҵ�ͬ��������ID���������д�� */
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
					
					u32EEPromByteAddr += u32ByteLen;			/* ����EEProm   */
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

/* ���eeprom�������������ԣ���������������EEProm�еĵ�ַ */
uint32 CheckConfValidFromEEProm(CONF_SAVE_GROUP AcsGrp, BOOL bConfAcsDscp_New0_Old1, uint32 u32EEPromByteAddr)
{
	uint32 reslt = 0;
	uint32 u32DataBuf[CONF_ITEM_MAX_DATA_U32LEN];
	uint32 u32EEPromByteAddrStart = u32EEPromByteAddr;

	/* ������ */
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
	/* Ϊ�˱��ּ����ԣ�֮ǰ�汾 u32CrcForEEPromֵ��0xFFFFFFFF��AcsMedia[0].u32OrigEEAddr��0���������ط���ʼ����u32OrigEEAddr-1 */
	if(bConfAcsDscp_New0_Old1) {
		pDataAcsProp = (CONF_n_STAT_PROP*)((uint32)pOldBootLoaderConf->pConfProp + FLASH_HALF_BLEN);
		u32ItemNum = pOldBootLoaderConf->u32ItemNum;
        u32CrcForEEProm = pOldBootLoaderConf->AcsMedia[0].u32OrigEEAddr - 1;
	} else {
		pDataAcsProp = pBootLoaderConf->pConfProp;
		u32ItemNum = pBootLoaderConf->u32ItemNum;
		u32CrcForEEProm = pBootLoaderConf->AcsMedia[0].u32OrigEEAddr - 1;
	}

	/* ͷ����־��� */
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

	/* CRC���� */
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


	/* CRC��� */
//	return u32EEPromByteAddrStart + 4;	/* ��Ҫ������ʼ�ĸ�/��һ��flash��־���� */
	EEPROMRead((uint32_t*)u32DataBuf, u32EEPromByteAddr, 4);
	if(u32DataBuf[0] == u32CrcForEEProm) {
//		return u32EEPromByteAddrStart + 4;	/* ��Ҫ������ʼ�ĸ�/��һ��flash��־���� */
		if(AcsGrp == SAVE_GRP_MCONF) {
			reslt = u32EEPromByteAddrStart + 4;
			goto end;
		} else {
			return u32EEPromByteAddrStart + 4;	/* ��Ҫ������ʼ�ĸ�/��һ��flash��־���� */
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
| Description	: ͨ���ļ�ϵͳ�洢conf��msg��acq
	�����ļ���/Ŀ¼����: �����ļ��������ļ����ر��ļ�����鲢����Ŀ¼
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-06-24
\=========================================================================*/
void ChkAndMkDir(void)
{
	/* ����ļ�ϵͳ:����ļ�ϵͳ��������Ҫ��fat32��ʽ��ʽ��TF�� */
	if(f_stat("0:", NULL) == FR_NO_FILESYSTEM) {
		f_mkfs(0, FM_FAT32, 0, g_NvMemAcsCtr.fatfsTempSpace, _MAX_SS);
	}
	
	/* ����Ŀ¼ */
	if((f_stat((const TCHAR*)"0:/Yoplore", NULL)) != FR_OK) {
		f_mkdir((const TCHAR*)"0:/Yoplore");
	}

	/* ��� �豸 Ŀ¼ */
	uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
	*pFilePath = 0;
	if((f_stat((const TCHAR*)g_NvMemAcsCtr.u8FilePath, NULL)) != FR_OK) {
		f_mkdir((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
	}

	/* ��� /CONF Ŀ¼ */
	uint8* pFilePathBak = pFilePath;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Conf");
	*pFilePath = 0;
	if(f_stat((const TCHAR*)g_NvMemAcsCtr.u8FilePath, NULL) != FR_OK) {
		f_mkdir((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
	}

	/* ��� /MSG2 Ŀ¼ */
	pFilePath = pFilePathBak;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Msg2");
	*pFilePath = 0;
	/* f_statִ��ʱ��Լ2.7ms */
	if(f_stat((const TCHAR*)g_NvMemAcsCtr.u8FilePath, NULL) != FR_OK) {
		f_mkdir((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
	}
	
	/* ��� /MSG Ŀ¼ */
#if SUPPORT_MsgV1RW_AcqV1R
	pFilePath = pFilePathBak;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Msg");
	*pFilePath = 0;
	if(f_stat((const TCHAR*)g_NvMemAcsCtr.u8FilePath, NULL) != FR_OK) {
		f_mkdir((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
	}
#endif
	
	/* ��� /ACQ Ŀ¼ */
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
		while((f_readdir(&dir, &fno) == FR_OK) && fno.fname[0]) {		/* �ɹ���ȡ�ļ����ļ����� */
			int8 i;
			for(i = 0; (i < FILE_NAME_BYTE_LEN) && fno.fname[i]; i++) {	/* �����ļ����ļ����� */
				pFilePath[i] = fno.fname[i];
			}
			if((fno.fattrib & AM_DIR) == 0) {	/* �ļ���ֱ��ɾ�� */
				pFilePath[i] = 0;
				f_unlink((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
			} else {				/* �ļ��У��������һ�㼶 */
				findFold = TRUE;
				pFilePath += i;
				*pFilePath = 0;		/* Ϊ��һ��ѭ����Ŀ¼��׼�� */
				break;
			}
		}
		/* ���¼��ļ��У���˵�����ļ����Ѿ���� */
		if(!findFold) {
			pFilePath--;	/* ���뱾��Ŀ¼��ʱ�򣬽�����һ�� *pFilePath++ = '/';  */
			if(pFilePath <= pFilePathIni) {		/* �Ѿ������Ŀ¼�����Ѿ�����ˣ�������� */
				break;
			} else {		/* �Ǹ�Ŀ¼����ɾ����Ŀ¼ */
				*pFilePath = 0;
				f_unlink((const TCHAR*)g_NvMemAcsCtr.u8FilePath);
				for(; (pFilePath >= pFilePathIni) && (*pFilePath != '/'); pFilePath--) {	/* ������һ���ֶϷ� */
				}
				*pFilePath = 0;	/* Ϊ��һ��ѭ����Ŀ¼��׼�� */
			}
		}
	}
}

/*==========================================================================
 *	����Ϊ ���� �ļ���������
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
| Description	: �������������ı�����ʽ�洢���ļ��У�����ʾ������
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
void SaveConfToFile(CONF_SAVE_GROUP ConfAcsGrp)
{
	/* ��������Ŀ¼ */
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

	/* ���ļ� */
	FIL File;
	UINT u32SavedByteNum;			/* �ɹ�������ֽ��� */
	if(f_open(&File, (const TCHAR*)g_NvMemAcsCtr.u8FilePath, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)	{
		return;
 	} 

	/* ��ӡ���к���汾�� */
	uint32 u32CrcForFile = 0xFFFFFFFF;
	uint8* pBuf = g_NvMemAcsCtr.u8FileCont;
	PrintString(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], "0000  ");	/* �����ȡConfIDʱ����� */
	PrintString(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], DEV_TYPE_MQTT_TOPIC_String);
	PrintU32(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], g_Sys.SerialNo.u32Dat, 0);
	*pBuf++ = ' ';
	PrintSoftVer(&pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], SOFTWARE_VER);
	*pBuf++ = 0x0D;
	*pBuf++ = 0x0A;
	if(pBuf + 200 > &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN]) {	/* ǰ��Ӧ�����������٣���������ж��Է���һ */
		return;
	}
	
	/* ��ӡ������ */
	CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
	int16 ItemNum = pBootLoaderConf->u32ItemNum;
	for(; ItemNum > 0; ItemNum--) {
		if((pDataAcsProp->ItemType.SaveGrp == ConfAcsGrp) && (pDataAcsProp->pData != NULL)) {
			pBuf = PrintLocalConfItem(DATA_USER_FILE, pDataAcsProp, pBuf, 100);

			/* ����CRC�����浱ǰ�� */
			UINT u32ByteNumToBeSaved = pBuf - g_NvMemAcsCtr.u8FileCont;
			if(u32ByteNumToBeSaved + 120 > FILE_CONT_BUF_LEN) {
				u32CrcForFile = CalCRC32ForFileByIni(g_NvMemAcsCtr.u8FileCont, u32ByteNumToBeSaved, u32CrcForFile);
				f_write(&File, g_NvMemAcsCtr.u8FileCont, u32ByteNumToBeSaved, &u32SavedByteNum);
				pBuf = g_NvMemAcsCtr.u8FileCont;
			}
		}

		/* 6. ׼����һ�� */
		pDataAcsProp++;
	}

	/* ���У�飬�����桢�ر��ļ�: ����У�����в��ᳬ�� */
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
| Description	: ����/����ļ��д洢���ı���ʽͳ��/�������ݣ�������CRC���
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

	/* ������ */
	if(iTryCnt_pRd_nChk == 0) {
		return DATA_ACS_RES_INPUT_ERR;
	}

	BOOL bRead1_Check0 = (iTryCnt_pRd_nChk > 0);
	if(!bRead1_Check0) {
		iTryCnt_pRd_nChk = 0 - iTryCnt_pRd_nChk;
	}
	
	/* ��������Ŀ¼ */
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
		/* �ļ�CRC��� */
		uint32 u32DataFilePt = 0;
		uint32 u32CrcForFile = 0xFFFFFFFF;
		UINT u32ByteNum;
		do {
			f_lseek(&File, u32DataFilePt);
			f_read(&File, g_NvMemAcsCtr.u8FileCont, FILE_CONT_BUF_LEN, &u32ByteNum);
			/* �����ļ���Ѱ�һس����� */
			pText = g_NvMemAcsCtr.u8FileCont;
			pLineStart = pText;
			pLineEnd = pLineStart;
			for( ; pText < &g_NvMemAcsCtr.u8FileCont[u32ByteNum]; pText++) {
				/* �ҵ��س����з���, ˵��һ����Ч������ */
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

		/* ͳ����Ҫ��ʼ��������Ŀ */
		int16 ItemNum = 0;
		CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
		int16 i;
		for(i = pBootLoaderConf->u32ItemNum; i > 0; i--) {
			if((pDataAcsProp->ItemType.SaveGrp == ConfAcsGrp) && (pDataAcsProp->pData != NULL)) {
				ItemNum++;
			}
			pDataAcsProp++;
		}

		/* ��ȡ��Ŀ */
		u32CrcForFile = 0xFFFFFFFF;
		u32DataFilePt = 0;								/* ��ʼ���ļ���ȡָ�� */
		u32ByteNum = 0;
		pText = g_NvMemAcsCtr.u8FileCont;
		pLineEnd = g_NvMemAcsCtr.u8FileCont;
		while(ItemNum > 0) {
			if(pText >= &g_NvMemAcsCtr.u8FileCont[u32ByteNum]) {
				u32DataFilePt += pLineEnd - g_NvMemAcsCtr.u8FileCont;		/* �޸��ļ���ȡָ�� */
				f_lseek(&File, u32DataFilePt);
				f_read(&File, g_NvMemAcsCtr.u8FileCont, FILE_CONT_BUF_LEN, &u32ByteNum);
				if(u32ByteNum == 0) {					/* �Ѿ����������� */
					break;
				} else {								/* �����ʼ��������ָ�� */
					pText = g_NvMemAcsCtr.u8FileCont;
				}
			}

			pLineStart = pText;
			for( ; pText < &g_NvMemAcsCtr.u8FileCont[u32ByteNum]; pText++) {
				/* �ҵ��س����з���, ˵��һ����Ч������ */
				if(*pText == 0x0A) {
					/* ��ø���ID */
					uint32 u32ItemID = GetConfIDInText(pLineStart);
					if(u32ItemID != 0) {
						/* ��Ѱ����ID����Ӧ��pDataAcsProp */
						pDataAcsProp = pBootLoaderConf->pConfProp;
						for(i = pBootLoaderConf->u32ItemNum; i > 0; i--) {
							if((pDataAcsProp->u32ItemID != u32ItemID) 
								|| (pDataAcsProp->ItemType.SaveGrp != ConfAcsGrp) 
								|| (pDataAcsProp->pData == NULL))
							{
								pDataAcsProp++;
							} else {	/* �ҵ���Ӧ��, ��ȡ�������� */
								/* ����ǰ��� ConfID �Լ�һ���ո� */
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
					break;		/* ������ǰ������ */
				}
			}

			/* �Ѿ���ɵ�ǰ�ļ����� */
			if((u32ByteNum < FILE_CONT_BUF_LEN) && (pText >= &g_NvMemAcsCtr.u8FileCont[u32ByteNum])) {
				break;
			}
		}

		/* ����������У�� */
		if(ItemNum == 0) {				/* δ���� */
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
| Description	: ��һ����������/ͳ���ı�����ȡ���������/ͳ�����ݣ�д����Ӧ������λ�û��߽�����飬ע���Ǵ���β��ʼ
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
BOOL ConvTextForNormalConf(uint8* pText, uint8* pTextStart, CONF_n_STAT_PROP* pDataAcsProp, BOOL bRead1_Check0)
{
	/* 0. ������ */
	if(pDataAcsProp->pData == NULL) {
		return FALSE;
	}
	float32 fData;

	/* 1. �ѵ�һ�����ݶ�ȡ���� */
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

	/* 2. ���ݾ���ı������ݸ�ʽ���д洢 */
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
				if(iExp > 0) {		/* С����λ�ú����벻һ�´��� */
					i64Data *= cnst_u64BcdQFactor[iExp];
				} else if(iExp < 0) {
					iExp = 0 - iExp;
					i64Data = (i64Data + (cnst_u64BcdQFactor[iExp]>>1))/cnst_u64BcdQFactor[iExp];
				}
			case ITEM_DAT_TOPIC:	/* ����һ��С���㡢ָ���� */
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
				if(iExp > 0) {		/* С����λ�ú����벻һ�´��� */
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
	
	/* 3. ����е�2���� */
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

	/* 4. ���ݾ���ı������ݸ�ʽ���д洢 */
		if(!bFindData) {
			return FALSE;
		} else if((pDataAcsProp->ItemType.DataType == ITEM_DAT_D2U32)
    	    || (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_ID_RO))
		{
			iExp += pDataAcsProp->ItemType.uBcdQ;
			if(iExp > 0) {		/* С����λ�ú����벻һ�´��� */
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
| Description	: ��һ�����ı�������ȡ�����CRC32���ݣ�����CRC32У�飬ע���Ǵ���β��ʼ
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
| Description	: ����������ȡ��������ID(����ǰ��)
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2017-10-13
\=========================================================================*/
uint32 GetConfIDInText(uint8* pLineStart)
{
	uint32 u32Data = 0;
	BOOL bFindData = FALSE;
	int8 i;
	
	for(i = 10; i > 0; i--) {	/* ���Ҳ��10λ�� */
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
| Description	: ���¶����õ��ı���Ϣת��Ϊ�����Ƶ���ʽ
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
BOOL ConvTextForTempSenConf(uint8* pText, uint8* pTextStart, TEMP_SEN_CONF* pData, BOOL bRead1_Check0)
{
	uint64 u64Hex;
	uint32 u32Data;
	uint16 uDigitNo;
	BOOL bFindData;

	/* 1. ȡ��u32TempSenPosition */
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

	/* 2. ȡ��u32TempSenChangeCount */
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

	/* 3. ȡ��u64TempSenRom */
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
| Description	: ���ı�ת��ΪIPv4��ַ
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
| Description	: ���ı��������ַ�����0~9,a~z,A~Z,.
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-05-26
\=========================================================================*/
BOOL ReadConfStringInText(uint8* pText, uint8* pTextStart, uint8* pU8Data, BOOL bRead1_Check0)
{
	BOOL bFindData = FALSE;
	int16 iByteCnt = 0;
	for( ; pText >= pTextStart; pText--) {			/* �����ַ������� */
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
| Description	: �Ѿ����Զ��ɼ����ܵ��ı�ת��Ϊ����, ����
	��ʾ���ı���: ��ǰʱ��(s);  0.0987 20 0.1
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-10-31
\=========================================================================*/
BOOL ConvTextForAcqSumm(uint8* pText, uint8* pTextEnd, ACQ_SUMM* pData, BOOL bRead1_Check0)
{
	/* 1. ȡ��fAcqSummary */
	float32 fData;
	if(!GetF32(&pText, pTextEnd, &fData)) {
		return FALSE;
	} else if(bRead1_Check0) {
		pData->fAcqSummary = fData;
	} else if(fabsf(pData->fAcqSummary - fData) > fabsf(fData)*0.001f) {
		return FALSE;
	}
	
	/* 2. ȡ��u32SummaryRealNum */
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
	
	/* 3. ȡ��i32SummaryPlanNum */
	int32 i32Data;
	if(!GetI32(&pText, pTextEnd, &i32Data, 0)) {
		return FALSE;
	} else if(bRead1_Check0) {
		pData->i32SummaryPlanNum = i32Data;
	} else if(pData->i32SummaryPlanNum != i32Data) {
		return FALSE;
	}

	/* 4. ȡ��fSetVal */
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

/* ��ȵ��ʱ�䣬�ϰ汾�洢��ʽΪhh.mm������Ϊs#hh:mm */
BOOL ReadPTimeInText(uint8* pText, uint8* pTextEnd, uint32* pU32, BOOL bRead1_Check0)
{
	BOOL bFindData = FALSE;
	uint32 u32Data[2] = {0, 0};
	int8 i = 0;		/* ���� */
	for( ; (pText < pTextEnd) && (i < 2); pText++) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u32Data[i] = u32Data[i]*10 + (*pText - '0');
		} else if((*pText == '#') || (*pText == ':') || (*pText == '.')) {	/* ʱ����ܳ���Ϊ 1#12:00 */
		} else if(bFindData) {
			i++;	/* ָ����һ������ */
			bFindData = FALSE;
		}
	}
	if(bFindData && (pText >= pTextEnd)) {/* Buf���һ����Ȼ����Ч���֣��Ͳ��ᴥ��i++ */
		i++;
	}

	if(i < 2) {	/* û�ж��� */
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
	����Ϊ Msg & Acq ����
  Msg, Acq  ���������Եı�֤
  1. ��ram�����������ֻ����һ�������н��У�msg���������Ա�ʶ��MsgNo, Acq���������Ա�ʶ��
	 ��������ݺ󣬲��޸�ItemNo_forAdd��������֤ItemNo_forAddָ�����Ч��
  2. ����ӵı�ʶ����Ҫ����MsgId��AcqId
  3. �洢ram���ݵ�NvMem�������ȼ���ͣ��洢������޸�NvMemָ�룬Ȼ��������ram���������Ա�ʶ
	 ֵ��ע����ǣ�����������Ҫ�洢file��eeprom�У�����������������Ա�ʶ�������Ƿ�NvMem����Ϊ׼(һ����file)
	 ����tf������ʧЧ�����file����Զд����ȥ���ݣ������RamBuf�Ƚ�����ʱ����Ҫ��������file
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
            
            /* ������ǰĿ¼�У�ָ�����ڵ��ļ����ļ��У�����ж����ѡ��ItemNo���� */
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
    
            if(u32MaxDirOrFileItemNo) {     /* �ҵ��ļ����ļ��� */
                pFilePath += i;
            } else {    /* û���ҵ�������Ҫ�����ļ��л����ļ� */
                /* �����ļ��С��ļ��� */
                pFilePath += 2;
                uint32 u32ItemNoTmp = u32ItemNo;
                for(i = 5; i >= 0; i--) {
                    pFilePath[i] = u32ItemNoTmp%10 + '0';
                    u32ItemNoTmp = u32ItemNoTmp/10;
                }
                pFilePath += 6;
                if(i8FilePathDepth) {   /* �����ļ��� */
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

    /* ����ļ� */
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

/* ��msg2,acq2�ļ� */
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
    
    /* �����ļ����У����һ���ļ� */
    *pFilePath = 0;
    DIR dir;
    if(f_opendir(&dir, (const TCHAR*)g_NvMemAcsCtr.u8FilePath) != FR_OK) {
        return FALSE;
    } else {
        *pFilePath++ = '/';
        /* ����Ŀ¼���Ƿ���ڱ�������ItemNo������ļ� */
        uint32 u32MaxDirOrFileItemNo = (u32ItemNo/100000)*100000;        
        uint32 u32CurDirOrFileItemNo;
        FILINFO fno;
        int8 i;
        uint8 u8FileNameLen = 0;    /* �ļ�������,0����û���ҵ� */
        while((f_readdir(&dir, &fno) == FR_OK) && fno.fname[0] && ((fno.fattrib & AM_DIR) == 0)) {
            u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname);            
            /* ֮ǰ���bug�������000000.txt��ʵ���������ĵ�һ�������1 */
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
        
        /* �γ�����Ŀ¼   */
        if(u8FileNameLen) {
            pFilePath += u8FileNameLen;
        } else {    /* û���ҵ���Ӧ���ļ���  */
            PrintU32WithLen(&pFilePath, u32ItemNo, 6);
            PrintStringNoOvChk(&pFilePath, ".txt");
        }
        *pFilePath = 0;

        /* д���ļ� */
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
	
	/* �����ļ� */
	uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
	if(DataPage == DATA_PAGE_MSG) {
		PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Msg");
	} else if(DataPage == DATA_PAGE_ACQ) {
		PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Acq");
	} else {
		return 0;
	}
	if(eReadItemVer == MSG_ACQ_ACS_V2) {               /* ��ȡMsg2����Acq2 */
	    *pFilePath++ = '2';
	}

	/* �����ꡢ�¡����ļ�����ItemNo���� */
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
			uint32 u32CurSeekMaxItemNo = 0; 		/* ��ǰ�ڶ�����֮�кţ�ÿ�ɹ���ȡһ������һ */
			i = 0;
			FILINFO fno;
			while((f_readdir(&dir, &fno) == FR_OK) && fno.fname[0]) {
				if((iFilePathDepth != 0) ^ ((fno.fattrib & AM_DIR) == 0)) {
			    	uint32 u32CurDirOrFileItemNo;
				    if(eReadItemVer == MSG_ACQ_ACS_V2) {
    					u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname); /* ��ʮ�����Ʊ��ĵ�ǰ�ڶ�����֮�кţ������ļ���/�ļ����� */
				    } else {
    					u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname + 2); /* ��ʮ�����Ʊ��ĵ�ǰ�ڶ�����֮�кţ������ļ���/�ļ����� */
    				}
    				/* ֮ǰ���bug�������000000.txt��ʵ���������ĵ�һ�������1 */
    				if(u32CurDirOrFileItemNo == 0) {
    				    u32CurDirOrFileItemNo = 1;
    				}
					if(u32CurSeekMaxItemNo < u32CurDirOrFileItemNo) {
						for(i = 0; (i < FILE_NAME_BYTE_LEN) && fno.fname[i]; i++) {
							pFilePath[i] = fno.fname[i];
						}
						u32CurSeekMaxItemNo = u32CurDirOrFileItemNo;
						u32LastItemNoFromFile = u32CurDirOrFileItemNo;	/* ���ļ��С��ļ�Ϊ׼ */
					}
				}
			}
			pFilePath += i;
		} else {
			break;
		}
	}

	/* ���ļ� */
	*pFilePath = 0;
	FIL File; 
	if((iFilePathDepth < 0) 	/* ǰ����ļ��ɹ� */
		&& (f_open(&File, (const TCHAR*)g_NvMemAcsCtr.u8FilePath, FA_READ) == FR_OK))
	{
		uint32 u32DataFilePt = f_size(&File);		/* ��ʼ���ļ�����ָ�� */
		int32 i32LineStartPt = 0;
		uint32 u32SkipItem = 0; 				/* ���CRCУ��ʧ�ܣ�������һ�У������Ӧ��������ItemNo��Ҫ������һ�� */
		do {
			/* ��ȡ���� */
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
			int32 i32LineSeekPt = u32BufByteNum - 2;	/* ������β�Ļس����� */
			if(i32LineSeekPt < 0) {
				break;
			}

			/* ������: �ҵ��س����з���, ˵��һ����Ч������ */
			int32 i32LineEndPt = u32BufByteNum;
			for( ; i32LineSeekPt >= 0; i32LineSeekPt--) {
				/* �ҵ��س����з���, ˵��һ����Ч������ */
				if((g_NvMemAcsCtr.u8FileCont[i32LineSeekPt] == 0x0A) || ((i32LineSeekPt == 0) && (u32DataFilePt == 0))) {
					if(i32LineEndPt - i32LineSeekPt > 10) {
						i32LineStartPt = i32LineSeekPt;
						if(g_NvMemAcsCtr.u8FileCont[i32LineSeekPt] == 0x0A) {
							i32LineStartPt++;
						}
						if(CheckTextWithCrc32ForMsgAndAcq(&g_NvMemAcsCtr.u8FileCont[i32LineStartPt], &g_NvMemAcsCtr.u8FileCont[i32LineEndPt])) {		/* һ����Ч���� */
							u32LastItemNoFromFile = ReadU32FixLen(&g_NvMemAcsCtr.u8FileCont[i32LineStartPt]) + u32SkipItem;
							u32DataFilePt = 0;	/* �޸��ļ�ָ�����˳��������ѭ�� */
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
	
	if(u32Hex == CalCRC32ForFileByIni(pLineStart, pText - pLineStart + 1, 0xFFFFFFFF)) {	/* CRCĬ�ϳ�ʼ��Ϊ 0xFFFFFFFF */
		return TRUE;
	} else {
		return FALSE;
	}
}


/*==========================================================================
| Function name	: 
| Description	: �¼���¼���ƣ��¼���������:
	A. ���¼�	: ����Ҫ���¼�������ʱ�򣬼�¼�µ�ǰֵ��ʱ�䣬��Ϣ����ӵ�ʱ�����ɣ����緢���;
	B. �����¼�	: ������A���ƣ�ֻ�������Ϣ�󣬳����ٴ�������򲻵��ٴ���ӣ����綯���������ޣ���ô������������¼�
	C. �����¼�	: �¼���Ҫ��¼����ʱ�̡���ֵ������Ҫȷ������ɣ����ѹ����;
	�ܵĹ��ܰ���:
		��ʼ��		: ��ʼ��g_MsgStrCtr(��Ϣ������)
		���		: ����µļ�¼��������ȷ��/ȡ��(C��) ���� ����(B��)
		�������ȡ	: ��g_MsgStrCtr�е��Ѿ���ɼ�¼����Ϣ���뷭����ı��������ļ�ϵͳ�������g_MsgStrCtr����Ӧ����Ϣ
					: ����A���¼��������Ϣ���������洢
					: ����B���¼����洢��ʱ�򲻵ø�λ����(��λһ���ǹػ���λ��)������ʶ����Ϣ�Ѿ��洢
					: ����C���¼�����ɼ�¼�󣬾Ϳ��Դ洢
					: Ϊ�˷�ֹ����ʱ����ķ����洢�ں��棬�洢��ʱ��Ҫ����Ram��δȷ�ϵ�C���¼��������洢ʱ�����"����δȷ��C���¼�֮ǰ"���¼�
		��ȡ��ͨѶ	: ���ݱ��(ItemNo)��ʱ����дӴ洢����(Ram/�ļ�)���ң����ύ��ͨѶϵͳ(������ʾ����)
		ItemNo����	: Ram�����Msg�ڷ���ItemNo��ʱ������Ϣȷ�ϵ�ʱ���Ⱥ����ItemNo����ÿȷ��һ����Ϣ��ʱ��ŷ���ItemNo
					: ���ǵ���Ram����ı�����Ϣ����ı������ļ���ʱ����Ҫ�����¼���Ϣʱ���Ⱥ�˳�����·���ItemNo
					: Ϊ�˷�ֹд�����ItemNo���·��������ͨѶ/��ʾ���ң�ֻ�е�ͨѶ/��ʾδ����Msg��ʱ��ſ��Խ��д洢����

	�����Ϣ�ĺ�������:
		A�������������	:
			�����Ϣ(������): �����Ϣ����������������ͬ������Ϣ�������ٴα����
		B�������������	:
			�����Ϣ(����)	: �����Ϣ����������������ͬ������Ϣ�������ٴα����
			ȡ������		: ȡ������������
		C�������������	:
			�����¼�������ֵ: ���û�п��Ը���ֵ�ļ�¼���ͽ�����¼(��Ҫ�Ǽ�¼������ʱ��)�����������ֵ
			ȷ���¼�		: ���ڱ������¼���������Ҫһ������ʱ����ȷ���¼�(�Է�����)
			��ɼ�¼		: ֵ�ָ������Ժ󣬲���Ϊһ���¼���¼���--����¼�����ȷ�ϣ���������¼��������ʶ���¼��Ѿ�������
	ע��:�����Ϣ�ຯ������ͬʱ����
			
	MsgBuf��־: ��⵽�¼���ʱ�򣬸���MsgId; ȷ���¼���ʱ�򣬸���ItemNo; �¼���ɵ�ʱ�򣬸�ֵCmpltFlag; �洢��ʱ�򣬸�ֵuEEPromNo

| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-10-3
\=========================================================================*/
#if MAX_MSG_TYPE
/* ������MsgID��Ĭ�ϵ� */
#define MSG_NULL 				0x0000			/* �� */
#define MSG_HuanYinShiYong		0x0001			/* ��ӭʹ��--������Ϣ�����ڿ�����ʾ����Ӧ����AddMsg���(�洢) */

typedef struct {								/* 160bit, 20byte */
	REAL_TIME_VAR MsgTimeStamp; 				/* �¼�ʱ��� */
	uint32 u32ItemNo;							/* ��Confirmʱ�򣬾͸�ֵ32ItemNo�����������¼��������� */
	uint16 uMsgId;								/* �¼�ID */
	uint16 uEEPromNo;							/* �洢��EEProm�е���� */
	float32 fMsgVal;							/* �¼�ֵ */
}MSG_ITEM;

/*	�¼���Ϣ��¼: �����Ϣ�ڷ�MSG_NUL�¼���MSG_NUL����	*/
#define MSG_ITEM_BUF_LEN		16
#define MSG_SHIELD_FLAG_LEN		((MAX_MSG_TYPE+31)/32)
#define MSG_SAVED_BUF_LEN       8
typedef struct {
	uint32 u32CmpltFlag;						/* Msg��ɱ�ʶ */
	uint32 u32ShieldFlag[MSG_SHIELD_FLAG_LEN];	/* �¼����α�־����ֹ��������¼� */
	MSG_ITEM MsgBuf[MSG_ITEM_BUF_LEN]; 			/* ��Ϣ������--�����Ϣ������������� */

	/* һЩ�����ٷ�����Ϣ��Buf */
	MSG_ITEM* pLastAddMsg;
	MSG_ITEM MsgSaved[MSG_SAVED_BUF_LEN];       /* �Ѿ��־û�����Ϣ */
	int8 i8Pt_MsgSaved;                         /* ָ��洢������ */
	uint8 u8Rsvd;

	/* ��ȡ��ص�ָ������ */
	uint16 uEEPromNo_ForSave;					/* ���ڴ洢��һ����Ϣ�Ĵ洢��EEProm�е���� */
	uint32 u32MsgNo_ram;                        /* ram�洢��MsgNo��ʼ(����) */
	/* ��4�������� ReadItemNoAndEEPromNo �г�ʼ�� */
	uint32 u32MsgNo_FileSaved;					/* �ļ����Ѿ��洢��MsgNo */
	uint32 u32MsgNo_EESaved;					/* EEProm�Ѿ��洢��MsgNo */
	uint32 u32MsgNo_ForAdd;						/* ���������һ����Ϣ��MsgNo */
	uint32 u32MsgNo_Cmplt;                      /* �Ѿ�����˵���ϢMsgNo */
}MSG_CTR;
SECTION(".NOT_ZeroInit") MSG_CTR g_MsgCtr;
extern const MSG_PROP cnst_MsgProp[];			/* Msg��������ģ���MsgID������ı� */

void InitMsgMdl(void)
{
	if(g_Sys.uRstCount) {	/* ���ϵ縴λ�ų�ʼ�� */
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
	/* ��ʼ��Mqtt���ʽӿ� */
	g_MqttItemProc.u32MsgNo_HavePubedForDB = 0xFFFFFFFFUL;

	/* ��ȡ���һ�е��к�:�������ļ��У�����ļ��ж�ȡʧ�����eeprom�� */
#if SUPPORT_FAT_FS
	uint32 u32LastItemNoFromFile = 0;
	/* ReadLastItemNoFromFile()����0˵����ʧ��--������һ�ο����Ƕ�ȡʧ�ܣ���˸ĳ�����5�� */
	for(i = 5; (i > 0) && ((u32LastItemNoFromFile = ReadLastItemNoFromFile(DATA_PAGE_MSG, MSG_ACQ_ACS_V2)) == 0); i--) {
	}
	#if 0 // ɾ����msgV1 ItemNo��֧��,������δ��뱣�����Ա�������ҪԶ�̶�ȡʹ�� SUPPORT_MsgV1RW_AcqV1R
		uint32 u32LastItemNoFromFileV1 = ReadLastItemNoFromFile(DATA_PAGE_MSG, MSG_ACQ_ACS_V1);
		if(u32LastItemNoFromFile < u32LastItemNoFromFileV1) {
			u32LastItemNoFromFile = u32LastItemNoFromFileV1;
		}
	#endif
#else
	uint32 u32LastItemNoFromFile = 0;
#endif

	/* ��eeprom������ItemNo��EEPromNo */
#if CPU_0ST_1TI
	uint32 u32EEPromAddr = EEPROM_ADDR_MSG;
	uint16 uRemainItemInEEProm = EEPROM_MSG_ITEM_NUM;	/* �����ܵĶ�ȡ���� */
	uint32 u32LastItemNoFromEEProm = 0; 				/* ����EEProm�����洢�����һ��ItemNo */
	uint16 uExpectEEPromNo = 0; 						/* ������EEPromNo */
	/* ����EEPromNo���������, ȡ���������ItemNo, EEPromNo;
	   ���һֱû�����䣬��˵�����ݴ洢λ�����ûص���ʼ��ȡĩλEEPromNo��һ��ΪEEPromNo */
	do {
		/* ��EEProm�ж������� */
		uint16 uCurReadItemNum = FILE_CONT_BUF_LEN/sizeof(MSG_ITEM);	/* ��ǰ�ζ�ȡ����������֤���������� */
		if(uCurReadItemNum > uRemainItemInEEProm) { 				/* ȷ�����ᳬ����Χ */
			uCurReadItemNum = uRemainItemInEEProm;
		}
		EEPROMRead((uint32_t*)g_NvMemAcsCtr.u8FileCont, u32EEPromAddr, uCurReadItemNum*sizeof(MSG_ITEM));

		/* �������� */
		MSG_ITEM* pSeekMsgBuf = (MSG_ITEM*)g_NvMemAcsCtr.u8FileCont;
		if(uExpectEEPromNo == 0) {	/* ��ʼ��uExpectEEPromNo */
			/* �²�����оƬ */
			if(((pSeekMsgBuf->u32ItemNo == 0xFFFFFFFFUL) && (pSeekMsgBuf->uMsgId == 0xFFFFUL) && (pSeekMsgBuf->uEEPromNo == 0xFFFFUL))
				|| (pSeekMsgBuf->uEEPromNo%EEPROM_MSG_ITEM_NUM))	/* ��һ������EEPromNoӦ����EEPROM_MSG_ITEM_NUM��������������������ƻ� */
			{
				u32LastItemNoFromEEProm = 0;
				break;
			} else {
				uExpectEEPromNo = pSeekMsgBuf->uEEPromNo;
			}
		}
		/* EEPromNo��Ԥ��Ĳ�һ��(������һ)���ǲ���� */
		for(i = uCurReadItemNum; (i > 0) && (pSeekMsgBuf->uEEPromNo == uExpectEEPromNo); i--) {
			u32LastItemNoFromEEProm = pSeekMsgBuf->u32ItemNo;
			uExpectEEPromNo++;
			pSeekMsgBuf++;
		}

		/* �н��������ѭ�� */
		if(i) {
			break;
		} else {	/* �޽����׼����һ��ѭ�� */
			uRemainItemInEEProm -= uCurReadItemNum;
			u32EEPromAddr += uCurReadItemNum*sizeof(MSG_ITEM);
		}
	} while(uRemainItemInEEProm);
	g_MsgCtr.uEEPromNo_ForSave = uExpectEEPromNo;
#endif
	
	/* ���� u32MsgNo_Saved */
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

	/* ��ʼ��g_MsgCtr.MsgSaved[0] */
    GetRealTime(&g_MsgCtr.MsgSaved[0].MsgTimeStamp);
    g_MsgCtr.MsgSaved[0].fMsgVal = 0;
    g_MsgCtr.MsgSaved[0].uMsgId = MSG_HuanYinShiYong;
    g_MsgCtr.MsgSaved[0].u32ItemNo = g_MsgCtr.u32MsgNo_ForAdd;
    g_MsgCtr.pLastAddMsg = &g_MsgCtr.MsgSaved[0];
	g_MsgCtr.i8Pt_MsgSaved = 0;
}

/*==========================================================================
| Description	: ���Msg����
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-11-04
\=========================================================================*/
void ResetMsgData(void)
{
	/* �ļ�ϵͳ���� */
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
	*pFilePath++ = '2';		/* ɾ�� /msg2 */
	EmptyFold(pFilePath);
#endif

#if CPU_0ST_1TI
	/* EEPROM���� */
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

/* ��ram�е�Msgͨ��modbus���ͣ�һ����Ϣռ��6��modbus16bit�Ĵ��� */
void ConvertMsgItem2MsgForModbus(uint8** ppBuf, MSG_ITEM* pMsg);
uint16 TxMsgForModbus(uint8** ppBuf, uint16 uRegNum)
{
    int8 i, iMsgNum;
	uint32 u32TxMsgNo = 0xFFFFFFFFUL;
	uint16 uTxMsgNum = uRegNum*2/sizeof(MSG_FOR_MODBUS);    /* Ҫ��ȡ����Ϣ���� */
	if(uTxMsgNum > MSG_SAVED_BUF_LEN) {
	    uTxMsgNum = MSG_SAVED_BUF_LEN;
	}
	
	Swi_disable();
    /* ��δ�־û�����Ϣ */
    for(iMsgNum = uTxMsgNum; iMsgNum > 0; iMsgNum--) {   /* ���16����������� */
        /* ����ram�е�MsgNo���� */
        MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
        uint32 u32MaxMsgNo = g_MsgCtr.u32MsgNo_EESaved;
        int8 i8MsgPt = -1;          /* ���ڴ洢��MsgBuf�±� */
        for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
            /* ������ u32ItemNo ���㣬������Ϣȷ�� */
            if((u32MaxMsgNo < pMsg->u32ItemNo) && (pMsg->u32ItemNo < u32TxMsgNo)) {
                u32MaxMsgNo = pMsg->u32ItemNo;
                i8MsgPt = i;
            }
            pMsg++;
        }
    
        if(i8MsgPt < 0) {   /* �Ѿ������� */
            break;
        } else {
            pMsg = &g_MsgCtr.MsgBuf[i8MsgPt];
            u32TxMsgNo = pMsg->u32ItemNo;
            ConvertMsgItem2MsgForModbus(ppBuf, pMsg);
        }
    }

    /* ������ram�У��������Ѿ��־û�����Ϣ */
    int8 i8Pt = g_MsgCtr.i8Pt_MsgSaved;
    if((i8Pt >= 0) && (i8Pt <= MSG_SAVED_BUF_LEN - 1)) {    /* ȷ��ָ��û���� */
        for( ; (iMsgNum > 0) && g_MsgCtr.MsgSaved[i8Pt].uMsgId; iMsgNum--) {
            ConvertMsgItem2MsgForModbus(ppBuf, &g_MsgCtr.MsgSaved[i8Pt]);
            i8Pt--;
            if(i8Pt < 0) {
                i8Pt = MSG_SAVED_BUF_LEN - 1;
            }
        }
    }
	Swi_enable();

    /* ʣ�ಿ����0 */
	if(iMsgNum != 0) {
        uint8* pBuf = *ppBuf;
        InitDataWithZero(pBuf, iMsgNum*sizeof(MSG_FOR_MODBUS));
        pBuf += iMsgNum*sizeof(MSG_FOR_MODBUS);
        *ppBuf = pBuf;
    }
	
	return uRegNum;     /* ����ط���������ʵ�ʼĴ�������Ҫ��Ȼ���ô���������� */
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
| Description	: �洢Msg��Ϣ(��Ram����Ŀ��Դ洢�ı���Msg������ı�Msg��д���ļ�)
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2009-10-15
\=========================================================================*/
BOOL PrintMsgCode(DATA_USER DataUser, uint8** ppBuf, uint8* pBufEnd, MSG_ITEM* pMsg, BOOL bMsgCmplt, BOOL bTimeShort);
void SaveMsgToNvMem(void)
{
	MSG_ITEM* pMsg;
	uint32 u32MsgNo_ForAdd = g_MsgCtr.u32MsgNo_ForAdd;	/* ���棬��øú��������ڼ䷢����Ϣ�������ȷ�������н�� */
	uint32 u32MinMsgNo;		/* �����Ѵ洢����СMsgNo */
	int8 i;

#if SUPPORT_FAT_FS
	/* ����Ϣ�洢���ļ��� */
	uint8* pBuf = g_NvMemAcsCtr.u8FileCont;
	BOOL bQuitFileSave = FALSE;
	int8 i8SearchCnt = MSG_ITEM_BUF_LEN;
	uint8 u8Year = 0;
	uint8 u8Month = 0;
	uint8 u8Day = 0;
	uint32 u32FileBufFirstItemNo = 0;
	uint32 u32MsgNo_FileSaved = g_MsgCtr.u32MsgNo_FileSaved;    /* ��Ҫ����ļ����������޸�g_MsgCtr.u32MsgNo_FileSaved */
	do {
		/* ���������Ѿ��洢��MsgNo */
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
		
		/* �������MSG_ITEM_BUF_LEN�Σ��Ƿ�������� */
		i8SearchCnt--;
		bQuitFileSave = (i8SearchCnt <= 0);
		
		/* ��ӡ���ı� */
		BOOL bPrintBreak = FALSE;       /* �µ���Ϣ��������Ϣ�ļ����ˣ���Ҫ�γ��µ��ļ� */
		if(i8MsgPt < 0) {   /* �Ѿ������� */
			bQuitFileSave = TRUE;
		} else if(((g_MsgCtr.u32CmpltFlag & (1<<i8MsgPt)) != 0) || (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) 
					|| (u32MinMsgNo + MSG_ITEM_BUF_LEN*3/4 <= u32MsgNo_ForAdd))	/* MsgBuf�Ѿ���MSG_ITEM_BUF_LEN*3/4�� */
		{
			pMsg = &g_MsgCtr.MsgBuf[i8MsgPt];
			/* ��һ�����Ϣ��������Ϣ���� */
			if((SUPPORT_MsgV1RW_AcqV1R && u8Day && ((u8Year != pMsg->MsgTimeStamp.u8Year) 
													|| (u8Month != pMsg->MsgTimeStamp.u8Month)
													|| (u8Day != pMsg->MsgTimeStamp.u8Day)))
		        || (u32FileBufFirstItemNo && (u32FileBufFirstItemNo/100000 != pMsg->u32ItemNo/100000)))	/* ÿʮ����һ���ļ� */
			{
			    bPrintBreak = TRUE;
			} else if(PrintMsgCode(DATA_USER_FILE, &pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], /* ���� */
				                        pMsg, (g_MsgCtr.u32CmpltFlag & (1<<i8MsgPt)) != 0, FALSE))
			{
				u32MsgNo_FileSaved = pMsg->u32ItemNo;
				/* ����¼��һ�γɹ���ӡ��ItemNo��Ҫ��Ȼ�洢�����Ǻ���Msg��ItemNo������ļ���bug(Ӧ���ǵ�һ������ʵ�ʲ���) */
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

		/* д���ļ� */
		if(bPrintBreak
		    || (bQuitFileSave && (pBuf != g_NvMemAcsCtr.u8FileCont))
			|| (pBuf - g_NvMemAcsCtr.u8FileCont + 120 > FILE_CONT_BUF_LEN))		/* ����MsgItem������120byte */
		{
            /* �����/msg2/sn.txt */
            BOOL bSaveRes = SaveMsgOrAcq_V2(DATA_PAGE_MSG, pBuf, u32FileBufFirstItemNo);
		#if SUPPORT_MsgV1RW_AcqV1R	/* �����/msg/��/��/��.txt */
            bSaveRes |= SaveMsgOrAcq_V1(DATA_PAGE_MSG, pBuf, u8Year, u8Month, u8Day, u32FileBufFirstItemNo);
		#endif
            if(bSaveRes) {  /* Ϊ��һ�δ�ӡ��ʼ�� */
                g_MsgCtr.u32MsgNo_FileSaved = u32MsgNo_FileSaved;
                pBuf = g_NvMemAcsCtr.u8FileCont;
                u32FileBufFirstItemNo = 0;
            } else {
                bQuitFileSave = TRUE;
            }
		}
	} while(!bQuitFileSave);
#endif

	/* ����Ϣ�洢��eeprom�� */
	for(i = MSG_ITEM_BUF_LEN; i > 0; i--) {
		/* ���������Ѿ��洢��MsgNo */
		MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
		u32MinMsgNo = 0xFFFFFFFFUL;		/* �����Ѵ洢����СMsgNo */
		int8 i8MsgPt = -1;
		int8 j;
		for(j = 0; j < MSG_ITEM_BUF_LEN; j++) {
			if((g_MsgCtr.u32MsgNo_EESaved < pMsg->u32ItemNo) && (pMsg->u32ItemNo < u32MinMsgNo)) {
				u32MinMsgNo = pMsg->u32ItemNo;
				i8MsgPt = j;
			}
			pMsg++;
		}

		/* �洢 */
		if(i8MsgPt < 0) {
			break;		
		} else if(((g_MsgCtr.u32CmpltFlag & (1<<i8MsgPt)) != 0) || (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) 
					|| (u32MinMsgNo + MSG_ITEM_BUF_LEN*3/4 <= u32MsgNo_ForAdd))	/* MsgBuf�Ѿ���MSG_ITEM_BUF_LEN*3/4�� */
		{
			/* ����Ϣ�洢��eeprom�� */
			pMsg = &g_MsgCtr.MsgBuf[i8MsgPt];
			pMsg->uEEPromNo = g_MsgCtr.uEEPromNo_ForSave++;
		#if CPU_0ST_1TI
			EEPROMProgram((uint32_t*)pMsg, EEPROM_ADDR_MSG + (pMsg->uEEPromNo%EEPROM_MSG_ITEM_NUM)*sizeof(MSG_ITEM), sizeof(MSG_ITEM));
		#endif
			g_MsgCtr.u32MsgNo_EESaved = pMsg->u32ItemNo;
            /* ���Ѿ��־û�����ϢǨ�Ƶ�MsgSaved���Ա����ٷ��� */
            Swi_disable();
			/* ����������Ե�ʱ����MsgSaved������һ����ItemNo */
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
#if !SUPPORT_FAT_FS	/* ���û��tf���洢������һ�����eeprom��������������Ϣ�������׳��� */
	g_MsgCtr.u32MsgNo_FileSaved = g_MsgCtr.u32MsgNo_EESaved;
#endif

	/* ����Ѿ��洢��MsgBuf */
	/* MsgBuf��ʵ�������� u32MsgNo_ForAdd - u32MsgNo_FileSaved - 1, �����Ϣ���� >= MSG_ITEM_BUF_LEN*3/4����������
	ǰ��洢��ʱ������Ϣ����MSG_ITEM_BUF_LEN*3/4��ʱ�򣬾ͻ᲻����ɾʹ洢���Լ���MsgBuf����Ϣ��������˳���tf���𻵣�Ҫ��Ȼ��Ӧ�ô��������� */
	if(u32MsgNo_ForAdd - g_MsgCtr.u32MsgNo_FileSaved > MSG_ITEM_BUF_LEN*3/4) {
		u32MinMsgNo = u32MsgNo_ForAdd - MSG_ITEM_BUF_LEN*3/4;
	} else {
		u32MinMsgNo = g_MsgCtr.u32MsgNo_FileSaved;
	}
	g_MsgCtr.u32MsgNo_ram = u32MinMsgNo;    /* ��MsgNo���ϵ�Msg���洢��ram�У������ȡ */
	pMsg = g_MsgCtr.MsgBuf;
	Swi_disable();
	/* ����û����ɵ���ϢҲ�����˴洢�������������"�������"�������Ҫ����u32MsgNo_Cmplt */
    if(g_MsgCtr.u32MsgNo_Cmplt < g_MsgCtr.u32MsgNo_ram) {
        g_MsgCtr.u32MsgNo_Cmplt = g_MsgCtr.u32MsgNo_ram;
    }
    /* ���ڲ��Ǵ���С��ItemNo��ʼ���������Ҫ��������� */
	for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
		if(pMsg->u32ItemNo <= u32MinMsgNo) {
			pMsg->uMsgId = 0;
			pMsg->u32ItemNo = 0;
			g_MsgCtr.u32CmpltFlag &= ~(1<<i);
		}
		pMsg++;
	}
	/* ��������Ϣָ��ָ�����Ϣ�Ѿ������ˣ�ָ��洢��Ϣ�������µ� */
    if(g_MsgCtr.pLastAddMsg->u32ItemNo == 0) {
        g_MsgCtr.pLastAddMsg = &g_MsgCtr.MsgSaved[g_MsgCtr.i8Pt_MsgSaved];
    }
	Swi_enable();

	/* �ú��������ڼ䣬���µ���Ϣ���룬��Ҫ�ٴ������洢 */
	if(u32MsgNo_ForAdd != g_MsgCtr.u32MsgNo_ForAdd) {
    	g_NvMemAcsCtr.bNeedSaveMsg = TRUE;
        Semaphore_post(g_DataAccessBlock.Sem_Op);   //Semaphore_post(SEM_NvMemAcs); /* �����洢���񣬰��²�������Ϣ��ӡ���������͸�DB */
	}
}

/*==========================================================================
| Description	: ��Msg�Ķ����Ʊ���ת��Ϊ�ı���ʽ
mcgs����� 60byte, ����ʱ�䲿��Ϊ22byte,��Ϣ���ֲ��ó���38byte--���������Ʊ��룬ʹ���ı�С��60byte
file = mcgs +23byte(���س�����)
net = mcgs + 52byte(������json�ַ���)
bTimeShort ��ʱ���ʽ��������ʱ���룬��Ҫ����VT��Ļ�������ʱ��Ҳ��������VT��Ļ
| In/Out/G var	: Buf��������False�����򷵻�TRUE
| Author		: Wang Renfei			Date	: 2016-07-16
\=========================================================================*/
BOOL PrintMsgCode(DATA_USER DataUser, uint8** ppBuf, uint8* pBufEnd, MSG_ITEM* pMsg, BOOL bMsgCmplt, BOOL bTimeShort)
{
	uint8* pBufStart = *ppBuf;
	uint8* pBuf = pBufStart;
	char* pPropText;

	/* ItemNo����: C1~C6 */
	uint32 u32ItemNo = pMsg->u32ItemNo;
	if(pMsg->uMsgId >= MAX_MSG_TYPE) {
		return FALSE;
	} else if(DataUser == DATA_USER_FILE) {
		if(pBuf + 84 > pBufEnd) {				/* Buf���ȼ��:��֤��ǰ��ȷ���ԵĽ�� */
			return FALSE;
		}
		PrintU32WithLen(&pBuf, u32ItemNo, 6); 	/* ItemNo����:�γ�6λ��ʽ */
		*pBuf++ = ' ';
	} else if(DataUser == DATA_USER_NET) {		/* Buf���ȼ��:��֤��ǰ��ȷ���ԵĽ�� */
		if(pBuf + 142 > pBufEnd) {
			return FALSE;
		}
		*pBuf++ = '{';
		PrintU32DatToJson(&pBuf, "no", u32ItemNo, 0);
		PrintStringNoOvChk(&pBuf, "\"time\":\"");
#if SUPPORT_SCREEN
	} else if(DataUser == DATA_USER_MCGS) {
		if(pBuf + MCGS_LEN_ITEM_CHAR > pBufEnd) {		/* Buf���ȼ�� */
			return FALSE;
		} else {
			pBufEnd = pBufStart + MCGS_LEN_ITEM_CHAR;	/* ���� MCGS ��Ļ����Ҫ�����������ڵ�ǰ���� */
		}
	} else if(DataUser == DATA_USER_VIEWTECH) {
		if(pBuf + VT_LEN_ITEM_BYTE > pBufEnd) {		/* Buf���ȼ�� */
			return FALSE;
		} else {
			pBufEnd = pBufStart + VT_LEN_ITEM_CHAR;	/* ����ViewTech��Ļ����Ҫ�����������ڵ�ǰ���� */
		}
	}
#endif

	if(pMsg->uMsgId != MSG_NULL) {	/* �������Ϣ���򲻴�ӡʱ�䣬��Ϊʱ��ܿ����Ǵ�� */
	    if(!bTimeShort) {
    		PrintT64(DataUser, &pBuf, pBufEnd, &pMsg->MsgTimeStamp);		/* ʱ�䲿�� */
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
	/* ֮�ϵ�Buf����������ڴ����ģ��Ѿ������������� */
	
	/* MsgProp + fMsgVal����, ���ڿ����в���㣬�����Ҫ�ֶϴ����ַ��� */
	if(g_Sys.u32LocalLang >= MAX_LANG_TYPE) {
		g_Sys.u32LocalLang = CHINESE;
	}
	pPropText = cnst_MsgProp[pMsg->uMsgId].pText[g_Sys.u32LocalLang];
	if(((cnst_MsgProp[pMsg->uMsgId].i8DataPosition >= 0)				/* �����֮ǰ��PropText���� */
			&& ((!PrintStringWithCharNum(&pBuf, pBufEnd, &pPropText, cnst_MsgProp[pMsg->uMsgId].i8DataPosition))
				|| (!PrintFM32(&pBuf, pBufEnd, pMsg->fMsgVal, cnst_MsgProp[pMsg->uMsgId].F32Meaning))))
		|| (!PrintString(&pBuf, pBufEnd, pPropText)))	/* �����֮���PropText���� */
	{
		if((DataUser == DATA_USER_MCGS) || (DataUser == DATA_USER_VIEWTECH)) {		/* ������Ļ�����ܲ�������һ�� */
		} else {
			return FALSE;
		}
	}

#if SUPPORT_SCREEN
	if(DataUser == DATA_USER_MCGS) {
		/* ���Bufδ����Ҫ���0��Ϊ�ַ����Ľ�β, ���˿��Բ��� */
		if(pBuf < pBufEnd) {
			*pBuf++ = 0;
			pBuf = pBufStart + MCGS_LEN_ITEM_CHAR;
		}
	} else if(DataUser == DATA_USER_VIEWTECH) {
		/* ���Bufδ����Ҫ���0��Ϊ�ַ����Ľ�β, ���˿��Բ��� */
		if(pBuf < pBufEnd) {
			*pBuf++ = 0;
			pBuf = pBufStart + VT_LEN_ITEM_CHAR;
		}
		*((uint16*)pBuf) = pMsg->uMsgId;	/* MsgID�ֶ� */
		pBuf += 2;
#endif
	} else if(DataUser == DATA_USER_FILE) {
		if(pBuf + 17 < pBufEnd) {		/* �����ܳ�16byte */
			/* ��� msg_id */
			*pBuf++ = ' ';
			PrintH16(&pBuf, pBufEnd, pMsg->uMsgId);
			if(!bMsgCmplt) {				/* ID�����*������Ϣд��ʱ��δ��� */
				*pBuf++ = '*';
			}
			
			/* ���msg_val */
			*pBuf++ = ' ';
			PrintF32(&pBuf, pBufEnd, pMsg->fMsgVal, 4);

			/* ���CRC���س����� */
			*pBuf++ = ' ';
			PrintH32(&pBuf, pBufEnd, CalCRC32ForFileByIni(pBufStart, pBuf - pBufStart, 0xFFFFFFFF));
			*pBuf = 0x0D;
			*(pBuf + 1) = 0x0A;
			pBuf += 2;
		} else {
			return FALSE;
		}
	} else if(DataUser == DATA_USER_NET) {	
		if(pBuf + 50 < pBufEnd) {		/* �����ܳ����42byte����8byte */
			if(bMsgCmplt) {			/* ���id */
				*pBuf++ = '"';
				*pBuf++ = ',';
				PrintH16DatToJson(&pBuf, "msg_id", pMsg->uMsgId);
				PrintF32DatToJson(&pBuf, "msg_val", pMsg->fMsgVal, 4);	/* ֵ���10���ַ� */
				pBuf--;
				*pBuf++ = '}';
				*pBuf++ = ',';
			} else {				/* ��Ϣ���������Ͳ����id */
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
| Description	: ��ConvertMsgCode2Text�����ϣ����EEProm��������ݽ�����������
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2018-7-24
\=========================================================================*/
CONV_MSG_ACQ_CODE_RES ConvertMsgEECode2Text(DATA_USER DataUser, uint8** ppBuf, uint8* pBufEnd, MSG_ITEM* pMsgEEItem)
{
	/* ������ */
	if((!isfinite(pMsgEEItem->fMsgVal)) || (pMsgEEItem->uMsgId == MSG_NULL)	|| (pMsgEEItem->uMsgId >= MAX_MSG_TYPE)) {
		return CONV_MSG_ACQ_CODE_ERR_DATA;
	} else if(PrintMsgCode(DataUser, ppBuf, pBufEnd, pMsgEEItem, TRUE, FALSE)) {
		return CONV_MSG_ACQ_CODE_SUC;
	} else {
		return CONV_MSG_ACQ_CODE_ERR_FULL;
	}
}

/* ����ram�����������һ�� �����Ϣ */
uint32 SeekCmpltMsgNo(void)
{
    uint32 u32CmpltMsgNo = g_MsgCtr.u32MsgNo_EESaved;
    int8 i, j;
    for(j = MSG_ITEM_BUF_LEN; j > 0; j--) {
        MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
        /* ��������u32CmpltMsgNo��С��MsgNo   */
        uint32 u32MinCmpltMsgNoInRam = 0xFFFFFFFFUL;
        int8 i8MsgPt = -1;          /* ���ڴ洢��MsgBuf�±� */
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
| Description	: A���¼�: ����¼���Ϣ����Ȼ�������������ǲ�������, �����Ǹú�����������
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2012-1-2
\=========================================================================*/
void AddMsgA_WithVal(uint16 uMsgId, float32 fMsgVal)
{
	if((!isfinite(fMsgVal)) || (uMsgId == MSG_NULL) || (uMsgId >= MAX_MSG_TYPE)) {	/* ������ */
		return;
#ifdef MSG_ABORNMAL_START
	} else if(uMsgId >= MSG_ABORNMAL_START) {	/* �쳣��Ϣ���� */
		g_Ctr.bBeepAbornmal = TRUE;
#endif
	}
	
	MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
	int8 i;
	BOOL bCmpltMsg = FALSE;
	Swi_disable();
	for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
		if(pMsg->uMsgId == MSG_NULL) {				/* Ѱ�ҿյĴ洢�ռ� */
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
        Semaphore_post(g_DataAccessBlock.Sem_Op);   //Semaphore_post(SEM_NvMemAcs);	/* �����洢���񣬰��²�������Ϣ��ӡ���������͸�DB */
    }
	Swi_enable();
}

/*==========================================================================
| Description	: B���¼�: ����¼���Ϣ(������)���ҽ�������
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-3-1
\=========================================================================*/
void AddMsgB_WithVal(uint16 uMsgId, float32 fMsgVal)
{
	if((!isfinite(fMsgVal)) || (uMsgId == MSG_NULL) || (uMsgId >= MAX_MSG_TYPE)) {	/* ������ */
		return;
#ifdef MSG_ABORNMAL_START
	} else if(uMsgId >= MSG_ABORNMAL_START) {	/* �쳣��Ϣ��������ʹ��������������ȻҪ�� */
		g_Ctr.bBeepAbornmal = TRUE;
#endif
	}
	if(g_MsgCtr.u32ShieldFlag[uMsgId/32] & (1UL<<(uMsgId%32))) { 		/* ������� */
		return;
	}
	
	MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
	int8 i;
	BOOL bCmpltMsg = FALSE;
	Swi_disable();
	for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
		if(pMsg->uMsgId == MSG_NULL) {				/* Ѱ�ҿյĴ洢�ռ� */
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
        Semaphore_post(g_DataAccessBlock.Sem_Op);   //Semaphore_post(SEM_NvMemAcs); 	/* �����洢���񣬰��²�������Ϣ��ӡ���������͸�DB */
    }
	Swi_enable();
}

/*==========================================================================
| Description	: C���¼�:�����¼���ֵ
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-10-3
\=========================================================================*/
void UpdateMsgC_WithVal(uint16 uMsgId, BOOL bMax1OrMin0, float32 fMsgVal)
{
	if((!isfinite(fMsgVal)) || (uMsgId == MSG_NULL)	|| (uMsgId >= MAX_MSG_TYPE)		/* ������ */
		|| (g_MsgCtr.u32ShieldFlag[uMsgId/32] & (1UL<<(uMsgId%32))))			/* ������� */
	{
		return;
	}

	/* ���������Ƿ��Ѿ�����δȷ�ϵ�Msg--��ȷ���Ǹ���ֵ�������Msg */
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

	/* ���Msg */
	if(bNeedAddNewMsg) {
		pMsg = g_MsgCtr.MsgBuf;
		Swi_disable();
		for(i = MSG_ITEM_BUF_LEN - 1; i >= 0; i--) {
			if(pMsg->uMsgId == 0) {						/* Ѱ�ҿյĴ洢�ռ� */
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
| Description	: C���¼�:ȷ���¼���Ϣ
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-10-8
\=========================================================================*/
void ConfirmMsgC(uint16 uMsgId)
{
	if((uMsgId == MSG_NULL) || (uMsgId >= MAX_MSG_TYPE)) {	/* ������ */
		return;		
#ifdef MSG_ABORNMAL_START
	} else if(uMsgId >= MSG_ABORNMAL_START) {				/* �쳣��Ϣ��������ʹ��������������ȻҪ�� */
		g_Ctr.bBeepAbornmal = TRUE;
#endif
	}
	if(g_MsgCtr.u32ShieldFlag[uMsgId/32] & (1UL<<(uMsgId%32))) {	/* ������� */
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
| Description	: C���¼�:�����Ϣ:����δȷ�ϵ���Ϣ������; ����ȷ�ϵ���Ϣ������
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-2-27
\=========================================================================*/
void FinishMsgC(uint16 uMsgId, BOOL bNeedBlock)
{
	/* ������� */
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
				if(bNeedBlock) {	/* ���� */
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
        Semaphore_post(g_DataAccessBlock.Sem_Op);   	//Semaphore_post(SEM_NvMemAcs);	/* �����洢���񣬰��²�������Ϣ��ӡ���������͸�DB */
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
| Description	: �ɼ���
	�ɼ��������ļ��еļ�¼��ʽΪ: AcqID + RecTime + Data(0~n) + LRC

	���ݴ洢����:
		1. ͨ���ӿں��������ݱ�����Buf�У��ӿ�����:"AcqID(ͨ�����ҹ�����Ч��������) + ����"������ʱ�����ItemNo
		2. �����ݴ�Buf�м���"ʱ�����ItemNo��У������"��ת��Ϊ�ı���д���ļ�(����������һ���ļ���)

	���Ʋ��ֶ�ȡ���ݹ���:
		1. ����(ACQID + ����ָ�� + ��ʼItemNo + ��ȡItem����)���ļ�ϵͳ��þ���ɼ���Ŀ����(������)��д��RAM
	
	ͨѶ/��ʾ�����������ı���ʽ(���ļ��ж�ȡ����)������:
		1. ����(ACQID + ��ʼItemNo + ��ȡItem����)���ļ�ϵͳ��þ���ɼ���Ŀ����(�ı�)���ύ��ͨѶ����

| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-6-6
\=========================================================================*/
#if ACQ_ITEM_DAT_NUM
BOOL ChkAcqIDWithReadAcqData(uint16 uReadAcqId, uint16 uSeekAcqId);
#define ACQ_WRITE_BUF_LEN			4			/* AcqдBuf��С */
#define ACQ_READ_BUF_LEN 			200			/* Acq��Buf��С */
#define ACQID_NULL					0			/* �յ�AcqID */
typedef struct {								/* �������� */
	REAL_TIME_VAR AcqTimeStamp; 				/* �¼�ʱ��� */
	uint32 u32ItemNo;							/* ����ItemNo�����¼��������� */
	uint16 uAcqId;
	uint16 uEEPromNo;							/* �洢��EEProm�е���� */
	float32 fD[ACQ_ITEM_DAT_NUM];
}ACQ_WRITE_ITEM;
typedef struct {								/* ��EEProm�д洢�����ݸ�ʽ */
	uint32 u32AcqTimeStamp;						/* ���洢:�ꡢ�¡��ա�ʱ */
	uint32 u32ItemNo;
	uint16 uAcqId;
	uint16 uEEPromNo;							/* �洢��EEProm�е���� */
	float32 fD[ACQ_ITEM_DAT_NUM];
}ACQ_EE_ITEM_VAR;
typedef struct {
	/* AcqNo���ڼ�¼Acq����� */
	uint32 u32AcqNo_ForAdd;						/* ��һ����ӵ�AcqNo */
	uint32 u32AcqNo_FileSaved;					/* �ļ����Ѿ��洢��AcqNo */
	uint32 u32AcqNo_EESaved;					/* EEProm�Ѿ��洢��AcqNo */
	uint32 u32AcqNo_ram;                        /* ram�洢��MsgNo��ʼ(����) */
	uint16 uEEPromNo_ForSave;					/* ���ڴ洢��һ����EEPromNo */
	
	/* ���Acq���ݽ���Buf */
	uint16 uBufPt_ForAdd;						/* ������д��ָ�룬ָ�����һ����д��Ram������ */
	uint16 uBufPt_ForFileSave;					/* ������д���ļ�ָ��, ָ�����һ���Ѵ����ļ������� */
	uint16 uBufPt_ForEEPromSave;				/* ������д��EEPromָ��, ָ�����һ���Ѵ���EEProm������ */
	ACQ_WRITE_ITEM AcqWriteBuf[ACQ_WRITE_BUF_LEN];/* д�뻺���� */

	/* ��NvMem�ж�ȡ���� */
	uint32 u32AcqSummNeedProc;					/* һ��λ���� cnst_Acq_Summ_Map_Table �ж�Ӧ������ �����ڸñ�̣ܶ�32bit�㹻����� */
	ACQ_READ_ITEM AcqReadBuf[ACQ_READ_BUF_LEN];
}ACQ_CTR;
SECTION(".NOT_ZeroInit") ACQ_CTR g_AcqCtr;
void InitAcqMdl(void)
{
	if(g_Sys.uRstCount) {	/* ���ϵ縴λ�ų�ʼ�� */
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
	
	/* ��ȡ���һ�е��к�:�������ļ��У�����ļ��ж�ȡʧ�����eeprom�� */
#if SUPPORT_FAT_FS		/* �����ļ�, ���ItemNo */
	uint32 u32LastItemNoFromFile = 0;
	/* ReadLastItemNoFromFile()����0˵����ʧ�� */
	for(i = 5; (i > 0) && ((u32LastItemNoFromFile = ReadLastItemNoFromFile(DATA_PAGE_ACQ, MSG_ACQ_ACS_V2)) == 0); i--) {
	}
	#if 0 // ɾ����acqV1 ItemNo��֧��,������δ��뱣�����Ա�������ҪԶ�̶�ȡʹ�� SUPPORT_MsgV1RW_AcqV1R
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
	uint16 uRemainItemInEEProm = EEPROM_ACQ_ITEM_NUM;	/* �����ܵĶ�ȡ���� */
	uint32 u32LastItemNoFromEEProm = 0; 				/* ����EEProm�����洢�����һ��ItemNo */
	uint16 uExpectEEPromNo = 0; 						/* ������EEPromNo */
	/* ����EEPromNo���������, ȡ���������ItemNo, EEPromNo;
	   ���һֱû�����䣬��˵�����ݴ洢λ�����ûص���ʼ��ȡĩλEEPromNo��һ��ΪEEPromNo */
	do {
		/* ��EEProm�ж������� */
		uint16 uCurReadItemNum = FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR); /* ��֤���������� */
		if(uCurReadItemNum > uRemainItemInEEProm) { 	/* ȷ�����ᳬ����Χ */
			uCurReadItemNum = uRemainItemInEEProm;
		}
		EEPROMRead((uint32_t*)g_NvMemAcsCtr.u8FileCont, u32EEPromAddr, uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR));
	
		/* �������� */
		ACQ_EE_ITEM_VAR* pSeekAcqBuf = (ACQ_EE_ITEM_VAR*)g_NvMemAcsCtr.u8FileCont;
		if(uExpectEEPromNo == 0) {	/* ��ʼ��uExpectEEPromNo */
			/* �²�����оƬ */
			if(((pSeekAcqBuf->u32ItemNo == 0xFFFFFFFFUL) && (pSeekAcqBuf->uAcqId == 0xFFFFUL) && (pSeekAcqBuf->uEEPromNo == 0xFFFFUL))
				|| (pSeekAcqBuf->uEEPromNo%EEPROM_ACQ_ITEM_NUM))	/* ��һ������EEPromNoӦ����EEPROM_ACQ_ITEM_NUM��������������������ƻ� */
			{
				u32LastItemNoFromEEProm = 0;
				break;
			} else {
				uExpectEEPromNo = pSeekAcqBuf->uEEPromNo;
			}
		}
		/* EEPromNo��Ԥ��Ĳ�һ��(������һ), ��˵���ҵ������ */
		for(i = uCurReadItemNum; (i > 0) && (pSeekAcqBuf->uEEPromNo == uExpectEEPromNo); i--) {
			u32LastItemNoFromEEProm = pSeekAcqBuf->u32ItemNo;
			uExpectEEPromNo++;
			pSeekAcqBuf++;
		}
	
		/* �н��������ѭ�� */
		if(i) {
			break;
		} else {	/* �޽����׼����һ��ѭ�� */
			uRemainItemInEEProm -= uCurReadItemNum;
			u32EEPromAddr += uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR);
		}
	} while(uRemainItemInEEProm);
	g_AcqCtr.uEEPromNo_ForSave = uExpectEEPromNo;
#endif

	/* ���� u32AcqNo_Saved */
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
| Description	: ���Acq����
| G/Out var		:
| Author		: Wang Renfei			Date	: 2016-11-04
\=========================================================================*/
void ResetAcqData(void)
{
	/* �ļ�ϵͳ���� */
	Swi_disable();
	InitDataWithZero((uint8*)(&g_AcqCtr), sizeof(g_AcqCtr));
	Swi_enable();
#if SUPPORT_FAT_FS
	uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
	PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Acq");
	#if SUPPORT_MsgV1RW_AcqV1R	/* ɾ��/acqĿ¼ */
		EmptyFold(pFilePath);
	#endif
	*pFilePath++ = '2';		/* ɾ�� /acq2Ŀ¼ */
	EmptyFold(pFilePath);
#endif

	/* EEPROM���� */
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
| Description	: ��Ӳɼ�������ֵ����������AcqItemBufС�ڵ���SAVE_ACQ_BUF_MAX_FREE_ITEM_NUM��������д��
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
		if(pAcqItem->AcqTimeStamp.u8Year > 19) {	/* RTCʱ����Ч */
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
	Semaphore_post(g_DataAccessBlock.Sem_Op);	//Semaphore_post(SEM_NvMemAcs);	/* ��Ҫ����AcqSumm���� */
}

/*==========================================================================
| Description	: ��Acq�Ķ����Ʊ���ת��Ϊ�ı���ʽ
mcgs����� 60byte, ����ʱ��+id����Ϊ20byte,��ֵ���ֲ��ó���40byte--����ܻ�ﵽ74�������������Ʊ��룬ʹ���ı�С��60byte
file = mcgs +28byte(���س�����)
net = mcgs + 49byte(������json�ַ���)
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-07-16
\=========================================================================*/
BOOL ConvertAcqCode2Text(DATA_USER DataUser, uint8** ppBuf, uint8* pBufEnd, ACQ_WRITE_ITEM* pAcqItem)
{
	uint8* pBufStart = *ppBuf;
	uint8* pBuf = pBufStart;
	int16 i;

	if(DataUser == DATA_USER_NET) {
		if(pBuf + 123 > pBufEnd) {	/* Buf���ȼ�� */
			return FALSE;
		}
		
		/* ItemNo����: C1~C6 */
		*pBuf++ = '{';
		PrintU32DatToJson(&pBuf, "no", pAcqItem->u32ItemNo, 0);
		PrintStringNoOvChk(&pBuf, "\"time\":\"");

		/* ʱ�䲿��: */
		*(pBuf + 18) = ',';
		*(pBuf + 17) = '"';
		*(pBuf + 16) = '0';
		*(pBuf + 15) = '0';
		*(pBuf + 14) = ':';
	} else if(DataUser == DATA_USER_FILE) {
		if(pBuf + 102 > pBufEnd) {	/* Buf���ȼ�� */
			return FALSE;
		}
		
		PrintU32WithLen(&pBuf, pAcqItem->u32ItemNo, 6); 	/* ItemNo����:�γ�6λ��ʽ */
		*pBuf++ = ' ';
		
	} else if(DataUser == DATA_USER_MCGS) {
		if(pBuf + MCGS_LEN_ITEM_CHAR > pBufEnd) {		/* Buf���ȼ�� */
			return FALSE;
		} else {
			pBufEnd = pBufStart + MCGS_LEN_ITEM_CHAR;	/* ������Ļ����Ҫ�����������ڵ�ǰ���� */
		}
	} else if(DataUser == DATA_USER_VIEWTECH) {
		if(pBuf + VT_LEN_ITEM_BYTE > pBufEnd) {		/* Buf���ȼ�� */
			return FALSE;
		} else {
			pBufEnd = pBufStart + VT_LEN_ITEM_CHAR;	/* ������Ļ����Ҫ�����������ڵ�ǰ���� */
		}
		
		/* ʱ�䲿��: */
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
		
		/* AcqID���� */
		PrintH16DatToJson(&pBuf, "acq_id", pAcqItem->uAcqId);
		/* ֮�ϵ�Buf����������ڴ����ģ��Ѿ������������� */
		
		/* ���ݲ���: */
		PrintStringNoOvChk(&pBuf, "\"data\":[");
		for(i = 0; i < ACQ_ITEM_DAT_NUM; i++) {
			*pBuf++ = '"';
			if(PrintF32(&pBuf, pBufEnd, pAcqItem->fD[i], -4) && (pBuf + 9 <= pBufEnd)) {	/* ����json��β��Ҫ5���ַ��������4���ַ� */
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
		
		/* AcqID���� */
		PrintH16(&pBuf, pBufEnd, pAcqItem->uAcqId);
		*pBuf++ = ' ';
		/* ֮�ϵ�Buf����������ڴ����ģ��Ѿ������������� */

		/* ���ݲ���: */
		for(i = 0; i < ACQ_ITEM_DAT_NUM; i++) {
			if(PrintF32(&pBuf, pBufEnd, pAcqItem->fD[i], -4) && (pBuf + 1 <= pBufEnd)) {
				*pBuf++ = ' ';
			} else {
				if((DataUser == DATA_USER_MCGS) || (DataUser == DATA_USER_VIEWTECH)) {	/* ������Ļ�����ܲ�������һ�� */
					break;
				} else {
					return FALSE;
				}
			}
		}
	}

	if(DataUser == DATA_USER_MCGS) {
		/* ���Bufδ����Ҫ���0��Ϊ�ַ����Ľ�β, ���˿��Բ��� */
		if(pBuf < pBufEnd) {
			*pBuf++ = 0;
			pBuf = pBufStart + MCGS_LEN_ITEM_CHAR;
		}
	} else if(DataUser == DATA_USER_VIEWTECH) {
		/* ���Bufδ����Ҫ���0��Ϊ�ַ����Ľ�β, ���˿��Բ��� */
		if(pBuf < pBufEnd) {
			*pBuf++ = 0;
			pBuf = pBufStart + VT_LEN_ITEM_CHAR;
		}
		*((uint16*)pBuf) = pAcqItem->uAcqId;	/* AcqID�ֶ� */
		pBuf += 2;
	} else if(DataUser == DATA_USER_FILE) {
		/* ���CRC���س����� */
		if(pBuf + 10 <= pBufEnd) {	/* �����ܳ�10 */
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

	/* ������ */
	if(!(isfinite(pAcqEE->fD[0]) && isfinite(pAcqEE->fD[1]) && isfinite(pAcqEE->fD[2]) 
		&& isfinite(pAcqEE->fD[3]) && isfinite(pAcqEE->fD[4])))
	{
		return CONV_MSG_ACQ_CODE_ERR_DATA;
	}

	/* ��ACQ_EE_ITEM_VARת����ACQ_ITEM_VAR */
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
| Description	: ��AcqBuf����Ķ�������ֵ�洢���ļ�
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-7-16
\=========================================================================*/
void SaveAcqToNvMem(void)
{
	ACQ_WRITE_ITEM* pAcqItem;
	int16 i;

#if SUPPORT_FAT_FS
	/* ��acq�洢���ļ��� */
	uint8* pBuf = g_NvMemAcsCtr.u8FileCont;
	BOOL bQuitFileSave = FALSE;	/* �����ļ����� */
	uint32 u32FileBufFirstItemNo = 0;
	uint32 u32AcqNo_FileSaved = g_AcqCtr.u32AcqNo_FileSaved;
	do {
		if(g_AcqCtr.uBufPt_ForFileSave >= ACQ_WRITE_BUF_LEN) {/* ָ�뷶Χ��飬����������λ�ã���AddAcqItem()����һ�� */
			g_AcqCtr.uBufPt_ForFileSave = 0;
		}
		
		/* �����Ƿ����µĿɴ洢���ݣ�����������洢 */
		BOOL bPrintBreak = FALSE;       /* �µ���Ϣ��������Ϣ�ļ����ˣ���Ҫ�γ��µ��ļ� */
		pAcqItem = &g_AcqCtr.AcqWriteBuf[g_AcqCtr.uBufPt_ForFileSave];
        if(u32FileBufFirstItemNo && (u32FileBufFirstItemNo/100000 != pAcqItem->u32ItemNo/100000)) {	/* ÿʮ����һ���ļ� */
            bPrintBreak = TRUE;
		} else if((pAcqItem->u32ItemNo > g_AcqCtr.u32AcqNo_FileSaved)
			&& ConvertAcqCode2Text(DATA_USER_FILE, &pBuf, &g_NvMemAcsCtr.u8FileCont[FILE_CONT_BUF_LEN], pAcqItem))
		{
			u32AcqNo_FileSaved = pAcqItem->u32ItemNo;
			g_AcqCtr.uBufPt_ForFileSave++;
			bQuitFileSave = (g_AcqCtr.uBufPt_ForFileSave%ACQ_WRITE_BUF_LEN == g_AcqCtr.uBufPt_ForAdd%ACQ_WRITE_BUF_LEN);/* �Ƿ���������� */
            /* ����¼��һ�γɹ���ӡ��ItemNo��Ҫ��Ȼ�洢�����Ǻ���Msg��ItemNo������ļ���bug(Ӧ���ǵ�һ������ʵ�ʲ���) */
			if(u32FileBufFirstItemNo == 0) {
    			u32FileBufFirstItemNo = pAcqItem->u32ItemNo;
    		}
		} else {
			bQuitFileSave = TRUE;
		}

		/* д���ļ� */
		if(bPrintBreak
		    || (bQuitFileSave && (pBuf != g_NvMemAcsCtr.u8FileCont))
			|| (pBuf - g_NvMemAcsCtr.u8FileCont + 100 > FILE_CONT_BUF_LEN))	/* ����AcqItem������100byte:����47byte */
		{
            /* �����/acq2/sn.txt */
            /* û�б����/acq/��/��/��.txt: acq���ݶ���Ӱ�첻��������������һ�µ��ļ��洢�Ƿ��� */
            if(SaveMsgOrAcq_V2(DATA_PAGE_ACQ, pBuf, u32FileBufFirstItemNo)) {
                /* Ϊ��һ�δ�ӡ��ʼ�� */
                g_AcqCtr.u32AcqNo_FileSaved = u32AcqNo_FileSaved;
                pBuf = g_NvMemAcsCtr.u8FileCont;
                u32FileBufFirstItemNo = 0;
            } else {
                bQuitFileSave = TRUE;
            }
		}
	} while(!bQuitFileSave);
#endif

	/* ����Ϣ�洢��eeprom�� */
#if CPU_0ST_1TI
    ACQ_EE_ITEM_VAR AcqEEItem;
	do {
		if(g_AcqCtr.uBufPt_ForEEPromSave >= ACQ_WRITE_BUF_LEN) {/* ָ�뷶Χ��飬����������λ�ã���AddAcqItem()����һ�� */
			g_AcqCtr.uBufPt_ForEEPromSave = 0;
		}
		/* �����Ƿ����µĿɴ洢���ݣ��������洢 */
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
		
		/* ����ָ�� */
		g_AcqCtr.uBufPt_ForEEPromSave++;
	} while(g_AcqCtr.uBufPt_ForEEPromSave%ACQ_WRITE_BUF_LEN != g_AcqCtr.uBufPt_ForAdd%ACQ_WRITE_BUF_LEN);
#endif

	/* ����Ѿ��洢��AcqBuf */
	/* ��Buf����������file���ܴ洢ʧ�ܣ���˵�u32AcqNo_FileSaved���̫���ʱ����������������� */
	uint32 u32MinAcqNo;
	if(g_AcqCtr.u32AcqNo_ForAdd - g_AcqCtr.u32AcqNo_FileSaved > ACQ_WRITE_BUF_LEN*3/4) {
		u32MinAcqNo = g_AcqCtr.u32AcqNo_ForAdd - ACQ_WRITE_BUF_LEN*2/3;
	} else {
		u32MinAcqNo = g_AcqCtr.u32AcqNo_FileSaved;
	}
	g_AcqCtr.u32AcqNo_ram = u32MinAcqNo;
	pAcqItem = g_AcqCtr.AcqWriteBuf;
	Swi_disable();				/* ���ڲ��Ǵ���С��ItemNo��ʼ���������Ҫ��������� */
	for(i = ACQ_WRITE_BUF_LEN; i > 0; i--) {
		if(pAcqItem->u32ItemNo <= u32MinAcqNo) {
			pAcqItem->u32ItemNo = 0;		/* �������ƻ����������Ա�ʶ */
			pAcqItem->uAcqId = ACQID_NULL;
		}
		pAcqItem++;
	}
	Swi_enable();
}

/*==========================================================================
| Description	: ������Acq-Summӳ���ϵ��(cnst_Acq_Summ_Map_Table)����ȡ���ݲ����д���
		���������¼�������: ����µ�Acq����(��ɴ���ָ�������ָ�벻һ��), �޸�����
		��Ҫ���ݲ�Ʒ����Acq-Summӳ���ϵ��(cnst_Acq_Summ_Map_Table)
		��ں�����RunAcqSummTask()����DataAccessTask������
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
				Swi_disable();				/* g_AcqCtr.u32AcqSummNeedProc���� */
				g_AcqCtr.u32AcqSummNeedProc &= ~(1UL<<i);
				Swi_enable();
				ProcAcqSummByMapItem(&cnst_Acq_Summ_Map_Table[i]);
			}
		}
	}
}

/* ����Acq-Summӳ�����AcqId�����ݼ�����AcqSumm */
int32 GetDataForAcq(uint16 uSeekAcqId, int32 i32ItemNum_InNeed_OutRemain);
void ProcAcqSummByMapItem(const ACQ_SUMM_MAP* pAcqSummMapItem)
{
	int32 i32ItemNum_NeedRead = 0;
	int16 i;

	/* ��ü��㳤��: ������Ǹ����������� */
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
	
	/* ��Ҫ����������������㣬��˵����Ҫ���д��� */
	if(i32ItemNum_NeedRead && (pAcqSummMapItem->AcqSummProc != NULL)) {
		i32ItemNum_NeedRead = 0 - i32ItemNum_NeedRead;	/* ��ǰ������Ҫ�޸�Ϊ�� */
		uint16 uItemNum_SucRead = GetDataForAcq(pAcqSummMapItem->uAcqId, i32ItemNum_NeedRead) - i32ItemNum_NeedRead;	/* ��ȡ���� */
		pAcqSummMapItem->AcqSummProc(g_AcqCtr.AcqReadBuf, uItemNum_SucRead);	/* �������� */
	}
}

/*==========================================================================
| Description	: ��ram��NvMem(�ļ�ϵͳ,eeprom)���ָ��AcqId�Լ����ȵ�Acq����
| In/Out/G var	: /���أ�ʣ�೤��/
| Author		: Wang Renfei			Date	: 2019-1-4
\=========================================================================*/
int32 GetDataForAcq(uint16 uSeekAcqId, int32 i32ItemNum_InNeed_OutRemain)
{
	uint32 u32ItemNo_InStart_OutEnd = 0xFFFFFFFFUL;
	ACQ_READ_ITEM* pAcqDataRead = g_AcqCtr.AcqReadBuf;
	int32 i, j;

	/* ������ */
	if((i32ItemNum_InNeed_OutRemain >= 0) || (i32ItemNum_InNeed_OutRemain < (int16)(0 - ACQ_READ_BUF_LEN))
		|| (u32ItemNo_InStart_OutEnd <= 1))
	{
		return 0;
	}

	/* �ȴ�ram�ж�ȡ���� */
	int16 iAcqPt = g_AcqCtr.uBufPt_ForAdd;
	for(i = ACQ_WRITE_BUF_LEN; (i > 0) && i32ItemNum_InNeed_OutRemain; i--) {
		iAcqPt--;
		if(iAcqPt < 0) {
			iAcqPt = ACQ_WRITE_BUF_LEN-1;
		}
		ACQ_WRITE_ITEM* pAcqItem = &g_AcqCtr.AcqWriteBuf[iAcqPt];
		if(!pAcqItem->u32ItemNo) {		/* �Ѿ������� */
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

	/* ��ȡNvMemǰ�ļ�� */
	if((i32ItemNum_InNeed_OutRemain == 0) || (u32ItemNo_InStart_OutEnd <= 1) 
	    || (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0)) 
	{
	    return i32ItemNum_InNeed_OutRemain;
	}
#if CPU_0ST_1TI	//ST����δ��뻹�����ƣ��������Ա��ּ���
    int32 i32ItemNum_BeforeFileRead = i32ItemNum_InNeed_OutRemain;  /* ���ڼ���ļ���ȡ�Ƿ�ɹ� */
#endif
#if SUPPORT_FAT_FS
	if(f_stat(DEVICE_PATH_String, NULL) == FR_OK) {				/* ���TF��: TF���������� */
    /* ֧���¾ɰ汾acq��ȡ
       �ȶ�ȡ DEVICE_PATH_String/acq2 ���û���㹻�����ݣ��ٶ�ȡ DEVICE_PATH_String/acq
       acq2�ǵ���Ŀ¼(/���.txt)��acq������Ŀ¼(/��/��/��.txt)
       Ϊ�˴�����ݣ�acq2 ģ�� acq���һ��Ŀ¼����i8SeekFilePathDepth=0������     */
		FIL File;
		DIR dir;
		FILINFO fno;
		uint8* pFilePathRecBuf[MSG_ACQ_FILE_PATH_DEPTH];		/* �ļ�·��ָ�룬���ڷֶ����ļ���(2)�����ļ���(1)�����ļ�(0) */
		uint8* pText;
		uint8* pLineStart;
		uint8* pLineEnd;										/* �ļ��е�����ָ�룬��ǰ�ı����п�ʼ���н�β */
		uint32 u32LastOpenDirOrFileItemNo[MSG_ACQ_FILE_PATH_DEPTH];	/* ��һ��������ItemNo */
		uint32 u32CurDirOrFileItemNo, u32CurSeekCloseItemNo;	/* ��ǰ���ļ������ļ���������ItemNo, ������Ŀ����ӽ����ļ�ItemNo */
		uint32 u32DataFilePt;
		uint32 u32ItemNo_ForSelectFileOrDir;					/* ��Ҫ��ȡ��ItemNo�����������ļ�����Ŀ¼,��Ҫ������"��" */
		int16 iSeekFilePathDepth;								/* ����������·����� */
		BOOL bFindFile, bCurDirSeekHaveResult;
		DATA_ACS_OPR DataAcsOpr = DATA_ACS_READ_ACQ;
        MSG_ACQ_ACS_VER eReadItemVer = MSG_ACQ_ACS_V2;
		
		/* ��ʼ������:��λ��Ӧ������·�� */
		uint8* pFilePath = g_NvMemAcsCtr.u8FilePath;
		PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], DEVICE_PATH_String);
		PrintString(&pFilePath, &g_NvMemAcsCtr.u8FilePath[FILE_PATH_BUF_LEN], "/Acq2");
		pFilePathRecBuf[MSG_ACQ_FILE_PATH_DEPTH - 1] = pFilePath-1; /* ָ��/Acq �Ա� MSG_ACQ_ACS_V1 ʱ��ʹ�� */
		pFilePathRecBuf[0] = pFilePath;                             /* ָ��/Acq2 */
		for(i = MSG_ACQ_FILE_PATH_DEPTH - 1; i >= 0; i--) {			/* �����һ��Ŀ¼ */
			u32LastOpenDirOrFileItemNo[i] = 0;
		}
		iSeekFilePathDepth = 0;
		u32ItemNo_ForSelectFileOrDir = u32ItemNo_InStart_OutEnd - 1;/* �����ļ�����Ŀ¼����Ҫ����"��"��ItemNo��Ŀ¼�����ļ� */

		while(i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr)) {
		/* ����Ŀ¼����λ��ǰ�����ȡ���ļ� */
		/* �����㷨: �����ǰĿ¼û��������Ŀ���ļ�/Ŀ¼�����˻���һ��Ŀ¼��������������ҵ�������һ��Ŀ¼������
			Ϊ�˷�ֹItemNo����������Ķ�ȡʧ�ܣ�������������, ��ȡ��������ItemNoһ��ҪС�ڵ���Ҫ��ȡ��ItemNo:
				1. �״�������Ŀ¼(ͨ���ж�u32LastOpenDirOrFileItemNo[] Ϊ��)��
					������С�ڵ���Ҫ��ȡ��ItemNo�����ģ���������洢��u32LastOpenDirOrFileItemNo[]
				2. ����������Ŀ¼(ͨ���ж�u32LastOpenDirOrFileItemNo[] ����)��
					���������ϴ�С�Ľ��������
			�������ʧ�ܣ����˻���һ��Ŀ¼��������
			Ŀ¼����: 0:���һ���ļ�, 1:���ļ���, 2:���ļ���		*/
			bFindFile = FALSE;
			do {
				/* ���ݵ�ǰ����Ƚ�������: ������Ӧ���ꡢ��Ŀ¼�Լ������ļ� */
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
						if((iSeekFilePathDepth != 0) ^ ((fno.fattrib & AM_DIR) == 0)) {		/* ���һ���������ļ������������Ŀ¼ */
						    if(eReadItemVer == MSG_ACQ_ACS_V2) {
    							u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname);
						    } else if(eReadItemVer == MSG_ACQ_ACS_V1) {
    							u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname + 2);
    						}
                            /* ֮ǰ���bug�������000000.txt��ʵ���������ĵ�һ�������1 */
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
					/* ����ò��Ŀ¼�����˳ɹ������� */
					} else if(bCurDirSeekHaveResult) {					/* ������Ŀ¼/�ļ� */
						u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] = u32CurSeekCloseItemNo;
						pFilePath += i;
						if(iSeekFilePathDepth) {					/* ������һ��Ŀ¼ */
							iSeekFilePathDepth--;
							pFilePathRecBuf[iSeekFilePathDepth] = pFilePath;
						} else {
							bFindFile = TRUE;
						}
                    } else if(eReadItemVer == MSG_ACQ_ACS_V2) {    /* acq2Ŀ¼�Ѿ����꣬�л���acq */
                        iSeekFilePathDepth = MSG_ACQ_FILE_PATH_DEPTH - 1;
                        u32LastOpenDirOrFileItemNo[0] = 0;  /* ����\acq2��ʱ��������� */
                        eReadItemVer = MSG_ACQ_ACS_V1;
					} else if(iSeekFilePathDepth < MSG_ACQ_FILE_PATH_DEPTH - 1) {	/* ��ֹͣ�����������Իص�������ѭ����������һ��Ŀ¼ */
						for(i = iSeekFilePathDepth; i >= 0; i--) {		/* �����һ��Ŀ¼ */
							u32LastOpenDirOrFileItemNo[i] = 0;
						}
						iSeekFilePathDepth++;
					} else {			/* ������ô�����Ѳ��� */
						DataAcsOpr = DATA_ACS_RES_EMPTY;
						break;
					}
				}
			} while(!bFindFile);
			if(DATA_ACS_RES(DataAcsOpr)) {
				break;
			}

			/* ��ȡ�ļ�:��ȡ���� */
			*pFilePath = 0;
			if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
				DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
				break;
			} else if(f_open(&File, (const TCHAR*)g_NvMemAcsCtr.u8FilePath, FA_READ) != FR_OK) {	/* �ļ���ʧ�� */
				break;												/* ����������ת������������� */
			} else {
				u32DataFilePt = f_size(&File);						/* ��ʼ���ļ�����ָ�� */
				pLineStart = g_NvMemAcsCtr.u8FileCont;
				do {
					/* ��ȡ�ļ� */
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

					/* �ڱ��ļ��н������� */
					for( ; i32ItemNum_InNeed_OutRemain && (pText >= g_NvMemAcsCtr.u8FileCont); pText--) {
						/* �н�β ���� �ļ���ͷ */
						if((*pText == 0x0A) || ((u32DataFilePt == 0) && (pText == g_NvMemAcsCtr.u8FileCont))) {
							pLineStart = pText;
							if(*pText == 0x0A) {
								pLineStart++;
							}

							/* ƥ������: ItemNo, AcqID, ������CRCУ�� */
							if(pLineStart + 10 < pLineEnd) {				/* �˳����У����߽������ֵ��� */
								uint32 u32CurLineItemNo = ReadU32(&pText, pLineEnd);
								uint16 uDaysFromOrigin = CalDaysFromOrigin(ReadU32(&pText, pLineEnd),		/* �� */
																			ReadU32(&pText, pLineEnd),	/* �� */
																			ReadU32(&pText, pLineEnd));	/* �� */
								pText += 6;	/* ���� ʱ:�� */
								uint16 uReadAcqId = ReadH32(&pText, pLineEnd);
								if((u32CurLineItemNo < u32ItemNo_InStart_OutEnd)		/* ��Ч���� */
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
							pText = pLineStart - 2;		/* �����س����з� */
						}
					}
				} while(i32ItemNum_InNeed_OutRemain && u32DataFilePt);
			}
		}
	}
#endif

#if CPU_0ST_1TI /* ����ļ�����ʧ�ܣ����EEProm�ж�ȡ */
	if((i32ItemNum_BeforeFileRead == i32ItemNum_InNeed_OutRemain)
	    && (g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt <= 0))
    {
		/* ������洢��λ�ÿ�ʼ���� */
		uint32 u32LastItemNo = 0xFFFFFFFFUL;
		uint16 uCurReadEEPromItemPt = g_AcqCtr.uEEPromNo_ForSave%EEPROM_ACQ_ITEM_NUM;		/* �������EEPromNo��ITEM_NUM�������Ĺ�ϵ */;
		uint16 uRemainItemInEEProm = EEPROM_ACQ_ITEM_NUM;	/* ��ʼ���ܵ�Item���� */
		uint16 uCurReadItemNum;
		while(uRemainItemInEEProm && i32ItemNum_InNeed_OutRemain) {
			/* �����ȡEEPromλ�� */
			if(uCurReadEEPromItemPt == 0) {
				uCurReadEEPromItemPt = EEPROM_ACQ_ITEM_NUM;
			}
			if(uCurReadEEPromItemPt > FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR)) {			/* ��֤���������� */
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

			/* �������� */
			ACQ_EE_ITEM_VAR* pAcqEE = (ACQ_EE_ITEM_VAR*)(&g_NvMemAcsCtr.u8FileCont[uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR)]);
			for(i = uCurReadItemNum; (i > 0) && i32ItemNum_InNeed_OutRemain; i--) {
				pAcqEE--;
				/* ItemNo����Ӧ���ǵݼ��ģ���������ת��˵�����ݿ��ܲ�����̨����ģ���Ҫ������ȡ */
				if((pAcqEE->u32ItemNo >= u32LastItemNo)
					|| (pAcqEE->u32ItemNo == 0))			/* ��Ч���� */
				{
					uRemainItemInEEProm = 0;
					break;
				/* ����Ҫ��ȡ������,���н��� */
				} else if((pAcqEE->u32ItemNo < u32ItemNo_InStart_OutEnd) && ChkAcqIDWithReadAcqData(pAcqEE->uAcqId, uSeekAcqId)) {
					if(isfinite(pAcqEE->fD[0]) && isfinite(pAcqEE->fD[1]) && isfinite(pAcqEE->fD[2])
						&& isfinite(pAcqEE->fD[3]) && isfinite(pAcqEE->fD[4])) 
					{
						pAcqDataRead->uAcqId = pAcqEE->uAcqId;
						pAcqDataRead->uDaysFromOrigin = CalDaysFromOrigin(pAcqEE->u32AcqTimeStamp/0x1000000UL,		/* �� */
																			(pAcqEE->u32AcqTimeStamp/0x10000UL)%0x100,	/* �� */
																			(pAcqEE->u32AcqTimeStamp/0x100UL)%0x100);	/* �� */
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

	/* �ɹ���ȡ */
	return i32ItemNum_InNeed_OutRemain;
}

/* ����ȡ��AcqID�Ƿ��Ƕ�ȡ������������ */
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
	/* ���޸���AcqSumm�������Ҫ���¼��� */
	if(pAcqSumm != NULL) {
		/* �ڴ�������������� pAcqSumm �Ĵ����� */
		int8 i, j;
		for(i = 0; cnst_Acq_Summ_Map_Table[i].uAcqId; i++) {
			for(j = ACQ_ITEM_DAT_NUM-1; (j >= 0) && (cnst_Acq_Summ_Map_Table[i].pAcqSumm[j] != pAcqSumm); j--) {
			}
			if(j >= 0) {	/* �ҵ���Ӧ��pCnstAcqSummProc������ */
				g_AcqCtr.u32AcqSummNeedProc |= (1<<i);
				Semaphore_post(SEM_NvMemAcs);
				break;
			}
		}
	}
#endif
}

/*==========================================================================
| ����ΪUI��ͨѶ����
\=========================================================================*/
/*==========================================================================
| Description	: �ı����ʽӿڣ���Ҫ������̫��ͨѶ/UI�ӿ�, ����ʵ��
	������ܷ������������: �ⲿ�ӿ�ע����Ӧ�������ڲ�ʵ����ɺ������
	Ϊ�˼��ڲ������߼���������ram�ж����Ƹ�ʽ����ת���ı��ĸ��ӣ�����ȡ�ļ�ϵͳ�е��ı�������ڲ�������Ҫ������:
	1. ����ӦMsgBuf/AcqBuf�ж����Ʊ���洢���ļ�ϵͳ��
	2. ���ļ�ϵͳ�ж�ȡ��Ӧ������
| In/Out/G var	: 
	DataUser	: �����û�, DATA_USER_NET���ݳ��Ȳ��̶���һ��Item�������̷�����һ��Item;
							DATA_USER_MCGS��DATA_USER_VIEWTECH�����ݳ��ȹ̶�ΪN��������䣬�ӵ�ItemNo��CRC����;
								�������Կ��ַ��������������������������(������pBufEnd)���������޸ķ��ص�Itemo, ItemNum
	DataPage					: Ŀǰ��֧��DATA_PAGE_MSG, DATA_PAGE_ACQ
	pU32ItemNo_InStart_OutEnd 	: ��ʼItemNo(����), ����ʵ�ʶ�ȡ�����һ��ItemNo--����ָ����ItemNo����������, 0xFFFFFFFFUL�����ȡTopic
	pI32ItemNum_InNeed_OutSuc 	: ��ȡ��Item����(��:ItemNo���ӣ��������;��:ItemNo���٣�����ǰ��)������ʵ�ʶ�ȡ��Item����(�����ţ���ָ��ǰ�����ɹ���ȡ)
	ppBuf						: Buf��㣬����д�룬���ط�����ɺ��ָ��
	pBufEnd						: Buf�յ㣬����д��
| Author		: Wang Renfei			Date	: 2016-07-16
\=========================================================================*/
DATA_ACS_OPR GetTextForMsgOrAcq(DATA_USER DataUser, DATA_PAGE DataPage, 
		uint32* pU32ItemNo_InStart_OutEnd, int32* pI32ItemNum_InNeed_OutSuc, uint8** ppBuf, uint8* pBufEnd)
{
	DATA_ACS_OPR DataAcsOpr = (DATA_ACS_OPR)(DATA_ACS_READ_DATA_PAGE - DataPage);
	int32 i32ItemNum_InNeed_OutRemain = *pI32ItemNum_InNeed_OutSuc;
	
	/* �Ƿ���Ҫ��NvMem, ��Buf�п� */
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
	    /* �ȴ���ȡ��� */
	    int8 i8TryCnt = 100;
		for(; (i8TryCnt > 0) && (!DATA_ACS_RES(g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr)); i8TryCnt--) {
			Task_sleep(OS_TICK_KHz*5);
		}

        /* ������ɶ�ȡ */
        if(DATA_ACS_RES(g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr)) {
			DataAcsOpr = g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr;
        /* �����ʱ�ˣ�Ҳ��һ�ֽ���ɣ��ȴ���ȡ�����˳������ж������ݾ�����ٰ� */
		} else {
			g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr = DATA_ACS_RES_TIMEOUT;	/* ������ȡ */
			Task_sleep(OS_TICK_KHz*100);	/* �ȴ� ReadTextForMsgOrAcq() �˳����������ֽ�����۲쵽ʵ����Ҫ��ʱ��7~14ms */
			DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
		}

		/* ���ݴ洢 */
		Swi_disable();
	    /* ��ʱ��δ���������޸�u32ItemNo_InStart_OutEndָ��ram�еĵ�һ�����ݣ�����߼����� */
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
		g_NvMemAcsCtr.ReadMsgOrAcqTxt[i8ReadTaskCtrNo].DataAcsOpr = DATA_ACS_END;	/* �ͷ� ReadMsgOrAcqTxt ���ƿ� */
	}
	
	/* ���� */
	if(DataAcsOpr == DATA_ACS_RES_FULL) {
		return DATA_ACS_RES_FULL;
	} else if(i32ItemNum_InNeed_OutRemain == 0) {
		return DATA_ACS_RES_SUC;
	} else {
		return DATA_ACS_RES_EMPTY;
	}
}

/* VT��Ļ��ҳ�����ȡһ�У���ʾLastAddMsg���� */
uint16 ReadLastAddMsgForVT(uint8* pBuf, uint8* pBufEnd, BOOL bTimeShort)
{
#if MAX_MSG_TYPE
	PrintMsgCode(DATA_USER_VIEWTECH, &pBuf, pBufEnd, g_MsgCtr.pLastAddMsg, TRUE, bTimeShort);
    return g_MsgCtr.pLastAddMsg->uMsgId;
#else
    PrintString(&pBuf, pBufEnd, "00:00:00 ��ӭʹ��");
    *pBuf = 0;
    return 0;
#endif

}

/*==========================================================================
| Description	: �����ǰ��(i32ItemNum_InNeed_OutRemain < 0)�����ȡu32ItemNo_InStart_OutEnd(����)֮ǰ�������е�����
				  ��������(i32ItemNum_InNeed_OutRemain > 0)�����ȡu32ItemNo_InStart_OutEnd(����)֮��������е�����
				�������ʾ��ȡ������Ҫ����ǰ���ItemNo, �����CRC��س�����
				�����ͨѶ��ȡ������Ҫ��������Ļس����У��ĳ�0
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-07-18
\=========================================================================*/
#define READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)  ((pReadMsgOrAcqCtr == NULL) || DATA_ACS_MISSION(pReadMsgOrAcqCtr->DataAcsOpr))
BOOL ReadTextFromLineForMsgOrAcq_MustSwiDisable(DATA_USER DataUser, DATA_ACS_OPR DataAcsOpr, uint8** ppBuf, uint8* pBufEnd, uint8* pLineStart, uint8* pLineEnd, BOOL bCrcOK);	/* ��һ�е��ļ����������Ӧ��Buf */
void ReadTextForMsgOrAcq(READ_MSG_ACQ_TXT* pReadMsgOrAcqCtr)
{
    /* ��Щ�����ɺ�������ʼ�� */
	uint8* pBuf;
	uint8* pBufEnd;
	uint32 u32ItemNo_InStart_OutEnd;
	int32 i32ItemNum_InNeed_OutRemain;
	DATA_USER DataUser;
	DATA_ACS_OPR DataAcsOpr;
	int16 i;

    /* ��ʼ�� */
    /* pReadMsgOrAcqCtr == NULL ��������Ϣ�����ݿ� */
    if(pReadMsgOrAcqCtr == NULL) {
    #if (!MAX_MSG_TYPE)
        return;
    #else
        if(g_MsgCtr.u32MsgNo_Cmplt <= g_MqttItemProc.u32MsgNo_HavePubedForDB) {  /* û�������ģ������˳� */
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
    	/* �����ȡ�����û�п�����Դ���򷵻� */
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
	
	/* ������ */
	if((i32ItemNum_InNeed_OutRemain == 0)
		|| ((i32ItemNum_InNeed_OutRemain < 0) && (u32ItemNo_InStart_OutEnd <= 1)))
	{
		DataAcsOpr = DATA_ACS_RES_INPUT_ERR;
	}

	/* ͷ������ */
    Swi_disable();      /* ��Ҫ��ֹ�������Ѿ���ȡ���ˣ����¶���ram�в������ܿ� */
    if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
        DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
    } else if((!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) || (!DATA_ACS_MISSION(DataAcsOpr))) {
        DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
	} else if(DataUser == DATA_USER_NET) {		/* ������Buf�ռ���topic����������� */
		*pBuf++ = '{';
		/* ��λ����: Topic�� */
		if((pReadMsgOrAcqCtr != NULL) && (u32ItemNo_InStart_OutEnd == 0xFFFFFFFFUL)) {
			if(0) {
		#if MAX_MSG_TYPE
			} else if(DataAcsOpr == DATA_ACS_READ_MSG) {
				u32ItemNo_InStart_OutEnd = g_MsgCtr.u32MsgNo_ForAdd;
				PrintU32DatToJson(&pBuf, "no", u32ItemNo_InStart_OutEnd, 0);
				PrintStringToJson(&pBuf, "title", "������Ϣ");
		#endif
		#if ACQ_ITEM_DAT_NUM
			} else if(DataAcsOpr == DATA_ACS_READ_ACQ) {
				u32ItemNo_InStart_OutEnd = g_AcqCtr.u32AcqNo_ForAdd;
				PrintU32DatToJson(&pBuf, "no", u32ItemNo_InStart_OutEnd, 0);
				PrintStringToJson(&pBuf, "title", "�ɼ�����");
		#endif
			} else {
				DataAcsOpr = DATA_ACS_RES_INPUT_ERR;
			}
		}
		PrintStringNoOvChk(&pBuf, "\"cont\":[");
		pBufEnd -= 3;		/* ��ҪΪ����json��βԤ��3��byte */
	} else if((DataUser == DATA_USER_MCGS) || (DataUser == DATA_USER_VIEWTECH)) {
		/* ��λ����: Topic�� */
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

	/* ���Ĵ��� */
	/* ����ǴӴ����С������Ҫ�ȼ����������ram�л����ļ���: �����ram�У��͵���ram���룻������ļ��У��ͷ����ļ� */
	if(i32ItemNum_InNeed_OutRemain < 0) {
		if(0) {
	#if MAX_MSG_TYPE
		} else if(DataAcsOpr == DATA_ACS_READ_MSG) {
            /* ��ram�У���δ�־û�����Ϣ */
			do {
				/* ����ram�е�MsgNo���� */
				MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
				uint32 u32MaxMsgNo = 0;
				int8 i8MsgPt = -1;			/* ���ڴ洢��MsgBuf�±� */
				for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
					/* ������ u32ItemNo ���㣬������Ϣȷ�� */
					if((u32MaxMsgNo < pMsg->u32ItemNo) && (pMsg->u32ItemNo < u32ItemNo_InStart_OutEnd)) {
						u32MaxMsgNo = pMsg->u32ItemNo;
						i8MsgPt = i;
					}
					pMsg++;
				}

				if(i8MsgPt < 0) { 	/* �Ѿ������� */
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

            /* ������MsgBuf�У��������Ѿ��־û�����Ϣ������ MsgSaved ���֡�
            ע: g_MsgCtr.MsgSaved�洢����ϢItemNo�������ģ���g_MsgCtr.i8Pt_MsgSavedָ�����һ���洢��Ϣ������˳����Ҽ��� */
            int8 i8Pt = g_MsgCtr.i8Pt_MsgSaved;
            int8 i;
            if((i8Pt >= 0) && (i8Pt <= MSG_SAVED_BUF_LEN - 1)) {    /* ȷ��ָ��û���� */
                for(i = MSG_SAVED_BUF_LEN; i32ItemNum_InNeed_OutRemain && (i > 0) && g_MsgCtr.MsgSaved[i8Pt].uMsgId; i--) {
					/* ��һ�����ϵ��ʼ���ģ�������Ϣ��������ǵ���������˵��MsgSaved�ǿյ� */
					if((i8Pt == 0) && (g_MsgCtr.MsgSaved[i8Pt].uMsgId == MSG_HuanYinShiYong)) {
						break;
                    /* ��ǰ�������MsgBuf�Ѿ��������ˣ�MsgSaved�����µ���
                       �Ҵ�ǰ��������MsgSaved���ҵ���һ��С��u32ItemNo_InStart_OutEnd���ǿ��õ� */
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
				if(!g_AcqCtr.AcqWriteBuf[iAcqPt].u32ItemNo) {			/* �Ѿ������� */
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
	} else if(DataAcsOpr == DATA_ACS_READ_MSG) {    /* ����(i32ItemNum_InNeed_OutRemain > 0) */
        /* ������MsgBuf�У��������Ѿ��־û�����Ϣ������ MsgSaved ����
        ע: g_MsgCtr.MsgSaved�洢����ϢItemNo�������ģ���g_MsgCtr.i8Pt_MsgSavedָ�����һ���洢��Ϣ������˳����Ҽ��� */
        int8 i8Pt = g_MsgCtr.i8Pt_MsgSaved;
        int8 i;
        if((i8Pt >= 0) && (i8Pt <= MSG_SAVED_BUF_LEN - 1)) {    /* ȷ��ָ��û���� */
            for(i = MSG_SAVED_BUF_LEN; i32ItemNum_InNeed_OutRemain && (i > 0); i--) {
                i8Pt++; /* ָ���һ���洢��Ϣ */
                if(i8Pt >= MSG_SAVED_BUF_LEN) {
                    i8Pt = 0;
                }
				/* ��һ�����ϵ��ʼ���ģ�������Ϣ��������ǵ���������˵��MsgSaved�ǿյ� */
				if((i8Pt == 0) && (g_MsgCtr.MsgSaved[i8Pt].uMsgId == MSG_HuanYinShiYong)) {
					break;
                } else if(!g_MsgCtr.MsgSaved[i8Pt].uMsgId) {   /* ��û�д洢��Ϣ */
                /* MsgSaved�Ӻ���ǰ��������������ItemNo���Ѿ���������Ҫ��ȡ�ģ���û�б�Ҫ������ */
                } else if(g_MsgCtr.MsgSaved[i8Pt].u32ItemNo > u32ItemNo_InStart_OutEnd + 1) {
                    break;
                /* �ǴӺ���ǰ������MsgSavedҲ�ǴӺ���ǰ��������Ҫ������ǡ��u32ItemNo_InStart_OutEnd + 1������Ч�� */
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
                && ((i32ItemNum_InNeed_OutRemain < 0)    /* ��ǰ�������ǰ������Ѿ�ram�ж�ȡ�ˣ�����ֻ��NvMem�ж�ȡ */
                /* �Ӻ���ǰ�����Ҷ�ȡ��������ram�л��У���û�б�ҪȥNvMem�ж�ȡ */
                #if MAX_MSG_TYPE        /* g_MsgCtr.u32MsgNo_ram �� ram�洢��MsgNo��ʼ(����) */
                    || ((DataAcsOpr == DATA_ACS_READ_MSG) && (u32ItemNo_InStart_OutEnd < g_MsgCtr.u32MsgNo_ram))
                #endif
                #if ACQ_ITEM_DAT_NUM    /* g_AcqCtr.u32AcqNo_ram �� ram�洢��AcqNo��ʼ(����) */
                    || ((DataAcsOpr == DATA_ACS_READ_ACQ) && (u32ItemNo_InStart_OutEnd < g_AcqCtr.u32AcqNo_ram))
                #endif
                    || 0);
    int32 i32ItemNum_BeforeFileRead = i32ItemNum_InNeed_OutRemain;
#endif
    /* ��NvMem�ж�ȡ:tf�� */
#if SUPPORT_FAT_FS
	if(bNeedReadNvMem && (f_stat(DEVICE_PATH_String, NULL) == FR_OK)) {
    /* ֧���¾ɰ汾msg(����acq,���治���о�)��ȡ
       �������ɶ�(i32ItemNum_InNeed_OutRemain<0)���ȶ�ȡ DEVICE_PATH_String/Msg2 ���ٶ�ȡ DEVICE_PATH_String/Msg
       �Ӿ����¶�(i32ItemNum_InNeed_OutRemain>0)���ȶ�ȡ DEVICE_PATH_String/Msg ���ٶ�ȡ DEVICE_PATH_String/Msg2
       msg2�ǵ���Ŀ¼(/���.txt)��msg������Ŀ¼(/��/��/��.txt)
       Ϊ�˴�����ݣ�msg2 ģ�� msg,acq���һ��Ŀ¼����i8SeekFilePathDepth=0������    */
		FIL File;
		DIR dir;
		FILINFO fno;
		uint8* pFilePathRecBuf[MSG_ACQ_FILE_PATH_DEPTH];	/* �ļ�·��ָ�룬���ڷֶ����ļ��С����ļ��С����ļ� */
		uint8* pText;
		uint8* pLineStart;									/* �ļ��е�����ָ�룬��ǰ�ı����п�ʼ���н�β */
		uint8* pLineEnd;									
		uint32 u32LastOpenDirOrFileItemNo[MSG_ACQ_FILE_PATH_DEPTH]; /* ��һ��������ItemNo */
		uint32 u32CurDirOrFileItemNo;/* ��ǰ���ļ������ļ���������ItemNo, ������Ŀ����ӽ����ļ�ItemNo */
		uint32 u32MaybeCurLineItemNo;						/* ��ǰ�к��ƶ�ֵ������CRCУ������ʱ�򣬾����ƶ�ֵ */
		uint32 u32MaybeFileLastItemNo;                      /* �ļ����һ��ItemNo��ȡ��βֵ����+1 */
		uint32 u32MaybeFileFirstItemNo;                     /* �ļ���һ��ItemNo�������ڴ򿪵��ļ��� */
		uint32 u32FileSaveItemNo;                           /* �ļ����洢�����һ��ItemNo */
		int16 i;
		int16 iSeekFilePathDepth;							/* ����������·�����:��2,��1,��0 */
		BOOL bFindFile, bCurDirSeekHaveResult;				/* �������ͣ���λ��Ӧ������·�� */
        triST tCRC_nF_0N_pS = 0;                            /* ��һ����ϢCRC״̬��<0:crc���� 0:��ʼ�� >0:crc�ɹ� */
        MSG_ACQ_ACS_VER eReadItemVer;

		/* ��ʼ���ļ�Ŀ¼ */
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
        u32MaybeCurLineItemNo = u32FileSaveItemNo + 1;              /* ������1 */
		pFilePathRecBuf[MSG_ACQ_FILE_PATH_DEPTH-1] = pFilePath;     /* ָ��/Msg */
		for(i = MSG_ACQ_FILE_PATH_DEPTH - 1; i >= 0; i--) {			/* �����һ��Ŀ¼ */
			u32LastOpenDirOrFileItemNo[i] = 0;
		}
        *pFilePath = 0;
		if(0) {
	#if SUPPORT_MsgV1RW_AcqV1R
		} else if((i32ItemNum_InNeed_OutRemain > 0)	/* �Ӿ����¶����ȶ�msg���ٶ�msg2 */
				&& (f_opendir(&dir, (const TCHAR*)g_NvMemAcsCtr.u8FilePath) == FR_OK))
		{
		    eReadItemVer = MSG_ACQ_ACS_V1;
            iSeekFilePathDepth = 2;
	#endif
		} else {
		    eReadItemVer = MSG_ACQ_ACS_V2;
		    *pFilePath++ = '2';                                     /* ����Ŀ¼������/msg2 */
            pFilePathRecBuf[0] = pFilePath;                         /* ָ��/Msg2 */
            iSeekFilePathDepth = 0;
		}

		/* ������Ҫ�������� */
		while(i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr)) {
            uint32 u32MachItemNo;       /* Ҫ��Ѱ���кţ����������ļ��Լ����ļ��ж�λ��Ϣ */
            if(i32ItemNum_InNeed_OutRemain > 0) {
                u32MachItemNo = u32ItemNo_InStart_OutEnd + 1;
            } else {
                u32MachItemNo = u32ItemNo_InStart_OutEnd - 1;
            }
            
			/* ����Ŀ¼����λ��ǰ�����ȡ���ļ� */
			/* �����㷨: �����ǰĿ¼û��������Ŀ���ļ�/Ŀ¼�����˻���һ��Ŀ¼��������������ҵ�������һ��Ŀ¼������
				Ŀ¼����: 0:���һ���ļ�, 1:���ļ���, 2:���ļ���		*/
			bFindFile = FALSE;
			do {
				/* ���ݵ�ǰ����Ƚ�������: ������Ӧ���ꡢ��Ŀ¼�Լ������ļ� */
				pFilePath = pFilePathRecBuf[iSeekFilePathDepth];
				*pFilePath = 0;
                if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
                    DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
                } else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {
                    DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
                } else if(f_opendir(&dir, (const TCHAR*)g_NvMemAcsCtr.u8FilePath) != FR_OK) {
                    if((eReadItemVer == MSG_ACQ_ACS_V1) && (iSeekFilePathDepth == 2)) { /* ����û��/msg ���� /msg2Ŀ¼ */
    					DataAcsOpr = DATA_ACS_RES_EMPTY;
                    } else {
    					DataAcsOpr = DATA_ACS_RES_OPEN_FILE_FAIL;
    				}
				} else {
				    /* �ڵ�ǰ��Ŀ¼�������� */
					*pFilePath++ = '/';
					if(u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] && (i32ItemNum_InNeed_OutRemain > 0)) { /* �����������Ҷ�u32ItemNo_InStart_OutEnd֮��� */
						u32MaybeFileFirstItemNo = 0xFFFFFF;
					} else {
						u32MaybeFileFirstItemNo = 0;
					}
					u32MaybeFileLastItemNo = u32FileSaveItemNo + 1;   /* ��β */
					bCurDirSeekHaveResult = FALSE;
					while((f_readdir(&dir, &fno) == FR_OK) && fno.fname[0]) {
						if((iSeekFilePathDepth != 0) ^ ((fno.fattrib & AM_DIR) == 0)) {
						    if(eReadItemVer == MSG_ACQ_ACS_V2) {
    							u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname);
						    } else if(eReadItemVer == MSG_ACQ_ACS_V1) {
    							u32CurDirOrFileItemNo = ReadU32FixLen((uint8*)fno.fname + 2);
    						}
                            /* ֮ǰ���bug�������000000.txt��ʵ���������ĵ�һ�������1 */
    						if(u32CurDirOrFileItemNo == 0) {
    						    u32CurDirOrFileItemNo = 1;
    						}

							/* �ļ����ܵ�����к�(��β) */
							if((u32MachItemNo < u32CurDirOrFileItemNo) && (u32CurDirOrFileItemNo < u32MaybeFileLastItemNo)) {
								u32MaybeFileLastItemNo = u32CurDirOrFileItemNo;
							}

							/* 	�����������ݵ�Ŀ¼�����ļ�--Ϊ�˷�ֹItemNo����������Ķ�ȡʧ�ܣ�������������:
								1. �״�������Ŀ¼(ͨ���ж�u32LastOpenDirOrFileItemNo[] Ϊ��)��
									������С�ڵ���Ҫ��ȡ��ItemNo�����ģ���������洢��u32LastOpenDirOrFileItemNo[]
									�������û�н������(i32ItemNum_InNeed_OutRemain > 0)�����u32LastOpenDirOrFileItemNo[]��λ1���Ա��������
								2. ����������Ŀ¼(ͨ���ж�u32LastOpenDirOrFileItemNo[] ����)��
									���(i32ItemNum_InNeed_OutRemain < 0)���������ϴ�С�Ľ��������
									���(i32ItemNum_InNeed_OutRemain > 0)���������ϴδ�Ľ������С��
							��ΪҪ������ǰ�к��ƶ�ֵu32MaybeCurLineItemNo����˾����ѵ�ǡ�õ��ļ�Ҳ���˳����� */
							if(((u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] == 0) && (u32MaybeFileFirstItemNo < u32CurDirOrFileItemNo)
							        && (u32CurDirOrFileItemNo <= u32MachItemNo))
								|| (u32LastOpenDirOrFileItemNo[iSeekFilePathDepth]		/* �������� */
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

					/* ����ò��Ŀ¼�����˳ɹ������� */
					if(bCurDirSeekHaveResult) { 		/* ������Ŀ¼/�ļ� */
						u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] = u32MaybeFileFirstItemNo;
						pFilePath += i;
						if(iSeekFilePathDepth) {
							iSeekFilePathDepth--;										/* ������һ��Ŀ¼���� */
							pFilePathRecBuf[iSeekFilePathDepth] = pFilePath;
						} else {
							bFindFile = TRUE;
							/* ��õ�ǰ�к��ƶ�ֵ: �������������õ�ǰ��Ŀ¼/�ļ�ItemNo��1,��Ϊ�������1����ǰ����������� */
							if(i32ItemNum_InNeed_OutRemain > 0) {
								u32MaybeCurLineItemNo = u32MaybeFileFirstItemNo - 1;	
							} else {
								u32MaybeCurLineItemNo = u32MaybeFileLastItemNo;	    /* ���Ǹ���βֵ��������һ�������� */
							}
						}
					/* ���(i32ItemNum_InNeed_OutRemain > 0), �Ҹ�Ŀ¼��ǰ�����������״�����������1���Է�����ж������� */
					} else if((i32ItemNum_InNeed_OutRemain > 0) && (u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] == 0)) {
						u32LastOpenDirOrFileItemNo[iSeekFilePathDepth] = 1;
					} else if(eReadItemVer == MSG_ACQ_ACS_V2) {   /* msg2Ŀ¼�Ѿ����꣬�л���msg */
						if(0) {
					#if SUPPORT_MsgV1RW_AcqV1R
						} else if(i32ItemNum_InNeed_OutRemain < 0) {	/* �������ɶ�������Ҫ����msgĿ¼ */
    					    iSeekFilePathDepth = MSG_ACQ_FILE_PATH_DEPTH - 1;
    					    u32LastOpenDirOrFileItemNo[0] = 0;  /* ����\msg2��ʱ��������� */
    					    eReadItemVer = MSG_ACQ_ACS_V1;
					#endif						
						} else {	/* �Ӿ����¶�������msgĿ¼��msg2Ŀ¼������������ */
                        	DataAcsOpr = DATA_ACS_RES_EMPTY;
						}
					} else if(iSeekFilePathDepth < MSG_ACQ_FILE_PATH_DEPTH - 1) {	/* ��ֹͣ��Ŀ¼������������һ��Ŀ¼ */
						for(i = iSeekFilePathDepth; i >= 0; i--) {					/* �����һ��Ŀ¼ */
							u32LastOpenDirOrFileItemNo[i] = 0;
						}
						iSeekFilePathDepth++;
					} else {							/* �Ѿ������������Ŀ¼�������Ѳ���, �����ֹͣ�������� */
					    if(i32ItemNum_InNeed_OutRemain < 0) {       /* ��ǰ�����������msg2Ŀ¼��msgĿ¼������������ */
                            DataAcsOpr = DATA_ACS_RES_EMPTY;
					    } else {    /* �Ӻ���Ǯ��������Ҫ����msg2Ŀ¼ */
    					    iSeekFilePathDepth = 0;
    					    pFilePath = pFilePathRecBuf[MSG_ACQ_FILE_PATH_DEPTH-1]; /* ָ��/msg */
                            *pFilePath++ = '2';                                     /* ����Ŀ¼������/msg2 */
                            pFilePathRecBuf[0] = pFilePath;                         /* ָ��/Msg2 */
    					    u32LastOpenDirOrFileItemNo[0] = 0;  /* ����\msg��ʱ��������� */
    					    eReadItemVer = MSG_ACQ_ACS_V2;
					    }
					}
				}
			} while((!bFindFile) && DATA_ACS_MISSION(DataAcsOpr));

			/* ��ȡ�ļ�: ��λ��Ӧ���кţ�Ϊ����ǿ��������(��ֹ��������Ķ�ȡʧ��)��ÿ��ȡһ�ж���u32CurReadItemNoƥ�� */
			*pFilePath = 0;
			if(DATA_ACS_RES(DataAcsOpr)) {				/* ǰ�������Ѿ��н���� */
			} else if((!bFindFile) || (f_open(&File, (const TCHAR*)g_NvMemAcsCtr.u8FilePath, FA_READ) != FR_OK)) {	/* �ļ���ʧ�� */
				DataAcsOpr = DATA_ACS_RES_OPEN_FILE_FAIL;           /* ����������ת������������� */
			} else {
                int32 i32DataFilePt = 0;
                UINT u32BufByteNum = 0;
                BOOL bFileCmplt = FALSE;                    /* �ļ��Ѿ�ȫ������Buf */
                BOOL bNextFile = FALSE;                     /* ��Ҫ������һ���ļ� */

                /* �����ļ���С��������һ����ȫ�������������ȶ�λ���ȡ */
                if(f_size(&File) < FILE_CONT_BUF_LEN) {     /* �ļ���С����һ����ȫ������ */
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
                    /* ����ļ��ϴ���Ҫ�ȿ��ٲ��ҵİ취��λ��Ϣ���ڵ�λ�ã�ÿ��������Ϣ���ȴ�Լ83.7Byte
                    ѭ����������Ҫ���������ؼ�������Ԥ����������ΧZone, ��Ԥ����Χ����ʼStartNo, ��Ԥ����Χ�ĳ�βEndNo
                    1. ��ȫ���ļ���Χ��ʼ��Zone
                    2. ÿ�ζ�ȡ����Sector(512B*2), Buf�м�(512B)���ļ��е�λ�����¾�����
                        HalfPt = (ReadNo - StartNo)*Zone_len/(EndNo - StartNo)
                    3. �����Ҫ��ȡ��ItemNo�ڸ�Sector�У���ֹͣ����
                        �����Ҫ��ȡ��ItemNo�ڸ�Sector�����Ը�Sector����Ϊ�µ�������Χ���ظ�23
                        �����Ҫ��ȡ��ItemNo�ڸ�Sectorǰ�����Ը�Sectorǰ��Ϊ�µ�������Χ���ظ�23                  */
                    /* ��ʼ���� */
                    uint16 uLineBLen;
                    if(u32MaybeFileLastItemNo < u32MaybeFileFirstItemNo) {
                        uLineBLen = 84; /* ÿ��������Ϣ���ȴ�Լ83.7Byte */
                    } else {
                        uint32 u32FileItemNum = u32MaybeFileLastItemNo - u32MaybeFileFirstItemNo;
                        uLineBLen = (f_size(&File) + u32FileItemNum/2)/u32FileItemNum;
                    }
                    i32DataFilePt = (u32MachItemNo - u32MaybeFileFirstItemNo)*uLineBLen;
                    triST tItemPlace_nBeforeSecotr_0InSector_pAfterSector;
                    do {
                        /* һ�ζ�ȡ2��Sector */
                        i32DataFilePt -= 256;   /* ���Ҫ��ȡ����Sectorǰһ���������ȡǰһ��Sector;��֮��ȡ��һ��Sector */
                        if(i32DataFilePt < 0) {
                            i32DataFilePt = 0;
                        } else if(i32DataFilePt > f_size(&File)) {
                            i32DataFilePt = f_size(&File) - 512;
                        }
                        i32DataFilePt = (i32DataFilePt/512)*512;    /* ָ��һ��������Sector */
                        f_lseek(&File, i32DataFilePt);
                        if(FILE_CONT_BUF_LEN > 1024) {
                            f_read(&File, g_NvMemAcsCtr.u8FileCont, 1024, &u32BufByteNum);  /* һ�ζ�����Sector */
                        } else {    /* ����һ�ζ�����Sector���κ�Buf̫С */
                            f_read(&File, g_NvMemAcsCtr.u8FileCont, FILE_CONT_BUF_LEN, &u32BufByteNum);
                        }
                        if(u32BufByteNum == 0) {
                            break;
                        } else {
                            pText = g_NvMemAcsCtr.u8FileCont;
                            if(i32DataFilePt == 0) {    /* ����Ǵ��ļ���ͷ��ȡ�����һ���ֽ�����ͷ */
                                pLineStart = pText;
                            } else {    /* ������Ҫ���ҵ�0x0A��Ϊ��һ�еĿ�ʼ */
                                pLineStart = NULL;
                            }
                        }

                        /* Sector�Ƿ������Ҫ��ȡ��ItemNo */
                        uint32 u32LineItemNo = u32MaybeCurLineItemNo;
                        tItemPlace_nBeforeSecotr_0InSector_pAfterSector = 1;
                        BOOL bBufEnd = FALSE;
                        BOOL bLineItemNoLsMatchItemNo = FALSE;
                        do {
                            bBufEnd = (pText >= &g_NvMemAcsCtr.u8FileCont[u32BufByteNum - 1]);
                            if((*pText == 0x0A)     /* �������н�β ���� �ļ���β */
                                || (bBufEnd && (i32DataFilePt + u32BufByteNum > f_size(&File))))
                            {
                                pLineEnd = pText + 1;
                                /* �ҵ���һ�У���ʼ��ȡ����Ҫ�˳����У����߽������ֵ��� */
                                if((pLineStart != NULL) && (pLineEnd - pLineStart > 10)) {
    								/* ����һ�����ݽ���CRCУ�飬����ȡ��Ӧ���б�� */
    								if(CheckTextWithCrc32ForMsgAndAcq(pLineStart, pLineEnd)) {
        								u32LineItemNo = ReadU32FixLen(pLineStart);
                                        uLineBLen = pLineEnd - pLineStart;   /* ����ʵ����Ϣ���Ƚ������� */
        								/* �����ǰSector�����Ѿ���������Ҫ��ȡ�ģ���û�б�Ҫ�������������� */
        								if(u32MachItemNo < u32LineItemNo) {
        							/* ��Ϣ���ܻ���ֿն���֮ǰ��u32MachItemNo > u32LineItemNo�������u32MachItemNo < u32LineItemNo */
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
    								/* ��ǰ�к���ƥ���ItemNo֮ǰ�������ñ�ʶ���Ա���Ϣ�ն� */
    								if(u32LineItemNo < u32MachItemNo) {
    								    bLineItemNoLsMatchItemNo = TRUE;
    								    
    								/* �ҵ��˶�Ӧ���� */
    								} else if(u32MachItemNo == u32LineItemNo) {
        								tItemPlace_nBeforeSecotr_0InSector_pAfterSector = 0;
    								    break;
    								}
    							}
                                pLineStart = pLineEnd;
                            }
                            pText++;
                        } while(!bBufEnd);

                        /* ��Ҫ������ȡ */
                        if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
                            DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
                        } else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {
                            DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;

                        /* Ҫ��ȡ��������Sector֮ǰ */
                        } else if(tItemPlace_nBeforeSecotr_0InSector_pAfterSector < 0) {
                            if(i32DataFilePt == 0) {    /* �Ѿ����ļ��Ŀ�ͷ�����ļ��Ѿ��ұ��ˣ���ҪѰ�������ļ� */
                                bNextFile = TRUE;
                            } else {
                                i32DataFilePt += pLineStart - g_NvMemAcsCtr.u8FileCont; /* �п�ͷ���ļ��е�λ�� */
                                i32DataFilePt -= (u32LineItemNo - u32MachItemNo)*uLineBLen;    /* �����������ļ��е�λ�� */
                            }

                        /* ӵ��ƥ���ѡ�� */
                        } else if(tItemPlace_nBeforeSecotr_0InSector_pAfterSector == 0) {
                            /* �������ǰ����Ϊ�˶ԽӺ���Ķ��Ĵ��룬��Ҫ����pText */
                            if(i32ItemNum_InNeed_OutRemain < 0) {
                                pText = pLineStart - 1; /* ָ����һ�еĽ�β:0x0A */
                            }

                        /* Ҫ��ȡ��������Sector֮�� */
                        } else if(tItemPlace_nBeforeSecotr_0InSector_pAfterSector > 0) {
                            if(i32DataFilePt + u32BufByteNum >= f_size(&File)) {  /* �Ѿ����ļ��Ľ�β�����ļ��ұ��ˣ���ҪѰ�������ļ� */
                                bNextFile = TRUE;
                            } else {
                                i32DataFilePt += pLineStart - g_NvMemAcsCtr.u8FileCont; /* �п�ͷ���ļ��е�λ�� */
                                /* �����������ļ��е�λ�ã�����pLineStartʵ��ָ����һ�е����� */
                                i32DataFilePt += (u32MachItemNo - u32LineItemNo - 1)*uLineBLen; 
                            }
                        }
                    } while(tItemPlace_nBeforeSecotr_0InSector_pAfterSector && (!bNextFile) && DATA_ACS_MISSION(DataAcsOpr));
                }

                /* ���ļ���û�У�Ҫȥ����һ���ļ�; ������Ҫ���� */
                if(bNextFile || DATA_ACS_RES(DataAcsOpr)) {
                
                /* ���ļ��ж�ȡ */
    			} else if(i32ItemNum_InNeed_OutRemain > 0) {
    				do {
    					/* �ڶ�ȡ���ļ�Buf�н��о����һ������ */
    					BOOL bBufEnd = FALSE;
    					do {
    					    bBufEnd = (pText >= &g_NvMemAcsCtr.u8FileCont[u32BufByteNum-1]);
                            /* �н�β ���� �ļ���β */
                            if((*pText == 0x0A) || (bBufEnd && bFileCmplt)) {
    							pLineEnd = pText + 1;
                                /* ��Ҫ������ȡ */
                                if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {   
                                    DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
                                    break;
                                } else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {
                                    DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
                                    break;
                                    
                                /* �ҵ���һ�У���ʼ��ȡ����Ҫ�˳����У����߽������ֵ��� */
    							} else if((pLineStart != NULL) && (pLineStart + 10 < pLineEnd)) {
    								/* ����һ�����ݽ���CRCУ�飬���鿴�Ƿ�����Ҫ��������? */
    								BOOL bCrcOK = CheckTextWithCrc32ForMsgAndAcq(pLineStart, pLineEnd);
    								if(bCrcOK) {
    									u32MaybeCurLineItemNo = ReadU32FixLen(pLineStart);
    								} else {	/* ���CRCУ��ʧ�ܣ�����ǰһ�е�ItemNo��1ʵ�� */
    									u32MaybeCurLineItemNo++;
    								}

    								if(u32MaybeCurLineItemNo > u32ItemNo_InStart_OutEnd) {			/* ����Ҫ��ȡ���� */
    									Swi_disable();		/* ��Ҫ��ֹ�������Ѿ���ȡ���� */
    									if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {
    										Swi_enable();
                                            DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
    										break;
                                        /* ����ͨѶ�����ƻ�����Ϣ���ͳ�����json����Ҳ���ܻ������⡣
                                            ���  ���CRC״̬�����仯�������json��Ϣ֡����ʹ�úû�json���� */
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
    										if(i32ItemNum_InNeed_OutRemain == 0) {  /* �Ѿ���ɶ�ȡ */
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

    					/* �ļ��Ѿ����� */
    					if(bFileCmplt) {
    					    break;
    					} else {    /* ������ȡ�ļ� */
        					i32DataFilePt += pLineEnd - g_NvMemAcsCtr.u8FileCont;		/* �޸��ļ���ȡָ�� */
        					f_lseek(&File, i32DataFilePt);
        					f_read(&File, g_NvMemAcsCtr.u8FileCont, FILE_CONT_BUF_LEN, &u32BufByteNum);
                            bFileCmplt = (u32BufByteNum < FILE_CONT_BUF_LEN);
        					if(u32BufByteNum) {
        						pText = g_NvMemAcsCtr.u8FileCont;
        						pLineStart = g_NvMemAcsCtr.u8FileCont;
        					} else {		/* �Ѿ����������ݣ��������ļ�����ʼ������һ���ļ� */
        						break;
        					}
        				}
    				} while(i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr));
    			} else {									/* i32ItemNum_InNeed_OutRemain < 0, ����ļ�ĩβ��ʼ�� */
    				do {
    					/* �ڶ�ȡ���ļ�Buf�н��о����һ������ */
    					BOOL bBufEnd = FALSE;
    					do {
    						/* �н�β ���� �ļ���ͷ */
                            bBufEnd = (pText <= g_NvMemAcsCtr.u8FileCont);
                            if((*pText == 0x0A) || (bBufEnd && bFileCmplt)) {
    							pLineStart = pText;
    							if(*pText == 0x0A) {
    								pLineStart++;
    							}

                                /* �쳣��Ҫ�˳������ */
                                if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
                                    DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
                                    break;
                                } else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {
                                    DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
                                    break;
                                    
                                 /* �ҵ���һ�У���ʼ��ȡ����Ҫ�˳����У����߽������ֵ��� */
    							} else if((pLineEnd != NULL) && (pLineStart + 10 < pLineEnd)) {
    								/* ����һ�����ݽ���CRCУ�飬���鿴�Ƿ�����Ҫ��������? */
                                    BOOL bCrcOK = CheckTextWithCrc32ForMsgAndAcq(pLineStart, pLineEnd);
                                    if(bCrcOK) {
    									u32MaybeCurLineItemNo = ReadU32FixLen(pLineStart);
    								} else {	/* ���CRCУ��ʧ�ܣ�����ǰһ�е�ItemNo��1ʵ�� */
    									u32MaybeCurLineItemNo--;
    								}

    								if(u32MaybeCurLineItemNo < u32ItemNo_InStart_OutEnd) {			/* ����Ҫ��ȡ���� */
    									Swi_disable();		/* ��Ҫ��ֹ�������Ѿ���ȡ���� */
    									if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {									
    										Swi_enable();
                                            DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
                                            break;
                                        /* ����ͨѶ�����ƻ�����Ϣ���ͳ�����json����Ҳ���ܻ������⡣
                                            ���  ���CRC״̬�����仯�������json��Ϣ֡����ʹ�úû�json���� */
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

                        /* ��ȡ�ļ� */
                        if(bFileCmplt) {     /* �ļ��Ѿ����� */
                            break;
                        } else {            /* ������ȡ�ļ� */
                            u32BufByteNum = i32DataFilePt + (pLineStart - g_NvMemAcsCtr.u8FileCont);    /* ���㻹ʣ��������δ��ȡ */
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
	/* ��NvMem�ж�ȡ: eeprom��ǰ���ļ���û�ж�ȡ�����ٽ���eeprom��ȡ */
    /* bNeedReadNvMem ������ļ�ǰ���Ƿ�Ҫ������ط�������Ӧ�ø���bNeedReadNvMem�ģ�������Ϊ��eepromǰ���Ƕ��ļ�û�л�����ݣ���Ҳ��û�б�Ҫ������ */
	if(bNeedReadNvMem && (i32ItemNum_BeforeFileRead == i32ItemNum_InNeed_OutRemain)) {
        /* ǰ���ļ�û�л�ȡ�κ����ݣ�����eeprom��ȡ��Ҫ�ٴγ�ʼ����
            ����DataAcsOpr������ DATA_ACS_RES_EMPTY, ����DATA_ACS_RES_OPEN_FILE_FAIL */
        if(pReadMsgOrAcqCtr == NULL) {
            DataAcsOpr = DATA_ACS_READ_MSG;
        } else {
            DataAcsOpr = pReadMsgOrAcqCtr->DataAcsOpr;
        }
        
		/* ������洢��λ�ÿ�ʼ���� */
		if(0) {
	#if MAX_MSG_TYPE
		} else if(DataAcsOpr == DATA_ACS_READ_MSG) {
			uint32 u32LastItemNo = 0xFFFFFFFFUL;
			uint16 uCurReadEEPromItemPt = g_MsgCtr.uEEPromNo_ForSave%EEPROM_MSG_ITEM_NUM;		/* �������EEPromNo��ITEM_NUM�������Ĺ�ϵ */
			uint16 uRemainItemInEEProm = EEPROM_MSG_ITEM_NUM;		/* ��ʼ���ܵ�Item���� */
			uint16 uCurReadItemNum;
			int16 i;
			if(i32ItemNum_InNeed_OutRemain > 0) {
				/* �����µ�������ǰ������������Ҫ����ItemNo�����Item */
				do {
					/* �����ȡEEPromλ�á���ȡ���� */
					if(uCurReadEEPromItemPt == 0) {
						uCurReadEEPromItemPt = EEPROM_MSG_ITEM_NUM;
					}
					if(uCurReadEEPromItemPt > FILE_CONT_BUF_LEN/sizeof(MSG_ITEM)) {			/* ��֤���������� */
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
					
					/* �������� */
					MSG_ITEM* pMsgEE = (MSG_ITEM*)(&g_NvMemAcsCtr.u8FileCont[uCurReadItemNum*sizeof(MSG_ITEM)]);
					for(i = uCurReadItemNum; i > 0; i--) {
						pMsgEE--;
						/* ItemNo����Ӧ���ǵ����ģ���������ת��˵������ǰ�����ݿ��ܲ�����̨����� */
						if((pMsgEE->u32ItemNo >= u32LastItemNo)
							|| (pMsgEE->u32ItemNo <= u32ItemNo_InStart_OutEnd))	/* ItemNo�Ѿ����˶����� */
						{
							break;
						}
						u32LastItemNo = pMsgEE->u32ItemNo;
					}

					/* �н��������ѭ�� */
					if(i) {
						uRemainItemInEEProm -= (uCurReadItemNum - i);
						uCurReadEEPromItemPt += i;
						break;
					} else {
						uRemainItemInEEProm -= uCurReadItemNum;
					}
				} while(uRemainItemInEEProm);

				/* �Ѿ��ҵ���㿪ʼ��ȡ */
				uRemainItemInEEProm = EEPROM_MSG_ITEM_NUM - uRemainItemInEEProm;	/* ǰ����ʣ��δ���� */
				while(uRemainItemInEEProm && i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr)) {
					/* �����ȡEEPromλ�� */
					if(uCurReadEEPromItemPt >= EEPROM_MSG_ITEM_NUM) {
						uCurReadEEPromItemPt = 0;
					}
					uCurReadItemNum = EEPROM_MSG_ITEM_NUM - uCurReadEEPromItemPt;
					if(uCurReadItemNum > FILE_CONT_BUF_LEN/sizeof(MSG_ITEM)) {	/* ��֤���������� */
						uCurReadItemNum = FILE_CONT_BUF_LEN/sizeof(MSG_ITEM);
					}
					if(uCurReadItemNum > uRemainItemInEEProm) {
						uCurReadItemNum = uRemainItemInEEProm;
					}
					EEPROMRead((uint32_t*)g_NvMemAcsCtr.u8FileCont, EEPROM_ADDR_MSG + uCurReadEEPromItemPt*sizeof(MSG_ITEM),
								uCurReadItemNum*sizeof(MSG_ITEM));
					uCurReadEEPromItemPt += uCurReadItemNum;
					uRemainItemInEEProm -= uCurReadItemNum;
					
					/* �������� */
					MSG_ITEM* pMsgEE = (MSG_ITEM*)(g_NvMemAcsCtr.u8FileCont);
					Swi_disable();		/* ��Ҫ��ֹ�������Ѿ���ȡ���� */
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
			} else {	/* �����µ�������ǰ������ */
				while(uRemainItemInEEProm && i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr)) {
					/* �����ȡEEPromλ�� */
					if(uCurReadEEPromItemPt == 0) {
						uCurReadEEPromItemPt = EEPROM_MSG_ITEM_NUM;
					}
					if(uCurReadEEPromItemPt > FILE_CONT_BUF_LEN/sizeof(MSG_ITEM)) {			/* ��֤���������� */
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

					/* �������� */
					MSG_ITEM* pMsgEE = (MSG_ITEM*)(&g_NvMemAcsCtr.u8FileCont[uCurReadItemNum*sizeof(MSG_ITEM)]);
					Swi_disable();		/* ��Ҫ��ֹ�������Ѿ���ȡ���� */
                    if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
                        DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
					} else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {					    
                        DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
					} else {
						for(i = uCurReadItemNum; (i > 0) && i32ItemNum_InNeed_OutRemain; i--) {
							pMsgEE--;
							/* ItemNo����Ӧ���ǵ����ģ���������ת��˵�����ݿ��ܲ�����̨����ģ���Ҫ������ȡ */
							if((pMsgEE->u32ItemNo >= u32LastItemNo)
								|| (pMsgEE->u32ItemNo == 0))	/* ��Ч���� */
							{
								uRemainItemInEEProm = 0;
								break;
							/* ����Ҫ��ȡ������,���н��� */
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
			uint16 uCurReadEEPromItemPt = g_AcqCtr.uEEPromNo_ForSave%EEPROM_ACQ_ITEM_NUM;		/* �������EEPromNo��ITEM_NUM�������Ĺ�ϵ */
			uint16 uRemainItemInEEProm = EEPROM_ACQ_ITEM_NUM;		/* ��ʼ���ܵ�Item���� */
			uint16 uCurReadItemNum;
			int16 i;
			if(i32ItemNum_InNeed_OutRemain > 0) {
				/* �����µ�������ǰ������������Ҫ����ItemNo�����Item */
				do {
					/* �����ȡEEPromλ�á���ȡ���� */
					if(uCurReadEEPromItemPt == 0) {
						uCurReadEEPromItemPt = EEPROM_ACQ_ITEM_NUM;
					}
					if(uCurReadEEPromItemPt > FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR)) {			/* ��֤���������� */
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
					
					/* �������� */
					ACQ_EE_ITEM_VAR* pAcqEE = (ACQ_EE_ITEM_VAR*)(&g_NvMemAcsCtr.u8FileCont[uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR)]);
					for(i = uCurReadItemNum; i > 0; i--) {
						pAcqEE--;
						/* ItemNo����Ӧ���ǵ����ģ���������ת��˵������ǰ�����ݿ��ܲ�����̨����� */
						if((pAcqEE->u32ItemNo >= u32LastItemNo)
							|| (pAcqEE->u32ItemNo <= u32ItemNo_InStart_OutEnd))		/* ItemNo�Ѿ����˶����� */
						{
							break;
						}
						u32LastItemNo = pAcqEE->u32ItemNo;
					}
					
					/* �н��������ѭ�� */
					if(i) {
						uRemainItemInEEProm -= (uCurReadItemNum - i);
						uCurReadEEPromItemPt += i;
						break;
					} else {
						uRemainItemInEEProm -= uCurReadItemNum;
					}
				} while(uRemainItemInEEProm);

				/* �Ѿ��ҵ���㿪ʼ��ȡ */
				uRemainItemInEEProm = EEPROM_ACQ_ITEM_NUM - uRemainItemInEEProm;	/* ǰ����ʣ��δ���� */
				while(uRemainItemInEEProm && i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr)) {
					/* �����ȡEEPromλ�� */
					if(uCurReadEEPromItemPt >= EEPROM_ACQ_ITEM_NUM) {
						uCurReadEEPromItemPt = 0;
					}
					uCurReadItemNum = EEPROM_ACQ_ITEM_NUM - uCurReadEEPromItemPt;
					if(uCurReadItemNum > FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR)) {		/* ��֤���������� */
						uCurReadItemNum = FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR);
					}
					if(uCurReadItemNum > uRemainItemInEEProm) {
						uCurReadItemNum = uRemainItemInEEProm;
					}
					EEPROMRead((uint32_t*)g_NvMemAcsCtr.u8FileCont, EEPROM_ADDR_ACQ + uCurReadEEPromItemPt*sizeof(ACQ_EE_ITEM_VAR),
								uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR));
					uCurReadEEPromItemPt += uCurReadItemNum;
					uRemainItemInEEProm -= uCurReadItemNum;
					
					/* �������� */
					ACQ_EE_ITEM_VAR* pAcqEE = (ACQ_EE_ITEM_VAR*)(g_NvMemAcsCtr.u8FileCont);
					Swi_disable();		/* ��Ҫ��ֹ�������Ѿ���ȡ���� */
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
			} else {	/* �����µ�������ǰ������ */
				while(uRemainItemInEEProm && i32ItemNum_InNeed_OutRemain && DATA_ACS_MISSION(DataAcsOpr)) {
					/* �����ȡEEPromλ�� */
					if(uCurReadEEPromItemPt == 0) {
						uCurReadEEPromItemPt = EEPROM_ACQ_ITEM_NUM;
					}
					if(uCurReadEEPromItemPt > FILE_CONT_BUF_LEN/sizeof(ACQ_EE_ITEM_VAR)) {			/* ��֤���������� */
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

					/* �������� */
					ACQ_EE_ITEM_VAR* pAcqEE = (ACQ_EE_ITEM_VAR*)(&g_NvMemAcsCtr.u8FileCont[uCurReadItemNum*sizeof(ACQ_EE_ITEM_VAR)]);
					Swi_disable();		/* ��Ҫ��ֹ�������Ѿ���ȡ���� */
					if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
                        DataAcsOpr = DATA_ACS_RES_URGENT_QUIT;
					} else if(!READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr)) {					    
                        DataAcsOpr = DATA_ACS_RES_NOT_CMPLT;
					} else {
						for(i = uCurReadItemNum; (i > 0) && i32ItemNum_InNeed_OutRemain; i--) {
							pAcqEE--;
							/* ItemNo����Ӧ���ǵݼ��ģ���������ת��˵�����ݿ��ܲ�����̨����ģ���Ҫ������ȡ */
							if((pAcqEE->u32ItemNo >= u32LastItemNo)
								|| (pAcqEE->u32ItemNo == 0))			/* ��Ч���� */
							{
								uRemainItemInEEProm = 0;
								break;
							/* ����Ҫ��ȡ������,���н��� */
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
        /* ǰ���NvMem������գ����DataAcsOpr�޸ĳ�DATA_ACS_RES_EMPTY����������£�����Ӧ�ü����� */
        if(DataAcsOpr == DATA_ACS_RES_EMPTY) {
            if(pReadMsgOrAcqCtr == NULL) {
                DataAcsOpr = DATA_ACS_READ_MSG;
            } else {
                DataAcsOpr = pReadMsgOrAcqCtr->DataAcsOpr;
            }
        }
        
        /* ����Ǵ�С����������Ҫ����ram�е����� */
        if((i32ItemNum_InNeed_OutRemain > 0) && DATA_ACS_MISSION(DataAcsOpr)) {
            if(0) {
        #if MAX_MSG_TYPE
            } else if(DataAcsOpr == DATA_ACS_READ_MSG) {
                do {
                    /* ����ram�е�MsgNo��С�� */
                    MSG_ITEM* pMsg = g_MsgCtr.MsgBuf;
                    uint32 u32MinMsgNo = 0xFFFFFFFF;
                    int8 i8MsgPt = -1;          /* ���ڴ洢��MsgBuf�±� */
                    for(i = 0; i < MSG_ITEM_BUF_LEN; i++) {
                        if((u32MinMsgNo > pMsg->u32ItemNo) && (pMsg->u32ItemNo > u32ItemNo_InStart_OutEnd)) {
                            u32MinMsgNo = pMsg->u32ItemNo;
                            i8MsgPt = i;
                        }
                        pMsg++;
                    }
                    if(i8MsgPt < 0) { /* �Ѿ������� */
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
                /* ����ram�е�AcqNo��С�� */
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
                
                /* ��ȡ���� */
                if(u32MinAcqNo != 0xFFFFFFFFUL) {   /* ǰ��������н�� */
                    for(i = ACQ_WRITE_BUF_LEN; (i > 0) && i32ItemNum_InNeed_OutRemain; i--) {
                        if(g_AcqCtr.AcqWriteBuf[iAcqPt].u32ItemNo <= u32ItemNo_InStart_OutEnd) {    /* �Ѿ������� */
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

        /* ���Ͻ�β */
        if(DataUser == DATA_USER_NET) { /* �����Լ����Ҫ3��byte����Ҫ��һ��ʼ��ʱ��Ԥ������ */
            pBuf--;
            if(*pBuf == '[') {          /* ˵�������ǿյģ�Ϊ�˱���'['����� */
                pBuf++;
            }
            *pBuf++ = ']';
            *pBuf++ = '}';
        
        /* �Ѿ����꣬��û����������������Guiʣ����ʾBuf */
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
                *((uint16*)pBuf) = 0;   /* MsgID �� AcqID�ֶ� */
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

    /* ��������ֱ�Ӿ������﷢�� */
    if(pReadMsgOrAcqCtr == NULL) {      /* ���緢�������ݿ� */
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

    /* ͨ��GetTextForMsgOrAcq()��ӵ����񣬷�ֹ�������Ѿ���ȡ���� */
	} else if(READ_MSG_ACQ_MISSION(pReadMsgOrAcqCtr) || (pReadMsgOrAcqCtr->DataAcsOpr == DATA_ACS_RES_TIMEOUT)) {
        pReadMsgOrAcqCtr->pBuf = pBuf;
        pReadMsgOrAcqCtr->u32ItemNo_InStart_OutEnd = u32ItemNo_InStart_OutEnd;
        pReadMsgOrAcqCtr->i32ItemNum_InNeed_OutRemain = i32ItemNum_InNeed_OutRemain;
        pReadMsgOrAcqCtr->DataAcsOpr = DataAcsOpr;
    }
	Swi_enable();
}

/* ��һ�е��ļ����������Ӧ��Buf
Msg�ı�A   : 000035 18/06/20 11:29:46.230 ������Ƿ��,Ƶ��0.02794Hz 0041 7F1B7044
Msg�ı�B	 : 000035 18/06/20 11:29:46.230 ������Ƿ��,Ƶ��0.02794Hz 0041 0.02794 7F1B7044
Acq�ı�	 : 000065 18/06/20 11:29 6000 1.00 4.00 1.80 8.00 0.100 7F1B7044 */
BOOL ReadTextFromLineForMsgOrAcq_MustSwiDisable(DATA_USER DataUser, DATA_ACS_OPR DataAcsOpr, uint8** ppBuf, uint8* pBufEnd, uint8* pLineStart, uint8* pLineEnd, BOOL bCrcOK)
{
	uint8* pBuf = *ppBuf;
	int16 i;
	pLineEnd -= 2;	/* ȥ�����Ļس����� */

	/* 2024.3.24ע�ͣ���δ���û�ã��Ѿ����Թ������Է���һ�������������� */
	/* Msg�ı���ʽ�����������������msg_val�ֶΣ�Ϊ�˼������ϰ汾����Ҫʶ�� */
	/* volatile BOOL bHaveMsgVal = (SkipCharInString(pLineStart, pLineEnd, ' ', 6) != pLineEnd); */
	
	/* �����UI��ȡ������Ҫ����ǰ���ItemNo, �����MsgID(����еĻ�),CRC��س�����,�����0�Ա�ʾ�ַ��Ľ�β */
	if(0) {
#if SUPPORT_SCREEN
	} else if((DataUser == DATA_USER_MCGS) || (DataUser == DATA_USER_VIEWTECH)) {
		/* ������ַ�(Ӣ��)���� */
		uint16 uItemLineByteLen = MCGS_LEN_ITEM_CHAR;
		if(DataUser == DATA_USER_VIEWTECH) {
			uItemLineByteLen = VT_LEN_ITEM_BYTE;
		}
		/* Buf���೤���ж� */
		if(pBufEnd - pBuf < uItemLineByteLen) {
			return FALSE;
		}
		pLineStart += 7;	/* ����MsgNo,AcqNo���� */
		int32 i32LineReadByteNum;
		if(DataAcsOpr == DATA_ACS_READ_MSG) {
			/* Ϊ�˼������ϰ汾Msg��ʽ��������Acq�İ취(�й̶���β������)����Ҫ������3��' ', ��MsgIDȻ���˻�һ��' ' */
			i32LineReadByteNum = SkipCharInString(pLineStart, pLineEnd, ' ', 3) - pLineStart - 1;
		} else {	/* Acqֻ��Ҫ�ӵ�����CRC���� */
			i32LineReadByteNum = pLineEnd - pLineStart - 9;		/* 9:8λCRC+һ���ո� */
		}
		if(i32LineReadByteNum >= uItemLineByteLen) { 			/* ��� uItemLineByteLen ���� */
			i32LineReadByteNum = uItemLineByteLen;
			/* ����UI��Լ򵥣�ֱ�ӿ������� */
			for(i = i32LineReadByteNum-1; i >= 0; i--) {		/* ��������0 */
				pBuf[i] = pLineStart[i];
			}
		} else {
			/* ����UI��Լ򵥣�ֱ�ӿ������� */
			for(i = i32LineReadByteNum-1; i >= 0; i--) {
				pBuf[i] = pLineStart[i];
			}
			pBuf[i32LineReadByteNum] = 0;
		}
		if(DataUser == DATA_USER_VIEWTECH) {
			if(DataAcsOpr == DATA_ACS_READ_MSG) {
				pLineStart = SkipCharInString(pLineStart, pLineEnd, ' ', 3);	/* ָ��MsgID */
			} else if(DataAcsOpr == DATA_ACS_READ_ACQ) {
				pLineStart += 15;				/* ָ��AcqID */
			}
			*((uint16*)(pBuf + VT_LEN_ITEM_CHAR)) = ReadH32(&pLineStart, pLineEnd);
		}
		pBuf += uItemLineByteLen;							/* ���������ݣ�ÿ��Itemռ�õĳ����ǹ̶��� */
#endif
	/* �������̫��ͨѶ��ȡ������Ҫ��һ�������json��ʽ */
	} else if(DataUser == DATA_USER_NET) {
		if(DataAcsOpr == DATA_ACS_READ_MSG) {
			/* �����⣺Msg���ı�ת��json��Ҫ���������ַ���:50+17(17��δ�����Ϣ�� ,"unfinish":"true")��������10���Ա���ȫ */
			if(pBufEnd - pBuf < pLineEnd - pLineStart + 77) {
				return FALSE;
			}
	/*	Msg�ı�A	 : 000035 18/06/20 11:29:46.230 ������Ƿ��,Ƶ��0.02794Hz 0041 7F1B7044
		Msg�ı�B	 : 000035 18/06/20 11:29:46.230 ������Ƿ��,Ƶ��0.02794Hz 0041 0.02794 7F1B7044
	ת��json��ʽ
		{"no":"66","time":"20180620 11:29:48.750","text":"������Ƿ��,Ƶ��0.02794Hz","msg_id":"0041","msg_val":"0.02794","cmplt":"false"},		*/
			/* no���� */
			*pBuf++ = '{';
            for( ; (pLineStart < pLineEnd) && ((*pLineStart <= '0') || (*pLineStart > '9')); pLineStart++) {/* ����ǰ��0�Լ������ֵĲ��� */
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
			pLineStart++;					/* �����ո񲿷� */
			/* time���� */
			PrintStringNoOvChk(&pBuf, "\"time\":\"20");
			*pBuf++ = *pLineStart++;	/* �� */
			*pBuf++ = *pLineStart++;
			pLineStart++;					/* ����\���� */
			*pBuf++ = *pLineStart++;	/* �� */
			*pBuf++ = *pLineStart++;
			pLineStart++;					/* ����\���� */
			for(i = 15; i > 0; i--) {		/* �� ʱ:��:�� */
				*pBuf++ = *pLineStart++;
			}
			pLineStart++;					/* �����ո񲿷� */
			uint8* pMsgVal = pLineStart;	/* ����ָ��Msg��ͷ�Ĳ��֣����ں������ܵĻ�ȡMsgVal */
			/* msg���� */
			PrintStringNoOvChk(&pBuf, "\",\"text\":\"");
			while((pLineStart < pLineEnd) && (*pLineStart != ' ')) {
				*pBuf++ = *pLineStart++;
			}
			uint8* pMsgEnd = pLineStart;	/* ����ָ��Msg��β�Ĳ��֣����ں������ܵĻ�ȡMsgVal */
			pLineStart++;					/* �����ո񲿷� */
			/* msg_id���� */
			PrintStringNoOvChk(&pBuf, "\",\"msg_id\":\"");
			while((pLineStart < pLineEnd) 		/* ʮ�����Ʊ�� */
					&& ((('0' <= *pLineStart) && (*pLineStart <= '9')) || (('A' <= *pLineStart) && (*pLineStart <= 'F'))))
			{
				*pBuf++ = *pLineStart++;
			}
			BOOL bUnFinish = (*pLineStart == '*');	/* MsgID��*��˵������������Ϣ */
			if(bUnFinish) {	/* ����* */
				pLineStart++;
			}
			/* msg_val���� */
			PrintStringNoOvChk(&pBuf, "\",\"msg_val\":\"");
			if(pLineEnd - pLineStart > 9) {	/* �ж�ΪMsg�ı�B��msg_id�����msg_val�ֶ�*/
				pLineStart++;				/* ����msg_valǰ����Ǹ� ' ' */
				while((pLineStart < pLineEnd) && (pBuf < pBufEnd) && (*pLineStart != ' ')) {/* ����MsgVal */
					*pBuf++ = *pLineStart++;
				}
			} else {	/* Msg�ı�A,��Ҫ����Ϣ�ı�������MsgVal */
				while((pMsgVal < pMsgEnd)								/* ����MsgVal */
					&& (!((('0' <= *pMsgVal) && (*pMsgVal <= '9'))		/* ֵ */
							|| ((*pMsgVal == '-') && (('0' <= pMsgVal[1]) && (pMsgVal[1] <= '9'))))))	/* ���� */
				{
					pMsgVal++;
				}
				BOOL bFindNeg = FALSE;
				BOOL bFindExp = FALSE;
				BOOL bFindPoint = FALSE;
				while((pMsgVal < pMsgEnd) && (pBuf < pBufEnd)) {	/* ����MsgVal */
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
						bFindPoint = TRUE;	/* eָ�����治Ӧ������С���� */
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
			if(pBuf + 3 > pBufEnd) {	/* ������ */
				return FALSE;
			} else {
				*pBuf++ = '"';
				*pBuf++ = '}';
				*pBuf++ = ',';
			}
		} else if(DataAcsOpr == DATA_ACS_READ_ACQ) {
			/* �����⣺Acq���ı�ת��json��Ҫ��������31�ַ���������4���Ա���ȫ */
			if(pBufEnd - pBuf < pLineEnd - pLineStart + 35) {
				return FALSE;
			}
			/* �� 000065 18/06/20 11:29 6000 1.00 4.00 1.80 8.00 0.100 7F1B7044 ת����json��ʽ
				{"no":"65", "time":"20180703 00:07:00", "acq_id":"6000", "data":[1.00,4.00,1.80,8.00,0.100]},	*/
			/* no���� */
			PrintStringNoOvChk(&pBuf, "{\"no\":\"");
			for( ; (pLineStart < pLineEnd) && (*pLineStart == '0'); pLineStart++) {
			}
			while((pLineStart < pLineEnd) && ('0' <= *pLineStart) && (*pLineStart <= '9')) {
				*pBuf++ = *pLineStart++;
			}
			pLineStart++;					/* �����ո񲿷� */
			/* time���� */
			PrintStringNoOvChk(&pBuf, "\",\"time\":\"20");
			*pBuf++ = *pLineStart++;		/* �� */
			*pBuf++ = *pLineStart++;
			pLineStart++;						/* ����\���� */
			*pBuf++ = *pLineStart++;		/* �� */
			*pBuf++ = *pLineStart++;
			pLineStart++;						/* ����\���� */
			for(i = 8; i > 0; i--) {			/* �� ʱ:�� */
				*pBuf++ = *pLineStart++;
			}
			pLineStart++;						/* �����ո񲿷� */
			/* time�벿��+acq_id */
			PrintStringNoOvChk(&pBuf, ":00\",\"acq_id\":\"");
			while((pLineStart < pLineEnd) && (*pLineStart != ' ')) {
				*pBuf++ = *pLineStart++;
			}
			/* data���� */
			PrintStringNoOvChk(&pBuf, "\",\"data\":[");
			for(i = 5; i > 0; i--) {
				pLineStart++;					/* �����ո񲿷� */
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

/* Mqtt����Msg, Acq */
void RegisterPageReq(DATA_PAGE DataPage, uint8* pU8Msg, uint8* pU8MsgEnd)
{
    /* ���ݿ���Ϣָ���ʼ�� */
    if((DataPage == DATA_PAGE_MSG) && CompareMsg(pU8Msg+2, "msgpt")) {  /* ָ��Ӧ����{"msgpt":"**"} */
        g_MqttItemProc.u32MsgNo_HavePubedForDB = ReadU32(&pU8Msg, pU8MsgEnd);
        Semaphore_post(g_DataAccessBlock.Sem_Op);		//        Semaphore_post(SEM_NvMemAcs);
        return;
    }

    /* ҳ����ʣ��ӿڴ��� */
    pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 3);
    uint32 u32ItemNo = 0xFFFFFFFFUL;    /* ���ڽӿں�����˵, 0xFFFFFFFFUL�����ʼ�� */
    int32 i32ItemNum = 0;
    if(!CompareMsg(pU8Msg, "ini")) {
        u32ItemNo = ReadU32(&pU8Msg, pU8MsgEnd);
    }
    GetI32(&pU8Msg, pU8MsgEnd, &i32ItemNum, 0);

    /* ҳ����ʣ�����ע�� */
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

/* ����Msg,Acq */
void PrintMsgAcqPageRes(uint8** ppTxBuf);
BOOL PubMsgAndAcq(MQTT_COMM* pMqttComm)
{
    /* ���� */
    Swi_disable();
	int8 i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage = g_MqttItemProc.i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage;
	uint16 uContLen = g_MqttItemProc.uContLen;
    Swi_enable();
    
	if(uContLen > MQTT_ITEM_PROC_BUF_BLEN) { /* ����Buf����������ݳ��� */
	    g_MqttItemProc.uContLen = 0;
        g_MqttItemProc.i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage = 0;
	} else if(i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage > 0) {
		/* ���� */
		uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
		uint8 u8QoS = 0;			/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
		/* ��ӡTopic */
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
			pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
		}
		memcpy(pTxBuf, g_MqttItemProc.u8Buf, uContLen);
		pTxBuf += uContLen;
		/* ����ҳ�淢��:���� */
		if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) {	/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
			return FALSE;
		}
		
		/* ҳ�棺��� */
		if(i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage == 1) {
			g_MqttItemProc.u32MsgNo_HavePubedForDB = g_MqttItemProc.u32ItemNo_OutEnd;
		} else { /* ֻ��ҳ��������н�� */
    		pTxBuf = pMqttComm->u8TRBuf + 5; /* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
    		u8QoS = 0;			/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
    		/* Msg�������:��ӡTopic */
    		PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
    		if(i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage == 2) {
        		PrintStringNoOvChk(&pTxBuf, "/RES/MSG");
    		} else {
        		PrintStringNoOvChk(&pTxBuf, "/RES/ACQ");
        	}
    		uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
    		if(u8QoS) {
    			pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
    		}
            PrintMsgAcqPageRes(&pTxBuf);
    		/* Msgҳ��������:���� */
    		if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) {	/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
    			return FALSE;
    		}
		}

        g_MqttItemProc.i8BufCont_0Idle_nBusy_1MsgDB_2MsgPage_3AcqPage = 0;
#if MAX_MSG_TYPE
        if(g_MqttItemProc.u32MsgNo_HavePubedForDB < g_MsgCtr.u32MsgNo_Cmplt) {
            Semaphore_post(g_DataAccessBlock.Sem_Op);		//            Semaphore_post(SEM_NvMemAcs);
        }
    } else if(g_MqttItemProc.u32MsgNo_HavePubedForDB == 0xFFFFFFFFUL) {
        /* ���� */
        uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
        uint8 u8QoS = 0;            /* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
        /* ��ӡTopic */
        PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
        PrintStringNoOvChk(&pTxBuf, "/REQ/MSGPT");
        uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
        if(u8QoS) {
            pTxBuf += 2;    /* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
        }
        PrintStringNoOvChk(&pTxBuf, "{\"req\":\"msgpt\"}");
        /* ����ҳ�淢��:���� */
        if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) {  /* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
            return FALSE;
        }
#endif
	}

	return TRUE;
}

void PrintMsgAcqPageRes(uint8** ppTxBuf)
{
    uint8* pTxBuf = *ppTxBuf;
    *pTxBuf++ = '{';    /* Json��ʼ */
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
    *pTxBuf++ = '}';    /* Json��β */

    *ppTxBuf = pTxBuf;
}
/******************************** FILE END ********************************/
