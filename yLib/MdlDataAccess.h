	/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved.	ģ��V1.3
 *				: 
 * File			: stencil.h
 * Author		: Wang Renfei
 * Description	: ��������ģ�飺���á���Ϣ���ɼ������ݴ�ȡ
 * Date			: 2009-5-31
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 * V1.3	| 2015-9-7	  |	king | 1. �ļ�ͷ������ģ��汾�Ų�����ģ�����Լ��
 * 		|			  |		 | 2. ɾ��Comments��Ŀ
 * 		| 2015-9-27	  |		 | 3. ���RunMdl������DrvMdl_100Hz��ʱ������
 * V1.2	| 2009-5-31	  |	king | 1. �޸�Ϊ��Ӧģ�黯,������Ҫ�ⲿ�����ĺ���
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
 * Msg���֣�ʹ��MSGģ�飬��Ҫ���������ã�
1. #define MAX_MSG_TYPE			0x0040				// �ܵ���Ϣ���������������0x20����������������
   #define MSG_ABORNMAL_START	0x0020				// �쳣��Ϣ��ʼλ�ã���һ���������û��

2. ��Ϣ����
 	typedef enum {					// MSG_***Ҫ�� 0x0002��ʼ
		MSG_NULL					= 0x0000,		// �գ���������û��
		MSG_HuanYinShiYong 			= 0x0001,		// ��ӭʹ�ã���������û��
		//��������ʽ��Ϣ����
		MSG_***						= 0x0002,
		 ...
	} MSG_ID;

3. ��Ϣ���벿�֣�������	MAX_MSG_TYPE ��
	const MSG_PROP cnst_MsgProp[MAX_MSG_TYPE]		// Ϊ�˱���ʶ���и���ֵ����Ϣ���Ĳ��ֲ����Դ�����
	= {	{-1,	"", ""},
		{-1,	"��ӭʹ��***", "Reserved"},
		...
	};
 *==========================================================================*/


/*===========================================================================
 * ACQģ�飺�Բɼ����ݽ��� д�롢��ȡ���ۺϳ�AcqSumm�����ʹ����Ҫ���������ã�
 1. #define ACQ_ITEM_DAT_NUM 5					//����ACQ�ɼ�����¼��������
 2. 
 *==========================================================================*/
/* AcqID˵�� */
typedef struct {
	BITBOOL	ID		: 8;	/* ��ID���� */
	BITBOOL SubID	: 8;	/* ��ID���֣����������ʱ��ò�����ֵ��Ϊ��ȷƥ�䣬����Ϊ��ƥ����ֶ� */
}STRUCT_ACQ_ID;

#if ACQ_ITEM_DAT_NUM
typedef struct {								/* ��ȡ�������� */
	uint16 uAcqId;								/* ���ڴ�������һ����֤ */
	uint16 uDaysFromOrigin;						/* ��2000/01/01��ʼ���������Ա��ж����ݵ����ʳ̶� */
	float32 fD[ACQ_ITEM_DAT_NUM];
}ACQ_READ_ITEM;

typedef struct {								/* ��AcqData��AcqSumm����������ϵ���� */
	uint16 uAcqId;
	uint16 uRsvd;
	void (*AcqSummProc)(ACQ_READ_ITEM* pAcqDataStart, uint16 uItemNum);	/* ������ */
	ACQ_SUMM* pAcqSumm[ACQ_ITEM_DAT_NUM];		/* ָ����ص�AcqSumm���Ա���AcqSumm�������ĵ�ʱ�򣬲�����Ӧ�Ĵ��� */
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
 * ���ñ���
 *--------------------------------------------------------------------------*/
/* <NULL> */

/*---------------------------------------------------------------------------
 * ���ñ�������
 *--------------------------------------------------------------------------*/
/* <NULL> */

/*---------------------------------------------------------------------------
 * ���õ�½ҳ��
 *--------------------------------------------------------------------------*/
/* <NULL> */

/*---------------------------------------------------------------------------
 * ���ýṹ��
 *--------------------------------------------------------------------------*/
/* <NULL> */

/***************************************************************************
 					functions must be driven by extern
***************************************************************************/
EXT void DataAccessTask(void const * argument);

/***************************************************************************
 					functions could be used as extern
***************************************************************************/
/* ���ݴ洢������ӿ� */
EXT void SaveStatMsgAcq(void);			/* ����Stat��Msg��Acq���ݽ��д洢 */
EXT void UrgentSaveNvData(void);		/* ���ϵ�Դ���: �͵�ѹ�����������ݴ洢����Ҫ���ߴ洢�������ȼ� */
EXT BOOL ChkAndWaitInitConf(CONF_SAVE_GROUP ConfStatSaveMedia);				/* ��ʼ��������ͳ�� */
EXT BOOL RegisterSysFunc(SYS_FUNCTION SysFunc, BOOL bUnLock);
EXT void RegisterPageReq(DATA_PAGE DataPage, uint8* pU8Msg, uint8* pU8MsgEnd);  /* MQTTͨѶ */
EXT DATA_ACS_OPR ReadConfFromEEProm(CONF_SAVE_GROUP AcsGrp); 
EXT BOOL CheckConfFromEEProm(CONF_SAVE_GROUP AcsGrp, uint32 u32EEPromByteAddr); 	/* �ᵽ.h���ɣ�ΪDrvXX�ܿ��� */
EXT void SaveConfToEEProm(CONF_SAVE_GROUP AcsGrp, uint32 u32EEPromByteAddr);

/* ��Ϣ��ӹ��� */
EXT void AddMsgA_WithVal(uint16 uMsgId, float32 fMsgVal);		/* ���ú���֧�����룬��Ϣ�����ظ���� */
EXT void AddMsgB_WithVal(uint16 uMsgId, float32 fMsgVal);		/* ��Ϣ�������һ�� */
EXT void UpdateMsgC_WithVal(uint16 uMsgId, BOOL bMax1OrMin0, float32 fMsgVal);
EXT void ConfirmMsgC(uint16 uMsgId);
EXT void FinishMsgC(uint16 uMsgId, BOOL bNeedBlock);
EXT void ClearAllMsgShieldFlag(void);
EXT void ClearSingleMsgShieldFlag(uint16 uMsgId);

/* �ɼ���ȡ���� */
EXT void AddAcqItem(uint16 uAcqID, float32 fD0, float32 fD1, float32 fD2, float32 fD3, float32 fD4);
EXT void InformAcqSummConfChange(void* pAcqSumm);

/*	MSG, ACQ �ı����ʽӿ�
	DataUser	: �����û�, DATA_USER_NET���ݳ��Ȳ��̶���һ��Item�������̷�����һ��Item;
							DATA_USER_MCGS��DATA_USER_VIEWTECH�����ݳ��ȹ̶�ΪN��������䣬�ӵ�ItemNo��CRC����;
							DATA_USER_VIEWTECH��������ֽ�ΪMsgID��AcqID
								�������Կ��ַ��������������������������(������pBufEnd)���������޸ķ��ص�Itemo, ItemNum
	DataPage					: Ŀǰ��֧��DATA_PAGE_MSG, DATA_PAGE_ACQ
	bNetAbortUnCmpltMsg			: ���緢�����ݣ���������������Ϣ��ֹ����������������Ϣ--�������ݿ����
	pU32ItemNo_InStart_OutEnd 	: ��ʼItemNo(����), ����ʵ�ʶ�ȡ�����һ��ItemNo--����ָ����ItemNo����������, 0xFFFFFFFFUL�����ȡTopic
	pI32ItemNum_InNeed_OutSuc 	: ��ȡ��Item����(��:ItemNo���ӣ��������;��:ItemNo���٣�����ǰ��)������ʵ�ʶ�ȡ��Item����(�����ţ���ָ��ǰ�����ɹ���ȡ)
	ppBuf						: Buf��㣬����д�룬���ط�����ɺ��ָ��
	pBufEnd						: Buf�յ㣬����д��	*/
EXT DATA_ACS_OPR GetTextForMsgOrAcq(DATA_USER DataUser, DATA_PAGE DataPage, uint32* pU32ItemNo_InStart_OutEnd, 
					                int32* pI32ItemNum_InNeed_OutSuc, uint8** ppBuf, uint8* pBufEnd);
EXT BOOL PubMsgAndAcq(MQTT_COMM* pMqttComm);	/* ����Msg��Acq */
EXT uint16 ReadLastAddMsgForVT(uint8* pBuf, uint8* pBufEnd, BOOL bTimeShort);

/* ��ram�е�Msgͨ��modbus���ͣ�һ����Ϣռ��6��modbus16bit�Ĵ��� */
typedef struct {    /* ����ṹ�岢û��ʵ��Ӧ�ã�����Ϊmodbus�ӿڴ����ʾ�� */
	uint8 u8Year;					/* �꣬1-99�������λΪ0���������ʱ����Ч */
	uint8 u8Month;					/* �£�1-12 */
	uint8 u8Day;					/* �գ�1-31 */
	uint8 u8Hour;					/* ʱ��0-23 */
	uint8 u8Minute;					/* �֣�0-59 */
	uint8 u8Second;					/* �룬0-59 */
	uint16 uMsgId;                  /* MsgId����Ҫ����MSG_ID�Ա���� */
	float32 fMsgVal;                /* ��Ϣֵ��ע���Ǹ����� */
}MSG_FOR_MODBUS;
EXT uint16 TxMsgForModbus(uint8** ppBuf, uint16 uRegNum);

/************   exclude redefinition and c++ environment   ****************/
#ifdef __cplusplus
}
#endif 					/* extern "C" */

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
/******************************** FILE END ********************************/
