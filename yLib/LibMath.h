/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved
 *
 * File			: LibMath.h
 * Author		: Wang Renfei
 * Description	: 
 *				: 
 * Comments		:
 * Date			: 2015-8-31
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 * 		|			  |		 | 
 **************************************************************************/

/***************************************************************************
 				exclude redefinition and c++ environment
***************************************************************************/
#ifndef _LIB_MATH_H_
#define _LIB_MATH_H_

/***************************************************************************
 							include files
***************************************************************************/
#include "math.h"
#include "GlobalVar.h"

#define SUPPORT_SAMPLE ((DEVICE_TYPE != V5_YBT2) && (DEVICE_TYPE != V5_YBT3) && (DEVICE_TYPE != V5_YBT4))
/***************************************************************************
				hardware parameter and global definitions
***************************************************************************/


/***************************************************************************
 		use EXT to distinguish stencil.c or others source files
***************************************************************************/
#ifdef _LIB_MATH_C_
#define EXT
#else
#define EXT extern
#endif

/***************************************************************************
 					variables could be used as extern
***************************************************************************/
#define SQRT_3			1.7320508075688772935274463415059
#define KW_W			1000.0f
#ifdef _LIB_MATH_C_
const float32 cnst_fBcdQFactor[] = {1e00, 1e01, 1e02, 1e03, 1e04, 1e05, 1e06, 1e07, 1e08, 1e09, 
										1e10, 1e11, 1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19};
const uint64 cnst_u64BcdQFactor[] = {1e00, 1e01, 1e02, 1e03, 1e04, 1e05, 1e06, 1e07, 1e08, 1e09, 
										1e10, 1e11, 1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19};
const uint32 cnst_u32BcdQFactor[] = {1e00, 1e01, 1e02, 1e03, 1e04, 1e05, 1e06, 1e07, 1e08, 1e09};
const uint32 cnst_u32IPv4Factor[] = {1, 0x100, 0x10000, 0x1000000};
#else
extern const float32 cnst_fBcdQFactor[];
extern const uint64 cnst_u64BcdQFactor[];
extern const uint32 cnst_u32BcdQFactor[];
extern const uint32 cnst_u32IPv4Factor[];
#endif

/***************************************************************************
 					functions could be used as extern
***************************************************************************/
/* �ڴ������ʼ�� */
EXT	void InitDataWithZero(uint8* pU8Dat, uint32 u32ByteLen);

/* �������������ʼ��� */
EXT float32 CalCosFromPQ(float32 fPowerP, float32 fPowerQ);
EXT float32 CalPowerSFromUI(float32 fVol, float32 fCur);
EXT float32 CalPowerQFromPnCos(float32 fPowerP, float32 fCos);
EXT float32 CalPowerPFromUICos(float32 fVol, float32 fCur, float32 fCos);
EXT float32 CalCosDiff(float32 fLagCos, float32 fLeadCos);

/* CRCУ�� */
EXT uint16 CalCRC16ForModbus(uint8* pU8Dat, int16 iByteLen);
#define CalCRC16ForViewTech(pU8Dat, iByteLen)	CalCRC16ForModbus(pU8Dat, iByteLen)
EXT uint32 CalCRC32ForFileByIni(uint8* pU8Dat, int32 i32ByteLen, uint32 u32CrcIni);

/* ���ݼ�¼�봦�� */
typedef struct {					/* ���ݾ��Ȳ�����¼ */
	float32 fAvr;
	float32 fRmsE;
	uint16 uDatOKDecCnt;
	uint16 uRecPt;					/* ����ָ��ǰ���ݴ洢λ�� */
	uint16 uDatBuf[];
}uDAT_UNI_REC;

typedef struct {					/* ���ݾ��Ȳ�����¼ */
	float32 fAvr;
	float32 fRmsE;
	uint16 uDatOKDecCnt;
	uint16 uRecPt;					/* ����ָ��ǰ���ݴ洢λ�� */
	uint16 uFilterPt;				/* �˳�ë�̺��ָ�� */
	uint16 uRsvd;
	uint32 u32DatBuf[];
}u32DAT_UNI_REC;

/* ģ���������㷨1 */
typedef struct {		/* ģ���źŲ������� */
	float32 fZeroCorr;							/* ���� */
	float32 fGainCorr;							/* ���� */
}ANALOG_SIG_CORR;

/* ģ���������㷨2 */
typedef struct {
    /* ������������Ҫ���� ITEM_DAT_ACQ_F32 */
    float32 fNoCorrVal_Zero;                /* 0�����޽�������ֵ */
    float32 fNoCorrVal_Now;                 /* ��ǰ�޽�������ֵ */
    /* ������������Ҫ���� ITEM_DAT_D2F32 */
    float32 fNoCorrVal_FullRange;           /* �������޽�������ֵ */
    float32 fRealVal_FullRange;             /* ������ʵ��ֵ */
}ANALOG_SIG_CORR2;

EXT void InitUDatUniRec(uDAT_UNI_REC* pDatRec, uint16 uBufLen);
EXT void UniRecUDatToBuf(uDAT_UNI_REC* pDatRec, uint16 uBufLen, uint16 uData);
EXT BOOL ProcUniRecUDatAsWave(uDAT_UNI_REC* pDatRec, uint16 uBufLen, uint16 uCalLen);

EXT void InitU32DatUniRec(u32DAT_UNI_REC* pDatRec, uint16 uBufLen);
EXT void UniRecU32DatToBuf(u32DAT_UNI_REC* pDatRec, uint16 uBufLen, uint32 uData);
EXT BOOL ProcUniRecU32DatAsWave(u32DAT_UNI_REC* pDatRec, uint16 uBufLen, uint16 uCalStartPt, uint16 uCalLen);
EXT BOOL FilterUniRecU32Dat(u32DAT_UNI_REC* pDatRec, uint16 uBufLen, uint16 uCalStartPt);

/* ����ֵ<>�ı��໥ת��, ����pTextEndָ���β��һ��(���ɴ洢)�ĵ�ַ */
EXT BOOL PrintF32(uint8** ppBuf, uint8* pBufEnd, float32 fData, int8 i8SgnDigits_pMax_nExpect);
EXT BOOL PrintFM32(uint8** ppBuf, uint8* pBufEnd, float32 fData, F32_MEANING F32Meaning);
EXT void PrintFM32NoOvChk(uint8* pBuf, float32 fData, F32_MEANING F32Meaning);
EXT BOOL PrintI16(uint8** ppBuf, uint8* pBufEnd, int16 iData, uint16 uBcdQ);
EXT BOOL PrintI32(uint8** ppBuf, uint8* pBufEnd, int32 i32Data, uint16 uBcdQ);
EXT BOOL PrintI64(uint8** ppBuf, uint8* pBufEnd, int64 i64Data, uint16 uBcdQ);
EXT BOOL PrintU32(uint8** ppBuf, uint8* pBufEnd, uint32 u32Data, uint16 uBcdQ);
EXT void PrintU32WithLen(uint8** ppBuf, uint32 u32Data, uint16 uLen);
EXT BOOL PrintH16(uint8** ppBuf, uint8* pBufEnd, uint16 uData);
EXT BOOL PrintH32(uint8** ppBuf, uint8* pBufEnd, uint32 u32Data);
EXT BOOL PrintH64(uint8** ppBuf, uint8* pBufEnd, uint64 u64Data);
EXT BOOL PrintIPv4(uint8** ppBuf, uint8* pBufEnd, uint32 u32IPv4);
EXT BOOL PrintT64(DATA_USER DataUser, uint8** ppBuf, uint8* pBufEnd, REAL_TIME_VAR* pRealTime);
EXT BOOL PrintT32(DATA_USER DataUser, uint8** ppBuf, uint8* pBufEnd, uint32 u32Seconds);
EXT BOOL PrintSoftVer(uint8** ppBuf, uint8* pBufEnd, uint16 uData);
EXT BOOL PrintPriceTime(uint8** ppBuf, uint8* pBufEnd, uint32 u32pvTime);
EXT void PrintDateTime(uint8** ppBuf, char DateDelmtr, char TimeDelmtr);
/* ���º������Ǵ��ַ�����߿�ʼ��
	Get**()����ҵ����ݷ���TRUE�����򷵻�FALSE
	Read**()������С���㡢eָ�����ַ���, ���һֱû���ҵ����ݷ�����0 */
EXT BOOL GetF32(uint8** ppText, uint8* pTextEnd, float32* pF32);
EXT BOOL GetU32(uint8** ppText, uint8* pTextEnd, uint32* pU32, uint8 u8BcdQ);	/* �����С���㡢eָ�����ַ��� */
EXT BOOL GetI32(uint8** ppText, uint8* pTextEnd, int32* pI32, uint8 u8BcdQ);	/* �����С���㡢eָ�����ַ��� */
EXT BOOL GetI64(uint8** ppText, uint8* pTextEnd, int64* pI64, uint8 u8BcdQ);	/* �����С���㡢eָ�����ַ��� */
EXT BOOL GetHexArray(uint8* pText, uint8* pTextEnd, uint8* pU8Hex, uint16 uDataLen);
EXT BOOL GetIPv4(uint8** ppText, uint8* pTextEnd, uint32* pIPv4);
EXT BOOL GetPriceTime(uint8** ppText, uint8* pTextEnd, uint32* pPVTime);
EXT BOOL GetT64(DATA_USER DataUser, uint8** ppText, uint8* pTextEnd, REAL_TIME_VAR* pRealTime);
EXT BOOL GetT32(uint8** ppText, uint8* pTextEnd, uint32* u32Seconds);
EXT uint32 ReadU32(uint8** ppText, uint8* pTextEnd);	/*  */
EXT uint32 ReadH32(uint8** ppText, uint8* pTextEnd);	/* ������С���㡢eָ�����ַ���, ���һֱû���ҵ����ݷ�����0 */
EXT uint64 ReadH64(uint8** ppText, uint8* pTextEnd);	/* ������С���㡢eָ�����ַ���, ���һֱû���ҵ����ݷ�����0 */
EXT int16 ReadI16(uint8** ppText, uint8* pTextEnd);		/* ������С���㡢eָ�����ַ���, ���һֱû���ҵ����ݷ�����0 */
EXT uint32 ReadH32FixLen(uint8* pText);	/* ��ΪpTextָ������ */
EXT uint32 ReadU32FixLen(uint8* pText);	/* ������С���㡢eָ�����ַ���, ���һֱû���ҵ����ݷ�����0 */

/* ����ֵ��JSON��ʽ��ӡ����,���γ� "var_name":"var_value", ���������������飬����Ԥ������ж� */
EXT void PrintStringToJson(uint8** ppJson, const char* pVarName, const char* pString);
EXT void PrintStringAndU32DataToJson(uint8** ppJson, const char* pVarName, const char* pString, uint32 u32Data);
EXT void PrintHexToJson(uint8** ppJson, const char* pVarName, uint8* pU8Hex, uint16 uByteLen);
EXT void PrintBase64ToJson(uint8** ppJson, const char* pVarName, uint8* pU8Dat, uint16 uByteLen);
EXT void PrintU32DatToJson(uint8** ppJson, const char* pVarName, uint32 u32Data, uint16 uBcdQ);
EXT void PrintU32ArrayToJson(uint8** ppJson, const char* pVarName, uint32* pU32Data, uint16 uDataNum);
EXT void PrintU32AsBinaryToJson(uint8** ppJson, const char* pVarName, const char* pBinaryDispChar, uint32 u32Data);
EXT void PrintF32DatToJson(uint8** ppJson, const char* pVarName, float32 fData, int8 i8SgnDigits_pMax_nExpect);
EXT void PrintF32WithStatusToJson(uint8** ppJson, const char* pVarName, float32 fData, int8 i8SgnDigits_pMax_nExpect, BOOL bStatus);
EXT void PrintF32ArrayToJson(uint8** ppJson, const char* pVarName, float32* pfData, int8 i8SgnDigits_pMax_nExpect, uint16 uDataNum);
EXT void PrintFM32DatToJson(uint8** ppJson, const char* pVarName, float32 fData, F32_MEANING F32Meaning);
EXT void PrintH64DatToJson(uint8** ppJson, const char* pVarName, uint64 u64Data, uint8 u8HexNum);
EXT void PrintH32DatToJson(uint8** ppJson, const char* pVarName, uint32 u32Data);
EXT void PrintH16DatToJson(uint8** ppJson, const char* pVarName, uint16 uData);
EXT void PrintI32DatToJson(uint8** ppJson, const char* pVarName, int32 i32Data, uint16 uBcdQ);
EXT void PrintI32ArrayToJson(uint8** ppJson, const char* pVarName, int32* pI32Data, uint16 uDataNum);
EXT void PrintI64DatToJson(uint8** ppJson, const char* pVarName, int64 i64Data, uint16 uBcdQ);
EXT void PrintIPv4ToJson(uint8** ppJson, const char* pVarName, int32 u32IPv4);
EXT void PrintT32ToJson(uint8** ppJson, const char* pVarName, uint32 u32Seconds);
EXT void PrintEnumContToJson(uint8** ppJson, const char* pVarName, uint32 u32Data, uint16 uBcdQ);
EXT void PrintSoftVerToJson(uint8** ppJson, const char* pVarName, uint32 u32Data);
EXT void PrintBoolToJson(uint8** ppJson, const char* pVarName, BOOL bData);
EXT void PrintTempToJson(uint8** ppJson, LANG_TYPE Language);

/* MQTT��Ϣ�������� */
EXT uint8* SkipCharInString(uint8* pU8Msg, uint8* pU8MsgEnd, uint8 u8Char, uint16 uCharNum);
EXT BOOL CompareTopic(uint8* pU8TopicName, const char* pU8TopicString);
EXT BOOL CompareMsg(uint8* pU8MsgName, const char* pU8NameString);
EXT BOOL CompareInstr(uint8** ppU8Buf, uint8* pU8BufEnd, const char* pU8Instr);

/* �ַ������� */
EXT BOOL PrintString(uint8** ppBuf, uint8* pBufEnd, const char* pString);
EXT void PrintStringNoOvChk(uint8** ppBuf, const char* pString);			/* �ú������������⣬��Ҫ�ڵ��ô�ȷ��������� */
EXT void PrintStringWithMaxLen(uint8** ppBuf, const char* pString, uint8 u8MaxLen);
EXT void PrintTopicString(uint8** ppBuf, uint32 uTopicLineLen, const char* pString);	/* ���Topic���˶��д��� */
/* ��ppPropText����iCharacterNum���ַ���ppText���ַ����������������Ļ�����ĸ������һ���ַ� */
EXT BOOL PrintStringWithCharNum(uint8** ppText, uint8* pTextEnd, char** ppPropText, int16 iCharacterNum);
/* �ַ�����飺���ַ���������ָ��������δ���ֽ�����0������FALSE */
EXT BOOL CheckString(uint8* pU8String, uint8 u8MaxLen);

/* �����ֽ�(���)˳���CPU�ֽ�˳��(С��)֮�以�࿽�� */
EXT uint16 CopyL32DatToTxBuf(uint8** ppDest, uint8* pSrc, uint16 uRegNum, int16 iMaxCopyRegNum);
EXT uint16 CopyL16DatToTxBuf(uint8** ppDest, uint8* pSrc, uint16 uRegNum, int16 iMaxCopyRegNum);
EXT uint16 CopyLU16asF32ToTxBuf(uint8** ppDest, uint16* pUSrc, uint16 uRegNum, int16 iMaxCopyRegNum);
EXT uint16 CopyRxBufToL32Dat(uint32* pU32Dest, uint8** ppSrc, uint16 uRegNum, int16 iMaxCopyRegNum);
EXT uint16 CopyRxBufToL16Dat(uint16* pUDest, uint8** ppSrc, uint16 uRegNum, int16 iMaxCopyRegNum);

/* ���� */
EXT uint16 Cal1InBits16(uint16 Bits16);
EXT uint32 F32ToU32(float32 f32Dat);
EXT int32 F32ToI32(float32 f32Dat);
EXT uint16 F32ToU16(float32 f32Dat);
EXT int16 F32ToI16(float32 f32Dat);
EXT float32 CalMaxFromFloatDat(float32* fDat, uint8 u8DatNum);
EXT float32 CalMinFromFloatDat(float32* fDat, uint8 u8DatNum);
#define CalMinU16(uNum1, uNum2)		((uNum1 > uNum2) ? uNum2 : uNum1)

/* AES�ӽ��� */
EXT void EncryptWithAES(AUTH_TYPE AuthType, uint32* pU32Data, uint32 u32ByteLen, uint32 u32IV);
EXT void DecryptWithAES(AUTH_TYPE AuthType, uint32* pU32Data, uint32 u32ByteLen, uint32 u32IV);

/* base64����� */
EXT uint8* EncodeByBase64(uint8* pU8Data, uint16 uByteLen, uint8* pU8Base64Buf);
EXT uint8* DecodeByBase64(uint8* pU8Base64Buf, uint16 uBase64BLen, uint8* pU8Data, uint16 uDataBLen);

/* MD5��ϢժҪ�㷨 */
typedef struct {					/* MD5 */
	uint32 u32Count[2];
	uint32 u32State[4];
	uint8 u8Buffer[64];
}MD5_CTX;
EXT void MD5Encode(uint8 *pOutput, uint32 *pInput, uint32 u32Len);
EXT void MD5Decode(uint32 *pOutput, uint8 *pInput, uint32 u32Len);
EXT void MD5Transform (uint32 state[4], uint8 block[64]);
EXT void MD5Init(MD5_CTX *pContext);
EXT void MD5Update(MD5_CTX *pContext, uint8 *pInput, uint32 u32InputLen);
EXT void MD5Final(uint8 digest[16], MD5_CTX *pContext);

/* Զ�̴�������غ��� */
EXT uint8* PackRmtSenForUartComm(uint16 uCoder, uint16 uDevAdd, float32* pfData, uint8* pTxBuf);
EXT BOOL GetRmtSenFromUartComm(RMT_SEN_DAT *pRmtSenDat, uint16 uDevAdd, uint8* pRxBuf, uint16 uRxNum);
EXT void PrintRmtSenToJson(uint8** ppTxBuf, uint16 uCoder, float32* pfData);
EXT void GetRmtSenFromMqttComm(RMT_SEN_DAT *pRmtSenDat, uint8* pRxBuf, uint8* pRxBufEnd);

/* ͳ�Ʋ��� */
EXT void CountStatItem(STAT_COUNT_ITEM* pStatItem, uint16 uMsgID);	/* ����STAT_COUNT_ITEM��ʽ����ͳ�ƣ����ڳ�������ʱ�򱨾� */
EXT BOOL CumulateFlow(STAT_CUM_ITEM* pCumItemStart, float32* pFlowStart, uint8 u8CumItemNum, BOOL bFragFlag);
EXT uint16 CopyDCumDatToTxBuf(uint8** ppDest, STAT_CUM_ITEM* pCumItemStart, uint16 uRegNum, uint16 uCumItemNum, uint32 u32Div);
EXT uint16 CopyLCumDatToTxBuf(uint8** ppDest, STAT_CUM_ITEM* pCumItem, uint16 uRegNum, int16 iMaxCopyRegNum);

/* �����������������--��ҪBoardSupport.h,  DrvSample.h����ģ�����������߿���Ĳ�Ʒ����Ҫ */
EXT void ProcDInFilter(void);
EXT void CalACSig(uint8 u8ACSigNo, uint16 uCalDatLen, float32 fExtTransRatio, float32* pfVal);
EXT void ProcCap(uint32 u32CapTime, uint8 u8ACSigNo);
EXT float32 CalACSigFreq(uint8 u8ACSigNo);
EXT void CalACSigAsDCSig(uint8 u8ACSigNo, uint8 u8DCSigNo, uint16 uCalDatLen, float32 fExtTransRatio, float32* pfVal);
EXT void CalDCSig(uint8 u8DCSigNo, uint16 uCalDatLen, float32 fExtTransRatio, float32* pfVal, ANALOG_SIG_CORR* pAnaSigCorr);
EXT float32 CalPValFromDCSigVal(float32 fSigVal, float32 fExtTransRatio, uint8 u8DCSigNo, ANALOG_SIG_CORR* pAnaSigCorr);
EXT void CalDCSig2(uint8 u8DCSigNo, uint16 uCalDatLen, float32 fExtTransRatio, float32* pfVal, ANALOG_SIG_CORR2* pDCSigCorr);
EXT float32 CalPValFromDCSigVal2(float32 fSigVal, float32 fExtTransRatio, uint8 u8DCSigNo, ANALOG_SIG_CORR2* pDCSigCorr);

/*===========================================================================
 * ������
 *==========================================================================*/
#ifdef TOTAL_ABN_NUM
typedef struct {
	uint32 u32Abnormal[ABNORMAL_BUF_b32LEN];    /* �쳣��ǰ˲ʱֵ�������� g_AnaRes.u32Abnormal���Ǹ��Ǵ����ֵ����������ͨѶ�ӿڷ��� */
	uint16 uAbnDealFlag;						/* �쳣�����ʶ������Ӧ��ִ�к������н���ִ�У���Ҫÿ�������� */
	uint16 uRsvd;
}ABN_OUT;
EXT ABN_OUT g_AbnOut;
#endif
EXT BOOL OutputAndHoldAbnormal(uint8 u8CallPeriod_ms);   /* ����쳣��g_AnaRes.Abnormal��������һ��ʱ�䣬�Ա�ͨѶ�ӿڷ��� */
EXT BOOL DelayVerifyOvAbn(uint8 u8AbnNo, BOOL bFuncSw, uint16 uAbnDealFlag, uint32 u32VerifyTicks, float32 fVal, float32 fThr, uint16 uMsgId);
EXT BOOL DelayVerifyNoRcvrAbn(uint8 u8AbnNo, BOOL bFuncSw, uint16 uAbnDealFlag, uint32 u32VerifyTicks, float32 fVal, float32 fThr, uint16 uMsgId);
EXT void RatioDiffOvAbn(uint8 u8AbnNo, BOOL bFuncSw, uint16 uAbnDealFlag, float32 fIcd, float32 fIcdqd, float32 fIzd, uint16 uWarnMsgId, uint16 uActMsgId);
EXT void VerifyCurOvWithVolLow(uint8 u8AbnNo, BOOL bFuncSw, uint16 uAbnDealFlag, uint32 u32VerifyTicks, float32 fCur, float32 fCurThr, float32 fVol, float32 fVolThr, uint16 uMsgId);
EXT BOOL InverseTimeVerify(uint8 u8AbnNo, uint16 uAbnDealFlag, uint8 u8Alg, float32 fTp, float32 fI, float32 fIp, uint16 uMsgId);

/*===========================================================================
 * �̵��������Լ�Ч������
 *==========================================================================*/
/* T.FKL���ø�ʽ��T������ʱ��, F������, K�źű���(EΪ�ⲿ���֣������һ������; IΪ�ڲ����֣��������ʱ��), L���� */
#define GET_T_FKL_ActTime(TfklConf)				(TfklConf/1000UL)
#define GET_T_FKL_HaveFdBak(TfklConf)			((TfklConf%1000UL)/100 != 0)
#define GET_T_FKL_KeepFlag(TfklConf)			((TfklConf%100UL)/10 != 0)
#define GET_T_FKL_Level(TfklConf)				(TfklConf%10)
#define CHK_T_FKL_NO_ACTnFB(TfklConf)			(TfklConf/100UL == 0)

typedef enum {
	ACT_RELAY_SUC			= 0,	/* �̵����ɹ����� */
	ACT_RELAY_NUM_INVALID	= 1,	/* �̵����ŷǷ�(����MAX_RELAY_NUM) */
	ACT_RELAY_TIME_ZERO		= 2,	/* ����ʱ����0 */
	ACT_RELAY_THR			= 3,	/* ��λ���� */
	ACT_RELAY_SAVE_URGENT	= 4,	/* ���ϵ͵�ѹ��������洢 */
	ACT_RELAY_TIME_OUT      = 5     /* �ü̵����Ѿ���ʱ */
}ACT_RELAY_RES;

typedef enum {						/* �̵���-���뷴�� */
	RELAY_DIN_NULL		= 0,
	RELAY_DIN_ACTIVE	= 1,		/* Ч���������̵�����������ʱһ��ʱ��󣬿��붯�����索�� */
	RELAY_DIN_DEACTIVE	= 2,		/* Ч���������̵�����������ʱһ��ʱ��󣬿�����ʧ�����բ */
	RELAY_DIN_THR		= -1		/* ��λ�������̵���������������һ��ʱ��󣬿��붯�����翪�ȶ��� */
}RELAY_DIN_EFF;

typedef struct {
	int8 i8RelayDIn_0N_1A_2D_N1T;	/* �̵�������Ч��: 0��, 1:Ч���������붯��, 2:Ч������������ʧ, -1:��λ�������붯�� */
	uint8 u8DInNo;					/* ��Ӧ�Ŀ����� */
	uint8 u8MutexRelayNo;           /* ������Ҫ����Ļ���̵�����Ч���翪��������� */
	uint8 u8ClearEffRelayNo;		/* ���������������̵�����Ч�����բ��������� */
	uint16 uDOutRstDelay_10ms;		/* �ȶ���ʱ���̵���������λ����Ҫ�ȴ���е�����ʧ���뷴��ʱ�䡢��ʱ��Ϣ�޹� */
	uint16 uOvTimeMsgID;			/* ��ʱ��Ϣ */
}RELAY_DIN_ITEM;

EXT void InitDOut(void);
EXT void DriveDOutTick_100Hz(void);
EXT void ActRelay(uint8 u8RelayNo, uint32 u32ActTime_10ms);
EXT void RstAllRelay(void);
EXT void RstRelay(uint8 u8RelayNo);
EXT uint32 GetRelayActTmr(uint8 u8RelayNo);
EXT BOOL ChkRelayAct(uint8 u8RelayNo);          /* �̵����Ƿ����� */
/* �̵�������Ч(Eff)���ܽӿ�, Eff����� RELAY_DIN_EFF����ϸ���ͼ�ģ��ͷ�ļ� */
EXT ACT_RELAY_RES ActRelayWithRstDelay(uint8 u8RelayNo, uint32 u32ActTime_10ms, uint32 u32RstDelay_10ms);	/* �������ü̵����ͷź���ʱ */
EXT ACT_RELAY_RES ActRelayWithFeedBack(uint8 u8RelayNo, uint32 u32ActTime_10ms, uint32 u32MaxFBTime_10ms);	/* ���DIn��鹦�� */
EXT ACT_RELAY_RES ActRelayWithTfkl(uint8 u8RelayNo, uint32 u32Conf_Tfkl, BOOL bDefaultKeep_Ext0_Int1, BOOL bIniAct);/* ��Tfkl��ʽ�������ݵļ��DIn���� */
EXT void ClearRelayEff(uint8 u8RelayNo);
EXT BOOL ChkRelayEff_Busy(uint8 u8RelayNo);     /* �̵���æ�У����������ڡ��ͷź��ȶ��� */
EXT BOOL ChkRelayEff_Clear(uint8 u8RelayNo);
EXT BOOL ChkRelayEff_Cmplt(uint8 u8RelayNo);
EXT BOOL ChkRelayEff_TimeOut(uint8 u8RelayNo);
/* ���º�������ڽ���GetRelayEff()����ֵ */
#define CHK_RELAY_EFF_SUC(i32RelayEff)		(i32RelayEff > 0)	/* �̵�����Ч������� */
#define CHK_RELAY_EFF_BUSY(i32RelayEff)		(i32RelayEff == 0)	/* �̵�����Ч�������� */
#define CHK_RELAY_EFF_IDLE(i32RelayEff)		(i32RelayEff == -1)	/* �̵�����Ч�������� */
#define CHK_RELAY_EFF_TIMEOUT(i32RelayEff)	(i32RelayEff == -2)	/* �̵���-���뷴����ʱ */
EXT int32 GetRelayEff(uint8 u8RelayNo);		/* -2:��ʱ; -1:����; 0:æ; >0:�����,��ʱ */

/*===========================================================================
 * ����ת��
 *==========================================================================*/
EXT uint32 CalRTCSecondsByDate(uint8 u8Year, uint8 u8Month, uint8 u8Day, uint8 u8Hour, uint8 u8Min, uint8 u8Sec);
EXT void CalDateByRTCSeconds(uint32 u32Seconds, REAL_TIME_VAR* pRealTime);
EXT uint16 CalDaysFromOrigin(uint8 u8Year, uint8 u8Month, uint8 u8Day);

/*===========================================================================
 * ��������
 *==========================================================================*/
EXT void sortF32Array(float32* pF32, uint16 uBufLen);

/************   exclude redefinition and c++ environment   ****************/

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
/******************************** FILE END ********************************/
