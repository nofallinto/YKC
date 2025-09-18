/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved
 *
 * File			: LibMath.c
 * Author		: Wang Renfei
 * Description	: alogrithm function
 * Comments		:
 * Date			: 2015-8-31
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * X0.1 | 2015-8-31   | king | 1. Create
 * 		|			  |		 | 
 **************************************************************************/

/***************************************************************************
 						macro definition
***************************************************************************/
#define _LIB_MATH_C_		/* exclude redefinition */


/***************************************************************************
 						include files
***************************************************************************/
/* 项目公共头文件 */
#include "stdlib.h"
#include "GlobalVar.h"
#include "MdlSys.h"

/* 本级以及下级模块头文件 */
#include "LibMath.h"
#include <string.h>
#include <math.h>

/***************************************************************************
 						global variables definition
***************************************************************************/


/***************************************************************************
						internal functions declaration
***************************************************************************/
/* <NULL> */


/***************************************************************************
 							functions definition
***************************************************************************/
/*==========================================================================
| Description	: 把变量初始化为0
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-10-13
\=========================================================================*/
void InitDataWithZero(uint8* pU8Dat, uint32 u32ByteLen)
{
	uint32* pU32Dat;
	int32 i;

	/* 头部非4字节对齐部分 */
	i = ((uint32)pU8Dat)%sizeof(uint32);
	if(i > u32ByteLen) {
	    i = u32ByteLen;
	}
	u32ByteLen -= i;
	for( ; i > 0; i--) {
		*pU8Dat++ = 0;
	}

	/* 中间4字节对齐部分 */
	pU32Dat = (uint32*)pU8Dat;
	i = u32ByteLen/sizeof(uint32);
	u32ByteLen -= i*sizeof(uint32);
	for(; i > 0; i--) {
		*pU32Dat++ = 0;
	}
	
	/* 尾部非4字节对齐部分 */
	pU8Dat = (uint8*)pU32Dat;
	for(i = u32ByteLen; i > 0; i--) {
		*pU8Dat++ = 0;
	}
}

/*==========================================================================
| Description	: 根据有功、无功计算功率因数
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2009-9-14
\=========================================================================*/
float32 CalCosFromPQ(float32 fPowerP, float32 fPowerQ)
{
	if(fPowerP == 0) {
		return 0;
	} else if(fPowerQ == 0) {
		return 1;
	} else {
		float32 fCosVal = fabsf(fPowerP)/sqrtf(fPowerP*fPowerP + fPowerQ*fPowerQ);
		if(fPowerQ < 0) {
			fCosVal = 0 - fCosVal; 
		}
		return fCosVal;
	}
}

/*==========================================================================
| Description	: 计算视在功率
| G/Out/ var	: fPowerP
| Author		: Wang Renfei			Date	: 2009-9-14
\=========================================================================*/
float32 CalPowerSFromUI(float32 fVol, float32 fCur)
{
	return fVol*fCur*(SQRT_3/KW_W);
}

/*==========================================================================
| Description	: 计算无功功率
| G/Out/ var	: / fPowerQ
| Author		: Wang Renfei			Date	: 2009-9-14
\=========================================================================*/
float32 CalPowerQFromPnCos(float32 fPowerP, float32 fCos)
{
	if(fCos == 0) {
		return fPowerP*1000;	/* 避免计算溢出 */
	} else if(fCos > 1.0f) {
		fCos = 1.0f;
	} else if(fCos < -1.0f) {
		fCos = -1.0f;
	}
	return fabsf(fPowerP)*sqrtf(1.0f - fCos*fCos)/fCos;
}

/*==========================================================================
| Description	: 计算有功功率
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2009-9-14
\=========================================================================*/
float32 CalPowerPFromUICos(float32 fVol, float32 fCur, float32 fCos)
{
	float32 fPowerP;

	fPowerP = fVol * fCur;
	fPowerP *= fabsf(fCos);
	fPowerP = fPowerP * (SQRT_3/KW_W);
	
	return fPowerP;
}

/*==========================================================================
| Description	: 计算两个功率因数之差：如果 fLagCos 滞后于 fLeadCos，返回正值；反之返回负值
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2009-9-14
\=========================================================================*/
float32 CalCosDiff(float32 fLagCos, float32 fLeadCos)
{
	if((fLagCos < 0) && (fLeadCos > 0)) {
		return 0 - ((1 + fLagCos) + (1 - fLeadCos));
	} else if((fLagCos > 0) && (fLeadCos < 0)) {
		return (1 - fLagCos) + (1 + fLeadCos);
	} else {
		return fLeadCos - fLagCos;
	}
}

/*==========================================================================
| Function name	: uint16 CalCRC(uint8 *pU8Dat, uint16 uDataLen)
| Description	: 一个给定的 ModbusAscii 消息帧，计算LRC
| In/Out/G var	: uint8*, uint16/uint16
| Author		: Wang Renfei			Date	: 2007-7-11
\=========================================================================*/
const uint16 cnst_CRC16_modbus[256] = { 
    0x0000, 0xC1C0, 0x81C1, 0x4001, 0x01C3, 0xC003, 0x8002, 0x41C2,
    0x01C6, 0xC006, 0x8007, 0x41C7, 0x0005, 0xC1C5, 0x81C4, 0x4004,
    0x01CC, 0xC00C, 0x800D, 0x41CD, 0x000F, 0xC1CF, 0x81CE, 0x400E,
    0x000A, 0xC1CA, 0x81CB, 0x400B, 0x01C9, 0xC009, 0x8008, 0x41C8,
    0x01D8, 0xC018, 0x8019, 0x41D9, 0x001B, 0xC1DB, 0x81DA, 0x401A,
    0x001E, 0xC1DE, 0x81DF, 0x401F, 0x01DD, 0xC01D, 0x801C, 0x41DC,
    0x0014, 0xC1D4, 0x81D5, 0x4015, 0x01D7, 0xC017, 0x8016, 0x41D6,
    0x01D2, 0xC012, 0x8013, 0x41D3, 0x0011, 0xC1D1, 0x81D0, 0x4010,
    0x01F0, 0xC030, 0x8031, 0x41F1, 0x0033, 0xC1F3, 0x81F2, 0x4032,
    0x0036, 0xC1F6, 0x81F7, 0x4037, 0x01F5, 0xC035, 0x8034, 0x41F4,
    0x003C, 0xC1FC, 0x81FD, 0x403D, 0x01FF, 0xC03F, 0x803E, 0x41FE,
    0x01FA, 0xC03A, 0x803B, 0x41FB, 0x0039, 0xC1F9, 0x81F8, 0x4038,
    0x0028, 0xC1E8, 0x81E9, 0x4029, 0x01EB, 0xC02B, 0x802A, 0x41EA,
    0x01EE, 0xC02E, 0x802F, 0x41EF, 0x002D, 0xC1ED, 0x81EC, 0x402C,
    0x01E4, 0xC024, 0x8025, 0x41E5, 0x0027, 0xC1E7, 0x81E6, 0x4026,
    0x0022, 0xC1E2, 0x81E3, 0x4023, 0x01E1, 0xC021, 0x8020, 0x41E0,
    0x01A0, 0xC060, 0x8061, 0x41A1, 0x0063, 0xC1A3, 0x81A2, 0x4062,
    0x0066, 0xC1A6, 0x81A7, 0x4067, 0x01A5, 0xC065, 0x8064, 0x41A4,
    0x006C, 0xC1AC, 0x81AD, 0x406D, 0x01AF, 0xC06F, 0x806E, 0x41AE,
    0x01AA, 0xC06A, 0x806B, 0x41AB, 0x0069, 0xC1A9, 0x81A8, 0x4068,
    0x0078, 0xC1B8, 0x81B9, 0x4079, 0x01BB, 0xC07B, 0x807A, 0x41BA,
    0x01BE, 0xC07E, 0x807F, 0x41BF, 0x007D, 0xC1BD, 0x81BC, 0x407C,
    0x01B4, 0xC074, 0x8075, 0x41B5, 0x0077, 0xC1B7, 0x81B6, 0x4076,
    0x0072, 0xC1B2, 0x81B3, 0x4073, 0x01B1, 0xC071, 0x8070, 0x41B0,
    0x0050, 0xC190, 0x8191, 0x4051, 0x0193, 0xC053, 0x8052, 0x4192,
    0x0196, 0xC056, 0x8057, 0x4197, 0x0055, 0xC195, 0x8194, 0x4054,
    0x019C, 0xC05C, 0x805D, 0x419D, 0x005F, 0xC19F, 0x819E, 0x405E,
    0x005A, 0xC19A, 0x819B, 0x405B, 0x0199, 0xC059, 0x8058, 0x4198,
    0x0188, 0xC048, 0x8049, 0x4189, 0x004B, 0xC18B, 0x818A, 0x404A,
    0x004E, 0xC18E, 0x818F, 0x404F, 0x018D, 0xC04D, 0x804C, 0x418C,
    0x0044, 0xC184, 0x8185, 0x4045, 0x0187, 0xC047, 0x8046, 0x4186,
    0x0182, 0xC042, 0x8043, 0x4183, 0x0041, 0xC181, 0x8180, 0x4040 };
uint16 CalCRC16ForModbus(uint8* pU8Dat, int16 iByteLen)
{
	uint16 uCrcIni = 0xFFFF;

    for ( ; iByteLen > 0; iByteLen--) {
		uCrcIni = (uCrcIni<<8) ^ cnst_CRC16_modbus[(uCrcIni>>8) ^ (*pU8Dat++)] ; 
	}
	return uCrcIni;
}

/*==========================================================================
| Function name	: uint32 CalCRC32ForFileByIni(uint8* pBufStart, uint32 u32CrcIni, uint32 u32ByteLen)
| Description	: 计算CRC
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-7-1
\=========================================================================*/
const uint32 cnst_CRC32_table[256] = {
	0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
	0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
	0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
	0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
	0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
	0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
	0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
	0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
	0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
	0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
	0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
	0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
	0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
	0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
	0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
	0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
	0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
	0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
	0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
	0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
	0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
	0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
	0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
	0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
	0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
	0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
	0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
	0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
	0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
	0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
	0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
	0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D };
uint32 CalCRC32ForFileByIni(uint8* pU8Dat, int32 i32ByteLen, uint32 u32CrcIni)
{
    for( ; i32ByteLen > 0; i32ByteLen--) {
		u32CrcIni = (u32CrcIni>>8) ^ cnst_CRC32_table[((uint8)(u32CrcIni&0xff)) ^ (*pU8Dat++)];
    }
	
    return u32CrcIni;	/* 标准版本需要取反，但不合适分段计算 */
}

/*==========================================================================
| Description	: 数据均匀采样记录、处理函数族
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-8-31
\=========================================================================*/
void InitUDatUniRec(uDAT_UNI_REC* pDatRec, uint16 uBufLen)
{
	pDatRec->uDatOKDecCnt = uBufLen;
	pDatRec->uRecPt = 0;
	pDatRec->fAvr = 0;
	pDatRec->fRmsE = 0;
}
void UniRecUDatToBuf(uDAT_UNI_REC* pDatRec, uint16 uBufLen, uint16 uData)
{
	uint16 uRecPt;

	uRecPt = pDatRec->uRecPt + 1;
	if(uRecPt >= uBufLen) {
		uRecPt = 0;
	}

	pDatRec->uDatBuf[uRecPt] = uData;
	pDatRec->uRecPt = uRecPt;
	if(pDatRec->uDatOKDecCnt) {
		pDatRec->uDatOKDecCnt--;
	}
}

/*==========================================================================
| Description	: 以水平线方式处理uDAT_UNI_REC, 即认为斜率是0
	  Avr	= sum(Dn)/DataLen
	  Rms_E = sqrt(sum((Dn - Avr)^2)/DataLen)
			= sqrt(sum(Dn^2 - 2*Dn*Avr + Avr^2)/DataLen)
			= sqrt((sum(Dn^2) - 2*sum(Dn)*Avr + (Avr^2)*DataLen)/DataLen)
			= sqrt((sum(Dn^2) - 2*sum(Dn)*Avr)/DataLen + Avr^2)
			= sqrt(sum(Dn^2)/DataLen - 2*Avr^2 + Avr^2)
			= sqrt(sum(Dn^2)/DataLen - Avr^2)
		
		[0.1.2...								N-1]	: DataBuf
	C1. ->->->->->->->->->->->->->->->->->->->->->	 -> : 采样数据填充方向
				^VVVVVVVVVVVVVVVVVVVVVVV^			 V	: 有效计算数据
			CalStartPt				RecPt-1(CalEndPt) 	: 计算未发生卷绕

	C2. ->->->->->->->->->->->->->->->->->->->->->	 -> : 采样数据填充方向
		VVVVVVVVVVVV^				^VVVVVVVVVVVVV	 V	: 有效计算数据
		Seg2 RecPt-1(CalEndPt) 	CalStartPt  Seg1		: 计算发生卷绕，分为Seg1\2两个段进行计算

| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-09-11
\=========================================================================*/
BOOL ProcUniRecUDatAsWave(uDAT_UNI_REC* pDatRec, uint16 uBufLen, uint16 uCalLen)
{
	uint16 uRecPt;										/* 计算开始数据指针 */
	uint16 *puData; 									/* 计算数据指针 */
	int16 i;											/* 内循环计数 */
	uint32 u32Sum_Dn;									/* 信号和 */
	uint64 u64Sum_Dn2;

	if(uCalLen > uBufLen - pDatRec->uDatOKDecCnt) {		/* 采样数据如果不完整 */
		return FALSE;
	} else {
		uRecPt = pDatRec->uRecPt;						/* 锁存指针 */
		puData = &(pDatRec->uDatBuf[uRecPt]);
		u32Sum_Dn = 0;
		u64Sum_Dn2 = 0;
		if(uRecPt+1 >= uCalLen) {						/* C1情况 */
			for(i = uCalLen; i > 0; i--) {
				u32Sum_Dn += (uint32)(*puData);
				u64Sum_Dn2 += (uint64)(((uint32)(*puData)) * ((uint32)(*puData)));
				puData--;
			}
		} else {										/* C2情况 */
			/* Seg2段数据 */
			for(i = uRecPt; i >= 0; i--) {
				u32Sum_Dn += (uint32)(*puData);
				u64Sum_Dn2 += (uint64)(((uint32)(*puData)) * ((uint32)(*puData)));
				puData--;
			}

			/* Seg1段数据 */
			puData = &(pDatRec->uDatBuf[uBufLen-1]);
			for(i = uCalLen - uRecPt - 1; i > 0; i--) {
				u32Sum_Dn += (uint32)(*puData);
				u64Sum_Dn2 += (uint64)(((uint32)(*puData)) * ((uint32)(*puData)));
				puData--;
			}
		}
		
		float64 f64Avr = ((float64)u32Sum_Dn)/uCalLen;
		float32 fTmp = ((float64)u64Sum_Dn2)/uCalLen - f64Avr*f64Avr;
		if(fTmp <= 0) {
			pDatRec->fRmsE = 0;
		} else {
			pDatRec->fRmsE = sqrtf(fTmp);
		}
		pDatRec->fAvr = (float32)f64Avr;
		return TRUE;
	}
}

/*==========================================================================
| Description	: 数据均匀采样记录、处理函数族
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-8-31
\=========================================================================*/
void InitU32DatUniRec(u32DAT_UNI_REC* pDatRec, uint16 uBufLen)
{
	pDatRec->fAvr = 0;
	pDatRec->fRmsE = 0;
	pDatRec->uDatOKDecCnt = uBufLen;
	pDatRec->uRecPt = 0;
	pDatRec->uFilterPt = 0;
	pDatRec->uRsvd = 0;
}
void UniRecU32DatToBuf(u32DAT_UNI_REC* pDatRec, uint16 uBufLen, uint32 u32Data)
{
	uint16 uRecPt;

	uRecPt = pDatRec->uRecPt + 1;
	if(uRecPt >= uBufLen) {
		uRecPt = 0;
	}

	pDatRec->u32DatBuf[uRecPt] = u32Data;
	pDatRec->uRecPt = uRecPt;
	if(pDatRec->uDatOKDecCnt) {
		pDatRec->uDatOKDecCnt--;
	}
}

/*==========================================================================
| Description	: 以水平线方式处理u32DAT_UNI_REC, 即认为斜率是0
	  Avr	= sum(Dn)/DataLen
	  Rms_E = sqrt(sum((Dn - Avr)^2)/DataLen)
			= sqrt(sum(Dn^2 - 2*Dn*Avr + Avr^2)/DataLen)
			= sqrt((sum(Dn^2) - 2*sum(Dn)*Avr + (Avr^2)*DataLen)/DataLen)
			= sqrt((sum(Dn^2) - 2*sum(Dn)*Avr)/DataLen + Avr^2)
			= sqrt(sum(Dn^2)/DataLen - 2*Avr^2 + Avr^2)
			= sqrt(sum(Dn^2)/DataLen - Avr^2)
		
		[0.1.2...								N-1]	: DataBuf
	C1. ->->->->->->->->->->->->->->->->->->->->->	 -> : 采样数据填充方向
				^VVVVVVVVVVVVVVVVVVVVVVV^			 V	: 有效计算数据
			CalStartPt				RecPt-1(CalEndPt) 	: 计算未发生卷绕

	C2. ->->->->->->->->->->->->->->->->->->->->->	 -> : 采样数据填充方向
		VVVVVVVVVVVV^				^VVVVVVVVVVVVV	 V	: 有效计算数据
		Seg2 RecPt-1(CalEndPt) 	CalStartPt  Seg1		: 计算发生卷绕，分为Seg1\2两个段进行计算

| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-09-11
\=========================================================================*/
BOOL ProcUniRecU32DatAsWave(u32DAT_UNI_REC* pDatRec, uint16 uBufLen, uint16 uCalStartPt, uint16 uCalLen)
{
	if(pDatRec->uDatOKDecCnt) {					/* 采样数据如果不完整 */
		return FALSE;
	} else {
		uint32* pu32Data = &(pDatRec->u32DatBuf[uCalStartPt]);
		uint64 u64Sum_Dn = 0;
		uint64 u64Sum_Dn2 = 0;
		int16 i;
		if(uCalStartPt+1 >= uCalLen) {						/* C1情况 */
			for(i = uCalLen; i > 0; i--) {
				if((*pu32Data) > 0x8000000UL) {			/* 防止溢出 */
					*pu32Data = 0x8000000UL;
				}
				u64Sum_Dn += ((uint64)(*pu32Data));
				u64Sum_Dn2 += ((uint64)(*pu32Data)) * ((uint64)(*pu32Data));
				pu32Data--;
			}
		} else {										/* C2情况 */
			/* Seg2段数据 */
			for(i = uCalStartPt; i >= 0; i--) {
				if((*pu32Data) > 0x8000000UL) {			/* 防止溢出 */
					*pu32Data = 0x8000000UL;
				}
				u64Sum_Dn += ((uint64)(*pu32Data));
				u64Sum_Dn2 += ((uint64)(*pu32Data)) * ((uint64)(*pu32Data));
				pu32Data--;
			}

			/* Seg1段数据 */
			pu32Data = &(pDatRec->u32DatBuf[uBufLen-1]);
			for(i = uCalLen - uCalStartPt - 1; i > 0; i--) {
				if((*pu32Data) > 0x8000000UL) {			/* 防止溢出 */
					*pu32Data = 0x8000000UL;
				}
				u64Sum_Dn += ((uint64)(*pu32Data));
				u64Sum_Dn2 += ((uint64)(*pu32Data)) * ((uint64)(*pu32Data));
				pu32Data--;
			}
		}
		
		float64 f64Avr = ((float64)u64Sum_Dn)/uCalLen;
		float32 fTmp = ((float64)u64Sum_Dn2)/uCalLen - f64Avr*f64Avr;
		if(fTmp <= 0) {
			pDatRec->fRmsE = 0;
		} else {
			pDatRec->fRmsE = sqrtf(fTmp);
		}
		pDatRec->fAvr = f64Avr;
		return TRUE;
	}
}

/*==========================================================================
| Description	: 滤除严重偏离的点(超过10倍rms值)，把它修正成平均值
				  不考虑两次调用之间数据填满Buf的情况，这种情况的避免依靠控制填数据速度保证
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-07-28
\=========================================================================*/
BOOL FilterUniRecU32Dat(u32DAT_UNI_REC* pDatRec, uint16 uBufLen, uint16 uCalStartPt)
{
	if(pDatRec->uDatOKDecCnt) {					/* 采样数据如果不完整 */
		return FALSE;
	} else {
		uint32* pu32Data = &(pDatRec->u32DatBuf[uCalStartPt]);
		uint32 u32HighThr = pDatRec->fAvr + pDatRec->fRmsE*10.0f;	/* 这段代码已经经过溢出性验证，上溢出会变成0xFFFFFFFF */
		uint32 u32LowThr = pDatRec->fAvr - pDatRec->fRmsE*10.0f;	/* 这段代码已经经过溢出性验证，下溢出会变成0 */
		int16 i;
		if(uCalStartPt >= pDatRec->uFilterPt) {			/* C1情况 */
			for(i = uCalStartPt - pDatRec->uFilterPt; i > 0; i--) {
				if(*pu32Data < u32LowThr) {
					*pu32Data = u32LowThr;
				} else if(*pu32Data > u32HighThr) {
					*pu32Data = u32HighThr;
				}
				pu32Data--;
			}
		} else {										/* C2情况 */
			/* Seg2段数据 */
			for(i = uCalStartPt; i >= 0; i--) {
				if(*pu32Data < u32LowThr) {
					*pu32Data = u32LowThr;
				} else if(*pu32Data > u32HighThr) {
					*pu32Data = u32HighThr;
				}
				pu32Data--;
			}

			/* Seg1段数据 */
			pu32Data = &(pDatRec->u32DatBuf[uBufLen-1]);
			for(i = uBufLen - pDatRec->uFilterPt - 1; i > 0; i--) {
				if(*pu32Data < u32LowThr) {
					*pu32Data = u32LowThr;
				} else if(*pu32Data > u32HighThr) {
					*pu32Data = u32HighThr;
				}
				pu32Data--;
			}
		}
		pDatRec->uFilterPt = uCalStartPt;		
		return TRUE;
	}
}

/*==========================================================================
| Description	: PrintStringNoOvChk(): 把一个字符串填充到Buf里面，不带溢出检查
				  PrintString(): 把一个字符串填充到Buf里面
				  PrintTopicString(): 把一个字符串填充到TopicBuf里面，即要对中
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-07-28
\=========================================================================*/
BOOL PrintString(uint8** ppBuf, uint8* pBufEnd, const char* pString)
{
	uint8* pBuf = *ppBuf;
	int32 i;
	for(i = pBufEnd - pBuf; (i > 0) && (*pString); i--) {
		*pBuf++ = *pString++;
	}
	*ppBuf = pBuf;

	if(*pString) {
		return FALSE;
	} else {
		return TRUE;
	}
}

void PrintStringNoOvChk(uint8** ppBuf, const char* pString)
{
	uint8* pBuf = *ppBuf;
	while(*pString) {
		*pBuf++ = *pString++;
	}
	*ppBuf = pBuf;	/* 修正指针 */
}

/* 拷贝最多256个字符串，带长度限制，strcpy费内存*/
void PrintStringWithMaxLen(uint8** ppBuf, const char* pString, uint8 u8MaxLen)
{
	uint8* pBuf = *ppBuf;
    for(; (u8MaxLen != 0) && (*pString != 0); u8MaxLen--) {
		*pBuf++ = *pString++;
    }
	*ppBuf = pBuf;	/* 修正指针 */
}

void PrintTopicString(uint8** ppBuf, uint32 uTopicLineLen, const char* pString)
{
	/* 计算前半空格个数 */
	int32 i32StringLen = uTopicLineLen - strlen((char*)pString);
	if(i32StringLen < 0) {
		i32StringLen = 0;
	} else {
		i32StringLen = i32StringLen/2;
	}
	
	/* 填充空格 */
	uint8* pBuf = *ppBuf;
	int32 i;
	for(i = i32StringLen; i > 0; i--) {
		*pBuf++ = ' ';
	}
	
	/* 填充内容 */
	for(i = uTopicLineLen - i32StringLen; (i > 0) && (*pString); i--) {
		*pBuf++ = *pString++;
	}

	*ppBuf = pBuf;
}

/* 从ppPropText拷贝iCharacterNum个字符到ppText；字符数量，不管是中文还是字母，都算一个字符 */
BOOL PrintStringWithCharNum(uint8** ppBuf, uint8* pBufEnd, char** ppString, int16 iCharacterNum)
{
	int32 i;

	uint8* pBuf = *ppBuf;
	char* pString = *ppString;
	
	for(i = pBufEnd - pBuf - 1; (iCharacterNum > 0) && (i > 0) && (*pString); iCharacterNum--) {
		if(*pString <= 0x7F) {
			*pBuf++ = *pString++;
			i--;
		} else if(i > 2) {
			*pBuf++ = *pString++;
			*pBuf++ = *pString++;
			i -= 2;
		} else {
			break;
		}
	}

	*ppBuf = pBuf;
	*ppString = pString;
	
	if((iCharacterNum > 0) && (*pString)) {	/* 说明字符串更长 */
		return FALSE;
	} else {
		return TRUE;
	}
}

BOOL CheckString(uint8* pU8String, uint8 u8MaxLen)
{
	if(pU8String[0] == 0) {
		return FALSE;
	} else {
		for( ; (u8MaxLen > 0) && (*pU8String != 0); u8MaxLen--) {
			pU8String++;
		}
		return (u8MaxLen != 0);
	}
}

/*==========================================================================
| Description	: 打印日期时间到缓冲,注意:结尾未填充0
| G/Out var		:
| Author		: cuiyehong			Date	: 2019-12-15
\=========================================================================*/
void PrintDateTime(uint8** ppBuf, char DateDelmtr, char TimeDelmtr)
{
	uint8 *pBuf = *ppBuf;
	REAL_TIME_VAR RealTime;
	GetRealTime(&RealTime);
	if(RealTime.u8Year > 20) {  /* 时间如果没有正确初始化，把年份显示为0001，更显眼一点 */
	    *pBuf++ = '2';
	} else {
	    *pBuf++ = '0';
	}
	*pBuf++ = '0';
	*pBuf++ = '0' + (RealTime.u8Year%100)/10;
	*pBuf++ = '0' + (RealTime.u8Year%100)%10;
	*pBuf++ = DateDelmtr;
	*pBuf++ = '0' + RealTime.u8Month/10;
	*pBuf++ = '0' + RealTime.u8Month%10;
	*pBuf++ = DateDelmtr;
	*pBuf++ = '0' + RealTime.u8Day/10;
	*pBuf++ = '0' + RealTime.u8Day%10;
	*pBuf++ = ' ';
	*pBuf++ = '0' + RealTime.u8Hour/10;
	*pBuf++ = '0' + RealTime.u8Hour%10;
	*pBuf++ = TimeDelmtr;
	*pBuf++ = '0' + RealTime.u8Minute/10;
	*pBuf++ = '0' + RealTime.u8Minute%10;
	*pBuf++ = TimeDelmtr;
	*pBuf++ = '0' + RealTime.u8Second/10;
	*pBuf++ = '0' + RealTime.u8Second%10;

	*ppBuf = pBuf;
}

/*==========================================================================
| Description	: 把浮点数转化为文本形式, 有如下形式:
|				: 根据Buf剩余空间约束、指定有效位，把浮点数打印成文本
| In/Out/G var	: iSgnDigits_pMax_nExp: 有效位，正值最多有效位，负值期望有效位
| 				     举例： fData = 1, iSgnDigits_pMax_nExp = 3, 打印出来是 1; iSgnDigits_pMax_nExp = -3, 打印出来是1.00
|               : 返回是否打印成功，如果遇到异常但buf空间够则打印的是null
| Author		: Wang Renfei			Date	: 2016-6-9
\=========================================================================*/
#define PRINTF32_SUC_WITH_BUF_UPDATE	1
#define PRINTF32_FAIL_NO_BUF_UPDATE		0
#define PRINTF32_NULL_WITH_BUF_UPDATE	(-1)
BOOL PrintF32(uint8** ppBuf, uint8* pBufEnd, float32 fData, int8 i8SgnDigits_pMax_nExpect)
{
	/* 预处理 */
	uint8* pBuf = *ppBuf;
	int16 iBufLen = pBufEnd - pBuf;
	int8 i8Res = PRINTF32_FAIL_NO_BUF_UPDATE;
	do {
		if(iBufLen == 0) {
			i8Res = PRINTF32_FAIL_NO_BUF_UPDATE;
			break;
		} else if(!isfinite(fData)) {
			i8Res = PRINTF32_NULL_WITH_BUF_UPDATE;
			break;
		} else if(fData == 0) {
			*pBuf++ = '0';
			i8Res = PRINTF32_SUC_WITH_BUF_UPDATE;
			break;
		} else if(fData < 0) {
			iBufLen--;
			if(iBufLen == 0) {
				i8Res = PRINTF32_FAIL_NO_BUF_UPDATE;
				break;
			} else {
				*pBuf++ = '-';
				fData = fabsf(fData);
			}
		}
		if(i8SgnDigits_pMax_nExpect == 0) {	/* 避免输入错误 */
			i8SgnDigits_pMax_nExpect = 3;
		}
		BOOL bSgnDigitsMax = (i8SgnDigits_pMax_nExpect >= 0);
		i8SgnDigits_pMax_nExpect = abs(i8SgnDigits_pMax_nExpect);
	
		/* 计算数据指数位 不够长度，将以e指数表达; 否则以普通的N位有效数字表达 */
		float32 fTmp = fData;
		int8 i8Exp = 0;			/* e指数位置 */
		if(fTmp >= 10.0f) {		/* 本来用log10f()的，但是对于一般小数据来说这种算法更快，大数据log10f()更快 */
			while(fTmp >= 10.0f) {
				fTmp = fTmp/10;
				i8Exp++;
			}
		} else if(fTmp < 1.0f) {
			while(fTmp < 1.0f) {
				fTmp = fTmp*10.0f;
				i8Exp--;
			}
		}

		/* 由Buf空间计算能够支持的最多有效位数、最少指数位数 */
		int16 iMinExp = 0;
		int16 iMaxSgnDigits;
		BOOL bNoExp = FALSE;					/* 无需指数表达 */
		BOOL bNoPoint = FALSE;					/* 无需小数点 */
		if(i8Exp >= 0) { /* fData >= 1 */
			int16 iIntDigitLen = i8Exp + 1;		/* 整数部分位数 */
			if(iBufLen < iIntDigitLen) {		/* 必须通过指数位折叠，否则表达不完 */
				iMinExp = iIntDigitLen - iBufLen + 2;	/* 计算e指数最小值 */
				if(iMinExp <= 9) {
					iMaxSgnDigits = iBufLen - 2;
				} else {
					iMaxSgnDigits = iBufLen - 3;
				}
				if(iMaxSgnDigits <= i8SgnDigits_pMax_nExpect) {
					i8SgnDigits_pMax_nExpect = iMaxSgnDigits;
					bNoPoint = TRUE;
				}
			} else {	/* 可以不通过指数折叠 */
				if(iBufLen > iIntDigitLen + 1) { /* 留存的Buf空间超过整数位数 */
					iMaxSgnDigits = iBufLen-1;			/* 表达为 nnn.mmm 考虑小数点 */
				} else {	/* ((iBufLen == iIntDigitLen) || (iBufLen == iIntDigitLen+1)) 正好整数表达 */
					iMaxSgnDigits = iIntDigitLen;		/* 有效位数不带任何小数点 */
					if(iBufLen <= i8SgnDigits_pMax_nExpect + 2) { 	/* 按照有效位数进行缩减，会产生小数点 */
						bNoPoint = TRUE;
					}
				}
				if(iIntDigitLen <= i8SgnDigits_pMax_nExpect) {		/* 有效位数正好够表达 */
					bNoExp = TRUE;
				} else if(iBufLen <= i8SgnDigits_pMax_nExpect + 2) {		/* 按照有效位数进行缩减，会产生小数点 */
					i8SgnDigits_pMax_nExpect = iMaxSgnDigits;
					bNoExp = TRUE;
				}
			}
		} else if(i8Exp >= -2) { 					/* fData > 0.01, 利用e指数不可能减少打印出来的长度 */
			iMaxSgnDigits = iBufLen - 1 + i8Exp; 	/* 0.nnn 或 0.0nnn */
			if(iMaxSgnDigits <= 0) {
				/* *pBuf++ = (uint32)(fData + 0.5f) + '0';  调试用，至少表达1位 */
				i8Res = PRINTF32_NULL_WITH_BUF_UPDATE;
				break;
			}
			bNoExp = TRUE;
		} else if(i8Exp >= -9) { 					/* fData < 0.01, 利用e指数才可以减少打印出来的长度 */
			iMinExp = -9;
			iMaxSgnDigits = iBufLen - 3;			/* ne-m */
			if(iMaxSgnDigits >= 2) {
				iMaxSgnDigits -= 1; 				/* n.nne-m，需要扣除小数点 */
			}
		} else {									/* fData < 1e-9， 太小了，不管了，直接认为是0 */
			*pBuf++ = '0';
			i8Res = PRINTF32_SUC_WITH_BUF_UPDATE;
			break;
		}

		if(iMaxSgnDigits <= 0) {			/* 不够表达 */
			i8Res = PRINTF32_NULL_WITH_BUF_UPDATE;
			break;
		} else if(iMaxSgnDigits < i8SgnDigits_pMax_nExpect) {
			i8SgnDigits_pMax_nExpect = iMaxSgnDigits;
		}
	
		/* 计算表达式的移位、移位后整数 */
		int8 i8ShiftExp = i8Exp + 1 - i8SgnDigits_pMax_nExpect;	/* fData需要乘的e指数 */
		if(i8ShiftExp > 19) {						/* 避免cnst_fBcdQFactor溢出 */
			i8ShiftExp = 19;
			i8Exp = 20 + (i8SgnDigits_pMax_nExpect - 1);
			if(i8Exp < iMinExp) {					/* 修正后，再次检查iExp */
				i8Res = PRINTF32_NULL_WITH_BUF_UPDATE;
				break;
			}
		} else if(i8ShiftExp < -19) {				/* 避免cnst_fBcdQFactor溢出 */
			i8ShiftExp = -19;
		}
		uint32 u32Data;
		if(i8ShiftExp >= 0) {
			u32Data = (uint32)(fData/cnst_fBcdQFactor[i8ShiftExp] + 0.5f);
		} else {
			u32Data = (uint32)(fData * cnst_fBcdQFactor[0 - i8ShiftExp] + 0.5f);
		}
		/* 指数位与小数点位置 */
		uint8 u8PointPlace; 			/* 小数点位置 */
		if(bNoPoint) {
			u8PointPlace = 0;
			i8Exp = i8ShiftExp;
		} else if(bNoExp) {
			u8PointPlace = i8SgnDigits_pMax_nExpect - 1 - i8Exp;
			i8Exp = 0;
		} else {
			u8PointPlace = i8SgnDigits_pMax_nExpect - 1;
		}
	
		/* 计算数值位数 */
		int16 iDigitNo; 				/* 位数 */
		if(u8PointPlace > i8SgnDigits_pMax_nExpect - 1) {
			iDigitNo = u8PointPlace;
		} else {
			iDigitNo = i8SgnDigits_pMax_nExpect - 1;
		}
		if(u32Data >= cnst_u32BcdQFactor[iDigitNo+1]) {	/* 在前面(uint32)(fData + 0.5)产生的误差 */
			u32Data = (u32Data + 5)/10;
			i8Exp++;
		}
		if((iDigitNo > 8) || (u32Data >= cnst_u32BcdQFactor[iDigitNo+1])) { 	/* 防止下面除法溢出 */
			i8Res = PRINTF32_NULL_WITH_BUF_UPDATE;
			break;
		}

		/* 具体译成text: 如果指定的有效位是最多有效位，即 1 打印成 "1"; 如果指定的有效位是期望有效位，即 1 打印成"1.00" */
		for( ; iDigitNo >= u8PointPlace; iDigitNo--) {		/* 小数点前 */
			*pBuf++ = u32Data/cnst_u32BcdQFactor[iDigitNo] + '0';
			u32Data = u32Data%cnst_u32BcdQFactor[iDigitNo];
		}
		if((u32Data != 0) || ((!bSgnDigitsMax) && (iDigitNo >= 0))) {
			*pBuf++ = '.';
		}
		if(bSgnDigitsMax) {
			for( ; (iDigitNo >= 0) && u32Data; iDigitNo--) {
				*pBuf++ = u32Data/cnst_u32BcdQFactor[iDigitNo] + '0';
				u32Data = u32Data%cnst_u32BcdQFactor[iDigitNo];
			}
		} else {
			for( ; iDigitNo >= 0; iDigitNo--) {
				*pBuf++ = u32Data/cnst_u32BcdQFactor[iDigitNo] + '0';
				u32Data = u32Data%cnst_u32BcdQFactor[iDigitNo];
			}
		}

		/* 指数位部分 */
		if(i8Exp) {
			*pBuf++ = 'e';
			if(i8Exp < 0) {
				*pBuf++ = '-';
				i8Exp = 0 - i8Exp;
			}
			if(i8Exp >= 10) {
				*pBuf++ = i8Exp/10 + '0';
			}
			*pBuf++ = i8Exp%10 + '0';
		}
		i8Res = PRINTF32_SUC_WITH_BUF_UPDATE;
	} while(FALSE);		/* 方便随时跳到结果处理 */

	/* 结果处理 */
	switch(i8Res) {
	case PRINTF32_SUC_WITH_BUF_UPDATE:		/* 成功，且需更新buf */
		*ppBuf = pBuf;
		return TRUE;

	case PRINTF32_NULL_WITH_BUF_UPDATE:		/* 数值未打印成功但buf有空间，则打印null且更新buf */
		if(iBufLen < 4) {
			return FALSE;
		} else {
			pBuf = *ppBuf;		/* 填充点回最原始，防止此函数执行至此已经填充了其他内容 */
			*pBuf++ = 'n';
			*pBuf++ = 'u';
			*pBuf++ = 'l';
			*pBuf++ = 'l';					/* 为避免打印数值失败时，buf里什么都不填充造成调用方解析异常  */
			*ppBuf = pBuf;
			return TRUE;
		}

	case PRINTF32_FAIL_NO_BUF_UPDATE:		/* 失败，且不需更新buf */
	default:
		return FALSE;
	}
}

void LeftAlignStringAndAddPoint(uint8** ppBuf, uint8* pBufEnd, uint16 uStringOff, uint16 uBcdQ);	/* 把转化成字符串的U32/U64数据，加上小数点，进行左对齐 */
BOOL PrintFM32(uint8** ppBuf, uint8* pBufEnd, float32 fData, F32_MEANING F32Meaning)
{
	if(F32Meaning == F32_DEFAULT) {
		return PrintF32(ppBuf, pBufEnd, fData, 5);
	} else if(F32Meaning <= F32_SgnDigits_10) {		/* 1~10位有效数字 */
		return PrintF32(ppBuf, pBufEnd, fData, F32Meaning);
	} else {
		uint8* pBuf = *ppBuf;
		/* 输入检查 */
		if(pBuf == pBufEnd) {		/* Buf溢出检查 */
			return FALSE;
		} else if(isnan(fData)) {
			fData = 0;
		} else if(fData < 0) {
			*pBuf++ = '-';
			fData = fabsf(fData);
		}
		uint32 u32Data;
		uint8 u8Carry = 0;	/* 进位标识 */
		switch(F32Meaning) {
		    case F32_INT:   /* 整数, 无小数点. [-999,9999]:sabc,s是符号,    <-999:-999, >9999:9999 */
                u32Data = (uint32)(fData + 0.5f);
                if(u32Data > 9999) {
                    u32Data = 9999;
                }
                if(u32Data/1000) {
                    *pBuf++ = u32Data/1000 + '0';
                    u32Data = u32Data%1000;
                }
                if(u32Data/100) {
                    *pBuf++ = u32Data/100 + '0';
                    u32Data = u32Data%100;
                }
                if(u32Data/10) {
                    *pBuf++ = u32Data/10 + '0';
                    u32Data = u32Data%10;
                }
                *pBuf++ = u32Data + '0';
		        break;
		        
		    case F32_TIME:	/* 最多5byte, [1s,60s):abs, [1Min,60Min):abMin, [1H,9999H]:abcdH, >9999H:9999H*/
				u32Data = (uint32)(fData + 0.5f);
				if(u32Data > 3600*9999) {   /* 限幅 */
				    u32Data = 3600*9999;
				}
				if(u32Data >= 3600) {
				    u32Data = u32Data/3600;
                    int16 i = pBufEnd - pBuf;
					i--;
					*(pBuf + i) = 'H';
					do{
						i--;
						*(pBuf + i) = u32Data%10 +'0';
						u32Data = u32Data/10;
					} while(u32Data && (i>=0));
					LeftAlignStringAndAddPoint(&pBuf, pBufEnd, i, 0);
				} else if(u32Data >= 60) {
				    u32Data = u32Data/60;
					if(u32Data/10) {
						*pBuf++ = u32Data/10 + '0';
					}
					*pBuf++ = u32Data%10 + '0';
					*pBuf++ = 'M';
					*pBuf++ = 'i';
					*pBuf++ = 'n';
				} else {
					if(u32Data/10) {
						*pBuf++ = u32Data/10 + '0';
					}
					*pBuf++ = u32Data%10 + '0';
					*pBuf++ = 's';
				}
				break;

	/* 最多9byte: <1mV:0.abcmV/m, [1mV,10mV):a.bcmV/m, [10mV,100mV):ab.cmV/m, [100mV,1V):abcmV/m, [1V,10V):a.bcV/m,
			[10V,100V):ab.cV/m, [100,1000):abcV/m, >=999:999V/m */
			case F32_VpM:
				if(pBufEnd - pBuf < 9) {
					return FALSE;
				} else {
					if(fData > 999.0f) {
						fData = 999;
					}
					if(fData + 5e-4f >= 1.0f) {
						if(fData + 5e-2f >= 100.0f) {
							u32Data =(uint32)(fData + 0.5f);
							*pBuf++ = u32Data/100 + '0';
							u32Data = u32Data%100;
							*pBuf++ = u32Data/10 + '0';
							*pBuf++ = u32Data%10 + '0';
						} else if(fData + 5e-3f >= 10.0f) {
							u32Data =(uint32)(fData*10.0f + 0.5f);
							*pBuf++ = u32Data/100 + '0';
							u32Data = u32Data%100;
							*pBuf++ = u32Data/10 + '0';
							*pBuf++ = '.';
							*pBuf++ =u32Data%10 + '0';
						} else {
							u32Data =(uint32)(fData*100.0f + 0.5f);
							*pBuf++ = u32Data/100 + '0';
							*pBuf++ = '.';
							u32Data = u32Data%100;
							*pBuf++ = u32Data/10 + '0';
							*pBuf++ =u32Data%10 + '0';
						}
						*pBuf++ = 'V';
						*pBuf++ = '/';
						*pBuf++ = 'm';
					} else {
						if(fData + 5e-5f >= 0.1f) {
							u32Data =(uint32)(fData*1e3f + 0.5f);
							*pBuf++ = u32Data/100 + '0';
							u32Data = u32Data%100;
							*pBuf++ = u32Data/10 + '0';
							*pBuf++ = u32Data%10 + '0';
						} else if(fData + 5e-6f >= 0.01f) {
							u32Data =(uint32)(fData*1e4f + 0.5f);
							*pBuf++ = u32Data/100 + '0';
							u32Data = u32Data%100;
							*pBuf++ = u32Data/10 + '0';
							*pBuf++ = '.';
							*pBuf++ =u32Data%10 + '0';
						} else if(fData + 5e-7f >= 0.001f) {
							u32Data =(uint32)(fData*1e5f + 0.5f);
							*pBuf++ = u32Data/100 + '0';
							*pBuf++ = '.';
							u32Data = u32Data%100;
							*pBuf++ = u32Data/10 + '0';
							*pBuf++ =u32Data%10 + '0';
						} else {
							u32Data =(uint32)(fData*1e6f + 0.5f);
							*pBuf++ = '0';
							*pBuf++ = '.';
							*pBuf++ = u32Data/100 + '0';
							u32Data = u32Data%100;
							*pBuf++ = u32Data/10 + '0';
							*pBuf++ =u32Data%10 + '0';
						}
						*pBuf++ = 'm';
						*pBuf++ = 'V';
						*pBuf++ = '/';
						*pBuf++ = 'm';
					}
				}
				break;

			case F32_CENT:		/* 最多5byte, [0,100):ab.c%, [100,999):abc%, >=999:999% */
			case F32_TEMP:		/* 最多6byte, [0,100):ab.c℃, [100,999):abc℃, >=999:999℃ */
				if(((F32Meaning == F32_CENT) && (pBufEnd - pBuf < 5)) || ((F32Meaning == F32_TEMP) && (pBufEnd - pBuf < 6))) {	
					return FALSE;
				} else {
					if(fData > 999.0f) {
						fData = 999;
					}
					u32Data = (uint32)(fData*10.0f + 0.5f);
					if(u32Data < 1000) {
						if(u32Data/100) {
							*pBuf++ = u32Data/100 + '0';
							u32Data = u32Data%100;
						}
						*pBuf++ = u32Data/10 + '0';
						*pBuf++ = '.';
						*pBuf++ = u32Data%10 + '0';
					} else {
						u32Data = (u32Data + 5)/10;
						*pBuf++ = u32Data/100 + '0';
						u32Data = u32Data%100;
						*pBuf++ = u32Data/10 + '0';
						*pBuf++ = u32Data%10 + '0';
					}
				}
				if(F32Meaning == F32_CENT) {
					*pBuf++ = '%';
				} else if(F32Meaning == F32_TEMP) {
					PrintString(&pBuf, pBufEnd, "℃");
				}
				break;

			case F32_WATER:	/* 最多6byte：[0, 10)显示为a.bcdm; [10, inf]显示为 ab.cdm， 最多6字符 */
			case F32_MPa:   /* 最多6byte：[0,9.999]:a.bcdMPa, (9.999,99.99]:ab.cdMPa, >99.99:99.99Mpa， 最多8字符 */
				if(((F32Meaning == F32_WATER) && (pBufEnd - pBuf < 6))
				    || ((F32Meaning == F32_MPa) && (pBufEnd - pBuf < 8)))
				{	
					return FALSE;
				} else {
					if(fData > 99.99f) {
						fData = 99.99;
					}
					uint32 u32Data = (uint32)(fData*1000.0f + 0.5f);
					if(u32Data < 10000) {
						*pBuf++ = u32Data/1000 + '0';
						u32Data = u32Data%1000;
						*pBuf++ = '.';
						*pBuf++ = u32Data/100 + '0';
						u32Data = u32Data%100;
						*pBuf++ = u32Data/10 + '0';
						*pBuf++ = u32Data%10 + '0';
					} else {
						u32Data = (u32Data + 5)/10;
						*pBuf++ = u32Data/1000 + '0';
						u32Data = u32Data%1000;
						*pBuf++ = u32Data/100 + '0';
						u32Data = u32Data%100;
						*pBuf++ = '.';
						*pBuf++ = u32Data/10 + '0';
						*pBuf++ = u32Data%10 + '0';
					}
				}
				if(F32Meaning == F32_WATER) {
    				*pBuf++ = 'm';
    			} else {
    				*pBuf++ = 'M';
    				*pBuf++ = 'P';
    				*pBuf++ = 'a';
    			}
				break;

			case F32_FREQ:	/*最多7byte, [0,99.99]:ab.cdHz, [100,999.9):abc.dHz, >=999: 999.0Hz */
			case F32_BatAH: /*最多7byte, [0,99.99]:ab.cdAH, [100,999.9):abc.dAH, >=999: 999.0AH */
				if(pBufEnd - pBuf < 7) {	
					return FALSE;
				} else {
					if(fData > 999.9f) {
						fData = 999.9;
					}
					u32Data = (uint32)(fData*100.0f + 0.5f);
					if(u32Data < 10000) {
						if(u32Data/1000) {
							*pBuf++ = u32Data/1000 + '0';
							u32Data = u32Data%1000;
						}
						*pBuf++ = u32Data/100 + '0';
						u32Data = u32Data%100;
						*pBuf++ = '.';
						*pBuf++ = u32Data/10 + '0';
						*pBuf++ = u32Data%10 + '0';
					} else {
						u32Data = (u32Data + 5)/10;
						*pBuf++ = u32Data/1000 + '0';
						u32Data = u32Data%1000;
						*pBuf++ = u32Data/100 + '0';
						u32Data = u32Data%100;
						*pBuf++ = u32Data/10 + '0';
						*pBuf++ = '.';
						*pBuf++ = u32Data%10 + '0';
					}
				}
				/* 单位部分 */
				if(F32Meaning == F32_FREQ) {
					*pBuf++ = 'H';
					*pBuf++ = 'z';
				} else if(F32Meaning == F32_BatAH) {
					*pBuf++ = 'A';
					*pBuf++ = 'H';
				}
				break;
			
			case F32_VOL:/*最多6byte [0,10):a.bcV, [10,100):ab.cV, [100,8e3):  abcdV, [8e3,1e4):a.bcKV, [1e4, 1e5):ab.cKV, >=1e5:abcdKV */
			case F32_CUR:/* 最多6byte */
			case F32_PwrP:/*最多6byte:  [0,10):a.bcKW, [10,100):ab.cKW, [100,1000): abcKW, [1000, 1e4):a.bcMW, [1e4, 1e5):ab.cMW, [1e5, 1e6):abcdMW */
			case F32_PwrQ:/*最多8byte:  [0,10):a.bcKVar, [10,100):ab.cKVar, [100,1000): abcKVar, [1000, 1e4):a.bcMVar, [1e4, 1e5):ab.cMVar, [1e5, 1e6):abcdMVar */
				/* 溢出与进位检查 */
				if((F32Meaning == F32_VOL) || (F32Meaning == F32_CUR)) {
					if(pBufEnd - pBuf < 6) {	
						return FALSE;
					} else if(fData >= 8000.0f) {
						fData = fData/1000.0f;
						u8Carry = 1;
					}
				} else if(F32Meaning == F32_PwrP) {
					if(pBufEnd - pBuf < 6) {	
						return FALSE;
					} else if(fData >= 1000.0f) {
						fData = fData/1000.0f;
						u8Carry = 1;
					}
				} else if(F32Meaning == F32_PwrQ) {
					if(pBufEnd - pBuf < 8) {	
						return FALSE;
					} else if(fData >= 1000.0f) {
						fData = fData/1000.0f;
						u8Carry = 1;
					}
				}
				
				/* 把数字译成text */
				if(fData >= 9999.0f) {
					fData = 9999;
				}
				u32Data = (uint32)(fData*100.0f + 0.5f);
				if(u32Data < 1000) {
					*pBuf++ = u32Data/100 + '0';
					u32Data = u32Data%100;
					*pBuf++ = '.';
					*pBuf++ = u32Data/10 + '0';
					u32Data = u32Data%10;
					*pBuf++ = u32Data + '0';
				} else if(u32Data < 9995) {
					u32Data = (u32Data + 5)/10;
					*pBuf++ = u32Data/100 + '0';
					u32Data = u32Data%100;
					*pBuf++ = u32Data/10 + '0';
					u32Data = u32Data%10;
					*pBuf++ = '.';
					*pBuf++ = u32Data + '0';
				} else {
					u32Data = (u32Data + 50)/100;
					if(u32Data >= 1000) {
						*pBuf++ = u32Data/1000 + '0';
						u32Data = u32Data%1000;
					}
					*pBuf++ = u32Data/100 + '0';
					u32Data = u32Data%100;
					*pBuf++ = u32Data/10 + '0';
					u32Data = u32Data%10;
					*pBuf++ = u32Data + '0';
				}
				
				/* 单位部分 */
				if(u8Carry == 1) {
					if((F32Meaning == F32_VOL) || (F32Meaning == F32_CUR)) {
						*pBuf++ = 'K';
					} else {
						*pBuf++ = 'M';
					}
				} else if((F32Meaning == F32_PwrP) || (F32Meaning == F32_PwrQ)) {	/* 隐含u8Carry == 0 */
					*pBuf++ = 'K';
				}
				if(F32Meaning == F32_VOL) {
					*pBuf++ = 'V';
				} else if(F32Meaning == F32_CUR) {
					*pBuf++ = 'A';
				} else if(F32Meaning == F32_PwrP) {
					*pBuf++ = 'W';
				} else if(F32Meaning == F32_PwrQ) {
					*pBuf++ = 'V';
					*pBuf++ = 'a';
					*pBuf++ = 'r';
				}
				break;
			
			case F32_COS:	/* 最多5byte, [-1.1,1.1]:a.bcd, 除此之外，仅显示整数部分 */
				if(pBufEnd - pBuf < 5) {	
					return FALSE;
				} else if(fData > 1.1f) {	/* 特殊含义的指令 */
					if(fData >= 9.0f) {
						*pBuf++ = '9';
					} else {
						*pBuf++ = (uint16)fData + '0';
					}
				} else {	/* 普通cos */
					u32Data = (uint32)(fData*1000.0f + 0.5f);
					*pBuf++ = u32Data/1000 + '0';
					u32Data = u32Data%1000;
					*pBuf++ = '.';
					*pBuf++ = u32Data/100 + '0';
					u32Data = u32Data%100;
					*pBuf++ = u32Data/10 + '0';
					u32Data = u32Data%10;
					*pBuf++ = u32Data + '0';
				}
				break;

/* 有/无功电度(中,8byte):[0,10):a.bc度, [10,1e3):abc.d度, [1e3,1e4):abcd度, [1e4,1e5)a.bc万度, [1e5,1e7):ab.c万度, [1e7,1e8):abcd万度, 最大9999亿度
   有功电度(英,8byte):[0,100):ab.cdKWH, [100,1e3):abc.dKWH, [1e3,1e5):ab.cdMWH, [1e5,1e6)abc.dMWH, 最大999.9TWH
   无功电度(英,10byte):[0,100):ab.cdKVarH, [100,1e3):abc.dKVarH, [1e3,1e5):ab.cdMVarH, [1e5,1e6)abc.dMVarH, 最大999.9TVarH */
			case F32_EngP:
			case F32_EngQ:	
				if(g_Sys.u32LocalLang == CHINESE) {	/* 中文采用 万/亿进制 */
					if(pBufEnd - pBuf < 8) {	
						return FALSE;
					} else if(fData >= 1e8f) {
						fData = fData/1e8f;
						u8Carry = 2;
					} else if(fData >= 1e4f) {
						fData = fData/1e4f;
						u8Carry = 1;
					}
					
					/* 把数字译成text */
					if(fData > 9999.0f) {
						fData = 9999;
					}
					u32Data = (uint32)(fData*100.0f + 0.5f);
					if(u32Data < 1000) {
						*pBuf++ = u32Data/100 + '0';
						u32Data = u32Data%100;
						*pBuf++ = '.';
						*pBuf++ = u32Data/10 + '0';
						u32Data = u32Data%10;
						*pBuf++ = u32Data + '0';
					} else if(u32Data < 99995) {
						u32Data = (u32Data + 5)/10;
						if(u32Data >= 1000) {
							*pBuf++ = u32Data/1000 + '0';
							u32Data = u32Data%1000;
						}
						*pBuf++ = u32Data/100 + '0';
						u32Data = u32Data%100;
						*pBuf++ = u32Data/10 + '0';
						*pBuf++ = '.';
						*pBuf++ = u32Data%10 + '0';
					} else {
						u32Data = (u32Data + 50)/100;	/* (99995 + 50)/100 = 1000 */
						*pBuf++ = u32Data/1000 + '0';
						u32Data = u32Data%1000;
						*pBuf++ = u32Data/100 + '0';
						u32Data = u32Data%100;
						*pBuf++ = u32Data/10 + '0';
						u32Data = u32Data%10;
						*pBuf++ = u32Data + '0';
					}

					if(u8Carry == 0) {
						PrintString(&pBuf, pBufEnd, "度");
					} else if(u8Carry == 1) {
						PrintString(&pBuf, pBufEnd, "万度");
					} else if(u8Carry == 2) {
						PrintString(&pBuf, pBufEnd, "亿度");
					}
				} else {	/* 否则采用K/M/G进制 */
					if(((F32Meaning == F32_EngP) && (pBufEnd - pBuf < 8)) || ((F32Meaning == F32_EngQ) && (pBufEnd - pBuf < 10))) {
						return FALSE;
					} else if(fData >= 1e9f) {
						fData = fData/1e9f;
						u8Carry = 3;
					} else if(fData >= 1e6f) {
						fData = fData/1e6f;
						u8Carry = 2;
					} else if(fData >= 1e3f) {
						fData = fData/1e3f;
						u8Carry = 1;
					}
					
					/* 把数字译成text */
					if(fData > 999) {
						fData = 999;
					}
					u32Data = (uint32)(fData*100.0f + 0.5f);
					if(u32Data < 10000) {
						if(u32Data >= 1000) {
							*pBuf++ = u32Data/1000 + '0';
							u32Data = u32Data%1000;
						}
						*pBuf++ = u32Data/100 + '0';
						u32Data = u32Data%100;
						*pBuf++ = '.';
						*pBuf++ = u32Data/10 + '0';
						*pBuf++ = u32Data%10 + '0';
					} else {
						u32Data = (u32Data + 5)/10;
						if(u32Data >= 1000) {
							*pBuf++ = u32Data/1000 + '0';
							u32Data = u32Data%1000;
						}
						*pBuf++ = u32Data/100 + '0';
						u32Data = u32Data%100;
						*pBuf++ = u32Data/10 + '0';
						*pBuf++ = '.';
						*pBuf++ = u32Data%10 + '0';
					}

					/* 单位 */
					if(u8Carry == 0) {
						*pBuf++ = 'K';
					} else if(u8Carry == 1) {
						*pBuf++ = 'M';
					} else if(u8Carry == 2) {
						*pBuf++ = 'G';
					} else if(u8Carry == 3) {
						*pBuf++ = 'T';
					}
					if(F32Meaning == F32_EngP) {
						*pBuf++ = 'W';
					} else if(F32Meaning == F32_EngQ) {
						*pBuf++ = 'V';
						*pBuf++ = 'a';
						*pBuf++ = 'r';
					}
					*pBuf++ = 'H';
				}
				break;

    /* 最多8Byte: [0,10):a.bcOhm, [10,100):ab.cOhm, [100,1e3):  abcOhm, [1e3,1e4):a.bcKOhm, [1e4, 1e5):ab.cKOhm, [1e5, 1e6):abcKOhm,
            [1e6,1e7):a.bcMOhm, [1e7,1e8):ab.cMOhm, [1e8,1e9):abcMOhm, >=1e9:999MOhm   */
            case F32_OHM:
                if(pBufEnd - pBuf < 8) {
                    return FALSE;
				} else if(fData >= 1e6f) {
				    fData = fData/1e6f;
					u8Carry = 2;
                } else if(fData >= 1e3f) {
                    fData = fData/1e3f;
					u8Carry = 1;
				} else {				    
                    u8Carry = 0;
				}
				
                /* 把数字译成text */
                if(fData > 999.0f) {
                    fData = 999;
                }
                u32Data = (uint32)(fData*100.0f + 0.5f);
				if(u32Data < 1000) {
					*pBuf++ = u32Data/100 + '0';
					u32Data = u32Data%100;
					*pBuf++ = '.';
					*pBuf++ = u32Data/10 + '0';
					u32Data = u32Data%10;
					*pBuf++ = u32Data + '0';
				} else if(u32Data < 9995) {
					u32Data = (u32Data + 5)/10;
					*pBuf++ = u32Data/100 + '0';
					u32Data = u32Data%100;
					*pBuf++ = u32Data/10 + '0';
					u32Data = u32Data%10;
					*pBuf++ = '.';
					*pBuf++ = u32Data + '0';
				} else {
					u32Data = (u32Data + 50)/100;
					*pBuf++ = u32Data/100 + '0';
					u32Data = u32Data%100;
					*pBuf++ = u32Data/10 + '0';
					u32Data = u32Data%10;
					*pBuf++ = u32Data + '0';
				}

                if(u8Carry == 1) {
                    *pBuf++ = 'K';
                } else if(u8Carry == 2) {
                    *pBuf++ = 'M';
                }
                *pBuf++ = 'O';
                *pBuf++ = 'h';
                *pBuf++ = 'm';
				break;
				
			case F32_FlowRate:		/* 最多11byte：(-inf, 10]显示为 -ab.cdem3/s; (-10, 0)显示为-a.bcdem3/s; [0, 10)显示为a.bcdem3/s; [10, inf)显示为 ab.cdem3/s */
			case F32_FlowVlct:		/* 最多10byte：(-inf, 10]显示为 -ab.cdem/s; (-10, 0)显示为-a.bcdem/s; [0, 10)显示为a.bcdem/s; [10, inf)显示为 ab.cdem/s */
				if(pBufEnd - pBuf < (F32Meaning == F32_FlowRate ? 11 : 10)) {
					return FALSE;
				} else {
					if(fData > 99.99f) {
						fData = 99.99;
					}
					uint32 u32Data = (uint32)(fData*10000.0f + 0.5f);
					if(u32Data < 100000) {		/* 1位整数 */
						*pBuf++ = u32Data/10000 + '0';
						u32Data = u32Data%10000;
						*pBuf++ = '.';
						*pBuf++ = u32Data/1000 + '0';
						u32Data = u32Data%1000;
						*pBuf++ = u32Data/100 + '0';
						u32Data = u32Data%100;
						*pBuf++ = u32Data/10 + '0';
						*pBuf++ = u32Data%10 + '0';
					} else {	/* 2位整数 */
						u32Data = (u32Data + 5)/10;			/* 四舍五入 */
						*pBuf++ = u32Data/100000 + '0';
						u32Data = u32Data%100000;
						*pBuf++ = u32Data/10000 + '0';
						u32Data = u32Data%10000;
						*pBuf++ = '.';
						*pBuf++ = u32Data/1000 + '0';
						u32Data = u32Data%1000;
						*pBuf++ = u32Data/100 + '0';
						u32Data = u32Data%100;
						*pBuf++ = u32Data/10 + '0';
					}
				}
				/* 单位部分 */
				*pBuf++ = 'm';
				if(F32Meaning == F32_FlowRate) {
					*pBuf++ = '3';
				}
				*pBuf++ = '/';
				*pBuf++ = 's';
				break;

			default:
				return FALSE;
		}
		*ppBuf = pBuf;
	}
	return TRUE;
}

void PrintFM32NoOvChk(uint8* pBuf, float32 fData, F32_MEANING F32Meaning)
{
	PrintFM32(&pBuf, pBuf + 20, fData, F32Meaning);
	*pBuf = 0;
}

/*==========================================================================
| Description	: 把I16/I32/I64/U32/H64变量转化为字符串，要求:起始指针之后至少30Byte空余长度
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-6-9
\=========================================================================*/
BOOL PrintI16(uint8** ppBuf, uint8* pBufEnd, int16 iData, uint16 uBcdQ)
{
	uint8* pBuf = *ppBuf;
	int16 i = pBufEnd - pBuf;
	
	if(i < 7) {				/* 16位变量加上小数点所可能的占用最大宽度 */
		return FALSE;
	} else {
		if(iData < 0) {
			*pBuf++ = '-';
			iData = 0 - iData;
			i--;
		}
		do {
			i--;
			*(pBuf + i) = iData%10 + '0';
			iData = iData/10;
		} while(iData && (i > 0));	/* 第0位保存了符号 */
		LeftAlignStringAndAddPoint(&pBuf, pBufEnd, i, uBcdQ);
		*ppBuf = pBuf;
		return TRUE;
	}
}

BOOL PrintI32(uint8** ppBuf, uint8* pBufEnd, int32 i32Data, uint16 uBcdQ)
{
	uint8* pBuf = *ppBuf;
	int16 i = pBufEnd - pBuf;
	if(i < 11) {				/* 32位变量加上小数点所可能的占用最大宽度 */
		return FALSE;
	} else {
		if(i32Data < 0) {		/* 负值最小在-999999999UL,以免Buf溢出 */
			*pBuf++ = '-';
			i32Data = 0 - i32Data;
			i--;
			if(i32Data > 999999999UL) {
				i32Data = 999999999UL;
			}
		}
		do {
			i--;
			*(pBuf + i) = i32Data%10 + '0';
			i32Data = i32Data/10;
		} while(i32Data && (i > 0));	/* 第0位保存了符号 */
		LeftAlignStringAndAddPoint(&pBuf, pBufEnd, i, uBcdQ);
		*ppBuf = pBuf;
		return TRUE;
	}
}

BOOL PrintI64(uint8** ppBuf, uint8* pBufEnd, int64 i64Data, uint16 uBcdQ)
{
	uint8* pBuf = *ppBuf;
	int16 i = pBufEnd - pBuf;
	
	if(i < 22) {		/* int64最长22：19个数字+小数点+可能最前面的0+前面可能的负号 */
		return FALSE;
	} else {
		if(i64Data < 0) {
			*pBuf++ = '-';
			i64Data = 0 - i64Data;
			i--;
		}
		do {
			i--;
			*(pBuf + i) = i64Data%10 + '0';
			i64Data = i64Data/10;
		} while(i64Data && (i > 0));
		LeftAlignStringAndAddPoint(&pBuf, pBufEnd, i, uBcdQ);
		*ppBuf = pBuf;
		return TRUE;
	}
}

BOOL PrintU32(uint8** ppBuf, uint8* pBufEnd, uint32 u32Data, uint16 uBcdQ)
{
	uint8* pBuf = *ppBuf;
	int16 i = pBufEnd - pBuf;
	
	if(i < 11) {	/* 32位变量加上小数点所可能的占用最大宽度 */
		return FALSE;
	} else {
		do {
			i--;
			*(pBuf + i) = u32Data%10 + '0';
			u32Data = u32Data/10;
		} while(u32Data && (i > 0));
		LeftAlignStringAndAddPoint(ppBuf, pBufEnd, i, uBcdQ);
		return TRUE;
	}
}

void PrintU32WithLen(uint8** ppBuf, uint32 u32Data, uint16 uLen)
{
	uint8* pBuf = *ppBuf;
	int16 i;
	
	for(i = uLen-1 ; i >= 0; i--) {				/* 形成4位格式 */
		*(pBuf + i) = u32Data%10 + '0';
		u32Data = u32Data/10;
	}

	*ppBuf = pBuf + uLen;
}

BOOL PrintH16(uint8** ppBuf, uint8* pBufEnd, uint16 uData)
{
	uint8* pBuf = *ppBuf;
	int16 i = pBufEnd - pBuf;
	uint8 u8Data;
	
	if(i < 4) {
		return FALSE;
	} else {
		for(i = 3; i >= 0; i--) {
			u8Data = uData&0x0F;
			if(u8Data < 10) {
				*(pBuf + i) = u8Data + '0';
			} else {
				*(pBuf + i) = u8Data + 'A' - 10;
			}
			uData = (uData>>4);
		}
		*ppBuf = pBuf + 4;
		return TRUE;
	}
}

BOOL PrintH32(uint8** ppBuf, uint8* pBufEnd, uint32 u32Data)
{
	uint8* pBuf = *ppBuf;
	int16 i = pBufEnd - pBuf;
	uint8 u8Data;

	if(i < 8) {
		return FALSE;
	} else {
		for(i = 7; i >= 0; i--) {
			u8Data = u32Data&0x0F;
			if(u8Data < 10) {
				*(pBuf + i) = u8Data + '0';
			} else {
				*(pBuf + i) = u8Data + 'A' - 10;
			}
			u32Data = (u32Data>>4);
		}
		*ppBuf = pBuf + 8;
		return TRUE;
	}
}

BOOL PrintH64(uint8** ppBuf, uint8* pBufEnd, uint64 u64Data)
{
	uint8* pBuf = *ppBuf;
	int16 i = pBufEnd - pBuf;
	uint8 u8Data;
	
	if(i < 16) {
		return FALSE;
	} else {
		for(i = 15; i >= 0; i--) {
			u8Data = u64Data&0x0F;
			if(u8Data < 10) {
				*(pBuf + i) = u8Data + '0';
			} else {
				*(pBuf + i) = u8Data + 'A' - 10;
			}
			u64Data = (u64Data>>4);
		}
		*ppBuf = pBuf + 16;
		return TRUE;
	}
}

BOOL PrintIPv4(uint8** ppBuf, uint8* pBufEnd, uint32 u32IPv4)
{
	uint8* pBuf = *ppBuf;
	int16 i = pBufEnd - pBuf;
	uint8 u8Data, u8Quot;
	BOOL bHundredsNOTZero;

	if(i < 15) {	/* 32位变量加上小数点所可能的占用最大宽度 */
		return FALSE;
	} else {
		for(i = 3; i >= 0; i--) {
			u8Data = u32IPv4/cnst_u32IPv4Factor[i];
			u32IPv4 = u32IPv4%cnst_u32IPv4Factor[i];

			u8Quot = u8Data/100;
			u8Data = u8Data%100;
			if(u8Quot) {
				*pBuf++ = u8Quot + '0';
				bHundredsNOTZero = TRUE;
			} else {
				bHundredsNOTZero = FALSE;
			}
			u8Quot = u8Data/10;
			u8Data = u8Data%10;
			if(u8Quot || bHundredsNOTZero) {
				*pBuf++ = u8Quot + '0';
			}
			*pBuf++ = u8Data + '0';
			*pBuf++ = '.';
		}
		*ppBuf = pBuf - 1;
		return TRUE;
	}
}

BOOL PrintT64(DATA_USER DataUser, uint8** ppBuf, uint8* pBufEnd, REAL_TIME_VAR* pRealTime)
{
	uint8* pBuf = *ppBuf;

	if(pBufEnd - pBuf < 21) {
		return FALSE;
	} else {
		uint16 uData = pRealTime->uMilliSec;
		int16 i;
		for(i = 2; i >= 0; i--) {
			*(pBuf + 18 + i) = uData%10 + '0';
			uData = uData/10;
		}
		*(pBuf + 17) = '.';
		uData = pRealTime->u8Second;
		*(pBuf + 16) = uData%10 + '0';
		*(pBuf + 15) = (uData/10)%10 + '0';
		*(pBuf + 14) = ':';
		uData = pRealTime->u8Minute;
		*(pBuf + 13) = uData%10 + '0';
		*(pBuf + 12) = (uData/10)%10 + '0';
		*(pBuf + 11) = ':';
		uData = pRealTime->u8Hour;
		*(pBuf + 10) = uData%10 + '0';
		*(pBuf + 9) = (uData/10)%10 + '0';
		*(pBuf + 8) = ' ';
		uData = pRealTime->u8Day;
		*(pBuf + 7) = uData%10 + '0';
		*(pBuf + 6) = (uData/10)%10 + '0';
		if(DataUser == DATA_USER_NET) {
			uData = pRealTime->u8Month;
			*(pBuf + 5) = uData%10 + '0';
			*(pBuf + 4) = uData/10 + '0';
			uData = pRealTime->u8Year;
			*(pBuf + 3) = uData%10 + '0';
			*(pBuf + 2) = (uData/10)%10 + '0';
			*(pBuf + 1) = '0';
			*(pBuf + 0) = '2';
		} else {
			*(pBuf + 5) = '/';
			uData = pRealTime->u8Month;
			*(pBuf + 4) = uData%10 + '0';
			*(pBuf + 3) = (uData/10)%10 + '0';
			*(pBuf + 2) = '/';
			uData = pRealTime->u8Year;
			*(pBuf + 1) = uData%10 + '0';
			*(pBuf + 0) = (uData/10)%10 + '0';
		}
		pBuf += 21;
	}

	*ppBuf = pBuf;
	return TRUE;
}

BOOL PrintT32(DATA_USER DataUser, uint8** ppBuf, uint8* pBufEnd, uint32 u32Seconds)
{
	uint8* pBuf = *ppBuf;

	if(pBufEnd - pBuf < 17) {
		return FALSE;
	} else {
		REAL_TIME_VAR RealTime;
		extern void CalDateByRTCSeconds(uint32 u32Seconds, REAL_TIME_VAR* pRealTime);
		CalDateByRTCSeconds(u32Seconds, &RealTime);
		
		uint16 uData = RealTime.u8Second;
		*(pBuf + 16) = uData%10 + '0';
		*(pBuf + 15) = (uData/10)%10 + '0';
		*(pBuf + 14) = ':';
		uData = RealTime.u8Minute;
		*(pBuf + 13) = uData%10 + '0';
		*(pBuf + 12) = (uData/10)%10 + '0';
		*(pBuf + 11) = ':';
		uData = RealTime.u8Hour;
		*(pBuf + 10) = uData%10 + '0';
		*(pBuf + 9) = (uData/10)%10 + '0';
		*(pBuf + 8) = ' ';
		uData = RealTime.u8Day;
		*(pBuf + 7) = uData%10 + '0';
		*(pBuf + 6) = (uData/10)%10 + '0';
		if(DataUser == DATA_USER_NET) {
			uData = RealTime.u8Month;
			*(pBuf + 5) = uData%10 + '0';
			*(pBuf + 4) = uData/10 + '0';
			uData = RealTime.u8Year;
			*(pBuf + 3) = uData%10 + '0';
			*(pBuf + 2) = (uData/10)%10 + '0';
			*(pBuf + 1) = '0';
			*(pBuf + 0) = '2';
		} else {
			*(pBuf + 5) = '/';
			uData = RealTime.u8Month;
			*(pBuf + 4) = uData%10 + '0';
			*(pBuf + 3) = (uData/10)%10 + '0';
			*(pBuf + 2) = '/';
			uData = RealTime.u8Year;
			*(pBuf + 1) = uData%10 + '0';
			*(pBuf + 0) = (uData/10)%10 + '0';
		}
		pBuf += 17;
	}

	*ppBuf = pBuf;
	return TRUE;
}

BOOL PrintSoftVer(uint8** ppBuf, uint8* pBufEnd, uint16 uData)
{
	uint8* pBuf = *ppBuf;
	if(pBufEnd - pBuf < 6) {
		return FALSE;
	}
	
	/* 变量数值部分 */
	*pBuf++ = uData/1000 + '0';
	uData = uData%1000;
	*pBuf++ = '.';
	*pBuf++ = uData/100 + '0';
	uData = uData%100;
	*pBuf++ = '.';
	*pBuf++ = uData/10 + '0';
	uData = uData%10;
	*pBuf++ = uData + '0';

	*ppBuf = pBuf;	/* 修正指针 */
	return TRUE;
}

/* 峰谷电时间起点，显示为p#hh:mm */
BOOL PrintPriceTime(uint8** ppBuf, uint8* pBufEnd, uint32 u32pvTime)
{
	uint8* pBuf = *ppBuf;
	if(pBufEnd - pBuf < 7) {
		return FALSE;
	}

	/* 变量数值部分 */
	if(u32pvTime >= 100000) {	/* 防止输入出错 */
		*pBuf++ = '9';				
		u32pvTime = u32pvTime%100000;
	}
	if(u32pvTime >= 10000) {
		if(u32pvTime >= 99999) {				/* 防止输入出错 */
			*pBuf++ = '9';
		} else {
			*pBuf++ = u32pvTime/10000 + '0';
		}
		u32pvTime = u32pvTime%10000;
		*pBuf++ = '#';
	}
	*pBuf++ = u32pvTime/1000 + '0';
	u32pvTime = u32pvTime%1000;
	*pBuf++ = u32pvTime/100 + '0';
	u32pvTime = u32pvTime%100;
	*pBuf++ = ':';
	*pBuf++ = u32pvTime/10 + '0';
	u32pvTime = u32pvTime%10;
	*pBuf++ = u32pvTime + '0';
	
	*ppBuf = pBuf;	/* 修正指针 */
	return TRUE;
}

BOOL GetF32(uint8** ppText, uint8* pTextEnd, float32* pF32)
{
	uint8* pText = *ppText;
	int32 i32BytesToLineEnd = pTextEnd - pText;
	uint32 u32Data = 0;
	uint8 u8PointPlace = 0;
	int8 i8Exp = 0;
	BOOL bFindData = FALSE;
	BOOL bFindNeg = FALSE;
	BOOL bFindPoint = FALSE;
	
	/* 数值部分 */
	for( ; i32BytesToLineEnd > 0; i32BytesToLineEnd--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u32Data = u32Data*10 + ((uint32)(*pText - '0'));
			if(bFindPoint) {
				u8PointPlace++;
			}
		} else if((*pText == '.') && (!bFindPoint) && (('0' <= *(pText+1)) && (*(pText+1) <= '9'))) {/* 小数点需要后面有数字 */
			bFindPoint = TRUE;
		} else if((*pText == '-') && (!bFindNeg) && (!bFindData) && (('0' <= *(pText+1)) && (*(pText+1) <= '9'))) {	/* 负号需要后面有数字 */
			bFindNeg = TRUE;
		} else if(bFindData) {
			break;
		}
		pText++;
	}
	
	/* 指数部分 */
	if((i32BytesToLineEnd > 1) && (*pText == 'e')) {
		pText++;
		BOOL bExpNeg = FALSE;
		if(*pText == '-') {
			bExpNeg = TRUE;
			pText++;
			i32BytesToLineEnd -= 2;
		} else {			
			i32BytesToLineEnd--;
		}
		for( ; i32BytesToLineEnd > 0; i32BytesToLineEnd--) {
			if(('0' <= *pText) && (*pText <= '9')) {
				i8Exp = i8Exp*10 + (*pText - '0');		/* 原先是i8Exp = i8Exp*10 + ((uint32)(*pText - '0')); */
				pText++;
			} else {
				break;
			}
		}
		if(bExpNeg) {
			i8Exp = 0 - i8Exp;
		}
	}

	/* 返回处理 */
	*ppText = pText;
	if(bFindData) {
		/* 数据合成 */
		float32 fData;
		if(bFindPoint) {
			i8Exp -= u8PointPlace;
		}
		if(abs(i8Exp) > 19) {	/* 避免cnst_fBcdQFactor溢出 */
			return FALSE;
		} else if(i8Exp >= 0) {
			fData = ((float32)u32Data)*cnst_fBcdQFactor[i8Exp];
		} else {
			i8Exp = 0 - i8Exp;
			fData = ((float32)u32Data)/cnst_fBcdQFactor[i8Exp];
		}
		if(bFindNeg) {
			fData = 0 - fData;
		}
		*pF32 = fData;
		
		return TRUE;
	} else {
		return FALSE;
	}
}

BOOL GetU32(uint8** ppText, uint8* pTextEnd, uint32* pU32, uint8 u8BcdQ)
{
	uint8* pText = *ppText;
	int32 i32BytesToLineEnd = pTextEnd - pText;
	uint32 u32Data = 0;
	uint8 u8PointPlace = 0;
	int8 i8Exp = 0;
	BOOL bFindData = FALSE;
	BOOL bFindPoint = FALSE;

	/* 数值部分 */
	for( ; i32BytesToLineEnd > 0; i32BytesToLineEnd--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u32Data = u32Data*10 + ((uint32)(*pText - '0'));
			if(bFindPoint) {
				u8PointPlace++;
			}
		} else if((*pText == '.') && (!bFindPoint) && (('0' <= *(pText+1)) && (*(pText+1) <= '9'))) {/* 小数点需要后面有数字 */
			bFindPoint = TRUE;
		} else if(bFindData) {
			break;
		}
		pText++;
	}

	/* 指数部分 */
	if((i32BytesToLineEnd > 1) && (*pText == 'e')) {
		pText++;
		BOOL bExpNeg = FALSE;
		if(*pText == '-') {
			bExpNeg = TRUE;
			pText++;
			i32BytesToLineEnd -= 2;
		} else {			
			i32BytesToLineEnd--;
		}
		for( ; i32BytesToLineEnd > 0; i32BytesToLineEnd--) {
			if(('0' <= *pText) && (*pText <= '9')) {
				i8Exp = i8Exp*10 + ((uint32)(*pText - '0'));
				pText++;
			} else {
				break;
			}
		}
		if(bExpNeg) {
			i8Exp = 0 - i8Exp;
		}
	}

	/* 返回处理 */
	*ppText = pText;
	if(bFindData) {
		/* 数据合成 */
		if(bFindPoint) {
			i8Exp -= u8PointPlace;
		}
		i8Exp += u8BcdQ;
		
		if(abs(i8Exp) > 9) {		/* 避免 cnst_u32BcdQFactor[] 溢出 */
			return FALSE;
		} else if(i8Exp > 0) {
			u32Data *= cnst_u32BcdQFactor[i8Exp];
		} else if(i8Exp < 0) {
			i8Exp = 0 - i8Exp;
			u32Data = (u32Data + (cnst_u32BcdQFactor[i8Exp]>>1))/cnst_u32BcdQFactor[i8Exp];
		}
		*pU32 = u32Data;

		return TRUE;
	} else {
		return FALSE;
	}
}

BOOL GetI32(uint8** ppText, uint8* pTextEnd, int32* pI32, uint8 u8BcdQ)
{
	uint8* pText = *ppText;
	int32 i32BytesToLineEnd = pTextEnd - pText;
	uint32 u32Data = 0;
	uint8 u8PointPlace = 0;
	int8 i8Exp = 0;
	BOOL bFindData = FALSE;
	BOOL bFindPoint = FALSE;
	BOOL bFindNeg = FALSE;

	/* 数值部分 */
	for( ; i32BytesToLineEnd > 0; i32BytesToLineEnd--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u32Data = u32Data*10 + ((uint32)(*pText - '0'));
			if(bFindPoint) {
				u8PointPlace++;
			}
		} else if((*pText == '-') && (!bFindNeg) && (!bFindData) && (('0' <= *(pText+1)) && (*(pText+1) <= '9'))) {/* 负号需要后面有数字 */
			bFindNeg = TRUE;
		} else if((*pText == '.') && (!bFindPoint) && (('0' <= *(pText+1)) && (*(pText+1) <= '9'))) {	/* 小数点需要后面有数字 */
			bFindPoint = TRUE;
		} else if(bFindData) {
			break;
		}
		pText++;
	}

	/* 指数部分 */
	if((i32BytesToLineEnd > 1) && (*pText == 'e')) {
		pText++;
		BOOL bExpNeg = FALSE;
		if(*pText == '-') {
			bExpNeg = TRUE;
			pText++;
			i32BytesToLineEnd -= 2;
		} else {			
			i32BytesToLineEnd--;
		}
		for( ; i32BytesToLineEnd > 0; i32BytesToLineEnd--) {
			if(('0' <= *pText) && (*pText <= '9')) {
				i8Exp = i8Exp*10 + ((uint32)(*pText - '0'));
				pText++;
			} else {
				break;
			}
		}
		if(bExpNeg) {
			i8Exp = 0 - i8Exp;
		}
	}

	/* 返回处理 */
	*ppText = pText;
	if(bFindData) {
		/* 数据合成 */
		if(bFindPoint) {
			i8Exp -= u8PointPlace;
		}
		i8Exp += u8BcdQ;
		if(abs(i8Exp) > 9) {		/* 避免 cnst_u32BcdQFactor[] 溢出 */
			return FALSE;
		} else if(i8Exp > 0) {
			u32Data *= cnst_u32BcdQFactor[i8Exp];
		} else if(i8Exp < 0) {
			i8Exp = 0 - i8Exp;
			u32Data = (u32Data + (cnst_u32BcdQFactor[i8Exp]>>1))/cnst_u32BcdQFactor[i8Exp];
		}
		
		if(bFindNeg) {	/* 负号处理 */
			*pI32 = 0 - u32Data;
		} else {
			*pI32 = u32Data;
		}
		return TRUE;
	} else {
		return FALSE;
	}
}

BOOL GetI64(uint8** ppText, uint8* pTextEnd, int64* pI64, uint8 u8BcdQ)
{
	uint8* pText = *ppText;
	int32 i32BytesToLineEnd = pTextEnd - pText;
	uint64 u64Data = 0;
	uint8 u8PointPlace = 0;
	int8 i8Exp = 0;
	BOOL bFindData = FALSE;
	BOOL bFindPoint = FALSE;
	BOOL bFindNeg = FALSE;

	/* 数值部分 */
	for( ; i32BytesToLineEnd > 0; i32BytesToLineEnd--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u64Data = u64Data*10 + ((uint64)(*pText - '0'));
			if(bFindPoint) {
				u8PointPlace++;
			}
		} else if((*pText == '-') && (!bFindNeg) && (!bFindData) && (('0' <= *(pText+1)) && (*(pText+1) <= '9'))) {/* 负号需要后面有数字 */
			bFindNeg = TRUE;
		} else if((*pText == '.') && (!bFindPoint) && (('0' <= *(pText+1)) && (*(pText+1) <= '9'))) {	/* 小数点需要后面有数字 */
			bFindPoint = TRUE;
		} else if(bFindData) {
			break;
		}
		pText++;
	}

	/* 指数部分 */
	if((i32BytesToLineEnd > 1) && (*pText == 'e')) {
		pText++;
		BOOL bExpNeg = FALSE;
		if(*pText == '-') {
			bExpNeg = TRUE;
			pText++;
			i32BytesToLineEnd -= 2;
		} else {			
			i32BytesToLineEnd--;
		}
		for( ; i32BytesToLineEnd > 0; i32BytesToLineEnd--) {
			if(('0' <= *pText) && (*pText <= '9')) {
				i8Exp = i8Exp*10 + ((uint32)(*pText - '0'));
				pText++;
			} else {
				break;
			}
		}
		if(bExpNeg) {
			i8Exp = 0 - i8Exp;
		}
	}

	/* 返回处理 */
	*ppText = pText;
	if(bFindData) {
		/* 数据合成 */
		if(bFindPoint) {
			i8Exp -= u8PointPlace;
		}
		i8Exp += u8BcdQ;
		if(abs(i8Exp) > 19) {		/* 避免cnst_u64BcdQFactor[]溢出 */
			return FALSE;
		} else if(i8Exp > 0) {
			u64Data *= cnst_u64BcdQFactor[i8Exp];
		} else if(i8Exp < 0) {
			i8Exp = 0 - i8Exp;
			u64Data = (u64Data + (cnst_u64BcdQFactor[i8Exp]>>1))/cnst_u64BcdQFactor[i8Exp];
		}
		Swi_disable();
		if(bFindNeg) {	/* 负号处理 */
			*pI64 = 0 - u64Data;
		} else {
			*pI64 = u64Data;
		}
		Swi_enable();

		return TRUE;
	} else {
		return FALSE;
	}
}

BOOL GetHexArray(uint8* pText, uint8* pTextEnd, uint8* pU8Hex, uint16 uDataLen)
{
	if(pTextEnd - pText < uDataLen*2) {		/* 长度不够长 */
		return FALSE;
	}
	
	/* 开始转换 */
	uint8 u8Hex;
	for( ; uDataLen > 0; uDataLen--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			u8Hex = (*pText - '0')<<4;
		} else if(('A' <= *pText) && (*pText <= 'F')) {
			u8Hex = (*pText - 'A' + 10)<<4;
		} else if(('a' <= *pText) && (*pText <= 'f')) {
			u8Hex = (*pText - 'a' + 10)<<4;
		} else {
			return FALSE;
		}
		pText++;
		
		if(('0' <= *pText) && (*pText <= '9')) {
			u8Hex += (*pText - '0');
		} else if(('A' <= *pText) && (*pText <= 'F')) {
			u8Hex += (*pText - 'A' + 10);
		} else if(('a' <= *pText) && (*pText <= 'f')) {
			u8Hex += (*pText - 'a' + 10);
		} else {
			return FALSE;
		}
		pText++;
		*pU8Hex++ = u8Hex;
	}
	return TRUE;
}

/*==========================================================================
| Description	: 把文本转化为IPv4地址
| G/Out var		:
| Author		: Wang Renfei			Date	: 2017-03-21
\=========================================================================*/
BOOL GetIPv4(uint8** ppText, uint8* pTextEnd, uint32* pIPv4)
{
	uint8* pText = *ppText;
	uint32 u32Data = 0;
	BOOL bFindData = FALSE;
	int8 i;
	for(i = 3; i >= 0; i--) {
		uint8 u8Seg = 0;
		bFindData = FALSE;
		for( ; pText < pTextEnd; pText++) {
			if(('0' <= *pText) && (*pText <= '9')) {
				bFindData = TRUE;
				u8Seg = u8Seg*10 + ((uint8)(*pText - '0'));
			} else if(bFindData) {
				pText++;
				break;
			}
		}
		u32Data += cnst_u32IPv4Factor[i]*u8Seg;
	}

	*ppText = pText - 1;	/* 修正到最后一个非0~9数字位置 */
	if(bFindData && (i < 0)) {
		*pIPv4 = u32Data;
		return TRUE;
	} else {
		return FALSE;
	}
}

/* 获取pvTIME，一次获得两个数据 */
BOOL GetPriceTime(uint8** ppText, uint8* pTextEnd, uint32* pPVTime)
{
	uint8* pText = *ppText;
	uint32 u32Data = 0;
	uint32 u32Data_WithSymbol = 0;
	uint32 u32Price = 0;
	BOOL bFindData = FALSE;
	for( ; pText < pTextEnd; pText++) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u32Data = u32Data*10 + (*pText - '0');
		} else if(bFindData) {
			if(*pText == '#') {	/* 这个是价位部分 */
				u32Price = u32Data*10000;
				u32Data = 0;
			} else if((*pText == ':') || (*pText == '.')) {	/* 输入可能为 a.b.c 或 a:b:c */
				u32Data_WithSymbol = (u32Data_WithSymbol + u32Data)*100;
				u32Data = 0;
			} else {
				break;
			}
		}
	}

	/* 返回处理 */
	*ppText = pText;	/* 修正到最后一个非0~9数字位置 */
	if(bFindData) {
		*pPVTime = (u32Data + u32Data_WithSymbol + u32Price)%100000;	/* 限幅在9#99:99之内，要不然显示之类的会出错 */
		return TRUE;
	} else {
		return FALSE;
	}
}

/* UataUser == DATA_USER_NET, 把形如 20020000 00:00:00.000 转化成 REAL_TIME_VAR
   UataUser == 其他, 把形如 20/06/11 00:00:00.000 转化成 REAL_TIME_VAR */
BOOL GetT64(DATA_USER DataUser, uint8** ppText, uint8* pTextEnd, REAL_TIME_VAR* pRealTime)
{
	uint8* pText = *ppText;

	if(DataUser == DATA_USER_NET) {
		/* 年 */
		pText += 2;
		if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
			pRealTime->u8Year = (pText[0] - '0')*10 + pText[1] - '0';
		} else {
			return FALSE;
		}

		/* 月*/
		pText += 2;
		if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
			pRealTime->u8Month = (pText[0] - '0')*10 + pText[1] - '0';
		} else {
			return FALSE;
		}
		
		/* 日 */
		pText += 2;
		if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
			pRealTime->u8Day = (pText[0] - '0')*10 + pText[1] - '0';
		} else {
			return FALSE;
		}
	} else {
		/* 年 */
		if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
			pRealTime->u8Year = (pText[0] - '0')*10 + pText[1] - '0';
		} else {
			return FALSE;
		}
		
		/* 月*/
		pText += 3;
		if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
			pRealTime->u8Month = (pText[0] - '0')*10 + pText[1] - '0';
		} else {
			return FALSE;
		}
		
		/* 日 */
		pText += 3;
		if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
			pRealTime->u8Day = (pText[0] - '0')*10 + pText[1] - '0';
		} else {
			return FALSE;
		}
	}
	
	/* 时 */
	pText += 3;
	if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
		pRealTime->u8Hour = (pText[0] - '0')*10 + pText[1] - '0';
	} else {
		return FALSE;
	}
	
	/* 分 */
	pText += 3;
	if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
		pRealTime->u8Minute = (pText[0] - '0')*10 + pText[1] - '0';
	} else {
		return FALSE;
	}
	
	/* 秒 */
	pText += 3;
	if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
		pRealTime->u8Second = (pText[0] - '0')*10 + pText[1] - '0';
	} else {
		return FALSE;
	}
	
	/* 毫秒 */
	pText += 3;
	if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9') && ('0' <= pText[2]) && (pText[2] <= '9')) {
		pRealTime->uMilliSec = (pText[0] - '0')*100 + (pText[1] - '0')*10 + (pText[2] - '0');
	} else {
		return FALSE;
	}

	*ppText = pText + 3;
	return TRUE;
}

/* 把形如 20010101 00:00:00 转化成 REAL_TIME_VAR */
BOOL GetT32(uint8** ppText, uint8* pTextEnd, uint32* u32Seconds)
{
	uint8* pText = *ppText;
	uint8 u8Year, u8Month, u8Day, u8Hour, u8Minute, u8Second;

	/* 年 */
	pText += 2;
	if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
		u8Year = (pText[0] - '0')*10 + pText[1] - '0';
	} else {
		return FALSE;
	}

	/* 月*/
	pText += 2;
	if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
		u8Month = (pText[0] - '0')*10 + pText[1] - '0';
	} else {
		return FALSE;
	}
	
	/* 日 */
	pText += 2;
	if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
		u8Day = (pText[0] - '0')*10 + pText[1] - '0';
	} else {
		return FALSE;
	}
	
	/* 时 */
	pText += 3;
	if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
		u8Hour = (pText[0] - '0')*10 + pText[1] - '0';
	} else {
		return FALSE;
	}
	
	/* 分 */
	pText += 3;
	if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
		u8Minute = (pText[0] - '0')*10 + pText[1] - '0';
	} else {
		return FALSE;
	}
	
	/* 秒 */
	pText += 3;
	if(('0' <= pText[0]) && (pText[0] <= '9') && ('0' <= pText[1]) && (pText[1] <= '9')) {
		u8Second = (pText[0] - '0')*10 + pText[1] - '0';
	} else {
		return FALSE;
	}

	*ppText = pText + 2;
	*u32Seconds = CalRTCSecondsByDate(u8Year, u8Month, u8Day, u8Hour, u8Minute, u8Second);
	return TRUE;
}

uint32 ReadU32(uint8** ppText, uint8* pTextEnd)
{
	uint8* pText = *ppText;
	int32 i32BytesToLineEnd = pTextEnd - pText;
	uint32 u32Data = 0;
	BOOL bFindData = FALSE;

	for( ; i32BytesToLineEnd > 0; i32BytesToLineEnd--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u32Data = u32Data*10 + ((uint32)(*pText - '0'));
		} else if(bFindData) {
			break;
		}
		pText++;
	}

	*ppText = pText;
	return u32Data;
}

uint32 ReadH32(uint8** ppText, uint8* pTextEnd)
{
	uint8* pText = *ppText;
	int8 i8BytesToLineEnd = pTextEnd - pText;
	uint32 u32Hex = 0;
	BOOL bFindData = FALSE;

	for( ; i8BytesToLineEnd > 0; i8BytesToLineEnd--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			u32Hex = u32Hex*0x10 + (*pText - '0');
			bFindData = TRUE;
		} else if(('A' <= *pText) && (*pText <= 'F')) {
			u32Hex = u32Hex*0x10 + (*pText - 'A' + 10);
			bFindData = TRUE;
		} else if(('a' <= *pText) && (*pText <= 'f')) {
			u32Hex = u32Hex*0x10 + (*pText - 'a' + 10);
			bFindData = TRUE;
		} else if(bFindData) {
			break;
		}
		pText++;
	}

	*ppText = pText;
	return u32Hex;
}


uint64 ReadH64(uint8** ppText, uint8* pTextEnd)
{
	uint8* pText = *ppText;
	int8 i8BytesToLineEnd = pTextEnd - pText;
	BOOL bFindData = FALSE;
	uint64 u64Hex = 0;

	for( ; i8BytesToLineEnd > 0; i8BytesToLineEnd--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			u64Hex = u64Hex*0x10 + (*pText - '0');
			bFindData = TRUE;
		} else if(('A' <= *pText) && (*pText <= 'F')) {
			u64Hex = u64Hex*0x10 + (*pText - 'A' + 10);
			bFindData = TRUE;
		} else if(('a' <= *pText) && (*pText <= 'f')) {
			u64Hex = u64Hex*0x10 + (*pText - 'a' + 10);
			bFindData = TRUE;
		} else if(bFindData) {
			break;
		}
		pText++;
	}

	*ppText = pText;
	return u64Hex;
}

int16 ReadI16(uint8** ppText, uint8* pTextEnd)
{
	uint8* pText = *ppText;
	int32 i32BytesToLineEnd = pTextEnd - pText;
	int16 i16Data = 0;
	BOOL bFindData = FALSE;
	BOOL bFindNeg = FALSE;

	for( ; i32BytesToLineEnd > 0; i32BytesToLineEnd--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			i16Data = i16Data*10 + ((uint32)(*pText - '0'));
		} else if((*pText == '-') && (!bFindNeg) && (!bFindData) && (('0' <= *(pText+1)) && (*(pText+1) <= '9'))) {/* 负号需要后面有数字 */
			bFindNeg = TRUE;
		} else if(bFindData) {
			break;
		}
		pText++;
	}
	if(bFindNeg) {
		i16Data = 0 - i16Data;
	}

	*ppText = pText;
	return i16Data;
}

uint32 ReadH32FixLen(uint8* pText)
{
	uint32 u32Hex = 0;
	int8 i8BytesToLineEnd;

	for(i8BytesToLineEnd = 8; i8BytesToLineEnd > 0; i8BytesToLineEnd--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			u32Hex = (u32Hex<<4) + (uint32)(*pText - '0');
		} else if(('A' <= *pText) && (*pText <= 'F')) {
			u32Hex = (u32Hex<<4) + (uint32)(*pText - 'A' + 10);
		} else if(('a' <= *pText) && (*pText <= 'f')) {
			u32Hex = (u32Hex<<4) + (uint32)(*pText - 'a' + 10);
		} else {
			u32Hex = (u32Hex<<4);
		}
		pText++;
	}
	return u32Hex;
}

uint32 ReadU32FixLen(uint8* pText)
{
	uint32 u32Data = 0;
	BOOL bFindData = FALSE;
	int8 i8BytesToConv;
	
	for(i8BytesToConv = 10; i8BytesToConv > 0; i8BytesToConv--) {
		if(('0' <= *pText) && (*pText <= '9')) {
			bFindData = TRUE;
			u32Data = u32Data*10UL + ((uint32)(*pText - '0'));
		} else if(bFindData) {
			break;
		}
		pText++;
	}

	return u32Data;
}

/*==========================================================================
| Description	: 把转化成字符串、右对齐的U32/U64数据，加上小数点，进行左对齐
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-6-9
\=========================================================================*/
void LeftAlignStringAndAddPoint(uint8** ppText, uint8* pTextEnd, uint16 uStringOff, uint16 uBcdQ)
{
	uint8* pText;
	uint16 uStringLen;				/* 总的占用的字符数量 */
	int16 i;

	pText = *ppText;
	uStringLen = pTextEnd - pText - uStringOff;
	if(uBcdQ == 0) {
		for(i = uStringLen; i > 0; i--) {
			*pText = *(pText + uStringOff);
			pText++;
		}
	} else if(uStringLen <= uBcdQ) {
		*pText++ = '0';
		*pText++ = '.';
		for(i = uBcdQ - uStringLen; i > 0; i--) {
			*pText++ = '0';
		}
		if(uStringOff > uBcdQ - uStringLen + 2) {
			uStringOff -= uBcdQ - uStringLen + 2;
			for(i = uStringLen; i > 0; i--) {
				*pText = *(pText + uStringOff);
				pText++;
			}
		}
	} else {
		for(i = uStringLen - uBcdQ; i > 0; i--) {
			*pText = *(pText + uStringOff);
			pText++;
		}
		*pText++ = '.';
		if(uStringOff > 1) {
			uStringOff -= 1;
			for(i = uBcdQ; i > 0; i--) {
				*pText = *(pText + uStringOff);
				pText++;
			}
		}
	}
	
	*ppText = pText;
}

/*==========================================================================
| Description	: 数值转化成JSON格式，即形成 "var_name":"var_value",格式
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2017-9-9
\=========================================================================*/
void PrintStringToJson(uint8** ppJson, const char* pVarName, const char* pString)
{
	uint8* pJson = *ppJson;
	/* 变量名部分 */
	*pJson++ = '"';
	while(*pVarName) {
		*pJson++ = *pVarName++;
	}
	*pJson++ = '"';
	*pJson++ = ':';
	*pJson++ = '"';
	while(*pString) {
		*pJson++ = *pString++;
	}
	*pJson++ = '"';
	*pJson++ = ',';
	
	*ppJson = pJson;	/* 修正指针 */
}

/* 显示格式：  "var_name":"string:ddd" */
void PrintStringAndU32DataToJson(uint8** ppJson, const char* pVarName, const char* pString, uint32 u32Data)
{
	uint8* pJson = *ppJson;
	
	/* 变量名部分 */
	*pJson++ = '"';
	while(*pVarName) {
		*pJson++ = *pVarName++;
	}
	*pJson++ = '"';
	*pJson++ = ':';
	*pJson++ = '"';
	
	/* 字符部分 */
	while(*pString) {
		*pJson++ = *pString++;
	}
	*pJson++ = ':';
	
	/* 数值部分 */
	if(u32Data == 0) {
		*pJson++ = '0';
	} else {
		int16 i;
		for(i = 9; (i >= 0) && (u32Data/cnst_u32BcdQFactor[i] == 0); i--) {
		}
		for( ; i >= 0; i--) {
			*pJson++ = '0' + u32Data/cnst_u32BcdQFactor[i];
			u32Data = u32Data%cnst_u32BcdQFactor[i];
		}
	}
	*pJson++ = '"';
	*pJson++ = ',';
	
	*ppJson = pJson;	/* 修正指针 */
}

void PrintHexToJson(uint8** ppJson, const char* pVarName, uint8* pU8Hex, uint16 uByteLen)
{
	uint8* pJson = *ppJson;
	uint8 u8Data;

	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}
	
	/* 变量数值部分 */
	*pJson++ = '"';
	for( ; uByteLen > 0; uByteLen--) {
		u8Data = ((*pU8Hex)>>4);
		if(u8Data < 10) {
			*pJson = u8Data + '0';
		} else {
			*pJson = u8Data + 'A' - 10;
		}
		pJson++;

		u8Data = *pU8Hex & 0x0F;
		if(u8Data < 10) {
			*pJson = u8Data + '0';
		} else {
			*pJson = u8Data + 'A' - 10;
		}
		pJson++;
		
		pU8Hex++;
	}
	*pJson++ = '"';
	
	*ppJson = pJson;	/* 修正指针 */
}

void PrintBase64ToJson(uint8** ppJson, const char* pVarName, uint8* pU8Dat, uint16 uByteLen)
{
	uint8* pJson = *ppJson;

	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}
	
	/* 变量数值部分 */
	*pJson++ = '"';
	pJson = EncodeByBase64(pU8Dat, uByteLen, pJson);
	*pJson++ = '"';
	
	*ppJson = pJson;	/* 修正指针 */
}

void PrintU32DatToJson(uint8** ppJson, const char* pVarName, uint32 u32Data, uint16 uBcdQ)
{
	uint8* pJson = *ppJson;
	
	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}

	/* 变量数值部分 */
	*pJson++ = '"';
	int8 i8StrOff = 12;		/* uint32最长10个字符+小数点+可能最前面的0 */
	do {				/* 先以最大长度打印 */
		i8StrOff--;
		*(pJson + i8StrOff) = u32Data%10 + '0';
		u32Data = u32Data/10;
	} while(u32Data && (i8StrOff > 0));
	LeftAlignStringAndAddPoint(&pJson, pJson+12, i8StrOff, uBcdQ);	/* 整体左移 */
	*pJson++ = '"';
	*pJson++ = ',';

	*ppJson = pJson;	/* 修正指针 */
}

void PrintU32ArrayToJson(uint8** ppJson, const char* pVarName, uint32* pU32Data, uint16 uDataNum)
{
	uint8* pJson = *ppJson;
	
	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}

	/* 变量数值部分 */
	*pJson++ = '[';
	for( ; uDataNum > 0; uDataNum--) {
		int8 i8StrOff = 12;		/* uint32最长10个字符+小数点+可能最前面的0 */
		uint32 u32Data = *pU32Data++;
		do {				/* 先以最大长度打印 */
			i8StrOff--;
			*(pJson + i8StrOff) = u32Data%10 + '0';
			u32Data = u32Data/10;
		} while(u32Data && (i8StrOff > 0));
		LeftAlignStringAndAddPoint(&pJson, pJson+12, i8StrOff, 0);	/* 整体左移 */
		*pJson++ = ',';
	}

	pJson--;
	*pJson++ = ']';
	*pJson++ = ',';
	*ppJson = pJson;	/* 修正指针 */
}

/* 把32位二进制数据以字符串形式显示,每一个位一个字符. 如果该位为TRUE,显示为特征字符; 如果该位FALSE,则显示为'-' */
void PrintU32AsBinaryToJson(uint8** ppJson, const char* pVarName, const char* pBinaryDispChar, uint32 u32Data)
{
	uint8* pJson = *ppJson;
	
	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}

	/* 变量数值部分 */
	*pJson++ = '"';
	int16 i;
	for(i = 0; i < 32; i++) {
		if(u32Data & (1UL<<i)) {
			*pJson++ = pBinaryDispChar[i];
		} else {
			*pJson++ = '-';
		}
	}
	*pJson++ = '"';
	*pJson++ = ',';

	*ppJson = pJson;	/* 修正指针 */
}

void PrintF32DatToJson(uint8** ppJson, const char* pVarName, float32 fData, int8 i8SgnDigits_pMax_nExpect)
{
	uint8* pJson = *ppJson;
	
	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}

	/* 变量数值部分 */
	*pJson++ = '"';
	PrintF32(&pJson, pJson + abs(i8SgnDigits_pMax_nExpect) + 6, fData, i8SgnDigits_pMax_nExpect);	/* 浮点数表达如-nn.nnne-mm 最多增加6位 */
	*pJson++ = '"';
	*pJson++ = ',';

	*ppJson = pJson;	/* 修正指针 */
}

void PrintFM32DatToJson(uint8** ppJson, const char* pVarName, float32 fData, F32_MEANING F32Meaning)
{
	uint8* pJson = *ppJson;
	
	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}

	/* 变量数值部分 */
	*pJson++ = '"';
	PrintFM32(&pJson, pJson + 20, fData, F32Meaning);
	*pJson++ = '"';
	*pJson++ = ',';

	*ppJson = pJson;	/* 修正指针 */
}


void PrintF32WithStatusToJson(uint8** ppJson, const char* pVarName, float32 fData, int8 i8SgnDigits_pMax_nExpect, BOOL bStatus)
{
	uint8* pJson = *ppJson;

	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}
	*pJson++ = '{';

	/* 变量数值部分 */
	pVarName = "data";
	*pJson++ = '"';
	while(*pVarName) {
		*pJson++ = *pVarName++;
	}
	*pJson++ = '"';
	*pJson++ = ':';
	*pJson++ = '"';
	PrintF32(&pJson, pJson + abs(i8SgnDigits_pMax_nExpect) + 6, fData, i8SgnDigits_pMax_nExpect);	/* 浮点数表达如-nn.nnne-mm 最多增加6位 */
	*pJson++ = '"';
	*pJson++ = ',';	

	/* 变量状态部分 */
	*pJson++ = '"';
	*pJson++ = 's';
	*pJson++ = 't';
	*pJson++ = '"';
	*pJson++ = ':';
	*pJson++ = '"';
	if(bStatus) {
		*pJson++ = 's';
		*pJson++ = 'u';
		*pJson++ = 'c';
	} else {
		*pJson++ = 'f';
		*pJson++ = 'a';
		*pJson++ = 'i';
		*pJson++ = 'l';
	}
	*pJson++ = '"';

	*pJson++ = '}';		/* 和前面的{相对应 */
	*pJson++ = ',';	
	*ppJson = pJson;	/* 修正指针 */
}

void PrintF32ArrayToJson(uint8** ppJson, const char* pVarName, float32* pfData, int8 i8SgnDigits_pMax_nExpect, uint16 uDataNum)
{
	uint8* pJson = *ppJson;
	
	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}

	/* 变量数值部分 */
	*pJson++ = '[';
	for( ; uDataNum > 0; uDataNum--) {
		PrintF32(&pJson, pJson + abs(i8SgnDigits_pMax_nExpect) + 6, *pfData, i8SgnDigits_pMax_nExpect);	/* 浮点数表达如-nn.nnne-mm 最多增加6位 */
		pfData++;
		*pJson++ = ',';
	}

	pJson--;
	*pJson++ = ']';
	*pJson++ = ',';
	*ppJson = pJson;	/* 修正指针 */
}

void PrintH64DatToJson(uint8** ppJson, const char* pVarName, uint64 u64Data, uint8 u8HexNum)
{
	uint8* pJson = *ppJson;
	if(u8HexNum > 16) {
		u8HexNum = 16;
	}
	
	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}
	
	/* 变量数值部分 */
	*pJson++ = '"';
	int8 i;
	for(i = u8HexNum-1; i >= 0; i--) {
		uint8 u8Data = u64Data&0x0F;
		if(u8Data < 10) {
			*(pJson + i) = u8Data + '0';
		} else {
			*(pJson + i) = u8Data + 'A' - 10;
		}
		u64Data = (u64Data>>4);
	}
	pJson += u8HexNum;
	*pJson++ = '"';
	*pJson++ = ',';
	
	*ppJson = pJson;	/* 修正指针 */
}

void PrintH32DatToJson(uint8** ppJson, const char* pVarName, uint32 u32Data)
{
	uint8* pJson = *ppJson;
	uint8 u8Data;
	int16 i;
	
	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}
	
	/* 变量数值部分 */
	*pJson++ = '"';
	for(i = 7; i >= 0; i--) {
		u8Data = u32Data&0x0F;
		if(u8Data < 10) {
			*(pJson + i) = u8Data + '0';
		} else {
			*(pJson + i) = u8Data + 'A' - 10;
		}
		u32Data = (u32Data>>4);
	}
	pJson += 8;
	*pJson++ = '"';
	*pJson++ = ',';
	
	*ppJson = pJson;	/* 修正指针 */
}

void PrintH16DatToJson(uint8** ppJson, const char* pVarName, uint16 uData)
{
	uint8* pJson = *ppJson;
	uint8 u8Data;
	int16 i;
	
	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}
	
	/* 变量数值部分 */
	*pJson++ = '"';
	for(i = 3; i >= 0; i--) {
		u8Data = uData&0x0F;
		if(u8Data < 10) {
			*(pJson + i) = u8Data + '0';
		} else {
			*(pJson + i) = u8Data + 'A' - 10;
		}
		uData = (uData>>4);
	}
	pJson += 4;
	*pJson++ = '"';
	*pJson++ = ',';
	
	*ppJson = pJson;	/* 修正指针 */
}

void PrintI32DatToJson(uint8** ppJson, const char* pVarName, int32 i32Data, uint16 uBcdQ)
{
	uint8* pJson = *ppJson;
	
	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}

	/* 变量数值部分 */
	*pJson++ = '"';
	if(i32Data < 0) { 	/* 符号 */
		*pJson++ = '-';
		i32Data = 0 - i32Data;
	}
	int8 i8StrOff = 12;		/* int32最长10个字符+小数点+可能最前面的0 */
	do {				/* 先以最大长度打印 */
		i8StrOff--;
		*(pJson + i8StrOff) = i32Data%10 + '0';
		i32Data = i32Data/10;
	} while(i32Data && (i8StrOff > 0));
	LeftAlignStringAndAddPoint(&pJson, pJson+12, i8StrOff, uBcdQ);	/* 整体左移 */
	*pJson++ = '"';
	*pJson++ = ',';

	*ppJson = pJson;	/* 修正指针 */
}

void PrintI32ArrayToJson(uint8** ppJson, const char* pVarName, int32* pI32Data, uint16 uDataNum)
{
	uint8* pJson = *ppJson;
	
	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}

	/* 变量数值部分 */
	*pJson++ = '[';
	for( ; uDataNum > 0; uDataNum--) {
		int32 i32Data = *pI32Data++;
		if(i32Data < 0) { 	/* 符号 */
			*pJson++ = '-';
			i32Data = 0 - i32Data;
		}
		int8 i8StrOff = 12; 	/* int32最长10个字符+小数点+可能最前面的0 */
		do {					/* 先以最大长度打印 */
			i8StrOff--;
			*(pJson + i8StrOff) = i32Data%10 + '0';
			i32Data = i32Data/10;
		} while(i32Data && (i8StrOff > 0));
		LeftAlignStringAndAddPoint(&pJson, pJson+12, i8StrOff, 0);	/* 整体左移 */
		*pJson++ = ',';
	}

	pJson--;
	*pJson++ = ']';
	*pJson++ = ',';
	*ppJson = pJson;	/* 修正指针 */
}

void PrintI64DatToJson(uint8** ppJson, const char* pVarName, int64 i64Data, uint16 uBcdQ)
{
	uint8* pJson = *ppJson;
	
	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
		*pJson++ = ':';
	}

	/* 变量数值部分 */
	*pJson++ = '"';
	if(i64Data < 0) { 	/* 符号 */
		*pJson++ = '-';
		i64Data = 0 - i64Data;
	}
	int8 i8StrOff = 22;		/* int64最长22：19个数字+小数点+可能最前面的0+前面可能的负号 */
	do {				/* 先以最大长度打印 */
		i8StrOff--;
		*(pJson + i8StrOff) = i64Data%10 + '0';
		i64Data = i64Data/10;
	} while(i64Data && (i8StrOff > 0));
	LeftAlignStringAndAddPoint(&pJson, pJson+22, i8StrOff, uBcdQ);	/* 整体左移 */
	*pJson++ = '"';
	*pJson++ = ',';

	*ppJson = pJson;	/* 修正指针 */
}

void PrintIPv4ToJson(uint8** ppJson, const char* pVarName, int32 u32IPv4)
{
	uint8* pJson = *ppJson;
	
	/* 变量名部分 */
	*pJson++ = '"';
	while(*pVarName) {
		*pJson++ = *pVarName++;
	}
	*pJson++ = '"';
	*pJson++ = ':';
	
	/* 变量值部分 */
	*pJson++ = '"';
	PrintIPv4(&pJson, pJson+16, u32IPv4);
	*pJson++ = '"';
	*pJson++ = ',';

	*ppJson = pJson;	/* 修正指针 */
}

void PrintT32ToJson(uint8** ppJson, const char* pVarName, uint32 u32Seconds)
{
	uint8* pJson = *ppJson;
	
	/* 变量名部分 */
	*pJson++ = '"';
	while(*pVarName) {
		*pJson++ = *pVarName++;
	}
	*pJson++ = '"';
	*pJson++ = ':';
	
	/* 变量值部分 */
	*pJson++ = '"';
	PrintT32(DATA_USER_NET, &pJson, pJson+17, u32Seconds);
	*pJson++ = '"';
	*pJson++ = ',';

	*ppJson = pJson;	/* 修正指针 */
}

void PrintEnumContToJson(uint8** ppJson, const char* pVarName, uint32 u32Data, uint16 uBcdQ)
{
	uint8* pJson = *ppJson;
	
	/* 变量数值部分:枚举值最多两位数 */
	*pJson++ = '"';
	if(u32Data/10) {
		*pJson++ = '0' + u32Data/10;
	}
	*pJson++ = '0' + u32Data%10;
	*pJson++ = '"';

	/* 变量名部分 */
	if(pVarName) {
		*pJson++ = ':';
		*pJson++ = '"';
		while(*pVarName) {
			*pJson++ = *pVarName++;
		}
		*pJson++ = '"';
	}
	*pJson++ = ',';

	*ppJson = pJson;	/* 修正指针 */
}

void PrintSoftVerToJson(uint8** ppJson, const char* pVarName, uint32 u32Data)
{
	uint8* pJson = *ppJson;
	
	/* 变量名部分 */
	*pJson++ = '"';
	while(*pVarName) {
		*pJson++ = *pVarName++;
	}
	*pJson++ = '"';
	*pJson++ = ':';

	/* 变量数值部分 */
	*pJson++ = '"';
	*pJson++ = u32Data/1000 + '0';
	u32Data = u32Data%1000;
	*pJson++ = '.';
	*pJson++ = u32Data/100 + '0';
	u32Data = u32Data%100;
	*pJson++ = '.';
	*pJson++ = u32Data/10 + '0';
	u32Data = u32Data%10;
	*pJson++ = u32Data + '0';
	*pJson++ = '"';
	*pJson++ = ',';

	*ppJson = pJson;	/* 修正指针 */
}

void PrintBoolToJson(uint8** ppJson, const char* pVarName, BOOL bData)
{
	uint8* pJson = *ppJson;

	/* 变量名部分 */
	*pJson++ = '"';
	while(*pVarName) {
		*pJson++ = *pVarName++;
	}
	*pJson++ = '"';
	*pJson++ = ':';
	
	/* 变量数值部分 */
	*pJson++ = '"';
	if(bData) {
		pVarName = "true";
	} else {
		pVarName = "false";
	}
	while(*pVarName) {
		*pJson++ = *pVarName++;
	}
	*pJson++ = '"';
	*pJson++ = ',';
	
	*ppJson = pJson;	/* 修正指针 */
}

void PrintTempToJson(uint8** ppJson, LANG_TYPE Language)
{
	/* 开始打印 */
	uint8* pJson = *ppJson;
	PrintStringNoOvChk(&pJson, ",\"temperature\":{");

#if MAX_TEMP_SEN_NUM
	TEMPERATURE_DATA* pTemperature = g_AnaRes.Temperature;
	BOOL bHaveTempSensor = FALSE;				/* 用以区分是否有传感器 */
	int8 i;
	for(i = 0; i < MAX_TEMP_SEN_NUM; i++) {
		int8 i8Placement = pTemperature->i8Placement;
		if(i8Placement != TEMP_SEN_NUL) {
			/* 传感器断线判断 */
			BOOL bTempSenBreak = (i8Placement < 0);	/* 该传感器是否断线 */
			if(bTempSenBreak) {
				i8Placement = 0 - i8Placement;
			}
			if(i8Placement >= TEMP_SEN_PLC_NUM) {	/* 防止后续访问溢出 */
				i8Placement = TEMP_SEN_PLC_ANY;
			}
			
			/* 变量名部分 */
			const char* pVarName = cnst_TempPlacementName[i8Placement][Language];
			*pJson++ = '"';
			while(*pVarName) {
				*pJson++ = *pVarName++;
			}
			uint8 u8PlaceNo = pTemperature->u8PlaceNo;
			if(u8PlaceNo) {			/* 同一个位置有多个传感器，需要加上序号 */
				if(u8PlaceNo >= 10) {
					*pJson++ = '0' + u8PlaceNo/10;
					u8PlaceNo = u8PlaceNo%10;
				}
				*pJson++ = '0' + u8PlaceNo;
			}
			*pJson++ = '"';
			*pJson++ = ':';

			/* 变量数值部分 */
			*pJson++ = '"';
			if(bTempSenBreak) {		/* 打印"断线" */
				pVarName = cnst_TempPlacementName[TEMP_SEN_BREAK_NAME][Language];
				while(*pVarName) {
					*pJson++ = *pVarName++;
				}
			} else {
				PrintI16(&pJson, pJson+10, pTemperature->iDegree_100mC, 1);
			}
			*pJson++ = '"';
			*pJson++ = ',';

			/* 标记 */
			bHaveTempSensor = TRUE;
		}
		pTemperature++;			/* 指向下一条数据 */
	}
	if(bHaveTempSensor) {
		pJson--;					/* 前面多了一个',' */
	}
#endif

	*pJson++ = '}';				/* 和 temperature:{ 配对 */
	*ppJson = pJson;
}

/*==========================================================================
| Description	: MQTT消息解析函数
	SkipCharInString():在消息字符串中跳过特征字符,返回特征字符后面位置
	CompareMsg()	:消息中变量名称查找
	CompareTopic()	:标题中变量名称查找
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2017-9-9
\=========================================================================*/
uint8* SkipCharInString(uint8* pU8Msg, uint8* pU8MsgEnd, uint8 u8Char, uint16 uCharNum)
{
	int32 i = pU8MsgEnd - pU8Msg;
	
	for( ; (i > 0) && uCharNum; i--) {
		if(*pU8Msg++ == u8Char) {
			uCharNum--;
		}
	}
	
	return pU8Msg;
}

BOOL CompareTopic(uint8* pU8TopicBuf, const char* pU8NameString)
{
	if(*pU8NameString == 0) {	/* 字符串是空 */
		return FALSE;
	}
	
    for(;;) {
    	if((*pU8NameString == 0) || (*pU8NameString == '#')) {
			return TRUE;
    	} else if(*pU8TopicBuf++ == *pU8NameString++) {
		} else {
			return FALSE;
    	}
    }
}

BOOL CompareMsg(uint8* pU8MsgBuf, const char* pU8NameString)
{
	while(*pU8MsgBuf == *pU8NameString) {
		pU8MsgBuf++;
		pU8NameString++;
	}

	/* 不相等的地方查看原因 */
	if((*pU8MsgBuf == '"') && (*pU8NameString == 0)) {
		return TRUE;
	} else {
		return FALSE;
	}
}

BOOL CompareInstr(uint8** ppU8Buf, uint8* pU8BufEnd, const char* pU8Instr)
{
	uint8* pU8Buf = *ppU8Buf;
	int16 i = pU8BufEnd - pU8Buf;

	for(; (i > 0) && (*pU8Buf == *pU8Instr) && *pU8Instr; i--) {
		pU8Buf++;
		pU8Instr++;
	}
	
	/* 不相等的地方查看原因 */
	if(((i == 0) || (*pU8Buf == '"') || (*pU8Buf == 0)) && (*pU8Instr == 0)) {	/* Buf比较完，或者遇到Json字符串结束，或者String结束 */
		*ppU8Buf = pU8Buf;
		return TRUE;
	} else {
		return FALSE;
	}
}

/*==========================================================================
| Description	: 计算16位数中，"1"的个数
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2007-1-12
\=========================================================================*/
#define POW2(c) (1<<(c))
#define MASK(c) (((uint16)-1) / (POW2(POW2(c)) + 1))
#define ROUND(n, c) (((n) & MASK(c)) + ((n) >> POW2(c) & MASK(c)))
uint16 Cal1InBits16(uint16 Bits16)
{
	Bits16 = ROUND(Bits16, 0);
	Bits16 = ROUND(Bits16, 1);
	Bits16 = ROUND(Bits16, 2);
	Bits16 = ROUND(Bits16, 3);
	return Bits16;
}

uint32 F32ToU32(float32 f32Dat)
{
	if(f32Dat >= 0xFFFFFFFF) {
		return 0xFFFFFFFF;
	} else if(f32Dat < 0) {
		return 0;
	} else {
		return (uint32)(f32Dat + 0.5f);
	}
}

int32 F32ToI32(float32 f32Dat)
{
	if(f32Dat >= 0x7FFFFFFF) {
		return 0x7FFFFFFF;
	} else if(f32Dat < -0x7FFFFFFF) {
		return -0x7FFFFFFF;
	} else if(f32Dat > 0) {
		return (int32)(f32Dat + 0.5f);
	} else if(f32Dat < 0) {
		return (int32)(f32Dat - 0.5f);
	} else {
		return 0;
	}
}

uint16 F32ToU16(float32 f32Dat)
{
	if(f32Dat >= 0xFFFF) {
		return 0xFFFF;
	} else if(f32Dat < 0) {
		return 0;
	} else {
		return (uint16)(f32Dat + 0.5f);
	}
}

int16 F32ToI16(float32 f32Dat)
{
	if(f32Dat >= 0x7FFF) {
		return 0x7FFF;
	} else if(f32Dat < -0x7FFF) {
		return -0x7FFF;
	} else if(f32Dat > 0) {
		return (int16)(f32Dat + 0.5f);
	} else if(f32Dat < 0) {
		return (int16)(f32Dat - 0.5f);
	} else {
		return 0;
	}
}

/*==========================================================================
| Function name	: float32 CalMaxFromFloatDat(float32 * fDat, uint16 uDatNum)
| Description	: 
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-10-08
\=========================================================================*/
float32 CalMaxFromFloatDat(float32 * fDat, uint8 u8DatNum)
{
	float32 fMax;

	fMax = *fDat;
	u8DatNum--;
	fDat++;
	for( ; u8DatNum != 0; u8DatNum--) {
		if(fMax < *fDat) {
			fMax = *fDat;
		}
		fDat++;
	}

	return fMax;
}

/*==========================================================================
| Function name	: float32 CalMinFromFloatDat(float32 * fDat, uint16 uDatNum)
| Description	: 
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-10-08
\=========================================================================*/
float32 CalMinFromFloatDat(float32 * fDat, uint8 u8DatNum)
{
	float32 fMin;

	fMin = *fDat;
	u8DatNum--;
	fDat++;
	for( ; u8DatNum != 0; u8DatNum--) {
		if(fMin > *fDat) {
			fMin = *fDat;
		}
		fDat++;
	}

	return fMin;
}


/*==========================================================================
| Description	: 按照小端模式的32b/16b格式把带传输数据填进 TxBuf(ModbusRTU, 以太网均为大端模式), 值得注意的是: uRegNum, iMaxCopyRegNum 均按照16bit为准
				  使用注意：不检查数据是否溢出，需要其他机制保证
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-07-22
\=========================================================================*/
uint16 CopyL32DatToTxBuf(uint8** ppDest, uint8* pSrc, uint16 uRegNum, int16 iMaxCopyRegNum)
{
	/* 初始化 */
	uint32* pU32Dest = (uint32*)(*ppDest);
	uint32* pU32Src = (uint32*)pSrc;
	if(iMaxCopyRegNum < 0) {
		iMaxCopyRegNum = 0;
	} else if(uRegNum < iMaxCopyRegNum) {
		iMaxCopyRegNum = uRegNum;
	}

	/* 填传输内容 */
	int16 i;
	for(i = iMaxCopyRegNum/2 ; i > 0; i--) {
		*pU32Dest++ = __rev(*pU32Src++);
	}
	
	*ppDest = (uint8*)pU32Dest;
	return iMaxCopyRegNum;
}

uint16 CopyL16DatToTxBuf(uint8** ppDest, uint8* pU8Src, uint16 uRegNum, int16 iMaxCopyRegNum)
{
	/* 初始化 */
	uint16* pU16Dest = (uint16*)(*ppDest);
	uint16* pU16Src = (uint16*)pU8Src;
 	if(iMaxCopyRegNum < 0) {
		iMaxCopyRegNum = 0;
	} else if(uRegNum < iMaxCopyRegNum) {
		iMaxCopyRegNum = uRegNum;
	}

	/* 填传输内容 */
	int16 i;
	for(i = iMaxCopyRegNum ; i > 0; i--) {
		*pU16Dest++ = __rev16(*pU16Src++);
	}
	
	*ppDest = (uint8*)pU16Dest;
	return iMaxCopyRegNum;
}

/* 把本地uint16的数据，先转成float32，再拷贝到TxBuf里 */
uint16 CopyLU16asF32ToTxBuf(uint8** ppDest, uint16* pUSrc, uint16 uRegNum, int16 iMaxCopyRegNum)
{
	/* 初始化 */
 	if(iMaxCopyRegNum < 0) {
		iMaxCopyRegNum = 0;
	} else if(uRegNum < iMaxCopyRegNum) {
		iMaxCopyRegNum = uRegNum;
	}

	/* 填传输内容 */
	uint32* pU32Dest = (uint32*)*ppDest;
	int16 i;
	for(i = iMaxCopyRegNum/2 ; i > 0; i--) {
		float32 fData = (float32)(*pUSrc++);
		*pU32Dest++ = __rev(*((uint32*)&fData));
	}

	*ppDest = (uint8*)pU32Dest;
	return iMaxCopyRegNum;
}

/*==========================================================================
| Description	: 把大端模式(ModbusRTU, 以太网均为大端模式)的64b/32b/16b(通讯过来的数据)拷进数据Buf(小端模式),
					值得注意: uRegOff, MaxDatNum 均按照16bit为准
					使用注意：不检查数据是否溢出，需要其他机制保证
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2015-12-2
\=========================================================================*/
uint16 CopyRxBufToL32Dat(uint32* pU32Dest, uint8** ppSrc, uint16 uRegNum, int16 iMaxCopyRegNum)
{
	/* 初始化 */
	if(iMaxCopyRegNum < 0) {
		iMaxCopyRegNum = 0;
 	} else if(uRegNum < iMaxCopyRegNum) {
		iMaxCopyRegNum = uRegNum;
	}

	/* 填传输内容 */
	uint32* pU32Src = (uint32*)(*ppSrc);
	int16 i;
	for(i = iMaxCopyRegNum/2; i > 0; i--) {
		*pU32Dest++ = __rev(*pU32Src++);
	}
	
	*ppSrc = (uint8*)pU32Src;
	return iMaxCopyRegNum;
}
uint16 CopyRxBufToL16Dat(uint16* pUDest, uint8** ppSrc, uint16 uRegNum, int16 iMaxCopyRegNum)
{
	/* 初始化 */
	if(iMaxCopyRegNum < 0) {
		iMaxCopyRegNum = 0;
	} else if(uRegNum < iMaxCopyRegNum) {
		iMaxCopyRegNum = uRegNum;
	}

	/* 填传输内容 */
	uint16* pU16Src = (uint16*)(*ppSrc);
	int16 i;
	for(i = iMaxCopyRegNum ; i > 0; i--) {
		*pUDest++ = __rev16(*pU16Src++);
	}
	
	*ppSrc = (uint8*)pU16Src;
	return iMaxCopyRegNum;
}

/*==========================================================================
| Description	: 加密与解密部分，采用AES加解密方式
	AES 采取CBC模式, IV = 0
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2017-4-2
\=========================================================================*/
#define KEYBITS 128
#if KEYBITS == 128
  #define ROUNDS 	10 	//!< Number of rounds.
  #define KEYLENGTH 4 	//!< Key length in number of dword.
#elif KEYBITS == 192
  #define ROUNDS 	12 	//!< Number of rounds.
  #define KEYLENGTH 6 	//!< // Key length in number of dword.
#elif KEYBITS == 256
  #define ROUNDS 	14 	//!< Number of rounds.
  #define KEYLENGTH 8 	//!< Key length in number of dword.
#else
  #error Key must be 128, 192 or 256 bits!
#endif

#define BLOCKSIZE 	4 	//!< Block size in number of dword.
#define BPOLY 0x1b 		//!< Lower 8 bits of (x^8+x^4+x^3+x+1), ie. (x^4+x^3+x+1).

const uint8 cnst_u8SBox[] = {
	0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76, 	/*0*/
	0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, 	/*1*/
	0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15, 	/*2*/
	0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75, 	/*3*/
	0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, 	/*4*/
	0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf, 	/*5*/
	0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, 	/*6*/
	0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 	/*7*/
	0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73, 	/*8*/
	0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, 	/*9*/
	0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 	/*a*/
	0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, 	/*b*/
	0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, 	/*c*/
	0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 	/*d*/
	0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf, 	/*e*/
	0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16		/*f*/
};
const uint8 cnst_u8InvSBox[] = {
	0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb, 	/*0*/
	0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb, 	/*1*/
	0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e, 	/*2*/
	0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25, 	/*3*/
	0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92, 	/*4*/
	0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84, 	/*5*/
	0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06, 	/*6*/
	0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b, 	/*7*/
	0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73, 	/*8*/
	0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e, 	/*9*/
	0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b, 	/*a*/
	0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4, 	/*b*/
	0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f, 	/*c*/
	0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef, 	/*d*/
	0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61, 	/*e*/
	0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d		/*f*/
};

void KeyExpansion(uint32* pU32ExpandedKey)
{
	uint32 u32Temp;
	uint32 u32Rcon = 0x01;
	int16 i;
	
	u32Temp = pU32ExpandedKey[KEYLENGTH - 1];	/* 最后一个初始密码 */
	pU32ExpandedKey += KEYLENGTH;

	for(i = KEYLENGTH; i < BLOCKSIZE*(ROUNDS+1); i++) {
		if((i % KEYLENGTH) == 0) {				// Are we at the start of a multiple of the key size?
			u32Temp = ((u32Temp>>8) | (u32Temp<<24));	/* 循环左移8位--大端与小端模式的区别 */
			u32Temp = (((uint32)cnst_u8SBox[(u32Temp>>24)&0xFF])<<24) + (((uint32)cnst_u8SBox[(u32Temp>>16)&0xFF])<<16)
						+ (((uint32)cnst_u8SBox[(u32Temp>>8)&0xFF])<<8) + (((uint32)cnst_u8SBox[(u32Temp>>0)&0xFF])<<0);
			u32Temp = u32Temp^u32Rcon;
			if(u32Rcon == 0x80) {
				u32Rcon = 0x1B;
			} else {
				u32Rcon <<= 1;
			}
#if KEYLENGTH > 6								// Keysize larger than 24 bytes, ie. larger that 192 bits?
		} else if((i % KEYLENGTH) == BLOCKSIZE) {	// Are we right past a block size?
			u32Temp = (((uint32)cnst_u8SBox[(u32Temp>>24)&0xFF])<<24) + (((uint32)cnst_u8SBox[(u32Temp>>16)&0xFF])<<16)
					+ (((uint32)cnst_u8SBox[(u32Temp>>8)&0xFF])<<8) + (((uint32)cnst_u8SBox[(u32Temp>>0)&0xFF])<<0);
#endif
		}
		u32Temp ^= *(pU32ExpandedKey - KEYLENGTH);
		*pU32ExpandedKey++ = u32Temp;
	}
}

void AddRoundKey(uint32* pU32Data, uint32* pU32Pwd)
{
	*pU32Data = (*pU32Data) ^ (*pU32Pwd++);
	pU32Data++;
	*pU32Data = (*pU32Data) ^ (*pU32Pwd++);
	pU32Data++;
	*pU32Data = (*pU32Data) ^ (*pU32Pwd++);
	pU32Data++;
	*pU32Data = (*pU32Data) ^ (*pU32Pwd++);
}

/* GF(28)域乘法: 注意y不得超过0x0F, x不得超过0xFF */
#define xtime(x) 		((x<<1) ^ (((x>>7) & 1) * 0x1b))
#define Multiply(x,y) 	(((y & 1) * x) ^ ((y>>1 & 1) * xtime(x)) ^ ((y>>2 & 1) * xtime(xtime(x))) 						\
							^ ((y>>3 & 1) * xtime(xtime(xtime(x)))) ^ ((y>>4 & 1) * xtime(xtime(xtime(xtime(x))))))
void MixColumn(uint8* column)
{
	unsigned char result[4];
	
	result[0] = Multiply(column[0], 0x02) ^ Multiply(column[1], 0x03) ^ Multiply(column[2], 0x01) ^ Multiply(column[3], 0x01);
	result[1] = Multiply(column[0], 0x01) ^ Multiply(column[1], 0x02) ^ Multiply(column[2], 0x03) ^ Multiply(column[3], 0x01);
	result[2] = Multiply(column[0], 0x01) ^ Multiply(column[1], 0x01) ^ Multiply(column[2], 0x02) ^ Multiply(column[3], 0x03);
	result[3] = Multiply(column[0], 0x03) ^ Multiply(column[1], 0x01) ^ Multiply(column[2], 0x01) ^ Multiply(column[3], 0x02);
	
	column[0] = result[0];
	column[1] = result[1];
	column[2] = result[2];
	column[3] = result[3];
}

void SubBytesAndShiftRows(uint8* pU8Data)
{
	/* S盒替代 */
	int16 i;
	for(i = 0; i < 16; i++) {
		*pU8Data = cnst_u8SBox[*pU8Data];
		pU8Data++;
	}
	pU8Data -= 16;	/* 修复指针 */
	
	/* 行移位：第一行移位 */
	uint8 u8Temp;
	u8Temp = pU8Data[1 + 0*4];
	pU8Data[1 + 0*4] = pU8Data[1 + 1*4];
	pU8Data[1 + 1*4] = pU8Data[1 + 2*4];
	pU8Data[1 + 2*4] = pU8Data[1 + 3*4];
	pU8Data[1 + 3*4] = u8Temp;
	
	/* 行移位：第二行移位 */
	u8Temp = pU8Data[2 + 0*4];
	pU8Data[2 + 0*4] = pU8Data[2 + 2*4];
	pU8Data[2 + 2*4] = u8Temp;
	u8Temp = pU8Data[2 + 1*4];
	pU8Data[2 + 1*4] = pU8Data[2 + 3*4];
	pU8Data[2 + 3*4] = u8Temp;
	
	/* 行移位：第三行移位 */
	u8Temp = pU8Data[3 + 3*4];
	pU8Data[3 + 3*4] = pU8Data[3 + 2*4];
	pU8Data[3 + 2*4] = pU8Data[3 + 1*4];
	pU8Data[3 + 1*4] = pU8Data[3 + 0*4];
	pU8Data[3 + 0*4] = u8Temp;
}

void EncryptOneBlock(uint32* pU32Data, uint32* pU32ExpandKey)
{
	int16 i;

	AddRoundKey(pU32Data, pU32ExpandKey);		/* 初始轮: 轮密钥加 */
	pU32ExpandKey += 4;
	for(i = 1; i < ROUNDS; i++) {				/* 1 ~ N-1轮 */
		SubBytesAndShiftRows((uint8*)pU32Data);/* S盒替换、行移位 */
		MixColumn((uint8*)(pU32Data + 0));		/* 列混淆 */
		MixColumn((uint8*)(pU32Data + 1));
		MixColumn((uint8*)(pU32Data + 2));
		MixColumn((uint8*)(pU32Data + 3));
		AddRoundKey(pU32Data, pU32ExpandKey);	/* 轮密钥加 */
		pU32ExpandKey += 4; 					/* 指向下一组密钥 */
	}
	/* 最后一轮 */
	SubBytesAndShiftRows((uint8*)pU32Data); 	/* S盒替换、行移位 */
	AddRoundKey(pU32Data, pU32ExpandKey);		/* 轮密钥加 */
}

void InvShiftRowsAndSubBytes(uint8* pU8Data)
{
	/* 第二行移位 */
	uint8 u8Temp;
	u8Temp = pU8Data[1 + 3*4];
	pU8Data[1 + 3*4] = pU8Data[1 + 2*4];
	pU8Data[1 + 2*4] = pU8Data[1 + 1*4];
	pU8Data[1 + 1*4] = pU8Data[1 + 0*4];
	pU8Data[1 + 0*4] = u8Temp;
	
	/* 第三行移位 */
	u8Temp = pU8Data[2 + 0*4];
	pU8Data[2 + 0*4] = pU8Data[2 + 2*4];
	pU8Data[2 + 2*4] = u8Temp;
	u8Temp = pU8Data[2 + 1*4];
	pU8Data[2 + 1*4] = pU8Data[2 + 3*4];
	pU8Data[2 + 3*4] = u8Temp;
	
	/* 第四行移位 */
	u8Temp = pU8Data[3 + 0*4];
	pU8Data[3 + 0*4] = pU8Data[3 + 1*4];
	pU8Data[3 + 1*4] = pU8Data[3 + 2*4];
	pU8Data[3 + 2*4] = pU8Data[3 + 3*4];
	pU8Data[3 + 3*4] = u8Temp;

	/* S盒替代 */
	int16 i;
	for(i = 0; i < 16; i++) {
		*pU8Data = cnst_u8InvSBox[*pU8Data];
		pU8Data++;
	};
}

void InvMixColumn(uint8* column)
{
	unsigned char r0, r1, r2, r3;
	
	r0 = column[1] ^ column[2] ^ column[3];
	r1 = column[0] ^ column[2] ^ column[3];
	r2 = column[0] ^ column[1] ^ column[3];
	r3 = column[0] ^ column[1] ^ column[2];
	
	column[0] = (column[0] << 1) ^ (column[0] & 0x80 ? BPOLY : 0);
	column[1] = (column[1] << 1) ^ (column[1] & 0x80 ? BPOLY : 0);
	column[2] = (column[2] << 1) ^ (column[2] & 0x80 ? BPOLY : 0);
	column[3] = (column[3] << 1) ^ (column[3] & 0x80 ? BPOLY : 0);
	
	r0 ^= column[0] ^ column[1];
	r1 ^= column[1] ^ column[2];
	r2 ^= column[2] ^ column[3];
	r3 ^= column[0] ^ column[3];
	
	column[0] = (column[0] << 1) ^ (column[0] & 0x80 ? BPOLY : 0);
	column[1] = (column[1] << 1) ^ (column[1] & 0x80 ? BPOLY : 0);
	column[2] = (column[2] << 1) ^ (column[2] & 0x80 ? BPOLY : 0);
	column[3] = (column[3] << 1) ^ (column[3] & 0x80 ? BPOLY : 0);
	
	r0 ^= column[0] ^ column[2];
	r1 ^= column[1] ^ column[3];
	r2 ^= column[0] ^ column[2];
	r3 ^= column[1] ^ column[3];
	
	column[0] = (column[0] << 1) ^ (column[0] & 0x80 ? BPOLY : 0);
	column[1] = (column[1] << 1) ^ (column[1] & 0x80 ? BPOLY : 0);
	column[2] = (column[2] << 1) ^ (column[2] & 0x80 ? BPOLY : 0);
	column[3] = (column[3] << 1) ^ (column[3] & 0x80 ? BPOLY : 0);
	
	column[0] ^= column[1] ^ column[2] ^ column[3];
	r0 ^= column[0];
	r1 ^= column[0];
	r2 ^= column[0];
	r3 ^= column[0];
	
	column[0] = r0;
	column[1] = r1;
	column[2] = r2;
	column[3] = r3;
}

void DecryptOneBlock(uint32* pU32Data, uint32* pU32ExpandKey)
{
	int16 i;
	
	pU32ExpandKey -= 4;
	AddRoundKey(pU32Data, pU32ExpandKey);			/* 初始轮: 轮密钥加 */
	for(i = 1; i < ROUNDS; i++) {					/* 1 ~ N-1轮 */
		InvShiftRowsAndSubBytes((uint8*)pU32Data);	/* 行移位、S盒替换 */
		pU32ExpandKey -= 4;
		AddRoundKey(pU32Data, pU32ExpandKey);		/* 轮密钥加 */
		InvMixColumn((uint8*)(pU32Data + 0));		/* 列混淆 */
		InvMixColumn((uint8*)(pU32Data + 1));
		InvMixColumn((uint8*)(pU32Data + 2));
		InvMixColumn((uint8*)(pU32Data + 3));
	}
	InvShiftRowsAndSubBytes((uint8*)pU32Data);		/* 行移位、S盒替换 */
	pU32ExpandKey -= 4;
	AddRoundKey(pU32Data, pU32ExpandKey);			/* 轮密钥加 */
}

void EncryptWithAES(AUTH_TYPE AuthType, uint32* pU32Data, uint32 u32ByteLen, uint32 u32IV)
{
	assert(0 == (u32ByteLen % 16));
	uint32 u32ExpandKey[BLOCKSIZE*(ROUNDS+1)];

	GetAuthKey(AuthType, u32ExpandKey);				/* 根据操作类型，产生授权Key */
	KeyExpansion(u32ExpandKey);						/* AES加密的产生扩展密码 */
	
	/* 采样AES-CBC模式加密 */
	*pU32Data ^= u32IV;
	u32ByteLen = u32ByteLen/16;
	while(1) {
		EncryptOneBlock(pU32Data, u32ExpandKey);
		u32ByteLen--;
		if(u32ByteLen > 0) {
			pU32Data[4] ^= pU32Data[0];
			pU32Data[5] ^= pU32Data[1];
			pU32Data[6] ^= pU32Data[2];
			pU32Data[7] ^= pU32Data[3];
			pU32Data += 4;
		} else {
			break;
		}
	}
}

void DecryptWithAES(AUTH_TYPE AuthType, uint32* pU32Data, uint32 u32ByteLen, uint32 u32IV)
{
	uint32 u32ExpandKey[BLOCKSIZE*(ROUNDS+1)];
	uint32 u32CipherData[BLOCKSIZE], u32LastCipherData[BLOCKSIZE];

	if(AuthType == AUTH_AES_COMM_SN_LIC) {
		u32ExpandKey[0] = g_PubSoftAuthCtr.u32SnLicCommAesKey[0];
		u32ExpandKey[1] = g_PubSoftAuthCtr.u32SnLicCommAesKey[1];
		u32ExpandKey[2] = 0;
		u32ExpandKey[3] = 0;
	} else {
		GetAuthKey(AuthType, u32ExpandKey);				/* 根据操作类型，产生授权Key */
	}
	KeyExpansion(u32ExpandKey);						/* AES加密的产生扩展密码 */
	
	/* 采样AES-CBC模式加密 */
	u32LastCipherData[0] = u32IV;
	u32LastCipherData[1] = 0;
	u32LastCipherData[2] = 0;
	u32LastCipherData[3] = 0;
	for(u32ByteLen = u32ByteLen/16; u32ByteLen > 0; u32ByteLen--) {
		u32CipherData[0] = pU32Data[0];
		u32CipherData[1] = pU32Data[1];
		u32CipherData[2] = pU32Data[2];
		u32CipherData[3] = pU32Data[3];
		DecryptOneBlock(pU32Data, &u32ExpandKey[BLOCKSIZE*(ROUNDS+1)]);
		pU32Data[0] ^= u32LastCipherData[0];
		pU32Data[1] ^= u32LastCipherData[1];
		pU32Data[2] ^= u32LastCipherData[2];
		pU32Data[3] ^= u32LastCipherData[3];
		u32LastCipherData[0] = u32CipherData[0];
		u32LastCipherData[1] = u32CipherData[1];
		u32LastCipherData[2] = u32CipherData[2];
		u32LastCipherData[3] = u32CipherData[3];
		pU32Data += 4;
	}
}


void EncryptWithRSA(uint16 uPwd, uint32 u32FlashAdd, uint8* FlashData)
{
}
void DecryptWithRSA(uint16 uPwd, uint32 u32FlashAdd, uint8* FlashData)
{
}

/*==========================================================================
| Description	: Base64编码、解码
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2018-11-23
\=========================================================================*/
const uint8 cnst_u8Base64Table[64] = {
	'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',	'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
	'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',	'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
	'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
	'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/' };
uint8* EncodeByBase64(uint8* pU8Data, uint16 uByteLen, uint8* pU8Base64Buf)
{
	uint8 u8Data;

	/* 整24位处理 */
	for( ; uByteLen >= 3; uByteLen -= 3) {
		u8Data = *pU8Data++;
		*pU8Base64Buf++ = cnst_u8Base64Table[u8Data>>2];
		u8Data = ((u8Data&0x03)<<4) | ((*pU8Data)>>4);
		*pU8Base64Buf++ = cnst_u8Base64Table[u8Data];
		u8Data = (*pU8Data++ & 0x0F)<<2;
		u8Data |= *pU8Data>>6;
		*pU8Base64Buf++ = cnst_u8Base64Table[u8Data];
		*pU8Base64Buf++ = cnst_u8Base64Table[*pU8Data++ & 0x3F];
	}
	
	/* 尾数处理 */
	if(uByteLen == 1) {			/* 剩余1个字节，需要补充2个0 */
		u8Data = *pU8Data++;
		*pU8Base64Buf++ = cnst_u8Base64Table[u8Data>>2];
		*pU8Base64Buf++ = cnst_u8Base64Table[((u8Data&0x03)<<4)];
		*pU8Base64Buf++ = '=';
		*pU8Base64Buf++ = '=';
	} else if(uByteLen == 2) {	/* 剩余2个字节，需要补充1个0 */
		u8Data = *pU8Data++;
		*pU8Base64Buf++ = cnst_u8Base64Table[u8Data>>2];
		u8Data = ((u8Data&0x03)<<4) | ((*pU8Data)>>4);
		*pU8Base64Buf++ = cnst_u8Base64Table[u8Data];
		*pU8Base64Buf++ = cnst_u8Base64Table[(*pU8Data & 0x0F)<<2];
		*pU8Base64Buf++ = '=';
	}
	
	return pU8Base64Buf;
}

uint8* DecodeByBase64(uint8* pU8Base64Buf, uint16 uBase64BLen, uint8* pU8Data, uint16 uDataBLen)
{
	int16 i;
	int16 iEnd = -1;	/* 如果Base64编码遇到'='，需要记录结束位置 */
	uint8 u8Data[4];
	
	for( ; uBase64BLen >= 4; uBase64BLen -= 4) {
		for(i = 3; i >= 0; i--) {
			if(('A' <= *pU8Base64Buf) && (*pU8Base64Buf <= 'Z')) {
				u8Data[i] = *pU8Base64Buf - 'A' + 0;
			} else if(('a' <= *pU8Base64Buf) && (*pU8Base64Buf <= 'z')) {
				u8Data[i] = *pU8Base64Buf - 'a' + 26;
			} else if(('0' <= *pU8Base64Buf) && (*pU8Base64Buf <= '9')) {
				u8Data[i] = *pU8Base64Buf - '0' + 52;
			} else if(*pU8Base64Buf == '+') {
				u8Data[i] = 62;
			} else if(*pU8Base64Buf == '/') {
				u8Data[i] = 63;
			} else if(*pU8Base64Buf == '=') {
				iEnd = i;
				break;
			} else {
				break;
			}
			pU8Base64Buf++;
		}
		if(i >= 0) {	/* 有尾数 */
			break;
		} else if(uDataBLen >= 3) {
			uDataBLen -= 3;
			*pU8Data++ = (u8Data[3]<<2) | (u8Data[2]>>4);
			*pU8Data++ = (u8Data[2]<<4) | (u8Data[1]>>2);
			*pU8Data++ = (u8Data[1]<<6) | (u8Data[0]>>0);
		} else {		/* 数据Buf不够了 */
			break;
		}
	}
	
	/* 尾数处理 */
	if((iEnd == 0) && (uDataBLen >= 2)) {			/* 一个'=' */
		*pU8Data++ = (u8Data[3]<<2) | (u8Data[2]>>4);
		*pU8Data++ = (u8Data[2]<<4) | (u8Data[1]>>2);
	} else if((iEnd == 1) && (uDataBLen >= 1)) {	/* 两个'=' */
		*pU8Data++ = (u8Data[3]<<2) | (u8Data[2]>>4);
	}
	
	return pU8Data;
}

/*==========================================================================
| Description	: MD5计算
| G/Out var		:
| Author		: Cui YeHong				Date	: 2019-03-26
\=========================================================================*/
#define S11 7
#define S12 12
#define S13 17
#define S14 22
#define S21 5
#define S22 9
#define S23 14
#define S24 20
#define S31 4
#define S32 11
#define S33 16
#define S34 23
#define S41 6
#define S42 10
#define S43 15
#define S44 21
const uint8 cnst_PADDING[64] = {
	0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
#define F(x, y, z) (((x) & (y)) | ((~x) & (z)))
#define G(x, y, z) (((x) & (z)) | ((y) & (~z)))
#define H(x, y, z) ((x) ^ (y) ^ (z))
#define I(x, y, z) ((y) ^ ((x) | (~z)))
#define ROTATE_LEFT(x, n) (((x) << (n)) | ((x) >> (32-(n))))
#define FF(a, b, c, d, x, s, ac) {  (a) += F ((b), (c), (d)) + (x) + (uint32)(ac);  (a) = ROTATE_LEFT ((a), (s));  (a) += (b);  }
#define GG(a, b, c, d, x, s, ac) {  (a) += G ((b), (c), (d)) + (x) + (uint32)(ac);  (a) = ROTATE_LEFT ((a), (s));  (a) += (b);  }
#define HH(a, b, c, d, x, s, ac) {  (a) += H ((b), (c), (d)) + (x) + (uint32)(ac);  (a) = ROTATE_LEFT ((a), (s));  (a) += (b);  }
#define II(a, b, c, d, x, s, ac) {  (a) += I ((b), (c), (d)) + (x) + (uint32)(ac);  (a) = ROTATE_LEFT ((a), (s));  (a) += (b); 	}

void MD5Init(MD5_CTX *pContext)
{
	pContext->u32Count[0] = pContext->u32Count[1] = 0;
	pContext->u32State[0] = 0x67452301;
	pContext->u32State[1] = 0xefcdab89;
	pContext->u32State[2] = 0x98badcfe;
	pContext->u32State[3] = 0x10325476;
}

void MD5Encode(uint8 *pOutput, uint32 *pInput, uint32 u32Len)
{
	uint32 u32Count1, u32Count2;
	for(u32Count1 = 0, u32Count2 = 0; u32Count2 < u32Len; u32Count1++, u32Count2 += 4) {
		pOutput[u32Count2] = (uint8) (pInput[u32Count1] & 0xff);
		pOutput[u32Count2 + 1] = (uint8) ((pInput[u32Count1] >> 8) & 0xff);
		pOutput[u32Count2 + 2] = (uint8) ((pInput[u32Count1] >> 16) & 0xff);
		pOutput[u32Count2 + 3] = (uint8) ((pInput[u32Count1] >> 24) & 0xff);
	}
}

void MD5Decode(uint32 *pOutput, uint8 *pInput, uint32 u32Len)
{
	uint32 u32Count1, u32Count2;
	for(u32Count1 = 0, u32Count2 = 0; u32Count2 < u32Len; u32Count1++, u32Count2 += 4) {
		pOutput[u32Count1] = ((uint32) pInput[u32Count2]) | (((uint32) pInput[u32Count2 + 1]) << 8) |
				(((uint32) pInput[u32Count2 + 2]) << 16) | (((uint32) pInput[u32Count2 + 3]) << 24);
	}
}

void MD5Transform (uint32 state[4], uint8 block[64])
{
	uint32 a = state[0], b = state[1], c = state[2], d = state[3], x[16];
	MD5Decode(x, block, 64);
	FF(a, b, c, d, x[0], S11, 0xd76aa478); /* 1 */
	FF(d, a, b, c, x[1], S12, 0xe8c7b756); /* 2 */
	FF(c, d, a, b, x[2], S13, 0x242070db); /* 3 */
	FF(b, c, d, a, x[3], S14, 0xc1bdceee); /* 4 */
	FF(a, b, c, d, x[4], S11, 0xf57c0faf); /* 5 */
	FF(d, a, b, c, x[5], S12, 0x4787c62a); /* 6 */
	FF(c, d, a, b, x[6], S13, 0xa8304613); /* 7 */
	FF(b, c, d, a, x[7], S14, 0xfd469501); /* 8 */
	FF(a, b, c, d, x[8], S11, 0x698098d8); /* 9 */
	FF(d, a, b, c, x[9], S12, 0x8b44f7af); /* 10 */
	FF(c, d, a, b, x[10], S13, 0xffff5bb1); /* 11 */
	FF(b, c, d, a, x[11], S14, 0x895cd7be); /* 12 */
	FF(a, b, c, d, x[12], S11, 0x6b901122); /* 13 */
	FF(d, a, b, c, x[13], S12, 0xfd987193); /* 14 */
	FF(c, d, a, b, x[14], S13, 0xa679438e); /* 15 */
	FF(b, c, d, a, x[15], S14, 0x49b40821); /* 16 */
	GG(a, b, c, d, x[1], S21, 0xf61e2562); /* 17 */
	GG(d, a, b, c, x[6], S22, 0xc040b340); /* 18 */
	GG(c, d, a, b, x[11], S23, 0x265e5a51); /* 19 */
	GG(b, c, d, a, x[0], S24, 0xe9b6c7aa); /* 20 */
	GG(a, b, c, d, x[5], S21, 0xd62f105d); /* 21 */
	GG(d, a, b, c, x[10], S22, 0x2441453); /* 22 */
	GG(c, d, a, b, x[15], S23, 0xd8a1e681); /* 23 */
	GG(b, c, d, a, x[4], S24, 0xe7d3fbc8); /* 24 */
	GG(a, b, c, d, x[9], S21, 0x21e1cde6); /* 25 */
	GG(d, a, b, c, x[14], S22, 0xc33707d6); /* 26 */
	GG(c, d, a, b, x[3], S23, 0xf4d50d87); /* 27 */
	GG(b, c, d, a, x[8], S24, 0x455a14ed); /* 28 */
	GG(a, b, c, d, x[13], S21, 0xa9e3e905); /* 29 */
	GG(d, a, b, c, x[2], S22, 0xfcefa3f8); /* 30 */
	GG(c, d, a, b, x[7], S23, 0x676f02d9); /* 31 */
	GG(b, c, d, a, x[12], S24, 0x8d2a4c8a); /* 32 */
	HH(a, b, c, d, x[5], S31, 0xfffa3942); /* 33 */
	HH(d, a, b, c, x[8], S32, 0x8771f681); /* 34 */
	HH(c, d, a, b, x[11], S33, 0x6d9d6122); /* 35 */
	HH(b, c, d, a, x[14], S34, 0xfde5380c); /* 36 */
	HH(a, b, c, d, x[1], S31, 0xa4beea44); /* 37 */
	HH(d, a, b, c, x[4], S32, 0x4bdecfa9); /* 38 */
	HH(c, d, a, b, x[7], S33, 0xf6bb4b60); /* 39 */
	HH(b, c, d, a, x[10], S34, 0xbebfbc70); /* 40 */
	HH(a, b, c, d, x[13], S31, 0x289b7ec6); /* 41 */
	HH(d, a, b, c, x[0], S32, 0xeaa127fa); /* 42 */
	HH(c, d, a, b, x[3], S33, 0xd4ef3085); /* 43 */
	HH(b, c, d, a, x[6], S34, 0x4881d05); /* 44 */
	HH(a, b, c, d, x[9], S31, 0xd9d4d039); /* 45 */
	HH(d, a, b, c, x[12], S32, 0xe6db99e5); /* 46 */
	HH(c, d, a, b, x[15], S33, 0x1fa27cf8); /* 47 */
	HH(b, c, d, a, x[2], S34, 0xc4ac5665); /* 48 */
	II(a, b, c, d, x[0], S41, 0xf4292244); /* 49 */
	II(d, a, b, c, x[7], S42, 0x432aff97); /* 50 */
	II(c, d, a, b, x[14], S43, 0xab9423a7); /* 51 */
	II(b, c, d, a, x[5], S44, 0xfc93a039); /* 52 */
	II(a, b, c, d, x[12], S41, 0x655b59c3); /* 53 */
	II(d, a, b, c, x[3], S42, 0x8f0ccc92); /* 54 */
	II(c, d, a, b, x[10], S43, 0xffeff47d); /* 55 */
	II(b, c, d, a, x[1], S44, 0x85845dd1); /* 56 */
	II(a, b, c, d, x[8], S41, 0x6fa87e4f); /* 57 */
	II(d, a, b, c, x[15], S42, 0xfe2ce6e0); /* 58 */
	II(c, d, a, b, x[6], S43, 0xa3014314); /* 59 */
	II(b, c, d, a, x[13], S44, 0x4e0811a1); /* 60 */
	II(a, b, c, d, x[4], S41, 0xf7537e82); /* 61 */
	II(d, a, b, c, x[11], S42, 0xbd3af235); /* 62 */
	II(c, d, a, b, x[2], S43, 0x2ad7d2bb); /* 63 */
	II(b, c, d, a, x[9], S44, 0xeb86d391); /* 64 */
	state[0] += a;
	state[1] += b;
	state[2] += c;
	state[3] += d;
}

void MD5Update(MD5_CTX *pContext, uint8 *pInput, uint32 u32InputLen)
{
	uint32 u32Count, u32Index, u32PartLen;
	u32Index = (uint32)((pContext->u32Count[0] >> 3) & 0x3F);
	if((pContext->u32Count[0] += ((uint32) u32InputLen << 3)) < ((uint32) u32InputLen << 3)) {
		pContext->u32Count[1]++;
	}
	pContext->u32Count[1] += ((uint32) u32InputLen >> 29);
	u32PartLen = 64 - u32Index;
	if(u32InputLen >= u32PartLen) { //如果填充完毕，则进行换算
		memcpy((uint8*)&pContext->u8Buffer[u32Index], (uint8*)pInput, u32PartLen);
		MD5Transform(pContext->u32State, pContext->u8Buffer);
		for(u32Count = u32PartLen; u32Count + 63 < u32InputLen; u32Count += 64) {
			MD5Transform(pContext->u32State, &pInput[u32Count]);
		}
		u32Index = 0;
	} else {
		u32Count = 0;
	}
	memcpy((uint8*)&pContext->u8Buffer[u32Index], (uint8*)&pInput[u32Count], u32InputLen - u32Count);  //进行填充
}

void MD5Final(uint8 digest[16], MD5_CTX *pContext)
{
	uint8 bits[8];
	uint32 u32Index, u32PadLen;
	MD5Encode(bits, pContext->u32Count, 8);    							//把输入信息长度放入bit中
	u32Index = (uint32) ((pContext->u32Count[0] >> 3) & 0x3f); 			//输入信息的长度模512后的长度，即输入信息的最后512位占了几位
	u32PadLen = (u32Index < 56) ? (56 - u32Index) : (120 - u32Index);  	//算出补充多少个位，就可以给最后一个512凑够448
	MD5Update(pContext, (uint8*)cnst_PADDING, u32PadLen);  						//填充信息，使最后512位凑够448
	MD5Update(pContext, bits, 8);           							//在448后面填充输入信息长度，此次填充64位，凑够512，然后进行转换
	MD5Encode(digest, pContext->u32State, 16);  						//把state中信息转换到digest中，从而输出加密结果
}

/*==========================================================================
| Description	: 远程传感器编解码部分，实现跨产品数据交换
					采样同样的编码格式，意味着同样的数据结构
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2018-8-8
\=========================================================================*/
uint8* PackRmtSenForUartComm(uint16 uCoder, uint16 uDevAdd, float32* pfData, uint8* pTxBuf)
{
	*pTxBuf++ = uDevAdd;
	*pTxBuf++ = uCoder;
	float32 fDCPwrVol = *pfData++;
	if(fDCPwrVol <= 0) {
		*((uint16*)pTxBuf) = 0;
	} else {
		*((uint16*)pTxBuf) = F32ToU16(fDCPwrVol*1000.0f);	/* 直流电压传输过程单位变为0.001V */
	}
	pTxBuf += 2;
	for( ; uCoder > 0; uCoder--) {
		*((float32*)pTxBuf) = *pfData++;
		pTxBuf += 4;
	}
	return pTxBuf;
}

#include "MdlUARTnModbus.h"
BOOL GetRmtSenFromUartComm(RMT_SEN_DAT *pRmtSenDat, uint16 uDevAdd, uint8* pRxBuf, uint16 uRxNum)
{
	if(((pRxBuf[0] == uDevAdd) || (uDevAdd == 0) || (pRxBuf[0] == 0) || (pRxBuf[0] == 0xFF))	/* 地址校验 */
		&& (CheckCrcForModbusRTU(pRxBuf, uRxNum) > 0))											/* CRC校验 */
	{
		uint16 uSensorNum = pRxBuf[1];			/* 依据编码格式计算传感器数量 */
		uint16 uDataLen = uSensorNum*4 + 6;		/* 传感器数量*4+2(电池电压)+1(地址)+1(Coder)+2(crc) */
		if((uSensorNum < MAX_RMT_SEN_NUM) && (uDataLen == uRxNum)) {		/* 接收长度校验 */
			float32* pfDest = &pRmtSenDat->fDCPwrVol;
			*pfDest++ = (*((uint16*)(pRxBuf + 2)))*0.001f;	/* 直流电源电压传输过程单位变为0.001V */
			float32* pfSrc = (float32*)(pRxBuf + 4);
			int16 i;
			for(i = uSensorNum; i > 0; i--) {
				*pfDest++ = *pfSrc++;
			}
			pRmtSenDat->uNum_DataValid = uSensorNum;
			pRmtSenDat->u32RcvTime_RTCSeconds = GetRTCSeconds();			/* 光纤最长时间10s */
			
			return TRUE;
		}
	}
	return FALSE;
}

void PrintRmtSenToJson(uint8** ppTxBuf, uint16 uCoder, float32* pfData)
{
	uint8* pTxBuf = *ppTxBuf;

	PrintStringNoOvChk(&pTxBuf, "\"rmts\":{");
	PrintU32DatToJson(&pTxBuf, "coder", uCoder, 0);
	PrintF32DatToJson(&pTxBuf, "dc_pwr_vol", *pfData++, 5);		/* 使用5位有效数字 */
	PrintF32ArrayToJson(&pTxBuf, "rmt_sen", pfData, 5, uCoder);	/* 使用5位有效数字 */
	PrintH32DatToJson(&pTxBuf, "din", g_Ctr.u32DinDat);
	pTxBuf--;					/* 前面多了一个',' */
	*pTxBuf++ = '}';			/* 和前面{对应 */
	*pTxBuf++ = ',';
	
	*ppTxBuf = pTxBuf;
}

void GetRmtSenFromMqttComm(RMT_SEN_DAT *pRmtSenDat, uint8* pRxBuf, uint8* pRxBufEnd)
{
	uint16 uSensorNo = 0;
	uint16 uCoder = ReadU32(&pRxBuf, pRxBufEnd);		/* 获取编码格式，方便后面解码 */
	if(uCoder > MAX_RMT_SEN_NUM) {
		uCoder = MAX_RMT_SEN_NUM;
	}
	GetF32(&pRxBuf, pRxBufEnd, &pRmtSenDat->fDCPwrVol);
	uint8* pRmtSenEnd = SkipCharInString(pRxBuf, pRxBufEnd, ']', 1); 	/* 搜索']'，因为传感器数据以数组形式发布 */
	while((uSensorNo < uCoder) && GetF32(&pRxBuf, pRmtSenEnd, &pRmtSenDat->fSenData[uSensorNo])) {
		uSensorNo++;
	}
	pRmtSenDat->uNum_DataValid = uSensorNo;
	pRmtSenDat->u32RcvTime_RTCSeconds = GetRTCSeconds();
}

/* 按照STAT_ITEM_VAL格式进行统计，并在超次数的时候报警 */
void CountStatItem(STAT_COUNT_ITEM* pStatItem, uint16 uMsgID)
{
	if(pStatItem->u32StatVal != 0xFFFFFFFF) {
		pStatItem->u32StatVal++;
	}
	if(pStatItem->u32StatThr && (pStatItem->u32StatVal >= pStatItem->u32StatThr)) {
		AddMsgB_WithVal(uMsgID, 0);
	}
}

/*==========================================================================
| Description	: 累计统计，如流量、电能统计
	首先在一个时间片内对流量求和，以便求平均
	在获得时间片结束标志(bFragFlag)后，把这个时间片内的流量的整数部分数据添加到总的积量数据中，										并把余数留存以便和下个时间片的数据相加
	积量数据的单位，就是流量在这个时间片积量的整数，如：
		流量是m3/s，时间片是s,则积量单位是m3；		流量是KW，时间片是3.6s，则积量单位是mKWH
	如果当前时间片不方便进行积量，则下次仍需给出时间片标志
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2018-12-25
\=========================================================================*/
BOOL CumulateFlow(STAT_CUM_ITEM* pCumItemStart, float32* pFlowStart, uint8 u8CumItemNum, BOOL bFragFlag)
{
	STAT_CUM_ITEM* pCumulantItem = pCumItemStart;
	static uint16 s_uCount_Sum = 0;
	int8 i;

	/* 时间片内求和以便求平均 */
	for(i = u8CumItemNum; i > 0; i--) {
		if(isfinite(*pFlowStart)) {
			pCumulantItem->fFragCumulant += *pFlowStart;
		}
		pCumulantItem++;
		pFlowStart++;
	}
	s_uCount_Sum++;
	
	/* 当前时间片结束，且可以写入，进行积量操作 */
	Swi_disable();
	if(bFragFlag && g_DataAcsIntf.bConfFree[SAVE_GRP_STAT]) {
		pCumulantItem = pCumItemStart;
		for(i = u8CumItemNum; i > 0; i--) {
			pCumulantItem->fCumulantRem += pCumulantItem->fFragCumulant/s_uCount_Sum;
			int64 i64Cumulant = pCumulantItem->fCumulantRem;
			pCumulantItem->i64TotalCumulant += i64Cumulant;
			pCumulantItem->fCumulantRem -= i64Cumulant;
			pCumulantItem->fFragCumulant = 0;
			pCumulantItem++;
		}
		s_uCount_Sum = 0;
	} else {
		bFragFlag = FALSE;
	}
	Swi_enable();
	
	return bFragFlag;
}

#if SUPPORT_SAMPLE
/*==========================================================================
| Description	: 开入与直流量处理
| Author		: Wang Renfei			Date	: 2008-3-4
\=========================================================================*/
void ProcDInFilter(void)
{
	uint16 uDinSampOffNum;
	int16 i;
	
	#define DIN_ON 1
#if (PERIOD_CTR_TASK_ms == 10)	/* 10ms的控制周期，DIn滤波长度不得超过10ms */
	#define DIN_OFF 9
	#define DIN_FILT_MASK	0x03FF
#elif(PERIOD_CTR_TASK_ms == 20)
	#define DIN_OFF 15
	#define DIN_FILT_MASK	0xFFFF
#endif
	for(i = MAX_DIN_NUM - 1; i >= 0; i--) {
		uDinSampOffNum = Cal1InBits16(g_uDInFilt[i] & DIN_FILT_MASK);
		if(uDinSampOffNum <= DIN_ON) {
			g_Ctr.u32DinDat |= (1UL<<i);
		#if (DEVICE_TYPE == V5_YYT3) || (DEVICE_TYPE == V5_YYT4)
			OnDInIndicator(i);
		#endif
		} else if(uDinSampOffNum >= DIN_OFF) {
			g_Ctr.u32DinDat &= ~(1UL<<i);
		#if (DEVICE_TYPE == V5_YYT3) || (DEVICE_TYPE == V5_YYT4)
			OffDInIndicator(i);
		#endif
		}
	}
}
#ifdef AC_CAP_CHN	/* 交流信号则必然有相应的测频功能 */
/* 交流量计算 */
void CalACSig(uint8 u8ACSigNo, uint16 uCalDatLen, float32 fExtTransRatio, float32* pfVal)
{
	volatile ANALOG_ADSAMP_VAR* pACSigADSamp = &g_ACSigADSamp[u8ACSigNo];
	if((u8ACSigNo < AC_TOTAL_CHN) && (u8ACSigNo < sizeof(cnst_fACSigRMS2Val)/sizeof(float32))
		&& ProcUniRecUDatAsWave((uDAT_UNI_REC*)pACSigADSamp, ANALOG_ADSAMP_BUF_LEN, uCalDatLen)) 
	{
		*pfVal = pACSigADSamp->fSig_rms*g_BrdHrdConf.fACSigBrdCorr[u8ACSigNo]*cnst_fACSigRMS2Val[u8ACSigNo]*fExtTransRatio;
	}
}

#if(!SOFT_RUN1_TEST0)
/* 采样并计算交流量频率--依据成形为方波来进行 */
typedef struct {
	uint32 u32MainPulseEdge;						/* 主信号边沿信息 */
	uint32 u32Cycle;
}ACSIG_PULSE_VAR;
ACSIG_PULSE_VAR g_ACSigPulse[AC_CAP_CHN];

void ProcCap(uint32 u32CapTime, uint8 u8ACSigNo)
{
	if((u8ACSigNo < AC_CAP_CHN) && (u32CapTime != g_ACSigPulse[u8ACSigNo].u32MainPulseEdge)) {
		g_ACSigPulse[u8ACSigNo].u32Cycle = u32CapTime - g_ACSigPulse[u8ACSigNo].u32MainPulseEdge;
		g_ACSigPulse[u8ACSigNo].u32MainPulseEdge = u32CapTime;
	}
}

float32 CalACSigFreq(uint8 u8ACSigNo)
{
	if((u8ACSigNo < AC_TOTAL_CHN) && g_ACSigPulse[u8ACSigNo].u32Cycle) {
		return (float32)CAP_CLK_FREQ_Hz/g_ACSigPulse[u8ACSigNo].u32Cycle;
	} else {
		return 0;
	}
}
#endif

/* 虽然采样是交流量，但是以直流量的方式进行真值计算 */
void CalACSigAsDCSig(uint8 u8ACSigNo, uint8 u8DCSigNo, uint16 uCalDatLen, float32 fExtTransRatio, float32* pfVal)
{
	volatile ANALOG_ADSAMP_VAR* pACSigADSamp = &g_ACSigADSamp[u8ACSigNo];
	if((u8ACSigNo < AC_TOTAL_CHN) && (u8DCSigNo < sizeof(cnst_DCSigFixCorr)/sizeof(ANALOG_SIG_CORR))
		&& ProcUniRecUDatAsWave((uDAT_UNI_REC*)pACSigADSamp, ANALOG_ADSAMP_BUF_LEN, uCalDatLen)) 
	{
		*pfVal = CalPValFromDCSigVal(pACSigADSamp->fSig_avr, fExtTransRatio, u8DCSigNo, NULL);
	}
}
#endif
/* 直流量计算 */
void CalDCSig(uint8 u8DCSigNo, uint16 uCalDatLen, float32 fExtTransRatio, float32* pfVal, ANALOG_SIG_CORR* pAnaSigCorr)
{
	volatile ANALOG_ADSAMP_VAR* pDCSigADSamp = &g_DCSigADSamp[u8DCSigNo];
	if((u8DCSigNo < DC_TOTAL_CHN) && ProcUniRecUDatAsWave((uDAT_UNI_REC*)pDCSigADSamp, ANALOG_ADSAMP_BUF_LEN, uCalDatLen)) {
		/* 信号值(avr值)转化为物理值 */
		*pfVal = CalPValFromDCSigVal(pDCSigADSamp->fSig_avr, fExtTransRatio, u8DCSigNo, pAnaSigCorr);
	}
}

float32 CalPValFromDCSigVal(float32 fSigVal, float32 fExtTransRatio, uint8 u8DCSigNo, ANALOG_SIG_CORR* pAnaSigCorr)
{
	/* 真值计算 */
	fSigVal -= cnst_DCSigFixCorr[u8DCSigNo].fZeroCorr;					/* 固有零漂 */
	fSigVal *= cnst_DCSigFixCorr[u8DCSigNo].fGainCorr;
	if(pAnaSigCorr != NULL) {
    	fSigVal -= pAnaSigCorr->fZeroCorr;
    	fSigVal *= pAnaSigCorr->fGainCorr;
	}
	fSigVal *= fExtTransRatio;

	return fSigVal;
}

/* 使用 ANALOG_SIG_CORR2 进行矫正，取平均值 */
void CalDCSig2(uint8 u8DCSigNo, uint16 uCalDatLen, float32 fExtTransRatio, float32* pfVal, ANALOG_SIG_CORR2* pAnaSigCorr2)
{
	volatile ANALOG_ADSAMP_VAR* pDCSigADSamp = &g_DCSigADSamp[u8DCSigNo];
	if((u8DCSigNo < DC_TOTAL_CHN) && ProcUniRecUDatAsWave((uDAT_UNI_REC*)pDCSigADSamp, ANALOG_ADSAMP_BUF_LEN, uCalDatLen)) {
		/* 信号值转化为物理值 */
		*pfVal = CalPValFromDCSigVal2(pDCSigADSamp->fSig_avr, fExtTransRatio, u8DCSigNo, pAnaSigCorr2);
	}
}

/* 使用 ANALOG_SIG_CORR2 进行矫正，由采样值计算物理值 */
float32 CalPValFromDCSigVal2(float32 fSigVal, float32 fExtTransRatio, uint8 u8DCSigNo, ANALOG_SIG_CORR2* pAnaSigCorr2)
{
    /* 计算出未校验值 */
    fSigVal -= cnst_DCSigFixCorr[u8DCSigNo].fZeroCorr;                   /* 固有零漂 */
    fSigVal *= cnst_DCSigFixCorr[u8DCSigNo].fGainCorr;

    if(pAnaSigCorr2 != NULL) {
        pAnaSigCorr2->fNoCorrVal_Now = fSigVal;
        float32 fRange = pAnaSigCorr2->fNoCorrVal_FullRange - pAnaSigCorr2->fNoCorrVal_Zero;
        if(fRange > 0) {   /* 防止除法出错 */
            fSigVal = (fSigVal - pAnaSigCorr2->fNoCorrVal_Zero)*pAnaSigCorr2->fRealVal_FullRange/fRange;
        }
    }
    fSigVal *= fExtTransRatio;

    return fSigVal;
}
#endif

#ifdef TOTAL_ABN_NUM
/*==========================================================================
	以下是保护函数，具体异常位定义由各个产品进行，参见 ABN_***_No
\=========================================================================*/
#ifndef ABN_TIME_INVERSE_START_No       /* 没有使用反时限保护功能 */
    #define ABN_TIME_INVERSE_START_No TOTAL_ABN_NUM
#endif
#define MAX_OV_OR_LS_ABN_NUM        ABN_TIME_INVERSE_START_No
#define MAX_TIME_INVERSE_ABN_NUM    (TOTAL_ABN_NUM - ABN_TIME_INVERSE_START_No)
#define ABN_REC_BUF_DEPTH	        2
typedef struct {
	/* 延时判断Abn:  用于判断过高或者过低的异常,当超限延迟到足够时间,判断为异常,用于滤除系统暂态过程 */
	uint16 uTimer_OvOrLsAbn[MAX_OV_OR_LS_ABN_NUM];
	uint8 u8Tmr_OvOrLsOk[MAX_OV_OR_LS_ABN_NUM];	/* 恢复正常定时器 */
	/* 反时限判断Abn */
	float32 fInverseSum[MAX_TIME_INVERSE_ABN_NUM];

	/* 异常保持部分(保护运行并不依赖这个部分):   		[0]用于收集当前秒异常,[1]是上一时刻时刻;
	g_AnaRes是异常Buf或的结果，即可以保持之前N~2N秒异常，方便其他设备截获异常   */
	uint32 u32AbnormalBuf[ABN_REC_BUF_DEPTH][ABNORMAL_BUF_b32LEN];
    uint16 uTmr_AbnHold_tick;
    uint16 uRsvd;
}ABN_VERIFY;
ABN_VERIFY g_AbnVerify;
#define VERIFY_PAR_OK_ticks						(100/20)

/* 把异常 从g_AbnOut.u32Abnormal 输出到 g_AnaRes.Abnormal，并保持一段时间，以便通讯接口访问 */
BOOL OutputAndHoldAbnormal(uint8 u8CallPeriod_ms)
{
	/* 异常综合输出：输出可以"保持"一阵子，以便通讯接口可以访问 */
	/* 形成当前时间的异常报告 */
	int8 i;
	for(i = ABNORMAL_BUF_b32LEN - 1; i >= 0; i--) {
		g_AbnVerify.u32AbnormalBuf[0][i] |= g_AbnOut.u32Abnormal[i];
		g_AbnOut.u32Abnormal[i] = 0;		/* 消费掉该标志 */
	}
	/* 输出到g_AnaRes */
	BOOL bAbnHappen = FALSE;
	for(i = ABNORMAL_BUF_b32LEN - 1; i >= 0; i--) {
	    uint32 u32Abnormal = g_AbnVerify.u32AbnormalBuf[0][i] | g_AbnVerify.u32AbnormalBuf[1][i];
		bAbnHappen |= (((~g_AnaRes.u32Abnormal[i]) & u32Abnormal) != 0);
		g_AnaRes.u32Abnormal[i] = u32Abnormal;
	}
	/* 形成记录队列 */
	if(g_AbnVerify.uTmr_AbnHold_tick) {
	    g_AbnVerify.uTmr_AbnHold_tick--;
	} else {
		g_AbnVerify.uTmr_AbnHold_tick = 4000/u8CallPeriod_ms;
		for(i = ABNORMAL_BUF_b32LEN - 1; i >= 0; i--) {
			g_AbnVerify.u32AbnormalBuf[1][i] = g_AbnVerify.u32AbnormalBuf[0][i];
			g_AbnVerify.u32AbnormalBuf[0][i] = 0;
		}
	}

	return bAbnHappen;
}

/* 过阈值判断 */
BOOL DelayVerifyOvAbn(uint8 u8AbnNo, BOOL bFuncSw, uint16 uAbnDealFlag, uint32 u32VerifyTicks, float32 fVal, float32 fThr, uint16 uMsgId)
{
	BOOL bAbn = FALSE;
	
	if(u8AbnNo >= MAX_OV_OR_LS_ABN_NUM) {					/* 防止溢出，输入检查 */
	} else if((u32VerifyTicks == 0) || (!bFuncSw)) {		/* 定时为0则关闭 */
		g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] = 0;
		g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo] = 0;
		FinishMsgC(uMsgId, FALSE);
	} else if(fVal > fThr) {
		if(g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] < 0xFFFF) {		
            g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo]++;
		}
		g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo] = VERIFY_PAR_OK_ticks;
		
		UpdateMsgC_WithVal(uMsgId, TRUE, fVal);
		if(g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] >= u32VerifyTicks) { /* 定时结束 */
			g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] = u32VerifyTicks;
			ConfirmMsgC(uMsgId);
			g_AbnOut.uAbnDealFlag |= uAbnDealFlag;
			g_AbnOut.u32Abnormal[u8AbnNo/32] |= (1UL<<(u8AbnNo%32));
			bAbn = TRUE;
		}
	} else {
		if(g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo]) { 		/* 值正常，且正在进行故障定时 */
			g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo]--;
		}
		if(g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo]) {
			g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo]--;
		}
		
		if((g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo] == 0) || (g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] == 0)) {
			FinishMsgC(uMsgId, TRUE);
		}
	}

	return bAbn;
}

/*==========================================================================
| Description	: 不可恢复类异常，例如低压、低周、励磁接地电阻位移等
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2007-8-14
\=========================================================================*/
BOOL DelayVerifyNoRcvrAbn(uint8 u8AbnNo, BOOL bFuncSw, uint16 uAbnDealFlag, uint32 u32VerifyTicks, float32 fVal, float32 fThr, uint16 uMsgId)
{
	BOOL bAbn = FALSE;

	if(u8AbnNo >= MAX_OV_OR_LS_ABN_NUM) {							/* 防止溢出，输入检查 */
	} else if((u32VerifyTicks == 0) || (!bFuncSw)) {				/* 定时为0则关闭 */
		g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] = 0;
		g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo] = 0;
		FinishMsgC(uMsgId, FALSE);
	} else if(fVal < fThr) {
		if(g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] < 0xFFFF) {		
            g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo]++;
		}
		g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo] = VERIFY_PAR_OK_ticks;
		
		UpdateMsgC_WithVal(uMsgId, FALSE, fVal);
		if(g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] >= u32VerifyTicks) {    /* 定时结束 */
			g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] = u32VerifyTicks;
			ConfirmMsgC(uMsgId);
			FinishMsgC(uMsgId, TRUE);
			g_AbnOut.uAbnDealFlag |= uAbnDealFlag;
			g_AbnOut.u32Abnormal[u8AbnNo/32] |= (1UL<<(u8AbnNo%32));
			bAbn = TRUE;
		}
		g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo] = VERIFY_PAR_OK_ticks;
	} else {
		if(g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo]) { 		/* 值正常，且正在进行故障定时 */
			g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo]--;
		}
		if(g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo]) {
			g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo]--;
		}
	}

	return bAbn;
}

/* 比例差分 */
void RatioDiffOvAbn(uint8 u8AbnNo, BOOL bFuncSw, uint16 uAbnDealFlag, float32 fIcd, float32 fIcdqd, float32 fIzd, uint16 uWarnMsgId, uint16 uActMsgId)
{
	if(u8AbnNo >= MAX_OV_OR_LS_ABN_NUM) {						/* 防止溢出，输入检查 */
	} else if((fIcdqd == 0) || (fIzd == 0) || (!bFuncSw)) {			/* 定时为0则关闭 */
		g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] = 0;
		g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo] = 0;
		FinishMsgC(uActMsgId, FALSE);
    } else {
        BOOL bActAbn = FALSE;
        if((fIcd > fIcdqd) && (fIcd > fIzd)) {
            if(fIcd > fIzd) {
                g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo] = VERIFY_PAR_OK_ticks;
                UpdateMsgC_WithVal(uActMsgId, TRUE, fIcd);
                ConfirmMsgC(uActMsgId);
                g_AbnOut.uAbnDealFlag |= uAbnDealFlag;
                g_AbnOut.u32Abnormal[u8AbnNo/32] |= (1UL<<(u8AbnNo%32));
                bActAbn = TRUE;
            } else {
                AddMsgB_WithVal(uWarnMsgId, fIcd);
            }
        }

        if(!bActAbn) {
    		if(g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo]) {
    			g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo]--;
    		} else {
    			FinishMsgC(uActMsgId, TRUE);
    		}
        }
    }
}

/*==========================================================================
| Description	: 复合电压过流
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2007-8-14
\=========================================================================*/
void VerifyCurOvWithVolLow(uint8 u8AbnNo, BOOL bFuncSw, uint16 uAbnDealFlag, uint32 u32VerifyTicks, float32 fCur, float32 fCurThr, float32 fVol, float32 fVolThr, uint16 uMsgId)
{
	if(u8AbnNo >= MAX_OV_OR_LS_ABN_NUM) {					/* 防止溢出，输入检查 */
	} else if((u32VerifyTicks == 0) || (fCurThr == 0) || (!bFuncSw)) {	/* 定时为0则关闭 */
		g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] = 0;
		g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo] = 0;
		FinishMsgC(uMsgId, FALSE);
	} else if((fCur > fCurThr) && (fVol < fVolThr)) {
		if(g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] < 0xFFFF) {		
            g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo]++;
		}
		g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo] = VERIFY_PAR_OK_ticks;
		
		UpdateMsgC_WithVal(uMsgId, TRUE, fCur);
		if(g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] >= u32VerifyTicks) {   /* 定时结束 */
			g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] = u32VerifyTicks;
			ConfirmMsgC(uMsgId);
			g_AbnOut.uAbnDealFlag |= uAbnDealFlag;
            g_AbnOut.u32Abnormal[u8AbnNo/32] |= (1UL<<(u8AbnNo%32));
		}
		g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo] = VERIFY_PAR_OK_ticks;
	} else {
		if(g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo]) { 		/* 值正常，且正在进行故障定时 */
			g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo]--;
		}
		if(g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo]) {
			g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo]--;
		}
		
		if((g_AbnVerify.u8Tmr_OvOrLsOk[u8AbnNo] == 0) || (g_AbnVerify.uTimer_OvOrLsAbn[u8AbnNo] == 0)) {
			FinishMsgC(uMsgId, TRUE);
		}
	}
}

/*==========================================================================
| Description	: 反时限判断
    反时限基本公式为： t = k/((I/Ip)^r - 1), 其中: k时间, I故障电流, Ip保护启动电流, r反时限特性
       Ip = 1倍额定电流. 并且,反时限的启动电流为1.05倍额定电流,即额定电流1.05倍以内反时限不工作
       一般反时限(Alg == 1):
            r = 0.02, k = 0.14tp
            tp = 1时 : 1.2倍38.3秒，1.5倍17.2秒, 2倍10秒, 3倍6.3秒, 4倍5.0秒, 5倍4.3秒, 6倍3.8秒...
       非常反时限(Alg == 2):
            r = 1, k = 13.5tp
            tp = 1时 : 1.2倍67.5秒，1.5倍27秒, 2倍13.5秒, 3倍6.8秒, 4倍4.5秒, 5倍3.4秒, 6倍2.7秒...
        极端反时限(Alg == 3): 对于电动机、发电机转子、变压器等过热保护，一般选择这个
            r = 2，k = 80tp
            tp = 1时 : 1.2倍181.8秒，1.5倍64秒, 2倍26.7秒, 3倍10秒, 4倍5.3秒, 5倍3.3秒, 6倍2.3...
            如果参考杭州南瑞tp取0.05时: 1.2倍9.05秒，1.5倍3.2秒, 2倍1.335秒, 3倍0.5秒, 4倍0.265秒, 5倍0.165秒, 6倍0.115秒
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-9-10
\=========================================================================*/
#if MAX_TIME_INVERSE_ABN_NUM
BOOL InverseTimeVerify(uint8 u8AbnNo, uint16 uAbnDealFlag, uint8 u8Alg, float32 fTp, float32 fI, float32 fIp, uint16 uMsgId)
{
    uint8 u8TimeInvNo = u8AbnNo - ABN_TIME_INVERSE_START_No;
	BOOL bAbn = FALSE;
	
	if(u8TimeInvNo >= MAX_TIME_INVERSE_ABN_NUM) {		/* 防止溢出，输入检查 */
	} else if((u8Alg == 0) || (fTp == 0)) {
		g_AbnVerify.fInverseSum[u8TimeInvNo] = 0;
		FinishMsgC(uMsgId, FALSE);
	} else {
		float32 fFrag = fI/fIp;
		if(u8Alg == 1) {            /* 一般反时限, r = 0.02, k = 0.14tp */
		    fFrag = powf(fFrag, 0.02f) - 1.0f;  /* ((I/Ip)^r - 1) */
		    fTp *= 0.14f;
		} else if(u8Alg == 2) {     /* 非常反时限, r = 1, k = 13.5tp */
            fFrag = fFrag*1 - 1.0f;            /* ((I/Ip)^r - 1) */
            fTp *= 13.5f;
        } else if(u8Alg == 3) {     /* 极端反时限,   r = 2，k = 80tp*/
            fFrag = fFrag*fFrag - 1.0f;        /* ((I/Ip)^r - 1) */
            fTp *= 80.0f;
		}
		g_AbnVerify.fInverseSum[u8TimeInvNo] += fFrag;

		if(fI > fIp*1.05f) {						/* 过电流计算才有意义 */
			UpdateMsgC_WithVal(uMsgId, TRUE, fI);			
			if(g_AbnVerify.fInverseSum[u8TimeInvNo] > fTp) {
				ConfirmMsgC(uMsgId);
				g_AbnOut.uAbnDealFlag |= uAbnDealFlag;
                g_AbnOut.u32Abnormal[u8AbnNo/32] |= (1UL<<(u8AbnNo%32));
				bAbn = TRUE;
				g_AbnVerify.fInverseSum[u8TimeInvNo] = fTp;
			}
		} else if(g_AbnVerify.fInverseSum[u8TimeInvNo] < 0) {
			g_AbnVerify.fInverseSum[u8TimeInvNo] = 0;
			FinishMsgC(uMsgId, TRUE);
		}
	}
	
	return bAbn;
}
#endif
#endif

#if MAX_RELAY_NUM
/*==========================================================================
| Description	: 继电器驱动模块,分为：
	1. 驱动继电器按照规定的时长动作
	2. 继电器动作后效分以下几个阶段：效果反馈/限位反馈(动作结束) 然后 等待机械冲击结束
		反馈分：无反馈、效果反馈(动作信号发出后，延时多长时间会看到效果)、限位反馈(到位后，需要停止继电器动作)
				限位反馈: 不可以用于启停控制(如普通储能)，只能点动控制(如闸阀控制)；
						  且得到信号就会释放继电器，无需接口额外设置(即i32FBTime_InMax_OpReal_10ms < 0)
			
		冲击分：无冲击(其实是不考虑)、效果反馈后
		a. 无开入反馈，无后效，继电器信号(长或短)消失后，无需等待；
		b. 无开入反馈，有机械冲击，继电器需要动作长信号，但已经包括了后效，即继电器仍发信号，但动作其实已经完成
		c. 无开入反馈，继电器信号(长或短)消失后，需要等待一个的固定时间(中等或短)，以免机械冲击；b其实可以包含在c中，把后效时间改为1
		d. 无开入反馈，继电器短信号消失后，需要等待一个较长的可配置时间，以待动作完成，较长时间可以包括机械冲击时间
		e. 有开入反馈(反馈时间可配置)，然后再等一个固定时间(中等或短或零)，以免机械冲击
		f. 有开入反馈(反馈时间可配置)，然后再等一个较长的可配置时间，以待任务完成

	3. 继电器后效评估，分为两个阶段：
		a. 等待效果:如果有DIn,则需要有DIn信号(动作或者消失); 如果没有DIn,则单纯延时
		b. 有DIn情况下，DIn信号后的延时，以免冲击
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-8-29
\=========================================================================*/
typedef struct {
	uint32 u32RelayDat[(MAX_RELAY_NUM+31)/32];	/* Relay数据 */
	uint32 u32Tmr_RelayAct_10ms[MAX_RELAY_NUM];	/* Relay动作定时器 */
	uint32 u32Tmr_RelayEff_10ms[MAX_RELAY_NUM]; /* 继电器动作后效定时器，用于判断动作是否完成Cmplt: 反馈阶段作为反馈定时器; 非反馈阶段，就作为延时定时器 */
	int32 i32FBTime_InMax_OpReal_10ms[MAX_RELAY_NUM];	/* 等待开入信号时间，0:空闲(效果被释放); <0:输入预置值; >0:输出实际时间 */
	uint16 uTmr_RelayRst_10ms[MAX_RELAY_NUM];   /* 继电器释放后定时，主要用于判断继电器是否忙Busy */

	/* 以下用于RelayDat显示 */
	uint32 u32RelayDatKeep[(MAX_RELAY_NUM+31)/32];/* RelayDat保持寄存器 */
	uint8 u8RelayDatKeepTmr_10ms;				/* RelayDat影子定时器,延长RelayDat用于GUI观察时方便 */
	uint8 u8Rsvd;
	uint16 uRsvd;
}DOUT_CTR_VAR;
SECTION(".NOT_ZeroInit") DOUT_CTR_VAR g_RelayCtr;

void InitDOut(void)
{
	/* 初始化配置 */
	/* <NULL> */

	/* 初始化输入接口变量 */
	/* <NULL> */

	/* 初始化输出接口变量 */
	/* <NULL> */

	/* 初始化内部全局变量 */
	if(g_Sys.uRstCount == 0) {
		InitDataWithZero((uint8*)(&g_RelayCtr), sizeof(g_RelayCtr));
	}

	/* 初始化下层模块 */
	/* <NULL> */
	
	/* 启动硬件 */
	/* <NULL> */
}

/*==========================================================================
| Description	: DOut控制
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-8-29
\=========================================================================*/
void DriveDOutTick_100Hz(void)
{
	uint8 u8RelayNo;

	for(u8RelayNo = 0; u8RelayNo < MAX_RELAY_NUM; u8RelayNo++) {
		/* 开入反馈阶段 */
        if(g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo]) {
			/* 开入限位反馈，仅在继电器动作的时候才进行反馈计时 */
			if((g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] < 0)
			    && (cnst_RelayDInTable[u8RelayNo].i8RelayDIn_0N_1A_2D_N1T < 0))
		    {
				if(g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo]) {
					g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo]--;
				}
			} else {	/* 开入效果反馈阶段 或 无反馈阶段，一旦启动，就进行反馈计时 */
				g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo]--;
			}

            /* 在等待反馈的阶段 */
            if((g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] < 0)
                && (cnst_RelayDInTable[u8RelayNo].u8DInNo < MAX_DIN_NUM)
                && cnst_RelayDInTable[u8RelayNo].i8RelayDIn_0N_1A_2D_N1T)
            {
    			/* 开入信号: 是否动作或消失 */
    			BOOL bGetSignal = FALSE;
    			if(cnst_RelayDInTable[u8RelayNo].i8RelayDIn_0N_1A_2D_N1T == RELAY_DIN_DEACTIVE) {
    				if((g_Ctr.u32DinDat & (1UL<<cnst_RelayDInTable[u8RelayNo].u8DInNo)) == 0) {
    					bGetSignal = TRUE;
    				}
    			} else if(g_Ctr.u32DinDat & (1UL<<cnst_RelayDInTable[u8RelayNo].u8DInNo)) {
    				bGetSignal = TRUE;
    			}
    		
    			if(bGetSignal) {	/* 进入继电器复位后的补充延时阶段 */
    				int32 i32FBTime_InMax_OpReal_10ms = 0 - g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo];	/* 取正 */
    				i32FBTime_InMax_OpReal_10ms -= g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo];
    				if(i32FBTime_InMax_OpReal_10ms < 0) { 		            /* 理论上最少应该是1，如果小于0，则是异常 */
    					i32FBTime_InMax_OpReal_10ms = 0;
    				}
    				g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = i32FBTime_InMax_OpReal_10ms;
    				g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] = cnst_RelayDInTable[u8RelayNo].uDOutRstDelay_10ms;  /* 避免到限位后立刻开始下一个动作 */
    			} else if(g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] == 0) {/* 超时，i32FBTime_InMax_OpReal_10ms保持负 */
    				AddMsgB_WithVal(cnst_RelayDInTable[u8RelayNo].uOvTimeMsgID, 0);
    				if(cnst_RelayDInTable[u8RelayNo].i8RelayDIn_0N_1A_2D_N1T < 0) {	/* 限位反馈 */
    					g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo] = 0;				/* 关闭该继电器 */
    				}
    			}
			}
		}

		/* 继电器动作:定时器维护 */
		if(g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo]) {
			/* 具有限位，且限位动作 */
			if((cnst_RelayDInTable[u8RelayNo].i8RelayDIn_0N_1A_2D_N1T < 0)
				&& (cnst_RelayDInTable[u8RelayNo].u8DInNo < MAX_DIN_NUM)
				&& (g_Ctr.u32DinDat & (1UL<<cnst_RelayDInTable[u8RelayNo].u8DInNo)))
			{
				g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo] = 0;
			} else {
				g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo]--;
			}
			
        /* 继电器动作结束后, 等待机械冲击结束阶段,主要避免感官冲击 */
		} else if(g_RelayCtr.uTmr_RelayRst_10ms[u8RelayNo]) {
            g_RelayCtr.uTmr_RelayRst_10ms[u8RelayNo]--;
        }
		
		/* 继电器动作:输出 */
		if(g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo]) {	/* 防止重启引起的开出复位 */
			OnRelay(u8RelayNo);
		} else {
			OffRelay(u8RelayNo);
			g_RelayCtr.u32RelayDat[u8RelayNo/32] &= ~(1UL<<(u8RelayNo%32));
		}
	}

	/* 形成显示 */
	int8 i;
	if(g_RelayCtr.u8RelayDatKeepTmr_10ms == 0) {
		g_RelayCtr.u8RelayDatKeepTmr_10ms = 20;
		for(i = 0; i < (MAX_RELAY_NUM+31)/32; i++) {
			g_Ctr.u32RelayDat[i] = g_RelayCtr.u32RelayDatKeep[i];
			g_RelayCtr.u32RelayDatKeep[i] = g_RelayCtr.u32RelayDat[i];
		}
	} else {
		for(i = 0; i < (MAX_RELAY_NUM+31)/32; i++) {
			g_RelayCtr.u32RelayDatKeep[i] |= g_RelayCtr.u32RelayDat[i];
			g_Ctr.u32RelayDat[i] |= g_RelayCtr.u32RelayDat[i];
		}
		g_RelayCtr.u8RelayDatKeepTmr_10ms--;
	}
}

/*==========================================================================
| Description	: 继电器简单动作接口
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2015-8-29
\=========================================================================*/
static void ClearMutexRelayAndOppEff_MustSwiDisable(uint8 u8RelayNo);
/* 动作某个继电器一定时间 */
void ActRelay(uint8 u8RelayNo, uint32 u32ActTime_10ms)
{
	if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
	} else if(u32ActTime_10ms == 0) {
	} else if(u8RelayNo >= MAX_RELAY_NUM) {

	/* 有限位反馈，且已经动作 */
	} else if((cnst_RelayDInTable[u8RelayNo].i8RelayDIn_0N_1A_2D_N1T < 0)
				&& (cnst_RelayDInTable[u8RelayNo].u8DInNo < MAX_DIN_NUM) 
				&& (g_Ctr.u32DinDat & (1UL<<cnst_RelayDInTable[u8RelayNo].u8DInNo)))
	{
	} else {
		Swi_disable();
		/* 清除互斥(如开增对开减)，相关(如合闸对储能)的继电器后效 */
		ClearMutexRelayAndOppEff_MustSwiDisable(u8RelayNo);

		/* 继电器动作 */
		g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo] = u32ActTime_10ms;
		g_RelayCtr.u32RelayDat[u8RelayNo/32] |= (1UL<<(u8RelayNo%32));
		OnRelay(u8RelayNo);

		/* 自身后效部分 */
		g_RelayCtr.uTmr_RelayRst_10ms[u8RelayNo] = cnst_RelayDInTable[u8RelayNo].uDOutRstDelay_10ms;
		g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = u32ActTime_10ms;
		g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] = u32ActTime_10ms;
		Swi_enable();
	}
}

/* 释放所有开关量输出 */
void RstAllRelay(void)
{
	uint8 u8RelayNo;
	
	Swi_disable();
	for(u8RelayNo = 0; u8RelayNo < MAX_RELAY_NUM; u8RelayNo++) {
		OffRelay(u8RelayNo);
		g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo] = 0;
	}
	Swi_enable();
}

/* 释放某个继电器 */
void RstRelay(uint8 u8RelayNo)
{
	if(u8RelayNo < MAX_RELAY_NUM) {
		OffRelay(u8RelayNo);
		g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo] = 0;
	}
}

/* 检查对应的RelayNo是否已经动作,已经动作则返回TRUE, 否则返回FALSE */
BOOL ChkRelayAct(uint8 u8RelayNo)
{
	if((u8RelayNo < MAX_RELAY_NUM) && g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo]) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/* 获得Relay的定时器 */
uint32 GetRelayActTmr(uint8 u8RelayNo)
{
	if(u8RelayNo < MAX_RELAY_NUM) {
		return g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo];
	} else {
		return 0;
	}
}

/*==========================================================================
| Description	: 继电器效果动作接口
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2019-4-17
\=========================================================================*/
/* 带延时功能，即继电器复位后，还需要一定延时才算: 不Busy */
ACT_RELAY_RES ActRelayWithRstDelay(uint8 u8RelayNo, uint32 u32ActTime_10ms, uint32 u32RstDelay_10ms)
{
	if(u8RelayNo >= MAX_RELAY_NUM) {    /* 输入检查 */
		return ACT_RELAY_NUM_INVALID;
	} else if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
		return ACT_RELAY_SAVE_URGENT;
	} else if(u32ActTime_10ms == 0) {
		return ACT_RELAY_TIME_ZERO;

	/* 有限位反馈，且已经动作 */
	} else if((cnst_RelayDInTable[u8RelayNo].i8RelayDIn_0N_1A_2D_N1T < 0)
				&& (cnst_RelayDInTable[u8RelayNo].u8DInNo < MAX_DIN_NUM) 
				&& (g_Ctr.u32DinDat & (1UL<<cnst_RelayDInTable[u8RelayNo].u8DInNo)))
	{
		return ACT_RELAY_THR;
	} else {
		Swi_disable();
		/* 清除对立的继电器后效，如合闸对储能的影响 */
		ClearMutexRelayAndOppEff_MustSwiDisable(u8RelayNo);
		
		/* 继电器动作 */
		g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo] = u32ActTime_10ms;
		g_RelayCtr.u32RelayDat[u8RelayNo/32] |= (1UL<<(u8RelayNo%32));
		OnRelay(u8RelayNo);

		/* 自身后效部分 */
		g_RelayCtr.uTmr_RelayRst_10ms[u8RelayNo] = u32RstDelay_10ms;			/* 启动定时器 */
		g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = u32ActTime_10ms;
		g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] = u32ActTime_10ms;
		Swi_enable();
		return ACT_RELAY_SUC;
	}
}

/* 带延时且检查开入功能 */
ACT_RELAY_RES ActRelayWithFeedBack(uint8 u8RelayNo, uint32 u32ActTime_10ms, uint32 u32MaxFBTime_10ms)
{
	if(u8RelayNo >= MAX_RELAY_NUM) {    /* 输入检查 */
		return ACT_RELAY_NUM_INVALID;
	}

	BOOL bNOFdBack = (u32MaxFBTime_10ms == 0) 
					|| (cnst_RelayDInTable[u8RelayNo].i8RelayDIn_0N_1A_2D_N1T == 0) 
					|| (cnst_RelayDInTable[u8RelayNo].u8DInNo >= MAX_DIN_NUM);
	if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
		return ACT_RELAY_SAVE_URGENT;
	} else if((u32ActTime_10ms == 0) && bNOFdBack) {
		return ACT_RELAY_TIME_ZERO;
	} else {
		ACT_RELAY_RES ActRelayRes = ACT_RELAY_SUC;
		Swi_disable();
		if(u32ActTime_10ms == 0) {		/* 纯粹检查反馈 */
			g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] = 0;
			if(g_Ctr.u32DinDat & (1UL<<cnst_RelayDInTable[u8RelayNo].u8DInNo)) {
				g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = 1;	/* 标记反馈成功 */
				ActRelayRes = ACT_RELAY_THR;
			} else {
				g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = -1;	/* 标记反馈超时 */
				ActRelayRes = ACT_RELAY_TIME_ZERO;
			}
			
		/* 有限位反馈，且已经动作 */
		} else if(u32MaxFBTime_10ms && (cnst_RelayDInTable[u8RelayNo].i8RelayDIn_0N_1A_2D_N1T < 0)
				&& (cnst_RelayDInTable[u8RelayNo].u8DInNo < MAX_DIN_NUM) 
				&& (g_Ctr.u32DinDat & (1UL<<cnst_RelayDInTable[u8RelayNo].u8DInNo)))
		{
			ActRelayRes = ACT_RELAY_THR;
		} else {
			/* 清除对立的继电器后效，如合闸对储能的影响 */
			ClearMutexRelayAndOppEff_MustSwiDisable(u8RelayNo);
			
			/* 继电器动作 */
			g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo] = u32ActTime_10ms;
			g_RelayCtr.u32RelayDat[u8RelayNo/32] |= (1UL<<(u8RelayNo%32));
			OnRelay(u8RelayNo);

			/* 自身后效部分 */
            g_RelayCtr.uTmr_RelayRst_10ms[u8RelayNo] = cnst_RelayDInTable[u8RelayNo].uDOutRstDelay_10ms;
			if(bNOFdBack) {			/* 无反馈 */
				g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = u32ActTime_10ms;
                g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] = u32ActTime_10ms;
			} else if((g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] == 0)     /* 之前的反馈定时已经结束 */
                || (g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] != 0 - u32MaxFBTime_10ms))   /* 限值发生修改 */
			{    
				g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = 0 - u32MaxFBTime_10ms;
                g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] = u32MaxFBTime_10ms;
			}
			ActRelayRes = ACT_RELAY_SUC;
		}
		Swi_enable();
		return ActRelayRes;
	}
}

/* 以Tfkl格式配置的带延时且检查开入功能，如果u32Conf_Tfkl.T==0，仅检查反馈
	bDefaultKeep_Ext0_Int1: 默认保持模式，例如:储能默认外部自保持, 闸阀默认需要依赖控制器保持
	bIniAct: 初次动作，例如: 开机时，闸阀第一次开是初次动作; 开到中间位置达到额定频率，停下，并上网再开闸阀就不是初始动作
	         主要用于闸阀，可能分多次动作的情况 */
ACT_RELAY_RES ActRelayWithTfkl(uint8 u8RelayNo, uint32 u32Conf_Tfkl, BOOL bDefaultKeep_Ext0_Int1, BOOL bIniAct)
{
	if(u8RelayNo >= MAX_RELAY_NUM) {    /* 输入检查 */
		return ACT_RELAY_NUM_INVALID;
	}
	
	uint32 u32ActTime_10ms = GET_T_FKL_ActTime(u32Conf_Tfkl)*100;
	BOOL bNOFdBack = (!GET_T_FKL_HaveFdBak(u32Conf_Tfkl)) 
					|| (cnst_RelayDInTable[u8RelayNo].i8RelayDIn_0N_1A_2D_N1T == 0) 
					|| (cnst_RelayDInTable[u8RelayNo].u8DInNo >= MAX_DIN_NUM);
	if(g_DataAcsIntf.tSaveUrgent_0Idle_pReq_nCmplt > 0) {
		return ACT_RELAY_SAVE_URGENT;
	} else if(u32ActTime_10ms == 0) {      /* 纯粹检查反馈 */
		ACT_RELAY_RES ActRelayRes = ACT_RELAY_TIME_ZERO;
		Swi_disable();
        g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] = 0;
        if((cnst_RelayDInTable[u8RelayNo].u8DInNo >= MAX_DIN_NUM)      /* 没有反馈 */
            || (cnst_RelayDInTable[u8RelayNo].i8RelayDIn_0N_1A_2D_N1T == 0))
        {
            g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = 0;
        } else if(g_Ctr.u32DinDat & (1UL<<cnst_RelayDInTable[u8RelayNo].u8DInNo)) {
            g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = 1;      /* 标记反馈成功 */
            ActRelayRes = ACT_RELAY_THR;
        } else if(GET_T_FKL_HaveFdBak(u32Conf_Tfkl)) {
            g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = -1;     /* 标记反馈超时 */
            ActRelayRes = ACT_RELAY_TIME_OUT;
        } else {
            g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = 0;
        }
		Swi_enable();
		return ActRelayRes;
	} else if((!bIniAct) && ChkRelayEff_TimeOut(u8RelayNo)) {   /* 续动的话，需要检查之前是否超时 */
		return ACT_RELAY_TIME_OUT;
	} else {
		ACT_RELAY_RES ActRelayRes = ACT_RELAY_SUC;
		Swi_disable();
		/* 有限位反馈，且已经动作 */
		if(u32ActTime_10ms && (cnst_RelayDInTable[u8RelayNo].i8RelayDIn_0N_1A_2D_N1T < 0)
			&& (cnst_RelayDInTable[u8RelayNo].u8DInNo < MAX_DIN_NUM) 
			&& (g_Ctr.u32DinDat & (1UL<<cnst_RelayDInTable[u8RelayNo].u8DInNo)))
		{
			ActRelayRes = ACT_RELAY_THR;
		} else {
			/* 清除对立的继电器后效，如合闸对储能的影响 */
			ClearMutexRelayAndOppEff_MustSwiDisable(u8RelayNo);
			
			/* 继电器动作 */
			BOOL bRelayActLong = GET_T_FKL_KeepFlag(u32Conf_Tfkl) ^ bDefaultKeep_Ext0_Int1;
			if(bRelayActLong) {
				g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo] = u32ActTime_10ms;
			} else {
				g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo] = 50;
			}
			g_RelayCtr.u32RelayDat[u8RelayNo/32] |= (1UL<<(u8RelayNo%32));
			OnRelay(u8RelayNo);
			
			/* 自身后效部分 */
            g_RelayCtr.uTmr_RelayRst_10ms[u8RelayNo] = cnst_RelayDInTable[u8RelayNo].uDOutRstDelay_10ms;
			if(bNOFdBack) {			/* 无反馈, 则u32Time_10ms != 0 */
				g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = u32ActTime_10ms;
                g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] = u32ActTime_10ms;
			} else if((g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] != 0 - u32ActTime_10ms)    /* 限位反馈，且限值发生修改 */
					|| (g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] == 0)                            /* 之前的反馈定时已经结束 */
					|| bIniAct)                                                                     /* 限位反馈，初次动作 */
			{
				g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = 0 - u32ActTime_10ms;
				g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] = u32ActTime_10ms;
			}
			ActRelayRes = ACT_RELAY_SUC;
		}
		Swi_enable();
		return ActRelayRes;
	}
}

/* 清除对应继电器动作与后效，输入合闸继电器，复位分闸动作，清除储能后效(包括开入反馈) */
static void ClearMutexRelayAndOppEff_MustSwiDisable(uint8 u8RelayNo)
{
	if(cnst_RelayDInTable[u8RelayNo].u8MutexRelayNo < MAX_RELAY_NUM) {
    	uint8 u8MutexRelayNo = cnst_RelayDInTable[u8RelayNo].u8MutexRelayNo;
    	OffRelay(u8MutexRelayNo);                               /* 赶紧复位，后面就要动作了 */
        g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8MutexRelayNo] = 0;
        g_RelayCtr.u32Tmr_RelayEff_10ms[u8MutexRelayNo] = 0;
		if(g_RelayCtr.u32Tmr_RelayAct_10ms[u8MutexRelayNo]) {	/* 如果该继电器在动作中，则需要等待稳定 */
			g_RelayCtr.u32Tmr_RelayAct_10ms[u8MutexRelayNo] = 0;
			g_RelayCtr.uTmr_RelayRst_10ms[u8MutexRelayNo] = cnst_RelayDInTable[u8MutexRelayNo].uDOutRstDelay_10ms;
		}
    }
	if(cnst_RelayDInTable[u8RelayNo].u8ClearEffRelayNo < MAX_RELAY_NUM) {
		uint8 u8ClearEffRelayNo = cnst_RelayDInTable[u8RelayNo].u8ClearEffRelayNo;
		g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8ClearEffRelayNo] = 0;
        g_RelayCtr.u32Tmr_RelayEff_10ms[u8ClearEffRelayNo] = 0;
	}
}

/* 清除继电器动作后效，例如合闸后，储能即释放了 */
void ClearRelayEff(uint8 u8RelayNo)
{
	if(u8RelayNo < MAX_RELAY_NUM) {
		Swi_disable();
        /* 如果该继电器在动作中，则需要等待稳定; 否则放弃等待稳定 */
        if(g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo] == 0) {
            g_RelayCtr.uTmr_RelayRst_10ms[u8RelayNo] = 0;
        }
        g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] = 0;
		g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] = 0;
		Swi_enable();
	}
}

/*==========================================================================
| Description	: 检查RelayNo动作效果，
    (u32Tmr_RelayAct_10ms == 0) && (uTmr_RelayRst_10ms[u8RelayNo] == 0) 代表继电器动作完成(继电器释放、且机械冲击已过)
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2019-4-20
\=========================================================================*/
/* 继电器忙中，包括动作期、释放后稳定期 */
BOOL ChkRelayEff_Busy(uint8 u8RelayNo)
{
	return (u8RelayNo < MAX_RELAY_NUM) 
	        && (g_RelayCtr.uTmr_RelayRst_10ms[u8RelayNo] || g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo]);
}
/* 继电器效果已经被释放，如断路器合闸后储能效果就被释放，继电器动作超时也算没有效果 */
BOOL ChkRelayEff_Clear(uint8 u8RelayNo)
{
	return (u8RelayNo < MAX_RELAY_NUM) 
	        && (g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] == 0) 
	        && (g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] <= 0);
}
/* 继电器效果完成，如断路器完成储能 */
BOOL ChkRelayEff_Cmplt(uint8 u8RelayNo)
{
	return (u8RelayNo < MAX_RELAY_NUM) 
	        && (g_RelayCtr.uTmr_RelayRst_10ms[u8RelayNo] == 0) 
            && (g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo] == 0) 
            && (g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] == 0)
	        && (g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] > 0);
}
/* 继电器动作超时:效果反馈或限位反馈 超时 */
BOOL ChkRelayEff_TimeOut(uint8 u8RelayNo)
{
	return (u8RelayNo < MAX_RELAY_NUM) 
	        && (g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo] == 0) 
	        && (g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] < 0);
}
/* 	>0:到达效果(即DIn产生改变)耗时
	 0: 说明还没有确定性结果，即未到限位
	-1: 空闲
	-2: 超时，即时间用完
	-3: 输入错误 		*/
int32 GetRelayEff(uint8 u8RelayNo)
{
	if(u8RelayNo >= MAX_RELAY_NUM) {
		return -1;
    /* 定时器还在运行 */
	} else if(g_RelayCtr.u32Tmr_RelayAct_10ms[u8RelayNo] 
	        || g_RelayCtr.uTmr_RelayRst_10ms[u8RelayNo]
	        || g_RelayCtr.u32Tmr_RelayEff_10ms[u8RelayNo])
	{
		return 0;
	} else if(g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] == 0) {	/* 空闲 */
		return -1;
	} else if(g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo] < 0) {	/* 定时器已经停止，但是该等待时间依然没有修正，说明超时 */
		return -2;
	} else {
		return g_RelayCtr.i32FBTime_InMax_OpReal_10ms[u8RelayNo];		/* 如有开入反馈,则到开入反馈的时间; 否则就是输入的时间 */
	}
}
#endif

/***************************************************************************
		RTC模块
***************************************************************************/
#define MAX_IRIGb_DISC_TIME_ms	10000
#define IS_LEAP_YEAR(u8Year)	(u8Year%4 == 0)
const uint16 cnst_uDayFromLeapYear[] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
const uint16 cnst_uDayFromNonLeapYear[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
/*==========================================================================
| Description	: 计算RTC的秒, 以日历时间为表达方式
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2016-12-20
\=========================================================================*/
uint32 CalRTCSecondsByDate(uint8 u8Year, uint8 u8Month, uint8 u8Day, uint8 u8Hour, uint8 u8Min, uint8 u8Sec)
{
	uint32 u32Seconds;
	
	u32Seconds = (uint32)(u8Year%100)*365UL + (uint32)((u8Year-1)>>2);
	if(IS_LEAP_YEAR(u8Year)) {
		u32Seconds += cnst_uDayFromLeapYear[u8Month - 1];
	} else {
		u32Seconds += cnst_uDayFromNonLeapYear[u8Month - 1];
	}
	u32Seconds += (uint32)(u8Day-1);
	u32Seconds *= 24UL;
	u32Seconds += (uint32)u8Hour;
	u32Seconds *= 60UL;
	u32Seconds += (uint32)u8Min;
	u32Seconds *= 60UL;
	u32Seconds += (uint32)u8Sec;
	u32Seconds += 3600UL*24UL*(365*30 + 8);	/* 加上从1970.1.1 - 2000.1.1时间 */
	return u32Seconds;
}

/*==========================================================================
| Description	: 把RTC秒计数解析成年月日、时分秒数据
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2019-4-25
\=========================================================================*/
void CalDateByRTCSeconds(uint32 u32Seconds, REAL_TIME_VAR* pRealTime)
{
	uint16* puDayFromNewYear;
	uint32 u32DayOfYear;
	int16 i;
	
	if(u32Seconds < 3600UL*24UL*(365UL*31UL + 8)) {	/* 扣除1970.1.1 - 2001.1.1，值得注意的是:2000不是闰年 */
		u32Seconds = 0;
	} else {
		u32Seconds -= 3600UL*24UL*(365UL*31UL + 8);
	}

	/* 计算年 */
	pRealTime->u8Year = u32Seconds/((4UL*365UL + 1UL)*24UL*3600UL);
	u32Seconds = u32Seconds%((4UL*365UL + 1UL)*24UL*3600UL);
	pRealTime->u8Year = pRealTime->u8Year*4 + 1;
	if(u32Seconds >= 3600UL*24UL*365UL*3) {
		pRealTime->u8Year += 3;
		u32Seconds -= 3600UL*24UL*365UL*3;
	} else {
		pRealTime->u8Year += u32Seconds/(3600UL*24UL*365UL);
		u32Seconds = u32Seconds%(3600UL*24UL*365UL);
	}
	u32DayOfYear = u32Seconds/(24UL*3600UL);
	u32Seconds = u32Seconds%(24UL*3600UL);
	
	/* 计算月、日 */
	if(IS_LEAP_YEAR(pRealTime->u8Year)) {
		puDayFromNewYear = (uint16*)cnst_uDayFromLeapYear;
	} else {
		puDayFromNewYear = (uint16*)cnst_uDayFromNonLeapYear;
	}
	for(i = 1; i < 12; i++) {
		if(u32DayOfYear < puDayFromNewYear[i]) {
			break;
		}
	}
	pRealTime->u8Month = i;
	pRealTime->u8Day = u32DayOfYear - puDayFromNewYear[i-1] + 1;

	/* 计算时分秒 */
	pRealTime->u8Hour = u32Seconds/3600;
	u32Seconds = u32Seconds%3600;
	pRealTime->u8Minute = u32Seconds/60;
	pRealTime->u8Second = u32Seconds%60;
}

/*==========================================================================
| Description	: 从2000/00/00开始计算到指定日期的天数
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2020-11-14
\=========================================================================*/
uint16 CalDaysFromOrigin(uint8 u8Year, uint8 u8Month, uint8 u8Day)
{
	const uint16 cnst_DayFromNewYear[] = {0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
	uint16 uDaysFromOrigin = (uint32)u8Year*365UL + (uint32)cnst_DayFromNewYear[u8Month] + u8Day - 1;
	/* 闰年处理 */
	if(u8Year > 4) {
		uDaysFromOrigin += (u8Year - 4)/4;
	}
	if((u8Year%4 == 0) && (u8Month > 2)) {
		uDaysFromOrigin += 1;
	}
	return uDaysFromOrigin;
}

/* 调整堆节点以使其满足大顶堆的要求，完全二叉树一父二子节点的数组索引中，只要已知其中一个，另外两个可以直接计算出来。
 * pAcqDataStart: AcqData数组
 * uNodeIndex:以此索引的节点做顶节点，检查子节点的是否都小于等于此节点，子孙节点按需检查
 * uBufLen:检查堆数组长度
 * u8SortItem:排序的项 */
void adjustF32_ForSort(float32* pF32, uint16 uNodeIndex, uint16 uBufLen)
{
	float32 fTopNode = pF32[uNodeIndex];
	int16 i;
	for(i = 2*uNodeIndex + 1; i < uBufLen; i = i*2 + 1) { // 沿节点值较大的子结点向下筛选（根据完全二叉树性质，ix2+1是左子节点，ix2+2是右子节点）
		if((i+1 < uBufLen) && (pF32[i] < pF32[i + 1])) {	//如果子节点索引未超上限，且比较两个子节点
			i++; // 如果右子节点较大，则将i更新为较大的下标
		}
		if(fTopNode >= pF32[i]) {	//再和子节点最大值比较
			break;	//如果当前节点已经是最大的则调整任务提前完成。
		} else {
			pF32[uNodeIndex] = pF32[i];	//否则子节点值上移
			uNodeIndex = i;	// 交换节点索引，以实现往下一级节点检查
		}
	}
	pF32[uNodeIndex] = fTopNode;	//将节点值放入最终确定的位置
}

/* 堆排序
	第一步：先n个元素的无序序列，构建成大顶堆
	第二步：将根节点与最后一个元素交换位置，（将最大元素"沉"到数组末端）
	第三步：交换过后可能不再满足大顶堆的条件，所以需要将剩下的n-1个元素重新构建成大顶堆
	第四步：重复第二步、第三步直到整个数组排序完成
 * pAcqDataStart: AcqData数组
 * uBufLen:检查堆数组长度
 * u8SortItem:排序的项 */
void sortF32Array(float32* pF32, uint16 uBufLen)
{
	int16 i;

	// 构建一个大顶堆
	for(i = uBufLen/2 - 1; i >= 0; i--) {	//从最后一个非子结点开始扫描（叶节点无子，所以不需要扫描）,确保全扫描完就是一个合法的大顶堆（所有父节点大于等于子节点）
		adjustF32_ForSort(pF32, i, uBufLen);
	}

	// 将堆顶记录和当前未经排序子序列的最后一个记录交换
	for(i = uBufLen - 1; i >= 0; i--) {
		// 交换堆顶和尾(i-1)节点值
		float32 fTmp = pF32[0];
		pF32[0] = pF32[i];
		pF32[i] = fTmp;
		// 将a中前i-1个记录重新调整为合法的大顶堆
		adjustF32_ForSort(pF32, 0, i);
	}
}

/* 获取队列大小 */
uint32 GetF32QueueSize(F32_QUEUE_t *pQueue)
{
	return ((pQueue->uEnd + pQueue->uMaxLen - pQueue->uStart) % pQueue->uMaxLen);
}

/* f32队列初始化 */
BOOL InitF32Queue(F32_QUEUE_t *pQueue, uint16 uLen)
{
	pQueue->pFData = (float32 *)malloc(sizeof(float32) * uLen);
	if(pQueue->pFData == NULL) {
		return FALSE;
	}
	pQueue->uEnd = 0;
	pQueue->uStart = 0;
	pQueue->uMaxLen = uLen;
	return TRUE;
}

/* f32入队 */
void F32QueueEnter(F32_QUEUE_t *pQueue, float32 fVal)
{
	if(((pQueue->uEnd + 1) % pQueue->uMaxLen) == pQueue->uStart) {	/* 队满 */
		pQueue->uStart = (pQueue->uStart + 1) % pQueue->uMaxLen;
	}
	pQueue->pFData[pQueue->uEnd] = fVal;
	pQueue->uEnd = (pQueue->uEnd + 1) % pQueue->uMaxLen;
}

/* 清空队列 */
void F32QueueClear(F32_QUEUE_t *pQueue)
{
	pQueue->uEnd = 0;
	pQueue->uStart = 0;
}

/* 获取F32队列中最大的元素 */
float32 F32QueueGetMaxElement(F32_QUEUE_t *pQueue)
{
	uint16 uQueueLen = GetF32QueueSize(pQueue);
	if(uQueueLen == 0) {
		return 0.0f;
	}
	float32 fMaxVal = pQueue->pFData[pQueue->uStart];
	for(uint16 i = 1; i < uQueueLen; i++) {
		float32 fVal = pQueue->pFData[(pQueue->uStart + i) % pQueue->uMaxLen];
		if(fMaxVal < fVal) {
			fMaxVal = fVal;
		}
	}
	return fMaxVal;
}

/**
 * 使用线性回归计算队列中元素的斜率, 横坐标必须是均匀分布
 * 参数2： 横坐标分布间隔
 * 参数3： 队列中最小元素个数
 */
float GetF32QueueSlope(F32_QUEUE_t *pQueue, float32 fXInterval, uint16 uMinSize)
{
    uint16 uQueueLen = GetF32QueueSize(pQueue);
    if(uQueueLen < uMinSize) {
    	return 0.0f;
    }

    float32 fSumX = 0, fSumY = 0, fSumXY = 0, fSumX2 = 0;
    for(int i = 0; i < uQueueLen; i++) {
        float fX = i * fXInterval;  /* x轴坐标 */
        float fY = pQueue->pFData[(pQueue->uStart + i) % pQueue->uMaxLen];
        fSumX  += fX;
        fSumY  += fY;
        fSumXY += fX * fY;
        fSumX2 += fX * fX;
    }

    float fDenominator = (uQueueLen * fSumX2 - fSumX * fSumX);
    if(fDenominator == 0) {
    	return 0.0f;
    }

    return ((uQueueLen * fSumXY - fSumX * fSumY) / fDenominator);
}
/******************************** FILE END ********************************/
