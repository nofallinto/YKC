
#ifndef __BMP280_C_
#define __BMP280_C_

#include "BMP280.h"
#include <cmsis_os.h>

BOOL WriteByteToBmp280(uint8 u8DataAddr, uint8 u8Data, uint8 u8Step);
inline BOOL WriteByteToBmp280(uint8 u8DataAddr, uint8 u8Data, uint8 u8Step)
{
	uint8 u8Buf[2] = {u8DataAddr, u8Data};
	return WriteToI2C(I2C_PORT_BMP280, I2C_DEV_BMP280, u8Buf, 2, 0);
}

BOOL ReadDatFromBmp280(uint8 u8DataAddr, uint8* pU8, uint16 uBLen, uint8 u8Step);
inline BOOL ReadDatFromBmp280(uint8 u8DataAddr, uint8* pU8, uint16 uBLen, uint8 u8Step)
{
	return WriteToI2C(I2C_PORT_BMP280, I2C_DEV_BMP280, &u8DataAddr, 1, 0)
			&& ReadFromI2C(I2C_PORT_BMP280, I2C_DEV_BMP280, pU8, uBLen, 0);
}

/* 初始化 */
BOOL Bmp280Init(void)
{
	OpenI2CComm(I2C_PORT_BMP280);

	/* 下发初始化代码 */
	WriteByteToBmp280(0xE0, 0xB6, 0); /* 清除状态 */
	osDelay(OS_TICK_KHz*200);
	Bmp280Wake();
	uint8 u8Val = 0;
	if(ReadDatFromBmp280(0xD0, &u8Val, 1, 0) && (u8Val == 0x58)
			&& WriteByteToBmp280(0xF4, 0x55, 0)
			&& WriteByteToBmp280(0xF5, 0x10, 0)
			&& ReadDatFromBmp280(0x88, (uint8*)&g_Bmp280Comm.uDigT1, 2, 0)
			&& ReadDatFromBmp280(0x8A, (uint8*)&g_Bmp280Comm.iDigT2, 2, 0)
			&& ReadDatFromBmp280(0x8C, (uint8*)&g_Bmp280Comm.iDigT3, 2, 0)
			&& ReadDatFromBmp280(0x8E, (uint8*)&g_Bmp280Comm.uDigP1, 2, 0)
			&& ReadDatFromBmp280(0x90, (uint8*)&g_Bmp280Comm.iDigP2, 2, 0)
			&& ReadDatFromBmp280(0x92, (uint8*)&g_Bmp280Comm.iDigP3, 2, 0)
			&& ReadDatFromBmp280(0x94, (uint8*)&g_Bmp280Comm.iDigP4, 2, 0)
			&& ReadDatFromBmp280(0x96, (uint8*)&g_Bmp280Comm.iDigP5, 2, 0)
			&& ReadDatFromBmp280(0x98, (uint8*)&g_Bmp280Comm.iDigP6, 2, 0)
			&& ReadDatFromBmp280(0x9A, (uint8*)&g_Bmp280Comm.iDigP7, 2, 0)
			&& ReadDatFromBmp280(0x9C, (uint8*)&g_Bmp280Comm.iDigP8, 2, 0)
			&& ReadDatFromBmp280(0x9E, (uint8*)&g_Bmp280Comm.iDigP9, 2, 0))
	{
		osDelay(OS_TICK_KHz*200);
		return TRUE;
	} else {
		return FALSE;
	}
}

/* 读取气压和温度 */
BOOL Bmp280GetAirAndTemp(void)
{
	if(!WriteByteToBmp280(0xf4, 0x56, 1)) {
		return FALSE;
	}
	
	int32_t i32Adc_T = 0;
	int32_t i32Adc_P = 0;
	if(ReadDatFromBmp280(0xFA, (uint8*)&i32Adc_T, 3, 0)
		&& ReadDatFromBmp280(0xF7, (uint8*)&i32Adc_P, 3, 0))
	{ /* 0xFA 0xFB 0xFC		0xF7 0xF8 0xF9 */
		i32Adc_T = (int32) (((i32Adc_T & 0xFF) << 12) | ((i32Adc_T & 0xFF00) >> 4) | ((i32Adc_T & 0xFF0000) >> 20));
		i32Adc_P = (int32) (((i32Adc_P & 0xFF) << 12) | ((i32Adc_P & 0xFF00) >> 4) | ((i32Adc_P & 0xFF0000) >> 20));
		if(i32Adc_P && i32Adc_T) {
			int32_t i32Var1 = ((((i32Adc_T >> 3) - ((int32_t) g_Bmp280Comm.uDigT1 << 1))) * ((int32_t) g_Bmp280Comm.iDigT2)) >> 11;
			int32_t i32Var2 = (((((i32Adc_T >> 4) - ((int32_t) g_Bmp280Comm.uDigT1)) * ((i32Adc_T >> 4) - ((int32_t) g_Bmp280Comm.uDigT1))) >> 12) * ((int32_t) g_Bmp280Comm.iDigT3)) >> 14;
			int32_t t_fine = i32Var1 + i32Var2;

			float32 T = (t_fine * 5 + 128) >> 8;
			g_Bmp280Comm.fTemperature = T / 100;

			int64 i64Var1, i64Var2, i64Press;
			i64Var1 = ((int64) t_fine) - 128000;
			i64Var2 = i64Var1 * i64Var1 * (int64) g_Bmp280Comm.iDigP6;
			i64Var2 = i64Var2 + ((i64Var1 * (int64) g_Bmp280Comm.iDigP5) << 17);
			i64Var2 = i64Var2 + (((int64) g_Bmp280Comm.iDigP4) << 35);
			i64Var1 = ((i64Var1 * i64Var1 * (int64) g_Bmp280Comm.iDigP3) >> 8)
					+ ((i64Var1 * (int64) g_Bmp280Comm.iDigP2) << 12);
			i64Var1 = (((((int64) 1) << 47) + i64Var1)) * ((int64) g_Bmp280Comm.uDigP1) >> 33;

			if(i64Var1) {		/* avoid exception caused by division by zero */
				i64Press = 1048576 - i32Adc_P;
				i64Press = (((i64Press << 31) - i64Var2) * 3125) / i64Var1;
				i64Var1 = (((int64) g_Bmp280Comm.iDigP9) * (i64Press >> 13) * (i64Press >> 13)) >> 25;
				i64Var2 = (((int64) g_Bmp280Comm.iDigP8) * i64Press) >> 19;

				i64Press = ((i64Press + i64Var1 + i64Var2) >> 8) + (((int64) g_Bmp280Comm.iDigP7) << 4);
				g_Bmp280Comm.fAirPressure = (float32) i64Press / 256;
				return TRUE;
			}
		}
	}
	return FALSE;
}

#if 0
/* 睡眠 */
BOOL Bmp280Sleep(void)
{
	uint8 u8Val = 0;
	if(Bmp280ReadByte(0xf4, &u8Val)) {
		u8Val &= 0xff << 2;
		if(WriteDataToI2C(I2C_PORT_BMP280, 0xf4, u8Val)) {
			return TRUE;
		}
	}
	return FALSE;
}
#endif

/* 唤醒 */
BOOL Bmp280Wake(void)
{
	uint8 u8Val = 0;
	if(ReadDatFromBmp280(0xf4, &u8Val, 1, 0)) {
		u8Val &= 0xff << 2;
		u8Val |= 3;
		if(WriteByteToBmp280(0xf4, u8Val, 0)) {
			return TRUE;
		}
	}
	return FALSE;
}

#endif
