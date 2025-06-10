#ifndef __BMP280_H_
#define __BMP280_H_

#include <cmsis_os.h>
#include <math.h>
#include "GlobalVar.h"

#ifdef __BMP280_C_
#define EXT
#else
#define EXT extern
#endif

#define I2C_PORT_BMP280		0
#define I2C_DEV_BMP280 		0xEC				/* 从机地址+写信号  SDO默认接地 */

typedef struct {
	float32 fAirPressure;			/* 气压 */
	float32 fTemperature;			/* 温度 */
	uint16 uDigT1;					/* 驱动临时变量 */
	int16 iDigT2;					/* 驱动临时变量 */
	int16 iDigT3;					/* 驱动临时变量 */
	uint16 uDigP1;					/* 驱动临时变量 */
	int16 iDigP2;					/* 驱动临时变量 */
	int16 iDigP3;					/* 驱动临时变量 */
	int16 iDigP4;					/* 驱动临时变量 */
	int16 iDigP5;					/* 驱动临时变量 */
	int16 iDigP6;					/* 驱动临时变量 */
	int16 iDigP7;					/* 驱动临时变量 */
	int16 iDigP8;					/* 驱动临时变量 */
	int16 iDigP9;					/* 驱动临时变量 */
}Bmp280DataTypeDef;
EXT Bmp280DataTypeDef g_Bmp280Comm;

EXT BOOL Bmp280Init(void);												/* 初始化 */
EXT BOOL Bmp280GetAirAndTemp(void);										/* 读取气压和温度 */
EXT BOOL SleepBmp280(void);												/* 暂时没用 */
EXT BOOL Bmp280Wake(void);												/* 唤醒 */

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
