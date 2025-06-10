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
#define I2C_DEV_BMP280 		0xEC				/* �ӻ���ַ+д�ź�  SDOĬ�Ͻӵ� */

typedef struct {
	float32 fAirPressure;			/* ��ѹ */
	float32 fTemperature;			/* �¶� */
	uint16 uDigT1;					/* ������ʱ���� */
	int16 iDigT2;					/* ������ʱ���� */
	int16 iDigT3;					/* ������ʱ���� */
	uint16 uDigP1;					/* ������ʱ���� */
	int16 iDigP2;					/* ������ʱ���� */
	int16 iDigP3;					/* ������ʱ���� */
	int16 iDigP4;					/* ������ʱ���� */
	int16 iDigP5;					/* ������ʱ���� */
	int16 iDigP6;					/* ������ʱ���� */
	int16 iDigP7;					/* ������ʱ���� */
	int16 iDigP8;					/* ������ʱ���� */
	int16 iDigP9;					/* ������ʱ���� */
}Bmp280DataTypeDef;
EXT Bmp280DataTypeDef g_Bmp280Comm;

EXT BOOL Bmp280Init(void);												/* ��ʼ�� */
EXT BOOL Bmp280GetAirAndTemp(void);										/* ��ȡ��ѹ���¶� */
EXT BOOL SleepBmp280(void);												/* ��ʱû�� */
EXT BOOL Bmp280Wake(void);												/* ���� */

#undef EXT				/* release EXT */
#endif					/* end of exclude redefinition */
