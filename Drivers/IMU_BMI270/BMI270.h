#ifndef _BMI270_H_
#define _BMI270_H_

#include "GlobalVar.h"

#ifdef _BMI270_C_
#define EXT
#else
#define EXT extern
#endif

#define SPI_PORT_BMI270		0

#define HALF_T 0.005f               /* �������ڵ�һ�루��ʱ����һ�룩 0.005����100Hz���ı�ˢ��Ƶ�����ҲҪ�ı�������� */
//#define KP 100.0f                   /* ��������֧�������������ٶȼ�/��ǿ��	*/
//#define KI 0.002f                	/* ��������֧���ʵ�������ƫ�����ν� */
//#define KP 20.0f                   /* ��������֧�������������ٶȼ�/��ǿ��	*/
//#define KI 0.0004f                	/* ��������֧���ʵ�������ƫ�����ν� */

#define KP 2.0f                   /* ��������֧�������������ٶȼ�/��ǿ��	*/
#define KI 0.002f                	/* ��������֧���ʵ�������ƫ�����ν� */


#define BMI270_TIMEOUT_COUNT      (0xFF)                                    /* BMI270��ʱ����,����ܳ���Uint8���ֵ */

#define BMI270_DEV_ADDR           (0x69)                                    /* SA0�ӵأ�0x68 SA0������0x69 ģ��Ĭ������ */
#define BMI270_SPI_W              (0x00)
#define BMI270_SPI_R              (0x80)

#define BMI270_CHIP_ID            (0x00)
#define BMI270_PWR_CONF           (0x7C)
#define BMI270_PWR_CTRL           (0x7D)
#define BMI270_INIT_CTRL          (0x59)
#define BMI270_INIT_DATA          (0x5E)
#define BMI270_INT_STA            (0x21)
#define BMI270_ACC_ADDRESS        (0x0C)
#define BMI270_GYRO_ADDRESS       (0x12)
#define BMI270_ACC_CONF           (0x40)
#define BMI270_ACC_RANGE          (0x41)

#define BMI270_GYR_CONF           (0x42)
#define BMI270_GYR_RANGE          (0x43)

#define BMI270_GYR_SAMPLE         (0x00)                                      /* ���������� */
/* ����Ϊ:0x00 ����������Ϊ:��2000dps     ��ȡ�������������� ���� 16.4       ����ת��Ϊ������λ������ ��λΪ����/s	*/
/* ����Ϊ:0x01 ����������Ϊ:��1000dps     ��ȡ�������������� ���� 32.8       ����ת��Ϊ������λ������ ��λΪ����/s	*/
/* ����Ϊ:0x02 ����������Ϊ:��500 dps     ��ȡ�������������� ���� 65.6       ����ת��Ϊ������λ������ ��λΪ����/s	*/
/* ����Ϊ:0x03 ����������Ϊ:��250 dps     ��ȡ�������������� ���� 131.2      ����ת��Ϊ������λ������ ��λΪ����/s	*/
/* ����Ϊ:0x04 ����������Ϊ:��125 dps     ��ȡ�������������� ���� 262.4      ����ת��Ϊ������λ������ ��λΪ����/s	*/

#define BMI270_ACC_SAMPLE         (0x00)                                      /* ���ٶȼ�����	*/
/* ����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g         ��ȡ���ļ��ٶȼ����� ���� 16384   ����ת��Ϊ������λ������ ��λ��g(m/s^2)	*/
/* ����Ϊ:0x01 ���ٶȼ�����Ϊ:��4g         ��ȡ���ļ��ٶȼ����� ���� 8192    ����ת��Ϊ������λ������ ��λ��g(m/s^2)	*/
/* ����Ϊ:0x02 ���ٶȼ�����Ϊ:��8g         ��ȡ���ļ��ٶȼ����� ���� 4096    ����ת��Ϊ������λ������ ��λ��g(m/s^2)	*/
/* ����Ϊ:0x03 ���ٶȼ�����Ϊ:��16g        ��ȡ���ļ��ٶȼ����� ���� 2048    ����ת��Ϊ������λ������ ��λ��g(m/s^2)	*/

typedef struct {
	int16 iGyrX;		/* ��/s */
	int16 iGyrY;		/* ��/s */
	int16 iGyrZ;		/* ��/s */
	int16 iAccX;		/* G */
	int16 iAccY;		/* G */
	int16 iAccZ;		/* G */
}BMI270;
EXT BMI270 g_Bmi270Comm;

#ifdef _BMI270_C_
	BMI270 g_Bmi270Comm = {0, 0, 0, 0, 0, 0};
#else
	extern BMI270 g_Bmi270Comm;
#endif

EXT uint8 Bmi270Init(void);                                     /* ��ʼ�� */
EXT BOOL Bmi270GetAcc(void);                                    /* ��ȡ���ٶȼ����� */
EXT BOOL Bmi270GetGyro(void);                                   /* ��ȡ���������� */
EXT float32 Bmi270AccTransition(int16 iAccValue);               /* �����ٶȼ�����ת��Ϊʵ���������� */
EXT float32 Bmi270GyroTransition(int16 iGyroValue);       		/* ������������ת��Ϊʵ���������� */

#endif

