#ifndef _BMI270_H_
#define _BMI270_H_

#include "GlobalVar.h"

#ifdef _BMI270_C_
#define EXT
#else
#define EXT extern
#endif

#define SPI_PORT_BMI270		0

#define HALF_T 0.005f               /* 采样周期的一半（定时器的一半） 0.005代表100Hz，改变刷新频率务必也要改变这里！！！ */
//#define KP 100.0f                   /* 比例增益支配率收敛到加速度计/磁强计	*/
//#define KI 0.002f                	/* 积分增益支配率的陀螺仪偏见的衔接 */
//#define KP 20.0f                   /* 比例增益支配率收敛到加速度计/磁强计	*/
//#define KI 0.0004f                	/* 积分增益支配率的陀螺仪偏见的衔接 */

#define KP 2.0f                   /* 比例增益支配率收敛到加速度计/磁强计	*/
#define KI 0.002f                	/* 积分增益支配率的陀螺仪偏见的衔接 */


#define BMI270_TIMEOUT_COUNT      (0xFF)                                    /* BMI270超时计数,最大不能超过Uint8最大值 */

#define BMI270_DEV_ADDR           (0x69)                                    /* SA0接地：0x68 SA0上拉：0x69 模块默认上拉 */
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

#define BMI270_GYR_SAMPLE         (0x00)                                      /* 陀螺仪量程 */
/* 设置为:0x00 陀螺仪量程为:±2000dps     获取到的陀螺仪数据 除以 16.4       可以转化为带物理单位的数据 单位为：°/s	*/
/* 设置为:0x01 陀螺仪量程为:±1000dps     获取到的陀螺仪数据 除以 32.8       可以转化为带物理单位的数据 单位为：°/s	*/
/* 设置为:0x02 陀螺仪量程为:±500 dps     获取到的陀螺仪数据 除以 65.6       可以转化为带物理单位的数据 单位为：°/s	*/
/* 设置为:0x03 陀螺仪量程为:±250 dps     获取到的陀螺仪数据 除以 131.2      可以转化为带物理单位的数据 单位为：°/s	*/
/* 设置为:0x04 陀螺仪量程为:±125 dps     获取到的陀螺仪数据 除以 262.4      可以转化为带物理单位的数据 单位为：°/s	*/

#define BMI270_ACC_SAMPLE         (0x00)                                      /* 加速度计量程	*/
/* 设置为:0x00 加速度计量程为:±2g         获取到的加速度计数据 除以 16384   可以转化为带物理单位的数据 单位：g(m/s^2)	*/
/* 设置为:0x01 加速度计量程为:±4g         获取到的加速度计数据 除以 8192    可以转化为带物理单位的数据 单位：g(m/s^2)	*/
/* 设置为:0x02 加速度计量程为:±8g         获取到的加速度计数据 除以 4096    可以转化为带物理单位的数据 单位：g(m/s^2)	*/
/* 设置为:0x03 加速度计量程为:±16g        获取到的加速度计数据 除以 2048    可以转化为带物理单位的数据 单位：g(m/s^2)	*/

typedef struct {
	int16 iGyrX;		/* °/s */
	int16 iGyrY;		/* °/s */
	int16 iGyrZ;		/* °/s */
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

EXT uint8 Bmi270Init(void);                                     /* 初始化 */
EXT BOOL Bmi270GetAcc(void);                                    /* 获取加速度计数据 */
EXT BOOL Bmi270GetGyro(void);                                   /* 获取陀螺仪数据 */
EXT float32 Bmi270AccTransition(int16 iAccValue);               /* 将加速度计数据转换为实际物理数据 */
EXT float32 Bmi270GyroTransition(int16 iGyroValue);       		/* 将陀螺仪数据转换为实际物理数据 */

#endif

