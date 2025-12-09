/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void _c_int00(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define cliff_btn_left_down_Pin GPIO_PIN_5
#define cliff_btn_left_down_GPIO_Port GPIOE
#define left_motor_curr_Pin GPIO_PIN_0
#define left_motor_curr_GPIO_Port GPIOC
#define dc16_ext24_Pin GPIO_PIN_3
#define dc16_ext24_GPIO_Port GPIOC
#define right_motor_curr_Pin GPIO_PIN_0
#define right_motor_curr_GPIO_Port GPIOA
#define ir1_Pin GPIO_PIN_1
#define ir1_GPIO_Port GPIOA
#define ir2_Pin GPIO_PIN_2
#define ir2_GPIO_Port GPIOA
#define cliff_btn_right_down_Pin GPIO_PIN_3
#define cliff_btn_right_down_GPIO_Port GPIOA
#define SPI1_nss_IMU_Pin GPIO_PIN_4
#define SPI1_nss_IMU_GPIO_Port GPIOA
#define SPI1_sck_IMU_Pin GPIO_PIN_5
#define SPI1_sck_IMU_GPIO_Port GPIOA
#define SPI1_miso_IMU_Pin GPIO_PIN_6
#define SPI1_miso_IMU_GPIO_Port GPIOA
#define SPI1_mosi_IMU_Pin GPIO_PIN_7
#define SPI1_mosi_IMU_GPIO_Port GPIOA
#define right_bounce_btn_____Pin GPIO_PIN_4
#define right_bounce_btn_____GPIO_Port GPIOC
#define right_bounce_btn____C5_Pin GPIO_PIN_5
#define right_bounce_btn____C5_GPIO_Port GPIOC
#define right_bounce_btn____B0_Pin GPIO_PIN_0
#define right_bounce_btn____B0_GPIO_Port GPIOB
#define right_bounce_btn____B1_Pin GPIO_PIN_1
#define right_bounce_btn____B1_GPIO_Port GPIOB
#define cliff_btn_right_up_Pin GPIO_PIN_7
#define cliff_btn_right_up_GPIO_Port GPIOE
#define machine_start_or_stop_control_Pin GPIO_PIN_8
#define machine_start_or_stop_control_GPIO_Port GPIOE
#define red_led_Pin GPIO_PIN_9
#define red_led_GPIO_Port GPIOE
#define blue_led_Pin GPIO_PIN_10
#define blue_led_GPIO_Port GPIOE
#define green_led_Pin GPIO_PIN_11
#define green_led_GPIO_Port GPIOE
#define test_Pin GPIO_PIN_12
#define test_GPIO_Port GPIOE
#define testE13_Pin GPIO_PIN_13
#define testE13_GPIO_Port GPIOE
#define tim2_ch4_main_motor_oe_Pin GPIO_PIN_10
#define tim2_ch4_main_motor_oe_GPIO_Port GPIOB
#define tim2_ch4_main_motor_a_Pin GPIO_PIN_11
#define tim2_ch4_main_motor_a_GPIO_Port GPIOB
#define uart3_tx_voice_Pin GPIO_PIN_8
#define uart3_tx_voice_GPIO_Port GPIOD
#define uart3_tx_voiceD9_Pin GPIO_PIN_9
#define uart3_tx_voiceD9_GPIO_Port GPIOD
#define TIM3_CH1_rMotor_in1_Pin GPIO_PIN_6
#define TIM3_CH1_rMotor_in1_GPIO_Port GPIOC
#define TIM3_CH2_rMotor_in2_Pin GPIO_PIN_7
#define TIM3_CH2_rMotor_in2_GPIO_Port GPIOC
#define TIM3_CH3_lMotor_in1_Pin GPIO_PIN_8
#define TIM3_CH3_lMotor_in1_GPIO_Port GPIOC
#define TIM3_CH4_lMotor_in2_Pin GPIO_PIN_9
#define TIM3_CH4_lMotor_in2_GPIO_Port GPIOC
#define cliff_btn_left_up_Pin GPIO_PIN_15
#define cliff_btn_left_up_GPIO_Port GPIOA
#define DEBUG_UART4_TX_Pin GPIO_PIN_10
#define DEBUG_UART4_TX_GPIO_Port GPIOC
#define DEBUG_UART4_RX_Pin GPIO_PIN_11
#define DEBUG_UART4_RX_GPIO_Port GPIOC
#define right_pump_output_Pin GPIO_PIN_12
#define right_pump_output_GPIO_Port GPIOC
#define left_pump_output_Pin GPIO_PIN_1
#define left_pump_output_GPIO_Port GPIOD
#define left_bounce_btn_____Pin GPIO_PIN_2
#define left_bounce_btn_____GPIO_Port GPIOD
#define left_bounce_btn____D3_Pin GPIO_PIN_3
#define left_bounce_btn____D3_GPIO_Port GPIOD
#define left_bounce_btn____D4_Pin GPIO_PIN_4
#define left_bounce_btn____D4_GPIO_Port GPIOD
#define left_bounce_btn____D5_Pin GPIO_PIN_5
#define left_bounce_btn____D5_GPIO_Port GPIOD
#define pump_input_Pin GPIO_PIN_6
#define pump_input_GPIO_Port GPIOD
#define uart1_tx_gprs_Pin GPIO_PIN_6
#define uart1_tx_gprs_GPIO_Port GPIOB
#define uart1_rx_gprs_Pin GPIO_PIN_7
#define uart1_rx_gprs_GPIO_Port GPIOB
#define air_sda_Pin GPIO_PIN_8
#define air_sda_GPIO_Port GPIOB
#define air_scl_Pin GPIO_PIN_9
#define air_scl_GPIO_Port GPIOB
#define other_3_3_en_Pin GPIO_PIN_0
#define other_3_3_en_GPIO_Port GPIOE

#define BMI270_INT_Pin 			GPIO_PIN_1
#define BMI270_INT_GPIO_Port 	GPIOC
#define BMI270_INT_EXTI_IRQn 	EXTI1_IRQn

#define GPRS_4G_EN_Pin			GPIO_PIN_2
#define GPRS_4G_EN_Port			GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
