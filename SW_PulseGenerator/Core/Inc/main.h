/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
HAL_StatusTypeDef HAL_TIM_PWM_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_PWMN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_PWMN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

void led_task(void);
void KeyStaIn(uint8_t num, uint8_t sta);


/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define key_output_Pin GPIO_PIN_13
#define key_output_GPIO_Port GPIOC
#define key_pulse_Pin GPIO_PIN_14
#define key_pulse_GPIO_Port GPIOC
#define key_add_Pin GPIO_PIN_15
#define key_add_GPIO_Port GPIOC
#define DISP_F_Pin GPIO_PIN_1
#define DISP_F_GPIO_Port GPIOB
#define DISP_D_Pin GPIO_PIN_2
#define DISP_D_GPIO_Port GPIOB
#define DISP_DIG2_Pin GPIO_PIN_10
#define DISP_DIG2_GPIO_Port GPIOB
#define DISP_C_Pin GPIO_PIN_11
#define DISP_C_GPIO_Port GPIOB
#define DISP_J_Pin GPIO_PIN_12
#define DISP_J_GPIO_Port GPIOB
#define DISP_H_Pin GPIO_PIN_14
#define DISP_H_GPIO_Port GPIOB
#define DISP_DIG1_Pin GPIO_PIN_15
#define DISP_DIG1_GPIO_Port GPIOB
#define DISP_DP_Pin GPIO_PIN_9
#define DISP_DP_GPIO_Port GPIOA
#define DISP_A_Pin GPIO_PIN_6
#define DISP_A_GPIO_Port GPIOC
#define DISP_E_Pin GPIO_PIN_7
#define DISP_E_GPIO_Port GPIOC
#define DISP_DIG3_Pin GPIO_PIN_10
#define DISP_DIG3_GPIO_Port GPIOA
#define DISP_N_Pin GPIO_PIN_11
#define DISP_N_GPIO_Port GPIOA
#define DISP_B_Pin GPIO_PIN_12
#define DISP_B_GPIO_Port GPIOA
#define DISP_M_Pin GPIO_PIN_15
#define DISP_M_GPIO_Port GPIOA
#define DISP_DIG4_Pin GPIO_PIN_0
#define DISP_DIG4_GPIO_Port GPIOD
#define DISP_L_Pin GPIO_PIN_1
#define DISP_L_GPIO_Port GPIOD
#define DISP_DIG5_Pin GPIO_PIN_2
#define DISP_DIG5_GPIO_Port GPIOD
#define DISP_G2_Pin GPIO_PIN_3
#define DISP_G2_GPIO_Port GPIOD
#define DISP_K_Pin GPIO_PIN_3
#define DISP_K_GPIO_Port GPIOB
#define DISP_G1_Pin GPIO_PIN_4
#define DISP_G1_GPIO_Port GPIOB
#define DISP_DIG6_Pin GPIO_PIN_5
#define DISP_DIG6_GPIO_Port GPIOB
#define key_sub_Pin GPIO_PIN_6
#define key_sub_GPIO_Port GPIOB
#define key_volt_Pin GPIO_PIN_7
#define key_volt_GPIO_Port GPIOB
#define key_move_Pin GPIO_PIN_8
#define key_move_GPIO_Port GPIOB
#define key_freq_Pin GPIO_PIN_9
#define key_freq_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */




#define MODE_NORMAL 		0x00
#define MODE_SINGEL_BURST 	0xAA
#define MODE_REPEAT_BURST 	0xBB

extern volatile uint8_t output_mode;

#define DISP_NUM_0  0x003F
#define DISP_NUM_1  0x0406
#define DISP_NUM_2	0x00DB
#define DISP_NUM_3	0x008F
#define DISP_NUM_4	0x00E6
#define DISP_NUM_5	0x00ED
#define DISP_NUM_6	0x00FD
#define DISP_NUM_7	0x0007
#define DISP_NUM_8	0x00FF
#define DISP_NUM_9	0x00EF


#define DISP_CHAR_A	0x00F7
#define DISP_CHAR_B	0x128F
#define DISP_CHAR_C	0x0039
#define DISP_CHAR_D	0x120F
#define DISP_CHAR_E	0x00F9
#define DISP_CHAR_F	0x00F1
#define DISP_CHAR_G	0x00BD
#define DISP_CHAR_H	0x00F6
#define DISP_CHAR_I	0x1209
#define DISP_CHAR_J	0x001E
#define DISP_CHAR_K	0x0C70
#define DISP_CHAR_L	0x0038
#define DISP_CHAR_M	0x0536
#define DISP_CHAR_N	0x0936
#define DISP_CHAR_O	0x003F
#define DISP_CHAR_P	0x00F3
#define DISP_CHAR_Q	0x083F
#define DISP_CHAR_R	0x08F3
#define DISP_CHAR_S	0x00ED
#define DISP_CHAR_T	0x1201
#define DISP_CHAR_U	0x003E
#define DISP_CHAR_V	0x2430
#define DISP_CHAR_W	0x2836
#define DISP_CHAR_X	0x2D00
#define DISP_CHAR_Y	0x1500
#define DISP_CHAR_Z	0x2409

#define DISP_CHAR_u	0x001C
#define DISP_CHAR_n	0x00D4
#define DISP_CHAR_m	0x10D4

void adc_switch(uint8_t state);

#define ADC_BUFFER_SIZE 32
#define ADC_CHANNEL_NUM 4

#define BAT_CH 0
#define HV_CH 1
#define DAC_CH 2
#define VREF_CH 3

extern volatile uint16_t adc_buffer[ADC_BUFFER_SIZE][ADC_CHANNEL_NUM];

extern volatile double adc_value[ADC_CHANNEL_NUM];
extern uint8_t output_sta;

#define KEY_ADD		0
#define KEY_SUB		1
#define KEY_MOVE	2
#define KEY_PULSE	3
#define KEY_FREQ	4
#define KEY_VOLT	5
#define KEY_OUTPUT	6

#define SET_FREQ_HZ 	0
#define SET_FREQ_KHZ 	1
#define SET_FREQ_MHZ 	2

#define SET_PULSE_NS 	3
#define SET_PULSE_US 	4

#define SET_VOLT_MV 	5
#define SET_VOLT_V 		6

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
