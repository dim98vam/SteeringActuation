/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern volatile uint16_t      counterTimeout;             //helping variable to help with interrupt reading wheel position every 10ms
                                                          //interrupt is done with systick at first

extern volatile uint16_t      counter;                    //variable to initiate position transmition

extern volatile uint8_t      timeOutOccured;      //indicates a communication timeout has occured
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//systick interrupt defines
#define TIMEOUT 200           //defines the milliseconds until timeout of communication
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Wheel_Position_Measurement_Pin GPIO_PIN_0
#define Wheel_Position_Measurement_GPIO_Port GPIOA
#define Step_Signal_Pin GPIO_PIN_4
#define Step_Signal_GPIO_Port GPIOA
#define PWM_OUTPUT_Pin GPIO_PIN_5
#define PWM_OUTPUT_GPIO_Port GPIOA
#define Direction_Of_Rotation_Pin GPIO_PIN_8
#define Direction_Of_Rotation_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
void ADC_Handle(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
