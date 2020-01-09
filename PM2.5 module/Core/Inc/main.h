/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define A0_LCD_Pin GPIO_PIN_0
#define A0_LCD_GPIO_Port GPIOA
#define D6_LCD_Pin GPIO_PIN_10
#define D6_LCD_GPIO_Port GPIOB
#define LED_Green_Pin GPIO_PIN_13
#define LED_Green_GPIO_Port GPIOB
#define LED_Yellow_Pin GPIO_PIN_14
#define LED_Yellow_GPIO_Port GPIOB
#define LED_Red_Pin GPIO_PIN_15
#define LED_Red_GPIO_Port GPIOB
#define PM2_5_UART6_TX_Pin GPIO_PIN_6
#define PM2_5_UART6_TX_GPIO_Port GPIOC
#define D9_LCD_Pin GPIO_PIN_7
#define D9_LCD_GPIO_Port GPIOC
#define D7_LCD_Pin GPIO_PIN_8
#define D7_LCD_GPIO_Port GPIOA
#define D8_LCD_Pin GPIO_PIN_9
#define D8_LCD_GPIO_Port GPIOA
#define PM2_5_UART6_RX_Pin GPIO_PIN_12
#define PM2_5_UART6_RX_GPIO_Port GPIOA
#define D5_LCD_Pin GPIO_PIN_4
#define D5_LCD_GPIO_Port GPIOB
#define D4_LCD_Pin GPIO_PIN_5
#define D4_LCD_GPIO_Port GPIOB
#define D10_LCD_Pin GPIO_PIN_6
#define D10_LCD_GPIO_Port GPIOB
#define RTC_I2C_SCL_Pin GPIO_PIN_8
#define RTC_I2C_SCL_GPIO_Port GPIOB
#define RTC_I2C_SDA_Pin GPIO_PIN_9
#define RTC_I2C_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
