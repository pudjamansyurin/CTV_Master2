/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define ACCP_Pin GPIO_PIN_2
#define ACCP_GPIO_Port GPIOE
#define ACCN_Pin GPIO_PIN_3
#define ACCN_GPIO_Port GPIOE
#define CFB1_Pin GPIO_PIN_4
#define CFB1_GPIO_Port GPIOE
#define CFB2_Pin GPIO_PIN_5
#define CFB2_GPIO_Port GPIOE
#define CINJN_Pin GPIO_PIN_6
#define CINJN_GPIO_Port GPIOE
#define TX0_Pin GPIO_PIN_0
#define TX0_GPIO_Port GPIOF
#define TX1_Pin GPIO_PIN_1
#define TX1_GPIO_Port GPIOF
#define TX2_Pin GPIO_PIN_2
#define TX2_GPIO_Port GPIOF
#define TX3_Pin GPIO_PIN_3
#define TX3_GPIO_Port GPIOF
#define TX4_Pin GPIO_PIN_4
#define TX4_GPIO_Port GPIOF
#define TX5_Pin GPIO_PIN_5
#define TX5_GPIO_Port GPIOF
#define TX6_Pin GPIO_PIN_6
#define TX6_GPIO_Port GPIOF
#define TX7_Pin GPIO_PIN_7
#define TX7_GPIO_Port GPIOF
#define TX8_Pin GPIO_PIN_8
#define TX8_GPIO_Port GPIOF
#define TX9_Pin GPIO_PIN_9
#define TX9_GPIO_Port GPIOF
#define TX10_Pin GPIO_PIN_10
#define TX10_GPIO_Port GPIOF
#define MSTR_Pin GPIO_PIN_1
#define MSTR_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_4
#define CS1_GPIO_Port GPIOA
#define TX11_Pin GPIO_PIN_11
#define TX11_GPIO_Port GPIOF
#define TX12_Pin GPIO_PIN_12
#define TX12_GPIO_Port GPIOF
#define TX13_Pin GPIO_PIN_13
#define TX13_GPIO_Port GPIOF
#define TX14_Pin GPIO_PIN_14
#define TX14_GPIO_Port GPIOF
#define TX15_Pin GPIO_PIN_15
#define TX15_GPIO_Port GPIOF
#define SWRUN_Pin GPIO_PIN_1
#define SWRUN_GPIO_Port GPIOG
#define CINJP_Pin GPIO_PIN_8
#define CINJP_GPIO_Port GPIOE
#define CS2_Pin GPIO_PIN_11
#define CS2_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOG
#define CS3_Pin GPIO_PIN_8
#define CS3_GPIO_Port GPIOG
#define VREF_Pin GPIO_PIN_0
#define VREF_GPIO_Port GPIOE
#define CTV_Pin GPIO_PIN_1
#define CTV_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
