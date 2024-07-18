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
#include "stm32l0xx_hal.h"

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
#define LSM6_INT1_Pin GPIO_PIN_13
#define LSM6_INT1_GPIO_Port GPIOC
#define RF_AUX_Pin GPIO_PIN_14
#define RF_AUX_GPIO_Port GPIOC
#define W25Q_CS_Pin GPIO_PIN_15
#define W25Q_CS_GPIO_Port GPIOC
#define LSM6_CS_Pin GPIO_PIN_0
#define LSM6_CS_GPIO_Port GPIOH
#define A355_CS_Pin GPIO_PIN_1
#define A355_CS_GPIO_Port GPIOH
#define VBAT_ADC_Pin GPIO_PIN_4
#define VBAT_ADC_GPIO_Port GPIOA
#define BLU_CTRL_Pin GPIO_PIN_0
#define BLU_CTRL_GPIO_Port GPIOB
#define GRN_CTRL_Pin GPIO_PIN_1
#define GRN_CTRL_GPIO_Port GPIOB
#define RED_CTRL_Pin GPIO_PIN_2
#define RED_CTRL_GPIO_Port GPIOB
#define ESP_RST_Pin GPIO_PIN_12
#define ESP_RST_GPIO_Port GPIOB
#define ESP_EN_Pin GPIO_PIN_13
#define ESP_EN_GPIO_Port GPIOB
#define ESP_GPIO0_Pin GPIO_PIN_14
#define ESP_GPIO0_GPIO_Port GPIOB
#define RS232_EN_Pin GPIO_PIN_15
#define RS232_EN_GPIO_Port GPIOB
#define RS232_SD_Pin GPIO_PIN_8
#define RS232_SD_GPIO_Port GPIOA
#define RS485_TX_EN_Pin GPIO_PIN_11
#define RS485_TX_EN_GPIO_Port GPIOA
#define RS485_RX_EN_Pin GPIO_PIN_12
#define RS485_RX_EN_GPIO_Port GPIOA
#define EXT_EN_Pin GPIO_PIN_15
#define EXT_EN_GPIO_Port GPIOA
#define CHG_STBY_Pin GPIO_PIN_5
#define CHG_STBY_GPIO_Port GPIOB
#define CHG_STAT_Pin GPIO_PIN_6
#define CHG_STAT_GPIO_Port GPIOB
#define CHG_EN_Pin GPIO_PIN_7
#define CHG_EN_GPIO_Port GPIOB
#define RF_M0_Pin GPIO_PIN_8
#define RF_M0_GPIO_Port GPIOB
#define RF_M1_Pin GPIO_PIN_9
#define RF_M1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
