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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

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
void MX_USART1_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USB_CC2_Pin LL_GPIO_PIN_0
#define USB_CC2_GPIO_Port GPIOF
#define USB_CC1_Pin LL_GPIO_PIN_1
#define USB_CC1_GPIO_Port GPIOF
#define A_Pin LL_GPIO_PIN_0
#define A_GPIO_Port GPIOA
#define B_Pin LL_GPIO_PIN_1
#define B_GPIO_Port GPIOA
#define VCOM_TX_Pin LL_GPIO_PIN_2
#define VCOM_TX_GPIO_Port GPIOA
#define VCOM_RX_Pin LL_GPIO_PIN_3
#define VCOM_RX_GPIO_Port GPIOA
#define T_NRESET_Pin LL_GPIO_PIN_4
#define T_NRESET_GPIO_Port GPIOA
#define VSENSE_LV_Pin LL_GPIO_PIN_5
#define VSENSE_LV_GPIO_Port GPIOA
#define ISENSE_OUT_Pin LL_GPIO_PIN_6
#define ISENSE_OUT_GPIO_Port GPIOA
#define ISENSE_COMP_Pin LL_GPIO_PIN_7
#define ISENSE_COMP_GPIO_Port GPIOA
#define ISENSE_LV_Pin LL_GPIO_PIN_0
#define ISENSE_LV_GPIO_Port GPIOB
#define T_SCK_Pin LL_GPIO_PIN_8
#define T_SCK_GPIO_Port GPIOA
#define T_MOSI_Pin LL_GPIO_PIN_9
#define T_MOSI_GPIO_Port GPIOA
#define T_MISO_Pin LL_GPIO_PIN_10
#define T_MISO_GPIO_Port GPIOA
#define LED_Pin LL_GPIO_PIN_15
#define LED_GPIO_Port GPIOA
#define LS_OE_Pin LL_GPIO_PIN_4
#define LS_OE_GPIO_Port GPIOB
#define T_5V_EN_Pin LL_GPIO_PIN_5
#define T_5V_EN_GPIO_Port GPIOB
#define T_3V3_EN_Pin LL_GPIO_PIN_6
#define T_3V3_EN_GPIO_Port GPIOB
#define NFAULT_Pin LL_GPIO_PIN_7
#define NFAULT_GPIO_Port GPIOB
#define SW_Pin LL_GPIO_PIN_8
#define SW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
