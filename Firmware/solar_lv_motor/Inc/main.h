/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "motorcontrol.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PHBL_TSENSE_Pin GPIO_PIN_0
#define PHBL_TSENSE_GPIO_Port GPIOC
#define PHBH_TSENSE_Pin GPIO_PIN_1
#define PHBH_TSENSE_GPIO_Port GPIOC
#define PHAH_TSENSE_Pin GPIO_PIN_2
#define PHAH_TSENSE_GPIO_Port GPIOC
#define PHAL_TSENSE_Pin GPIO_PIN_3
#define PHAL_TSENSE_GPIO_Port GPIOC
#define DC_ISENSE_Pin GPIO_PIN_0
#define DC_ISENSE_GPIO_Port GPIOA
#define M1_CURR_AMPL_U_Pin GPIO_PIN_1
#define M1_CURR_AMPL_U_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_2
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_3
#define UART_RX_GPIO_Port GPIOA
#define M1_CURR_AMPL_V_Pin GPIO_PIN_4
#define M1_CURR_AMPL_V_GPIO_Port GPIOA
#define M1_OCP_Pin GPIO_PIN_6
#define M1_OCP_GPIO_Port GPIOA
#define POT2_TEMP_Pin GPIO_PIN_7
#define POT2_TEMP_GPIO_Port GPIOA
#define M1_TEMPERATURE_Pin GPIO_PIN_4
#define M1_TEMPERATURE_GPIO_Port GPIOC
#define M1_BUS_VOLTAGE_Pin GPIO_PIN_5
#define M1_BUS_VOLTAGE_GPIO_Port GPIOC
#define POT1_Pin GPIO_PIN_0
#define POT1_GPIO_Port GPIOB
#define DRV_DIS_Pin GPIO_PIN_1
#define DRV_DIS_GPIO_Port GPIOB
#define MTR_OC_Pin GPIO_PIN_12
#define MTR_OC_GPIO_Port GPIOB
#define M1_PWM_UL_Pin GPIO_PIN_13
#define M1_PWM_UL_GPIO_Port GPIOB
#define M1_PWM_VL_Pin GPIO_PIN_14
#define M1_PWM_VL_GPIO_Port GPIOB
#define M1_PWM_WL_Pin GPIO_PIN_15
#define M1_PWM_WL_GPIO_Port GPIOB
#define FET_OT_Pin GPIO_PIN_7
#define FET_OT_GPIO_Port GPIOC
#define M1_PWM_UH_Pin GPIO_PIN_8
#define M1_PWM_UH_GPIO_Port GPIOA
#define M1_PWM_VH_Pin GPIO_PIN_9
#define M1_PWM_VH_GPIO_Port GPIOA
#define M1_PWM_WH_Pin GPIO_PIN_10
#define M1_PWM_WH_GPIO_Port GPIOA
#define MTR_OT_Pin GPIO_PIN_11
#define MTR_OT_GPIO_Port GPIOA
#define FLT_OUT_Pin GPIO_PIN_12
#define FLT_OUT_GPIO_Port GPIOA
#define HALLA_OUT_Pin GPIO_PIN_15
#define HALLA_OUT_GPIO_Port GPIOA
#define GPIO_OUT_Pin GPIO_PIN_11
#define GPIO_OUT_GPIO_Port GPIOC
#define HV_OV_Pin GPIO_PIN_6
#define HV_OV_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
