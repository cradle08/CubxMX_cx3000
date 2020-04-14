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
#define Beep_Pin GPIO_PIN_3
#define Beep_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOE
#define Motor3_V_ADC_Pin GPIO_PIN_3
#define Motor3_V_ADC_GPIO_Port GPIOF
#define Motor4_V_Cur_Pin GPIO_PIN_4
#define Motor4_V_Cur_GPIO_Port GPIOF
#define LED_CUR_ADC_Pin GPIO_PIN_6
#define LED_CUR_ADC_GPIO_Port GPIOF
#define Elec_ADC_Pin GPIO_PIN_8
#define Elec_ADC_GPIO_Port GPIOF
#define BC_Cur_ADC_Pin GPIO_PIN_9
#define BC_Cur_ADC_GPIO_Port GPIOF
#define Cur12P_ADC_Pin GPIO_PIN_10
#define Cur12P_ADC_GPIO_Port GPIOF
#define Temp_ADC_Pin GPIO_PIN_0
#define Temp_ADC_GPIO_Port GPIOC
#define ETH_MDC_Pin GPIO_PIN_1
#define ETH_MDC_GPIO_Port GPIOC
#define Press_ADC_Pin GPIO_PIN_2
#define Press_ADC_GPIO_Port GPIOC
#define XK_ADC_Pin GPIO_PIN_3
#define XK_ADC_GPIO_Port GPIOC
#define ETH_REF_CLK_Pin GPIO_PIN_1
#define ETH_REF_CLK_GPIO_Port GPIOA
#define ETH_MDIO_Pin GPIO_PIN_2
#define ETH_MDIO_GPIO_Port GPIOA
#define Press_I2C_CLK_Pin GPIO_PIN_4
#define Press_I2C_CLK_GPIO_Port GPIOH
#define Press_I2C_SDA_Pin GPIO_PIN_5
#define Press_I2C_SDA_GPIO_Port GPIOH
#define Cur12N_ADC_Pin GPIO_PIN_3
#define Cur12N_ADC_GPIO_Port GPIOA
#define BC_SIG_1_Pin GPIO_PIN_5
#define BC_SIG_1_GPIO_Port GPIOA
#define ETHT_CRS_DV_Pin GPIO_PIN_7
#define ETHT_CRS_DV_GPIO_Port GPIOA
#define ETH_RXD0_Pin GPIO_PIN_4
#define ETH_RXD0_GPIO_Port GPIOC
#define ETH_RXD1_Pin GPIO_PIN_5
#define ETH_RXD1_GPIO_Port GPIOC
#define Walve_Air_Pin GPIO_PIN_11
#define Walve_Air_GPIO_Port GPIOF
#define Conuter_SW_Pin GPIO_PIN_12
#define Conuter_SW_GPIO_Port GPIOF
#define Motor3_In_OC_Pin GPIO_PIN_0
#define Motor3_In_OC_GPIO_Port GPIOG
#define Motor3_Out_OC_Pin GPIO_PIN_1
#define Motor3_Out_OC_GPIO_Port GPIOG
#define CUR_BC_Pin GPIO_PIN_7
#define CUR_BC_GPIO_Port GPIOE
#define Motor4_CLK_Pin GPIO_PIN_9
#define Motor4_CLK_GPIO_Port GPIOE
#define Motor4_DIR_Pin GPIO_PIN_11
#define Motor4_DIR_GPIO_Port GPIOE
#define Motor4_EN_Pin GPIO_PIN_13
#define Motor4_EN_GPIO_Port GPIOE
#define Liquild_Walve_Pin GPIO_PIN_6
#define Liquild_Walve_GPIO_Port GPIOH
#define Motor_DIR_Pin GPIO_PIN_9
#define Motor_DIR_GPIO_Port GPIOD
#define Motor1_EN_Pin GPIO_PIN_10
#define Motor1_EN_GPIO_Port GPIOD
#define Motor2_CLK_Pin GPIO_PIN_14
#define Motor2_CLK_GPIO_Port GPIOD
#define MotorX_IN_OC_Pin GPIO_PIN_2
#define MotorX_IN_OC_GPIO_Port GPIOG
#define MotorX_Out_OC_Pin GPIO_PIN_3
#define MotorX_Out_OC_GPIO_Port GPIOG
#define MotorY_IN_OC_Pin GPIO_PIN_4
#define MotorY_IN_OC_GPIO_Port GPIOG
#define MotorY_Out_OC_Pin GPIO_PIN_5
#define MotorY_Out_OC_GPIO_Port GPIOG
#define Motor4_In_OC_Pin GPIO_PIN_6
#define Motor4_In_OC_GPIO_Port GPIOG
#define Motor4_Out_OC_Pin GPIO_PIN_7
#define Motor4_Out_OC_GPIO_Port GPIOG
#define Motor4_Out_OC_EXTI_IRQn EXTI9_5_IRQn
#define Motor3_CLK_Pin GPIO_PIN_6
#define Motor3_CLK_GPIO_Port GPIOC
#define Motor3_DIR_Pin GPIO_PIN_7
#define Motor3_DIR_GPIO_Port GPIOC
#define Motor1_CLK_Pin GPIO_PIN_8
#define Motor1_CLK_GPIO_Port GPIOC
#define Motor3_EN_Pin GPIO_PIN_9
#define Motor3_EN_GPIO_Port GPIOC
#define Elec_OC_Pin GPIO_PIN_11
#define Elec_OC_GPIO_Port GPIOA
#define D_Register_CS_Pin GPIO_PIN_0
#define D_Register_CS_GPIO_Port GPIOI
#define D_Register_CLK_Pin GPIO_PIN_1
#define D_Register_CLK_GPIO_Port GPIOI
#define D_Regitster_MOSI_Pin GPIO_PIN_3
#define D_Regitster_MOSI_GPIO_Port GPIOI
#define Motor2_DIR_Pin GPIO_PIN_0
#define Motor2_DIR_GPIO_Port GPIOD
#define LAN8720_RST_Pin GPIO_PIN_3
#define LAN8720_RST_GPIO_Port GPIOD
#define Micro_OC_Pin GPIO_PIN_4
#define Micro_OC_GPIO_Port GPIOD
#define Motor2_EN_Pin GPIO_PIN_7
#define Motor2_EN_GPIO_Port GPIOD
#define ETH_TX_EN_Pin GPIO_PIN_11
#define ETH_TX_EN_GPIO_Port GPIOG
#define ETH_TXD0_Pin GPIO_PIN_13
#define ETH_TXD0_GPIO_Port GPIOG
#define ETH_TXD1_Pin GPIO_PIN_14
#define ETH_TXD1_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
