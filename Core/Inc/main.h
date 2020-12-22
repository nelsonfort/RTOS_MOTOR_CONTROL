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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define PID_COEF_LEN 3
typedef struct InputCapture_Signals {
	uint8_t Is_First_Captured;
	uint32_t IC_Value1;
	uint32_t IC_Value2;
	uint32_t Period;
	float	Frequency;
	uint8_t CalculationOK;
	uint8_t cont;
	int8_t direction;
} IC_Sig_t;
typedef struct UART_DATA_SEND {
	float MotorA_speed;
	uint32_t Period;
	float	Frequency;
	uint32_t time_stamp;
	float reference;
} UART_DATA_SEND_t;
typedef struct PID_Controller{
	float a[PID_COEF_LEN];
	float b[PID_COEF_LEN];
	float y_n[PID_COEF_LEN];
	float x_n[PID_COEF_LEN];
	float K; //-- Adjustable gain
}PID_Cont_t;
typedef struct MotorControl {
	float speedRef;
	float MotorA_speed;
	float MotorB_speed;
	float errorA;
	float errorB;
	PID_Cont_t PID_MA;
	PID_Cont_t PID_MB;
	float pwmMotor;
	uint8_t ready;
}Motor_Cont_t;
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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define PIN_DIR_Pin GPIO_PIN_2
#define PIN_DIR_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define MotorA_INA_Pin GPIO_PIN_11
#define MotorA_INA_GPIO_Port GPIOA
#define MotorA_INB_Pin GPIO_PIN_12
#define MotorA_INB_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define ENCODER_SHAFT_CPR 304 //CPR*Reduccion/4 =64*19/4, only used A period (risingEdge A signal)
#define millis5 5
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
