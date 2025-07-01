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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "settings.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	ATC_NO_ROTATION, ATC_CW, ATC_CCW
} ATCRotationDirection_t;

typedef enum {
	ATC_LOCKED, ATC_UNLOCKED
} ATCLockState_t;

/**
 * @brief State (keadaan) untuk state machine ATC dalam super-loop.
 * Menggantikan logika task RTOS yang bersifat blocking.
 */
typedef enum
{
	ATC_STATE_IDLE,			   // Menunggu perintah baru
	ATC_STATE_UNLOCKING_START, // Memulai proses unlock
	ATC_STATE_UNLOCKING_WAIT,  // Menunggu sinyal unlock selesai
	ATC_STATE_ROTATING_START,  // Memulai proses rotasi
	ATC_STATE_ROTATING_WAIT,   // Menunggu sinyal rotasi selesai
	ATC_STATE_LOCKING_START,   // Memulai proses lock
	ATC_STATE_LOCKING_WAIT,	   // Menunggu sinyal lock selesai
	ATC_STATE_TIMEOUT_ERROR	   // Terjadi error karena timeout
} ATC_State_t;
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
void USB_CDC_RxHandler(uint8_t *Buf, uint32_t Len);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_BTN_Pin GPIO_PIN_0
#define USER_BTN_GPIO_Port GPIOA
#define NEWKER_TOOL_1_Pin GPIO_PIN_1
#define NEWKER_TOOL_1_GPIO_Port GPIOA
#define NEWKER_TOOL_2_Pin GPIO_PIN_2
#define NEWKER_TOOL_2_GPIO_Port GPIOA
#define NEWKER_TOOL_3_Pin GPIO_PIN_3
#define NEWKER_TOOL_3_GPIO_Port GPIOA
#define NEWKER_TOOL_4_Pin GPIO_PIN_4
#define NEWKER_TOOL_4_GPIO_Port GPIOA
#define NEWKER_TOOL_5_Pin GPIO_PIN_5
#define NEWKER_TOOL_5_GPIO_Port GPIOA
#define NEWKER_TOOL_6_Pin GPIO_PIN_6
#define NEWKER_TOOL_6_GPIO_Port GPIOA
#define NEWKER_TOOL_7_Pin GPIO_PIN_7
#define NEWKER_TOOL_7_GPIO_Port GPIOA
#define NEWKER_TOOL_8_Pin GPIO_PIN_0
#define NEWKER_TOOL_8_GPIO_Port GPIOB
#define USER_LED_Pin GPIO_PIN_2
#define USER_LED_GPIO_Port GPIOB
#define IN_NEWKER_TOK_Pin GPIO_PIN_12
#define IN_NEWKER_TOK_GPIO_Port GPIOB
#define IN_NEWKER_T_PLUS_Pin GPIO_PIN_13
#define IN_NEWKER_T_PLUS_GPIO_Port GPIOB
#define IN_NEWKER_T_MIN_Pin GPIO_PIN_14
#define IN_NEWKER_T_MIN_GPIO_Port GPIOB
#define OUT_ATC_LOCK_Pin GPIO_PIN_15
#define OUT_ATC_LOCK_GPIO_Port GPIOB
#define STEPPER_PULSE_Pin GPIO_PIN_8
#define STEPPER_PULSE_GPIO_Port GPIOA
#define STEPPER_DIR_Pin GPIO_PIN_9
#define STEPPER_DIR_GPIO_Port GPIOA
#define STEPPER_ENA_Pin GPIO_PIN_10
#define STEPPER_ENA_GPIO_Port GPIOA
#define PROXY_TOOL_D_Pin GPIO_PIN_15
#define PROXY_TOOL_D_GPIO_Port GPIOA
#define PROXY_TOOL_D_EXTI_IRQn EXTI15_10_IRQn
#define PROXY_TOOL_C_Pin GPIO_PIN_3
#define PROXY_TOOL_C_GPIO_Port GPIOB
#define PROXY_TOOL_C_EXTI_IRQn EXTI3_IRQn
#define PROXY_TOOL_B_Pin GPIO_PIN_4
#define PROXY_TOOL_B_GPIO_Port GPIOB
#define PROXY_TOOL_B_EXTI_IRQn EXTI4_IRQn
#define PROXY_TOOL_A_Pin GPIO_PIN_5
#define PROXY_TOOL_A_GPIO_Port GPIOB
#define PROXY_TOOL_A_EXTI_IRQn EXTI9_5_IRQn
#define PROXY_POSITION_Pin GPIO_PIN_6
#define PROXY_POSITION_GPIO_Port GPIOB
#define PROXY_POSITION_EXTI_IRQn EXTI9_5_IRQn
#define PROXY_LOCK_Pin GPIO_PIN_7
#define PROXY_LOCK_GPIO_Port GPIOB
#define PROXY_LOCK_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
