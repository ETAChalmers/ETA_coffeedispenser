/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#define DRV_PWM_CH3N_Pin GPIO_PIN_0
#define DRV_PWM_CH3N_GPIO_Port GPIOF
#define ENC_B_Pin GPIO_PIN_0
#define ENC_B_GPIO_Port GPIOA
#define ENC_BA1_Pin GPIO_PIN_1
#define ENC_BA1_GPIO_Port GPIOA
#define TUNE_Pin GPIO_PIN_4
#define TUNE_GPIO_Port GPIOA
#define TUNE_EXTI_IRQn EXTI4_IRQn
#define DRV_ENABLE_Pin GPIO_PIN_5
#define DRV_ENABLE_GPIO_Port GPIOA
#define S_INDICATOR_Pin GPIO_PIN_7
#define S_INDICATOR_GPIO_Port GPIOA
#define CUPBUS_1_Pin GPIO_PIN_0
#define CUPBUS_1_GPIO_Port GPIOB
#define BREW_BTN_Pin GPIO_PIN_1
#define BREW_BTN_GPIO_Port GPIOB
#define BREW_BTN_EXTI_IRQn EXTI1_IRQn
#define DRV_PWM_CH3_Pin GPIO_PIN_10
#define DRV_PWM_CH3_GPIO_Port GPIOA
#define ENC_I_Pin GPIO_PIN_3
#define ENC_I_GPIO_Port GPIOB
#define ENC_I_EXTI_IRQn EXTI3_IRQn
#define ACT_ARM_Pin GPIO_PIN_4
#define ACT_ARM_GPIO_Port GPIOB
#define ACT_VALVE_Pin GPIO_PIN_5
#define ACT_VALVE_GPIO_Port GPIOB
#define CUPBUS_3_Pin GPIO_PIN_6
#define CUPBUS_3_GPIO_Port GPIOB
#define CUPBUS_2_Pin GPIO_PIN_7
#define CUPBUS_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
