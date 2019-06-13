/*
 * encoder.h
 *
 *  Created on: 6 apr. 2018
 *      Author: sjaxel
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stm32f3xx_hal.h"
//#include "stm32f4xx_hal_def.h"

#define ENCODER_CPR 2000
#define ENCODER_SPR 8
#define ENCODER_STEPS_PER_COMPARTMENT ENCODER_CPR / ENCODER_SPR

typedef enum
{
  HAL_ENC_STATE_RESET             = 0x00U,   /*!< Peripheral is not yet Initialized         */
  HAL_ENC_STATE_READY             = 0x1U,   /*!< Peripheral Initialized and ready for use  */
  HAL_ENC_STATE_BUSY              = 0x2U,   /*!< An internal process is ongoing            */
  HAL_ENC_STATE_TIMEOUT           = 0x3U,   /*!< Timeout state                             */
  HAL_ENC_STATE_ERROR             = 0x4U    /*!< Error                                     */
}HAL_ENC_StateTypeDef;

typedef struct
{
  TIM_HandleTypeDef          *Instance;      /*!< I2C registers base address               */
  uint16_t					 True_index;
  uint16_t					 True_index_offset;
  HAL_ENC_StateTypeDef		 State;


}ENC_HandleTypeDef;

HAL_StatusTypeDef HAL_ENC_Init(ENC_HandleTypeDef *hencoder);

HAL_StatusTypeDef HAL_ENC_Reset(ENC_HandleTypeDef *hencoder);

volatile int16_t HAL_ENC_GetPosition(ENC_HandleTypeDef *hencoder);

HAL_StatusTypeDef HAL_ENC_Clear_CNT(ENC_HandleTypeDef *hencoder);

HAL_StatusTypeDef HAL_ENC_Index_Callback(ENC_HandleTypeDef *hencoder);

#endif /* ENCODER_H_ */
