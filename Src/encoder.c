/*
 * encoder.c
 *
 *  Created on: 6 apr. 2018
 *      Author: eta
 */

#include "encoder.h"
#include "stm32f3xx_hal.h"

HAL_StatusTypeDef HAL_ENC_Init(ENC_HandleTypeDef *hencoder) {
	if(hencoder->State != HAL_ENC_STATE_RESET){
		return HAL_ERROR;
	}
	if (HAL_TIM_Encoder_Start(hencoder->Instance, TIM_CHANNEL_ALL) == HAL_OK){
		hencoder->State = HAL_ENC_STATE_READY;
		return HAL_OK;
	}
	return HAL_ERROR;
}

volatile int16_t HAL_ENC_GetPosition(ENC_HandleTypeDef *hencoder) {
	if(hencoder->State != HAL_ENC_STATE_READY){
		return HAL_ERROR;
	}
	return hencoder->Instance->Instance->CNT;
}

inline static void HAL_ENC_Adjust_offset(ENC_HandleTypeDef *hencoder, int16_t offset) {

	hencoder->Instance->Instance->CNT += offset;
	return;
}

HAL_StatusTypeDef HAL_ENC_Clear_CNT(ENC_HandleTypeDef *hencoder) {
	if(hencoder->State != HAL_ENC_STATE_READY){
		return HAL_ERROR;
	}
	hencoder->Instance->Instance->CNT = 0;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_ENC_Index_Callback(ENC_HandleTypeDef *hencoder) {
	if(hencoder->State != HAL_ENC_STATE_READY){
		return HAL_ERROR;
	}
	hencoder->True_index = HAL_ENC_GetPosition(hencoder) % 250;

	if(hencoder->True_index != hencoder->True_index_offset){
		HAL_ENC_Adjust_offset(hencoder, hencoder->True_index_offset - hencoder->True_index);
	}

	return HAL_OK;
}

