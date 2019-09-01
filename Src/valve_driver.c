/*
 * valve_driver.c
 *
 *  Created on: 2 sep. 2018
 *      Author: sjaxel
 */

#include <valve_driver.h>


HAL_StatusTypeDef HAL_VALVE_Init(VALVE_HandleTypeDef *hvalve){
	if(hvalve->State != HAL_VALVE_STATE_RESET){
		HAL_VALVE_DeInit(hvalve);
	}
	//Deassert valve driver pin.
	HAL_GPIO_WritePin(hvalve->act_valve_port, hvalve->act_valve_pin, GPIO_PIN_RESET);

	//Clear the valve time buffers
	hvalve->valve_units_total = 0;
	hvalve->valve_units_buffer = 0;

	hvalve->State = HAL_VALVE_STATE_READY;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_VALVE_DeInit(VALVE_HandleTypeDef *hvalve){
	HAL_GPIO_WritePin(hvalve->act_valve_port, hvalve->act_valve_pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(hvalve->htim_valve);
	hvalve->State = HAL_VALVE_STATE_RESET;
	return HAL_OK;
}



HAL_StatusTypeDef HAL_VALVE_Dispense_water(VALVE_HandleTypeDef *hvalve, uint32_t units){
	if(!units || units > MAX_WATER_UNITS){
		return HAL_ERROR;
	}
	if(hvalve->State == HAL_VALVE_STATE_RUNNING){
		hvalve->valve_units_buffer = units;
		return HAL_OK;
	} else if (hvalve->State == HAL_VALVE_STATE_READY) {
		hvalve->State = HAL_VALVE_STATE_RUNNING;
		hvalve->valve_units_total = units;
		hvalve->htim_valve->Instance->ARR = units * VALVE_TIME_PER_UNIT;
		HAL_GPIO_WritePin(hvalve->act_valve_port, hvalve->act_valve_pin, GPIO_PIN_SET);
		__HAL_TIM_CLEAR_FLAG(hvalve->htim_valve, TIM_FLAG_UPDATE);
		return HAL_TIM_Base_Start_IT(hvalve->htim_valve);
	}
	return HAL_ERROR;
}

void HAL_VALVE_callback(VALVE_HandleTypeDef *hvalve){
	uint32_t units;
	uint32_t units_more;
	if(hvalve->valve_units_buffer && (hvalve->valve_units_total < MAX_WATER_UNITS)){
		units = hvalve->valve_units_total + hvalve->valve_units_buffer;
		if(units > MAX_WATER_UNITS){
			units_more = MAX_WATER_UNITS - hvalve->valve_units_total;
			hvalve->valve_units_total = MAX_WATER_UNITS;
		} else {
			units_more = hvalve->valve_units_buffer;
			hvalve->valve_units_total = units;
		}
		hvalve->valve_units_buffer = 0;
		hvalve->htim_valve->Instance->ARR = units_more * VALVE_TIME_PER_UNIT;
		HAL_TIM_Base_Start_IT(hvalve->htim_valve);
		return;
	}
	HAL_GPIO_WritePin(hvalve->act_valve_port, hvalve->act_valve_pin, GPIO_PIN_RESET);
	hvalve->State = HAL_VALVE_STATE_READY;
	return;
}
