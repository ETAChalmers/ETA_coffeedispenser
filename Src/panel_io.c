/*
 * encoder.c
 *
 *  Created on: 6 apr. 2018
 *      Author: eta
 */

#include "panel_io.h"
#include "stm32f3xx_hal.h"

static uint32_t HAL_PANEL_Read_cupbus(PANEL_HandleTypeDef *hpanel);


HAL_StatusTypeDef HAL_PANEL_Init(PANEL_HandleTypeDef *hpanel) {
	HAL_GPIO_WritePin(hpanel->brew_btn.port, hpanel->brew_btn.pin, GPIO_PIN_SET);
	return HAL_OK;
}

void HAL_PANEL_BrewBTN_CB(PANEL_HandleTypeDef *hpanel) {
	//Read the CUPBUS from the panel and multiply with a scaler
	uint32_t cups = COMPARTMENTS_PER_CUP * HAL_PANEL_Read_cupbus(hpanel);

	//Dispense the cups
	HAL_DRIVER_Dispense(hpanel->hdriver, cups);

}

static uint32_t HAL_PANEL_Read_cupbus(PANEL_HandleTypeDef *hpanel){
	uint32_t cupbus1 = HAL_GPIO_ReadPin(hpanel->cupbus1.port, hpanel->cupbus1.pin);
	uint32_t cupbus2 = HAL_GPIO_ReadPin(hpanel->cupbus2.port, hpanel->cupbus2.pin);
	uint32_t cupbus3 = HAL_GPIO_ReadPin(hpanel->cupbus3.port, hpanel->cupbus3.pin);
	uint32_t cups =	(cupbus1 << 2) | (cupbus2 << 1) | (cupbus3 << 0);
	return cups;
}

