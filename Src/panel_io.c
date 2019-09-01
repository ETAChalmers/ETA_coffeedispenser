/*
 * encoder.c
 *
 *  Created on: 6 apr. 2018
 *      Author: eta
 */

#include "panel_io.h"
#include "disp_driver.h"
#include "valve_driver.h"
#include "stm32f3xx_hal.h"

static uint32_t HAL_PANEL_Read_cupbus(PANEL_HandleTypeDef *hpanel);


HAL_StatusTypeDef HAL_PANEL_Init(PANEL_HandleTypeDef *hpanel) {
	HAL_GPIO_WritePin(hpanel->brew_btn.port, hpanel->brew_btn.pin, GPIO_PIN_SET);
	hpanel->State = HAL_PANEL_STATE_READY;
	return HAL_OK;
}

void HAL_PANEL_BrewBTN_CB(PANEL_HandleTypeDef *hpanel) {
	if(hpanel->State != HAL_PANEL_STATE_READY){
		return;
	}
	hpanel->State = HAL_PANEL_STATE_BUSY;

	if(hpanel->hdriver->State == HAL_DRIVER_STATE_READY){
		//Read the CUPBUS from the panel and multiply with a scaler
		uint32_t cups = HAL_PANEL_Read_cupbus(hpanel);
		uint32_t compartments = COMPARTMENTS_COFFE_PER_CUP * cups;		//Compartment ~11g coffee
		uint32_t water = CL_WATER_PER_CUP * cups;						//centilitres of water per cup
		//Dispense the cups
		if(HAL_DRIVER_Dispense_coffee(hpanel->hdriver, compartments) == HAL_OK){
			HAL_VALVE_Dispense_water(hpanel->hvalve, water);
		}
		hpanel->State = HAL_PANEL_STATE_READY;
		return;
	}


	hpanel->State = HAL_PANEL_STATE_READY;
	return;

}

static uint32_t HAL_PANEL_Read_cupbus(PANEL_HandleTypeDef *hpanel){
	uint32_t cupbus1 = HAL_GPIO_ReadPin(hpanel->cupbus1.port, hpanel->cupbus1.pin);
	uint32_t cupbus2 = HAL_GPIO_ReadPin(hpanel->cupbus2.port, hpanel->cupbus2.pin);
	uint32_t cupbus3 = HAL_GPIO_ReadPin(hpanel->cupbus3.port, hpanel->cupbus3.pin);
	uint32_t cups =	(cupbus1 << 2) | (cupbus2 << 1) | (cupbus3 << 0);
	return cups;
}

