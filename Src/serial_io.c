/*
 * serial_io.c
 *
 *  Created on: 6 nov. 2018
 *      Author: eta
 */

#include "serial_io.h"
#include "disp_driver.h"
#include "stm32f3xx_hal_uart.h"

HAL_StatusTypeDef HAL_SIO_Init(SIO_HandleTypeDef *hsio){
	HAL_UART_Receive_DMA(hsio->huart, (uint8_t *)&hsio->RX_data.sync_package, 1);
	return HAL_OK;
}

void HAL_SIO_RX_Callback(SIO_HandleTypeDef *hsio){
	switch (hsio->RX_data.identifier){
		case TOKEN_REQUEST_DATA :
			hsio->TX_data.identifier = TOKEN_POS_DATA;
			*(int16_t *)(&hsio->TX_data.d1) = (int16_t)HAL_ENC_GetPosition(hsio->hdriver->encoder);
			HAL_UART_Transmit_DMA(hsio->huart, (uint8_t *)&hsio->TX_data, 4);
			break;
		case TOKEN_START_PWM :
			HAL_DRIVER_Start(hsio->hdriver);
		case TOKEN_SET_DUTY :
			HAL_DRIVER_Set_speed(hsio->hdriver, (int8_t)hsio->RX_data.d1);
			break;
		case TOKEN_STOP_PWM :
			HAL_DRIVER_Stop(hsio->hdriver);
			break;
		case TOKEN_CLEAR_CNT :
			HAL_ENC_Clear_CNT(hsio->hdriver->encoder);
			break;
	}
	return;
}
