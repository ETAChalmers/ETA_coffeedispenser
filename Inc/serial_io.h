/*
 * serial_io.h
 *
 *  Created on: 6 nov. 2018
 *      Author: eta
 */

#ifndef SERIAL_IO_H_
#define SERIAL_IO_H_

#include "disp_driver.h"
#include "stm32f3xx_hal.h"
#include "arm_math.h"


/* Exported types ------------------------------------------------------------*/

#define SIO_SOF	0x02
#define SIO_LF	0x0A

#define TOKEN_STOP_PWM		0x1
#define TOKEN_START_PWM		0x2
#define TOKEN_SET_DUTY		0x3
#define TOKEN_REQUEST_DATA	0x4
#define TOKEN_POS_DATA		0x5
#define TOKEN_CLEAR_CNT     0x6


typedef enum {
  HAL_SIO_STATE_RESET             = 0x00U,    /*!< Peripheral not yet initialized or disabled  */
  HAL_SIO_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
  HAL_SIO_STATE_RUNNING           = 0x02U,    /*!< An internal process is ongoing              */
  HAL_SIO_STATE_ERROR             = 0x03      /*!< Reception process is ongoing                */
}HAL_SIO_StateTypeDef;


typedef struct{
	uint8_t sync_package;
	uint8_t identifier;
	uint8_t d1;
	uint8_t d2;
}IO_Data;

typedef struct
{
		DRIVER_HandleTypeDef		*hdriver;
		UART_HandleTypeDef			*huart;
		IO_Data						TX_data;
		IO_Data						RX_data;
		HAL_LockTypeDef          	Lock;          /*!< Locking object                    */
	    HAL_DRIVER_StateTypeDef    	State;         /*!< MIMBOX operation state               */
}SIO_HandleTypeDef;

/* Exported functions ******************************/

HAL_StatusTypeDef HAL_SIO_Init(SIO_HandleTypeDef *hsio);

void HAL_SIO_RX_Callback(SIO_HandleTypeDef *hsio);

#endif /* SERIAL_IO_H_ */
