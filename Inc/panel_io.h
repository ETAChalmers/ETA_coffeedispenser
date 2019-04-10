/*
 * encoder.h
 *
 *  Created on: 6 apr. 2018
 *      Author: sjaxel
 */

#ifndef PANEL_IO_H_
#define PANEL_IO_H_

#include "stm32f3xx_hal.h"
#include "disp_driver.h"

#define COMPARTMENTS_PER_CUP 2

typedef enum
{
  HAL_PANEL_STATE_RESET             = 0x00U,   /*!< Peripheral is not yet Initialized         */
  HAL_PANEL_STATE_READY             = 0x1U,   /*!< Peripheral Initialized and ready for use  */
  HAL_PANEL_STATE_BUSY              = 0x2U,   /*!< An internal process is ongoing            */
  HAL_PANEL_STATE_TIMEOUT           = 0x3U,   /*!< Timeout state                             */
  HAL_PANEL_STATE_ERROR             = 0x4U    /*!< Error                                     */
}HAL_PANEL_StateTypeDef;

typedef struct
{
	GPIO_TypeDef		*port;
	uint32_t			pin;
}PANEL_GPIO;


typedef struct
{
	DRIVER_HandleTypeDef		*hdriver;
	PANEL_GPIO					cupbus1;
	PANEL_GPIO					cupbus2;
	PANEL_GPIO					cupbus3;
	PANEL_GPIO					brew_btn;
	HAL_PANEL_StateTypeDef		State;
}PANEL_HandleTypeDef;

HAL_StatusTypeDef HAL_PANEL_Init(PANEL_HandleTypeDef *hpanel);

void HAL_PANEL_BrewBTN_CB(PANEL_HandleTypeDef *hpanel);




#endif /* PANEL_IO_H_ */
