/*
 * valve_driver.h
 *
 *  Created on: 6 jul. 2019
 *      Author: sjaxel
 */

#ifndef VALVE_DRIVER_H_
#define VALVE_DRIVER_H_

/* Includes ------------------------------------------------------------------*/

#include "stm32f3xx_hal.h"

/* Exported constants ------------------------------------------------------------*/


#define MAX_WATER_UNITS 125					//cl of water
#define VALVE_TIME_PER_UNIT 150				//millisekunder per cl
/* Exported types ------------------------------------------------------------*/


typedef enum {
  HAL_VALVE_STATE_RESET             = 0x00U,    /*!< Peripheral not yet initialized or disabled  */
  HAL_VALVE_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
  HAL_VALVE_STATE_RUNNING           = 0x02U,    /*!< An internal process is ongoing              */
  HAL_VALVE_STATE_ERROR             = 0x03U,      /*!< Reception process is ongoing                */
  HAL_VALVE_STATE_BUSY              = 0x04    /*!< An internal process is ongoing              */
}HAL_VALVE_StateTypeDef;


typedef struct
{

	TIM_HandleTypeDef				*htim_valve;
	uint32_t						act_valve_pin;
	GPIO_TypeDef					*act_valve_port;
	uint32_t						valve_units_total;
	uint32_t						valve_units_buffer;
	HAL_LockTypeDef          		Lock;          /*!< Locking object                    */
  __IO HAL_VALVE_StateTypeDef    	State;         /*!< MIMBOX operation state               */
}VALVE_HandleTypeDef;

/* Exported functions ******************************/

HAL_StatusTypeDef HAL_VALVE_Init(VALVE_HandleTypeDef *hvalve);

HAL_StatusTypeDef HAL_VALVE_DeInit(VALVE_HandleTypeDef *hvalve);

HAL_StatusTypeDef HAL_VALVE_Dispense_water(VALVE_HandleTypeDef *hvalve, uint32_t units);

void HAL_VALVE_callback(VALVE_HandleTypeDef *hvalve);



#endif /* VALVE_DRIVER_H_ */
