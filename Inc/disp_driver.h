/*
 * disp_driver.h
 *
 *  Created on: 2 sep. 2018
 *      Author: sjaxel
 */

#ifndef DISP_DRIVER_H_
#define DISP_DRIVER_H_

/* Includes ------------------------------------------------------------------*/

#include "encoder.h"
#include "stm32f3xx_hal.h"
#include "arm_math.h"

/* Exported constants ------------------------------------------------------------*/

#define CASCADE_RATIO 10
#define PID_INNER_RATE 1000
#define SPEED_TAU 0.1
#define MA_ALFA (1 / (SPEED_TAU * PID_INNER_RATE))
#define STABLE_CYCLES 10

/* Exported types ------------------------------------------------------------*/

typedef enum {
	CW = 0,
	CCW = 0x1U,
}DRIVER_Direction;

typedef enum {
  HAL_DRIVER_STATE_RESET             = 0x00U,    /*!< Peripheral not yet initialized or disabled  */
  HAL_DRIVER_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
  HAL_DRIVER_STATE_RUNNING           = 0x02U,    /*!< An internal process is ongoing              */
  HAL_DRIVER_STATE_ERROR             = 0x03U,      /*!< Reception process is ongoing                */
  HAL_DRIVER_STATE_BUSY              = 0x04    /*!< An internal process is ongoing              */
}HAL_DRIVER_StateTypeDef;

typedef struct {
	float32_t						inner_setpoint;
	int16_t							outer_setpoint;
	int16_t							position;
	float32_t						avg_speed;
	uint32_t						inner_pid_cycles;
	uint32_t						stable_cycles;
	uint32_t						missed_cycles;
}HAL_DRIVER_System_state;

typedef struct
{
	TIM_HandleTypeDef				*htim_pwm;	   /*!< PWM timer handle                   */
	TIM_HandleTypeDef				*htim_pid;	   /*!< PID rate timer handle                   */
	uint32_t						pwm_ch;
	ENC_HandleTypeDef				*encoder;
	arm_pid_instance_f32			pid_outer;
	arm_pid_instance_f32			pid_inner;
	HAL_DRIVER_System_state			system;
	HAL_LockTypeDef          		Lock;          /*!< Locking object                    */
  __IO HAL_DRIVER_StateTypeDef    	State;         /*!< MIMBOX operation state               */
}DRIVER_HandleTypeDef;

/* Exported functions ******************************/

HAL_StatusTypeDef HAL_DRIVER_Init(DRIVER_HandleTypeDef *hdriver);

HAL_StatusTypeDef HAL_DRIVER_DeInit(DRIVER_HandleTypeDef *hdriver);

HAL_StatusTypeDef HAL_DRIVER_Dispense(DRIVER_HandleTypeDef *hdriver, uint32_t units);

HAL_StatusTypeDef HAL_DRIVER_Start_PID(DRIVER_HandleTypeDef *hdriver, int16_t setpoint);

HAL_StatusTypeDef HAL_DRIVER_Stop_PID(DRIVER_HandleTypeDef *hdriver);

HAL_StatusTypeDef HAL_DRIVER_Update_PID(DRIVER_HandleTypeDef *hdriver);



#endif /* DISP_DRIVER_H_ */
