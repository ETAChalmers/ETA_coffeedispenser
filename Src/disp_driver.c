/*
 * disp_driver.c
 *
 *  Created on: 2 sep. 2018
 *      Author: sjaxel
 */

#include "disp_driver.h"
#include "arm_math.h"
#include "stm32f3xx_hal_tim_ex.h"

float32_t deadzone_lookup[] = {
		-1.003,-0.99578,-0.98856,-0.98134,-0.97412,-0.9669,-0.95968,-0.95246,-0.94524,
		-0.93801,-0.93079,-0.92357,-0.91635,-0.90913,-0.90191,-0.89469,-0.88747,-0.88025,
		-0.87303,-0.86581,-0.85858,-0.85136,-0.84414,-0.83692,-0.8297,-0.82248,-0.81526,
		-0.80804,-0.80082,-0.7936,-0.78637,-0.77915,-0.77193,-0.76471,-0.75749,-0.75027,
		-0.74305,-0.73583,-0.72861,-0.72139,-0.71416,-0.70694,-0.69972,-0.6925,-0.68528,
		-0.67806,-0.67084,-0.66362,-0.6564,-0.64918,-0.64195,-0.63473,-0.62751,-0.62029,
		-0.61307,-0.60585,-0.59863,-0.59141,-0.58419,-0.57697,-0.56974,-0.56252,-0.5553,
		-0.54808,-0.54086,-0.53364,-0.52642,-0.5192,-0.51198,-0.50476,-0.49753,-0.49031,
		-0.48309,-0.47587,-0.46865,-0.46143,-0.45421,-0.44699,-0.43977,-0.43255,-0.42532,
		-0.4181,-0.41088,-0.40366,-0.39644,-0.38922,-0.382,-0.37478,-0.36756,-0.36034,
		-0.35312,-0.34589,-0.33867,-0.33145,-0.32423,-0.31701,-0.30979,-0.30257,-0.29535,
		-0.28813,-0.28091,-0.27368,-0.26646,-0.25924,-0.25202,-0.2448,-0.23758,-0.23036,
		-0.22314,-0.21592,-0.2087,-0.20147,-0.19425,-0.18703,-0.17981,-0.17259,-0.16537,
		-0.15815,-0.15093,-0.14371,-0.13649,-0.12926,-0.12204,-0.11482,-0.1076,-0.10038,
		-0.09316,0,0.09316,0.10038,0.1076,0.11482,0.12204,0.12926,0.13649,0.14371,0.15093,
		0.15815,0.16537,0.17259,0.17981,0.18703,0.19425,0.20147,0.2087,0.21592,0.22314,
		0.23036,0.23758,0.2448,0.25202,0.25924,0.26646,0.27368,0.28091,0.28813,0.29535,
		0.30257,0.30979,0.31701,0.32423,0.33145,0.33867,0.34589,0.35312,0.36034,0.36756,
		0.37478,0.382,0.38922,0.39644,0.40366,0.41088,0.4181,0.42532,0.43255,0.43977,
		0.44699,0.45421,0.46143,0.46865,0.47587,0.48309,0.49031,0.49753,0.50476,0.51198,
		0.5192,0.52642,0.53364,0.54086,0.54808,0.5553,0.56252,0.56974,0.57697,0.58419,
		0.59141,0.59863,0.60585,0.61307,0.62029,0.62751,0.63473,0.64195,0.64918,0.6564,
		0.66362,0.67084,0.67806,0.68528,0.6925,0.69972,0.70694,0.71416,0.72139,0.72861,
		0.73583,0.74305,0.75027,0.75749,0.76471,0.77193,0.77915,0.78637,0.7936,0.80082,
		0.80804,0.81526,0.82248,0.8297,0.83692,0.84414,0.85136,0.85858,0.86581,0.87303,
		0.88025,0.88747,0.89469,0.90191,0.90913,0.91635,0.92357,0.93079,0.93801,0.94524,
		0.95246,0.95968,0.9669,0.97412,0.98134,0.98856,0.99578,1.003
};

static HAL_StatusTypeDef HAL_DRIVER_Set_speed(DRIVER_HandleTypeDef *hdriver, int8_t speed);

static HAL_StatusTypeDef HAL_DRIVER_Start(DRIVER_HandleTypeDef *hdriver);

static HAL_StatusTypeDef HAL_DRIVER_Stop(DRIVER_HandleTypeDef *hdriver);

static HAL_StatusTypeDef HAL_DRIVER_Dispense_water(DRIVER_HandleTypeDef *hdriver, uint32_t time);

static void TIM_CCxNChannelCmd(TIM_TypeDef* TIMx, uint32_t Channel, uint32_t ChannelNState);

static float32_t HAL_DRIVER_Filter_speed(DRIVER_HandleTypeDef *hdriver, float32_t speed_sample);

static float32_t HAL_DRIVER_Saturate(float32_t value, float32_t gain, float32_t lower, float32_t upper);

HAL_StatusTypeDef HAL_DRIVER_Init(DRIVER_HandleTypeDef *hdriver){
	  HAL_GPIO_WritePin(hdriver->drv_enable_port, hdriver->drv_enable_pin, GPIO_PIN_RESET);

	  /* Enable the Capture compare channel */
	  TIM_CCxChannelCmd(hdriver->htim_pwm->Instance, hdriver->pwm_ch, TIM_CCx_ENABLE);
	  TIM_CCxNChannelCmd(hdriver->htim_pwm->Instance, hdriver->pwm_ch, TIM_CCxN_ENABLE);
	  hdriver->State = HAL_DRIVER_STATE_READY;
	  return HAL_OK;
}

HAL_StatusTypeDef HAL_DRIVER_Dispense(DRIVER_HandleTypeDef *hdriver, uint32_t units){
	if(hdriver->State != HAL_DRIVER_STATE_READY){
		return HAL_ERROR;
	} else if (!units || units > MAX_UNITS){
		return HAL_ERROR;
	}
	HAL_DRIVER_Dispense_water(hdriver, units);
	HAL_DRIVER_Start_PID(hdriver, units * ENCODER_STEP);
	return HAL_OK;
}

static HAL_StatusTypeDef HAL_DRIVER_Dispense_water(DRIVER_HandleTypeDef *hdriver, uint32_t units){
	if(units > MAX_WATER_UNITS){
		units = MAX_WATER_UNITS;
	}
	if((hdriver->htim_valve->Instance->CR1 & 1)){
		hdriver->htim_valve->Instance->ARR += units * VALVE_TIME_PER_UNIT;
	} else {
	hdriver->htim_valve->Instance->ARR = units * VALVE_TIME_PER_UNIT;
	}

	HAL_GPIO_WritePin(hdriver->act_valve_port, hdriver->act_valve_pin, GPIO_PIN_SET);
	return HAL_TIM_Base_Start_IT(hdriver->htim_valve);

}


HAL_StatusTypeDef HAL_DRIVER_Update_PID(DRIVER_HandleTypeDef *hdriver){
	if(hdriver->State == HAL_DRIVER_STATE_BUSY){
		if(hdriver->system.missed_cycles++ > MAX_MISSED_CYCLES){
			HAL_DRIVER_Stop_PID(hdriver);
			return HAL_ERROR;
		}
		return HAL_BUSY;
	} else if (hdriver->State != HAL_DRIVER_STATE_RUNNING){
		return HAL_ERROR;
	}
	hdriver->State = HAL_DRIVER_STATE_BUSY;

	float32_t error;

	if(hdriver->system.inner_pid_cycles++ == 0){

	    int16_t error_fixed_point = (hdriver->system.outer_setpoint - hdriver->system.position);

		if(!error_fixed_point){
			if(hdriver->system.stable_cycles > STABLE_CYCLES) {
				return HAL_DRIVER_Stop_PID(hdriver);
			}
			hdriver->system.stable_cycles++;
		} else {
			hdriver->system.stable_cycles = 0;
		}

		error = (float32_t)error_fixed_point;
		hdriver->system.inner_setpoint = HAL_DRIVER_Saturate(
											arm_pid_f32(&(hdriver->pid_outer), error),
											1,
											-5.5,
											5.5);
	}

	//Inner loop
	int16_t new_position = HAL_ENC_GetPosition(hdriver->encoder);	//Current position
	float32_t speed = (new_position - hdriver->system.position)
			* ((2 * 3.1415 * PID_INNER_RATE) / ENCODER_CPR);		//Current in rad/s

	//Update the system position with new value
	hdriver->system.position = new_position;
	//Filter the speed to remove quantization noise.
	float32_t filtered_speed = HAL_DRIVER_Filter_speed(hdriver, speed);
	//Calculate the inner PID error
	error = hdriver->system.inner_setpoint - filtered_speed;
	//Calculate the inner PID output and saturate the normalized value.
	float32_t inner_pid_output = HAL_DRIVER_Saturate(
											arm_pid_f32(&(hdriver->pid_inner), error),
											1,
											-1,
											1);
	//Calculate the lookup constant for the system feed forward value.
	int8_t lookup_ff = HAL_DRIVER_Saturate(
								hdriver->system.inner_setpoint,
								23.3,
								-127,
								127);
	//Calculate the duty cycle by combining the feed forward lookup value and inner pid output.
	//Scale from normalized -1,1 to int8 and saturate.
	float32_t duty_cycle  = HAL_DRIVER_Saturate(
									(deadzone_lookup[(uint8_t)(lookup_ff + 127)] + inner_pid_output),
									127,
									-128,
									127);

	//Check the ratio of pid loops and reset counter greater than CASCADE_RATIO.
	if(hdriver->system.inner_pid_cycles >= CASCADE_RATIO){
		hdriver->system.inner_pid_cycles = 0;
	}

	//Drive the plant.
	HAL_DRIVER_Set_speed(hdriver, (int8_t)duty_cycle);

	hdriver->State = HAL_DRIVER_STATE_RUNNING;
	return HAL_OK;

}

HAL_StatusTypeDef HAL_DRIVER_Start_PID(DRIVER_HandleTypeDef *hdriver, int16_t setpoint){
	hdriver->State = HAL_DRIVER_STATE_RUNNING;
	//Clear the system state.
	HAL_DRIVER_System_state newstate = {0};
	hdriver->system = newstate;

	//Reset the internal state of the inner and outer PID
	arm_pid_init_f32(&hdriver->pid_inner ,1);
	arm_pid_init_f32(&hdriver->pid_outer ,1);

	//Calculate the setpoint from system position and setpoint.
	hdriver->system.outer_setpoint = hdriver->system.position + setpoint;
	//Start the driver outputs and start PID update timer.
	HAL_DRIVER_Start(hdriver);
	return HAL_TIM_Base_Start_IT(hdriver->htim_pid);

}

HAL_StatusTypeDef HAL_DRIVER_Stop_PID(DRIVER_HandleTypeDef *hdriver){
	//Stop the PID update timer
	HAL_TIM_Base_Stop_IT(hdriver->htim_pid);
	HAL_DRIVER_Stop(hdriver);
	hdriver->State = HAL_DRIVER_STATE_READY;
	return HAL_OK;
}


static HAL_StatusTypeDef HAL_DRIVER_Start(DRIVER_HandleTypeDef *hdriver){
	  HAL_GPIO_WritePin(hdriver->drv_enable_port, hdriver->drv_enable_pin, GPIO_PIN_SET);
	  HAL_ENC_Clear_CNT(hdriver->encoder);

	  if(IS_TIM_BREAK_INSTANCE(hdriver->htim_pwm->Instance) != RESET)
	  {
	    /* Enable the main output */
	    __HAL_TIM_MOE_ENABLE(hdriver->htim_pwm);
	  }

	  /* Enable the Peripheral */
	  __HAL_TIM_ENABLE(hdriver->htim_pwm);

	  /* Return function status */
	  return HAL_OK;
}

static HAL_StatusTypeDef HAL_DRIVER_Stop(DRIVER_HandleTypeDef *hdriver){
	  //Deactive the H-bridge.
	  HAL_GPIO_WritePin(hdriver->drv_enable_port, hdriver->drv_enable_pin, GPIO_PIN_RESET);
	  //Deactive the PWM timer.
	  __HAL_TIM_DISABLE(hdriver->htim_pwm);
	  return HAL_OK;;
}

static HAL_StatusTypeDef HAL_DRIVER_Set_speed(DRIVER_HandleTypeDef *hdriver, int8_t speed){
	  hdriver->htim_pwm->Instance->CCR3 = (uint8_t)(speed + 0x80);
	  return HAL_ERROR;
}

static float32_t HAL_DRIVER_Filter_speed(DRIVER_HandleTypeDef *hdriver, float32_t speed_sample){
	  //MA_ALFA is the exponential moving avarage forgetting factor.
	  hdriver->system.avg_speed = (MA_ALFA*speed_sample)+((1-MA_ALFA)*hdriver->system.avg_speed);
	  return hdriver->system.avg_speed;
}

static float32_t HAL_DRIVER_Saturate(float32_t value, float32_t gain, float32_t lower, float32_t upper){
	  value = gain * value;
	  if(value < lower){
		  value = lower;
	  } else if (value > upper) {
		  value = upper;
	  }
	  return value;
}

/**
  * @brief  Enables or disables the TIM Capture Compare Channel xN.
  * @param  TIMx to select the TIM peripheral
  * @param  Channel specifies the TIM Channel
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1
  *            @arg TIM_CHANNEL_2: TIM Channel 2
  *            @arg TIM_CHANNEL_3: TIM Channel 3
  * @param  ChannelNState specifies the TIM Channel CCxNE bit new state.
  *          This parameter can be: TIM_CCxN_ENABLE or TIM_CCxN_Disable.
  * @retval None
  */
static void TIM_CCxNChannelCmd(TIM_TypeDef* TIMx, uint32_t Channel, uint32_t ChannelNState)
{
  uint32_t tmp = 0U;

  tmp = TIM_CCER_CC1NE << Channel;

  /* Reset the CCxNE Bit */
  TIMx->CCER &=  ~tmp;

  /* Set or reset the CCxNE Bit */
  TIMx->CCER |=  (uint32_t)(ChannelNState << Channel);
}


