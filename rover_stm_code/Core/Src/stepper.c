/*
 * stepper.c
 *
 *  Created on: Nov 20, 2025
 *      Author: shrid
 */

/*
 * Command Library:
 * 0 : forward
 * 1 : backwards
 * 2 : left
 * 3 : right
 * 4 : do nothing (or anything else)
 */

#include "stepper.h"
#include "main.h"

_Bool rotate = 0;
_Bool step_pulse = 0;

void wrist_turn(uint8_t command){
	switch(command){
	case 0: //both forward
		rotate = 1;
		HAL_GPIO_WritePin(DIR_STEP_1_GPIO_Port, DIR_STEP_1_Pin, 0);
		HAL_GPIO_WritePin(DIR_STEP_2_GPIO_Port, DIR_STEP_2_Pin, 1);
		break;
	case 1: //both backwards
		rotate = 1;
		HAL_GPIO_WritePin(DIR_STEP_1_GPIO_Port, DIR_STEP_1_Pin, 1);
		HAL_GPIO_WritePin(DIR_STEP_2_GPIO_Port, DIR_STEP_2_Pin, 0);
		break;
	case 2: //turn
		rotate = 1;
		HAL_GPIO_WritePin(DIR_STEP_1_GPIO_Port, DIR_STEP_1_Pin, 0);
		HAL_GPIO_WritePin(DIR_STEP_2_GPIO_Port, DIR_STEP_2_Pin, 0);
		break;
	case 3: //turn
		rotate = 1;
		HAL_GPIO_WritePin(DIR_STEP_1_GPIO_Port, DIR_STEP_1_Pin, 1);
		HAL_GPIO_WritePin(DIR_STEP_2_GPIO_Port, DIR_STEP_2_Pin, 1);
		break;
	default: //do nothing
		rotate = 0;
		break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM7){
		if(rotate){
			step_pulse = !step_pulse;
			HAL_GPIO_WritePin(PULSE1_GPIO_Port, PULSE1_Pin, step_pulse);
			HAL_GPIO_WritePin(PULSE2_GPIO_Port, PULSE2_Pin, step_pulse);
		}
		else{
			HAL_GPIO_WritePin(PULSE1_GPIO_Port, PULSE1_Pin, 0);
			HAL_GPIO_WritePin(PULSE2_GPIO_Port, PULSE2_Pin, 0);
		}
	}

    // If using TIM6 for encoder updates
    if (htim->Instance == TIM6) {
        Encoder_StartReading();  // trigger new reading cycle (I2C mux)
    }

    if(htim -> Instance == TIM16){
    	ADC_Reading();
    }
}

