/*
 * stepper.c
 *
 *  Created on: Nov 20, 2025
 *      Author: shrid
 */

#include "stepper.h"
#include "main.h"

_Bool rotate = 0;

void wrist_turn(int command){
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

	if(rotate){

	}
}

