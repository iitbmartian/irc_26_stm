/*
 * asc.h
 *
 *  Created on: Oct 31, 2025
 *      Author: Radhika Agarwal
 */

#include "main.h"

#ifndef INC_ASC_H_
#define INC_ASC_H_

#define Current_Threshold 6 // Current threshold in Ampere (adjust as needed)
#define ADC_MAX_VALUE 4095.0f   // 12-bit ADC
#define CAP_DURATION 200 // in milliseconds

void Start_ADC_DMA_All(void);
void ADC_Reading(void);

#endif /* INC_ASC_H_ */
