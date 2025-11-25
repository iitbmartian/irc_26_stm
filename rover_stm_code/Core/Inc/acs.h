/*
 * asc.h
 *
 *  Created on: Oct 31, 2025
 *      Author: Radhika Agarwal
 */

#include "main.h"

#ifndef INC_ASC_H_
#define INC_ASC_H_

#define Current_Threshold 800    // Current threshold in mA (adjust as needed)
#define ADC_MAX_VALUE 4095   // 12-bit ADC
#define CAP_DURATION 100 // in ms
//void Read_All_ADC(void);
//void Calculate_Currents(void);
//void Set_Cap_Timestamp(uint8_t channel);
//void Update_Cap_Timers(void);
//uint32_t Voltage_To_Current(uint16_t adc_val);

void ADC1_StartSequence(void);
void ADC2_StartSequence(void);
void ADC1_SetChannel(uint32_t channel);
void ADC2_SetChannel(uint32_t channel);

#endif /* INC_ASC_H_ */
