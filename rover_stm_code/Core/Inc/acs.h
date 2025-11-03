/*
 * asc.h
 *
 *  Created on: Oct 31, 2025
 *      Author: Radhika Agarwal
 */

#include "main.h"

#ifndef INC_ASC_H_
#define INC_ASC_H_

#define Current_Threshold  500    // Current threshold in mA (adjust as needed)
#define ADC_MAX_VALUE 4095   // 12-bit ADC
#define CAP_DURATION 100 // in ms

#define ADC1_CH0 ADC_CHANNEL_1
#define ADC1_CH1 ADC_CHANNEL_2
#define ADC1_CH2 ADC_CHANNEL_3
#define ADC1_CH3 ADC_CHANNEL_4

#define ADC2_CH0 ADC_CHANNEL_1
#define ADC2_CH1 ADC_CHANNEL_2
#define ADC2_CH2 ADC_CHANNEL_3
#define ADC2_CH3 ADC_CHANNEL_4

void Read_All_ADC(void);
void Calculate_Currents(void);
void Set_Cap_Timestamp(uint8_t channel);
void Update_Cap_Timers(void);
uint32_t Voltage_To_Current(uint16_t adc_val);


#endif /* INC_ASC_H_ */
