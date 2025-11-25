#include "main.h"
#include "acs.h"

//uint16_t adc1_values[4] = {0};
//uint16_t adc2_values[5] = {0};
//uint32_t current_ma[8] = {0};
//uint32_t cap_timestamp[8] = {0};    // Timestamp when PWM was capped (in ms)
//volatile uint8_t overcurrent_flags[8] = {0};
//volatile uint8_t current = {0};
//
//extern ADC_HandleTypeDef hadc1;
//extern ADC_HandleTypeDef hadc2;
//
//void Read_All_ADC(void){
//	for(int i =0; i<4; i++){
//		if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK){ /*Polling which means we are continuously checking if the conversion to a digital value is done
//		 Waits 100ms before timeout*/
//			adc1_values[i] = HAL_ADC_GetValue(&hadc1);
//		}
//		if (current_ma[i] > Current_Threshold)
//		{
//			overcurrent_flags[i] = 1;
//		}
//	}
//
//	for(int i =0; i<5; i++){
//		if (HAL_ADC_PollForConversion(&hadc2, 100) == HAL_OK){
//			adc1_values[i] = HAL_ADC_GetValue(&hadc2);
//		}
//		if (current_ma[i + 4] > Current_Threshold){
//			overcurrent_flags[i + 4] = 1;
//		}
//	}
//}
//
//uint32_t Voltage_To_Current(uint16_t adc_val)
//{
//    uint32_t current = 10*(adc_val*3/2 - 2.5);
//    return current;
//}
//
//void Calculate_Current(void){
//    for (int i = 0; i < 4; i++)
//    {
//        current_ma[i] = Voltage_To_Current(adc1_values[i]);
//
//        if (current_ma[i] > Current_Threshold)
//            overcurrent_flags[i] = 1;
//        else
//            overcurrent_flags[i] = 0;
//    }
//
//    for (int i = 0; i < 4; i++)
//    {
//        current_ma[i + 4] = Voltage_To_Current(adc2_values[i]);
//
//        if (current_ma[i + 4] > Current_Threshold)
//            overcurrent_flags[i + 4] = 1;
//        else
//            overcurrent_flags[i + 4] = 0;
//    }
//}
//
//void Update_Cap_Timers(void)
//{
//    uint32_t current_time = HAL_GetTick();  // Get current system time in ms
//
//    for (int i = 0; i < 8; i++)
//    {
//        if (cap_timestamp[i] != 0)
//        {
//            if ((current_time - cap_timestamp[i]) >= CAP_DURATION)
//            {
//                cap_timestamp[i] = 0;   // Clear timestamp
//            }
//        }
//    }
//}
//
//void Set_Cap_Timestamp(uint8_t channel)
//{
//    cap_timestamp[channel] = HAL_GetTick();
//}

volatile uint32_t adc1_buf[4];
volatile uint32_t adc2_buf[5];

uint32_t sequence_adc1[4] = {
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_6
};

uint32_t sequence_adc2[5] = {
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4,
    ADC_CHANNEL_11
};

void ADC1_StartSequence(void)
{
    current_adc1_index = 0;
    ADC1_SetChannel(sequence_adc1[current_adc1_index]);
    HAL_ADC_Start_DMA(&hadc1, &adc1_buf[current_adc1_index], 1);
}

void ADC2_StartSequence(void)
{
    current_adc2_index = 0;
    ADC2_SetChannel(sequence_adc2[current_adc2_index]);
    HAL_ADC_Start_DMA(&hadc2, &adc2_buf[current_adc2_index], 1);
}

void ADC1_SetChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef s = {0};
    s.Channel = channel;
    s.Rank = 1;
    s.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &s);
}

void ADC2_SetChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef s = {0};
    s.Channel = channel;
    s.Rank = 1;
    s.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc2, &s);
}
























