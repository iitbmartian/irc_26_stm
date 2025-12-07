#include "main.h"
#include "acs.h"
#include "uart.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern uint8_t TxData_buf[]; //extern TxData_buf in uart code

//TxData_buf[0] to TxData_buf[12*NUM_QUAD + 12 + 2*NUM_ENCODER - 1] are used already, start recording from index [12*NUM_QUAD + 12 + 2*NUM_ENCODER]

//For ADC, Vout = current*0.1 + 2.5 volts
// V to ADC = (current*0.1 + 2.5) * 1/3
// ADC input is 0 at 0 Volt and 4096 at 3.3 volts
// At current max we want ADC cut as:

uint16_t adc_cut_value = (uint16_t)((Current_Threshold*0.1f + 2.5f)*(2.0f/3.0f))*(ADC_MAX_VALUE/3.3f);

volatile uint8_t motor_overcurrent_flags[NUM_ACS] = {0};
volatile uint32_t motor_overcurrent_timestamp[NUM_ACS] = {0};

uint32_t adc1_buf[4];
uint32_t adc2_buf[5];

void Start_ADC_DMA_All(void)
{
    HAL_ADC_Start_DMA(&hadc1, adc1_buf, 4);
    HAL_ADC_Start_DMA(&hadc2, adc2_buf, 5);

    // Start_ADC_DMA_All(); --> might need cyclic nature
}


//Timer callback in stepper.c since the callback function must be unique
void ADC_Reading(void)
{
	for(int i = 0; i < 9; i++)
	{
	    if(HAL_GetTick() - motor_overcurrent_timestamp[i] > CAP_DURATION){
	    	motor_overcurrent_flags[i] = 0;
	    }
	}

	for(int i = 0 ; i < 4; i++)
	{
		uint16_t val = adc1_buf[i];
		TxData_buf[12*(NUM_QUAD + 1) + 2*NUM_ENCODERS + 2*i] = ((val >> 8) & 0xFF); //high byte
		TxData_buf[12*(NUM_QUAD + 1) + 2*NUM_ENCODERS + 2*i + 1] = (val & 0xFF); //low byte
		if((val > adc_cut_value) && (motor_overcurrent_flags[i] == 0)){
			motor_overcurrent_flags[i] = 1;
			motor_overcurrent_timestamp[i] = HAL_GetTick(); //milliseconds from startup
		}
	}
	for(int i = 0 ; i < 5 ; i ++){
		uint16_t val = adc2_buf[i];
		TxData_buf[12*(NUM_QUAD + 1) + 2*NUM_ENCODERS + 8 + 2*i] = ((val >> 8) & 0xFF); //high byte
		TxData_buf[12*(NUM_QUAD + 1) + 2*NUM_ENCODERS + 8 + 2*i + 1] = (val & 0xFF); //low byte
		if((val > adc_cut_value) && (motor_overcurrent_flags[i+4] == 0)){
			motor_overcurrent_flags[i + 4] = 1;
			motor_overcurrent_timestamp[i + 4] = HAL_GetTick(); //milliseconds from startup
		}
	}
}






















