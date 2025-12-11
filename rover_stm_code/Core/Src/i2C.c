#include "i2C.h"
#include "main.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef htim6;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart4;
extern uint8_t TxData_buf[]; //extern TxData_buf in uart code
extern _Bool rotate;

extern volatile uint8_t motor_overcurrent_flags[]; //overflow value

//angle readings from encoders:
volatile float encoder_angles[NUM_ENCODERS];
volatile uint8_t high_byte;
volatile uint8_t low_byte;

volatile uint8_t past_value_low = 0;
volatile uint8_t past_value_high = 0;

//Data from I2C
volatile uint8_t tx_data[2];
volatile uint8_t rx_data[2];

//angle value
volatile float raw_angle;

//char err[50];

//encoder angle calculation from raw value
float RawToDegrees(uint16_t raw){
    return (float)raw * 360.0f / 4096.0f;
}

//I2C section

void PCA9685_MOTOR_Init(void) {
    uint8_t mode1 = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS_MOTOR, PCA9685_MODE1, 1, &mode1, 1, HAL_MAX_DELAY);
    HAL_Delay(3);
}

void PCA9685_CAM_Init(void) {
    uint8_t mode1 = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS_CAM, PCA9685_MODE1, 1, &mode1, 1, HAL_MAX_DELAY);
    HAL_Delay(3);
}


void select_mux_channel(uint8_t ch) {
    uint8_t data = (1 << ch);
    HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDRESS, &data, 1, HAL_MAX_DELAY);
//    	sprintf(err, "Switched to channel %d\r\n", ch);
    	//HAL_UART_Transmit(&huart4, (uint8_t *) err, strlen(err), 1000);
    HAL_Delay(10);

}


void read_magnetic_encoder(void)
{
    uint8_t buf[2];

    for (uint8_t i = 0; i < NUM_ENCODERS; i++)
    {
        select_mux_channel(i);

        HAL_StatusTypeDef st =
            HAL_I2C_Mem_Read(&hi2c1,  AS5600_ADDRESS, 0x0E, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);

        HAL_Delay(1);

        if (st != HAL_OK) {
            TxData_buf[2*i] = past_value_high;
            TxData_buf[2*i + 1] = past_value_low;
            continue;
        }

        uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];

        raw &= 0x0FFF;

        raw_angle = RawToDegrees(raw);

        uint16_t scaled = (uint16_t)(raw_angle * 100);

        TxData_buf[12*(NUM_QUAD + 1) + 2*i] = (scaled >> 8) & 0xFF;
        TxData_buf[12*(NUM_QUAD + 1) + 2*i + 1] = scaled & 0xFF;

        past_value_high = (scaled >> 8) & 0xFF;
        past_value_low = scaled & 0xFF;
    }
}

// Set PWM frequency for Motors
void PCA9685_MOTOR_SetFrequency(uint16_t freq) {
    uint8_t prescale = (uint8_t)(25000000.0 / (4096 * freq) - 1);

    uint8_t oldmode;
    HAL_I2C_Mem_Read(&hi2c2, PCA9685_ADDRESS_MOTOR, PCA9685_MODE1, 1, &oldmode, 1, HAL_MAX_DELAY);

    uint8_t newmode = (oldmode & 0x7F) | 0x10; // Sleep (make reset 0 and sleep bit 1)
    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS_MOTOR, PCA9685_MODE1, 1, &newmode, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS_MOTOR, PCA9685_PRESCALE, 1, &prescale, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS_MOTOR, PCA9685_MODE1, 1, &oldmode, 1, HAL_MAX_DELAY);
    HAL_Delay(3);

    uint8_t mode = oldmode | 0xA0; // Auto-increment
    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS_MOTOR, PCA9685_MODE1, 1, &mode, 1, HAL_MAX_DELAY);
}


void PCA9685_CAM_SetFrequency(uint16_t freq) {
    uint8_t prescale = (uint8_t)(25000000.0 / (4096 * freq) - 1);

    uint8_t oldmode;
    HAL_I2C_Mem_Read(&hi2c2, PCA9685_ADDRESS_CAM, PCA9685_MODE1, 1, &oldmode, 1, HAL_MAX_DELAY);

    uint8_t newmode = (oldmode & 0x7F) | 0x10; // Sleep (make reset 0 and sleep bit 1)
    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS_CAM, PCA9685_MODE1, 1, &newmode, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS_CAM, PCA9685_PRESCALE, 1, &prescale, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS_CAM, PCA9685_MODE1, 1, &oldmode, 1, HAL_MAX_DELAY);
    HAL_Delay(3);

    uint8_t mode = oldmode | 0xA0; // Auto-increment
    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS_CAM, PCA9685_MODE1, 1, &mode, 1, HAL_MAX_DELAY);
}

//on is count out of 4095 on which rising edge occurs, off is count on which falling edge occurs (both are uint16_t).
// Set PWM on a channel (0-15)
void PCA9685_MOTOR_SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t data[4];
    data[0] = on & 0xFF; //last 8 bits on LED_ON_L
    data[1] = on >> 8; //top 8 (last 4 of those) on LED_ON_H
    data[2] = off & 0xFF; //last 8 bits on LED_OFF_L
    data[3] = off >> 8; //top 8 (last 4 of those) on LED_OFF_H

    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS_MOTOR, PCA9685_LED0_ON_L + 4*channel, 1, data, 4, HAL_MAX_DELAY);
}

// Set PWM on a channel (0-15)
void PCA9685_CAM_SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t data[4];
    data[0] = on & 0xFF; //last 8 bits on LED_ON_L
    data[1] = on >> 8; //top 8 (last 4 of those) on LED_ON_H
    data[2] = off & 0xFF; //last 8 bits on LED_OFF_L
    data[3] = off >> 8; //top 8 (last 4 of those) on LED_OFF_H

    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS_CAM, PCA9685_LED0_ON_L + 4*channel, 1, data, 4, HAL_MAX_DELAY);
}


