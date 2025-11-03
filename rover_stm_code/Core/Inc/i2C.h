/*
 * i2C.h
 *
 *  Created on: Oct 31, 2025
 *      Author: shrid
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "main.h"

//I2C address are 7 bit. 0x40 is 0b1000000 (MSB of 4 is cut out). Shift left because LSB is 0/1 for read or write onto module
#define PCA9685_ADDRESS_MOTOR (0x40 << 1) //default address at I2C bus 1
#define PCA9685_ADDRESS_CAM (0x41 << 1) //solder address pad 1
#define AS5600_ADDRESS (0x36 << 1)
#define AS5600_ANGLE_H 0x0E	//lower 4 of upper 8 bit register
#define AS5600_ANGLE_L 0x0F //lower 8 bits of angle (12 bit)
#define NUM_ENCODERS 6 //number of AS5600 encoders
#define TCA9548A_ADDRESS (0x70 << 1)
#define PCA9685_PRESCALE 0xFE
#define PCA9685_MODE1 0x00 //normal working --> mode1
#define PCA9685_LED0_ON_L 0x06 //starting register of channel 0 of 16. Each channel has 4 registers


void PCA9685_MOTOR_Init(void);
void PCA9685_CAM_Init(void);
void MUX_ENCODER_Init(void);
void PCA9685_MOTOR_SetFrequency(uint16_t freq);
void PCA9685_CAM_SetFrequency(uint16_t freq);
void PCA9685_MOTOR_SetPWM(uint8_t channel, uint16_t on, uint16_t off);
void PCA9685_CAM_SetPWM(uint8_t channel, uint16_t on, uint16_t off);

float RawToDegrees(uint16_t raw);
void Encoder_StartReading(void);
void Encoder_ReadValues(void);

#endif /* INC_I2C_H_ */
