#include "i2C.h"
#include "main.h"

extern TIM_HandleTypeDef htim6;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern uint8_t TxData_buf[]; //extern TxData_buf in uart code

//angle readings from encoders:
volatile float encoder_angles[NUM_ENCODERS];
volatile uint8_t high_byte;
volatile uint8_t low_byte;
//state of encoder measurement
typedef enum {
    STATE_IDLE,
    STATE_SELECT_CHANNEL,
    STATE_WRITE_REG,
    STATE_READ_DATA,
    STATE_NEXT_ENCODER
} EncoderState_t;

static EncoderState_t current_state = STATE_IDLE;
static uint8_t current_channel = 0;

//Data from I2C
static uint8_t tx_data[2];
static uint8_t rx_data[2];

//I2C busy check
static volatile uint8_t i2C_busy = 0;
volatile uint8_t encoder_ready_flag = 0;
volatile uint16_t raw_angle;

//encoder angle calculation from raw value
float RawToDegrees(uint16_t raw){
    return (float)raw * 360.0f / 4096.0f;
}

//I2C section

void PCA9685_MOTOR_Init(void) {
    uint8_t mode1 = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS_MOTOR, PCA9685_MODE1, 1, &mode1, 1, HAL_MAX_DELAY);
    HAL_Delay(3);
}

void PCA9685_CAM_Init(void) {
    uint8_t mode1 = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS_CAM, PCA9685_MODE1, 1, &mode1, 1, HAL_MAX_DELAY);
    HAL_Delay(3);
}

//Encoder reading
void Encoder_StartReading(void){
	if (i2C_busy){ //already reading values
		return;
	}
	current_channel = 0;
	current_state = STATE_SELECT_CHANNEL;
	i2C_busy = 1; //reading reading flag

	Encoder_ReadValues();
}

void Encoder_ReadValues(void){
	switch(current_state){
	case STATE_IDLE:
		break;
	case STATE_SELECT_CHANNEL:
		tx_data[0] = (1 << current_channel); //select k channel: give k << 1
		if(HAL_I2C_Master_Transmit_IT(&hi2c2, TCA9548A_ADDRESS, tx_data,1) == HAL_OK){ //non_blocking reading of angle. send pointer of data buffer tx_data and send 1 byte. also check for HAL_OK return
			current_state = STATE_WRITE_REG;
		}
		break;
	case STATE_WRITE_REG:
		tx_data[0] = AS5600_ANGLE_H;
		if(HAL_I2C_Master_Transmit_IT(&hi2c2, AS5600_ADDRESS, tx_data, 1) == HAL_OK){
			current_state = STATE_READ_DATA;
		}
		break;
	case STATE_READ_DATA:
		if(HAL_I2C_Master_Receive_IT(&hi2c2, AS5600_ADDRESS, rx_data, 2) == HAL_OK){
			current_state = STATE_NEXT_ENCODER;
		}
		break;
	case STATE_NEXT_ENCODER:
		raw_angle = (((uint16_t)rx_data[0] << 8) | ((uint16_t)rx_data[1])) & 0x0FFF; //rx_data[0] has higher 8 bits of which last 4 are useful. rx_data[1] has lower 8 bits. This just combines both into a single uint16_t
		encoder_angles[current_channel] = RawToDegrees(raw_angle);

		current_channel++; //move to next encoder

		if (current_channel >= NUM_ENCODERS) { //reset all flags

			for(int i = 0; i < NUM_ENCODERS; i++){
				//current abc.de --> abcde (16 bit integer). then high_byte is the top 8 bits and low_byte is lower 8 bits. Both are sent to TxData_buf
				high_byte = (uint16_t)(encoder_angles[i]*100) >> 8;
				low_byte = ((uint16_t)(encoder_angles[i]*100)) & 0xFF;
				TxData_buf[2*i] = high_byte;
				TxData_buf[2*i+1] = low_byte;
				//Final TxData_buf is [enc1_high,enc1_low,enc2_high,enc2_low,enc3_high,enc3_low ... ]
			}
			current_state = STATE_IDLE;
			i2C_busy = 0;
			encoder_ready_flag = 1;  //flag that all encoders are ready
		}
		else {
			// Read next encoder
			current_state = STATE_SELECT_CHANNEL;
			Encoder_ReadValues();
		}
		break;
	}
}



// Set PWM frequency for Motors
void PCA9685_MOTOR_SetFrequency(uint16_t freq) {
    uint8_t prescale = (uint8_t)(25000000.0 / (4096 * freq) - 1);

    uint8_t oldmode;
    HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS_MOTOR, PCA9685_MODE1, 1, &oldmode, 1, HAL_MAX_DELAY);

    uint8_t newmode = (oldmode & 0x7F) | 0x10; // Sleep (make reset 0 and sleep bit 1)
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS_MOTOR, PCA9685_MODE1, 1, &newmode, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS_MOTOR, PCA9685_PRESCALE, 1, &prescale, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS_MOTOR, PCA9685_MODE1, 1, &oldmode, 1, HAL_MAX_DELAY);
    HAL_Delay(3);

    uint8_t mode = oldmode | 0xA0; // Auto-increment
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS_MOTOR, PCA9685_MODE1, 1, &mode, 1, HAL_MAX_DELAY);
}


void PCA9685_CAM_SetFrequency(uint16_t freq) {
    uint8_t prescale = (uint8_t)(25000000.0 / (4096 * freq) - 1);

    uint8_t oldmode;
    HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS_CAM, PCA9685_MODE1, 1, &oldmode, 1, HAL_MAX_DELAY);

    uint8_t newmode = (oldmode & 0x7F) | 0x10; // Sleep (make reset 0 and sleep bit 1)
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS_CAM, PCA9685_MODE1, 1, &newmode, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS_CAM, PCA9685_PRESCALE, 1, &prescale, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS_CAM, PCA9685_MODE1, 1, &oldmode, 1, HAL_MAX_DELAY);
    HAL_Delay(3);

    uint8_t mode = oldmode | 0xA0; // Auto-increment
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS_CAM, PCA9685_MODE1, 1, &mode, 1, HAL_MAX_DELAY);
}

//on is count out of 4095 on which rising edge occurs, off is count on which falling edge occurs (both are uint16_t).
// Set PWM on a channel (0-15)
void PCA9685_MOTOR_SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t data[4];
    data[0] = on & 0xFF; //last 8 bits on LED_ON_L
    data[1] = on >> 8; //top 8 (last 4 of those) on LED_ON_H
    data[2] = off & 0xFF; //last 8 bits on LED_OFF_L
    data[3] = off >> 8; //top 8 (last 4 of those) on LED_OFF_H

    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS_MOTOR, PCA9685_LED0_ON_L + 4*channel, 1, data, 4, HAL_MAX_DELAY);
}

// Set PWM on a channel (0-15)
void PCA9685_CAM_SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t data[4];
    data[0] = on & 0xFF; //last 8 bits on LED_ON_L
    data[1] = on >> 8; //top 8 (last 4 of those) on LED_ON_H
    data[2] = off & 0xFF; //last 8 bits on LED_OFF_L
    data[3] = off >> 8; //top 8 (last 4 of those) on LED_OFF_H

    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS_CAM, PCA9685_LED0_ON_L + 4*channel, 1, data, 4, HAL_MAX_DELAY);
}

//I2C2 for encoder callback functions
//automatic callback Tx
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        Encoder_ReadValues();  // re-run case statement function
    }
}

//automatic callback Rx
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        Encoder_ReadValues();  // re-run case statement function
    }
}

//periodic trigger of entire encoder value reading (10ms for now --> Prescalar 7199 of TIM6)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // If using TIM6 for encoder updates
    if (htim->Instance == TIM6) {
        Encoder_StartReading();  // trigger new reading cycle
    }
}

//end callback functions


