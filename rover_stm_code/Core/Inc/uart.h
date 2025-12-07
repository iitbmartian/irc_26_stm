/*
 * uart. *
 *  Created on: Oct 31, 2025
 *      Author: shrid
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"

#define NUM_QUAD 5 //number of quadrature on timers
#define NUM_ENCODERS 1 //number of AS5600 encoders (1 for testing, 6 in reality)
#define NUM_MOTORS 2
#define NUM_ACS 9 //4 in adc1 and 5 in adc2
#define data_in_length 12 //input byte array
#define data_out_length 12*(NUM_QUAD + 1) + 2*NUM_ENCODERS + 2*NUM_ACS + 1
//FRAME FORMAT
//  12 * num. of quadratures through timers (there are 5 timers)
//+ 12 for drill quadrature input (not through timer)
//+ 2 * AS5600 magnetic encoders
//+ 2 * ACS
//+ 1 * newline


void UART_DMA_START(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void UART_Transmit_DMA(uint8_t *data, uint16_t length);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_UART_H_ */
