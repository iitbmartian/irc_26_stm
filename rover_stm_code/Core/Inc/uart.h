/*
 * uart. *
 *  Created on: Oct 31, 2025
 *      Author: shrid
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"

#define data_in_length 12 //input byte array
#define data_out_length 2 //2*(magnetic encoder number) bytes of encoder feed xx.xx degrees

void UART_DMA_START(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void UART_Transmit_DMA(uint8_t *data, uint16_t length);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_UART_H_ */
