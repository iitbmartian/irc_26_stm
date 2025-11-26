#include "main.h"
#include "uart.h"

extern UART_HandleTypeDef huart4;

//both the buffers are of the same size
uint8_t RxData_buf[data_in_length]; //DMA buffer for reception
volatile uint8_t RxData[data_in_length] = {0}; //copy buffer holding data

volatile uint8_t TxData_buf[data_out_length] = {0};
//
//void UART_DMA_START(void) { //start first receive of UART
//    HAL_UART_Receive_DMA(&huart4, RxData_buf, data_in_length);
//}
//
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	for(int i = 0; i < data_in_length; i++){
//		RxData[i] = RxData_buf[i];
//	}
//	HAL_UART_Receive_DMA(&huart4, RxData_buf, data_in_length);
//}

void UART_Transmit_DMA(uint8_t *data, uint16_t length) {
    // Start UART transmission via DMA (non-blocking)
    HAL_UART_Transmit_DMA(&huart4, data, length);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == huart4.Instance) {
        // Transmission complete logic
    }
}

