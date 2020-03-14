/*
 * receiver.h
 *
 *  Created on: 2019. 5. 20.
 *      Author: ragalia
 */

#ifndef INTERFACE_RECEIVER_H_
#define INTERFACE_RECEIVER_H_


#include <usart.h>


#define Rx_Buffer_Size 25


void usart_DMA_init();


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

int SignalGet();


void CommandGet(uint16_t* Throtle, float* Roll, float* Pitch, int* SA, int* SB, int* S1, int* SC, int* Start_flag);


void Busprotocol();


#endif /* INTERFACE_RECEIVER_H_ */
