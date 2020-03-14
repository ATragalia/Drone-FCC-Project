/*
 * s_busProtocol.h
 *
 *  Created on: 2019. 5. 1.
 *      Author: ragalia
 */

#ifndef MY_INTERFACE_S_BUSPROTOCOL_H_
#define MY_INTERFACE_S_BUSPROTOCOL_H_


void pointValueInit();

void S_busProtocol(uint8_t Rx_data[], uint16_t* throtle, float* roll, float* pitch, int* SA, int* SB, int* S1, int* SC, int* Start_flag);


#endif /* MY_INTERFACE_S_BUSPROTOCOL_H_ */
