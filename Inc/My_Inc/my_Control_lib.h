/*
 * my_Control_lib.h
 *
 *  Created on: 2019. 4. 15.
 *      Author: ragalia
 */

#ifndef MY_INTERFACE_MY_CONTROL_LIB_H_
#define MY_INTERFACE_MY_CONTROL_LIB_H_


//IMU, Motor, Receiver Init set
void Total_Init();

void calibration_Loop();


//stabilizer Loop
void stabilizer_Loop();

#endif /* MY_INTERFACE_MY_CONTROL_LIB_H_ */

