/*
 * my_MotorControl_lib.h
 *
 *  Created on: 2019. 4. 16.
 *      Author: ragalia
 */

#ifndef MY_INTERFACE_MY_MOTORCONTROL_LIB_H_
#define MY_INTERFACE_MY_MOTORCONTROL_LIB_H_

#include <stdint.h>


void Motor_init();


void Motor_Tinit();


void Motor_Test(int16_t Roll);


void Motor_Control(uint16_t Thrust, int16_t Roll, int16_t Pitch, int16_t Yaw, int AThrust, int sa, int sc);


//MotorESC calibration
void MotorESCcalibration(uint16_t Thrust);

#endif /* MY_INTERFACE_MY_MOTORCONTROL_LIB_H_ */
