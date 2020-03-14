/*
 * LPF.h
 *
 *  Created on: 2019. 5. 21.
 *      Author: ragalia
 */

#ifndef MY_INTERFACE_LPF_H_
#define MY_INTERFACE_LPF_H_

#define Tau10hz 0.079577
#define Tau20hz 0.078


#include "My_Inc/my_GY_86_lib.h"


void LPF_Init();

void Calc_LPF(axis *Accel, axis *Gyro, float dt);

void Calc_LPF_PID(int* Din, int Target);

#endif /* MY_INTERFACE_LPF_H_ */


