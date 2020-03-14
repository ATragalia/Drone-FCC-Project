/*
 * Complementary_Filter.h
 *
 *  Created on: 2019. 5. 3.
 *      Author: ragalia
 */

#ifndef MY_INTERFACE_COMPLEMENTARY_FILTER_H_
#define MY_INTERFACE_COMPLEMENTARY_FILTER_H_



void conmp_Filter_Calc(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);

void Get_9Axis(float* Roll, float*Pitch, float* Yaw);

#endif /* MY_INTERFACE_COMPLEMENTARY_FILTER_H_ */
