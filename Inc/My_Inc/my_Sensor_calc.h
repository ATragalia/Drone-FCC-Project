/*
 * my_Sensor_calc.h
 *
 *  Created on: 2019. 3. 27.
 *      Author: ragalia
 */

#ifndef MY_INTERFACE_MY_SENSOR_CALC_H_
#define MY_INTERFACE_MY_SENSOR_CALC_H_




#define M_PI_F ((float) M_PI)

//madwick_q_gain
#define BETA_DEF 0.01f  // 2 * proportional gain

//mahony_q_gain
#define TWO_KP_DEF (2.0f * 0.5f )  // 2 * proportional gain
#define TWO_KI_DEF  (2.0f * 0.0f) // 2 * integral gain



//invSqrt
static float invSqrt(float x);


void Yaw_data_get(float mx, float my, float mz, float Roll, float Pitch);

//madwic_q_sensor calc
void sensorcalcQ_madwic(float gx, float gy, float gz, float ax, float ay, float az, float dt);



//mahony_q_sensorcalc
void sensorcalcQ_mahony(float gx, float gy, float gz, float ax, float ay, float az, float dt);



// madwic Q_sensorCalc add pilter
void sensorcalcQ_madwic_addpilter(float gx, float gy, float gz, float ax, float ay, float az, float dt, int flag);



//mahony Q_sensorcalc use 9axis
void sensorcalcQ_mahony_9(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);



// Quaternion to Euler convert
void calc_Q_to_E(float* roll, float* pitch, float* yaw);



//get accZ without gravity
float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az);




#endif /* MY_INTERFACE_MY_SENSOR_CALC_H_ */
