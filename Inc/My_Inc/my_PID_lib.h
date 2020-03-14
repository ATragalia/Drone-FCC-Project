/*
 * my_PID_lib.h
 *
 *  Created on: 2019. 2. 7.
 *      Author: ragalia
 */

#ifndef MY_INTERFACE_MY_PID_LIB_H_
#define MY_INTERFACE_MY_PID_LIB_H_



#define PID_Roll_integration_limit 200.0
#define PID_Pitch_integration_limit 200.0
#define PID_Yaw_integration_limit 360.0

#define PID_Roll_Rate_integration_limit 222
#define PID_Pitch_Rate_integration_limit 222
#define PID_Yaw_Rate_integration_limit 166.7

#define Limite_Rate_Roll 5000
#define Limite_Rate_Pitch 5000
#define Limite_Rate_Yaw 5000



typedef struct
{
	float preverror;
	float error;
	float desired;
	float integ;
	float deriv;
	float kp;
	float ki;
	float kd;
	float iLimitH;
	float iLimitL;
	float dt;

} PIDOb;

//PID value Init
void pidInit(PIDOb* pid, const float desired, const float kp, const float ki, const float kd, const float dt);


float pidUpdateRate(PIDOb* pid, float measured, int target);

//PID data update
float pidUpdate(PIDOb* pid, float measured);


//PID Integral Limit set
void pidSetIntegralLimit(PIDOb* pid, const float LimitH, const float LimitL);


//PID value reset
void pidReset(PIDOb* pid);


//PID desired value set
void pidSetdesired(PIDOb* pid, float desired);


//PID kp value set
void pidSetKp(PIDOb* pid, const float kp);


//PID ki value set
void pidSetKi(PIDOb* pid, const float ki);


//PID kd value set
void pidSetKd(PIDOb* pid, const float kd);


//PID dt value set
void pidSetDt(PIDOb* pid, const float dt);




#endif /* MY_INTERFACE_MY_PID_LIB_H_ */
