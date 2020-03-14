/*
 * my_PID_lib.c
 *
 *  Created on: 2019. 4. 12.
 *      Author: ragalia
 */


#include "My_Inc/my_PID_lib.h"
#include "My_Inc/LPF.h"

int Pre_Err, Err, Filter_Err;



void pidInit(PIDOb* pid, const float desired, const float kp, const float ki, const float kd, const float dt)
{
	pid->error = 0;
	pid->preverror = 0;
	pid->integ = 0;
	pid->deriv = 0;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->desired = desired;
	pid->iLimitH = 2000.0;
	pid->iLimitL = -2000.0;
	pid->dt = dt;
}


float pidUpdateRate(PIDOb* pid, float measured, int target)
{
	float output;

	pid->error = pid->desired - measured;
	/*Err = pid->error;
	if(target == 2)
		Pre_Err = Err;

	switch(target)
			{
				case 1:
				{
					Calc_LPF_PID(&Err, 1);
					break;
				}

				case 2:
				{
					Calc_LPF_PID(&Err, 2);
					break;
				}

				case 3:
				{
					Calc_LPF_PID(&Err, 3);
					break;
				}

			}


	Filter_Err = Err;
	pid->error = Err;*/

	pid->integ += pid->error * pid->dt;

	if(pid->integ > pid->iLimitH)
		pid->integ = pid->iLimitH;
	else if(pid->integ < pid->iLimitL)
		pid->integ = pid->iLimitL;



	pid->deriv = (pid->error - pid->preverror) / pid->dt;



	output = (pid->kp * pid->error) + (pid->ki * pid->integ) + (pid->kd * pid->deriv);

	pid->preverror = pid->error;

	return output;
}



float pidUpdate(PIDOb* pid, float measured)
{
	float output;

	pid->error = pid->desired - measured;

	pid->integ += pid->error * pid->dt;

	if(pid->integ > pid->iLimitH)
		pid->integ = pid->iLimitH;
	else if(pid->integ < pid->iLimitL)
		pid->integ = pid->iLimitL;

	pid->deriv = (pid->error - pid->preverror) / pid->dt;

	output = (pid->kp * pid->error) + (pid->ki * pid->integ) + (pid->kd * pid->deriv);

	pid->preverror = pid->error;

	return output;
}


void pidSetIntegralLimit(PIDOb* pid, const float LimitH, const float LimitL)
{
	pid->iLimitH = LimitH;
	pid->iLimitL = LimitL;
}


void pidReset(PIDOb* pid)
{
	pid->error     = 0;
	pid->preverror = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
}


void pidSetdesired(PIDOb* pid, float desired)
{
	pid->desired = desired;
}


void pidSetKp(PIDOb* pid, const float kp)
{
	pid->kp = kp;
}


void pidSetKi(PIDOb* pid, const float ki)
{
	pid->ki = ki;
}


void pidSetKd(PIDOb* pid, const float kd)
{
	pid->kd = kd;
}


void pidSetDt(PIDOb* pid, const float dt)
{
	pid->dt = dt;
}

