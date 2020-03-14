/*
 * my_PIDContorl.c
 *
 *  Created on: 2019. 4. 13.
 *      Author: ragalia
 */


#include "My_Inc/my_PIDControl.h"
#include "My_Inc/my_PID_lib.h"



float PID_Thrust_Kp = 1.0;
float PID_Thrust_Ki = 0.5;
float PID_Thrust_Kd = 0.1;

float PID_Roll_Kp = 5.0;
float PID_Roll_Ki = 2.0;
float PID_Roll_Kd = 0.2;


float PID_Pitch_Kp = 5.0;
float PID_Pitch_Ki = 2.0;
float PID_Pitch_Kd = 0.2;

float PID_Yaw_Kp = 5.0;
float PID_Yaw_Ki = 0.0;
float PID_Yaw_Kd = 0.35;


float PID_Roll_Rate_Kp = 10.0;
float PID_Roll_Rate_Ki = 0.0;
float PID_Roll_Rate_Kd = 0.3;

float PID_Pitch_Rate_Kp = 10.0;
float PID_Pitch_Rate_Ki = 0.0;
float PID_Pitch_Rate_Kd = 0.3;

float PID_Yaw_Rate_Kp = 5.0;
float PID_Yaw_Rate_Ki = 1.0;
float PID_Yaw_Rate_Kd = 0.2;

float Roll_Temp = 0.0;



PIDOb pidRollRate;
PIDOb pidPitchRate;
PIDOb pidYawRate;
PIDOb pidRoll;
PIDOb pidPitch;
PIDOb pidYaw;
PIDOb pidThrust;

int16_t rollOutput;
int16_t pitchOutput;
int16_t yawOutput;
int16_t ThrustOut;


int16_t rollOut_last, RollOut_E, R_CNT;

void PIDControl_Init()
{
	pidInit(&pidRoll, 0, PID_Roll_Kp, PID_Roll_Ki, PID_Roll_Kd, 0.002);
	pidInit(&pidPitch, 0, PID_Pitch_Kp, PID_Pitch_Ki, PID_Pitch_Kd, 0.002);
	pidInit(&pidYaw, 0, PID_Yaw_Kp, PID_Yaw_Ki, PID_Yaw_Kd, 0.002);
	pidInit(&pidThrust, 0, PID_Thrust_Kp, PID_Thrust_Ki, PID_Thrust_Kd, 0.002);
	pidSetIntegralLimit(&pidRoll, PID_Roll_integration_limit, -PID_Roll_integration_limit);
	pidSetIntegralLimit(&pidPitch, PID_Pitch_integration_limit, -PID_Pitch_integration_limit);
	pidSetIntegralLimit(&pidYaw, PID_Yaw_integration_limit, -PID_Yaw_integration_limit);

	pidInit(&pidRollRate, 0, PID_Roll_Rate_Kp, PID_Roll_Rate_Ki, PID_Roll_Rate_Kd, 0.002);
	pidInit(&pidPitchRate, 0, PID_Pitch_Rate_Kp, PID_Pitch_Rate_Ki, PID_Pitch_Rate_Kd, 0.002);
	pidInit(&pidYawRate, 0, PID_Yaw_Rate_Kp, PID_Yaw_Rate_Ki, PID_Yaw_Rate_Kd, 0.002);
	pidSetIntegralLimit(&pidRollRate, PID_Roll_Rate_integration_limit, -PID_Roll_Rate_integration_limit);
	pidSetIntegralLimit(&pidPitchRate, PID_Pitch_Rate_integration_limit, -PID_Pitch_Rate_integration_limit);
	pidSetIntegralLimit(&pidYawRate, PID_Yaw_Rate_integration_limit, -PID_Yaw_Rate_integration_limit);

}
int16_t PIDThrustControl(int ThrustIn, float ThrustDesired)
{
	pidSetdesired(&pidThrust, ThrustDesired);
	ThrustOut = pidUpdate(&pidThrust, (float)ThrustIn);
	ThrustOut = -ThrustOut;
	return ThrustOut;

}


void PIDAttitudeControl(
		float RollIn, float PitchIn, float YawIn,
		float RollDesired, float PitchDesired, float YawDesired,
		float* RollRateDesired, float* PitchRateDesired, float* YawRateDesired)
{
	pidSetdesired(&pidRoll, RollDesired);
	*RollRateDesired = pidUpdate(&pidRoll, RollIn);

	pidSetdesired(&pidPitch, PitchDesired);
	*PitchRateDesired = pidUpdate(&pidPitch, PitchIn);

	float yawError;
	yawError = YawDesired - YawIn;
	if(yawError > 180) yawError -=360;
	else if (yawError < -180) yawError += 360;

	pidSetdesired(&pidYaw, yawError);
	*YawRateDesired = pidUpdate(&pidYaw, YawIn);

}


void PIDRateControl(
		float RollRateIn, float PitchRateIn, float YawRateIn,
		float RollRateDesired, float PitchRateDesired, float YawRateDesired)
{
	pidSetdesired(&pidRollRate, RollRateDesired);
	rollOutput = saturateSignedInt16(pidUpdateRate(&pidRollRate, RollRateIn, 1));

	pidSetdesired(&pidPitchRate, PitchRateDesired);
	pitchOutput = saturateSignedInt16(pidUpdateRate(&pidPitchRate, PitchRateIn, 2));

	pidSetdesired(&pidYawRate, YawRateDesired);
	yawOutput = saturateSignedInt16(pidUpdateRate(&pidYawRate, YawRateIn, 3));
}


void PIDContorolAllReset()
{
	pidReset(&pidRoll);
	pidReset(&pidPitch);
	pidReset(&pidYaw);
	pidReset(&pidRollRate);
	pidReset(&pidPitchRate);
	pidReset(&pidYawRate);
}


void PIDdtSet(float dt)
{
	pidSetDt(&pidRoll, dt);
	pidSetDt(&pidRollRate, dt);
	pidSetDt(&pidPitch, dt);
	pidSetDt(&pidPitchRate, dt);
	pidSetDt(&pidYaw, dt);
	pidSetDt(&pidYawRate, dt);
}

void rategainset(int value, int target, int flag)
{

	switch(target){
		case 1:
		{
			if(flag == -1)
				pidSetKp(&pidPitchRate, ((value - 172)*0.01));
			break;
		}
		case 0:
		{
			if(flag == -1)
				pidSetKi(&pidPitchRate, ((value - 172)*0.002) );
			break;
		}
		case -1:
		{
			if(flag == -1)
				pidSetKd(&pidPitchRate, ((value - 172)*0.002) );
			break;
		}
	}

}


void PIDControlOut(int16_t* Roll, int16_t* Pitch, int16_t* Yaw)
{
	*Roll = rollOutput;
	*Pitch = pitchOutput;
	*Yaw = yawOutput;


}


int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}


