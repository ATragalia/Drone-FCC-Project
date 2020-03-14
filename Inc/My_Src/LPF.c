/*
 * LPF.c
 *
 *  Created on: 2019. 5. 21.
 *      Author: ragalia
 */

#include "My_Inc/LPF.h"
#include "math.h"

axis Pre_Accel;
axis Pre_Gyro;
float PreRateRollD, PreRatePitchD, PreRateYawD;
float Sdt;


void LPF_Init()
{
	Pre_Accel.X = 0.0;
	Pre_Accel.Y = 0.0;
	Pre_Accel.Z = 0.0;

	Pre_Gyro.X = 0.0;
	Pre_Gyro.Y = 0.0;
	Pre_Gyro.Z = 0.0;
}

void Calc_LPF(axis* Accel, axis* Gyro, float dt)
{
	Sdt = dt;
/*	Pre_Accel.X = (Tau10hz * Pre_Accel.X + dt * Accel -> X) / (Tau10hz + dt);
	Pre_Accel.Y = (Tau10hz * Pre_Accel.Y + dt * Accel -> Y) / (Tau10hz + dt);
	Pre_Accel.Z = (Tau10hz * Pre_Accel.Z + dt * Accel -> Z) / (Tau10hz + dt);*/

	Pre_Gyro.X = (Tau10hz * Pre_Gyro.X + dt * Gyro -> X) / (Tau10hz + dt);
	Pre_Gyro.Y = (Tau10hz * Pre_Gyro.Y + dt * Gyro -> Y) / (Tau10hz + dt);
	Pre_Gyro.Z = (Tau10hz * Pre_Gyro.Z + dt * Gyro -> Z) / (Tau10hz + dt);

/*	Accel -> X = Pre_Accel.X;
	Accel -> Y = Pre_Accel.Y;
	Accel -> Z = Pre_Accel.Z;*/

	Gyro -> X = Pre_Gyro.X;
	Gyro -> Y = Pre_Gyro.Y;
	Gyro -> Z = Pre_Gyro.Z;
}

void Calc_LPF_PID(int* Din, int Target)
{
	int num = *Din;

	if(Target == 1)
	{
		PreRateRollD = (Tau20hz * PreRateRollD + Sdt * num) / (Tau20hz + Sdt);
		*Din = (int)PreRateRollD;
	}
	if(Target == 2)
	{
		PreRatePitchD = (Tau20hz * PreRatePitchD + Sdt * num) / (Tau20hz + Sdt);
		*Din = (int)PreRatePitchD;
	}
	if(Target == 3)
	{
		PreRateYawD = (Tau20hz * PreRateYawD + Sdt * num) / (Tau20hz + Sdt);
		*Din = (int)PreRateYawD;
	}

}
