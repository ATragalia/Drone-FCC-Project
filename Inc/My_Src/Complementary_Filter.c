/*
 * Complementary_Filter.c
 *
 *  Created on: 2019. 5. 3.
 *      Author: ragalia
 */


#include <math.h>
#include "My_Inc/Complementary_Filter.h"


float AccXangle, AccYangle, GyroXangle, GyroYangle, CRoll, CPitch, CYaw;

void conmp_Filter_Calc(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float dt)
{

	//AccXangle = (atan2(ay, (sqrt((pow(ax, 2) + pow(az, 2))))) * 57.3);
	//AccYangle = (atan2(ax, (sqrt((pow(ay, 2) + pow(az, 2))))) * 57.3);
	if(mx < 0)
		CYaw = 180 - atan2(my, mx);
	else if(mx > 0 && my < 0)
		CYaw = -atan2(my,mx);
	else if(mx > 0 && my > 0)
		CYaw = 360 - atan2(my,mx);
	else if(mx == 0 && my < 0)
		CYaw = 90;
	else if(mx == 0 && my > 0)
		CYaw = 270;

	//GyroXangle += (gx)*dt;
	//GyroYangle += (gy)*dt;

	//CRoll = (0.99*(CRoll + ((gx) * dt))) + (0.01*AccXangle);
	//CPitch = (0.99*(CPitch + ((gy) * dt))) + (0.01*AccYangle);

}

void Get_9Axis(float* Roll, float*Pitch, float* Yaw)
{
	//*Roll = CRoll;
	//*Pitch = CPitch;
	*Yaw = CYaw;
}
