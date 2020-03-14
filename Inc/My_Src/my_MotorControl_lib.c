/*
 * my_MotorControl_lib.c
 *
 *  Created on: 2019. 4. 20.
 *      Author: ragalia
 */



#include "My_Inc/my_MotorControl_lib.h"
#include "tim.h"

int LMoter1, LMoter2, LMoter3, LMoter4;
int Moter1, Moter2, Moter3, Moter4;
int M_cnt, CCnt;
int E1CNT, E2CNT, E3CNT, E4CNT, E5CNT;
int LThrost;
void Motor_init()
{


	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

	Moter1 = 30000;
	Moter2 = 30000;
	Moter3 = 30000;
	Moter4 = 30000;


	TIM8->CCR1 = 30000;
	TIM8->CCR2 = 30000;
	TIM8->CCR3 = 30000;
	TIM8->CCR4 = 30000;
	HAL_Delay(1000);

}


void MotorESCcalibration(uint16_t Thrust)
{
	//low position = 31000, high position = 61000
	//calibration is OK;

	Moter1 = (uint16_t)(Thrust * 18.3038438 + 30000);
	Moter2 = (uint16_t)(Thrust * 18.3038438 + 30000);
	Moter3 = (uint16_t)(Thrust * 18.3038438 + 30000);
	Moter4 = (uint16_t)(Thrust * 18.3038438 + 30000);
	/*

	Moter1 = (uint16_t)(Thrust * 37.21781574);
	Moter2 = (uint16_t)(Thrust * 37.21781574);
	Moter3 = (uint16_t)(Thrust * 37.21781574);
	Moter4 = (uint16_t)(Thrust * 37.21781574);

	*/



	TIM8->CCR1 = Moter1;
	TIM8->CCR2 = Moter2;
	TIM8->CCR3 = Moter3;
	TIM8->CCR4 = Moter4;
}

void Motor_Tinit()
{
	while(CCnt < 4400){
		CCnt++;
		Moter1++;
		Moter2++;
		Moter3++;
		Moter4++;

		TIM8->CCR1 = Moter1;
		TIM8->CCR2 = Moter2;
		TIM8->CCR3 = Moter3;
		TIM8->CCR4 = Moter4;
	}
}


void Motor_Test(int16_t Roll)
{

	Moter1 -= Roll;
	Moter2 += Roll;
	Moter3 += Roll;
	Moter4 -= Roll;

	TIM8->CCR1 = Moter1;
	TIM8->CCR2 = Moter2;
	TIM8->CCR3 = Moter3;
	TIM8->CCR4 = Moter4;

}


void Motor_Control(uint16_t Thrust, int16_t Roll, int16_t Pitch, int16_t Yaw, int AThrust,  int sa, int sc)
{
	if(sa == -1){
		if(sc == 0)
		{
			Moter1 = (uint16_t)(AThrust) + 31000 - Roll * 2 + Pitch * 2 - Yaw;
			Moter2 = (uint16_t)(AThrust) + 31000 + Roll * 2 - Pitch * 2 - Yaw;
			Moter3 = (uint16_t)(AThrust) + 31000 + Roll * 2 + Pitch * 2 + Yaw;
			Moter4 = (uint16_t)(AThrust) + 31000 - Roll * 2 - Pitch * 2 + Yaw;

			if(Moter1 >= 61000)Moter1 = 61000;
			if(Moter2 >= 61000)Moter2 = 61000;
			if(Moter3 >= 61000)Moter3 = 61000;
			if(Moter4 >= 61000)Moter4 = 61000;

			if(Moter1 <= 31000)Moter1 = 31000;
			if(Moter2 <= 31000)Moter2 = 31000;
			if(Moter3 <= 31000)Moter3 = 31000;
			if(Moter4 <= 31000)Moter4 = 31000;


			TIM8->CCR1 = Moter1;
			TIM8->CCR2 = Moter2;
			TIM8->CCR3 = Moter3;
			TIM8->CCR4 = Moter4;
		}

		else
		{
			if(Thrust > 1700){
				TIM8->CCR1 = 30000;
				TIM8->CCR2 = 30000;
				TIM8->CCR3 = 30000;
				TIM8->CCR4 = 30000;
			}
			else{
				Moter1 = (uint16_t)(Thrust * 18.3038438) + 31000 - Roll * 2 + Pitch * 2 - Yaw;
				Moter2 = (uint16_t)(Thrust * 18.3038438) + 31000 + Roll * 2 - Pitch * 2 - Yaw;
				Moter3 = (uint16_t)(Thrust * 18.3038438) + 31000 + Roll * 2 + Pitch * 2 + Yaw;
				Moter4 = (uint16_t)(Thrust * 18.3038438) + 31000 - Roll * 2 - Pitch * 2 + Yaw;

				if(Moter1 >= 61000)Moter1 = 61000;
				if(Moter2 >= 61000)Moter2 = 61000;
				if(Moter3 >= 61000)Moter3 = 61000;
				if(Moter4 >= 61000)Moter4 = 61000;

				if(Moter1 <= 31000)Moter1 = 31000;
				if(Moter2 <= 31000)Moter2 = 31000;
				if(Moter3 <= 31000)Moter3 = 31000;
				if(Moter4 <= 31000)Moter4 = 31000;


				TIM8->CCR1 = Moter1;
				TIM8->CCR2 = Moter2;
				TIM8->CCR3 = Moter3;
				TIM8->CCR4 = Moter4;
			}
		}
	}
	else{
		TIM8->CCR1 = 30000;
		TIM8->CCR2 = 30000;
		TIM8->CCR3 = 30000;
		TIM8->CCR4 = 30000;
	}
}

/*
void Motor_Control(uint16_t Thrust, int16_t Roll, int16_t Pitch, int16_t Yaw, int sa, int Start_flag)
{
	if(sa == -1){
		if(Start_flag == 1)
		{
			M_cnt++;

		if(M_cnt < 3000){
			TIM8->CCR1 = 30000;
			TIM8->CCR2 = 30000;
			TIM8->CCR3 = 30000;
			TIM8->CCR4 = 30000;
			}
		else{
			if(Thrust > 1700){
				TIM8->CCR1 = 30000;
				TIM8->CCR2 = 30000;
				TIM8->CCR3 = 30000;
				TIM8->CCR4 = 30000;
			}
			else{
				Moter1 = (uint16_t)(Thrust * 18.3038438) + 31000 - Roll * 2 + Pitch * 2 - Yaw;
				Moter2 = (uint16_t)(Thrust * 18.3038438) + 31000 + Roll * 2 - Pitch * 2 - Yaw;
				Moter3 = (uint16_t)(Thrust * 18.3038438) + 31000 + Roll * 2 + Pitch * 2 + Yaw;
				Moter4 = (uint16_t)(Thrust * 18.3038438) + 31000 - Roll * 2 - Pitch * 2 + Yaw;

				if(Moter1 >= 61000)Moter1 = 61000;
				if(Moter2 >= 61000)Moter2 = 61000;
				if(Moter3 >= 61000)Moter3 = 61000;
				if(Moter4 >= 61000)Moter4 = 61000;

				if(Moter1 <= 31000)Moter1 = 31000;
				if(Moter2 <= 31000)Moter2 = 31000;
				if(Moter3 <= 31000)Moter3 = 31000;
				if(Moter4 <= 31000)Moter4 = 31000;


				TIM8->CCR1 = Moter1;
				TIM8->CCR2 = Moter2;
				TIM8->CCR3 = Moter3;
				TIM8->CCR4 = Moter4;
			}
		}
		}
	}
	else{
		TIM8->CCR1 = 30000;
		TIM8->CCR2 = 30000;
		TIM8->CCR3 = 30000;
		TIM8->CCR4 = 30000;
	}
}
*/

