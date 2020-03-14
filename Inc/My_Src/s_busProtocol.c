/*
 * s_BusProtocol.c
 *
 *  Created on: 2019. 5. 1.
 *      Author: ragalia
 */

#include <stdint.h>
#include "My_Inc/s_busProtocol.h"


int16_t T_Throtle, T_Roll, T_Pitch, T_Yaw, T_SA, T_SB, T_SC, T_S1;
float C_Yaw;
int Start_cnt;

uint8_t *calc_T1 = &T_Throtle;  //stick channel
uint8_t *calc_T2 = &T_Roll;
uint8_t *calc_T3 = &T_Pitch;
uint8_t *calc_T4 = &T_Yaw;
uint8_t *calc_T5 = &T_SA;		//switch channel
uint8_t *calc_T6 = &T_SB;
uint8_t *calc_T7 = &T_S1;		//variable switch channel
uint8_t *calc_T8 = &T_SC;



void pointValueInit(){
	  calc_T1 += 1;
	  calc_T2 += 1;
	  calc_T3 += 1;
	  calc_T4 += 1;
	  calc_T5 += 1;
	  calc_T6 += 1;
	  calc_T7 += 1;
	  calc_T8 += 1;
}

void S_busProtocol(uint8_t Rx_data[], uint16_t* throtle, float* roll, float* pitch, int* SA, int* SB, int* S1, int* SC , int* Start_flag)
{

	  T_Throtle = Rx_data[1] ;
	  *calc_T1 = (Rx_data[2] & 0x7) ;
	  *throtle = T_Throtle;

	  T_Roll = Rx_data[2] ;
	  *calc_T2 = Rx_data[3] & 0x3f;
	  T_Roll = T_Roll >>  3;

	  T_Pitch = Rx_data[4];
	  *calc_T3 = Rx_data[5] & 0x1;
	  T_Pitch = (T_Pitch << 2) | ((Rx_data[3] & 0xc0) >> 6);

	  T_Yaw = Rx_data[5];
	  *calc_T4 = Rx_data[6] & 0x0f;
	  T_Yaw = T_Yaw >> 1;

	  T_SA = Rx_data[6];
	  *calc_T5 = Rx_data[7] & 0x7f;
	  T_SA = T_SA >> 4;

	  T_SB = Rx_data[8];
	  *calc_T6 = Rx_data[9] & 0x03;
	  T_SB = (T_SB << 1) | ((Rx_data[7] & 0x80) >> 7);

	  T_S1 = Rx_data[9];
	  *calc_T7 = Rx_data[10] & 0x1f;
	  T_S1 = T_S1 >> 2;
	  *S1 = (int)T_S1;

	  T_SC = Rx_data[10];
	  *calc_T8 = Rx_data[11];
	  T_SC = T_SC >> 5;

	  if(T_Roll >= 994) *roll = 45.0 -  (float)(1811 - T_Roll) * 0.0550796;
	  else *roll = -45.0 + (float)(T_Roll - 172) * 0.0547446;

	  if(T_Pitch <= 924) *pitch = (float)(T_Pitch - 924) * 0.059841;
	  else *pitch = 45.0 - (float)(1811 - T_Pitch) * 0.050733;

	  if(T_SA > 1200)*SA = 1;
	  else if(T_SA > 800)*SA = 0;
	  else if(T_SA > 100)*SA = -1;

	  if(T_SB > 1200)*SB = 1;
	  else if(T_SB > 800)*SB = 0;
	  else if(T_SB > 100)*SB = -1;

	  if(T_SC > 1200)*SC = 1;
	  else if(T_SC > 800)*SC = 0;
	  else if(T_SC > 100)*SC = -1;

	  /*
	  if (T_Roll >= 1800 && T_Pitch <= 180 && T_Throtle <= 180 && T_Yaw <= 180) Start_cnt++;
	  else Start_cnt = 0;

	  if(Start_cnt >= 100) *Start_flag = 1;
	*/
}
