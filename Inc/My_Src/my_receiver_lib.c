/*
 * receiver.c
 *
 *  Created on: 2019. 5. 20.
 *      Author: ragalia
 */


#include "stdint.h"
#include "My_Inc/my_receiver_lib.h"
#include "My_Inc/s_busProtocol.h"
#include "My_Inc/GPS.h"
#include "My_Inc/TinyGPS.h"

uint8_t RxBuffer[Rx_Buffer_Size];
uint8_t UTemp_Buffer[10];
int rx_Cnt = 0;
int res_flag = 0;
uint8_t S_data[25];
uint16_t Rx_Len = 0;
float roll, pitch, yaw;
uint16_t throtle;
int Sa, Sb, s1, Sc, start_flag, delay_cnt;
uint8_t sample0, sample1, sample2, sample3, sample4, sample5, sample6, sample7, sample8, sample9, sample10, sample11, sample12;
uint8_t sample13, sample14, sample15, sample16, sample17, sample18, sample19, sample20, sample21, sample22, sample23, sample24;
int coundt1 = 0;

void usart_DMA_init()
{
	HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
	HAL_UART_Receive_IT(&huart3, UTemp_Buffer, 1);
	pointValueInit();
	TinyGPS();
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart -> Instance == USART3)
	{
		if(encode(UTemp_Buffer[0]))
		{
			getGPS();
		}
		HAL_UART_Receive_IT(&huart3, UTemp_Buffer, 1);
		return ;
	}

	if(huart -> Instance == USART1)
	{
		if(RxBuffer[0] == 0x0f)res_flag = 1;
		if(res_flag == 1){
			S_data[rx_Cnt++] = RxBuffer[0];
			if(rx_Cnt == 24 && S_data[24] == 0 && RxBuffer[0] == 0x00)
			{
				rx_Cnt = 0;
				res_flag = 0;
			}
		}
		if(rx_Cnt > 25){
			coundt1++;
			rx_Cnt = 0;
		}

		/*
		for(int i = 0 ; i < Rx_Buffer_Size ; i++){

			if(RxBuffer[i] == 0x0f && RxBuffer[i+24] == 0x00 && ((RxBuffer[i+23] >> 4) & 0x0f) == 0x00){
				coundt1 = i;

				/*if(i != 0)
				{
					HAL_UART_DMAStop(&huart1);
					HAL_Delay(10);
					HAL_UART_Receive_DMA(&huart1, RxBuffer, Rx_Buffer_Size);
				}

				for(int j = 0 ; j < 25 ; j++, i++){
					S_data[j] = RxBuffer[i];
				}
				if(S_data[23] == 0x04 || S_data[23] == 0x0c){
					throtle = 0;
					roll = 0.0;
					pitch = 0.0;
					Sa = 0;
					break;
				}
/*
					sample0 = S_data[0];
					sample1 = S_data[1];
					sample2 = S_data[2];
					sample3 = S_data[3];
					sample4 = S_data[4];
					sample5 = S_data[5];
					sample6 = S_data[6];
					sample7 = S_data[7];
					sample8 = S_data[8];
					sample9 = S_data[9];
					sample10 = S_data[10];
					sample11 = S_data[11];
					sample12 = S_data[12];
					sample13 = S_data[13];
					sample14 = S_data[14];
					sample15 = S_data[15];
					sample16 = S_data[16];
					sample17 = S_data[17];
					sample18 = S_data[18];
					sample19 = S_data[19];
					sample20 = S_data[20];
					sample21 = S_data[21];
					sample22 = S_data[22];
					sample23 = S_data[23];
					sample24 = S_data[24];


				S_busProtocol(S_data, &throtle, &roll, &pitch, &Sa, &Sb, &s1, &Sc, &start_flag);
				break;
			}
		}
		*/
		HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
		return;
	}

}


int SignalGet()
{
	return Sc;

}

void CommandGet(uint16_t* Throtle, float* Roll, float* Pitch, int* SA, int* SB, int* S1, int* SC, int* Start_flag)
{
	/*
	if(delay_cnt < 3000)
	{
		roll = 0.0;
		pitch = 0.0;
	}
	*/
	if(throtle != 0)*Throtle = throtle - 172;
	*Roll = roll;
	*Pitch = pitch;
	*SA = Sa;
	*SB = Sb;
	*S1 = s1;
	*SC = Sc;
	*Start_flag = start_flag;

	//if(delay_cnt > 3200 && delay_cnt < 4000) return;

	//if(start_flag == 1) delay_cnt++;
}

void Busprotocol()
{
	S_busProtocol(S_data, &throtle, &roll, &pitch, &Sa, &Sb, &s1, &Sc, &start_flag);
}
