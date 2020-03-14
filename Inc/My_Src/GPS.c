/*
 * GPS.C
 *
 *  Created on: 2019. 10. 27.
 *      Author: ragalia
 */

#include "my_inc/TinyGPS.h"
#include "my_inc/GPS.h"



unsigned char gps_data[1024];
unsigned char temp_data[1024];

float latitude, longitude;
unsigned long fix_age;


/*
void gps_setup()
{
	HAL_UART_Receive_IT(&huart3, temp_data, 1);
	TinyGPS();
	//encode(temp_data[0]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart -> Instance == USART3)
	{
		if(encode(temp_data[0]))
		{
			getGPS();
		}
	}

	HAL_UART_Receive_IT(&huart3, temp_data, 1);

}
*/
void getGPS()
{
	f_get_position(&latitude, &longitude, &fix_age);

}
