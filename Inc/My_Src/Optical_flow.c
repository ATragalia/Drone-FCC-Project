/*
 * Optical_flow.c
 *
 *  Created on: 2019. 6. 9.
 *      Author: ragalia
 */


#include "My_Inc/Optical_flow.h"


uint16_t frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
int16_t pixel_flow_x_integral;//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
int16_t pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
int16_t gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000]
int16_t gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000]
int16_t gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000]
uint32_t integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
uint32_t sonar_timestamp;// time since last sonar update [microseconds]
int16_t ground_distance;// Ground distance in meters*1000 [meters*1000]
int16_t gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
uint8_t quality;// averaged quality of accumulated flow values [0:bad quality;255: max quality]

uint8_t data_buffer[26];
uint8_t samplE1,samplE2,samplE3,samplE4,samplE5,samplE6,samplE7,samplE8,samplE9,samplE10;

//uint8_t *calc_O1 = &frame_count_since_last_readout;

void pointer_value_set()
{
	//calc_O1 += 1;
}

void Read_iframe(uint8_t senser_Add)
{
	data_buffer[0] = 0x16;
	//HAL_I2C_Mem_Read(&hi2c1, senser_Add, 0x16, I2C_MEMADD_SIZE_8BIT, &data_buffer[0], 26, 100);
	HAL_I2C_Master_Transmit(&hi2c1, senser_Add, &data_buffer[0], 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, (senser_Add | 0x01), data_buffer, 26, 3000);

}

void Separate_data()
{

	//*calc_O1 = data_buffer[1];
	samplE1 = data_buffer[2];
	samplE2 = data_buffer[3];
	samplE3 = data_buffer[4];
	samplE4 = data_buffer[5];
	samplE5 = data_buffer[6];
	samplE6 = data_buffer[7];
	samplE7 = data_buffer[8];
	samplE8 = data_buffer[9];
	samplE9 = data_buffer[10];
	samplE10 = data_buffer[11];

}
