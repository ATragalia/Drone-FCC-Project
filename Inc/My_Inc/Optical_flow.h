/*
 * Optical_flow.h
 *
 *  Created on: 2019. 6. 9.
 *      Author: ragalia
 */

#ifndef MY_INTERFACE_OPTICAL_FLOW_H_
#define MY_INTERFACE_OPTICAL_FLOW_H_

#define PX4FLOW_ADDRESS (0x42 << 1)

#include <i2c.h>
#include <stdint.h>



/*
typedef struct i2c_integral_frame
{
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
} i2c_integral_frame;
*/

void pointer_value_set();

void Read_iframe(uint8_t senser_Add);

void Separate_data();

#endif /* MY_INTERFACE_OPTICAL_FLOW_H_ */
