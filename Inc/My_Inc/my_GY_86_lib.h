/*
 * my_GY_86_lib.h
 *
 *  Created on: 2019. 1. 22.
 *      Author: ragalia
 */

#ifndef MY_INTERFACE_MY_GY_86_LIB_H_
#define MY_INTERFACE_MY_GY_86_LIB_H_



#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <i2c.h>


//#define IMU_DEG_PER_LSB_CFG  (float)((2 * 2000.0) / 65536.0)
#define IMU_DEG_PER_LSB_CFG  (float)(1.0 / 16.384)
#define IMU_G_PER_LSB_CFG    (float)((2 * 8) / 65536.0)

//���� ��巹���� ������ ����� ���� ����
#define MPU_6050_ADDRESS (0x68 << 1)
#define HMC5883L_ADDRESS (0x1e << 1)
#define MS5611_ADDRESS (0x77 << 1)
#define CNT_Register (*(volatile unsigned *)0x40014424)
#define PI 3.141592
#define Max_x 251.0
#define Max_y 266.6
#define Low_x -428.0
#define Low_y -285.0


typedef struct
{
	float X;
	float Y;
	float Z;
}axis;


short int Read_reg_8(unsigned char senser_Add, unsigned char Register_Add);						//I2C ������� �������� ������ Ȯ�ο� �Լ�


void Write_reg(unsigned char senser_Add, unsigned char Register_Add, unsigned char data);		//I2C ������� ������ �������Ϳ� ������ �Է¿� �Լ�


void Calc_TP();																					//���� Ķ���� �����͸� ������ ��а� �µ� ���õ����͸� ����ϴ� �Լ�


void MPU_6050_init();									//MPU_6050 �ʱ� �������� ���� �Լ�


void HMC5883L_init();									//HMC5883L �ʱⷹ�����Ϳ� MPU6050 ������ ��� ����


void MS5611_init();													//MS5611 ������ ���� ���� ������ �Լ�


void Read_MPU6050_Data(unsigned char senser_Add, unsigned char Register_Add);					//MPU6050 ���� ������ ���� �Լ�


void Read_HMC5883L_Data(unsigned char senser_Add, unsigned char Register_Add);


void Read_MS5611_Data(int Type);


int32_t Total_Calibration();					//while �� �ۿ��� �ʱ� ���̽� ������ ������ ���� Ķ���극�̼� ����


void Mag_calibration();


void Read_9axisData(axis* Accel, axis* Gyro, axis* Mage);


uint32_t Read_MSData(int cnt);


void DT_Reset();




#endif /* MY_INTERFACE_MY_GY_86_LIB_H_ */
