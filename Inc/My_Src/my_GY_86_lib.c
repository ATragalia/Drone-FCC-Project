/*
 * my_GY_86_lib.c
 *
 *  Created on: 2019. 4. 18.
 *      Author: ragalia
 */


#include "My_Inc/my_GY_86_lib.h"

float Accel_X, Accel_Y, Accel_Z, Temperature;
float Gyro_X, Gyro_Y, Gyro_Z;
float Magnet_X, Magnet_Y, Magnet_Z, MagAngle, Base_Angle, Heading;
float Base_Accel_X, Base_Accel_Y, Base_Accel_Z;
float Base_Gyro_X, Base_Gyro_Y, Base_Gyro_Z;
float Base_Mage_X, Base_Mage_Y, Base_Mage_Z;
int p_flag = 1;
uint8_t cammand[3] = { 0x48, 0x58, 0x00 };						//MS5611 ADC cammand
uint16_t SENS_t1, OFF_t1, TCS, TCO, TREF, TEMPSENS;
uint32_t D_PRESS, D_TEMPER;
int32_t d_T, TEMP, Pressure, base_Pressure, commandA;
int64_t OFF, SENS, T2, OFF2, SENS2;
float mag_mx, mag_my, mag_lx, mag_ly;
float offset_x, offset_y;





short int Read_reg_8(unsigned char senser_Add, unsigned char Register_Add)						//I2C ������� �������� ������ Ȯ�ο� �Լ�
{
	unsigned char Tmp_buffer[1] = { Register_Add };

	HAL_I2C_Master_Transmit(&hi2c1, senser_Add, Tmp_buffer, 1, 100);
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, (senser_Add | 0x01), Tmp_buffer, 1, 200);
	return Tmp_buffer[0];
}


void Write_reg(unsigned char senser_Add, unsigned char Register_Add, unsigned char data)		//I2C ������� ������ �������Ϳ� ������ �Է¿� �Լ�
{
	unsigned char buffer[2] = { Register_Add, data };
	HAL_I2C_Master_Transmit(&hi2c1, senser_Add, buffer, 2, 100);
}


void Calc_TP()																					//���� Ķ���� �����͸� ������ ��а� �µ� ���õ����͸� ����ϴ� �Լ�
{
	d_T = D_TEMPER - TREF * pow(2, 8);
	TEMP = 2000 + d_T * TEMPSENS / pow(2, 23);

	OFF = OFF_t1 * pow(2, 16) + (TCO * d_T) / pow(2, 7);
	SENS = SENS_t1 * pow(2, 15) + (TCS * d_T) / pow(2, 8);


	//�µ� �����Ϳ� ���� �ι�° ������ ������ ���
	if (TEMP < 2000) {
		T2 = pow(d_T, 2) / pow(2, 31);
		OFF2 = 5 * pow((TEMP - 2000), 2) / 2;
		SENS2 = 5 * pow((TEMP - 2000), 2) / 4;
		if (TEMP < -1500) {
			OFF2 = OFF2 + 7 * pow((TEMP + 1500), 2);
			SENS2 = SENS2 + 11 * pow((TEMP + 1500), 2) / 2;
		}
	}
	else {
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}

	TEMP -= T2;
	OFF -= OFF2;
	SENS -= SENS2;

	Pressure = (D_PRESS * SENS / pow(2, 21) - OFF) / pow(2, 15);

}


void MPU_6050_init()									//MPU_6050 �ʱ� �������� ���� �Լ�
{
	Write_reg(MPU_6050_ADDRESS, 0x6b, 0x00);			//sleep mode disable
	Write_reg(MPU_6050_ADDRESS, 0x19, 0x00);			//sample rate setting	1khz / 1 = 1Khz
	Write_reg(MPU_6050_ADDRESS, 0x1a, 0x06);			//DLPF setting 5hz, 18.6ms,  total sample rate = 1khz
	Write_reg(MPU_6050_ADDRESS, 0x1b, 0x18);			//gyroscope full scale	+- 2000dps		131LSB
	Write_reg(MPU_6050_ADDRESS, 0x1c, 0x10);			//accelerometer full scale  +-8g	8192

	HAL_Delay(100);										//���� ����ȭ ������ Ÿ��
}


void HMC5883L_init()									//HMC5883L �ʱⷹ�����Ϳ� MPU6050 ������ ��� ����
{

	Write_reg(MPU_6050_ADDRESS, 0x6a, 0x00);			//mpu_6050 master mode off
	Write_reg(MPU_6050_ADDRESS, 0x37, 0x02);			//mpu_6050 bypass mode on

	Write_reg(HMC5883L_ADDRESS, 0x00, 0x78);			//configuration register : Samples averaged = 8, Output rate = 75Hz, Measurement Mode = Normal
	Write_reg(HMC5883L_ADDRESS, 0x01, 0x40);			//configuration register : Sensor Field Range = +-1.9Ga, Gain = 820
	Write_reg(HMC5883L_ADDRESS, 0x02, 0x00);			//Mode register : Operating Mode = Continuous-Measurement Mode

	Write_reg(MPU_6050_ADDRESS, 0x37, 0x00);			//mpu_6050 bypass mode off

	HAL_Delay(100);										//HMC5883 ���� ����ȭ ������ Ÿ��

	//magdata_X slave setting
	Write_reg(MPU_6050_ADDRESS, 0x25, 0x9e);			//slave read,write set and slave addr set
	Write_reg(MPU_6050_ADDRESS, 0x26, 0x03);			//slave register addr set
	Write_reg(MPU_6050_ADDRESS, 0x27, 0x82);			//slave en, bytsw, dis, group, len set

	//magdata_Z slave setting
	Write_reg(MPU_6050_ADDRESS, 0x28, 0x9e);			//slave read,write set and slave addr set
	Write_reg(MPU_6050_ADDRESS, 0x29, 0x05);			//slave register addr set
	Write_reg(MPU_6050_ADDRESS, 0x2a, 0x82);			//slave en, bytsw, dis, group, len set

	//magdata_Y slave setting
	Write_reg(MPU_6050_ADDRESS, 0x2b, 0x9e);			//slave read,write set and slave addr set
	Write_reg(MPU_6050_ADDRESS, 0x2c, 0x07);			//slave register addr set
	Write_reg(MPU_6050_ADDRESS, 0x2d, 0x82);			//slave en, bytsw, dis, group, len set

	Write_reg(MPU_6050_ADDRESS, 0x6a, 0x20);			//mpu_6050 master mode on

	HAL_Delay(100);										//MPU6050 ���� ����ȭ ������ Ÿ��
}


void MS5611_init()													//MS5611 ������ ���� ���� ������ �Լ�
{
	uint8_t command = 0x1e;
	uint8_t temp_R[6] = { 0xa2, 0xa4, 0xa6, 0xa8, 0xaa, 0xac };		//calibration data register
	uint8_t temp_D[2];
	uint16_t Data[6];
	unsigned short int calc_data;									//16bit unsigned int value pointer calc
	uint8_t *calcA = &calc_data, *calcB;
	calcB = calcA + 1;

	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &command, 1, 100);
	HAL_Delay(10);
	for (int i = 0; i < 6; i++) {
		HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &temp_R[i], 1, 100);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDRESS | 0x01), temp_D, 2, 200);
		calc_data = temp_D[1];
		*calcB = temp_D[0];
		Data[i] = calc_data;
	}
	//calibration data
	SENS_t1 = Data[0];
	OFF_t1 = Data[1];
	TCS = Data[2];
	TCO = Data[3];
	TREF = Data[4];
	TEMPSENS = Data[5];
	HAL_Delay(10);
}


void Read_MPU6050_Data(unsigned char senser_Add, unsigned char Register_Add)					//MPU6050 ���� ������ ���� �Լ�
{
	unsigned char Tmp_buffer[14] = { Register_Add };
	short int Out_data;
	short int temp_d[7];
	char *calcA = &Out_data;
	char *calcB;
	calcB = calcA + 1;

	HAL_I2C_Mem_Read(&hi2c1, senser_Add, Register_Add, I2C_MEMADD_SIZE_8BIT, &Tmp_buffer[0], 14, 100);		//�Լ� �ϳ��� �б� ������ �۵� �ð����� ������ �۾���
	//HAL_I2C_Master_Transmit(&hi2c1, senser_Add, Tmp_buffer, 1, 100);
	//HAL_Delay(10);
	//HAL_I2C_Master_Receive(&hi2c1, (senser_Add | 0x01), Tmp_buffer, 14, 200);								//burst read sequence

	for (int i = 0; i < 7; i++) {
		*calcB = Tmp_buffer[(2 * i)];
		*calcA = Tmp_buffer[(2 * i) + 1];
		temp_d[i] = Out_data;
	}

	//Accel X,Y,Z  and Gyro X,Y,Z data save
	Accel_X = temp_d[0];
	Accel_Y = temp_d[1];
	Accel_Z = temp_d[2];
	Temperature = temp_d[3] / 340 + 36.53;
	Gyro_X = temp_d[4];
	Gyro_Y = temp_d[5];
	Gyro_Z = temp_d[6];
}


void Read_HMC5883L_Data(unsigned char senser_Add, unsigned char Register_Add)
{
	unsigned char Tmp_buffer[6] = { Register_Add };
	short int Out_data;
	short int temp_d[3];
	char *calcA = &Out_data;
	char *calcB;
	calcB = calcA + 1;

	HAL_I2C_Mem_Read(&hi2c1, senser_Add, Register_Add, I2C_MEMADD_SIZE_8BIT, &Tmp_buffer[0], 6, 100);
	//HAL_I2C_Master_Transmit(&hi2c1, senser_Add, Tmp_buffer, 1, 100);
	//HAL_Delay(10);
	//HAL_I2C_Master_Receive(&hi2c1, (senser_Add | 0x01), Tmp_buffer, 6, 200);

	for (int i = 0; i < 3; i++) {
		*calcB = Tmp_buffer[(2 * i)];
		*calcA = Tmp_buffer[(2 * i) + 1];
		temp_d[i] = Out_data;
	}

	//Magnet X,Y,Z data save
	Magnet_X = temp_d[0];
	Magnet_Y = temp_d[2];
	Magnet_Z = temp_d[1];
	if(Magnet_X > mag_mx) mag_mx = Magnet_X;
	if(Magnet_Y > mag_my) mag_my = Magnet_Y;
	if(Magnet_X < mag_lx) mag_lx = Magnet_X;
	if(Magnet_Y < mag_ly) mag_ly = Magnet_Y;

}


void Read_MS5611_Data(int Type)
{

	uint8_t temp_A[4] = { 0, };
	uint32_t Data = 0;
	char *calcA, *calcB, *calcC = &Data;
	calcB = calcC + 1;
	calcA = calcC + 2;

	//MS5611 ������ ������ ������ ������ ADC��ȯ Ŀ�ǵ带 �Է��� �ִ� 10ms�� ������ Ÿ���� �ʿ��ϱ� ������ switch ���� �̿��ؼ� ADC ��ȯ�� 10������ �ѹ��� ����
	//�� 20 ������ ������Ŀ� ����� �����͸� ������ �����۾��� ����
	switch (Type) {

	case 1:			//��� ���� �����͸� �޴� switch
		//HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[0], 1, 100);
		//HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[2], 1, 100);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDRESS | 0x01), temp_A, 4, 200);
		*calcC = temp_A[2];
		*calcB = temp_A[1];
		*calcA = temp_A[0];
		D_PRESS = Data;
		break;

	case -1:		//�µ� ���� �����͸� �޴� switch
		//HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[1], 1, 100);
		//HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[2], 1, 100);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDRESS | 0x01), temp_A, 4, 200);
		*calcC = temp_A[2];
		*calcB = temp_A[1];
		*calcA = temp_A[0];
		D_TEMPER = Data;
		break;
	}
}


int32_t Total_Calibration()					//while �� �ۿ��� �ʱ� ���̽� ������ ������ ���� Ķ���극�̼� ����
{

	MPU_6050_init();
	HMC5883L_init();
	MS5611_init();

	for(int i = 0 ; i < 200 ; i++){
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[0], 1, 100);
	HAL_Delay(15);
	Read_MS5611_Data(1);
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[1], 1, 100);
	HAL_Delay(15);
	Read_MS5611_Data(-1);

	Calc_TP();

	base_Pressure += Pressure;

	}
	base_Pressure /= 200;

	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[0], 1, 100);
	HAL_Delay(15);
	Read_MS5611_Data(1);
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[1], 1, 100);
	HAL_Delay(15);
	Read_MS5611_Data(-1);

	Calc_TP();

	commandA = base_Pressure - 2000;

	for (int i = 0; i < 2000; i++)
	{
		Read_HMC5883L_Data(MPU_6050_ADDRESS, 0x49);
		Read_MPU6050_Data(MPU_6050_ADDRESS, 0x3b);

		Base_Accel_X += Accel_X;
		Base_Accel_Y += Accel_Y;
		Base_Accel_Z = Base_Accel_Z + (Accel_Z - 4096.0);

		Base_Gyro_X += Gyro_X;
		Base_Gyro_Y += Gyro_Y;
		Base_Gyro_Z += Gyro_Z;

		Base_Mage_X += Magnet_X;
		Base_Mage_Y += Magnet_Y;
		Base_Mage_Z += Magnet_Z;
	}



	Base_Accel_X /= 2000;
	Base_Accel_Y /= 2000;
	Base_Accel_Z /= 2000;

	Base_Gyro_X /= 2000;
	Base_Gyro_Y /= 2000;
	Base_Gyro_Z /= 2000;


	Base_Mage_X /= 2000;
	Base_Mage_Y /= 2000;
	Base_Mage_Z /= 2000;


	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[0], 1, 100);



	return commandA;

}

void Mag_calibration()
{
	Read_HMC5883L_Data(MPU_6050_ADDRESS, 0x49);

	Magnet_X -= Base_Mage_X;
	Magnet_Y -= Base_Mage_Y;

	if(Magnet_X > mag_mx) mag_mx = Magnet_X;
	if(Magnet_Y > mag_my) mag_my = Magnet_Y;
	if(Magnet_X < mag_lx) mag_lx = Magnet_X;
	if(Magnet_Y < mag_ly) mag_ly = Magnet_Y;

	HAL_Delay(30);

	offset_x = (mag_mx + mag_lx) /2;
	offset_y = (mag_my + mag_ly) /2;
}


void Read_9axisData(axis* Accel, axis* Gyro, axis* Mage)
{

	Read_MPU6050_Data(MPU_6050_ADDRESS, 0x3b);
	Read_HMC5883L_Data(MPU_6050_ADDRESS, 0x49);

	Accel_X -= Base_Accel_X;
	Accel_Y -= Base_Accel_Y;
	Accel_Z -= Base_Accel_Z;

	Gyro_X -= Base_Gyro_X;
	Gyro_Y -= Base_Gyro_Y;
	Gyro_Z -= Base_Gyro_Z;

	Magnet_X -= Base_Mage_X;
	Magnet_Y -= Base_Mage_Y;
	Magnet_Z -= Base_Mage_Z;

	Accel->X = (Accel_X) * IMU_G_PER_LSB_CFG;
	Accel->Y = (Accel_Y) * IMU_G_PER_LSB_CFG;
	Accel->Z = (Accel_Z) * IMU_G_PER_LSB_CFG;

	Gyro->X = (Gyro_X) * IMU_DEG_PER_LSB_CFG;
	Gyro->Y = (Gyro_Y) * IMU_DEG_PER_LSB_CFG;
	Gyro->Z = (Gyro_Z) * IMU_DEG_PER_LSB_CFG;


	/*
	Mage->X = Magnet_X / 820.0;
	Mage->Y = Magnet_Y / 820.0;
	Mage->Z = Magnet_Z / 820.0;
	*/
	Mage->X = (Magnet_X - offset_x) / 820.0;
	Mage->Y = (Magnet_Y - offset_y) / 820.0;
	Mage->Z = Magnet_Z / 820.0;

}


uint32_t Read_MSData(int cnt)
{
	if(cnt >= 100){
		Read_MS5611_Data(p_flag);
		if (p_flag == 1) HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[0], 1, 100);
		else
			{
				HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[1], 1, 100);
				Calc_TP();
			}

		p_flag = -p_flag;

	}

	//D_PRESS -= base_Pressure;
	/*
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[0], 1, 100);
	HAL_Delay(10);
	Read_MS5611_Data(1);
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[1], 1, 100);
	HAL_Delay(10);
	Read_MS5611_Data(-1);

	Calc_TP();
	*/
	return Pressure;
}



void DT_Reset()
{
	CNT_Register = 0x00;
}
