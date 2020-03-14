/*
 * my_Control_lib.c
 *
 *  Created on: 2019. 4. 15.
 *      Author: ragalia
 */


#include "My_Inc/my_Control_lib.h"
#include "My_Inc/my_GY_86_lib.h"
#include "My_Inc/my_PIDControl.h"
#include "My_Inc/my_Sensor_calc.h"
#include "My_Inc/my_receiver_lib.h"
#include "My_Inc/my_MotorControl_lib.h"
#include "My_Inc/LPF.h"
#include "My_Inc/Optical_flow.h"
#include "My_Inc/GPS.h"

axis Accel;
axis Gyro;
axis Mage;

uint16_t actuatorThrust;
int16_t actuatorRoll;
int16_t actuatorPitch;
int16_t actuatorYaw;
uint32_t altitude;
uint32_t commandaltitude;

float Roll, Pitch, Yaw;
float RollRate, PitchRate, YawRate;
float Command_Roll= 0, Command_Pitch = 0, Command_Yaw = 0;
float dt = 0.001, dt_E;
int cnt_t;
int Pilter_flag = 0, S_flag = 0;
int sa, sb, S1, sc;
int Thrust;


void Total_Init()
{





	//Motor PWM Init
	Motor_init();

	//PID gain value default Set
	PIDControl_Init();

	//IMU Sensor calibration
	commandaltitude = Total_Calibration();

	//RxInterupt Set
	usart_DMA_init();

	Busprotocol();

	sc = SignalGet();
	while(1)
	{
		Mag_calibration();
		Busprotocol();
		sc = SignalGet();
		//CommandGet(&actuatorThrust, &Command_Roll, &Command_Pitch, &sa, &sb, &S1, &sc, &S_flag);
		if(sc != 0)break;
	}
	//setup();
	//gps_setup();

	LPF_Init();
}

void stabilizer_Loop()
{
	//pressure data Read
	altitude = Read_MSData(cnt_t);
	if(cnt_t >= 100 ) cnt_t = 0;


	//Read 9axis data
	Read_9axisData(&Accel, &Gyro, &Mage);


	//first LPF Filter
	//Calc_LPF(&Accel, &Gyro, dt);


	Busprotocol();


	//Receiver get Command
	CommandGet(&actuatorThrust, &Command_Roll, &Command_Pitch, &sa, &sb, &S1, &sc, &S_flag);


	cnt_t += CNT_Register;
	dt = CNT_Register / 10000.0;

	//dt_Reset
	DT_Reset();


	//convert raw to Quaternion use 6axis raw data
	//sensorcalcQ_madwic_addpilter(Gyro.X, Gyro.Y, Gyro.Z, Accel.X, Accel.Y, Accel.Z, 0.004, Pilter_flag);
	sensorcalcQ_mahony(Gyro.X, Gyro.Y, Gyro.Z, Accel.X, Accel.Y, Accel.Z, dt);
	//sensorcalcQ_madwic_addpilter(Gyro.X, Gyro.Y, Gyro.Z, Accel.X, Accel.Y, Accel.Z, 0.002, Pilter_flag);
	conmp_Filter_Calc(Gyro.X, Gyro.Y, Gyro.Z, Accel.X, Accel.Y, Accel.Z, Mage.X, Mage.Y, Mage.Z, dt);
	//sensorcalcQ_mahony_9(Gyro.X, Gyro.Y, Gyro.Z, Accel.X, Accel.Y, Accel.Z, Mage.X, Mage.Y, Mage.Z, dt);


	//PID variable Dt set
	PIDdtSet(dt);


	//convert Quaternion to euler without mage data
	calc_Q_to_E(&Roll, &Pitch, &Yaw);

	Yaw_data_get(Mage.X, Mage.Y, Mage.Z, Roll, Pitch);


	//ratePgain set
	//rategainset(S1, sb, sc);


	Thrust = PIDThrustControl((int)altitude, commandaltitude);

	//Attitude PIDControl start

	PIDAttitudeControl(Roll, Pitch, Yaw , Command_Roll, -Command_Pitch, Command_Yaw, &RollRate, &PitchRate, &YawRate);


		//Rate PIDControl start
	PIDRateControl(Gyro.X, -Gyro.Y, Gyro.Z, RollRate, PitchRate, YawRate);



		//total Axis value out
	PIDControlOut(&actuatorRoll, &actuatorPitch, &actuatorYaw);



	//Motor Output Control
	Motor_Control(actuatorThrust, actuatorRoll, actuatorPitch, actuatorYaw, Thrust, sa, sc);


	//dt_E = CNT_Register - dt * 10000.0;










}


void calibration_Loop()
{
	CommandGet(&actuatorThrust, &Command_Roll, &Command_Pitch, &sa, &sb, &S1, &sc, &S_flag);
	Busprotocol();
	MotorESCcalibration(actuatorThrust);
}

