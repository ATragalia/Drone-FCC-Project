/*
 * my_Sensor_calc.c
 *
 *  Created on: 2019. 4. 15.
 *      Author: ragalia
 */

#include <stdbool.h>
#include <math.h>
#include "My_Inc/my_Sensor_calc.h"


//madwic_value
float Beta = BETA_DEF;
float GyroMeasError = 3.14159265358979323846f * (60.0f / 180.0f);
float GyroMeasDrift = 3.14159265358979323846f * (1.0f / 180.0f);
float zeta = sqrt(3.0f / 4.0f) * 1.0471975511965977461533333333333;
float beta = sqrt(3.0f / 4.0f) * 0.01745329251994329576922222222222;
float Yaw_l, Yaw_t;
double Xh, Yh;

//mahony_value
float twoKp = TWO_KP_DEF;
float twoKi = TWO_KI_DEF;    // 2 * integral gain (Ki)
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;  // integral error terms scaled by Ki

//Quaternion contaner
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;



void sensorcalcQ_madwic(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= Beta * s0;
    qDot2 -= Beta * s1;
    qDot3 -= Beta * s2;
    qDot4 -= Beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}


void sensorcalcQ_mahony(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx = gx * M_PI_F / 180.0f;
  gy = gy * M_PI_F / 180.0f;
  gz = gz * M_PI_F / 180.0f;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    // Normalise accelerometer measurement
    recipNorm = sqrt(ax * ax + ay * ay + az * az);
    ax /= recipNorm;
    ay /= recipNorm;
    az /= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);   // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= recipNorm;
  q1 /= recipNorm;
  q2 /= recipNorm;
  q3 /= recipNorm;
}

void Yaw_data_get(float mx, float my, float mz, float Roll, float Pitch)
{
	Xh = mx * cosf(Pitch) + my * sinf(Roll)*sinf(Pitch) - mz * cosf(Roll)*sinf(Pitch);
	Yh = my * cosf(Roll) + mz * sinf(Pitch);
	Yaw_l = atan(mx/ my)* 180.0 / M_PI;


	if(mx >= 0 && my > 0)Yaw_l = Yaw_l;
	else if(mx < 0 && my >= 0) Yaw_l = 180 + Yaw_l;
	else if(mx <= 0 && my < 0) Yaw_l = 180 + Yaw_l;
	else if(mx >= 0 && my <= 0) Yaw_l = 360 + Yaw_l;



	if(Xh < 0)
		Yaw_t = 180 - atan2(Yh, Xh)* 57.3;
	else if(Xh > 0 && Yh < 0)
		Yaw_t = -atan2(Yh,Xh )* 57.3;
	else if(Xh > 0 && Yh > 0)
		Yaw_t = 360 - atan2(Yh,Xh)* 57.3;
	else if(Xh == 0 && Yh < 0)
		Yaw_t = 90;
	else if(Xh == 0 && Yh > 0)
		Yaw_t = 270;

}

void sensorcalcQ_madwic_addpilter(float gx, float gy, float gz, float ax, float ay, float az, float dt, int flag)
{
	float recipNorm;
	float f1, f2, f3;
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;  // gyro bias error


    float _halfq1 = 0.5f * q0;
    float _halfq2 = 0.5f * q1;
    float _halfq3 = 0.5f * q2;
    float _halfq4 = 0.5f * q3;
    float _2q1 = 2.0f * q0;
    float _2q2 = 2.0f * q1;
    float _2q3 = 2.0f * q2;
    float _2q4 = 2.0f * q3;

    if(flag == 1){
		 beta = 0.04;  // decrease filter gain after stabilized
		 zeta = 0.015; // increasey bias drift gain after stabilized
    }


    gx = gx * M_PI_F / 180;
    gy = gy * M_PI_F / 180;
    gz = gz * M_PI_F / 180;



    recipNorm = sqrt(ax * ax + ay * ay + az * az);
    if (recipNorm == 0.0f) return; // handle NaN
    recipNorm = 1.0f/recipNorm;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    f1 = _2q2 * q3 - _2q1 * q2 - ax;
    f2 = _2q1 * q1 + _2q3 * q3 - ay;
    f3 = 1.0f - _2q2 * q1 - _2q3 * q2 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    recipNorm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= recipNorm;
    hatDot2 /= recipNorm;
    hatDot3 /= recipNorm;
    hatDot4 /= recipNorm;

    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

    gbiasx += gerrx * dt * zeta;
    gbiasy += gerry * dt * zeta;
    gbiasz += gerrz * dt * zeta;

    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    q0 += (qDot1 -(beta * hatDot1)) * dt;
    q1 += (qDot2 -(beta * hatDot2)) * dt;
    q2 += (qDot3 -(beta * hatDot3)) * dt;
    q3 += (qDot4 -(beta * hatDot4)) * dt;

    recipNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);    // normalise quaternion
    recipNorm = 1.0f/recipNorm;
    q0 = q0 * recipNorm;
    q1 = q1 * recipNorm;
    q2 = q2 * recipNorm;
    q3 = q3 * recipNorm;

}


void sensorcalcQ_mahony_9(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		sensorcalcQ_mahony(gx, gy, gz, ax, ay, az, dt);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / dt);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / dt);
			integralFBz += twoKi * halfez * (1.0f / dt);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / dt));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / dt));
	gz *= (0.5f * (1.0f / dt));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}



void calc_Q_to_E(float* roll, float* pitch, float* yaw)
{
  float gx, gy, gz; // estimated gravity direction

  gx = 2 * (q1*q3 - q0*q2);
  gy = 2 * (q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  if (gx>1) gx=1;
  if (gx<-1) gx=-1;

  *yaw = (atan2f(2 *(q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3) * (180.0f / M_PI_F));
  *pitch = (asinf(gx) * (180.0f / M_PI_F)); //Pitch seems to be inverted
  *roll = (atan2f(gy, gz) * (180.0f / M_PI_F));
}


float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az)
{
  float gx, gy, gz; // estimated gravity direction

  gx = 2 * (q1*q3 - q0*q2);
  gy = 2 * (q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // return vertical acceleration without gravity
  // (A dot G) / |G| - 1G (|G| = 1) -> (A dot G) - 1G
  return ((ax*gx + ay*gy + az*gz) - 1.0);
}


float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
