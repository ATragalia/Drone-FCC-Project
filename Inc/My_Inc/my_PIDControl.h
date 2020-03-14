/*
 * my_PIDContorl.h
 *
 *  Created on: 2019. 4. 13.
 *      Author: ragalia
 */

#ifndef MY_INTERFACE_MY_PIDCONTROL_H_
#define MY_INTERFACE_MY_PIDCONTROL_H_

#include <stdint.h>



int16_t saturateSignedInt16(float in);



//PID gain value Init
void PIDControl_Init();


int16_t PIDThrustControl(int ThrustIn, float ThrustDesired);


//Attitude Control use PID
void PIDAttitudeControl(
		float RollIn, float PitchIn, float YawIn,
		float RollDesired, float PitchDesired, float YawDesired,
		float* RollRateDesired, float* PitchRateDesired, float* YawRateDesired);



//Rate Control use PID
void PIDRateControl(
		float RollRateIn, float PitchRateIn, float YawRateIn,
		float RollRateDesired, float PitchRateDesired, float YawRateDesired);



//pid DT set
void PIDdtSet(float dt);



//pid pgain set
void rategainset(int value, int target, int flag);



//All PID value Reset
void PIDContorlAllReset();


//Total RPY value Output
void PIDControlOut(int16_t* Roll, int16_t* Pitch, int16_t* Yaw);


#endif /* MY_INTERFACE_MY_PIDCONTROL_H_ */
