#pragma once
#include "../../build/vs2015/PID.h"
#include <math.h> 
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#define M_PI 3.14159265358979323846

using namespace std;

class ControllerDrone
{
private:
	PID mXPID, mYPID, mZPID, mRollPID, mPitchPID, mYawPID;
	float m, g, maxRPM, l, thrustConst, hubTorqueConst;
public :
	ControllerDrone(PID xPID, PID yPID, PID zPID, PID rollPID, PID pitchPID, PID yawPID, float droneMass, float motorsMaxRPM, float motorsRadius, float motorToCenterL);
	float mapValueInRange(float x, float in_min, float in_max, float out_min, float out_max);
	float getMotorAngularVel(float motorThrottle);
	float getMotorThrust(float motorAngularVel);
	float getMotorsTorq(float motorAngularVel);
	float* getDroneForceAndTorqs(float motorsThrust[4], float motorsTorq[4]);
	float* controlMotors(float wantedPosture[6], float realPosture[6], float dt);

};

