#include "PID.h"

PID::PID() {
	mKp = 0;
	mKi = 0;
	mKd = 0;
	mIntegratedErr = 0;
	mLastErr = 0;
}

PID::PID(float kP, float kI, float kD) {
	mKp = kP;
	mKi = kI;
	mKd = kD;
	mIntegratedErr = 0;
	mLastErr = 0;
}


PID::~PID() {
}

float PID::compute(float wantedVal, float realVal, float dt)
{
	float err = wantedVal - realVal;
	mIntegratedErr += err * dt;
	float pTerm = mKp * err;
	float iTerm = mKi * mIntegratedErr;
	float dTerm = mKd * ((err - mLastErr)/dt);
	mLastErr = err;
	return pTerm + iTerm - dTerm;
}
