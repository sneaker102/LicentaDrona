#pragma once
class PID
{
private:
	float mKp, mKi, mKd, mIntegratedErr, mLastErr;
public:
	PID(float kP, float kI, float kD);
	~PID();
	float compute(float wantedVal, float realVal, float dt);
};
