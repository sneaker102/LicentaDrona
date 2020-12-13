#include "ControllerDrone.h"

ControllerDrone::ControllerDrone(PID xPID, PID yPID, PID zPID, PID rollPID, PID pitchPID, PID yawPID, float droneMass, float motorsMaxRPM, float motorsRadius, float motorToCenterL) {
	mXPID = xPID;
	mYPID = yPID;
	mZPID = zPID;
	mRollPID = rollPID;
	mPitchPID = pitchPID;
	mYawPID = yawPID;
	g = 9.80665f;
	m = droneMass;
	maxRPM = motorsMaxRPM;
	l = motorToCenterL;
	float thrustCoef = 0.137f;
	float dragMomentCoef = 0.0092f;
	float airDensity = 1.135f; //kg/m^3
	// motorsRadius 0.079615f m
	thrustConst = (thrustCoef * airDensity * pow(motorsRadius, 4)) / (pow(2 * M_PI, 2));
	hubTorqueConst = (dragMomentCoef * airDensity * pow(motorsRadius, 5)) / (pow(2 * M_PI, 2));

}
float  ControllerDrone::mapValueInRange(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
float ControllerDrone::getMotorAngularVel(float motorThrottle) {
	// map throttle to RPM
	// and RPM to rad/s
	return (2 * M_PI) / 60 * (maxRPM * motorThrottle);
}
float ControllerDrone::getMotorThrust(float motorAngularVel) {
	// map angular vel to thrust
	float mThrust = thrustConst * pow(motorAngularVel, 2);
	return mThrust;
}
float ControllerDrone::getMotorsTorq(float motorAngularVel) {
	float mTorq = hubTorqueConst * pow(motorAngularVel, 2);
	return mTorq;
}
float* ControllerDrone::getDroneForceAndTorqs(float motorsThrust[4], float motorsTorq[4]) {
	float motorsThrustSum = m*g;
	float motorsTorqSum = 0;
	for (int i = 0; i < 4; i++) {
		motorsThrustSum += motorsThrust[i];
		motorsTorqSum += motorsTorq[i];
	}
	float torqX = (motorsThrust[1] - motorsThrust[3]) * l;
	float torqY = (motorsThrust[2] - motorsThrust[0]) * l;
	float forceAndTorqs[4] = {
		motorsThrustSum,
		torqX,
		torqY,
		motorsTorqSum
	};
	return forceAndTorqs;
}
float*  ControllerDrone::controlMotors(float wantedPosture[6], float realPosture[6], float dt) {
	// Control
	// Position Control
	// map x,y to desired pitch and roll
	float wantedPitch = mXPID.compute(wantedPosture[0],realPosture[0], dt);
	float wantedRoll = mYPID.compute(wantedPosture[1], realPosture[1], dt);
	// AttitudeControl
	float corr[4] = { mRollPID.compute(wantedPosture[3], realPosture[3], dt), mPitchPID.compute(wantedPosture[4],realPosture[4],dt), mYawPID.compute(wantedPosture[5], realPosture[5],dt), mZPID.compute(wantedPosture[2], realPosture[2] + g, dt)};
	float motorsThrottle[4] = { mapValueInRange(corr[3] - corr[2] - corr[1],-1023, 1023, 0, 1), mapValueInRange(corr[3] - corr[2] + corr[1],-1023, 1023, 0, 1), mapValueInRange(corr[3] + corr[2] + corr[0],-1023, 1023, 0, 1), mapValueInRange(corr[3] + corr[2] - corr[0],-1023, 1023, 0, 1) };
	cout << "Throttle m1: "<< motorsThrottle[0] << endl;
	cout << "Throttle m2: " << motorsThrottle[1] << endl;
	cout << "Throttle m3: " << motorsThrottle[2] << endl;
	cout << "Throttle m4: " << motorsThrottle[3] << endl;
	//Dynamics
	float m1AngularVel = getMotorAngularVel(motorsThrottle[0]);
	float m2AngularVel = getMotorAngularVel(motorsThrottle[1]);
	float m3AngularVel = getMotorAngularVel(motorsThrottle[2]);
	float m4AngularVel = getMotorAngularVel(motorsThrottle[3]);
	cout << "AngularVel m1: " << m1AngularVel << endl;
	cout << "AngularVel m2: " << m2AngularVel << endl;
	cout << "AngularVel m3: " << m3AngularVel << endl;
	cout << "AngularVel m4: " << m4AngularVel << endl;
	float motorsAngularVel[4] = { 
		m1AngularVel,
		m2AngularVel,
		m3AngularVel,
		m4AngularVel };
	// motors Fz
	float m1Thrust = getMotorThrust(motorsAngularVel[0]);
	float m2Thrust = getMotorThrust(motorsAngularVel[1]);
	float m3Thrust = getMotorThrust(motorsAngularVel[2]);
	float m4Thrust = getMotorThrust(motorsAngularVel[3]);
	cout << "Thrust m1: " << m1Thrust << endl;
	cout << "Thrust m2: " << m2Thrust << endl;
	cout << "Thrust m3: " << m3Thrust << endl;
	cout << "Thrust m4: " << m4Thrust << endl;
	float motorsThrust[4] = {
		m1Thrust,
		m2Thrust,
		m3Thrust,
		m4Thrust };
	float m1Torq = getMotorsTorq(motorsAngularVel[0]);
	float m2Torq = getMotorsTorq(motorsAngularVel[1]);
	float m3Torq = getMotorsTorq(motorsAngularVel[2]);
	float m4Torq = getMotorsTorq(motorsAngularVel[3]);
	cout << "m1Torq: " << m1Torq << endl;
	cout << "m2Torq: " << m2Torq << endl;
	cout << "m3Torq: " << m3Torq << endl;
	cout << "m4Torq: " << m4Torq << endl;
	float motorsTorq[4] = {
		m1Torq,m2Torq,m3Torq,m4Torq
	};
	float* droneForceAndMoments = getDroneForceAndTorqs(motorsThrust, motorsTorq);
	return droneForceAndMoments;


	
}