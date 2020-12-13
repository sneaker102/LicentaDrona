#include "ardrone/ardrone.h"
#include <math.h> 
#include <vector>
#include "../../cvdrone-master/build/vs2015/PID.h"
#include <random>
#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "../../cvdrone-master/build/vs2015/ControllerDrone.h"

extern "C" {
#include "extApi.h"
}

// Fallback, in case M_PI doesn't exist.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using namespace cv;


float degToRad(float deg) {
	return deg * (M_PI / 180.f);
}

float radToDeg(float rad) {
	return rad * (180.f / M_PI);
}

float rpmToRadS(float rpm) {
	return (2 * M_PI) / 60 * rpm;
}

float radSToRpm(float radS) {
	return 60 / (2 * M_PI) * radS;
}

Mat calcQuinticCoeffs( float t_s, float t_f, float q_s, float q_f, float dq_s = 0.f, float dq_f = 0.f, float ddq_s = 0.f, float ddq_f = 0.f) {
	float timeData[6][6] = {
		{1.f, t_s, pow(t_s, 2), pow(t_s, 3), pow(t_s, 4), pow(t_s, 5)},
		{0.f, 1.f, 2*t_s, 3*pow(t_s, 2), 4*pow(t_s, 3), 5*pow(t_s, 4)},
		{0.f, 0.f, 2.f, 6*t_s, 12*pow(t_s, 2), 20*pow(t_s, 3)},
		{1.f, t_f, pow(t_f, 2), pow(t_f, 3), pow(t_f, 4), pow(t_f, 5)},
		{0.f, 1.f, 2*t_f, 3*pow(t_f, 2), 4*pow(t_f, 3), 5*pow(t_f, 4)},
		{0.f, 0.f, 2.f, 6*t_f, 12*pow(t_f, 2), 20*pow(t_f, 3)}
	};
	Mat quinticTimeMatrix(6, 6, CV_32F, timeData);
	Vec6f constraints(q_s, dq_s, ddq_s, q_f, dq_f, ddq_f);
	return quinticTimeMatrix.inv() * Mat(constraints);
}
Mat getWantedPostureVelAcc(Mat* quinticCoeffs, float t) {
	// position, velocity and acceleration for every axis linear and angular
	float xyzVelAcc[6][3] = {
		{
		(quinticCoeffs[0].at<float>(0) + quinticCoeffs[0].at<float>(1) * t + quinticCoeffs[0].at<float>(2) * pow(t, 2) + quinticCoeffs[0].at<float>(3) * pow(t, 3) + quinticCoeffs[0].at<float>(4) * pow(t, 4) + quinticCoeffs[0].at<float>(5) * pow(t, 5)),
		(quinticCoeffs[0].at<float>(1) + 2 * quinticCoeffs[0].at<float>(2) * t + 3 * quinticCoeffs[0].at<float>(3) * pow(t, 2) + 4 * quinticCoeffs[0].at<float>(4) * pow(t, 3) + 5 * quinticCoeffs[0].at<float>(5) * pow(t, 4)),
		(2 * quinticCoeffs[0].at<float>(2) + 6 * quinticCoeffs[0].at<float>(3) * t + 12 * quinticCoeffs[0].at<float>(4) * pow(t, 2) + 20 * quinticCoeffs[0].at<float>(5) * pow(t, 3))
	},
		{
		(quinticCoeffs[1].at<float>(0) + quinticCoeffs[1].at<float>(1) * t + quinticCoeffs[1].at<float>(2) * pow(t, 2) + quinticCoeffs[1].at<float>(3) * pow(t, 3) + quinticCoeffs[1].at<float>(4) * pow(t, 4) + quinticCoeffs[1].at<float>(5) * pow(t, 5)),
		(quinticCoeffs[1].at<float>(1) + 2 * quinticCoeffs[1].at<float>(2) * t + 3 * quinticCoeffs[1].at<float>(3) * pow(t, 2) + 4 * quinticCoeffs[1].at<float>(4) * pow(t, 3) + 5 * quinticCoeffs[1].at<float>(5) * pow(t, 4)),
		(2 * quinticCoeffs[1].at<float>(2) + 6 * quinticCoeffs[1].at<float>(3) * t + 12 * quinticCoeffs[1].at<float>(4) * pow(t, 2) + 20 * quinticCoeffs[1].at<float>(5) * pow(t, 3))
	},
		{
		(quinticCoeffs[2].at<float>(0) + quinticCoeffs[2].at<float>(1) * t + quinticCoeffs[2].at<float>(2) * pow(t, 2) + quinticCoeffs[2].at<float>(3) * pow(t, 3) + quinticCoeffs[2].at<float>(4) * pow(t, 4) + quinticCoeffs[2].at<float>(5) * pow(t, 5)),
		(quinticCoeffs[2].at<float>(1) + 2 * quinticCoeffs[2].at<float>(2) * t + 3 * quinticCoeffs[2].at<float>(3) * pow(t, 2) + 4 * quinticCoeffs[2].at<float>(4) * pow(t, 3) + 5 * quinticCoeffs[2].at<float>(5) * pow(t, 4)),
		(2 * quinticCoeffs[2].at<float>(2) + 6 * quinticCoeffs[2].at<float>(3) * t + 12 * quinticCoeffs[2].at<float>(4) * pow(t, 2) + 20 * quinticCoeffs[2].at<float>(5) * pow(t, 3))
	},
		{
		(quinticCoeffs[3].at<float>(0) + quinticCoeffs[3].at<float>(1) * t + quinticCoeffs[3].at<float>(2) * pow(t, 2) + quinticCoeffs[3].at<float>(3) * pow(t, 3) + quinticCoeffs[3].at<float>(4) * pow(t, 4) + quinticCoeffs[3].at<float>(5) * pow(t, 5)),
		(quinticCoeffs[3].at<float>(1) + 2 * quinticCoeffs[3].at<float>(2) * t + 3 * quinticCoeffs[3].at<float>(3) * pow(t, 2) + 4 * quinticCoeffs[3].at<float>(4) * pow(t, 3) + 5 * quinticCoeffs[3].at<float>(5) * pow(t, 4)),
		(2 * quinticCoeffs[3].at<float>(2) + 6 * quinticCoeffs[3].at<float>(3) * t + 12 * quinticCoeffs[3].at<float>(4) * pow(t, 2) + 20 * quinticCoeffs[3].at<float>(5) * pow(t, 3))
	},
		{
		(quinticCoeffs[4].at<float>(0) + quinticCoeffs[4].at<float>(1) * t + quinticCoeffs[4].at<float>(2) * pow(t, 2) + quinticCoeffs[4].at<float>(3) * pow(t, 3) + quinticCoeffs[4].at<float>(4) * pow(t, 4) + quinticCoeffs[4].at<float>(5) * pow(t, 5)),
		(quinticCoeffs[4].at<float>(1) + 2 * quinticCoeffs[4].at<float>(2) * t + 3 * quinticCoeffs[4].at<float>(3) * pow(t, 2) + 4 * quinticCoeffs[4].at<float>(4) * pow(t, 3) + 5 * quinticCoeffs[4].at<float>(5) * pow(t, 4)),
		(2 * quinticCoeffs[4].at<float>(2) + 6 * quinticCoeffs[4].at<float>(3) * t + 12 * quinticCoeffs[4].at<float>(4) * pow(t, 2) + 20 * quinticCoeffs[4].at<float>(5) * pow(t, 3))
	},
		{
	(quinticCoeffs[5].at<float>(0) + quinticCoeffs[5].at<float>(1) * t + quinticCoeffs[5].at<float>(2) * pow(t, 2) + quinticCoeffs[5].at<float>(3) * pow(t, 3) + quinticCoeffs[5].at<float>(4) * pow(t, 4) + quinticCoeffs[5].at<float>(5) * pow(t, 5)),
	(quinticCoeffs[5].at<float>(1) + 2 * quinticCoeffs[5].at<float>(2) * t + 3 * quinticCoeffs[5].at<float>(3) * pow(t, 2) + 4 * quinticCoeffs[5].at<float>(4) * pow(t, 3) + 5 * quinticCoeffs[5].at<float>(5) * pow(t, 4)),
	(2 * quinticCoeffs[5].at<float>(2) + 6 * quinticCoeffs[5].at<float>(3) * t + 12 * quinticCoeffs[5].at<float>(4) * pow(t, 2) + 20 * quinticCoeffs[5].at<float>(5) * pow(t, 3))
	}
	};
	return Mat(6, 3, CV_32F, xyzVelAcc);
}
void setTrajectory(vector<float> (&traj)[3], Mat quinticCoeffs, float t) {
	traj[0].push_back((quinticCoeffs.at<float>(0) + quinticCoeffs.at<float>(1)*t + quinticCoeffs.at<float>(2)*pow(t, 2) + quinticCoeffs.at<float>(3)*pow(t, 3) + quinticCoeffs.at<float>(4)*pow(t, 4) + quinticCoeffs.at<float>(5)*pow(t, 5)));
	traj[1].push_back((quinticCoeffs.at<float>(1) + 2 * quinticCoeffs.at<float>(2)*t + 3 * quinticCoeffs.at<float>(3)*pow(t, 2) + 4 * quinticCoeffs.at<float>(4)*pow(t, 3) + 5 * quinticCoeffs.at<float>(5)*pow(t, 4)));
	traj[2].push_back((2 * quinticCoeffs.at<float>(2) + 6 * quinticCoeffs.at<float>(3)*t + 12 * quinticCoeffs.at<float>(4)*pow(t, 2) + 20 * quinticCoeffs.at<float>(5)*pow(t, 3)));
}

void plot(Mat plotImg, String windowName, float x, float y, float z = 0) {

	char text[100];
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = 1;
	int thickness = 1;
	cv::Point textOrg(10, 50);
	circle(plotImg, Point(x + 300, y + 500), 1, CV_RGB(255, 0, 0), 2);
	rectangle(plotImg, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
	sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", x, -y, z);
	putText(plotImg, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
	imshow(windowName, plotImg);
	waitKey(100);
}

Mat constructRotMatrix(float roll, float pitch, float yaw, bool inv = false) {
	float rotData[3][3] = {
	{cos(yaw) * cos(pitch), cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll), cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll)},
	{sin(yaw) * cos(pitch), sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll), sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll)},
	{-sin(pitch), -cos(pitch) * sin(roll), cos(pitch) * cos(roll)}
	};
	Mat rotMatrix(3, 3, CV_32F, rotData);
	if (inv) {
		return rotMatrix.inv();
	}
	return rotMatrix;
}

Mat getBodyLinearVel(Vec3f inertialLinearVel, float roll, float pitch, float yaw) {
	Mat rotMatrixInv = constructRotMatrix(roll, pitch, yaw, true);
	cout << "bodyLinearVel" << rotMatrixInv * Mat(inertialLinearVel) << endl;
	return rotMatrixInv * Mat(inertialLinearVel);
}

Mat getInertialLinearVel(Vec3f bodyLinearVel, float roll, float pitch, float yaw) {
	Mat rotMatrix = constructRotMatrix(roll, pitch, yaw);
	return rotMatrix * Mat(bodyLinearVel);
}

Mat getBodyAngularVel(Vec3f inertialAngularVel, float roll, float pitch) {
	float transformData[3][3] = {
		{1.f, 0.f, -sin(pitch)},
		{0.f, cos(roll), cos(pitch) * sin(roll)},
		{0.f, -sin(roll), cos(pitch) * cos(roll)}
	};
	Mat transformMatrix(3, 3, CV_32F, transformData);
	cout << "bodyAngularVel" << transformMatrix * Mat(inertialAngularVel) << endl;
	return transformMatrix * Mat(inertialAngularVel);
}

Mat getInertialAngularVel(Vec3f bodyAngularVel, float roll, float pitch) {
	float transformData[3][3] = {
		{1.f, sin(roll)*tan(pitch), cos(roll)*tan(pitch)},
		{0.f, cos(roll), -sin(roll)},
		{0.f, sin(roll)/cos(pitch), cos(roll)/cos(pitch)}
	};
	Mat transformMatrix(3, 3, CV_32F, transformData);
	cout << "inertialAngularVel" << transformMatrix * Mat(bodyAngularVel) << endl;
	return transformMatrix * Mat(bodyAngularVel);
}


// not sure if needed
Mat getAppliedForces(Mat bodyAngularVel, Mat bodyLinearVel) {
	float m = 0.12f; //kg
	
	Mat forces = m * (bodyAngularVel.cross(bodyLinearVel) + bodyLinearVel);
	return forces;
}

Mat getAppliedTorques(Mat bodyAngularVel) {
	// m^2/kg
	float inertiaData[3][3] = {
		{0.00006667f, 0.f, 0.f},
		{0.f, 0.007533f, 0.f},
		{0.f, 0.f, 0.007533f}
	};
	Mat inertiaMatrix(3, 3, CV_32F, inertiaData);
	Mat inerAngularProd = inertiaMatrix * bodyAngularVel;
	return (inerAngularProd + bodyAngularVel.cross(inerAngularProd));
}


int main(int argc, char *argv[])
{

	simxFinish(-1);                                                     //! Close any previously unfinished business
	int clientID = simxStart((simxChar*)"127.0.0.1", 19997, true, true, 5000, 5);  //!< Main connection to V-REP
	Sleep(10);
	if (clientID != -1)
	{
		cout << " Connection status to VREP: SUCCESS" << endl;

		int tableTop,drone, droneBase, m1, m2, m3, m4;
		simxGetObjectHandle(clientID, "table_top", &tableTop, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "Quadricopter", &drone, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "Quadricopter_base", &droneBase, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "Quadricopter_propeller_respondable1", &m1, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "Quadricopter_propeller_respondable2", &m2, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "Quadricopter_propeller_respondable3", &m3, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "Quadricopter_propeller_respondable4", &m4, simx_opmode_oneshot_wait);

		float tablePos[3], dronePos[3], droneOrientation[3], droneLinearVel[3], droneAngularVel[3];
		simxGetObjectPosition(clientID, tableTop, -1, tablePos, simx_opmode_oneshot_wait);
		simxGetObjectPosition(clientID, droneBase, -1, dronePos, simx_opmode_oneshot_wait);
		simxGetObjectOrientation(clientID, droneBase, -1, droneOrientation, simx_opmode_oneshot_wait);

		// x,y,z,roll,pitch,yaw
		float waypoints[2][6] = {
			{dronePos[0],dronePos[1],dronePos[2], droneOrientation[0],droneOrientation[1], droneOrientation[2]},
			{tablePos[0], tablePos[1], tablePos[2]+1, 0.f, 0.f, degToRad(90.f)}
		};
		float t_f = 5.f;

		Mat quinticCoeff[6] = { 
			calcQuinticCoeffs(0.f, t_f, waypoints[0][0], waypoints[1][0]),
			calcQuinticCoeffs(0.f, t_f, waypoints[0][1], waypoints[1][1]),
			calcQuinticCoeffs(0.f, t_f, waypoints[0][2], waypoints[1][2]),
			calcQuinticCoeffs(0.f, t_f, waypoints[0][3], waypoints[1][3]),
			calcQuinticCoeffs(0.f, t_f, waypoints[0][4], waypoints[1][5]),
			calcQuinticCoeffs(0.f, t_f, waypoints[0][5], waypoints[1][5]) };

		//namedWindow("Trajectory", WINDOW_AUTOSIZE);
		//Mat plotImg = Mat::zeros(800, 800, CV_8UC3);
		//namedWindow("Velocities", WINDOW_AUTOSIZE);
		//Mat plotVel = Mat::zeros(800, 800, CV_8UC3);
		//namedWindow("VelocitiesPID", WINDOW_AUTOSIZE);
		//Mat plotVelPID = Mat::zeros(800, 800, CV_8UC3);
		//PID PIDs[4] = { PID(1.f,0.f,1.f),PID(1.f,0.f,1.f),PID(1.f,0.f,1.f),PID(1.f,0.f,1.f) };
		//default_random_engine generator;
		//uniform_real_distribution<double> distribution(0.1, 1.3);
		Mat linearVel, angularVel, bodyForces, bodyTorques, motorsAngularVel;
		float dt = 0.05f;
		ControllerDrone droneController(PID(1.f, 0.f, 1.f), PID(1.f, 0.f, 1.f), PID(1.f, 0.f, 1.f), PID(1.f, 0.f, 1.f), PID(1.f, 0.f, 1.f), PID(1.f, 0.f, 1.f), 0.12f, 50000.f, 0.079615f, 0.0183f);
		for (float t = 0.f; t <= t_f; t += dt) {
			cout << "Timp: " << t << endl;
			Mat wantedPostureVelAcc = getWantedPostureVelAcc(quinticCoeff, t);
			float wantedPosture[6] = {
				wantedPostureVelAcc.at<float>(0,0),
				wantedPostureVelAcc.at<float>(1,0),
				wantedPostureVelAcc.at<float>(2,0),
				wantedPostureVelAcc.at<float>(3,0),
				wantedPostureVelAcc.at<float>(4,0),
				wantedPostureVelAcc.at<float>(5,0) };
			float realPosture[6] = {
				dronePos[0],dronePos[1],dronePos[2], droneOrientation[0],droneOrientation[1],droneOrientation[2]
			};

			float* droneForceAndMoments = droneController.controlMotors(wantedPosture, realPosture, dt);
			float forceAndTorque[6] = { 0,0,droneForceAndMoments[0], droneForceAndMoments[1],droneForceAndMoments[2],droneForceAndMoments[3] };
			cout << "Fz drone:" << droneForceAndMoments[0] << endl;
			simxCallScriptFunction(clientID, "Quadricopter", sim_scripttype_childscript, "addForceAndTorque_function", 1, &drone, 6, forceAndTorque, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);

			simxGetObjectOrientation(clientID, droneBase, -1, droneOrientation, simx_opmode_oneshot_wait);
			simxGetObjectPosition(clientID, droneBase, -1, dronePos, simx_opmode_oneshot_wait);


		//	float rmse = sqrt((
		//		pow(x[0].at(i) - dronePos[0], 2)
		//		+ pow(y[0].at(i) - dronePos[1], 2)
		//		+ pow(z[0].at(i) - dronePos[2], 2)
		//		+ pow(roll[0].at(i) - droneOrientation[0], 2)
		//		+ pow(pitch[0].at(i) - droneOrientation[1], 2)
		//		+ pow(yaw[0].at(i) - droneOrientation[2], 2)
		//		) / 6);
			//	while (abs(rmse) >= 0.02) {
		//	cout << "rmse: " << rmse << endl;
			


			//plot(plotImg, "Trajectory", x[0].at(x[0].size() - 1), -y[0].at(y[0].size() - 1), z[0].at(z[0].size() - 1));
			//plot(plotVel, "Velocities",t*5, -x[1].at(x[1].size() - 1));
			//float pidVal = PIDs[0].compute(-x[1].at(x[1].size() - 1), -x[1].at(x[1].size() - 1) * distribution(generator), 0.1);
			//plot(plotVelPID, "VelocitiesPID", t * 5, pidVal);

			

				//Mat realInertialLinearVel = getInertialLinearVel(Vec3f(droneOrientation[0], droneLinearVel[1], droneLinearVel[2]), droneOrientation[0], droneOrientation[1], droneOrientation[2]);
				//Mat realInertialAngularVel = getInertialAngularVel(Vec3f(droneAngularVel[0], droneAngularVel[1], droneAngularVel[2]), droneOrientation[0], droneOrientation[1]);
				//float linearVels[3] = { x[1].at(i) - realInertialLinearVel.at<float>(0),  y[1].at(i) - realInertialLinearVel.at<float>(1), z[1].at(i) - realInertialLinearVel.at<float>(2) };
			//	float angularVels[3] = { roll[1].at(i) - realInertialAngularVel.at<float>(0), pitch[1].at(i) - realInertialAngularVel.at<float>(1), yaw[1].at(i) - realInertialAngularVel.at<float>(2) };
				//linearVel = Mat(3, 1, CV_32F, linearVels);
				//angularVel = Mat(3, 1, CV_32F, angularVels);
				
			//	linearVel = getBodyLinearVel(Vec3f(linearVels[0], linearVels[1], linearVels[2]), droneOrientation[0], droneOrientation[1], droneOrientation[2]);
			//	angularVel = getBodyAngularVel(Vec3f(angularVels[0], angularVels[1], angularVels[2]), droneOrientation[0], droneOrientation[1]);
			//	bodyForces = getAppliedForces(angularVel, linearVel);
			//	bodyTorques = getAppliedTorques(angularVel);

				
				//motorsAngularVel = getMotorsAngularVel(Vec4f(bodyForces.at<float>(2), bodyTorques.at<float>(0), bodyTorques.at<float>(1), bodyTorques.at<float>(2)));
				//float forceAndTorque[6] = { 0.f,0.f,getMotorForce(motorsAngularVel.at<float>(0)), 0.f, 0.f, getMotorTorque(motorsAngularVel.at<float>(0)) };
				//simxSynchronous(clientID, true);
				//simxCallScriptFunction(clientID, "Quadricopter", sim_scripttype_childscript, "addForceAndTorque_function", 1, &m1, 6, forceAndTorque, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
				//float forceAndTorque1[6] = { 0.f,0.f,getMotorForce(motorsAngularVel.at<float>(1)), 0.f, 0.f, getMotorTorque(motorsAngularVel.at<float>(1)) };
				//simxCallScriptFunction(clientID, "Quadricopter", sim_scripttype_childscript, "addForceAndTorque_function", 1, &m2, 6, forceAndTorque1, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
				//float forceAndTorque2[6] = { 0.f,0.f,getMotorForce(motorsAngularVel.at<float>(2)), 0.f, 0.f, getMotorTorque(motorsAngularVel.at<float>(2)) };
				//simxCallScriptFunction(clientID, "Quadricopter", sim_scripttype_childscript, "addForceAndTorque_function", 1, &m3, 6, forceAndTorque2, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
				//float forceAndTorque3[6] = { 0.f,0.f,getMotorForce(motorsAngularVel.at<float>(3)), 0.f, 0.f, getMotorTorque(motorsAngularVel.at<float>(3)) };
				//simxCallScriptFunction(clientID, "Quadricopter", sim_scripttype_childscript, "addForceAndTorque_function", 1, &m4, 6, forceAndTorque3, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
				//simxSynchronous(clientID, false);
				Sleep(80);
			//	rmse = sqrt((
			//		pow(x[0].at(i) - dronePos[0], 2)
			//		+ pow(y[0].at(i) - dronePos[1], 2)
			//		+ pow(z[0].at(i) - dronePos[2], 2)
			//		+ pow(roll[0].at(i) - droneOrientation[0], 2)
			//		+ pow(pitch[0].at(i) - droneOrientation[1], 2)
			//		+ pow(yaw[0].at(i) - droneOrientation[2], 2)
			//		) / 6);
			
	//		}
			
		}
		//waitKey();
			
	}
	else
	{
		cout << " Connection status to VREP: FAILED" << endl;
	}
	simxFinish(clientID);
    return 0;
}