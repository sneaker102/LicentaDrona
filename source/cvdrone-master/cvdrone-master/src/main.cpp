#include "ardrone/ardrone.h"
#include <math.h> 
#include <vector>
#include "../../cvdrone-master/build/vs2015/PID.h"
#include <random>

// Fallback, in case M_PI doesn't exist.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using namespace cv;


float degToRad(float deg) {
	return deg * M_PI / 180.f;
}

float radToDeg(float rad) {
	return rad * 180.f / M_PI;
}

void calcQuinticCoeffs(Mat &quinticCoeffs, float t_s, float t_f, float q_s, float q_f, float dq_s = 0, float dq_f = 0, float ddq_s = 0, float ddq_f = 0) {
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
	quinticCoeffs = quinticTimeMatrix.inv() * Mat(constraints);
}

void setTrajectory(vector<float> (&traj)[3], Mat quinticCoeffs, float t) {
	traj[0].push_back(quinticCoeffs.at<float>(0) + quinticCoeffs.at<float>(1)*t + quinticCoeffs.at<float>(2)*pow(t, 2) + quinticCoeffs.at<float>(3)*pow(t, 3) + quinticCoeffs.at<float>(4)*pow(t, 4) + quinticCoeffs.at<float>(5)*pow(t, 5));
	traj[1].push_back(quinticCoeffs.at<float>(1) + 2 * quinticCoeffs.at<float>(2)*t + 3 * quinticCoeffs.at<float>(3)*pow(t, 2) + 4 * quinticCoeffs.at<float>(4)*pow(t, 3) + 5 * quinticCoeffs.at<float>(5)*pow(t, 4));
	traj[2].push_back(2 * quinticCoeffs.at<float>(2) + 6 * quinticCoeffs.at<float>(3)*t + 12 * quinticCoeffs.at<float>(4)*pow(t, 2) + 20 * quinticCoeffs.at<float>(5)*pow(t, 3));
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
	return transformMatrix * Mat(inertialAngularVel);
}

Mat getInertialAngularVel(Vec3f bodyAngularVel, float roll, float pitch) {
	float transformData[3][3] = {
		{1.f, sin(roll)*tan(pitch), cos(roll)*tan(pitch)},
		{0.f, cos(roll), -sin(roll)},
		{0.f, sin(roll)/cos(pitch), cos(roll)/cos(pitch)}
	};
	Mat transformMatrix(3, 3, CV_32F, transformData);
	return transformMatrix * Mat(bodyAngularVel);
}

Mat getAppliedForces(Mat bodyAngularVel, Mat bodyLinearVel) {
	float m = 0.06; //kg
	return m * (bodyAngularVel.cross(bodyLinearVel) + bodyLinearVel);
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
	return inerAngularProd + bodyAngularVel.cross(inerAngularProd);
}

int main(int argc, char *argv[])
{
	// x,y,z,roll,pitch,yaw
	float waypoints[2][6] = {
		{0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{50.f, 35.5f, 15.f, 0.f, 0.f, degToRad(90.f)}
	};
	float t_f = 5.f;
	Mat xQuinticCoeffs, yQuinticCoeffs, zQuinticCoeffs, pitchQuinticCoeffs, rollQuinticCoeffs, yawQuinticCoeffs;
	calcQuinticCoeffs(xQuinticCoeffs, 0.f, t_f, waypoints[0][0], waypoints[1][0]);
	calcQuinticCoeffs(yQuinticCoeffs, 0.f, t_f, waypoints[0][1], waypoints[1][1]);
	calcQuinticCoeffs(zQuinticCoeffs, 0.f, t_f, waypoints[0][2], waypoints[1][2]);
	calcQuinticCoeffs(pitchQuinticCoeffs, 0.f, t_f, waypoints[0][3], waypoints[1][3]);
	calcQuinticCoeffs(rollQuinticCoeffs, 0.f, t_f, waypoints[0][4], waypoints[1][5]);
	calcQuinticCoeffs(yawQuinticCoeffs, 0.f, t_f, waypoints[0][5], waypoints[1][5]);

	vector<float> x[3];
	vector<float> y[3];
	vector<float> z[3];
	vector<float> roll[3];
	vector<float> pitch[3];
	vector<float> yaw[3];
	//namedWindow("Trajectory", WINDOW_AUTOSIZE);
	//Mat plotImg = Mat::zeros(800, 800, CV_8UC3);
	//namedWindow("Velocities", WINDOW_AUTOSIZE);
	//Mat plotVel = Mat::zeros(800, 800, CV_8UC3);
	//namedWindow("VelocitiesPID", WINDOW_AUTOSIZE);
	//Mat plotVelPID = Mat::zeros(800, 800, CV_8UC3);
	//PID PIDs[4] = { PID(1.f,0.f,1.f),PID(1.f,0.f,1.f),PID(1.f,0.f,1.f),PID(1.f,0.f,1.f) };
	//default_random_engine generator;
	//uniform_real_distribution<double> distribution(0.1, 1.3);
	for (float t = 0.f; t <= t_f; t += 0.1f) {
		setTrajectory(x, xQuinticCoeffs, t);
		setTrajectory(y, yQuinticCoeffs, t);
		setTrajectory(z, zQuinticCoeffs, t);
		setTrajectory(roll, rollQuinticCoeffs, t);
		setTrajectory(pitch, pitchQuinticCoeffs, t);
		setTrajectory(yaw, yawQuinticCoeffs, t);
		//plot(plotImg, "Trajectory", x[0].at(x[0].size() - 1), -y[0].at(y[0].size() - 1), z[0].at(z[0].size() - 1));
		//plot(plotVel, "Velocities",t*5, -x[1].at(x[1].size() - 1));
		//float pidVal = PIDs[0].compute(-x[1].at(x[1].size() - 1), -x[1].at(x[1].size() - 1) * distribution(generator), 0.1);
		//plot(plotVelPID, "VelocitiesPID", t * 5, pidVal);
		
	
	}
	//waitKey();

	// todo interface with v-rep to get roll, pitch, yaw
	Mat linearVel, angularVel, bodyForces, bodyTorques;
	for (int i = 0; i < x[1].size(); i++) {
		linearVel = getBodyLinearVel(Vec3f(x[1].at(i), y[1].at(i), z[1].at(i)), 0, 0, 0);
		angularVel = getBodyAngularVel(Vec3f(roll[1].at(i), pitch[1].at(i), yaw[1].at(i)), 0, 0);
		bodyForces = getAppliedForces(angularVel, linearVel);
		bodyTorques = getAppliedTorques(angularVel);
	}
	
	
    return 0;
}