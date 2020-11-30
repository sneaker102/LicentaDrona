#include "ardrone/ardrone.h"
#include <math.h> 
#include <vector>

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
	vector<Point> points;
	//namedWindow("Trajectory", WINDOW_AUTOSIZE);
	//Mat plotImg = Mat::zeros(800, 800, CV_8UC3);
	//namedWindow("Velocities", WINDOW_AUTOSIZE);
	//Mat plotVel = Mat::zeros(800, 800, CV_8UC3);
	//namedWindow("Accelerations", WINDOW_AUTOSIZE);
	//Mat plotAcc = Mat::zeros(800, 800, CV_8UC3);
	for (float t = 0.f; t <= t_f; t += 0.1f) {
		setTrajectory(x, xQuinticCoeffs, t);
		setTrajectory(y, yQuinticCoeffs, t);
		setTrajectory(z, zQuinticCoeffs, t);
		setTrajectory(roll, rollQuinticCoeffs, t);
		setTrajectory(pitch, pitchQuinticCoeffs, t);
		setTrajectory(yaw, yawQuinticCoeffs, t);
		//plot(plotImg, "Trajectory", x[0].at(x[0].size() - 1), -y[0].at(y[0].size() - 1), z[0].at(z[0].size() - 1));
		//plot(plotVel, "Velocities",t*5, -y[1].at(y[1].size() - 1));
		//plot(plotAcc, "Accelerations", t * 5, -y[2].at(y[2].size() - 1));
	}

	//waitKey();
    return 0;
}