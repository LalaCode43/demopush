#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "matoperator.h"
using namespace std;
using namespace cv;

void getTranslation(double x, double y, double z, Mat& T) {
	T = (Mat_<double>(3, 1) << x, y, z);
}


void getDHHomogenousTransform(double a, double alpha, double d, double theta, Mat& H) {

	Mat Rx, Tx, Rz, Tz, Hz, Hx;

	getRotationZ(theta, Rz);
	getTranslation(0, 0, d, Tz);
	getRotationX(alpha, Rx);
	getTranslation(a, 0, 0, Tx);
	getHomogenousTransorm(Rz, Tz, Hz);
	getHomogenousTransorm(Rx, Tx, Hx);

	/*H = Rd * Td * Ta * Ra;*/
	H = Hz * Hx;
}

void getInverseRT(Mat R1, Mat T1, Mat&	R_inv, Mat& T_inv) {
	R_inv = R1.t();
	T_inv = -R_inv * T1;
}

//convert degree to rad
double deg2rad(double deg) {
	return deg * CV_PI / 180.0;
}

//convert rad to degree 
double rad2deg(double rad) {
	return rad * 180.0 / CV_PI;
}

// convert pulse to degre
double pulse2deg(double pulse, int idx) {
	if (idx == 1) {
		return pulse * 30 / 34816.0;
	}
	else if (idx == 2) {
		return pulse * 90 / 102400.0;
	}
	else if (idx == 3) {
		return pulse * 90 / 51200.;
	}
	else
	{
		return pulse * 30 / 10204.;
	}
}

void getHomogenousTransorm(Mat R, Mat T, Mat& H) {

	Mat unit = (Mat_<double>(1, 4) << 0, 0, 0, 1);
	hconcat(R, T, H);
	vconcat(H, unit, H);
}

void getRTFromHomogenousTrasnform(Mat H, Mat& R, Mat& T) {
	R = H(Range(0, 3), Range(0, 3)).clone();
	T = H(Range(0, 3), Range(3, 4)).clone();
}

void getInverseHomogenousTransorm(Mat H, Mat& H_inv) {
	Mat R, T;

	getRTFromHomogenousTrasnform(H, R, T);

	getHomogenousTransorm(R.inv(), -R.inv() * T, H_inv);
}




// Get Matrix rotate about X axis with angle roll
void getRotationX(double roll, Mat& R) {
	R = (Mat_<double>(3, 3) << 1, 0, 0,
		0, cos(roll), -sin(roll),
		0, sin(roll), cos(roll));
}

// Get Matrix rotate about Y axis with angle pitch
void getRotationY(double pitch, Mat& R) {
	R = (Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch),
		0, 1, 0,
		-sin(pitch), 0, cos(pitch));
}
// Get Matrix rotate about Z axis with angle yaw
void getRotationZ(double yaw, Mat& R) {
	R = (Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0,
		sin(yaw), cos(yaw), 0,
		0, 0, 1);
}
void getRPYMatrix(double roll, double pitch, double yaw, Mat& R) {

	Mat Rx, Ry, Rz;
	getRotationX(roll, Rx);
	getRotationY(pitch, Ry);
	getRotationZ(yaw, Rz);

	R = Rz * Ry * Rx;
}
// Calculate roll, pitch, yaw from Rotation matrix
void getRPYAngle(Mat R, Vec3d& vec1, Vec3d& vec2) {
	double r11 = R.at<double>(0, 0);
	double r21 = R.at<double>(1, 0);
	double r31 = R.at<double>(2, 0);
	double r32 = R.at<double>(2, 1);
	double r33 = R.at<double>(2, 2);

	double pitch1, pitch2;
	pitch1 = atan2(-r31, sqrt(r32 * r32 + r33 * r33));
	pitch2 = atan2(-r31, -sqrt(r32 * r32 + r33 * r33));

	double roll1, roll2;
	roll1 = atan2(r32 / cos(pitch1), r33 / cos(pitch1));
	roll2 = atan2(r32 / cos(pitch2), r33 / cos(pitch2));

	double yaw1, yaw2;
	yaw1 = atan2(r21 / cos(pitch1), r11 / cos(pitch1));
	yaw2 = atan2(r21 / cos(pitch2), r11 / cos(pitch2));
	/*vec1 = Vec3d(roll1, pitch1, yaw1);
	vec2 = Vec3d(roll2, pitch2, yaw2);*/

	vec1 = Vec3d(rad2deg(roll1), rad2deg(pitch1), rad2deg(yaw1));
	vec2 = Vec3d(rad2deg(roll2), rad2deg(pitch2), rad2deg(yaw2));
}

void getHomogenousRobotFromDegree(double j1, double j2, double j3, double j4, double j5, double j6, Mat& H) {
    Mat H1, H2, H3, H4, H5, H6, H7;
    getDHHomogenousTransform(20, deg2rad(-90), 0, deg2rad(j1), H1);
	getDHHomogenousTransform(165, deg2rad(-180), 0, deg2rad(j2 - 90), H2);
	getDHHomogenousTransform(0, deg2rad(-90), 0, deg2rad(j3), H3);
	getDHHomogenousTransform(0, deg2rad(90), -165, deg2rad(j4), H4);
	getDHHomogenousTransform(0, deg2rad(-90), 0, deg2rad(j5 - 90), H5);
    //getDHHomogenousTransform(0, deg2rad(90), -80, deg2rad(j6), H6);

    getDHHomogenousTransform(0, 0, -40, deg2rad(j6), H6);
    getDHHomogenousTransform(0, deg2rad(180), -40, 0, H7);
    H = H1 * H2 * H3 * H4 * H5 * H6 * H7;
}
void getHomogenousRobotFromRad(double j1, double j2, double j3, double j4, double j5, double j6, Mat& H) {
    Mat H1, H2, H3, H4, H5, H6;
	getDHHomogenousTransform(20, deg2rad(-90), 0, j1, H1);
	getDHHomogenousTransform(165, deg2rad(180), 0, j2 - CV_PI / 2, H2);
	getDHHomogenousTransform(0, deg2rad(90), 0, j3, H3);
	getDHHomogenousTransform(0, deg2rad(-90), 165, j4, H4);
	getDHHomogenousTransform(0, deg2rad(-90), 0, j5 - CV_PI / 2, H5);
	getDHHomogenousTransform(0, 0, -40, j6, H6);
	H = H1 * H2 * H3 * H4 * H5 * H6;
}
void getHomogenousRobotFromPulse(double j1, double j2, double j3, double j4, double j5, double j6, Mat& H) {
    Mat H1, H2, H3, H4, H5, H6, H7;

	getDHHomogenousTransform(20, deg2rad(-90), 0, deg2rad(pulse2deg(j1, 1)), H1);
	getDHHomogenousTransform(165, deg2rad(-180), 0, deg2rad(pulse2deg(j2, 2) - 90), H2);
	getDHHomogenousTransform(0, deg2rad(-90), 0, deg2rad(pulse2deg(j3, 3)), H3);
	getDHHomogenousTransform(0, deg2rad(90), -165, deg2rad(pulse2deg(j4, 4)), H4);
    getDHHomogenousTransform(0, deg2rad(-90), 0, deg2rad(pulse2deg(j5, 5) - 90), H5);
    getDHHomogenousTransform(0, 0, -40, deg2rad(pulse2deg(j6, 6)), H6);
    getDHHomogenousTransform(0, deg2rad(180), 0, 0, H7);

    H = H1 * H2 * H3 * H4 * H5 * H6 * H7;

}

