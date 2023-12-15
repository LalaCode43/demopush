#ifndef MATOPERATOR_H
#define MATOPERATOR_H
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "librealsense2/rs.hpp"
using namespace std;
using namespace cv;

//convert degree to rad
double deg2rad(double deg);

//convert rad to degree 
double rad2deg(double rad);

//convert pulse to degree
double pulse2deg(double pulse, int idx);

// Get Matrix traslation
void getTranslation(double x, double y, double z, Mat& T);

// Get Rotation Matrix rotating about X axis by the angle roll
void getRotationX(double roll, Mat& R);

// Get Rotation Matrix rotating about Y axis by the angle pitch
void getRotationY(double pitch, Mat& R);

// Get Rotation Matrix rotating about Z axis by the angle yaw
void getRotationZ(double yaw, Mat& R);

// Get Homogenous Transform Matrix from frame i to frame i-1
void getDHHomogenousTransform(double a, double alpha, double d, double theta, Mat& H);

// Get Inverse rotation R_inv and translation T_inv from rotation R and translation T
void getInverseRT(Mat R1, Mat T1, Mat& R_inv, Mat& T_inv);

// Get Homogenous Transform with rotation R and translation T
void getHomogenousTransorm(Mat R, Mat T, Mat& H);

// Get Inverse Homogenous Transform H_inv from H
void getInverseHomogenousTransorm(Mat H, Mat& H_inv);


// Extract rotation matrix R and translation T from Homogenous Transform H
void getRTFromHomogenousTrasnform(Mat H, Mat& R, Mat& T);

// Calculate Rotation Matrix with rotation angles roll, pitch and yaw
void getRPYMatrix(double roll, double pitch, double yaw, Mat& R);

// Calculate roll, pitch, yaw from Rotation matrix
void getRPYAngle(Mat R, Vec3d& vec1, Vec3d& vec2);
void getHomogenousRobotFromDegree(double j1, double j2, double j3, double j4, double j5, double j6, Mat& H);
void getHomogenousRobotFromRad(double j1, double j2, double j3, double j4, double j5, double j6, Mat& H);
void getHomogenousRobotFromPulse(int j1, int j2, int j3, int j4, int j5, int j6, Mat& H);

///////////////////////////////////////
#endif // MATOPERATOR_H

