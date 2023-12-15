#ifndef COLOR_UTILS_H
#define COLOR_UTILS_H
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;


enum class ObjectColor
{
	YELLOW,
	BLUE,
    RED
};

enum class ObjectShape
{
    CIRCLE,
	RECTANGLE,
    TRIANGLE
};

// detect YELLOW, BLUE, RED
void  detectColor(const Mat& hsvImg, Mat& mask, ObjectColor& labelColor);

// draw rotated box with 4 points
void drawRotatedBox(Mat& img, const Point& pt1, const Point& pt2, const Point& pt3, const Point& pt4);

// draw rotated box with array of 4 points
void drawRotatedBox(Mat& img, const Point2f pts[4]);

// detect shape of Rectange, Circle
void detectShape(const vector<Point>& contours, vector<Point>& approx, ObjectShape& objectShape);

//
 void backGroundSubtraction(const Mat& img, const Mat& bg, Mat &fg, const Mat& element, int blurSize = 5, int T = 20);

//
float getAngle(const Point& centroid, const Point& minPoint);

//
double distance3d(float point1[3], float point2[3]);
double distance3d(const Mat& point1, const Mat& point2);
double distance2d(float pixel1[2], float pixel2[2]);
double distance2d(const Point& p1, const Point& p2);

#endif // COLOR_UTILS_H
