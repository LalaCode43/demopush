#include "color_utils.h"
#include <QDebug>
Scalar upperYellow(25, 255, 255);
Scalar lowerYellow(20, 210, 160);

//Scalar upperBlue(110, 255, 146);
//Scalar lowerBlue(103, 245, 100);
Scalar upperBlue(110, 255, 255);
Scalar lowerBlue(100, 197, 80);

Scalar upperRed(15, 255, 255);
Scalar lowerRed(0, 130, 110);

void  detectColor(const Mat& hsvImg, Mat& mask, ObjectColor& labelColor) {
    int count = 0, max_count = 0;
    Mat red_mask, blue_mask, yellow_mask;

    // YELLOW FILTER
    //inRange(hsvImg, Scalar(20, 210, 160), Scalar(25, 255, 255), yellow_mask);
    inRange(hsvImg, lowerYellow, upperYellow, yellow_mask);
    max_count = countNonZero(yellow_mask);
    mask = yellow_mask.clone();
    labelColor = ObjectColor::YELLOW;


    // BLUE FILTER
    inRange(hsvImg, lowerBlue, upperBlue, blue_mask);
    count = countNonZero(blue_mask);
    if(count > max_count){
        max_count = count;
        mask = blue_mask.clone();
        labelColor = ObjectColor::BLUE;
    }

    //RED FILTER
    inRange(hsvImg, lowerRed, upperRed, red_mask);
    count = countNonZero(red_mask);
    if(count > max_count){
        //max_count = count;
        mask = red_mask.clone();
        labelColor = ObjectColor::RED;
    }

}

void drawRotatedBox(Mat& img, const Point& pt1, const Point& pt2, const Point& pt3, const Point& pt4) {
	line(img, pt1, pt2, Scalar(0, 0, 255), 1);
	line(img, pt2, pt3, Scalar(0, 0, 255), 1);
	line(img, pt3, pt4, Scalar(0, 0, 255), 1);
	line(img, pt4, pt1, Scalar(0, 0, 255), 1);
}

void drawRotatedBox(Mat& img, const Point2f pts[4]) {
	for (int i = 0; i < 4; i++) {
		line(img, pts[i], pts[(i + 1) % 4], Scalar(0, 0, 255), 1);
	}
}

void detectShape(const vector<Point>& contours, vector<Point>& approx, ObjectShape& objectShape)
{
    approxPolyDP(contours, approx, arcLength(contours, true) * 0.04, true);
	if (approx.size() == 4) {
        objectShape = ObjectShape::RECTANGLE;
    } else if(approx.size() == 3){
        objectShape = ObjectShape::TRIANGLE;
    }
    else if (approx.size()>4){
		objectShape = ObjectShape::CIRCLE;
	}
}

void backGroundSubtraction(const Mat& img, const Mat& bg, Mat &fg, const Mat& element, int blurSize, int T) {

    absdiff(img, bg, fg);
    GaussianBlur(fg, fg, Size(blurSize, blurSize), 0);
    threshold(fg, fg, T, 255, THRESH_BINARY);
    // openning and closing
    erode(fg, fg, element);
    dilate(fg, fg, element);
    dilate(fg, fg, element);
    erode(fg, fg, element);
}

float getAngle(const Point& centroid, const Point& minPoint) {

    float angle = atan2(-(minPoint.y - centroid.y), minPoint.x - centroid.x)* 57.296;
    //qDebug() << "angle" << angle;
    if (angle > 90.0 and angle < 185) {

        return (angle - 180);
    }
    else if (angle<-90.0 and angle>-185) {

        return (angle + 180);
    } else {
        return angle;
    }

}

double distance3d(float point1[3], float point2[3]){
    return sqrt(pow(point1[0] - point2[0] , 2) + pow(point1[1] - point2[1] , 2) + pow(point1[2] - point2[2] , 2));
}

double distance3d(const Mat& point1, const Mat& point2){
    return norm(point1-point2, NORM_L2);
}

double distance2d(float pixel1[2], float pixel2[2]){
    return sqrt(pow(pixel1[0] - pixel2[0], 2) +  pow(pixel1[1] - pixel2[1], 2));
}
double distance2d(const Point& p1, const Point& p2){
    return sqrt(pow(p1.x-p2.x, 2) +  pow(p1.y-p2.y, 2));
}
