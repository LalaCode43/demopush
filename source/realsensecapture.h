#ifndef REALSENSECAPTURE_H
#define REALSENSECAPTURE_H

#include <QObject>
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <QPixmap>
#include <QDebug>
#include <QThread>
#include <QElapsedTimer>
#include <stdio.h>
#include "color_utils.h"
using namespace cv;
using namespace rs2;
using namespace std;

typedef struct ObjectInfo{
    float distance;
    int color;
} ObjectInfo;

class RealSenseCapture : public QObject
{
    Q_OBJECT
public:
    explicit RealSenseCapture(QObject *parent = nullptr);

    void runCapture();

    void getRotationFromAfrray(rs2_extrinsics extrinsic, Mat& R);

    void getTranslationFromAfrray(rs2_extrinsics extrinsic, Mat& T);

    void showIntrinsicsInfo(rs2_intrinsics intrinsics);

    void color_to_depth(const uint16_t *data, float from_pixel[], float to_pixel[]);

    void setCameraOpen(bool newCameraOpen);

    void setIs_object_detected(bool newIs_object_detected);

    void setTracking_object(bool newTracking_object);

    Mat getColorImg() const;

    Mat getPoint_from_base() const;

    Mat getMask() const;

    void displayMat3x1(Mat r);

    float getMinDistance(vector<Point> contour, Point centroid, Point &minPoint);

    float getYaw_angle() const;


    float getObject_min_distance() const;


    float getObject_height() const;

    Mat getDepthImg() const;
    ObjectColor getObjectColor() const;


    void setMotionDetect(bool newMotionDetect);

    void setColorDetect(bool newColorDetect);

    ObjectShape getObjectShape() const;

    Mat getObject_detect_image() const;

signals:
    void realsenseCaptureSignal();
    void objectDetected();
    void objectPosition();
    void getBackGround();

private:
    Ptr<BackgroundSubtractor> pBgSubtract;
    bool cameraOpen;
    int top_limit, bottom_limit, left_limit, right_limit;
    pipeline pipe;
    config cfg;
    pipeline_profile profile;
    stream_profile depth_stream, color_stream;
    rs2_intrinsics depth_intrin, color_intrin;
    rs2_extrinsics depth_extrin, color_extrin;
    Mat R_cam2base, t_cam2base;
    Mat depthImg, colorImg, mask, display;
    Mat point_from_base;
    Mat background, roi_object;
    float object_height, object_min_distance;
    float conveyor_height;
    float time;
    void configRealsense();
    bool is_object_detected;
    bool tracking_object;
    Mat roi_img, roi_bg, roi_mask, roi_gray;
    float yaw_angle;
    float minDist3D;
    bool motionDetect, colorDetect;
    int colorTrigger;
    ObjectColor objectColor;
    ObjectShape objectShape;
    Scalar upperLimit, lowerLimit;
    Mat roi_color, object_detect_image;
    int numVertices;

};

#endif // REALSENSECAPTURE_H
