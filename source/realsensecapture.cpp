#include "realsensecapture.h"

RealSenseCapture::RealSenseCapture(QObject *parent)
    : QObject{parent}
{
    background = imread("../resources/background.png");
    pBgSubtract = createBackgroundSubtractorMOG2(500, 150, false);
    mask = Mat::zeros(480, 640, CV_8UC1);

    ///////////////// OBJECT PARAMETERS ///////////
    object_height = 0.;
    conveyor_height = -93;
    yaw_angle = -90;

    ///////// READ CAMERA MODEL /////////
    FileStorage fs;
    fs.open("../eye-to-hand/eye-to-hand-2.xml", FileStorage::READ);
    fs["R_cam2base"] >> R_cam2base;
    fs["t_cam2base"] >> t_cam2base;
    fs.release();
    configRealsense();
    object_min_distance = 0;


    ///////////// WORKING AREA //////////
    top_limit = 70;
    bottom_limit = 260;
    left_limit = 50;
    right_limit = 550;

    //////////// TRIGGERS //////////
    colorTrigger = 150;

    ////////////// FLAGS //////////////
    motionDetect = false;
    colorDetect = false;
    is_object_detected = false;
    tracking_object = false;
    cameraOpen = false;
}





void RealSenseCapture::displayMat3x1(Mat r){
    qDebug()<<QString::number(r.at<double>(0))<<", "<<QString::number(r.at<double>(1))<<", "<<QString::number(r.at<double>(2));
}



float RealSenseCapture::getMinDistance(vector<Point> contour,Point centroid, Point& minPoint) {

    float dist, dist_min = 1000000;
    for (int j = 0; j < contour.size(); j++) {
        dist = sqrt(pow(centroid.x - contour.at(j).x, 2) + pow(centroid.y - contour.at(j).y, 2));
        if (dist < dist_min) {
            dist_min = dist;
            minPoint.x = contour.at(j).x;
            minPoint.y = contour.at(j).y;
        }
    }

    return dist_min;
}



void RealSenseCapture::runCapture()
{

    float color_centroid_pix[2] = {0.}, depth_centroid_pix[2] = {0.}, color_min_pix[2] = {0},depth_min_pix[2] = {0},color_pix[2] = {0}, depth_pix[2] = {0.};
    float color_centroid_point[3] = {0.}, color_min_point[3] = {0.}, color_point[3] = {0.};
    Point color_centroid_pix_point, color_min_pix_point, color_pix_point;

    float color_centroid_DEPTH, color_min_DEPTH, color_DEPTH;
    float distance_;
    vector<vector<Point>> contours;
    vector<vector<Point>> contour_object, contour_simple;

    vector<vector<Point>>approx(1);
    Moments M, M_object;
    frameset data;
    frame depthFrame, colorFrame;
    colorizer color_map;
    Mat point_from_depth, point_from_color;
    float area;
    char info[40];

    int count = 0;
    Mat gray_background;
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));


    int trigger_point = 230;
    rs2::decimation_filter dec;
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    rs2::hole_filling_filter hole_filter;
    hole_filter.set_option(RS2_OPTION_HOLES_FILL, 1);
    rs2::disparity_transform depth_to_disparity(true);
    rs2::spatial_filter spat_filter;
    //spat_filter.set_option(RS2_OPTION_HOLES_FILL, 5);
    rs2::disparity_transform disparity_to_depth(false);
    cvtColor(background, gray_background, COLOR_BGR2GRAY);
    roi_bg = gray_background(Range(top_limit, bottom_limit), Range(left_limit, right_limit));
    roi_mask = mask(Range(top_limit, bottom_limit), Range(left_limit, right_limit));
    Mat hsvImg;
    Rect object_rect;
    Mat roi_object_mask;
    int x_rect, y_rect, w_rect, h_rect;
    int offset = 10;
    while(cameraOpen){
        ++count;

        //////////////// EXTRACT COLOR AND DEPTH IMAGE FROM REALSENSE CAMERA ////////////
        data = pipe.wait_for_frames();

        // apply filter
        depth_frame depth = data.get_depth_frame().apply_filter(depth_to_disparity).apply_filter(spat_filter).apply_filter(disparity_to_depth).apply_filter(hole_filter);
        depthFrame = depth.apply_filter(color_map);
        colorFrame = data.get_color_frame().apply_filter(color_map);
        // convert frame to CV MAT
        depthImg = Mat(depth_intrin.height, depth_intrin.width, CV_8UC3, (void*)depthFrame.get_data());
        colorImg = Mat(color_intrin.height, color_intrin.width, CV_8UC3, (void*)colorFrame.get_data());
        // convert to BGR
        cvtColor(colorImg, colorImg, COLOR_RGB2BGR);
        cvtColor(depthImg, depthImg, COLOR_RGB2BGR);

        roi_img = colorImg(Range(top_limit, bottom_limit), Range(left_limit, right_limit));

        if(count == 60){
            motionDetect = true;
           // qDebug() << "GET BACKFROUND, START SUBTRACTION";
            emit getBackGround();
        }
        if(motionDetect==true){
            // Back ground subtraction
            pBgSubtract->apply(roi_img, roi_mask, 0);
            //erode(roi_mask, roi_mask, element);
            //dilate(roi_mask, roi_mask, element);
            //dilate(roi_mask, roi_mask, element);
            //erode(roi_mask, roi_mask, element);
            // find contours
            findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            for (int i = 0; i < contours.size(); i++){
                area = contourArea(contours.at(i));
                // check valid Area
                if (area > 500 and area <4000){
                    //Calculate Moment and centroid of object
                    M = moments(contours.at(i));
                    color_centroid_pix_point.x = static_cast<int>(M.m10 / M.m00);
                    color_centroid_pix_point.y = static_cast<int>(M.m01 / M.m00);

                    //////// OBJECT PASS TRIGGER POINT /////////
                    if(color_centroid_pix_point.x > trigger_point and is_object_detected == false){

                        object_rect = boundingRect(contours.at(i));
                        x_rect = object_rect.x, y_rect = object_rect.y, w_rect = object_rect.width, h_rect = object_rect.height;
                       // object_detect_image = colorImg(Range(y_rect-offset, y_rect+h_rect+offset), Range(x_rect-offset, x_rect+w_rect+offset)).clone();
                        object_detect_image = colorImg(Range(y_rect-offset, y_rect+h_rect+offset), Range(x_rect-offset, x_rect+w_rect+offset));

                        cvtColor(object_detect_image, hsvImg, COLOR_BGR2HSV);

                        detectColor(hsvImg, roi_object_mask, objectColor);
                        findContours(roi_object_mask, contour_simple, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
                        findContours(roi_object_mask, contour_object, RETR_EXTERNAL, CHAIN_APPROX_NONE);

                        for(int j =0; j< contour_object.size(); j++){
                            area = contourArea(contour_object.at(j));
                            if(area >300 and area < 4000){
                                // Calculate Moment and centroid

                                //circle(colorImg, color_centroid_pix_point, 2, Scalar(20, 55, 100), 2);
                                M_object = moments(contour_object.at(j));
                                color_centroid_pix[0] = M_object.m10 / M_object.m00 + x_rect-offset;
                                color_centroid_pix[1] = M_object.m01 / M_object.m00 + y_rect-offset;

                                // map color pixel to depth pixel
                                color_to_depth(reinterpret_cast<const uint16_t *>(depth.get_data()), color_centroid_pix, depth_centroid_pix);
                                // calculate depth from centroid to depht camera
                                color_centroid_DEPTH = depth.get_distance(static_cast<int>(depth_centroid_pix[0]), static_cast<int>(depth_centroid_pix[1]));
                                // deproject pixel to 3D point from color camera
                                rs2_deproject_pixel_to_point(color_centroid_point, &color_intrin, color_centroid_pix, color_centroid_DEPTH);
                                point_from_color = (Mat_<double>(3,1)<<color_centroid_point[0]*1000, color_centroid_point[1]*1000, color_centroid_point[2]*1000);
                                point_from_base = R_cam2base*point_from_color + t_cam2base;
                                sprintf(info, "(%f, %f, %f)", point_from_base.at<double>(0), point_from_base.at<double>(1), point_from_base.at<double>(2));
                               // putText(colorImg, info, Point(left_limit-40, bottom_limit+10), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(10, 100, 200), 1 );
                                object_min_distance = 1000;
                                for(int idx = 0; idx < contour_object[j].size(); idx++){
                                    color_pix[0] = contour_object[j].at(idx).x + x_rect-offset;
                                    color_pix[1] = contour_object[j].at(idx).y + y_rect-offset;
                                    color_to_depth(reinterpret_cast<const uint16_t *>(depth.get_data()), color_pix, depth_pix);
                                    color_DEPTH =depth.get_distance(static_cast<int>(depth_pix[0]), static_cast<int>(depth_pix[1]));
                                    rs2_deproject_pixel_to_point(color_point, &color_intrin, color_pix, color_DEPTH);
                                    distance_ = distance3d(color_point, color_centroid_point);
                                    if(object_min_distance > distance_){
                                        object_min_distance = distance_;
                                        color_min_pix[0] = color_pix[0];
                                        color_min_pix[1] = color_pix[1];

                                    }
                                }//

                                for(int object_idx =0;object_idx<contour_simple.size();object_idx++){
                                    if(contourArea(contour_simple.at(object_idx)) > 300){
                                        approxPolyDP(contour_simple.at(object_idx), approx[0], arcLength(contour_simple.at(object_idx), true) * 0.035, true);
                                       // for(int idx =0; idx <approx[0].size(); idx++){
                                       //   circle(object_detect_image,Point(approx[0].at(idx).x, approx[0].at(idx).y), 2, Scalar(100, 150, 100), 2);
                                       //  }

                                        numVertices = approx[0].size();
                                       // qDebug() <<"num of vertices: " <<numVertices;


                                        if (numVertices == 3) {
                                            objectShape = ObjectShape::TRIANGLE;
                                        } else if(numVertices == 4){
                                            objectShape = ObjectShape::RECTANGLE;
                                        }
                                        else if (numVertices>4){
                                            objectShape = ObjectShape::CIRCLE;
                                        }
                                    }
                                }


                                yaw_angle = getAngle(Point(color_centroid_pix[0], color_centroid_pix[1]), Point(color_min_pix[0], color_min_pix[1]));
                                object_height = point_from_base.at<double>(2) - conveyor_height;
                                circle(colorImg,Point(color_centroid_pix[0], color_centroid_pix[1]), 2, Scalar(0, 0, 255), 2);
                                drawContours(object_detect_image, contour_object, j, Scalar(0, 0, 255), 2);

                               // circle(object_detect_image, Point(color_min_pix[0]-x_rect+offset, color_min_pix[1]-y_rect+offset), 2, Scalar(20, 55, 100), 2);
                                //line(colorImg, Point(color_min_pix[0], color_min_pix[1]),Point(color_centroid_pix[0], color_centroid_pix[1]),Scalar(0, 0, 22), 2 );
                                //putText(colorImg,to_string(object_min_distance),Point(50, 360), FONT_HERSHEY_COMPLEX_SMALL, 1, 1);
                               if(color_centroid_pix[0] <right_limit-20){
                                emit objectDetected();
                                }
                            }// check obejct area
                        } // loop through vector of OBEJCT  contours
                    }// object pass  trigger
                    if(color_centroid_pix_point.x < trigger_point){
                        drawContours(colorImg, contours, i, Scalar(255, 0, 0), 2);
                        circle(colorImg, color_centroid_pix_point, 2, Scalar(255, 0, 0), 2);
                } // check area

                }
            } // loop throuh vector of contours
        } // detect motion



        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        rectangle(colorImg, Point(left_limit, top_limit), Point(right_limit, bottom_limit), Scalar(0, 0, 255), 2);

        //putText(colorImg, "Motion", Point(left_limit-40, top_limit-10), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(10, 100, 200), 1 );
        //putText(colorImg, "Detect", Point(trigger_point-40, top_limit-10), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(10, 100, 200), 1 );

        //line(colorImg, Point(trigger_point, top_limit), Point(trigger_point, bottom_limit), Scalar(200, 200, 200));




        emit realsenseCaptureSignal();
    }
    pipe.stop();
}

float RealSenseCapture::getObject_height() const
{
    return object_height;
}

Mat RealSenseCapture::getDepthImg() const
{
    return depthImg;
}

Mat RealSenseCapture::getMask() const
{
    return mask;
}

Mat RealSenseCapture::getPoint_from_base() const
{
    return point_from_base;
}

Mat RealSenseCapture::getColorImg() const
{
   return colorImg;
}

void RealSenseCapture::getRotationFromAfrray(rs2_extrinsics extrinsic, Mat& R) {
    R = (Mat_<double>(3, 3) << extrinsic.rotation[0], extrinsic.rotation[3], extrinsic.rotation[6],
         extrinsic.rotation[1], extrinsic.rotation[4], extrinsic.rotation[7],
         extrinsic.rotation[2], extrinsic.rotation[5], extrinsic.rotation[8]);
}

void RealSenseCapture::getTranslationFromAfrray(rs2_extrinsics extrinsic, Mat& T) {
    T = (Mat_<double>(3, 1) << extrinsic.translation[0], extrinsic.translation[1], extrinsic.translation[2]);
}

void RealSenseCapture::showIntrinsicsInfo(rs2_intrinsics intrinsics) {
    qDebug() <<  "Width:\t" << QString::number(intrinsics.width) ;
    qDebug() <<  "Height:\t " << QString::number(intrinsics.height) ;
    qDebug() <<  "Fx:\t " << QString::number(intrinsics.fx) ;
    qDebug() <<  "Fy:\t " << QString::number(intrinsics.fy) ;
    qDebug() <<  "ppx:\t " << QString::number(intrinsics.ppx) ;
    qDebug() <<  "ppy:\t " << QString::number(intrinsics.ppy) ;
    qDebug() <<  "Distorion model:\t " << QString::number(intrinsics.model) ;
    qDebug() <<  "Distorion coefficients:\t" << "(" << QString::number(intrinsics.coeffs[0]) << ", " << QString::number(intrinsics.coeffs[1]) << ", " << QString::number(intrinsics.coeffs[2]) << ", " << QString::number(intrinsics.coeffs[4]) << ", " << QString::number(intrinsics.coeffs[4]) << ")" ;
}

void RealSenseCapture::configRealsense()
{
    cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_DEPTH);
    profile =  pipe.start(cfg);

    // get color and depth stream
    depth_stream = profile.get_stream(RS2_STREAM_DEPTH);
    color_stream = profile.get_stream(RS2_STREAM_COLOR);
    //int c_fps = color_stream.fps();
    //int d_fps = depth_stream.fps();
    //qDebug() << "Color fps: "<< QString::number(c_fps);
    //qDebug() << "Depth fps: "<<QString::number(d_fps);
    // intrinsics
    depth_intrin = depth_stream.as<video_stream_profile>().get_intrinsics();
    color_intrin = color_stream.as<video_stream_profile>().get_intrinsics();
    // extrinsics
    depth_extrin = depth_stream.get_extrinsics_to(color_stream);
    color_extrin = color_stream.get_extrinsics_to(depth_stream);

    //showIntrinsicsInfo(color_intrin);
    //showIntrinsicsInfo(depth_intrin);
}

Mat RealSenseCapture::getObject_detect_image() const
{
    return object_detect_image;
}

ObjectShape RealSenseCapture::getObjectShape() const
{
    return objectShape;
}

void RealSenseCapture::setColorDetect(bool newColorDetect)
{
    colorDetect = newColorDetect;
}

void RealSenseCapture::setMotionDetect(bool newMotionDetect)
{
    motionDetect = newMotionDetect;
}


ObjectColor RealSenseCapture::getObjectColor() const
{
    return objectColor;
}

float RealSenseCapture::getObject_min_distance() const
{
    return object_min_distance;
}

float RealSenseCapture::getYaw_angle() const
{
    return yaw_angle;
}

void RealSenseCapture::setTracking_object(bool newTracking_object)
{
    tracking_object = newTracking_object;
}

void RealSenseCapture::setIs_object_detected(bool newIs_object_detected)
{
    is_object_detected = newIs_object_detected;
    //qDebug() << "is_object_detected: " << is_object_detected;
}

void RealSenseCapture::color_to_depth(const uint16_t *data, float from_pixel[], float to_pixel[])
{

    rs2_project_color_pixel_to_depth_pixel(to_pixel,
                                           data,
                                           0.001, 0.3, 3,
                                           &depth_intrin, &color_intrin,
                                           &color_extrin, &depth_extrin,
                                           from_pixel);
}

void RealSenseCapture::setCameraOpen(bool newCameraOpen)
{
    cameraOpen = newCameraOpen;
}


