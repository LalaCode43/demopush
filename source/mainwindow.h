#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QDebug>
#include "yrc1000micro_com.h"
#include "realsensecapture.h"
#include "uart.h"
#include <QTimer>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
    QThread realSenseThread;
    QThread uartThread;

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_btn_connect_clicked();

    void on_btn_servo_clicked();

    void on_btn_go_home_clicked();

    void on_btn_get_postion_clicked();

    void on_btn_get_joint_clicked();

    void on_btn_set_postion_clicked();

    void on_rbtn_movj_clicked();

    void on_rbtn_movl_clicked();

    void on_rbtn_movi_clicked();

    void on_btn_load_job_clicked();

    void on_btn_start_job_clicked();

    void on_btn_open_camera_clicked();

    void on_btn_stop_job_clicked();

    void on_btn_send_clicked();

    void on_btn_depth_map_clicked();

    void on_btn_mask_clicked();

    void updateColorLabel(const ObjectColor objectColor);

    void updateShapeLabel(const ObjectShape objectShape);

    void on_btn_depth_map_2_clicked();

    void on_btn_save_data_clicked();

    void on_btn_inc_X_clicked();

    void on_btn_dec_X_clicked();

    void on_btn_inc_Y_clicked();

    void on_btn_dec_Y_clicked();

    void on_btn_inc_Z_clicked();

    void on_btn_dec_Z_clicked();

    void on_btn_inc_Roll_clicked();

    void on_btn_dec_Roll_clicked();

    void on_btn_inc_Pitch_clicked();

    void on_btn_dec_Pitch_clicked();

    void on_btn_inc_Yaw_clicked();

    void on_btn_dec_Yaw_clicked();

public slots:
    void updateUdpData(quint8);
    void updateRealSenseCapture();
    void updateObjectDetected();
    void updateBackgroundStatus();
private:
    Ui::MainWindow *ui;
    YRC1000micro_com *yrc_com;
    RealSenseCapture *rsCapture;
    UART *uart;

    float conveyor_rate;
    QTimer *systemTimer, *statusTimer;
    QVector<double> robot_position;
    QVector<double> robot_pulse;

    QPixmap displayPix, displayObject;
    Mat objectDetectImg;
    Mat object_position;
    // Position for placing object
    QVector<Point> yellowPose;
    QVector<Point> bluePose;
    QVector<Point> redPose;
    QVector<Point> rectPose;
    QVector<Point> triPose;
    QVector<Point> cirPose;
    QVector<Point> allPlacePose;
    Point rightBox, middleBox, leftBox;
    // Job name
    QString job_name ;

    // number of detected objects
    int numObjectDetect;
    int numRedObject, numBlueObject, numYellowObject;
    int numCircleObject,numTriangleObject, numRectangleObject;

    double counter;
    double system_cycle;

    // Position and Point
    Mat readyPoint, waitingPoint, homePoint, pickPoint, placePoint, placeUpPoint, pickUpPoint;
    QVector<double> readyPos,waitingPos, homePos, pickPos, placePos, placeUpPos, pickUpPos;

    // Object informations
    float object_height, yaw_angle, object_min_distance;
    ObjectColor objectColor;
    ObjectShape objectShape;

    quint32 val_d1, val_d2,val_d3,val_d4,val_d5;
    quint8 byte_3, byte_9, byte_10;

    Mat display;
    int type_display;
    float distance_to_object, distance_to_pick;

    // flags of auto systems
    bool object_detected, system_obejct_detect;
    bool is_tracking;
    bool waiting_to_place;
    bool waiting_for_complete;

    // Flags system status
    bool servo_on, is_running, is_teaching, is_playback, is_remote;

    float tracking_y;
    void updateStatusServo();
    void updateRobotPosionGUI();
    void updateRobotPulseGUI();
    void displayImage(Mat);
    void convertToQtPixmap(const Mat& imgMat, QPixmap &pixmap);
    void systemRunAuto();
    void updateObejctInfoGUI();
    void updateSystemStatus();
    void readSystemStatus();

    //
    void setWaitingPosition(double pos0, double pos1, double pos2,double pos3,double pos4,double pos5);
    void setPickingPosition(double pos0, double pos1, double pos2,double pos3,double pos4,double pos5);

    // vector to storing measurement data
    vector<double> fsDataHeight;
    vector<double> fsDataWidth;
    FileStorage fsDataFile;
};
#endif // MAINWINDOW_H
