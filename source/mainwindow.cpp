#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QElapsedTimer>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    for(int i =0; i < 6; i++){
        robot_pulse.append(0.);
    }
    job_name = "";

    yrc_com = new YRC1000micro_com(this);
    connect(yrc_com, &YRC1000micro_com::udpDataReceiveUi, this, &MainWindow::updateUdpData);
    fsDataHeight.clear();
    fsDataWidth.clear();

    ///////// REALSENSE CAPTURE //////////
    QThread::currentThread()->setObjectName("Main thread");
    qDebug() << "Current thread: " <<QThread::currentThread();
    realSenseThread.setObjectName("RealSense Thread");
    rsCapture = new RealSenseCapture();
    rsCapture->moveToThread(&realSenseThread);

    connect(rsCapture, &RealSenseCapture::realsenseCaptureSignal, this, &MainWindow::updateRealSenseCapture);
    connect(&realSenseThread, &QThread::started, rsCapture, &RealSenseCapture::runCapture);
    connect(rsCapture, &RealSenseCapture::objectDetected, this, &MainWindow::updateObjectDetected);
    connect(rsCapture, &RealSenseCapture::getBackGround, this, &MainWindow::updateBackgroundStatus);


    ///////// UART PORT ////////////
    uartThread.setObjectName("Uart Thread");
    uart = new UART();
    uart->setUartPortName(ui->comboBox_port_name->currentText());
    uart->openUartPort();

    uart->moveToThread(&uartThread);
    uartThread.start();
    //////////////////////
    systemTimer = new QTimer(this);
    system_cycle = 100;
    systemTimer->setInterval(system_cycle);
    connect(systemTimer, &QTimer::timeout, this, &MainWindow::systemRunAuto);


    /////////////////////////
    statusTimer = new QTimer(this);
    statusTimer->setInterval(500);
    connect(statusTimer, &QTimer::timeout, this, &MainWindow::readSystemStatus);
    // Object Information
    object_min_distance =0;
    object_height =0.;
    distance_to_pick = 0.;

    numObjectDetect = 0;

    numCircleObject = 0;
    numTriangleObject = 0;
    numRectangleObject = 0;

    numRedObject = 0;
    numBlueObject = 0;
    numYellowObject = 0;
    counter = 0.;
    distance_to_object = 0;
    /////////////////// FLAGS ///////////////////
    byte_9 = 0;
    byte_10 = 0;
    byte_3 =0;
    is_tracking = false;
    object_detected = false;
    system_obejct_detect = false;
    waiting_for_complete = false;
    waiting_to_place = false;
    ////////////////// QVECTOR POS ///////////////////
    // ready position
    readyPos.clear();
    readyPos.append(195.0);
    readyPos.append(-190);
    readyPos.append(-50.0);
    readyPos.append(-180);
    readyPos.append(0.);
    readyPos.append(0.);

    // home position
    homePos.clear();
    homePos.append(185.);
    homePos.append(0.);
    homePos.append(35.);
    homePos.append(-180.);
    homePos.append(0);
    homePos.append(0);

    pickUpPos.clear();
    pickUpPos.append(185);
    pickUpPos.append(-215);
    pickUpPos.append(-25);
    pickUpPos.append(-180);
    pickUpPos.append(0);
    pickUpPos.append(-90);


    /////////////////// MAT POINT ///////////////////
    readyPoint = (Mat_<double>(3, 1) << 195.0, -170.0, -50.0);
    homePoint = (Mat_<double>(3, 1) << 185.0, 0. , 35.);    

    ///////////////// PLACE POSITION/////////////

    // YELLOW
    //yellowPose.append(Point(-15,-185));
    //yellowPose.append(Point(20,-185));
    //yellowPose.append(Point(55,-185));
    //yellowPose.append(Point(90,-185));
    yellowPose.append(Point(125,-185));
    yellowPose.append(Point(160,-185));


    // BLUE
    //bluePose.append(Point(-15, -234));
    //bluePose.append(Point(45, -234));
    //bluePose.append(Point(100, -234));
    //bluePose.append(Point(155, -234));
    //bluePose.append(Point(-15, -234));
    //bluePose.append(Point(20, -234));
    //bluePose.append(Point(55, -234));
    //bluePose.append(Point(90, -234));
    bluePose.append(Point(125, -234));
    bluePose.append(Point(160, -234));



    // RED
    //redPose.append(Point(-15, -280));
    //redPose.append(Point(20, -280));
    //redPose.append(Point(55, -280));
    //redPose.append(Point(90, -280));
    redPose.append(Point(125, -280));
    redPose.append(Point(160, -280));
    //////////////
    /// Rect
    rectPose.append(Point(-15,-185));
    rectPose.append(Point(20,-185));
    rectPose.append(Point(55,-185));
    rectPose.append(Point(90,-185));
    rectPose.append(Point(125,-185));
    rectPose.append(Point(160,-185));

    /// Triangle
    triPose.append(Point(-15, -234));
    triPose.append(Point(20, -234));
    triPose.append(Point(55, -234));
    triPose.append(Point(90, -234));
    triPose.append(Point(125, -234));
    triPose.append(Point(160, -234));

    // Circle
    cirPose.append(Point(-15, -280));
    cirPose.append(Point(20, -280));
    cirPose.append(Point(55, -280));
    cirPose.append(Point(90, -280));
    cirPose.append(Point(125, -280));
    cirPose.append(Point(160, -280));


    // all
    /*allPlacePose.append(Point(-15, -280));
    allPlacePose.append(Point(20, -280));
    allPlacePose.append(Point(55, -280));
    allPlacePose.append(Point(90, -280));
    */
    allPlacePose.append(Point(125, -280));
    allPlacePose.append(Point(160, -280));

    /*allPlacePose.append(Point(-15, -234));
    allPlacePose.append(Point(20, -234));
    allPlacePose.append(Point(55, -234));
    allPlacePose.append(Point(90, -234));
    */
    allPlacePose.append(Point(125, -234));
    allPlacePose.append(Point(160, -234));
/*
    allPlacePose.append(Point(-15,-185));
    allPlacePose.append(Point(20,-185));
    allPlacePose.append(Point(55,-185));
    allPlacePose.append(Point(90,-185));
*/
    allPlacePose.append(Point(125,-185));
    allPlacePose.append(Point(160,-185));


    rightBox = Point(132, -156);
    middleBox = Point(132, -224);
    leftBox = Point(131, -295);
    //////////////// FLAGS ROBOT /////////
    servo_on = false;
    is_running = false;

    /////////////////////////////////////
    val_d1 = 0;
    val_d2 = 0;
    val_d3 = 0;
    val_d4 = 0;
    val_d5 = 0;


}
void displayMat3x1(Mat r){
    qDebug()<<QString::number(r.at<double>(0))<<", "<<QString::number(r.at<double>(1))<<", "<<QString::number(r.at<double>(2));
}
MainWindow::~MainWindow()
{
    rsCapture->setCameraOpen(false);
    realSenseThread.quit();
    realSenseThread.wait();
    uartThread.quit();
    uartThread.wait();

    delete ui;
}
/////////////////// SYSTEM RUN AUTO /////////////////


void MainWindow::systemRunAuto(){
    if(object_detected == true and system_obejct_detect == false){
        counter = 0.;
        byte_9 = 0;
        byte_10 = 0;

        // camera stops detecting objects
        rsCapture->setMotionDetect(false);
        rsCapture->setIs_object_detected(true);
        system_obejct_detect = true;
        object_detected = false;
        is_tracking = true;

        // get and update color
        objectColor = rsCapture->getObjectColor();
        updateColorLabel(objectColor);

        // get and update shape
        objectShape = rsCapture->getObjectShape();
        updateShapeLabel(objectShape);

        // Get and update conveyor rate
        conveyor_rate = uart->getRate();
        ui->label_conveyor_rate->setText(QString::number(round(conveyor_rate*100)/100));
     //   qDebug() << "Conveyor Rate: " << conveyor_rate;
        // update total number of objects
        ++numObjectDetect;
        ui->label_total_object->setText(QString::number(numObjectDetect));

        // get and update minimum distance
        object_min_distance = rsCapture->getObject_min_distance();
        if(objectShape==ObjectShape::TRIANGLE){
            object_min_distance *= 3000;
        }
        else {object_min_distance *= 2000;}
        fsDataWidth.push_back(round(100*object_min_distance)/100);

        // get object position
        object_position = rsCapture->getPoint_from_base().clone();

        // get starting y
        tracking_y =rsCapture->getPoint_from_base().at<double>(1);

        // get yaw angle
        yaw_angle = rsCapture->getYaw_angle();
        // if object is cirlce, gripper does not rotate
        if(objectShape == ObjectShape::CIRCLE){
            yaw_angle = 21;
        }

        // get and update object heigth
        object_height = round(rsCapture->getObject_height()*10)/10;
        fsDataHeight.push_back(round(100*object_height)/100);

        // update GUI
        updateObejctInfoGUI();

        // Set postion to waiting
        //qDebug() << "Min distance object: " << object_min_distance;

        if(object_min_distance<19){
            setWaitingPosition(object_position.at<double>(0),
                               object_position.at<double>(1)+40, //44
                               object_position.at<double>(2),
                               -180., 0., yaw_angle);
        }else{
            setWaitingPosition(object_position.at<double>(0),
                               object_position.at<double>(1)+40, //44
                               object_position.at<double>(2),
                               -180., 0., yaw_angle);}
        // Picking position
        setPickingPosition(object_position.at<double>(0),
                           waitingPos.at(1),
                           object_position.at<double>(2)-15,
                           -180., 0., yaw_angle);

        if(object_min_distance >20){
            if(yaw_angle < 20 and yaw_angle >-20)
            {
             distance_to_pick = 24;
            }else
            {
             distance_to_pick = 27;
            }
        } else{
            distance_to_pick = 27;
        }
        waitingPoint = (Mat_<double>(3, 1) << waitingPos.at(0) ,waitingPos.at(1), waitingPos.at(2));

        // calculate distance from ready position to waiting position
        float move_to_waiting_dist = norm(waitingPoint, readyPoint, NORM_L2);

        // calculate velocity from ready pos to waiting pos
        val_d2 = quint32(move_to_waiting_dist/(0.25-0.06)*10);
        if(object_position.at<double>(1) > readyPos.at(1)){ // if object.y > readyPos.y
            val_d2 = val_d2 + 370;
        }else { val_d2 = val_d2 + 100;}

        // set variable to teach pendant
        yrc_com->setDoubleVaribale(2, val_d2);
        yrc_com->setByteVaribale(2, 1);


    }

    if(is_tracking){
        // calulate distance from object to waiting pos
        distance_to_object = waitingPos.at(1) - (tracking_y+conveyor_rate*counter);
        // update time
        counter+=system_cycle/1000;
        // if distance < threshole, pick object

        //qDebug() <<"Distance to obejct: " << distance_to_object;
        if((distance_to_object < distance_to_pick))
        {
           // qDebug() << "Pick object at: " << distance_to_object;
            pickPoint = (Mat_<double>(3 ,1) << object_position.at<double>(0), waitingPos.at(1),object_position.at<double>(2)-10);
            float distance_moving = norm(pickPoint, waitingPoint, NORM_L2);
            val_d3 = quint32(distance_moving/(0.08-0.06)*10 );
            //qDebug() << "v3: " <<val_d3;

            yrc_com->setDoubleVaribale(3, val_d3);
            ///////////////////


            yrc_com->setIntegerVaribale(0, 0);

            ///////// COLOR SORTING/////////////

            if(ui->rbtn_color->isChecked())
            {
                switch (objectColor)
                {
                    case ObjectColor::YELLOW:
                        placePos.clear();
                        placePos.append(rightBox.x);
                        placePos.append(rightBox.y);
                        placePos.append(-44 + object_height-10);
                        placePos.append(180);
                        placePos.append(0);
                        placePos.append(yaw_angle);
                        break;
                    case ObjectColor::BLUE:
                        placePos.clear();
                        placePos.append(middleBox.x);
                        placePos.append(middleBox.y);
                        placePos.append(-44 + object_height-10);
                        placePos.append(180);
                        placePos.append(0);
                        placePos.append(yaw_angle);
                        break;
                    case ObjectColor::RED:
                        placePos.clear();
                        placePos.append(leftBox.x);
                        placePos.append(leftBox.y);
                        placePos.append(-44 + object_height-10);
                        placePos.append(180);
                        placePos.append(0);
                        placePos.append(yaw_angle);

                        break;
                    default:
                        break;
                }
            }
            ////////// SHAPE SORTING //////////
            if(ui->rbtn_shape->isChecked())
            {
                switch (objectShape)
                {
                case ObjectShape::RECTANGLE:
                        ui->txt_message->setPlainText("rect");
                        placePos.clear();
                        placePos.append(rightBox.x);
                        placePos.append(rightBox.y);
                        placePos.append(-44 + object_height-10);
                        placePos.append(180);
                        placePos.append(0);
                        placePos.append(yaw_angle);


                        break;
                case ObjectShape::CIRCLE:
                        ui->txt_message->setPlainText("CIRCLE");
                        placePos.clear();
                        placePos.append(middleBox.x);
                        placePos.append(middleBox.y);
                        placePos.append(-44 + object_height-10);
                        placePos.append(180);
                        placePos.append(0);
                        placePos.append(yaw_angle);

                        break;
                case ObjectShape::TRIANGLE:
                        ui->txt_message->setPlainText("TRI");
                        placePos.clear();
                        placePos.append(leftBox.x);
                        placePos.append(leftBox.y);
                        placePos.append(-44 + object_height-10);
                        placePos.append(180);
                        placePos.append(0);
                        placePos.append(yaw_angle);

                        break;
                default:
                        break;
                }
            }

            //////////// NOT SORTING ////////////


            // pick up obejct position
            waitingPos.clear();
            waitingPos.append(object_position.at<double>(0));
            waitingPos.append(object_position.at<double>(1)+44);
            waitingPos.append(object_position.at<double>(2)+20);
            waitingPos.append(-180.);
            waitingPos.append(0.);
            waitingPos.append(yaw_angle);
            //qDebug() << "Yellow";

            // set variable to teach pendant
            yrc_com->setRobotPositionVariable(2, waitingPos);
            // yrc_com->setRobotPositionVariable(4, placeUpPos);
            yrc_com->setRobotPositionVariable(5, placePos);
            yrc_com->setByteVaribale(3, 1);
            // qDebug() << " distance moving: " << distance_moving;
            // qDebug() << " v3: " << val_d3;

            if(object_min_distance <18){
               // qDebug() << "18";
               uart->writeToSTM32("S GAP : 400 120 78 \n");
            }else if (object_min_distance <22.5){
                uart->writeToSTM32("S GAP : 400 120 82 \n");
               // qDebug() << "22.5";
            }else if (object_min_distance<32){
                //qDebug() << "32";
                uart->writeToSTM32("S LON : 400 120 87 \n");
            } else{
               // qDebug() << "else";
                uart->writeToSTM32("S LON : 400 120 105 \n");
            }
            is_tracking = false;
            waiting_to_place = true;

        }

    }
    if(waiting_to_place){
        //read byte 10
        yrc_com->readByteVariable(10, REQUEST_ID_READ_BYTE_10);
        // if byte 10 == 1, robot has came place postion, open gripper to place object
        if(byte_10 == 1){
            byte_10 = 0;
            waiting_to_place = false;
            uart->writeToSTM32("S PWM : 120 0 0 \n");
            yrc_com->setByteVaribale(4, 1);
            yrc_com->setByteVaribale(10, 0);
            waiting_for_complete = true;

        }
    }

    if(waiting_for_complete){
        //read byte 9
        yrc_com->readByteVariable(9, REQUEST_ID_READ_BYTE_9);
        // if byte 9 == 1, robot has came ready postion, camera starts detecting obejct and robot ready to pick object
        if(byte_9 == 1){
            byte_9 = 0;
            yrc_com->setByteVaribale(9, 0);
            waiting_for_complete = false;
            rsCapture->setIs_object_detected(false);
            system_obejct_detect = false;
            rsCapture->setMotionDetect(true);
             //uart->writeToSTM32("S PWM : 105 0 0 \n");
        }
    }


}

void MainWindow::updateObejctInfoGUI()
{
    ui->label_object_min_distance->setText(QString::number(object_min_distance));
    ui->label_object_height->setText(QString::number(object_height));
    ui->label_yaw_angle->setText(QString::number(yaw_angle));
}

void MainWindow::updateSystemStatus()
{
    servo_on = yrc_com->getServo_status();
    is_running = yrc_com->getRunning_status();
    is_teaching = yrc_com->getIs_teaching();
    is_playback = yrc_com->getIs_playback();
    is_remote = yrc_com->getIs_remote();
    if(servo_on){
        ui->label_servo_status->setText("ON");
    } else {ui->label_servo_status->setText("OFF");
    ui->btn_servo->setText("SERVO ON");}

    if(is_running){
        ui->label_robot_status->setText("RUNNING");
    } else{ui->label_robot_status->setText("STOPPING");}
    if(is_teaching){
        ui->label_mode->setText("TEACHING");
    }else if(is_playback){
        ui->label_mode->setText("PLAY BACK");
    } else if(is_remote) {
        ui->label_mode->setText("REMOTE COMMAND");
    }


}

void MainWindow::readSystemStatus()
{
    yrc_com->readStatusInformation();
}

void MainWindow::setWaitingPosition(double pos0, double pos1, double pos2, double pos3, double pos4, double pos5)
{
    waitingPos.clear();
    waitingPos.append(pos0);
    waitingPos.append(pos1);
    waitingPos.append(pos2);
    waitingPos.append(pos3);
    waitingPos.append(pos4);
    waitingPos.append(pos5);

    yrc_com->setRobotPositionVariable(2, waitingPos);

}

void MainWindow::setPickingPosition(double pos0, double pos1, double pos2, double pos3, double pos4, double pos5)
{
    pickPos.clear();
    pickPos.append(pos0);
    pickPos.append(pos1);
    pickPos.append(pos2);
    pickPos.append(pos3);
    pickPos.append(pos4);
    pickPos.append(pos5);

    yrc_com->setRobotPositionVariable(3, pickPos);

}

//////////////////// START JOB ////////////////////
void MainWindow::on_btn_start_job_clicked()
{
    if(job_name=="RUNAUTO"){

        if(servo_on)
        {
            yrc_com->setByteVaribale(0, 1);
            yrc_com->startUp();
            systemTimer->start();
            ui->txt_message->setPlainText("JOB START");
            uart->writeToSTM32("S PWM : 120 0 0 \n");

        }else{
        ui->txt_message->setPlainText("SERVO IS OFF, TURN SERVO ON TO START JOB");
        }

    }

;
}
void MainWindow::on_btn_load_job_clicked()
{
    if(servo_on){
    job_name = ui->txt_job_name->text();
    yrc_com->selectJob(job_name);
    yrc_com->displayCommand(job_name);
    if(job_name == "RUNAUTO"){

        fsDataHeight.clear();
        fsDataWidth.clear();
        numObjectDetect = 0;
        numRedObject = 0;
        numBlueObject = 0;
        numYellowObject = 0;

        numRectangleObject =0;
        numTriangleObject = 0;
        numCircleObject = 0;

        ui->label_total_object->setText("0");
        ui->label_yellow_object->setText("0");
        ui->label_blue_object->setText("0");
        ui->label_red_object->setText("0");
        ui->label_conveyor_rate->setText("0");
        ui->label_object_color->setText("Unknown");
        ui->label_object_shape->setText("Unknow");
        ui->label_yaw_angle->setText("0");

        yrc_com->setRobotPositionVariable(0, homePos);
        yrc_com->setRobotPositionVariable(1, readyPos);
        yrc_com->setRobotPositionVariable(6, pickUpPos);

        yrc_com->setByteVaribale(1, 1);
        yrc_com->setByteVaribale(2, 0);
        yrc_com->setByteVaribale(3, 0);
        yrc_com->setByteVaribale(4, 0);
        yrc_com->setByteVaribale(5, 0);
        yrc_com->setByteVaribale(9, 0);
        yrc_com->setByteVaribale(10, 0);
        yrc_com->setByteVaribale(0, 0);
        ui->txt_message->setPlainText("LOAD JOB " + job_name);
    }

    }else{
        ui->txt_message->setPlainText("SERVO IS OFF, TURN ON SERVO TO LOAD JOB");

    }
}
////////////////////////////////////////////////////////

void MainWindow::on_btn_connect_clicked()
{
    if(ui->btn_connect->text() == "Connect"){
        QHostAddress udp_address;
        quint16 udp_port;
        QString ip_string = ui->txt_ip->text();
        QStringList ip_list = ip_string.split(".");
        quint32 ip_int32 = (ip_list.at(0).toUInt() << 24) |
                           (ip_list.at(1).toUInt() << 16) | (ip_list.at(2).toUInt() << 8) |
                           ip_list.at(3).toUInt();
        udp_address.setAddress(ip_int32);
        udp_port = ui->txt_port->text().toUShort();

        yrc_com->setConnection(udp_address, udp_port);
        yrc_com->connectRobot();
        ui->btn_connect->setText("Disconnect");
        yrc_com->readStatusInformation(REQUEST_ID_READ_STATUS_SERVO);
        statusTimer->start();
    }else if(ui->btn_connect->text() =="Disconnect"){
        yrc_com->disconnectRobot();
        ui->btn_connect->setText("Connect");
    }
}


void MainWindow::on_btn_servo_clicked()
{
    if(ui->btn_servo->text()=="SERVO ON"){
        if(is_teaching){
        ui->txt_message->setPlainText("Change to Play or Remote Mode to turn on Servo");
        }

        yrc_com->servoOn();

    } else if (ui->btn_servo->text()=="SERVO OFF"){

        yrc_com->servoOff();

    }
}


void MainWindow::on_btn_go_home_clicked()
{
    QVector<double> posHome_deg;
    for(int i = 0; i < 6; i++){
        posHome_deg.append(0.);
    }
    yrc_com->moveRobotPulse(1, 0, 200, &posHome_deg);
}


void MainWindow::on_btn_get_postion_clicked()
{
    yrc_com->readRobotPosition();
}

void MainWindow::updateRobotPosionGUI()
{
    ui->txt_get_X->setText(QString::number(robot_position.at(0)));
    ui->txt_get_Y->setText(QString::number(robot_position.at(1)));
    ui->txt_get_Z->setText(QString::number(robot_position.at(2)));
    ui->txt_get_Roll->setText(QString::number(robot_position.at(3)));
    ui->txt_get_Pitch->setText(QString::number(robot_position.at(4)));
    ui->txt_get_Yaw->setText(QString::number(robot_position.at(5)));
}

void MainWindow::updateRobotPulseGUI()
{
    ui->txt_get_J1->setText(QString::number(robot_pulse.at(0)));
    ui->txt_get_J2->setText(QString::number(robot_pulse.at(1)));
    ui->txt_get_J3->setText(QString::number(robot_pulse.at(2)));
    ui->txt_get_J4->setText(QString::number(robot_pulse.at(3)));
    ui->txt_get_J5->setText(QString::number(robot_pulse.at(4)));
    ui->txt_get_J6->setText(QString::number(robot_pulse.at(5)));

}

void MainWindow::displayImage(Mat frame)
{
    //cv::resize(frame, frame, Size(640, 480), 1, 1);
    convertToQtPixmap(frame, displayPix);
    ui->imgLabel->setPixmap(displayPix);
}

void MainWindow::convertToQtPixmap(const Mat& imgMat, QPixmap &pixmap)
{
    QImage::Format qFormat = QImage::Format_Indexed8;
    if (imgMat.type() == CV_8UC3){
        qFormat = QImage::Format_RGB888;
    } else if (imgMat.type() == CV_8UC4){
        qFormat = QImage::Format_RGBA8888;
    }

    QImage imgQt = QImage(imgMat.data, imgMat.cols, imgMat.rows, imgMat.step, qFormat);
    imgQt.rgbSwap();

    pixmap = QPixmap::fromImage(imgQt);
}


///////////////////////// UDP DATA UPDATE////////////////////
void MainWindow::updateUdpData(quint8 requset_id)
{
    switch (requset_id) {
    case REQUEST_ID_READ_ROBOT_POSITION:
        robot_position = yrc_com->updateRobotPostion();
        updateRobotPosionGUI();
        break;
    case REQUEST_ID_READ_ROBOT_PULSE:
        robot_pulse = yrc_com->updateRobotPulse();
        updateRobotPulseGUI();
        break;

    case REQUEST_ID_READ_BYTE_3:
        byte_3 = yrc_com->updateByteData();
        break;

    case REQUEST_ID_READ_BYTE_10:
        byte_10 = yrc_com->updateByteData();
        break;
    case REQUEST_ID_READ_BYTE_9:
        byte_9 = yrc_com->updateByteData();
        break;
    case REQUEST_ID_READ_STATUS_SERVO:
        updateStatusServo();
        break;
    case REQUEST_ID_READ_STATUS_INFORMATION:
        updateSystemStatus();
        break;
    default:
        break;
    }

}

void MainWindow::updateRealSenseCapture()
{


    switch(type_display){
    case 0:
        cv::resize(rsCapture->getColorImg(), display, Size(480, 360), 1, 1 );
        displayImage(display);

        break;
    case 1:
        cv::resize(rsCapture->getDepthImg(), display, Size(480, 360), 1, 1 );
        displayImage(display);
        break;
    case 2:
        cv::resize(rsCapture->getMask(), display, Size(480, 360), 1, 1 );
        displayImage(display);
        break;
    default:
        break;
    }
}



void MainWindow::updateObjectDetected()
{
    if(system_obejct_detect==false){
        object_detected = true;
        objectDetectImg = rsCapture->getObject_detect_image();
        convertToQtPixmap(objectDetectImg, displayObject);
        ui->imgLabelObject->setPixmap(displayObject);
    }

}

void MainWindow::updateBackgroundStatus()
{
    ui->txt_message->setPlainText("GET BACKGROUND!");
}



void MainWindow::updateStatusServo()
{

    servo_on = yrc_com->getServo_status();
    if(servo_on){
        ui->btn_servo->setText("SERVO OFF");
        //ui->txt_message->setPlainText("SERVO ON");
    }else{
        ui->btn_servo->setText("SERVO ON");
       // ui->txt_message->setPlainText("SERVO OFF");
    }
}


void MainWindow::on_btn_get_joint_clicked()
{
    yrc_com->readRobotPulse();
}


void MainWindow::on_btn_set_postion_clicked()
{
    // Get position to move
    QVector<double> move_pos;
    move_pos.append(ui->txt_setX->text().toDouble());
    move_pos.append(ui->txt_setY->text().toDouble());
    move_pos.append(ui->txt_setZ->text().toDouble());
    move_pos.append(ui->txt_setRoll->text().toDouble());
    move_pos.append(ui->txt_setPitch->text().toDouble());
    move_pos.append(ui->txt_setYaw->text().toDouble());
    // get speed
    double speed = ui->txt_setSpeed->text().toDouble();
    if(ui->rbtn_movj->isChecked()){
        yrc_com->moveRobotCartesian(17, 1, 0, speed*10, &move_pos);
    }
    else if(ui->rbtn_movl->isChecked()){
        yrc_com->moveRobotCartesian(17, 2, 1, 10*speed, &move_pos);
    }
    else if(ui->rbtn_movi->isChecked()){
        yrc_com->moveRobotCartesian(17, 3, 1, 10*speed, &move_pos);
    }
}


void MainWindow::on_rbtn_movj_clicked()
{
    if(ui->rbtn_movj->isChecked()){
        ui->label_speed_unit->setText("%");
    }
}


void MainWindow::on_rbtn_movl_clicked()
{
    if(ui->rbtn_movl->isChecked()){
        ui->label_speed_unit->setText("mm/s");
    }
}


void MainWindow::on_rbtn_movi_clicked()
{
    if(ui->rbtn_movi->isChecked()){
        ui->label_speed_unit->setText("mm/s");
        ui->txt_setX->setText(QString::number(0));
        ui->txt_setY->setText(QString::number(0));
        ui->txt_setZ->setText(QString::number(0));
        ui->txt_setRoll->setText(QString::number(0));
        ui->txt_setPitch->setText(QString::number(0));
        ui->txt_setYaw->setText(QString::number(0));
        ui->txt_setSpeed->setText(QString::number(5));
    }
}


void MainWindow::on_btn_open_camera_clicked()
{
    if(ui->btn_open_camera->text() == "OPEN CAMERA"){
        rsCapture->setCameraOpen(true);
        type_display = 0;
        realSenseThread.start();
    ui->btn_open_camera->setText("CLOSE CAMERA");
    }
    else if(ui->btn_open_camera->text() == "CLOSE CAMERA"){
        rsCapture->setCameraOpen(false);
        ui->btn_open_camera->setText("OPEN CAMERA");
    }
}



void MainWindow::on_btn_stop_job_clicked()
{
    systemTimer->stop();
    yrc_com->setByteVaribale(0, 0);
    QVector<double> posHome_deg;
    for(int i = 0; i < 6; i++){
        posHome_deg.append(0.);
    }
    //uart->writeToSTM32("S PWM : 100 \n");
    yrc_com->setByteVaribale(5, 1);
    //yrc_com->moveRobotPulse(1, 0, 100, &posHome_deg);

}

void MainWindow::on_btn_send_clicked()
{
    char data[20];
    sprintf(data, "S PWM : %d 0 \n", ui->txt_value->text().toInt());
    uart->writeToSTM32(data);
}


void MainWindow::on_btn_depth_map_clicked()
{
    type_display = 1;

}

void MainWindow::on_btn_mask_clicked()
{
    type_display = 2;
}

void MainWindow::updateColorLabel(const ObjectColor objectColor)
{
    switch (objectColor)
    {
    case ObjectColor::YELLOW:
        ++numYellowObject;
        ui->label_object_color->setText("YELLOW");
        ui->label_yellow_object->setText(QString::number(numYellowObject));
        break;
    case ObjectColor::BLUE:
        ++numBlueObject;
        ui->label_object_color->setText("BLUE");
        ui->label_blue_object->setText(QString::number(numBlueObject));


        break;
    case ObjectColor::RED:
        ++numRedObject;
        ui->label_object_color->setText("RED");
        ui->label_red_object->setText(QString::number(numRedObject));
        break;
    default:
        break;
    }
}

void MainWindow::updateShapeLabel(const ObjectShape objectShape)
{
    switch (objectShape)
    {
    case ObjectShape::RECTANGLE:
        ++numRectangleObject;
        ui->label_object_shape->setText("RECTANGLE");
        break;
    case ObjectShape::CIRCLE:
        ++numCircleObject;
        ui->label_object_shape->setText("CIRCLE");
        break;
    case ObjectShape::TRIANGLE:
        ++numTriangleObject;
        ui->label_object_shape->setText("TRIANGLE");
        break;
    default:
        break;
    }
}

void MainWindow::on_btn_depth_map_2_clicked()
{
    type_display = 0;

}

void MainWindow::on_btn_save_data_clicked()
{
    string file_name = ui->txt_data_file_name->text().toStdString();
    fsDataFile.open(file_name, FileStorage::WRITE);
    fsDataFile <<"height" << fsDataHeight;
    fsDataFile <<"width" <<fsDataWidth;
    fsDataFile.release();
}


void MainWindow::on_btn_inc_X_clicked()
{
    QVector<double> move_pos;
    move_pos.append(ui->txt_inc_distance->text().toDouble());
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    yrc_com->moveRobotCartesian(17, 3, 1, 10*ui->txt_inc_speed->text().toDouble(), &move_pos);

}


void MainWindow::on_btn_dec_X_clicked()
{
    QVector<double> move_pos;
    move_pos.append(-ui->txt_inc_distance->text().toDouble());
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    yrc_com->moveRobotCartesian(17, 3, 1, 10*ui->txt_inc_speed->text().toDouble(), &move_pos);
}


void MainWindow::on_btn_inc_Y_clicked()
{
    QVector<double> move_pos;
    move_pos.append(0);
    move_pos.append(ui->txt_inc_distance->text().toDouble());
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    yrc_com->moveRobotCartesian(17, 3, 1, 10*ui->txt_inc_speed->text().toDouble(), &move_pos);
}


void MainWindow::on_btn_dec_Y_clicked()
{
    QVector<double> move_pos;
    move_pos.append(0);
    move_pos.append(-ui->txt_inc_distance->text().toDouble());
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    yrc_com->moveRobotCartesian(17, 3, 1, 10*ui->txt_inc_speed->text().toDouble(), &move_pos);
}


void MainWindow::on_btn_inc_Z_clicked()
{
    QVector<double> move_pos;
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(ui->txt_inc_distance->text().toDouble());
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    yrc_com->moveRobotCartesian(17, 3, 1, 10*ui->txt_inc_speed->text().toDouble(), &move_pos);
}


void MainWindow::on_btn_dec_Z_clicked()
{
    QVector<double> move_pos;
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(-ui->txt_inc_distance->text().toDouble());
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    yrc_com->moveRobotCartesian(17, 3, 1, 10*ui->txt_inc_speed->text().toDouble(), &move_pos);
}


void MainWindow::on_btn_inc_Roll_clicked()
{
    QVector<double> move_pos;
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(ui->txt_inc_distance->text().toDouble());
    move_pos.append(0);
    move_pos.append(0);
    yrc_com->moveRobotCartesian(17, 3, 1, 10*ui->txt_inc_speed->text().toDouble(), &move_pos);
}


void MainWindow::on_btn_dec_Roll_clicked()
{
    QVector<double> move_pos;
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(-ui->txt_inc_distance->text().toDouble());
    move_pos.append(0);
    move_pos.append(0);
    yrc_com->moveRobotCartesian(17, 3, 1, 10*ui->txt_inc_speed->text().toDouble(), &move_pos);
}


void MainWindow::on_btn_inc_Pitch_clicked()
{
    QVector<double> move_pos;
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(ui->txt_inc_distance->text().toDouble());
    move_pos.append(0);
    yrc_com->moveRobotCartesian(17, 3, 1, 10*ui->txt_inc_speed->text().toDouble(), &move_pos);
}


void MainWindow::on_btn_dec_Pitch_clicked()
{
    QVector<double> move_pos;
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(-ui->txt_inc_distance->text().toDouble());
    move_pos.append(0);
    yrc_com->moveRobotCartesian(17, 3, 1, 10*ui->txt_inc_speed->text().toDouble(), &move_pos);
}


void MainWindow::on_btn_inc_Yaw_clicked()
{
    QVector<double> move_pos;
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(ui->txt_inc_distance->text().toDouble());
    yrc_com->moveRobotCartesian(17, 3, 1, 10*ui->txt_inc_speed->text().toDouble(), &move_pos);
}


void MainWindow::on_btn_dec_Yaw_clicked()
{
    QVector<double> move_pos;
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(0);
    move_pos.append(-ui->txt_inc_distance->text().toDouble());
    yrc_com->moveRobotCartesian(17, 3, 1, 10*ui->txt_inc_speed->text().toDouble(), &move_pos);
}

