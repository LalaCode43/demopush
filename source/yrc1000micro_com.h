#ifndef YRC1000MICRO_COM_H
#define YRC1000MICRO_COM_H

#include <QObject>
#include "yrc1000micro_command.h"
#include "udp.h"
#include <QThread>
class YRC1000micro_com : public QObject
{
    Q_OBJECT
    QThread udpThread;
public:
    explicit YRC1000micro_com(QObject *parent = nullptr);
    // ~YRC1000micro_com();

    // Create communication with Robot
    void setConnection(QHostAddress address, quint16 port);
    bool connectRobot();
    void disconnectRobot();

    ///////////////////////// Turn ON/OFF servo /////////////////////////
    void servoOn();
    void servoOff();

    ///////////////////////// Set/read multiple Types Variable /////////////////////////
    //Byte type Variable
    void setByteVaribale(quint8 byte_number, quint8 byte_value, quint8 request_id = REQUEST_ID_WRITE_BYTE);
    void readByteVariable(quint8 byte_number, quint8 request_id = REQUEST_ID_READ_BYTE);
    // Register type Variable
    void setRegisterVaribale(quint8 register_number, quint16 register_value);
    // Interger type variable
    void setIntegerVaribale(quint8 int_number, qint16 int_value);
    //Double percesion type variable
    void setDoubleVaribale(quint8 double_number, qint32 double_value);
    void setRealVaribale(quint8 real_number, float real_value);
    // String 16 characters type Variable
    void setString16Variable(quint8 string_number, QString string_value);
    // Robot Position (Cartesian) type Variable
    void setRobotPositionVariable(quint8 pos_number, QVector<double> pos);
    void readRobotVariable(quint8 pos_number);

    //void readRobotPositionVariable(quint8 pos_number);


    void readStatusInformation(quint8 request_id = REQUEST_ID_READ_STATUS_INFORMATION);
    void readRobotPosition();
    void readRobotPulse();
    /////////////////////// Read and Process response data ///////////////////////
    void readRobotVariableResponse(QByteArray data);
    void readRobotPositionResponse(QByteArray data);
    void readRobotStatusResponse(QByteArray data);
    void readRobotStatusServoResponse(QByteArray data);
    void readRobotPulseResponse(QByteArray data);
    void readByteVariableResponse(QByteArray data);

    void displayCommand(QString command);
    void selectJob(QString job_name);
    void startUp();

    ////////////////////// Moving Robot instruction /////////////////////
    // Moving type cartersian coordiante
    void moveRobotCartesian(quint8 coordinate, quint8 move_type, quint8 speed_type, double speed, QVector<double>* posiotion);
    // Moving type pulse
    void moveRobotPulse(quint8 move_type, quint8 speed_type, double speed, QVector<double>* posiotion);

    /////////// return robot status, robot, varibale /////////////
    QVector<double> updateRobotPostion();
    QVector<double> updateRobotPulse();    
    QVector<double> updateRobotVariable();
    quint8 updateByteData();

    bool getServo_status() const;
    void setRunning_status(bool newRunning_status);

    bool getRunning_status() const;

    bool getIs_teaching() const;

    bool getIs_playback() const;

    bool getIs_remote() const;

signals:
    void udpDataReceiveUi(quint8);

public slots:
    void udpDataReceiveCallBack(QByteArray);

private:
    UDP *udp_server;
    QHostAddress udp_address;
    quint16 udp_port;
    YRC1000micro_command yrc_command;
    bool servo_status, running_status;
    bool is_teaching, is_playback,is_remote;
    QVector<double> robot_variable;
    QVector<double> robot_position;
    QVector<double> robot_pulse;
    quint8 byteData;
};

#endif // YRC1000MICRO_COM_H
