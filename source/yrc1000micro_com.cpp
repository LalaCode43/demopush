#include "yrc1000micro_com.h"

YRC1000micro_com::YRC1000micro_com(QObject *parent)
    : QObject{parent}
{
    udp_server = new UDP();
    connect(udp_server, &UDP::udpDataReceive,this,&YRC1000micro_com::udpDataReceiveCallBack);

    // Robot variable and Robot position
    for(int i = 0; i < 6; i++){
        robot_position.append(0.);
        robot_variable.append(0.);
        robot_pulse.append(0.);
    }

}

void YRC1000micro_com::setConnection(QHostAddress address, quint16 port){
    udp_address = address;
    udp_port = port;
}

bool YRC1000micro_com::connectRobot(){
    bool status = udp_server->udpConnect(udp_address, udp_port);
    return status;
}
void YRC1000micro_com::disconnectRobot(){
    udp_server->udpDisconnect();
}


void YRC1000micro_com::udpDataReceiveCallBack(QByteArray data)
{
   // QByteArray data = udp_server->getUdpData();
    //qDebug() << "Data Receive: " << data;
    quint8 request_id = (quint8)data[CMD_REQUEST_ID];
    //qDebug() << "Request id: " << request_id;


    switch (request_id) {
    case REQUEST_ID_READ_ROBOT_POSITION:
        readRobotPositionResponse(data);
        break;
    case REQUEST_ID_READ_ROBOT_VARIABLE:
        readRobotVariableResponse(data);
        break;
    case REQUEST_ID_READ_STATUS_INFORMATION:
        readRobotStatusResponse(data);
        break;
    case REQUEST_ID_READ_ROBOT_PULSE:
        readRobotPulseResponse(data);
        break;
    case REQUEST_ID_READ_BYTE:
        readByteVariableResponse(data);
        break;
    case REQUEST_ID_READ_BYTE_3:
        readByteVariableResponse(data);
        break;
    case REQUEST_ID_READ_BYTE_4:
        readByteVariableResponse(data);
        break;
    case REQUEST_ID_READ_BYTE_9:
        readByteVariableResponse(data);
        break;
    case REQUEST_ID_READ_BYTE_10:
        readByteVariableResponse(data);
        break;
    case REQUEST_ID_READ_BYTE_5:
        readByteVariableResponse(data);
        break;
    case REQUEST_ID_SERVO_ON:
        readStatusInformation(REQUEST_ID_READ_STATUS_SERVO);
        break;
    case REQUEST_ID_SERVO_OFF:
        readStatusInformation(REQUEST_ID_READ_STATUS_SERVO);
        break;
    case REQUEST_ID_READ_STATUS_SERVO:
        readRobotStatusServoResponse(data);
        break;
    default:
        break;

    }

    emit udpDataReceiveUi(request_id);
}

bool YRC1000micro_com::getIs_remote() const
{
    return is_remote;
}

bool YRC1000micro_com::getIs_playback() const
{
    return is_playback;
}

bool YRC1000micro_com::getIs_teaching() const
{
    return is_teaching;
}

bool YRC1000micro_com::getRunning_status() const
{
    return running_status;
}

void YRC1000micro_com::setRunning_status(bool newRunning_status)
{
    running_status = newRunning_status;
}

bool YRC1000micro_com::getServo_status() const
{
    return servo_status;
}


void YRC1000micro_com::servoOn(){
    QByteArray cmd = yrc_command.setServoOn();
    udp_server->udpSendata(cmd, udp_address, udp_port);

}


void YRC1000micro_com::servoOff(){
    QByteArray cmd = yrc_command.setServoOff();
    udp_server->udpSendata(cmd, udp_address, udp_port);

}
void YRC1000micro_com::setByteVaribale(quint8 byte_number, quint8 byte_value, quint8 request_id){
    QByteArray cmd = yrc_command.cmdSetByteVaribale(byte_number, byte_value, request_id);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::readByteVariable(quint8 byte_number, quint8 request_id)
{
    QByteArray cmd = yrc_command.cmdReadByteVaribale(byte_number, request_id);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::setRegisterVaribale(quint8 register_number, quint16 register_value){
    QByteArray cmd = yrc_command.cmdSetRegisterVaribale(register_number, register_value);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::setIntegerVaribale(quint8 int_number, qint16 int_value)
{
    QByteArray cmd = yrc_command.cmdSetIntegerVaribale(int_number, int_value);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::setDoubleVaribale(quint8 double_number, qint32 double_value)
{
    QByteArray cmd = yrc_command.cmdSetDoubleVariable(double_number, double_value);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::setRealVaribale(quint8 real_number, float real_value)
{
    QByteArray cmd = yrc_command.cmdSetRealVariable(real_number, real_value);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::setString16Variable(quint8 string_number, QString string_value)
{
    QByteArray cmd = yrc_command.cmdSetString16Variable(string_number, string_value);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}
void YRC1000micro_com::setRobotPositionVariable(quint8 pos_number, QVector<double> pos){
    QByteArray cmd = yrc_command.cmdSetRobotVariable(pos_number, pos);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::readStatusInformation(quint8 request_id)
{
    QByteArray cmd = yrc_command.cmdReadStatusInformation(request_id);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}


void YRC1000micro_com::readRobotVariableResponse(QByteArray data){
    qint32 x;
    qint32 y;
    qint32 z;
    qint32 roll;
    qint32 pitch;
    qint32 yaw;
    qDebug() << "run";
    x = (quint8)(quint32)(data[HEADER_SIZE+ 20 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 20 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 20 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 20]);

    y = (quint8)(quint32)(data[HEADER_SIZE+ 24 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 24 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 24 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 24]);

    z = (quint8)(quint32)(data[HEADER_SIZE+ 28 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 28 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 28 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 28]);

    roll = (quint8)(quint32)(data[HEADER_SIZE+ 32 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 32 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 32 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 32]);

    pitch = (quint8)(quint32)(data[HEADER_SIZE+ 36 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 36 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 36 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 36]);

    yaw = (quint8)(quint32)(data[HEADER_SIZE+ 40 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 40 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 40 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 40]);


    robot_variable[0] = x/1000.;
    robot_variable[1] = y/1000.;
    robot_variable[2] = z/1000.;
    robot_variable[3] = roll/10000.;
    robot_variable[4] = pitch/10000.;
    robot_variable[5] = yaw/10000.;
    qDebug() << robot_variable;
}
void YRC1000micro_com::readRobotPositionResponse(QByteArray data){
    qint32 x;
    qint32 y;
    qint32 z;
    qint32 roll;
    qint32 pitch;
    qint32 yaw;

    x = (quint8)(quint32)(data[HEADER_SIZE+ 20 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 20 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 20 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 20]);

    y = (quint8)(quint32)(data[HEADER_SIZE+ 24 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 24 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 24 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 24]);

    z = (quint8)(quint32)(data[HEADER_SIZE+ 28 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 28 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 28 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 28]);

    roll = (quint8)(quint32)(data[HEADER_SIZE+ 32 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 32 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 32 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 32]);

    pitch = (quint8)(quint32)(data[HEADER_SIZE+ 36 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 36 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 36 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 36]);

    yaw = (quint8)(quint32)(data[HEADER_SIZE+ 40 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 40 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 40 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 40]);


    robot_position[0] = x/1000.;
    robot_position[1] = y/1000.;
    robot_position[2] = z/1000.;
    robot_position[3] = roll/10000.;
    robot_position[4] = pitch/10000.;
    robot_position[5] = yaw/10000.;
}

void YRC1000micro_com::readRobotPulseResponse(QByteArray data){
    qint32 x;
    qint32 y;
    qint32 z;
    qint32 roll;
    qint32 pitch;
    qint32 yaw;

    x = (quint8)(quint32)(data[HEADER_SIZE+ 20 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 20 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 20 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 20]);

    y = (quint8)(quint32)(data[HEADER_SIZE+ 24 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 24 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 24 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 24]);

    z = (quint8)(quint32)(data[HEADER_SIZE+ 28 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 28 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 28 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 28]);

    roll = (quint8)(quint32)(data[HEADER_SIZE+ 32 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 32 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 32 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 32]);

    pitch = (quint8)(quint32)(data[HEADER_SIZE+ 36 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 36 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 36 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 36]);

    yaw = (quint8)(quint32)(data[HEADER_SIZE+ 40 +3]) << 24 | (quint8)(quint32)(data[HEADER_SIZE+ 40 +2]) << 16 |(quint8)(quint32)(data[HEADER_SIZE+ 40 +1]) << 8 |(quint8)(quint32)(data[HEADER_SIZE+ 40]);


    robot_pulse[0] = x * 30. / 34816;
    robot_pulse[1] = y * 90. / 102400;
    robot_pulse[2] = z * 90. / 51200;
    robot_pulse[3] = roll * 30. / 10204;
    robot_pulse[4] = pitch * 30. / 10204;
    robot_pulse[5] = yaw * 30. / 10204;
    qDebug() <<robot_pulse;
}

void YRC1000micro_com::readByteVariableResponse(QByteArray data)
{
    byteData = (quint8)data[CMD_DATA_BYTE0];
}

void YRC1000micro_com::readRobotStatusResponse(QByteArray data)
{
    quint8 data1 = (quint8)data[32];
    quint8 data2 = (quint8)data[36];
    //qDebug()<<"Data 1: "<<data1;
    //qDebug()<<"Data 2: "<<data2;
    servo_status = data2 >> 6 & 1;
    running_status = data1 >> 3 & 1;
    is_teaching = data1 >> 5 & 1;
    is_playback = data1 >> 6 & 1;
    is_remote = data1 >> 7 & 1;

}

void YRC1000micro_com::readRobotStatusServoResponse(QByteArray data)
{
    quint8 data2 = (quint8)data[36];
    servo_status = data2>>6 & 1;

}

void YRC1000micro_com::readRobotVariable(quint8 pos_number)
{
    QByteArray cmd = yrc_command.cmdReadRobotVariable(pos_number);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::readRobotPosition(){
    QByteArray cmd = yrc_command.cmdReadRobotPostion();
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::readRobotPulse()
{
    QByteArray cmd = yrc_command.cmdReadRobotPulse();
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::displayCommand(QString command)
{
    QByteArray cmd = yrc_command.cmdDisplayCommand(command);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::selectJob(QString job_name){
    QByteArray cmd = yrc_command.cmdSelectJob(job_name);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::startUp()
{
    QByteArray cmd = yrc_command.cmdStartUp();
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::moveRobotCartesian(quint8 coordinate, quint8 move_type, quint8 speed_type, double speed, QVector<double> *posiotion)
{
    QByteArray cmd = yrc_command.cmdMoveRobotCartesian(coordinate, move_type, speed_type, speed, posiotion);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

void YRC1000micro_com::moveRobotPulse(quint8 move_type, quint8 speed_type, double speed, QVector<double> *posiotion)
{
    QByteArray cmd = yrc_command.cmdMoveRobotPulse(move_type, speed_type, speed, posiotion);
    udp_server->udpSendata(cmd, udp_address, udp_port);
}

QVector<double> YRC1000micro_com::updateRobotPostion()
{
    return robot_position;
}

QVector<double> YRC1000micro_com::updateRobotPulse()
{
    return robot_pulse;
}

quint8 YRC1000micro_com::updateByteData(){
    return byteData;
}
