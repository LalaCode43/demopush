#include "yrc1000micro_command.h"

YRC1000micro_command::YRC1000micro_command(QObject *parent)
    : QObject{parent}
{
    // Data to send 4 bytes
    for(int i = 0; i < 4; i++){
        data_to_send.append('\x00');
        line_number.append('\x00');
    }
    data_one_byte.append('\x00');
    data_two_bytes.append('\x00');
    data_two_bytes.append('\x00');
    // Header to send 32 bytes
    const char header_char[] = {
        '\x59', '\x45', '\x52', '\x43',
        '\x20', '\x00', '\x00', '\x00',
        '\x03', '\x01', '\x00', '\00',
        '\x00', '\x00', '\x00', '\x00',
        '\x39', '\x39', '\x39', '\x39',
        '\x39', '\x39', '\x39', '\x39',
        '\x83', '\x00', '\x02', '\x00',
        '\x01', '\x10', '\x00', '\x00'
    };

    for(int i = 0; i < 32; i++){
        header_to_send.append(header_char[i]);
    }

    // Robot position 52 bytes
    for(int i = 0; i < 52; i++){
        robot_postion.append('\x00');
    }
    for(int i = 0; i < 16; i++){
        string_data_16.append('\x00');
    }
    for (int i = 0; i<32; i++){
        string_data.append('\x00');
    }
    // init Move data cartesian
    for (int i = 0; i < 104; ++i) {
        move_cartesian_data.append('\x00');
    }
    move_cartesian_data[0] = '\x01';
    move_cartesian_data[64] = '\x01';

    // init Move data cartesian
    for (int i = 0; i < 88; ++i) {
        move_pulse_data.append('\x00');
    }
    move_pulse_data[0] = '\x01';

}
QByteArray YRC1000micro_command::setServoOn(){
    QByteArray cmd = header_to_send + data_to_send;
    cmd[CMD_DATA_SIZE] = '\x04';
    cmd[CMD_REQUEST_ID] = REQUEST_ID_SERVO_ON;
    cmd[CMD_ADDRESS_ID] = '\x83';
    cmd[CMD_INSTANCE] = 2;
    cmd[CMD_ATTRIBUTE] = 1;
    cmd[CMD_SERVICE] = '\x10';
    cmd[CMD_DATA_BYTE0] = '\x01';
    return cmd;
}
QByteArray YRC1000micro_command::setServoOff(){
    QByteArray cmd = header_to_send + data_to_send;
    cmd[CMD_DATA_SIZE] = '\x04';
    cmd[CMD_REQUEST_ID] = REQUEST_ID_SERVO_OFF;
    cmd[CMD_ADDRESS_ID] = '\x83';
    cmd[CMD_INSTANCE] = '\x02';
    cmd[CMD_ATTRIBUTE] = '\x01';
    cmd[CMD_SERVICE] = '\x10';
    cmd[CMD_DATA_BYTE0] = '\x02';
    return cmd;
}

QByteArray YRC1000micro_command::cmdSetByteVaribale(quint8 byte_number, quint8 byte_value, quint8 request_id){
    QByteArray cmd = header_to_send + data_one_byte;
    cmd[CMD_DATA_SIZE] = 1;
    cmd[CMD_REQUEST_ID] = request_id;
    cmd[CMD_ADDRESS_ID] = '\x07A';
    cmd[CMD_INSTANCE] = byte_number;
    cmd[CMD_ATTRIBUTE] = 1;
    cmd[CMD_SERVICE] = '\x10';
    cmd[CMD_DATA_BYTE0] = byte_value;
    return cmd;
}

QByteArray YRC1000micro_command::cmdReadByteVaribale(quint8 byte_number, quint8 request_id)
{
    QByteArray cmd = header_to_send;
    cmd[CMD_DATA_SIZE] = 0;
    cmd[CMD_REQUEST_ID] = request_id;
    cmd[CMD_ADDRESS_ID] = '\x07A';
    cmd[CMD_INSTANCE] = byte_number;
    cmd[CMD_ATTRIBUTE] = 1;
    cmd[CMD_SERVICE] = '\x01';
    return cmd;
}
QByteArray YRC1000micro_command::cmdSetRegisterVaribale(quint8 register_number, quint16 register_value){
    QByteArray cmd = header_to_send + data_two_bytes;
    cmd[CMD_DATA_SIZE] = 2;
    cmd[CMD_REQUEST_ID] = REQUEST_ID_WRITE_REGISTER;
    cmd[CMD_ADDRESS_ID] = '\x079';
    cmd[CMD_INSTANCE] = register_number;
    cmd[CMD_ATTRIBUTE] = 1;
    cmd[CMD_SERVICE] = '\x10';

    cmd[CMD_DATA_BYTE0] = register_value;
    cmd[CMD_DATA_BYTE1] = register_value>>8;
    return cmd;
}
QByteArray YRC1000micro_command::cmdSetIntegerVaribale(quint8 int_number, qint16 int_value)
{
    QByteArray cmd = header_to_send + data_two_bytes;
    cmd[CMD_DATA_SIZE] = 2;
    cmd[CMD_REQUEST_ID] = REQUEST_ID_WRITE_INTEGER;
    cmd[CMD_ADDRESS_ID] = '\x7B';
    cmd[CMD_INSTANCE] = int_number;
    cmd[CMD_ATTRIBUTE] = 1;
    cmd[CMD_SERVICE] = '\x10';

    cmd[CMD_DATA_BYTE0] = int_value;
    cmd[CMD_DATA_BYTE1] = int_value>>8;
    return cmd;
}
QByteArray YRC1000micro_command::cmdSetDoubleVariable(quint8 double_number, qint32 double_value)
{
    QByteArray cmd = header_to_send + data_to_send;
    cmd[CMD_DATA_SIZE] = 4;
    cmd[CMD_REQUEST_ID] = REQUEST_ID_WRITE_DOUBLE;
    cmd[CMD_ADDRESS_ID] = '\x7C';
    cmd[CMD_INSTANCE] = double_number;
    cmd[CMD_ATTRIBUTE] = 1;
    cmd[CMD_SERVICE] = '\x10';

    cmd[CMD_DATA_BYTE0] = double_value;
    cmd[CMD_DATA_BYTE1] = double_value>>8;
    cmd[CMD_DATA_BYTE2] = double_value>>16;
    cmd[CMD_DATA_BYTE3] = double_value>>24;
    return cmd;
}

QByteArray YRC1000micro_command::cmdSetRealVariable(quint8 real_number, float real_value)
{
    QByteArray cmd = header_to_send + data_to_send;
    cmd[CMD_DATA_SIZE] = 4;
    cmd[CMD_REQUEST_ID] = REQUEST_ID_WRITE_REAL;
    cmd[CMD_ADDRESS_ID] = '\x7D';
    cmd[CMD_INSTANCE] = 0;
    cmd[CMD_ATTRIBUTE] = 1;
    cmd[CMD_SERVICE] = '\x02';
   // qDebug() << real_value;
    quint32 value = (quint32)(real_value*1000);
    cmd[CMD_DATA_BYTE0] = (quint8)(value);
    cmd[CMD_DATA_BYTE1] = (quint8)(value>>8);
    cmd[CMD_DATA_BYTE2] = (quint8)(value>>16);
    cmd[CMD_DATA_BYTE3] = (quint8)(value>>24);
    return cmd;
}

QByteArray YRC1000micro_command::cmdSetString16Variable(quint8 string_number, QString string_value)
{
    QByteArray cmd = header_to_send + string_data_16;
    cmd[CMD_DATA_SIZE] = 16;
    cmd[CMD_REQUEST_ID] = REQUEST_ID_WRITE_STRING;
    cmd[CMD_ADDRESS_ID] = '\x7E';
    cmd[CMD_INSTANCE] = string_number;
    cmd[CMD_ATTRIBUTE] = 1;
    cmd[CMD_SERVICE] = '\x02';
    //qDebug() << real_value;
    QByteArray string_byte = string_value.toUtf8();
    for(int i =0; i<string_byte.size(); i++){
        cmd[HEADER_SIZE + i] = string_byte[i];
    }
    return cmd;
}
QByteArray YRC1000micro_command::cmdSetRobotVariable(quint8 pos_number, QVector<double> pos){
    QByteArray cmd = header_to_send + robot_postion;
    cmd[CMD_DATA_SIZE] = '\x34';
    cmd[CMD_REQUEST_ID] = REQUEST_ID_WRITE_ROBOT_VARIABLE;
    cmd[CMD_ADDRESS_ID] = '\x07F';
    cmd[CMD_INSTANCE] = pos_number;
    cmd[CMD_ATTRIBUTE] = 0;
    cmd[CMD_SERVICE] = '\x02';
    cmd[32] = '\x11';
    cmd[36] = '\x04';

    qint32 x = (qint32)(pos.at(0)*1000);
    qint32 y = (qint32)(pos.at(1)*1000);
    qint32 z = (qint32)(pos.at(2)*1000);
    qint32 roll = (qint32)(pos.at(3)*10000);
    qint32 pitch = (qint32)(pos.at(4)*10000);
    qint32 yaw = (qint32)(pos.at(5)*10000);

    cmd[32+20 + 3] = (qint32)(x) >>24;
    cmd[32+20 + 2] = (qint32)(x) >>16;
    cmd[32+20 + 1] = (qint32)(x) >>8;
    cmd[32+20 ] = (qint32)(x);

    cmd[32+24 + 3] = (qint32)(y) >>24;
    cmd[32+24 + 2] = (qint32)(y) >>16;
    cmd[32+24 + 1] = (qint32)(y) >>8;
    cmd[32+24 ] = (qint32)(y);

    cmd[32+28 + 3] = (qint32)(z) >>24;
    cmd[32+28 + 2] = (qint32)(z) >>16;
    cmd[32+28 + 1] = (qint32)(z) >>8;
    cmd[32+28 ] = (qint32)(z);

    cmd[32+32 + 3] = (qint32)(roll) >>24;
    cmd[32+32 + 2] = (qint32)(roll) >>16;
    cmd[32+32 + 1] = (qint32)(roll) >>8;
    cmd[32+32 ] = (qint32)(roll);

    cmd[32+36 + 3] = (qint32)(pitch) >>24;
    cmd[32+36 + 2] = (qint32)(pitch) >>16;
    cmd[32+36 + 1] = (qint32)(pitch) >>8;
    cmd[32+36 ] = (qint32)(pitch);

    cmd[32+40 + 3] = (qint32)(yaw) >>24;
    cmd[32+40 + 2] = (qint32)(yaw) >>16;
    cmd[32+40 + 1] = (qint32)(yaw) >>8;
    cmd[32+40 ] = (qint32)(yaw);

    return cmd;
}

QByteArray YRC1000micro_command::cmdReadStatusInformation(quint8 request_id)
{
    QByteArray cmd = header_to_send;
    cmd[CMD_DATA_SIZE] = 0;
    cmd[CMD_REQUEST_ID] = request_id;
    cmd[CMD_ADDRESS_ID] = '\x72';
    cmd[CMD_INSTANCE] = 1;
    cmd[CMD_ATTRIBUTE] = 0;
    cmd[CMD_SERVICE] = '\x01';

    return cmd;
}
QByteArray YRC1000micro_command::cmdReadRobotVariable(quint8 pos_number){
    QByteArray cmd = header_to_send;

    cmd[CMD_DATA_SIZE] = 0;
    cmd[CMD_REQUEST_ID] = REQUEST_ID_READ_ROBOT_VARIABLE;
    cmd[CMD_ADDRESS_ID] = '\x07F';
    cmd[CMD_INSTANCE] = pos_number;
    cmd[CMD_ATTRIBUTE] = 0;
    cmd[CMD_SERVICE] = '\x01';

    return cmd;
}
QByteArray YRC1000micro_command::cmdReadRobotPostion(){
    QByteArray cmd = header_to_send;
    cmd[CMD_DATA_SIZE] = 0;
    cmd[CMD_REQUEST_ID] = REQUEST_ID_READ_ROBOT_POSITION;
    cmd[CMD_ADDRESS_ID] = '\x75';
    cmd[CMD_INSTANCE] = '\x65';
    cmd[CMD_ATTRIBUTE] = 0;
    cmd[CMD_SERVICE] = '\x01';
    return cmd;
}
QByteArray YRC1000micro_command::cmdReadRobotPulse(){
    QByteArray cmd = header_to_send;
    cmd[CMD_DATA_SIZE] = 0;
    cmd[CMD_REQUEST_ID] = REQUEST_ID_READ_ROBOT_PULSE;
    cmd[CMD_ADDRESS_ID] = '\x75';
    cmd[CMD_INSTANCE] = 1;
    cmd[CMD_ATTRIBUTE] = 0;
    cmd[CMD_SERVICE] = '\x01';
    return cmd;
}
QByteArray YRC1000micro_command::cmdDisplayCommand(QString command)
{
    QByteArray cmd = header_to_send + string_data;
    cmd[CMD_DATA_SIZE] = 32;
    cmd[CMD_REQUEST_ID] = REQUEST_ID_DISPLAY_COMMAND;
    cmd[CMD_ADDRESS_ID] = '\x85';
    cmd[CMD_INSTANCE] = 1;
    cmd[CMD_ATTRIBUTE] = 1;
    cmd[CMD_SERVICE] = '\x10';
    QByteArray stringByteArray = command.toUtf8();
    for(int i =0; i<stringByteArray.size() ;i++){
        cmd[HEADER_SIZE+i] = stringByteArray[i];
    }
    return cmd;

}
QByteArray YRC1000micro_command::cmdSelectJob(QString job_name){

    QByteArray cmd = header_to_send + string_data +line_number;
    cmd[CMD_DATA_SIZE] = 36;
    cmd[CMD_REQUEST_ID] = REQUEST_ID_SELECT_JOB;
    cmd[CMD_ADDRESS_ID] = '\x87';
    cmd[CMD_INSTANCE] = 1;
    cmd[CMD_ATTRIBUTE] = 0;
    cmd[CMD_SERVICE] = '\x02';
    QByteArray stringByteArray = job_name.toUtf8();
    for(int i =0; i<stringByteArray.size() ;i++){
        cmd[HEADER_SIZE+i] = stringByteArray[i];
    }
    return cmd;

}
#define REQUEST_ID_START_UP 23
QByteArray YRC1000micro_command::cmdStartUp()
{
    QByteArray cmd = header_to_send + data_to_send;
    cmd[CMD_DATA_SIZE] = 4;
    cmd[CMD_REQUEST_ID] = REQUEST_ID_START_UP;
    cmd[CMD_ADDRESS_ID] = '\x86';
    cmd[CMD_INSTANCE] = 1;
    cmd[CMD_ATTRIBUTE] = 1;
    cmd[CMD_SERVICE] = '\x10';
    cmd[CMD_DATA_BYTE0] = 1;
    return cmd;
}
QByteArray YRC1000micro_command::cmdMoveRobotCartesian(quint8 coordinate, quint8 move_type, quint8 speed_type, double speed, QVector<double> *position)
{
    QByteArray cmd = header_to_send + move_cartesian_data;
    cmd[CMD_DATA_SIZE] = 104;
    cmd[CMD_REQUEST_ID] = REQUEST_ID_MOVE_CARTESIAN;
    cmd[CMD_ADDRESS_ID] = '\x8A';
    cmd[CMD_INSTANCE] = move_type;
    cmd[CMD_ATTRIBUTE] = 1;
    cmd[CMD_SERVICE] = '\x02';
    if (speed < 0) speed = 0;
    quint32 speed_u = (quint32)(speed*10);
    quint32 x_u = (quint32)(position->at(0) * 1000);
    quint32 y_u = (quint32)(position->at(1) * 1000);
    quint32 z_u = (quint32)(position->at(2) * 1000);
    quint32 roll_u = (quint32)(position->at(3) * 10000);
    quint32 pitch_u = (quint32)(position->at(4) * 10000);
    quint32 yaw_u = (quint32)(position->at(5) * 10000);


    cmd[DATA_MOVE_SPEED_TYPE+HEADER_SIZE] = speed_type;
    cmd[DATA_MOVE_COORDINATE+HEADER_SIZE]= coordinate;

    cmd[DATA_MOVE_SPEED+HEADER_SIZE+3] = (quint8)(speed_u >>24);
    cmd[DATA_MOVE_SPEED+HEADER_SIZE+2] = (quint8)(speed_u >>16);
    cmd[DATA_MOVE_SPEED+HEADER_SIZE+1] = (quint8)(speed_u >>8);
    cmd[DATA_MOVE_SPEED+HEADER_SIZE] = (quint8)(speed_u);

    cmd[DATA_MOVE_X_CARTESIAN+HEADER_SIZE+3]= (quint8)(x_u>>24);
    cmd[DATA_MOVE_X_CARTESIAN+HEADER_SIZE+2]= (quint8)(x_u>>16);
    cmd[DATA_MOVE_X_CARTESIAN+HEADER_SIZE+1]= (quint8)(x_u>>8);
    cmd[DATA_MOVE_X_CARTESIAN+HEADER_SIZE]= (quint8)(x_u);

    cmd[DATA_MOVE_Y_CARTESIAN+HEADER_SIZE+3]= (quint8)(y_u>>24);
    cmd[DATA_MOVE_Y_CARTESIAN+HEADER_SIZE+2]= (quint8)(y_u>>16);
    cmd[DATA_MOVE_Y_CARTESIAN+HEADER_SIZE+1]= (quint8)(y_u>>8);
    cmd[DATA_MOVE_Y_CARTESIAN+HEADER_SIZE]= (quint8)(y_u);

    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE+3]= (quint8)(z_u>>24);
    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE+2]= (quint8)(z_u>>16);
    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE+1]= (quint8)(z_u>>8);
    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE]= (quint8)(z_u);

    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE+3]= (quint8)(roll_u>>24);
    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE+2]= (quint8)(roll_u>>16);
    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE+1]= (quint8)(roll_u>>8);
    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE]= (quint8)(roll_u);

    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE+3]= (quint8)(pitch_u>>24);
    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE+2]= (quint8)(pitch_u>>16);
    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE+1]= (quint8)(pitch_u>>8);
    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE]= (quint8)(pitch_u);

    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+3]= (quint8)(yaw_u>>24);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+2]= (quint8)(yaw_u>>16);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+1]= (quint8)(yaw_u>>8);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE]= (quint8)(yaw_u);

    return cmd;
}

QByteArray YRC1000micro_command::cmdMoveRobotPulse(quint8 move_type, quint8 speed_type, double speed, QVector<double> *position)
{
    QByteArray cmd = header_to_send + move_pulse_data;
    cmd[CMD_DATA_SIZE] = 88;
    cmd[CMD_REQUEST_ID] = REQUEST_ID_MOVE_PULSE;
    cmd[CMD_ADDRESS_ID] = '\x8B';
    cmd[CMD_INSTANCE] = move_type;
    cmd[CMD_ATTRIBUTE] = 1;
    cmd[CMD_SERVICE] = '\x02';
    if (speed < 0) speed = 0;
    quint32 speed_u = (quint32)(speed*10);
    quint32 x_u = (quint32)(position->at(0) * 1000);
    quint32 y_u = (quint32)(position->at(1) * 1000);
    quint32 z_u = (quint32)(position->at(2) * 1000);
    quint32 roll_u = (quint32)(position->at(3) * 10000);
    quint32 pitch_u = (quint32)(position->at(4) * 10000);
    quint32 yaw_u = (quint32)(position->at(5) * 10000);


    cmd[DATA_MOVE_SPEED_TYPE+HEADER_SIZE] = speed_type;

    cmd[DATA_MOVE_SPEED+HEADER_SIZE+3] = (quint8)(speed_u >>24);
    cmd[DATA_MOVE_SPEED+HEADER_SIZE+2] = (quint8)(speed_u >>16);
    cmd[DATA_MOVE_SPEED+HEADER_SIZE+1] = (quint8)(speed_u >>8);
    cmd[DATA_MOVE_SPEED+HEADER_SIZE] = (quint8)(speed_u);

    cmd[DATA_MOVE_X_PULSE+HEADER_SIZE+3]= (quint8)(x_u>>24);
    cmd[DATA_MOVE_X_PULSE+HEADER_SIZE+2]= (quint8)(x_u>>16);
    cmd[DATA_MOVE_X_PULSE+HEADER_SIZE+1]= (quint8)(x_u>>8);
    cmd[DATA_MOVE_X_PULSE+HEADER_SIZE]= (quint8)(x_u);

    cmd[DATA_MOVE_Y_PULSE+HEADER_SIZE+3]= (quint8)(y_u>>24);
    cmd[DATA_MOVE_Y_PULSE+HEADER_SIZE+2]= (quint8)(y_u>>16);
    cmd[DATA_MOVE_Y_PULSE+HEADER_SIZE+1]= (quint8)(y_u>>8);
    cmd[DATA_MOVE_Y_PULSE+HEADER_SIZE]= (quint8)(y_u);

    cmd[DATA_MOVE_Z_PULSE+HEADER_SIZE+3]= (quint8)(z_u>>24);
    cmd[DATA_MOVE_Z_PULSE+HEADER_SIZE+2]= (quint8)(z_u>>16);
    cmd[DATA_MOVE_Z_PULSE+HEADER_SIZE+1]= (quint8)(z_u>>8);
    cmd[DATA_MOVE_Z_PULSE+HEADER_SIZE]= (quint8)(z_u);

    cmd[DATA_MOVE_ROLL_PULSE+HEADER_SIZE+3]= (quint8)(roll_u>>24);
    cmd[DATA_MOVE_ROLL_PULSE+HEADER_SIZE+2]= (quint8)(roll_u>>16);
    cmd[DATA_MOVE_ROLL_PULSE+HEADER_SIZE+1]= (quint8)(roll_u>>8);
    cmd[DATA_MOVE_ROLL_PULSE+HEADER_SIZE]= (quint8)(roll_u);

    cmd[DATA_MOVE_PITCH_PULSE+HEADER_SIZE+3]= (quint8)(pitch_u>>24);
    cmd[DATA_MOVE_PITCH_PULSE+HEADER_SIZE+2]= (quint8)(pitch_u>>16);
    cmd[DATA_MOVE_PITCH_PULSE+HEADER_SIZE+1]= (quint8)(pitch_u>>8);
    cmd[DATA_MOVE_PITCH_PULSE+HEADER_SIZE]= (quint8)(pitch_u);

    cmd[DATA_MOVE_YAW_PULSE+HEADER_SIZE+3]= (quint8)(yaw_u>>24);
    cmd[DATA_MOVE_YAW_PULSE+HEADER_SIZE+2]= (quint8)(yaw_u>>16);
    cmd[DATA_MOVE_YAW_PULSE+HEADER_SIZE+1]= (quint8)(yaw_u>>8);
    cmd[DATA_MOVE_YAW_PULSE+HEADER_SIZE]= (quint8)(yaw_u);

    return cmd;
}
