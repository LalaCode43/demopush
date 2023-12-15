#ifndef YRC1000MICRO_COMMAND_H
#define YRC1000MICRO_COMMAND_H

#include <QObject>
#include <QDebug>
#define CMD_DATA_SIZE 6
#define CMD_REQUEST_ID 11
#define CMD_ADDRESS_ID 24
#define CMD_INSTANCE 26
#define CMD_ATTRIBUTE 28
#define CMD_SERVICE 29
#define CMD_DATA_BYTE0 32
#define CMD_DATA_BYTE1 33
#define CMD_DATA_BYTE2 34
#define CMD_DATA_BYTE3 35

#define HEADER_SIZE 32

///////////////// DEFINE REQUEST ID INDEX ////////////////////
#define REQUEST_ID_SERVO_ON 0
#define REQUEST_ID_SERVO_OFF 1

#define REQUEST_ID_WRITE_ROBOT_VARIABLE 3
#define REQUEST_ID_WRITE_REGISTER 4
#define REQUEST_ID_WRITE_BYTE 5
#define REQUEST_ID_WRITE_INTEGER 6
#define REQUEST_ID_WRITE_DOUBLE 7
#define REQUEST_ID_WRITE_REAL 8
#define REQUEST_ID_WRITE_STRING 9

#define REQUEST_ID_READ_ROBOT_VARIABLE 10
#define REQUEST_ID_READ_ROBOT_POSITION 11
#define REQUEST_ID_READ_ROBOT_PULSE 12
#define REQUEST_ID_READ_BYTE 13
#define REQUEST_ID_SELECT_JOB 21
#define REQUEST_ID_DISPLAY_COMMAND 22

#define REQUEST_ID_READ_STATUS_INFORMATION 30
#define REQUEST_ID_READ_STATUS_SERVO 31
#define REQUEST_ID_READ_ROBOT_RUNNING 32

#define REQUEST_ID_MOVE_CARTESIAN 40

#define REQUEST_ID_MOVE_PULSE 41


#define REQUEST_ID_READ_BYTE_0 150
#define REQUEST_ID_READ_BYTE_1 151
#define REQUEST_ID_READ_BYTE_2 152
#define REQUEST_ID_READ_BYTE_3 153
#define REQUEST_ID_READ_BYTE_4 154
#define REQUEST_ID_READ_BYTE_5 155
#define REQUEST_ID_READ_BYTE_6 156
#define REQUEST_ID_READ_BYTE_9 189
#define REQUEST_ID_READ_BYTE_10 160

////////////////////////////////////////////////////////////

#define DATA_MOVE_X_CARTESIAN 20
#define DATA_MOVE_Y_CARTESIAN 24
#define DATA_MOVE_Z_CARTESIAN 28
#define DATA_MOVE_ROLL_CARTESIAN 32
#define DATA_MOVE_PITCH_CARTESIAN 36
#define DATA_MOVE_YAW_CARTESIAN 40

#define DATA_MOVE_X_PULSE 16
#define DATA_MOVE_Y_PULSE 20
#define DATA_MOVE_Z_PULSE 24
#define DATA_MOVE_ROLL_PULSE 28
#define DATA_MOVE_PITCH_PULSE 32
#define DATA_MOVE_YAW_PULSE 36

#define DATA_MOVE_SPEED_TYPE 8
#define DATA_MOVE_SPEED 12
#define DATA_MOVE_COORDINATE 16

class YRC1000micro_command : public QObject
{
    Q_OBJECT
public:
    explicit YRC1000micro_command(QObject *parent = nullptr);
    QByteArray setServoOn();
    QByteArray setServoOff();

    // Byte type Variable
    QByteArray cmdSetByteVaribale(quint8 byte_number, quint8 byte_value, quint8 request_id=REQUEST_ID_WRITE_BYTE);
    QByteArray cmdReadByteVaribale(quint8 byte_number, quint8 request_id=REQUEST_ID_READ_BYTE);

    QByteArray cmdSetRegisterVaribale(quint8 register_number, quint16 register_value);
    QByteArray cmdSetIntegerVaribale(quint8 int_number, qint16 int_value);
    QByteArray cmdSetDoubleVariable(quint8 double_number, qint32 double_value);
    QByteArray cmdSetRealVariable(quint8 real_number, float real_value);
    QByteArray cmdSetString16Variable(quint8 string_number, QString string_value);
    // Robot variable
    QByteArray cmdSetRobotVariable(quint8 pos_number, QVector<double> pos);
    QByteArray cmdReadRobotVariable(quint8 pos_number);

    QByteArray cmdReadStatusInformation(quint8 request_id = REQUEST_ID_READ_STATUS_INFORMATION);

    QByteArray cmdReadRobotPostion();
    QByteArray cmdReadRobotPulse();
    QByteArray cmdDisplayCommand(QString command);
    QByteArray cmdSelectJob(QString job_name);
    QByteArray cmdStartUp();
    QByteArray cmdMoveRobotCartesian(quint8 coordinate, quint8 move_type, quint8 speed_type, double speed, QVector<double> *position);
    QByteArray cmdMoveRobotPulse(quint8 move_type, quint8 speed_type, double speed, QVector<double> *position);
signals:
private:
    QByteArray header_to_send;
    QByteArray data_to_send;
    QByteArray byte_data;
    QByteArray register_data;
    QByteArray robot_postion;
    QByteArray string_data;
    QByteArray line_number;
    QByteArray data_one_byte;
    QByteArray data_two_bytes;
    QByteArray move_cartesian_data;
    QByteArray move_pulse_data;
    QByteArray string_data_16;
};

#endif // YRC1000MICRO_COMMAND_H
