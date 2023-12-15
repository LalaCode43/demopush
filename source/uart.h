#ifndef UART_H
#define UART_H

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include "stdio.h"

using namespace std;
class UART : public QObject
{
    Q_OBJECT
public:
    explicit UART(QObject *parent = nullptr);

    void setUartPortName(const QString &newUartPortName);
    void openUartPort();
    void writeToSTM32(const QByteArray &data);

    float getRate() const;

    void setObject_detected(bool newObject_detected);

    void setRate(float newRate);

signals:
    void uartDataRecevie(float);
public slots:
    void readyRead();
private:
    QSerialPort *uartPort;
    bool isOpened;
    QString uartPortName;
    bool is_data_receive;
    QByteArray buffer;
    QByteArray receiveData;
    float rate;
    bool object_detected;
    int pulse_count;
};

#endif // UART_H
