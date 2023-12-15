#include "uart.h"

UART::UART(QObject *parent)
    : QObject{parent}
{
    uartPort = new QSerialPort();
    uartPort->setFlowControl(QSerialPort::NoFlowControl);
    uartPort->setBaudRate(QSerialPort::Baud115200);
    uartPort->setDataBits(QSerialPort::Data8);
    uartPort->setParity(QSerialPort::NoParity);
    uartPort->setStopBits(QSerialPort::OneStop);
    isOpened = false;
    is_data_receive = false;
    rate = 0;
    object_detected = false;
    pulse_count = 0;
}

void UART::setUartPortName(const QString &newUartPortName)
{
    uartPortName = newUartPortName;
    uartPort->setPortName(uartPortName);
}

void UART::openUartPort()
{
    if(uartPort->open(QIODevice::ReadWrite)){
        isOpened = true;
        connect(uartPort, &QSerialPort::readyRead, this, &UART::readyRead);
    }else{
        isOpened = false;
    }
    if (isOpened){
        qDebug() << "Open port success: " + QString(uartPortName);
    }
    else{
        qDebug() << "Failed to Open UART";
    }
}

void UART::writeToSTM32(const QByteArray &data)
{
    //qDebug() << data;
    uartPort->write(data);
}

void UART::readyRead()
{
    while(uartPort->bytesAvailable()){
        buffer += uartPort->readAll();
        //qDebug() << buffer;
        if(buffer.endsWith(char(10))){
            is_data_receive = true;
        }
    }
    if(is_data_receive){
        is_data_receive = false;
        sscanf(buffer, "rate : %f \n", &rate);
        buffer.clear();

    }
}

void UART::setRate(float newRate)
{
    rate = newRate;
}

void UART::setObject_detected(bool newObject_detected)
{
    object_detected = newObject_detected;
}

float UART::getRate() const
{
    return rate;
}
