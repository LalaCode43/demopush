#include "udp.h"

UDP::UDP(QObject *parent)
    : QObject{parent}
{
    socket = new QUdpSocket();
}

UDP::~UDP(){
    socket->close();
}

bool UDP::udpConnect(QHostAddress address, quint16 port){
    bool status = socket->bind(address, port);
    qDebug() << "Status connect: "<< status;
    connect(socket, &QUdpSocket::readyRead, this, &UDP::readyRead);
    return status;
}
void UDP::udpDisconnect(){
    socket->close();
}

QByteArray UDP::getUdpData(){
    return this->buffer;
}

void UDP::udpSendata(QByteArray data, QHostAddress address, quint16 port){
    //qDebug() << "Send data: "<<data;
    socket->writeDatagram(data, address, port);

}

void UDP::readyRead(){
    buffer.resize(socket->pendingDatagramSize());
    QHostAddress sender;
    quint16 senderPort;
    socket->readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);
    emit udpDataReceive(buffer);
}
