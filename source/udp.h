#ifndef UDP_H
#define UDP_H

#include <QObject>
#include <QUdpSocket>

class UDP : public QObject
{
    Q_OBJECT
public:
    explicit UDP(QObject *parent = nullptr);
    ~UDP();
    bool udpConnect(QHostAddress address, quint16 port);
    void udpDisconnect();
    void udpSendata(QByteArray data, QHostAddress address, quint16 port);
    QByteArray getUdpData();

signals:
    void udpDataReceive(QByteArray);
public slots:
    void readyRead();
private:
    QByteArray buffer;
    QUdpSocket *socket;
};

#endif // UDP_H
