#ifndef MYTCPSOCKET_HPP
#define MYTCPSOCKET_HPP

#include <QObject>
#include <QTcpSocket>

class MyTcpSocket : public QTcpSocket
{
     Q_OBJECT
public:
    MyTcpSocket();
    explicit MyTcpSocket(int sockDesc, QObject *parent = nullptr);

signals:
    void dataReady(QString addr,int port,int sockDesc, QByteArray data);

private slots:
    void recvData();
    void sendData_slot(QByteArray data);

private:
    int m_sockDesc;
};

#endif // MYTCPSOCKET_HPP
