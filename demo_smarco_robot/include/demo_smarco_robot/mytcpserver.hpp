#ifndef MYTCPSERVER_HPP
#define MYTCPSERVER_HPP

#include <QObject>
#include <QTcpServer>
#include "serverthread.hpp"

class MyTcpServer : public QTcpServer
{
     Q_OBJECT
public:
    MyTcpServer();
    ~MyTcpServer();

private:
    void incomingConnection(qintptr sockDesc) override;

signals:
    void dataReady(QString addr, int port, int sockDesc, QByteArray data);
    void sendData_signal(QByteArray data);
public slots:
    void recvData(QString addr, int port, int sockDesc, QByteArray data);
    void sendData(QByteArray data);
    void on_disconnected(int sockDesc);

private:
    QList<int> m_socketList; // 服务器 socket 链表


};

#endif // MYTCPSERVER_HPP
