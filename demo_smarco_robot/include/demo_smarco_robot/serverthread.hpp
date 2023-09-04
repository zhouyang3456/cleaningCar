#ifndef SERVERTHREAD_HPP
#define SERVERTHREAD_HPP

#include <QObject>
#include <QThread>
#include <QTcpSocket>
#include "mytcpsocket.hpp"

class ServerThread : public QThread
{
     Q_OBJECT
public:
    ServerThread();
    ServerThread(int sockDesc, QObject *parent = nullptr);

private:
    void run(void) override;

private slots:
    void recvData(QString addr, int port, int sockDesc, QByteArray data);
    void on_disconnected();
    void sendData_slot(QByteArray data);

signals:
    void dataReady(QString addr, int port, int sockDesc, QByteArray data);
    void disconnected_signal(int sockDesc);
    void sendData_signal(QByteArray data);

private:
    int m_sockDesc;
    MyTcpSocket *m_socket;
};

#endif // SERVERTHREAD_HPP
