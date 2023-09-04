#include "serverthread.hpp"

ServerThread::ServerThread()
{

}

ServerThread::ServerThread(int sockDesc, QObject *parent) :
    m_sockDesc(sockDesc)
{

}

void ServerThread::run()
{
    m_socket = new MyTcpSocket(m_sockDesc);
    connect(m_socket, SIGNAL(dataReady(QString, int, int, QByteArray)),
            this, SLOT(recvData(QString, int, int, QByteArray)));
    connect(m_socket, SIGNAL(disconnected()),
            this, SLOT(on_disconnected()));
    connect(this, SIGNAL(sendData_signal(QByteArray)),
            m_socket, SLOT(sendData_slot(QByteArray)));
    this->exec();
}

void ServerThread::recvData(QString addr, int port, int sockDesc, QByteArray data)
{
    // qDebug() << "Server Received data: " << data.toHex(' ').toUpper();
    emit dataReady(addr, port, sockDesc, data);
}

void ServerThread::on_disconnected()
{
    emit disconnected_signal(m_sockDesc);
    this->quit();
}

void ServerThread::sendData_slot(QByteArray data)
{
    emit sendData_signal(data);
}
