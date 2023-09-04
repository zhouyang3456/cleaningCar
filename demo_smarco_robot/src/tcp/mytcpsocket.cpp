#include "mytcpsocket.hpp"
#include <QHostAddress>

MyTcpSocket::MyTcpSocket()
{

}

MyTcpSocket::MyTcpSocket(int sockDesc, QObject *parent) :
    m_sockDesc(sockDesc)
{
    setSocketDescriptor(m_sockDesc);
    connect(this, SIGNAL(readyRead()), this, SLOT(recvData()));
}

void MyTcpSocket::recvData()
{
    QString addr = peerAddress().toString();
    int port = peerPort();
    QByteArray data = readAll();
    emit dataReady(addr, port, m_sockDesc, data);
}

void MyTcpSocket::sendData_slot(QByteArray data)
{
    if (!data.isEmpty() && this->state() == QAbstractSocket::ConnectedState)
        this->write(data);
    this->waitForBytesWritten();
}
