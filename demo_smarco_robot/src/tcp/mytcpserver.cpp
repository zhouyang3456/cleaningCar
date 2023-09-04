#include "mytcpserver.hpp"

MyTcpServer::MyTcpServer()
{

}

MyTcpServer::~MyTcpServer()
{

}

void MyTcpServer::incomingConnection(qintptr sockDesc)
{
    // 客户端 socket 加入链表
    m_socketList.append(sockDesc);
    qDebug() << "New Socket" << sockDesc;

    // 创建服务器处理进程
    ServerThread *thread = new ServerThread(sockDesc);
    connect(thread, SIGNAL(dataReady(QString, int, int, QByteArray)),
            this, SLOT(recvData(QString, int, int, QByteArray)));
    connect(thread, SIGNAL(disconnected_signal(int)),
            this, SLOT(on_disconnected(int)));
    connect(this, SIGNAL(sendData_signal(QByteArray)),
            thread, SLOT(sendData_slot(QByteArray)));

    thread->start();
}

void MyTcpServer::recvData(QString addr, int port, int sockDesc, QByteArray data)
{
    emit dataReady(addr, port, sockDesc, data);
}

void MyTcpServer::sendData(QByteArray data)
{
    emit sendData_signal(data);
}

void MyTcpServer::on_disconnected(int sockDesc)
{
    m_socketList.removeOne(sockDesc);
    qDebug() << "Socket" << sockDesc << "disconnected from host";
}

