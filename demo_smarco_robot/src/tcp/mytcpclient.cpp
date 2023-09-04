#include "mytcpclient.hpp"

MyTcpClient::MyTcpClient()
{

}

MyTcpClient::MyTcpClient(QString ip, uint16_t port) :
    server_ip(ip), server_port(port), m_isConnected(false)
{

}

void MyTcpClient::run()
{
    socket = new QTcpSocket();
    socket->abort();
    connect(socket, SIGNAL(readyRead()), this, SLOT(readyRead()));
    connect(socket, SIGNAL(connected()), this, SLOT(connected()));
    connect(socket, SIGNAL(disconnected()), this, SLOT(disconnected()));
    connect(socket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(error_handler(QAbstractSocket::SocketError)));
    connect(this, SIGNAL(tryConnectSignal()), this, SLOT(tryConnectSlot()));

    if (!connect_to_host(server_ip, server_port))
        emit tryConnectSignal();

    exec(); // event loop
}

bool MyTcpClient::is_connected()
{
    return m_isConnected;
}

bool MyTcpClient::send_data(QByteArray data)
{
    if (!(socket->state() == QAbstractSocket::ConnectedState))
        return false;

    socket->write(data);

    if (!socket->waitForBytesWritten())
        return false;

    return true;
}

bool MyTcpClient::connect_to_host(const QString &address, uint16_t port)
{
    if( socket == nullptr )
    {
        m_isConnected = false;
        return false;
    }

    // If connected, return
    if( socket->state() == QAbstractSocket::ConnectedState )
    {
        if(socket->isValid()) {
            m_isConnected = true;
            return true;
        } else {
            m_isConnected = false;
            return false;
        }
    }

    // Try to connect
    socket->abort();
    socket->connectToHost(address, port);
    if( socket->waitForConnected(1000) )
        m_isConnected = true;
    else
        m_isConnected = false;

    return m_isConnected;
}

void MyTcpClient::readyRead()
{
    while(socket->bytesAvailable() > 0)
    {
        QByteArray datagram;
        datagram.resize(socket->bytesAvailable());
        socket->read(datagram.data(), datagram.size());
        emit clientReadyRead_signal(datagram);
    }
}

void MyTcpClient::connected()
{
    qDebug() << "Connected to server" << "\033[1;32msuccess\033[0m";
    m_isConnected = true;

    QByteArray arr("hello from tcp client");
    send_data(arr);
}

void MyTcpClient::disconnected()
{
    qDebug() << "Disconnected from server";
    socket->abort();
    m_isConnected = false;
    if(socket->state() != QAbstractSocket::UnconnectedState)
        socket->waitForDisconnected(1000);
    emit tryConnectSignal();
}

void MyTcpClient::tryConnectSlot()
{
    while (m_isConnected != true) {
        if (connect_to_host(server_ip, server_port)) {
            break;
        } else {
            qDebug() << "\033[1;31mFail\033[0m" << "connect to host, wait 5 seconds~";
            sleep_ms(reconnect_interval);
        }
    }
}

void MyTcpClient::error_handler(QAbstractSocket::SocketError err)
{
     qDebug() << "TCP Client Error: " << err;
}

void MyTcpClient::sleep_ms(uint msec)
{
        QTime reachTime = QTime::currentTime().addMSecs(msec);
        while(QTime::currentTime()<reachTime){
            QApplication::processEvents(QEventLoop::AllEvents, 100);
        }
}
