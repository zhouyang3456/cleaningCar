#ifndef MYTCPCLIENT_HPP
#define MYTCPCLIENT_HPP

#include <QObject>
#include <QThread>
#include <QtNetwork/QTcpSocket>
#include <QTime>
#include <QApplication>

class MyTcpClient : public QThread
{
    Q_OBJECT
public:
    MyTcpClient();
    MyTcpClient(QString ip, uint16_t port);
    void run() override;
    bool is_connected();
    bool send_data(QByteArray data);

private:
    QTcpSocket *socket = nullptr;
    QString server_ip;
    uint16_t server_port;
    bool m_isConnected;
    uint reconnect_interval = 5000;

private:
    void sleep_ms(uint msec);  // Delay
    bool connect_to_host(const QString & address,uint16_t port);

signals:
    void tryConnectSignal();
    void clientReadyRead_signal(QByteArray datagram);

private slots:
    void readyRead();
    void connected();
    void disconnected();
    void tryConnectSlot();
    void error_handler(QAbstractSocket::SocketError err);
};

#endif // MYTCPCLIENT_HPP
