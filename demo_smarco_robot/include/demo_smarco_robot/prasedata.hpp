#ifndef PRASEDATA_HPP
#define PRASEDATA_HPP

#include <QObject>
#include <QDebug>

class PraseData : public QObject
{
    Q_OBJECT
public:
    PraseData(QObject *parent = nullptr);

signals:
    void praseCloudData_signal(uint8_t command,uint32_t roboid, QByteArray r_data);
    void praseServerData_signal(uint8_t command,uint32_t roboid, QByteArray r_data);
public slots:

public:
    void praseCloudData(QByteArray datagram);
    void praseServerData(QByteArray datagram);
    QByteArray serialize_data(uint8_t cmd, uint32_t robot_id, QByteArray mdata);

private:
    uint32_t robot_ID = 230412061;

    // 收到协议中的命令,0XFF为初始状态，命令为空
    uint8_t cloud_m_commend = 0xFF;
    // 一条数据的长度，数据位3-7位
    uint32_t cloud_m_count = 0;
    // 处理数据状态机，false表示没有未处理完的数据，true表示上一帧数据还有没有处理完全的数据
    bool cloud_datahandle_statue = false;
    // 解析出来的数据，用完需clear
    QByteArray cloud_datahandle;
    QByteArray cloud_data_sur;

    uint8_t server_m_commend = 0xFF;
    uint32_t server_m_count = 0;
    bool server_datahandle_statue = false;
    QByteArray server_datahandle;
    QByteArray server_data_sur;

};

#endif // PRASEDATA_HPP
