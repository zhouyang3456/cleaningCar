#include "prasedata.hpp"

PraseData::PraseData(QObject *parent) : QObject(parent) {

}

void PraseData::praseCloudData(QByteArray datagram)
{
    // qDebug() << "Client Received data: " << datagram.toHex(' ').toUpper();

    if (!cloud_data_sur.isEmpty())
    {
        datagram = cloud_data_sur + datagram;
        if (((uint8_t)datagram.at(0) != 0x83) || ((uint8_t)datagram.at(1) != 0x84))  // 包头
        {
            datagram.clear();
            cloud_data_sur.clear();
            return;
        }
    }
    if(((uint8_t)datagram.at(0) != 0x83) || ((uint8_t)datagram.at(1) != 0x84))  // 包头
    {
        datagram.clear();
        return;
    }

    int datagarm_count = datagram.count();
    while (datagarm_count)  // 等待处理完这一帧数据
    {
        if (datagarm_count < 7)    // 剩余数据读不到长度
        {
            cloud_data_sur = datagram;  // 将剩余数据存下，下次继续解析
            return;
        }
        if ((uint8_t)datagram.at(0) != 0x83 || (uint8_t)datagram.at(1) != 0x84)
            return; // 包头
        // 现将二进制数据流转为uint_8型，否则数据转32位会出错
        uint8_t datagram_at6 = (uint8_t)datagram.at(6);
        uint8_t datagram_at5 = (uint8_t)datagram.at(5);
        uint8_t datagram_at4 = (uint8_t)datagram.at(4);
        uint8_t datagram_at3 = (uint8_t)datagram.at(3);
        // 得到发送的数据包的大小
        cloud_m_count = ((uint32_t)datagram_at3 << 24) +
                ((uint32_t)datagram_at4 << 16) +
                ((uint32_t)datagram_at5 << 8) +
                (uint32_t)datagram_at6;
        if (datagarm_count < cloud_m_count) // 剩余数据小于包数据长度
        {
            cloud_data_sur = datagram;  // 将剩余数据存下，下次继续解析
            return;
        }
        cloud_m_commend = (uint8_t)datagram.at(2);  // 取出命令
        // 将二进制数据流转为uint_8型，否则数据转32位会出错
        uint8_t datagram_at10 = (uint8_t)datagram.at(10);
        uint8_t datagram_at9 = (uint8_t)datagram.at(9);
        uint8_t datagram_at8 = (uint8_t)datagram.at(8);
        uint8_t datagram_at7 = (uint8_t)datagram.at(7);
        // 得到机器人ID
        robot_ID = ((uint32_t)datagram_at7 << 24) +
                ((uint32_t)datagram_at8 << 16) +
                ((uint32_t)datagram_at9 << 8) +
                (uint32_t)datagram_at10;
        // 判断包尾
        if (((uint8_t)datagram.at(cloud_m_count - 2) != 0xF4)
                || ((uint8_t)datagram.at(cloud_m_count - 1) != 0x4F))
        {
            qDebug()<<"Receive \033[1;31mError\033[0m\n Commend";
            cloud_datahandle.clear();   // 清理处理完的数据，释放内存
            cloud_data_sur.clear();
            cloud_m_count=0;
            datagram.clear();
            datagarm_count = 0;
            robot_ID = 0;
            // m_commend = 0xFF;    // 将命令复位
            return;
        }
        cloud_datahandle = datagram;
        // 先去除尾部本次不处理数据
        cloud_datahandle.remove(cloud_m_count - 1, datagarm_count - cloud_m_count);
        // cloud_datahandle.remove(cloud_m_count - 4,4);
        cloud_datahandle.remove(cloud_m_count-2,2); // no CRC
        cloud_datahandle.remove(0,11);
        // 处理本次取出的数据
        // cloud_handledata(cloud_m_commend, robot_ID, cloud_datahandle);
        emit praseCloudData_signal(cloud_m_commend, robot_ID, cloud_datahandle);
        cloud_m_commend = 0xFF; // 数据处理完将命令还原
        // 去除数据链中处理完的数据
        datagram.remove(0, cloud_m_count);
        // 记录剩下未处理的数据长度
        datagarm_count = datagarm_count - cloud_m_count;
        // 清理处理完的数据，释放内存
        cloud_datahandle.clear();
        cloud_data_sur.clear();
    }
}

void PraseData::praseServerData(QByteArray datagram)
{
    if (!server_data_sur.isEmpty())
    {
        datagram = server_data_sur + datagram;
        if (((uint8_t)datagram.at(0) != 0x83) || ((uint8_t)datagram.at(1) != 0x84))  // 包头
        {
            datagram.clear();
            server_data_sur.clear();
            return;
        }
    }
    if(((uint8_t)datagram.at(0) != 0x83) || ((uint8_t)datagram.at(1) != 0x84))  // 包头
    {
        datagram.clear();
        return;
    }

    int datagarm_count = datagram.count();
    while (datagarm_count)  // 等待处理完这一帧数据
    {
        if (datagarm_count < 7)    // 剩余数据读不到长度
        {
            server_data_sur = datagram;  // 将剩余数据存下，下次继续解析
            return;
        }
        if ((uint8_t)datagram.at(0) != 0x83 || (uint8_t)datagram.at(1) != 0x84)
            return; // 包头
        // 现将二进制数据流转为uint_8型，否则数据转32位会出错
        uint8_t datagram_at6 = (uint8_t)datagram.at(6);
        uint8_t datagram_at5 = (uint8_t)datagram.at(5);
        uint8_t datagram_at4 = (uint8_t)datagram.at(4);
        uint8_t datagram_at3 = (uint8_t)datagram.at(3);
        // 得到发送的数据包的大小
        server_m_count = ((uint32_t)datagram_at3 << 24) +
                ((uint32_t)datagram_at4 << 16) +
                ((uint32_t)datagram_at5 << 8) +
                (uint32_t)datagram_at6;
        if (datagarm_count < server_m_count) // 剩余数据小于包数据长度
        {
            server_data_sur = datagram;  // 将剩余数据存下，下次继续解析
            return;
        }
        server_m_commend = (uint8_t)datagram.at(2);  // 取出命令
        // 将二进制数据流转为uint_8型，否则数据转32位会出错
        uint8_t datagram_at10 = (uint8_t)datagram.at(10);
        uint8_t datagram_at9 = (uint8_t)datagram.at(9);
        uint8_t datagram_at8 = (uint8_t)datagram.at(8);
        uint8_t datagram_at7 = (uint8_t)datagram.at(7);
        // 得到机器人ID
        robot_ID = ((uint32_t)datagram_at7 << 24) +
                ((uint32_t)datagram_at8 << 16) +
                ((uint32_t)datagram_at9 << 8) +
                (uint32_t)datagram_at10;
        // 判断包尾
        if (((uint8_t)datagram.at(server_m_count - 2) != 0xF4)
                || ((uint8_t)datagram.at(server_m_count - 1) != 0x4F))
        {
            qDebug()<<"Receive \033[1;31mError\033[0m\n Commend";
            server_datahandle.clear();   // 清理处理完的数据，释放内存
            server_data_sur.clear();
            server_m_count=0;
            datagram.clear();
            datagarm_count = 0;
            robot_ID = 0;
            // m_commend = 0xFF;    // 将命令复位
            return;
        }
        server_datahandle = datagram;
        // 先去除尾部本次不处理数据
        server_datahandle.remove(server_m_count - 1, datagarm_count - server_m_count);
        // server_datahandle.remove(server_m_count - 4,4);
        server_datahandle.remove(server_m_count-2,2); // no CRC
        server_datahandle.remove(0,11);
        // 处理本次取出的数据
        // server_handledata(server_m_commend, robot_ID, server_datahandle);
        emit praseServerData_signal(server_m_commend, robot_ID, server_datahandle);
        server_m_commend = 0xFF; // 数据处理完将命令还原
        // 去除数据链中处理完的数据
        datagram.remove(0, server_m_count);
        // 记录剩下未处理的数据长度
        datagarm_count = datagarm_count - server_m_count;
        // 清理处理完的数据，释放内存
        server_datahandle.clear();
        server_data_sur.clear();
    }
}

QByteArray PraseData::serialize_data(uint8_t cmd, uint32_t robot_id, QByteArray mdata)
{
    QByteArray mdata_senddata;
    QByteArray mdata_val;
    // int mdata_count = mdata.count() + 15;
    int mdata_count = mdata.count() + 13;
    uint8_t data_send[11];
    data_send[0] = 0x83;
    data_send[1] = 0x84;
    data_send[2] = cmd;
    data_send[3] = mdata_count >> 24;
    data_send[4] = mdata_count >> 16;
    data_send[5] = mdata_count >> 8;
    data_send[6] = mdata_count;
    data_send[7] = robot_id >> 24;
    data_send[8] = robot_id >> 16;
    data_send[9] = robot_id >> 8;
    data_send[10] = robot_id;
    for (int j = 0; j < 11; j++)
    {
        mdata_val.push_back(data_send[j]);
    }
    mdata_senddata = mdata_val + mdata;
    // uint16_t crc16 = modbus_crc16(mdata_senddata);
    // uint8_t crc_high = (uint8_t)(crc16 >>8 &0x0FF);
    // uint8_t crc_low = (uint8_t)(crc16 & 0x0FF);
    // mdata_senddata.push_back(crc_high);
    // mdata_senddata.push_back(crc_low);
    mdata_senddata.push_back(0xF4);
    mdata_senddata.push_back(0x4F);
    // add 0x7c 0x7c 0x7c as flag to Separate packets
    // mdata_senddata.push_back(0x7c);
    // mdata_senddata.push_back(0x7c);
    // mdata_senddata.push_back(0x7c);
    return mdata_senddata;
}


