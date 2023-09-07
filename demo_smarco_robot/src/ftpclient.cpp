#include "ftpclient.hpp"

FtpClient::FtpClient(QObject *parent) : QObject(parent)
{

}

FtpClient::FtpClient(QString remoteHost_, int port_, QString user_, QString pwd_):
    remoteHost(remoteHost_), port(port_), user(user_), pwd(pwd_)
{
    m_pManager = new QNetworkAccessManager();
    m_pUrl = new QUrl();
    m_pUrl->setScheme ( "ftp" );
    m_pUrl->setHost(remoteHost);
    m_pUrl->setPort(port);
    m_pUrl->setUserName(user);
    m_pUrl->setPassword(pwd);
    connect(m_pManager, SIGNAL(finished(QNetworkReply*)),
            this, SLOT(finished(QNetworkReply*)));
}

bool FtpClient::ftpPut(QString source, QString dest)
{
    // is_down = false; /* 该选项是上传选项 */
    QFile file(source);
    if (file.open(QIODevice::ReadOnly)) {
        QByteArray data = file.readAll();
        m_pUrl->setPath(dest);
        m_pManager->put(QNetworkRequest(*m_pUrl), data);
        return true;
    } else {
        return false;
    }
}

void FtpClient::finished(QNetworkReply *reply)
{
    if (reply->error() == QNetworkReply::NoError){
        qDebug() << "upload success";
        reply->deleteLater();
    }
    else{
        qDebug() << "upload error !!!";
        reply->deleteLater();
    }
}
