#ifndef FTPCLIENT_HPP
#define FTPCLIENT_HPP

#include <QObject>
#include <QNetworkAccessManager>
#include <QFile>
#include <QUrl>
#include <QNetworkReply>

class FtpClient : public QObject
{
    Q_OBJECT
public:
    explicit FtpClient(QObject *parent = nullptr);
    explicit FtpClient(QString, int, QString, QString);

    bool ftpPut(QString source, QString dest);

private slots:
    void finished(QNetworkReply *reply);

signals:

private:
    QString remoteHost;
    int port;
    QString user;
    QString pwd;

    QFile *m_pFile;
    QNetworkReply *m_pReply;
    QNetworkAccessManager *m_pManager;
    QUrl *m_pUrl;
    // bool is_down = true; /* 判断是否为下载选项 */

};

#endif // FTPCLIENT_HPP
