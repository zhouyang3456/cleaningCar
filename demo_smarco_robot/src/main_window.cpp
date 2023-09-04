/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "main_window.hpp"
#include "prasedata.hpp"
#include "demo_smarco_robot/stringVector.h"
#include <tf/transform_broadcaster.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace demo_smarco_robot {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc, argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    // Before Auto Start
    connect(&qnode, SIGNAL(robotInit_signal(Robot*)), this, SLOT(robotInit_handler(Robot*)));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    ui.startPaths->setDisabled(false);
    ui.stopPaths->setDisabled(true);
    ui.startRecordPath->setDisabled(false);
    ui.stopRecordPath->setDisabled(true);
}

MainWindow::~MainWindow() {}

void MainWindow::robotInit_handler(Robot* robot_)
{
    robot = robot_;
    while (!ros::ok());

    pd = new PraseData;
    connect(pd, SIGNAL(praseCloudData_signal(uint8_t, uint32_t, QByteArray)),
            this, SLOT(cloud_handleData(uint8_t, uint32_t, QByteArray)));
    connect(pd, SIGNAL(praseServerData_signal(uint8_t, uint32_t, QByteArray)),
            this, SLOT(server_handleData(uint8_t, uint32_t, QByteArray)));

    m_tcpClient = new MyTcpClient(robot->server_addr, robot->server_port);
    m_tcpClient->moveToThread(m_tcpClient);
    // m_tcpClient->start();
    connect(m_tcpClient, SIGNAL(clientReadyRead_signal(QByteArray)), this, SLOT(clientReadyRead_handler(QByteArray)));

    // qnode.pub_poseEstimate(robot->poseEstimate);

    m_tcpServer = new MyTcpServer();
    m_tcpServer->listen(QHostAddress::Any, 12346);
    qDebug() << "TCP Server" << "\033[1;32mListening ...\033[0m";
    connect(m_tcpServer, SIGNAL(dataReady(QString, int, int, QByteArray)),
            this, SLOT(serverDataReady_handler(QString, int, int, QByteArray)));

    timer_init();

    connect(&qnode, SIGNAL(progress_signal(int)),
            this, SLOT(progress_handler(int)));
    connect(&qnode, SIGNAL(runTime_signal(uint)),
            this, SLOT(runTime_handler(uint)));

    pubElectronicFence();


}

void MainWindow::timer_init()
{
    recordPath_timer = new QTimer(this);
    connect(recordPath_timer, SIGNAL(timeout()), this, SLOT(slot_recordPath_timeout()));

    currentPose_timer = new QTimer(this);
    connect(currentPose_timer, SIGNAL(timeout()), this, SLOT(currentPose_timeout()));
    // currentPose_timer->start(5000);
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
    ui.view_logging->scrollToBottom();
}

void MainWindow::handleData(uint8_t command, uint32_t roboid, QByteArray r_data)
{
    switch (command) {
        case BATTERY_CAPACITY: {
            QByteArray datagram = handle_batteryCapacity();
            m_tcpServer->sendData(datagram);
            break;
        }

        case UP:
            qnode.move_base('i', robot->line_speed, robot->raw_speed);
            break;

        case LEFT:
            qnode.move_base('j', robot->line_speed, robot->raw_speed);
            break;

        case RIGHT:
            qnode.move_base('l', robot->line_speed, robot->raw_speed);
            break;

        case DOWN:
            qnode.move_base(',', robot->line_speed, robot->raw_speed);
            break;

        case CURRENT_POSE_AS_RETURN_POINT:
            robot->set_poseEstimate();
            break;

        case RECORD_PATH:
            handle_recordPath(r_data);
            break;

        case CLEANING:
            handle_cleaning(r_data);
            break;

        default:
            break;
    }
}

void MainWindow::cloud_handleData(uint8_t command, uint32_t roboid, QByteArray r_data)
{
    handleData(command, roboid, r_data);
}

void MainWindow::server_handleData(uint8_t command, uint32_t roboid, QByteArray r_data)
{
    handleData(command, roboid, r_data);
}

void MainWindow::handle_recordPath(QByteArray data_)
{
    switch (data_.at(0))
    {
        case 0x00 :
            if (robot->status == Robot::RECORD_PATH) {
                recordPath_timer->stop();
                // Add the last position point to path
                robot->recordPath_.poses.push_back(robot->pose);
                if(robot->recordPath_.poses.size() >= 2)
                    write_pathPointsToJson(robot->recordPath_, robot->recordPathJson_dir);
                else
                    ROS_INFO("Path Points less than 2");
                robot->status = Robot::FREE;
                ROS_INFO("Stop record path -------------------------");
            }
        break;

        case 0x01 :
            robot->status = Robot::RECORD_PATH;
            robot->recordPath_.poses.clear();
            recordPath_timer->start(100);
            ROS_INFO("Start record path -------------------------");
            break;

        case 0x02 :
            if (robot->status == Robot::RECORD_PATH) {
                robot->recordPath_.poses.push_back(robot->pose);
                ROS_INFO_STREAM("Record one point:\n" << robot->pose);
                robot->recordPath_.header.stamp = ros::Time::now();
                qnode.pub_rvizPath(robot->recordPath_);
            }
            break;

        default:
            break;
    }
}

void MainWindow::slot_recordPath_timeout()
{
    if(robot->recordPath_.poses.empty())
        robot->recordPath_.poses.push_back(robot->pose);
    if(robot->distance(robot->recordPath_.poses.back().pose, robot->pose.pose) > robot->recordPath_distance)
        robot->recordPath_.poses.push_back(robot->pose);
    if(robot->angular(robot->recordPath_.poses.back().pose, robot->pose.pose) > robot->recordPath_angular)
        robot->recordPath_.poses.push_back(robot->pose);

    robot->recordPath_.header.stamp = ros::Time::now();
    qnode.pub_rvizPoseArray(robot->recordPath_);
    qnode.pub_rvizPath(robot->recordPath_);
}

void MainWindow::currentPose_timeout()
{
    QByteArray poseByteArray = poseToQbyteArray(robot->pose.pose);
    QByteArray dataSend = pd->serialize_data(ROBOT_POS, robot->id, poseByteArray);
    m_tcpServer->sendData(dataSend);
}

void MainWindow::write_pathPointsToJson(nav_msgs::Path path_, std::string filePath_)
{
    Json::Value root;   // 表示整个 json 对象
    Json::Value goalItem;

    root["A_metaData"]["time"] = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();

    for(uint i = 0; i < path_.poses.size(); i++) {
        goalItem.clear();
        goalItem["location"].append(path_.poses.at(i).pose.position.x);
        goalItem["location"].append(path_.poses.at(i).pose.position.y);
        goalItem["location"].append(path_.poses.at(i).pose.orientation.z);
        goalItem["location"].append(path_.poses.at(i).pose.orientation.w);
        goalItem["name"] = Json::Value(std::to_string(i));
        root["goalname"].append(goalItem);
    }
    QDateTime dateTime= QDateTime::currentDateTime();
    std::string dateTime_str = dateTime .toString("yyyy_MM_dd_hh_mm_ss").toStdString();
    std::string fileName = "path_" + robot->floor + "_" + dateTime_str;
    std::fstream fs(filePath_ + fileName, std::fstream::out | std::fstream::trunc);
    Json::StyledWriter styled_writer;
    fs << styled_writer.write(root);
    fs.close();
}

void MainWindow::handle_cleaning(QByteArray data_)
{
    switch (data_.at(0))
    {
        case 0x00 :
            if (robot->status == Robot::CLEANING) {
                qnode.pub_multiNaviCmd(false);
                robot->status = Robot::FREE;
            }
            break;

        case 0x01 :
            robot->status = Robot::CLEANING;
            qnode.pub_multiNaviCmd(true);
            break;

        case 0x02 :
            robot->status = Robot::CLEANING;
            send_dirFiles();
            // startCleaning(data_);
            break;

        default:
            break;
    }
}

QByteArray MainWindow::handle_batteryCapacity()
{
    QByteArray battery_data = robot->battery_str.toLatin1();
    QByteArray dataSend = pd->serialize_data(BATTERY_CAPACITY, robot->id, battery_data);
    return dataSend;
}


void MainWindow::startCleaning(QByteArray data_)
{
    data_.remove(0, 1);

    stringVector s;
    if (data_.at(0) == s.SINGLE)
        s.mode = s.SINGLE;
    if (data_.at(0) == s.LOOP)
        s.mode = s.LOOP;
    data_.remove(0, 1);

    // s.strings.resize(data_.at(0));
    data_.remove(0, 1);

    QByteArray str_array;
    for (auto i = data_.begin(); i != data_.end(); i++) {
        if ((*i) == 0x01) {
            s.strings.push_back(str_array.toStdString());
            str_array.clear();
            continue;
        }
        str_array.append(*i);
    }
    for(auto i = s.strings.begin(); i != s.strings.end(); i++) {
       *i = robot->recordPathJson_dir + (*i);
    }
    qnode.pub_pathFiles(s);
}

void MainWindow::pubElectronicFence()
{
    std::string fileRealpath = robot->electronicFence_dir + "fence.json";
    std::cout << "fileRealpath = " << fileRealpath << std::endl;
    geometry_msgs::Polygon  fence;

    // Read Json Data
    Json::Reader reader_;
    Json::Value root_;
    std::ifstream in_;

    in_.open(fileRealpath, std::ios::binary);
    if (!in_.is_open()) {
        std::cout << "Error opening file\n";
        return;
    }
    if (reader_.parse(in_, root_)) {
        for (uint i = 0; i < root_["fencePoints"].size(); i++) {
            geometry_msgs::Point32 tmp;
            tmp.x = root_["fencePoints"][i]["point"][0].asFloat();
            tmp.y = root_["fencePoints"][i]["point"][1].asFloat();
            fence.points.push_back(tmp);
        }
    }
    in_.close();

    qnode.pub_electronicFence(fence);
}

QByteArray MainWindow::poseToQbyteArray(geometry_msgs::Pose pos)
{
    tf::Quaternion quat1;
    tf::quaternionMsgToTF(pos.orientation, quat1);
    double roll1, pitch1, yaw1;
    tf::Matrix3x3(quat1).getRPY(roll1, pitch1, yaw1);

    QByteArray RoboPose_data_cloud;
    double RoboPose_x_cloud, RoboPose_y_cloud, RoboPose_raw_cloud;
    RoboPose_x_cloud = pos.position.x;
    RoboPose_y_cloud = pos.position.y;
    RoboPose_raw_cloud = yaw1;
    RoboPose_data_cloud.clear();
    qint32 int32_RoboPose_x = static_cast<qint32>(RoboPose_x_cloud * 10000);
    QString str_int32_RoboPose_x_hex = QString("%1").arg(int32_RoboPose_x, 8, 16, QLatin1Char('0'));
    RoboPose_data_cloud[0] = str_int32_RoboPose_x_hex.mid(0, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[1] = str_int32_RoboPose_x_hex.mid(2, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[2] = str_int32_RoboPose_x_hex.mid(4, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[3] = str_int32_RoboPose_x_hex.mid(6, 2).toInt(nullptr, 16);

    qint32 int32_RoboPose_y = static_cast<qint32>(RoboPose_y_cloud * 10000);
    QString str_int32_RoboPose_y_hex = QString("%1").arg(int32_RoboPose_y, 8, 16, QLatin1Char('0'));
    RoboPose_data_cloud[4] = str_int32_RoboPose_y_hex.mid(0, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[5] = str_int32_RoboPose_y_hex.mid(2, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[6] = str_int32_RoboPose_y_hex.mid(4, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[7] = str_int32_RoboPose_y_hex.mid(6, 2).toInt(nullptr, 16);

    qint32 int32_RoboPose_raw = static_cast<qint32>(RoboPose_raw_cloud * 10000);
    QString str_int32_RoboPose_raw_hex = QString("%1").arg(int32_RoboPose_raw, 8, 16, QLatin1Char('0'));
    RoboPose_data_cloud[8] = str_int32_RoboPose_raw_hex.mid(0, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[9] = str_int32_RoboPose_raw_hex.mid(2, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[10] = str_int32_RoboPose_raw_hex.mid(4, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[11] = str_int32_RoboPose_raw_hex.mid(6, 2).toInt(nullptr, 16);

    return RoboPose_data_cloud;
}

void MainWindow::send_dirFiles()
{
    QDir directory(robot->recordPathJson_dir.c_str());
    QStringList files = directory.entryList(QDir::Files);
    qDebug() << files;

    stringVector s;
    for (auto i = files.begin(); i != files.end(); i++) {
        std::string fileRealPath =robot->recordPathJson_dir + (*i).toStdString();
        s.strings.push_back(fileRealPath);
    }
    // s.mode = s.SINGLE;
    s.mode = s.LOOP;
    qnode.pub_pathFiles(s);
}

void MainWindow::clientReadyRead_handler(QByteArray datagram)
{
    qDebug() << "Client Received data: " << datagram.toHex(' ').toUpper();
    pd->praseCloudData(datagram);
}

void MainWindow::serverDataReady_handler(QString addr, int port, int sockDesc, QByteArray datagram)
{
    qDebug() << "Server Received data from socket" << sockDesc << ":"
             << datagram.toHex(' ').toUpper();
    pd->praseServerData(datagram);
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>CLEANING ROBOT Test Program 0.10</h2><p>Copyright YCZK Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "demo_smarco_robot");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://127.0.0.1:11311/")).toString();
    QString host_url = settings.value("host_url", QString("127.0.0.1")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "demo_smarco_robot");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::clear_rviz()
{
    nav_msgs::Path path_empty;
    path_empty.header.frame_id = "map";
    for (int i = 0; i < 10; i++) {
        qnode.pub_rvizPath(path_empty);
        qnode.pub_rvizPoseArray(path_empty);
    }
    visualization_msgs::Marker marker_msg;
    qnode.pub_rvizMarker(marker_msg);
    ros::spinOnce();
}

void MainWindow::on_refresh_clicked()
{
    m_fileModel.setRootPath(QString::fromStdString(robot->recordPathJson_dir));
    ui.listView->setModel(&m_fileModel);
    ui.listView->setViewMode(QListView::ListMode);
    ui.listView->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.listView->setRootIndex(m_fileModel.index(QString::fromStdString(robot->recordPathJson_dir)));
}

void MainWindow::on_startPaths_clicked()
{
    clear_rviz();

    stringVector s;
    QModelIndexList indexes = ui.listView->selectionModel()->selectedIndexes(); // 获取已选择的项
    if (indexes.size() == 0)
        return;
    foreach(QModelIndex index, indexes) {
        QString fileName = QString::fromStdString(robot->recordPathJson_dir) +
                index.sibling(index.row(), 0).data().toString();
        s.strings.push_back(fileName.toStdString());
    }

    // s.mode = s.SINGLE;
    s.mode = s.LOOP;
    qnode.pub_pathFiles(s);
    ui.label_status->setText("Cleaning ...");

    ui.startPaths->setDisabled(true);
    ui.stopPaths->setDisabled(false);
    ui.startRecordPath->setDisabled(true);
    ui.stopRecordPath->setDisabled(true);
    ui.progressBar->setValue(0);
}

void MainWindow::on_stopPaths_clicked()
{
    qnode.pub_multiNaviCmd(false);
    ui.label_status->setText("FREE");

    ui.startPaths->setDisabled(false);
    ui.stopPaths->setDisabled(true);
    ui.startRecordPath->setDisabled(false);
    ui.stopRecordPath->setDisabled(true);
}

void MainWindow::on_startRecordPath_clicked()
{
    clear_rviz();

    robot->status = Robot::RECORD_PATH;
    robot->recordPath_.poses.clear();
    recordPath_timer->start(100);
    ROS_INFO("Start record path -------------------------");
    ui.label_status->setText("Record path ...");

    ui.startPaths->setDisabled(true);
    ui.stopPaths->setDisabled(true);
    ui.startRecordPath->setDisabled(true);
    ui.stopRecordPath->setDisabled(false);
}

void MainWindow::on_stopRecordPath_clicked()
{
    if (robot->status == Robot::RECORD_PATH) {
        recordPath_timer->stop();
        // Add the last position point to path
        robot->recordPath_.poses.push_back(robot->pose);
        if(robot->recordPath_.poses.size() >= 2)
            write_pathPointsToJson(robot->recordPath_, robot->recordPathJson_dir);
        else
            ROS_INFO("Path Points less than 2");
        robot->status = Robot::FREE;
        ROS_INFO("Stop record path -------------------------");
        ui.label_status->setText("FREE");
    }

    ui.startPaths->setDisabled(false);
    ui.stopPaths->setDisabled(true);
    ui.startRecordPath->setDisabled(false);
    ui.stopRecordPath->setDisabled(true);

    ui.refresh->clicked(true);
}

void MainWindow::progress_handler(int percentage)
{
    ui.progressBar->setValue(static_cast<int>(percentage));

    char per = static_cast<char>(percentage);
    QByteArray perArray;
    perArray.append(per);
    QByteArray dataSend = pd->serialize_data(CLEANING_PROGRESS, robot->id, perArray);
    m_tcpServer->sendData(dataSend);
}

void MainWindow::runTime_handler(uint time)
{
    // 将秒数转化为时分秒格式
    int H = time / (60*60);
    int M = (time- (H * 60 * 60)) / 60;
    int S = (time - (H * 60 * 60)) - M * 60;
    QString hour = QString::number(H);
    if (hour.length() == 1) hour = "0" + hour;
    QString min = QString::number(M);
    if (min.length() == 1) min = "0" + min;
    QString sec = QString::number(S);
    if (sec.length() == 1) sec = "0" + sec;
    QString qTime = hour + ":" + min + ":" + sec;
    ui.label_cleaningTime->setText(qTime);

    uint32_t uint32Value = time;
    char charArray[4];
    memcpy(charArray, &uint32Value, sizeof(uint32_t));

    QByteArray byteArray(charArray, 4);
    std::reverse(byteArray.begin(), byteArray.end());   // Big-endian
    QByteArray dataSend = pd->serialize_data(CLEANING_TIME, robot->id, byteArray);
    m_tcpServer->sendData(dataSend);
}

void MainWindow::on_delete_2_clicked()
{
    QModelIndex currentIndex = ui.listView->currentIndex();
    QString m_strFileName = QString::fromStdString(robot->recordPathJson_dir);
    m_strFileName += currentIndex.sibling(currentIndex.row(),0).data().toString();

    QFile m_clsFile(m_strFileName);
    QFileInfo m_clsFileInfo(m_strFileName);

    if(!m_clsFileInfo.isFile())
    {
        QMessageBox::warning(this,tr("Warning"),tr("Please select a file!"));
        return;
    }
    else
    {
        int ret = QMessageBox::information(this, tr("Message"),
                                           tr("Are you sure delete the file?"),
                                           QMessageBox::Cancel|QMessageBox::Ok);
        switch (ret) {
        case QMessageBox::Cancel:
            break;

        case QMessageBox::Ok:
        {
            m_clsFile.remove();
        }
            break;
        default:
            break;
        }
    }
}

}  // namespace demo_smarco_robot
