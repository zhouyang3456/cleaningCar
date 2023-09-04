/**
 * @file /include/demo_smarco_robot/main_window.hpp
 *
 * @brief Qt based gui for demo_smarco_robot.
 *
 * @date November 2010
 **/
#ifndef demo_smarco_robot_MAIN_WINDOW_H
#define demo_smarco_robot_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

// #include <QtGui/QMainWindow>

#include <QtWidgets/QMainWindow>
#include <yaml-cpp/yaml.h>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <QDateTime>
#include <geometry_msgs/Pose.h>
#include <qmath.h>

#include "ui_main_window.h"
#include "qnode.hpp"
#include "robot.hpp"
#include "mytcpclient.hpp"
#include "prasedata.hpp"
#include "mytcpserver.hpp"

#include <QFileSystemModel>
#include <visualization_msgs/Marker.h>
#include <jsoncpp/json/json.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace demo_smarco_robot {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = nullptr);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
    Robot* robot;
	QNode qnode;
    MyTcpClient* m_tcpClient;
    PraseData* pd;
    MyTcpServer* m_tcpServer;
    QFileSystemModel m_fileModel;

    QTimer* recordPath_timer;
    QTimer* currentPose_timer;

private:
    void handleData(uint8_t command,uint32_t roboid, QByteArray r_data);
    void handle_recordPath(QByteArray data_);
    void write_pathPointsToJson(nav_msgs::Path path_, std::string filePath_);
    void handle_cleaning(QByteArray data_);
    QByteArray handle_batteryCapacity();
    void timer_init();
    void startCleaning(QByteArray data_);
    void pubElectronicFence();
    QByteArray poseToQbyteArray(geometry_msgs::Pose pos);

    // TEST
    void send_dirFiles();
    void clear_rviz();

private slots:
    void clientReadyRead_handler(QByteArray datagram);
    void serverDataReady_handler(QString addr, int port, int sockDesc, QByteArray datagram);

    void robotInit_handler(Robot* robot_);
    void cloud_handleData(uint8_t command,uint32_t roboid, QByteArray r_data);
    void server_handleData(uint8_t command,uint32_t roboid, QByteArray r_data);
    void on_refresh_clicked();
    void on_startPaths_clicked();
    void on_stopPaths_clicked();
    void on_startRecordPath_clicked();
    void on_stopRecordPath_clicked();

    void slot_recordPath_timeout();
    void currentPose_timeout();

    void progress_handler(int percentage);
    void runTime_handler(uint time);
    void on_delete_2_clicked();
};

}  // namespace demo_smarco_robot

const uint8_t BATTERY_CAPACITY = 0x01;
const uint8_t ROBOT_POS = 0x05;
const uint8_t UP = 0x11;
const uint8_t LEFT = 0x13;
const uint8_t RIGHT = 0x15;
const uint8_t DOWN = 0x17;
const uint8_t CURRENT_POSE_AS_RETURN_POINT = 0x49;
const uint8_t RECORD_PATH = 0x53;
const uint8_t CLEANING = 0x55;
const uint8_t CLEANING_PROGRESS = 0x61;
const uint8_t CLEANING_TIME = 0x62;


#endif // demo_smarco_robot_MAIN_WINDOW_H
