/**
 * @file /include/demo_smarco_robot/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef demo_smarco_robot_QNODE_HPP_
#define demo_smarco_robot_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QStringListModel>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "robot.hpp"
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include "demo_smarco_robot/stringVector.h"
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Polygon.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace demo_smarco_robot {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void robotInit_signal(Robot* robot_);
    void progress_signal(int percentage);
    void runTime_signal(uint time);

public:
    void move_base(char k, float speed_linear, float speed_trun);
    void pub_rvizPath(nav_msgs::Path & path_);
    void pub_rvizPoseArray(nav_msgs::Path & path_);
    void pub_rvizMarker(visualization_msgs::Marker mark);
    void pub_multiNaviCmd(bool state_);
    void pub_poseEstimate(geometry_msgs::PoseWithCovarianceStamped& pose_);
    void pub_pathFiles(stringVector & sv);
    void pub_electronicFence(geometry_msgs::Polygon fence);

private:
	int init_argc;
	char** init_argv;
    QStringListModel logging_model;
    Robot* robot;
    QMutex* m_mutex;

    ros::Subscriber currentPos_sub;
    ros::Subscriber multiNaviResult_sub;
    ros::Subscriber multiNaviRunTime_sub;

    ros::Publisher chatter_publisher;
    ros::Publisher cmd_pub;
    ros::Publisher rvizPath_pub;
    ros::Publisher rvizPoseArray_pub;
    ros::Publisher rvizMarker_pub;
    ros::Publisher multiNavi_pub;
    ros::Publisher pathFiles_pub;
    ros::Publisher poseEstimate_pub;
    ros::Publisher electronicFenceMaster_pub;


private:
    void topic_init(ros::NodeHandle& n);
    void currentPos_callback(const geometry_msgs::PoseWithCovarianceStamped& pos);
    void multiNaviResult_callback(std_msgs::UInt8 percentage);
    void multiNaviRunTime_callback(std_msgs::Float64 time_sec);

};

}  // namespace demo_smarco_robot

#endif /* demo_smarco_robot_QNODE_HPP_ */
