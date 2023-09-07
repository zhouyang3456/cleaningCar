/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "qnode.hpp"
#include <geometry_msgs/Twist.h>
#include <QDebug>
#include <QDateTime>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace demo_smarco_robot {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv),
    robot(new Robot),
    m_mutex(new QMutex)
    {}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"demo_smarco_robot");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    // Add your ros communications here.
    topic_init(n);
    start();

    /***********************************
    **  Robot Init
    ************************************/
    std::string robot_config_path;
    ros::param::get("~robot_config_path", robot_config_path);
    robot->configYaml_dir = robot_config_path;
    robot->robot_init(robot_config_path);
    emit robotInit_signal(robot);

    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"demo_smarco_robot");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    topic_init(n);
    start();

    /***********************************
    **  Robot Init
    ************************************/
    std::string robot_config_path;
    ros::param::get("robot_config_path", robot_config_path);
    robot->configYaml_dir = robot_config_path;
    robot->robot_init(robot_config_path);
    emit robotInit_signal(robot);

	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
    // int count = 0;
	while ( ros::ok() ) {

        // std_msgs::String msg;
        // std::stringstream ss;
        // ss << "hello world " << count;
        // msg.data = ss.str();
        // chatter_publisher.publish(msg);
        // log(Info,std::string("I sent: ")+msg.data);
// ////////////////////////////////////////////////
//        log(Info, "robot.x = " + QString::number(robot->pose.position.x).toStdString());
// ////////////////////////////////////////////////
		ros::spinOnce();
		loop_rate.sleep();
        // ++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::topic_init(ros::NodeHandle &n)
{
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

    /******************************
    **  SUBSCRIBE
    *******************************/
    currentPos_sub = n.subscribe("mypose", 1, &QNode::currentPos_callback, this);
    multiNaviResult_sub = n.subscribe("/multiPoints_navi/result", 1, &QNode::multiNaviResult_callback, this);
    multiNaviRunTime_sub = n.subscribe("/multiPoints_navi/runTime", 1, &QNode::multiNaviRunTime_callback, this);

    /******************************
    **  PUBLISH
    *******************************/
    cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    rvizPath_pub = n.advertise<nav_msgs::Path>("/record_path/rviz_path", 10);
    rvizPoseArray_pub = n.advertise<geometry_msgs::PoseArray>("/multiPoints_navi/rviz/poseArray", 10);
    rvizMarker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    multiNavi_pub = n.advertise<std_msgs::Bool>("/multiPoints_navi/cmd", 1);
    pathFiles_pub = n.advertise<stringVector>("/multiPoints_navi/pathFiles", 1);
    poseEstimate_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    electronicFenceMaster_pub = n.advertise<geometry_msgs::Polygon>("/ElectronicFenceMaster", 1);
    initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    // Sleep to ensure topic register success
    ros::Duration(3.0).sleep();
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

// 发布机器人速度控制
void QNode::move_base(char k, float speed_linear, float speed_trun) {
    std::map<char, std::vector<float>> moveBindings{
        {'i', {1, 0, 0, 0}},  {'o', {1, 0, 0, -1}},  {'j', {0, 0, 0, 1}},
        {'l', {0, 0, 0, -1}}, {'u', {1, 0, 0, 1}},   {',', {-1, 0, 0, 0}},
        {'.', {-1, 0, 0, 1}}, {'m', {-1, 0, 0, -1}}, {'O', {1, -1, 0, 0}},
        {'I', {1, 0, 0, 0}},  {'J', {0, 1, 0, 0}},   {'L', {0, -1, 0, 0}},
        {'U', {1, 1, 0, 0}},  {'<', {-1, 0, 0, 0}},  {'>', {-1, -1, 0, 0}},
        {'M', {-1, 1, 0, 0}}, {'t', {0, 0, 1, 0}},   {'b', {0, 0, -1, 0}},
        {'k', {0, 0, 0, 0}},  {'K', {0, 0, 0, 0}}};
    char key = k;
    // 计算是往哪个方向
    float x = moveBindings[key][0];
    float y = moveBindings[key][1];
    float z = moveBindings[key][2];
    float th = moveBindings[key][3];
    // 计算线速度和角速度
    float speed = speed_linear;
    float turn = speed_trun;
    // Update the Twist message
    geometry_msgs::Twist twist;
    twist.linear.x = x * speed;
    twist.linear.y = y * speed;
    twist.linear.z = z * speed;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn;

    // Publish it and resolve any remaining callbacks
    cmd_pub.publish(twist);
    ros::spinOnce();
}

void QNode::pub_rvizPath(nav_msgs::Path & path_)
{
    path_.header.frame_id = "map";
    rvizPath_pub.publish(path_);
    ros::spinOnce();
}

void QNode::pub_rvizPoseArray(nav_msgs::Path &path_)
{
    geometry_msgs::PoseArray pa;
    pa.poses.clear();
    for (auto i = path_.poses.begin(); i != path_.poses.end(); i++) {
        pa.poses.push_back((*i).pose);
    }
    pa.header.frame_id = "map";
    pa.header.stamp = ros::Time::now();
    pa.header.seq = 1;
    rvizPoseArray_pub.publish(pa);
    ros::spinOnce();
}

void QNode::pub_rvizMarker(visualization_msgs::Marker mark)
{
    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id = "map";
    marker_msg.ns = "path_number";
    marker_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_msg.scale.x = 0.8;
    marker_msg.scale.y = 0.8;
    marker_msg.scale.z = 0.8;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    marker_msg.text = "1";
    marker_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < 10; i++) {
        marker_msg.id = i;
        rvizMarker_pub.publish(marker_msg);
    }
//    rvizMarker_pub.publish(mark);
    ros::spinOnce();
}

void QNode::pub_multiNaviCmd(bool state_)
{
    std_msgs::Bool bl;
    bl.data = state_;
    multiNavi_pub.publish(bl);
    ros::spinOnce();
}

void QNode::pub_poseEstimate(geometry_msgs::PoseWithCovarianceStamped& pose_)
{
    pose_.header.frame_id = "map";
    pose_.header.stamp = ros::Time::now();
    poseEstimate_pub.publish(pose_);
    ros::spinOnce();
}

void QNode::pub_pathFiles(stringVector &sv)
{
    pathFiles_pub.publish(sv);
    ros::spinOnce();
}

void QNode::pub_electronicFence(geometry_msgs::Polygon fence)
{
    electronicFenceMaster_pub.publish(fence);
    ros::spinOnce();
}

void QNode::pub_initialpose(geometry_msgs::PoseWithCovarianceStamped pose)
{
    initialpose_pub.publish(pose);
    ros::spinOnce();
}

void QNode::currentPos_callback(const geometry_msgs::PoseWithCovarianceStamped &pos)
{
    // ROS_INFO_STREAM_THROTTLE(2, "pos.x = " << pos.pose.pose.position.x);
    QMutexLocker locker(m_mutex);
    robot->pose.header = pos.header;
    robot->pose.pose = pos.pose.pose;
}

void QNode::multiNaviResult_callback(std_msgs::UInt8 percentage)
{
    // qDebug() << "Rate of progress:" << percentage.data << "%";
    emit progress_signal(static_cast<int>(percentage.data));
}

void QNode::multiNaviRunTime_callback(std_msgs::Float64 time_sec)
{
    emit runTime_signal(static_cast<uint>(time_sec.data));
}

}  // namespace demo_smarco_robot
