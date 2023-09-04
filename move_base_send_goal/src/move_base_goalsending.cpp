/**
 * @file goalsengding.cpp
 * @author sifan
 * @brief
 * 循环发布目标点程序，从json文件读取目标点进行发布,从move_base反馈状态判断机器人是否达到目标点，四个点的目标循环，如需增加点，则修改goal_point的维度大小。
 *         或者将存放目标点的数组改为vector储存的形式，可以不限制点数量。
 * @version 0.1
 * @date 2022-08-26
 *
 * @copyright Copyright (c) 2022
 *
 */
// #include "move_base_goalsending.h"
#include "../include/move_base_goalsending.h"
#include <iostream>
#include <csignal>
#include <std_msgs/UInt8.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>

GoalSending::GoalSending() :
    ac_("move_base", true),
    runStatus(Status::FREE),
    initialized_(false)
{
    // ros::param::get("~points_path_", points_path_);
    // ros::param::get("~recordPath_dir", recordPath_dir);

    // Start Timer to send point
    timer_ = nh_.createTimer(ros::Duration(0.3), &GoalSending::goalPointPub_loop, this, false, false);
    timer_pubRunTime = nh_.createTimer(ros::Duration(1), &GoalSending::runTime_pub, this, false, false);

    control_sub = nh_.subscribe("/multiPoints_navi/cmd", 1, &GoalSending::controlSub_callback, this);
    pathFiles_sub = nh_.subscribe("/multiPoints_navi/pathFiles", 1, &GoalSending::pathFilesSub_callback, this);

    runTime_pub_ = nh_.advertise<std_msgs::Float64>("/multiPoints_navi/runTime", 1);
    result_pub_ = nh_.advertise<std_msgs::UInt8>("/multiPoints_navi/result", 1);
    rvizPoseArray_pub = nh_.advertise<geometry_msgs::PoseArray>("/multiPoints_navi/rviz/poseArray", 1);
    rvizPath_pub = nh_.advertise<nav_msgs::Path>("/record_path/rviz_path", 1);
    rvizMarker_pub = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
}

void GoalSending::openFile(std::string m_path_)
{
    goal_point_.clear();
    total_count = 0;

    // Read Json Data
    Json::Reader reader_;
    Json::Value root_;
    std::ifstream in_;

    in_.open(m_path_, std::ios::binary);
    if (!in_.is_open()) {
        std::cout << "Error opening file\n";
        return;
    }
    if (reader_.parse(in_, root_)) {
        total_count = root_["goalname"].size();
        geometry_msgs::PoseStamped tempPose;
        for (uint i = 0; i < root_["goalname"].size(); i++) {
            geometry_msgs::Pose tmp;
            tmp.position.x = root_["goalname"][i]["location"][0].asDouble();
            tmp.position.y = root_["goalname"][i]["location"][1].asDouble();
            tmp.orientation.z = root_["goalname"][i]["location"][2].asDouble();
            tmp.orientation.w = root_["goalname"][i]["location"][3].asDouble();
            goal_point_.push_back(tmp);
        }
    }
    in_.close();
}

void GoalSending::openFiles(const demo_smarco_robot::stringVector &filesVect)
{
    total_count = 0;
    goal_point_.clear();

    // Read Json Data
    Json::Reader reader_;
    Json::Value root_;
    std::ifstream in_;

    for (auto i = filesVect.strings.begin(); i != filesVect.strings.end(); i++) {
        std::string fileRealPath = *i;
        in_.open(fileRealPath, std::ios::binary);
        if (!in_.is_open()) {
            std::cout << "Error opening file: " << fileRealPath << std::endl;
            continue;
        }
        if (reader_.parse(in_, root_)) {
            total_count += root_["goalname"].size();
            for (uint j = 0; j < root_["goalname"].size(); j++) {
                geometry_msgs::Pose tmp;
                tmp.position.x= root_["goalname"][j]["location"][0].asDouble();
                tmp.position.y = root_["goalname"][j]["location"][1].asDouble();
                tmp.orientation.z = root_["goalname"][j]["location"][2].asDouble();
                tmp.orientation.w = root_["goalname"][j]["location"][3].asDouble();
                goal_point_.push_back(tmp);
            }
        }
        in_.close();
    }
}

void GoalSending::pubPoint(uint seq)
{
    target_pose_.header.seq = seq + 1;
    target_pose_.header.frame_id = "map";
    target_pose_.header.stamp = ros::Time::now();
    target_pose_.pose.position.x = goal_point_.at(count).position.x;
    target_pose_.pose.position.y = goal_point_.at(count).position.y;
    target_pose_.pose.orientation.z = goal_point_.at(count).orientation.z;
    target_pose_.pose.orientation.w = goal_point_.at(count).orientation.w;
    goal_.target_pose = target_pose_;
    if (!ac_.waitForServer(ros::Duration(60))) {
      ROS_INFO("Can't connected to move base server");
    }
    ac_.sendGoal(goal_, boost::bind(&GoalSending::doneCb, this, _1, _2),
                 boost::bind(&GoalSending::activeCb, this),
                 boost::bind(&GoalSending::feedbackCb, this, _1));

    recovery_time = ros::Time::now().toSec();
}

void GoalSending::doneCb(const actionlib::SimpleClientGoalState& state,
                         const move_base_msgs::MoveBaseResultConstPtr& result) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("SUCCEEDED");
    }
    else if(state == actionlib::SimpleClientGoalState::PREEMPTED) {
        // ROS_INFO("PREEMPTED");
    }
}

void GoalSending::activeCb() {
  ROS_INFO("Goal Received, count %d", count + 1);
}

void GoalSending::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    // ROS_INFO_STREAM(ac_.getState().toString());
}

void GoalSending::recovery_handler()
{
    // If reach the last point, send first point
    if (++count == total_count) {
        if (runMode == Mode::SINGLE) {
            timer_.stop();
            ac_.cancelAllGoals();
            ROS_WARN("Finish path points with SINGLE mode");
            return;
        }
        initialized_ = false;
        count = 0;
        return;
    }
    pubPoint(count);
    ROS_INFO("Recovery handler ~");
}

void GoalSending::rvizPub_poseArray(const demo_smarco_robot::stringVector &filesVect)
{
    geometry_msgs::PoseArray poseArray_rviz;
    poseArray_rviz.poses.clear();
    poseArray_rviz.header.frame_id = "map";

    // Read Json Data
    Json::Reader reader_;
    Json::Value root_;
    std::ifstream in_;

    for (auto i = filesVect.strings.begin(); i != filesVect.strings.end(); i++) {
        std::string fileRealPath = *i;
        in_.open(fileRealPath, std::ios::binary);
        if (!in_.is_open()) {
            std::cout << "Error opening file: " << fileRealPath << std::endl;
            continue;
        }
        if (reader_.parse(in_, root_)) {
            for (uint j = 0; j < root_["goalname"].size(); j++) {
                geometry_msgs::Pose tmp;
                tmp.position.x= root_["goalname"][j]["location"][0].asDouble();
                tmp.position.y = root_["goalname"][j]["location"][1].asDouble();
                tmp.orientation.z = root_["goalname"][j]["location"][2].asDouble();
                tmp.orientation.w = root_["goalname"][j]["location"][3].asDouble();
                poseArray_rviz.poses.push_back(tmp);
            }
        }
        in_.close();
    }

    poseArray_rviz.header.stamp = ros::Time::now();
    poseArray_rviz.header.seq = 1;
    rvizPoseArray_pub.publish(poseArray_rviz);
    ros::spinOnce();
}

void GoalSending::rvizPub_path(const demo_smarco_robot::stringVector &filesVect)
{
    nav_msgs::Path rvizPath;
    rvizPath.poses.clear();
    rvizPath.header.frame_id = "map";

    // Read Json Data
    Json::Reader reader_;
    Json::Value root_;
    std::ifstream in_;

    uint pathNum = 1;
    geometry_msgs::PoseStamped tempPose;
    tempPose.header.frame_id = "map";
    for (auto i = filesVect.strings.begin(); i != filesVect.strings.end(); i++) {
        std::string fileRealPath = *i;
        in_.open(fileRealPath, std::ios::binary);
        if (!in_.is_open()) {
            std::cout << "Error opening file: " << fileRealPath << std::endl;
            continue;
        }
        rvizPath.poses.clear();
        if (reader_.parse(in_, root_)) {
            for (uint j = 0; j < root_["goalname"].size(); j++) {
                geometry_msgs::Pose tmp;
                tmp.position.x= root_["goalname"][j]["location"][0].asDouble();
                tmp.position.y = root_["goalname"][j]["location"][1].asDouble();
                tmp.orientation.z = root_["goalname"][j]["location"][2].asDouble();
                tmp.orientation.w = root_["goalname"][j]["location"][3].asDouble();
                tempPose.pose = tmp;
                rvizPath.poses.push_back(tempPose);
            }
        }
        in_.close();
        rvizPath.header.stamp = ros::Time::now();
        rvizPath.header.seq = pathNum;
        rvizPath_pub.publish(rvizPath);
        pathNum++;
    }

    ros::spinOnce();
}

void GoalSending::rvizPub_marker(const demo_smarco_robot::stringVector &filesVect)
{
    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id = "map";
    marker_msg.ns = "path_number";
    marker_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker_msg.scale.x = 0.5;
    marker_msg.scale.y = 0.5;
    marker_msg.scale.z = 0.5;

    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;

    // Read Json Data
    Json::Reader reader_;
    Json::Value root_;
    std::ifstream in_;

    int pathNum = 1;
    for (auto i = filesVect.strings.begin(); i != filesVect.strings.end(); i++) {
        std::string fileRealPath = *i;
        in_.open(fileRealPath, std::ios::binary);
        if (!in_.is_open()) {
            std::cout << "Error opening file: " << fileRealPath << std::endl;
            continue;
        }
        if (reader_.parse(in_, root_)) {
            for (uint j = 0; j < root_["goalname"].size(); j++) {
                geometry_msgs::Pose tmp;
                tmp.position.x= root_["goalname"][j]["location"][0].asDouble();
                tmp.position.y = root_["goalname"][j]["location"][1].asDouble();
                tmp.orientation.z = root_["goalname"][j]["location"][2].asDouble();
                tmp.orientation.w = root_["goalname"][j]["location"][3].asDouble();
                if (j == 0)
                    marker_msg.pose = tmp;
            }
        }
        in_.close();
        marker_msg.id = pathNum;
        marker_msg.text = "PATH_" + std::to_string(pathNum);
        marker_msg.header.stamp = ros::Time::now();
        rvizMarker_pub.publish(marker_msg);
        pathNum++;
    }

    ros::spinOnce();
}

void GoalSending::controlSub_callback(const std_msgs::Bool &cmd)
{
    if (cmd.data == true) {
        // Read points from files
        // this->openFile(points_path_);
        initialized_ = false;
        total_count = 0;
        count = 0;
        timer_.start();
        ROS_INFO("Starting success, Total count = %d", total_count);
    } else if (cmd.data == false) {
        timer_.stop();
        ac_.cancelAllGoals();
        timer_pubRunTime.stop();
        startTime = 0;
        runStatus = Status::FREE;
        ROS_INFO("Stop, cancel all goals");
    }
}

void GoalSending::pathFilesSub_callback(const demo_smarco_robot::stringVector &filesVect)
{
    this->openFiles(filesVect);
    rvizPub_poseArray(filesVect);
    rvizPub_path(filesVect);
    rvizPub_marker(filesVect);

    if (filesVect.mode == filesVect.SINGLE)
        runMode = Mode::SINGLE;
    if (filesVect.mode == filesVect.LOOP)
        runMode = Mode::LOOP;

    initialized_ = false;
    count = 0;
    timer_.start();
    runStatus = Status::RUNNING;
    startTime = ros::Time::now().toSec();
    timer_pubRunTime.start();
    ROS_INFO("Starting success, Total count = %d", total_count);
}

void GoalSending::runTime_pub(const ros::TimerEvent &event)
{
    double runningTimes = ros::Time::now().toSec() - startTime;
    std_msgs::Float64 f_runTime;
    f_runTime.data = runningTimes;
    runTime_pub_.publish(f_runTime);
}

void GoalSending::goalPointPub_loop(const ros::TimerEvent& event) {
    // Send first point
    if (!initialized_) {
        pubPoint(count);
        initialized_ = true;
    }

    // If preempted, send the current point again
    if (ac_.getState() == actionlib::SimpleClientGoalState::PREEMPTED ||
        ac_.getState() == actionlib::SimpleClientGoalState::RECALLED  ||
        ac_.getState() == actionlib::SimpleClientGoalState::ABORTED   ||
        ac_.getState() == actionlib::SimpleClientGoalState::REJECTED  ||
        ac_.getState() == actionlib::SimpleClientGoalState::LOST) {
        pubPoint(count);
        ROS_WARN("No preemption");
    }

    // If goal SUCCEEDED, send next point
    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        count++;
        std_msgs::UInt8 progress;
        progress.data = count * 100.0 / total_count;
        result_pub_.publish(progress);
        // If reach the last point, send first point
        if (count == total_count) {
            if (runMode == Mode::SINGLE) {
                timer_.stop();
                ac_.cancelAllGoals();
                ROS_WARN("Finish path points with SINGLE mode");
                return;
            }
            initialized_ = false;
            count = 0;
            return;
        }
        pubPoint(count);
    }

    if ((ros::Time::now().toSec() - recovery_time) > RECOVERY_INTERVAL) {
        recovery_handler();
    }
}

GoalSending::~GoalSending() {
  timer_.stop();
  ac_.cancelAllGoals();
}

static bool RUNNING_FLAG = true;
void Get_CtrlC_handler(int sig) {
    // signal(sig, SIG_IGN);
    RUNNING_FLAG = false;
    // exit(0);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "goalsending");
  GoalSending gs;
  signal(SIGINT, Get_CtrlC_handler);
  while (ros::ok() && RUNNING_FLAG) {
      ros::spinOnce();
  }
  // ros::spin();
  return 0;
}
