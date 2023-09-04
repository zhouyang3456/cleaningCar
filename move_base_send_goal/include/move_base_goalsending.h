/**
 * @file goalsending.h
 * @author sifan
 * @brief  循环发布目标点程序，从json文件读取目标点进行发布,从move_base反馈接受机器人当前位姿
 * @version 0.1
 * @date 2022-08-26
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef GOAL_SENDING_H
#define GOAL_SENDING_H

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsoncpp/json/json.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <fstream>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include "demo_smarco_robot/stringVector.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/Time.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class GoalSending {
 private:
  // ros
  ros::NodeHandle nh_;
  ros::Publisher goal_pub_;
  ros::Publisher result_pub_;

  ros::Subscriber control_sub;
  void controlSub_callback(const std_msgs::Bool & cmd);
  ros::Subscriber pathFiles_sub;
  void pathFilesSub_callback(const demo_smarco_robot::stringVector & filesVect);

  ros::Publisher rvizPath_pub;
  ros::Publisher rvizPoseArray_pub;
  ros::Publisher rvizMarker_pub;

  ros::Timer timer_;
  geometry_msgs::PoseStamped target_pose_;
  move_base_msgs::MoveBaseGoal goal_;
  // Create MoveBaseAction Object
  Client ac_;

  enum Status {
      FREE,
      RUNNING,
      SUSPENDED
  };
  Status runStatus;
  enum Mode {
      SINGLE,
      LOOP
  };
  Mode runMode;

  bool initialized_;
  uint total_count = 0;
  uint count = 0;
  const double RECOVERY_INTERVAL = 100; // Seconds
  double recovery_time;

  double startTime = 0;
  ros::Publisher runTime_pub_;
  ros::Timer timer_pubRunTime;
  void runTime_pub(const ros::TimerEvent& event);

  //  std::string points_path_;
  //  std::string recordPath_dir;

  // 存放目标点的数组
  // double goal_point_[4][2];
  std::vector<geometry_msgs::Pose> goal_point_;
  /**
   * @brief 发布目标点
   */
  void goalPointPub_loop(const ros::TimerEvent& event);
  /**
   * @brief 打开Json文件，读取目标点,赋值给goal_point_数组
   */
  void openFile(std::string m_path_);
  void openFiles(const demo_smarco_robot::stringVector &filesVect);
  void pubPoint(uint seq);
  // MoveBaseAction callback
  void activeCb();
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const move_base_msgs::MoveBaseResultConstPtr& result);
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
  void recovery_handler();

  void rvizPub_poseArray(const demo_smarco_robot::stringVector &filesVect);
  void rvizPub_path(const demo_smarco_robot::stringVector &filesVect);
  void rvizPub_marker(const demo_smarco_robot::stringVector &filesVect);

 public:
  GoalSending();
  ~GoalSending();
};

#endif  // GOAL_SENDING_H
