#include "robot.hpp"
#include <QDebug>
#include <fstream>
#include <qmath.h>
#include <tf/transform_broadcaster.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace demo_smarco_robot {

Robot::Robot()
{
    status = FREE;
    recordPath_.header.frame_id = "map";
    poseEstimate.header.frame_id = "map";
}

void Robot::robot_init(std::string config_path)
{
    using namespace std;
    YAML::Node config = YAML::LoadFile(config_path);
    id = config["robot"]["id"].as<uint32_t>();
    server_addr = QString::fromStdString(config["robot"]["server_addr"].as<string>());
    server_port = config["robot"]["server_port"].as<uint16_t>();
    line_speed = config["robot"]["line_speed"].as<float>();
    raw_speed = config["robot"]["raw_speed"].as<float>();
    poseEstimate.pose.pose.position.x =
            config["robot"]["pose_estimate"]["pos_x"].as<double>();
    poseEstimate.pose.pose.position.y =
            config["robot"]["pose_estimate"]["pos_y"].as<double>();
    poseEstimate.pose.pose.orientation.z =
            config["robot"]["pose_estimate"]["ori_z"].as<double>();
    poseEstimate.pose.pose.orientation.w =
            config["robot"]["pose_estimate"]["ori_w"].as<double>();
    floor = config["robot"]["floor"].as<string>();
    filesDir = config["robot"]["filesDir"].as<string>();

    // recordPathJson_dir = config["filePath"]["recordPathJson_dir"].as<string>();
    recordPathJson_dir = filesDir + "/" + floor + "/recordPath/";
    electronicFence_dir = filesDir + "/" + floor + "/fence/";
}

void Robot::write_config()
{
    using namespace std;
    YAML::Node config = YAML::LoadFile(configYaml_dir);
    config["robot"]["line_speed"] = line_speed;
    config["robot"]["raw_speed"] = raw_speed;

    config["robot"]["pose_estimate"]["pos_x"] =
                            poseEstimate.pose.pose.position.x;
    config["robot"]["pose_estimate"]["pos_y"] =
                            poseEstimate.pose.pose.position.y;
    config["robot"]["pose_estimate"]["ori_z"] =
                            poseEstimate.pose.pose.orientation.z;
    config["robot"]["pose_estimate"]["ori_w"] =
                            poseEstimate.pose.pose.orientation.w;

    std::fstream fs(configYaml_dir, std::fstream::out | std::fstream::trunc);
    fs << config;
    fs.close();
}

void Robot::set_poseEstimate()
{
    poseEstimate.pose.pose.position.x = pose.pose.position.x;
    poseEstimate.pose.pose.position.y = pose.pose.position.y;
    poseEstimate.pose.pose.orientation.z = pose.pose.orientation.z;
    poseEstimate.pose.pose.orientation.w = pose.pose.orientation.w;
    write_config();
}


double Robot::distance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
    double dis2 = qPow((a.position.x - b.position.x), 2) +
                  qPow((a.position.y - b.position.y), 2);
    return qSqrt(dis2);
}

double Robot::angular(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
    tf::Quaternion quat1;
    tf::quaternionMsgToTF(a.orientation, quat1);
    double roll1, pitch1, yaw1;
    tf::Matrix3x3(quat1).getRPY(roll1, pitch1, yaw1);

    tf::Quaternion quat2;
    tf::quaternionMsgToTF(b.orientation, quat2);
    double roll2, pitch2, yaw2;
    tf::Matrix3x3(quat2).getRPY(roll2, pitch2, yaw2);

    return qAbs(yaw1 - yaw2);
}

}  // namespace demo_smarco_robot
