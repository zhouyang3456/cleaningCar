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

    floor = config["robot"]["floor"].as<string>();

    filesDir = config["robot"]["filesDir"].as<string>();
     recordPathJson_dir = filesDir + "/" + floor + "/recordPath/";
    electronicFence_dir = filesDir + "/" + floor + "/fence/";
    map_dir             = filesDir + "/" + floor + "/map/";
    initialPose_dir     = filesDir + "/" + floor + "/initialPose/";

    ftp_host = QString::fromStdString(config["ftp"]["host"].as<string>());
    ftp_port = config["ftp"]["port"].as<int>();
    ftp_user = QString::fromStdString(config["ftp"]["user"].as<string>());
    ftp_pwd  = QString::fromStdString(config["ftp"]["pwd"].as<string>());

    ftpUpload_dir  = config["ftp"]["uploadDir"].as<string>();
    ftpUpload_recordPathJson_dir =
            ftpUpload_dir + "/" + std::to_string(id) + "/" + floor + "/recordPath/";
    ftpUpload_electronicFence_dir =
            ftpUpload_dir + "/" + std::to_string(id) + "/" + floor + "/fence/";
    ftpUpload_map_dir =
            ftpUpload_dir + "/" + std::to_string(id) + "/" + floor + "/map/";
    ftpUpload_initialPose_dir =
            ftpUpload_dir + "/" + std::to_string(id) + "/" + floor + "/initialPose/";
}

void Robot::write_config()
{
    using namespace std;
    YAML::Node config = YAML::LoadFile(configYaml_dir);
    config["robot"]["line_speed"] = line_speed;
    config["robot"]["raw_speed"] = raw_speed;
    config["robot"]["floor"] = floor;

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
