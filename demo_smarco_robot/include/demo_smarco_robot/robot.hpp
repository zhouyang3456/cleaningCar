#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <QObject>
#include <geometry_msgs/PoseStamped.h>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace demo_smarco_robot {

class QNode;
class Robot
{
public:
    friend class QNode;
    friend class MainWindow;
    Robot();

/***********************************
**  Parameters Configuration
************************************/
private:
    std::string configYaml_dir; // Initial in qnode

    enum STATUS{
        FREE,
        RECORD_PATH,
        CLEANING
    };
    uint status = FREE;

    uint32_t id;
    QString server_addr;
    uint16_t server_port;
    float line_speed;
    float raw_speed;
    std::string floor;
    std::string filesDir;

    std::string recordPathJson_dir;
    std::string electronicFence_dir;

    double recordPath_distance = 4.0;
    double recordPath_angular = 3.14 / 8;
    geometry_msgs::PoseWithCovarianceStamped poseEstimate;

private:
    geometry_msgs::PoseStamped pose;   // Current pose
    QString battery_str = "84";
    nav_msgs::Path recordPath_rviz;
    nav_msgs::Path recordPath_;

private:
    void robot_init(std::string config_path);
    void write_config();
    void set_poseEstimate();
public:
    double distance(geometry_msgs::Pose a, geometry_msgs::Pose b);
    double angular(geometry_msgs::Pose a, geometry_msgs::Pose b);
};

}  // namespace demo_smarco_robot

#endif // ROBOT_HPP
