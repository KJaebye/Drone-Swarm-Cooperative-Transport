#include <ros/ros.h>
#include <mav_msgs/CommandTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <time.h>

namespace drone_control{

class RandomWayPointFlyingController
{
    public:
    RandomWayPointFlyingController();
    ~RandomWayPointFlyingController();

    void InitializeParams();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);

    private:
    ros::NodeHandle nh;
    ros::Publisher random_waypoint_pub;
    ros::Timer timer;
    std::string waypoint_topic_id;

    double space_size_x, space_size_y;
    double wp_x, wp_y, wp_z;
    int random_seed;

    mav_msgs::CommandTrajectory random_waypoint;
    void TimerCallback(const ros::TimerEvent&);
};



}