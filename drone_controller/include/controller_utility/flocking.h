#include <ros/ros.h>
#include <mav_msgs/CommandTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <vector>

#include "PID.h"


namespace drone_control{

class FlockingController
{
    public:
    FlockingController();
    ~FlockingController();
    void InitializeParams();

    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);
    
    // pass params by pointer
    void CommandGenerate(std::vector<std::vector<double> > *pos);
    // pass params by const reference
    void CommandGenerate(const std::vector<std::vector<double> > &pos);
    std::vector<double> v_mig;
    double flocking_speed;
    double flocking_centroid[2];

    private:
    // set the threshold to define three areas
    // 1. Near Area; 2. Mid Area; 3. Far Area
    double near_distance;
    double far_distance;

    int object_num;
    double k_sep, k_coh, k_frict;
    double speed_max;
    std::vector<double> v_sep, v_coh, v_frict;
    std::vector<double> sep_sum, coh_sum;
    std::vector<std::vector<double> > pos_xyz;

    std::vector<double> assumption_pos;
    std::vector<double> relative_velocity;

    // in order to get the flocking height. this height is given by the flocking algorithm
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;
    std::string drone_id;
    std::string gps_topic_id;
    ros::Time sim_time;
    ros::Time last_time;
    double dt;
    
    double last_pos_x, last_pos_y, last_pos_z;

    void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

    PID level_pid;
    double level;
    double odometry_height;

    
    double Dot(std::vector<double> v1, std::vector<double> v2);

};




}