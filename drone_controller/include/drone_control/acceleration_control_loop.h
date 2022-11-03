#include <Eigen/Eigen>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/CommandMotorSpeed.h>
#include <mav_msgs/CommandRollPitchYawrateThrust.h>
#include <mav_msgs/CommandVelocityTrajectory.h>
#include <mav_msgs/MotorSpeed.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <planning_msgs/WayPoint.h>
#include <planning_msgs/eigen_planning_msgs.h>
#include <planning_msgs/conversions.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "controller_utility.h"
#include "PID.h"

namespace drone_control{

class AccelerationController
{
    public:

    AccelerationController();
    ~AccelerationController();

    void InitializeParameters(const ros::NodeHandle& nh);
    void CalculateAccelerationControl(mav_msgs::CommandVelocityTrajectory acc,
                                    nav_msgs::Odometry current_gps,
                                    sensor_msgs::Imu current_imu,
                                    mav_msgs::CommandRollPitchYawrateThrust *des_attitude_output);
    
    
    private:

    //General
    tf::Quaternion q;
    double gps_roll, gps_pitch, gps_yaw;
    double sin_yaw, cos_yaw;
    double gps_vel_x, gps_vel_y, gps_vel_z;
    
    ros::Time last_time;
    ros::Time sim_time;
    double dt;

    ControllerUtility controller_utility;
    PID x_acc_PID, y_acc_PID, z_acc_PID;

    double x_acc_PID_params[4];
    double y_acc_PID_params[4];
    double z_acc_PID_params[4];

    double x_base_acc, y_base_acc, z_base_acc;
    double roll_des, pitch_des, yaw_des, thrust_des;

    mav_msgs::CommandRollPitchYawrateThrust des_attitude_cmds;

};


}