/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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

class VelocityController
{
    public:
    VelocityController();
    ~VelocityController();

    void InitializeParameters(const ros::NodeHandle& nh);
    void CalculateVelocityControl(mav_msgs::CommandVelocityTrajectory vel_,
                                nav_msgs::Odometry current_gps,
                                mav_msgs::CommandVelocityTrajectory *des_acc_output);

    mav_msgs::CommandVelocityTrajectory des_acc_cmds;

    private:

    //General
    tf::Quaternion q;
    double gps_roll, gps_pitch, gps_yaw;
    double sin_yaw, cos_yaw;
    double gps_vel_x, gps_vel_y, gps_vel_z;

    ros::Time last_time;
    ros::Time sim_time;
    double dt;

    mav_msgs::CommandVelocityTrajectory transformed;
    ControllerUtility controller_utility;

    PID x_vel_PID, y_vel_PID, z_vel_PID;

    double x_vel_PID_params[4];
    double y_vel_PID_params[4];
    double z_vel_PID_params[4];


    /*//velocity Controller
    double x_vel_er, y_vel_er, z_vel_er;
    double x_vel_er_sum, y_vel_er_sum, z_vel_er_sum, yaw_vel_er_sum;
    double cp, ci, cd;

    //X PID
    double x_vel_KI_max;
    double x_vel_KP;
    double x_vel_KI;
    double x_vel_KD;

    double x_acc_des;
    double x_base_acc_des;
    double x_linear_acc;
    double last_linear_acc_x;

    //Y PID
    double y_vel_KI_max;
    double y_vel_KP;
    double y_vel_KI;
    double y_vel_KD;

    double y_acc_des;
    double y_base_acc_des;
    double y_linear_acc;
    double last_linear_acc_y;

    //Z PID
    double z_vel_KI_max;
    double z_vel_KP;
    double z_vel_KI;
    double z_vel_KD;
  
    double z_acc_des;
    double z_base_acc_des;
    double z_linear_acc;
    double last_linear_acc_z;
*/
    double roll_des, pitch_des, yaw_des, thrust_des;
    double x_acc_des, y_acc_des, z_acc_des;
    double x_base_acc_des, y_base_acc_des, z_base_acc_des;

    //double thrust_des;
    //double x_linear_acc, y_linear_acc, z_linear_acc;
};


}