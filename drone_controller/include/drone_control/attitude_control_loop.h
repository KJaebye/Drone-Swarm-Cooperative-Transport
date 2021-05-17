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

class AttitudeController
{
    public:
    AttitudeController();
    ~AttitudeController();

    void InitializeParameters(const ros::NodeHandle& nh);
    void CalculateAttitudeControl(mav_msgs::CommandRollPitchYawrateThrust control_cmd_input, sensor_msgs::Imu current_imu, Eigen::VectorXd* des_rate_output);
    void CalculateRateControl(Eigen::VectorXd des_rate_input, sensor_msgs::Imu current_imu, Eigen::VectorXd* des_control_output);
    void CalculateMotorCommands(Eigen::VectorXd control_inputs, Eigen::VectorXd* des_rotor_velocities_output);

    mav_msgs::CommandRollPitchYawrateThrust current_control_cmd_;
    sensor_msgs::Imu current_imu_;
    Eigen::VectorXd desired_angular_rates;
    Eigen::VectorXd desired_control_cmds;
    Eigen::VectorXd desired_motor_velocities;

    private:

    //General
    tf::Quaternion q;
    double meas_roll, meas_pitch, meas_yaw;

    ros::Time last_time;
    ros::Time sim_time;
    double dt;

    ControllerUtility controller_utility;
    PID roll_PID, pitch_PID, yaw_PID;
    PID roll_rate_PID, pitch_rate_PID, yaw_rate_PID;

    double roll_PID_params[4];
    double pitch_PID_params[4];
    double yaw_PID_params[4];

    double p_des, q_des, r_des;

    double roll_rate_PID_params[4];
    double pitch_rate_PID_params[4];
    double yaw_rate_PID_params[4];

    double U1, U2, U3, U4;

    /*//Attitude Controller
    double roll_er, pitch_er, yaw_er;
    double roll_er_sum, pitch_er_sum, yaw_er_sum;
    double cp, ci, cd;

    //Roll PID
    double roll_KI_max;
    double roll_KP;
    double roll_KI;
    double roll_KD;

    //Pitch PID
    double pitch_KI_max;
    double pitch_KP;
    double pitch_KI;
    double pitch_KD;

    //Yaw PID
    double yaw_KI_max;
    double yaw_KP;
    double yaw_KI;
    double yaw_KD;
    double yaw_target;

    double p_des, q_des, r_des;

    //Rate Controller
    double p_er, q_er, r_er;
    double p_er_sum, q_er_sum, r_er_sum;
  
    //P Controller
    double p_KI_max;
    double p_KP;
    double p_KI;
    double p_KD;
    double x_ang_acc;
    double last_ang_vel_x;

    //Q Controller
    double q_KI_max;
    double q_KP;
    double q_KI;
    double q_KD;
    double y_ang_acc;
    double last_ang_vel_y;

    //R Controller
    double r_KI_max;
    double r_KP;
    double r_KI;
    double r_KD;
    double yaw_vel_target;
    double z_ang_acc;
    double last_ang_vel_z;
*/
    

    //Motor Mapping 
    double KT;
    double Kd;
    double Kl;
    double l;
    double motor_lim;
    double w1, w2, w3, w4;
    double s;

};
}


