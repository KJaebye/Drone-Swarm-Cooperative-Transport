#include "velocity_control_loop.h"

namespace drone_control{

///////////////////////////////////////////////////////////////////////////////
///////                  Velocity Controller          /////////////////////////
/////////////////////////////////////////////////////////////////////////////
VelocityController::VelocityController() {}
VelocityController::~VelocityController() {}

void VelocityController::InitializeParameters(const ros::NodeHandle& nh)
{
  last_time = ros::Time::now();

  double x_vel_PID_params[] = {1, 2, 0, 0, 0};
  x_vel_PID.SetGainParameters(x_vel_PID_params);

  double y_vel_PID_params[] = {1, 2, 0, 0, 0};
  y_vel_PID.SetGainParameters(y_vel_PID_params);

  double z_vel_PID_params[] = {1, 5, 0, 0, 0};
  z_vel_PID.SetGainParameters(z_vel_PID_params);


}

void VelocityController::CalculateVelocityControl(
  mav_msgs::CommandVelocityTrajectory vel_,
  nav_msgs::Odometry current_gps,
  mav_msgs::CommandVelocityTrajectory *des_acc_output)
{
  //Convert quaternion to Euler angles
  tf:quaternionMsgToTF(current_gps.pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(gps_roll, gps_pitch, gps_yaw);
  ROS_DEBUG("RPY = (%lf, %lf, %lf)", gps_roll, gps_pitch, gps_yaw);
  sin_yaw = sin(gps_yaw);
  cos_yaw = cos(gps_yaw);
  // Get simulator time
  sim_time = ros::Time::now();
  dt = (sim_time - last_time).toSec();
  if (dt == 0.0) return;

  gps_vel_x = current_gps.twist.twist.linear.x;
  gps_vel_y = current_gps.twist.twist.linear.y;
  gps_vel_z = current_gps.twist.twist.linear.z;


  x_acc_des = x_vel_PID.ComputeCorrection(vel_.velocity.x, current_gps.twist.twist.linear.x, dt);
  y_acc_des = y_vel_PID.ComputeCorrection(vel_.velocity.y, current_gps.twist.twist.linear.y, dt);
  z_acc_des = z_vel_PID.ComputeCorrection(vel_.velocity.z, current_gps.twist.twist.linear.z, dt);

  x_acc_des = controller_utility.limit(x_acc_des, -1, 1);
  y_acc_des = controller_utility.limit(y_acc_des, -1, 1);
  //z_acc_des = controller_utility.limit(z_acc_des, -1, 1);


  des_acc_cmds.acceleration.x = x_acc_des;	
  des_acc_cmds.acceleration.y = y_acc_des;
  des_acc_cmds.acceleration.z = z_acc_des;
  des_acc_cmds.yaw_rate = vel_.yaw_rate;

  *des_acc_output = des_acc_cmds;
  last_time = sim_time;
}
}
