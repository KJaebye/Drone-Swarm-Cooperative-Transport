#include "position_control_loop.h"
namespace drone_control{

/////////////////////////////////////////////////////////////////////////
//////////                Position Controller             ///////////////
/////////////////////////////////////////////////////////////////////////

PositionController::PositionController() {}
PositionController::~PositionController() {}


void PositionController::InitializeParameters(const ros::NodeHandle& nh)
{
  //General parameters
  last_time = ros::Time::now();

  double x_PID_params[] = {1, 1, 0.001, 0.0001, 0.1};//1, 0.001, 0.0001, 0.1
  x_PID.SetGainParameters(x_PID_params);

  double y_PID_params[] = {1, 1, 0.001, 0.0001, 0.1};//1, 0.001, 0.0001, 0.1
  y_PID.SetGainParameters(y_PID_params);


  //double z_PID_params[] = {1, 1, 0.05, 0.0001, 0};// for non-dynamic case
  double z_PID_params[] = {1, 1, 0.03, 0.0001, 0};// for dynamic case 
  z_PID.SetGainParameters(z_PID_params);

}

void PositionController::CalculatePositionControl(
  mav_msgs::CommandTrajectory wp,
  nav_msgs::Odometry current_gps,
  mav_msgs::CommandVelocityTrajectory *des_velocity_output)
{
  //Convert quaternion to Euler angles
  tf:quaternionMsgToTF(current_gps.pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(gps_roll, gps_pitch, gps_yaw);
  ROS_DEBUG("RPY = (%lf, %lf, %lf)", gps_roll, gps_pitch, gps_yaw);

  // Get simulator time
  sim_time = ros::Time::now();
  dt = (sim_time - last_time).toSec();
  if (dt == 0.0) return;
  
  gps_x = current_gps.pose.pose.position.x;
  gps_y = current_gps.pose.pose.position.y;
  gps_z = current_gps.pose.pose.position.z;

  //x, y, z PID
  x_vel_des = x_PID.ComputeCorrection(wp.position.x, current_gps.pose.pose.position.x, dt);
  y_vel_des = y_PID.ComputeCorrection(wp.position.y, current_gps.pose.pose.position.y, dt);
  z_vel_des = z_PID.ComputeCorrection(wp.position.z, current_gps.pose.pose.position.z, dt);
  
  x_vel_des = controller_utility_.limit(x_vel_des, -1, 1);
  y_vel_des = controller_utility_.limit(y_vel_des, -1, 1);
  //z_vel_des = controller_utility_.limit(z_vel_des, -1, 1);

  //Yaw PID
  // In this part, do not calcuate the yaw_rate, sent the yaw information into the attitude control loop
  // and calculate in that loop.

  des_velocity_cmds.velocity.x = x_vel_des;	
  des_velocity_cmds.velocity.y = y_vel_des;
  des_velocity_cmds.velocity.z = z_vel_des;
  //this is not yaw_rate, it is yaw
  //des_velocity_cmds.yaw_rate = yaw_des; sent this information to the attitude control loop directly
  des_velocity_cmds.yaw_rate = wp.yaw;
  *des_velocity_output = des_velocity_cmds;
  last_time = sim_time;
}

}
