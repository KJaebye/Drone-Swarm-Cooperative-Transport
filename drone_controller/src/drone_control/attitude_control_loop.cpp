#define wrap_180(x) (x < -M_PI ? x+(2*M_PI) : (x > M_PI ? x - (2*M_PI): x))
#include "attitude_control_loop.h"
namespace drone_control{

/////////////////////////////////////////////////////////////////////////////////
///////                 Attitude Controller          //////////////////////////
/////////////////////////////////////////////////////////////////////////////////
AttitudeController::AttitudeController() {}
AttitudeController::~AttitudeController() {}

void AttitudeController::InitializeParameters(const ros::NodeHandle& nh)
{
  //General parameters
  last_time = ros::Time::now();

  // angular loop PID
  double roll_PID_params[5] = {1, 4.5, 0, 0, 0};
  roll_PID.SetGainParameters(roll_PID_params);

  double pitch_PID_params[5] = {1, 4.5, 0, 0, 0};
  pitch_PID.SetGainParameters(pitch_PID_params);

  double yaw_PID_params[5] = {1, 1, 0, 0, 0};
  yaw_PID.SetGainParameters(yaw_PID_params);

  // angular rate loop PID
  double roll_rate_PID_params[5] = {1, 10, 0, 0.5, 0};//10, 0, 0.5
  roll_rate_PID.SetGainParameters(roll_rate_PID_params);

  double pitch_rate_PID_params[5] = {1, 10, 0, 0.5, 0};//10, 0, 0.5
  pitch_rate_PID.SetGainParameters(pitch_rate_PID_params);

  double yaw_rate_PID_params[5] = {1, 8, 0, 0, 0};
  yaw_rate_PID.SetGainParameters(yaw_rate_PID_params);

  //Control input to motor mapping
  //KT = 1.33e-05;
  //Kd = 1.39e-06;
  //l = 0.2;

  KT = 0.07;
  Kd = 0.0139;
  l = 20;


  //motor_lim = 8900*8900;
  motor_lim = 89*89;

  w1 = 0;
  w2 = 0;
  w3 = 0;
  w4 = 0;
}


void AttitudeController::CalculateAttitudeControl(
  mav_msgs::CommandRollPitchYawrateThrust control_cmd_input,
  sensor_msgs::Imu current_imu,
  Eigen::VectorXd* des_rate_output)
{
  //Determine vector size based on number of motors
  des_rate_output->resize(4);
  desired_angular_rates.resize(4);

  //Convert quaternion to Euler angles
  tf:quaternionMsgToTF(current_imu.orientation, q);
  tf::Matrix3x3(q).getRPY(meas_roll, meas_pitch, meas_yaw);
  ROS_DEBUG("RPY = (%lf, %lf, %lf)", meas_roll, meas_pitch, meas_yaw);

  // Get simulator time
  sim_time = ros::Time::now();
  dt = (sim_time - last_time).toSec();
  if (dt == 0.0) return;
  

  p_des = roll_PID.ComputeCorrection(control_cmd_input.roll, meas_roll, dt);
  q_des = pitch_PID.ComputeCorrection(control_cmd_input.pitch, meas_pitch, dt);
  r_des = - yaw_PID.ComputeCorrectionLimit(control_cmd_input.yaw_rate, meas_yaw, dt);

  r_des = controller_utility.limit(r_des, -50.0 * M_PI / 180.0, 50.0 * M_PI / 180.0);

/*
  //Roll PID
  roll_er = control_cmd_input.roll - meas_roll;
  cp = roll_er * roll_KP;
  cd = roll_KD * current_imu.angular_velocity.x;
  p_des = cp - cd;

  //Pitch PID
  pitch_er = control_cmd_input.pitch - meas_pitch;
  cd = pitch_KD * current_imu.angular_velocity.y;
  cp = pitch_er * pitch_KP;
  q_des = cp - cd;

  //Yaw PID
  yaw_er = wrap_180(control_cmd_input.yaw_rate - meas_yaw);
  cp = yaw_er * yaw_KP;
  cd = yaw_KD * current_imu.angular_velocity.z;
  r_des = -(cp - cd);
  r_des = controller_utility.limit(r_des, -50.0 * M_PI / 180.0, 50.0 * M_PI / 180.0);
*/
  desired_angular_rates[0] = control_cmd_input.thrust;
  desired_angular_rates[1] = p_des;
  desired_angular_rates[2] = q_des;
  desired_angular_rates[3] = r_des;
  *des_rate_output = desired_angular_rates;
  
}



void AttitudeController::CalculateRateControl(
  Eigen::VectorXd des_rate_input,
  sensor_msgs::Imu current_imu,
  Eigen::VectorXd* des_control_output_ptr)
{

  //Determine vector size based on number of motors
  des_control_output_ptr->resize(4);
  
  desired_control_cmds.resize(4);

  //Convert quaternion to Euler angles
  tf:quaternionMsgToTF(current_imu.orientation, q);
  tf::Matrix3x3(q).getRPY(meas_roll, meas_pitch, meas_yaw);
  ROS_DEBUG("RPY = (%lf, %lf, %lf)", meas_roll, meas_pitch, meas_yaw);

  //Get simulator time
  sim_time = ros::Time::now();
  dt = (sim_time - last_time).toSec();
  if (dt == 0.0) return;
  //ROS_INFO_STREAM(dt);

  U1 = des_rate_input[0];
  U2 = roll_rate_PID.ComputeCorrection(des_rate_input[1], current_imu.angular_velocity.x, dt);
  U3 = pitch_rate_PID.ComputeCorrection(des_rate_input[2], current_imu.angular_velocity.y, dt);
  U4 = yaw_rate_PID.ComputeCorrection(des_rate_input[3], -current_imu.angular_velocity.z, dt);
/*
  //P PID
  p_er = des_rate_input[1] - current_imu.angular_velocity.x;
  cp = p_er * p_KP;
  x_ang_acc = (current_imu.angular_velocity.x - last_ang_vel_x)/dt;
  cd = p_KD * x_ang_acc;
  U2 = cp - cd;
  last_ang_vel_x = current_imu.angular_velocity.x;

  //Q PID
  q_er = des_rate_input[2] - current_imu.angular_velocity.y;
  cp = q_er * q_KP;
  y_ang_acc = (current_imu.angular_velocity.y - last_ang_vel_y)/dt;
  cd = q_KD * y_ang_acc;
  U3 = cp - cd;
  last_ang_vel_y = current_imu.angular_velocity.y;


  //Yaw PID
  yaw_vel_target = des_rate_input[3];
  r_er = yaw_vel_target - (-current_imu.angular_velocity.z);
  cp = r_er * r_KP;
  z_ang_acc = (-current_imu.angular_velocity.z - last_ang_vel_z)/dt;
  cd = r_KD * z_ang_acc;
  U4 = cp - cd;
  last_ang_vel_z = current_imu.angular_velocity.z;
  */

  desired_control_cmds[0] = U1;
  desired_control_cmds[1] = U2;
  desired_control_cmds[2] = U3;
  desired_control_cmds[3] = U4;

  *des_control_output_ptr = desired_control_cmds;
  last_time = sim_time;

}
void AttitudeController::CalculateMotorCommands(
  Eigen::VectorXd control_inputs,
  Eigen::VectorXd* des_rotor_velocities_output)
{

  //Determine vector size based on number of motors
  desired_motor_velocities.resize(4);

  U1 = control_inputs[0];
  U2 = control_inputs[1];
  U3 = control_inputs[2];
  U4 = control_inputs[3];

  //Control input to motor mapping  
  // 十字形混控器
  /*w1 = U1/(4*KT) - U3/(2*KT*l) + U4/(4*Kd);
  w2 = U1/(4*KT) - U2/(2*KT*l) - U4/(4*Kd);
  w3 = U1/(4*KT) + U3/(2*KT*l) + U4/(4*Kd);
  w4 = U1/(4*KT) + U2/(2*KT*l) - U4/(4*Kd);*/

  // x字型混控器
  w1 = U1/(4*KT) + U2/(2*KT*l) -U3/(2*KT*l) + U4/(4*Kd);
  w2 = U1/(4*KT) + U2/(2*KT*l) +U3/(2*KT*l) - U4/(4*Kd);
  w3 = U1/(4*KT) - U2/(2*KT*l) +U3/(2*KT*l) + U4/(4*Kd);
  w4 = U1/(4*KT) - U2/(2*KT*l) -U3/(2*KT*l) - U4/(4*Kd);

  /*w1 = U1/(4*KT) + U2/(2*Kl) -U3/(2*Kl) + U4/(4*Kd);
  w2 = U1/(4*KT) + U2/(2*Kl) +U3/(2*Kl) - U4/(4*Kd);
  w3 = U1/(4*KT) - U2/(2*Kl) +U3/(2*Kl) + U4/(4*Kd);
  w4 = U1/(4*KT) - U2/(2*Kl) -U3/(2*Kl) - U4/(4*Kd);*/

  // Limit Based on Motor Parameters
  w1 = controller_utility.limit(w1, 0, motor_lim);
  w2 = controller_utility.limit(w2, 0, motor_lim);
  w3 = controller_utility.limit(w3, 0, motor_lim);
  w4 = controller_utility.limit(w4, 0, motor_lim);

  //Calculate motor speeds
  w1 = sqrt(w1);
  w2 = sqrt(w2);
  w3 = sqrt(w3);
  w4 = sqrt(w4);  

  desired_motor_velocities[0] = w1;
  desired_motor_velocities[1] = w2;
  desired_motor_velocities[2] = w3;
  desired_motor_velocities[3] = w4;

  *des_rotor_velocities_output = desired_motor_velocities;

}

}