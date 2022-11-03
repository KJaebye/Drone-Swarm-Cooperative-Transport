#include "acceleration_control_loop.h"
#define GRAVITY 9.81

/////////////////////////////////////////////////////////////////////////
//////////              Acceleration Controller             ///////////////
////////////////////////////////////////////////////////////////////////

namespace drone_control{

AccelerationController::AccelerationController() {}
AccelerationController::~AccelerationController() {}

void AccelerationController::InitializeParameters(const ros::NodeHandle& nh)
{
    last_time = ros::Time::now();
    double x_acc_PID_params[] = {1, 1, 0, 0, 0};//1
    x_acc_PID.SetGainParameters(x_acc_PID_params);

    double y_acc_PID_params[] = {1, 1, 0, 0, 0};//1
    y_acc_PID.SetGainParameters(y_acc_PID_params);

    double z_acc_PID_params[] = {1, 0.8, 0, 0, 200};
    z_acc_PID.SetGainParameters(z_acc_PID_params);

}

void AccelerationController::CalculateAccelerationControl(mav_msgs::CommandVelocityTrajectory acc,
                                                        nav_msgs::Odometry current_gps,
                                                        sensor_msgs::Imu current_imu,
                                                        mav_msgs::CommandRollPitchYawrateThrust *des_attitude_output_ptr)
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

    // calculate the acceleration to base frame
    x_base_acc  =  cos_yaw * acc.acceleration.x  + sin_yaw * acc.acceleration.y;
    y_base_acc =  - sin_yaw * acc.acceleration.x  + cos_yaw * acc.acceleration.y;
    z_base_acc = acc.acceleration.z;

    //this value is equal to 7.9426 in this simulator under this mixer;

    roll_des = - controller_utility.limit(atan(y_base_acc / GRAVITY), -0.26, 0.26);
    pitch_des = controller_utility.limit(atan(x_base_acc / GRAVITY), -0.26, 0.26);
    yaw_des = acc.yaw_rate;

    thrust_des = z_acc_PID.ComputeCorrection(z_base_acc, current_imu.linear_acceleration.z + GRAVITY, dt);
    thrust_des = thrust_des + 7.9426;
    
    
    //thrust_des = z_base_acc + gravity;

    //thrust_des = controller_utility.limit(thrust_des, 2.75, 20);

    des_attitude_cmds.roll = roll_des;	
    des_attitude_cmds.pitch = pitch_des;
    des_attitude_cmds.yaw_rate = yaw_des;
    des_attitude_cmds.thrust = thrust_des;

    *des_attitude_output_ptr = des_attitude_cmds;
    last_time = sim_time;
}

}