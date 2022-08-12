/*
 * Copyright 2015 Wil Selby
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

#include "drone.h"
namespace drone_control{

Drone::Drone()
{
    InitializeParams();
    //publishers for test
    vel_pub = nh.advertise<mav_msgs::CommandVelocityTrajectory>("/command/linear_velocity", 10);
    roll_pitch_yaw_thrust_pub = nh.advertise<mav_msgs::CommandRollPitchYawrateThrust>("/command/attitude", 10);
    acc_pub = nh.advertise<mav_msgs::CommandVelocityTrajectory>("/command/linear_accelerater", 10);
}

Drone::~Drone(){}

void Drone::InitializeParams()
{

    position_controller.InitializeParameters(nh);
    velocity_controller.InitializeParameters(nh);
    attitude_controller.InitializeParameters(nh);
    acceleration_controller.InitializeParameters(nh);
    
    command_wp_msg.position.x=0;
    command_wp_msg.position.y=0;
    command_wp_msg.position.z=1.5;
    command_wp_msg.yaw=0;

    bacterium_v[0] = 0;
    bacterium_v[1] = 0;
    bacterium_v[2] = 0;

    //flocking_situation = false;
    bacterium_situation = false;
    flocking_situation = true;
    //bacterium_situation = true;

    destination_situation = false;
    liftland_situation = false;

    G_b = 0.5;// gain of bacterium
    G_f = 0.5;// gain of flocking
    G_f_t = 0.05;//0.15

    double gain = 2;
    G_b = G_b * gain;
    G_f = G_f * gain;
    G_f_t = G_f_t * gain;

    destination_x = -1.8;//25
    destination_y = 0;//0

    counter = 0;
    error[0] = 0;
    error[1] = 0;
    dispersion = 1.25;
}


//////////////////////////////////////////////////////////////////////////////////////////////
/////////////////// Specialization Processing ////////////////////////////////////////////////

void Drone::SpecializeFirstDroneParams()
{
    //specialize the name of this object
    drone_id = "Quadricopter";
    // drone controller topic id
    waypoint_topic_id = drone_id + "/command/way_point";
    motor_speed_topic_id = drone_id + "/command/motor_speed";
    // swarm flocking situation topic id
    flocking_topic_id = drone_id + "/command/flocking";
    // bacterium situation topic id
    bacterium_topic_id = drone_id + "/command/bacterium";
    // situation topic id
    destination_topic_id = drone_id + "/command/destination";
    liftland_topic_id = drone_id + "/command/liftland";
  
    gps_topic_id = drone_id + "/sensor/gps";
    imu_topic_id = drone_id + "/sensor/imu";
 
    //getting sensors msg
    gps_sub = nh.subscribe(gps_topic_id, 1000, &Drone::OdometryCallback, this);
    imu_sub = nh.subscribe(imu_topic_id, 1000, &Drone::IMUCallback, this);

    // drone control commands in
    command_trajectory_sub = nh.subscribe(waypoint_topic_id, 10, &Drone::CommandTrajectoryCallback, this);
    // drone control commands out
    command_motor_speed_pub = nh.advertise<mav_msgs::CommandMotorSpeed>(motor_speed_topic_id, 10);

    // swarm situation commands in
    command_flocking_sub = nh.subscribe(flocking_topic_id, 10, &Drone::CommandFlockingCallback, this);
    // bacterium situation commands in
    command_bacterium_sub = nh.subscribe(bacterium_topic_id, 10, &Drone::CommandBacteriumCallback, this);

    // situation commands in
    command_destination_sub = nh.subscribe(destination_topic_id, 10, &Drone::CommandDestinationCallback, this);
    command_liftland_sub = nh.subscribe(liftland_topic_id, 10, &Drone::CommandLiftLandCallback, this);

    // vacuum and camera parameters specialize
    vacuum.SpecializeFirstDroneParams();
    //vision_sensor_360.SpecializeFirstDroneParams(); //(discard now)
    vision_sensor_panoramic.SpecializeFirstDroneParams();
    //random_wp_flying_controller.SpecializeFirstDroneParams();
    random_velocity_flying_controller.SpecializeFirstDroneParams();

    ultrasonic_sensor.SpecializeFirstDroneParams();// this is for distance concentration
    //vision_sensor_down.SpecializeFirstDroneParams();// this is for image concentration

    obstacle_avoidance_controller.SpecializeFirstDroneParams();
}

void Drone::SpecializeParams(int id_number)
{
    //specialize the name of this object
    drone_id = "Quadricopter_" + std::to_string(id_number);

    waypoint_topic_id = drone_id + "/command/way_point";
    motor_speed_topic_id = drone_id + "/command/motor_speed";
    // swarm flocking situation topic id
    flocking_topic_id = drone_id + "/command/flocking";
    // bacterium situation topic id
    bacterium_topic_id = drone_id + "/command/bacterium";
    // situation topic id
    destination_topic_id = drone_id + "/command/destination";
    liftland_topic_id = drone_id + "/command/liftland";
  
    gps_topic_id = drone_id + "/sensor/gps";
    imu_topic_id = drone_id + "/sensor/imu";

    // getting sensors msg
    gps_sub = nh.subscribe(gps_topic_id, 1000, &Drone::OdometryCallback, this);
    imu_sub = nh.subscribe(imu_topic_id, 1000, &Drone::IMUCallback, this);
  
    // commands in
    command_trajectory_sub = nh.subscribe(waypoint_topic_id, 10, &Drone::CommandTrajectoryCallback, this);
    // commands out
    command_motor_speed_pub = nh.advertise<mav_msgs::CommandMotorSpeed>(motor_speed_topic_id, 10);

    // swarm situation commands in
    command_flocking_sub = nh.subscribe(flocking_topic_id, 10, &Drone::CommandFlockingCallback, this);
    // bacterium situation commands in
    command_bacterium_sub = nh.subscribe(bacterium_topic_id, 10, &Drone::CommandBacteriumCallback, this);

    // situation commands in
    command_destination_sub = nh.subscribe(destination_topic_id, 10, &Drone::CommandDestinationCallback, this);
    command_liftland_sub = nh.subscribe(liftland_topic_id, 10, &Drone::CommandLiftLandCallback, this);

    // vacuum and camera parameters specialize
    vacuum.SpecializeParams(id_number);
    //vision_sensor_360.SpecializeParams(id_number);
    vision_sensor_panoramic.SpecializeParams(id_number);
    //random_wp_flying_controller.SpecializeParams(id_number);
    random_velocity_flying_controller.SpecializeParams(id_number);

    ultrasonic_sensor.SpecializeParams(id_number);// this is for distance concentration
    //vision_sensor_down.SpecializeParams(id_number); // this is for image concentration

    obstacle_avoidance_controller.SpecializeParams(id_number);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/////////////////// Publishing ///////////////////////////////////////////////////////////////

// drone controller publish
void Drone::Publish()
{
    mav_msgs::CommandMotorSpeedPtr cmd_motors_msg(new mav_msgs::CommandMotorSpeed);
    cmd_motors_msg->motor_speed.clear();

    for (int i = 0; i < command_motor_speed.size(); i++)
    {
        cmd_motors_msg->motor_speed.push_back(command_motor_speed[i]);
    }

    ros::Time update_time = ros::Time::now();
    cmd_motors_msg->header.stamp = update_time;
    cmd_motors_msg->header.frame_id = "drone_";
    command_motor_speed_pub.publish(cmd_motors_msg);
}


void Drone::PublishForTest()
{
    vel_pub.publish(command_velocity_msg);
    roll_pitch_yaw_thrust_pub.publish(command_attitude_msg);
    acc_pub.publish(command_acc_msg);
}


////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Command Processing ///////////////////////////////////////////////////////

void Drone::CommandTrajectoryCallback(const mav_msgs::CommandTrajectoryConstPtr& trajectory_msg)
{
    //ROS_INFO_ONCE("Drone got the first trajectory command.");
    command_wp_msg = *trajectory_msg;
}

void Drone::CommandFlockingCallback(const std_msgs::BoolConstPtr& command_flocking_msg)
{

    if(command_flocking_msg->data)
    {
        // change the flocking situation
        flocking_situation = true;
    }
}

void Drone::CommandBacteriumCallback(const std_msgs::BoolConstPtr& command_bacterium_msg)
{

    if(command_bacterium_msg->data)
    {
        // change the flocking situation
        bacterium_situation = true;
    }
}

void Drone::CommandDestinationCallback(const std_msgs::BoolConstPtr& command_destination_msg)
{
    destination_situation = command_destination_msg->data;
}

void Drone::CommandLiftLandCallback(const std_msgs::BoolConstPtr& command_liftland_msg)
{
    liftland_situation = command_liftland_msg->data;
}


//////////////////////////////////////////////////////////////////////////////////////////////
/////////////////// Sensor MSG Processing /////////////////////////////////////////////////////

void Drone::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
    //ROS_INFO_ONCE("Drone got the first GPS message.");
    current_gps = *odometry_msg;
    
    if(destination_situation)
    {
        if(isnan(vision_sensor_panoramic.flocking_controller.flocking_centroid[0]))
        {
            vision_sensor_panoramic.flocking_controller.flocking_centroid[0] = 0;
            vision_sensor_panoramic.flocking_controller.flocking_centroid[1] = 0;
        }
        
        /*// record relative position
        if(counter == 0)
        {
            //get flocking centroid
            ROS_INFO_STREAM("Flocking Centroid is: "
            <<vision_sensor_panoramic.flocking_controller.flocking_centroid[0]
            <<","<<vision_sensor_panoramic.flocking_controller.flocking_centroid[1]<<" # "<<drone_id);
            error[0] = vision_sensor_panoramic.flocking_controller.flocking_centroid[0];
            error[1] = vision_sensor_panoramic.flocking_controller.flocking_centroid[1];
            counter++;
        }
        command_wp_msg.position.x = destination_x - error[0];
        command_wp_msg.position.y = destination_y - error[1];*/
        
        //centroid gps
        if(counter == 0)
        {
            //get flocking centroid
            //ROS_INFO_STREAM("Flocking Centroid is: "
            //    <<vision_sensor_panoramic.flocking_controller.flocking_centroid[0]
            //    <<","<<vision_sensor_panoramic.flocking_controller.flocking_centroid[1]<<" # "<<drone_id);
            // consider the position relationship between virtual drone and this drone, the centroid of the flocking should be the center point between virtual drone and this drone
            error[0] = vision_sensor_panoramic.flocking_controller.flocking_centroid[0];
            error[1] = vision_sensor_panoramic.flocking_controller.flocking_centroid[1];
            counter++;
        }
        current_gps.pose.pose.position.x = current_gps.pose.pose.position.x + dispersion * error[0];
        current_gps.pose.pose.position.y = current_gps.pose.pose.position.y + dispersion * error[1];

        command_wp_msg.position.x = destination_x;
        command_wp_msg.position.y = destination_y;

    }

    position_controller.CalculatePositionControl(command_wp_msg, current_gps, &command_velocity_msg);
    
    int situation = 0;
    if(destination_situation)
    {
        situation = 1;//"destination mode"
    }else if(liftland_situation)
    {
        situation = 2;//"lift and land mode"
    }
    //ROS_INFO_STREAM("Current mode is: "<<situation);
    switch(situation)
    {
        case 1:
        {
            if (isnan(vision_sensor_panoramic.flocking_controller.v_mig[0]))
            {
            }else
            {
                //command_velocity_msg.velocity.x = command_velocity_msg.velocity.x + G_f_t * vision_sensor_panoramic.flocking_controller.v_mig[0];
                //command_velocity_msg.velocity.y = command_velocity_msg.velocity.y + G_f_t * vision_sensor_panoramic.flocking_controller.v_mig[1];
            }

            if(obstacle_avoidance_controller.activate)
            {
                obstacle_avoidance_velocity = obstacle_avoidance_controller.GetVelocityCommand();
                command_velocity_msg.velocity.x = command_velocity_msg.velocity.x + obstacle_avoidance_velocity[0];// ignore vertical velocity
                command_velocity_msg.velocity.y = command_velocity_msg.velocity.y + obstacle_avoidance_velocity[1];
            }
            // speed limitation
            if(command_velocity_msg.velocity.x > 0.5)
            {
                command_velocity_msg.velocity.x = 0.5;
            }
            if(command_velocity_msg.velocity.x < -0.5)
            {
                command_velocity_msg.velocity.x = -0.5;
            }
            if(command_velocity_msg.velocity.y > 0.5)
            {
                command_velocity_msg.velocity.y = 0.5;
            }
            if(command_velocity_msg.velocity.y < -0.5)
            {
                command_velocity_msg.velocity.y = -0.5;
            }
            break;
        }
        case 2:
        {
            /*if(obstacle_avoidance_controller.activate)
            {
                obstacle_avoidance_velocity = obstacle_avoidance_controller.GetVelocityCommand();
                command_velocity_msg.velocity.x = command_velocity_msg.velocity.x + obstacle_avoidance_velocity[0];// ignore vertical velocity
                command_velocity_msg.velocity.y = command_velocity_msg.velocity.y + obstacle_avoidance_velocity[1];
            }*/

            if (isnan(vision_sensor_panoramic.flocking_controller.v_mig[0]))
            {
            }else
            {
                command_velocity_msg.velocity.x = command_velocity_msg.velocity.x + G_f_t * vision_sensor_panoramic.flocking_controller.v_mig[0];
                command_velocity_msg.velocity.y = command_velocity_msg.velocity.y + G_f_t * vision_sensor_panoramic.flocking_controller.v_mig[1];
            }
            break;
        }
        default:
        {
            //check if use the obstacle avoidance controller
            if(obstacle_avoidance_controller.activate)
            {
                obstacle_avoidance_velocity = obstacle_avoidance_controller.GetVelocityCommand();
                command_velocity_msg.velocity.x = obstacle_avoidance_velocity[0];// ignore vertical velocity
                command_velocity_msg.velocity.y = obstacle_avoidance_velocity[1];
                //ROS_INFO_STREAM("Avoidance speed:"
                //    <<obstacle_avoidance_velocity[3]<<","<<obstacle_avoidance_velocity[0]<<","<<obstacle_avoidance_velocity[1]<<" #"<<drone_id);
            }else
            {
                if (bacterium_situation)
                {
                    // add bacterium behaviour velocity(random velocity for tumble phase and run phase)
                    bacterium = random_velocity_flying_controller.GetVelocityCommand();
                    bacterium_v[0] = bacterium[0];
                    bacterium_v[1] = bacterium[1];

                    command_velocity_msg.velocity.x = G_b * bacterium_v[0];
                    command_velocity_msg.velocity.y = G_b * bacterium_v[1];
                }
        
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                if (flocking_situation)
                {   
                    // check the agent is beyond the target or not. If yes, shut down the flocking behaviour and only use the bacterium behaviour
                    // if no, run flocking behaviour and bacterium behaviour together
                    if (random_velocity_flying_controller.current_concentration > 0.02) // means the height of object is higher than 2 centimeter, and think it is the target
                    {
                        if (isnan(vision_sensor_panoramic.flocking_controller.v_mig[0]))
                        {
                            //ROS_INFO_STREAM("Bacterium speed:"<<bacterium[3]<<","<<bacterium[0]<<","<<bacterium[1]<<" #"<<drone_id);
                            command_velocity_msg.velocity.x = G_b * bacterium_v[0];
                            command_velocity_msg.velocity.y = G_b * bacterium_v[1];
                            command_velocity_msg.velocity.z = G_b * bacterium_v[2] + vision_sensor_panoramic.flocking_controller.v_mig[2];
                        }else
                        {
                            //ROS_INFO_STREAM("I have found the target object. Flocking shut down. #"<<drone_id);
                            //ROS_INFO_STREAM("Bacterium speed:"<<bacterium[3]<<","<<bacterium[0]<<","<<bacterium[1]<<" #"<<drone_id);
                            command_velocity_msg.velocity.x = G_b * bacterium_v[0] + G_f_t * vision_sensor_panoramic.flocking_controller.v_mig[0];
                            command_velocity_msg.velocity.y = G_b * bacterium_v[1] + G_f_t * vision_sensor_panoramic.flocking_controller.v_mig[1];
                            command_velocity_msg.velocity.z = G_b * bacterium_v[2] + vision_sensor_panoramic.flocking_controller.v_mig[2];
                        }
                    }else
                    {
                        // check the agent is in a flocking area which means it can get the flocking cmd
                        if (isnan(vision_sensor_panoramic.flocking_controller.v_mig[0]))
                        {
                            //ROS_INFO_STREAM("Bacterium speed:"<<bacterium[3]<<","<<bacterium[0]<<","<<bacterium[1]<<" #"<<drone_id);
                            command_velocity_msg.velocity.x = G_b * bacterium_v[0];
                            command_velocity_msg.velocity.y = G_b * bacterium_v[1];
                            command_velocity_msg.velocity.z = G_b * bacterium_v[2] + vision_sensor_panoramic.flocking_controller.v_mig[2];
                        }else
                        {
                            //ROS_INFO_STREAM("Bacterium speed:"<<bacterium[3]<<","<<bacterium[0]<<","<<bacterium[1]<<" #"<<drone_id);
                            //ROS_INFO_STREAM("Flocking speed:"<<vision_sensor_panoramic.flocking_controller.flocking_speed<<" #"<<drone_id);
                            command_velocity_msg.velocity.x = G_b * bacterium_v[0] + G_f * vision_sensor_panoramic.flocking_controller.v_mig[0];
                            command_velocity_msg.velocity.y = G_b * bacterium_v[1] + G_f * vision_sensor_panoramic.flocking_controller.v_mig[1];
                            command_velocity_msg.velocity.z = G_b * bacterium_v[2] + vision_sensor_panoramic.flocking_controller.v_mig[2];
                        }
                    }
                }
            }
            break;
        }
    }
    


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    velocity_controller.CalculateVelocityControl(command_velocity_msg, current_gps, &command_acc_msg);
    acceleration_controller.CalculateAccelerationControl(command_acc_msg, current_gps, current_imu, &command_attitude_msg);
    //PublishForTest();
    // reset the flocking situation to false so that the swarm will continue to fly randomly if there is no flocking cmd
    //flocking_situation = false;// need to reset situatio if use flocking_cmd_generator node.
    //bacterium_situation = false;// need to reset situation if use baterium_cmd_generator node.
    
    destination_situation = false;
    liftland_situation = false;

}

void Drone::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    //ROS_INFO_ONCE("Drone got the first IMU message.");
    current_imu = *imu_msg;
    attitude_controller.CalculateAttitudeControl(command_attitude_msg, current_imu, &command_angular_rate_msg);
    attitude_controller.CalculateRateControl(command_angular_rate_msg, current_imu, &command_control_output);
    attitude_controller.CalculateMotorCommands(command_control_output, &command_motor_speed);
    Publish();
}



}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_controller_node");
    int client_number = 2;
    drone_control::Drone drones[client_number];
    if(client_number==1)
    {
        drones[0].SpecializeFirstDroneParams();
    }else
    {
        drones[0].SpecializeFirstDroneParams();
        for(int i = 1; i < client_number; i++)
        {
            drones[i].SpecializeParams(i-1);
        }
    }

    
    ros::spin();
    return 0;
}