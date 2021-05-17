#include "proximity_sensor.h"

namespace drone_control{

ProximitySensor::ProximitySensor()
{
    InitializeParams();
}

ProximitySensor::~ProximitySensor(){}
void ProximitySensor::InitializeParams()
{
    // calculate the transform matrix for every sensor
    rotrix_front = RotationMatrix(-M_PI_2, 0, -M_PI_2);
    rotrix_left = RotationMatrix(-M_PI_2, 0, M_PI);
    rotrix_right = RotationMatrix(-M_PI_2, 0, 0);
    rotrix_back = RotationMatrix(-M_PI_2, 0, M_PI_2);

}

Eigen::Matrix3d ProximitySensor::RotationMatrix(double gamma, double theta, double alpha)
{
    // calculate the rotation matrix
    x_rotation_matrix<<
        1,          0,           0,
        0, cos(gamma), -sin(gamma),
        0, sin(gamma),  cos(gamma);

    y_rotation_matrix<<
        cos(theta), 0, -sin(theta),
                 0, 1,           0,
        sin(theta), 0,  cos(theta);

    z_rotation_matrix<<
        cos(alpha), -sin(alpha), 0,
        sin(alpha),  cos(alpha), 0,
                 0,           0, 1;

    rotation_matrix = x_rotation_matrix*y_rotation_matrix*z_rotation_matrix;
    return rotation_matrix;
}

void ProximitySensor::SpecializeFirstDroneParams()
{
    drone_id = "Quadricopter";
    proximity_left_topic_id = drone_id + "/sensor/proximity/left";
    proximity_right_topic_id = drone_id + "/sensor/proximity/right";
    proximity_front_topic_id = drone_id + "/sensor/proximity/front";
    proximity_back_topic_id = drone_id + "/sensor/proximity/back";

    proximity_left_sub = nh.subscribe(proximity_left_topic_id, 10, &ProximitySensor::LeftCallback, this);
    proximity_right_sub = nh.subscribe(proximity_right_topic_id, 10, &ProximitySensor::RightCallback, this);
    proximity_front_sub = nh.subscribe(proximity_front_topic_id, 10, &ProximitySensor::FrontCallback, this);
    proximity_back_sub = nh.subscribe(proximity_back_topic_id, 10, &ProximitySensor::BackCallback, this);
}

void ProximitySensor::SpecializeParams(int id_number)
{
    drone_id = "Quadricopter_" + std::to_string(id_number);
    proximity_left_topic_id = drone_id + "/sensor/proximity/left";
    proximity_right_topic_id = drone_id + "/sensor/proximity/right";
    proximity_front_topic_id = drone_id + "/sensor/proximity/front";
    proximity_back_topic_id = drone_id + "/sensor/proximity/back";

    proximity_left_sub = nh.subscribe(proximity_left_topic_id, 10, &ProximitySensor::LeftCallback, this);
    proximity_right_sub = nh.subscribe(proximity_right_topic_id, 10, &ProximitySensor::RightCallback, this);
    proximity_front_sub = nh.subscribe(proximity_front_topic_id, 10, &ProximitySensor::FrontCallback, this);
    proximity_back_sub = nh.subscribe(proximity_back_topic_id, 10, &ProximitySensor::BackCallback, this);
}

void ProximitySensor::LeftCallback(const sensor_msgs::LaserEchoConstPtr& proximity_left_msg)
{
    left_distance = proximity_left_msg->echoes[0];
    // array stores the coordinate of the proximity point with respect to the sensor reference system
    left_array[0] = proximity_left_msg->echoes[1];
    left_array[1] = proximity_left_msg->echoes[2];
    left_array[2] = proximity_left_msg->echoes[3];
    //ROS_INFO_STREAM("Left obstacle position: "<<left_array[0]<<", "<<left_array[1]<<", "<<left_array[2]);
    //ROS_INFO_STREAM("Rotrix left: "<<rotrix_left);
    // transform into bady reference system
    left_array = left_array*rotrix_left;
    //ROS_INFO_STREAM("Left proximity sensor: "<<left_distance);
    //ROS_INFO_STREAM("Obstacle position: "<<left_array[0]<<", "<<left_array[1]<<", "<<left_array[2]<<" (from left sensor)");
    
    // record current time
    left_time = ros::Time::now();
    //ROS_INFO_STREAM("Left time:"<<left_time);
    // calculate rate
    double dt = (left_time - last_left_time).toSec();
    if (dt == 0.0) return;
    left_distance_rate = (left_distance - pre_left_distance)/dt;
    last_left_time = left_time;
}

void ProximitySensor::RightCallback(const sensor_msgs::LaserEchoConstPtr& proximity_right_msg)
{
    right_distance = proximity_right_msg->echoes[0];
    // array stores the coordinate of the proximity point with respect to the sensor reference system
    right_array[0] = proximity_right_msg->echoes[1];
    right_array[1] = proximity_right_msg->echoes[2];
    right_array[2] = proximity_right_msg->echoes[3];
    // transform into bady reference system
    right_array = right_array*rotrix_right;
    //ROS_INFO_STREAM("Right proximity sensor: "<<right_distance);
    //ROS_INFO_STREAM("Obstacle position: "<<right_array[0]<<", "<<right_array[1]<<", "<<right_array[2]<<" (from right sensor)");
    
    right_time = ros::Time::now();
    //ROS_INFO_STREAM("Right time::"<<right_time);
    // calculate rate
    double dt = (right_time - last_right_time).toSec();
    if (dt == 0.0) return;
    right_distance_rate = (right_distance - pre_right_distance)/dt;
    last_right_time = right_time;
}

void ProximitySensor::FrontCallback(const sensor_msgs::LaserEchoConstPtr& proximity_front_msg)
{
    front_distance = proximity_front_msg->echoes[0];
    // array stores the coordinate of the proximity point with respect to the sensor reference system
    front_array[0] = proximity_front_msg->echoes[1];
    front_array[1] = proximity_front_msg->echoes[2];
    front_array[2] = proximity_front_msg->echoes[3];
    // transform into bady reference system
    front_array = front_array*rotrix_front;
    //ROS_INFO_STREAM("Front proximity sensor: "<<front_distance);
    //ROS_INFO_STREAM("Obstacle position: "<<front_array[0]<<", "<<front_array[1]<<", "<<front_array[2]<<" (from front sensor)");
    
    front_time = ros::Time::now();
    //ROS_INFO_STREAM("Front time:"<<front_time);
    // calculate rate
    double dt = (front_time - last_front_time).toSec();
    if (dt == 0.0) return;
    front_distance_rate = (front_distance - pre_front_distance)/dt;
    last_front_time = front_time;
}

void ProximitySensor::BackCallback(const sensor_msgs::LaserEchoConstPtr& proximity_back_msg)
{
    back_distance = proximity_back_msg->echoes[0];
    // array stores the coordinate of the proximity point with respect to the sensor reference system
    back_array[0] = proximity_back_msg->echoes[1];
    back_array[1] = proximity_back_msg->echoes[2];
    back_array[2] = proximity_back_msg->echoes[3];
    // transform into bady reference system
    back_array = back_array*rotrix_back;
    //ROS_INFO_STREAM("Back proximity sensor: "<<back_distance);
    //ROS_INFO_STREAM("Obstacle position: "<<back_array[0]<<", "<<back_array[1]<<", "<<back_array[2]<<" (from back sensor)");
    
    back_time = ros::Time::now();
    //ROS_INFO_STREAM("Back time:"<<back_time);
    // calculate rate
    double dt = (back_time - last_back_time).toSec();
    if (dt == 0.0) return;
    back_distance_rate = (back_distance - pre_back_distance)/dt;
    last_back_time = back_time;
}

void ProximitySensor::Merge()
{
    
}
}