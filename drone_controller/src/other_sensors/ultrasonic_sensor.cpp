#include "ultrasonic_sensor.h"

namespace drone_control{

UltrasonicSensor::UltrasonicSensor()
{
    InitializeParams();
}

UltrasonicSensor::~UltrasonicSensor(){}

void UltrasonicSensor::InitializeParams()
{
    ultrasonic_sensor_distance = 0.5;// the value is 0.5 meter in drone.ttm model
}

void UltrasonicSensor::SpecializeFirstDroneParams()
{
    // create suubscriber for the first agent
    drone_id = "Quadricopter";
    ultrasonic_sensor_topic_id = drone_id + "/sensor/ultrasonic_distance";
    odometry_topic_id = drone_id + "/sensor/gps";
    concentration_topic_id = drone_id + "/concentration";
    
    ultrasonic_sensor_sub = nh.subscribe(ultrasonic_sensor_topic_id, 10, &UltrasonicSensor::UltrasonicSensorCallback, this);
    odometry_sub = nh.subscribe(odometry_topic_id, 10, &UltrasonicSensor::OdometryCallback, this);
    concentration_pub = nh.advertise<std_msgs::Float32MultiArray>(concentration_topic_id, 10);
}

void UltrasonicSensor::SpecializeParams(int id_number)
{
    // create suubscriber for other agents
    drone_id = "Quadricopter_" + std::to_string(id_number);
    ultrasonic_sensor_topic_id = drone_id + "/sensor/ultrasonic_distance";
    odometry_topic_id = drone_id + "/sensor/gps";
    concentration_topic_id = drone_id + "/concentration";
    
    ultrasonic_sensor_sub = nh.subscribe(ultrasonic_sensor_topic_id, 10, &UltrasonicSensor::UltrasonicSensorCallback, this);
    odometry_sub = nh.subscribe(odometry_topic_id, 10, &UltrasonicSensor::OdometryCallback, this);
    concentration_pub = nh.advertise<std_msgs::Float32MultiArray>(concentration_topic_id, 10);
}

void UltrasonicSensor::UltrasonicSensorCallback(const sensor_msgs::RangeConstPtr& ultrasonic_msg)
{
    distance = ultrasonic_msg->range;
    concentration_msg.data.resize(1);
    concentration_msg.data[0] = height - ultrasonic_sensor_distance - distance;
    concentration_pub.publish(concentration_msg);
}

void UltrasonicSensor::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
    height = odometry_msg->pose.pose.position.z;
}



}