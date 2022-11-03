#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>

namespace drone_control{

class UltrasonicSensor
{
    public:
    UltrasonicSensor();
    ~UltrasonicSensor();
    void InitializeParams();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);
    
    private:
    ros::NodeHandle nh;
    ros::Subscriber ultrasonic_sensor_sub;
    ros::Subscriber odometry_sub;
    ros::Publisher concentration_pub;

    std::string drone_id;
    std::string ultrasonic_sensor_topic_id;
    std::string odometry_topic_id;
    std::string concentration_topic_id;
 
    double ultrasonic_sensor_distance;// this value shows the distance between agent and sensor
    double height;// this value is the absolute height of agents
    double distance;// this value is the distance from sensor down to the ground
    std_msgs::Float32MultiArray concentration_msg;

    void UltrasonicSensorCallback(const sensor_msgs::RangeConstPtr& ultrasonic_msg);
    void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
};


}