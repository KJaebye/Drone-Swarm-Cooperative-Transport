// some neccesary msg types
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>

// drone controller loops
#include "attitude_control_loop.h"
#include "position_control_loop.h"
#include "velocity_control_loop.h"
#include "acceleration_control_loop.h"

// vacuum
#include "vacuum.h"

// vision sensors
//#include "vision_sensor_stitching360.h"
#include "vision_sensor_panoramic.h"
//#include "vision_sensor_down.h"

// other sensors
#include "ultrasonic_sensor.h"

// random fly controller
#include "random_wp_flying.h"
#include "random_velocity_flying.h"

#include "obstacle_avoidance.h"


namespace drone_control{

class Drone
{
    public:
    Drone();
    ~Drone();

    //initialize
    void InitializeParams();

    //publish functions
    void Publish();
    void VacuumPublish();
    void PublishForTest();

    //specialize each drone
    void SpecializeParams(int id_number);
    void SpecializeFirstDroneParams();

    //friend sensor_msgs::Imu GetCurrentIMU(Drone& obj);
    //friend nav_msgs::Odometry GetCurrentGPS(Drone& obj);
    //friend sensor_msgs::Range GetCurrentUltrasonicDistance(Drone& obj);

    private:

    // create a vacuum object
    Vacuum vacuum;
    // create a vision sensor
    // this is a stitching image camera(discard now)
    //VisionSensorImageStitching_360 vision_sensor_360;

    // create a panoramic vision sensor
    VisionSensorPanoramic vision_sensor_panoramic;// Flocking behaviour
    
    // create a down vision sensor for image concentration
    //VisionSensorDown vision_sensor_down;
    // create an ultrasonic sensor for distance concentration
    UltrasonicSensor ultrasonic_sensor;

    // create the proximity sensor for obstacle avoidance
    //ProximitySensor proximity_sensor;
    
    
    // create a random fly controller(random waypoint generator)
    //RandomWayPointFlyingController random_wp_flying_controller;
    RandomVelocityFlyingController random_velocity_flying_controller;// Bacterium behaviour

    //controller loops
    AttitudeController attitude_controller;
    PositionController position_controller;
    VelocityController velocity_controller;
    AccelerationController acceleration_controller;

    // obstacle avoidance controller
    ObstacleAvoidanceController obstacle_avoidance_controller;


    // names of the drone and topics
    std::string drone_id;
    std::string gps_topic_id;
    std::string imu_topic_id;

    std::string waypoint_topic_id;
    std::string motor_speed_topic_id;

    std::string flocking_topic_id;
    std::string bacterium_topic_id;

    std::string destination_topic_id;
    std::string liftland_topic_id;

    // command
    mav_msgs::CommandTrajectory command_wp_msg; // external command
    //commands
    mav_msgs::CommandVelocityTrajectory command_velocity_msg;
    mav_msgs::CommandVelocityTrajectory command_acc_msg;
    mav_msgs::CommandRollPitchYawrateThrust command_attitude_msg;
    Eigen::VectorXd command_angular_rate_msg;
    Eigen::VectorXd command_control_output;
    Eigen::VectorXd command_motor_speed;

    // situation commands
    std_msgs::Bool command_flocking_msg;
    bool flocking_situation;
    bool bacterium_situation;
    bool destination_situation;
    bool liftland_situation;

    // bacterium cmds(random flying behaviour)
    double *bacterium;
    double bacterium_v[3];
    // obstacle avoidance velocity cmd
    double *obstacle_avoidance_velocity;

    // subscribers and publishers for drone
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber ultrasonic_distance_sub;

    ros::Subscriber command_trajectory_sub;
    ros::Publisher command_motor_speed_pub;
    
    ros::Subscriber command_flocking_sub;
    ros::Subscriber command_bacterium_sub;

    ros::Subscriber command_destination_sub;
    ros::Subscriber command_liftland_sub;// receive msg to shut down height maintain
    
    
    //sensor msgs processing
    void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
    void IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg);

    //command waypoint processing
    void CommandTrajectoryCallback(const mav_msgs::CommandTrajectoryConstPtr& trajectory_msg);
    void CommandFlockingCallback(const std_msgs::BoolConstPtr& command_flocking_msg);
    void CommandBacteriumCallback(const std_msgs::BoolConstPtr& command_bacterium_msg);
    void CommandDestinationCallback(const std_msgs::BoolConstPtr& command_destination_msg);
    void CommandLiftLandCallback(const std_msgs::BoolConstPtr& command_liftland_msg);

    // initial position
    mav_msgs::CommandTrajectory initial_position;

    //sensors msgs
    nav_msgs::Odometry current_gps;
    sensor_msgs::Imu current_imu;
    sensor_msgs::Range current_ultrasonic_distance;


    // gains of flocking behaviour
    double G_b, G_f, G_f_t;
    // target destination
    double destination_x;
    double destination_y;

    int counter;
    double error[2];//error respect to the flocking centroid
    double dispersion;// means the dispersion in swarm when transporting

/////////////////// test publishers //////////////////////////////////////////////////////////
    ros::Publisher vel_pub;
    ros::Publisher roll_pitch_yaw_thrust_pub;
    ros::Publisher acc_pub;


};


}