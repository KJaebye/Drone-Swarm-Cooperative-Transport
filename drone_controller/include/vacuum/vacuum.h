// this class should be short, brief and clear
// only include the necessary component.
//
// the purpose of the class is, to realize that active the vacuum, suck the object when
// the vacuum revieve command, and check the suction situation
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace drone_control{

class Vacuum
{
    public:
    Vacuum();
    ~Vacuum();
    void InitializeParams();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);
    void Publish();


    private:
    // name of the vacuum topic
    std::string vacuum_id;
    std::string vacuum_topic_id;
    std::string suction_situation_topic_id;

    ros::NodeHandle nh;
    // subscribers and publishers for vacuum
    ros::Subscriber command_vacuum_sub;
    ros::Subscriber suction_situation_sub;
    //ros::Publisher  command_vacuum_pub; // active or inactive the vacuum

    // properties
    double vacuum_height;
    double vacuum_weight;
    double vacuum_max_suction; // for declaring the max mass that the vacuum can suck
    bool   rope_type; // true or false, means different models with or without a cable
    double suction_distance; // the distance from the drone to vacuum suction interface

    std_msgs::Bool command_activate_vacuum_msg;
    
    bool activate; // external command from drone or other node
    bool suction_situation;

    void SuctionSituationCallback(const std_msgs::BoolConstPtr& suction_msg);
    void CommandVacuumCallback(const std_msgs::BoolConstPtr& vacuum_msg);
};





}