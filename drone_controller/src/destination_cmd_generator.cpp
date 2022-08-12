#include <ros/ros.h>
#include <mav_msgs/CommandTrajectory.h>
#include <std_msgs/Bool.h>

namespace drone_control{

class DestinationCommandGenerator
{
    public:
    DestinationCommandGenerator();
    ~DestinationCommandGenerator();
    void InitializeParams();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);
    void Publish();

    private:
    ros::NodeHandle nh;
    ros::Publisher destination_cmd_pub;
    ros::Publisher destination_situation_cmd_pub;
    mav_msgs::CommandTrajectory destination_cmd_msg;
    std_msgs::Bool destination_situation_msg;

    std::string drone_id;
    std::string destination_topic_id;
    std::string destination_situation_topic_id;


};

DestinationCommandGenerator::DestinationCommandGenerator()
{
    InitializeParams();
}

DestinationCommandGenerator::~DestinationCommandGenerator(){}

void DestinationCommandGenerator::InitializeParams()
{
    //destination_cmd_msg.position.x = 0;
    //destination_cmd_msg.position.y = 0;

    destination_situation_msg.data = true;
}

void DestinationCommandGenerator::Publish()
{
    //destination_cmd_pub.publish(destination_cmd_msg);
    destination_situation_cmd_pub.publish(destination_situation_msg);
}

void DestinationCommandGenerator::SpecializeFirstDroneParams()
{
    drone_id = "Quadricopter";
    destination_topic_id = drone_id + "/command/way_point";
    destination_situation_topic_id = drone_id + "/command/destination";

    //destination_cmd_pub = nh.advertise<mav_msgs::CommandTrajectory>(destination_topic_id, 1000);
    destination_situation_cmd_pub = nh.advertise<std_msgs::Bool>(destination_situation_topic_id, 10);
}

void DestinationCommandGenerator::SpecializeParams(int id_number)
{
    drone_id = "Quadricopter_" + std::to_string(id_number);
    destination_topic_id = drone_id + "/command/way_point";
    destination_situation_topic_id = drone_id + "/command/destination";

    //destination_cmd_pub = nh.advertise<mav_msgs::CommandTrajectory>(destination_topic_id, 1000);
    destination_situation_cmd_pub = nh.advertise<std_msgs::Bool>(destination_situation_topic_id, 10);
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "destination_cmd_generator_node");
    int client_number = 6;
    drone_control::DestinationCommandGenerator destination_cmd_generators[client_number];
    if(client_number==1)
    {
        destination_cmd_generators[0].SpecializeFirstDroneParams();
    }else
    {
        destination_cmd_generators[0].SpecializeFirstDroneParams();
        for(int i = 1; i < client_number; i++)
        {
            destination_cmd_generators[i].SpecializeParams(i-1);
        }
    }

    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        for (int i = 0; i < client_number; i++)
        {
            destination_cmd_generators[i].Publish();
        }
        loop_rate.sleep();
    }
    return 0;
}
