#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace drone_control{

class FlockingCommandGenerator
{
    public:
    FlockingCommandGenerator();
    ~FlockingCommandGenerator();

    void InitializeParams();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);
    void Publish();

    private:
    ros::NodeHandle nh;
    ros::Publisher flocking_cmd_pub;
    std_msgs::Bool flocking_cmd_msg;

    std::string flocking_topic_id;
};

FlockingCommandGenerator::FlockingCommandGenerator()
{
    InitializeParams();
    flocking_cmd_msg.data = true;
}

void FlockingCommandGenerator::InitializeParams(){}

void FlockingCommandGenerator::SpecializeFirstDroneParams()
{
    flocking_topic_id = "Quadricopter/command/flocking";
    flocking_cmd_pub = nh.advertise<std_msgs::Bool>(flocking_topic_id, 10);
}

void FlockingCommandGenerator::SpecializeParams(int id_number)
{
    flocking_topic_id = "Quadricopter_" + std::to_string(id_number) + "/command/flocking";
    flocking_cmd_pub = nh.advertise<std_msgs::Bool>(flocking_topic_id, 10);
}

void FlockingCommandGenerator::Publish()
{
    flocking_cmd_pub.publish(flocking_cmd_msg);
    //ROS_INFO_STREAM(std::to_string(flocking_cmd_msg.data));
}

FlockingCommandGenerator::~FlockingCommandGenerator(){}


}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "flocking_cmd_generator_node");
    int client_number = 3;
    drone_control::FlockingCommandGenerator flocking_cmd_generators[client_number];
    if(client_number==1)
    {
        flocking_cmd_generators[0].SpecializeFirstDroneParams();
    }else
    {
        flocking_cmd_generators[0].SpecializeFirstDroneParams();
        for(int i = 1; i < client_number; i++)
        {
            flocking_cmd_generators[i].SpecializeParams(i-1);
        }
    }
    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        for (int i = 0; i < client_number; i++)
        {
            flocking_cmd_generators[i].Publish();
        }
        loop_rate.sleep();
    }



    //flocking_cmd_generators[0].Publish();
    //ros::spin();
    return 0;
}