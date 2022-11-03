#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace drone_control{

class BacteriumCommandGenerator
{
    public:
    BacteriumCommandGenerator();
    ~BacteriumCommandGenerator();

    void InitializeParams();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);
    void Publish();

    private:
    ros::NodeHandle nh;
    ros::Publisher bacterium_cmd_pub;
    std_msgs::Bool bacterium_cmd_msg;

    std::string bacterium_topic_id;
};

BacteriumCommandGenerator::BacteriumCommandGenerator()
{
    InitializeParams();
    bacterium_cmd_msg.data = true;
}

void BacteriumCommandGenerator::InitializeParams(){}

void BacteriumCommandGenerator::SpecializeFirstDroneParams()
{
    bacterium_topic_id = "Quadricopter/command/bacterium";
    bacterium_cmd_pub = nh.advertise<std_msgs::Bool>(bacterium_topic_id, 10);
}

void BacteriumCommandGenerator::SpecializeParams(int id_number)
{
    bacterium_topic_id = "Quadricopter_" + std::to_string(id_number) + "/command/bacterium";
    bacterium_cmd_pub = nh.advertise<std_msgs::Bool>(bacterium_topic_id, 10);
}

void BacteriumCommandGenerator::Publish()
{
    bacterium_cmd_pub.publish(bacterium_cmd_msg);
    //ROS_INFO_STREAM(std::to_string(bacterium_cmd_msg.data));
}

BacteriumCommandGenerator::~BacteriumCommandGenerator(){}


}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bacterium_cmd_generator_node");
    int client_number = 3;
    drone_control::BacteriumCommandGenerator bacterium_cmd_generators[client_number];
    if(client_number==1)
    {
        bacterium_cmd_generators[0].SpecializeFirstDroneParams();
    }else
    {
        bacterium_cmd_generators[0].SpecializeFirstDroneParams();
        for(int i = 1; i < client_number; i++)
        {
            bacterium_cmd_generators[i].SpecializeParams(i-1);
        }
    }
    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        for (int i = 0; i < client_number; i++)
        {
            bacterium_cmd_generators[i].Publish();
        }
        loop_rate.sleep();
    }

    return 0;
}