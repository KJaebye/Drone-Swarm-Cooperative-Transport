#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace drone_control{

class VacuumCommandGenerator
{
    public:
    VacuumCommandGenerator();
    ~VacuumCommandGenerator();
    void InitializeParams();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);
    void Publish();

    private:
    ros::NodeHandle nh;
    ros::Publisher vacuum_cmd_pub;
    std_msgs::Bool vacuum_cmd_msg;

    std::string vacuum_id;
    std::string vacuum_topic_id;
};

VacuumCommandGenerator::VacuumCommandGenerator()
{
    InitializeParams();
}

VacuumCommandGenerator::~VacuumCommandGenerator(){}
void VacuumCommandGenerator::InitializeParams(){}


void VacuumCommandGenerator::SpecializeFirstDroneParams()
{
    vacuum_id = "uarmVacuumGripper";
    vacuum_topic_id = vacuum_id + "/vacuum_activate";
    vacuum_cmd_pub = nh.advertise<std_msgs::Bool>(vacuum_topic_id, 10);
}

void VacuumCommandGenerator::SpecializeParams(int id_number)
{
    vacuum_id = "uarmVacuumGripper_" + std::to_string(id_number);
    vacuum_topic_id = vacuum_id + "/vacuum_activate";
    vacuum_cmd_pub = nh.advertise<std_msgs::Bool>(vacuum_topic_id, 10);
}

void VacuumCommandGenerator::Publish()
{
    vacuum_cmd_msg.data = true;
    vacuum_cmd_pub.publish(vacuum_cmd_msg);
    //ROS_INFO_STREAM(std::to_string(flocking_cmd_msg.data));
}

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vacuum_cmd_generator_node");
    int client_number = 6;
    drone_control::VacuumCommandGenerator vaccum_cmd_generators[client_number];
    if(client_number==1)
    {
        vaccum_cmd_generators[0].SpecializeFirstDroneParams();
    }else
    {
        vaccum_cmd_generators[0].SpecializeFirstDroneParams();
        for(int i = 1; i < client_number; i++)
        {
            vaccum_cmd_generators[i].SpecializeParams(i-1);
        }
    }

    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        for (int i = 0; i < client_number; i++)
        {
            vaccum_cmd_generators[i].Publish();
        }
        loop_rate.sleep();
    }
    return 0;
}