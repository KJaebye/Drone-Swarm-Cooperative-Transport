#include "vacuum.h"

namespace drone_control{

Vacuum::Vacuum() {InitializeParams();}
Vacuum::~Vacuum() {}

void Vacuum::InitializeParams()
{
    activate = false;
}

void Vacuum::SpecializeFirstDroneParams()
{
    vacuum_id = "uarmVacuumGripper";
    vacuum_topic_id = vacuum_id + "/vacuum_activate";
    suction_situation_topic_id = vacuum_id + "/suction_situation";

    command_vacuum_sub = nh.subscribe(vacuum_topic_id, 10, &Vacuum::CommandVacuumCallback, this);
    suction_situation_sub = nh.subscribe(suction_situation_topic_id, 10, &Vacuum::SuctionSituationCallback, this);
    //command_vacuum_pub = nh.advertise<std_msgs::Bool>(vacuum_topic_id, 10);
}

void Vacuum::SpecializeParams(int id_number)
{
    vacuum_id = "uarmVacuumGripper_" + std::to_string(id_number);
    vacuum_topic_id = vacuum_id + "/vacuum_activate";
    suction_situation_topic_id = vacuum_id + "/suction_situation";

    command_vacuum_sub = nh.subscribe(vacuum_topic_id, 10, &Vacuum::CommandVacuumCallback, this);
    suction_situation_sub = nh.subscribe(suction_situation_topic_id, 10, &Vacuum::SuctionSituationCallback, this);
    //command_vacuum_pub = nh.advertise<std_msgs::Bool>(vacuum_topic_id, 10);
}

// vacuum publish
/*void Vacuum::Publish()
{
    command_activate_vacuum_msg.data = activate;
    command_vacuum_pub.publish(command_activate_vacuum_msg);
}*/

void Vacuum::CommandVacuumCallback(const std_msgs::BoolConstPtr& vacuum_msg)
{
    activate = vacuum_msg->data;
}

void Vacuum::SuctionSituationCallback(const std_msgs::BoolConstPtr& suction_msg)
{
    suction_situation = suction_msg->data;
}

}