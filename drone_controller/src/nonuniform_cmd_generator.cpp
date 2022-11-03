#include <ros/ros.h>
#include <mav_msgs/CommandTrajectory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uniform_position_cmd");
    ros::NodeHandle n;

    ros::Publisher drone_pos_pub = n.advertise<mav_msgs::CommandTrajectory>("Quadricopter/command/way_point",1000);
    ros::Publisher drone0_pos_pub = n.advertise<mav_msgs::CommandTrajectory>("Quadricopter_0/command/way_point",1000);
    ros::Publisher drone1_pos_pub = n.advertise<mav_msgs::CommandTrajectory>("Quadricopter_1/command/way_point",1000);
    ros::Publisher drone2_pos_pub = n.advertise<mav_msgs::CommandTrajectory>("Quadricopter_2/command/way_point",1000);
    ros::Publisher drone3_pos_pub = n.advertise<mav_msgs::CommandTrajectory>("Quadricopter_3/command/way_point",1000);
    ros::Publisher drone4_pos_pub = n.advertise<mav_msgs::CommandTrajectory>("Quadricopter_4/command/way_point",1000);

    // set the position
    mav_msgs::CommandTrajectory drone_pos;
    mav_msgs::CommandTrajectory drone0_pos;
    mav_msgs::CommandTrajectory drone1_pos;
    mav_msgs::CommandTrajectory drone2_pos;
    mav_msgs::CommandTrajectory drone3_pos;
    mav_msgs::CommandTrajectory drone4_pos;

    // drone
    drone_pos.position.x = -2.8;
    drone_pos.position.y = 0.7;
    drone_pos.position.z = 1.7;
    // drone0
    drone0_pos.position.x = -1.55;
    drone0_pos.position.y = -1.15;
    drone0_pos.position.z = 1.7;
    // drone1
    drone1_pos.position.x = -1.4;
    drone1_pos.position.y = 1;
    drone1_pos.position.z = 1.7;
    // drone2
    drone2_pos.position.x = -0.55;
    drone2_pos.position.y = -0.125;
    drone2_pos.position.z = 1.7;
    // drone3
    drone3_pos.position.x = -2.85;
    drone3_pos.position.y = -0.4;
    drone3_pos.position.z = 1.7;
    // drone4
    drone4_pos.position.x = -1.7;
    drone4_pos.position.y = 0;
    drone4_pos.position.z = 1.7;

    ros::Rate loop_rate(10);
    int count = 0;
    while(ros::ok())
    {
        drone_pos_pub.publish(drone_pos);
        drone0_pos_pub.publish(drone0_pos);
        drone1_pos_pub.publish(drone1_pos);
        drone2_pos_pub.publish(drone2_pos);
        drone3_pos_pub.publish(drone3_pos);
        drone4_pos_pub.publish(drone4_pos);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}