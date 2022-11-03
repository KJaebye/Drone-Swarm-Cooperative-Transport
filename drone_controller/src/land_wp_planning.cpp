// This is a way point planner and publisher
// When launch this node, the drone (swarm) will land and loose the target object, then return to a predefined height
// and this node is only for the herizontal control
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <mav_msgs/CommandTrajectory.h>

using namespace std;
using namespace ros;

namespace drone_control{

class LandWaypointPlanner
{
    public:
    LandWaypointPlanner();
    ~LandWaypointPlanner();
    void InitializeParams();
    void Publish();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);

    private:
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;
    ros::Publisher waypoint_pub;
    ros::Publisher vacuum_shutdown_pub;
    ros::Publisher situation_pub;

    string drone_id;
    string vacuum_id;
    string waypoint_topic_id;
    string vacuum_shutdown_topic_id;
    string gps_topic_id;
    string liftland_topic_id;

    mav_msgs::CommandTrajectory target_land_waypoint;
    mav_msgs::CommandTrajectory initial_waypoint;
    mav_msgs::CommandTrajectory return_waypoint;
    nav_msgs::Odometry current_gps;
    std_msgs::Bool vacuum_cmd;
    std_msgs::Bool situation_msg;

    int counter;
    int counter_1;
    double vertical_error;

    void OdometryCallback(const nav_msgs::OdometryConstPtr& gps_msg);
    
};

LandWaypointPlanner::LandWaypointPlanner()
{
    InitializeParams();
}

LandWaypointPlanner::~LandWaypointPlanner(){}
void LandWaypointPlanner::InitializeParams()
{
    vacuum_cmd.data = false;
    counter = 0;
    counter_1 = 0;

    situation_msg.data = true;
}

void LandWaypointPlanner::SpecializeFirstDroneParams()
{
    drone_id = "Quadricopter";
    gps_topic_id = drone_id + "/sensor/gps";
    waypoint_topic_id = drone_id + "/command/way_point";
    // situation topic id
    liftland_topic_id = drone_id + "/command/liftland";

    vacuum_id = "uarmVacuumGripper";
    vacuum_shutdown_topic_id = vacuum_id + "/vacuum_activate";

    gps_sub = nh.subscribe(gps_topic_id, 1000, &LandWaypointPlanner::OdometryCallback, this);
    waypoint_pub = nh.advertise<mav_msgs::CommandTrajectory>(waypoint_topic_id, 1000);
    vacuum_shutdown_pub = nh.advertise<std_msgs::Bool>(vacuum_shutdown_topic_id, 10);
    situation_pub = nh.advertise<std_msgs::Bool>(liftland_topic_id, 10);
}

void LandWaypointPlanner::SpecializeParams(int id_number)
{
    drone_id = "Quadricopter_" + to_string(id_number);
    gps_topic_id = drone_id + "/sensor/gps";
    waypoint_topic_id = drone_id + "/command/way_point";
    // situation topic id
    liftland_topic_id = drone_id + "/command/liftland";

    vacuum_id = "uarmVacuumGripper_" + to_string(id_number);
    vacuum_shutdown_topic_id = vacuum_id + "/vacuum_activate";

    gps_sub = nh.subscribe(gps_topic_id, 1000, &LandWaypointPlanner::OdometryCallback, this);
    waypoint_pub = nh.advertise<mav_msgs::CommandTrajectory>(waypoint_topic_id, 1000);
    vacuum_shutdown_pub = nh.advertise<std_msgs::Bool>(vacuum_shutdown_topic_id, 10);
    situation_pub = nh.advertise<std_msgs::Bool>(liftland_topic_id, 10);
}

void LandWaypointPlanner::Publish()
{
    situation_pub.publish(situation_msg);
    if(counter == 0)
    {
        waypoint_pub.publish(target_land_waypoint);
    }
    
    double vertical_error = fabs(target_land_waypoint.position.z - current_gps.pose.pose.position.z);
    if(vertical_error < 0.03)
    {
        counter += 1;
    }
    //ROS_INFO_STREAM(counter);
    if(counter != 0)
    {
        vacuum_shutdown_pub.publish(vacuum_cmd);
        waypoint_pub.publish(return_waypoint);
    }
}

void LandWaypointPlanner::OdometryCallback(const nav_msgs::OdometryConstPtr& gps_msg)
{
    if (counter_1 == 0)
    {
        initial_waypoint.position.x = gps_msg->pose.pose.position.x;
        initial_waypoint.position.y = gps_msg->pose.pose.position.y;

        target_land_waypoint.position.x = initial_waypoint.position.x;
        target_land_waypoint.position.y = initial_waypoint.position.y;
        target_land_waypoint.position.z = 1;

        return_waypoint = initial_waypoint;
        return_waypoint.position.z = 2; // return to 2 meters level
    }
    counter_1 += 1;
    current_gps = *gps_msg;
    Publish();
}


}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "land_wp_planning_node");
    int client_number = 3;
    drone_control::LandWaypointPlanner land_wp_planner[client_number];
    if(client_number==1)
    {
        land_wp_planner[0].SpecializeFirstDroneParams();
    }else
    {
        land_wp_planner[0].SpecializeFirstDroneParams();
        for(int i = 1; i < client_number; i++)
        {
            land_wp_planner[i].SpecializeParams(i-1);
        }
    }

    
    ros::spin();
    return 0;
}