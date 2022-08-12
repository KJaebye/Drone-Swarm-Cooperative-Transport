// This is a way point planner and publisher
// When launch this node, the drone (swarm) will land and suck the target object, then lift it to a predefined height
// and this node is only for the herizontal control
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <mav_msgs/CommandTrajectory.h>
#include <std_msgs/Bool.h>


namespace drone_control{

class LiftWaypointPlanner
{
    public:
    LiftWaypointPlanner();
    ~LiftWaypointPlanner();
    void InitializeParams();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);
    void Publish();

    private:
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;
    ros::Subscriber ultrasonic_sub;
    ros::Subscriber suction_sub;
    ros::Publisher waypoint_pub;
    ros::Publisher situation_pub;

    std::string drone_id;
    std::string vacuum_id;
    std::string gps_topic_id;
    std::string ultrasonic_topic_id;
    std::string suction_topic_id;
    std::string waypoint_topic_id;
    std::string liftland_topic_id;

    void OdometryCallback(const nav_msgs::OdometryConstPtr& gps_msg);
    void UltrasonicDistanceCallback(const sensor_msgs::RangeConstPtr& ultrasonic_msg);
    void SuctionSituationCallback(const std_msgs::BoolConstPtr& suction_msg);

    // create some spaces for storing the msg info
    nav_msgs::Odometry current_gps;
    nav_msgs::Odometry initial_gps;
    double ultrasonic_distance;
    double hover_height;
    double object_height;
    double object_desired_height;
    bool suction_situation;
    bool last_suction_situation;

    void WaypointPlanning(nav_msgs::Odometry current_gps, double ultrasonic_distance, double hover_height, mav_msgs::CommandTrajectory *cmd_way_point);
    mav_msgs::CommandTrajectory cmd_way_point;
    std_msgs::Bool situation_msg;
    int counter;
};

LiftWaypointPlanner::LiftWaypointPlanner()
{
    InitializeParams();
}
LiftWaypointPlanner::~LiftWaypointPlanner(){}
void LiftWaypointPlanner::InitializeParams()
{
    hover_height = 0;
    object_desired_height = 0.5;
    counter = 0;

    situation_msg.data = true;
}

void LiftWaypointPlanner::SpecializeFirstDroneParams()
{
    drone_id = "Quadricopter";
    gps_topic_id = drone_id + "/sensor/gps";
    ultrasonic_topic_id = drone_id + "/sensor/ultrasonic_distance";
    waypoint_topic_id = drone_id + "/command/way_point";
    // situation topic id
    liftland_topic_id = drone_id + "/command/liftland";

    vacuum_id = "uarmVacuumGripper";
    suction_topic_id = vacuum_id + "/suction_situation";

    gps_sub = nh.subscribe(gps_topic_id, 1000, &LiftWaypointPlanner::OdometryCallback, this);
    ultrasonic_sub = nh.subscribe(ultrasonic_topic_id, 1000, &LiftWaypointPlanner::UltrasonicDistanceCallback, this);
    suction_sub = nh.subscribe(suction_topic_id, 1000, &LiftWaypointPlanner::SuctionSituationCallback, this);
    waypoint_pub = nh.advertise<mav_msgs::CommandTrajectory>(waypoint_topic_id, 1000);
    situation_pub = nh.advertise<std_msgs::Bool>(liftland_topic_id, 10);
}

void LiftWaypointPlanner::SpecializeParams(int id_number)
{
    drone_id = "Quadricopter_" + std::to_string(id_number);
    gps_topic_id = drone_id + "/sensor/gps";
    ultrasonic_topic_id = drone_id + "/sensor/ultrasonic_distance";
    waypoint_topic_id = drone_id + "/command/way_point";
    // situation topic id
    liftland_topic_id = drone_id + "/command/liftland";

    vacuum_id = "uarmVacuumGripper_" + std::to_string(id_number);
    suction_topic_id = vacuum_id + "/suction_situation";

    gps_sub = nh.subscribe(gps_topic_id, 1000, &LiftWaypointPlanner::OdometryCallback, this);
    ultrasonic_sub = nh.subscribe(ultrasonic_topic_id, 1000, &LiftWaypointPlanner::UltrasonicDistanceCallback, this);
    suction_sub = nh.subscribe(suction_topic_id, 1000, &LiftWaypointPlanner::SuctionSituationCallback, this);
    waypoint_pub = nh.advertise<mav_msgs::CommandTrajectory>(waypoint_topic_id, 1000);
    situation_pub = nh.advertise<std_msgs::Bool>(liftland_topic_id, 10);

}

////////// msgs call back processing ///////////////////////////////////////////////////////////////////////
void LiftWaypointPlanner::OdometryCallback(const nav_msgs::OdometryConstPtr& gps_msg)
{
    // store gps info
    current_gps = *gps_msg;
    // needs to store initial position as x y coordinates
    if (counter == 0)
    {
        initial_gps = *gps_msg;
        object_height = current_gps.pose.pose.position.z - ultrasonic_distance - 0.5;
        counter++;
    }

    WaypointPlanning(current_gps, ultrasonic_distance, hover_height, &cmd_way_point);

    if(suction_situation)
    {
        cmd_way_point.position.z = object_height + object_desired_height + 1;
    }
    Publish();

    
    //ROS_INFO_STREAM("last suction situation: "<<std::to_string(last_suction_situation)<<" # "<<drone_id);
    //ROS_INFO_STREAM("Object height is: "<<object_height<<" # "<<drone_id);
    
}

void LiftWaypointPlanner::UltrasonicDistanceCallback(const sensor_msgs::RangeConstPtr& ultrasonic_msg)
{
    ultrasonic_distance = ultrasonic_msg->range;
}

void LiftWaypointPlanner::SuctionSituationCallback(const std_msgs::BoolConstPtr& suction_msg)
{
    suction_situation = suction_msg->data;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LiftWaypointPlanner::WaypointPlanning(nav_msgs::Odometry current_gps, double ultrasonic_distance, double hover_height, mav_msgs::CommandTrajectory *cmd_way_point)
{
    // I wish the drone can reach to the desired waypoint slowly, close to the object and hovering
    // so calculate desired waypoint, 10 centimeters higher than object surface.
    // Do not have to set the position along x and y axis, because we have made sure the drone
    // is on the top of the object. 0.09 is the distance from ultrasonic sensor to drone.

    // 0.05 is a redundancy
    cmd_way_point->position.x = initial_gps.pose.pose.position.x;
    cmd_way_point->position.y = initial_gps.pose.pose.position.y;
    cmd_way_point->position.z = current_gps.pose.pose.position.z - ultrasonic_distance + hover_height - 0.05;
}

void LiftWaypointPlanner::Publish()
{
    waypoint_pub.publish(cmd_way_point);
    situation_pub.publish(situation_msg);
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lift_wp_planning_node");
    int client_number = 6;
    drone_control::LiftWaypointPlanner lift_wp_planner[client_number];
    if(client_number==1)
    {
        lift_wp_planner[0].SpecializeFirstDroneParams();
    }else
    {
        lift_wp_planner[0].SpecializeFirstDroneParams();
        for(int i = 1; i < client_number; i++)
        {
            lift_wp_planner[i].SpecializeParams(i-1);
        }
    }

    
    ros::spin();
    return 0;
}