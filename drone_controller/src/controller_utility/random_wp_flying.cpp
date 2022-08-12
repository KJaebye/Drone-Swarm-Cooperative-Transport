#include "random_wp_flying.h"

namespace drone_control{

RandomWayPointFlyingController::RandomWayPointFlyingController()
{
    InitializeParams();
    
    timer = nh.createTimer(ros::Duration(3.0), &RandomWayPointFlyingController::TimerCallback, this);
}

RandomWayPointFlyingController::~RandomWayPointFlyingController(){}

void RandomWayPointFlyingController::InitializeParams()
{
    space_size_x = 5;// the maximum of x axis(positive and negative)
    space_size_y = 2.5;
    // minus a half of the drone for avoiding collision with the wall
    space_size_x = space_size_x - 0.2;
    space_size_y = space_size_y - 0.2;
    wp_x = 0;
    wp_y = 0;
    wp_z = 1.5;
}

void RandomWayPointFlyingController::SpecializeFirstDroneParams()
{
    waypoint_topic_id = "Quadricopter/command/way_point";
    random_waypoint_pub = nh.advertise<mav_msgs::CommandTrajectory>(waypoint_topic_id, 10);
    random_seed = 0;
}

void RandomWayPointFlyingController::SpecializeParams(int id_number)
{
    waypoint_topic_id = "Quadricopter_" + std::to_string(id_number) + "/command/way_point";
    random_waypoint_pub = nh.advertise<mav_msgs::CommandTrajectory>(waypoint_topic_id, 10);
    random_seed = id_number + 1;
}

void RandomWayPointFlyingController::TimerCallback(const ros::TimerEvent&)
{
    //ROS_INFO("TimerCallbeck triggered.");
    //srand((int)time(0));
    // random seed by time is not useful because every agent has the same time, so using the id is better
    srand(time(0)+random_seed);
    wp_x = rand()%100;
    wp_y = rand()%100;
    // convert to the space size
    if(wp_x != 0)
    {
        wp_x = (wp_x - 50)/50 * space_size_x;
    }
    if(wp_y != 0)
    {
        wp_y = (wp_y - 50)/50 * space_size_y;
    }
    // transfer the random waypoint as cmd
    random_waypoint.position.x = wp_x;
    random_waypoint.position.y = wp_y;
    random_waypoint.position.z = wp_z;
    // publish the cmd to the drone
    random_waypoint_pub.publish(random_waypoint);
}
}