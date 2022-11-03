#include "random_velocity_flying.h"
#include <iomanip>

namespace drone_control{

RandomVelocityFlyingController::RandomVelocityFlyingController()
{

    InitializeParams();
    //timer = nh.createTimer(ros::Duration(T), &RandomVelocityFlyingController::TimerCallback, this);
}

RandomVelocityFlyingController::~RandomVelocityFlyingController()
{
    concentration_record.clear();
    p_rate_record.clear();
}

void RandomVelocityFlyingController::InitializeParams()
{
    field_x_min = -7.5;
    field_x_max = 7.5;
    field_y_min = -7.5;
    field_y_max = 7.5;
    
    velocity_x = 0;
    velocity_y = 0;
    velocity_z = 0;
    max_speed = 1;

    T = 0;
    T0 = 60;
    threshold_value = 30;
    Tm = 1;//2
    alpha = 1e5;//300 6000
    kd = 30;//default 30

    dP_dt = 0;
    dC_dt = 0;
    dP_dt_weight = 0;
    current_concentration = 0;
    previous_concentration = 0;

    memory_capacity = 10;
    concentration_record.resize(memory_capacity);
    p_rate_record.resize(memory_capacity);

    counter = 0;
}

// specialize the drone for getting random seed
void RandomVelocityFlyingController::SpecializeFirstDroneParams()
{
    random_seed = 0;

    drone_id = "Quadricopter";
    concentration_topic_id = drone_id + "/concentration";
    gps_topic_id = drone_id + "/sensor/gps";

    concentration_sub = nh.subscribe(concentration_topic_id, 10, &RandomVelocityFlyingController::ConcentrationCallback, this);
    gps_sub = nh.subscribe(gps_topic_id, 10, &RandomVelocityFlyingController::OdometryCallback, this);
}
void RandomVelocityFlyingController::SpecializeParams(int id_number)
{
    random_seed = id_number + 1;

    drone_id = "Quadricopter_" + std::to_string(id_number);
    concentration_topic_id = drone_id + "/concentration";
    gps_topic_id = drone_id + "/sensor/gps";

    concentration_sub = nh.subscribe(concentration_topic_id, 10, &RandomVelocityFlyingController::ConcentrationCallback, this);
    gps_sub = nh.subscribe(gps_topic_id, 10, &RandomVelocityFlyingController::OdometryCallback, this);
}

/*void RandomVelocityFlyingController::TimerCallback(const ros::TimerEvent&)
{
    //ROS_INFO("TimerCallbeck triggered.");
    //srand((int)time(0));
    // random seed by time is not useful because every agent has the same time, so using the id is better
    srand(time(0)+random_seed);
    // get a random bearing for the agent and bearing belongs to [0,360]
    random_bearing = rand()%360;
    // convert into radian
    random_bearing = (random_bearing - 180)/180 * M_PI;
    // generate velocity cmd
    velocity_x = speed * cos(random_bearing);
    velocity_y = speed * sin(random_bearing);
}*/

double *RandomVelocityFlyingController::GetVelocityCommand()
{
    //velocity[0] = velocity_x;
    //velocity[1] = velocity_y;
    //velocity[2] = velocity_z;
    return velocity;
}

void RandomVelocityFlyingController::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
    position[0] = odometry_msg->pose.pose.position.x;
    position[1] = odometry_msg->pose.pose.position.y;
    position[2] = odometry_msg->pose.pose.position.z;

    orientation[0] = odometry_msg->pose.pose.orientation.w;
    orientation[1] = odometry_msg->pose.pose.orientation.x;
    orientation[2] = odometry_msg->pose.pose.orientation.y;
    orientation[3] = odometry_msg->pose.pose.orientation.z;

    //ROS_INFO_STREAM("Position: "<<position[0]<<", "<<position[1]<<", "<<position[2]);
}

void RandomVelocityFlyingController::ConcentrationCallback(const std_msgs::Float32MultiArrayConstPtr& concentration_msg)
{
    counter++;
    
    /*/////////////// this part is for image concentration /////////////////////////////////////////////////
    target_pixel_number = concentration_msg->data[0];
    //ROS_INFO_STREAM("pixel number: "<<target_pixel_number);
    resolution[0] = concentration_msg->data[1];
    resolution[1] = concentration_msg->data[2];
    current_concentration = (double)target_pixel_number/(resolution[0] * resolution[1])*50;
    current_concentration = floor(current_concentration);
    //ROS_INFO_STREAM("Current concentrtion: "<<current_concentration);*/


    ////////////// this part is for distance concetration ///////////////////////////////////////////////////
    current_concentration = concentration_msg->data[0];
    //ROS_INFO_STREAM("Current concentrtion: "<<current_concentration);



    /*if (current_concentration < threshold_value)
    {
        dP_dt = kd/((kd + current_concentration)*(kd + current_concentration)) * dC_dt;
    }else
    {
        dP_dt = - kd/((kd + current_concentration)*(kd + current_concentration)) * dC_dt;
    }*/
    
    dP_dt = kd/((kd + current_concentration)*(kd + current_concentration)) * dC_dt;
    //ROS_INFO_STREAM("dP_dt: "<<dP_dt);
    //ROS_INFO_STREAM("dC_dt: "<<dC_dt);

    // record the data which has 10 elements memory
    auto it = concentration_record.begin();
    concentration_record.erase(it);
    concentration_record.push_back(current_concentration);
    /*for (size_t i = 0; i < memory_capacity; i++)
    {
        ROS_INFO_STREAM(concentration_record[i]);
    }*/

    auto it1 = p_rate_record.begin();
    p_rate_record.erase(it1);
    p_rate_record.push_back(dP_dt);
    /*ROS_INFO_STREAM("p_rate_record:");
    for (size_t i = 0; i < memory_capacity; i++)
    {
        ROS_INFO_STREAM(p_rate_record[i]);
    }*/
    

    for (int i = 0; i < memory_capacity; i++)
    {
        // dP_dt_w is the weighted rate of the change of P
        if (i==0)
        {
            dP_dt_weight = dP_dt_weight + p_rate_record[i] * exp(i/Tm);
        }else
        {
            dP_dt_weight = dP_dt_weight + p_rate_record[i] * exp((i-10)/Tm);
        }
    }
    
    if (current_concentration < 0)
    {
        current_concentration = 0;
    }

    speed = max_speed/(current_concentration*10 + 1);
    //ROS_INFO_STREAM("Bacterium speed:"<<speed);
    dP_dt_weight = dP_dt_weight/Tm;
    //dP_dt_weight = - fabs(dP_dt_weight);
    //ROS_INFO_STREAM("dP_dt_weight: "<<dP_dt_weight<<" # "<<drone_id);
    T = T0 * exp(alpha * dP_dt_weight);
    //ROS_INFO_STREAM("Tao:"<<T<<" # "<<drone_id);
    

    dP_dt_weight = 0;
    //ROS_INFO_STREAM("Previous concentration: "<<previous_concentration);
    dC_dt = current_concentration - previous_concentration;
    previous_concentration = current_concentration;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // this part is for random direction generate(caculate the counter)
    //ROS_INFO_STREAM("Counter: "<<counter);
    if(counter > T)
    {
        RandomVelocityGenerator();
        //ROS_INFO_STREAM("Bacterium Speed of "<<drone_id<<":"<<speed<<", x="<<velocity[0]<<", y="<<velocity[1]);
        counter = 0;
    }
    
}

void RandomVelocityFlyingController::RandomVelocityGenerator()
{
    // random seed by time is not useful because every agent has the same time, so using the id is better
    srand(time(0)+random_seed);
    // get a random bearing for the agent and bearing belongs to [0,360]
    random_bearing = rand()%360;
    // convert into radian
    random_bearing = (random_bearing - 180)/180 * M_PI;
    // generate velocity cmd
    velocity_x = speed * cos(random_bearing);
    velocity_y = speed * sin(random_bearing);

    /*// determine the drones' position, chech it is in the predefined field
    // if not then execute the while loop until it is in.
    current_time = ros::Time().now();
    ros::Duration dt = current_time - previous_time;

    estimated_position[0] = velocity_x * dt.toSec() + position[0];
    estimated_position[1] = velocity_y * dt.toSec() + position[1];
    estimated_position[2] = velocity_z * dt.toSec() + position[2];

    // if check the drone would get out of the area then inverse the velocity
    if(estimated_position[0]<field_x_min || estimated_position[1]>field_x_max)
    {
        velocity_x = - velocity_x;
    }
    if(estimated_position[1]<field_y_min || estimated_position[1]>field_y_max)
    {
        velocity_y = - velocity_y;
    }*/

    if(position[0]<field_x_min+1 || position[0]>field_x_max-1)
    {
        velocity_x = - max_speed * position[0]/sqrt(position[0]*position[0] + position[1]*position[1]);
    }
    if(position[1]<field_y_min+1 || position[1]>field_y_max-1)
    {
        velocity_y = - max_speed * position[1]/sqrt(position[0]*position[0] + position[1]*position[1]);
    }

    velocity[0] = velocity_x;
    velocity[1] = velocity_y;
    velocity[2] = velocity_z;
    velocity[3] = speed;

    previous_time = current_time;
}
}