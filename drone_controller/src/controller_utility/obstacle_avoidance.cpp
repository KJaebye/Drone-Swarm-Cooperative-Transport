#include "obstacle_avoidance.h"

namespace drone_control{

ObstacleAvoidanceController::ObstacleAvoidanceController(){InitializeParams();}
ObstacleAvoidanceController::~ObstacleAvoidanceController(){}

void ObstacleAvoidanceController::InitializeParams()
{
    // initialize params for detection area
    max_detection_distance = 0.5 + 0.22; // 0.5 is the range and 0.22 is the radius
    mu = 0.05;// for velocity method in FIRAS
    //mu = 2;// for position method in FIRAS

    activate = false;

    v_left.resize(5);
    v_right.resize(5);
    v_front.resize(5);
    v_back.resize(5);
}

void ObstacleAvoidanceController::SpecializeFirstDroneParams()
{
    proximity_sensor.SpecializeFirstDroneParams();

    drone_id = "Quadricopter";
    gps_topic_id = drone_id + "/sensor/gps";
    gps_sub = nh.subscribe(gps_topic_id, 10, &ObstacleAvoidanceController::OdometryCallback, this);
}

void ObstacleAvoidanceController::SpecializeParams(int id_number)
{
    proximity_sensor.SpecializeParams(id_number);

    drone_id = "Quadricopter_" + to_string(id_number);
    gps_topic_id = drone_id + "/sensor/gps";
    gps_sub = nh.subscribe(gps_topic_id, 10, &ObstacleAvoidanceController::OdometryCallback, this);
}

void ObstacleAvoidanceController::OdometryCallback(const nav_msgs::OdometryConstPtr& gps_msg)
{
    VelocityGenerator();
}

void ObstacleAvoidanceController::VelocityGenerator()
{
    // check msgs are updated or not. If rostime is not changed, msg is not update which means there is no obstacle from this sensor.
    //reset activate situation
    activate = false;
    if(left_time != proximity_sensor.left_time)
    {
        v_left.push_back(1);
        v_left.push_back(proximity_sensor.left_distance);
        v_left.push_back(proximity_sensor.left_array[0]);
        v_left.push_back(proximity_sensor.left_array[1]);
        v_left.push_back(proximity_sensor.left_array[2]);
        obs.push_back(v_left);

        left_time  = proximity_sensor.left_time;
        //ROS_INFO_STREAM(1);
        activate = true;
    }
    if(right_time != proximity_sensor.right_time)
    {
        v_right.push_back(2);
        v_right.push_back(proximity_sensor.right_distance);
        v_right.push_back(proximity_sensor.right_array[0]);
        v_right.push_back(proximity_sensor.right_array[1]);
        v_right.push_back(proximity_sensor.right_array[2]);
        obs.push_back(v_right);

        right_time = proximity_sensor.right_time;
        //ROS_INFO_STREAM(2);
        activate = true;
    }
    if(front_time != proximity_sensor.front_time)
    {
        v_front.push_back(3);
        v_front.push_back(proximity_sensor.front_distance);
        v_front.push_back(proximity_sensor.front_array[0]);
        v_front.push_back(proximity_sensor.front_array[1]);
        v_front.push_back(proximity_sensor.front_array[2]);
        obs.push_back(v_front);

        front_time = proximity_sensor.front_time;
        //ROS_INFO_STREAM(3);
        activate = true;
    }
    if(back_time != proximity_sensor.back_time)
    {
        v_back.push_back(4);
        v_back.push_back(proximity_sensor.back_distance);
        v_back.push_back(proximity_sensor.back_array[0]);
        v_back.push_back(proximity_sensor.back_array[1]);
        v_back.push_back(proximity_sensor.back_array[2]);
        obs.push_back(v_back);

        back_time  = proximity_sensor.back_time;
        //ROS_INFO_STREAM(4);
        activate = true;
    }

    /*for(size_t i = 0; i < obs.size(); i++)
    {
        std::string sensor_id;
        if(obs[i][0]==1){ sensor_id = "left";}// 1 means left
        if(obs[i][0]==2){ sensor_id = "right";}// 2 means right
        if(obs[i][0]==3){ sensor_id = "front";}// 3 means front
        if(obs[i][0]==4){ sensor_id = "back";}// 4 means back
        ROS_INFO_STREAM("Obs "<<sensor_id<<": "<<obs[i][1]<<","<<obs[i][2]<<","<<obs[i][3]<<","<<obs[i][4]);
    }*/

    if(activate)
    {
        //ROS_INFO_STREAM(endl<<"Time L,R,F,B:"<<endl<<left_time<<endl<<right_time<<endl<<front_time<<endl<<back_time);
    }

    ///////// generate desired velocity for every proximity sensors ////////////////////////////////////////////////
    // by using FIRAS function: //////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // F(speed) = mu(1/rho - 1/rho_0)* 1/(rho* rho) * rho_rate; if rho <= rho_0
    //            0; if rho >= rho_0
    ////////////////////////////////////////////////////////////////////////////////////
    // store in a 2 dimension vector: potential_field_velocity
    // potential_field_velocity[i][0] shows the velocity cmd generated from which sensor
    // potential_field_velocity[i][1] shows the speed of this velocity cmd
    // potential_field_velocity[i][2]~[4] shows the direction which has been normalized
    for(size_t i = 0; i<obs.size(); i++)
    {
        if(obs[i][0]==1)// left sensor
        {
            speed_left = mu * (1/obs[i][1] - 1/max_detection_distance) * 1 / (obs[i][1] * obs[i][1]) * proximity_sensor.left_distance_rate;
            //speed_left = mu * (1/obs[i][1] - 1/max_detection_distance) * (1/obs[i][1] - 1/max_detection_distance);
            v_left_velocity.push_back(1);
            v_left_velocity.push_back(speed_left);
            v_left_velocity.push_back(-obs[i][2]);
            v_left_velocity.push_back(-obs[i][3]);
            v_left_velocity.push_back(-obs[i][4]);
            potential_field_velocity.push_back(v_left_velocity);

            //ROS_INFO_STREAM("PF speed left:"<<speed_left);
            std::string sensor_id;
            sensor_id = "left";
            //ROS_INFO_STREAM("PF "<<sensor_id<<": "<<potential_field_velocity[i][1]<<",direction:"<<potential_field_velocity[i][2]<<","<<potential_field_velocity[i][3]<<","<<potential_field_velocity[i][4]);
        }
        if(obs[i][0]==2)// means right sensor 
        {
            speed_right = mu * (1/obs[i][1] - 1/max_detection_distance) * 1 / (obs[i][1] * obs[i][1]) * proximity_sensor.right_distance_rate;
            //speed_right = 1/2 * mu * (1/obs[i][1] - 1/max_detection_distance) * (1/obs[i][1] - 1/max_detection_distance);
            v_right_velocity.push_back(1);
            v_right_velocity.push_back(speed_right);
            v_right_velocity.push_back(-obs[i][2]);
            v_right_velocity.push_back(-obs[i][3]);
            v_right_velocity.push_back(-obs[i][4]);
            potential_field_velocity.push_back(v_right_velocity);

            //ROS_INFO_STREAM("PF speed right:"<<speed_right);
            std::string sensor_id;
            sensor_id = "right";
            //ROS_INFO_STREAM("PF "<<sensor_id<<": "<<potential_field_velocity[i][1]<<",direction:"<<potential_field_velocity[i][2]<<","<<potential_field_velocity[i][3]<<","<<potential_field_velocity[i][4]);
        }
        if(obs[i][0]==3)// front sensor
        {
            speed_front = mu * (1/obs[i][1] - 1/max_detection_distance) * 1 / (obs[i][1] * obs[i][1]) * proximity_sensor.front_distance_rate;
            //speed_front = 1/2 * mu * (1/obs[i][1] - 1/max_detection_distance) * (1/obs[i][1] - 1/max_detection_distance);
            v_front_velocity.push_back(1);
            v_front_velocity.push_back(speed_front);
            v_front_velocity.push_back(-obs[i][2]);
            v_front_velocity.push_back(-obs[i][3]);
            v_front_velocity.push_back(-obs[i][4]);
            potential_field_velocity.push_back(v_front_velocity);

            //ROS_INFO_STREAM("PF speed front:"<<speed_front);
            std::string sensor_id;
            sensor_id = "front";
            //ROS_INFO_STREAM("PF "<<sensor_id<<": "<<potential_field_velocity[i][1]<<",direction:"<<potential_field_velocity[i][2]<<","<<potential_field_velocity[i][3]<<","<<potential_field_velocity[i][4]);
        }
        if(obs[i][0]==4)// back sensor
        {
            speed_back = mu * (1/obs[i][1] - 1/max_detection_distance) * 1 / (obs[i][1] * obs[i][1]) * proximity_sensor.back_distance_rate;
            //speed_back = 1/2 * mu * (1/obs[i][1] - 1/max_detection_distance) * (1/obs[i][1] - 1/max_detection_distance);
            v_back_velocity.push_back(1);
            v_back_velocity.push_back(speed_back);
            v_back_velocity.push_back(-obs[i][2]);
            v_back_velocity.push_back(-obs[i][3]);
            v_back_velocity.push_back(-obs[i][4]);
            potential_field_velocity.push_back(v_back_velocity);

            //ROS_INFO_STREAM("PF speed back:"<<speed_back);
            std::string sensor_id;
            sensor_id = "back";
            //ROS_INFO_STREAM("PF "<<sensor_id<<": "<<potential_field_velocity[i][1]<<",direction:"<<potential_field_velocity[i][2]<<","<<potential_field_velocity[i][3]<<","<<potential_field_velocity[i][4]);
        }
    }

    /*for(size_t i = 0; i < potential_field_velocity.size(); i++)
    {
        std::string sensor_id;
        if(obs[i][0]==1){ sensor_id = "left";}// 1 means left
        if(obs[i][0]==2){ sensor_id = "right";}// 2 means right
        if(obs[i][0]==3){ sensor_id = "front";}// 3 means front
        if(obs[i][0]==4){ sensor_id = "back";}// 4 means back
        ROS_INFO_STREAM("PF "<<sensor_id<<": "<<potential_field_velocity[i][1]<<","<<potential_field_velocity[i][2]<<","<<potential_field_velocity[i][3]<<","<<potential_field_velocity[i][4]);
    }*/

    // merge
    // reset merge_velocity
    merge_v[0] = 0;
    merge_v[1] = 0;
    merge_v[2] = 0;
    for(size_t i = 0; i < potential_field_velocity.size(); i++)
    {
        array = NormalizeVector(potential_field_velocity[i][2], potential_field_velocity[i][3], potential_field_velocity[i][4]);
        merge_v[0] = merge_v[0] + potential_field_velocity[i][1] * array[0];
        merge_v[1] = merge_v[1] + potential_field_velocity[i][1] * array[1];
        merge_v[2] = merge_v[2] + potential_field_velocity[i][1] * array[2];
    }
    merge_speed = sqrt(merge_v[0]*merge_v[0] + merge_v[1]*merge_v[1] + merge_v[2]*merge_v[2]);
    merge_v[3] = merge_speed;
    //ROS_INFO_STREAM("PF velocity: "<<merge_v[0]<<","<<merge_v[1]<<","<<merge_v[2]<<". Speed:"<<merge_speed<<" #"<<drone_id);


    //release vectors
    vector<double>().swap(v_left);
    vector<double>().swap(v_right);
    vector<double>().swap(v_front);
    vector<double>().swap(v_back);
    vector<vector<double> >().swap(obs);

    vector<double>().swap(v_left_velocity);
    vector<double>().swap(v_right_velocity);
    vector<double>().swap(v_front_velocity);
    vector<double>().swap(v_back_velocity);
    vector<vector<double> >().swap(potential_field_velocity);

}

double * ObstacleAvoidanceController::GetVelocityCommand()
{
    return merge_v;
}


double *ObstacleAvoidanceController::NormalizeVector(double x, double y, double z)
{
    double norm = sqrt(x * x + y * y + z * z);
    normalized_vector[0] = x/norm;
    normalized_vector[1] = y/norm;
    normalized_vector[2] = z/norm;
    
    return normalized_vector;
}

}