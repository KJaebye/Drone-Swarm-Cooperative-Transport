#include "flocking.h"

namespace drone_control{


FlockingController::FlockingController()
{
    InitializeParams();
}

FlockingController::~FlockingController(){}

void FlockingController::InitializeParams()
{
    near_distance = 0.5;
    far_distance = 3;

    k_sep = 1.5;//1.5
    k_coh = 1.8;//1.2
    k_frict = 0.5;
    flocking_centroid[0]=0;
    flocking_centroid[1]=0;

    speed_max = 1;// maximum speed for flocking


// set the flocking level! this is the level for flocking, must to be set
    double gains[] = {1, 1, 0, 0, 0};
    level_pid.SetGainParameters(gains);
    level = 1.5;
}


//// this part ensure every flocking algorithm on every agent can subscribe its right and distinct gps
void FlockingController::SpecializeFirstDroneParams()
{
    drone_id = "Quadricopter";
    gps_topic_id = "/Quadricopter/sensor/gps";
    gps_sub = nh.subscribe(gps_topic_id, 10, &FlockingController::OdometryCallback, this);
}
void FlockingController::SpecializeParams(int id_number)
{
    drone_id = "Quadricopter_" + std::to_string(id_number);
    gps_topic_id = "/Quadricopter_" + std::to_string(id_number) + "/sensor/gps";
    gps_sub = nh.subscribe(gps_topic_id, 10, &FlockingController::OdometryCallback, this);
}



void FlockingController::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
    odometry_height = odometry_msg->pose.pose.position.z;
}

//////////// pass the params by const reference
void FlockingController::CommandGenerate(const std::vector<std::vector<double> > &pos)
{
// add the friction between drones
    sim_time = ros::Time::now();
    dt = (sim_time - last_time).toSec();
    if(dt == 0.0) return;

    // how many drones that can be detected in the image(but not sure whether they are in the mid area)
    object_num = pos.size();
    pos_xyz.resize(object_num);
    //ROS_INFO_STREAM("size: "<<object_num);
    
    // initialize the vectors
    v_coh.resize(3);
    v_sep.resize(3);
    v_mig.resize(3);
    
    sep_sum.clear();
    sep_sum.resize(3);
    
    coh_sum.clear();
    coh_sum.resize(3);

    assumption_pos.clear();
    assumption_pos.resize(3);

    relative_velocity.clear();
    relative_velocity.resize(3);

    v_frict.clear();
    v_frict.resize(3);

    //initialize the centroid of the flocking
    flocking_centroid[0] = 0;
    flocking_centroid[1] = 0;
    // get every vector, from the pole coordinates to xyz coordinates
    for (size_t i = 0;i<object_num;i++)
    {
        double theta = pos[i][0];
        double phi = pos[i][1];
        double distance = pos[i][2];
        pos_xyz[i].resize(3);
        // get the coordinates of the objects in initial frame
        pos_xyz[i][0] = distance * sin(theta) * cos(phi);
        pos_xyz[i][1] = -distance * cos(theta) * cos(phi);

        //iteration for calculate the centroid of the flocking
        flocking_centroid[0] = flocking_centroid[0] + pos_xyz[i][0];
        flocking_centroid[1] = flocking_centroid[1] + pos_xyz[i][1];

        //pos_xyz[i][2] = distance * sin(phi);
        // minus 0.1 because the distance between green ball and drone is 0.1
        // the pos_xyz stores the position of each drone now after minus 0.1
        pos_xyz[i][2] = distance * sin(phi) - 0.1;
        //ROS_INFO_STREAM("pos_xyz: "<<pos_xyz[i][0]<<","<<pos_xyz[i][1]<<","<<pos_xyz[i][2]);

        // determine the effection area and set the angula
        if(distance <= far_distance && distance >= near_distance && fabs(phi) <= 30*M_PI/180)
        {
            // separation speed
            // ensure the value is not 0, which may cause nan problem
            if (sqrt(Dot(pos_xyz[i], pos_xyz[i])) != 0)
            {
                sep_sum[0] = sep_sum[0] + pos_xyz[i][0] / sqrt(Dot(pos_xyz[i], pos_xyz[i]));
                sep_sum[1] = sep_sum[1] + pos_xyz[i][1] / sqrt(Dot(pos_xyz[i], pos_xyz[i]));
                sep_sum[2] = sep_sum[2] + pos_xyz[i][2] / sqrt(Dot(pos_xyz[i], pos_xyz[i]));
            }

            // aggregation speed
            coh_sum[0] = coh_sum[0] + pos_xyz[i][0];
            coh_sum[1] = coh_sum[1] + pos_xyz[i][1];
            coh_sum[2] = coh_sum[2] + pos_xyz[i][2];

            // assume the other drones as one drone in order to calculate the velocity in total
            assumption_pos[0] = assumption_pos[0] + pos_xyz[i][0];
            assumption_pos[1] = assumption_pos[1] + pos_xyz[i][1];
            assumption_pos[2] = assumption_pos[2] + pos_xyz[i][2];
        }

    }
    //calculate the centroid of the flocking, this coordinate is under the body reference system
    flocking_centroid[0] = flocking_centroid[0]/(object_num+1);
    flocking_centroid[1] = flocking_centroid[1]/(object_num+1);

    // calculate the assumption drone relative velocity and friction
    relative_velocity[0] = (assumption_pos[0] - last_pos_x)/dt;
    relative_velocity[1] = (assumption_pos[1] - last_pos_y)/dt;
    relative_velocity[2] = (assumption_pos[2] - last_pos_z)/dt;
    //ROS_INFO_STREAM("relative velocity:"<<relative_velocity[0]<<", "<<relative_velocity[1]);

    v_frict[0] =  k_frict * relative_velocity[0];
    v_frict[1] =  k_frict * relative_velocity[1];
    v_frict[2] =  k_frict * relative_velocity[2];
    //ROS_INFO_STREAM("frict speed:"<<v_frict[0]<<", "<<v_frict[1]);

    // record the last pos and time
    last_pos_x = assumption_pos[0];
    last_pos_y = assumption_pos[1];
    last_pos_z = assumption_pos[2];
    last_time = ros::Time::now();
    

    // velocity of separation
    v_sep[0] = - k_sep / object_num * sep_sum[0];
    v_sep[1] = - k_sep / object_num * sep_sum[1];
    v_sep[2] = - k_sep / object_num * sep_sum[2];
    //ROS_INFO_STREAM(v_sep[0]<<","<<v_sep[1]<<","<<v_sep[2]);

    // velocity of cohision
    v_coh[0] = k_coh / object_num * coh_sum[0];
    v_coh[1] = k_coh / object_num * coh_sum[1];
    v_coh[2] = k_coh / object_num * coh_sum[2];


    /*// velocity of migration for leader follower model
    // onlt use the cohesion on verticle level when using learder-follower model
    v_mig[0] = v_coh[0] + v_sep[0];
    v_mig[1] = v_coh[1] + v_sep[1];
    v_mig[2] = v_coh[2];*/


    // velocity of migration for decentralized model
    v_mig[0] = v_coh[0] + v_sep[0] + v_frict[0];
    v_mig[1] = v_coh[1] + v_sep[1] + v_frict[1];
    // set the maximum speed for flocking
    if (v_mig[0]>0.7071*speed_max)
    {
        v_mig[0] = 0.7071*speed_max;
    }
    if (v_mig[1]>0.7071*speed_max)
    {
        v_mig[1] = 0.7071*speed_max;
    }
    flocking_speed = sqrt(v_mig[0]*v_mig[0] + v_mig[1]*v_mig[1]);
    

    // add a height controller to set the drones on the same level in the decentral model!
    // this is the flocking level, which is very important because in the decentral swarm,
    // no one knows the level they need to hover. 
    // delete this line in the leader follower model!
    //v_mig[2] = v_mig[2] + level_pid.ComputeCorrection(level, odometry_height, 0.01);
    v_mig[2] = level_pid.ComputeCorrection(level, odometry_height, 0.01);
    //ROS_INFO_STREAM("desired speed: "<<v_sep[0]+v_coh[0]<<", "<<v_sep[1]+v_coh[1]);
    //ROS_INFO_STREAM("desired speed with frict: "<<v_mig[0]<<", "<<v_mig[1]);
}

///////////////////////////////////////////////////////////////////////////////////////
///////////// Some Math Operations ////////////////////////////////////////////////////

// this return a double value which is the dot product of two vectors
double FlockingController::Dot(std::vector<double> v1, std::vector<double> v2)
{
    double dot = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
    return dot;
}

}