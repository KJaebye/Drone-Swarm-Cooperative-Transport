#include <nav_msgs/Odometry.h>
#include <vector>
#include <Eigen/Eigen>

#include "proximity_sensor.h"

using namespace std;

namespace drone_control{

class ObstacleAvoidanceController
{
    public:
    ObstacleAvoidanceController();
    ~ObstacleAvoidanceController();
    void InitializeParams();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);
    
    double * GetVelocityCommand();

    ProximitySensor proximity_sensor;
    ros::Time front_time, left_time, right_time, back_time;
    bool activate;

    private:
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;

    std::string drone_id;
    std::string gps_topic_id;

    double merge_v[4];
    double merge_speed;
    double normalized_vector[3];

    vector<double> v_left, v_right, v_front, v_back;
    vector<vector<double> > obs;
    vector<double> v_left_velocity, v_right_velocity, v_front_velocity, v_back_velocity;
    vector<vector<double> > potential_field_velocity;


    void VelocityGenerator();
    void OdometryCallback(const nav_msgs::OdometryConstPtr& gps_msg);
    double *NormalizeVector(double x, double y, double z);
    
    double *array;
    double speed_left, speed_right, speed_front, speed_back;

    // params
    double max_detection_distance;
    double mu;
};


}