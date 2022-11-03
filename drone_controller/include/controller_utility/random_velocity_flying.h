#include <ros/ros.h>
#include <time.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <nav_msgs/Odometry.h>

namespace drone_control{

class RandomVelocityFlyingController
{
    public:
    RandomVelocityFlyingController();
    ~RandomVelocityFlyingController();

    void InitializeParams();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);
    double *GetVelocityCommand();

    double current_concentration;
    double previous_concentration;

    private:
    ros::NodeHandle nh;
    ros::Timer timer;
    ros::Subscriber concentration_sub;
    ros::Subscriber gps_sub;

    std::string drone_id;
    std::string concentration_topic_id;
    std::string gps_topic_id;

    // the range of the experiment aera, first is the min, and second is the max
    double field_x_min, field_x_max;
    double field_y_min, field_y_max;
    double estimated_position[3];

    double position[3];
    double orientation[4];
    double random_bearing;
    double speed;
    double max_speed;
    double velocity_x, velocity_y, velocity_z;
    int random_seed;
    double velocity[4];

///// bacterium behaviour params
    double T;// run time
    double T0;// mean run time in the absence of concentration gradient
    double Tm;
    // indicate how much part of the object that drone can see by look down camera
    // this gives how many pixels inside the view_flied to replace the concentration
    int target_pixel_number;
    int resolution[2]; //X & Y
    
    
    
    double kd;// the chemical sensitivity of the bacteria
    double alpha;// the amplification factor of the bacterial system

    double dP_dt, dC_dt;
    double dP_dt_weight;
    int threshold_value;

    int memory_capacity;
    std::vector<double> concentration_record;
    std::vector<double> p_rate_record;

    ros::Time current_time;
    ros::Time previous_time;
    ros::Duration run_time;

    int counter;

    //void TimerCallback(const ros::TimerEvent&);
    void ConcentrationCallback(const std_msgs::Float32MultiArrayConstPtr& concentration_msg);
    void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
    void RandomVelocityGenerator();

};



}