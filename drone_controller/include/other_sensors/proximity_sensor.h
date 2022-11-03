#include <ros/ros.h>
#include <sensor_msgs/LaserEcho.h>
#include <Eigen/Eigen>



namespace drone_control{

class ProximitySensor
{
    public:
    ProximitySensor();
    ~ProximitySensor();
    void InitializeParams();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);

    private:
    friend class ObstacleAvoidanceController;
    friend class Drone;

    ros::NodeHandle nh;
    ros::Subscriber proximity_left_sub;
    ros::Subscriber proximity_right_sub;
    ros::Subscriber proximity_front_sub;
    ros::Subscriber proximity_back_sub;
    ros::Time front_time, left_time, right_time, back_time;
    ros::Time last_left_time, last_right_time, last_front_time, last_back_time;

    std::string drone_id;
    std::string proximity_left_topic_id;
    std::string proximity_right_topic_id;
    std::string proximity_front_topic_id;
    std::string proximity_back_topic_id;

    Eigen::Matrix3d x_rotation_matrix;
    Eigen::Matrix3d y_rotation_matrix;
    Eigen::Matrix3d z_rotation_matrix;
    Eigen::Matrix3d rotation_matrix;
    Eigen::Matrix3d rotrix_front, rotrix_left, rotrix_right, rotrix_back;
    double theta, alpha, gamma;

    void LeftCallback(const sensor_msgs::LaserEchoConstPtr& proximity_left_msg);
    void RightCallback(const sensor_msgs::LaserEchoConstPtr& proximity_right_msg);
    void FrontCallback(const sensor_msgs::LaserEchoConstPtr& proximity_front_msg);
    void BackCallback(const sensor_msgs::LaserEchoConstPtr& proximity_back_msg);
    void Merge();

    Eigen::Matrix3d RotationMatrix(double gamma, double theta, double alpha);

    // parameters
    double left_distance, right_distance, front_distance, back_distance;
    double pre_left_distance, pre_right_distance, pre_front_distance, pre_back_distance;
    double left_distance_rate, right_distance_rate, front_distance_rate, back_distance_rate;
    Eigen::RowVector3d front_array, left_array, right_array, back_array;
};
}