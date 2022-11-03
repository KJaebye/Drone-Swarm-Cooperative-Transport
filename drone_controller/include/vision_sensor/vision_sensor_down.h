#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>


using namespace std;
using namespace cv;

namespace drone_control{

class VisionSensorDown
{
    public:

    VisionSensorDown();
    ~VisionSensorDown();
    void InitializeParams();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);
    void Publish();

    private:

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    ros::Publisher concentration_pub;// advertise how many pixels of object in the image

    std::string drone_id;
    std::string vision_sensor_id;
    std::string vision_sensor_topic_id;
    std::string concentration_topic_id;
    std::string OPENCV_WINDOW;
    

    // inertial properties
    int field_of_view;
    // X and Y
    int resolution[2]; 
    // image buffer
    Mat sensor_img;
    cv_bridge::CvImagePtr cv_ptr;

    std_msgs::Float32MultiArray concentration_msg;
    int target_pixel_number;
    
    void ImageCallback(const sensor_msgs::ImageConstPtr& image_msg);
    void ImageProcessing(Mat &src);
};

}