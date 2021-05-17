#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


using namespace std;
using namespace cv;

namespace drone_control{

class VisionSensorPerspectiveType
{
    public:

    VisionSensorPerspectiveType();
    ~VisionSensorPerspectiveType();
    void InitializeParams();

    private:

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

    friend class VisionSensorImageStitching_360;
    std::string vision_sensor_id;
    std::string vision_sensor_topic_id;
    

    // inertial properties
    int field_of_view;
    // X and Y
    int resolution[2]; 
    // image buffer
    Mat sensor_img;
    cv_bridge::CvImagePtr cv_ptr;
    
    void ImageCallback(const sensor_msgs::ImageConstPtr& image_msg);
};

}