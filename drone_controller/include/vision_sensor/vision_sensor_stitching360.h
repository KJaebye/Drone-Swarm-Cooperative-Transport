
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


class VisionSensorImageStitching_360
{
    public:
    VisionSensorImageStitching_360();
    ~VisionSensorImageStitching_360();

    void InitializeParams();
    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);
    void PublishStitchingImage();

    private:
    // use 4 vision sensors with 90 degree view angular and stitch them into a fisheye camera

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub_front;
    image_transport::Subscriber image_sub_right;
    image_transport::Subscriber image_sub_back;
    image_transport::Subscriber image_sub_left;
    image_transport::Publisher stitching_image_pub;

    std::string vision_sensor_id_front;
    std::string vision_sensor_id_right;
    std::string vision_sensor_id_back;
    std::string vision_sensor_id_left;
    std::string vision_sensor_id_stitching;

    std::string vision_sensor_topic_id_front;
    std::string vision_sensor_topic_id_right;
    std::string vision_sensor_topic_id_back;
    std::string vision_sensor_topic_id_left;
    std::string vision_sensor_topic_id_stitching;

    cv_bridge::CvImagePtr cv_ptr_front;
    cv_bridge::CvImagePtr cv_ptr_right;
    cv_bridge::CvImagePtr cv_ptr_back;
    cv_bridge::CvImagePtr cv_ptr_left;
    sensor_msgs::ImagePtr stitching_msg;

    std::string OPENCV_WINDOW_FRONT;
    std::string OPENCV_WINDOW_RIGHT;
    std::string OPENCV_WINDOW_BACK;
    std::string OPENCV_WINDOW_LEFT;

    // inertial properties
    int stitching_resolution[2]; 
    // X and Y. resolution for each camera should be the same.
    int box_resolution[2];
    // image buffer and image properties
    Mat front_img_mirror;
    Mat right_img_mirror;
    Mat back_img_mirror;
    Mat left_img_mirror;

    Mat stitching_img;
    Mat circles_img;
    vector<Vec3f> circles; // the first value is x, second is v, third is radius

    vector<Mat> hsvSplit;
    int stitching_image_size[2];
    int H,S,V;


    // bearing and the distance variables
    vector<Vec3f> bearing_distance; // the first value is bearing, second is distance

    //ros::Publisher image_pub;

    void FrontImageCallback(const sensor_msgs::ImageConstPtr& front_image_msg);
    void RightImageCallback(const sensor_msgs::ImageConstPtr& right_image_msg);
    void BackImageCallback(const sensor_msgs::ImageConstPtr& back_image_msg);
    void LeftImageCallback(const sensor_msgs::ImageConstPtr& left_image_msg);

    void ImageProcessing();
    void MirrorTrans(const Mat &src, Mat &dst);
    void GetPredefinedArea(Mat &image_original);
    void HoughDetect(const Mat &src);
    void DroneDetect();

    
};


}
