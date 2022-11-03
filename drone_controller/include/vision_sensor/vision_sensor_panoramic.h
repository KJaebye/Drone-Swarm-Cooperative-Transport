#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <vector>

#include "flocking.h"


using namespace std;
using namespace cv;

namespace drone_control{

class VisionSensorPanoramic
{
    public:

    VisionSensorPanoramic();
    ~VisionSensorPanoramic();
    void InitializeParams();

    void SpecializeFirstDroneParams();
    void SpecializeParams(int id_number);
    void Publish();
    void Transfer(vector<vector<double> > &flocking_pos);

    private:

    friend class Drone;

    // instantiate a flocking controller to calculate the flocking cmd
    FlockingController flocking_controller;

    // camera properties
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    std::string vision_sensor_id;
    std::string vision_sensor_topic_id;

    std::string vision_back_id;
    std::string vision_back_topic_id;
    
    // image properties
    Mat panoramic_img;
    Mat circles_img;
    cv_bridge::CvImagePtr cv_ptr_panoramic;
    sensor_msgs::ImagePtr panoramic_msg;

    vector<Mat> hsvSplit;
    vector<Vec3f> circles;
    int H,S,V;

    int resolution_x;
    int resolution_y;

    // bearing and distance
    vector<double> v;
    vector<vector<double> > pos;

    Mat stats, centroids;
    int num_labels;
    double x,y,width,height,area;
    Vec2d pt;
    
    double theta, phi;
    double distance;

    
    void ImageCallback(const sensor_msgs::ImageConstPtr& image_msg);
    void ImageProcessing();
    void GetPredefinedArea(Mat &src);
    void HoughDetect(const Mat &src);
    void GetBearingDistanceByHough();

    void ConnectedComponentStatsDetect(Mat &image);
    
    
};

}