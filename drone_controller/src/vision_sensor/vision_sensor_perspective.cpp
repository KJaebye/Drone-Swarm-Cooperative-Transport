// discard now

#include "vision_sensor_perspective.h"

static const std::string OPENCV_WINDOW = "image window";
namespace drone_control{

// initialize 'it' by using nodehandle nh
VisionSensorPerspectiveType::VisionSensorPerspectiveType():it(nh) {}

VisionSensorPerspectiveType::~VisionSensorPerspectiveType()
{
    cv::destroyAllWindows();
}

void VisionSensorPerspectiveType::InitializeParams()
{
    image_sub = it.subscribe(vision_sensor_topic_id,1000,&VisionSensorPerspectiveType::ImageCallback, this);
    
    cv::namedWindow(OPENCV_WINDOW);
}

void VisionSensorPerspectiveType::ImageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        //std::cerr << e.what() << '\n';
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

}


}