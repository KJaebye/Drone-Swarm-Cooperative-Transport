// discard now

#include "vision_sensor_down.h"

//static const std::string OPENCV_WINDOW = "image window";
namespace drone_control{

// initialize 'it' by using nodehandle nh
VisionSensorDown::VisionSensorDown():it(nh) {}

VisionSensorDown::~VisionSensorDown()
{
    cv::destroyAllWindows();
}

void VisionSensorDown::InitializeParams()
{
    //image_sub = it.subscribe(vision_sensor_topic_id,1000,&VisionSensorDown::ImageCallback, this);
    
    cv::namedWindow(OPENCV_WINDOW);
    target_pixel_number = 0;
}

void VisionSensorDown::SpecializeFirstDroneParams()
{
    drone_id = "Quadricopter";
    vision_sensor_id = "down";
    vision_sensor_topic_id = drone_id + "/vision/" + vision_sensor_id;
    concentration_topic_id = drone_id + "/concentration";

    // subscriber and publisher
    image_sub = it.subscribe(vision_sensor_topic_id, 1000, &VisionSensorDown::ImageCallback, this);
    concentration_pub = nh.advertise<std_msgs::Float32MultiArray>(concentration_topic_id, 10);

    // define and create an image window
    //OPENCV_WINDOW = drone_id + " Image Down";
    //cv::namedWindow(OPENCV_WINDOW);
}

void VisionSensorDown::SpecializeParams(int id_number)
{
    drone_id = "Quadricopter_" + to_string(id_number);
    vision_sensor_id = "down";
    vision_sensor_topic_id = drone_id + "/vision/" + vision_sensor_id;
    concentration_topic_id = drone_id + "/concentration";

    // subscriber and publisher
    image_sub = it.subscribe(vision_sensor_topic_id, 1000, &VisionSensorDown::ImageCallback, this);
    concentration_pub = nh.advertise<std_msgs::Float32MultiArray>(concentration_topic_id, 10);

    // define and create an image window
    //OPENCV_WINDOW = drone_id + " Image Down";
    //cv::namedWindow(OPENCV_WINDOW);
}

void VisionSensorDown::ImageCallback(const sensor_msgs::ImageConstPtr& image_msg)
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
    flip(cv_ptr->image, sensor_img, 1);
    //cv::imshow(OPENCV_WINDOW, sensor_img);
    //cv::waitKey(3);
    ImageProcessing(sensor_img);
}

// this image processing is for recording the agent opsition where correspond to the target
// object. Data is stored in a limited memory within 10.
void VisionSensorDown::ImageProcessing(Mat &src)
{
    // assuming camera is the fixed version so that pixels number of image is constant
    // check the simulator property of Vision_sensor_down can get the resolution
    // for now resolution is 256*256

    resolution[0] = src.cols;
    resolution[1] = src.rows;

    // convert image into HSV type because this type is similar to the human eyes
    cv::cvtColor(src, src, COLOR_BGR2HSV);
        // equalize the value channal to make sure the image has the suitable contrast
    vector<Mat> hsvSplit;
    hsvSplit.resize(3);
    split(src, hsvSplit);
    equalizeHist(hsvSplit[2],hsvSplit[2]);
    merge(hsvSplit, src);

    // use filter for remove salt noise
    medianBlur(src,src,3);
    int nl = src.rows;
	int nc = src.cols;

	for (int j = 0; j < nl; j++)
	{
		for (int i = 0; i < nc; i++)
		{
			// calculate every pixel
			int H=src.at<Vec3b>(j, i)[0];
			int S=src.at<Vec3b>(j, i)[1];
			int V=src.at<Vec3b>(j, i)[2];
			//cout << "H=" << H << "S=" << S << "V=" << V << endl;
			if ((H >= 26) && (H <= 34) && (S >= 43) && (S <= 255) && (V >= 46) && (V <= 255))//检测黄色
			{
                // let the green region to be black
				src.at<Vec3b>(j, i)[0] = 0;
				src.at<Vec3b>(j, i)[1] = 0;
				src.at<Vec3b>(j, i)[2] = 255;
                target_pixel_number++;
			}else
            {
                src.at<Vec3b>(j, i)[0] = 0;
                src.at<Vec3b>(j, i)[1] = 0;
				src.at<Vec3b>(j, i)[2] = 0;
            }
        }
    }
    cv::cvtColor(src, src, COLOR_HSV2BGR);
    //medianBlur(src,src,3);
    //imshow(OPENCV_WINDOW, src);
    //waitKey(3);
    // publish the concentration of this area(pixel number)
    Publish();
    // reset concentration
    target_pixel_number = 0;
}

void VisionSensorDown::Publish()
{
    concentration_msg.data.resize(3);
    concentration_msg.data[0] = target_pixel_number;
    concentration_msg.data[1] = resolution[0];
    concentration_msg.data[2] = resolution[1];

    concentration_pub.publish(concentration_msg);
}


}