#include "vision_sensor_stitching360.h"

namespace drone_control{

VisionSensorImageStitching_360::VisionSensorImageStitching_360():it(nh){}
VisionSensorImageStitching_360::~VisionSensorImageStitching_360()
{
    cv::destroyAllWindows();
}

void VisionSensorImageStitching_360::InitializeParams() {}

void VisionSensorImageStitching_360::SpecializeFirstDroneParams()
{
    vision_sensor_id_front = "front";
    vision_sensor_topic_id_front = "Quadricopter/vision/" + vision_sensor_id_front;
    vision_sensor_id_right = "right";
    vision_sensor_topic_id_right = "Quadricopter/vision/" + vision_sensor_id_right;
    vision_sensor_id_back = "back";
    vision_sensor_topic_id_back = "Quadricopter/vision/" + vision_sensor_id_back;
    vision_sensor_id_left = "left";
    vision_sensor_topic_id_left = "Quadricopter/vision/" + vision_sensor_id_left;
    vision_sensor_id_stitching = "stitching";
    vision_sensor_topic_id_stitching = "Quadricopter/vision/" + vision_sensor_id_stitching;
    

    image_sub_front = it.subscribe(vision_sensor_topic_id_front,1000,&VisionSensorImageStitching_360::FrontImageCallback, this);
    //std::string OPENCV_WINDOW_FRONT = vision_sensor_id_front + "image window";
    //cv::namedWindow(OPENCV_WINDOW_FRONT);
    image_sub_right = it.subscribe(vision_sensor_topic_id_right,1000,&VisionSensorImageStitching_360::RightImageCallback, this);
    //std::string OPENCV_WINDOW_RIGHT = vision_sensor_id_right + "image window";
    //cv::namedWindow(OPENCV_WINDOW_RIGHT);
    image_sub_back = it.subscribe(vision_sensor_topic_id_back,1000,&VisionSensorImageStitching_360::BackImageCallback, this);
    //std::string OPENCV_WINDOW_BACK = vision_sensor_id_back + "image window";
    //cv::namedWindow(OPENCV_WINDOW_BACK);
    image_sub_left = it.subscribe(vision_sensor_topic_id_left,1000,&VisionSensorImageStitching_360::LeftImageCallback, this);
    //std::string OPENCV_WINDOW_LEFT = vision_sensor_id_left + "image window";
    //cv::namedWindow(OPENCV_WINDOW_LEFT);

    // a publisher for advertising stitching image to vrep
    stitching_image_pub = it.advertise(vision_sensor_topic_id_stitching,1);
}

void VisionSensorImageStitching_360::SpecializeParams(int id_number)
{
    vision_sensor_id_front = "front";
    vision_sensor_topic_id_front = 
        "Quadricopter_" + to_string(id_number) + "/vision/" + vision_sensor_id_front;
    vision_sensor_id_right = "right";
    vision_sensor_topic_id_right = 
        "Quadricopter_" + to_string(id_number) + "/vision/" + vision_sensor_id_right;
    vision_sensor_id_back = "back";
    vision_sensor_topic_id_back = 
        "Quadricopter_" + to_string(id_number) + "/vision/" + vision_sensor_id_back;
    vision_sensor_id_left = "left";
    vision_sensor_topic_id_left = 
        "Quadricopter_" + to_string(id_number) + "/vision/" + vision_sensor_id_left;

    image_sub_front = it.subscribe(vision_sensor_topic_id_front,1000,&VisionSensorImageStitching_360::FrontImageCallback, this);
    //std::string OPENCV_WINDOW_FRONT = vision_sensor_id_front + "image window";
    //cv::namedWindow(OPENCV_WINDOW_FRONT);
    image_sub_right = it.subscribe(vision_sensor_topic_id_right,1000,&VisionSensorImageStitching_360::RightImageCallback, this);
    //std::string OPENCV_WINDOW_RIGHT = vision_sensor_id_right + "image window";
    //cv::namedWindow(OPENCV_WINDOW_RIGHT);
    image_sub_back = it.subscribe(vision_sensor_topic_id_back,1000,&VisionSensorImageStitching_360::BackImageCallback, this);
    //std::string OPENCV_WINDOW_BACK = vision_sensor_id_back + "image window";
    //cv::namedWindow(OPENCV_WINDOW_BACK);
    image_sub_left = it.subscribe(vision_sensor_topic_id_left,1000,&VisionSensorImageStitching_360::LeftImageCallback, this);
    //std::string OPENCV_WINDOW_LEFT = vision_sensor_id_left + "image window";
    //cv::namedWindow(OPENCV_WINDOW_LEFT);
}

void VisionSensorImageStitching_360::PublishStitchingImage()
{
    stitching_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", stitching_img).toImageMsg();
    stitching_image_pub.publish(stitching_msg);
}


//////////////////////////////////////////////////////////////////////////////////////////////
////////////// Image Callback function ///////////////////////////////////////////////////////

void VisionSensorImageStitching_360::FrontImageCallback(const sensor_msgs::ImageConstPtr& front_image_msg)
{
    try
    {
        cv_ptr_front = cv_bridge::toCvCopy(front_image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        //std::cerr << e.what() << '\n';
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    MirrorTrans(cv_ptr_front->image, front_img_mirror);
    //cv::imshow(OPENCV_WINDOW_FRONT, front_img_mirror);
    //cv::waitKey(3);
}

void VisionSensorImageStitching_360::RightImageCallback(const sensor_msgs::ImageConstPtr& right_image_msg)
{
    try
    {
        cv_ptr_right = cv_bridge::toCvCopy(right_image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        //std::cerr << e.what() << '\n';
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    MirrorTrans(cv_ptr_right->image, right_img_mirror);
    //cv::imshow(OPENCV_WINDOW_RIGHT, right_img_mirror);
    //cv::waitKey(3);
}

void VisionSensorImageStitching_360::BackImageCallback(const sensor_msgs::ImageConstPtr& back_image_msg)
{
    try
    {
        cv_ptr_back = cv_bridge::toCvCopy(back_image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        //std::cerr << e.what() << '\n';
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    MirrorTrans(cv_ptr_back->image, back_img_mirror);
    //cv::imshow(OPENCV_WINDOW_BACK, back_img_mirror);
    //cv::waitKey(3);
}

void VisionSensorImageStitching_360::LeftImageCallback(const sensor_msgs::ImageConstPtr& left_image_msg)
{
    try
    {
        cv_ptr_left = cv_bridge::toCvCopy(left_image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        //std::cerr << e.what() << '\n';
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    MirrorTrans(cv_ptr_left->image, left_img_mirror);
    //cv::imshow(OPENCV_WINDOW_LEFT, left_img_mirror);
    //cv::waitKey(3);

    ImageProcessing();
    PublishStitchingImage();

}

//////////////////////////////////////////////////////////////////////////////////////////////
////////////// Image Processing function /////////////////////////////////////////////////////

void VisionSensorImageStitching_360::ImageProcessing()
{
    // image stitching simply
    // hconcat() function is a function for horizontal stitching
    cv::Mat stitching_1, stitching_2;
    hconcat(front_img_mirror, right_img_mirror, stitching_1);
    hconcat(back_img_mirror, left_img_mirror, stitching_2);
    hconcat(stitching_1, stitching_2, stitching_img);
    

    // transform img into binarization type
    GetPredefinedArea(stitching_img);
    circles_img = stitching_img;
    cvtColor(stitching_img, stitching_img, COLOR_BGR2GRAY);
    cv::imshow("stitching image binarization", stitching_img);
    cv::waitKey(3);

    // detect the green balls by using hough circle detect
    HoughDetect(stitching_img);
    cv::imshow("draw circle", circles_img);
    cv::waitKey(3);

}

//////////////////////////////////////////////////////////////////////////////////////////////
////////////// Some useful function /////////////////////////////////////////////////////

void VisionSensorImageStitching_360::GetPredefinedArea(Mat &image_original)
{
    // convert image into HSV type because this type is similar to the human eyes
    cvtColor(image_original, image_original, COLOR_BGR2HSV);

    // equalize the value channal to make sure the image has the suitable contrast
    hsvSplit.resize(3);
    split(image_original, hsvSplit);
    equalizeHist(hsvSplit[2],hsvSplit[2]);
    merge(hsvSplit, image_original);

    int nl = image_original.rows; //行数
	int nc = image_original.cols; //列数

    // let the green part be white
	for (int j = 0; j < nl; j++)
	{
		for (int i = 0; i < nc; i++)
		{
			// calculate very pixel
			H=image_original.at<Vec3b>(j, i)[0];
			S=image_original.at<Vec3b>(j, i)[1];
			V=image_original.at<Vec3b>(j, i)[2];
			//cout << "H=" << H << "S=" << S << "V=" << V << endl;
			if ((H >= 35) && (H <= 77) && (S >= 43) && (S <= 255) && (V >= 46) && (V <= 255))//检测黄色
			{
				image_original.at<Vec3b>(j, i)[0] = 0;
				image_original.at<Vec3b>(j, i)[1] = 0;
				image_original.at<Vec3b>(j, i)[2] = 255;
			}else
            {
                image_original.at<Vec3b>(j, i)[0] = 0;
                image_original.at<Vec3b>(j, i)[1] = 0;
				image_original.at<Vec3b>(j, i)[2] = 0;
            }
            
        }
    }
    cvtColor(image_original, image_original, COLOR_HSV2BGR);
}


// this is a mirror transform function, because of the pinhole camera principle
void VisionSensorImageStitching_360::MirrorTrans(const Mat &src, Mat &dst)
{
    CV_Assert(src.depth() == CV_8U);
    dst.create(src.rows, src.cols, src.type());

    int rows = src.rows;
    int cols = src.cols;

    switch (src.channels())
    {
    case 1:
        const uchar *origal;
        uchar *p;
        for (int i = 0; i < rows; i++){
            origal = src.ptr<uchar>(i);//ptr<>函数得到一行的指针，并用[]操作符访问某一列的像素值
            p = dst.ptr<uchar>(i);
            for (int j = 0; j < cols; j++){
                p[j] = origal[cols - 1 - j];
            }
        }
        break;
    case 3:
        const Vec3b *origal3;
        Vec3b *p3;
        for (int i = 0; i < rows; i++) {
            origal3 = src.ptr<Vec3b>(i);
            p3 = dst.ptr<Vec3b>(i);
            for (int j = 0; j < cols; j++){
                p3[j] = origal3[cols - 1 - j];
            }
        }
        break;
    default:
        break;
    }
}


void VisionSensorImageStitching_360::HoughDetect(const Mat &src)
{
    HoughCircles(src, circles, CV_HOUGH_GRADIENT, 2, 2, 130, 20, 5, 60);
    // these params 
    ROS_INFO_STREAM("Circle:"<<circles.size());
    for (size_t i = 0; i < circles.size(); i++)
    {
        ROS_INFO_STREAM("Circle_"<<i<<":("<<circles[i][0]<<","<<circles[i][1]<<"),"<<circles[i][2]);
    }
    for(size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        circle(circles_img, center, 0, Scalar(0,255,0), -1, 8, 0);
        circle(circles_img, center, radius, Scalar(0,0,255), 2, 8, 0);
    }
}

void VisionSensorImageStitching_360::DroneDetect()
{
    
    for (size_t i = 0; i < circles.size(); i++)
    {
        // getting the bearing, firstly divide the x pixel-axis into 2pi
        bearing_distance[i][0] = circles[i][0]/circles_img.cols * 2 * M_PI;
    }
}


}

