/*
    This class is for panoramic vision sensor, which subscribes image from V-REP, and then
    process the image related to a flocking  algorithm.
*/
#include "vision_sensor_panoramic.h"


namespace drone_control{


VisionSensorPanoramic::VisionSensorPanoramic():it(nh){}
VisionSensorPanoramic::~VisionSensorPanoramic()
{
    cv::destroyAllWindows();
}

void VisionSensorPanoramic::InitializeParams(){}

void VisionSensorPanoramic::SpecializeFirstDroneParams()
{
    vision_sensor_id = "panoramic";
    vision_sensor_topic_id = "Quadricopter/vision/" + vision_sensor_id;
    vision_back_id = "back";
    vision_back_topic_id = "Quadricopter/vision/" + vision_back_id;

    image_sub = it.subscribe(vision_sensor_topic_id,1000,&VisionSensorPanoramic::ImageCallback, this);
    // a publisher for advertising stitching image to vrep
    image_pub = it.advertise(vision_back_topic_id,1000);

    flocking_controller.SpecializeFirstDroneParams();
}

void VisionSensorPanoramic::SpecializeParams(int id_number)
{
    vision_sensor_id = "panoramic";
    vision_sensor_topic_id =
        "Quadricopter_" + to_string(id_number) + "/vision/" + vision_sensor_id;
    vision_back_id = "back";
    vision_back_topic_id =
        "Quadricopter_" + to_string(id_number) + "/vision/" + vision_back_id;

    image_sub = it.subscribe(vision_sensor_topic_id,1000,&VisionSensorPanoramic::ImageCallback, this);
    // a publisher for advertising stitching image to vrep
    image_pub = it.advertise(vision_back_topic_id,1000);

    flocking_controller.SpecializeParams(id_number);
}


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////

void VisionSensorPanoramic::ImageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    try
    {
        cv_ptr_panoramic = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        //std::cerr << e.what() << '\n';
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    flip(cv_ptr_panoramic->image, panoramic_img, -1);
    // get the image resolution size
    resolution_x = cv_ptr_panoramic->image.cols;
    resolution_y = cv_ptr_panoramic->image.rows;
    ImageProcessing();
    Publish();
}

void VisionSensorPanoramic::Publish()
{
    // if the back image is a gray scale image
    //panoramic_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", panoramic_img).toImageMsg();
    // if the back image is a color image for type BGR OR RBG
    panoramic_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", panoramic_img).toImageMsg();
    image_pub.publish(panoramic_msg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
///////////// image processing part //////////////////////////////////////////////////////////

void VisionSensorPanoramic::ImageProcessing()
{
/*////////// hough detect part ////////////
    
    // transform img into binarization type
    GetPredefinedArea(panoramic_img);
    circles_img = panoramic_img;


    cv::cvtColor(panoramic_img, panoramic_img, COLOR_BGR2GRAY);
    //GaussianBlur(panoramic_img, panoramic_img, Size(5,5), 0,0);
    // after experiment, the performance of median blur is better
    medianBlur(panoramic_img, panoramic_img, 5);
    cv::imshow("image gray", panoramic_img);
    cv::waitKey(3);

    // detect the green balls by using hough circle detect and draw the circles
    HoughDetect(panoramic_img);
    cv::imshow("draw circle", circles_img);
    cv::waitKey(3);
    GetBearingDistance();
///////// end ////////////*/

///////// connected area detect part //////////

    // transform img into binarization type
    GetPredefinedArea(panoramic_img);
    //imshow("image binary", panoramic_img);
    //waitKey(3);
    // this calculate the area, center point coordinates, and distance and bearing
    ConnectedComponentStatsDetect(panoramic_img);

}


void VisionSensorPanoramic::GetPredefinedArea(Mat &src)
{
    // convert image into HSV type because this type is similar to the human eyes
    cv::cvtColor(src, src, COLOR_BGR2HSV);

    // equalize the value channal to make sure the image has the suitable contrast
    hsvSplit.resize(3);
    split(src, hsvSplit);
    equalizeHist(hsvSplit[2],hsvSplit[2]);
    merge(hsvSplit, src);

    int nl = src.rows; //行数
	int nc = src.cols; //列数

    // let the green part be white
	for (int j = 0; j < nl; j++)
	{
		for (int i = 0; i < nc; i++)
		{
			// calculate very pixel
			H=src.at<Vec3b>(j, i)[0];
			S=src.at<Vec3b>(j, i)[1];
			V=src.at<Vec3b>(j, i)[2];
			//cout << "H=" << H << "S=" << S << "V=" << V << endl;
			if ((H >= 35) && (H <= 77) && (S >= 43) && (S <= 255) && (V >= 46) && (V <= 255))//检测黄色
			{
                // let the green region to be black
				src.at<Vec3b>(j, i)[0] = 0;
				src.at<Vec3b>(j, i)[1] = 0;
				src.at<Vec3b>(j, i)[2] = 255;
			}else
            {
                src.at<Vec3b>(j, i)[0] = 0;
                src.at<Vec3b>(j, i)[1] = 0;
				src.at<Vec3b>(j, i)[2] = 0;
            }
        }
    }
    cv::cvtColor(src, src, COLOR_HSV2BGR);
}

//////////////// hough circle detect ////////////////////////////////////////////////////
void VisionSensorPanoramic::HoughDetect(const Mat &src)
{
    HoughCircles(src, circles, CV_HOUGH_GRADIENT, 1.5, 20, 100, 20, 0, 40);
    // these params 
    /*ROS_INFO_STREAM("Circle:"<<circles.size());
    for (size_t i = 0; i < circles.size(); i++)
    {
        // values in circles, 1. x; 2. y; 3. radius;
        ROS_INFO_STREAM("Circle_"<<i<<":("<<circles[i][0]<<","<<circles[i][1]<<"),"<<circles[i][2]);
    }*/
    for(size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        circle(circles_img, center, 0, Scalar(0,255,0), -1, 8, 0);
        circle(circles_img, center, radius, Scalar(0,0,255), 2, 8, 0);
    }
}

void VisionSensorPanoramic::GetBearingDistanceByHough()
{
    pos.resize(0);
    for (size_t i = 0;i < circles.size();i++)
    {
    // assume that the distance and the radius have a linear corresponding relationship
    // as radius = a * distance + b;
    // as radius = a * x^2 + b * x + c -> 0 = a * x^2 + b * x + c -radius
    // here define a = -9.5, b = 19, and the distance has a constaint [0.5, 2];
    // the valus a and b getting from the MATLAB curve fitting by using several discrete data points
    // in vrep experiments
        //double a = -6;
        //double b = 18.5;
        double a = 7.8;
        double b = -30;
        double c = 35 - circles[i][2];
        if (circles[i][2]<6.1538)
        {
            circles[i][2]=6.1538;
            // this is a limit value which means the farther distance cannot be detected 
            // mathematically speaking, this value ensure the function inside the sqrt() is an 
            //absolutely positive value
        }
        double distance = (-b - sqrt(b*b-4*a*c))/(2*a);
    // this model comes from the panoramic camera, where the central of the image is the positive
    // direction of the drone.
    // where the bearing is expressed as the radian unit.
        double bearing = (circles[i][0] - resolution_x/2 + 0.5)/resolution_x *2* M_PI;
        // the first value is bearing, second is distance
        v.push_back(bearing);
        v.push_back(distance);
        pos.push_back(v);
        vector<double>().swap(v);
        
    }
   /*for(size_t i = 0; i < pos.size(); i++)
    {
        ROS_INFO_STREAM("Get the neighborhoods pos! bearing: "<<pos[i][0]<<", distance: "<<pos[i][1]);
        //ROS_INFO_STREAM("Get the neighborhoods pos! bearing and distance: "<<pos[i][0]);
    }
    ROS_INFO_STREAM("last:size="<<pos.size()<<"capacity="<<pos.capacity());*/
}
////////////////////////////////////////////////////////////////


/*
void SeedFilling(const cv::Mat& binImg, cv::Mat& lableImg)   //种子填充法
{
	// 4邻接方法
 
	if (binImg.empty() ||
		binImg.type() != CV_8UC1)
	{
		return;
	}
 
	lableImg.release();
	binImg.convertTo(lableImg, CV_32SC1);
 
	int label = 1;  
 
	int rows = binImg.rows - 1;  
	int cols = binImg.cols - 1;
	for (int i = 1; i < rows-1; i++)
	{
		int* data= lableImg.ptr<int>(i);
		for (int j = 1; j < cols-1; j++)
		{
			if (data[j] == 1)
			{
				std::stack<std::pair<int,int>> neighborPixels;   
				neighborPixels.push(std::pair<int,int>(i,j));     // 像素位置: <i,j>
				++label;  // 没有重复的团，开始新的标签
				while (!neighborPixels.empty())
				{
					std::pair<int,int> curPixel = neighborPixels.top(); //如果与上一行中一个团有重合区域，则将上一行的那个团的标号赋给它
					int curX = curPixel.first;
					int curY = curPixel.second;
					lableImg.at<int>(curX, curY) = label;
 
					neighborPixels.pop();
 
					if (lableImg.at<int>(curX, curY-1) == 1)
					{//左边
						neighborPixels.push(std::pair<int,int>(curX, curY-1));
					}
					if (lableImg.at<int>(curX, curY+1) == 1)
					{// 右边
						neighborPixels.push(std::pair<int,int>(curX, curY+1));
					}
					if (lableImg.at<int>(curX-1, curY) == 1)
					{// 上边
						neighborPixels.push(std::pair<int,int>(curX-1, curY));
					}
					if (lableImg.at<int>(curX+1, curY) == 1)
					{// 下边
						neighborPixels.push(std::pair<int,int>(curX+1, curY));
					}
				}		
			}
		}
	}
	
}
*/

void VisionSensorPanoramic::ConnectedComponentStatsDetect(Mat &image)
// this function calculate the connected area, center point, bearing and distance.
{
    // 二值化
    Mat gray, binary;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    threshold(gray, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
    // 形态学操作
    /*Mat k = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
    morphologyEx(binary, binary, MORPH_OPEN, k);
    morphologyEx(binary, binary, MORPH_CLOSE, k);*/
    Mat labels = Mat::zeros(image.size(), CV_32S);
    num_labels = connectedComponentsWithStats(binary, labels, stats, centroids, 8, 4);
    
    //vector<Vec3b> colors(num_labels);

    // background color
    //colors[0] = Vec3b(0, 0, 0);

    /*RNG rng(12345);
    // object color
    int b = rng.uniform(0, 256);
    int g = rng.uniform(0, 256);
    int r = rng.uniform(0, 256);
    for (int i = 1; i < num_labels; i++) {
        colors[i] = Vec3b(0, 255, 0);
    }*/

    // render result
    Mat dst = Mat::zeros(image.size(), image.type());
    int w = image.cols;
    int h = image.rows;
    /*for (int row = 0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            int label = labels.at<int>(row, col);
            if (label == 0) continue;
            dst.at<Vec3b>(row, col) = colors[label];
        }
    }*/

    for (int i = 1; i < num_labels; i++)
    {
        pt = centroids.at<Vec2d>(i, 0);
        x = stats.at<int>(i, CC_STAT_LEFT);
        y = stats.at<int>(i, CC_STAT_TOP);
        width = stats.at<int>(i, CC_STAT_WIDTH);
        height = stats.at<int>(i, CC_STAT_HEIGHT);
        area = stats.at<int>(i, CC_STAT_AREA);
        
        //ROS_INFO_STREAM("area: "<<area<<", center point : ["<<pt[0]<<","<<pt[1]<<"]");
        circle(panoramic_img, Point(pt[0], pt[1]), 2, Scalar(0, 0, 255), -1, 8, 0);
        rectangle(panoramic_img, Rect(x, y, width, height), Scalar(255, 0, 255), 1, 8, 0);

    /////////////////////////////////////////////////////////////////////////////////////////////
    // this model comes from the panoramic camera, where the central of the image is the positive
    // direction of the drone. However, the original of 
    // where the bearing including theta and phi, is expressed as the radian unit.
        theta = (-pt[0] + 3*resolution_x/4 + 0.5)/resolution_x *2* M_PI;
    // the model satisfies the function phi = f(y_) = 0.35*y_
        double y_ = resolution_y/2 - pt[1] + 0.5;
        phi = y_ * 0.35 / 180 * M_PI;
    ////////////////////////////////////////////////////////////////////////////////////////////
    // remove the influence of phi
    // the function in this part comes from MATLAB curve fiiting toolbox
    // area# = area * (a+c)/(a*exp(b*phi)+c*exp(d*phi)), where area# means transform area value
    // from a phi condition to phi = 0 condition, because there is no distortion on the horizontal level
        double a = 370;
        double b = -0.6;
        double c = 100;
        double d = 1.863;
        area = area * (a+c)/(a*exp(b*phi)+c*exp(d*phi));
    ////////////////////////////////////////////////////////////////////////////////////////////
    // distance 
    // using curve fiiting
    // y = a*x^b, where a = 467, b = -2
        distance = sqrtf(470/area);
    ////////////////////////////////////////////////////////////////////////////////////////////
        // the first value is bearing, second is distance
        v.push_back(theta);
        v.push_back(phi);
        v.push_back(distance);
        pos.push_back(v);
        vector<double>().swap(v);
    }

    //ROS_INFO_STREAM("Flocking Centroid is: "<<flocking_centroid);
    /*for(size_t i = 0; i < pos.size(); i++)
    {
        ROS_INFO_STREAM("Get the neighborhoods pos! theta: "<<pos[i][0]<<", phi: "<<pos[i][1]<<", distance: "<<pos[i][2]);
    }*/
    //ROS_INFO_STREAM("area number: "<< (num_labels - 1) );
    //ROS_INFO_STREAM("last:size="<<pos.size()<<"capacity="<<pos.capacity());
    
    //flocking_controller.CommandGenerate(&pos); // pass the params by pointer
    flocking_controller.CommandGenerate(pos); // pass the params by const reference

    // need to release the vector at here
    vector<vector<double> >().swap(pos);
    //imshow("object detected", panoramic_img);
    //waitKey(3);
}

void VisionSensorPanoramic::Transfer(vector<vector<double> > &flocking_pos)
{
    flocking_pos = pos;
}




}