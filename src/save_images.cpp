#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>  
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

unsigned int num=1;
unsigned int numd=1;

void imgCB(const sensor_msgs::ImageConstPtr& msgRGB)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    std::stringstream s;
    s << "/home/vance/vance_ws/src/loam_velodyne/data/" << num << ".jpg";
    cv::imwrite(s.str(), cv_ptrRGB->image);
    num++;

    std::cout << "image " << num << " wrote." << std::endl;
}


void imgdCB(const sensor_msgs::ImageConstPtr& msgD)
{
    cv_bridge::CvImageConstPtr cv_ptrD;
    try {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    
    std::stringstream sd;
    sd << "/home/vance/vance_ws/src/loam_velodyne/data/" << numd << "_d.png";
    cv::imwrite(sd.str(), cv_ptrD->image);
    numd++;
    std::cout << "image depth " << numd << " wrote." << std::endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    ros::NodeHandle nh;
    std::cout << "waiting for images..." << std::endl;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber rgb_sub =  it.subscribe("/camera/rgb/image_raw", 10, imgCB);
    // image_transport::Subscriber depth_sub = it.subscribe("camera/depth_registered/poitns", 1, imgdCB);

    std::cout << "setting..." << std::endl;

    while (ros::ok()) {
      ros::spinOnce();
    }

    return 0;
}


