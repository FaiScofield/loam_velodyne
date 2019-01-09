#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
//#include <math.h>
#include <iomanip>  // std::setprecision
#include <fstream>

void odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
  geometry_msgs::Point p = odom->pose.pose.position;
 // geometry_msgs::Quaternion q = odom->pose.pose.orientation;

  std::ofstream f;
  f.open("lidar_trajectory.txt", std::ios_base::app);
  f << std::fixed;
  f << std::setprecision(12) << odom->header.stamp << " " << p.x << " " << p.y << " " 
    << p.z << " " << float(0.0) << " " << float(0.0) << " " << float(0.0) << " " 
    << float(0.0) << std::endl;

  f.close();
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_odometry_KITTI");
  ros::NodeHandle nh;

  ROS_INFO("Saving lidar trajectory to lidar_trajectory.txt");
  // clean the data before
  std::ofstream f;
  f.open("lidar_trajectory.txt");
  f << std::fixed;
  f.close();

  ros::Subscriber sub = nh.subscribe("/integrated_to_init", 1000, odomCallback);

  while (ros::ok()) {
    ros::spinOnce();
  }
//  ros::spin();
  ROS_INFO("Trajectory saved.");

  return 0;
}
