#include <ros/ros.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>


class DownsampleInputPoints
{
public:
  DownsampleInputPoints()
  {
    _leaf_size = getLeafSize();
    _subCloud = _nh.subscribe<sensor_msgs::PointCloud2>
        ("/velodyne_points_orig", 2, &DownsampleInputPoints::CloudHandler, this);
    _pubCloud = _nh.advertise<sensor_msgs::PointCloud2>
        ("/velodyne_points_down", 2);
  }

  void CloudHandler(const sensor_msgs::PointCloud2ConstPtr& CloudMsg)
  {
    setHeader(CloudMsg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*CloudMsg, *non_filtered_cloud);

    _voxel_filter.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
    _voxel_filter.setInputCloud(non_filtered_cloud);
    _voxel_filter.filter(_cloud_out_pcl);

    pcl::toROSMsg(_cloud_out_pcl, _cloud_out_ros);

    ros::Rate r(100);
//    ROS_INFO("Input cloud size: %d", int(non_filtered_cloud->size()));
//    ROS_INFO("Output cloud size: %d", int(_cloud_out_pcl.size()));

    while (ros::ok()){
      _pubCloud.publish(_cloud_out_ros);
      ros::spinOnce();
      r.sleep();
    }
  }

  // Get the leafsize  parameter
  float getLeafSize()
  {
    float leafsize;
    bool ok = ros::param::get("~leafsize", leafsize);
    if(!ok) {
      ROS_INFO("Use default leafsize: 0.33");
      leafsize = 0.33;
    }
    ROS_INFO("Setting leafsize: %f", leafsize);

    return leafsize;
  }

  void setHeader(const sensor_msgs::PointCloud2ConstPtr& msgIn)
  {
    _cloud_out_ros.header = msgIn->header;
    _cloud_out_ros.header.frame_id = "velodyne_link";
  }


private:
  ros::NodeHandle _nh;
  ros::Subscriber _subCloud;
  ros::Publisher _pubCloud;
  sensor_msgs::PointCloud2 _cloud_out_ros;
  pcl::PointCloud<pcl::PointXYZ> _cloud_out_pcl;
  pcl::VoxelGrid<pcl::PointXYZ> _voxel_filter;
  float _leaf_size;
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "downsample_input_points");

  DownsampleInputPoints dip;

  ros::spin();

  return 0;
}



