#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <iostream>
#include <thread>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

ros::Publisher pub;
std::string cloud_param;
std::string cloud_output;
std::string filterFieldName;

float distanceThreshold;
float filterLimitMin;
float filterLimitMax;
float pointColorThreshold;
float regionColorThreshold;
float minClusterSize;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //creating variable for initial cloud

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //creating variable for final cloud

    pcl::fromROSMsg(*cloud_msg, *raw_cloud);

    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (raw_cloud);
    pass.setFilterFieldName (filterFieldName);
    pass.setFilterLimits (filterLimitMin, filterLimitMax);
    pass.filter (*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (raw_cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (distanceThreshold);
    reg.setPointColorThreshold (pointColorThreshold);
    reg.setRegionColorThreshold (regionColorThreshold);
    reg.setMinClusterSize (minClusterSize);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    transformed_cloud = reg.getColoredCloud();

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*transformed_cloud, output_msg);

    output_msg.header.frame_id = cloud_msg->header.frame_id;
    output_msg.header.stamp = cloud_msg->header.stamp;

    pub.publish(output_msg);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_tutorial_cloud_node");
    ros::NodeHandle n;

    n.param<std::string>("cloud_param", cloud_param, "/camera/depth/color/points");
    n.param<std::string>("cloud_output", cloud_output, "/camera_depth_color_points_passthrough");

    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(cloud_param, 1, cloud_cb);

    pub = n.advertise<sensor_msgs::PointCloud2>(cloud_output, 1);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        n.param<std::string>("filter_field_name", filterFieldName, "z");

        n.param<float>("distance_threshold", distanceThreshold, 10);
        n.param<float>("filter_limit_min", filterLimitMin, 0.0);
        n.param<float>("filter_limit_max", filterLimitMax, 1.0);
        n.param<float>("point_color_threshold", pointColorThreshold, 6);
        n.param<float>("region_color_threshold", regionColorThreshold, 5);
        n.param<float>("min_cluster_size", minClusterSize, 600);
    }

    return 0;
} 

/*
#include <iostream>
#include <thread>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

using namespace std::chrono_literals;

int
main (int argc, char** argv)
{
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("region_growing_rgb_tutorial.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (10);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (5);
  reg.setMinClusterSize (600);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud (colored_cloud);
  while (!viewer.wasStopped ())
  {
    std::this_thread::sleep_for(100us);
  }

  return (0);
}
*/