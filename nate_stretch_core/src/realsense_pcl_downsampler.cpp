#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>

ros::Publisher pub;
std::string cloud_param;
std::string cloud_output;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //creating variable for initial cloud

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //creating variable for final cloud

    pcl::fromROSMsg(*cloud_msg, *raw_cloud)

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (raw_cloud);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (downsampled_cloud);

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*downsampled_cloud, output_msg);

    output_msg.header.frame_id = cloud_msg->header.frame_id;
    output_msg.header.stamp = cloud_msg->header.stamp;

    pub.publish(output_msg);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_tutorial_cloud_node");
    ros::NodeHandle n;

    n.param<std::string>("cloud_param", cloud_param, "/realsense/depth/color/points");
    n.param<std::string>("cloud_output", cloud_output, "/realsense_depth_color_points_downsample");

    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(cloud_param, 1, cloud_cb);

    pub = n.advertise<sensor_msgs::PointCloud2>(cloud_output, 1);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 
