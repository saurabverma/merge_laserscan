
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// Global variable
ros::NodeHandle node_;
laser_geometry::LaserProjection projector_;
tf::TransformListener tfListener_;

ros::Publisher scan_pub_;
ros::Subscriber scan_sub_;

pcl::PointCloud<pcl::PointXYZ> merged_cloud_;

// For every data that comes in with respect to ?? TIME ??
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    // Convert Laserscan to ROS
    sensor_msgs::PointCloud2Ptr pointcloud_msg;
    projector_.transformLaserScanToPointCloud("base_link", *scan, *pointcloud_msg, tfListener_);

    // Convert from ROS to PCL
    pcl::PointCloud::Ptr received_cloud_ptr;
    received_cloud_ptr.reset(new pcl::PointCloud);
    pcl::fromROSMsg(*pointcloud_msg.get(), *received_cloud_ptr.get());

    // Transform Pointcloud
    Eigen::Affine3f tf = Eigen::Affine3f::Identity();
    tf.translation() << 2.5, 0.0, 0.0;                             // Define a translation of 2.5 meters on the x axis.
    tf.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ())); // The same rotation matrix as before; theta radians around Z axis
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*received_cloud_ptr, *transformed_cloud, tf);

    // Merge all Pointcloud
    merged_cloud_ += transformed_cloud;
}

int main()
{
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan>("/scans", 100, scanCallback, this);
    scan_pub_ = node_.advertise<sensor_msgs::LaserScan>("/merged_scan", 100, false);

    // TODO:
    while ()
    {
        // Convert from PCL to ROS
        sensor_msgs::PointCloud2 merged_cloud;
        pcl::toROSMsg(*merged_cloud_.get(), merged_cloud);

        // Convert ROS to Laserscan
        // TODO:
        PointCloudToLaserScanNodelet projector;
        projector.transformLaserScanToPointCloud("base_link", *scan, *pointcloud_msg, tfListener_);

        // Publish and clear data
        scan_pub_.publish(merged_scan);
        merged_cloud_.points.clear()
    }
}