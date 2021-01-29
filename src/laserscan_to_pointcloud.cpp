#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "std_msgs/String.h"
#include "leica_scanstation_utils/LeicaUtils.h"
#include "laserscan_to_pointcloud.h"

ScanToPointCloud::ScanToPointCloud(ros::NodeHandle nh)
    :nh_(nh)
{
    total_cloud_ = boost::make_shared<PointCloudXYZ>();

    // Subscribe to the topic where the outline of the object that sees the laser is published
    sub_ = nh_.subscribe<sensor_msgs::LaserScan>("simulator/scan", 100, &ScanToPointCloud::scanCb, this);
    filename_sub_ = nh_.subscribe<std_msgs::String>("simulator/filename", 100, &ScanToPointCloud::filenameCb, this);

    // Create a topic where the point cloud will be published
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("simulator/cloud", 100, false);

    // Publish service so that it can be invoked when requested
    store_sub_ = nh_.advertiseService("store_cloud", &ScanToPointCloud::saveCloudCb, this);

    counter_ = 0;
}

void ScanToPointCloud::filenameCb(const std_msgs::String::ConstPtr& msg)
{
    file_name_ = LeicaUtils::getFilePath(msg->data, ".pcd", counter_);
    counter_++;
}

bool ScanToPointCloud::saveCloudCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ROS_INFO("Saving the point cloud...");

    // Save the point cloud on the hard disk
    pcl::io::savePCDFileASCII(file_name_, *total_cloud_);
    ROS_INFO("Saved %zu data points to %s", total_cloud_->size(), file_name_.c_str());

    // Reset the cloud after saving it
    total_cloud_.reset(new PointCloudXYZ);

    return true;
}

void ScanToPointCloud::scanCb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // Variables to store the current point cloud and the new chunk
    sensor_msgs::PointCloud2 cloud,total_cloud_msg;

    // Pointer to save ros scan info and update total_cloud_
    PointCloudXYZ::Ptr cloud_pcl;
    cloud_pcl.reset(new PointCloudXYZ);

    // Find that the tf you need to relate the frame of the sensor to the world frame exists
    if(tf_listener_.waitForTransform(
        scan->header.frame_id,
        "/world",
        scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
        ros::Duration(1.0)))
    { 
        // Transform the continuous line that sees the laser to a series of points
        projector_.transformLaserScanToPointCloud("/world", *scan, cloud, tf_listener_);
        
        // Convert to PCL is necessary to add new scan data to total_cloud_
        pcl::fromROSMsg(cloud, *cloud_pcl);
        
        // Execute this once
        if (init_cloud_)
        {
            *total_cloud_ = *cloud_pcl;
            init_cloud_ = false;
        }
        
        // Add the new set of point cloud to the current point cloud
        *total_cloud_ += *cloud_pcl;
        
        // Convert again to ros msg and publish
        pcl::toROSMsg(*total_cloud_, total_cloud_msg);
        pub_.publish(total_cloud_msg);
    }
}

int main(int argc, char** argv)
{
    // Launch the process of transforming the laser scan into the point cloud
    ros::init(argc, argv, "ScanToPointCloud");
    ros::NodeHandle nh;

    ROS_INFO("tf from laserscan to pointcloud");
    ScanToPointCloud cloud_converter(nh);
    
    ros::spin();

    return 0;
}
