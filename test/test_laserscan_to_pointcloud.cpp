#include <ros/ros.h>
#include <gtest/gtest.h>
#include "sensor_msgs/PointCloud2.h"
#include "std_srvs/Trigger.h"
#include "boost/filesystem.hpp"
#include "leica_scanstation_utils/LeicaUtils.h"


// Test laserscan_to_pointcloud publish total point cloud from laser_scan rosbag
TEST(TestNode, testTotalPointCloudPublisher)
{
    sensor_msgs::PointCloud2ConstPtr total_cloud_msg; 

    total_cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("simulator/cloud");

    EXPECT_TRUE((total_cloud_msg->row_step*total_cloud_msg->height) != 0);
}

// Test call save_pcloud service in laserscan_to_pointcloud, it should return success
TEST(TestNode, testSaveCloudService)
{
    ros::NodeHandle nh;
    ros::ServiceClient client;
    client = nh.serviceClient<std_srvs::Trigger>("store_cloud");
    bool exists = client.waitForExistence(ros::Duration(1));
    ASSERT_TRUE(exists);

    std_srvs::Trigger srv;
    
    if(client.call(srv))
    {
        // check if file exist
        std::string path = LeicaUtils::getFilePath("scan.pcd");
        ROS_INFO("Check for existance %s", path.c_str());

        ASSERT_TRUE(boost::filesystem::exists(path));
    }
    else ADD_FAILURE()<< "Failed to call service";
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "test_laserscan_to_pointcloud");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}