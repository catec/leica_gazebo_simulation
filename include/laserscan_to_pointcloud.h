#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "pcl_conversions/pcl_conversions.h"
// #include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.h>

class ScanToPointCloud {
     public:
        ScanToPointCloud();
        void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan);

     private:
        ros::NodeHandle _node;
        laser_geometry::LaserProjection _projector;
        tf::TransformListener _tfListener;

        pcl::PointCloud<pcl::PointXYZ> _total_cloud; // complete point cloud from scene
        bool init_cloud = true; // attach header info and more to _total_cloud in first iteration

        ros::Publisher _pub;
        ros::Subscriber _sub;
};