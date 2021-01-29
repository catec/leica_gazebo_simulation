#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "pcl_conversions/pcl_conversions.h"
#include "leica_scanstation_msgs/Scan.h"

#define TO_MM(x) x*1000

class LeicaServiceListener 
{
public:
    /**
     * @brief Construct a new Leica Service Listener object
     * 
     * @param nh 
     */
    LeicaServiceListener(ros::NodeHandle nh);

private:
    /**
     * @brief Callback for receiven service request. It will order information and publish to topics.
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */
    bool scanCb(leica_scanstation_msgs::Scan::Request& req, leica_scanstation_msgs::Scan::Response& res);
        
    /**
     * @brief Pointer to the ROS node
     * 
     */
    ros::NodeHandle nh_;

    /**
     * @brief Publisher for the point cloud file name to store
     * 
     */
    ros::Publisher name_pub_;

    /**
     * @brief Publisher for indicating scan parameters
     * 
     */ 
    ros::Publisher window_pub_;

    /**
     * @brief Publisher for indicating scan resolution
     * 
     */
    ros::Publisher resolution_pub_;

    /**
     * @brief Service to listen user set scan parameters
     * 
     */
    ros::ServiceServer srv_;
};