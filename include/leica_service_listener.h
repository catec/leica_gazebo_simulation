/**
 * @file leica_service_listener.h
 * @copyright Copyright (c) 2020, FADA-CATEC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#ifndef _LEICA_SERVICE_LISTENER_H
#define _LEICA_SERVICE_LISTENER_H

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

#endif