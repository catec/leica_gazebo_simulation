/**
 * @file leica_service_listener.cpp
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

#include "sensor_msgs/RegionOfInterest.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include "leica_service_listener.h"

LeicaServiceListener::LeicaServiceListener(ros::NodeHandle nh)
    :nh_(nh)
{
    name_pub_ = nh_.advertise<std_msgs::String>("simulator/filename", 1, false);
    window_pub_ = nh_.advertise<sensor_msgs::RegionOfInterest>("simulator/window", 1, false);
    resolution_pub_ = nh_.advertise<std_msgs::Int64>("simulator/resolution", 1, false);

    srv_ = nh_.advertiseService("scan", &LeicaServiceListener::scanCb, this);
}

bool LeicaServiceListener::scanCb(leica_scanstation_msgs::Scan::Request& req, 
                                  leica_scanstation_msgs::Scan::Response& res)
{
    std_msgs::String name_msg;
    std_msgs::Int64 resolution_msg;
    sensor_msgs::RegionOfInterest window_msg;
    name_msg.data = req.file_name;
    resolution_msg.data = req.horizontal_res;
    window_msg.height = TO_MM(req.height);
    window_msg.width = TO_MM(req.width);
    window_msg.x_offset = req.pan_center;
    window_msg.y_offset = req.tilt_center;

    ROS_INFO("Scan requested");
    name_pub_.publish(name_msg);
    window_pub_.publish(window_msg);
    resolution_pub_.publish(resolution_msg);

    res.message = "Scanning";
    res.success = true;
    return true;
}

int main(int argc, char** argv)
{
    // Launch the process of transforming the laser scan into the point cloud
    ros::init(argc, argv, "ServiceListener");
    ros::NodeHandle nh;

    ROS_INFO("leica_service_listener");
    LeicaServiceListener listener(nh);

    ros::spin();

    return 0;
}
