/*
 * Copyright 2012 Open Source Robotics Foundation
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
 *
*/

#ifndef GAZEBO_ROS_LASER_HH
#define GAZEBO_ROS_LASER_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <thread>

#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/Float32.h"
#include "sensor_msgs/RegionOfInterest.h"

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/GpuRayPlugin.hh>
#include <gazebo_plugins/PubQueue.h>

#include <condition_variable>


namespace gazebo
{
class LeicaPlugin : public GpuRayPlugin
{
public:
    /**
     * @brief Construct a new Gazebo ROS Laser object
     * 
     */
    LeicaPlugin();

    /**
     * @brief Destroy the Gazebo ROS Laser object
     * 
     */
    ~LeicaPlugin();

    /**
     * @brief Load the plugin controller
     * 
     * @param _parent Pointer to the laser sensor
     * @param _sdf Pointer to the *.sdf file with the Leica configuration
     */
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
    
    /**
     * @brief Configure the scan window from a topic
     * 
     * @param _msg Message with the scan window configuration format
     */
    void configureScanWindow(const sensor_msgs::RegionOfInterestConstPtr &_msg);

private:
    /**
     * @brief Register de new laser and connect to its topic
     * 
     */
    void connectToLaser();

    /**
     * @brief Unregister the laser
     * 
     */
    void disconnectFromLaser();

    /**
     * @brief Tailor the gazebo message to the ROS format
     * 
     * @param _msg Message to tailor
     */
    void gazeboMsgToROSMsg(ConstLaserScanStampedPtr &_msg);

    /**
     * @brief Thread that calls all functions in the callback queue when resources are available
     * 
     */
    void callbacksQueueHelperThread();

    /**
     * @brief Load the sensor controller
     * 
     */
    void gazeboLoaderThread();
    
    /**
     * @brief Seed for the Gaussian noise generator
     * 
     */
    unsigned int seed_;

    /**
     * @brief Flag to start the laser after the scan window has been set
     * 
     */
    bool start_laser_;

    /**
     * @brief A counter to register the number of laser sensors in use
     * 
     */
    int laser_connect_count_;

    /**
     * @brief Stores the name of the virtual world where the simulation takes place
     * 
     */
    std::string world_name_;
    
    /**
     * @brief Pointer to the virtual world where the simulation takes place
     * 
     */
    physics::WorldPtr world_;

    /**
     * @brief Saves the name of the topic where the laser scanner publishes what it reads
     * 
     */
    std::string topic_name_;

    /**
     * @brief Stores the name of the root of the reference system tree
     * 
     */
    std::string frame_name_;

    /**
     * @brief tf_prefix_ as the namespace
     * 
     */
    std::string tf_prefix_;

    /**
     * @brief The namespace of the Leica station
     * 
     */
    std::string namespace_;
    
    /**
     * @brief Mutex used so that no laser sensor data is published until the configuration of the scan window has been completed
     * 
     */
    std::mutex mutex_;

    /**
     * @brief Variable used to wait until the configuration of the scan window has been completed
     * 
     */
    std::condition_variable condition_variable_;
    
    /**
     * @brief Thread that calls all functions in the callback queue when resources are available
     * 
     */
    std::thread queue_helper_thread_;

    /**
     * @brief Pointer to the laser sensor object
     * 
     */
    sensors::GpuRaySensorPtr parent_ray_sensor_;

    /**
     * @brief Pointer to the ROS node
     * 
     */
    ros::NodeHandle *nh_;

    /**
     * @brief Publisher for the laser sensor
     * 
     */
    ros::Publisher pub_;

    /**
     * @brief Subscriber for the laser sensor
     * 
     */
    ros::Subscriber sub_;

    /**
     * @brief Queue for the callback functions
     * 
     */
    ros::CallbackQueue cb_queue_;

    /**
     * @brief Node for gazebo
     * 
     */
    gazebo::transport::NodePtr gazebo_node_;

    /**
     * @brief Gazebo part of the laser sensor subscriber
     * 
     */
    gazebo::transport::SubscriberPtr laser_scan_sub_;

    /**
     * @brief Publisher queue
     * 
     */
    PubQueue<sensor_msgs::LaserScan>::Ptr pub_queue_;

    /**
     * @brief Object for the ROS queue system
     * 
     */
    PubMultiQueue pmq_;

    /**
     * @brief Pointer to the *.sdf file
     * 
     */
    sdf::ElementPtr sdf_;

    /**
     * @brief Object for the sensor controller thread
     * 
     */
    boost::thread deferred_load_thread_;
};
}
#endif