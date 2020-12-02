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

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/LaserScan.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/GpuRayPlugin.hh>

#include <sdf/sdf.hh>

#include <gazebo_plugins/PubQueue.h>

#include <thread>

#include "std_msgs/Float32.h"
#include "sensor_msgs/RegionOfInterest.h"

// #include "leica_scanstation_msgs/LeicaConfig.h"

#include <condition_variable>


namespace gazebo
{
  class GazeboRosLaser : public GpuRayPlugin
  {
  public:
    GazeboRosLaser();

    ~GazeboRosLaser();

    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
    
    void ScanWindowConfig(const sensor_msgs::RegionOfInterestConstPtr &_msg);

  private:
    void LaserConnect();

    void LaserDisconnect();

    void QueueThread();
    
    void LoadThread();

    /// \brief Keep track of number of connctions
    int laser_connect_count_;

    // Pointer to the model
    std::string world_name_;

    physics::WorldPtr world_;

    sensors::GpuRaySensorPtr parent_ray_sensor_;

    ros::NodeHandle *rosnode_;

    ros::Publisher pub_;

    PubQueue<sensor_msgs::LaserScan>::Ptr pub_queue_;

    std::string topic_name_;

    /// \brief frame transform name, should match link name
    std::string frame_name_;

    std::string tf_prefix_;

    std::string robot_namespace_;

    // deferred load in case ros is blocking
    sdf::ElementPtr sdf;

    boost::thread deferred_load_thread_;

    unsigned int seed;

    gazebo::transport::NodePtr gazebo_node_;

    gazebo::transport::SubscriberPtr laser_scan_sub_;

    void OnScan_gzSensorMsg(ConstLaserScanStampedPtr &_msg);

    /// \brief prevents blocking
    PubMultiQueue pmq;

    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Subscriber rosSub;

    ros::CallbackQueue rosQueue;

    std::thread rosQueueThread;

    bool start_laser;

    std::mutex m;

    std::condition_variable cv;
  };
} // namespace gazebo
#endif