/**
 * @file leica_plugin.cpp
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

/*
   Desc: based on GazeboRosGpuLaser plugin for simulating ray sensors in Gazebo
   Author: Mihai Emanuel Dolha
   Date: 29 March 2012
*/

#include <algorithm>
#include <string>
#include <assert.h>

#include <sdf/sdf.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/rendering/rendering.hh>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "leica_plugin.h"
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <std_msgs/String.h>

#include <iostream>
#include <math.h>
#include <chrono>

namespace gazebo
{
    // Register this plugin so that the simulator will be able to use it
    GZ_REGISTER_SENSOR_PLUGIN(LeicaPlugin)

    LeicaPlugin::LeicaPlugin()
    {
        // Seed for the Gaussian noise generator
        this->seed_ = 0;
    }

    LeicaPlugin::~LeicaPlugin()
    {
        // End the node
        ROS_DEBUG_STREAM("Shutting down GPU Laser");
        this->nh_->shutdown();
        delete this->nh_;
        ROS_DEBUG_STREAM("Unloaded");
    }

    void LeicaPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
        // Load the plugin
        GpuRayPlugin::Load(_parent, this->sdf_);

        // Extract some information 
        std::string worldName = _parent->WorldName();

        // Save some pointers
        this->world_ = physics::get_world(worldName);
        this->sdf_ = _sdf;

        // Initialize some variables
        start_laser_ = false;
        this->laser_connect_count_ = 0;

        // Get the pointer to the laser
        GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
        this->parent_ray_sensor_ = dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);

        // Makes some comprobations
        if (!this->parent_ray_sensor_)
            gzthrow("LeicaPlugin controller requires a Ray Sensor as its parent");

        this->namespace_ = GetRobotNamespace(_parent, _sdf, "Laser");

        if (!this->sdf_->HasElement("frameName"))
        {
            ROS_INFO("LeicaPlugin missing <frameName>, defaults to /world");
            this->frame_name_ = "/world";
        }
        else
            this->frame_name_ = this->sdf_->Get<std::string>("frameName");

        if (!this->sdf_->HasElement("topicName"))
        {
            ROS_INFO("LeicaPlugin missing <topicName>, defaults to /world");
            this->topic_name_ = "/world";
        }
        else
            this->topic_name_ = this->sdf_->Get<std::string>("topicName");

        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                                    << "Load the Gazebo system plugin 'leica_plugin.so')");
            return;
        }

        // Calls gazeboLoaderThread func
        ROS_INFO("Starting LeicaPlugin (ns = %s)", this->namespace_.c_str());
        this->deferred_load_thread_ = boost::thread(boost::bind(&LeicaPlugin::gazeboLoaderThread, this));

        // Subscribe to the topic that configures the scan window
        ros::SubscribeOptions so =
            ros::SubscribeOptions::create<sensor_msgs::RegionOfInterest>("/c5/simulator/window",
            1,
            boost::bind(&LeicaPlugin::configureScanWindow, this, _1),
            ros::VoidPtr(),
            &this->cb_queue_);
        this->sub_ = this->nh_->subscribe(so);

        // Spin up the queue helper thread.
        this->queue_helper_thread_ = std::thread(boost::bind(&LeicaPlugin::callbacksQueueHelperThread, this));
    }

    void LeicaPlugin::gazeboLoaderThread()
    {
        // Initialise the node
        this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
        this->gazebo_node_->Init(this->world_name_);
        this->pmq_.startServiceThread();
        this->nh_ = new ros::NodeHandle(this->namespace_);

        // Name the tf
        this->tf_prefix_ = tf::getPrefixParam(*this->nh_);
        if (this->tf_prefix_.empty())
        {
            this->tf_prefix_ = this->namespace_;
            boost::trim_right_if(this->tf_prefix_, boost::is_any_of("/"));
        }
        ROS_INFO("Leica Plugin ns:%s <tf_prefix_>, set to \"%s\"", this->namespace_.c_str(), this->tf_prefix_.c_str());

        // resolve tf prefix
        this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

        // Prepare the publishers needed by the manager to know when a new laser instance has been created
        if (this->topic_name_ != "")
        {
            ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(this->topic_name_,
            1,
            boost::bind(&LeicaPlugin::connectToLaser, this),
            boost::bind(&LeicaPlugin::disconnectFromLaser, this),
            ros::VoidPtr(),
            NULL);

            this->pub_ = this->nh_->advertise(ao);
            this->pub_queue_ = this->pmq_.addPub<sensor_msgs::LaserScan>();
        }

        // sensor generation off by default
        this->parent_ray_sensor_->SetActive(false);

        ROS_INFO_STREAM("gazeboLoaderThread function completed");
    }

    void LeicaPlugin::connectToLaser()
    {
        // Wait until the configuration of the scan window has been completed
        std::unique_lock<std::mutex> lk(mutex_);
        condition_variable_.wait(lk, [&] { return start_laser_; });

        this->laser_connect_count_++;

        ROS_INFO("LaserConnect: laser connect count: %d", laser_connect_count_);
        if (this->laser_connect_count_ == 1)
            this->laser_scan_sub_ = this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(), 
                                                                  &LeicaPlugin::gazeboMsgToROSMsg, this);
        lk.unlock();
    }

    void LeicaPlugin::disconnectFromLaser()
    {
        this->laser_connect_count_--;
        if (this->laser_connect_count_ == 0)
            this->laser_scan_sub_.reset();
    }

    void LeicaPlugin::gazeboMsgToROSMsg(ConstLaserScanStampedPtr &_msg)
    {
        sensor_msgs::LaserScan laser_msg;
        laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
        laser_msg.header.frame_id = this->frame_name_;
        laser_msg.angle_min = _msg->scan().angle_min();
        laser_msg.angle_max = _msg->scan().angle_max();
        laser_msg.angle_increment = _msg->scan().angle_step();
        laser_msg.time_increment = 0;
        laser_msg.scan_time = 0;
        laser_msg.range_min = _msg->scan().range_min();
        laser_msg.range_max = _msg->scan().range_max();
        laser_msg.ranges.resize(_msg->scan().ranges_size());
        std::copy(_msg->scan().ranges().begin(), _msg->scan().ranges().end(), laser_msg.ranges.begin());
        laser_msg.intensities.resize(_msg->scan().intensities_size());
        std::copy(_msg->scan().intensities().begin(), _msg->scan().intensities().end(), laser_msg.intensities.begin());
        this->pub_queue_->push(laser_msg, this->pub_);
    }

    void LeicaPlugin::configureScanWindow(const sensor_msgs::RegionOfInterestConstPtr &_msg)
    {
        // Enable gazebo to ROS message converter
        std::lock_guard<std::mutex> lk(mutex_);

        // Get the pointer to the camera that the laser needs to function properly
        rendering::GpuLaserPtr cam = parent_ray_sensor_->LaserCamera();

        double MIN_ANGLE_LIMIT = -1.04;
        // Get angle values. [mm to m] for width and height 
        ignition::math::Angle half_angle = (parent_ray_sensor_->AngleMax() + parent_ray_sensor_->AngleMin()) / 2.0;
        ignition::math::Angle angle_min = atan(_msg->y_offset - (_msg->height)*1e-3 / 2.0);
        ignition::math::Angle angle_max = atan(_msg->y_offset + (_msg->height)*1e-3 / 2.0);
        if(angle_min.Radian() < MIN_ANGLE_LIMIT)
            angle_min = MIN_ANGLE_LIMIT;
        if((angle_max - angle_min) < ignition::math::Angle::HalfPi)
            angle_max = angle_min + ignition::math::Angle::HalfPi;
        // Apply angles to ray
        parent_ray_sensor_->SetAngleMax(angle_max.Radian());
        parent_ray_sensor_->SetAngleMin(angle_min.Radian());

        // Apply angles to camera
        ignition::math::Angle update_half_angle = (parent_ray_sensor_->AngleMax()+parent_ray_sensor_->AngleMin())/2.0;
        ignition::math::Angle cam_angle = half_angle - update_half_angle;
        cam->Roll(ignition::math::Angle(cam_angle), rendering::ReferenceFrame::RF_WORLD);
        cam->SetHFOV(angle_max - angle_min);

        // Enable gazebo to ROS message converter
        start_laser_ = true;
        condition_variable_.notify_one();
    }

    void LeicaPlugin::callbacksQueueHelperThread()
    {
        // invoque all callbacks available
        static const double timeout = 0.01;
        while (this->nh_->ok())
        {
            this->cb_queue_.callAvailable(ros::WallDuration(timeout));
        }
    }
} // namespace gazebo