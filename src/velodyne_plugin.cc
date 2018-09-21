/*
 * Copyright 2018 IBM
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

#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelodynePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
          ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
          return;
        }

    	// Safety check
    	if (_model->GetJointCount() == 0)
    	{
    		std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
    		return;
    	}

    	// Store the model pointer for convenience.
    	this->model = _model;

    	// Get the first joint. We are making an assumption about the model
    	// having one joint that is the rotational joint.
    	this->joint = _model->GetJoints()[0];

    	// Setup a P-controller, with a gain of 0.1.
    	this->pid = common::PID(0.1, 0, 0);

    	// Apply the P-controller to the joint.
    	this->model->GetJointController()->SetVelocityPID(
    			this->joint->GetScopedName(), this->pid);

    	// Default to zero velocity
    	double velocity = 0;

    	// Check that the velocity element exists, then read the value
    	if (_sdf->HasElement("velocity"))
    	  velocity = _sdf->Get<double>("velocity");

    	this->SetVelocity(velocity);

    	// Create the node
    	this->node = transport::NodePtr(new transport::Node());
    	#if GAZEBO_MAJOR_VERSION < 8
    	this->node->Init(this->model->GetWorld()->GetName());
    	#else
    	this->node->Init(this->model->GetWorld()->Name());
    	#endif

    	// Create a topic name
    	std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

    	// Subscribe to the topic, and register a callback
    	this->sub = this->node->Subscribe(topicName,
    	   &VelodynePlugin::OnMsg, this);
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetVelocity(_msg->x());
    }

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), _vel);
    }

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
