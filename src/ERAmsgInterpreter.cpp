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

#include <ros/ros.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/OccupancyGrid.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SpawnModel.h"
#include "era_gazebo/ERAMsg.h"

using namespace std;

class ERAmsgInterpreter
{
public:
	ERAmsgInterpreter()
	{
		era_msg_sub = n.subscribe("receive_msg", 100, &ERAmsgInterpreter::callback, this);
		n.param<bool>("ERAmsgBuilder_node/use_gazebo", use_gazebo, false);

		if(use_gazebo) {
			gazebo_model_sub = n.subscribe("/gazebo/model_states", 100, &ERAmsgInterpreter::gazebo_model_callback, this);
			gazebo_model_pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
			spawn_srv = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
		}

		era_msg_pub = n.advertise<era_gazebo::ERAMsg>("external_occ_grids", 100);

		counter = 0;
	}

private:
	ros::NodeHandle n;
	ros::Subscriber era_msg_sub;
	ros::Subscriber gazebo_model_sub;
	ros::Publisher gazebo_model_pub;
	ros::Publisher era_msg_pub;
	ros::ServiceClient spawn_srv;
	vector<string> current_models;
	bool use_gazebo;
	unsigned int counter;

	void callback(const boost::shared_ptr<const era_gazebo::ERAMsg>& msg)
	{
		//ROS_INFO_STREAM("Received a new message. Robot Name: " << msg->ID);

		//Check if the robot exist
		if(use_gazebo) {
			vector<string>::iterator it = find (current_models.begin(), current_models.end(), msg->ID);
			if (it != current_models.end()) {
			    // Found. We move it
				gazebo_msgs::ModelState state_msg;

				state_msg.model_name = msg->ID;
				state_msg.pose = msg->pose;

				state_msg.twist.linear.x = 0;
				state_msg.twist.linear.y = 0;
				state_msg.twist.linear.z = 0;
				state_msg.twist.angular.x = 0;
				state_msg.twist.angular.y = 0;
				state_msg.twist.angular.z = 0;

				state_msg.reference_frame = "world";

				gazebo_model_pub.publish(state_msg);
			}
			else {
				// Not Found. We spawn it
				gazebo_msgs::SpawnModel spawn_msg;

				spawn_msg.request.model_name = msg->ID;
				ROS_ERROR_STREAM(spawn_msg.request.model_name);
				n.getParam("simple_box_description", spawn_msg.request.model_xml);
				//ROS_ERROR_STREAM(spawn_msg.request.model_xml);

				spawn_msg.request.initial_pose = msg->pose;
				spawn_msg.request.reference_frame = "";

				if(spawn_srv.call(spawn_msg)) {
					ROS_INFO_STREAM("Robot " << msg->ID << " spawned in Gazebo");
				}
				else {
					ROS_INFO_STREAM("Cannot spawn " << msg->ID);
				}

			}
		}
		era_gazebo::ERAMsg stamped_msg = *msg;
		stamped_msg.header.stamp = ros::Time::now(); //this is debatable. We restamp the incoming package with current time. ROS 2.0 might have a better solution
		era_msg_pub.publish(stamped_msg);

	};


	void gazebo_model_callback(const boost::shared_ptr<const gazebo_msgs::ModelStates>& msg)
	{
		// This callback will be 100Hz
		// Let's slow it down to 1Hz
		if(current_models.empty() || counter == 100) {
			current_models = msg->name;
			counter = 0;
		}	
		counter++;
	};

	public:
	void run()
	{
		ros::spin();

	};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ERAmsgInterpreter");
  
  ERAmsgInterpreter I;
  I.run();


}
