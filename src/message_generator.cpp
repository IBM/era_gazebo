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

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "era_gazebo/ERAOccupancyGrid.h"

class MessageGenerator
{
	private:
	ros::NodeHandle n;
	ros::Subscriber loc_map_sub;
	ros::Publisher loc_map_pub;

	public:
	MessageGenerator()
	{
		loc_map_sub = n.subscribe<nav_msgs::OccupancyGrid>("map", 1000, &MessageGenerator::LocalMapCallback, this);
		loc_map_pub = n.advertise<era_gazebo::ERAOccupancyGrid>("local_map", 1000);
	}

	void LocalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
	{
	  ROS_INFO("Converting local OccupancyGrid message into ERAOccupancyGrid");
	  era_gazebo::ERAOccupancyGrid new_msg;

	  new_msg.header = msg->header;
	  new_msg.info   = msg->info;
	  new_msg.data   = msg->data;

	  loc_map_pub.publish(msg);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "message_generator");
	MessageGenerator message_gen;
	ros::spin();

	return 0;
}
