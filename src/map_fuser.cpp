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
#include "era_gazebo/ERAOccupancyGrid.h"

void local_map_callback(const era_gazebo::ERAOccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("I received a local map");
}

void remote_map_callback(const era_gazebo::ERAOccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("I received a remote map");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_fuser");

  ros::NodeHandle n;
  ros::Subscriber loc_map_sub = n.subscribe<era_gazebo::ERAOccupancyGrid>("local_map", 1000, local_map_callback);
  ros::Subscriber rem_map_sub = n.subscribe<era_gazebo::ERAOccupancyGrid>("remote_map", 1000, remote_map_callback);
  ros::spin();

  return 0;
}
