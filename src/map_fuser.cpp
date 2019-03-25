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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "era_gazebo/ERAMsg.h"
#include "nav_msgs/OccupancyGrid.h"

void callback(const nav_msgs::OccupancyGrid::ConstPtr& local_msg, 
			  const era_gazebo::ERAMsg::ConstPtr& remote_msg)
{
  
	ROS_INFO("maps received");


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_fuser");

  ros::NodeHandle n;
  message_filters::Subscriber<nav_msgs::OccupancyGrid> loc_map_sub(n, "/local_map", 1000);
  message_filters::Subscriber<era_gazebo::ERAMsg> rem_map_sub(n, "/external_occ_grids", 1000);

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::OccupancyGrid, era_gazebo::ERAMsg> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), loc_map_sub, rem_map_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  

  ros::spin();

  return 0;
}
