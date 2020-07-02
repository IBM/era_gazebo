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
#include "combine_grids.h"

#include "era_gazebo/ERAMsg.h"
#include "nav_msgs/OccupancyGrid.h"

typedef boost::shared_ptr<nav_msgs::OccupancyGrid> GridPtr;

ros::Publisher pub;
double max_distance = 100.0;

double distance (geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
	return sqrt(pow(pose1.position.x-pose2.position.x,2) + 
							pow(pose1.position.y-pose2.position.y,2) + 
							pow(pose1.position.z-pose2.position.z,2));
}


void callback(const nav_msgs::OccupancyGrid::ConstPtr& local_msg, 
			  const era_gazebo::ERAMsg::ConstPtr& remote_msg)
{
  
	ROS_DEBUG("maps received");

	if(local_msg->info.width >0 && local_msg->info.height >0 &&
	   remote_msg->grid.info.width >0 && remote_msg->grid.info.height >0 ) {
	
		if (distance(local_msg->info.origin, remote_msg->grid.info.origin) > max_distance) {
			ROS_DEBUG("Not fusing map that is too far");
			return;
		} 

		std::vector<nav_msgs::OccupancyGrid> grids;
		grids.push_back(*local_msg);

		grids.push_back(remote_msg->grid);


		GridPtr combined = occupancy_grid_utils::combineGrids(grids);

		pub.publish(*combined);
	}
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_fuser");

  ros::NodeHandle n;
  message_filters::Subscriber<nav_msgs::OccupancyGrid> loc_map_sub(n, "local_map", 10);
  message_filters::Subscriber<era_gazebo::ERAMsg> rem_map_sub(n, "external_occ_grids", 10);

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::OccupancyGrid, era_gazebo::ERAMsg> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), loc_map_sub, rem_map_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  pub = n.advertise<nav_msgs::OccupancyGrid>("combined_map", 10);

  ros::param::param<double>("max_distance", max_distance, 100.0);

  ros::spin();

  return 0;
}
