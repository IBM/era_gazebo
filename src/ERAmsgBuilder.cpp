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
#include "era_gazebo/ERAMsg.h"


class ERAmsgBuilder
{
public:
	ERAmsgBuilder(): tf() 
	{
		grid_sub.subscribe(n, "local_map", 100);
		n.getParam("tf_prefix", tf_prefix);
		n.param<std::string>("ERAmsgBuilder_node/map_frame", map_frame, "/map");
		n.param<std::string>("ERAmsgBuilder_node/robot_base_frame", base_frame, "base_footprint");

		tf_filter = new tf::MessageFilter<nav_msgs::OccupancyGrid>(grid_sub, tf, tf_prefix+"/"+base_frame, 100);
		tf_filter->registerCallback( boost::bind(&ERAmsgBuilder::callback, this, _1) );
		pub = n.advertise<era_gazebo::ERAMsg>("transmit_msg", 100);

		n.getParam("ERAmsgBuilder_node/ID", out_msg.ID);	
	}
private:
	message_filters::Subscriber<nav_msgs::OccupancyGrid> grid_sub;
	tf::TransformListener tf;
	tf::MessageFilter<nav_msgs::OccupancyGrid> * tf_filter;
	ros::NodeHandle n;
	ros::Publisher pub;
	std::string map_frame, base_frame;
    std::string tf_prefix;

	era_gazebo::ERAMsg out_msg;

	void callback(const boost::shared_ptr<const nav_msgs::OccupancyGrid>& grid_ptr) 
	{
		tf::StampedTransform transform;
		
    	try{
      		 tf.lookupTransform(map_frame, tf_prefix+base_frame,  
                                ros::Time(grid_ptr->header.stamp), transform);
    		}
    	catch (tf::TransformException ex){
      		ROS_ERROR("%s",ex.what());
       	}

       	//ROS_INFO_STREAM(transform.getOrigin()[0]);
       	//ROS_INFO_STREAM(out_msg.pose.position.x);

       	tf::poseTFToMsg(transform, out_msg.pose);
       	
       	out_msg.grid = *grid_ptr;
   		
       	out_msg.header = grid_ptr->header;


		pub.publish(out_msg);
	};
public:
	void run()
	{

		ros::Rate r(100); //100Hz
		while(ros::ok()) {
			ros::spinOnce();

			pub.publish(out_msg);
			
		  	r.sleep();
  		}

	};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ERAmsgBuilder");
  
  ERAmsgBuilder t;
  //t.run();
  ros::spin();

}
