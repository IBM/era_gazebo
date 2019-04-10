#include <ros/ros.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/OccupancyGrid.h"
#include "era_gazebo/ERAMsg.h"


class ERAmsgBuilder
{
public:
	ERAmsgBuilder(): tf(), target_frame("base_footprint") 
	{
		grid_sub.subscribe(n, "local_map", 100);
		n.getParam("tf_prefix", tf_prefix);
		tf_filter = new tf::MessageFilter<nav_msgs::OccupancyGrid>(grid_sub, tf, tf_prefix+"/"+target_frame, 100);
		tf_filter->registerCallback( boost::bind(&ERAmsgBuilder::callback, this, _1) );
		pub = n.advertise<era_gazebo::ERAMsg>("transmit_msg", 1000);

		n.getParam("ERAmsgBuilder_node/ID", out_msg.ID);
		ROS_INFO_STREAM("ROBOT ID: " << out_msg.ID);
	}
private:
	message_filters::Subscriber<nav_msgs::OccupancyGrid> grid_sub;
	tf::TransformListener tf;
	tf::MessageFilter<nav_msgs::OccupancyGrid> * tf_filter;
	ros::NodeHandle n;
	ros::Publisher pub;
	std::string target_frame;
      	std::string tf_prefix;
	era_gazebo::ERAMsg out_msg;

	void callback(const boost::shared_ptr<const nav_msgs::OccupancyGrid>& grid_ptr) 
	{
		tf::StampedTransform transform;
		
    	try{
      		 tf.lookupTransform("/map", tf_prefix+"/base_footprint",  
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

	};
public:
	void run()
	{

		ros::Rate r(1); //1Hz
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
  t.run();
//  ros::spin();

}
