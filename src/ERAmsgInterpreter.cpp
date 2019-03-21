#include <ros/ros.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/OccupancyGrid.h"
#include "gazebo_msgs/ModelState.h"
#include "era_gazebo/ERAMsg.h"

class ERAmsgInterpreter
{
public:
	ERAmsgInterpreter()
	{
		era_msg_sub = n.subscribe("receive_msg", 100, &ERAmsgInterpreter::callback, this);
		gazebo_model_pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);

	}

private:
	ros::NodeHandle n;
	ros::Subscriber era_msg_sub;
	ros::Publisher gazebo_model_pub;




	void callback(const boost::shared_ptr<const era_gazebo::ERAMsg>& msg)
	{
		ROS_INFO_STREAM("Received a new message. Robot Name: " << msg->ID);

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