#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
//#include "occgrid.h"

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	nav_msgs::OccupancyGrid occgrid;

	// Built the occ grid
//	laser_to_occgrid(msg->ranges, occgrid->data);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "occmap_node");

  ros::NodeHandle n;

	ros::Subscriber laser_sub = n.subscribe("scan", 10, laserCallback);
	ros::Publisher occ_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("local_map", 1000);


	ros::spin();

}
