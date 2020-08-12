#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <pcl/common/transforms.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cstdio>
#include <cmath>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>

#include "occgrid.h"

//Global Variables
std::string global_frame;
bool rolling_window;
double min_obstacle_height;
double max_obstacle_height;
double raytrace_range;
double width;
double height;
double size_x, size_y;
double resolution;
int default_value;

Odometry odom;

ros::Publisher occ_grid_pub;

tf2_ros::Buffer tfBuffer;

void cloudCallback (const sensor_msgs::PointCloud2::ConstPtr& cloud) {
	
    ROS_INFO("Received PointCloud");
    nav_msgs::OccupancyGrid occgrid;

    //Transform cloud's frame to global frame
    sensor_msgs::PointCloud2 global_cloud;
    geometry_msgs::Pose global_pose;

    geometry_msgs::TransformStamped transformStamped;
    try {
	    transformStamped = tfBuffer.lookupTransform(cloud->header.frame_id, "hero1", ros::Time(0), ros::Duration(2.0));
	    tf2::doTransform(*cloud, global_cloud, transformStamped);
    }

    catch (tf2::TransformException &ex) {
	    ROS_WARN("%s", ex.what());
	    ros::Duration(1.0).sleep();
	    return;
    }    

    //Convert cloud to an occupancy grid
    float* cloud_data = (float*) cloud->data.data();

    unsigned int cloud_size = cloud->width * cloud->height;
    unsigned char * data =  cloudToOccgrid(cloud_data, cloud_size, odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odom.twist.twist.angular.z, rolling_window, min_obstacle_height, max_obstacle_height, raytrace_range, size_x, size_y, resolution, default_value);
    unsigned int data_size = (unsigned int) (size_x * size_y) / (resolution * resolution) * sizeof(signed char); 
    occgrid.data = std::vector<signed char>(data, data + data_size);

    //Initialize Occupancy Grid fields
    occgrid.header = cloud->header;

    occgrid.info.map_load_time = cloud->header.stamp;
    occgrid.info.resolution = resolution;
    occgrid.info.width = (unsigned int) size_x / resolution;
    occgrid.info.height = (unsigned int) size_y / resolution;
    //ROS_INFO("Parameters: size_x, size_y, resolution = %f, %f, %f", size_x, size_y, resolution);
    //ROS_INFO("Parameters: Height, Width = %d, %d", (unsigned int) (size_x / resolution), (unsigned int) (size_y / resolution));

    //ROS_INFO("Occupancy Grid: <Height, Width, Resolution> = <%d, %d, %f>", occgrid.info.height, occgrid.info.width, occgrid.info.resolution);

    occgrid.info.origin.position.x = 0.0 - (size_x / resolution);
    occgrid.info.origin.position.y = 0.0 - (size_y / resolution); 
    occgrid.info.origin.position.z = 0.0;

    occgrid.info.origin.orientation.x = 0.0;
    occgrid.info.origin.orientation.y = 0.0;
    occgrid.info.origin.orientation.z = 0.0;
    occgrid.info.origin.orientation.w = 1.0;

    //ROS_INFO("Occupancy Grid Odometry: x, y, z = %f, %f, %f", odom.pose.pose.position.x - (size_x / resolution), odom.pose.pose.position.y - (size_y / resolution), occgrid.info.origin.position.z);

    //Publish occupancy grid
    occ_grid_pub.publish(occgrid);
}

void odomCallback (const nav_msgs::Odometry::ConstPtr& odometry) {
   //ROS_INFO("Odometry: x, y, z = %f, %f, %f", odometry->pose.pose.position.x, odometry->pose.pose.position.y, odometry->pose.pose.position.z);

   odom.pose.pose.position.x = odometry->pose.pose.position.x;
   odom.pose.pose.position.y = odometry->pose.pose.position.y;
   odom.pose.pose.position.z = odometry->pose.pose.position.z;
   odom.pose.pose.orientation.x = odometry->pose.pose.orientation.x;
   odom.pose.pose.orientation.y = odometry->pose.pose.orientation.y;
   odom.pose.pose.orientation.z = odometry->pose.pose.orientation.z;
   odom.pose.pose.orientation.w = odometry->pose.pose.orientation.w;
}

int main (int argc, char** argv) {
    ROS_INFO("Occupancy Grid Node: Initialize");
    ros::init(argc, argv, "occmap_node");

    ros::NodeHandle n;

    ros::Subscriber cloud_sub = n.subscribe("/carla/hero1/lidar/lidar1/point_cloud", 10, cloudCallback);

    ros::Subscriber odometry_sub = n.subscribe("/carla/hero1/odometry", 10, odomCallback);
    occ_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("local_map", 1000);

    ros::param::param<std::string>("global_frame", global_frame, "map");
    ros::param::param<double>("resolution", resolution, 2.0); //Default value to be 2 meters per cell
    ros::param::param<bool>("rolling_window", rolling_window, false);
    ros::param::param<double>("min_obstacle_height", min_obstacle_height, 0.05);
    ros::param::param<double>("max_obstacle_height", max_obstacle_height, 2.05);
    ros::param::param<double>("raytrace_range", raytrace_range, 101.0);
    ros::param::param<double>("width", size_x, 100.0);
    ros::param::param<double>("height", size_y, 100.0);
    ros::param::param<int>("default_value", default_value, 254);

    //Create listener object to receive incoming frames
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::spin();
}

