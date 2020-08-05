#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include "occgrid.h"

//Global Variables
float resolution_;
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

void cloudCallback (const sensor_msgs::PointCloud2::ConstPtr& cloud) {

    nav_msgs::OccupancyGrid occgrid;

    //Convert cloud to an occupancy grid

    unsigned char * cloud_data = (unsigned char *)cloud->data.data();

    unsigned char * data =  cloudToOccgrid(cloud_data, odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odom.twist.twist.angular.z, rolling_window, min_obstacle_height, max_obstacle_height, raytrace_range, size_x, size_y, resolution, default_value);
    unsigned int data_size = 199992; //char array size is a max of 199992
    occgrid.data = std::vector<signed char>(data, data + data_size);

    //Initialize Occupancy Grid fields
    occgrid.header = cloud->header;

    occgrid.info.map_load_time = cloud->header.stamp;
    occgrid.info.resolution = resolution;
    occgrid.info.width = cloud->width;
    occgrid.info.height = cloud->height;
    occgrid.info.origin.position.x = odom.pose.pose.position.x;
    occgrid.info.origin.position.y = odom.pose.pose.position.y;
    occgrid.info.origin.position.z = odom.pose.pose.position.z;

    //Publish occupancy grid
    occ_grid_pub.publish(occgrid);
}

void odomCallback (const nav_msgs::Odometry::ConstPtr& odometry) {
   odom.pose.pose.position.x = odometry->pose.pose.position.x;
   odom.pose.pose.position.y = odometry->pose.pose.position.y;
   odom.pose.pose.position.z = odometry->pose.pose.position.z;
   odom.twist.twist.angular.z = odometry->twist.twist.angular.z;
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "occmap_node");

    ros::NodeHandle n;

    ros::Subscriber cloud_sub = n.subscribe("/carla/hero1/lidar/lidar1/point_cloud", 10, cloudCallback);

    ros::Subscriber odometry_sub = n.subscribe("/carla/hero1/odometry", 10, odomCallback);
    occ_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("local_map", 1000);

    ros::param::param<double>("resolution", resolution, 2.0); //Default value to be 2 meters per cell
    ros::param::param<bool>("rolling_window", rolling_window, false);
    ros::param::param<double>("min_obstacle_height", min_obstacle_height, 0.05);
    ros::param::param<double>("max_obstacle_height", max_obstacle_height, 2.05);
    ros::param::param<double>("raytrace_range", raytrace_range, 101.0);
    ros::param::param<double>("width", size_x, 100.0);
    ros::param::param<double>("height", size_y, 100.0);
    ros::param::param<int>("default_value", default_value, 254);

    ros::spin();
}

