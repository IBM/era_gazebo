#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include "occgrid.h"

//Global Variables
float resolution_;
bool rolling_window;
double min_obstacle_height;
double max_obstacle_height;
double raytrae_range;
double width;
double height;
unsigned char default_value;

nav_msgs::Odometry* odom;

void cloudCallback (const sensor_msgs::PointCloud::ConstPtr& cloud) {

    nav_msgs::OccupancyGrid occgrid;

    //Convert cloud to an occupancy grid
    occgrid->data = cloudToOccgrid(cloud->data, odom, rolling_window, min_obstacle_height, max_obstacle_height, raytrace_range, size_x, size_y, resolution, default_value);

    //Initialize Occupancy Grid fields
    occgrid->header = msg->header;

    ros::Time now();
    occgrid->info->map_load_time = cloud->header->stamp; //TODO: map_load_time is of 'time' type
    occgrid->info->resolution = resolution_;
    occgrid->info->width = cloud->width;
    occgrid->info->height = cloud->height;
    occgrid->info->origin = odom->pose->pose;

    //Publish occupancy grid
    occ_grid_pub_.publish(occgrid);
}

void odomCallback (const nav_msgs::Odometry::ConstPtr& odometry) {
    odom = odometry;
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "occmap_node");

    ros::NodeHandle n;

    ros::Subscriber cloud_sub = n.subscribe("/carla/hero1/lidar/lidar1/point_cloud", 10, cloudCallback);
    ros::Subscriber odometry_sub = b.subscribe("carla/hero1/odometry", 10, odomCallback);
    ros::Publisher occ_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("local_map", 1000);

    n.param("resolution", resolution_, 2.0); //Default value to be 2 meters per cell
    n.param("rolling_window", rolling_window, false);
    n.param("min_obstacle_height", min_obstacle_height, 0.05);
    n.param("max_obstacle_height", max_obstacle_height, 2.05);
    n.param("raytrace_range", raytrace_range, 101.0);
    n.param("width", size_x, 100.0);
    n.param("height", size_y, 100.0);
    n.param("default_value", default_value, 254);

    ros::spin();
}
