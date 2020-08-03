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

    //unsigned char * data =  cloudToOccgrid(cloud_data, odom, rolling_window, min_obstacle_height, max_obstacle_height, raytrace_range, size_x, size_y, resolution, default_value);
    // AKIN do we know the size of the data? The following line is how we convert. We need the size
    // occgrid.data = std::vector<unsigned char>(data, data_size);

    //Initialize Occupancy Grid fields
    occgrid.header = cloud->header;

    occgrid.info.map_load_time = cloud->header.stamp; //TODO: map_load_time is of 'time' type
    occgrid.info.resolution = resolution;
    occgrid.info.width = cloud->width;
    occgrid.info.height = cloud->height;
   // AKIN  occgrid.info.origin = odom->pose; these are not of the same type

    //Publish occupancy grid
    occ_grid_pub.publish(occgrid);
}

void odomCallback (const nav_msgs::Odometry::ConstPtr& odometry) {
   //AKIN  odom = odometry; this won't work since they are not of the same time. You need to assign the contents by hand
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
