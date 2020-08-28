#include <ros/ros.h>
#include <ros/console.h>
//#include <tf/transform_datatypes.h>
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
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
//#include <tf/transform_listener.h>

#include "occgrid.h"

//Global Variables
std::string namespace_;
std::string role_name;
std::string robot_base_frame;
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
Odometry prev_odom; //Odometry info of the previous time frame
float * prev_data = (float *) malloc(sizeof(float));
sensor_msgs::PointCloud2 prev_cloud, prev_global_cloud;
unsigned char * prev_grid_data;

ros::Publisher occ_grid_pub;

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped mapToHero1;
unsigned int count = 0;//TODO: Remove 'count' 

tf2::Transform initTransform(float qx, float qy, float qz, float qw, float x, float y, float z) {
	tf2::Quaternion q = tf2::Quaternion(qx, qy, qz, qw);
	tf2::Vector3 t = tf2::Vector3(x, y, z);

	ROS_INFO("Transform Initialization: (qx, qy, qz, qw, tx, ty, tz) = (%f, %f, %f, %f, %f, %f, %f)", qx, qy, qz, qw, x, y, z);

	return tf2::Transform(q, t);
}

geometry_msgs::TransformStamped transformToTransformStamped(tf2::Transform t) {
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.transform.translation.x = (float) t.getOrigin().x();
	transformStamped.transform.translation.x = (float) t.getOrigin().y();
	transformStamped.transform.translation.x = (float) t.getOrigin().z();

	transformStamped.transform.rotation.x = (float) t.getRotation().x();
	transformStamped.transform.rotation.y = (float) t.getRotation().y();
	transformStamped.transform.rotation.y = (float) t.getRotation().z();
	transformStamped.transform.rotation.y = (float) t.getRotation().w();

	ROS_INFO("Transform Initialization: (qx, qy, qz, qw, tx, ty, tz) = (%f, %f, %f, %f, %f, %f, %f)", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w, transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
	
	return transformStamped;
	
}	

void cloudCallback (const sensor_msgs::PointCloud2::ConstPtr& cloud) {
	
    ROS_INFO("Received PointCloud");
    nav_msgs::OccupancyGrid occgrid;

    //Transform cloud's frame to global frame
    sensor_msgs::PointCloud2 global_cloud;

    geometry_msgs::TransformStamped lidarToHero; //Transform from lidar to hero
    geometry_msgs::TransformStamped mapToHero2; //Transform from map to Hero2
    geometry_msgs::TransformStamped H_12; //Transform from H1 to H2
    try {
	    std::string source_frame = robot_base_frame + "/lidar/lidar1";
	    std::string target_frame = (robot_base_frame == "hero1") ? "109" : "110";
	    //ROS_INFO("target_frame, source_frame = %c, %c \n", target_frame, source_frame);
	    bool canTransformCloud = tfBuffer.canTransform(target_frame, source_frame, ros::Time(0));
	    //ROS_INFO("CAN TRANSFORM CLOUD? -> %d", canTransformCloud);
	    lidarToHero = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(2.0));
	    tf2::doTransform(*cloud, global_cloud, lidarToHero);

            mapToHero2 = tfBuffer.lookupTransform(target_frame, global_frame, ros::Time(0), ros::Duration(2.0));

	    if (count > 0) { //If not the first frame

		//Construct transformStamped objects to Transform objects for user-defined operators
		tf2::Transform mapToHero1_Transform = initTransform(mapToHero1.transform.rotation.x, mapToHero1.transform.rotation.y, mapToHero1.transform.rotation.z, mapToHero1.transform.rotation.w, mapToHero1.transform.translation.x, mapToHero1.transform.translation.y, mapToHero1.transform.translation.z);
		tf2::Transform mapToHero2_Transform = initTransform(mapToHero2.transform.rotation.x, mapToHero2.transform.rotation.y, mapToHero2.transform.rotation.z, mapToHero2.transform.rotation.w, mapToHero2.transform.translation.x, mapToHero2.transform.translation.y, mapToHero2.transform.translation.z);

		tf2::Transform H12_Transform = mapToHero1_Transform.inverse() * mapToHero2_Transform; //Transfrom from previous frame to current frame

		ROS_INFO("Transform: qx = %f, qy = %f, qz = %f, qw = %f, x = %f, y = %f, z = %f", H12_Transform.getRotation().x(), H12_Transform.getRotation().y(), H12_Transform.getRotation().z(), H12_Transform.getRotation().w(), H12_Transform.getOrigin().x(), H12_Transform.getOrigin().y(), H12_Transform.getOrigin().z());

		H_12 = transformToTransformStamped(H12_Transform);
		ROS_INFO("Transform Check: (qx, qy, qz, qw, x, y, z) = (%f, %f, %f, %f, %f, %f, %f)", H_12.transform.rotation.x, H_12.transform.rotation.y, H_12.transform.rotation.z, H_12.transform.rotation.w, H_12.transform.translation.x, H_12.transform.translation.y, H_12.transform.translation.z);
		tf2::doTransform(prev_cloud, prev_global_cloud, H_12); //Transform previous cloud data to current frame

		//Construct costmap from newly transformed data
		float* prev_cloud_data = (float*) prev_global_cloud.data.data();
    		unsigned int cloud_size = prev_cloud.width * prev_cloud.height; 
    		prev_grid_data =  cloudToOccgrid(prev_cloud_data, cloud_size, prev_odom.pose.pose.position.x, prev_odom.pose.pose.position.y, prev_odom.pose.pose.position.z, odom.twist.twist.angular.z, rolling_window, min_obstacle_height, max_obstacle_height, raytrace_range, size_x, size_y, resolution, default_value);		
	    }
	    
	    //Allocate transform fields to 'older' transform variable
	    mapToHero1.transform.translation.x = mapToHero2.transform.translation.x;
	    mapToHero1.transform.translation.y = mapToHero2.transform.translation.y;
	    mapToHero1.transform.translation.z = mapToHero2.transform.translation.z;
	    mapToHero1.transform.rotation.x = mapToHero2.transform.rotation.x;
	    mapToHero1.transform.rotation.y = mapToHero2.transform.rotation.y;
	    mapToHero1.transform.rotation.z = mapToHero2.transform.rotation.z;
	    mapToHero1.transform.rotation.w = mapToHero2.transform.rotation.w;
	    count++;
    }

    catch (tf2::TransformException &ex) {
	    ROS_WARN("%s", ex.what());
	    ros::Duration(1.0).sleep();
	    return;
    }    

    //Convert cloud to an occupancy grid
    float* cloud_data = (float*) global_cloud.data.data();

    unsigned int cloud_size = cloud->width * cloud->height;
    unsigned char * data =  cloudToOccgrid(cloud_data, cloud_size, odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odom.twist.twist.angular.z, rolling_window, min_obstacle_height, max_obstacle_height, raytrace_range, size_x, size_y, resolution, default_value);
    unsigned int data_size = (unsigned int) (size_x * size_y) / (resolution * resolution) * sizeof(unsigned char); 
    occgrid.data = std::vector<signed char>(data, data + data_size);

    //Initialize Occupancy Grid fields
    occgrid.header = cloud->header;

    occgrid.info.map_load_time = cloud->header.stamp;
    occgrid.info.resolution = resolution;
    occgrid.info.width = (unsigned int) size_x / resolution;
    occgrid.info.height = (unsigned int) size_y / resolution;

    occgrid.info.origin.position.x = 0.0 - (size_x / resolution);
    occgrid.info.origin.position.y = 0.0 - (size_y / resolution); 
    occgrid.info.origin.position.z = 0.0;

    occgrid.info.origin.orientation.x = 0.0;
    occgrid.info.origin.orientation.y = 0.0;
    occgrid.info.origin.orientation.z = 0.0;
    occgrid.info.origin.orientation.w = 1.0;

    if (count > 0) 
    occ_grid_pub.publish(occgrid); //TODO: Modify such that we publish a map even in the first frame

    //Save odomery info for later use
    prev_odom.pose.pose.position.x = odom.pose.pose.position.x;
    prev_odom.pose.pose.position.y = odom.pose.pose.position.y;
    prev_odom.pose.pose.position.z = odom.pose.pose.position.z;

    //Allocate data to global variable for later use
    prev_data = (float *) realloc(prev_data, sizeof(cloud_data) * sizeof(float));
    memcpy(prev_data, cloud_data, sizeof(float) * sizeof(cloud_data));
    prev_cloud.fields = cloud->fields;
    prev_cloud.height = cloud->height;
    prev_cloud.width = cloud->width;
    prev_cloud.is_bigendian = cloud->is_bigendian;
    prev_cloud.point_step = cloud->point_step;
    prev_cloud.row_step = cloud->row_step;
    prev_cloud.data = cloud->data;
    prev_cloud.is_dense = cloud->is_dense;
}

void odomCallback (const nav_msgs::Odometry::ConstPtr& odometry) {
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
    namespace_ = n.getNamespace().c_str();
    namespace_.erase(namespace_.begin());
    role_name = namespace_;
    namespace_ = n.getNamespace().c_str();
    ROS_INFO("rolename: %s", role_name.c_str());

    if (ros::param::has("/hero2/costmap_node_hero2/costmap/robot_base_frame")) {
	    ROS_INFO("PARAMETERS ARE DETECTED!");
	    ros::param::get(namespace_ + "/costmap_node_" + role_name + "/costmap/robot_base_frame", robot_base_frame);
	    ROS_INFO("robot_base_frame: %s", robot_base_frame.c_str());
    }
    else ROS_INFO("PARAMETERS ARE NOT DETECTED!");
    ros::param::param<std::string>(namespace_ + "/costmap_node_" + role_name + "/costmap/robot_base_frame", robot_base_frame, "hero1");
    ROS_INFO(">>>> %s ", robot_base_frame.c_str());
    ros::Subscriber cloud_sub = n.subscribe("/carla/" + robot_base_frame + "/lidar/lidar1/point_cloud", 10, cloudCallback);

    ros::Subscriber odometry_sub = n.subscribe("/carla/" + robot_base_frame + "/odometry", 10, odomCallback);
    occ_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("local_map", 1000);

    ros::param::param<std::string>(namespace_ + "/costmap_node_" + role_name + "/costmap/global_frame", global_frame, "map");
    ros::param::param<double>(namespace_ + "/costmap_node_" + role_name + "/costmap/resolution", resolution, 2.0); //Default value to be 2 meters per cell
    ros::param::param<bool>(namespace_ + "/costmap_node_" + role_name + "/costmap/rolling_window", rolling_window, false);
    ros::param::param<double>(namespace_ + "/costmap_node_" + role_name + "/costmap/obstacles/pointcloud/min_obstacle_height", min_obstacle_height, 0.05);
    ROS_INFO(">>>>> %f", min_obstacle_height);
    ros::param::param<double>(namespace_ + "/costmap_node_" + role_name + "/costmap/obstacles/max_obstacle_height", max_obstacle_height, 2.05);
    ros::param::param<double>(namespace_ + "/costmap_node_" + role_name + "/costmap/obstacles/pointcloud/raytrace_range", raytrace_range, 101.0);
    ros::param::param<double>(namespace_ + "/costmap_node_" + role_name + "/costmap/width", size_x, 100.0);
    ros::param::param<double>(namespace_ + "/costmap_node_" + role_name + "/costmap/height", size_y, 100.0);
    ros::param::param<int>(namespace_ + "/costmap_node_" + robot_base_frame + "/costmap/default_value", default_value, 254);

    //Create listener object to receive incoming frames
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::spin();
}

