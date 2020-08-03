#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include "occgrid.h"

//Global Variables
float32 resolution_;

class LaserScanToPointCloud {

public:

    ros::NodeHandle n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscribe<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    ros::Publisher occ_pub_;
    nav_msgs::OccupancyGrd occgrid;

    LaserScanTointCloud(ros::NodeHandle n) :
        n_(n),
        laser_sub_(n_, "scan", 10),
        laser_notifier_(laser_sub_, listener_, /*target_frame*/ , 10) { //TODO: target_frame = scan_in->header.frame_id

        laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_ntifier_.setTolerance(ros::Duration(0.01));
        occ_pub_ = n.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);
    }

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in) {
        sensor_msgs::PointCloud cloud;
        try {
            projector_.transformLaserScanToPointCloud(scan_in->header.frame_id, *scan_in, cloud, listener_);
        }
        catch {
            std::cout <<e.what();
            return;
        }

        //QUESTION: prob declare occgrid outside of function for public access
        //Convert cloud to an occupancy grid
        occgrid->data = cloudToOccgrid(cloud->data, odom);

        //Initialize Occupancy Grid fields
        occgrid->header = msg->header; //TODO: Verify

        ros::Time now();
        occgrid->info->map_load_time = ros::Tim::now(); //TODO: Incompatible data-types
        occgrid->info->resolution = resolution_; //TODO: Verify
        occgrid->info->width = cloud->width;
        occgrid->info->height = cloud->height;
        occgrid->info->origin = odom->pose->pose;

        occ_pub_.publish(occgrid);
    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "occmap_node");

    ros::NodeHandle n;

    LaserScanToPoinCloud lstopc(n);

    n.param("resolution", resolution_, 2.0); //Default value to be 2 meters per cell //TODO: Verify

    ros::spin();
}
