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

#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/CameraInfo.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include "era_gazebo/DetectionBoxList.h"
#include "era_gazebo/DetectionBox.h"

using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Image, sensor_msgs::CameraInfo, era_gazebo::DetectionBoxList> mySyncPolicy;

tf::TransformListener * tfListener;

image_transport::Publisher result_pub;
ros::Publisher grid_pub;
ros::Publisher cloud_pub;

void callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg, const era_gazebo::DetectionBoxListConstPtr& detection_msg)
{

	if(detection_msg->detection_list.size()==0)
		return;

	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
	cv::Mat depth_mat;
	cv_ptr->image.convertTo(depth_mat, CV_32F, 0.001);


	nav_msgs::OccupancyGrid grid_msg;

	grid_msg.header.stamp = detection_msg->header.stamp; 

	// these values should set by a launch/config file
	grid_msg.header.frame_id = "camera_link";
	grid_msg.info.resolution = 0.01;
	grid_msg.info.width = 3.0 / grid_msg.info.resolution;
	grid_msg.info.height = 5.0 / grid_msg.info.resolution;
	grid_msg.info.origin.position.x = grid_msg.info.width * grid_msg.info.resolution / 2.0 * -1;
	grid_msg.info.origin.position.y = grid_msg.info.height * grid_msg.info.resolution / 2.0 * -1;
	grid_msg.info.origin.position.z = 0;
	grid_msg.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
	grid_msg.data = std::vector<int8_t>(grid_msg.info.width*grid_msg.info.height, 0);

	
	image_geometry::PinholeCameraModel cam_model;
 	cam_model.fromCameraInfo(info_msg);

 	for(int b = 0; b<detection_msg->detection_list.size(); b++) {

 		era_gazebo::DetectionBox box = (detection_msg->detection_list[b]);

 		PointCloud::Ptr msg (new PointCloud);
 		msg->header.frame_id = image_msg->header.frame_id;
 		msg->height = 1;

 		for(int u=box.left; u<box.right; u++) {
 			for(int v=box.top; v<box.bottom; v++) {
 				cv::Point2d uv(u, v);
 				cv::Point3d xyz = cam_model.projectPixelTo3dRay(uv);

 				double depth_value = depth_mat.at<float>(Point(u, v));

 				xyz = xyz * depth_value;

			//ROS_INFO_STREAM("Values: " << xyz << " " << depth_value);
			//Project it to the occupancy grid
			//ROS_INFO_STREAM("x*res: " << xyz.x/grid_msg.info.resolution << " y*res: " <<  xyz.y/grid_msg.info.resolution);

 				msg->points.push_back (pcl::PointXYZ(xyz.x, xyz.y, xyz.z ));

 			}
 		}

 		msg->width = msg->points.size();

 		pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);

 		/*
 		PointCloud::Ptr cloud_filtered (new PointCloud);
 		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    	// build the filter
    	outrem.setInputCloud(msg);
    	outrem.setRadiusSearch(0.1);
    	outrem.setMinNeighborsInRadius (10);
    	// apply filter
    	outrem.filter (*cloud_filtered);
		*/
		PointCloud::Ptr cloud_filtered (new PointCloud);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  		sor.setInputCloud (msg);
  		sor.setMeanK (50);
  		sor.setStddevMulThresh (1.0);
  		sor.filter (*cloud_filtered);

 		PointCloud::Ptr tf_msg (new PointCloud);
 		pcl_ros::transformPointCloud(grid_msg.header.frame_id, *cloud_filtered, *tf_msg, *tfListener);

 		// We have a pointcloud transformed and ready for projection

 		double cellResolution = grid_msg.info.resolution;


 		for(int i =0; i<tf_msg->points.size(); i++) {
 			double x = tf_msg->points[i].x;
 			double y = tf_msg->points[i].y;

 			int xCell = (int) ((x-grid_msg.info.origin.position.x) / cellResolution);
 			int yCell = (int) ((y-grid_msg.info.origin.position.y) / cellResolution);

 			grid_msg.data[yCell*grid_msg.info.width+xCell] = box.id;

 		}

 		cloud_pub.publish (tf_msg);

 		//cv::rectangle(cv_ptr->image, Point(box.left, box.top), Point(box.right, box.bottom), cv::Scalar(0, 255, 0), 6);
		//cv::putText(cv_ptr->image, std::to_string(box.id), Point(box.left, box.top), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0,255,0),5);
	
 	}

	grid_pub.publish(grid_msg);


	
	//sensor_msgs::ImagePtr result_image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, cv_ptr->image).toImageMsg();
	//result_image_msg->header = image_msg->header;
	//result_pub.publish(result_image_msg);


}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "og_labeler");
	ros::NodeHandle n("~");
  	//image_transport::ImageTransport it(n);

	tfListener = new tf::TransformListener();

	image_transport::ImageTransport it(n);
	image_transport::SubscriberFilter image_sub( it, "image_input", 100);
	message_filters::Subscriber<era_gazebo::DetectionBoxList> detection_sub(n, "object_input", 100);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n,"camera_info",100);

	tf::MessageFilter<era_gazebo::DetectionBoxList> tf_filter(detection_sub, *tfListener, "camera_link", 100);	


	message_filters::Synchronizer< mySyncPolicy > sync(mySyncPolicy(100), image_sub, info_sub, tf_filter);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3));

	grid_pub = n.advertise<nav_msgs::OccupancyGrid>("out_grid",1);
	cloud_pub = n.advertise<PointCloud> ("points2", 1);

  	ros::AsyncSpinner spinner(4); // Use 4 threads
  	spinner.start();

  //ros::Subscriber obj_sub = n.subscribe("object_input", 1, objectCallback);
  
  //image_transport::Subscriber image_sub = it.subscribe("image_input", 100, imageCallback);
  
  //quality_pub = n.advertise<std_msgs::Float64>("tracking_quality", 1);

  //object_pub = n.advertise<detection::DetectionBoxList>("tracked_objects",1);

  result_pub = it.advertise("occupancy_image", 1);
  
  ros::waitForShutdown();
  
}