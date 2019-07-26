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

// add the dynamic config dependencies
#include<dynamic_reconfigure/server.h>
#include<era_gazebo/testConfig.h> //add the config file as header file (add Config in the filename as syntax, ex: file name = test, here: testConfig)

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
/* #include<iostream>
#include<fstream> */

using namespace cv;
using namespace std;

//save image


#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>

//guided filter
#include <opencv2/ximgproc/edge_filter.hpp>
//virtual void cv::ximgproc::GuidedFilter::filter (const cv::Mat&, const cv::Mat& , int);
//cv::ximgproc::GuidedFilter *fil; //fil is the object
//cv::ximgproc::jointBilateralFilter *fil1
//(const cv::Mat&, const cv::Mat&, const cv::Mat&, int , double , double , int);

//bilateral filter
//void cv::ximgproc::jointBilateralFilter 	( 	InputArray  	joint,
		/* InputArray  	src,
		OutputArray  	dst,
		int  	d,
		double  	sigmaColor,
		double  	sigmaSpace,
		int  	borderType = BORDER_DEFAULT 
	) 	 */

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Image, era_gazebo::DetectionBoxList> mySyncPolicy; //changed

tf::TransformListener * tfListener;

image_transport::Publisher result_pub;
ros::Publisher grid_pub;
ros::Publisher cloud_pub;
sensor_msgs::CameraInfo cam_info;
image_geometry::PinholeCameraModel cam_model;
int Camera_flag = 0;
double mask_threshold = 0.5;
int kernel = 5;
bool filter_type = 0;

//new declarations
//typedef UINT8_C uint8_t;
//typedef CV_16U uint16_t;
//typedef UINT32_C uint32_t;
cv::Mat_<ushort> pad_input_16u(cv::Mat_<ushort> , int , int , int );
ushort median_16u(cv::Mat_<ushort> , int , int , int );
cv::Mat_<ushort> get_kernel_16u(cv::Mat_<ushort> , int , int , int );
cv::Mat_<ushort> sort_16u(cv::Mat_<ushort> , int );
ushort find_median_16u(cv::Mat_<ushort>  , int );


//new definitions
cv::Mat_<ushort> pad_input_16u(cv::Mat_<ushort> total_input, int imh, int imw, int k) 
{
	//cv::FileStorage file3("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/inside_reshaped_depth.txt", cv::FileStorage::WRITE);
	//file3 << "reshaped_inside" << total_input;
	//add padding on both sides
	int onesided = int(k/2);
	cv::Mat_<ushort> reshaped(imh+k-1, imw+k-1);
	
	for (int i=0; i<(imh+2*onesided);i++)
	{
		for (int j=0; j< (imw+2*onesided); j++)
		{
			bool left1, right1, top1, bottom1 = 0;
			bool cx1 = i<onesided;
			bool cx2 = i>(imh+onesided);
			bool cx3 = j<onesided;
			bool cx4 = j>(imw+onesided);
			if (cx1)
				left1 = 1;
			else
				left1 = 0;
			if (cx2)
				right1 = 1;
			else
				right1 = 0;
			if (cx3)
				top1 = 1;
			else
				top1 = 0;
			if (cx4)
				bottom1 = 1;
			else
				bottom1 = 0;
			bool res = ((left1 || right1) || (top1 || bottom1));
			if (res) //if (((i<onesided) \ (i>(imh+onesided))) \ ((j<onesided) \ (j>(imw+onesided))))
				reshaped.at<ushort>(i,j) = 0;
			else
				reshaped.at<ushort>(i,j) = total_input.at<ushort>(i,j);
		}	
	}
	return reshaped;
}

ushort median_16u(cv::Mat_<ushort> reshaped, int cx, int cy, int k)
{
	cv::Mat_<ushort> temp(1, k*k);
	ushort res1;
	temp = get_kernel_16u(reshaped, cx, cy, k); //get kernel elements
	temp = sort_16u(temp, k*k); //elements = kernel*kernel: sort the kernel bubble sort
	res1 = find_median_16u(temp, k*k); //elements = kernel*kernel: get the median of the sorted elemets
	return res1;
}

cv::Mat_<ushort> get_kernel_16u(cv::Mat_<ushort> reshaped, int cx, int cy, int k)
{
	//get only kernel 
	cv::Mat_<ushort> temp(1, k*k);
	int onesided = int(k/2);
	for (int i=0; i<k; i++)
	{
		for (int j=0; j< k; j++)
		{
			temp.at<ushort>(0, (i*k)+j)= reshaped.at<ushort>(cx+i,cy+j);
		}
	}
	return temp;
}

cv::Mat_<ushort> sort_16u(cv::Mat_<ushort> temp, int elements)
{
	//sort array
	ushort temp1=0;
    for(int i=0;i< elements;i++)
    {
        for(int j=i+1;j< elements;j++)
        {
            if(temp.at<ushort>(i) > temp.at<ushort>(j))
            {
                temp1   = temp.at<ushort>(i);
                temp.at<ushort>(i) = temp.at<ushort>(j);
                temp.at<ushort>(j) = temp1;
            }
        }
    }
	return temp;
}

ushort find_median_16u(cv::Mat_<ushort> temp , int n)
{
    ushort median=0;
    // if number of elements are even
    if(n%2 == 0)
	{
        median = (temp.at<ushort>((n-1)/2)+ temp.at<ushort>(n/2))/2.0;
	}
    // if number of elements are odd
    else
	{
        median = temp.at<ushort>(n/2);
	}
    return median;
}



//old code
//add callback for dynamic loading of mask threshold
void maskthresholdcallback(era_gazebo::testConfig &config, uint32_t level)
{
	ROS_INFO("new_value of mask threshold: [%f]", config.mask_threshold);
	mask_threshold =  config.mask_threshold;
}

void filterkernelcallback(era_gazebo::testConfig &config, uint32_t level)
{
	ROS_INFO("new_value of median filter type (0: standard, 1: Custom_CV_16u): [%d]", config.Median_Filter_type);
	ROS_INFO("new_value of median filter kernel: [%d]", config.Median_Filter_kernel);
	filter_type =  config.Median_Filter_type;
	kernel =  config.Median_Filter_kernel;
	if (kernel % 2 == 0)
	{
		kernel = kernel +1;
		ROS_INFO("Since value is even kernel changed to next odd number ");
	}
	
}

/* const image_geometry::PinholeCameraModel& cameraCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	image_geometry::PinholeCameraModel cam_model;
 	cam_model.fromCameraInfo(info_msg);
	return cam_model;
}
 */
void cameraCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	//cam_info = info_msg;
	cam_model.fromCameraInfo(info_msg);
	Camera_flag =1;
	
}
void callback(const sensor_msgs::ImageConstPtr& image_msg, const era_gazebo::DetectionBoxListConstPtr& detection_msg)
{
	if (Camera_flag !=1)
		return;
	
	if(detection_msg->detection_list.size()==0)
		return;
	
	//ros::NodeHandle n1("~");
	//image_geometry::PinholeCameraModel& cam_m1;
	//image_geometry::PinholeCameraModel& cam_m1 = n1.subscribe("camera_info",10,cameraCallback);
	//ros:: Subscriber sub = n1.subscribe("camera_info",10,cameraCallback);
	//sync.registerCallback(boost::bind(&callback, _1, _2));
	
	//const sensor_msgs::CameraInfoConstPtr& info_msg = message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n1,"camera_info",10); ;
	//message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n1,"camera_info",10); 
	//const sensor_msgs::CameraInfoConstPtr& info_msg = n1.subscribe("camera_info",10,cameraCallback);
	//ros:: Subscriber sub = n1.subscribe("camera_info",10,cameraCallback);
	//info_msg = info_sub;
	
	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
	
	
	//saving the file 
	//cv::FileStorage file("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/depth_info_tosave1.txt", cv::FileStorage::WRITE);
	//cv::Mat matx1;
	//cv_ptr->image.convertTo(matx1, CV_16U); // convert to 8uc1
	//write to a file
	//file << "matName" << matx1;
	
	//added depth image filter
	/* cv::Mat smther(cv::Mat cv_ptr)
	{
		medianBlur(cv_ptr->image, cv_ptr->image, 9);
		return cv_ptr->image;
	} */
	cv::Mat depth_mat;
	if (!filter_type)
	{
		//method1
		//cv::Mat depth_mat;
		if (kernel >5)
		{
			cv::Mat mod_image_mat; //declaration
			cv_ptr->image.convertTo(mod_image_mat, CV_16U); //, 1/256); // convert to 8uc1
			/* double minVal; 
			double maxVal; 
			minMaxLoc(mod_image_mat, &minVal, &maxVal);
			cout << "before u8 max val : " << maxVal << "\n"; 
			imwrite("/home/varun/Desktop/Varun_files_desk/Jul_files/2507/u8_before_filter_depth_full_image.png",mod_image_mat*255);*/
			cv::Mat mod_image_mat1; //declaration
			mod_image_mat1 = mod_image_mat/256;
			mod_image_mat1.convertTo(mod_image_mat1, CV_8U);
			//cv::FileStorage file22("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/depth_info_median_file_starndard_u16_type_large_kernel.txt", cv::FileStorage::WRITE);
			//file22 << "mod_image_mat" << mod_image_mat;
			
			//cv::FileStorage file222("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/depth_info_median_file_starndard_u8_type_large_kernel.txt", cv::FileStorage::WRITE);
			//file222 << "mod_image_mat1" << mod_image_mat1;
			
			medianBlur(mod_image_mat1, mod_image_mat1, kernel); //median filter
			//cv::FileStorage file22("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/depth_info_median_file_starndard_u8_type.txt", cv::FileStorage::WRITE);
			//write to a file
			//file22 << "mod_image_mat" << mod_image_mat;
			//imwrite("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/u8_after_filter_depth_full_image.png",mod_image_mat*255);
			/* double minVal1; 
			double maxVal1; 
			minMaxLoc(mod_image_mat, &minVal1, &maxVal1);
			cout << "after u8 max val : " << maxVal1 << "\n"; */
			mod_image_mat.convertTo(depth_mat, CV_32F, 0.001);
			//imwrite("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/depth_image_large_kernal_standard_filter.png",depth_mat*255);
			//cv::FileStorage file2222("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/depth_info_median_file_starndard_32F_type_large_kernel.txt", cv::FileStorage::WRITE);
			//file2222 << "mod_image_mat1" << mod_image_mat1;
			/* double minVal2; 
			double maxVal2; 
			minMaxLoc(depth_mat, &minVal2, &maxVal2);
			cout << "after 32F depth max val : " << maxVal2 << "\n"; */
		}
		else
		{
			cv_ptr->image.convertTo(depth_mat, CV_32F, 0.001); 
			/* double minVal3; 
			double maxVal3; 
			minMaxLoc(depth_mat, &minVal3, &maxVal3);
			cout << "before 32F depth max val : " << maxVal3 << "\n"; 
			imwrite("/home/varun/Desktop/Varun_files_desk/Jul_files/2507/32F_before_filter_depth_full_image.png",depth_mat*255);*/
			medianBlur(depth_mat, depth_mat, 3); //median filter only 3/5 are supported
			//cv::FileStorage file33("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/depth_info_median_file_standard_32f_type_small_kernel.txt", cv::FileStorage::WRITE);
			//write to a file
			//file33 << "depth_mat_32F" << depth_mat;
			//imwrite("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/depth_image_small_kernal_standard_filter.png",depth_mat*255);
			/* double minVal4; 
			double maxVal4;
			minMaxLoc(depth_mat, &minVal4, &maxVal4);
			cout << "after 32F depth max val : " << maxVal4 << "\n"; */
		} 
	}
	
	
	//method2
	//cv::Mat depth_mat_ip; for bilateral filter
	//cv::Mat depth_mat;
	//cv_ptr->image.convertTo(depth_mat, CV_32F, 0.001); 
	//double minVal; 
	//double maxVal; 
	//minMaxLoc(depth_mat, &minVal, &maxVal);
	//cout << "Before max val : " << maxVal << "\n";
	//imwrite("/home/varun/Desktop/Varun_files_desk/Jul_files/2407/32F_before_filter_depth_full_image.png",(depth_mat*255));
	//median filter
	//medianBlur(depth_mat, depth_mat, 7); //median filter
	
	//bilateral filter
	//bilateralFilter(InputArray src, OutputArray dst, int d, double sigmaColor, double sigmaSpace, int borderType=BORDER_DEFAULT )
	/*
	 Various border types, image boundaries are denoted with '|'

	 * BORDER_REPLICATE:     aaaaaa|abcdefgh|hhhhhhh
	 * BORDER_REFLECT:       fedcba|abcdefgh|hgfedcb
	 * BORDER_REFLECT_101:   gfedcb|abcdefgh|gfedcba
	 * BORDER_WRAP:          cdefgh|abcdefgh|abcdefg
	 * BORDER_CONSTANT:      iiiiii|abcdefgh|iiiiiii  with some specified 'i'
	 */
	 //int d= kernel size, double sigmaColor = some numbers, double sigmaSpace = some number;
	//bilateralFilter(depth_mat_ip, depth_mat, 3, 200, 3 );
	
	//joint bilateral filter
	//cv::Mat depth_mat_ip;
	//cv_ptr->image.convertTo(depth_mat_ip, CV_32F, 0.001);
	//jointBilateralFilter(depth_mat_ip, depth_mat_ip, depth_mat, 5, 75, 5);
	
	//guided filter
	//cv::ximgproc::GuidedFilter fil; //fil is the object
	//fil -> filter(depth_mat, depth_mat, -1) ; 
	
	//minMaxLoc(depth_mat, &minVal, &maxVal);
	//cout << "After max val : " << maxVal << "\n";
	//imwrite("/home/varun/Desktop/Varun_files_desk/Jul_files/2407/32F_after_filter_depth_full_image.png",(depth_mat*255));

	else
	{
		//16bit imple
		cv::Mat depth_mat1;
		cv_ptr->image.convertTo(depth_mat1, CV_16U); 
		//size
		cv::Size s = depth_mat1.size();
		int imh = s.height;
		int imw = s.width;
		int k = kernel;
		//cout << "hi2 " << imh << "\n";
		//cout << "hi22 " << imw << "\n";
		//cout << "hi222 " << k << "\n";
		
		cv::Mat_<ushort> reshaped(imh+(k-1), imw+(k-1)); //in size width, height
		//cv::Mat reshaped; //added padding values here
		//cout << "hi5" << "\n";
		reshaped =  pad_input_16u(depth_mat1, imh, imw, k); //k is kernel
		//cout << "hi3" << "\n";
		
		cv::Size ss= reshaped.size();
		int s1= ss.height;
		int s2 = ss.width;
		//cout << "s1 " << s1 << "\n";
		//cout << "s2 " << s2 << "\n";
		//cv::FileStorage file2("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/reshaped_depth.txt", cv::FileStorage::WRITE);
		//file2 << "reshaped" << reshaped;
		
		//cv::Mat output;
		cv::Mat_<ushort> output(imh, imw); //median output
		for (int i=0;i<imh;i++)
		{
			for (int j=0;j<imw;j++)
			{
				//cout << "hi4 " << "\n";
				output.at<ushort>(i,j) = median_16u(reshaped, i, j, k); //full array, present location and kernel size
				//cout << "hi44" << output.at<ushort>(i,j) << "\n";
			}
		}
		//cout << "hi" << "\n";
		//saving the file 
		//cv::FileStorage file1("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/depth_info_median_file_custom_16u_type_any_kernel.txt", cv::FileStorage::WRITE);
		//write to a file
		//file1 << "matName" << output;
		
		//final conversion
		//cv::Mat depth_mat;
		output.convertTo(depth_mat, CV_32F, 0.001);
		//imwrite("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/depth_image_any_kernal_custom_filter.png",depth_mat*255);
		//cv::FileStorage file111("/home/varun/Desktop/Varun_files_desk/Jul_files/2607/depth_info_median_file_custom_32F_type_any_kernel.txt", cv::FileStorage::WRITE);
		//write to a file
		//file111 << "matName" << depth_mat;
	}
	
	
	//actual code starts here
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
	
	
	//vector<int> compression_params;
	//compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	//compression_params.push_back(9);
		
	//imwrite("/home/varun/Desktop/Varun_files_desk/Jul_files/2407/depth_full_image.png",depth_mat*255);//,compression_params);

	
	//image_geometry::PinholeCameraModel cam_model;
 	//cam_model.fromCameraInfo(info_msg);
	int colors[][3] = {{0, 255, 0},{0, 0, 255},{255, 0, 0},{0, 255, 255},{255, 255, 0},{255, 0, 255},{80, 70, 180},{250, 80, 190},{245, 145, 50},{70, 150, 250},{50, 190, 190}};
	
	PointCloud::Ptr msg (new PointCloud); //all objects in one point cloud
 	msg->header.frame_id = image_msg->header.frame_id;
 	msg->height = 1;
	int count = 0;
	msg->points.resize(640*480*detection_msg->detection_list.size()); //set msg point full size
	//cout << " x1  is here " << msg->points.size() << "\n";
 	for(int b = 0; b<detection_msg->detection_list.size(); b++) {

 		era_gazebo::DetectionBox box = (detection_msg->detection_list[b]);

		// new code here box.mask_1d is the mask in 1D array(225= 15x15) elements
		int mask_rows = 15;
		int mask_cols = 15;
		//cout << "mask_rows" << mask_rows << "\n";
		int box_w = box.right - box.left;
		int box_h = box.bottom - box.top;
		// cout << "maks1d " <<  box.mask_1d << endl;
		cv::Mat_<float> mask_2d_in(mask_rows, mask_cols); //init for 2D mask of 15x15
		for(int i=0; i<mask_rows;i++)
		{
			for(int j=0;j<mask_cols;j++)
			{
			mask_2d_in.at<float>(i,j)= box.mask_1d[(i*mask_rows)+j]; //read the elements and put it in 2d matrix
			}
		}  
		
		cv::Mat_<float> mask_2d_out(box_w, box_h); //init for interpolated 2D mask
		resize(mask_2d_in, mask_2d_out, Size(box_w, box_h),  0,  0,  cv::INTER_AREA);  //interpolate to box_w, box_h dims
		//int total = box_w*box_h;
/* 		cout << "mask starts here" << "\n";
		for (int i=0;i<box_h;i++)
		{
			for (int j=0;j<box_w;j++)
			{
				cout << " " << mask_2d_out[i][j];
			}
			cout << " " << "\n";
		}
		cout << "mask ends here" << "\n"; */
		/* ofstream out("mask_2d_data.txt");
		out.write(mask_2d_out,total);
		out.close();
		 */
		int colorindex = b % 11; //available colors size is 11 here
		int *boxcolor = colors[colorindex]; //choose the color RGB values
		int count_new = 0; //init points counter
		//cout << "count (int) here at start " << count_new << "\n";
		//cout << "count inloop here at start " << count << "\n";
		//cout << "boxh here at start " << box_h << "\n";
		//cout << "boxw here at start " << box_w << "\n";
		//msg->points.resize(box_w*box_h); //set msg point full size
		//cout << " Mask threshold (in callback) is here " << mask_threshold << "\n";
		
		//uint8_t r = 255, g = 0, b = 0; //example: red color
		uint8_t r = *(boxcolor), g = *(boxcolor+1), bl = *(boxcolor+2); // color init feom colors list
		uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)bl); //get single float number from RGB values
		
		cv::Mat_<float> depth_to_save(box_w, box_h); //init for 2D mask of 15x15
		
 		for(int u=0; u<box_h; u++)
		{
 			for(int v=0; v<box_w; v++) 
			{
				//cout << " Mask threshold is here " << mask_threshold << "\n";
				if (mask_2d_out.at<float>(u,v) > mask_threshold)
				{
					cv::Point2d vu(v+box.left, u+box.top);
					cv::Point3d xyz = cam_model.projectPixelTo3dRay(vu);

					double depth_value = depth_mat.at<float>(Point(v+box.left, u+box.top));
					depth_to_save.at<float>(v,u) = depth_mat.at<float>(Point(v+box.left, u+box.top)); //save depth image fot this box
					xyz = xyz * depth_value;
					
					msg->points[count+count_new].x = xyz.x;
					msg->points[count+count_new].y = xyz.y;
					msg->points[count+count_new].z = xyz.z;
					
					msg->points[count+count_new].rgb = *reinterpret_cast<float*>(&rgb); //set this color  to points
					
					count_new = count_new+1; 
				}
 			}
 		}
		
		//save to a file
		//cout << "hi all" << "\n";
		//vector<int> compression_params;
		//compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		//compression_params.push_back(9);
		
		//imwrite("/home/varun/Desktop/Varun_files_desk/Jul_files/2207/depth_image.png",depth_to_save.t()*255);//,compression_params);
		//cv::rotate("/home/varun/Desktop/Varun_files_desk/Jul_files/1607/depth_image.png", "/home/varun/Desktop/Varun_files_desk/Jul_files/1607/depth_image.png", cv::ROTATE_90_CLOCKWISE);
		//imwrite("/home/varun/Desktop/Varun_files_desk/Jul_files/1907/mask_image.png",mask_2d_out*255);//,compression_params);
		//cv::rotate("/home/varun/Desktop/Varun_files_desk/Jul_files/1607/mask_image.png", "/home/varun/Desktop/Varun_files_desk/Jul_files/1607/mask_image.png", cv::ROTATE_90_CLOCKWISE);
		
		//cout << "masked points are   " << count_new << "\n";
 		//msg->width = msg->points.size();
		//cout << "points size: inner loop is  " << msg->points.size() << "\n";
 		pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
/* 		
 		double tr1 = (double)getTickCount();
 		PointCloud::Ptr cloud_filtered (new PointCloud);
 		pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    	// build the filter
    	outrem.setInputCloud(msg);
    	outrem.setRadiusSearch(0.1);
    	outrem.setMinNeighborsInRadius (10);
    	// apply filter
    	outrem.filter (*cloud_filtered);
		double tr2 = (double)getTickCount();
		double diff =  (tr2-tr1)/getTickFrequency();
		cout<< "The diff in time (removeoutlier) is (in sec) 	" << diff << "\n"; */
		
		
		/* double tk1 = (double)getTickCount();
		PointCloud::Ptr cloud_filtered (new PointCloud);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; //new RGB PCL
  		sor.setInputCloud (msg);
  		sor.setMeanK (50);
  		sor.setStddevMulThresh (1.0);
  		sor.filter (*cloud_filtered);
		double tk2 = (double)getTickCount();
		double diff =  (tk2-tk1)/getTickFrequency();
		cout<< "The diff in time is (in sec) 	" << diff << "\n";  */
		
		
		
		//int indices_rem[1000] = {0};
		//indices_rem = sorfilter.getRemovedIndices();
		//cout << "rem indices " << indices_rem << "\n";
		
 		//PointCloud::Ptr tf_msg (new PointCloud);
 		//pcl_ros::transformPointCloud(grid_msg.header.frame_id, *cloud_filtered, *tf_msg, *tfListener);

 		// We have a pointcloud transformed and ready for projection

 		double cellResolution = grid_msg.info.resolution;

 		for(int i =count; i<count+count_new; i++) 
		{
 			double x = msg->points[i].x;
 			double y = msg->points[i].y;
 			int xCell = (int) ((x-grid_msg.info.origin.position.x) / cellResolution);
 			int yCell = (int) ((y-grid_msg.info.origin.position.y) / cellResolution);
 			grid_msg.data[yCell*grid_msg.info.width+xCell] = box.id;
 		}
		
 		//cloud_pub.publish (msg); //tf_msg
		//cloud_pub.publish (msg);
		count = count+count_new; //add prev points
		//cout << " x10  is here " << count << "\n";
		//cout << " x11  is here " << count_new << "\n";
 		//cv::rectangle(cv_ptr->image, Point(box.left, box.top), Point(box.right, box.bottom), cv::Scalar(0, 255, 0), 6);
		//cv::putText(cv_ptr->image, std::to_string(box.id), Point(box.left, box.top), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0,255,0),5);
	
 	}
	
	msg->width = count;
	cloud_pub.publish (msg);
	//cout << "points size: outer loop is  " << msg->points.size() << "\n";
	//cloud_pub.publish (msg);
	
	
	grid_pub.publish(grid_msg);

	//sensor_msgs::ImagePtr result_image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, cv_ptr->image).toImageMsg();
	//result_image_msg->header = image_msg->header;
	//result_pub.publish(result_image_msg);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "og_labeler");
	ros::NodeHandle n("~");
	
	//new code
	dynamic_reconfigure::Server<era_gazebo::testConfig> server;
	dynamic_reconfigure::Server<era_gazebo::testConfig>::CallbackType f;
	f = boost::bind(&maskthresholdcallback, _1,_2);
	server.setCallback(f);
	
	f = boost::bind(&filterkernelcallback, _1,_2);
	server.setCallback(f);
	
  	//image_transport::ImageTransport it(n);

	tfListener = new tf::TransformListener();

	image_transport::ImageTransport it(n);
	image_transport::SubscriberFilter image_sub( it, "image_input", 5000);
	message_filters::Subscriber<era_gazebo::DetectionBoxList> detection_sub(n, "object_input", 10);
	//message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n,"camera_info",5000);
	ros:: Subscriber sub = n.subscribe("camera_info",10,cameraCallback);

	tf::MessageFilter<era_gazebo::DetectionBoxList> tf_filter(detection_sub, *tfListener, "camera_link", 5000);	

	message_filters::Synchronizer< mySyncPolicy > sync(mySyncPolicy(5000), image_sub, tf_filter);
	sync.registerCallback(boost::bind(&callback, _1, _2));

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

