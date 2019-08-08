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

#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Image, era_gazebo::DetectionBoxList> mySyncPolicy; //changed

//global values
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
int depth_filter_type = 0; //0 for none and 1 for median filter, 2 for CC
bool pcl_filter = 0; //pcl filter 0: none, 1: statistical outlier removal
int CC_threshold = 2;

//new declarations for functions
cv::Mat_<ushort> pad_input_16u(cv::Mat_<ushort> , int , int , int );
ushort median_16u(cv::Mat_<ushort> , int , int , int );
cv::Mat_<ushort> get_kernel_16u(cv::Mat_<ushort> , int , int , int );
cv::Mat_<ushort> sort_16u(cv::Mat_<ushort> , int );
ushort find_median_16u(cv::Mat_<ushort>  , int );


//new definitions
cv::Mat_<ushort> pad_input_16u(cv::Mat_<ushort> total_input, int imh, int imw, int k) 
{
	
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



//old code with opencv C++ and ROS
void configcallback(era_gazebo::testConfig &config, uint32_t level)
{
	depth_filter_type =  config.Depth_Filter_type;
	ROS_INFO("Depth filter type (0: None, 1: Median filter, 2: Connected components): [%d]", config.Depth_Filter_type);
	if (depth_filter_type ==1) //median filter
	{
		ROS_INFO("New_value of median filter type (0: standard, 1: Custom_CV_16u): [%d]", config.Median_Filter_type);
		ROS_INFO("New_value of median filter kernel: [%d]", config.Median_Filter_kernel);
		filter_type =  config.Median_Filter_type;
		kernel =  config.Median_Filter_kernel;
		if (kernel % 2 == 0)
		{
			kernel = kernel +1;
			ROS_INFO("Kernel size is even, so kernel is changed to next odd number internally ");
		}
	}
	
	if (depth_filter_type != 2) //median filter or no filter for mask threshold
	{
		mask_threshold =  config.Mask_threshold;
		ROS_INFO("New_value of mask threshold: [%f]", config.Mask_threshold);
	}
	if (depth_filter_type == 2) //median filter or no filter
	{
		CC_threshold =  config.CC_Depth_Filter_threshold;
		ROS_INFO("New_value of depth filter threshold: [%d]", config.CC_Depth_Filter_threshold);
	}
	pcl_filter = config.PCL_Filter; //use later
	ROS_INFO("PCL filter type (0: None, 1: Statisticaloutlierremoval): [%d]", config.PCL_Filter);
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
	cam_model.fromCameraInfo(info_msg);
	Camera_flag =1;
}
void callback(const sensor_msgs::ImageConstPtr& image_msg, const era_gazebo::DetectionBoxListConstPtr& detection_msg)
{
	double tc1 = (double)getTickCount();
	if (Camera_flag !=1)
		return;
	
	if(detection_msg->detection_list.size()==0)
		return;
	
	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
	
	//save full image
	//cv::Mat save_full_img;
	//cv_ptr->image.convertTo(save_full_img, CV_16U);  //direct reading from cv_ptr -> convert to float
	//cv::normalize(save_full_img, save_full_img, 0, 255, NORM_MINMAX, CV_8U);
	//imwrite("/home/varun/Desktop/Varun_files_desk/Aug_files/0808/depth_full_input_8u.png",save_full_img);
	
	
	cv::Mat depth_mat; //declaration for depth_image after filtering
	if (depth_filter_type == 0)
	{
		ROS_INFO("No filtering on depth image is applied ");
		cv_ptr->image.convertTo(depth_mat, CV_32F, 0.001);  //direct reading from cv_ptr -> convert to float
	}
	//cv::FileStorage file1("/home/varun/Desktop/Varun_files_desk/Aug_files/0808/input.txt", cv::FileStorage::WRITE);
	//file1 << "input image" << cv_ptr->image;
	
	//cv::FileStorage file2("/home/varun/Desktop/Varun_files_desk/Aug_files/0808/depth_mat.txt", cv::FileStorage::WRITE);
	//file2 << "depth_mat" << depth_mat;
	
	if (depth_filter_type == 1)
	{	
		ROS_INFO("Median filter on depth image is applied");
		if (!filter_type)
		{
			//method1
			if (kernel >5)
			{
				cv::Mat mod_image_mat; //declaration
				cv_ptr->image.convertTo(mod_image_mat, CV_16U); //, 1/256); // convert to 8uc1
				cv::normalize(mod_image_mat, mod_image_mat, 0, 255, NORM_MINMAX, CV_8U);
				//cv::Mat mod_image_mat1; //declaration
				//mod_image_mat1 = mod_image_mat/256;
				//mod_image_mat1.convertTo(mod_image_mat1, CV_8U);
				medianBlur(mod_image_mat, mod_image_mat, kernel); //median filter
				mod_image_mat.convertTo(depth_mat, CV_32F, 0.001);
			}
			else //kernel is either 3 or 5 only here
			{
				cv_ptr->image.convertTo(depth_mat, CV_32F, 0.001); 
				medianBlur(depth_mat, depth_mat, 3); //median filter only 3/5 are supported
			} 
		}
		
		else
		{
			//16bit implementation
			cv::Mat depth_mat1;
			cv_ptr->image.convertTo(depth_mat1, CV_16U); 
			//size
			cv::Size s = depth_mat1.size();
			int imh = s.height;
			int imw = s.width;
			int k = kernel;
			
			cv::Mat_<ushort> reshaped(imh+(k-1), imw+(k-1)); //in size width, height
			reshaped =  pad_input_16u(depth_mat1, imh, imw, k); //k is kernel
			
			//cv::Mat output;
			cv::Mat_<ushort> output(imh, imw); //median output
			for (int i=0;i<imh;i++)
			{
				for (int j=0;j<imw;j++)
				{
					output.at<ushort>(i,j) = median_16u(reshaped, i, j, k); //full array, present location and kernel size
				}
			}
			//final conversion for cv::Mat depth_mat;
			output.convertTo(depth_mat, CV_32F, 0.001);
		}	
	} //end of median filter
		
	//CC implementation starts from here
	if (depth_filter_type == 2) //at present per detection box it is applied so see inside for loop of objects
	{
		ROS_INFO("2.5D CC filtering on depth image (per object) is applied");
	}
	// end of CC filtering on depth image
	
	
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
	
	//init for diff colors for mask model
	int colors[][3] = {{0, 255, 0},{0, 0, 255},{255, 0, 0},{0, 255, 255},{255, 255, 0},{255, 0, 255},{80, 70, 180},{250, 80, 190},{245, 145, 50},{70, 150, 250},{50, 190, 190}};
	
	PointCloud::Ptr msg (new PointCloud); //all objects in one point cloud
 	msg->header.frame_id = image_msg->header.frame_id;
 	msg->height = 1;
	int count = 0; //present object points in point cloud
	
	msg->points.resize(640*480*detection_msg->detection_list.size()); //set msg point full size
	
	//Iteration for all detected objects with score more than threshold in detection_tensorflow.py file
 	for(int b = 0; b<detection_msg->detection_list.size(); b++) 
	{

 		era_gazebo::DetectionBox box = (detection_msg->detection_list[b]);
		
		int box_w = box.right - box.left;
		int box_h = box.bottom - box.top;
		cv::Mat_<float> mask_2d_out(box_w, box_h); //init for interpolated 2D mask
		cv::Mat_<float> depth_to_save(box_w, box_h); //save depth pixels that processed here only 
		
		if (depth_filter_type !=2) //for non CC part use mask info and project in 3D only if the mask value is more than mask threshold
		{	
			//init for variables here
			// new code here box.mask_1d is the mask in 1D array(225= 15x15) elements
			int mask_rows = 15;
			int mask_cols = 15;
			cv::Mat_<float> mask_2d_in(mask_rows, mask_cols); //init for 2D mask of 15x15
			
			for(int i=0; i<mask_rows;i++)
			{
				for(int j=0;j<mask_cols;j++)
				{
					mask_2d_in.at<float>(i,j)= box.mask_1d[(i*mask_rows)+j]; //read the elements and put it in 2d matrix
				}
			}  
			resize(mask_2d_in, mask_2d_out, Size(box_w, box_h),  0,  0,  cv::INTER_AREA);  //interpolate to box_w, box_h dims 
		}
		
		//CC on depth image applied here
		if (depth_filter_type ==2) //extract only req depth image here 
		{
			cv::Mat_<ushort> depth_out_save; //init for depth 2D image u16
			cv_ptr->image.convertTo(depth_out_save, CV_16U); // for saving and use at end only
			
			cv::Mat_<ushort> depth_stats(box_h,box_w); //declaration
			for (int u = 0; u < box_h ; u++)
			{
				for (int v = 0; v < box_w; v++)
				{
					depth_stats.at<ushort>(u, v) = depth_out_save.at<ushort>(u+box.top , v+box.left); //select our part here for this box for stats only
				}
			}
			double min_ind_2, max_ind_2;
			cv::Point min_ind_loc_2, max_ind_loc_2;
			cv::minMaxLoc(depth_stats, &min_ind_2, &max_ind_2, &min_ind_loc_2, &max_ind_loc_2);
			//cout << "max_val " << max_ind_2 << " min_val " << min_ind_2 << endl;
			
			cv::Mat_<uchar> depth_out(box_h, box_w); //init for depth 2D image u16
			for (int u = 0; u < box_h; u++)
			{
				for (int v = 0; v < box_w; v++)
				{
					depth_out.at<uchar>(u, v) = uchar((depth_stats.at<ushort>(u , v)-min_ind_2)*(255/max_ind_2)); //select our part here for this box
				}
			}
			
			//imwrite("/home/varun/Desktop/Varun_files_desk/Aug_files/0808/depth_input_8u.png",depth_out);
			// add equivalent code here
			//int CC_threshold = 3; //taken from dynamic config
			int counter = 0; //init the counter for detected objects in the detection box (in z-dimension howmany objects present like, main object/background)
			cv::Mat label_step1 = cv::Mat::zeros(box_h, box_w, CV_16S); //init for labels
			
			//1st row label
			int i = 0;
			for (int j = 1; j < box_w; j++)
			{
				if ((depth_out.at<uchar>(i, j) - depth_out.at<uchar>(i, j - 1)) > CC_threshold)
				{
					counter = counter + 1;
					label_step1.at<short>(i, j) = counter;
				}
				else
				{
					if (depth_out.at<uchar>(i, j) > 0)
					{
						label_step1.at<short>(i, j) = counter;
					}
				}
			}
			
			//2nd row onwards label
			for (int i = 1; i < box_h; i++)
			{
				for (int j = 0; j < box_w; j++) // all cols now
				{
					if (depth_out.at<uchar>(i, j) > 0)
					{
						
						if (j==0) //added code for 1st column here
						{
							double a_1 = double(depth_out.at<uchar>(i, j));
							double c_1 = double(depth_out.at<uchar>(i - 1, j));
							if (abs(a_1 - c_1) > CC_threshold)
							{
								counter = counter + 1;
								label_step1.at<short>(i, j) = counter;
							}
							else
							{
								label_step1.at<short>(i, j) = label_step1.at<short>(i-1, j); //top is connected
							}	
						}
						else //not 1st column
						{
							double a_1 = double(depth_out.at<uchar>(i, j));
							double b_1 = double(depth_out.at<uchar>(i, j - 1));
							double c_1 = double(depth_out.at<uchar>(i - 1, j));
							
							if (abs(a_1 - b_1) > CC_threshold)
							{
								if (abs(a_1 - c_1) > CC_threshold) //new label
								{
									counter = counter + 1;
									label_step1.at<short>(i, j) = counter;
								}
								else
								{
									label_step1.at<short>(i, j) = label_step1.at<short>(i - 1, j); //top is connected
								}
							}
							else
							{
								if (abs(a_1 - c_1) > CC_threshold)
								{
									label_step1.at<short>(i, j) = label_step1.at<short>(i, j - 1); //left is connected
								}
								else
								{
									ushort temp1;
									if (label_step1.at<short>(i, j - 1) > label_step1.at<short>(i - 1, j))
									{
										temp1 = label_step1.at<short>(i - 1, j);
									}
									else
									{
										temp1 = label_step1.at<short>(i , j-1);
									}
									label_step1.at<short>(i, j) = temp1; //min(label_step1.at<short>(i, j - 1), label_step1.at<short>(i - 1, j)); //top,left connected, select
									//minimum out of both
								}
							}
						}
					}
				}
			}
			
			//hash table custom
			cv::Mat eye_mat = cv::Mat::eye(counter, counter, CV_16S); //init the hash table (used matrix to store more than one value for each key value, i.e to store a list of values for each key)

			short set_mask = 1; //mark the position if mapping exists here
			//1st row
			i=0;
			for (int j = 1; j < box_w; j++)
			{
				short c1 = 0;
				short c2 = 0;
				double a_1 = double(depth_out.at<uchar>(i, j));
				double b_1 = double(depth_out.at<uchar>(i, j - 1));
				if ((label_step1.at<short>(i, j) != 0) && (label_step1.at<short>(i, j - 1) != 0))
				{
					if ((label_step1.at<short>(i, j) != label_step1.at<short>(i, j - 1)) && (abs(a_1 - b_1) < CC_threshold))
					{
						c1 = label_step1.at<short>(i, j);
						c2 = label_step1.at<short>(i, j - 1);
						eye_mat.at<float>(c1-1, c2-1) = set_mask;
						eye_mat.at<float>(c2-1, c1-1) = set_mask; //transpose here
					}
				}
			} 
			
			//all rows and cols
			for (int i = 1; i < box_h; i++)
			{
				for (int j = 1; j < box_w; j++)
				{
					short c1 = 0;
					short c2 = 0;
					short c3 = 0;
					short c4 = 0;
					double a_1 = double(depth_out.at<uchar>(i, j));
					double b_1 = double(depth_out.at<uchar>(i, j - 1));
					double c_1 = double(depth_out.at<uchar>(i - 1, j));
					if ((label_step1.at<short>(i, j) != 0) && (label_step1.at<short>(i, j - 1) != 0))
					{
						if ((label_step1.at<short>(i, j) != label_step1.at<short>(i, j - 1)) && (abs(a_1 - b_1) < CC_threshold))
						{
							c1 = label_step1.at<short>(i, j);
							c2 = label_step1.at<short>(i, j - 1);
							eye_mat.at<short>(c1-1, c2-1) = set_mask;
							eye_mat.at<short>(c2-1, c1-1) = set_mask; //transpose here
						}
					}
					if ((label_step1.at<short>(i, j) != 0) && (label_step1.at<short>(i - 1, j) != 0))
					{
						if ((label_step1.at<short>(i, j) != label_step1.at<short>(i - 1, j)) && (abs(a_1 - c_1) < CC_threshold))
						{
							c3 = label_step1.at<short>(i, j);
							c4 = label_step1.at<short>(i - 1, j);
							eye_mat.at<short>(c3-1, c4-1) = set_mask;
							eye_mat.at<short>(c4-1, c3-1) = set_mask; //transpose here
						}
					}
				}
			}
		 
			//1st cols
			int j=0;
			for (int i = 1; i < box_h; i++)
			{
				short c3 = 0;
				short c4 = 0;
				double a_1 = double(depth_out.at<uchar>(i, j));
				double c_1 = double(depth_out.at<uchar>(i - 1, j));
				if ((label_step1.at<short>(i, j) != 0) && (label_step1.at<short>(i - 1, j) != 0))
				{
					if ((label_step1.at<short>(i, j) != label_step1.at<short>(i - 1, j)) && (abs(a_1 - c_1) < CC_threshold))
					{
						c3 = label_step1.at<short>(i, j);
						c4 = label_step1.at<short>(i - 1, j);
						eye_mat.at<short>(c3, c4) = set_mask;
						eye_mat.at<short>(c4, c3) = set_mask; //transpose here
					}
				}
			} 

			//demap here
			for (short i = counter-1; i >= 1; i--)
			{
				for (short j = i - 1; j >= 0; j--)
				{
					if (eye_mat.at<short>(i, j) != 0) 
					{
						for (int ii = 0; ii < box_h; ii++)
						{
							for (int jj = 0; jj < box_w; jj++)
							{
								if (label_step1.at<short>(ii, jj) == i+1)
								{
									label_step1.at<short>(ii, jj) = (j+1); //map with smaller one
								}
							}
						}
					}
				}
			}
				
			double min_count, max_count; //find maximum labels in depth detection box
			cv::Point minIdx, maxIdx;
			cv::minMaxLoc(label_step1, &min_count, &max_count, &minIdx, &maxIdx);	
			int max_ele = max_count; //force to int value
			cv::Mat count_c1 = cv::Mat::zeros(1, max_ele, CV_32F);

			for (int k = 1; k < max_ele+1; k++)
			{
				for (int i = 0; i < box_h; i++)
				{
					for (int j = 0; j < box_w; j++)
					{
						if (label_step1.at<short>(i, j) == k)
						{
							count_c1.at<float>(0, k) = count_c1.at<float>(0, k) + 1;
						}
					}
				}
			}

			// only that label pixels here with maximum frequnecy
			double min_ind, max_ind;
			cv::Point min_ind_loc, max_ind_loc;
			cv::minMaxLoc(count_c1, &min_ind, &max_ind, &min_ind_loc, &max_ind_loc);
			int e1, e2;
			e1 = max_ind_loc.x;
			e2 = max_ind_loc.y;
			
			//reading only the depth values for the highest frequency label
			cv::Mat sub_image1 = cv::Mat::zeros(box_h, box_w, CV_8U);
			cv::Mat sub_image2 = cv::Mat::zeros(box_h, box_w, CV_16U);
			for (int i = 0; i < box_h; i++)
			{
				for (int j = 0; j < box_w; j++)
				{
					if (label_step1.at<short>(i, j) == e1)
					{
						sub_image1.at<uchar>(i, j) = depth_out.at<uchar>(i, j);  //mat_input_depth any one we can take
					}
				}
			}
			
			//convert u8 data type to u16 using previous scaling factors
			for (int i = 0; i < box_h; i++)
			{
				for (int j = 0; j < box_w; j++)
				{
					double aux_1 = double(sub_image1.at<uchar>(i, j)); //temp value
					sub_image2.at<ushort>(i, j) = (aux_1*max_ind_2/255)+min_ind_2; //scaled value of depth info (in u16 format)
				}
			}
			//imwrite("/home/varun/Desktop/Varun_files_desk/Aug_files/0808/sub_image1.png",sub_image1);
			sub_image2.convertTo(depth_mat, CV_32F, 0.001); //final step to convert to depth_mat
			//release mat to free the memory: src.release(); // free mem
			depth_stats.release();
			depth_out.release();
			depth_out_save.release();
			label_step1.release();
			eye_mat.release();
			count_c1.release();
			sub_image1.release();	
			sub_image2.release();	
			//save_full_img.release();
			// end of CC filtering on depth image
		}
		
		//color_information of this object
		int colorindex = b % 11; //available colors size is 11 here
		int *boxcolor = colors[colorindex]; //choose the color RGB values
		int count_new = 0; //init points counter
				
		//uint8_t r = 255, g = 0, b = 0; //example: red color
		uint8_t r = *(boxcolor), g = *(boxcolor+1), bl = *(boxcolor+2); // color init feom colors list
		uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)bl); //get single float number from RGB values
		
 		for(int u=0; u<box_h; u++)
		{
 			for(int v=0; v<box_w; v++) 
			{
				if (depth_filter_type ==2) //extract only req depth image here  
				{
					cv::Point2d vu(v+box.left, u+box.top);
					cv::Point3d xyz = cam_model.projectPixelTo3dRay(vu);
					
					double depth_value = depth_mat.at<float>(Point(v, u));
					depth_to_save.at<float>(v,u) = depth_mat.at<float>(Point(v, u)); //save depth image for this box taken in reverse way
					xyz = xyz * depth_value;
					
					msg->points[count+count_new].x = xyz.x;
					msg->points[count+count_new].y = xyz.y;
					msg->points[count+count_new].z = xyz.z;
					
					msg->points[count+count_new].rgb = *reinterpret_cast<float*>(&rgb); //set this color  to points
				
					count_new = count_new+1; 	//increase counter for number of points
				}
				
				else // not CC so take from mask information
				{
					if (mask_2d_out.at<float>(u,v) > mask_threshold)
					{
						cv::Point2d vu(v+box.left, u+box.top);
						cv::Point3d xyz = cam_model.projectPixelTo3dRay(vu);
						
						double depth_value = depth_mat.at<float>(Point(v+box.left, u+box.top));
						depth_to_save.at<float>(v,u) = depth_mat.at<float>(Point(v+box.left, u+box.top)); //save depth image for this box in reverse way
						xyz = xyz * depth_value;
						
						msg->points[count+count_new].x = xyz.x;
						msg->points[count+count_new].y = xyz.y;
						msg->points[count+count_new].z = xyz.z;
						
						msg->points[count+count_new].rgb = *reinterpret_cast<float*>(&rgb); //set this color  to points
						
						count_new = count_new+1; 
					}
				}
 			}
 		}
			
		//enable this for debug only
		//imwrite("/home/varun/Desktop/Varun_files_desk/Aug_files/0708/depth_image.png",depth_to_save.t()*255); //since saved one is in reverse order
		
 		pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
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
		
		count = count+count_new; //add to prev points
		
 		//cv::rectangle(cv_ptr->image, Point(box.left, box.top), Point(box.right, box.bottom), cv::Scalar(0, 255, 0), 6);
		//cv::putText(cv_ptr->image, std::to_string(box.id), Point(box.left, box.top), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0,255,0),5);
 	}
	
	msg->width = count; //total number of points here
	//display either filtered or not PCL
	if (pcl_filter ==1)
	{
		//statistical removal filter 
		PointCloud::Ptr cloud_filtered (new PointCloud);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; //new RGB PCL
		sor.setInputCloud (msg);
		sor.setMeanK (5);
		sor.setStddevMulThresh (1.0);
		sor.filter (*cloud_filtered);
		
		/* radius outlier removal  filter
 		PointCloud::Ptr cloud_filtered (new PointCloud);
 		pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    	// build the filter
    	outrem.setInputCloud(msg);
    	outrem.setRadiusSearch(0.1);
    	outrem.setMinNeighborsInRadius (10);
    	// apply filter
    	outrem.filter (*cloud_filtered); */

		PointCloud::Ptr tf_msg (new PointCloud);
 		pcl_ros::transformPointCloud(grid_msg.header.frame_id, *cloud_filtered, *tf_msg, *tfListener); //tf_msg here
		cloud_pub.publish (tf_msg); 
	}
	else
	{
		cloud_pub.publish (msg);
	}
	
	grid_pub.publish(grid_msg); //display grid_msg here

	//sensor_msgs::ImagePtr result_image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, cv_ptr->image).toImageMsg();
	//result_image_msg->header = image_msg->header;
	//result_pub.publish(result_image_msg);
	
	//enable this for checking time
	double tc2 = (double)getTickCount();
	double diff_time =  (tc2-tc1)/getTickFrequency();
	//cout<< "The callback time is: (in sec) 	" << diff_time << "\n"; 
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "og_labeler");
	ros::NodeHandle n("~");
	
	//new code for dynamic configuration of parametrs and callback
	dynamic_reconfigure::Server<era_gazebo::testConfig> server;
	dynamic_reconfigure::Server<era_gazebo::testConfig>::CallbackType f;
	f = boost::bind(&configcallback, _1,_2);
	server.setCallback(f); //all configurations are called here no sepearte mask threshold and filter kernel callbacks
	
  	//image_transport::ImageTransport it(n);

	tfListener = new tf::TransformListener();

	image_transport::ImageTransport it(n);
	image_transport::SubscriberFilter image_sub( it, "image_input", 10000);
	message_filters::Subscriber<era_gazebo::DetectionBoxList> detection_sub(n, "object_input", 10);
	//message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n,"camera_info",5000); 
	//taken camrea information directly (another callback) no need to sync more frames for objs and camreas infos and depth_image
	ros:: Subscriber sub = n.subscribe("camera_info",10,cameraCallback);

	tf::MessageFilter<era_gazebo::DetectionBoxList> tf_filter(detection_sub, *tfListener, "camera_link", 10000);	

	message_filters::Synchronizer< mySyncPolicy > sync(mySyncPolicy(10000), image_sub, tf_filter);
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