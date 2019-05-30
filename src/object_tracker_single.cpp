#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/circular_buffer.hpp>
#include <opencv2/video/tracking.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64.h>

#include <dlib/image_processing.h>
#include <dlib/opencv.h>

#include <iostream>
#include <mutex>

#include "era_gazebo/DetectionBoxList.h"
#include "era_gazebo/DetectionBox.h"

typedef boost::circular_buffer<sensor_msgs::Image> circular_buffer;

using namespace message_filters;
using namespace cv;
using namespace dlib;

std::list<sensor_msgs::Image> image_list;
circular_buffer image_list_cb{100};
ros::Publisher quality_pub;
ros::Publisher object_pub;

std::mutex mtx;

image_transport::Publisher result_pub;

// Maybe put these 3 variables in a class
std::list<sensor_msgs::Image>::iterator current_it;
drectangle bbox;
std::string object_type;
unsigned int object_id = -1;

bool tracking_started = false;
bool stop_tracking = false;
bool request_detection = true;

boost::thread *tracking_t;

void tracking_thread()
{
  while(!request_detection) {

    std_msgs::Float64 quality_msg;

    mtx.lock();
    
    correlation_tracker tracker;

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(*current_it, "bgr8");
    cv_image<bgr_pixel> cimg(cv_ptr->image);

    tracker.start_track(cimg, bbox);

    image_list.erase(image_list.begin(), current_it);
    
    mtx.unlock();

    ROS_INFO_STREAM("Tracking Started");

    while(!stop_tracking) {
      //begin = ros::Time::now();
      mtx.lock();

      if(image_list.back().header.stamp == current_it->header.stamp) {
	     // There are no new images
       mtx.unlock();
       continue;
     }

     if(image_list.size() > 3) {
      current_it++;
      current_it++;
      image_list.erase(image_list.begin(), current_it);
     }
     else {
      current_it++;
      image_list.pop_front();
     }

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(*current_it, "bgr8");
    cv_image<bgr_pixel> cimg(cv_ptr->image);

    double update_perf = tracker.update(cimg);

    quality_msg.data = update_perf;
    quality_pub.publish(quality_msg);

    if(update_perf < 5.0) {
	    // Tracking quality is not good
	     // Request for a new detection
      ROS_WARN_STREAM("Poor Tracking (" << update_perf << "). Request new detection");
      request_detection = true;
      stop_tracking = true;
    }


    bbox = tracker.get_position();

    era_gazebo::DetectionBoxList objs_msg;
    objs_msg.header = current_it->header;

    era_gazebo::DetectionBox obj_msg;
    obj_msg.type = object_type;
    obj_msg.id = object_id;
    obj_msg.left = bbox.left();
    obj_msg.right = bbox.right();
    obj_msg.top = bbox.top();
    obj_msg.bottom = bbox.bottom();

    objs_msg.detection_list.push_back(obj_msg);

    object_pub.publish(objs_msg);

    cv::rectangle(cv_ptr->image, Point(bbox.left(), bbox.top()), Point(bbox.right(), bbox.bottom()), cv::Scalar(0, 255, 0), 6);
    cv::putText(cv_ptr->image, std::to_string(object_id), Point(bbox.left(), bbox.top()), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0,255,0),5);

    sensor_msgs::ImagePtr result_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    result_image_msg->header = current_it->header;
    result_pub.publish(result_image_msg);

    mtx.unlock();
      //ROS_INFO_STREAM("Tracking loop time: " << ros::Time::now() - begin);

    }
  }
}



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  mtx.lock();
  image_list.push_back(*msg);
  mtx.unlock();
}

void objectCallback(const era_gazebo::DetectionBoxListConstPtr& msg)
{
  if(msg->detection_list.size()==0)
    return;

  // If we are not tracking anything
  if(object_id == -1) {
    mtx.lock();

    // Find the right image
    current_it = image_list.end(); 

    for(; current_it != image_list.begin(); --current_it) {
      if( current_it->header.stamp == msg->header.stamp ) {

       image_list.erase(image_list.begin(), current_it);

       era_gazebo::DetectionBox f = (msg->detection_list[0]);
       bbox  = drectangle(f.left, f.top, f.right, f.bottom);
       object_type = f.type;
       object_id = f.id;

       tracking_started = true;
       request_detection = false;

       tracking_t = new boost::thread(tracking_thread);

       break;
     }
   }

   mtx.unlock();
 }
 else {

  if(request_detection) {

      //check if the object we are tracking is detected again

      //ROS_INFO_STREAM("DetectionBox no: " << msg->detection_list.size());
    for(int i=0; i<msg->detection_list.size(); i++) {
     if(msg->detection_list[i].type == object_type) {
	  //ROS_INFO_STREAM("Found same category object. Checking if it is the same one.");
	  //Check if it is the same object

       if(object_id ==  msg->detection_list[i].id) {
	    // Found the object

         tracking_t->join();
         stop_tracking = false;

         mtx.lock();

	    // Find the right image
         current_it = image_list.end();

         for(; current_it != image_list.begin(); --current_it) {
           if( current_it->header.stamp == msg->header.stamp ) {

            image_list.erase(image_list.begin(), current_it);

            era_gazebo:DetectionBox f = (msg->detection_list[0]);
            bbox  = drectangle(f.left, f.top, f.right, f.bottom);

            tracking_started = true;
            request_detection = false;
            tracking_t = new boost::thread(tracking_thread);

            break;
          }
        }

        mtx.unlock();
        return;
      }	  
    }
  }     
}   
}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_tracker");
  ros::NodeHandle n("~");
  image_transport::ImageTransport it(n);

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  ros::Subscriber obj_sub = n.subscribe("object_input", 1, objectCallback);
  
  image_transport::Subscriber image_sub = it.subscribe("image_input", 100, imageCallback);
  
  quality_pub = n.advertise<std_msgs::Float64>("tracking_quality", 1);

  object_pub = n.advertise<era_gazebo:DetectionBoxList>("tracked_objects",1);

  result_pub = it.advertise("tracking_image", 1);
  
  ros::waitForShutdown();
  
}
