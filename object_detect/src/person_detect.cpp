// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <object_recognition_msgs/RecognizedObject.h>
// #include <object_recognition_msgs/ObjectType.h>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Boost headers
#include <boost/scoped_ptr.hpp>
#include <boost/foreach.hpp>

// Std C++ headers
#include <vector>
#include <person_detect.h>
#include <pcl_detection.h>


int main(int argc, char **argv) 
{
  ros::init(argc,argv,"person_detect_node"); // Create and name the Node
  ros::NodeHandle nh, pnh("~");

  ros::CallbackQueue cbQueue;
  nh.setCallbackQueue(&cbQueue);

  double scale = 1.0;
  pnh.param<double>("scale",   scale,    scale);

  double freq = 10;
  pnh.param<double>("rate",   freq,    freq);

  std::string imTransport = "raw";
  pnh.param<std::string>("transport",   imTransport,    imTransport);

  std::string topic = "/realsense/color/image_raw";
  pnh.param<std::string>("image", topic, topic);

  ROS_INFO_STREAM("Setting image scale factor to: " << scale);
  ROS_INFO_STREAM("Setting detector max rate to:  " << freq);
  ROS_INFO_STREAM("Image type:  " << imTransport);
  ROS_INFO(" ");

  ROS_INFO_STREAM("Creating person detector ...");

  PersonDetector detector(nh, pnh, scale, topic, imTransport);

  ROS_INFO_STREAM("Spinning to serve callbacks ...");

  ros::Rate rate(freq);
  while ( ros::ok() )
  {
    cbQueue.callAvailable();
    rate.sleep();
  }

  return 0;

}