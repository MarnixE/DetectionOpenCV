
// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include <boost/scoped_ptr.hpp>
#include <boost/foreach.hpp>
#include <vector>

// PCL includes
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <ros/callback_queue.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

// Header files
#include <person_detect.h>
#include <pcl_detection.h>

PersonDetector::PersonDetector(ros::NodeHandle& nh,
                               ros::NodeHandle& pnh,
                               double imageScaling, 
                               const std::string &topic, 
                               const std::string &transport):
  _nh(nh),
  _pnh(pnh),
  _imageScaling(imageScaling),
  _imageTransport(nh),
  _privateImageTransport(pnh)
{  

  _hogCPU.reset( new cv::HOGDescriptor );
  _hogCPU->setSVMDetector( cv::HOGDescriptor::getDefaultPeopleDetector() );

  image_transport::TransportHints transportHint(transport);

  _imageSub   = _imageTransport.subscribe(topic, 1, &PersonDetector::imageCallback, this, transportHint);

  _detectionPub = _pnh.advertise<object_recognition_msgs::RecognizedObject>("results", 1);

  cv::namedWindow("person detections");
}

PersonDetector::~PersonDetector()
{
  cv::destroyWindow("person detections");
}

void PersonDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Image handling from ROS msg to OpenCV format
  cv_bridge::CvImageConstPtr cvImgPtr;
  cvImgPtr = cv_bridge::toCvShare(msg);

  _imgTimeStamp = msg->header.stamp;

  cv::Mat img(static_cast<int>(_imageScaling*cvImgPtr->image.rows),
              static_cast<int>(_imageScaling*cvImgPtr->image.cols),
              cvImgPtr->image.type());

  if ( _imageScaling == 1.0 )
    cvImgPtr->image.copyTo(img);
  else
  {
    cv::resize(cvImgPtr->image, img, img.size());
  }

  std::vector<cv::Rect> detections;

  // Perform detection
  detectPersons(img, detections);

  if ( _imageScaling != 1.0 )
  {
    scaleDetections(detections,
                    static_cast<double>(cvImgPtr->image.cols)/static_cast<double>(img.cols),
                    static_cast<double>(cvImgPtr->image.rows)/static_cast<double>(img.rows));
  }

  publishDetections(detections);
}

void PersonDetector::scaleDetections(std::vector<cv::Rect>& detections,
                                     double scaleX, double scaleY) const
{
  BOOST_FOREACH(cv::Rect& detection, detections)
  {
    cv::Rect roi(detection);
    detection.x      = static_cast<long>(roi.x      * scaleX);
    detection.y      = static_cast<long>(roi.y      * scaleY);
    detection.width  = static_cast<long>(roi.width  * scaleX);
    detection.height = static_cast<long>(roi.height * scaleY);
  }
}


void PersonDetector::detectPersons(const cv::Mat& img,
                                   std::vector<cv::Rect>& detections)
{ 
  double start = static_cast<double>(cv::getTickCount());

  _hogCPU->detectMultiScale(img,
                            detections,
                            0,                //hit threshold: decrease in order to increase number of detections but also false alarms
                            cv::Size(8,8),    //win stride
                            cv::Size(0,0),    //padding 24,16
                            1.02,             //scaling
                            1,                //final threshold
                            false);            //use mean-shift to fuse detections

  double stop = static_cast<double>(cv::getTickCount());
  ROS_DEBUG_STREAM("Elapsed time in detectMultiScale: " << 1000.0*(stop-start)/cv::getTickFrequency() << " ms");
}

void PersonDetector::publishDetections(const std::vector<cv::Rect>& detections) const
{
  object_recognition_msgs::RecognizedObject msg;

  // Create message for detected human
  BOOST_FOREACH(const cv::Rect& roi, detections)
  {
    msg.header.frame_id = "";
    msg.header.stamp    = _imgTimeStamp;

    msg.type.key = "human";
  }

  _detectionPub.publish(msg);
}

/////////////////////////////////
///   PCL detection /////////////
/////////////////////////////////

PCLdetect::PCLdetect(ros::NodeHandle& nh,
                     ros::NodeHandle& pnh,
                     const std::string &topic):
  _nh(nh),
  _pnh(pnh)
{
    ros::Subscriber sub = _nh.subscribe(topic, 1, &PCLdetect::PCLcallback, this);

    _detectionPub = _pnh.advertise<std_msgs::Float64>("pcl_results", 1);
}

PCLdetect::~PCLdetect()
{
}


void PCLdetect::PCLcallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    ROS_INFO_STREAM("PCL CB");
    pcl_conversions::toPCL(*msg, cloud);

    PCLfilter();

    detection();

}

void PCLdetect::PCLfilter()
{
    ROS_INFO_STREAM("FILTER");
    // Create object and pointer for SOR
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud< pcl::PointXYZ>);
 
    pcl::fromPCLPointCloud2( cloud, *point_cloud);
    
    // Perform filtering
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (point_cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (cloud_filtered);

}

void PCLdetect::detection()
{
    double norm;
    double min_dist = INFINITY;
    BOOST_FOREACH( const pcl::PointXYZ pt, cloud_filtered)
    {
        norm = sqrt(std::pow(pt.x,2) + std::pow(pt.y,2) + std::pow(pt.z,2));
        if (norm < min_dist){
            min_dist = norm;
        }
    }
    std::cout << 1 << std::endl;
    ROS_INFO_STREAM("Publishing pcl");
    std_msgs::Float64 msg;
    msg.data = min_dist;
    _detectionPub.publish(msg);
}



int main(int argc, char **argv) 
{
  ros::init(argc,argv,"object_detect_node"); // Create and name the Node
  ros::NodeHandle nh, pnh("~");

  ros::CallbackQueue cbQueue;
  nh.setCallbackQueue(&cbQueue);

  double scale = 1.0;
  pnh.param<double>("scale",   scale,    scale);

  double freq = 10;
  pnh.param<double>("rate",   freq,    freq);

  std::string imTransport = "raw";
  pnh.param<std::string>("transport",   imTransport,    imTransport);

  // Creating topics to publish
  std::string topic_img = "/realsense/color/image_raw";
  std::string topic_pcl = "/velodyne_points";
  pnh.param<std::string>("image", topic_img, topic_img);
  pnh.param<std::string>("pcl", topic_pcl, topic_pcl);

  ROS_INFO_STREAM("Setting image scale factor to: " << scale);
  ROS_INFO_STREAM("Setting detector max rate to:  " << freq);
  ROS_INFO_STREAM("Image type:  " << imTransport);
  ROS_INFO(" ");

  ROS_INFO_STREAM("Creating person detector ...");

  PersonDetector ImgDetector(nh, pnh, scale, topic_img, imTransport);
  PCLdetect PCLDetector(nh, pnh, topic_pcl);

  ROS_INFO_STREAM("Spinning to serve callbacks ...");

  ros::Rate rate(freq);
  while ( ros::ok() )
  {
    cbQueue.callAvailable();
    rate.sleep();
  }

  return 0;

}