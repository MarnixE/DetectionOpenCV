// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <object_recognition_msgs/RecognizedObject.h>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Boost headers
#include <boost/scoped_ptr.hpp>
#include <boost/foreach.hpp>

// Std C++ headers
#include <vector>

class PersonDetector
{
public:
      PersonDetector(ros::NodeHandle& nh,
                 ros::NodeHandle& pnh,
                 double imageScaling = 1.0, 
                 const std::string &topic = "/realsense/color/image_raw", 
                 const std::string &transport="raw");
  virtual ~PersonDetector();

protected:

  ros::NodeHandle _nh, _pnh;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void detectPersons(const cv::Mat& img,
                     std::vector<cv::Rect>& detections);

  void scaleDetections(std::vector<cv::Rect>& detections,
                       double scaleX, double scaleY) const;

  void publishDetections(const std::vector<cv::Rect>& detections) const;

  double _imageScaling;

  boost::scoped_ptr<cv::HOGDescriptor> _hogCPU;

  image_transport::ImageTransport _imageTransport, _privateImageTransport;
  image_transport::Subscriber _imageSub;
  ros::Time _imgTimeStamp;

  ros::Publisher _detectionPub;

};


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
