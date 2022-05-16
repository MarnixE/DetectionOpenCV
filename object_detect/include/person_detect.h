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

  void publishDebugImage(cv::Mat& img,
                         const std::vector<cv::Rect>& detections) const;


  double _imageScaling;
  mutable cv_bridge::CvImage _cvImgDebug;

  boost::scoped_ptr<cv::HOGDescriptor> _hogCPU;

  image_transport::ImageTransport _imageTransport, _privateImageTransport;
  image_transport::Subscriber _imageSub;
  ros::Time _imgTimeStamp;

  ros::Publisher _detectionPub;
  image_transport::Publisher _imDebugPub;

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
  _imDebugPub = _privateImageTransport.advertise("debug", 1);

  _detectionPub = _pnh.advertise<object_recognition_msgs::RecognizedObject>("results", 1);

  cv::namedWindow("person detections");
}

PersonDetector::~PersonDetector()
{
  cv::destroyWindow("person detections");
}

void PersonDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
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

  detectPersons(img, detections);

  if ( _imageScaling != 1.0 )
  {
    scaleDetections(detections,
                    static_cast<double>(cvImgPtr->image.cols)/static_cast<double>(img.cols),
                    static_cast<double>(cvImgPtr->image.rows)/static_cast<double>(img.rows));
  }

  publishDetections(detections);

  cv::Mat imDebug = cvImgPtr->image.clone();

  publishDebugImage(imDebug, detections);
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
  object_recognition_msgs::RecognizedObject detection;

  msg.header.frame_id = "";
  msg.header.stamp    = _imgTimeStamp;

  BOOST_FOREACH(const cv::Rect& roi, detections)
  {
    detection.bounding_contours[0].x      = roi.x;
    detection.bounding_contours[0].y      = roi.y;
    // detection.width  = roi.width;
    // detection.height = roi.height;
    msg.bounding_contours[0].x      = roi.x;
    msg.bounding_contours[0].y      = roi.y;

    // msg.bounding_contours.push_back(detection.bounding_contours);
  }

  _detectionPub.publish(msg);
}

void PersonDetector::publishDebugImage(cv::Mat& img,
                                       const std::vector<cv::Rect>& detections) const
{
  //draw detections
  BOOST_FOREACH(const cv::Rect& roi, detections)
  {
    cv::rectangle(img, roi, CV_RGB(0,255,0), 2);
  }

  if ( img.channels() == 3 && img.depth() == CV_8U )
    _cvImgDebug.encoding = sensor_msgs::image_encodings::BGR8;

  else if ( img.channels() == 1 && img.depth() == CV_8U )
    _cvImgDebug.encoding = sensor_msgs::image_encodings::MONO8;
  else
    throw std::runtime_error("Error in Detector2dNode::publishDebug: only 24-bit BGR or 8-bit MONO images are currently supported");

  _cvImgDebug.image = img;
  sensor_msgs::Image imgMsg;
  imgMsg.header.stamp = _imgTimeStamp;
  _cvImgDebug.toImageMsg(imgMsg); //copy image data to ROS message

  _imDebugPub.publish(imgMsg);
}

