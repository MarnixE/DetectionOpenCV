// // ROS headers
// #include <ros/ros.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <ros/callback_queue.h>
// #include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
// #include <object_recognition_msgs/RecognizedObject.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// // Boost headers
// #include <boost/scoped_ptr.hpp>
// #include <boost/foreach.hpp>

// // Std C++ headers
// #include <vector>

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