#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PCLdetect
{
public:
    PCLdetect(ros::NodeHandle& nh,
    ros::NodeHandle& pnh,
    const std::string &topic = "/velodyne_points");
    // const std::string &transport = "raw");

virtual ~PCLdetect();

protected:
    ros::NodeHandle _nh, _pnh;
    
    void PCLcallback(const PointCloud::ConstPtr& msg);
    
    void downsample(const PointCloud::ConstPtr& msg);
    
    void objectDetect(const )
};