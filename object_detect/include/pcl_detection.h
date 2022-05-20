#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PCLdetect
{
public:
    PCLdetect(ros::NodeHandle& nh,
                ros::NodeHandle& pnh,
                const std::string &topic = "/velodyne_points");
    

virtual ~PCLdetect();

protected:
    ros::NodeHandle _nh, _pnh;
    
    void PCLcallback(const sensor_msgs::PointCloud2::ConstPtr& input);
    
    void PCLfilter();

    void detection();

    // Output container
    sensor_msgs::PointCloud2 output;
    
    // Container for filtered pointcloud
    pcl::PCLPointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        
    ros::Time _pclTimeStamp;
    ros::Publisher _detectionPub;
};

