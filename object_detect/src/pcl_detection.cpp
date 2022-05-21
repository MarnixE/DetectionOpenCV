// #include <ros/ros.h>

// #include <sensor_msgs/PointCloud2.h>
// #include <ros/callback_queue.h>

// #include <pcl/pcl_base.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/filters/voxel_grid.h>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// class PCLdetect
// {
// public:
//     PCLdetect(ros::NodeHandle& nh,
//                 ros::NodeHandle& pnh,
//                 const std::string &topic = "/velodyne_points");
    

// virtual ~PCLdetect();

// protected:
//     ros::NodeHandle _nh, _pnh;
    
//     void PCLcallback(const sensor_msgs::PointCloud2::ConstPtr& input);
    
//     void PCLfilter();

//     void detection();

//     // Output container
//     sensor_msgs::PointCloud2 output;
    
//     // Container for filtered pointcloud
//     pcl::PCLPointCloud2 cloud;
//     pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        
//     ros::Time _pclTimeStamp;
//     ros::Publisher _detectionPub;
// };

// PCLdetect::PCLdetect(ros::NodeHandle& nh,
//                      ros::NodeHandle& pnh,
//                      const std::string &topic):
//   _nh(nh),
//   _pnh(pnh)
// {
//     ros::Subscriber sub = _nh.subscribe(topic, 1, &PCLdetect::PCLcallback, this);
// }


// void PCLdetect::PCLcallback(const sensor_msgs::PointCloud2ConstPtr& msg)
// {
//     // _pclTimeStamp = msg->header;
//     pcl_conversions::toPCL(*msg, cloud);

//     PCLfilter();

// }

// void PCLdetect::PCLfilter()
// {
//     // Create object and pointer for SOR
//     pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud< pcl::PointXYZ>);
 
//     pcl::fromPCLPointCloud2( cloud, *point_cloud);
    
//     // Create the filtering object
//     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//     sor.setInputCloud (point_cloud);
//     sor.setMeanK (50);
//     sor.setStddevMulThresh (1.0);
//     sor.filter (cloud_filtered);

// }

// void PCLdetect::detection()
// {
//     // TOOD
// }

// int main(int argc, char **argv) 
// {
//   ros::init(argc,argv,"object_detect_node"); // Create and name the Node
//   ros::NodeHandle nh, pnh("~");

//   ros::CallbackQueue cbQueue;
//   nh.setCallbackQueue(&cbQueue);

//   // TODO

//   double freq = 10;
  
//   ros::Rate rate(freq);
//   while ( ros::ok() )
//   {
//     rate.sleep();
//   }

//   return 0;

// }