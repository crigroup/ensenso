// Standard headers
#include <string>
#include <fstream>

// ROS headers
#include <ros/ros.h>
#include <ros/service.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

// Image transport
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL headers
#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <boost/foreach.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub;
  
  pub = nh.advertise<sensor_msgs::PointCloud2> ("input_cloud", 1);
  
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
  pcl::PCDReader reader;
  //reader.read (argv[1], *cloud_blob);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read (argv[1], *cloud_1);
  
  
  /*ros::Subscriber sub; 
  sub = nh.subscribe ("input_cloud", 1, cloud_callback); */
  
  pcl::toPCLPointCloud2( *cloud_1, *cloud_blob);            // conversion from point_cloud_1 to point_cloud_2. 
  
 
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(*cloud_blob, output);           // conversion from point_cloud_2 to ros sensor msg
  
  ros::Rate loop_rate(10);
  while (nh.ok())
  {
    pub.publish (output);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}




