// Standard headers
#include <string>
#include <fstream>
#include <ctime>

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



boost::shared_ptr<pcl::visualization::PCLVisualizer>  givecentroid ( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud,centroid);
  std::cout << "The XYZ coordinates of the centroid are: ("<< centroid[0] << ", "<< centroid[1] << ", "<< centroid[2] << ")." << std::endl;
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //viewer->setBackgroundColor (255, 255, 255);
  srand(static_cast<unsigned int>(time(0)));
  std::cout<<rand() % 255<<endl;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, rand() % 255,rand() % 255,rand() % 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color,"sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  
  while (!viewer->wasStopped ())
  {
  viewer->spinOnce (100);
  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return (viewer);
    
    
   // return 0;
}




 void cloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    //do stuff with temp_cloud here
    ROS_INFO("inside Callback");
    givecentroid(temp_cloud);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cloud_sub");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub;
  
 // while (nh.ok())
  //{
    sub = nh.subscribe ("input_cloud", 1, cloud_callback);
  //  ros::spinOnce ();
  //  loop_rate.sleep ();
 // }

  // Spin
  ros::spin ();
}
