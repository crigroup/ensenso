#include <string>
#include <fstream>
#include <iostream>
#include <ctime>

// PCL headers
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

// Image transport
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <boost/foreach.hpp>

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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

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


void extract ( pcl::PCLPointCloud2::Ptr cloud_blob )
{
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector4f centroid;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);           //pcd saved in cloud_filtered_blob
  
  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);      // convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object.

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
  
  
  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  //writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);  

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);         //optimizing ransac best fit model coefficient to reduce noise. false to save time
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);                   // no visible difference iterating 100 or 1000 times
  seg.setDistanceThreshold (0.01);               //  (too low --> good data is rejected, too high --> outliers are not removed).

  // Create the filtering object`
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size (); 
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);        
    //	true if all points _except_ the input indices will be returned, false otherwise 
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);         
    pcl::compute3DCentroid(*cloud_p, centroid);
    std::cout << "The XYZ coordinates of the centroid are: ("<< centroid[0] << ", "<< centroid[1] << ", "<< centroid[2] << ")." << std::endl;
      
      
    
    givecentroid(cloud_p);
    // Create the filtering object
    extract.setNegative (true);
    //	true if all points _except_ the input indices will be returned, false otherwise 
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }
}


void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  /*printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z); */
      pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
      cloud_blob = reinterpret_cast <pcl::PCLPointCloud2::Ptr>(cloud);
}

int main (int argc, char** argv)
{
  
  ros::init(argc, argv, "metal_plate");
  ros::NodeHandle nh;
  
  //pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2); 
  ros::Subscriber sub = nh.subscribe<PointCloud>("depth/points", 1, callback);
  ros::spin();
  
  //pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2); 
 
  // read pcd from file.
  /*pcl::PCDReader reader;
  reader.read (argv[1], *cloud_blob);

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;*/
  
  //extract (cloud_blob);

  return (0);
}


