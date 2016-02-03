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
#include "ensenso/ensenso_grabber.h"

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


// Typedefs
typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;

void extract ( pcl::PCLPointCloud2::Ptr cloud_blob );

class HandeyeCalibration
{
  private:
    // Ros
    ros::NodeHandle                   nh_, nh_private_;
    // Images
    image_transport::CameraPublisher  l_raw_pub_;
    image_transport::CameraPublisher  r_raw_pub_;
    image_transport::Publisher        l_rectified_pub_;
    image_transport::Publisher        r_rectified_pub_;
    // Point cloud
    bool                              point_cloud_;
    ros::Publisher                    cloud_pub_;
    // TF
    std::string                       camera_frame_id_;
    // Ensenso grabber
    pcl::EnsensoGrabber::Ptr          ensenso_ptr_;
    
  public:
     HandeyeCalibration(): 
      nh_private_("~")
    {
      // Read parameters
      std::string serial_no;
      nh_private_.param(std::string("serial_no"), serial_no, std::string("150533"));
      if (!nh_private_.hasParam("serial_no"))
        ROS_WARN_STREAM("Parameter [~serial_no] not found, using default: " << serial_no);
      nh_private_.param("camera_frame_id", camera_frame_id_, std::string("ensenso_optical_frame"));
      if (!nh_private_.hasParam("camera_frame_id"))
        ROS_WARN_STREAM("Parameter [~camera_frame_id] not found, using default: " << camera_frame_id_);
      // Booleans
      bool front_light, projector;
      nh_private_.param("front_light", front_light, false);
      if (!nh_private_.hasParam("front_light"))
        ROS_WARN_STREAM("Parameter [~front_light] not found, using default: " << front_light);
      nh_private_.param("projector", projector, false);
      if (!nh_private_.hasParam("projector"))
        ROS_WARN_STREAM("Parameter [~projector] not found, using default: " << projector);
      nh_private_.param("point_cloud", point_cloud_, false);
      if (!nh_private_.hasParam("point_cloud"))
        ROS_WARN_STREAM("Parameter [~point_cloud] not found, using default: " << point_cloud_);
      // Advertise topics
      image_transport::ImageTransport it(nh_);
      l_raw_pub_ = it.advertiseCamera("left/image_raw", 2);
      r_raw_pub_ = it.advertiseCamera("right/image_raw", 2);
      l_rectified_pub_ = it.advertise("left/image_rect", 2);
      r_rectified_pub_ = it.advertise("right/image_rect", 2);
      //if (point_cloud_)
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2 >("depth/points", 2, true); // Latched
      // Initialize Ensenso
      ensenso_ptr_.reset(new pcl::EnsensoGrabber);
      ensenso_ptr_->openDevice(serial_no);
      ensenso_ptr_->openTcpPort();
      ensenso_ptr_->configureCapture();
      ensenso_ptr_->enableProjector(projector);
      ensenso_ptr_->enableFrontLight(front_light);
      // Start ensenso grabber
      boost::function<void(
          const boost::shared_ptr<PointCloudXYZ>&, 
          const boost::shared_ptr<PairOfImages>&,
          const boost::shared_ptr<PairOfImages>&)> f1 = boost::bind (&HandeyeCalibration::grabberCallback, this, _1, _2, _3);
      boost::function<void(
          const boost::shared_ptr<PairOfImages>&,
          const boost::shared_ptr<PairOfImages>&)> f2 = boost::bind (&HandeyeCalibration::grabberCallback, this, _1, _2);
      if (1)
      //(point_cloud_)
        ensenso_ptr_->registerCallback(f1);
      else
        ensenso_ptr_->registerCallback(f2);
      ensenso_ptr_->start();
    }
    
    ~HandeyeCalibration()
    {
      ensenso_ptr_->closeTcpPort();
      ensenso_ptr_->closeDevice();
    }
    
    void grabberCallback( const boost::shared_ptr<PairOfImages>& rawimages, const boost::shared_ptr<PairOfImages>& rectifiedimages)
    {
      // Get cameras info
      sensor_msgs::CameraInfo linfo, rinfo;
      ensenso_ptr_->getCameraInfo("Left", linfo);
      ensenso_ptr_->getCameraInfo("Right", rinfo);
      linfo.header.frame_id = camera_frame_id_;
      rinfo.header.frame_id = camera_frame_id_;
      // Images
      l_raw_pub_.publish(*toImageMsg(rawimages->first), linfo, ros::Time::now());
      r_raw_pub_.publish(*toImageMsg(rawimages->second), rinfo, ros::Time::now());
      l_rectified_pub_.publish(toImageMsg(rectifiedimages->first));
      r_rectified_pub_.publish(toImageMsg(rectifiedimages->second));
    }
    
    void grabberCallback( const boost::shared_ptr<PointCloudXYZ>& cloud,
                          const boost::shared_ptr<PairOfImages>& rawimages, const boost::shared_ptr<PairOfImages>& rectifiedimages)
    {
      // Get cameras info
      sensor_msgs::CameraInfo linfo, rinfo;
      ensenso_ptr_->getCameraInfo("Left", linfo);
      ensenso_ptr_->getCameraInfo("Right", rinfo);
      linfo.header.frame_id = camera_frame_id_;
      rinfo.header.frame_id = camera_frame_id_;
      // Images
      l_raw_pub_.publish(*toImageMsg(rawimages->first), linfo, ros::Time::now());
      r_raw_pub_.publish(*toImageMsg(rawimages->second), rinfo, ros::Time::now());
      l_rectified_pub_.publish(toImageMsg(rectifiedimages->first));
      r_rectified_pub_.publish(toImageMsg(rectifiedimages->second));
      // Point cloud
      cloud->header.frame_id = camera_frame_id_;
      sensor_msgs::PointCloud2 cloud_msg;
       pcl::toROSMsg 	(*  cloud, cloud_msg);
      cloud_pub_.publish(cloud_msg);
      pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
      pcl::PCLPointCloud2 pc1_pc2;
      // pcl::fromROSMsg (cloud_msg, cloud_blob);
      pcl_conversions::toPCL(cloud_msg, pc1_pc2);
      ROS_INFO ("publishing cloud");
      *cloud_blob = pc1_pc2;
      extract(cloud_blob);
    }
    
    sensor_msgs::ImagePtr toImageMsg(pcl::PCLImage pcl_image)
    {
      unsigned char *image_array = reinterpret_cast<unsigned char *>(&pcl_image.data[0]);
      int type(CV_8UC1);
      std::string encoding("mono8");
      if (pcl_image.encoding == "CV_8UC3")
      {
        type = CV_8UC3;
        encoding = "bgr8";
      }
      cv::Mat image_mat(pcl_image.height, pcl_image.width, type, image_array);
      std_msgs::Header header;
      header.frame_id = "world";
      header.stamp = ros::Time::now();
      return cv_bridge::CvImage(header, encoding, image_mat).toImageMsg();
    }
};

boost::shared_ptr<pcl::visualization::PCLVisualizer>  givecentroid ( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud,centroid);
  std::cout << "The XYZ coordinates of the centroid are: ("<< centroid[0] << ", "<< centroid[1] << ", "<< centroid[2] << ")." << std::endl;
  
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //viewer->setBackgroundColor (255, 255, 255);
 // srand(static_cast<unsigned int>(time(0)));
 // std::cout<<rand() % 255<<endl;
  /*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, rand() % 255,rand() % 255,rand() % 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color,"sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  
 /* while (!viewer->wasStopped ())
  {
  viewer->spinOnce (100);
  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }*/
  //return (viewer);
    
    
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
  while (cloud_filtered->points.size () > 0.7 * nr_points)
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

 void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    //do stuff with temp_cloud here
  }

int main(int argc, char **argv)
{
  ros::init (argc, argv, "ensenso");
  HandeyeCalibration cal;
  ros::spin();
  ros::shutdown();
  return 0;
}
