// Standard headers
#include <string>
#include <fstream>
#include <ctime>
#include <iostream>
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
#include <pcl/console/parse.h>

#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
// PCL noise filtering
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

bool voxel_reduction (false);
bool stat_removal (false);
bool radius_removal (false);

int rangeimager ( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
{
 
	// Parameters needed by the planar range image object:
 
	// Image size. Both Kinect and Xtion work at 640x480.
	int imageSizeX = 640;
	int imageSizeY = 480;
	// Center of projection. here, we choose the middle of the image.
	float centerX = 640.0f / 2.0f;
	float centerY = 480.0f / 2.0f;
	// Focal length. The value seen here has been taken from the original depth images.
	// It is safe to use the same value vertically and horizontally.
	float focalLengthX = 525.0f, focalLengthY = focalLengthX;
	// Sensor pose. Thankfully, the cloud includes the data.
	Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
								 cloud->sensor_origin_[1],
								 cloud->sensor_origin_[2])) *
								 Eigen::Affine3f(cloud->sensor_orientation_);
	// Noise level. If greater than 0, values of neighboring points will be averaged.
	// This would set the search radius (e.g., 0.03 == 3cm).
	float noiseLevel = 0.0f;
	// Minimum range. If set, any point closer to the sensor than this will be ignored.
	float minimumRange = 0.0f;
 
	// Planar range image object.
	pcl::RangeImagePlanar rangeImagePlanar;
	rangeImagePlanar.createFromPointCloudWithFixedSize(*cloud, imageSizeX, imageSizeY,
			centerX, centerY, focalLengthX, focalLengthX,
			sensorPose, pcl::RangeImage::CAMERA_FRAME,
			noiseLevel, minimumRange);
 
	// Visualize the image.
	/*pcl::visualization::RangeImageVisualizer viewer("Planar range image");
	viewer.showRangeImage(rangeImagePlanar);*/
  
  pcl::PointCloud<pcl::BorderDescription>::Ptr borders(new pcl::PointCloud<pcl::BorderDescription>);
  pcl::RangeImageBorderExtractor borderExtractor(&rangeImagePlanar);
	borderExtractor.compute(*borders);
  
  pcl::NarfKeypoint detector(&borderExtractor);
  pcl::PointCloud<int>::Ptr keypoints(new pcl::PointCloud<int>);
  detector.setRangeImage(&rangeImagePlanar);
  detector.getParameters().support_size = 0.2f;
	detector.compute(*keypoints);
  pcl::visualization::RangeImageVisualizer viewer("NARF keypoints");
	viewer.showRangeImage(rangeImagePlanar);
	for (size_t i = 0; i < keypoints->points.size(); ++i)
	{
		viewer.markPoint(keypoints->points[i] % rangeImagePlanar.width,
						 keypoints->points[i] / rangeImagePlanar.width,
						 // Set the color of the pixel to red (the background
						 // circle is already that color). All other parameters
						 // are left untouched, check the API for more options.
						 pcl::visualization::Vector3ub(1.0f, 0.0f, 0.0f));
	}
  
  /*
  // Visualize the borders.
	pcl::visualization::RangeImageVisualizer* viewer = NULL;
	viewer = pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(rangeImagePlanar,
			 -std::numeric_limits<float>::infinity(),
			 std::numeric_limits<float>::infinity(),
			 false, *borders, "Borders");*/
  
  
  
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		// Sleep 100ms to go easy on the CPU.
    std::cout<<"inside viewer"<<endl;
		pcl_sleep(0.1);
	}
}



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


class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    
    pub = nh.advertise<sensor_msgs::PointCloud2>("input_cloud", 1);

    //Topic you want to subscribe
    sub = nh.subscribe("/ensenso/depth/points", 2, &SubscribeAndPublish::callback, this);
  }

  void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr PCL1_temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr PCL1_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*PCL1_temp_cloud);
    //do stuff with temp_cloud here
    ROS_INFO("inside Callback");
    //givecentroid(PCL1_temp_cloud);
    
      
      // reduce pointcloud size
      // Create the filtering object: downsample the dataset using a leaf size of 1cm
      pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2 (pcl_pc2) ), cloud_filtered_blob (new pcl::PCLPointCloud2);

      
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      sensor_msgs::PointCloud2 output;
      
      if (voxel_reduction)
      {
        std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
        // Create the filtering object: downsample the dataset using a leaf size of 2cm
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cloud_blob);
        sor.setLeafSize (0.02f, 0.02f, 0.02f);
        sor.filter (*cloud_filtered_blob);           //pcd saved in cloud_filtered_blob
        
        // Convert to the templated PointCloud
        pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);      // convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object.
        
        std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
        pcl::toROSMsg 	(*  cloud_filtered, output);
      }
      
      
      if ( stat_removal )  // remove noisy measurements (StatisticalOutlierRemoval filter)
      {
        
        std::cerr << "Cloud before statistical filtering: " << std::endl;
        std::cerr << *PCL1_temp_cloud << std::endl;
        
        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (PCL1_temp_cloud);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*PCL1_cloud_filtered);
        
        std::cerr << "Cloud after filtering: " << std::endl;
        std::cerr << *PCL1_cloud_filtered << std::endl;
        pcl::toROSMsg 	(*PCL1_cloud_filtered, output);
        
      }
      
      if ( radius_removal )  // remove noisy measurement (Radius Outlier removal filter)
      {
        std::cerr << "Cloud before Radius filtering: " << std::endl;
        std::cerr << *PCL1_temp_cloud << std::endl;
        
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(PCL1_temp_cloud);
        outrem.setRadiusSearch(0.8);
        outrem.setMinNeighborsInRadius (2);
        outrem.filter (*PCL1_cloud_filtered);
        
        std::cerr << "Cloud after filtering: " << std::endl;
        std::cerr << *PCL1_cloud_filtered << std::endl;
        pcl::toROSMsg 	(*PCL1_cloud_filtered, output);
      }

  pub.publish (output);


  }

private:
  ros::NodeHandle nh; 
  ros::Publisher pub;
  ros::Subscriber sub;

};//End of class SubscribeAndPublish

 void cloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    //do stuff with temp_cloud here
    ROS_INFO("inside Callback");
    //givecentroid(temp_cloud);
    
      
      // reduce pointcloud size
      // Create the filtering object: downsample the dataset using a leaf size of 1cm
      pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2 (pcl_pc2) ), cloud_filtered_blob (new pcl::PCLPointCloud2);

      
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
      
      // Create the filtering object: downsample the dataset using a leaf size of 2cm
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloud_blob);
      sor.setLeafSize (0.01f, 0.01f, 0.01f);
      sor.filter (*cloud_filtered_blob);           //pcd saved in cloud_filtered_blob
      
      // Convert to the templated PointCloud
      pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);      // convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object.
      
      std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
      
  ros::NodeHandle nh;
  ros::Publisher pub;
  
  pub = nh.advertise<sensor_msgs::PointCloud2> ("input_cloud", 1);
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg 	(*  cloud_filtered, output);
  pub.publish (output);
  
  
      
   // rangeimager(cloud_filtered);
}


void parseCommandLine (int argc, char *argv[])
{
  if (pcl::console::find_switch (argc, argv, "-v"))
  {
    voxel_reduction = true;
  }
  if (pcl::console::find_switch (argc, argv, "-s"))
  {
    stat_removal = true;
  }
  if (pcl::console::find_switch (argc, argv, "-r"))
  {
    radius_removal = true;
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cloud_sub");
 // ros::NodeHandle nh;
//  ros::Rate loop_rate(10);
  ROS_INFO("inside main");
  // Create a ROS subscriber for the input point cloud
//  ros::Subscriber sub;
  parseCommandLine (argc, argv);
//    sub = nh.subscribe ("/ensenso/depth/points", 2, cloud_callback);
    SubscribeAndPublish SAPObject;
 // sub = nh.subscribe ("input_cloud", 2, cloud_callback);
  ros::spin ();
}
