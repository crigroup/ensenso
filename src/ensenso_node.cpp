// Standard headers
#include <string>
#include <fstream>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>

// PCL headers
#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ensenso_grabber.h>

/** Pair of PCL images */
typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;

/** PCL point object. */
typedef pcl::PointXYZ PointXYZ;

/** PCL Point cloud object */
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;

/** Pointer to the node */
boost::shared_ptr<ros::NodeHandle> node;

/** PCL Ensenso object pointer */
pcl::EnsensoGrabber::Ptr ensenso;
/** Left Ensenso image publisher */
boost::shared_ptr<ros::Publisher> l_image_pub;
/** Right Ensenso image publisher */
boost::shared_ptr<ros::Publisher> r_image_pub;

/** Point cloud publisher */
boost::shared_ptr<ros::Publisher> cloud_pub;

/** @brief Publish Ensenso grabber images and point cloud
 * @param[in] cloud The Ensenso point cloud
 * @param[in] images Pair of Ensenso images (raw or with overlay)
 * @warning Image type changes if a calibration pattern is discovered/lost;
 * check @c images->first.encoding */
void grabberCallback( const boost::shared_ptr<PointCloudXYZ>& cloud,
                      const boost::shared_ptr<PairOfImages>& images )
{
  // Images
  unsigned char *l_image_array = reinterpret_cast<unsigned char *>(&images->first.data[0]);
  unsigned char *r_image_array = reinterpret_cast<unsigned char *>(&images->second.data[0]);
  int type(CV_8UC1);
  std::string encoding("mono8");
  if (images->first.encoding == "CV_8UC3")
  {
    type = CV_8UC3;
    encoding = "bgr8";
  }
  cv::Mat l_image(images->first.height, images->first.width, type, l_image_array);
  cv::Mat r_image(images->first.height, images->first.width, type, r_image_array);
  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time::now();
  l_image_pub->publish(cv_bridge::CvImage(header, encoding, l_image).toImageMsg());
  r_image_pub->publish(cv_bridge::CvImage(header, encoding, r_image).toImageMsg());
  // Point cloud
  cloud->header.frame_id = "world";
  cloud_pub->publish(cloud);
}

/**
 * The main function
 * @param[in] argc
 * @param[in] argv
 * @return Exit status */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ensenso_node");
  node.reset(new ros::NodeHandle);
  
  // Initialize Ensenso
  ensenso.reset(new pcl::EnsensoGrabber);
  ensenso->openDevice(0);
  ensenso->openTcpPort();
  ensenso->configureCapture();

  l_image_pub.reset(new ros::Publisher);
  r_image_pub.reset(new ros::Publisher);
  *l_image_pub = node->advertise<sensor_msgs::Image>("left/image_raw", 2);
  *r_image_pub = node->advertise<sensor_msgs::Image>("right/image_raw", 2);

  cloud_pub.reset(new ros::Publisher);
  *cloud_pub = node->advertise<PointCloudXYZ>("depth/points", 2, true); // Latched

  // Register Ensenso callback and start it
 boost::function<void
  (const boost::shared_ptr<PointCloudXYZ>&,
   const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&grabberCallback, _1, _2);
  ensenso->registerCallback(f);
  ensenso->start();

  // Spin
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (node->ok())
  {
    ros::Duration(0.01).sleep();
  }

  ensenso->closeTcpPort();
  ensenso->closeDevice();
  return 0;
}
