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
#include <ensenso/CalibrationMoveRandom.h>
//image transport
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL headers
#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>
#include "cri_ensenso_grabber.h"


// Typedefs
typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;

class HandeyeCalibration
{
  private:
    // Ros
    ros::NodeHandle                   nh_, nh_private_;
    image_transport::ImageTransport   it_;
    image_transport::Publisher        l_raw_pub_;
    image_transport::Publisher        r_raw_pub_;
    image_transport::Publisher        l_rectified_pub_;
    image_transport::Publisher        r_rectified_pub_;
    
    // Ensenso grabber
    pcl::EnsensoGrabber::Ptr  ensenso_ptr_;
    
  public:
     HandeyeCalibration(): 
      nh_private_("~"),
      it_(nh_)
    { 
      // Initialize Ensenso
      ensenso_ptr_.reset(new pcl::EnsensoGrabber);
      ensenso_ptr_->openDevice(0);
      ensenso_ptr_->openTcpPort();
      ensenso_ptr_->configureCapture(true, true, 1, 0.32, true, 1, false, false, false, 10, false);
      // Setup image publishers      
      l_raw_pub_ = it_.advertise("/left/image_raw", 2);
      r_raw_pub_ = it_.advertise("/right/image_raw", 2);
      l_rectified_pub_ = it_.advertise("/left/image_rect", 2);
      r_rectified_pub_ = it_.advertise("/right/image_rect", 2);
      
      // Start ensenso grabber
      boost::function<void
      (const boost::shared_ptr<PointCloudXYZ>&,
       const boost::shared_ptr<PairOfImages>&,const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&HandeyeCalibration::grabberCallback, this, _1, _2, _3);
      ensenso_ptr_->registerCallback(f);
      ensenso_ptr_->start();
    }
    
    ~HandeyeCalibration()
    {
      ensenso_ptr_->closeTcpPort();
      ensenso_ptr_->closeDevice();
    }
    
    void grabberCallback( const boost::shared_ptr<PointCloudXYZ>& cloud,
                      const boost::shared_ptr<PairOfImages>& rawimages,  const boost::shared_ptr<PairOfImages>& rectifiedimages)
    {
      l_raw_pub_.publish(toImageMsg(rawimages->first));
      r_raw_pub_.publish(toImageMsg(rawimages->second));
      l_rectified_pub_.publish(toImageMsg(rectifiedimages->first));
      r_rectified_pub_.publish(toImageMsg(rectifiedimages->second));
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

int main(int argc, char **argv)
{
  ros::init (argc, argv, "handeye_calibration_node");
  HandeyeCalibration cal;
  ros::spin();
  ros::shutdown();
  return 0;
}
