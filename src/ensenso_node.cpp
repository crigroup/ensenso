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

// Ensenso package
#include "ensenso/ensenso_grabber.h"
#include "ensenso/CameraLights.h"


// Typedefs
typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;


class EnsensoNode
{
  private:
    // Ros
    ros::NodeHandle                   nh_, nh_private_;
    ros::ServiceServer                ligths_srv_;
    // Images
    image_transport::CameraPublisher  l_raw_pub_;
    image_transport::CameraPublisher  r_raw_pub_;
    image_transport::Publisher        l_rectified_pub_;
    image_transport::Publisher        r_rectified_pub_;
    // Point cloud
    bool                              point_cloud_;
    ros::Publisher                    cloud_pub_;
    // Camera info
    ros::Publisher                    linfo_pub_;
    ros::Publisher                    rinfo_pub_;

    // TF
    std::string                       camera_frame_id_;
    // Ensenso grabber
    pcl::EnsensoGrabber::Ptr          ensenso_ptr_;
    
  public:
     EnsensoNode(): 
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
      if (point_cloud_)
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2 >("depth/points", 2, true); // Latched
      linfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("left/camera_info", 2, true);
      rinfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("right/camera_info", 2, true);
      // Advertise services
      ligths_srv_ = nh_.advertiseService("camera_lights", &EnsensoNode::cameraLigthsCB, this);
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
          const boost::shared_ptr<PairOfImages>&)> f1 = boost::bind (&EnsensoNode::grabberCallback, this, _1, _2, _3);
      boost::function<void(
          const boost::shared_ptr<PairOfImages>&,
          const boost::shared_ptr<PairOfImages>&)> f2 = boost::bind (&EnsensoNode::grabberCallback, this, _1, _2);
      if (point_cloud_)
        ensenso_ptr_->registerCallback(f1);
      else
        ensenso_ptr_->registerCallback(f2);
      ensenso_ptr_->start();
    }
    
    ~EnsensoNode()
    {
      ensenso_ptr_->closeTcpPort();
      ensenso_ptr_->closeDevice();
    }
    
    bool cameraLigthsCB(ensenso::CameraLightsRequest  &req, ensenso::CameraLightsResponse &res)
    {
      ensenso_ptr_->enableProjector(req.projector);
      ensenso_ptr_->enableFrontLight(req.front_light);
      res.success = true;
      return true;
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
      // Camera_info
      linfo_pub_.publish(linfo);
      rinfo_pub_.publish(rinfo);
      // Point cloud
      cloud->header.frame_id = camera_frame_id_;
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud, cloud_msg);
      cloud_pub_.publish(cloud_msg);
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
  ros::init (argc, argv, "ensenso");
  EnsensoNode ensenso_node;
  ros::spin();
  ros::shutdown();
  return 0;
}
