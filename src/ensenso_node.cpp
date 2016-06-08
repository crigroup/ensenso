// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
// Ensenso grabber
#include <ensenso/ensenso_grabber.h>
// Image transport
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ensenso/CaptureParametersConfig.h>


// Typedefs
typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;


class EnsensoNode
{
  private:
    // ROS
    ros::NodeHandle                   nh_, nh_private_;
    dynamic_reconfigure::Server<ensenso::CaptureParametersConfig> reconfigure_server_;
    // Images
    image_transport::CameraPublisher  l_raw_pub_;
    image_transport::CameraPublisher  r_raw_pub_;
    image_transport::Publisher        l_rectified_pub_;
    image_transport::Publisher        r_rectified_pub_;
    // Point cloud
    ros::Publisher                    cloud_pub_;
    // Camera info
    ros::Publisher                    linfo_pub_;
    ros::Publisher                    rinfo_pub_;
    // TF
    std::string                       camera_frame_id_;
    // Ensenso grabber
    boost::signals2::connection       connection_;
    pcl::EnsensoGrabber::Ptr          ensenso_ptr_;
    
  public:
     EnsensoNode(): 
      nh_private_("~")
    {
      // Read parameters
      std::string serial;
      nh_private_.param(std::string("serial"), serial, std::string("150534"));
      if (!nh_private_.hasParam("serial"))
        ROS_WARN_STREAM("Parameter [~serial] not found, using default: " << serial);
      nh_private_.param("camera_frame_id", camera_frame_id_, std::string("ensenso_optical_frame"));
      if (!nh_private_.hasParam("camera_frame_id"))
        ROS_WARN_STREAM("Parameter [~camera_frame_id] not found, using default: " << camera_frame_id_);
      bool stream_cloud, stream_images;
      nh_private_.param("stream_cloud", stream_cloud, false);
      if (!nh_private_.hasParam("stream_cloud"))
        ROS_WARN_STREAM("Parameter [~stream_cloud] not found, using default: " << std::boolalpha << stream_cloud);
      nh_private_.param("stream_images", stream_images, true);
      if (!nh_private_.hasParam("stream_images"))
        ROS_WARN_STREAM("Parameter [~stream_images] not found, using default: " << std::boolalpha << stream_images);
      // Advertise topics
      image_transport::ImageTransport it(nh_);
      l_raw_pub_ = it.advertiseCamera("left/image_raw", 2);
      r_raw_pub_ = it.advertiseCamera("right/image_raw", 2);
      l_rectified_pub_ = it.advertise("left/image_rect", 2);
      r_rectified_pub_ = it.advertise("right/image_rect", 2);
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2 >("depth/points", 2, true); // Latched
      linfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("left/camera_info", 2, true);
      rinfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("right/camera_info", 2, true);
      // Initialize Ensenso
      ensenso_ptr_.reset(new pcl::EnsensoGrabber);
      ensenso_ptr_->openDevice(serial);
      ensenso_ptr_->openTcpPort();
      // Configure and start streaming
      configureStreaming(stream_cloud, stream_images);
      ensenso_ptr_->start();
      // Start dynamic reconfigure server
      dynamic_reconfigure::Server<ensenso::CaptureParametersConfig>::CallbackType f;
      f = boost::bind(&EnsensoNode::CaptureParametersCallback, this, _1, _2);
      reconfigure_server_.setCallback(f);
    }
    
    ~EnsensoNode()
    {
      ensenso_ptr_->closeTcpPort();
      ensenso_ptr_->closeDevice();
    }
    
    void CaptureParametersCallback(ensenso::CaptureParametersConfig &config, uint32_t level)
    {
      std::string trigger_mode;
      switch (config.TriggerMode)
      {
        case 0:
          trigger_mode = "Software";
          break;
        case 1:
          trigger_mode = "FallingEdge";
          break;
        case 2:
          trigger_mode = "RisingEdge";
          break;
        default:
          trigger_mode = "Software";
      }
      ROS_DEBUG("---");
      ROS_DEBUG("Reconfigure Request");
      ROS_DEBUG_STREAM("AutoBlackLevel: "   << std::boolalpha << config.AutoBlackLevel);
      ROS_DEBUG_STREAM("AutoExposure: "     << std::boolalpha << config.AutoExposure);
      ROS_DEBUG_STREAM("AutoGain: "         << std::boolalpha << config.AutoGain);
      ROS_DEBUG_STREAM("Binning: "          << config.Binning);
      ROS_DEBUG_STREAM("BlackLevelOffset: " << config.BlackLevelOffset);
      ROS_DEBUG_STREAM("Exposure: "         << config.Exposure);
      ROS_DEBUG_STREAM("FlexView: "         << std::boolalpha << config.FlexView);
      ROS_DEBUG_STREAM("FlexViewImages: "   << config.FlexViewImages);
      ROS_DEBUG_STREAM("FrontLight: "       << std::boolalpha << config.FrontLight);
      ROS_DEBUG_STREAM("Gain: "             << config.Gain);
      ROS_DEBUG_STREAM("GainBoost: "        << std::boolalpha << config.GainBoost);
      ROS_DEBUG_STREAM("HardwareGamma: "    << std::boolalpha << config.HardwareGamma);
      ROS_DEBUG_STREAM("Hdr: "              << std::boolalpha << config.Hdr);
      ROS_DEBUG_STREAM("PixelClock: "       << config.PixelClock);
      ROS_DEBUG_STREAM("Projector: "        << std::boolalpha << config.Projector);
      ROS_DEBUG_STREAM("TargetBrightness: " << config.TargetBrightness);
      ROS_DEBUG_STREAM("TriggerMode: "      << trigger_mode);
      ROS_DEBUG_STREAM("DisparityMapAOI: "  << std::boolalpha << config.DisparityMapAOI);
      ROS_DEBUG("---");
      ensenso_ptr_->setAutoBlackLevel(config.AutoBlackLevel);
      ensenso_ptr_->setAutoExposure(config.AutoExposure);
      ensenso_ptr_->setAutoGain(config.AutoGain);
      ensenso_ptr_->setBlackLevelOffset(config.BlackLevelOffset);
      ensenso_ptr_->setExposure(config.Exposure);
      ensenso_ptr_->setFrontLight(config.FrontLight);
      ensenso_ptr_->setGain(config.Gain);
      ensenso_ptr_->setGainBoost(config.GainBoost);
      ensenso_ptr_->setHardwareGamma(config.HardwareGamma);
      ensenso_ptr_->setHdr(config.Hdr);
      ensenso_ptr_->setPixelClock(config.PixelClock);
      ensenso_ptr_->setProjector(config.Projector);
      ensenso_ptr_->setTargetBrightness(config.TargetBrightness);
      ensenso_ptr_->setTriggerMode(trigger_mode);
      ensenso_ptr_->setUseDisparityMapAreaOfInterest(config.DisparityMapAOI);
      // Flexview and binning only work in 'Software' trigger mode and with the projector on
      if (trigger_mode.compare("Software") == 0 && config.Projector)
      {
        ensenso_ptr_->setBinning(config.Binning);
        ensenso_ptr_->setFlexView(config.FlexView, config.FlexViewImages);
      }
    }
    
    bool configureStreaming(const bool cloud, const bool images=true)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      // Disconnect previous connection
      connection_.disconnect();
      // Connect new signals
      if (cloud && images)
      {
        boost::function<void(
          const boost::shared_ptr<PointCloudXYZ>&, 
          const boost::shared_ptr<PairOfImages>&,
          const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&EnsensoNode::grabberCallback, this, _1, _2, _3);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      else if (images)
      {
        boost::function<void(
          const boost::shared_ptr<PairOfImages>&,
          const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&EnsensoNode::grabberCallback, this, _1, _2);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      else if (cloud)
      {
        boost::function<void(
            const boost::shared_ptr<PointCloudXYZ>&)> f = boost::bind (&EnsensoNode::grabberCallback, this, _1);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }
    
    void grabberCallback( const boost::shared_ptr<PointCloudXYZ>& cloud)
    {
      // Point cloud
      cloud->header.frame_id = camera_frame_id_;
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud, cloud_msg);
      cloud_pub_.publish(cloud_msg);
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
  ros::init (argc, argv, "ensenso_node");
  EnsensoNode ensenso_node;
  ros::spin();
  ros::shutdown();
  return 0;
}
