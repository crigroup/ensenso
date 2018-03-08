// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
// Conversions
#include <eigen_conversions/eigen_msg.h>
// Ensenso grabber
#include <ensenso/ensenso_grabber.h>
// Image transport
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ensenso/CameraParametersConfig.h>
// Messages
#include <ensenso/RawStereoPattern.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
// Services
#include <ensenso/CalibrateHandEye.h>
#include <ensenso/CollectPattern.h>
#include <ensenso/EstimatePatternPose.h>


// Typedefs
typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;


class EnsensoDriver
{
  private:
    // ROS
    ros::NodeHandle                   nh_, nh_private_;
    ros::ServiceServer                pattern_srv_;
    ros::ServiceServer                collect_srv_;
    ros::ServiceServer                calibrate_srv_;
    dynamic_reconfigure::Server<ensenso::CameraParametersConfig> reconfigure_server_;
    // Images
    image_transport::CameraPublisher  l_raw_pub_;
    image_transport::CameraPublisher  r_raw_pub_;
    image_transport::Publisher        l_rectified_pub_;
    image_transport::Publisher        r_rectified_pub_;
    // Publishers
    ros::Publisher                    cloud_pub_;
    ros::Publisher                    pattern_pose_pub_;
    ros::Publisher                    pattern_raw_pub_;
    // Streaming configuration
    bool                              is_streaming_cloud_;
    bool                              is_streaming_images_;
    bool                              stream_calib_pattern_;
    int                               trigger_mode_;
    // Camera info
    ros::Publisher                    linfo_pub_;
    ros::Publisher                    rinfo_pub_;
    // TF
    std::string                       camera_frame_id_;
    // Ensenso grabber
    boost::signals2::connection       connection_;
    pcl::EnsensoGrabber::Ptr          ensenso_ptr_;

  public:
     EnsensoDriver():
      is_streaming_images_(false),
      is_streaming_cloud_(false),
      trigger_mode_(-1),
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
      nh_private_.param("stream_calib_pattern", stream_calib_pattern_, false);
      if (!nh_private_.hasParam("stream_calib_pattern"))
        ROS_WARN_STREAM("Parameter [~stream_calib_pattern] not found, using default: " << (stream_calib_pattern_ ? "TRUE":"FALSE"));
      // Advertise topics
      image_transport::ImageTransport it(nh_);
      l_raw_pub_ = it.advertiseCamera("left/image_raw", 1);
      r_raw_pub_ = it.advertiseCamera("right/image_raw", 1);
      l_rectified_pub_ = it.advertise("left/image_rect", 1);
      r_rectified_pub_ = it.advertise("right/image_rect", 1);
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2 >("depth/points", 1, false);
      linfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("left/camera_info", 1, false);
      rinfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("right/camera_info", 1, false);
      pattern_raw_pub_=nh_.advertise<ensenso::RawStereoPattern> ("pattern/stereo", 1, false);
      pattern_pose_pub_=nh_.advertise<geometry_msgs::PoseStamped> ("pattern/pose", 1, false);
      // Initialize Ensenso
      ensenso_ptr_.reset(new pcl::EnsensoGrabber);
      ensenso_ptr_->openDevice(serial);
      ensenso_ptr_->openTcpPort();
      ensenso_ptr_->storeCalibrationPattern(stream_calib_pattern_);
      // Start dynamic reconfigure server
      dynamic_reconfigure::Server<ensenso::CameraParametersConfig>::CallbackType f;
      f = boost::bind(&EnsensoDriver::CameraParametersCallback, this, _1, _2);
      reconfigure_server_.setCallback(f);
      // Start the camera.
      ensenso_ptr_->start();
      // Advertise services
      calibrate_srv_ = nh_.advertiseService("calibrate_handeye", &EnsensoDriver::calibrateHandEyeCB, this);
      pattern_srv_ = nh_.advertiseService("estimate_pattern_pose", &EnsensoDriver::estimatePatternPoseCB, this);
      collect_srv_ = nh_.advertiseService("collect_pattern", &EnsensoDriver::collectPatternCB, this);
      ROS_INFO("Finished [ensenso_driver] initialization");
    }

    ~EnsensoDriver()
    {
      connection_.disconnect();
      ensenso_ptr_->closeTcpPort();
      ensenso_ptr_->closeDevice();
    }

    bool calibrateHandEyeCB(ensenso::CalibrateHandEye::Request& req, ensenso::CalibrateHandEye::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      // Check consistency between robot and pattern poses
      if ( req.robot_poses.poses.size() != ensenso_ptr_->getPatternCount() )
      {
        ROS_WARN("The number of robot_poses differs from the pattern count in the camera buffer");
        if (was_running)
          ensenso_ptr_->start();
        return true;
      }
      // Convert poses to Eigen::Affine3d
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > robot_eigen_list;
      for (size_t i = 0; i < req.robot_poses.poses.size(); i++) {
        Eigen::Affine3d pose;
        tf::poseMsgToEigen(req.robot_poses.poses[i], pose);
        robot_eigen_list.push_back(pose);
      }
      // Calibrate
      Eigen::Affine3d camera_seed, pattern_seed, estimated_camera_pose, estimated_pattern_pose;
      tf::poseMsgToEigen(req.camera_seed, camera_seed);
      tf::poseMsgToEigen(req.pattern_seed, pattern_seed);
      ROS_INFO("calibrateHandEye: It may take up to 5 minutes...");
      res.success = ensenso_ptr_->calibrateHandEye(robot_eigen_list, camera_seed, pattern_seed,
                      req.setup, estimated_camera_pose, estimated_pattern_pose, res.iterations,
                      res.reprojection_error);
      if (res.success)
      {
        ROS_INFO("Calibration computation finished");
        tf::poseEigenToMsg(estimated_camera_pose, res.estimated_camera_pose);
        tf::poseEigenToMsg(estimated_pattern_pose, res.estimated_pattern_pose);
      }
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }

    void CameraParametersCallback(ensenso::CameraParametersConfig &config, uint32_t level)
    {
      // Process enumerators
      std::string trigger_mode, profile;
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
      switch (config.OptimizationProfile)
      {
        case 0:
          profile = "Aligned";
          break;
        case 1:
          profile = "Diagonal";
          break;
        case 2:
          profile = "AlignedAndDiagonal";
          break;
        default:
          profile = "AlignedAndDiagonal";
      }
      ROS_DEBUG("---");
      ROS_DEBUG("Capture Parameters");
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
      ROS_DEBUG("Stereo Matching Parameters");
      ROS_DEBUG_STREAM("MinimumDisparity: "     << config.MinimumDisparity);
      ROS_DEBUG_STREAM("NumberOfDisparities: "  << config.NumberOfDisparities);
      ROS_DEBUG_STREAM("OptimizationProfile: "  << profile);
      ROS_DEBUG_STREAM("Scaling: "              << config.Scaling);
      ROS_DEBUG("Advanced Matching Parameters");
      ROS_DEBUG_STREAM("DepthChangeCost: " << config.DepthChangeCost);
      ROS_DEBUG_STREAM("DepthStepCost: " << config.DepthStepCost);
      ROS_DEBUG_STREAM("ShadowingThreshold: " << config.ShadowingThreshold);
      ROS_DEBUG("Postprocessing Parameters");
      ROS_DEBUG_STREAM("UniquenessRatio: " << config.UniquenessRatio);
      ROS_DEBUG_STREAM("MedianFilterRadius: "<< config.MedianFilterRadius);
      ROS_DEBUG_STREAM("SpeckleComponentThreshold: "<< config.SpeckleComponentThreshold);
      ROS_DEBUG_STREAM("SpeckleRegionSize: "<< config.SpeckleRegionSize);
      ROS_DEBUG_STREAM("FillBorderSpread: "<< config.FillBorderSpread);
      ROS_DEBUG_STREAM("FillRegionSize: " << config.FillRegionSize);
      ROS_DEBUG("Stream Parameters");
      ROS_DEBUG_STREAM("Cloud: "   << std::boolalpha << config.Cloud);
      ROS_DEBUG_STREAM("Images: "   << std::boolalpha << config.Images);
      ROS_DEBUG("---");
      // Capture parameters
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
      // Stereo parameters
      ensenso_ptr_->setMinimumDisparity(config.MinimumDisparity);
      ensenso_ptr_->setNumberOfDisparities(config.NumberOfDisparities);
      ensenso_ptr_->setOptimizationProfile(profile);
      ensenso_ptr_->setScaling(config.Scaling);
      ensenso_ptr_->setDepthChangeCost(config.DepthChangeCost);
      ensenso_ptr_->setDepthStepCost(config.DepthStepCost);
      ensenso_ptr_->setShadowingThreshold(config.ShadowingThreshold);
      //Postprocessing parameters
      ensenso_ptr_->setUniquenessRatio(config.UniquenessRatio);
      ensenso_ptr_->setMedianFilterRadius(config.MedianFilterRadius);
      ensenso_ptr_->setSpeckleComponentThreshold(config.SpeckleComponentThreshold);
      ensenso_ptr_->setSpeckleRegionSize(config.SpeckleRegionSize);
      ensenso_ptr_->setFillBorderSpread(config.FillBorderSpread);
      ensenso_ptr_->setFillRegionSize(config.FillRegionSize);
      // Streaming parameters
      configureStreaming(config.Cloud, config.Images, config.TriggerMode);
    }

    bool collectPatternCB(ensenso::CollectPattern::Request& req, ensenso::CollectPattern::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      // Check consistency
      if (!req.decode && req.grid_spacing <= 0)
      {
        ROS_WARN("grid_spacing not specify. Forgot to set the request.decode = True?");
        if (was_running)
          ensenso_ptr_->start();
        return true;
      }
      // Discard previously saved patterns
      if (req.clear_buffer)
        ensenso_ptr_->discardPatterns();
      // Set the grid spacing
      if (req.decode)
      {
        res.grid_spacing = ensenso_ptr_->decodePattern();
        // Check consistency
        if (res.grid_spacing <= 0)
        {
          ROS_WARN("Couldn't decode calibration pattern");
          if (was_running)
            ensenso_ptr_->start();
          return true;
        }
      }
      else
        res.grid_spacing = req.grid_spacing;
      ensenso_ptr_->setGridSpacing(res.grid_spacing);
      // Collect pattern
      int prev_pattern_count = ensenso_ptr_->getPatternCount();
      res.pattern_count = ensenso_ptr_->collectPattern(req.add_to_buffer);
      res.success = (res.pattern_count == prev_pattern_count+1);
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }

    bool configureStreaming(const bool cloud, const bool images, const int trigger_mode)
    {
      if ((is_streaming_cloud_ == cloud)
              && (is_streaming_images_ == images)
              && (trigger_mode_ == trigger_mode) ) return true;  // Nothing to be done here
      is_streaming_cloud_ = cloud;
      is_streaming_images_ = images;
      trigger_mode_ = trigger_mode;
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
          const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&EnsensoDriver::grabberCallback, this, _1, _2, _3);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      else if (images)
      {
        boost::function<void(
          const boost::shared_ptr<PairOfImages>&,
          const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&EnsensoDriver::grabberCallback, this, _1, _2);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      else if (cloud)
      {
        boost::function<void(
            const boost::shared_ptr<PointCloudXYZ>&)> f = boost::bind (&EnsensoDriver::grabberCallback, this, _1);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }

    bool estimatePatternPoseCB(ensenso::EstimatePatternPose::Request& req, ensenso::EstimatePatternPose::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      res.success = ensenso_ptr_->getPatternCount() > 0;
      if (res.success)
      {
        Eigen::Affine3d pattern_pose;
        res.success = ensenso_ptr_->estimatePatternPose(pattern_pose, req.average);
        tf::poseEigenToMsg(pattern_pose, res.pose);
      }
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }

    void grabberCallback( const boost::shared_ptr<PointCloudXYZ>& cloud)
    {
      // Point cloud
      if (cloud_pub_.getNumSubscribers() > 0)
      {
        cloud->header.frame_id = camera_frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = ros::Time::now();
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_pub_.publish(cloud_msg);
      }
    }

    void grabberCallback( const boost::shared_ptr<PairOfImages>& rawimages, const boost::shared_ptr<PairOfImages>& rectifiedimages)
    {
      ros::Time now = ros::Time::now();
      // Get cameras info
      sensor_msgs::CameraInfo linfo, rinfo;
      ensenso_ptr_->getCameraInfo("Left", linfo);
      ensenso_ptr_->getCameraInfo("Right", rinfo);
      linfo.header.stamp = now;
      linfo.header.frame_id = camera_frame_id_;
      rinfo.header.stamp = now;
      rinfo.header.frame_id = camera_frame_id_;
      // Images
      if (l_raw_pub_.getNumSubscribers() > 0)
        l_raw_pub_.publish(*toImageMsg(rawimages->first, now), linfo, now);
      if (r_raw_pub_.getNumSubscribers() > 0)
        r_raw_pub_.publish(*toImageMsg(rawimages->second, now), rinfo, now);
      if (l_rectified_pub_.getNumSubscribers() > 0)
        l_rectified_pub_.publish(toImageMsg(rectifiedimages->first, now));
      if (r_rectified_pub_.getNumSubscribers() > 0)
        r_rectified_pub_.publish(toImageMsg(rectifiedimages->second, now));
      // Publish calibration pattern info (if any)
      publishCalibrationPattern(now);
    }

    void grabberCallback( const boost::shared_ptr<PointCloudXYZ>& cloud,
                          const boost::shared_ptr<PairOfImages>& rawimages, const boost::shared_ptr<PairOfImages>& rectifiedimages)
    {
      ros::Time now = ros::Time::now();
      // Get cameras info
      sensor_msgs::CameraInfo linfo, rinfo;
      ensenso_ptr_->getCameraInfo("Left", linfo);
      ensenso_ptr_->getCameraInfo("Right", rinfo);
      linfo.header.stamp = now;
      linfo.header.frame_id = camera_frame_id_;
      rinfo.header.stamp = now;
      rinfo.header.frame_id = camera_frame_id_;
      // Images
      if (l_raw_pub_.getNumSubscribers() > 0)
        l_raw_pub_.publish(*toImageMsg(rawimages->first, now), linfo, now);
      if (r_raw_pub_.getNumSubscribers() > 0)
        r_raw_pub_.publish(*toImageMsg(rawimages->second, now), rinfo, now);
      if (l_rectified_pub_.getNumSubscribers() > 0)
        l_rectified_pub_.publish(toImageMsg(rectifiedimages->first, now));
      if (r_rectified_pub_.getNumSubscribers() > 0)
        r_rectified_pub_.publish(toImageMsg(rectifiedimages->second, now));
      // Publish calibration pattern info (if any)
      publishCalibrationPattern(now);
      // Camera_info
      linfo_pub_.publish(linfo);
      rinfo_pub_.publish(rinfo);
      // Point cloud
      if (cloud_pub_.getNumSubscribers() > 0)
      {
        cloud->header.frame_id = camera_frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = now;
        cloud_pub_.publish(cloud_msg);
      }
    }

    void publishCalibrationPattern(const ros::Time &now)
    {
      int pose_subs = pattern_pose_pub_.getNumSubscribers();
      int raw_subs = pattern_raw_pub_.getNumSubscribers();
      if (stream_calib_pattern_)
      {
        if ((pose_subs <= 0) && (raw_subs <= 0))
          return;
        int num_points;
        double grid_spacing;
        Eigen::Affine3d pattern_pose;
        std::vector<int> grid_size;
        std::vector<Eigen::Vector2d> left_points, right_points;
        if ( ensenso_ptr_->getLastCalibrationPattern (grid_size, grid_spacing, left_points, right_points, pattern_pose) )
        {
          if (raw_subs > 0)
          {
            // Populate RawStereoPattern msg
            ensenso::RawStereoPattern msg;
            msg.header.frame_id = camera_frame_id_;
            msg.header.stamp = now;
            msg.grid_spacing = grid_spacing;
            msg.grid_size = grid_size;
            num_points = grid_size[0]*grid_size[1];
            msg.left_points.resize(num_points);
            msg.right_points.resize(num_points);
            for (uint i = 0; i < left_points.size(); ++i)
            {
              msg.left_points[i].x = left_points[i][0];
              msg.left_points[i].y = left_points[i][1];
              msg.right_points[i].x = left_points[i][0];
              msg.right_points[i].y = left_points[i][1];
            }
            pattern_raw_pub_.publish(msg);
          }
          if (pose_subs > 0)
          {
            // Populate PoseStamped msg
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.frame_id = camera_frame_id_;
            pose_msg.header.stamp = now;
            tf::poseEigenToMsg(pattern_pose, pose_msg.pose);
            pattern_pose_pub_.publish(pose_msg);
          }
        }
      }
    }

    sensor_msgs::ImagePtr toImageMsg(pcl::PCLImage pcl_image, const ros::Time &now)
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
      header.frame_id = camera_frame_id_;
      header.stamp = now;
      return cv_bridge::CvImage(header, encoding, image_mat).toImageMsg();
    }
};


int main(int argc, char **argv)
{
  ros::init (argc, argv, "ensenso_driver");
  EnsensoDriver driver;
  ros::spin();
  ros::shutdown();
  return 0;
}
