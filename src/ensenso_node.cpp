// ROS headers
#include <ros/ros.h>
#include <ros/service.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

// Conversions
#include <eigen_conversions/eigen_msg.h>

// Image transport
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL headers
#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>

// Ensenso grabber
#include <ensenso/ensenso_grabber.h>
// Services
#include <ensenso/Lights.h>
#include <ensenso/CapturePattern.h>
#include <ensenso/ComputeCalibration.h>
#include <ensenso/ConfigureStreaming.h>
#include <ensenso/GridSpacing.h>
#include <ensenso/SetBool.h>
#include <std_srvs/Trigger.h>


// Typedefs
typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;


class EnsensoNode
{
  private:
    // ROS
    ros::NodeHandle                   nh_, nh_private_;
    ros::ServiceServer                calibrate_srv_;
    ros::ServiceServer                capture_srv_;
    ros::ServiceServer                grid_spacing_srv_;
    ros::ServiceServer                init_cal_srv_;
    ros::ServiceServer                ligths_srv_;
    ros::ServiceServer                start_srv_;
    ros::ServiceServer                configure_srv_;
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
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2 >("depth/points", 2, true); // Latched
      linfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("left/camera_info", 2, true);
      rinfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("right/camera_info", 2, true);
      // Initialize Ensenso
      ensenso_ptr_.reset(new pcl::EnsensoGrabber);
      ensenso_ptr_->openDevice(serial);
      ensenso_ptr_->openTcpPort();
      ensenso_ptr_->configureCapture();
      ensenso_ptr_->enableProjector(projector);
      ensenso_ptr_->enableFrontLight(front_light);
      // Start ensenso grabber
      ensenso::ConfigureStreaming::Request req;
      ensenso::ConfigureStreaming::Response res;
      req.cloud = point_cloud_;
      req.images = true;
      configureStreamingCB(req, res);
      ensenso_ptr_->start();
      // Advertise services
      capture_srv_ = nh_.advertiseService("capture_pattern", &EnsensoNode::capturePatternCB, this);
      calibrate_srv_ = nh_.advertiseService("compute_calibration", &EnsensoNode::computeCalibrationCB, this);
      configure_srv_ = nh_.advertiseService("configure_streaming", &EnsensoNode::configureStreamingCB, this);
      grid_spacing_srv_ = nh_.advertiseService("grid_spacing", &EnsensoNode::gridSpacingCB, this);
      init_cal_srv_ = nh_.advertiseService("init_calibration", &EnsensoNode::initCalibrationCB, this);
      ligths_srv_ = nh_.advertiseService("lights", &EnsensoNode::ligthsCB, this);
      start_srv_ = nh_.advertiseService("start_streaming", &EnsensoNode::startStreamingCB, this);
    }
    
    ~EnsensoNode()
    {
      ensenso_ptr_->closeTcpPort();
      ensenso_ptr_->closeDevice();
    }
    
    bool ligthsCB(ensenso::Lights::Request& req, ensenso::Lights::Response &res)
    {
      ensenso_ptr_->enableProjector(req.projector);
      ensenso_ptr_->enableFrontLight(req.front_light);
      res.success = true;
      return true;
    }
    
    bool capturePatternCB(ensenso::CapturePattern::Request& req, ensenso::CapturePattern::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      if (req.clear_buffer)
      {
        if (!ensenso_ptr_->clearCalibrationPatternBuffer ())
          return true;
      }
      res.pattern_count = ensenso_ptr_->captureCalibrationPattern();
      res.success = (res.pattern_count != -1);
      if (res.success)
      {
        // Pattern pose
        Eigen::Affine3d pattern_pose;
        ensenso_ptr_->estimateCalibrationPatternPose(pattern_pose);
        tf::poseEigenToMsg(pattern_pose, res.pose);
      }
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }
    
    bool computeCalibrationCB(ensenso::ComputeCalibration::Request& req, ensenso::ComputeCalibration::Response &res)
    {
      // Very important to stop the camera before performing the calibration
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > poses;
      for (size_t i = 0; i < req.robotposes.poses.size(); i++) {
        Eigen::Affine3d pose;
        tf::poseMsgToEigen(req.robotposes.poses[i], pose);
        poses.push_back(pose);
        ROS_INFO_STREAM(pose.matrix());
      }
      Eigen::Affine3d seed;
      tf::poseMsgToEigen(req.seed, seed);
      std::string result;
      double error;
      int iters;
      if (!ensenso_ptr_->computeCalibrationMatrix(poses, result, iters, error, "Moving", "Hand", seed))
        res.success = false;
      else {
        ROS_INFO("Calibration computation finished");
        // Populate the response
        res.success = true;
        Eigen::Affine3d eigen_result;
        ensenso_ptr_->jsonTransformationToMatrix(result, eigen_result);
        eigen_result.translation () /= 1000.0;  // Convert translation to meters (Ensenso API returns milimeters)
        tf::poseEigenToMsg(eigen_result, res.result);
        res.reprojection_error = error;
        res.iterations = iters;
        if (req.store_to_eeprom)
        {
          if (!ensenso_ptr_->clearEEPROMExtrinsicCalibration())
            ROS_WARN("Could not reset extrinsic calibration");
          ensenso_ptr_->storeEEPROMExtrinsicCalibration();
          ROS_INFO("Calibration stored into the EEPROM");
        }
      }
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }
    
    bool configureStreamingCB(ensenso::ConfigureStreaming::Request& req, ensenso::ConfigureStreaming::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      // Disconnect previous connection
      connection_.disconnect();
      // Connect new signals
      if (req.cloud && req.images)
      {
        boost::function<void(
          const boost::shared_ptr<PointCloudXYZ>&, 
          const boost::shared_ptr<PairOfImages>&,
          const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&EnsensoNode::grabberCallback, this, _1, _2, _3);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      else if (req.images)
      {
        boost::function<void(
          const boost::shared_ptr<PairOfImages>&,
          const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&EnsensoNode::grabberCallback, this, _1, _2);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      else if (req.cloud)
      {
        boost::function<void(
            const boost::shared_ptr<PointCloudXYZ>&)> f = boost::bind (&EnsensoNode::grabberCallback, this, _1);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      if (was_running)
        ensenso_ptr_->start();
      res.success = true;
      return true;
    }
    
    bool gridSpacingCB (ensenso::GridSpacing::Request& req, ensenso::GridSpacing::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      res.grid_spacing = ensenso_ptr_->getPatternGridSpacing();
      res.success = (res.grid_spacing > 0);
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }
    
    bool initCalibrationCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      double grid_spacing = ensenso_ptr_->getPatternGridSpacing();
      if (grid_spacing > 0) {
        ensenso_ptr_->initExtrinsicCalibration(grid_spacing);
        res.success = true;
      }
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }
    
    bool startStreamingCB(ensenso::SetBool::Request& req, ensenso::SetBool::Response &res)
    {
      if (req.data)
        ensenso_ptr_->start();
      else
        ensenso_ptr_->stop();
      res.success = true;
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
  ros::init (argc, argv, "ensenso");
  EnsensoNode ensenso_node;
  ros::spin();
  ros::shutdown();
  return 0;
}
