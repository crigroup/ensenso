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

// PCL headers
#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>
#include "ensenso/ensenso_grabber.h"


// Typedefs
typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;


class HandeyeCalibration
{
  private:
    // Ros
    ros::NodeHandle           nh_, nh_private_;
    ros::Publisher            l_raw_pub_;
    ros::Publisher            r_raw_pub_;
    ros::ServiceClient        move_client_;
    // Ensenso grabber
    pcl::EnsensoGrabber::Ptr  ensenso_ptr_;
    
  public:
     HandeyeCalibration(): 
      nh_private_("~")
    { 
      // Initialize Ensenso
      ensenso_ptr_.reset(new pcl::EnsensoGrabber);
      ensenso_ptr_->openDevice("150534");
      ensenso_ptr_->openTcpPort();
      ensenso_ptr_->configureCapture(true, true, 1, 0.32, true, 1, false, false, false, 10, false);
      // Setup image publishers
      l_raw_pub_ = nh_.advertise<sensor_msgs::Image>("left/image_raw", 2);
      r_raw_pub_ = nh_.advertise<sensor_msgs::Image>("right/image_raw", 2);

      // Start ensenso grabber
      boost::function<void
      (const boost::shared_ptr<PointCloudXYZ>&,
       const boost::shared_ptr<PairOfImages>&,const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&HandeyeCalibration::grabberCallback, this, _1, _2, _3);
      ensenso_ptr_->registerCallback(f);
      ensenso_ptr_->start();
      
      // Initialize calibration
      float grid_spacing = 12.5;
      int num_pose = 10;
      Eigen::Matrix4d pattern_pose;
      pattern_pose << 0,0,1,-1,
                      0,1,0,0,
                      -1,0,0,0.8,
                      0,0,0,1;
      Eigen::Affine3d est_pattern_pose;
      est_pattern_pose= pattern_pose; //Update this later
      
      if( performCalibration(num_pose,grid_spacing,est_pattern_pose))
        ROS_INFO("DONE CALIBRATION");
      else
        ROS_ERROR("FAIL TO CALIBRATE!");
    }
    
    ~HandeyeCalibration()
    {
      ensenso_ptr_->closeTcpPort();
      ensenso_ptr_->closeDevice();
    }
    
    void grabberCallback( const boost::shared_ptr<PointCloudXYZ>& cloud,
                      const boost::shared_ptr<PairOfImages>& rawimages,  const boost::shared_ptr<PairOfImages>& rectifiedimages)
    {
      // Images
      unsigned char *l_raw_image_array = reinterpret_cast<unsigned char *>(&rawimages->first.data[0]);
      unsigned char *r_raw_image_array = reinterpret_cast<unsigned char *>(&rawimages->second.data[0]);
      int type(CV_8UC1);
      std::string encoding("mono8");
      if (rawimages->first.encoding == "CV_8UC3")
      {
        type = CV_8UC3;
        encoding = "bgr8";
      }
      cv::Mat l_raw_image(rawimages->first.height, rawimages->first.width, type, l_raw_image_array);
      cv::Mat r_raw_image(rawimages->first.height, rawimages->first.width, type, r_raw_image_array);
      std_msgs::Header header;
      header.frame_id = "world";
      header.stamp = ros::Time::now();
      l_raw_pub_.publish(cv_bridge::CvImage(header, encoding, l_raw_image).toImageMsg());
      r_raw_pub_.publish(cv_bridge::CvImage(header, encoding, r_raw_image).toImageMsg());
    }
    
    bool performCalibration(int num_pose, float grid_spacing, Eigen::Affine3d est_pattern_pose)
    {
      // Setup Ensenso
      ensenso_ptr_->stop();
      ensenso_ptr_->clearCalibrationPatternBuffer(); // In case a previous calibration was launched!
      ensenso_ptr_->initExtrinsicCalibration(grid_spacing);
      ensenso_ptr_->start();
      
      // Setup services
      std::string srv_name = "calibration_move_random";
      move_client_ = nh_.serviceClient<ensenso::CalibrationMoveRandom>(srv_name.c_str());
      // Wait for move robot service
      ros::service::waitForService(srv_name.c_str());
      ensenso::CalibrationMoveRandom srv;
      // Convert req format to msg
      tf::poseEigenToMsg(est_pattern_pose, srv.request.patternpose);
      srv.request.minradius = 0.450;
      // Capture calibration data from the sensor, move the robot and repeat until enough data is acquired
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > robot_poses;
      srv.request.gotoinitpose = false;
      while (ros::ok() && robot_poses.size() < num_pose)
      {
        ensenso_ptr_->start();
        // Move the robot to a random position (still be able to capture the pattern)
        if (move_client_.call(srv))
        {
          if (!srv.response.success)
          {
            ROS_ERROR("Random_move: failed to plan the robot!");
            return false;
          }
          // Collect pattern pose
          ensenso_ptr_->stop();
          if (ensenso_ptr_->captureCalibrationPattern() == -1)
          {
            ROS_WARN_STREAM("Failed to capture calibration pattern: skipping to next pose");
            continue;
          }
          ROS_INFO("Pattern pose acquired!");
          // Collect robot pose
          Eigen::Affine3d robot_pose;
          tf::poseMsgToEigen(srv.response.robotpose, robot_pose);
          robot_poses.push_back(robot_pose);
          ROS_INFO("Robot pose acquired!");
          std::cout << robot_poses.size() << " of " << num_pose << " data acquired\n";  
        }
        else
          ROS_ERROR("Failed to call service");
      }
      sleep(1);
      //~ // Move the robot back to initial pose
      //~ srv.request.gotoinitpose = true;
      //~ if (move_client_.call(srv))
        //~ {
          //~ ROS_INFO("Moved robot back to init position");
        //~ }
      //~ else ROS_ERROR("Failed to call service");
      
      
      // Compute calibration matrix
      // TODO: Add guess calibration support
      ROS_INFO("Computing calibration matrix...");
      std::string result;

      if (!ensenso_ptr_->computeCalibrationMatrix(robot_poses, result, "Moving", "Hand"))
      {
        ROS_ERROR("Failed to compute calibration!");
        return false;
      }
      ROS_INFO("Calibration computation successful!");
      std::cout << "Result: " << result <<"\n";
      // Save the calibration result as another format?
      ///////////////////////////////////////////////
      return true;
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
