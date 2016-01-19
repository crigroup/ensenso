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
#include "ensenso_grabber.h"


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

      if( performCalibration(10,12.5))
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
    
    bool performCalibration(int number_of_poses, float grid_spacing)
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
      
      // Move the robot to the initial position???
      ////////////////////////////////////////////
      
      
      // Capture calibration data from the sensor, move the robot and repeat until enough data is acquired
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > robot_poses;
      while (ros::ok() && robot_poses.size() < number_of_poses)
      {
        ensenso_ptr_->start();
        
        // move the robot to the next random position??
        //////////////////////////////////////////////
        
        std::cout << robot_poses.size() << " of " << number_of_poses << " data acquired\n";
        ensenso_ptr_->stop();
        sleep(1); // Sleep time: the robot might oscillate little bit after moving
        if (ensenso_ptr_->captureCalibrationPattern() == -1)
        {
          ROS_WARN_STREAM("Failed to capture calibration pattern: skipping to next pose");
          continue;
        }

        // Collect robot pose
        srv.request.pattern_pose = 1;
        if (move_client_.call(srv))
          ROS_INFO("Coresponding robot pose acquired!");
        else
          ROS_ERROR("Failed to call service add_two_ints");
        Eigen::Affine3d robot_pose;
        tf::poseMsgToEigen(srv.response.pose, robot_pose);
        robot_poses.push_back(robot_pose);
      }

      sleep(1);
      // Move the robot back to initial pose (if possible) (it's DONE its job!)
      ////////////////////////////////////////////////////////////////////////
      
      // Compute calibration matrix
      // TODO: Add guess calibration support
      PCL_INFO("Computing calibration matrix...\n");
      std::string result;

      if (!ensenso_ptr_->computeCalibrationMatrix(robot_poses, result, "Moving", "Hand"))
      {
        PCL_ERROR("Failed to compute calibration!\n");
        return false;
      }
      PCL_INFO("Calibration computation successful!\n");
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
