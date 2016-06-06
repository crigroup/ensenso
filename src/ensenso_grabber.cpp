#include <pcl/pcl_config.h>
#include <pcl/exceptions.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <boost/format.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include "ensenso/ensenso_grabber.h"


void ensensoExceptionHandling (const NxLibException &ex,
                 std::string func_nam)
{
  PCL_ERROR ("%s: NxLib error %s (%d) occurred while accessing item %s.\n", func_nam.c_str (), ex.getErrorText ().c_str (), ex.getErrorCode (),
         ex.getItemPath ().c_str ());
  if (ex.getErrorCode () == NxLibExecutionFailed)
  {
    NxLibCommand cmd ("");
    PCL_WARN ("\n%s\n", cmd.result ().asJson (true, 4, false).c_str ());
  }
}


pcl::EnsensoGrabber::EnsensoGrabber () :
  device_open_ (false),
  tcp_open_ (false),
  running_ (false)
{
  point_cloud_signal_ = createSignal<sig_cb_ensenso_point_cloud> ();
  images_signal_ = createSignal<sig_cb_ensenso_images> ();
  point_cloud_images_signal_ = createSignal<sig_cb_ensenso_point_cloud_images> ();
  PCL_INFO ("Initialising nxLib\n");
  try
  {
    nxLibInitialize ();
    root_.reset (new NxLibItem);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "EnsensoGrabber");
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not initialise NxLib.");  // If constructor fails; throw exception
  }
}

pcl::EnsensoGrabber::~EnsensoGrabber () throw ()
{
  try
  {
    stop ();
    root_.reset ();

    disconnect_all_slots<sig_cb_ensenso_point_cloud> ();
    disconnect_all_slots<sig_cb_ensenso_images> ();
    disconnect_all_slots<sig_cb_ensenso_point_cloud_images> ();

    if (tcp_open_)
      closeTcpPort ();
    nxLibFinalize ();
  }
  catch (...)
  {
    // destructor never throws
  }
}

int pcl::EnsensoGrabber::enumDevices () const
{
  int camera_count = 0;
  try
  {
    NxLibItem cams = NxLibItem ("/Cameras/BySerialNo");
    camera_count = cams.count ();
    // Print information for all cameras in the tree
    PCL_INFO ("Number of connected cameras: %d\n", camera_count);
    PCL_INFO ("Serial No    Model   Status\n");
    for (int n = 0; n < cams.count (); ++n)
    {
      PCL_INFO ("%s   %s   %s\n", cams[n][itmSerialNumber].asString ().c_str (),
            cams[n][itmModelName].asString ().c_str (),
            cams[n][itmStatus].asString ().c_str ());
    }
    PCL_INFO ("\n");
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "enumDevices");
  }
  return (camera_count);
}

bool pcl::EnsensoGrabber::openDevice (std::string serial_no)
{
  if (device_open_)
    PCL_THROW_EXCEPTION (pcl::IOException, "Cannot open multiple devices!");
  PCL_INFO ("Opening Ensenso stereo camera S/N: %s\n", serial_no.c_str());
  try
  {
    // Create a pointer referencing the camera's tree item, for easier access:
    camera_ = (*root_)[itmCameras][itmBySerialNo][serial_no];
    if (!camera_.exists () || camera_[itmType] != valStereo)
      PCL_THROW_EXCEPTION (pcl::IOException, "Please connect a single stereo camera to your computer!");
    NxLibCommand open (cmdOpen);
    open.parameters ()[itmCameras] = camera_[itmSerialNumber].asString ();
    open.execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "openDevice");
    return (false);
  }
  device_open_ = true;
  return (true);
}

bool pcl::EnsensoGrabber::closeDevice ()
{
  if (!device_open_)
    return (false);

  stop ();
  PCL_INFO ("Closing Ensenso stereo camera\n");

  try
  {
    NxLibCommand (cmdClose).execute ();
    device_open_ = false;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "closeDevice");
    return (false);
  }
  return (true);
}

void pcl::EnsensoGrabber::start ()
{
  if (isRunning ())
    return;
  if (!device_open_)
    openDevice (0);
  running_ = true;
  grabber_thread_ = boost::thread (&pcl::EnsensoGrabber::processGrabbing, this);
}

void pcl::EnsensoGrabber::stop ()
{
  if (running_)
  {
    running_ = false;  // Stop processGrabbing () callback
    grabber_thread_.join ();
  }
}

bool pcl::EnsensoGrabber::isRunning () const
{
  return (running_);
}

bool pcl::EnsensoGrabber::isTcpPortOpen () const
{
  return (tcp_open_);
}

std::string pcl::EnsensoGrabber::getName () const
{
  return ("EnsensoGrabber");
}

bool pcl::EnsensoGrabber::configureCapture (const bool auto_exposure,
                      const bool auto_gain,
                      const int bining,
                      const float exposure,
                      const bool front_light,
                      const int gain,
                      const bool gain_boost,
                      const bool hardware_gamma,
                      const bool hdr,
                      const int pixel_clock,
                      const bool projector,
                      const int target_brightness,
                      const std::string trigger_mode,
                      const bool use_disparity_map_area_of_interest) const
{
  if (!device_open_)
    return (false);

  try
  {
    NxLibItem captureParams = camera_[itmParameters][itmCapture];
    captureParams[itmAutoExposure].set (auto_exposure);
    captureParams[itmAutoGain].set (auto_gain);
    captureParams[itmBinning].set (bining);
    captureParams[itmExposure].set (exposure);
    captureParams[itmFrontLight].set (front_light);
    captureParams[itmGain].set (gain);
    captureParams[itmGainBoost].set (gain_boost);
    captureParams[itmHardwareGamma].set (hardware_gamma);
    captureParams[itmHdr].set (hdr);
    captureParams[itmPixelClock].set (pixel_clock);
    captureParams[itmProjector].set (projector);
    captureParams[itmTargetBrightness].set (target_brightness);
    captureParams[itmTriggerMode].set (trigger_mode);
    captureParams[itmUseDisparityMapAreaOfInterest].set (use_disparity_map_area_of_interest);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "configureCapture");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::grabSingleCloud (pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);

  try
  {
    NxLibCommand (cmdCapture).execute ();
    // Stereo matching task
    NxLibCommand (cmdComputeDisparityMap).execute ();
    // Convert disparity map into XYZ data for each pixel
    NxLibCommand (cmdComputePointMap).execute ();
    // Get info about the computed point map and copy it into a std::vector
    double timestamp;
    std::vector<float> pointMap;
    int width, height;
    camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);  // Get raw image timestamp
    camera_[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
    camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);
    // Copy point cloud and convert in meters
    cloud.header.stamp = getPCLStamp (timestamp);
    cloud.resize (height * width);
    cloud.width = width;
    cloud.height = height;
    cloud.is_dense = false;
    // Copy data in point cloud (and convert milimeters in meters)
    for (size_t i = 0; i < pointMap.size (); i += 3)
    {
      cloud.points[i / 3].x = pointMap[i] / 1000.0;
      cloud.points[i / 3].y = pointMap[i + 1] / 1000.0;
      cloud.points[i / 3].z = pointMap[i + 2] / 1000.0;
    }
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "grabSingleCloud");
    return (false);
  }
}

float pcl::EnsensoGrabber::getFramesPerSecond () const
{
  boost::mutex::scoped_lock lock (fps_mutex_);
  return (0.0);
}

bool pcl::EnsensoGrabber::openTcpPort (const int port)
{
  try
  {
    nxLibOpenTcpPort (port);
    tcp_open_ = true;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "openTcpPort");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::closeTcpPort ()
{
  try
  {
    nxLibCloseTcpPort ();
    tcp_open_ = false;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "closeTcpPort");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::getCameraInfo(std::string cam, sensor_msgs::CameraInfo &cam_info) const
{
  try
  {
    cam_info.width = camera_[itmSensor][itmSize][0].asInt();
    cam_info.height = camera_[itmSensor][itmSize][1].asInt();
    cam_info.distortion_model = "plumb_bob";
    // Distorsion factors
    cam_info.D.resize(5);
    for(std::size_t i = 0; i < cam_info.D.size(); ++i)
      cam_info.D[i] = camera_[itmCalibration][itmMonocular][cam][itmDistortion][i].asDouble();
    // K and R matrices
    for(std::size_t i = 0; i < 3; ++i)
    {
      for(std::size_t j = 0; j < 3; ++j)
      {
        cam_info.K[3*i+j] = camera_[itmCalibration][itmMonocular][cam][itmCamera][j][i].asDouble();
        cam_info.R[3*i+j] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmRotation][j][i].asDouble();
      }
    }
    cam_info.P[0] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][0][0].asDouble();
    cam_info.P[1] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][1][0].asDouble();
    cam_info.P[2] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][2][0].asDouble();
    cam_info.P[3] = 0.0;
    cam_info.P[4] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][0][1].asDouble();
    cam_info.P[5] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][1][1].asDouble();
    cam_info.P[6] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][2][1].asDouble();
    cam_info.P[7] = 0.0;
    cam_info.P[10] = 1.0;
    if (cam == "Right")
    {
      double B = camera_[itmCalibration][itmStereo][itmBaseline].asDouble() / 1000.0;
      double fx = cam_info.P[0];
      cam_info.P[3] = (-fx * B);
    }
    return true;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getCameraInfo");
    return false;
  }
}

pcl::uint64_t pcl::EnsensoGrabber::getPCLStamp (const double ensenso_stamp)
{
#if defined _WIN32 || defined _WIN64
  return (ensenso_stamp * 1000000.0);
#else
  return ( (ensenso_stamp - 11644473600.0) * 1000000.0);
#endif
}

std::string pcl::EnsensoGrabber::getOpenCVType (const int channels,
                        const int bpe,
                        const bool isFlt)
{
  int bits = bpe * 8;
  char type = isFlt ? 'F' : (bpe > 3 ? 'S' : 'U');
  return (boost::str (boost::format ("CV_%i%cC%i") % bits % type % channels));
}

void pcl::EnsensoGrabber::processGrabbing ()
{
  bool continue_grabbing = running_;
  while (continue_grabbing)
  {
    try
    {
      // Publish cloud / images
      if (num_slots<sig_cb_ensenso_point_cloud> () > 0 || num_slots<sig_cb_ensenso_images> () > 0 || num_slots<sig_cb_ensenso_point_cloud_images> () > 0)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        boost::shared_ptr<PairOfImages> rawimages (new PairOfImages);
        boost::shared_ptr<PairOfImages> rectifiedimages (new PairOfImages);
        fps_mutex_.lock ();
        //~ frequency_.event ();
        fps_mutex_.unlock ();
        
        NxLibCommand (cmdCapture).execute ();
        double timestamp;
        camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);

        // Gather images
        if (num_slots<sig_cb_ensenso_images> () > 0 || num_slots<sig_cb_ensenso_point_cloud_images> () > 0)
        {
          // Rectify images
          NxLibCommand (cmdRectifyImages).execute ();
          int width, height, channels, bpe;
          bool isFlt, collected_pattern = false;
          
          try  // Try to collect calibration pattern, if not possible, publish RAW images and Rectified images instead
          {
            NxLibCommand collect_pattern (cmdCollectPattern);
            collect_pattern.parameters ()[itmBuffer].set (false);  // Do NOT store the pattern into the buffer!
            collect_pattern.execute ();
            collected_pattern = true;
          }
          catch (const NxLibException &ex)
          {
            // We failed to collect the pattern but the RAW images are available!
          }

          if (collected_pattern)
          {
            camera_[itmImages][itmWithOverlay][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            rawimages->first.header.stamp = rawimages->second.header.stamp = getPCLStamp (timestamp);
            rawimages->first.width = rawimages->second.width = width;
            rawimages->first.height = rawimages->second.height = height;
            rawimages->first.data.resize (width * height * sizeof(float));
            rawimages->second.data.resize (width * height * sizeof(float));
            rawimages->first.encoding = rawimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
            camera_[itmImages][itmWithOverlay][itmLeft].getBinaryData (rawimages->first.data.data (), rawimages->first.data.size (), 0, 0);
            camera_[itmImages][itmWithOverlay][itmRight].getBinaryData (rawimages->second.data.data (), rawimages->second.data.size (), 0, 0);
            // rectifiedimages
            camera_[itmImages][itmRectified][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            rectifiedimages->first.width = rectifiedimages->second.width = width;
            rectifiedimages->first.height = rectifiedimages->second.height = height;
            rectifiedimages->first.data.resize (width * height * sizeof(float));
            rectifiedimages->second.data.resize (width * height * sizeof(float));
            rectifiedimages->first.encoding = rectifiedimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
            camera_[itmImages][itmRectified][itmLeft].getBinaryData (rectifiedimages->first.data.data (), rectifiedimages->first.data.size (), 0, 0);
            camera_[itmImages][itmRectified][itmRight].getBinaryData (rectifiedimages->second.data.data (), rectifiedimages->second.data.size (), 0, 0);
          }
          else
          {
            camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            rawimages->first.header.stamp = rawimages->second.header.stamp = getPCLStamp (timestamp);
            rawimages->first.width = rawimages->second.width = width;
            rawimages->first.height = rawimages->second.height = height;
            rawimages->first.data.resize (width * height * sizeof(float));
            rawimages->second.data.resize (width * height * sizeof(float));
            rawimages->first.encoding = rawimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
            camera_[itmImages][itmRaw][itmLeft].getBinaryData (rawimages->first.data.data (), rawimages->first.data.size (), 0, 0);
            camera_[itmImages][itmRaw][itmRight].getBinaryData (rawimages->second.data.data (), rawimages->second.data.size (), 0, 0);
            // rectifiedimages
            camera_[itmImages][itmRectified][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            rectifiedimages->first.width = rectifiedimages->second.width = width;
            rectifiedimages->first.height = rectifiedimages->second.height = height;
            rectifiedimages->first.data.resize (width * height * sizeof(float));
            rectifiedimages->second.data.resize (width * height * sizeof(float));
            rectifiedimages->first.encoding = rectifiedimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
            camera_[itmImages][itmRectified][itmLeft].getBinaryData (rectifiedimages->first.data.data (), rectifiedimages->first.data.size (), 0, 0);
            camera_[itmImages][itmRectified][itmRight].getBinaryData (rectifiedimages->second.data.data (), rectifiedimages->second.data.size (), 0, 0);
          }
        }

        // Gather point cloud
        if (num_slots<sig_cb_ensenso_point_cloud> () > 0 || num_slots<sig_cb_ensenso_point_cloud_images> () > 0)
        {
          // Stereo matching task
          NxLibCommand (cmdComputeDisparityMap).execute ();
          // Convert disparity map into XYZ data for each pixel
          NxLibCommand (cmdComputePointMap).execute ();
          // Get info about the computed point map and copy it into a std::vector
          std::vector<float> pointMap;
          int width, height;
          camera_[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
          camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);
          // Copy point cloud and convert in meters
          cloud->header.stamp = getPCLStamp (timestamp);
          cloud->points.resize (height * width);
          cloud->width = width;
          cloud->height = height;
          cloud->is_dense = false;
          // Copy data in point cloud (and convert milimeters in meters)
          for (size_t i = 0; i < pointMap.size (); i += 3)
          {
            cloud->points[i / 3].x = pointMap[i] / 1000.0;
            cloud->points[i / 3].y = pointMap[i + 1] / 1000.0;
            cloud->points[i / 3].z = pointMap[i + 2] / 1000.0;
          }
        }
        // Publish signals
        if (num_slots<sig_cb_ensenso_point_cloud_images> () > 0)
          point_cloud_images_signal_->operator () (cloud, rawimages, rectifiedimages);
        else if (num_slots<sig_cb_ensenso_point_cloud> () > 0)
          point_cloud_signal_->operator () (cloud);
        else if (num_slots<sig_cb_ensenso_images> () > 0)
          images_signal_->operator () (rawimages,rectifiedimages);
      }
      continue_grabbing = running_;
    }
    catch (NxLibException &ex)
    {
      ensensoExceptionHandling (ex, "processGrabbing");
    }
  }
}
