#ifndef __PCL_IO_ENSENSO_GRABBER__
#define __PCL_IO_ENSENSO_GRABBER__

// PCL
#include <pcl/pcl_config.h>
#include <pcl/common/io.h>
#include <pcl/common/time.h>
#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/common/synchronizer.h>
// Others
#include <boost/thread.hpp>
#include <camera_info_manager/camera_info_manager.h>
// Ensenso SDK
#include <nxLib.h>

namespace pcl
{
struct PointXYZ;
template <typename T> class PointCloud;

/**
 * @brief Grabber for IDS-Imaging Ensenso's devices
 * @author Francisco Suarez-Ruiz
 */
class PCL_EXPORTS EnsensoGrabber : public Grabber
{
    typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;

public:
    /** @cond */
    typedef boost::shared_ptr<EnsensoGrabber> Ptr;
    typedef boost::shared_ptr<const EnsensoGrabber> ConstPtr;
    
    // Define callback signature typedefs
    typedef void
    (sig_cb_ensenso_point_cloud)(const pcl::PointCloud<pcl::PointXYZ>::Ptr &);
    
    typedef void
    (sig_cb_ensenso_images)(const boost::shared_ptr<PairOfImages> &,const boost::shared_ptr<PairOfImages> &);
    
    typedef void
    (sig_cb_ensenso_point_cloud_images)(const pcl::PointCloud<pcl::PointXYZ>::Ptr &,
                                        const boost::shared_ptr<PairOfImages> &,const boost::shared_ptr<PairOfImages> &);
    /** @endcond */
    
    /** @brief Constructor */
    EnsensoGrabber ();
    
    /** @brief Destructor inherited from the Grabber interface. It never throws. */
    virtual ~EnsensoGrabber () throw ();
    
    /** @brief Closes the Ensenso device
     * @return True if successful, false otherwise */
    bool closeDevice ();
    
    /** @brief Close TCP port program
     * @return True if successful, false otherwise
     * @warning If you do not close the TCP port the program might exit with the port still open, if it is the case
     * use @code ps -ef @endcode and @code kill PID @endcode to kill the application and effectively close the port. */
    bool closeTcpPort (void);
    
    /** @brief Searches for available devices
     * @returns The number of Ensenso devices connected */
    int enumDevices () const;
    
    /** @brief Get class name
     * @returns A string containing the class name */
    std::string getName () const;
    
    /** @brief Get meta information for a camera.
     * @param[in] cam A string containing the camera (Left or Right)
     * @param[out] cam_info meta information for a camera.
     * @return True if successful, false otherwise
     * @note See: [sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
     */
    bool getCameraInfo(std::string cam, sensor_msgs::CameraInfo &cam_info) const;
    
    /** @brief Capture a single point cloud and store it
     * @param[out] cloud The cloud to be filled
     * @return True if successful, false otherwise
     * @warning A device must be opened and not running */
    bool grabSingleCloud (pcl::PointCloud<pcl::PointXYZ> &cloud);

    /** @brief Obtain the number of frames per second (FPS) */
    float getFramesPerSecond () const;
    
    /** @brief Check if the data acquisition is still running
     * @return True if running, false otherwise */
    bool isRunning () const;

    /** @brief Check if a TCP port is opened
     * @return True if open, false otherwise */
    bool isTcpPortOpen () const;
    
    /** @brief Opens an Ensenso device
     * @param[in] device The device ID to open
     * @return True if successful, false otherwise */
    bool openDevice (std::string serial);
    
    /** @brief Open TCP port to enable access via the [nxTreeEdit](http://www.ensenso.de/manual/software_components.htm) program.
     * @param[in] port The port number
     * @return True if successful, false otherwise */
    bool openTcpPort (const int port = 24000);
    
    /** @brief Restores the default capture configuration parameters.
     * @return True if successful, false otherwise */
    bool restoreDefaultConfiguration () const;
    
    /** @brief Controls whether the sensor black level should be adjusted automatically by the image sensor.
     * @param[in] enable When set to true the image sensor black level will be adjusted automatically.
     * @return True if successful, false otherwise */
    bool setAutoBlackLevel (const bool enable=true) const;
    
    /** @brief Controls whether the exposure should be adjusted after each image capture.
     * @param[in] enable When set to true the Exposure will be adjusted after each Capture command involving this camera.
     * @return True if successful, false otherwise */
    bool setAutoExposure (const bool enable=true) const;
    
    /** @brief Controls whether the gain should be adjusted after each image capture.
     * @param[in] enable When set to true the Gain will be adjusted after each Capture command involving this camera.
     * @return True if successful, false otherwise */
    bool setAutoGain (const bool enable=true) const;
    
    /** @brief Adjusts the camera's binning factor.
     * Binning reduces the image resolution by an integer factor directly on the sensor, and thus greatly reduces 
     * the image transfer times. Changing this node's value directly reduces the resolution of all binary image nodes accordingly.
     * @param[in] binning A positive integer specifying the binning factor.
     * @return True if successful, false otherwise
     * @note Changing the binning factor cancels any running capture operation and clears all images for the corresponding camera. */
    bool setBinning (const int binning=1) const;
    
    /** @brief The current black level offset. When AutoBlackLevel is false this value specifies the sensor black level directly, 
     * otherwise the offset is applied on top of the automatically estimated sensor black level.
     * @param[in] offset A number between 0.0 and 1.0. Values closer to zero will yield darker images, values closer to one
     * will increase the image brightness at the expense of noise in dark image regions.
     * @return True if successful, false otherwise */
    bool setBlackLevelOffset (const float offset=1.0) const;
    
    /** @brief The current image exposure time.
     * @param[in] exposure Specifies the camera's exposure time in milliseconds.
     * @return True if successful, false otherwise 
     * @note Have a look at the exposure limits of the LED flash by looking at the illumination topic for your camera 
     * model and the MaxFlashTime node.*/
    bool setExposure (const float exposure=1.0) const;
    
    /** @brief The number of image pairs to capture. When FlexView is set to false the camera will operate in normal one-shot stereo mode. 
     * If FlexView is enabled the Capture command will automatically capture the indicated number of image pairs and shift the projectors 
     * pattern between each exposure. All image pairs will then be used to compute depth data in ComputeDisparityMap. Using more than one 
     * image pair will increase the effective X, Y and Z resolution.
     * @param[in] enable Specify false to disable the FlexView function.
     * @param[in] imagepairs A value in the range 2..8 specifying the number of image pairs used for depth computation.
     * @return True if successful, false otherwise
     * @note This parameter is only present if your camera supports FlexView (all N35 camera models).
     * @note FlexView is currently only supported in software triggered operation.
     * @note Depth computation from more than one image pair will only yield accurate depth data on scene parts which remained static in all image pairs.*/
    bool setFlexView (const bool enable=false, const int imagepairs=2) const;
    
    /** @brief Enables the diffuse front light during exposure. This should only be used when calibrating or tracking a calibration pattern. 
     * Please also note the illumination limitations.
     * @param[in] enable When set to true the camera's front LED will be switched on for the duration of the image exposure.
     * @return True if successful, false otherwise */
    bool setFrontLight (const bool enable=false) const;
    
    /** @brief The current analog gain factor. See also MaxGain.
     * @param[in] gain A value in the range 1..MaxGain specifying the camera's analog gain factor. E.g. setting a value of 2.0 
     * will double the brightness values.
     * @return True if successful, false otherwise */
    bool setGain (const float gain=1.0) const;
    
    /** @brief Enables the cameras analog gain boost function.
     * @param[in] enable When set to true an additional analog gain boost on the camera will be enabled.
     * @return True if successful, false otherwise */
    bool setGainBoost (const bool enable=false) const;
    
    /** @brief Enables the camera's internal analog gamma correction. This boosts dark pixels while compressing higher brightness values.
     * @param[in] enable When set to true the cameras analog gamma correction will be enabled.
     * @return True if successful, false otherwise */
    bool setHardwareGamma (const bool enable=true) const;
    
    /** @brief Enables the camera's high dynamic range function with a fixed, piece-wise linear response curve.
     * @param[in] enable When set to true the HDR function of the camera will be enabled.
     * @return True if successful, false otherwise
     * @note The response curve set by the HDR feature can currently not be modified. */
    bool setHdr (const bool enable=false) const;
    
    /** @brief Sets the pixel clock in MHz. If you have too many devices on the same bus the image transfer might 
     * fail when the clock is too high. This happens when the host PC does not request data from the camera fast enough. 
     * The sensor then outputs data faster than it can be transferred to the host and the cameras buffer will overflow. 
     * Thus the image transfer is incomplete and the image is lost.
     * @param[in] pixel_clock An integer number specifying the cameras pixel clock in MHz. Range: [7-43]
     * @return True if successful, false otherwise */
    bool setPixelClock (const int pixel_clock=24) const;
    
    /** @brief Enables the texture projector during exposure. This should only be used for depth map computation. 
     * Please also note the illumination limitations.
     * @param[in] enable When set to true the camera's pattern projector will be switched on for the duration of the image exposure.
     * @return True if successful, false otherwise */
    bool setProjector (const bool enable=true) const;
    
    /** @brief The desired average image brightness in gray values used for AutoExposure and AutoGain.
     * @param[in] target Positive number from 40 to 210, specifying the desired average gray value of both images.
     * @return True if successful, false otherwise */
    bool setTargetBrightness (const int target=80) const;
    
    /** @brief Specifies how an image capture is initiated.
     * @param[in] mode Three possible mode are accepted:
     *  - "Software": The camera starts the exposure by software trigger when the Capture command is issued.
     *  - "FallingEdge": The Capture command waits for a high-to-low transition on the trigger input before starting the exposure.
     *  - "RisingEdge": The Capture command waits for a low-to-high transition on the trigger input before starting the exposure.
     * @return True if successful, false otherwise 
     * @note Triggering on the rising edge is currently not supported by the N10 cameras due to hardware limitations. */
    bool setTriggerMode (const std::string mode="Software") const;
    
    /** @brief Reduces the camera's capture AOI to the region necessary for the currently set stereo matching AOI. 
     * This will reduce the image transfer time, especially when setting smaller AOIs for the stereo matching.
     * @param[in] enable When set to true the camera's capture AOI will be reduced.
     * @return True if successful, false otherwise 
     * @note On N20, N30 and N35 cameras the AOI will only reduce the number of lines transferred in the raw images. 
     * Each line will still contain valid pixels for the full sensor width.
     * @note This will also slightly improve transfer times when the stereo matching AOI is set to full size, because 
     * it crops image portions that will be thrown away during image rectification.
     * @note Beware that you cannot enlarge the stereo matching AOI when you captured an image with UseDisparityMapAreaOfInterest set to true, 
     * because the camera images contain no data outside the previously specified AOI. You need to capture another image after enlarging the 
     * stereo matching AOI in order to get valid depth data in the enlarged regions.*/
    bool setUseDisparityMapAreaOfInterest (const bool enable=false) const;
    
    /** @brief Start the point cloud and or image acquisition
     * @note Opens device "0" if no device is open */
    void start ();

    /** @brief Stop the data acquisition */
    void stop ();

protected:
    /** @brief Grabber thread */
    boost::thread grabber_thread_;
    
    /** @brief Boost point cloud signal */
    boost::signals2::signal<sig_cb_ensenso_point_cloud>* point_cloud_signal_;
    
    /** @brief Boost images signal */
    boost::signals2::signal<sig_cb_ensenso_images>* images_signal_;
    
    /** @brief Boost images + point cloud signal */
    boost::signals2::signal<sig_cb_ensenso_point_cloud_images>* point_cloud_images_signal_;
    
    /** @brief Reference to the camera tree */
    NxLibItem camera_;
    
    /** @brief Reference to the NxLib tree root */
    boost::shared_ptr<const NxLibItem> root_;
    
    /** @brief Whether an Ensenso device is opened or not */
    bool device_open_;
    
    /** @brief Whether an TCP port is opened or not */
    bool tcp_open_;
    
    /** @brief Whether an Ensenso device is running or not */
    bool running_;
    
    /** @brief Camera frames per second (FPS) */
    float fps_;
    
    /** @brief Mutual exclusion for FPS computation */
    mutable boost::mutex fps_mutex_;
    
    /** @brief Convert an Ensenso time stamp into a PCL/ROS time stamp
     * @param[in] ensenso_stamp
     * @return PCL stamp
     * The Ensenso API returns the time elapsed from January 1st, 1601 (UTC); on Linux OS the reference time is January 1st, 1970 (UTC).
     * See [time-stamp page](http://www.ensenso.de/manual/index.html?json_types.htm) for more info about the time stamp conversion. */
    pcl::uint64_t static getPCLStamp (const double ensenso_stamp);

    /** @brief Get OpenCV image type corresponding to the parameters given
     * @param channels number of channels in the image
     * @param bpe bytes per element
     * @param isFlt is float
     * @return the OpenCV type as a string */
    std::string static getOpenCVType (const int channels, const int bpe, const bool isFlt);

    /** @brief Continuously asks for images and or point clouds data from the device and publishes them if available.
     * PCL time stamps are filled for both the images and clouds grabbed (see @ref getPCLStamp)
     * @note The cloud time stamp is the RAW image time stamp */
    void processGrabbing ();
};
}  // namespace pcl

#endif // __PCL_IO_ENSENSO_GRABBER__

