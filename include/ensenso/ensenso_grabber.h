#ifndef __PCL_IO_ENSENSO_GRABBER__
#define __PCL_IO_ENSENSO_GRABBER__

#include <pcl/pcl_config.h>
#include <pcl/common/io.h>
#include <pcl/common/time.h>
#include <pcl/io/eigen.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <pcl/io/boost.h>
#include <boost/thread.hpp>

#include <pcl/io/grabber.h>
#include <pcl/common/synchronizer.h>

#include <camera_info_manager/camera_info_manager.h>

#include <nxLib.h> // Ensenso SDK

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

    /** @brief Searches for available devices
     * @returns The number of Ensenso devices connected */
    int enumDevices () const;

    /** @brief Opens an Ensenso device
     * @param[in] device The device ID to open
     * @return True if successful, false otherwise */
    bool openDevice (std::string serial_no);

    /** @brief Closes the Ensenso device
     * @return True if successful, false otherwise */
    bool closeDevice ();

    /** @brief Start the point cloud and or image acquisition
     * @note Opens device "0" if no device is open */
    void start ();

    /** @brief Stop the data acquisition */
    void stop ();

    /** @brief Check if the data acquisition is still running
     * @return True if running, false otherwise */
    bool isRunning () const;

    /** @brief Check if a TCP port is opened
     * @return True if open, false otherwise */
    bool isTcpPortOpen () const;

    /** @brief Get class name
     * @returns A string containing the class name */
    std::string getName () const;

    /** @brief Configure Ensenso capture settings
     * @param[in] auto_exposure If set to yes, the exposure parameter will be ignored
     * @param[in] auto_gain If set to yes, the gain parameter will be ignored
     * @param[in] bining Pixel bining: 1, 2 or 4
     * @param[in] exposure In milliseconds, from 0.01 to 20 ms
     * @param[in] front_light Infrared front light (useful for calibration)
     * @param[in] gain Float between 1 and 4
     * @param[in] gain_boost
     * @param[in] hardware_gamma
     * @param[in] hdr High Dynamic Range (check compatibility with other options in Ensenso manual)
     * @param[in] pixel_clock In MegaHertz, from 5 to 85
     * @param[in] projector Use the central infrared projector or not
     * @param[in] target_brightness Between 40 and 210
     * @param[in] trigger_mode
     * @param[in] use_disparity_map_area_of_interest
     * @return True if successful, false otherwise
     * @note See [Capture tree item](http://www.ensenso.com/manual/index.html?_cameras_byserialno_$stereoserial_parameters_capture.htm) for more
     * details about the parameters. */
    bool
    configureCapture (const bool auto_exposure = true,
                      const bool auto_gain = true,
                      const int bining = 1,
                      const float exposure = 0.32,
                      const bool front_light = false,
                      const int gain = 1,
                      const bool gain_boost = false,
                      const bool hardware_gamma = false,
                      const bool hdr = false,
                      const int pixel_clock = 10,
                      const bool projector = true,
                      const int target_brightness = 80,
                      const std::string trigger_mode = "Software",
                      const bool use_disparity_map_area_of_interest = false) const;

    /** @brief Capture a single point cloud and store it
     * @param[out] cloud The cloud to be filled
     * @return True if successful, false otherwise
     * @warning A device must be opened and not running */
    bool
    grabSingleCloud (pcl::PointCloud<pcl::PointXYZ> &cloud);

    /** @brief Obtain the number of frames per second (FPS) */
    float
    getFramesPerSecond () const;

    /** @brief Open TCP port to enable access via the [nxTreeEdit](http://www.ensenso.de/manual/software_components.htm) program.
     * @param[in] port The port number
     * @return True if successful, false otherwise */
    bool
    openTcpPort (const int port = 24000);

    /** @brief Close TCP port program
     * @return True if successful, false otherwise
     * @warning If you do not close the TCP port the program might exit with the port still open, if it is the case
     * use @code ps -ef @endcode and @code kill PID @endcode to kill the application and effectively close the port. */
    bool
    closeTcpPort (void);

    /** @brief Get meta information for a camera.
     * @param[in] cam A string containing the camera (Left or Right)
     * @param[out] cam_info meta information for a camera.
     * @return True if successful, false otherwise
     * @note See: [sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
     */
    bool getCameraInfo(std::string cam, sensor_msgs::CameraInfo &cam_info) const;
    
    /** @brief Reference to the NxLib tree root
     * @warning You must handle NxLib exceptions manually when playing with @ref root_ !
     * See ensensoExceptionHandling in ensenso_grabber.cpp */
    boost::shared_ptr<const NxLibItem> root_;

    /** @brief Reference to the camera tree
     *  @warning You must handle NxLib exceptions manually when playing with @ref camera_ ! */
    NxLibItem camera_;

protected:
    /** @brief Grabber thread */
    boost::thread grabber_thread_;
    
    /** @brief Boost point cloud signal */
    boost::signals2::signal<sig_cb_ensenso_point_cloud>* point_cloud_signal_;
    
    /** @brief Boost images signal */
    boost::signals2::signal<sig_cb_ensenso_images>* images_signal_;
    
    /** @brief Boost images + point cloud signal */
    boost::signals2::signal<sig_cb_ensenso_point_cloud_images>* point_cloud_images_signal_;
    
    /** @brief Whether an Ensenso device is opened or not */
    bool device_open_;
    
    /** @brief Whether an TCP port is opened or not */
    bool tcp_open_;
    
    /** @brief Whether an Ensenso device is running or not */
    bool running_;
    
    /** @brief Mutual exclusion for FPS computation */
    mutable boost::mutex fps_mutex_;
    
    /** @brief Convert an Ensenso time stamp into a PCL/ROS time stamp
     * @param[in] ensenso_stamp
     * @return PCL stamp
     * The Ensenso API returns the time elapsed from January 1st, 1601 (UTC); on Linux OS the reference time is January 1st, 1970 (UTC).
     * See [time-stamp page](http://www.ensenso.de/manual/index.html?json_types.htm) for more info about the time stamp conversion. */
    pcl::uint64_t
    static
    getPCLStamp (const double ensenso_stamp);

    /** @brief Get OpenCV image type corresponding to the parameters given
     * @param channels number of channels in the image
     * @param bpe bytes per element
     * @param isFlt is float
     * @return the OpenCV type as a string */
    std::string
    static
    getOpenCVType (const int channels,
                   const int bpe,
                   const bool isFlt);

    /** @brief Continuously asks for images and or point clouds data from the device and publishes them if available.
     * PCL time stamps are filled for both the images and clouds grabbed (see @ref getPCLStamp)
     * @note The cloud time stamp is the RAW image time stamp */
    void
    processGrabbing ();
};
}  // namespace pcl

#endif // __PCL_IO_ENSENSO_GRABBER__

