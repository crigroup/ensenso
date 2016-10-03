#ifndef __ENSENSO_ENSENSO_GRABBER__
#define __ENSENSO_ENSENSO_GRABBER__

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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
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
    
    /** @brief With this command you can calibrate the position of the camera with respect to a robot. 
    * The calibration routine currently supports two types of setups: either your camera is fixed with 
    * respect to the robot origin, or your camera mounted on the robot hand and is moving with the robot.
    * @param[in] robot_poses A list of robot poses, 1 for each pattern acquired (in the same order)
    * @param[in] camera_seed Initial guess of the camera pose. The pose must be given relative to the 
    * robot hand (for a moving camera setup), or relative to the robot origin (for the fixed camera setup).
    * @param[in] pattern_seed Initial guess of the pattern pose. This pose must be given relative to the 
    * robot hand (for a fixed camera setup), or relative to the robot origin (for the moving camera setup).
    * @param[in] setup Moving or Fixed, please refer to the Ensenso documentation
    * @param[out] estimated_camera_pose The Transformation between this camera's left eye coordinates and 
    * the next linked system.
    * @param[out] estimated_pattern_pose The estimated pattern pose. This pose is either relative to the 
    * robot hand (for a fixed camera setup), or relative to the robot origin (for the moving camera setup).
    * @param[out] iterations Indicates the number of optimization iterations performed on the final solution 
    * until it had converged.
    * @param[out] reprojection_error The reprojection error per pattern point over all collected patterns of 
    * the final solution.
    * @return True if successful, false otherwise
    * @warning This can take up to 120 seconds */
    bool calibrateHandEye ( const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &robot_poses,
                            const Eigen::Affine3d &camera_seed,
                            const Eigen::Affine3d &pattern_seed,
                            const std::string setup,
                            Eigen::Affine3d &estimated_camera_pose,
                            Eigen::Affine3d &estimated_pattern_pose,
                            int &iterations,
                            double &reprojection_error) const;
    
    /** @brief Closes the Ensenso device
     * @return True if successful, false otherwise */
    bool closeDevice ();
    
    /** @brief Close TCP port program
     * @return True if successful, false otherwise
     * @warning If you do not close the TCP port the program might exit with the port still open, if it is the case
     * use @code ps -ef @endcode and @code kill PID @endcode to kill the application and effectively close the port. */
    bool closeTcpPort (void);
    
    /** @brief Collects a calibration pattern
     * @param[in] buffer Specifies whether the pattern should be added to the pattern buffer.
     * @return the number of calibration patterns stored in the @code nxTree @endcode , -1 on error
     * @warning A device must be opened and must not be running.*/
    int collectPattern (const bool buffer=true) const;
    
    /** @brief Decodes the pattern grid size and thickness
     * @return the grid size in mm*/
    double decodePattern () const;
    
    /** @brief Clears the pattern buffers of monocular and stereo pattern observations. 
     * @return True if successful, false otherwise
     * @note The counters PatternCount and MonocularPatternCount will be zero after clearing.*/
    bool discardPatterns () const;
    
    /** @brief Searches for available devices
     * @returns The number of Ensenso devices connected */
    int enumDevices () const;
    
    /** @brief Estimate the calibration pattern pose
     * @param[out] pose the calibration pattern pose
     * @param[in] average Specifies if all pattern point coordinates in the buffer 
     * should be averaged to produce a more precise pose measurement. This will only 
     * produce a correct result if all patterns in the buffer originate from 
     * multiple images of the same pattern in the same pose.
     * @return true if successful, false otherwise
     * @warning A device must be opened and must not be running.
     * @note At least one calibration pattern must have been collected before, use collectPattern() before */
    bool estimatePatternPose (Eigen::Affine3d &pose, const bool average=false) const;
    
    /** @brief Get class name
     * @returns A string containing the class name */
    std::string getName () const;
    
    /** @brief Get meta information for a monocular camera.
     * @param[in] cam A string containing the camera (Left or Right)
     * @param[out] cam_info meta information for a camera.
     * @return True if successful, false otherwise
     * @note See: [sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
     */
    bool getCameraInfo(std::string cam, sensor_msgs::CameraInfo &cam_info) const;
    
    /** @brief Get the raw stereo pattern information and the pattern pose. Before using it enable the
     * storeCalibrationPattern.
     * @param[out] grid_size The number of points on the patterns grid as two element vector.
     * @param[out] grid_spacing Distance of two neighboring grid points along this pattern's x or y axis.
     * @param[out] left_points the raw image positions of the pattern dots in the first camera.
     * @param[out] right_points the raw image dot positions in the second camera. 
     * @param[out] pose the calibration pattern pose.
     * @return True if successful, false otherwise */
    bool getLastCalibrationPattern (std::vector<int> &grid_size, double &grid_spacing,
                                    std::vector<Eigen::Vector2d> &left_points,
                                    std::vector<Eigen::Vector2d> &right_points,
                                    Eigen::Affine3d &pose) const;
    
    /** @brief Obtain the number of frames per second (FPS) */
    float getFramesPerSecond () const;
    
    /** @brief Gets the number of collected patterns with successful observations in two cameras.
     * @returns The number of pattern in the camera buffer */
    int getPatternCount () const;
    
    /** @brief Capture a single point cloud and store it
     * @param[out] cloud The cloud to be filled
     * @return True if successful, false otherwise
     * @warning A device must be opened and not running */
    bool grabSingleCloud (pcl::PointCloud<pcl::PointXYZ> &cloud);
    
    /** @brief Check if the data acquisition is still running
     * @return True if running, false otherwise */
    bool isRunning () const;

    /** @brief Check if a TCP port is opened
     * @return True if open, false otherwise */
    bool isTcpPortOpen () const;
    
    /** @brief Opens an Ensenso device
     * @param[in] serial The camera serial
     * @return True if successful, false otherwise */
    bool openDevice (std::string serial);
    
    /** @brief Open TCP port to enable access via the 
     * [nxTreeEdit](http://www.ensenso.de/manual/software_components.htm) program.
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
     * the image transfer times. Changing this node's value directly reduces the resolution of all binary image 
     * nodes accordingly.
     * @param[in] binning A positive integer specifying the binning factor.
     * @return True if successful, false otherwise
     * @note Changing the binning factor cancels any running capture operation and clears all images for the 
     * corresponding camera. */
    bool setBinning (const int binning=1) const;
    
    /** @brief The current black level offset. When AutoBlackLevel is false this value specifies the sensor black level 
     * directly, otherwise the offset is applied on top of the automatically estimated sensor black level.
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
    
    /** @brief The number of image pairs to capture. When FlexView is set to false the camera will operate in normal 
     * one-shot stereo mode. 
     * If FlexView is enabled the Capture command will automatically capture the indicated number of image pairs and 
     * shift the projectors pattern between each exposure. All image pairs will then be used to compute depth data in 
     * ComputeDisparityMap. Using more than one 
     * image pair will increase the effective X, Y and Z resolution.
     * @param[in] enable Specify false to disable the FlexView function.
     * @param[in] imagepairs A value in the range 2..8 specifying the number of image pairs used for depth computation.
     * @return True if successful, false otherwise
     * @note This parameter is only present if your camera supports FlexView (all N35 camera models).
     * @note FlexView is currently only supported in software triggered operation.
     * @note Depth computation from more than one image pair will only yield accurate depth data on scene parts which 
     * remained static in all image pairs.*/
    bool setFlexView (const bool enable=false, const int imagepairs=2) const;
    
    /** @brief Enables the diffuse front light during exposure. This should only be used when calibrating or tracking 
     * a calibration pattern. 
     * Please also note the illumination limitations.
     * @param[in] enable When set to true the camera's front LED will be switched on for the duration of the 
     * image exposure.
     * @return True if successful, false otherwise */
    bool setFrontLight (const bool enable=false) const;
    
    /** @brief The current analog gain factor. See also MaxGain.
     * @param[in] gain A value in the range 1..MaxGain specifying the camera's analog gain factor.
     * E.g. setting a value of 2.0 
     * will double the brightness values.
     * @return True if successful, false otherwise */
    bool setGain (const float gain=1.0) const;
    
    /** @brief Enables the cameras analog gain boost function.
     * @param[in] enable When set to true an additional analog gain boost on the camera will be enabled.
     * @return True if successful, false otherwise */
    bool setGainBoost (const bool enable=false) const;
    
    /** @brief Sets the grid spacing of the calibration pattern
     * @param[in] grid_spacing distance of two neighboring grid points along the pattern's x or y axis.
     * @return True if successful, false otherwise */
    bool setGridSpacing (const double grid_spacing) const;
    
    /** @brief Enables the camera's internal analog gamma correction. This boosts dark pixels while compressing 
     * higher brightness values.
     * @param[in] enable When set to true the cameras analog gamma correction will be enabled.
     * @return True if successful, false otherwise */
    bool setHardwareGamma (const bool enable=true) const;
    
    /** @brief Enables the camera's high dynamic range function with a fixed, piece-wise linear response curve.
     * @param[in] enable When set to true the HDR function of the camera will be enabled.
     * @return True if successful, false otherwise
     * @note The response curve set by the HDR feature can currently not be modified. */
    bool setHdr (const bool enable=false) const;
    
    /** @brief The minimum disparity in pixels where correspondences in the stereo image pair are being searched. 
     * The resolution reductions by Scaling and Binning are automatically accounted for. The actual value used 
     * in the matching process is output in ScaledMinimumDisparity.
     * @param[in] disparity An integer specifying the minimum disparity in pixels where the stereo matching algorithm 
     * searches for correspondences between the two images.
     * @return True if successful, false otherwise */
    bool setMinimumDisparity (const int disparity=-64) const;
    
    /** @brief The number of disparities in pixels where correspondences in the stereo image pair are being searched, 
     * starting at MinDisparity. The resolution reductions by Scaling and Binning are automatically accounted for. 
     * The actual value used in the matching process is output in ScaledNumberOfDisparities.
     * @param[in] number An integer specifying the number of disparities in pixels where the images are being matched.
     * @return True if successful, false otherwise 
     * @note Note: The NumberOfDisparities parameter must be a multiple of 16.*/
    bool setNumberOfDisparities (const int number=128) const;
    
    /** @brief The type of Semi-Global-Matching optimization carried out on the cost function.
     * @param[in] profile Three possible types are accepted:
     *  - "Aligned": Propagate cost along 4 paths, corresonding to the pixel axes of the rectified images.
     *  - "Diagonal": Propagate cost on the 4 paths, corresponding the all 45 degree pixel diagonals.
     *  - "AlignedAndDiagonal": Propagate along all 8 paths, aligned and diagonal. This setting yields the 
     *    best matching results, 
     * but slowest performance.
     * @return True if successful, false otherwise 
     * @note The Aligned and Diagonal profiles have similar runtime, but object edges that are approximately 
     * aligned with one of the propagation directions might be estimated less accurately. You might for example 
     * choose the Diagonal profile, if you expect you object edges to be mostly pixel axis aligned and Aligned 
     * for best results on non-pixel aligned object boundaries.*/
    bool setOptimizationProfile (const std::string profile="AlignedAndDiagonal") const;
    
    /** @brief Sets the pixel clock in MHz. If you have too many devices on the same bus the image transfer might 
     * fail when the clock is too high. This happens when the host PC does not request data from the camera fast enough. 
     * The sensor then outputs data faster than it can be transferred to the host and the cameras buffer will overflow. 
     * Thus the image transfer is incomplete and the image is lost.
     * @param[in] pixel_clock An integer number specifying the cameras pixel clock in MHz. Range: [7-43]
     * @return True if successful, false otherwise */
    bool setPixelClock (const int pixel_clock=24) const;
    
    /** @brief Enables the texture projector during exposure. This should only be used for depth map computation. 
     * Please also note the illumination limitations.
     * @param[in] enable When set to true the camera's pattern projector will be switched on for the duration of the 
     * image exposure.
     * @return True if successful, false otherwise */
    bool setProjector (const bool enable=true) const;
    
    /** @brief Scaling allows to reduce the camera resolution by an arbitrary non-integer factor during rectification. 
     * The camera raw images stay at their original size, but the rectified images, DisparityMap and PointMap will be 
     * scaled by the specified factor to improve stereo matching runtime. This allows you to choose you own tradeoff 
     * between image resolution and performance.
     * @param[in] scaling An positive real number between 0.25 and 1.0.
     * @return True if successful, false otherwise 
     * @note Setting a new Scaling factor immediately clears and resizes the affected image nodes.
     * @note As Scaling only affects the rectified images you might set a new Scaling factor and rerun ComputeDisparityMap 
     * without capturing a new image pair! You could therefore use Scaling for fast object detection in low resolution, 
     * and then perform measurements in higher resolution by setting Scaling to 1 without the need to capture an 
     * additional image pair.*/
    bool setScaling (const float scaling=1.0) const;
    
    /** @brief The desired average image brightness in gray values used for AutoExposure and AutoGain.
     * @param[in] target Positive number from 40 to 210, specifying the desired average gray value of both images.
     * @return True if successful, false otherwise */
    bool setTargetBrightness (const int target=80) const;
    
    /** @brief Specifies how an image capture is initiated.
     * @param[in] mode Three possible mode are accepted:
     *  - "Software": The camera starts the exposure by software trigger when the Capture command is issued.
     *  - "FallingEdge": The Capture command waits for a high-to-low transition on the trigger input before 
     *    starting the exposure.
     *  - "RisingEdge": The Capture command waits for a low-to-high transition on the trigger input before 
     *    starting the exposure.
     * @return True if successful, false otherwise 
     * @note Triggering on the rising edge is currently not supported by the N10 cameras due 
     * to hardware limitations. */
    bool setTriggerMode (const std::string mode="Software") const;
    
    /** @brief Reduces the camera's capture AOI to the region necessary for the currently set stereo matching AOI. 
     * This will reduce the image transfer time, especially when setting smaller AOIs for the stereo matching.
     * @param[in] enable When set to true the camera's capture AOI will be reduced.
     * @return True if successful, false otherwise 
     * @note On N20, N30 and N35 cameras the AOI will only reduce the number of lines transferred in the raw images. 
     * Each line will still contain valid pixels for the full sensor width.
     * @note This will also slightly improve transfer times when the stereo matching AOI is set to full size, because 
     * it crops image portions that will be thrown away during image rectification.
     * @note Beware that you cannot enlarge the stereo matching AOI when you captured an image with 
     * UseDisparityMapAreaOfInterest set to true, because the camera images contain no data outside the previously 
     * specified AOI. You need to capture another image after enlarging the stereo matching AOI in order to get valid 
     * depth data in the enlarged regions.*/
    bool setUseDisparityMapAreaOfInterest (const bool enable=false) const;
    
    /** @brief The penalty for changes of +/- 1 disparity along an optimization path.
     * Setting a larger value for DepthChangeCost will result in smoother surfaces, but some details might get lost 
     * when setting this value too large.
     * @param[in] changecost A positive integer specifying the cost of disparity changes in the disparity map.
     * @note his value must be smaller than DepthStepCost. Default Value 5 */
    bool setDepthChangeCost(const int changecost) const;

    /** @brief The penalty for steps (changes of more than one disparity) along an optimization path.
     * Setting a larger value for DepthStepCost will yield better detection of planar surfaces in low contrast areas, 
     * but too large values will lead to a loss of geometry details and precise object boundaries.
     * @param[in] stepcost A positive integer, strictly larger than DepthChangeCost, specifying the cost of disparity 
     * steps (discontinuities) in the disparity map.
     * @note This value must be larger than DepthChangeCost. Default Value 30
     * @return True if successful, false otherwise */
    bool setDepthStepCost(const int stepcost) const;

    /** @brief The disparity map is checked for occluded pixels. This is usually called 'left-right consistency check'. 
     * A pixel is only accepted if it is a mutually best match with the assigned right image pixel. Due to subpixel 
     * interpolation and half-occluded pixels, it is reasonable to allow small deviations from 'exact mutual' matches. 
     * This threshold sets the allowed range of mismatch in pixels.
     * @param[in] shadowingthreshold An integer specifying the threshold in disparities by which a pixel might be occluded
     * by another pixel to still be accepted as valid. Negative values disable the occlusion detection and will leave 
     * wrongly associated regions in occluded image areas.
     * @return True if successful, false otherwise
     * @note Setting a negative value (e.g. -1) for this parameter will disable filtering of shadowed areas. 
     * This will leave arbitrary depth values in shadowed areas. Default Value 1 */
    bool setShadowingThreshold(const int shadowingthreshold) const;

    /** @brief Filters the pixels depending on the uniqueness of the found correspondence. The value indicates the 
     * percentage, by which the cost of the next best correspondence must be larger (compared to the best correspondence), 
     * such that the pixel is accepted.
     * @param[in] ratio An integer specifying the uniqueness margin in percent.
     * @note  Setting this parameter to 0 disables the uniqueness filter.
     * @return True if successful, false otherwise */
    bool setUniquenessRatio(const int ratio) const;

    /** @brief Specifies the size of the median filter as radius in pixels, excluding the center pixel. The filter is 
     * applied to the disparity map. Median filtering will reduce noise inside surfaces while maintaining sharp edges, 
     * but object corners will be rounded.
     * @param[in] radius An integer specifying half the median filter window size in pixels, excluding the center pixel. 
     * Allowed values are 0 to 2.
     * @note Setting the filter radius to 0 will disable median filtering.
     * @return True if successful, false otherwise */
    bool setMedianFilterRadius(const int radius) const;

    /** @brief Defines how the image is divided into regions for speckle filtering. Whenever two neighboring pixel 
     * disparities differ by more than ComponentThreshold disparities, the two pixels are considered as belonging 
     * to separate regions. Consequently, each resulting region will not have discontinuities larger or equal to 
     * ComponentThreshold in it's disparity map area.
     * @param[in] threshold An integer specifying the disparity step size, where surfaces should be cut into 
     * separate speckle regions.
     * @note  The smaller this threshold is set, the smaller the resulting disparity regions will be. Thus setting 
     * a smaller ComponentThreshold will result in more regions being filtered out, because some regions fall apart 
     * and their sizes drop below RegionSize.
     * @return True if successful, false otherwise */
    bool setSpeckleComponentThreshold(const int threshold) const;

    /** @brief The size in pixels of a disparity map region below which the region will be removed from the disparity 
     * map. The computation of the regions is controlled by ComponentThreshold.
     * @param[in] threshold An integer specifying the size in pixels below which a region will be removed from the 
     * disparity map.
     * @note  Setting this parameter to 0 disables the speckle filter.
     * @return True if successful, false otherwise */
    bool setSpeckleRegionSize(const int threshold) const;

    /** @brief Defines which missing regions will be filled by setting a threshold on the maximum spread of the 
     * disparities on the region boundary. Setting this value reasonably small will ensure that only missing patches 
     * inside planar faces will be filled whereas gaps at depth discontinuities are kept unfilled.
     * @param[in] maximumspread An integer specifying the maximum spread of the disparities at the fill region border.
     * @return True if successful, false otherwise*/
    bool setFillBorderSpread(const int maximumspread) const;

    /** @brief Defines an upper limit on the region size in pixels, up to which a region is accepted for filling. 
     * The region must also satisfy the BorderSpread condition to be filled.
     * @param[in] regionsize An integer specifying region size in pixels, up to which a missing region is being filled.
     * @note Setting this parameter to 0 disables the hole filling filter.
     * @return True if successful, false otherwise */
    bool setFillRegionSize(const int regionsize) const;
    
    /** @brief Start the point cloud and or image acquisition
     * @note Opens device "0" if no device is open */
    void start ();

    /** @brief Stop the data acquisition */
    void stop ();
    
    /** @brief Enables collecting the calibration pattern continuous during processGrabbing()
     * @param[in] enable When set to true the calibration pattern raw and pose will be collected. */
    void storeCalibrationPattern (const bool enable);

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
    
    /** @brief Last pose of the detected pattern during \ref processGrabbing() */
    Eigen::Affine3d last_pattern_pose_;
    
    /** @brief Last raw stereo info of the detected pattern during \ref processGrabbing() */
    std::string last_stereo_pattern_;
    
    /** @brief Whether to read the pattern pose at \ref processGrabbing() or not */
    bool store_calibration_pattern_;
    
    /** @brief Mutual exclusion for reading pattern pose */
    mutable boost::mutex pattern_mutex_;
    
    /** @brief Whether an Ensenso device is running or not */
    bool running_;
    
    /** @brief Camera frames per second (FPS) */
    float fps_;
    
    /** @brief Mutual exclusion for FPS computation */
    mutable boost::mutex fps_mutex_;
    
    /** @brief Convert an Ensenso time stamp into a PCL/ROS time stamp
     * @param[in] ensenso_stamp
     * @return PCL stamp
     * The Ensenso API returns the time elapsed from January 1st, 1601 (UTC); on Linux OS the 
     * reference time is January 1st, 1970 (UTC).
     * See [time-stamp page](http://www.ensenso.de/manual/index.html?json_types.htm) for more info about 
     * the time stamp conversion. */
    pcl::uint64_t static getPCLStamp (const double ensenso_stamp);

    /** @brief Get OpenCV image type corresponding to the parameters given
     * @param channels number of channels in the image
     * @param bpe bytes per element
     * @param isFlt is float
     * @return the OpenCV type as a string */
    std::string static getOpenCVType (const int channels, const int bpe, const bool isFlt);
    
    /** @brief Converts a JSON string into an Eigen::Affine3d
     * @param[in] json The input JSON transformation
     * @param[out] matrix An Eigen matrix containing the resulting transformation
     * @return True if successful, false otherwise
     */
    bool jsonToMatrix (const std::string json, Eigen::Affine3d &matrix) const;
    
    /** @brief Converts an Eigen::Affine3d into a JSON string transformation (4x4) 
     * @param[in] matrix An Eigen matrix
     * @param[out] json The output JSON transformation
     * @param[in] pretty_format JSON formatting style
     * @return True if successful, false otherwise
     */
    bool matrixToJson (const Eigen::Affine3d &matrix, std::string &json, const bool pretty_format=true) const;
    
    /** @brief Continuously asks for images and or point clouds data from the device and publishes them if available.
     * PCL time stamps are filled for both the images and clouds grabbed (see @ref getPCLStamp)
     * @note The cloud time stamp is the RAW image time stamp */
    void processGrabbing ();
};
}  // namespace pcl

#endif // __PCL_IO_ENSENSO_GRABBER__

