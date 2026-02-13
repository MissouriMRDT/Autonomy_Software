/******************************************************************************
 * @brief Defines constants for the autonomy software.
 *
 * @file AutonomyConstants.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-06
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "./AutonomyConstants.h"

/******************************************************************************
 * @brief Namespace containing all constants for autonomy software. Including
 *      AutonomyGlobals.h will also include this namespace.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-05
 ******************************************************************************/
namespace constants
{
    ///////////////////////////////////////////////////////////////////////////
    //// General Constants.
    ///////////////////////////////////////////////////////////////////////////

    // Sim mode constants.
#if defined(__AUTONOMY_SIM_MODE__) && __AUTONOMY_SIM_MODE__ == 1
    const bool MODE_SIM = true;    // SIM MODE ENABLED: Toggle RoveComm and Cameras to use local data from the SIM.
#else
    const bool MODE_SIM = false;    // REG MODE ENABLED: Toggle RoveComm and Cameras to use standard configuration.
#endif
    const std::string SIM_IP_ADDRESS   = "127.0.0.1";    // The IP address to use for simulation mode.
    const uint SIM_WEBSOCKET_PORT      = 8080;           // The port to use for the WebSocket in simulation mode.
    const uint SIM_WEBRTC_QP           = 25;             // The QP value to use for WebRTC in simulation mode. 0-51, 0 is lossless. If too high for network, frames drop.
    const std::string SIM_MAINCAM_NAME = "ZEDFront";     // The PixelStreaming identifier from RoveSoSimulator. This name is set internally in UE5 editor.
    const std::string SIM_REARCAM_NAME = "ZEDRear";      // The PixelStreaming identifier from RoveSoSimulator. This name is set internally in UE5 editor.

    // Safety constants.
    const double BATTERY_MINIMUM_CELL_VOLTAGE = 3.2;      // The minimum cell voltage of the battery before autonomy will forcefully enter Idle state.
    const bool BATTERY_CHECKS_ENABLED         = false;    // If autonomy should monitor PMS Currents and as a result have the ability to shutdown autonomy.

    // Logging constants.
    const std::string LOGGING_OUTPUT_PATH_ABSOLUTE = "../logs/";                  // The absolute path to write output logging and video files to.
    const quill::LogLevel CONSOLE_MIN_LEVEL        = quill::LogLevel::TraceL3;    // The minimum logging level that is allowed to send to the console log stream.
    const quill::LogLevel FILE_MIN_LEVEL           = quill::LogLevel::TraceL3;    // The minimum logging level that is allowed to send to the file log streams.
    const quill::LogLevel ROVECOMM_MIN_LEVEL       = quill::LogLevel::Info;       // The minimum logging level that is allowed to send to the RoveComm log stream.
    const quill::LogLevel CONSOLE_DEFAULT_LEVEL    = quill::LogLevel::Info;       // The default logging level for console stream.
    const quill::LogLevel FILE_DEFAULT_LEVEL       = quill::LogLevel::TraceL3;    // The default logging level for file streams.
    const quill::LogLevel ROVECOMM_DEFAULT_LEVEL   = quill::LogLevel::Info;       // The default logging level for RoveComm stream.

    // Logging color constants.
    const std::string szTraceL3Color   = "\033[30m";           // Standard Grey
    const std::string szTraceL2Color   = "\033[30m";           // Standard Grey
    const std::string szTraceL1Color   = "\033[30m";           // Standard Grey
    const std::string szDebugColor     = "\033[36m";           // Standard Cyan
    const std::string szInfoColor      = "\033[32m";           // Standard Green
    const std::string szNoticeColor    = "\033[97m\033[1m";    // Bright Bold White
    const std::string szWarningColor   = "\033[93m\033[1m";    // Bright Bold Yellow
    const std::string szErrorColor     = "\033[91m\033[1m";    // Bright Bold Red
    const std::string szCriticalColor  = "\033[95m\033[1m";    // Bright Bold Magenta
    const std::string szBacktraceColor = "\033[30m";           // Standard Grey

    // RoveComm constants.
    const int ROVECOMM_OUTGOING_UDP_PORT        = MODE_SIM ? 11001 : 11000;    // The UDP socket port to use for the main UDP RoveComm instance.
    const int ROVECOMM_OUTGOING_TCP_PORT        = MODE_SIM ? 12001 : 12000;    // The TCP socket port to use for the main TCP RoveComm instance.
    const std::string ROVECOMM_TCP_INTERFACE_IP = "127.0.0.1";    // The IP address to bind the socket to. If set to "", the socket will bind all available interfaces.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Drive Constants.
    ///////////////////////////////////////////////////////////////////////////

    // Power constants.
    // NOTE: NEVER CHANGE THESE VALUES UNLESS DRIVE BOARD HARDWARE IS CHANGED.
    const float DRIVE_MAX_POWER = 1.0;     // Internally autonomy uses -1.0 to 1.0 for drive powers. But this range should be mapped to the actual drive board max range.
    const float DRIVE_MIN_POWER = -1.0;    // Internally autonomy uses -1.0 to 1.0 for drive powers. But this range should be mapped to the actual drive board min range.
    const float DRIVE_MAX_SAFE_POWER = 0.4;    // The maximum absolute effort (0.0 to 1.0) that the drive system is allowed to use for any movement. Safety feature.

    // Control constants.
    const double DRIVE_PID_PROPORTIONAL      = 0.01;      // The proportional gain for the controller used to point the rover at a goal heading during navigation.
    const double DRIVE_PID_INTEGRAL          = 0.0003;    // The integral gain for the controller used to point the rover at a goal heading during navigation.
    const double DRIVE_PID_DERIVATIVE        = 0.008;     // The derivative gain for the controller used to point the rover at a goal heading during navigation.
    const double DRIVE_PID_FEEDFORWARD       = 0.0;       // The feedforward for the controller used to predict control output.
    const double DRIVE_PID_MAX_ERROR         = 180.0;     // The max allowable error the controller will see per iteration. This is on degrees from setpoint. 0 = Disable.
    const double DRIVE_PID_MAX_INTEGRAL_TERM = 0.1;       // The max effort the I term is allowed to contribute. 0 = Disable.
    const double DRIVE_PID_MAX_RAMP_RATE     = 0.08;      // The max ramp rate of the output of the PID controller. 0 = Disable.
    const double DRIVE_PID_OUTPUT_FILTER     = 0.0;       // Larger values will filter out large spikes or oscillations. 0.1 is a good starting point. 0 = Disable.
    const double DRIVE_PID_TOLERANCE         = 0.0;       // The max allowable error from the setpoint for the controller to be considered at the setpoint. 0 = Disable.
    const bool DRIVE_PID_OUTPUT_REVERSED     = false;     // Negates the output of the PID controller.
    const bool DRIVE_SQUARE_CONTROL_INPUTS   = false;     // This is used by the DifferentialDrive algorithms. True makes fine inputs smoother, but less responsive.
    const bool DRIVE_CURVATURE_KINEMATICS_ALLOW_TURN_WHILE_STOPPED = true;    // Whether to enable turning in-place when using curvature drive control.

    // Drive Board constants
    const float DRIVE_BOARD_MIN_SLOPE    = 3.5;     // The min slope in degrees for fTheta to start calculating multiplier. If fTheta is less: set to max multiplier.
    const float DRIVE_BOARD_MAX_SLOPE    = 30.0;    // The max slope in degrees for fTheta to stop calculating multiplier and set to min multiplier.
    const float DRIVE_BOARD_MIN_DAMP     = 0.5;     // The min multiplier used in variable drive speed applied to SetMaxDriveEffort().
    const float DRIVE_BOARD_MAX_DAMP     = 1.0;     // The max multiplier used in variable drive speed applied to SetMaxDriveEffort().
    const float DRIVE_BOARD_ROLL_WEIGHT  = 0.6;     // The weight in a percentage of importance for the roll position: 60%.
    const float DRIVE_BOARD_PITCH_WEIGHT = 0.4;     // The weight in a percentage of importance for the pitch position: 40%.
    const float DRIVE_BOARD_YAW_WEIGHT   = 0.0;     // The weight in a percentage of importance for the yaw position: 0% (added for future proofing).
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Recording Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Recording adjustments.
    const int RECORDER_FPS = 15;    // The FPS all recordings should run at.
    // Camera recording toggles.
    const bool ZED_MAINCAM_ENABLE_RECORDING = true;    // Whether or not to record the main ZED camera.
    const bool ZED_REARCAM_ENABLE_RECORDING = true;    // Whether or not to record the rear ZED camera.
    // TagDetector recording toggles.
    const bool TAGDETECT_MAINCAM_ENABLE_RECORDING = true;    // Whether or not to record the main ZED camera tag detector.
    const bool TAGDETECT_REARCAM_ENABLE_RECORDING = true;    // Whether or not to record the rear ZED camera tag detector.
    // ObjectDetector recording toggles.
    const bool OBJECTDETECT_MAINCAM_ENABLE_RECORDING = true;    // Whether or not to record the main ZED camera object detector.
    const bool OBJECTDETECT_REARCAM_ENABLE_RECORDING = true;    // Whether or not to record the rear ZED camera object detector.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Camera Constants.
    ///////////////////////////////////////////////////////////////////////////

    // ZedCam Basic Config.
    const sl::RESOLUTION ZED_BASE_RESOLUTION     = sl::RESOLUTION::HD720;                      // The base resolution to open the all cameras with.
    const sl::UNIT ZED_MEASURE_UNITS             = sl::UNIT::METER;                            // The base measurement unit to use for depth.
    const sl::COORDINATE_SYSTEM ZED_COORD_SYSTEM = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;    // Coordinate system to use for measurements.
    const sl::DEPTH_MODE ZED_DEPTH_MODE          = sl::DEPTH_MODE::NEURAL_PLUS;                // The measurement accuracy for depth. NEURAL is by far the best.
    const sl::VIEW ZED_RETRIEVE_VIEW             = sl::VIEW::LEFT;                             // The eye to retrieve regular and depth images from.
    const bool ZED_SDK_VERBOSE                   = false;                                      // Enable verbose output from the internal Camera library in the ZEDSDK.
    const bool ZED_SENSING_FILL                  = false;    // True provides a depth map with a Z value for every pixel (X, Y) in the left image. Slower and worse.
    const float ZED_DEFAULT_MINIMUM_DISTANCE     = 0.5;      // Minimum distance in ZED_MEASURE_UNITS to report from depth measurement.
    const float ZED_DEFAULT_MAXIMUM_DISTANCE     = 40.0;     // Maximum distance in ZED_MEASURE_UNITS to report from depth measurement.
    const float ZED_DEFAULT_FLOOR_PLANE_ERROR    = 0.5;      // The maximum distance that an estimated floor plane can be from the height of the camera from the ground.
    const int ZED_DEPTH_STABILIZATION            = 1;    // This parameter controls a stabilization filter that reduces oscillations in depth map. In the range [0-100]
    // ZedCam SVO Recording Config.
    const sl::SVO_COMPRESSION_MODE ZED_SVO_COMPRESSION = sl::SVO_COMPRESSION_MODE::H265;    // SVO file compression. H264/H265 minimally affect performance, but need GPU.
    const int ZED_SVO_BITRATE                          = 1000;                              // The video bitrate in kbits/s. 0 or [1000-60000]
    // ZedCam Positional Tracking Config.
    const sl::POSITIONAL_TRACKING_MODE ZED_POSETRACK_MODE = sl::POSITIONAL_TRACKING_MODE::GEN_3;    // Positional tracking accuracy.
    const bool ZED_POSETRACK_AREA_MEMORY                  = true;     // Enabled camera to remember its surroundings for better positioning. Uses more resources.
    const bool ZED_POSETRACK_POSE_SMOOTHING               = false;    // Smooth pose correction for small drift. Decreases overall precision for small movements.
    const bool ZED_POSETRACK_FLOOR_IS_ORIGIN              = true;     // Sets the floor plane as origin for tracking. This turns on floor plane detection temporarily.
    const bool ZED_POSETRACK_ENABLE_IMU_FUSION            = true;     // Allows ZED to use both optical odometry and IMU data for pose tracking.
    const float ZED_POSETRACK_USABLE_DEPTH_MIN            = 0.75;     // Minimum depth used for pose tracking, useful if a static object is partial in view of the camera.
    const bool ZED_POSETRACK_USE_GRAVITY_ORIGIN           = true;     // Override 2 of the 3 rotations from initial_world_transform using the IMU.
    // ZedCam Spatial Mapping Config.
    const sl::SpatialMappingParameters::SPATIAL_MAP_TYPE ZED_MAPPING_TYPE = sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;    // Mesh or point cloud output.
    const float ZED_MAPPING_RANGE_METER                                   = 20.0f;    // The max range in meters that the ZED cameras should use for mapping. 0 = auto.
    const float ZED_MAPPING_RESOLUTION_METER                              = 0.03f;    // The approx goal precision for spatial mapping in METERS. Higher = Faster.
    const int ZED_MAPPING_MAX_MEMORY                                      = 4096;     // The max amount of CPU RAM (MB) that can be allocated for spatial mapping.
    const bool ZED_MAPPING_USE_CHUNK_ONLY   = true;    // Only update chunks that have probably changed or have new data. Faster, less accurate.
    const int ZED_MAPPING_STABILITY_COUNTER = 3;       // Number of times that a point should be seen before adding to mesh.
    // ZedCam Object Detection Config.
    const bool ZED_OBJDETECTION_TRACK_OBJ                      = true;     // Whether or not to enable object tracking in the scene. Attempts to maintain OBJ UUIDs.
    const bool ZED_OBJDETECTION_SEGMENTATION                   = false;    // Use depth data to compute the segmentation for an object. (exact outline/shape)
    const sl::OBJECT_FILTERING_MODE ZED_OBJDETECTION_FILTERING = sl::OBJECT_FILTERING_MODE::NMS3D_PER_CLASS;    // Custom detection, use PER_CLASS or NONE.
    const float ZED_OBJDETECTION_TRACKING_PREDICTION_TIMEOUT   = 0.5;    // 0-1 second. Timeout to keep guessing object position when not in sight.
    const float ZED_OBJDETECTION_BATCH_RETENTION_TIME          = 240;    // The time in seconds to search for an object UUID before expiring the object.
    const float ZED_OBJDETECTION_BATCH_LATENCY = 2;    // Short latency will limit the search for previously seen object IDs but will be closer to real time output.
    // Zed Fusion Config.
    const sl::UNIT FUSION_MEASUREMENT_UNITS         = ZED_MEASURE_UNITS;    // The base measurement unit to use for depth and other measurements.
    const sl::COORDINATE_SYSTEM FUSION_COORD_SYSTEM = ZED_COORD_SYSTEM;     // Coordinate system to use for measurements.
    const bool FUSION_SDK_VERBOSE                   = false;                // Enable verbose output from the internal fusion library in the ZEDSDK.

    // BasicCam Basic Config.
    const cv::InterpolationFlags BASICCAM_RESIZE_INTERPOLATION_METHOD = cv::InterpolationFlags::INTER_LINEAR;    // The algorithm used to fill in pixels when resizing.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Camera Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Main ZED Camera.
    const int ZED_MAINCAM_RESOLUTIONX           = 1280;                       // The horizontal pixel resolution to resize the maincam images to.
    const int ZED_MAINCAM_RESOLUTIONY           = 720;                        // The vertical pixel resolution to resize the maincam images to.
    const int ZED_MAINCAM_FPS                   = 60;                         // The FPS to use for the maincam.
    const int ZED_MAINCAM_HORIZONTAL_FOV        = 110;                        // The horizontal FOV of the camera. Useful for future calculations.
    const int ZED_MAINCAM_VERTICAL_FOV          = 70;                         // The vertical FOV of the camera. Useful for future calculations.
    const bool ZED_MAINCAM_EXPORT_SVO_RECORDING = true;                       // Whether or not to record the leftcam to an SVO file.
    const bool ZED_MAINCAM_EXPORT_SPATIAL_MAP   = false;                      // Whether or not to export the spatial map to a file.
    const bool ZED_MAINCAM_USE_GPU_MAT          = MODE_SIM ? false : true;    // Whether or not to use CPU or GPU memory mats. GPU memory transfer/operations are faster.
    const bool ZED_MAINCAM_USE_HALF_PRECISION_DEPTH = true;                   // Whether of not to use float32 or unsigned short (16) for depth measure.
    const bool ZED_MAINCAM_FUSION_MASTER            = false;       // Whether or not this camera will host the master instance of the ZEDSDK Fusion capabilities.
    const int ZED_MAINCAM_FRAME_RETRIEVAL_THREADS   = 10;          // The number of threads allocated to the threadpool for performing frame copies to other threads.
    const int ZED_MAINCAM_SERIAL                    = 31237348;    // The serial number of the camera. Set to 0 to open the next available one. DEFAULT = 31237348

    // Rear ZED Camera.
    const bool MODE_REAR_ZED                    = true;                       // Whether or not to utilize the rear ZED.
    const int ZED_REARCAM_RESOLUTIONX           = 1280;                       // The horizontal pixel resolution to resize the rearcam images to.
    const int ZED_REARCAM_RESOLUTIONY           = 720;                        // The vertical pixel resolution to resize the rearcam images to.
    const int ZED_REARCAM_FPS                   = 60;                         // The FPS to use for the rearcam.
    const int ZED_REARCAM_HORIZONTAL_FOV        = 110;                        // The horizontal FOV of the camera. Useful for future calculations.
    const int ZED_REARCAM_VERTICAL_FOV          = 70;                         // The vertical FOV of the camera. Useful for future calculations.
    const bool ZED_REARCAM_EXPORT_SVO_RECORDING = true;                       // Whether or not to record the rearcam to an SVO file.
    const bool ZED_REARCAM_EXPORT_SPATIAL_MAP   = false;                      // Whether or not to export the spatial map to a file.
    const bool ZED_REARCAM_USE_GPU_MAT          = MODE_SIM ? false : true;    // Whether or not to use CPU or GPU memory mats. GPU memory transfer/operations are faster.
    const bool ZED_REARCAM_USE_HALF_PRECISION_DEPTH = true;                   // Whether of not to use float32 or unsigned short (16) for depth measure.
    const bool ZED_REARCAM_FUSION_MASTER            = false;       // Whether or not this camera will host the master instance of the ZEDSDK Fusion capabilities.
    const int ZED_REARCAM_FRAME_RETRIEVAL_THREADS   = 10;          // The number of threads allocated to the threadpool for performing frame copies to other threads.
    const int ZED_REARCAM_SERIAL                    = 39163798;    // The serial number of the camera. Set to 0 to open the next available one. DEFAULT = 31237348

    // Basic Cam.
    const int BASICCAM_CAM_RESOLUTIONX             = 1280;    // The horizontal pixel resolution to resize the basiccam images to.
    const int BASICCAM_CAM_RESOLUTIONY             = 720;     // The vertical pixel resolution to resize the basiccam images to.
    const int BASICCAM_CAM_FPS                     = 30;      // The FPS to use for the basiccam.
    const int BASICCAM_CAM_HORIZONTAL_FOV          = 110;     // The horizontal FOV of the camera. Useful for future calculations.
    const int BASICCAM_CAM_VERTICAL_FOV            = 70;      // The vertical FOV of the camera. Useful for future calculations.
    const int BASICCAM_CAM_FRAME_RETRIEVAL_THREADS = 5;       // The number of threads allocated to the threadpool for performing frame copies to other threads.
    const int BASICCAM_CAM_INDEX                   = 0;       // The /dev/video index of the camera.
    const PIXEL_FORMATS BASICCAM_CAM_PIXELTYPE     = PIXEL_FORMATS::eBGR;    // The pixel layout of the camera.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Bounding Box Tracking Constants.
    ///////////////////////////////////////////////////////////////////////////

    const double BBOX_MIN_LIFETIME_THRESHOLD = 0.15;     // How many seconds does the detection need to be detected before being validated as a good detection.
    const double BBOX_MIN_SCREEN_PERCENTAGE  = 0.001;    // Minumum percentage of the screen the detection must cover to be valid. 0-100
    const double BBOX_TRACKER_LOST_TIMEOUT   = 0.1;      // The time in seconds to wait before considering a tracker lost. This should always be less than MAX_LIFTTIME.
    const double BBOX_TRACKER_MAX_TRACK_TIME = 5.0;      // The maximum time in seconds to track a detection without new detection.MAX_TRACK_TIME
    const double BBOX_TRACKER_IOU_MATCH_THRESHOLD = 0.1;                             // The IOU threshold to match a new detection to an existing tracker.
    const tracking::TrackerType BBOX_TRACKER_TYPE = tracking::TrackerType::eCSRT;    // The type of tracker to use for the DNN detection.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Tag Detection Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Models to use for detection.
    const std::string TAGDETECT_TORCH_MODEL =
        "../data/models/yolo_models/tag/v8n_x640_200epochs_balanced/best.torchscript";    // The model path to use for tag detection.

    // Main ZED Camera.
    const int TAGDETECT_MAINCAM_DATA_RETRIEVAL_THREADS  = 2;     // The number of threads allocated to the threadpool for performing data copies to other threads.
    const int TAGDETECT_MAINCAM_CORNER_REFINE_MAX_ITER  = 30;    // The maximum number of iterations to run corner refinement on the image.
    const int TAGDETECT_MAINCAM_CORNER_REFINE_METHOD    = cv::aruco::CORNER_REFINE_NONE;    // Algorithm used to refine tag corner pixels.
    const bool TAGDETECT_MAINCAM_DETECT_INVERTED_MARKER = true;                             // Whether or not to detector upside-down tags.
    const int TAGDETECT_MAINCAM_MARKER_BORDER_BITS      = 1;                                // This number of bits on the border. A bit is one unit square of the tag.
    const bool TAGDETECT_MAINCAM_USE_ARUCO3_DETECTION   = true;                             // Whether or not to use the newer and faster Aruco detection strategy.
    const bool TAGDETECT_MAINCAM_ENABLE_TRACKING        = true;                             // Whether or not to use the tracking algorithm to track tags.
    const int TAGDETECT_MAINCAM_MAX_FPS                 = 30;                               // The max iterations per second of the tag detector.
    const bool TAGDETECT_MAINCAM_ENABLE_TORCH           = true;                             // Whether or not to use pytorch detection on top of ArUco.
    const float TAGDETECT_MAINCAM_TORCH_CONFIDENCE      = 0.7f;                             // The minimum confidence to consider a viable AR tag detection.
    const float TAGDETECT_MAINCAM_TORCH_NMS_THRESH      = 0.4f;                             // The threshold for non-max suppression filtering.

    // Rear ZED Camera.
    const int TAGDETECT_REARCAM_DATA_RETRIEVAL_THREADS  = 2;     // The number of threads allocated to the threadpool for performing data copies to other threads.
    const int TAGDETECT_REARCAM_CORNER_REFINE_MAX_ITER  = 30;    // The maximum number of iterations to run corner refinement on the image.
    const int TAGDETECT_REARCAM_CORNER_REFINE_METHOD    = cv::aruco::CORNER_REFINE_NONE;    // Algorithm used to refine tag corner pixels.
    const bool TAGDETECT_REARCAM_DETECT_INVERTED_MARKER = true;                             // Whether or not to detector upside-down tags.
    const int TAGDETECT_REARCAM_MARKER_BORDER_BITS      = 1;                                // This number of bits on the border. A bit is one unit square of the tag.
    const bool TAGDETECT_REARCAM_USE_ARUCO3_DETECTION   = true;                             // Whether or not to use the newer and faster Aruco detection strategy.
    const bool TAGDETECT_REARCAM_ENABLE_TRACKING        = true;                             // Whether or not to use the tracking algorithm to track tags.
    const int TAGDETECT_REARCAM_MAX_FPS                 = 30;                               // The max iterations per second of the tag detector.
    const bool TAGDETECT_REARCAM_ENABLE_TORCH           = true;                             // Whether or not to use pytorch detection on top of ArUco.
    const float TAGDETECT_REARCAM_TORCH_CONFIDENCE      = 0.7f;                             // The minimum confidence to consider a viable AR tag detection.
    const float TAGDETECT_REARCAM_TORCH_NMS_THRESH      = 0.4f;                             // The threshold for non-max suppression filtering.

    ///////////////////////////////////////////////////////////////////////////
    //// Object Detection Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Models to use for detection.
    const std::string OBJECTDETECT_TORCH_MODEL =
        "../data/models/yolo_models/bottle_mallet_new/v8n_x640_200epochs/best.torchscript";    // The model path to use for object detection.

    // Main ZED Camera.
    const int OBJECTDETECT_MAINCAM_DATA_RETRIEVAL_THREADS = 2;       // The number of threads allocated to the threadpool for performing data copies to other threads.
    const bool OBJECTDETECT_MAINCAM_ENABLE_TRACKING       = true;    // Whether or not to use the tracking algorithm to track objects.
    const int OBJECTDETECT_MAINCAM_MAX_FPS                = 30;      // The max iterations per second of the object detector.
    const bool OBJECTDETECT_MAINCAM_ENABLE_TORCH          = true;    // Whether or not to use pytorch detection.
    const float OBJECTDETECT_MAINCAM_TORCH_CONFIDENCE     = 0.8f;    // The minimum confidence to consider a viable object detection.
    const float OBJECTDETECT_MAINCAM_TORCH_NMS_THRESH     = 0.4f;    // The threshold for non-max suppression filtering.

                                                                     // Rear ZED Camera.
    const int OBJECTDETECT_REARCAM_DATA_RETRIEVAL_THREADS = 2;       // The number of threads allocated to the threadpool for performing data copies to other threads.
    const bool OBJECTDETECT_REARCAM_ENABLE_TRACKING       = true;    // Whether or not to use the tracking algorithm to track objects.
    const int OBJECTDETECT_REARCAM_MAX_FPS                = 30;      // The max iterations per second of the object detector.
    const bool OBJECTDETECT_REARCAM_ENABLE_TORCH          = true;    // Whether or not to use pytorch detection.
    const float OBJECTDETECT_REARCAM_TORCH_CONFIDENCE     = 0.8f;    // The minimum confidence to consider a viable object detection.
    const float OBJECTDETECT_REARCAM_TORCH_NMS_THRESH     = 0.4f;    // The threshold for non-max suppression filtering.

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// LiDAR Data Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // LiDAR Data Handler.
    const std::string LIDAR_HANDLER_DB_PATH = "../data/LiDAR/data/databases/Flat_SIM.db";    // The path to the LiDAR database file.

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Visualization Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // LiDAR Data Handler.
    const int VISUALIZER_WEBSERVER_PORT             = 3284;    // The port for the simple web server to use for serving the visualizer web UI.
    const std::string VISUALIZER_THREEJS_PATH       = "../data/Web_Visualizer/assets/three.module.js";     // The path to the ThreeJS library.
    const std::string VISUALIZER_ORBITCONTROLS_PATH = "../data/Web_Visualizer/assets/OrbitControls.js";    // The path to the OrbitControls library.

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// GeoPlanner Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Global GeoPlanner
    const double GEOPLANNER_TILE_SIZE = 50.0;    // The size of each tile in the GeoPlanner in meters.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// ArUco Vision Constants.
    ///////////////////////////////////////////////////////////////////////////

    // OpenCV ArUco detection config.
    const cv::aruco::PredefinedDictionaryType ARUCO_DICTIONARY = cv::aruco::DICT_4X4_50;    // The predefined ArUco dictionary to use for detections.
    const float ARUCO_TAG_SIDE_LENGTH                          = 0.015f;                    // Size of the white borders around the tag in meters.
    const double ARUCO_PIXEL_THRESHOLD                         = 175;                       // Pixel value threshold for pre-process threshold mask
    const double ARUCO_PIXEL_THRESHOLD_MAX_VALUE               = 255;                       // Pixel value to set to if pixel is within threshold
    const cv::Mat ARUCO_SHARPEN_KERNEL_FAST                    = (cv::Mat_<double>(3, 3) << 0, 0, 0, 0, 3, 0, 0, 0, 0);
    const cv::Mat ARUCO_SHARPEN_KERNEL_EXTRA                   = (cv::Mat_<double>(3, 3) << 0, 0, 0, 0, 9, 0, 0, 0, 0);
    const cv::Mat ARUCO_EDGE_KERNEL                            = (cv::Mat_<double>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// State Constants.
    ///////////////////////////////////////////////////////////////////////////

    // Handler.
    const int STATEMACHINE_MAX_IPS                  = 60;     // The maximum number of iteration per second of the state machines main thread.
    const double STATEMACHINE_ZED_REALIGN_THRESHOLD = 0.5;    // The threshold in meters that the error between GPS and ZED must be before realigning the ZED cameras.

    // Approaching Marker State
    const double APPROACH_MARKER_MOTOR_POWER          = DRIVE_MAX_SAFE_POWER * 0.2;    // The amount of power the motors use when approaching the marker.
    const double APPROACH_MARKER_PROXIMITY_THRESHOLD  = 2.0;      // How close in meters the rover must be to the target marker before completing its approach.
    const double APPROACH_MARKER_LOST_GIVE_UP_TIME    = 15.0;     // The time in seconds to wait before giving up on the approach AFTER the tag is lost.
    const bool APPROACH_MARKER_VERIFY_POSITION        = true;     // Whether or not the rover should sit and watch the tag for a while before moving on.
    const double APPROACH_MARKER_VERIFY_TIME          = 5.0;      // The time in seconds to watch the tag before moving on.
    const double APPROACH_MARKER_TAG_LOST_BUFFER_TIME = 2.0;      // The time in seconds to wait before considering the tag lost. This is used to prevent false negatives.
    const bool APPROACH_MARKER_ENABLE_STUCK_DETECT    = false;    // Whether or not to enable the stuck detection algorithm when approaching a marker.

    // Approaching Object State
    const double APPROACH_OBJECT_MOTOR_POWER         = DRIVE_MAX_SAFE_POWER * 0.2;    // The amount of power the motors use when approaching the object.
    const double APPROACH_OBJECT_PROXIMITY_THRESHOLD = 2.0;     // How close in meters the rover must be to the target object before completing its approach.
    const double APPROACH_OBJECT_LOST_GIVE_UP_TIME   = 15.0;    // The time in seconds to wait before giving up on the approach AFTER the object is lost.
    const bool APPROACH_OBJECT_VERIFY_POSITION       = true;    // Whether or not the rover should sit and watch the object for a while before moving on.
    const double APPROACH_OBJECT_VERIFY_TIME         = 5.0;     // The time in seconds to watch the object before moving on.
    const double APPROACH_OBJECT_LOST_BUFFER_TIME    = 2.0;    // The time in seconds to wait before considering the object lost. This is used to prevent false negatives.
    const bool APPROACH_OBJECT_ENABLE_STUCK_DETECT   = false;    // Whether or not to enable the stuck detection algorithm when approaching an object.

    // Stuck State
    const double STUCK_CHECK_INTERVAL        = 2.0;     // Period in seconds between consecutive checks of if the rover's rotating.
    const unsigned int STUCK_CHECK_ATTEMPTS  = 3;       // Max number of failed checks of the rover's rotation before next attempt.
    const double STUCK_CHECK_ROT_THRESH      = 1.0;     // Minimum angular velocity required to consider the rover as actively rotating.
    const double STUCK_CHECK_VEL_THRESH      = 0.01;    // Minimum velocity required to consider the rover as actively moving.
    const double STUCK_SAME_POINT_PROXIMITY  = 1.0;     // Points within this proximity of another point are considered the same.
    const double STUCK_HEADING_ALIGN_TIMEOUT = 5.0;     // The timeout in seconds before the rover gives up aligning to a certain heading.
    const double STUCK_ALIGN_DEGREES         = 65.0;    // The amount to rotate/realign for rover after a failed attempt.
    const double STUCK_ALIGN_TOLERANCE       = 5.0;     // Degree tolerance before realignment is considered complete.

    // Reverse State.
    const double REVERSE_MOTOR_POWER       = DRIVE_MAX_SAFE_POWER * 0.6;    // The speed to drive backwards at.
    const double REVERSE_DISTANCE          = 3.0;                           // The distance to reverse in meters.
    const double REVERSE_TIMEOUT_PER_METER = 5.0;                           // Reverse state timeout in seconds for each meter reversed.
    const bool REVERSE_MAINTAIN_HEADING    = true;                          // Whether or not the rover should maintain heading while reversing.

    // Search Pattern State
    const double SEARCH_MOTOR_POWER          = DRIVE_MAX_SAFE_POWER * 0.4;    // The amount of power the motors use when approaching the marker.
    const double SEARCH_ANGULAR_STEP_DEGREES = 57.0;                          // The amount the angle is incremented in each iteration of the loop (degrees).
    const double SEARCH_SPIRAL_SPACING       = 1.0;                           // The spacing between successive points in the spiral (meters).
    const double SEARCH_ZIGZAG_SPACING       = 4.0;                           // The spacing between successive points in the zigzag (meters).
    const double SEARCH_SNAKE_SLITHERS       = 2.0;                           // The number of slithers in the snake pattern.
    const double SEARCH_WAYPOINT_PROXIMITY   = 2.0;                           // How close a rover must be to a point to have it count as visited.
    const bool SEARCH_ENABLE_STUCK_DETECT    = false;                         // Whether or not to enable the stuck detection algorithm when searching for a marker.

    // Navigating State.
    const double NAVIGATING_MOTOR_POWER         = DRIVE_MAX_SAFE_POWER * 0.9;    // The speed to drive at when navigating.
    const double NAVIGATING_REACHED_GOAL_RADIUS = 2.0;                           // The radius in meters that the rover should get to the goal waypoint.
    const bool NAVIGATING_VERIFY_POSITION       = true;     // Whether or not the rover should sit and verify the rover's GPS position before moving on.
    const double NAVIGATING_VERIFY_SAMPLE_TIME  = 30.0;     // The time in seconds to collect GPS points before verifying the rover's GPS position.
    const bool NAVIGATING_ENABLE_STUCK_DETECT   = false;    // Whether or not to enable the stuck detection algorithm when navigating to a waypoint.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Algorithm Constants.
    ///////////////////////////////////////////////////////////////////////////

    // Stanley Controller config.
    const double STANLEY_CROSSTRACK_CONTROL_GAIN = 0.5;     // Determines how reactive the rover is to crosstrack error adjustments.
    const double STANLEY_ANGULAR_VELOCITY_LIMIT  = 90.0;    // The maximum angular velocity in degrees per second.
    const int STANLEY_PREDICTION_HORIZON         = 5;       // The number of predictions to make.
    const double STANLEY_PREDICTION_TIME_STEP    = 0.01;    // The time to pass in seconds between each prediction of the Stanley controller unicycle model.

    // ASTAR config.
    const double ASTAR_AVOIDANCE_MULTIPLIER = 1.2;       // Multiplier for marking extra nodes around objects as obstacles
    const double ASTAR_MAX_SEARCH_GRID      = 4000.0;    // Maximum search grid size (UTM)
    const double ASTAR_MAX_SEARCH_TIME      = 120.0;     // Maximum time to search for a path before giving up. Time is in seconds.
    const double ASTAR_NODE_SIZE            = 0.5;       // Represents the node size / accuracy in meters
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Driver Constants.
    ///////////////////////////////////////////////////////////////////////////

    // NavBoard.
    const double NAVBOARD_MAX_GPS_DATA_AGE     = 3.0;    // The maximum age of the current GPS data before printing warnings.
    const double NAVBOARD_MAX_COMPASS_DATA_AGE = 3.0;    // The maximum age of the current Compass data before printing warnings.
    ////////////////////////////////////////////////////////////////////////////

}    // namespace constants
