/******************************************************************************
 * @brief The constants header for Autonomy Software
 *
 * @file Consts.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-30
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef CONSTS_H
#define CONSTS_H

#include "./interfaces/Camera.hpp"

/// \cond
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

/// \endcond

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

    // Program mode constants.
#if defined(__AUTONOMY_SIM_MODE__) && __AUTONOMY_SIM_MODE__ == 1
    const bool MODE_SIM = true;    // SIM MODE ENABLED: Toggle RoveComm and Cameras to use local data from the Webots SIM.
#else
    const bool MODE_SIM = false;    // REG MODE ENABLED: Toggle RoveComm and Cameras to use standard configuration.
#endif

    // Safety constants.
    const double BATTERY_MINIMUM_CELL_VOLTAGE = 3.2;    // The minimum cell voltage of the battery before autonomy will forcefully enter Idle state.
    const bool BATTERY_CHECKS_ENABLED = false;          // If autonomy should monitor PMS Currents and as a result have the ability to shutdown autonomy.

    // Logging constants.
    const std::string LOGGING_OUTPUT_PATH_ABSOLUTE = "../logs/";    // The absolute to write output logging and video files to.

    // Logging color constants.
    const std::string szTraceL3Color   = "\033[30m";           // Standard Grey
    const std::string szTraceL2Color   = "\033[30m";           // Standard Grey
    const std::string szTraceL1Color   = "\033[30m";           // Standard Grey
    const std::string szDebugColor     = "\033[36m";           // Standard Cyan
    const std::string szInfoColor      = "\033[32m";           // Standard Green
    const std::string szWarningColor   = "\033[93m\033[1m";    // Bright Bold Yellow
    const std::string szErrorColor     = "\033[91m\033[1m";    // Bright Bold Red
    const std::string szCriticalColor  = "\033[95m\033[1m";    // Bright Bold Magenta
    const std::string szBacktraceColor = "\033[34m";           // Standard Blue

    // RoveComm constants.
    const int ROVECOMM_OUTGOING_UDP_PORT        = MODE_SIM ? 11001 : 11000;    // The UDP socket port to use for the main UDP RoveComm instance.
    const int ROVECOMM_OUTGOING_TCP_PORT        = MODE_SIM ? 12001 : 12000;    // The UDP socket port to use for the main UDP RoveComm instance.
    const std::string ROVECOMM_TCP_INTERFACE_IP = "";    // The IP address to bind the socket to. If set to "", the socket will be bound to all available interfaces.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Drive Constants.
    ///////////////////////////////////////////////////////////////////////////

    // Power constants.
    const float DRIVE_MAX_POWER  = 1.0;
    const float DRIVE_MIN_POWER  = -1.0;
    const float DRIVE_MAX_EFFORT = 0.5;
    const float DRIVE_MIN_EFFORT = -0.5;

    // Control constants.
    const double DRIVE_PID_PROPORTIONAL       = 0.01;     // The proportional gain for the controller used to point the rover at a goal heading during navigation.
    const double DRIVE_PID_INTEGRAL           = 0.005;    // The integral gain for the controller used to point the rover at a goal heading during navigation.
    const double DRIVE_PID_DERIVATIVE         = 0.02;     // The derivative gain for the controller used to point the rover at a goal heading during navigation.
    const double DRIVE_PID_FEEDFORWARD        = 0.0;      // The feedforward for the controller used to predict control output.
    const double DRIVE_PID_MAX_ERROR_PER_ITER = 180;      // The max allowable error the controller will see per iteration. This is on degrees from setpoint.
    const double DRIVE_PID_MAX_INTEGRAL_TERM  = 0.15;     // The max effort the I term is allowed to contribute.
    const double DRIVE_PID_MAX_RAMP_RATE      = 0.08;     // The max ramp rate of the output of the PID controller.
    const double DRIVE_PID_OUTPUT_FILTER      = 0.78;     // Larger values will filter out large spikes or oscillations. 0.1 is a good starting point.
    const double DRIVE_PID_TOLERANCE          = 1.0;      // The max allowable error from the setpoint for the controller to be considered at the setpoint.
    const bool DRIVE_PID_OUTPUT_REVERSED      = false;    // Negates the output of the PID controller.
    const bool DRIVE_SQUARE_CONTROL_INPUTS    = false;    // This is used by the DifferentialDrive algorithms. True makes fine inputs smoother, but less responsive.
    const bool DRIVE_CURVATURE_KINEMATICS_ALLOW_TURN_WHILE_STOPPED = true;    // This enabled turning in-place when using curvature drive control.
    ///////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////.;'//////////////////
    //// Recording Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Recording adjustments.
    const int RECORDER_FPS = 15;    // The FPS all recordings should run at.
    // Camera recording toggles.
    const bool ZED_MAINCAM_ENABLE_RECORDING        = true;    // Whether or not to record the main ZED camera.
    const bool ZED_LEFTCAM_ENABLE_RECORDING        = true;    // Whether or not to record the left ZED camera.
    const bool ZED_RIGHTCAM_ENABLE_RECORDING       = true;    // Whether or not to record the right ZED camera.
    const bool BASICCAM_GROUNDCAM_ENABLE_RECORDING = true;    // Whether or not to record the ground USB camera.
    // TagDetector recording toggles.
    const bool TAGDETECT_MAINCAM_ENABLE_RECORDING   = true;    // Whether or not to record the main ZED camera tag detector.
    const bool TAGDETECT_LEFTCAM_ENABLE_RECORDING   = true;    // Whether or not to record the left ZED camera tag detector.
    const bool TAGDETECT_RIGHTCAM_ENABLE_RECORDING  = true;    // Whether or not to record the right ZED camera tag detector.
    const bool TAGDETECT_GROUNDCAM_ENABLE_RECORDING = true;    // Whether of not to record the ground USB camera tag detector.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Camera Constants.
    ///////////////////////////////////////////////////////////////////////////

    // ZedCam Basic Config.
    const sl::RESOLUTION ZED_BASE_RESOLUTION     = sl::RESOLUTION::HD720;                      // The base resolution to open the all cameras with.
    const sl::UNIT ZED_MEASURE_UNITS             = sl::UNIT::METER;                            // The base measurement unit to use for depth.
    const sl::COORDINATE_SYSTEM ZED_COORD_SYSTEM = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;    // Coordinate system to use for measurements.
    const sl::DEPTH_MODE ZED_DEPTH_MODE          = sl::DEPTH_MODE::NEURAL;                     // The measurement accuracy for depth. NEURAL is by far the best.
    const sl::VIEW ZED_RETRIEVE_VIEW             = sl::VIEW::LEFT;                             // The eye to retrieve regular and depth images from.
    const bool ZED_SDK_VERBOSE                   = false;                                      // Enable verbose output from the internal Camera library in the ZEDSDK.
    const bool ZED_SENSING_FILL                  = false;    // True provides a depth map with a Z value for every pixel (X, Y) in the left image. Slower and worse.
    const float ZED_DEFAULT_MINIMUM_DISTANCE     = 0.5;      // Minimum distance in ZED_MEASURE_UNITS to report from depth measurement.
    const float ZED_DEFAULT_MAXIMUM_DISTANCE     = 40.0;     // Maximum distance in ZED_MEASURE_UNITS to report from depth measurement.
    const float ZED_DEFAULT_FLOOR_PLANE_ERROR    = 0.5;      // The maximum distance that an estimated floor plane can be from the height of the camera from the ground.
    const int ZED_DEPTH_STABILIZATION            = 1;    // This parameter controls a stabilization filter that reduces oscillations in depth map. In the range [0-100]
    // ZedCam Positional Tracking Config.
    const sl::POSITIONAL_TRACKING_MODE ZED_POSETRACK_MODE = sl::POSITIONAL_TRACKING_MODE::GEN_1;    // Positional tracking accuracy.
    const bool ZED_POSETRACK_AREA_MEMORY                  = true;     // Enabled camera to remember its surroundings for better positioning. Uses more resources.
    const bool ZED_POSETRACK_POSE_SMOOTHING               = false;    // Smooth pose correction for small drift. Decreases overall precision for small movements.
    const bool ZED_POSETRACK_FLOOR_IS_ORIGIN              = true;     // Sets the floor plane as origin for tracking. This turns on floor plane detection temporarily.
    const bool ZED_POSETRACK_ENABLE_IMU_FUSION            = true;     // Allows ZED to use both optical odometry and IMU data for pose tracking.
    const float ZED_POSETRACK_USABLE_DEPTH_MIN            = 0.75;      // Minimum depth used for pose tracking, useful if a static object is partial in view of the camera.
    const float ZED_POSETRACK_USE_GRAVITY_ORIGIN          = true;     // Override 2 of the 3 rotations from initial_world_transform using the IMU.
    // ZedCam Spatial Mapping Config.
    const sl::SpatialMappingParameters::SPATIAL_MAP_TYPE ZED_MAPPING_TYPE = sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;    // Mesh or point cloud output.
    const float ZED_MAPPING_RANGE_METER                                   = 20.0;    // The max range in meters that the ZED cameras should use for mapping. 0 = auto.
    const float ZED_MAPPING_RESOLUTION_METER                              = 0.03;    // The approx goal precision for spatial mapping in METERS. Higher = Faster.
    const int ZED_MAPPING_MAX_MEMORY                                      = 4096;    // The max amount of CPU RAM (MB) that can be allocated for spatial mapping.
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
    const bool FUSION_ENABLE_GNSS_FUSION            = true;                 // Enable the fusion of camera visual odometry tracking with GNSS data from NavBoard.

    // BasicCam Basic Config.
    const cv::InterpolationFlags BASICCAM_RESIZE_INTERPOLATION_METHOD = cv::InterpolationFlags::INTER_LINEAR;    // The algorithm used to fill in pixels when resizing.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Camera Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Main ZED Camera.
    const int ZED_MAINCAM_RESOLUTIONX               = 1280;    // The horizontal pixel resolution to resize the maincam images to.
    const int ZED_MAINCAM_RESOLUTIONY               = 720;     // The vertical pixel resolution to resize the maincam images to.
    const int ZED_MAINCAM_FPS                       = 60;      // The FPS to use for the maincam.
    const int ZED_MAINCAM_HORIZONTAL_FOV            = 110;     // The horizontal FOV of the camera. Useful for future calculations.
    const int ZED_MAINCAM_VERTICAL_FOV              = 70;      // The vertical FOV of the camera. Useful for future calculations.
    const bool ZED_MAINCAM_USE_GPU_MAT              = true;    // Whether or not to use CPU or GPU memory mats. GPU memory transfer/operations are faster.
    const bool ZED_MAINCAM_USE_HALF_PRECISION_DEPTH = true;    // Whether of not to use float32 or unsigned short (16) for depth measure.
    const bool ZED_MAINCAM_FUSION_MASTER            = false;    // Whether or not this camera will host the master instance of the ZEDSDK Fusion capabilities.
    const int ZED_MAINCAM_FRAME_RETRIEVAL_THREADS   = 10;       // The number of threads allocated to the threadpool for performing frame copies to other threads.
    const int ZED_MAINCAM_SERIAL                    = 0;       // The serial number of the camera. Set to 0 to open the next available one. 31237348

    // Left ZED Camera.
    const int ZED_LEFTCAM_RESOLUTIONX               = 1280;     // The horizontal pixel resolution to resize the leftcam images to.
    const int ZED_LEFTCAM_RESOLUTIONY               = 720;      // The vertical pixel resolution to resize the leftcam images to.
    const int ZED_LEFTCAM_FPS                       = 60;       // The FPS to use for the leftcam.
    const int ZED_LEFTCAM_HORIZONTAL_FOV            = 110;      // The horizontal FOV of the camera. Useful for future calculations.
    const int ZED_LEFTCAM_VERTICAL_FOV              = 70;       // The vertical FOV of the camera. Useful for future calculations.
    const bool ZED_LEFTCAM_USE_GPU_MAT              = true;     // Whether or not to use CPU or GPU memory mats. GPU memory transfer/operations are faster.
    const bool ZED_LEFTCAM_USE_HALF_PRECISION_DEPTH = true;     // Whether of not to use float32 or unsigned short (16) for depth measure.
    const bool ZED_LEFTCAM_FUSION_MASTER            = false;    // Whether or not this camera will host the master instance of the ZEDSDK Fusion capabilities.
    const int ZED_LEFTCAM_FRAME_RETRIEVAL_THREADS   = 5;        // The number of threads allocated to the threadpool for performing frame copies to other threads.
    const int ZED_LEFTCAM_SERIAL                    = 0;        // The serial number of the camera. Set to 0 to open the next available one. 15723847

    // Right ZED Camera.
    const int ZED_RIGHTCAM_RESOLUTIONX               = 1280;     // The horizontal pixel resolution to resize the rightcam images to.
    const int ZED_RIGHTCAM_RESOLUTIONY               = 720;      // The vertical pixel resolution to resize the rightcam images to.
    const int ZED_RIGHTCAM_FPS                       = 60;       // The FPS to use for the rightcam.
    const int ZED_RIGHTCAM_HORIZONTAL_FOV            = 110;      // The horizontal FOV of the camera. Useful for future calculations.
    const int ZED_RIGHTCAM_VERTICAL_FOV              = 70;       // The vertical FOV of the camera. Useful for future calculations.
    const bool ZED_RIGHTCAM_USE_GPU_MAT              = true;     // Whether or not to use CPU or GPU memory mats. GPU memory transfer/operations are faster.
    const bool ZED_RIGHTCAM_USE_HALF_PRECISION_DEPTH = true;     // Whether of not to use float32 or unsigned short (16) for depth measure.
    const bool ZED_RIGHTCAM_FUSION_MASTER            = false;    // Whether or not this camera will host the master instance of the ZEDSDK Fusion capabilities.
    const int ZED_RIGHTCAM_FRAME_RETRIEVAL_THREADS   = 5;        // The number of threads allocated to the threadpool for performing frame copies to other threads.
    const int ZED_RIGHTCAM_SERIAL                    = 0;        // The serial number of the camera. Set to 0 to open the next available one. 0

    // Ground Basic Cam.
    const int BASICCAM_GROUNDCAM_RESOLUTIONX             = 1280;    // The horizontal pixel resolution to resize the basiccam images to.
    const int BASICCAM_GROUNDCAM_RESOLUTIONY             = 720;     // The vertical pixel resolution to resize the basiccam images to.
    const int BASICCAM_GROUNDCAM_FPS                     = 30;      // The FPS to use for the basiccam.
    const int BASICCAM_GROUNDCAM_HORIZONTAL_FOV          = 110;     // The horizontal FOV of the camera. Useful for future calculations.
    const int BASICCAM_GROUNDCAM_VERTICAL_FOV            = 70;      // The vertical FOV of the camera. Useful for future calculations.
    const int BASICCAM_GROUNDCAM_FRAME_RETRIEVAL_THREADS = 5;       // The number of threads allocated to the threadpool for performing frame copies to other threads.
    const int BASICCAM_GROUNDCAM_INDEX                   = 0;       // The /dev/video index of the camera.
    const PIXEL_FORMATS BASICCAM_GROUNDCAM_PIXELTYPE     = PIXEL_FORMATS::eBGR;    // The pixel layout of the camera.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// ArUco Vision Constants.
    ///////////////////////////////////////////////////////////////////////////

    // OpenCV ArUco detection config.
    const cv::aruco::PredefinedDictionaryType ARUCO_DICTIONARY = cv::aruco::DICT_4X4_50;    // The predefined ArUco dictionary to use for detections.
    const float ARUCO_TAG_SIDE_LENGTH                          = 0.015;                     // Size of the white borders around the tag.
    const int ARUCO_VALIDATION_THRESHOLD             = 5;      // How many times does the tag need to be detected(hit) before being validated as an actual aruco tag.
    const int ARUCO_UNVALIDATED_TAG_FORGET_THRESHOLD = 5;      // How many times can an unvalidated tag be missing from frame before being forgotten.
    const int ARUCO_VALIDATED_TAG_FORGET_THRESHOLD   = 10;     // How many times can a validated tag be missing from frame before being forgotten.
    const double ARUCO_PIXEL_THRESHOLD               = 175;    // Pixel value threshold for pre-process threshold mask
    const double ARUCO_PIXEL_THRESHOLD_MAX_VALUE     = 255;    // Pixel value to set to if pixel is within threshold
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Tag Detection Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Main ZED Camera.
    const int TAGDETECT_MAINCAM_DATA_RETRIEVAL_THREADS  = 2;     // The number of threads allocated to the threadpool for performing data copies to other threads.
    const int TAGDETECT_MAINCAM_CORNER_REFINE_MAX_ITER  = 30;    // The maximum number of iterations to run corner refinement on the image.
    const int TAGDETECT_MAINCAM_CORNER_REFINE_METHOD    = cv::aruco::CORNER_REFINE_NONE;    // Algorithm used to refine tag corner pixels.
    const bool TAGDETECT_MAINCAM_DETECT_INVERTED_MARKER = false;                            // Whether or not to detector upside-down tags.
    const int TAGDETECT_MAINCAM_MARKER_BORDER_BITS      = 1;                                // This number of bits on the border. A bit is one unit square of the tag.
    const bool TAGDETECT_MAINCAM_USE_ARUCO3_DETECTION   = true;                             // Whether or not to use the newer and faster Aruco detection strategy.
    const int TAGDETECT_MAINCAM_MAX_FPS                 = 30;                               // The max iterations per second of the tag detector.
    const bool TAGDETECT_MAINCAM_ENABLE_DNN             = true;                             // Whether or not to use DNN detection on top of ArUco.
    const std::string TAGDETECT_MAINCAM_MODEL_PATH = "../data/models/yolo_models/tag/v5n_x320_200epochs/best_edgetpu.tflite";    // The model path to use for detection.
    const float TAGDETECT_MAINCAM_DNN_CONFIDENCE   = 0.4f;    // The minimum confidence to consider a viable AR tag detection.
    const float TAGDETECT_MAINCAM_DNN_NMS_THRESH   = 0.4f;    // The threshold for non-max suppression filtering.

    // Left ZED Camera.
    const int TAGDETECT_LEFTCAM_DATA_RETRIEVAL_THREADS  = 2;     // The number of threads allocated to the threadpool for performing data copies to other threads.
    const int TAGDETECT_LEFTCAM_CORNER_REFINE_MAX_ITER  = 30;    // The maximum number of iterations to run corner refinement on the image.
    const int TAGDETECT_LEFTCAM_CORNER_REFINE_METHOD    = cv::aruco::CORNER_REFINE_NONE;    // Algorithm used to refine tag corner pixels.
    const bool TAGDETECT_LEFTCAM_DETECT_INVERTED_MARKER = false;                            // Whether or not to detector upside-down tags.
    const int TAGDETECT_LEFTCAM_MARKER_BORDER_BITS      = 1;                                // This number of bits on the border. A bit is one unit square of the tag.
    const bool TAGDETECT_LEFTCAM_USE_ARUCO3_DETECTION   = true;                             // Whether or not to use the newer and faster Aruco detection strategy.
    const int TAGDETECT_LEFTCAM_MAX_FPS                 = 30;                               // The max iterations per second of the tag detector.
    const bool TAGDETECT_LEFTCAM_ENABLE_DNN             = false;                            // Whether or not to use DNN detection on top of ArUco.
    const std::string TAGDETECT_LEFTCAM_MODEL_PATH = "../data/models/yolo_models/tag/v5n_x320_200epochs/best_edgetpu.tflite";    // The model path to use for detection.
    const float TAGDETECT_LEFTCAM_DNN_CONFIDENCE   = 0.4f;    // The minimum confidence to consider a viable AR tag detection.
    const float TAGDETECT_LEFTCAM_DNN_NMS_THRESH   = 0.4f;    // The threshold for non-max suppression filtering.

    // Right ZED Camera.
    const int TAGDETECT_RIGHTCAM_DATA_RETRIEVAL_THREADS  = 2;     // The number of threads allocated to the threadpool for performing data copies to other threads.
    const int TAGDETECT_RIGHTCAM_CORNER_REFINE_MAX_ITER  = 30;    // The maximum number of iterations to run corner refinement on the image.
    const int TAGDETECT_RIGHTCAM_CORNER_REFINE_METHOD    = cv::aruco::CORNER_REFINE_NONE;    // Algorithm used to refine tag corner pixels.
    const bool TAGDETECT_RIGHTCAM_DETECT_INVERTED_MARKER = false;                            // Whether or not to detector upside-down tags.
    const int TAGDETECT_RIGHTCAM_MARKER_BORDER_BITS      = 1;                                // This number of bits on the border. A bit is one unit square of the tag.
    const bool TAGDETECT_RIGHTCAM_USE_ARUCO3_DETECTION   = true;                             // Whether or not to use the newer and faster Aruco detection strategy.
    const int TAGDETECT_RIGHTCAM_MAX_FPS                 = 30;                               // The max iterations per second of the tag detector.
    const bool TAGDETECT_RIGHTCAM_ENABLE_DNN             = false;                            // Whether or not to use DNN detection on top of ArUco.
    const std::string TAGDETECT_RIGHTCAM_MODEL_PATH = "../data/models/yolo_models/tag/v5n_x320_200epochs/best_edgetpu.tflite";    // The model path to use for detection.
    const float TAGDETECT_RIGHTCAM_DNN_CONFIDENCE   = 0.4f;    // The minimum confidence to consider a viable AR tag detection.
    const float TAGDETECT_RIGHTCAM_DNN_NMS_THRESH   = 0.4f;    // The threshold for non-max suppression filtering.

    ///////////////////////////////////////////////////////////////////////////
    //// Object Detection Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Main ZED Camera.
    const int OBJECTDETECT_MAINCAM_DATA_RETRIEVAL_THREADS = 5;    // The number of threads allocated to the threadpool for performing data copies to other threads.

    // Left Side Cam.
    const int OBJECTDETECT_LEFTCAM_DATA_RETRIEVAL_THREADS = 5;    // The number of threads allocated to the threadpool for performing data copies to other threads.

    // Right Side Cam.
    const int OBJECTDETECT_RIGHTCAM_DATA_RETRIEVAL_THREADS = 5;    // The number of threads allocated to the threadpool for performing data copies to other threads.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// State Constants.
    ///////////////////////////////////////////////////////////////////////////

    // Approaching Marker State
    const int APPROACH_MARKER_DETECT_ATTEMPTS_LIMIT      = 5;      // How many consecutive failed attempts at detecting a tag before giving up on marker.
    const double APPROACH_MARKER_MOTOR_POWER             = 0.5;    // The amount of power the motors use when approaching the marker.
    const double APPROACH_MARKER_PROXIMITY_THRESHOLD     = 5;      // How close the rover must be to the target marker before completing its approach.
    const double APPROACH_MARKER_TF_CONFIDENCE_THRESHOLD = 0.5;    // What is the minimal confidence necessary to consider a tensorflow tag as a target.

    // Stuck State
    const double STUCK_CHECK_INTERVAL        = 2.0;     // Period in seconds between consecutive checks of if the rover's rotating.
    const unsigned int STUCK_CHECK_ATTEMPTS  = 3;       // Max number of failed checks of the rover's rotation before next attempt.
    const double STUCK_CHECK_ROT_THRESH      = 10.0;    // Minimum angular velocity required to consider the rover as actively rotating.
    const double STUCK_CHECK_VEL_THRESH      = 0.5;     // Minimum velocity required to consider the rover as actively moving.
    const double STUCK_SAME_POINT_PROXIMITY  = 1.0;     // Points within this proximity of another point are considered the same.
    const double STUCK_HEADING_ALIGN_TIMEOUT = 5.0;     // The timeout in seconds before the rover gives up aligning to a certain heading.
    const double STUCK_ALIGN_DEGREES         = 65.0;    // The amount to rotate/realign for rover after a failed attempt.
    const double STUCK_ALIGN_TOLERANCE       = 5.0;     // Degree tolerance before realignment is considered complete.

    // Reverse State.
    const double REVERSE_POWER             = DRIVE_MAX_POWER;    // The speed to drive backwards at.
    const double REVERSE_DISTANCE          = 3.0;                // The distance to reverse in meters.
    const double REVERSE_TIMEOUT_PER_METER = 5.0;                // Reverse state timeout in seconds for each meter reversed.
    const bool REVERSE_MAINTAIN_HEADING    = true;               // Whether or not the rover should maintain heading while reversing.

    // Search Pattern State
    const double SEARCH_ANGULAR_STEP_DEGREES     = 57;                  // The amount the angle is incremented in each iteration of the loop (degrees).
    const double SEARCH_MAX_RADIUS               = 25;                  //  The maximum radius to cover in the search (meters).
    const double SEARCH_STARTING_HEADING_DEGREES = 0;                   // The angle the rover is facing at the start of the search(degrees).
    const double SEARCH_SPACING                  = 2;                   // The spacing between successive points in the spiral (meters).
    const double SEARCH_WAYPOINT_PROXIMITY       = 1;                   // How close a rover must be to a point to have it count as visited.
    const double SEARCH_MOTOR_POWER              = DRIVE_MAX_EFFORT;    // The amount of power the motors use when approaching the marker.
    // Handler.
    const int STATEMACHINE_MAX_IPS = 60;    // The maximum number of iteration per second of the state machines main thread.

    // Navigating State.
    const double NAVIGATING_REACHED_GOAL_RADIUS = 0.5;    // The radius in meters that the rover should get to the goal waypoint.

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Tag Detection Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // High Level Functionality Adjustments.

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Algorithm Constants.
    ///////////////////////////////////////////////////////////////////////////

    // Stanley Controller config.
    const double STANLEY_STEER_CONTROL_GAIN = 0.5;    // Determines how reactive the rover is to yaw adjustments.
    const double STANLEY_DIST_TO_FRONT_AXLE = 2.9;    // Distance from position sensor to the center of the front axle.
    const double STANLEY_YAW_TOLERANCE      = 1.0;    // Threshold for limiting unnecessary small movements.

    // ASTAR config.
    const double ASTAR_AVOIDANCE_MULTIPLIER = 1.2;          // Multiplier for marking extra nodes around objects as obstacles
    const double ASTAR_MAXIMUM_SEARCH_GRID  = 10.0;         // Maximum search grid size (UTM)
    const double ASTAR_NODE_SIZE            = 0.5;          // Represents the node size / accuracy in meters
    const double ASTAR_SQRT_NODE_SIZE       = M_SQRT1_2;    // Square root of m_dNodeSize

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Driver Constants.
    ///////////////////////////////////////////////////////////////////////////

    // NavBoard.
    const double NAVBOARD_MAX_GPS_DATA_AGE     = 3.0;    // The maximum age of the current GPS data before printing warnings.
    const double NAVBOARD_MAX_COMPASS_DATA_AGE = 3.0;    // The maximum age of the current Compass data before printing warnings.

    ///////////////////////////////////////////////////////////////////////////

}    // namespace constants

#endif    // CONSTS_H
