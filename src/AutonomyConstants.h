/******************************************************************************
 * @brief Declares constants for the autonomy software.
 *
 * @file AutonomyConstants.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-06
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef AUTONOMY_CONSTANTS_H
#define AUTONOMY_CONSTANTS_H

#include "./interfaces/Camera.hpp"
#include "./util/vision/BoundingBoxTracking.h"

/// \cond
#include <opencv2/opencv.hpp>
#include <quill/core/LogLevel.h>
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

    // Sim mode constants.
    extern const bool MODE_SIM;
    extern const std::string SIM_IP_ADDRESS;
    extern const uint SIM_WEBSOCKET_PORT;
    extern const uint SIM_WEBRTC_QP;

    // Safety constants.
    extern const double BATTERY_MINIMUM_CELL_VOLTAGE;
    extern const bool BATTERY_CHECKS_ENABLED;

    // Logging constants.
    extern const std::string LOGGING_OUTPUT_PATH_ABSOLUTE;
    extern const quill::LogLevel CONSOLE_MIN_LEVEL;
    extern const quill::LogLevel FILE_MIN_LEVEL;
    extern const quill::LogLevel ROVECOMM_MIN_LEVEL;
    extern const quill::LogLevel CONSOLE_DEFAULT_LEVEL;
    extern const quill::LogLevel FILE_DEFAULT_LEVEL;
    extern const quill::LogLevel ROVECOMM_DEFAULT_LEVEL;

    // Logging color constants.
    extern const std::string szTraceL3Color;
    extern const std::string szTraceL2Color;
    extern const std::string szTraceL1Color;
    extern const std::string szDebugColor;
    extern const std::string szInfoColor;
    extern const std::string szNoticeColor;
    extern const std::string szWarningColor;
    extern const std::string szErrorColor;
    extern const std::string szCriticalColor;
    extern const std::string szBacktraceColor;

    // RoveComm constants.
    extern const int ROVECOMM_OUTGOING_UDP_PORT;
    extern const int ROVECOMM_OUTGOING_TCP_PORT;
    extern const std::string ROVECOMM_TCP_INTERFACE_IP;
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Drive Constants.
    ///////////////////////////////////////////////////////////////////////////

    // Power constants.
    extern const float DRIVE_MAX_POWER;
    extern const float DRIVE_MIN_POWER;
    // NOTE: This should not be used to adjust the rover's speed for task. This is just a limit. Refer to the state machine constants for speed control.
    extern const float DRIVE_MAX_EFFORT;
    extern const float DRIVE_MIN_EFFORT;

    // Control constants.
    extern const double DRIVE_PID_PROPORTIONAL;
    extern const double DRIVE_PID_INTEGRAL;
    extern const double DRIVE_PID_DERIVATIVE;
    extern const double DRIVE_PID_FEEDFORWARD;
    extern const double DRIVE_PID_MAX_ERROR;
    extern const double DRIVE_PID_MAX_INTEGRAL_TERM;
    extern const double DRIVE_PID_MAX_RAMP_RATE;
    extern const double DRIVE_PID_OUTPUT_FILTER;
    extern const double DRIVE_PID_TOLERANCE;
    extern const bool DRIVE_PID_OUTPUT_REVERSED;
    extern const bool DRIVE_SQUARE_CONTROL_INPUTS;
    extern const bool DRIVE_CURVATURE_KINEMATICS_ALLOW_TURN_WHILE_STOPPED;
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Recording Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Recording adjustments.
    extern const int RECORDER_FPS;
    // Camera recording toggles.
    extern const bool ZED_MAINCAM_ENABLE_RECORDING;
    extern const bool ZED_LEFTCAM_ENABLE_RECORDING;
    extern const bool ZED_RIGHTCAM_ENABLE_RECORDING;
    extern const bool BASICCAM_GROUNDCAM_ENABLE_RECORDING;
    // TagDetector recording toggles.
    extern const bool TAGDETECT_MAINCAM_ENABLE_RECORDING;
    extern const bool TAGDETECT_LEFTCAM_ENABLE_RECORDING;
    extern const bool TAGDETECT_RIGHTCAM_ENABLE_RECORDING;
    extern const bool TAGDETECT_GROUNDCAM_ENABLE_RECORDING;
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Camera Constants.
    ///////////////////////////////////////////////////////////////////////////

    // ZedCam Basic Config.
    extern const sl::RESOLUTION ZED_BASE_RESOLUTION;
    extern const sl::UNIT ZED_MEASURE_UNITS;
    extern const sl::COORDINATE_SYSTEM ZED_COORD_SYSTEM;
    extern const sl::DEPTH_MODE ZED_DEPTH_MODE;
    extern const sl::VIEW ZED_RETRIEVE_VIEW;
    extern const bool ZED_SDK_VERBOSE;
    extern const bool ZED_SENSING_FILL;
    extern const float ZED_DEFAULT_MINIMUM_DISTANCE;
    extern const float ZED_DEFAULT_MAXIMUM_DISTANCE;
    extern const float ZED_DEFAULT_FLOOR_PLANE_ERROR;
    extern const int ZED_DEPTH_STABILIZATION;
    // ZedCam SVO Recording Config.
    extern const sl::SVO_COMPRESSION_MODE ZED_SVO_COMPRESSION;
    extern const int ZED_SVO_BITRATE;
    // ZedCam Positional Tracking Config.
    extern const sl::POSITIONAL_TRACKING_MODE ZED_POSETRACK_MODE;
    extern const bool ZED_POSETRACK_AREA_MEMORY;
    extern const bool ZED_POSETRACK_POSE_SMOOTHING;
    extern const bool ZED_POSETRACK_FLOOR_IS_ORIGIN;
    extern const bool ZED_POSETRACK_ENABLE_IMU_FUSION;
    extern const float ZED_POSETRACK_USABLE_DEPTH_MIN;
    extern const bool ZED_POSETRACK_USE_GRAVITY_ORIGIN;
    // ZedCam Spatial Mapping Config.
    extern const sl::SpatialMappingParameters::SPATIAL_MAP_TYPE ZED_MAPPING_TYPE;
    extern const float ZED_MAPPING_RANGE_METER;
    extern const float ZED_MAPPING_RESOLUTION_METER;
    extern const int ZED_MAPPING_MAX_MEMORY;
    extern const bool ZED_MAPPING_USE_CHUNK_ONLY;
    extern const int ZED_MAPPING_STABILITY_COUNTER;
    // ZedCam Object Detection Config.
    extern const bool ZED_OBJDETECTION_TRACK_OBJ;
    extern const bool ZED_OBJDETECTION_SEGMENTATION;
    extern const sl::OBJECT_FILTERING_MODE ZED_OBJDETECTION_FILTERING;
    extern const float ZED_OBJDETECTION_TRACKING_PREDICTION_TIMEOUT;
    extern const float ZED_OBJDETECTION_BATCH_RETENTION_TIME;
    extern const float ZED_OBJDETECTION_BATCH_LATENCY;
    // Zed Fusion Config.
    extern const sl::UNIT FUSION_MEASUREMENT_UNITS;
    extern const sl::COORDINATE_SYSTEM FUSION_COORD_SYSTEM;
    extern const bool FUSION_SDK_VERBOSE;
    extern const bool FUSION_ENABLE_GNSS_FUSION;

    // BasicCam Basic Config.
    extern const cv::InterpolationFlags BASICCAM_RESIZE_INTERPOLATION_METHOD;
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Camera Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Main ZED Camera.
    extern const int ZED_MAINCAM_RESOLUTIONX;
    extern const int ZED_MAINCAM_RESOLUTIONY;
    extern const int ZED_MAINCAM_FPS;
    extern const int ZED_MAINCAM_HORIZONTAL_FOV;
    extern const int ZED_MAINCAM_VERTICAL_FOV;
    extern const bool ZED_MAINCAM_EXPORT_SVO_RECORDING;
    extern const bool ZED_MAINCAM_EXPORT_SPATIAL_MAP;
    extern const bool ZED_MAINCAM_USE_GPU_MAT;
    extern const bool ZED_MAINCAM_USE_HALF_PRECISION_DEPTH;
    extern const bool ZED_MAINCAM_FUSION_MASTER;
    extern const int ZED_MAINCAM_FRAME_RETRIEVAL_THREADS;
    extern const int ZED_MAINCAM_SERIAL;

    // Left ZED Camera.
    extern const int ZED_LEFTCAM_RESOLUTIONX;
    extern const int ZED_LEFTCAM_RESOLUTIONY;
    extern const int ZED_LEFTCAM_FPS;
    extern const int ZED_LEFTCAM_HORIZONTAL_FOV;
    extern const int ZED_LEFTCAM_VERTICAL_FOV;
    extern const bool ZED_LEFTCAM_EXPORT_SVO_RECORDING;
    extern const bool ZED_LEFTCAM_EXPORT_SPATIAL_MAP;
    extern const bool ZED_LEFTCAM_USE_GPU_MAT;
    extern const bool ZED_LEFTCAM_USE_HALF_PRECISION_DEPTH;
    extern const bool ZED_LEFTCAM_FUSION_MASTER;
    extern const int ZED_LEFTCAM_FRAME_RETRIEVAL_THREADS;
    extern const int ZED_LEFTCAM_SERIAL;

    // Right ZED Camera.
    extern const int ZED_RIGHTCAM_RESOLUTIONX;
    extern const int ZED_RIGHTCAM_RESOLUTIONY;
    extern const int ZED_RIGHTCAM_FPS;
    extern const int ZED_RIGHTCAM_HORIZONTAL_FOV;
    extern const int ZED_RIGHTCAM_VERTICAL_FOV;
    extern const bool ZED_RIGHTCAM_EXPORT_SVO_RECORDING;
    extern const bool ZED_RIGHTCAM_EXPORT_SPATIAL_MAP;
    extern const bool ZED_RIGHTCAM_USE_GPU_MAT;
    extern const bool ZED_RIGHTCAM_USE_HALF_PRECISION_DEPTH;
    extern const bool ZED_RIGHTCAM_FUSION_MASTER;
    extern const int ZED_RIGHTCAM_FRAME_RETRIEVAL_THREADS;
    extern const int ZED_RIGHTCAM_SERIAL;

    // Ground Basic Cam.
    extern const int BASICCAM_GROUNDCAM_RESOLUTIONX;
    extern const int BASICCAM_GROUNDCAM_RESOLUTIONY;
    extern const int BASICCAM_GROUNDCAM_FPS;
    extern const int BASICCAM_GROUNDCAM_HORIZONTAL_FOV;
    extern const int BASICCAM_GROUNDCAM_VERTICAL_FOV;
    extern const int BASICCAM_GROUNDCAM_FRAME_RETRIEVAL_THREADS;
    extern const int BASICCAM_GROUNDCAM_INDEX;
    extern const PIXEL_FORMATS BASICCAM_GROUNDCAM_PIXELTYPE;
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// ArUco Vision Constants.
    ///////////////////////////////////////////////////////////////////////////

    // OpenCV ArUco detection config.
    extern const cv::aruco::PredefinedDictionaryType ARUCO_DICTIONARY;
    extern const float ARUCO_TAG_SIDE_LENGTH;
    extern const double ARUCO_PIXEL_THRESHOLD;
    extern const double ARUCO_PIXEL_THRESHOLD_MAX_VALUE;
    extern const cv::Mat ARUCO_SHARPEN_KERNEL_FAST;
    extern const cv::Mat ARUCO_SHARPEN_KERNEL_EXTRA;
    extern const cv::Mat ARUCO_EDGE_KERNEL;
    extern const tracking::TrackerType ARUCO_BBOX_TRACKER_TYPE;
    // NOTE: These next to constants are how we set the min amount of time the tracker needs to be seen before being considered a valid detection and how long to wait
    // NOTE: before considering a tracker lost. ARUCO_MIN_LIFETIME_THRESHOLD - ARUCO_BBOX_TRACKER_LOST_TIMEOUT is the time the tag needs to be seen before being
    // NOTE: considered valid.
    extern const double ARUCO_MIN_LIFETIME_THRESHOLD;
    extern const double ARUCO_BBOX_TRACKER_LOST_TIMEOUT;
    extern const double ARUCO_BBOX_TRACKER_IOU_MATCH_THRESHOLD;
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Tag Detection Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Main ZED Camera.
    extern const int TAGDETECT_MAINCAM_DATA_RETRIEVAL_THREADS;
    extern const int TAGDETECT_MAINCAM_CORNER_REFINE_MAX_ITER;
    extern const int TAGDETECT_MAINCAM_CORNER_REFINE_METHOD;
    extern const bool TAGDETECT_MAINCAM_DETECT_INVERTED_MARKER;
    extern const int TAGDETECT_MAINCAM_MARKER_BORDER_BITS;
    extern const bool TAGDETECT_MAINCAM_USE_ARUCO3_DETECTION;
    extern const int TAGDETECT_MAINCAM_MAX_FPS;
    extern const bool TAGDETECT_MAINCAM_ENABLE_TORCH;
    extern const std::string TAGDETECT_MAINCAM_TORCH_MODEL;
    extern const float TAGDETECT_MAINCAM_TORCH_CONFIDENCE;
    extern const float TAGDETECT_MAINCAM_TORCH_NMS_THRESH;

    // Left ZED Camera.
    extern const int TAGDETECT_LEFTCAM_DATA_RETRIEVAL_THREADS;
    extern const int TAGDETECT_LEFTCAM_CORNER_REFINE_MAX_ITER;
    extern const int TAGDETECT_LEFTCAM_CORNER_REFINE_METHOD;
    extern const bool TAGDETECT_LEFTCAM_DETECT_INVERTED_MARKER;
    extern const int TAGDETECT_LEFTCAM_MARKER_BORDER_BITS;
    extern const bool TAGDETECT_LEFTCAM_USE_ARUCO3_DETECTION;
    extern const int TAGDETECT_LEFTCAM_MAX_FPS;
    extern const bool TAGDETECT_LEFTCAM_ENABLE_TORCH;
    extern const std::string TAGDETECT_LEFTCAM_TORCH_MODEL;
    extern const float TAGDETECT_LEFTCAM_TORCH_CONFIDENCE;
    extern const float TAGDETECT_LEFTCAM_TORCH_NMS_THRESH;

    // Right ZED Camera.
    extern const int TAGDETECT_RIGHTCAM_DATA_RETRIEVAL_THREADS;
    extern const int TAGDETECT_RIGHTCAM_CORNER_REFINE_MAX_ITER;
    extern const int TAGDETECT_RIGHTCAM_CORNER_REFINE_METHOD;
    extern const bool TAGDETECT_RIGHTCAM_DETECT_INVERTED_MARKER;
    extern const int TAGDETECT_RIGHTCAM_MARKER_BORDER_BITS;
    extern const bool TAGDETECT_RIGHTCAM_USE_ARUCO3_DETECTION;
    extern const int TAGDETECT_RIGHTCAM_MAX_FPS;
    extern const bool TAGDETECT_RIGHTCAM_ENABLE_TORCH;
    extern const std::string TAGDETECT_RIGHTCAM_TORCH_MODEL;
    extern const float TAGDETECT_RIGHTCAM_TORCH_CONFIDENCE;
    extern const float TAGDETECT_RIGHTCAM_TORCH_NMS_THRESH;

    ///////////////////////////////////////////////////////////////////////////
    //// Object Detection Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Main ZED Camera.
    extern const int OBJECTDETECT_MAINCAM_DATA_RETRIEVAL_THREADS;

    // Left Side Cam.
    extern const int OBJECTDETECT_LEFTCAM_DATA_RETRIEVAL_THREADS;

    // Right Side Cam.
    extern const int OBJECTDETECT_RIGHTCAM_DATA_RETRIEVAL_THREADS;
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// State Constants.
    ///////////////////////////////////////////////////////////////////////////

    // Handler.
    extern const int STATEMACHINE_MAX_IPS;
    extern const double STATEMACHINE_ZED_REALIGN_THRESHOLD;

    // Approaching Marker State
    extern const double APPROACH_MARKER_MOTOR_POWER;
    extern const double APPROACH_MARKER_PROXIMITY_THRESHOLD;
    extern const double APPROACH_MARKER_VISION_DISTANCE;
    extern const double APPROACH_MARKER_LOST_GIVE_UP_TIME;

    // Stuck State
    extern const double STUCK_CHECK_INTERVAL;
    extern const unsigned int STUCK_CHECK_ATTEMPTS;
    extern const double STUCK_CHECK_ROT_THRESH;
    extern const double STUCK_CHECK_VEL_THRESH;
    extern const double STUCK_SAME_POINT_PROXIMITY;
    extern const double STUCK_HEADING_ALIGN_TIMEOUT;
    extern const double STUCK_ALIGN_DEGREES;
    extern const double STUCK_ALIGN_TOLERANCE;

    // Reverse State.
    extern const double REVERSE_MOTOR_POWER;
    extern const double REVERSE_DISTANCE;
    extern const double REVERSE_TIMEOUT_PER_METER;
    extern const bool REVERSE_MAINTAIN_HEADING;

    // Search Pattern State
    extern const double SEARCH_MOTOR_POWER;
    extern const double SEARCH_ANGULAR_STEP_DEGREES;
    extern const double SEARCH_SPIRAL_SPACING;
    extern const double SEARCH_ZIGZAG_SPACING;
    extern const double SEARCH_WAYPOINT_PROXIMITY;

    // Navigating State.
    extern const double NAVIGATING_MOTOR_POWER;
    extern const double NAVIGATING_REACHED_GOAL_RADIUS;
    extern const bool NAVIGATING_VERIFY_POSITION;

    // Avoidance State.
    extern const double AVOIDANCE_STATE_MOTOR_POWER;

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
    extern const double STANLEY_CROSSTRACK_CONTROL_GAIN;
    extern const double STANLEY_DIST_TO_FRONT_AXLE;
    extern const double STANLEY_STEERING_ANGLE_LIMIT;
    extern const int STANLEY_PREDICTION_HORIZON;
    extern const double STANLEY_PREDICTION_TIME_STEP;

    // ASTAR config.
    extern const double ASTAR_AVOIDANCE_MULTIPLIER;
    extern const double ASTAR_MAX_SEARCH_GRID;
    extern const double ASTAR_MAX_SEARCH_TIME;
    extern const double ASTAR_NODE_SIZE;

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Driver Constants.
    ///////////////////////////////////////////////////////////////////////////

    // NavBoard.
    extern const double NAVBOARD_MAX_GPS_DATA_AGE;
    extern const double NAVBOARD_MAX_COMPASS_DATA_AGE;

    ///////////////////////////////////////////////////////////////////////////

}    // namespace constants

#endif    // AUTONOMY_CONSTANTS_H
