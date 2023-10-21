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

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

#include "./interfaces/Camera.hpp"

#include "../external/nlohmann/json.hpp"

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
    // Check for USB drive
    std::filesystem::path pathToDrive = "/dev/bus/usb/AutonomyConfigs";
    if (!(std::filesystem::exists(pathToDrive)))    // no USB drive
    {
        ///////////////////////////////////////////////////////////////////////////
        //// Drive Constants.
        ///////////////////////////////////////////////////////////////////////////

        // Power constants.
        const float DRIVE_MAX_POWER  = 1.0;
        const float DRIVE_MIN_POWER  = -1.0;
        const float DRIVE_MAX_EFFORT = 0.5;
        const float DRIVE_MIN_EFFORT = -0.5;

        // Control constants.
        const double DRIVE_PID_PROPORTIONAL       = 0.1;      // The proportional gain for the controller used to point the rover at a goal heading during navigation.
        const double DRIVE_PID_INTEGRAL           = 0.01;     // The integral gain for the controller used to point the rover at a goal heading during navigation.
        const double DRIVE_PID_DERIVATIVE         = 0.0;      // The derivative gain for the controller used to point the rover at a goal heading during navigation.
        const double DRIVE_PID_MAX_ERROR_PER_ITER = 100;      // The max allowable error the controller will see per iteration. This is on degrees from setpoint.
        const double DRIVE_PID_MAX_INTEGRAL_TERM  = 0.3;      // The max effort the I term is allowed to contribute.
        const double DRIVE_PID_MAX_OUTPUT_EFFORT  = 0.5;      // The max effort the entire PID controller is allowed to output. Range is within DRIVE_MAX/MIN_POWER.
        const double DRIVE_PID_MAX_RAMP_RATE      = 0.4;      // The max ramp rate of the output of the PID controller.
        const double DRIVE_PID_OUTPUT_FILTER      = 0.0;      // Larger values will filter out large spikes or oscillations. 0.1 is a good starting point.
        const bool DRIVE_PID_OUTPUT_REVERSED      = false;    // Negates the output of the PID controller.
        const bool DRIVE_SQUARE_CONTROL_INPUTS    = false;    // This is used by the DifferentialDrive algorithms. True makes fine inputs smoother, but less responsive.
        const bool DRIVE_CURVATURE_KINEMATICS_ALLOW_TURN_WHILE_STOPPED = true;    // This enabled turning in-place when using curvature drive control.
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
        const bool ZED_SENSING_FILL                  = false;    // True provides a depth map with a Z value for every pixel (X, Y) in the left image. Slower and worse.
        const float ZED_DEFAULT_MINIMUM_DISTANCE     = 0.2;      // Minimum distance in ZED_MEASURE_UNITS to report from depth measurement.
        const float ZED_DEFAULT_MAXIMUM_DISTANCE     = 40.0;     // Maximum distance in ZED_MEASURE_UNITS to report from depth measurement.
        const int ZED_DEPTH_STABILIZATION = 1;    // This parameter controls a stabilization filter that reduces oscillations in depth map. In the range [0-100]
        // ZedCam Positional Tracking Config.
        const sl::POSITIONAL_TRACKING_MODE ZED_POSETRACK_MODE = sl::POSITIONAL_TRACKING_MODE::STANDARD;    // Positional tracking accuracy.
        const bool ZED_POSETRACK_AREA_MEMORY                  = true;     // Enabled camera to remember its surroundings for better positioning. Uses more resources.
        const bool ZED_POSETRACK_POSE_SMOOTHING               = false;    // Smooth pose correction for small drift. Decreases overall precision for small movements.
        const bool ZED_POSETRACK_FLOOR_IS_ORIGIN              = true;     // Sets the floor plane as origin for tracking. This turns on floor plane detection temporarily.
        const bool ZED_POSETRACK_ENABLE_IMU_FUSION            = true;     // Allows ZED to use both optical odometry and IMU data for pose tracking.
        const float ZED_POSETRACK_USABLE_DEPTH_MIN   = 0.2;     // Minimum depth used for pose tracking, useful if a static object is partial in view of the camera.
        const float ZED_POSETRACK_USE_GRAVITY_ORIGIN = true;    // Override 2 of the 3 rotations from initial_world_transform using the IMU.
        // ZedCam Spatial Mapping Config.
        const sl::SpatialMappingParameters::SPATIAL_MAP_TYPE ZED_MAPPING_TYPE = sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;    // Mesh or point cloud output.
        const float ZED_MAPPING_RANGE_METER                                   = 20.0;    // The max range in meters that the ZED cameras should use for mapping. 0 = auto.
        const float ZED_MAPPING_RESOLUTION_METER                              = 0.01;    // The approx goal precision for spatial mapping in METERS. Higher = Faster.
        const int ZED_MAPPING_MAX_MEMORY                                      = 4096;    // The max amount of CPU RAM (MB) that can be allocated for spatial mapping.
        const bool ZED_MAPPING_USE_CHUNK_ONLY   = true;    // Only update chunks that have probably changed or have new data. Faster, less accurate.
        const int ZED_MAPPING_STABILITY_COUNTER = 4;       // Number of times that a point should be seen before adding to mesh.
        // ZedCam Object Detection Config.
        const bool ZED_OBJDETECTION_IMG_SYNC     = true;     // True = Run detection for every frame. False = Run detection async, can lead to delayed detections.
        const bool ZED_OBJDETECTION_TRACK_OBJ    = true;     // Whether or not to enable object tracking in the scene. Attempts to maintain OBJ UUIDs.
        const bool ZED_OBJDETECTION_SEGMENTATION = false;    // Use depth data to compute the segmentation for an object. (exact outline/shape)
        const sl::OBJECT_FILTERING_MODE ZED_OBJDETECTION_FILTERING = sl::OBJECT_FILTERING_MODE::NMS3D_PER_CLASS;    // Custom detection, use PER_CLASS or NONE.
        const float ZED_OBJDETECTION_TRACKING_PREDICTION_TIMEOUT   = 0.5;    // 0-1 second. Timeout to keep guessing object position when not in sight.
        const float ZED_OBJDETECTION_BATCH_RETENTION_TIME          = 240;    // The time in seconds to search for an object UUID before expiring the object.
        const float ZED_OBJDETECTION_BATCH_LATENCY = 2;    // Short latency will limit the search for previously seen object IDs but will be closer to real time output.

        // BasicCam Basic Config.
        const cv::InterpolationFlags BASICCAM_RESIZE_INTERPOLATION_METHOD =
            cv::InterpolationFlags::INTER_LINEAR;    // The algorithm used to fill in pixels when resizing.
        ///////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////
        //// Camera Handler Adjustments.
        ///////////////////////////////////////////////////////////////////////////

        // Main ZED Camera.
        const int ZED_MAINCAM_RESOLUTIONX               = 1280;        // The horizontal pixel resolution to resize the maincam images to.
        const int ZED_MAINCAM_RESOLUTIONY               = 720;         // The vertical pixel resolution to resize the maincam images to.
        const int ZED_MAINCAM_FPS                       = 60;          // The FPS to use for the maincam.
        const int ZED_MAINCAM_HORIZONTAL_FOV            = 110;         // The horizontal FOV of the camera. Useful for future calculations.
        const int ZED_MAINCAM_VERTICAL_FOV              = 70;          // The vertical FOV of the camera. Useful for future calculations.
        const bool ZED_MAINCAM_USE_GPU_MAT              = false;       // Whether or not to use CPU or GPU memory mats. GPU memory transfer/operations are faster.
        const bool ZED_MAINCAM_USE_HALF_PRECISION_DEPTH = true;        // Whether of not to use float32 or unsigned short (16) for depth measure.
        const int ZED_MAINCAM_FRAME_RETRIEVAL_THREADS   = 20;          // The number of threads allocated to the threadpool for performing frame copies to other threads.
        const int ZED_MAINCAM_SERIAL                    = 31237348;    // The serial number of the camera. Set to 0 to open the next available one.

        // Left Side Cam.
        const int BASICCAM_LEFTCAM_RESOLUTIONX             = 1280;    // The horizontal pixel resolution to resize the maincam images to.
        const int BASICCAM_LEFTCAM_RESOLUTIONY             = 720;     // The vertical pixel resolution to resize the maincam images to.
        const int BASICCAM_LEFTCAM_FPS                     = 30;      // The FPS to use for the maincam.
        const int BASICCAM_LEFTCAM_HORIZONTAL_FOV          = 110;     // The horizontal FOV of the camera. Useful for future calculations.
        const int BASICCAM_LEFTCAM_VERTICAL_FOV            = 70;      // The vertical FOV of the camera. Useful for future calculations.
        const int BASICCAM_LEFTCAM_FRAME_RETRIEVAL_THREADS = 10;      // The number of threads allocated to the threadpool for performing frame copies to other threads.
        const int BASICCAM_LEFTCAM_INDEX                   = 0;       // The /dev/video index of the camera.
        const PIXEL_FORMATS BASICCAM_LEFTCAM_PIXELTYPE     = PIXEL_FORMATS::eBGR;    // The pixel layout of the camera.

        // Right Side Cam.
        const int BASICCAM_RIGHTCAM_RESOLUTIONX             = 1280;    // The horizontal pixel resolution to resize the maincam images to.
        const int BASICCAM_RIGHTCAM_RESOLUTIONY             = 720;     // The vertical pixel resolution to resize the maincam images to.
        const int BASICCAM_RIGHTCAM_FPS                     = 30;      // The FPS to use for the maincam.
        const int BASICCAM_RIGHTCAM_HORIZONTAL_FOV          = 110;     // The horizontal FOV of the camera. Useful for future calculations.
        const int BASICCAM_RIGHTCAM_VERTICAL_FOV            = 70;      // The vertical FOV of the camera. Useful for future calculations.
        const int BASICCAM_RIGHTCAM_FRAME_RETRIEVAL_THREADS = 10;      // The number of threads allocated to the threadpool for performing frame copies to other threads.
        const int BASICCAM_RIGHTCAM_INDEX                   = 1;       // The /dev/video index of the camera.
        const PIXEL_FORMATS BASICCAM_RIGHTCAM_PIXELTYPE     = PIXEL_FORMATS::eBGR;    // The pixel layout of the camera.
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
        const int TAGDETECT_MAINCAM_DATA_RETRIEVAL_THREADS  = 5;     // The number of threads allocated to the threadpool for performing data copies to other threads.
        const int TAGDETECT_MAINCAM_CORNER_REFINE_MAX_ITER  = 30;    // The maximum number of iterations to run corner refinement on the image.
        const int TAGDETECT_MAINCAM_CORNER_REFINE_METHOD    = cv::aruco::CORNER_REFINE_NONE;    // Algorithm used to refine tag corner pixels.
        const bool TAGDETECT_MAINCAM_DETECT_INVERTED_MARKER = true;                             // Whether or not to detector upside-down tags.
        const int TAGDETECT_MAINCAM_MARKER_BORDER_BITS      = 1;                                // This number of bits on the border. A bit is one unit square of the tag.
        const bool TAGDETECT_MAINCAM_USE_ARUCO3_DETECTION   = false;                            // Whether or not to use the newer and faster Aruco detection strategy.

        // Left Side Cam.
        const int TAGDETECT_LEFTCAM_DATA_RETRIEVAL_THREADS  = 5;     // The number of threads allocated to the threadpool for performing data copies to other threads.
        const int TAGDETECT_LEFTCAM_CORNER_REFINE_MAX_ITER  = 30;    // The maximum number of iterations to run corner refinement on the image.
        const int TAGDETECT_LEFTCAM_CORNER_REFINE_METHOD    = cv::aruco::CORNER_REFINE_NONE;    // Algorithm used to refine tag corner pixels.
        const bool TAGDETECT_LEFTCAM_DETECT_INVERTED_MARKER = true;                             // Whether or not to detector upside-down tags.
        const int TAGDETECT_LEFTCAM_MARKER_BORDER_BITS      = 1;                                // This number of bits on the border. A bit is one unit square of the tag.
        const bool TAGDETECT_LEFTCAM_USE_ARUCO3_DETECTION   = true;                             // Whether or not to use the newer and faster Aruco detection strategy.

        // Right Side Cam.
        const int TAGDETECT_RIGHTCAM_DATA_RETRIEVAL_THREADS  = 5;     // The number of threads allocated to the threadpool for performing data copies to other threads.
        const int TAGDETECT_RIGHTCAM_CORNER_REFINE_MAX_ITER  = 30;    // The maximum number of iterations to run corner refinement on the image.
        const int TAGDETECT_RIGHTCAM_CORNER_REFINE_METHOD    = cv::aruco::CORNER_REFINE_NONE;    // Algorithm used to refine tag corner pixels.
        const bool TAGDETECT_RIGHTCAM_DETECT_INVERTED_MARKER = true;                             // Whether or not to detector upside-down tags.
        const int TAGDETECT_RIGHTCAM_MARKER_BORDER_BITS      = 1;        // This number of bits on the border. A bit is one unit square of the tag.
        const bool TAGDETECT_RIGHTCAM_USE_ARUCO3_DETECTION   = false;    // Whether or not to use the newer and faster Aruco detection strategy.
        ///////////////////////////////////////////////////////////////////////////
    }
    else
    {
        nlohmann::json jsonData;
        std::string filePath = "/dev/bus/usb/AutonomyConfigs/AutonomyConstants.json";
        std::ifstream fin(filePath);

        if (!fin)
        {
            std::cerr << "Couldn't open AutonomyConstants.json file" << std::endl;
        }

        try
        {
            fin >> jsonData;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error parsing AutonomyConstants.json" << std::endl;
        }

        ///////////////////////////////////////////////////////////////////////////
        //// Drive Constants.
        ///////////////////////////////////////////////////////////////////////////
        const int MAX_DRIVE_POWER = jsonData["DriveConstants"]["max_drive_power"];
        const int MIN_DRIVE_POWER = jsonData["DriveConstants"]["min_drive_power"];

        ///////////////////////////////////////////////////////////////////////////
        //// Camera Constants.
        ///////////////////////////////////////////////////////////////////////////

        // ZedCam Basic Config.
        const sl::RESOLUTION ZED_BASE_RESOLUTION     = jsonData["CameraConstants"]["zed_base_resolution"];    // The base resolution to open the all cameras with.
        const sl::UNIT ZED_MEASURE_UNITS             = jsonData["CameraConstants"]["zed_measure_units"];      // The base measurement unit to use for depth.
        const sl::COORDINATE_SYSTEM ZED_COORD_SYSTEM = jsonData["CameraConstants"]["zed_coord_system"];       // Coordinate system to use for measurements.
        const sl::DEPTH_MODE ZED_DEPTH_MODE = jsonData["CameraConstants"]["zed_retrieve_view"];    // The measurement accuracy for depth. NEURAL is by far the best.
        const sl::VIEW ZED_RETRIEVE_VIEW    = jsonData["CameraConstants"]["zed_sensing_fill"];     // The eye to retrieve regular and depth images from.
        const bool ZED_SENSING_FILL =
            jsonData["CameraConstants"]
                    ["zed_default_min_distance"];    // True provides a depth map with a Z value for every pixel (X, Y) in the left image. Slower and worse.
        const float ZED_DEFAULT_MINIMUM_DISTANCE =
            jsonData["CameraConstants"]["zed_default_max_distance"];    // Minimum distance in ZED_MEASURE_UNITS to report from depth measurement.
        const float ZED_DEFAULT_MAXIMUM_DISTANCE =
            jsonData["CameraConstants"]["zed_depth_stabilization"];     // Maximum distance in ZED_MEASURE_UNITS to report from depth measurement.
        const int ZED_DEPTH_STABILIZATION = 1;    // This parameter controls a stabilization filter that reduces oscillations in depth map. In the range [0-100]
        // ZedCam Positional Tracking Config.
        const sl::POSITIONAL_TRACKING_MODE ZED_POSETRACK_MODE = jsonData["CameraConstants"]["zed_posetrack_mode"];    // Positional tracking accuracy.
        const bool ZED_POSETRACK_AREA_MEMORY =
            jsonData["CameraConstants"]["zed_posetrack_area_memory"];       // Enabled camera to remember its surroundings for better positioning. Uses more resources.
        const bool ZED_POSETRACK_POSE_SMOOTHING =
            jsonData["CameraConstants"]["zed_posetrack_pose_smoothing"];    // Smooth pose correction for small drift. Decreases overall precision for small movements.
        const bool ZED_POSETRACK_FLOOR_IS_ORIGIN =
            jsonData["CameraConstants"]
                    ["zed_posetrack_floor_is_origin"];    // Sets the floor plane as origin for tracking. This turns on floor plane detection temporarily.
        const bool ZED_POSETRACK_ENABLE_IMU_FUSION =
            jsonData["CameraConstants"]["zed_posetrack_enable_imu_fusion"];    // Allows ZED to use both optical odometry and IMU data for pose tracking.
        const float ZED_POSETRACK_USABLE_DEPTH_MIN =
            jsonData["CameraConstants"]
                    ["zed_posetrack_usable_depth_min"];    // Minimum depth used for pose tracking, useful if a static object is partial in view of the camera.
        const float ZED_POSETRACK_USE_GRAVITY_ORIGIN =
            jsonData["CameraConstants"]["zed_posetrack_use_gravity_origin"];    // Override 2 of the 3 rotations from initial_world_transform using the IMU.
        // ZedCam Spatial Mapping Config.
        const sl::SpatialMappingParameters::SPATIAL_MAP_TYPE ZED_MAPPING_TYPE = jsonData["CameraConstants"]["zed_mapping_type"];    // Mesh or point cloud output.
        const float ZED_MAPPING_RANGE_METER =
            jsonData["CameraConstants"]["zed_mapping_range_meter"];          // The max range in meters that the ZED cameras should use for mapping. 0 = auto.
        const float ZED_MAPPING_RESOLUTION_METER =
            jsonData["CameraConstants"]["zed_mapping_resolution_meter"];     // The approx goal precision for spatial mapping in METERS. Higher = Faster.
        const int ZED_MAPPING_MAX_MEMORY =
            jsonData["CameraConstants"]["zed_mapping_max_memory"];           // The max amount of CPU RAM (MB) that can be allocated for spatial mapping.
        const bool ZED_MAPPING_USE_CHUNK_ONLY =
            jsonData["CameraConstants"]["zed_mapping_use_chunk_only"];       // Only update chunks that have probably changed or have new data. Faster, less accurate.
        const int ZED_MAPPING_STABILITY_COUNTER =
            jsonData["CameraConstants"]["zed_mapping_stability_counter"];    // Number of times that a point should be seen before adding to mesh.
        // ZedCam Object Detection Config.
        const bool ZED_OBJDETECTION_IMG_SYNC =
            jsonData["CameraConstants"]
                    ["zed_objdetection_img_sync"];    // True = Run detection for every frame. False = Run detection async, can lead to delayed detections.
        const bool ZED_OBJDETECTION_TRACK_OBJ =
            jsonData["CameraConstants"]["zed_objdetection_track_obj"];       // Whether or not to enable object tracking in the scene. Attempts to maintain OBJ UUIDs.
        const bool ZED_OBJDETECTION_SEGMENTATION =
            jsonData["CameraConstants"]["zed_objdetection_segmentation"];    // Use depth data to compute the segmentation for an object. (exact outline/shape)
        const sl::OBJECT_FILTERING_MODE ZED_OBJDETECTION_FILTERING =
            jsonData["CameraConstants"]["zed_objdetection_filtering"];       // Custom detection, use PER_CLASS or NONE.
        const float ZED_OBJDETECTION_TRACKING_PREDICTION_TIMEOUT =
            jsonData["CameraConstants"]["zed_objdetection_tracking_prediction_timeout"];    // 0-1 second. Timeout to keep guessing object position when not in sight.
        const float ZED_OBJDETECTION_BATCH_RETENTION_TIME =
            jsonData["CameraConstants"]["zed_objdetection_batch_retention_time"];    // The time in seconds to search for an object UUID before expiring the object.
        const float ZED_OBJDETECTION_BATCH_LATENCY =
            jsonData["CameraConstants"]
                    ["zed_objdetection_batch_latency"];    // Short latency will limit the search for previously seen object IDs but will be closer to real time output.

        // BasicCam Basic Config.
        const cv::InterpolationFlags BASICCAM_RESIZE_INTERPOLATION_METHOD =
            jsonData["CameraConstants"]["basiccam_resize_interpolation_method"];    // The algorithm used to fill in pixels when resizing.
        ///////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////
        //// Camera Handler Adjustments.
        ///////////////////////////////////////////////////////////////////////////

        // MainCam
        const int ZED_MAINCAM_RESOLUTIONX =
            jsonData["CameraHandlerAdjustments"]["zed_maincam_resolutionx"];                    // The horizontal pixel resolution to resize the maincam images to.
        const int ZED_MAINCAM_RESOLUTIONY =
            jsonData["CameraHandlerAdjustments"]["zed_maincam_resolutiony"];                    // The vertical pixel resolution to resize the maincam images to.
        const int ZED_MAINCAM_FPS = jsonData["CameraHandlerAdjustments"]["zed_maincam_fps"];    // The FPS to use for the maincam.
        const int ZED_MAINCAM_HORIZONTAL_FOV =
            jsonData["CameraHandlerAdjustments"]["zed_maincam_horizontal_fov"];                 // The horizontal FOV of the camera. Useful for future calculations.
        const int ZED_MAINCAM_VERTICAL_FOV =
            jsonData["CameraHandlerAdjustments"]["zed_maincam_vertical_fov"];                   // The vertical FOV of the camera. Useful for future calculations.
        const bool ZED_MAINCAM_USE_GPU_MAT =
            jsonData["CameraHandlerAdjustments"]
                    ["zed_maincam_use_gpu_mat"];    // Whether or not to use CPU or GPU memory mats. GPU memory transfer/operations are faster.
        const bool ZED_MAINCAM_USE_HALF_PRECISION_DEPTH =
            jsonData["CameraHandlerAdjustments"]["zed_maincam_use_half_precision_depth"];    // Whether of not to use float32 or unsigned short (16) for depth measure.
        const int ZED_MAINCAM_FRAME_RETRIEVAL_THREADS =
            jsonData["CameraHandlerAdjustments"]
                    ["zed_maincam_frame_retrieval_threads"];    // The number of threads allocated to the threadpool for performing frame copies to other threads.
        const int ZED_MAINCAN_SERIAL = jsonData["CameraHandlerAdjustments"]["zed_maincam_serial"];    // The serial number of the camera.

        // Left Side Cam.
        const int BASICCAM_LEFTCAM_RESOLUTIONX =
            jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_resolutionx"];    // The horizontal pixel resolution to resize the maincam images to.
        const int BASICCAM_LEFTCAM_RESOLUTIONY =
            jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_resolutiony"];    // The vertical pixel resolution to resize the maincam images to.
        const int BASICCAM_LEFTCAM_FPS = jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_fps"];    // The FPS to use for the maincam.
        const int BASICCAM_LEFTCAM_HORIZONTAL_FOV =
            jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_horizontal_fov"];    // The horizontal FOV of the camera. Useful for future calculations.
        const int BASICCAM_LEFTCAM_VERTICAL_FOV =
            jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_vertical_fov"];      // The vertical FOV of the camera. Useful for future calculations.
        const int BASICCAM_LEFTCAM_FRAME_RETRIEVAL_THREADS =
            jsonData["CameraHandlerAdjustments"]
                    ["basiccam_leftcam_frame_retrieval_threads"];    // The number of threads allocated to the threadpool for performing frame copies to other threads.
        const int BASICCAM_LEFTCAM_INDEX               = jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_index"];        // The /dev/video index of the camera.
        const PIXEL_FORMATS BASICCAM_LEFTCAM_PIXELTYPE = jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_pixeltype"];    // The pixel layout of the camera.
        ///////////////////////////////////////////////////////////////////////////
    }
}    // namespace constants

#endif    // CONSTS_H
