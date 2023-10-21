/******************************************************************************
 * @brief Sets up functions and classes used project wide.
 *
 * @file AutonomyGlobals.cpp
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "AutonomyGlobals.h"
#include "../external/nlohmann/json.hpp"
#include "AutonomyLogging.h"
#include <filesystem>

/******************************************************************************
 * @brief Namespace containing all global types/structs that will be used project
 *      wide and ARE NOT SPECIFIC TO A CERTAIN CLASS.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
namespace globals
{
    /////////////////////////////////////////
    // Forward declarations for namespace variables and objects.
    /////////////////////////////////////////

    // Versioning:
    IdentitySoftware g_pIdentifySoftware;

    // Camera handler:
    CameraHandler* g_pCameraHandler;

    // Tag Detection Handler:
    TagDetectionHandler* g_pTagDetectionHandler;

    // Drivers:
    DriveBoard g_pDriveBoardInterface;
    MultimediaBoard g_pMultimediaBoard;
    NavigationBoard g_pNavigationBoard;

    void adjustAutonomyConstants()
    {
        // Check for USB drive
        std::filesystem::path pathToDrive = "/media/auto/AutonomyConfigs";
        if (std::filesystem::exists(pathToDrive))    // no USB drive
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

            // Power ants.
            DRIVE_MAX_POWER  = jsonData["DriveConstants"]["drive_max_power"];
            DRIVE_MIN_POWER  = jsonData["DriveConstants"]["drive_min_power"];
            DRIVE_MAX_EFFORT = jsonData["DriveConstants"]["drive_max_effort"];
            DRIVE_MIN_EFFORT = jsonData["DriveConstants"]["drive_min_effort"];

            // Control  ants.
            DRIVE_PID_PROPORTIONAL =
                jsonData["DriveConstants"]
                        ["drive_pid_proportional"];    // The proportional gain for the controller used to point the rover at a goal heading during navigation.
            DRIVE_PID_INTEGRAL =
                jsonData["DriveConstants"]["drive_pid_integral"];    // The integral gain for the controller used to point the rover at a goal heading during navigation.
            DRIVE_PID_DERIVATIVE =
                jsonData["DriveConstants"]
                        ["drive_pid_derivative"];    // The derivative gain for the controller used to point the rover at a goal heading during navigation.
            DRIVE_PID_MAX_ERROR_PER_ITER =
                jsonData["DriveConstants"]
                        ["drive_pid_max_error_per_iter"];    // The max allowable error the controller will see per iteration. This is on degrees from setpoint.
            DRIVE_PID_MAX_INTEGRAL_TERM = jsonData["DriveConstants"]["drive_pid_max_integral_term"];    // The max effort the I term is allowed to contribute.
            DRIVE_PID_MAX_OUTPUT_EFFORT =
                jsonData["DriveConstants"]
                        ["drive_pid_max_output_effort"];    // The max effort the entire PID controller is allowed to output. Range is within DRIVE_MAX/MIN_POWER.
            DRIVE_PID_MAX_RAMP_RATE = jsonData["DriveConstants"]["drive_pid_max_ramp_rate"];    // The max ramp rate of the output of the PID controller.
            DRIVE_PID_OUTPUT_FILTER =
                jsonData["DriveConstants"]["drive_pid_output_filter"];    // Larger values will filter out large spikes or oscillations. 0.1 is a good starting point.
            DRIVE_PID_OUTPUT_REVERSED = jsonData["DriveConstants"]["drive_pid_output_reversed"];    // Negates the output of the PID controller.
            DRIVE_SQUARE_CONTROL_INPUTS =
                jsonData["DriveConstants"]
                        ["drive_square_control_inputs"];    // This is used by the DifferentialDrive algorithms. True makes fine inputs smoother, but less responsive.
            DRIVE_CURVATURE_KINEMATICS_ALLOW_TURN_WHILE_STOPPED =
                jsonData["DriveConstants"]["drive_curvature_kinematics_allow_turn_while_stopped"];    // This enabled turning in-place when using curvature drive control.

            ///////////////////////////////////////////////////////////////////////////
            //// Camera Constants.
            ///////////////////////////////////////////////////////////////////////////

            // ZedCam Basic Config.
            ZED_BASE_RESOLUTION = jsonData["CameraConstants"]["zed_base_resolution"];    // The base resolution to open the all cameras with.
            ZED_MEASURE_UNITS   = jsonData["CameraConstants"]["zed_measure_units"];      // The base measurement unit to use for depth.
            ZED_COORD_SYSTEM    = jsonData["CameraConstants"]["zed_coord_system"];       // Coordinate system to use for measurements.
            ZED_DEPTH_MODE      = jsonData["CameraConstants"]["zed_retrieve_view"];      // The measurement accuracy for depth. NEURAL is by far the best.
            ZED_RETRIEVE_VIEW   = jsonData["CameraConstants"]["zed_sensing_fill"];       // The eye to retrieve regular and depth images from.
            ZED_SENSING_FILL =
                jsonData["CameraConstants"]
                        ["zed_default_min_distance"];    // True provides a depth map with a Z value for every pixel (X, Y) in the left image. Slower and worse.
            ZED_DEFAULT_MINIMUM_DISTANCE =
                jsonData["CameraConstants"]["zed_default_max_distance"];    // Minimum distance in ZED_MEASURE_UNITS to report from depth measurement.
            ZED_DEFAULT_MAXIMUM_DISTANCE =
                jsonData["CameraConstants"]["zed_depth_stabilization"];     // Maximum distance in ZED_MEASURE_UNITS to report from depth measurement.
            ZED_DEPTH_STABILIZATION = 1;    // This parameter controls a stabilization filter that reduces oscillations in depth map. In the range [0-100]
            // ZedCam Positional Tracking Config.
            ZED_POSETRACK_MODE = jsonData["CameraConstants"]["zed_posetrack_mode"];    // Positional tracking accuracy.
            ZED_POSETRACK_AREA_MEMORY =
                jsonData["CameraConstants"]["zed_posetrack_area_memory"];    // Enabled camera to remember its surroundings for better positioning. Uses more resources.
            ZED_POSETRACK_POSE_SMOOTHING =
                jsonData["CameraConstants"]
                        ["zed_posetrack_pose_smoothing"];    // Smooth pose correction for small drift. Decreases overall precision for small movements.
            ZED_POSETRACK_FLOOR_IS_ORIGIN =
                jsonData["CameraConstants"]
                        ["zed_posetrack_floor_is_origin"];    // Sets the floor plane as origin for tracking. This turns on floor plane detection temporarily.
            ZED_POSETRACK_ENABLE_IMU_FUSION =
                jsonData["CameraConstants"]["zed_posetrack_enable_imu_fusion"];    // Allows ZED to use both optical odometry and IMU data for pose tracking.
            ZED_POSETRACK_USABLE_DEPTH_MIN =
                jsonData["CameraConstants"]
                        ["zed_posetrack_usable_depth_min"];    // Minimum depth used for pose tracking, useful if a static object is partial in view of the camera.
            ZED_POSETRACK_USE_GRAVITY_ORIGIN =
                jsonData["CameraConstants"]["zed_posetrack_use_gravity_origin"];    // Override 2 of the 3 rotations from initial_world_transform using the IMU.
            // ZedCam Spatial Mapping Config.
            ZED_MAPPING_TYPE = jsonData["CameraConstants"]["zed_mapping_type"];    // Mesh or point cloud output.
            ZED_MAPPING_RANGE_METER =
                jsonData["CameraConstants"]["zed_mapping_range_meter"];            // The max range in meters that the ZED cameras should use for mapping. 0 = auto.
            ZED_MAPPING_RESOLUTION_METER =
                jsonData["CameraConstants"]["zed_mapping_resolution_meter"];       // The approx goal precision for spatial mapping in METERS. Higher = Faster.
            ZED_MAPPING_MAX_MEMORY =
                jsonData["CameraConstants"]["zed_mapping_max_memory"];             // The max amount of CPU RAM (MB) that can be allocated for spatial mapping.
            ZED_MAPPING_USE_CHUNK_ONLY =
                jsonData["CameraConstants"]["zed_mapping_use_chunk_only"];       // Only update chunks that have probably changed or have new data. Faster, less accurate.
            ZED_MAPPING_STABILITY_COUNTER =
                jsonData["CameraConstants"]["zed_mapping_stability_counter"];    // Number of times that a point should be seen before adding to mesh.
            // ZedCam Object Detection Config.
            ZED_OBJDETECTION_IMG_SYNC =
                jsonData["CameraConstants"]
                        ["zed_objdetection_img_sync"];    // True = Run detection for every frame. False = Run detection async, can lead to delayed detections.
            ZED_OBJDETECTION_TRACK_OBJ =
                jsonData["CameraConstants"]["zed_objdetection_track_obj"];       // Whether or not to enable object tracking in the scene. Attempts to maintain OBJ UUIDs.
            ZED_OBJDETECTION_SEGMENTATION =
                jsonData["CameraConstants"]["zed_objdetection_segmentation"];    // Use depth data to compute the segmentation for an object. (exact outline/shape)
            ZED_OBJDETECTION_FILTERING = jsonData["CameraConstants"]["zed_objdetection_filtering"];    // Custom detection, use PER_CLASS or NONE.
            ZED_OBJDETECTION_TRACKING_PREDICTION_TIMEOUT =
                jsonData["CameraConstants"]["zed_objdetection_tracking_prediction_timeout"];    // 0-1 second. Timeout to keep guessing object position when not in sight.
            ZED_OBJDETECTION_BATCH_RETENTION_TIME =
                jsonData["CameraConstants"]["zed_objdetection_batch_retention_time"];    // The time in seconds to search for an object UUID before expiring the object.
            ZED_OBJDETECTION_BATCH_LATENCY = jsonData["CameraConstants"]["zed_objdetection_batch_latency"];    // Short latency will limit the search for previously seen
                                                                                                               // object IDs but will be closer to real time output.

            // BasicCam Basic Config.
            BASICCAM_RESIZE_INTERPOLATION_METHOD =
                jsonData["CameraConstants"]["basiccam_resize_interpolation_method"];    // The algorithm used to fill in pixels when resizing.
            ///////////////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////////////
            //// Camera Handler Adjustments.
            ///////////////////////////////////////////////////////////////////////////

            // MainCam
            ZED_MAINCAM_RESOLUTIONX =
                jsonData["CameraHandlerAdjustments"]["zed_maincam_resolutionx"];          // The horizontal pixel resolution to resize the maincam images to.
            ZED_MAINCAM_RESOLUTIONY =
                jsonData["CameraHandlerAdjustments"]["zed_maincam_resolutiony"];          // The vertical pixel resolution to resize the maincam images to.
            ZED_MAINCAM_FPS = jsonData["CameraHandlerAdjustments"]["zed_maincam_fps"];    // The FPS to use for the maincam.
            ZED_MAINCAM_HORIZONTAL_FOV =
                jsonData["CameraHandlerAdjustments"]["zed_maincam_horizontal_fov"];       // The horizontal FOV of the camera. Useful for future calculations.
            ZED_MAINCAM_VERTICAL_FOV =
                jsonData["CameraHandlerAdjustments"]["zed_maincam_vertical_fov"];         // The vertical FOV of the camera. Useful for future calculations.
            ZED_MAINCAM_USE_GPU_MAT = jsonData["CameraHandlerAdjustments"]
                                              ["zed_maincam_use_gpu_mat"];    // Whether or not to use CPU or GPU memory mats. GPU memory transfer/operations are faster.
            ZED_MAINCAM_USE_HALF_PRECISION_DEPTH =
                jsonData["CameraHandlerAdjustments"]
                        ["zed_maincam_use_half_precision_depth"];    // Whether of not to use float32 or unsigned short (16) for depth measure.
            ZED_MAINCAM_FRAME_RETRIEVAL_THREADS =
                jsonData["CameraHandlerAdjustments"]
                        ["zed_maincam_frame_retrieval_threads"];    // The number of threads allocated to the threadpool for performing frame copies to other threads.
            ZED_MAINCAN_SERIAL = jsonData["CameraHandlerAdjustments"]["zed_maincam_serial"];    // The serial number of the camera.

            // Left Side Cam.
            BASICCAM_LEFTCAM_RESOLUTIONX =
                jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_resolutionx"];                // The horizontal pixel resolution to resize the maincam images to.
            BASICCAM_LEFTCAM_RESOLUTIONY =
                jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_resolutiony"];                // The vertical pixel resolution to resize the maincam images to.
            BASICCAM_LEFTCAM_FPS = jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_fps"];     // The FPS to use for the maincam.
            BASICCAM_LEFTCAM_HORIZONTAL_FOV =
                jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_horizontal_fov"];             // The horizontal FOV of the camera. Useful for future calculations.
            BASICCAM_LEFTCAM_VERTICAL_FOV =
                jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_vertical_fov"];               // The vertical FOV of the camera. Useful for future calculations.
            BASICCAM_LEFTCAM_FRAME_RETRIEVAL_THREADS =
                jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_frame_retrieval_threads"];    // The number of threads allocated to the threadpool for performing
                                                                                                     // frame copies to other threads.
            BASICCAM_LEFTCAM_INDEX     = jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_index"];        // The /dev/video index of the camera.
            BASICCAM_LEFTCAM_PIXELTYPE = jsonData["CameraHandlerAdjustments"]["basiccam_leftcam_pixeltype"];    // The pixel layout of the camera.
            ///////////////////////////////////////////////////////////////////////////
        }
    }
}    // namespace globals
