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

#include <sl/Camera.hpp>

#include "./interfaces/Camera.hpp"

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
    //// Drive Constants.
    ///////////////////////////////////////////////////////////////////////////

    // Power constants.
    const float DRIVE_MAX_POWER  = 1.0;
    const float DRIVE_MIN_POWER  = -1.0;
    const float DRIVE_MAX_EFFORT = 0.5;
    const float DRIVE_MIN_EFFORT = -0.5;

    // Control constants.
    const bool DRIVE_SQUARE_CONTROL_INPUTS = false;    // This is used by the DifferentialDrive algorithms. True makes fine inputs smoother, but less responsive.
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
    const bool ZED_SENSING_FILL                  = false;    // True provides a depth map with a Z value for every pixel (X, Y) in the left image. Slower.
    const float ZED_DEFAULT_MINIMUM_DISTANCE     = 0.2;      // Minimum distance in ZED_MEASURE_UNITS to report from depth measurement.
    const float ZED_DEFAULT_MAXIMUM_DISTANCE     = 40.0;     // Maximum distance in ZED_MEASURE_UNITS to report from depth measurement.
    const int ZED_DEPTH_STABILIZATION            = 1;    // This parameter controls a stabilization filter that reduces oscillations in depth map. In the range [0-100]
    // ZedCam Positional Tracking Config.
    const sl::POSITIONAL_TRACKING_MODE ZED_POSETRACK_MODE = sl::POSITIONAL_TRACKING_MODE::STANDARD;    // Positional tracking accuracy.
    const bool ZED_POSETRACK_AREA_MEMORY                  = true;     // Enabled camera to remember its surroundings for better positioning. Uses more resources.
    const bool ZED_POSETRACK_POSE_SMOOTHING               = false;    // Smooth pose correction for small drift. Decreases overall precision for small movements.
    const bool ZED_POSETRACK_FLOOR_IS_ORIGIN              = true;     // Sets the floor plane as origin for tracking. This turns on floor plane detection temporarily.
    const bool ZED_POSETRACK_ENABLE_IMU_FUSION            = true;     // Allows ZED to use both optical odometry and IMU data for pose tracking.
    const float ZED_POSETRACK_USABLE_DEPTH_MIN            = 0.2;      // Minimum depth used for pose tracking, useful if a static object is partial in view of the camera.
    const float ZED_POSETRACK_USE_GRAVITY_ORIGIN          = true;     // Override 2 of the 3 rotations from initial_world_transform using the IMU.
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
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Camera Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // MainCam
    const int ZED_MAINCAM_RESOLUTIONX               = 1280;        // The horizontal pixel resolution to resize the maincam images to.
    const int ZED_MAINCAM_RESOLUTIONY               = 720;         // The vertical pixel resolution to resize the maincam images to.
    const int ZED_MAINCAM_FPS                       = 60;          // The FPS to use for the maincam.
    const int ZED_MAINCAM_HORIZONTAL_FOV            = 110;         // The horizontal FOV of the camera. Useful for future calculations.
    const int ZED_MAINCAM_VERTICAL_FOV              = 70;          // The vertical FOV of the camera. Useful for future calculations.
    const bool ZED_MAINCAM_USE_GPU_MAT              = true;        // Whether or not to use CPU or GPU memory mats. GPU memory transfer/operations are faster.
    const bool ZED_MAINCAM_USE_HALF_PRECISION_DEPTH = false;       // Whether of not to use float32 or unsigned short (16) for depth measure.
    const int ZED_MAINCAM_FRAME_RETRIEVAL_THREADS   = 30;          // The number of threads allocated to the threadpool for performing frame copies to other threads.
    const int ZED_MAINCAN_SERIAL                    = 31237348;    // The serial number of the camera.

    // Left Side Cam.
    const int BASIC_LEFTCAM_RESOLUTIONX             = 1280;    // The horizontal pixel resolution to resize the maincam images to.
    const int BASIC_LEFTCAM_RESOLUTIONY             = 720;     // The vertical pixel resolution to resize the maincam images to.
    const int BASIC_LEFTCAM_FPS                     = 30;      // The FPS to use for the maincam.
    const int BASIC_LEFTCAM_HORIZONTAL_FOV          = 110;     // The horizontal FOV of the camera. Useful for future calculations.
    const int BASIC_LEFTCAM_VERTICAL_FOV            = 70;      // The vertical FOV of the camera. Useful for future calculations.
    const int BASIC_LEFTCAM_FRAME_RETRIEVAL_THREADS = 10;      // The number of threads allocated to the threadpool for performing frame copies to other threads.
    const int BASIC_LEFTCAM_INDEX                   = 0;       // The /dev/video index of the camera.
    const PIXEL_FORMATS BASIC_LEFTCAM_PIXELTYPE     = PIXEL_FORMATS::eBGR;    // The pixel layout of the camera.
    ///////////////////////////////////////////////////////////////////////////

}    // namespace constants

#endif    // CONSTS_H
