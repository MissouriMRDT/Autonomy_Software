/******************************************************************************
 * @brief The  ants header for Autonomy Software
 *
 * @file  s.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-30
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef S_H
#define S_H

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

#include "./interfaces/Camera.hpp"

/******************************************************************************
 * @brief Namespace containing all  ants for autonomy software. Including
 *      AutonomyGlobals.h will also include this namespace.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-05
 ******************************************************************************/
namespace ants
{
    ///////////////////////////////////////////////////////////////////////////
    //// Drive  ants.
    ///////////////////////////////////////////////////////////////////////////

    // Power  ants.
    float DRIVE_MAX_POWER  = 1.0;
    float DRIVE_MIN_POWER  = -1.0;
    float DRIVE_MAX_EFFORT = 0.5;
    float DRIVE_MIN_EFFORT = -0.5;

    // Control  ants.
    double DRIVE_PID_PROPORTIONAL       = 0.1;      // The proportional gain for the controller used to point the rover at a goal heading during navigation.
    double DRIVE_PID_INTEGRAL           = 0.01;     // The integral gain for the controller used to point the rover at a goal heading during navigation.
    double DRIVE_PID_DERIVATIVE         = 0.0;      // The derivative gain for the controller used to point the rover at a goal heading during navigation.
    double DRIVE_PID_MAX_ERROR_PER_ITER = 100;      // The max allowable error the controller will see per iteration. This is on degrees from setpoint.
    double DRIVE_PID_MAX_INTEGRAL_TERM  = 0.3;      // The max effort the I term is allowed to contribute.
    double DRIVE_PID_MAX_OUTPUT_EFFORT  = 0.5;      // The max effort the entire PID controller is allowed to output. Range is within DRIVE_MAX/MIN_POWER.
    double DRIVE_PID_MAX_RAMP_RATE      = 0.4;      // The max ramp rate of the output of the PID controller.
    double DRIVE_PID_OUTPUT_FILTER      = 0.0;      // Larger values will filter out large spikes or oscillations. 0.1 is a good starting point.
    bool DRIVE_PID_OUTPUT_REVERSED      = false;    // Negates the output of the PID controller.
    bool DRIVE_SQUARE_CONTROL_INPUTS    = false;    // This is used by the DifferentialDrive algorithms. True makes fine inputs smoother, but less responsive.
    bool DRIVE_CURVATURE_KINEMATICS_ALLOW_TURN_WHILE_STOPPED = true;    // This enabled turning in-place when using curvature drive control.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Camera  ants.
    ///////////////////////////////////////////////////////////////////////////

    // ZedCam Basic Config.
    sl::RESOLUTION ZED_BASE_RESOLUTION     = sl::RESOLUTION::HD720;                      // The base resolution to open the all cameras with.
    sl::UNIT ZED_MEASURE_UNITS             = sl::UNIT::METER;                            // The base measurement unit to use for depth.
    sl::COORDINATE_SYSTEM ZED_COORD_SYSTEM = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;    // Coordinate system to use for measurements.
    sl::DEPTH_MODE ZED_DEPTH_MODE          = sl::DEPTH_MODE::NEURAL;                     // The measurement accuracy for depth. NEURAL is by far the best.
    sl::VIEW ZED_RETRIEVE_VIEW             = sl::VIEW::LEFT;                             // The eye to retrieve regular and depth images from.
    bool ZED_SENSING_FILL                  = false;    // True provides a depth map with a Z value for every pixel (X, Y) in the left image. Slower and worse.
    float ZED_DEFAULT_MINIMUM_DISTANCE     = 0.2;      // Minimum distance in ZED_MEASURE_UNITS to report from depth measurement.
    float ZED_DEFAULT_MAXIMUM_DISTANCE     = 40.0;     // Maximum distance in ZED_MEASURE_UNITS to report from depth measurement.
    int ZED_DEPTH_STABILIZATION            = 1;        // This parameter controls a stabilization filter that reduces oscillations in depth map. In the range [0-100]
                                                       // ZedCam Positional Tracking Config.
    sl::POSITIONAL_TRACKING_MODE ZED_POSETRACK_MODE = sl::POSITIONAL_TRACKING_MODE::STANDARD;    // Positional tracking accuracy.
    bool ZED_POSETRACK_AREA_MEMORY                  = true;     // Enabled camera to remember its surroundings for better positioning. Uses more resources.
    bool ZED_POSETRACK_POSE_SMOOTHING               = false;    // Smooth pose correction for small drift. Decreases overall precision for small movements.
    bool ZED_POSETRACK_FLOOR_IS_ORIGIN              = true;     // Sets the floor plane as origin for tracking. This turns on floor plane detection temporarily.
    bool ZED_POSETRACK_ENABLE_IMU_FUSION            = true;     // Allows ZED to use both optical odometry and IMU data for pose tracking.
    float ZED_POSETRACK_USABLE_DEPTH_MIN            = 0.2;      // Minimum depth used for pose tracking, useful if a static object is partial in view of the camera.
    float ZED_POSETRACK_USE_GRAVITY_ORIGIN          = true;     // Override 2 of the 3 rotations from initial_world_transform using the IMU.
                                                                // ZedCam Spatial Mapping Config.
    sl::SpatialMappingParameters::SPATIAL_MAP_TYPE ZED_MAPPING_TYPE = sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;    // Mesh or point cloud output.
    float ZED_MAPPING_RANGE_METER                                   = 20.0;    // The max range in meters that the ZED cameras should use for mapping. 0 = auto.
    float ZED_MAPPING_RESOLUTION_METER                              = 0.01;    // The approx goal precision for spatial mapping in METERS. Higher = Faster.
    int ZED_MAPPING_MAX_MEMORY                                      = 4096;    // The max amount of CPU RAM (MB) that can be allocated for spatial mapping.
    bool ZED_MAPPING_USE_CHUNK_ONLY                                 = true;    // Only update chunks that have probably changed or have new data. Faster, less accurate.
    int ZED_MAPPING_STABILITY_COUNTER                               = 4;       // Number of times that a point should be seen before adding to mesh.
                                                                               // ZedCam Object Detection Config.
    bool ZED_OBJDETECTION_IMG_SYNC                       = true;     // True = Run detection for every frame. False = Run detection async, can lead to delayed detections.
    bool ZED_OBJDETECTION_TRACK_OBJ                      = true;     // Whether or not to enable object tracking in the scene. Attempts to maintain OBJ UUIDs.
    bool ZED_OBJDETECTION_SEGMENTATION                   = false;    // Use depth data to compute the segmentation for an object. (exact outline/shape)
    sl::OBJECT_FILTERING_MODE ZED_OBJDETECTION_FILTERING = sl::OBJECT_FILTERING_MODE::NMS3D_PER_CLASS;    // Custom detection, use PER_CLASS or NONE.
    float ZED_OBJDETECTION_TRACKING_PREDICTION_TIMEOUT   = 0.5;    // 0-1 second. Timeout to keep guessing object position when not in sight.
    float ZED_OBJDETECTION_BATCH_RETENTION_TIME          = 240;    // The time in seconds to search for an object UUID before expiring the object.
    float ZED_OBJDETECTION_BATCH_LATENCY = 2;    // Short latency will limit the search for previously seen object IDs but will be closer to real time output.

                                                 // BasicCam Basic Config.
    cv::InterpolationFlags BASICCAM_RESIZE_INTERPOLATION_METHOD = cv::InterpolationFlags::INTER_LINEAR;    // The algorithm used to fill in pixels when resizing.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Camera Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Main ZED Camera.
    int ZED_MAINCAM_RESOLUTIONX               = 1280;        // The horizontal pixel resolution to resize the maincam images to.
    int ZED_MAINCAM_RESOLUTIONY               = 720;         // The vertical pixel resolution to resize the maincam images to.
    int ZED_MAINCAM_FPS                       = 60;          // The FPS to use for the maincam.
    int ZED_MAINCAM_HORIZONTAL_FOV            = 110;         // The horizontal FOV of the camera. Useful for future calculations.
    int ZED_MAINCAM_VERTICAL_FOV              = 70;          // The vertical FOV of the camera. Useful for future calculations.
    bool ZED_MAINCAM_USE_GPU_MAT              = false;       // Whether or not to use CPU or GPU memory mats. GPU memory transfer/operations are faster.
    bool ZED_MAINCAM_USE_HALF_PRECISION_DEPTH = true;        // Whether of not to use float32 or unsigned short (16) for depth measure.
    int ZED_MAINCAM_FRAME_RETRIEVAL_THREADS   = 20;          // The number of threads allocated to the threadpool for performing frame copies to other threads.
    int ZED_MAINCAM_SERIAL                    = 31237348;    // The serial number of the camera. Set to 0 to open the next available one.

                                                             // Left Side Cam.
    int BASICCAM_LEFTCAM_RESOLUTIONX             = 1280;    // The horizontal pixel resolution to resize the maincam images to.
    int BASICCAM_LEFTCAM_RESOLUTIONY             = 720;     // The vertical pixel resolution to resize the maincam images to.
    int BASICCAM_LEFTCAM_FPS                     = 30;      // The FPS to use for the maincam.
    int BASICCAM_LEFTCAM_HORIZONTAL_FOV          = 110;     // The horizontal FOV of the camera. Useful for future calculations.
    int BASICCAM_LEFTCAM_VERTICAL_FOV            = 70;      // The vertical FOV of the camera. Useful for future calculations.
    int BASICCAM_LEFTCAM_FRAME_RETRIEVAL_THREADS = 10;      // The number of threads allocated to the threadpool for performing frame copies to other threads.
    int BASICCAM_LEFTCAM_INDEX                   = 0;       // The /dev/video index of the camera.
    PIXEL_FORMATS BASICCAM_LEFTCAM_PIXELTYPE     = PIXEL_FORMATS::eBGR;    // The pixel layout of the camera.

                                                                           // Right Side Cam.
    int BASICCAM_RIGHTCAM_RESOLUTIONX             = 1280;    // The horizontal pixel resolution to resize the maincam images to.
    int BASICCAM_RIGHTCAM_RESOLUTIONY             = 720;     // The vertical pixel resolution to resize the maincam images to.
    int BASICCAM_RIGHTCAM_FPS                     = 30;      // The FPS to use for the maincam.
    int BASICCAM_RIGHTCAM_HORIZONTAL_FOV          = 110;     // The horizontal FOV of the camera. Useful for future calculations.
    int BASICCAM_RIGHTCAM_VERTICAL_FOV            = 70;      // The vertical FOV of the camera. Useful for future calculations.
    int BASICCAM_RIGHTCAM_FRAME_RETRIEVAL_THREADS = 10;      // The number of threads allocated to the threadpool for performing frame copies to other threads.
    int BASICCAM_RIGHTCAM_INDEX                   = 1;       // The /dev/video index of the camera.
    PIXEL_FORMATS BASICCAM_RIGHTCAM_PIXELTYPE     = PIXEL_FORMATS::eBGR;    // The pixel layout of the camera.
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// ArUco Vision  ants.
    ///////////////////////////////////////////////////////////////////////////

    // OpenCV ArUco detection config.
    cv::aruco::PredefinedDictionaryType ARUCO_DICTIONARY = cv::aruco::DICT_4X4_50;    // The predefined ArUco dictionary to use for detections.
    float ARUCO_TAG_SIDE_LENGTH                          = 0.015;                     // Size of the white borders around the tag.
    int ARUCO_VALIDATION_THRESHOLD                       = 5;      // How many times does the tag need to be detected(hit) before being validated as an actual aruco tag.
    int ARUCO_UNVALIDATED_TAG_FORGET_THRESHOLD           = 5;      // How many times can an unvalidated tag be missing from frame before being forgotten.
    int ARUCO_VALIDATED_TAG_FORGET_THRESHOLD             = 10;     // How many times can a validated tag be missing from frame before being forgotten.
    double ARUCO_PIXEL_THRESHOLD                         = 175;    // Pixel value threshold for pre-process threshold mask
    double ARUCO_PIXEL_THRESHOLD_MAX_VALUE               = 255;    // Pixel value to set to if pixel is within threshold
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Tag Detection Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Main ZED Camera.
    int TAGDETECT_MAINCAM_DATA_RETRIEVAL_THREADS  = 5;     // The number of threads allocated to the threadpool for performing data copies to other threads.
    int TAGDETECT_MAINCAM_CORNER_REFINE_MAX_ITER  = 30;    // The maximum number of iterations to run corner refinement on the image.
    int TAGDETECT_MAINCAM_CORNER_REFINE_METHOD    = cv::aruco::CORNER_REFINE_NONE;    // Algorithm used to refine tag corner pixels.
    bool TAGDETECT_MAINCAM_DETECT_INVERTED_MARKER = true;                             // Whether or not to detector upside-down tags.
    int TAGDETECT_MAINCAM_MARKER_BORDER_BITS      = 1;                                // This number of bits on the border. A bit is one unit square of the tag.
    bool TAGDETECT_MAINCAM_USE_ARUCO3_DETECTION   = false;                            // Whether or not to use the newer and faster Aruco detection strategy.

                                                                                      // Left Side Cam.
    int TAGDETECT_LEFTCAM_DATA_RETRIEVAL_THREADS  = 5;     // The number of threads allocated to the threadpool for performing data copies to other threads.
    int TAGDETECT_LEFTCAM_CORNER_REFINE_MAX_ITER  = 30;    // The maximum number of iterations to run corner refinement on the image.
    int TAGDETECT_LEFTCAM_CORNER_REFINE_METHOD    = cv::aruco::CORNER_REFINE_NONE;    // Algorithm used to refine tag corner pixels.
    bool TAGDETECT_LEFTCAM_DETECT_INVERTED_MARKER = true;                             // Whether or not to detector upside-down tags.
    int TAGDETECT_LEFTCAM_MARKER_BORDER_BITS      = 1;                                // This number of bits on the border. A bit is one unit square of the tag.
    bool TAGDETECT_LEFTCAM_USE_ARUCO3_DETECTION   = true;                             // Whether or not to use the newer and faster Aruco detection strategy.

                                                                                      // Right Side Cam.
    int TAGDETECT_RIGHTCAM_DATA_RETRIEVAL_THREADS  = 5;     // The number of threads allocated to the threadpool for performing data copies to other threads.
    int TAGDETECT_RIGHTCAM_CORNER_REFINE_MAX_ITER  = 30;    // The maximum number of iterations to run corner refinement on the image.
    int TAGDETECT_RIGHTCAM_CORNER_REFINE_METHOD    = cv::aruco::CORNER_REFINE_NONE;    // Algorithm used to refine tag corner pixels.
    bool TAGDETECT_RIGHTCAM_DETECT_INVERTED_MARKER = true;                             // Whether or not to detector upside-down tags.
    int TAGDETECT_RIGHTCAM_MARKER_BORDER_BITS      = 1;                                // This number of bits on the border. A bit is one unit square of the tag.
    bool TAGDETECT_RIGHTCAM_USE_ARUCO3_DETECTION   = false;                            // Whether or not to use the newer and faster Aruco detection strategy.
    ///////////////////////////////////////////////////////////////////////////
}    // namespace  ants

#endif    //  S_H
