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
    // Drive Constants
    const int MAX_DRIVE_POWER = 250;
    const int MIN_DRIVE_POWER = 50;

    //// Camera Constants.
    // ZedCam Basic Config.
    const sl::RESOLUTION ZED_BASE_RESOLUTION     = sl::RESOLUTION::HD1080;
    const sl::UNIT ZED_MEASURE_UNITS             = sl::UNIT::METER;
    const sl::COORDINATE_SYSTEM ZED_COORD_SYSTEM = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;
    const sl::DEPTH_MODE ZED_DEPTH_MODE          = sl::DEPTH_MODE::QUALITY;
    const sl::VIEW ZED_RETRIEVE_VIEW             = sl::VIEW::LEFT;
    const bool ZED_SENSING_FILL                  = true;    // True provides a depth map with a Z value for every pixel (X, Y) in the left image. Slower.
    const float ZED_DEFAULT_MINIMUM_DISTANCE     = 0.2;     // Minimum distance in ZED_MEASURE_UNITS to report from depth measurement.
    const float ZED_DEFAULT_MAXIMUM_DISTANCE     = 40.0;    // Maximum distance in ZED_MEASURE_UNITS to report from depth measurement.
    const int ZED_DEPTH_STABILIZATION            = 1;       // This parameter controls a stabilization filter that reduces oscillations in depth map. In the range [0-100]
    // ZedCam Positional Tracking Config.
    const bool ZED_POSETRACK_AREA_MEMORY                  = true;     // Enabled camera to remember its surroundings for better positioning. Uses more resources.
    const bool ZED_POSETRACK_POSE_SMOOTHING               = false;    // Smooth pose correction for small drift. Decreases overall precision for small movements.
    const bool ZED_POSETRACK_FLOOR_IS_ORIGIN              = true;     // Sets the floor plane as origin for tracking. This turns on floor plane detection temporarily.
    const bool ZED_POSETRACK_ENABLE_IMU_FUSION            = false;    // Allows ZED to use both optical odometry and IMU data for pose tracking.
    const float ZED_POSETRACK_USABLE_DEPTH_MIN            = 0.2;      // Minimum depth used for pose tracking, useful if a static object is partial in view of the camera.
    const float ZED_POSETRACK_USE_GRAVITY_ORIGIN          = true;     // Override 2 of the 3 rotations from initial_world_transform using the IMU.
    const sl::POSITIONAL_TRACKING_MODE ZED_POSETRACK_MODE = sl::POSITIONAL_TRACKING_MODE::STANDARD;    // Positional tracking accuracy.

}    // namespace constants

#endif    // CONSTS_H
