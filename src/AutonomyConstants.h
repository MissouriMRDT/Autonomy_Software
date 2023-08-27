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
    // ZedCam.
    const sl::RESOLUTION ZED_BASE_RESOLUTION     = sl::RESOLUTION::HD1080;
    const sl::UNIT ZED_MEASURE_UNITS             = sl::UNIT::METER;
    const sl::COORDINATE_SYSTEM ZED_COORD_SYSTEM = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;
    const sl::DEPTH_MODE ZED_DEPTH_MODE          = sl::DEPTH_MODE::QUALITY;
    const float ZED_MINIMUM_DISTANCE             = 0.2;     // Minimum distance in ZED_MEASURE_UNITS to report from depth measurement.
    const float ZED_MAXIMUM_DISTANCE             = 40.0;    // Maximum distance in ZED_MEASURE_UNITS to report from depth measurement.
    const int ZED_DEPTH_STABILIZATION            = 1;       // This parameter controls a stabilization filter that reduces oscillations in depth map. In the range [0-100]

}    // namespace constants

#endif    // CONSTS_H
