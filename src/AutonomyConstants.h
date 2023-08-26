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

}    // namespace constants

#endif    // CONSTS_H
