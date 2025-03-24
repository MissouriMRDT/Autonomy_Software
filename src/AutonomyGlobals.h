/******************************************************************************
 * @brief Defines functions and objects used project wide.
 *
 * @file AutonomyGlobals.h
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef AUTONOMY_GLOBALS_H
#define AUTONOMY_GLOBALS_H

#include "drivers/DriveBoard.h"
#include "drivers/MultimediaBoard.h"
#include "drivers/NavigationBoard.h"
#include "handlers/CameraHandler.h"
#include "handlers/StateMachineHandler.h"
#include "handlers/TagDetectionHandler.h"
#include "handlers/WaypointHandler.h"

/// \cond
#include <chrono>
#include <ctime>
#include <iostream>

/// \endcond

/******************************************************************************
 * @brief Namespace containing all global types/structs that will be used project
 *      wide and ARE NOT SPECIFIC TO A CERTAIN CLASS.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com), Eli Byrd (edbgkk@mst.edu)
 * @date 2023-09-23
 ******************************************************************************/
namespace globals
{
    /////////////////////////////////////////
    // Declare namespace external variables and objects.
    /////////////////////////////////////////

    // Waypoint Handler:
    extern std::shared_ptr<WaypointHandler> g_pWaypointHandler;    // Global Waypoint Handler

    // Camera Handler:
    extern std::shared_ptr<CameraHandler> g_pCameraHandler;    // Global Camera Handler

    // Tag Detection Handler:
    extern std::shared_ptr<TagDetectionHandler> g_pTagDetectionHandler;    // Global Tag Detection Handler

    // State Machine Handler:
    extern std::shared_ptr<StateMachineHandler> g_pStateMachineHandler;    // Global State Machine Handler

    // Board Interfaces:
    extern std::shared_ptr<DriveBoard> g_pDriveBoard;              // Global Drive Board Driver
    extern std::shared_ptr<MultimediaBoard> g_pMultimediaBoard;    // Global Multimedia Board Driver
    extern std::shared_ptr<NavigationBoard> g_pNavigationBoard;    // Global Navigation Board Driver
}    // namespace globals

#endif    // AUTONOMY_GLOBALS_H
