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

#include "IdentitySoftware.h"
#include "drivers/DriveBoard.h"
#include "drivers/MultimediaBoard.h"
#include "drivers/NavigationBoard.h"
#include "handlers/CameraHandler.h"
#include "handlers/StateMachineHandler.h"
#include "handlers/TagDetectionHandler.h"

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
    // Versioning:
    extern IdentitySoftware g_pIdentifySoftware;    // Global Version Handler

    // Camera Handler:
    extern CameraHandler* g_pCameraHandler;    // Global Camera Handler

    // Tag Detection Handler:
    extern TagDetectionHandler* g_pTagDetectionHandler;    // Global Tag Detection Handler

    // Board Interfaces:
    extern DriveBoard g_pDriveBoardInterface;              // Global Drive Board Interface
    extern MultimediaBoard g_pMultimediaBoardInterface;    // Global Multimedia Board Interface
    extern NavigationBoard g_pNavigationBoardInterface;    // Global Navigation Board Interface

    // State Machine Handler:
    extern StateMachineHandler* g_pStateMachineHandler;    // Global State Machine Handler
}    // namespace globals

#endif    // AUTONOMY_GLOBALS_H
