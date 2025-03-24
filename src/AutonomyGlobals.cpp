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
    // Forward declarations for namespace variables and objects.
    /////////////////////////////////////////

    // Waypoint Handler:
    std::shared_ptr<WaypointHandler> g_pWaypointHandler;

    // Camera Handler:
    std::shared_ptr<CameraHandler> g_pCameraHandler;

    // Tag Detection Handler:
    std::shared_ptr<TagDetectionHandler> g_pTagDetectionHandler;

    // State Machine Handler:
    std::shared_ptr<StateMachineHandler> g_pStateMachineHandler;

    // Drivers:
    std::shared_ptr<DriveBoard> g_pDriveBoard;
    std::shared_ptr<MultimediaBoard> g_pMultimediaBoard;
    std::shared_ptr<NavigationBoard> g_pNavigationBoard;
}    // namespace globals
