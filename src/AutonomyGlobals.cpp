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
}    // namespace globals
