/******************************************************************************
 * @brief Defines functions and objects used project wide.
 *
 * @file AutonomyGlobals.h
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "IdentitySoftware.h"
#include "interfaces/DriveBoard.h"
#include "interfaces/MultimediaBoard.h"
#include "interfaces/NavigationBoard.h"

#include <chrono>
#include <ctime>
#include <iostream>
#include <quill/Quill.h>

#ifndef AUTONOMY_GLOBALS_H
#define AUTONOMY_GLOBALS_H

namespace constants
{
    // Drive constants.
    const int MAX_DRIVE_POWER = 250;
    const int MIN_DRIVE_POWER = 50;
}    // namespace constants

// Logging:
extern quill::Logger* g_qFileLogger;
extern quill::Logger* g_qConsoleLogger;
extern quill::Logger* g_qSharedLogger;

void InitializeLoggers();    // Method to set up the loggers

// Versioning:
extern IdentitySoftware g_pIdentifySoftware;    // Global Version Handler

// Board Interfaces:
extern DriveBoard g_pDriveBoardInterface;              // Global Drive Board Interface
extern MultimediaBoard g_pMultimediaBoardInterface;    // Global Multimedia Board Interface
extern NavigationBoard g_pNavigationBoardInterface;    // Global Navigation Board Interface

#endif                                                 // MRDT_AutonomyGlobals_H
