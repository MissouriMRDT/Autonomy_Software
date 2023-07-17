/******************************************************************************
 * @brief Defines functions and objects used project wide.
 *
 * @file Autonomy_Globals.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-0620
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "./Autonomy_IdentitySoftware.h"
#include "./drivers/DriveBoard.h"
#include "./drivers/MultimediaBoard.h"
#include "./drivers/NavigationBoard.h"

#include <chrono>
#include <ctime>
#include <plog/Initializers/ConsoleInitializer.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>

#ifndef AUTONOMY_GLOBALS_H
#define AUTONOMY_GLOBALS_H

namespace constants
{
    // Drive constants.
    const int MAX_DRIVE_POWER = 250;
    const int MIN_DRIVE_POWER = 50;
}    // namespace constants

// Logging:
enum AutonomyLogger
{
    AL_FileLogger,
    AL_ConsoleLogger
};                                                                           // Enum to specify logging location

extern plog::RollingFileAppender<plog::TxtFormatter> g_pFileAppender;        // Sends log message to file
extern plog::ColorConsoleAppender<plog::TxtFormatter> g_pConsoleAppender;    // Sends log message to file and console

void InitializeAutonomyLoggers();                                            // Method to set up the loggers

// Versioning:
extern Autonomy_IdentitySoftware g_pIdentifySoftware;    // Global Version Handler

// Board Interfaces:
extern DriveBoard g_pDriveBoardInterface;              // Global Drive Board Interface
extern MultimediaBoard g_pMultimediaBoardInterface;    // Global Multimedia Board Interface
extern NavigationBoard g_pNavigationBoardInterface;    // Global Navigation Board Interface

#endif                                                 // MRDT_AUTONOMY_GLOBALS_H
