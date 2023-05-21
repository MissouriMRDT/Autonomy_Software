/*
   MRDT_Autonomy_Globals.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Defines global defines, variables, and functions for MRDT Software.
*/

#include <chrono>
#include <ctime>
#include <plog/Log.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Initializers/ConsoleInitializer.h>

#include "MRDT_Autonomy_IdentitySoftware.h"
#include "../Communication/MRDT_DriveBoard.h"
#include "../Communication/MRDT_MultimediaBoard.h"
#include "../Communication/MRDT_NavigationBoard.h"

#define MAX_DRIVE_POWER     250
#define MIN_DRIVE_POWER     50

#ifndef MRDT_AUTONOMY_GLOBALS_H
#define MRDT_AUTONOMY_GLOBALS_H

// Logging:
enum AutonomyLogger { AL_FileLogger, AL_ConsoleLogger };                // Enum to specify logging location

extern plog::RollingFileAppender<plog::TxtFormatter> g_pFileAppender;   // Sends log message to file
extern plog::ConsoleAppender<plog::TxtFormatter> g_pConsoleAppender;    // Sends log message to file and console

void InitializeAutonomyLoggers();                                       // Method to set up the loggers

// Versioning:
extern MRDT_Autonomy_IdentitySoftware g_pIdentifySoftware;              // Global Version Handler

// Board Interfaces:
extern MRDT_DriveBoard      g_pDriveBoardInterface;                     // Global Drive Board Interface
extern MRDT_MultimediaBoard g_pMultimediaBoardInterface;                // Global Multimedia Board Interface
extern MRDT_NavigationBoard g_pNavigationBoardInterface;                // Global Navigation Board Interface

#endif // MRDT_AUTONOMY_GLOBALS_H
