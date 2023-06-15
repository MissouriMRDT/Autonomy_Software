/*
   MRDT_Autonomy_Globals.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Defines global defines, variables, and functions for MRDT Software.
*/

#include "Autonomy_IdentitySoftware.h"
#include "interfaces/DriveBoard.h"
#include "interfaces/MultimediaBoard.h"
#include "interfaces/NavigationBoard.h"

#include <chrono>
#include <ctime>
#include <plog/Initializers/ConsoleInitializer.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>

#define MAX_DRIVE_POWER 250
#define MIN_DRIVE_POWER 50

#ifndef AUTONOMY_GLOBALS_H
#	define AUTONOMY_GLOBALS_H

// Logging:
enum AutonomyLogger
{
	AL_FileLogger,
	AL_ConsoleLogger
};	  // Enum to specify logging location

extern plog::RollingFileAppender<plog::TxtFormatter> g_pFileAppender;	 // Sends log message to file
extern plog::ColorConsoleAppender<plog::TxtFormatter> g_pConsoleAppender;	 // Sends log message to file and console

void InitializeAutonomyLoggers();	 // Method to set up the loggers

// Versioning:
extern Autonomy_IdentitySoftware g_pIdentifySoftware;	 // Global Version Handler

// Board Interfaces:
extern DriveBoard g_pDriveBoardInterface;	 // Global Drive Board Interface
extern MultimediaBoard g_pMultimediaBoardInterface;	   // Global Multimedia Board Interface
extern NavigationBoard g_pNavigationBoardInterface;	   // Global Navigation Board Interface

#endif	  // MRDT_AUTONOMY_GLOBALS_H
