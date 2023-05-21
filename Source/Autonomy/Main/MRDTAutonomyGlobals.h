/*
   MRDTAutonomyGlobals.h
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

#include "MRDTAutonomyIdentitySoftware.h"

#ifndef MRDTAUTONOMYGLOBALS_H
#define MRDTAUTONOMYGLOBALS_H

// Logging:
enum AutonomyLogger { AL_FileLogger, AL_ConsoleLogger };             // Enum to specify logging location

extern plog::RollingFileAppender<plog::TxtFormatter> pFileAppender;  // Sends log message to file
extern plog::ConsoleAppender<plog::TxtFormatter> pConsoleAppender;   // Sends log message to file and console

void InitializeAutonomyLoggers();                                    // Method to set up the loggers

// Versioning:
extern MRDTAutonomyIdentitySoftware pIdentifySoftware;

#endif // MRDTAUTONOMYGLOBALS_H
