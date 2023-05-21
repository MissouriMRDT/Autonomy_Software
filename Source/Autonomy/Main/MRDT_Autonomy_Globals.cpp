/*
   MRDT_Autonomy_Globals.cpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Defines global defines, variables, and functions for MRDT Software.
*/

#include "MRDT_Autonomy_Globals.h"

plog::RollingFileAppender<plog::TxtFormatter> g_pFileAppender("file.log", 1000000000, 10);
plog::ConsoleAppender<plog::TxtFormatter> g_pConsoleAppender;

void InitializeAutonomyLoggers() {

    // Retrieve the current time for the log file name
    time_t     tCurrentTime = time(nullptr);
    struct tm  sTimeStruct = *localtime(&tCurrentTime);
    char       cCurrentTime[80];

    // Format the current time in a format that can be used as a file name
    std::strftime(cCurrentTime, sizeof(cCurrentTime), "%Y%m%d-%H%M%S", &sTimeStruct);

    // Turn the current time into a file name
    std::string szFilenameWithExtension;
    szFilenameWithExtension =   cCurrentTime;
    szFilenameWithExtension +=  ".log";

    // Assign the file logger the file name that was just created
    g_pFileAppender.setFileName(szFilenameWithExtension.c_str());

    // Initialize file logger
    plog::init<AutonomyLogger::AL_FileLogger>(plog::debug, &g_pFileAppender);

    // Initialize console logger so that it also sends to the file logger
    plog::init<AutonomyLogger::AL_ConsoleLogger>(plog::debug, &g_pConsoleAppender)
            .addAppender(&g_pFileAppender);
}

MRDT_Autonomy_IdentitySoftware g_pIdentifySoftware;

MRDT_DriveBoard g_pDriveBoardInterface;
MRDT_NavigationBoard g_pNavigationBoard;