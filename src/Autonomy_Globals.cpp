/******************************************************************************
 * @brief Sets up functions and classes used project wide.
 *
 * @file Autonomy_Globals.cpp
 * @author Byrdman32 (eli@byrdneststudios.com), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0620
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "Autonomy_Globals.h"

plog::RollingFileAppender<plog::TxtFormatter> g_pFileAppender("file.log", 1000000000, 10);
plog::ColorConsoleAppender<plog::TxtFormatter> g_pConsoleAppender;

/******************************************************************************
 * @brief Sets up project wide logger.
 *
 *
 * @author Byrdman32 (eli@byrdneststudios.com), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0620
 ******************************************************************************/
void InitializeAutonomyLoggers()
{
    // Retrieve the current time for the log file name
    time_t tCurrentTime   = time(nullptr);
    struct tm sTimeStruct = *localtime(&tCurrentTime);
    char cCurrentTime[80];

    // Format the current time in a format that can be used as a file name
    std::strftime(cCurrentTime, sizeof(cCurrentTime), "%Y%m%d-%H%M%S", &sTimeStruct);

    // Turn the current time into a file name
    std::string szFilenameWithExtension;
    szFilenameWithExtension = cCurrentTime;
    szFilenameWithExtension += ".log";

    // Assign the file logger the file name that was just created
    g_pFileAppender.setFileName(szFilenameWithExtension.c_str());

    // Initialize file logger
    plog::init<AutonomyLogger::AL_FileLogger>(plog::debug, &g_pFileAppender);

    // Initialize console logger so that it also sends to the file logger
    plog::init<AutonomyLogger::AL_ConsoleLogger>(plog::debug, &g_pConsoleAppender).addAppender(&g_pFileAppender);
}

Autonomy_IdentitySoftware g_pIdentifySoftware;

DriveBoard g_pDriveBoardInterface;
MultimediaBoard g_pMultimediaBoard;
NavigationBoard g_pNavigationBoard;
