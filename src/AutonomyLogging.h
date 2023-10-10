/******************************************************************************
 * @brief Implements Logging for Autonomy
 *
 *        Note: Functions and Variables are defined in AutonomyGlobals.cpp
 *              by having the declarations occur in a seperate header we are
 *              more easily able to use the functionallity of the logger in
 *              areas of the program that would normally be unaccessable
 *              due to it being included in the Main Globals Header.
 *
 * @file AutonomyLogging.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-08-22
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include <quill/Quill.h>

#ifndef AUTONOMY_LOGGING_H
#define AUTONOMY_LOGGING_H

/******************************************************************************
 * @brief Logging Levels:
 *
 *        Priority > Level     > Description
 *        Level 1  > TRACE_L3  > Unused
 *        Level 2  > TRACE_L2  > Unused
 *        Level 3  > TRACE_L1  > Unused
 *        Level 4  > DEBUG     > Details that would only be useful in a debug environment
 *        Level 5  > INFO      > State Changes, RoveComm Updates GPS/IMU/Autonomy, etc
 *        Level 6  > WARNING   > Something unexpected happened - application could potentially error soon.
 *        Level 7  > ERROR     > Something went wrong - application could potentially have critical error soon.
 *        Level 8  > CRITICAL  > Something went very wrong - application will exit after logging is sent.
 *
 *        Note: At testing sessions we will have "DEBUG" Logging set as the level that is being outputed.
 *              However, at competition and when using "RELEASE" code we will be using "INFO" Logging. When
 *              a logging level is set, we only recieve logging messages that are that level or higher
 *              priority.
 *
 *        Example: When INFO is set, we only recieve: INFO, WARNING, ERROR, CRITICAL
 *                 When DEBUG is set, we only recieve: DEBUG, INFO, WARNING, ERROR, CRITICAL
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-08-22
 ******************************************************************************/

/******************************************************************************
 * @brief File Logger - Send logging message to only a file
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-08-22
 ******************************************************************************/
extern quill::Logger* g_qFileLogger;

/******************************************************************************
 * @brief Console Logger - Send logging message to only the console
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-08-22
 ******************************************************************************/
extern quill::Logger* g_qConsoleLogger;

/******************************************************************************
 * @brief Shared Logger - Send logging message to both the consle and a file.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-08-22
 ******************************************************************************/
extern quill::Logger* g_qSharedLogger;

/******************************************************************************
 * @brief Logger Initializer - Sets Up all the logging handlers required for
 *        having the above loggers.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-08-22
 ******************************************************************************/
void InitializeLoggers();

//detect USB
//initialize input and output stream
//while loop iterate through input file
	//parse and log input file onto USB

#endif    // AUTONOMY_LOGGING_H
