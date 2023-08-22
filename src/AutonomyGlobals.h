/******************************************************************************
 * @brief Defines functions and objects used project wide.
 *
 * @file AutonomyGlobals.h
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "AutonomyConstants.h"
#include "IdentitySoftware.h"
#include "drivers/DriveBoard.h"
#include "drivers/MultimediaBoard.h"
#include "drivers/NavigationBoard.h"

#include <chrono>
#include <ctime>
#include <iostream>

#include <quill/Quill.h>

#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/transition.hpp>

#include <boost/mpl/list.hpp>

namespace sc  = boost::statechart;
namespace mpl = boost::mpl;

#ifndef AUTONOMY_GLOBALS_H
#define AUTONOMY_GLOBALS_H

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

#endif    // AUTONOMY_GLOBALS_H
