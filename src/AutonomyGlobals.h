/******************************************************************************
 * @brief Defines functions and objects used project wide.
 *
 * @file AutonomyGlobals.h
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef AUTONOMY_GLOBALS_H
#define AUTONOMY_GLOBALS_H

#include "AutonomyConstants.h"
#include "IdentitySoftware.h"
#include "drivers/DriveBoard.h"
#include "drivers/MultimediaBoard.h"
#include "drivers/NavigationBoard.h"
#include "threads/CameraHandlerThread.h"

#include <chrono>
#include <ctime>
#include <iostream>

#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/transition.hpp>

#include <boost/mpl/list.hpp>

namespace sc  = boost::statechart;
namespace mpl = boost::mpl;

// Versioning:
extern IdentitySoftware g_pIdentifySoftware;    // Global Version Handler

// Camera Handler:
extern CameraHandlerThread* g_pCameraHandler;

// Board Interfaces:
extern DriveBoard g_pDriveBoardInterface;              // Global Drive Board Interface
extern MultimediaBoard g_pMultimediaBoardInterface;    // Global Multimedia Board Interface
extern NavigationBoard g_pNavigationBoardInterface;    // Global Navigation Board Interface

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
    /******************************************************************************
     * @brief This struct stores/contains information about a GPS data.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-23
     ******************************************************************************/
    struct GPSCoordinate
    {
        public:
            // Declare struct public attributes
            double dLatitude;
            double dLongitude;
            double dAltitude;
            double d2DAccuracy;
            double d3DAccuracy;

            /////////////////////////////////////////
            // Declare public methods.
            /////////////////////////////////////////
            GPSCoordinate();
            GPSCoordinate(double dLatitude, double dLongitude);
            GPSCoordinate(double dLatitude, double dLongitude, double Altitude);
            GPSCoordinate(double dLatitude, double dLongitude, double dAltitude, double d2DAccurary, double d3DAccuracy);
    };

    /******************************************************************************
     * @brief This struct stores/contains information about a UTM coordinate.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-23
     ******************************************************************************/
    struct UTMCoordinate
    {
        public:
            // Declare struct public attributes.
            double dEasting;
            double dNorthing;
            double dAltitude;
            double dZone;
            double d2DAccuracy;
            double d3DAccuracy;
    };
}    // namespace globals

#endif    // AUTONOMY_GLOBALS_H
