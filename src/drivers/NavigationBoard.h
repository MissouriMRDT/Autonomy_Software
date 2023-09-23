/******************************************************************************
 * @brief Defines the NavigationBoard class.
 *
 * @file NavigationBoard.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef NAVIGATIONBOARD_H
#define NAVIGATIONBOARD_H

#include "../AutonomyGlobals.h"
#include <chrono>
#include <ctime>

class NavigationBoard
{
    public:
        /////////////////////////////////////////
        // Declare public enums and structs that are specific to and used withing this class.
        /////////////////////////////////////////

        struct IMUData;

        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////
        NavigationBoard();
        ~NavigationBoard();
        void ProcessIMUData(IMUData stPacket);
        void ProcessGPSData(globals::GPSCoordinate stPacket);

        /////////////////////////////////////////
        // Setters
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Getters
        /////////////////////////////////////////
        IMUData GetIMUData() const;
        globals::GPSCoordinate GetGPSData() const;
        globals::UTMCoordinate GetUTMData() const;

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        double m_dPitch;
        double m_dRoll;
        double m_dHeading;

        // TODO: RoveComm Node

        // TODO: RoveComm Callback -> IMU Data
        // TODO: RoveComm Callback -> GPS Data
};

#endif    // NAVIGATIONBOARD_H
