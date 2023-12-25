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

#include "../util/GeospatialOperations.hpp"

#include <shared_mutex>

/******************************************************************************
 * @brief This class handles communication with the navigation board on the rover
 *      by sending RoveComm packets over the network.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
class NavigationBoard
{
    public:
        /////////////////////////////////////////
        // Declare public enums and structs that are specific to and used withing this class.
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////

        NavigationBoard();
        ~NavigationBoard();

        /////////////////////////////////////////
        // Setters
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Getters
        /////////////////////////////////////////

        geoops::IMUData GetIMUData();
        geoops::GPSCoordinate GetGPSData();
        geoops::UTMCoordinate GetUTMData();

    private:
        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ProcessIMUData(geoops::IMUData stPacket);
        void ProcessGPSData(geoops::GPSCoordinate stPacket);

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        geoops::UTMCoordinate m_stLocation;        // Store current global position in UTM format.
        geoops::IMUData m_stOrientation;           // Store current IMU orientation.
        std::shared_mutex m_muLocationMutex;       // Mutex for acquiring read and write lock on location member variable.
        std::shared_mutex m_muOrientationMutex;    // Mutex for acquiring read and write lock on orientation member variable.

                                                   // TODO: RoveComm Node

        // TODO: RoveComm Callback -> IMU Data
        // TODO: RoveComm Callback -> GPS Data
};

#endif    // NAVIGATIONBOARD_H
