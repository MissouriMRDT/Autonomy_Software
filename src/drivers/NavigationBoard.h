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
        void ProcessIMUData(geoops::IMUData stPacket);
        void ProcessGPSData(geoops::GPSCoordinate stPacket);

        /////////////////////////////////////////
        // Setters
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Getters
        /////////////////////////////////////////
        geoops::IMUData GetIMUData() const;
        geoops::GPSCoordinate GetGPSData() const;
        geoops::UTMCoordinate GetUTMData() const;

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        geoops::UTMCoordinate m_stLocation;
        geoops::IMUData m_stOrientation;

        // TODO: RoveComm Node

        // TODO: RoveComm Callback -> IMU Data
        // TODO: RoveComm Callback -> GPS Data
};

#endif    // NAVIGATIONBOARD_H
