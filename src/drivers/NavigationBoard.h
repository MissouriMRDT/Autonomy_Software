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

/// \cond
#include <RoveComm/RoveComm.h>
#include <RoveComm/RoveCommManifest.h>
#include <chrono>
#include <shared_mutex>

/// \endcond

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

        geoops::GPSCoordinate GetGPSData();
        geoops::UTMCoordinate GetUTMData();
        double GetHeading();
        double GetHeadingAccuracy();
        double GetVelocity();
        double GetAngularVelocity();
        std::chrono::system_clock::duration GetGPSLastUpdateTime();
        std::chrono::system_clock::duration GetCompassLastUpdateTime();
        bool IsOutOfDate();

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        geoops::GPSCoordinate m_stLocation;                                 // Store current global position in UTM format.
        double m_dHeading;                                                  // Store current GPS heading.
        double m_dHeadingAccuracy;                                          // Store current GPS heading accuracy in degrees.
        double m_dVelocity;                                                 // Store current GPS-based velocity.
        double m_dAngularVelocity;                                          // Store current compass-based angular velocity.
        std::shared_mutex m_muLocationMutex;                                // Mutex for acquiring read and write lock on location member variable.
        std::shared_mutex m_muHeadingMutex;                                 // Mutex for acquiring read and write lock on heading member variable.
        std::shared_mutex m_muVelocityMutex;                                // Mutex for acquiring read and write lock on velocity member variable.
        std::shared_mutex m_muAngularVelocityMutex;                         // Mutex for acquiring read and write lock on angular velocity member variable.
        std::chrono::system_clock::time_point m_tmLastGPSUpdateTime;        // A time point for storing the timestamp of the last GPS update. Also used for velocity.
        std::chrono::system_clock::time_point m_tmLastCompassUpdateTime;    // A time point for storing the time of the last compass update. Used for angular velocity.
        bool m_bNavBoardOutOfDate;                                          // A boolean to store whether the GPS is out of date.

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ProcessGPSData(const rovecomm::RoveCommPacket<double>& stPacket);
        void ProcessAccuracyData(const rovecomm::RoveCommPacket<float>& stPacket);
        void ProcessCompassData(const rovecomm::RoveCommPacket<float>& stPacket);
};

#endif    // NAVIGATIONBOARD_H
