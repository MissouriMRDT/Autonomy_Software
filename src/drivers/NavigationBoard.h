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

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        geoops::GPSCoordinate m_stLocation;     // Store current global position in UTM format.
        double m_dHeading;                      // Store current GPS heading.
        std::shared_mutex m_muLocationMutex;    // Mutex for acquiring read and write lock on location member variable.
        std::shared_mutex m_muHeadingMutex;     // Mutex for acquiring read and write lock on heading member variable.

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Callback function that is called whenever RoveComm receives new GPS data.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-03
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<double>&, const sockaddr_in&)> ProcessGPSData =
            [this](const rovecomm::RoveCommPacket<double>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stdAddr;

            // Acquire write lock for writing to GPS struct.
            std::unique_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
            // Repack data from RoveCommPacket into member variable.
            m_stLocation.dLatitude  = stPacket.vData[0];
            m_stLocation.dLongitude = stPacket.vData[1];
            m_stLocation.dAltitude  = stPacket.vData[2];
            // Unlock mutex.
            lkGPSProcessLock.unlock();

            // Submit logger message.
            LOG_DEBUG(logging::g_qSharedLogger, "Incoming GPS Data: ({} lat, {} lon, {} alt)", m_stLocation.dLatitude, m_stLocation.dLongitude, m_stLocation.dAltitude);
        };

        /******************************************************************************
         * @brief Callback function that is called whenever RoveComm receives new Accuracy data.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-03
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<float>&, const sockaddr_in&)> ProcessAccuracyData =
            [this](const rovecomm::RoveCommPacket<float>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stdAddr;

            // Acquire write lock for writing to GPS struct.
            std::unique_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
            // Repack data from RoveCommPacket into member variable.
            m_stLocation.d2DAccuracy = stPacket.vData[0];
            m_stLocation.d3DAccuracy = stPacket.vData[1];
            // Unlock mutex.
            lkGPSProcessLock.unlock();

            // Submit logger message.
            LOG_DEBUG(logging::g_qSharedLogger,
                      "Incoming Accuracy Data: (2D: {}, 3D: {}, Compass: {})",
                      m_stLocation.d2DAccuracy,
                      m_stLocation.d3DAccuracy,
                      stPacket.vData[2]);
        };

        /******************************************************************************
         * @brief Callback function that is called whenever RoveComm receives new Compass data.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-03
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<float>&, const sockaddr_in&)> ProcessCompassData =
            [this](const rovecomm::RoveCommPacket<float>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stdAddr;

            // Acquire write lock for writing to GPS struct.
            std::unique_lock<std::shared_mutex> lkCompassProcessLock(m_muHeadingMutex);
            // Repack data from RoveCommPacket into member variable.
            m_dHeading = stPacket.vData[0];
            // Unlock mutex.
            lkCompassProcessLock.unlock();

            // Submit logger message.
            LOG_DEBUG(logging::g_qSharedLogger, "Incoming Compass Data: {}", m_dHeading);
        };
};

#endif    // NAVIGATIONBOARD_H
