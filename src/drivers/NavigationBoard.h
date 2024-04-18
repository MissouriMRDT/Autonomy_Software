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
        std::chrono::system_clock::duration GetGPSDataAge();
        std::chrono::system_clock::duration GetCompassDataAge();

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

            // Get current time.
            std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
            // Acquire read lock for getting GPS struct.
            std::shared_lock<std::shared_mutex> lkGPSReadProcessLock(m_muLocationMutex);
            // Calculate distance of new GPS coordinate to old GPS coordinate.
            geoops::GeoMeasurement geMeasurement =
                geoops::CalculateGeoMeasurement(m_stLocation, geoops::GPSCoordinate(stPacket.vData[0], stPacket.vData[1], stPacket.vData[2]));
            // Unlock mutex.
            lkGPSReadProcessLock.unlock();

            // Acquire write lock for writing to velocity member variable.
            std::unique_lock<std::shared_mutex> lkVelocityProcessLock(m_muVelocityMutex);
            // Calculate rover velocity based on GPS distance traveled over time.
            m_dVelocity = geMeasurement.dDistanceMeters / (std::chrono::duration_cast<std::chrono::microseconds>(tmCurrentTime - m_tmLastGPSUpdateTime).count() / 1e6);
            // Unlock mutex.
            lkVelocityProcessLock.unlock();

            // Acquire write lock for writing to GPS struct.
            std::unique_lock<std::shared_mutex> lkGPSWriteProcessLock(m_muLocationMutex);
            // Repack data from RoveCommPacket into member variable.
            m_stLocation.dLatitude  = stPacket.vData[0];
            m_stLocation.dLongitude = stPacket.vData[1];
            m_stLocation.dAltitude  = stPacket.vData[2];
            // Update GPS time.
            m_tmLastGPSUpdateTime = tmCurrentTime;
            // Unlock mutex.
            lkGPSWriteProcessLock.unlock();

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
            std::unique_lock<std::shared_mutex> lkCompassProcessLock(m_muHeadingMutex);
            // Repack data from RoveCommPacket into member variable.
            m_stLocation.d2DAccuracy                = std::fabs(stPacket.vData[0]);
            m_stLocation.d3DAccuracy                = std::fabs(stPacket.vData[1]);
            m_dHeadingAccuracy                      = std::fabs(stPacket.vData[2]);
            m_stLocation.eCoordinateAccuracyFixType = static_cast<geoops::PositionFixType>(stPacket.vData[3]);
            m_stLocation.bIsDifferential            = static_cast<bool>(stPacket.vData[4]);
            // Unlock mutex.
            lkCompassProcessLock.unlock();
            lkGPSProcessLock.unlock();

            // Submit logger message.
            LOG_DEBUG(logging::g_qSharedLogger,
                      "Incoming Accuracy Data: (2D: {}, 3D: {}, Compass: {}, FIX_TYPE: {}, Differential?: {})",
                      stPacket.vData[0],
                      stPacket.vData[1],
                      stPacket.vData[2],
                      stPacket.vData[3],
                      stPacket.vData[4]);
        };

        /******************************************************************************
         * @brief Callback function that is called whenever RoveComm receives new Compass data.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com), Jason Pittman (jspencerpittman@gmail.com)
         * @date 2024-03-03
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<float>&, const sockaddr_in&)> ProcessCompassData =
            [this](const rovecomm::RoveCommPacket<float>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stdAddr;

            // Get current time.
            std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();

            // Acquire read lock for heading.
            std::shared_lock<std::shared_mutex> lkCompassReadLock(m_muHeadingMutex);
            // Calculate the total change in angle with respect to the last recorded heading.
            double dNewHeading = stPacket.vData[0];
            double dDeltaAngle = dNewHeading - m_dHeading;
            // Assume that the change in angle can't be greater than 180 degrees in a single timestep.
            // This accounts for changes in angle across the 0/360 degree line.
            if (std::abs(dDeltaAngle) > 180)
            {
                dDeltaAngle = dNewHeading > m_dHeading ? -(360 - dDeltaAngle) : 360 + dDeltaAngle;
            }
            // Unlock mutex.
            lkCompassReadLock.unlock();

            // Acquire write lock for writing to angular velocity member variable.
            std::unique_lock<std::shared_mutex> lkAngularVelocityProcessLock(m_muAngularVelocityMutex);
            // Calculate rover angular velocity based on change in heading over time.
            m_dAngularVelocity = dDeltaAngle / (std::chrono::duration_cast<std::chrono::microseconds>(tmCurrentTime - m_tmLastCompassUpdateTime).count() / 1e6);
            // Unlock mutex.
            lkAngularVelocityProcessLock.unlock();

            // Acquire write lock for heading and compass timestamp.
            std::unique_lock<std::shared_mutex> lkCompassProcessLock(m_muHeadingMutex);
            // Repack data from RoveCommPacket into member variable.
            m_dHeading = dNewHeading;
            // Update compass time.
            m_tmLastCompassUpdateTime = tmCurrentTime;
            // Unlock mutex.
            lkCompassProcessLock.unlock();

            // Submit logger message.
            LOG_DEBUG(logging::g_qSharedLogger, "Incoming Compass Data: {}", m_dHeading);
        };
};

#endif    // NAVIGATIONBOARD_H
