/******************************************************************************
 * @brief Implements NavigationBoard class.
 *
 * @file NavigationBoard.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "NavigationBoard.h"
#include "../AutonomyLogging.h"

/******************************************************************************
 * @brief Construct a new Navigation Board:: Navigation Board object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
NavigationBoard::NavigationBoard() {}

/******************************************************************************
 * @brief Destroy the Navigation Board:: Navigation Board object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
NavigationBoard::~NavigationBoard() {}

/******************************************************************************
 * @brief Unpack and store data from an IMU packet.
 *
 * @param stPacket - The special nav board packet containing IMU data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
void NavigationBoard::ProcessIMUData(geoops::IMUData stPacket)
{
    // Acquire write lock for writing to IMU struct.
    std::unique_lock<std::shared_mutex> lkIMUProcessLock(m_muOrientationMutex);
    // Update member variables attributes.
    m_stOrientation.dPitch   = stPacket.dPitch;
    m_stOrientation.dRoll    = stPacket.dRoll;
    m_stOrientation.dHeading = stPacket.dHeading;

    // Submit logger message.
    LOG_DEBUG(logging::g_qSharedLogger, "Incoming IMU Data: ({}, {}, {})", m_stOrientation.dPitch, m_stOrientation.dRoll, m_stOrientation.dHeading);
}

/******************************************************************************
 * @brief Unpack and store data from a GPS packet.
 *
 * @param stPacket - The special nav board struct containing GPS data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
void NavigationBoard::ProcessGPSData(geoops::GPSCoordinate stPacket)
{
    // Acquire write lock for writing to GPS struct.
    std::unique_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
    // Submit logger message.
    LOG_DEBUG(logging::g_qSharedLogger,
              "Incoming GPS Data: ({} lat, {} lon, {} alt, {} acc)",
              stPacket.dLatitude,
              stPacket.dLongitude,
              stPacket.dAltitude,
              stPacket.d2DAccuracy);

    // Store GPS data in member variable.
    m_stLocation = stPacket;
}

/******************************************************************************
 * @brief Accessor for most recent IMU data received from NavBoard.
 *
 * @return geoops::IMUData - Struct containing roll, pitch, and yaw/heading info.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
geoops::IMUData NavigationBoard::GetIMUData()
{
    // Acquire read lock for getting IMU struct.
    std::shared_lock<std::shared_mutex> lkIMUProcessLock(m_muOrientationMutex);
    // Return the orientation struct member variable.
    return m_stOrientation;
}

/******************************************************************************
 * @brief Accessor for most recent GPS data received from NavBoard.
 *
 * @return geoops::GPSCoordinate - Struct containing lat, lon, alt, and accuracy data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
geoops::GPSCoordinate NavigationBoard::GetGPSData()
{
    // Acquire read lock for getting UTM struct.
    std::shared_lock<std::shared_mutex> lkIMUProcessLock(m_muLocationMutex);
    // Convert the currently stored UTM coord to GPS and return.
    return m_stLocation;
}

/******************************************************************************
 * @brief Accessor for most recent GPS data received from NavBoard converted to UTM coords.
 *
 * @return geoops::UTMCoordinate - Struct containing easting, northing, alt, zone,
 *                                          and accuracy data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
geoops::UTMCoordinate NavigationBoard::GetUTMData()
{
    // Acquire read lock for getting UTM struct.
    std::shared_lock<std::shared_mutex> lkIMUProcessLock(m_muLocationMutex);
    return geoops::ConvertGPSToUTM(m_stLocation);
}
