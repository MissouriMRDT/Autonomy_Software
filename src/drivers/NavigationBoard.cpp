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
    // Submit logger message.
    LOG_DEBUG(logging::g_qSharedLogger,
              "Incoming GPS Data: ({} lat, {} lon, {} alt, {} acc)",
              stPacket.dLatitude,
              stPacket.dLongitude,
              stPacket.dAltitude,
              stPacket.d2DAccuracy);
}

/******************************************************************************
 * @brief Accessor for most recent IMU data received from NavBoard.
 *
 * @return geoops::IMUData - Struct containing roll, pitch, and yaw/heading info.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
geoops::IMUData NavigationBoard::GetIMUData() const
{
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
geoops::GPSCoordinate NavigationBoard::GetGPSData() const
{
    // TODO: Return empty coordinate for now.
    return geoops::GPSCoordinate();
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
geoops::UTMCoordinate NavigationBoard::GetUTMData() const
{
    // TODO: Return empty coordinate for now.
    return geoops::UTMCoordinate();
}
