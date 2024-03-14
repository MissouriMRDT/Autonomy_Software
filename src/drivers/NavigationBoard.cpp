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
#include "../AutonomyGlobals.h"
#include "../AutonomyLogging.h"

/// \cond
// Put implicit #includes here.

/// \endcond

/******************************************************************************
 * @brief Construct a new Navigation Board:: Navigation Board object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
NavigationBoard::NavigationBoard()
{
    // Subscribe to NavBoard packets.
    rovecomm::RoveCommPacket<u_int8_t> stSubscribePacket;
    stSubscribePacket.unDataId    = manifest::System::SUBSCRIBE_DATA_ID;
    stSubscribePacket.unDataCount = 0;
    stSubscribePacket.eDataType   = manifest::DataTypes::UINT8_T;
    stSubscribePacket.vData       = std::vector<uint8_t>{};
    globals::g_pRoveCommUDPNode->SendUDPPacket(stSubscribePacket, manifest::Nav::IP_ADDRESS.IP_STR.c_str(), constants::ROVECOMM_UDP_PORT);

    // Set RoveComm callbacks.
    globals::g_pRoveCommUDPNode->AddUDPCallback<double>(ProcessGPSData, constants::ROVECOMM_UDP_PORT);
    globals::g_pRoveCommUDPNode->AddUDPCallback<float>(ProcessAccuracyData, constants::ROVECOMM_UDP_PORT);
    globals::g_pRoveCommUDPNode->AddUDPCallback<float>(ProcessCompassData, constants::ROVECOMM_UDP_PORT);
}

/******************************************************************************
 * @brief Destroy the Navigation Board:: Navigation Board object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
NavigationBoard::~NavigationBoard() {}

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
    // Acquire read lock for getting GPS struct.
    std::shared_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
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
    std::shared_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
    // Convert the currently stored GPS coord to UTM and return.
    return geoops::ConvertGPSToUTM(m_stLocation);
}

/******************************************************************************
 * @brief Accessor for the most recent compass heading received from the NavBoard.
 *
 * @return double - The last known compass heading.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-03
 ******************************************************************************/
double NavigationBoard::GetHeading()
{
    // Acquire read lock for getting compass double.
    std::shared_lock<std::shared_mutex> lkCompassProcessLock(m_muHeadingMutex);
    return m_dHeading;
}

/******************************************************************************
 * @brief The rover's current velocity based off of the distance covered over the
 *      last two GPSCoordinates.
 *
 * @return double - The rover's velocity in meters per second.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-13
 ******************************************************************************/
double NavigationBoard::GetVelocity()
{
    // Acquire read lock for getting velocity double.
    std::shared_lock<std::shared_mutex> lkVelocityProcessLock(m_muVelocityMutex);
    return m_dVelocity;
}

/******************************************************************************
 * @brief The rover's current angular velocity based off of the change in angle over the
 *      last two headings.
 *
 * @return double - The rover's angular velocity in degrees per second.
 *
 * @author Jason Pittman (jspencerpittman@gmail.com)
 * @date 2024-03-14
 ******************************************************************************/
double NavigationBoard::GetAngularVelocity()
{
    // Acquire read lock for getting angular velocity double.
    std::shared_lock<std::shared_mutex> lkVelocityProcessLock(m_muAngularVelocityMutex);
    return m_dAngularVelocity;
}

/******************************************************************************
 * @brief A chrono timestamp storing the last time autonomy's GPS location was updated
 *      over RoveComm via the NavBoard.
 *
 * @return std::chrono::system_clock::time_point - The timestamp that the current GPSCoordinate location was updated.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-13
 ******************************************************************************/
std::chrono::system_clock::time_point NavigationBoard::GetGPSTimestamp()
{
    // Acquire read lock for getting GPS timestamp.
    std::shared_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
    return m_tmLastGPSUpdateTime;
}

/******************************************************************************
 * @brief A chrono timestamp storing the last time autonomy's compass location was updated
 *      over RoveComm via the NavBoard.
 *
 * @return std::chrono::system_clock::time_point - The timestamp that the current heading was updated.
 *
 * @author Jason Pittman (jspencerpittman@gmail.com)
 * @date 2024-03-14
 ******************************************************************************/
std::chrono::system_clock::time_point NavigationBoard::GetCompassTimestamp()
{
    // Acquire read lock for getting Heading timestamp.
    std::shared_lock<std::shared_mutex> lkCompassProcessLock(m_muHeadingMutex);
    return m_tmLastCompassUpdateTime;
}
