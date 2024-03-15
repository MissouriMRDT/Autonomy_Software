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
    globals::g_pRoveCommUDPNode->SendUDPPacket(stSubscribePacket, manifest::Nav::IP_ADDRESS.IP_STR.c_str(), constants::ROVECOMM_OUTGOING_UDP_PORT);

    // Set RoveComm callbacks.
    globals::g_pRoveCommUDPNode->AddUDPCallback<double>(ProcessGPSData, manifest::Nav::TELEMETRY.find("GPSLATLONALT")->second.DATA_ID);
    globals::g_pRoveCommUDPNode->AddUDPCallback<float>(ProcessAccuracyData, manifest::Nav::TELEMETRY.find("ACCURACYDATA")->second.DATA_ID);
    globals::g_pRoveCommUDPNode->AddUDPCallback<float>(ProcessCompassData, manifest::Nav::TELEMETRY.find("COMPASSDATA")->second.DATA_ID);
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
    // Acquire read lock for getting UTM struct.
    std::shared_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
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
    std::shared_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
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
