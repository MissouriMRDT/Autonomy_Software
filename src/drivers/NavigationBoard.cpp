/******************************************************************************
 * @brief Implements NavigationBoard class.
 *
 * @file NavigationBoard.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "NavigationBoard.h"

#include "../Autonomy_Globals.h"

/******************************************************************************
 * @brief Construct a new Navigation Board:: Navigation Board object.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
NavigationBoard::NavigationBoard()
{
    m_dPitch            = 0;
    m_dRoll             = 0;
    m_dHeading          = 0;

    m_sLocation         = {0, 0};

    m_iDistanceToGround = 0;
    m_iLidarQuality     = 0;

    m_tLastTime         = time(nullptr);
}

/******************************************************************************
 * @brief Destroy the Navigation Board:: Navigation Board object.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
NavigationBoard::~NavigationBoard() {}

/******************************************************************************
 * @brief Unpack and store data from an IMU packet.
 *
 * @param packet - The special nav board packet containing IMU data.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
void NavigationBoard::ProcessIMUData(NavBoardPacket_IMU packet)
{
    m_dPitch   = packet.dPitch;
    m_dRoll    = packet.dRoll;
    m_dHeading = packet.dHeading;

    PLOG_DEBUG_(AL_ConsoleLogger) << "Incoming IMU Data: (" << m_dPitch << ", " << m_dRoll << ", " << m_dHeading << ")";
}

/******************************************************************************
 * @brief Unpack and store data from a GPS packet.
 *
 * @param packet - The special nav board packet containing GPS data.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
void NavigationBoard::ProcessGPSData(NavBoardPacket_GPS packet)
{
    m_sLocation.dLatitude  = packet.dLatitude;
    m_sLocation.dLongitude = packet.dLongitude;

    m_tLastTime            = time(nullptr);

    PLOG_DEBUG_(AL_ConsoleLogger) << "Incoming GPS Data: (" << m_sLocation.dLatitude << ", " << m_sLocation.dLongitude << ")";
}

/******************************************************************************
 * @brief Get stored nav board IMU data.
 *
 * @param eKey - Enum detailing which component to retrieve.
 * @return double - The Roll, Pitch, or Heading data.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
double NavigationBoard::GetDData(NavigationBoardPacketDoubleComponents eKey) const
{
    double dValue = 0.0;

    switch (eKey)
    {
        case NBPC_PITCH: dValue = m_dPitch; break;
        case NBPC_ROLL: dValue = m_dRoll; break;
        case NBPC_HEADING: dValue = m_dHeading; break;
        default: break;
    }

    return dValue;
}

/******************************************************************************
 * @brief Get stored GPS data.
 *
 * @param eKey - Enum detailing which data to retrieve.
 * @return NavBoardPacket_GPS - Struct storing GPS data.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
NavBoardPacket_GPS NavigationBoard::GetSData(NavigationBoardPacketCoordinateComponents eKey) const
{
    NavBoardPacket_GPS sValue = NavBoardPacket_GPS();

    switch (eKey)
    {
        case NBPCC_LOCATION: sValue = m_sLocation; break;
        default: break;
    }

    return sValue;
}
