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
 * @brief This struct stores/contains information about an IMU packet recieved
 *      from RoveComm.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
struct NavigationBoard::IMUData
{
    public:
        // Declare struct public member variables.
        double dPitch;
        double dRoll;
        double dHeading;

        /******************************************************************************
         * @brief Construct a new IMUData object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-09-23
         ******************************************************************************/
        IMUData()
        {
            // Initialize member variables to default values.
            dPitch   = 0.0;
            dRoll    = 0.0;
            dHeading = 0.0;
        }

        /******************************************************************************
         * @brief Construct a new IMUData object.
         *
         * @param dPitch - The pitch of the navboard in degrees.
         * @param dRoll - The roll of the navboard in degrees.
         * @param dHeading - The heading/yaw of the navboard in degrees.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-09-23
         ******************************************************************************/
        IMUData(double dPitch, double dRoll, double dHeading)
        {
            // Initialize member variables with given values.
            this->dPitch   = dPitch;
            this->dRoll    = dRoll;
            this->dHeading = dHeading;
        }
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * @brief Construct a new Navigation Board:: Navigation Board object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
NavigationBoard::NavigationBoard()
{
    m_dPitch   = 0;
    m_dRoll    = 0;
    m_dHeading = 0;
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
 * @brief Unpack and store data from an IMU packet.
 *
 * @param stPacket - The special nav board packet containing IMU data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
void NavigationBoard::ProcessIMUData(IMUData stPacket)
{
    m_dPitch   = stPacket.dPitch;
    m_dRoll    = stPacket.dRoll;
    m_dHeading = stPacket.dHeading;

    LOG_DEBUG(logging::g_qSharedLogger, "Incoming IMU Data: ({}, {}, {})", m_dPitch, m_dRoll, m_dHeading);
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
 * @brief Accessor for most recent IMU data recieved from NavBoard.
 *
 * @return NavigationBoard::IMUData - Struct containing roll, pitch, and yaw/heading info.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
NavigationBoard::IMUData NavigationBoard::GetIMUData() const
{
    //
}

/******************************************************************************
 * @brief Accessor for most recent GPS data recieved from NavBoard.
 *
 * @return geoops::GPSCoordinate - Struct containing lat, lon, alt, and accuracy data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
geoops::GPSCoordinate NavigationBoard::GetGPSData() const
{
    //
}

/******************************************************************************
 * @brief Accesor for most recent GPS data recieved from NavBoard converted to UTM coords.
 *
 * @return geoops::UTMCoordinate - Struct containing easting, northing, alt, zone,
 *                                          and accuracy data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
geoops::UTMCoordinate NavigationBoard::GetUTMData() const
{
    //
}
