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
#include "../AutonomyNetworking.h"

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
    // Initialize member variables.
    m_stLocation              = geoops::GPSCoordinate(37.951771, -91.778114, 315.0);
    m_tmLastGPSUpdateTime     = std::chrono::system_clock::now();
    m_tmLastCompassUpdateTime = std::chrono::system_clock::now();
    m_dHeading                = 0.0;
    m_dHeadingAccuracy        = 0.0;
    m_dVelocity               = 0.0;
    m_dAngularVelocity        = 0.0;
    m_bNavBoardOutOfDate      = false;

    // Subscribe to NavBoard packets.
    rovecomm::RoveCommPacket<u_int8_t> stSubscribePacket;
    stSubscribePacket.unDataId    = manifest::System::SUBSCRIBE_DATA_ID;
    stSubscribePacket.unDataCount = 0;
    stSubscribePacket.eDataType   = manifest::DataTypes::UINT8_T;
    stSubscribePacket.vData       = std::vector<uint8_t>{};
    network::g_pRoveCommUDPNode->SendUDPPacket(stSubscribePacket, manifest::Nav::IP_ADDRESS.IP_STR.c_str(), constants::ROVECOMM_OUTGOING_UDP_PORT);

    // Set RoveComm callbacks.
    network::g_pRoveCommUDPNode->AddUDPCallback<double>(ProcessGPSData, manifest::Nav::TELEMETRY.find("GPSLATLONALT")->second.DATA_ID);
    network::g_pRoveCommUDPNode->AddUDPCallback<float>(ProcessAccuracyData, manifest::Nav::TELEMETRY.find("ACCURACYDATA")->second.DATA_ID);
    network::g_pRoveCommUDPNode->AddUDPCallback<float>(ProcessCompassData, manifest::Nav::TELEMETRY.find("COMPASSDATA")->second.DATA_ID);
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
    // Create static boolean for printing out warnings.
    static bool bAlreadyPrintedWarning = false;

    // Acquire read lock for getting GPS struct.
    std::shared_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
    // Calculate time elapsed since last GPS data update.
    int nGPSDataAge = std::chrono::duration_cast<std::chrono::seconds>(this->GetGPSLastUpdateTime()).count();
    // Check the last time that our current GPS data has been updated.
    if (nGPSDataAge >= constants::NAVBOARD_MAX_GPS_DATA_AGE && !bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Current GPS data is out of date! GPS timestamp is {} seconds old!", nGPSDataAge);
        // Set toggle.
        bAlreadyPrintedWarning = true;
        // Set Out of Date.
        m_bNavBoardOutOfDate = true;
    }
    else if (nGPSDataAge < constants::NAVBOARD_MAX_GPS_DATA_AGE && bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "GPS data recovered!");
        // Reset toggle.
        bAlreadyPrintedWarning = false;
        // Reset Out of Date.
        m_bNavBoardOutOfDate = false;
    }

    // Return current GPS location.
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
    // Create static boolean for printing out warnings.
    static bool bAlreadyPrintedWarning = false;

    // Acquire read lock for getting UTM struct.
    std::shared_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
    // Calculate time elapsed since last GPS data update.
    int nGPSDataAge = std::chrono::duration_cast<std::chrono::seconds>(this->GetGPSLastUpdateTime()).count();
    // Check the last time that our current GPS data has been updated.
    if (nGPSDataAge >= constants::NAVBOARD_MAX_GPS_DATA_AGE && !bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Current GPS data is out of date! GPS timestamp is {} seconds old!", nGPSDataAge);
        // Set toggle.
        bAlreadyPrintedWarning = true;
    }
    else if (nGPSDataAge < constants::NAVBOARD_MAX_GPS_DATA_AGE && bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "GPS data recovered!");
        // Reset toggle.
        bAlreadyPrintedWarning = false;
    }

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
    // Create static boolean for printing out warnings.
    static bool bAlreadyPrintedWarning = false;

    // Acquire read lock for getting compass double.
    std::shared_lock<std::shared_mutex> lkCompassProcessLock(m_muHeadingMutex);
    // Calculate time elapsed since last GPS data update.
    int nCompassDataAge = std::chrono::duration_cast<std::chrono::seconds>(this->GetCompassLastUpdateTime()).count();
    // Check the last time that our current GPS data has been updated.
    if (nCompassDataAge >= constants::NAVBOARD_MAX_COMPASS_DATA_AGE && !bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Current Compass data is out of date! Compass timestamp is {} seconds old!", nCompassDataAge);
        // Set toggle.
        bAlreadyPrintedWarning = true;
    }
    else if (nCompassDataAge < constants::NAVBOARD_MAX_COMPASS_DATA_AGE && bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Compass data recovered!");
        // Reset toggle.
        bAlreadyPrintedWarning = false;
    }

    // Return current Compass data.
    return m_dHeading;
}

/******************************************************************************
 * @brief Accessor for the most recent compass heading accuracy received from NavBoard.
 *
 * @return double - The last know compass heading accuracy.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-08
 ******************************************************************************/
double NavigationBoard::GetHeadingAccuracy()
{
    // Create static boolean for printing out warnings.
    static bool bAlreadyPrintedWarning = false;

    // Acquire read lock for getting compass double.
    std::shared_lock<std::shared_mutex> lkCompassProcessLock(m_muHeadingMutex);
    // Calculate time elapsed since last GPS data update.
    int nCompassDataAge = std::chrono::duration_cast<std::chrono::seconds>(this->GetCompassLastUpdateTime()).count();
    // Check the last time that our current GPS data has been updated.
    if (nCompassDataAge >= constants::NAVBOARD_MAX_COMPASS_DATA_AGE && !bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Current Compass data is out of date! Compass timestamp is {} seconds old!", nCompassDataAge);
        // Set toggle.
        bAlreadyPrintedWarning = true;
    }
    else if (nCompassDataAge < constants::NAVBOARD_MAX_COMPASS_DATA_AGE && bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Compass data recovered!");
        // Reset toggle.
        bAlreadyPrintedWarning = false;
    }

    // Return current Compass data.
    return m_dHeadingAccuracy;
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
    // Create static boolean for printing out warnings.
    static bool bAlreadyPrintedWarning = false;

    // Acquire read lock for getting velocity double.
    std::shared_lock<std::shared_mutex> lkVelocityProcessLock(m_muVelocityMutex);
    // Calculate time elapsed since last GPS data update.
    int nGPSDataAge = std::chrono::duration_cast<std::chrono::seconds>(this->GetGPSLastUpdateTime()).count();
    // Check the last time that our current GPS data has been updated.
    if (nGPSDataAge >= constants::NAVBOARD_MAX_GPS_DATA_AGE && !bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Current Velocity data is out of date! GPS timestamp is {} seconds old!", nGPSDataAge);
        // Set toggle.
        bAlreadyPrintedWarning = true;
    }
    else if (nGPSDataAge < constants::NAVBOARD_MAX_GPS_DATA_AGE && bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "GPS data recovered!");
        // Reset toggle.
        bAlreadyPrintedWarning = false;
    }

    // Return current velocity.
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
    // Create static boolean for printing out warnings.
    static bool bAlreadyPrintedWarning = false;

    // Acquire read lock for getting angular velocity double.
    std::shared_lock<std::shared_mutex> lkAngularVelocityProcessLock(m_muAngularVelocityMutex);
    // Calculate time elapsed since last GPS data update.
    int nCompassDataAge = std::chrono::duration_cast<std::chrono::seconds>(this->GetCompassLastUpdateTime()).count();
    // Check the last time that our current GPS data has been updated.
    if (nCompassDataAge >= constants::NAVBOARD_MAX_COMPASS_DATA_AGE && !bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Current Angular Velocity data is out of date! Compass timestamp is {} seconds old!", nCompassDataAge);
        // Set toggle.
        bAlreadyPrintedWarning = true;
    }
    else if (nCompassDataAge < constants::NAVBOARD_MAX_COMPASS_DATA_AGE && bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Compass data recovered!");
        // Reset toggle.
        bAlreadyPrintedWarning = false;
    }

    // Return angular velocity.
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
std::chrono::system_clock::duration NavigationBoard::GetGPSLastUpdateTime()
{
    // Get current time.
    std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
    // Acquire read lock for getting GPS timestamp.
    std::shared_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
    // Return the difference.
    return tmCurrentTime - m_tmLastGPSUpdateTime;
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
std::chrono::system_clock::duration NavigationBoard::GetCompassLastUpdateTime()
{
    // Get current time.
    std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
    // Acquire read lock for getting Heading timestamp.
    std::shared_lock<std::shared_mutex> lkCompassProcessLock(m_muHeadingMutex);
    // Return the difference.
    return tmCurrentTime - m_tmLastCompassUpdateTime;
}

/******************************************************************************
 * @brief
 *
 * @return true -
 * @return false -
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-05-24
 ******************************************************************************/
bool NavigationBoard::IsOutOfDate()
{
    return m_bNavBoardOutOfDate;
}
