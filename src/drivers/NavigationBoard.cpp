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

    if (network::g_pRoveCommUDPNode)
    {
        // Determine the IP address to send the subscribe packet to.
        const manifest::AddressEntry& stIPAddress = constants::MODE_SIM ? constants::SIM_IP_ADDRESS : manifest::Nav::IP_ADDRESS;

        // Send subscribe packet to NavBoard.
        network::g_pRoveCommUDPNode->Subscribe(stIPAddress, constants::ROVECOMM_OUTGOING_UDP_PORT);

        // Set RoveComm callbacks.
        using namespace manifest::Nav::Telemetry;
        network::g_pRoveCommUDPNode->On<GPSLATLONALT>([this](const auto& stPacket) { ProcessGPSData(stPacket); });
        // network::g_pRoveCommUDPNode->On<ACCURACYDATA>([this](const auto& stPacket) { ProcessAccuracyData(stPacket); });
        network::g_pRoveCommUDPNode->On<COMPASSDATA>([this](const auto& stPacket) { ProcessCompassData(stPacket); });
    }
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
    // Make a copy of the GPS data to return.
    geoops::GPSCoordinate stGPSCopy = m_stLocation;
    // Release lock before modifying data.
    lkGPSProcessLock.unlock();

    // Adjust the GPS data by the configured offsets.
    geoops::UTMCoordinate stUTMData = geoops::ConvertGPSToUTM(stGPSCopy);
    stUTMData.dEasting += constants::NAVBOARD_EASTING_OFFSET;
    stUTMData.dNorthing += constants::NAVBOARD_NORTHING_OFFSET;
    stGPSCopy = geoops::ConvertUTMToGPS(stUTMData);

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
        LOG_NOTICE(logging::g_qSharedLogger, "GPS data recovered!");
        // Reset toggle.
        bAlreadyPrintedWarning = false;
        // Reset Out of Date.
        m_bNavBoardOutOfDate = false;
    }

    // Return current GPS location.
    return stGPSCopy;
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
    // Make a copy of the GPS data to return.
    geoops::GPSCoordinate stGPSCopy = m_stLocation;
    // Release lock before modifying data.
    lkGPSProcessLock.unlock();

    // Adjust the GPS data by the configured offsets.
    geoops::UTMCoordinate stUTMData = geoops::ConvertGPSToUTM(stGPSCopy);
    stUTMData.dEasting += constants::NAVBOARD_EASTING_OFFSET;
    stUTMData.dNorthing += constants::NAVBOARD_NORTHING_OFFSET;

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
        LOG_NOTICE(logging::g_qSharedLogger, "GPS data recovered!");
        // Reset toggle.
        bAlreadyPrintedWarning = false;
        // Reset Out of Date.
        m_bNavBoardOutOfDate = false;
    }

    // Convert the currently stored GPS coord to UTM and return.
    return stUTMData;
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
        // Set Out of Date.
        m_bNavBoardOutOfDate = true;
    }
    else if (nCompassDataAge < constants::NAVBOARD_MAX_COMPASS_DATA_AGE && bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_NOTICE(logging::g_qSharedLogger, "Compass data recovered!");
        // Reset toggle.
        bAlreadyPrintedWarning = false;
        // Reset Out of Date.
        m_bNavBoardOutOfDate = false;
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
        // Set Out of Date.
        m_bNavBoardOutOfDate = true;
    }
    else if (nCompassDataAge < constants::NAVBOARD_MAX_COMPASS_DATA_AGE && bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_NOTICE(logging::g_qSharedLogger, "Compass data recovered!");
        // Reset toggle.
        bAlreadyPrintedWarning = false;
        // Reset Out of Date.
        m_bNavBoardOutOfDate = false;
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
        // Set Out of Date.
        m_bNavBoardOutOfDate = true;
    }
    else if (nGPSDataAge < constants::NAVBOARD_MAX_GPS_DATA_AGE && bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_NOTICE(logging::g_qSharedLogger, "GPS data recovered!");
        // Reset toggle.
        bAlreadyPrintedWarning = false;
        // Reset Out of Date.
        m_bNavBoardOutOfDate = false;
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
        // Set Out of Date.
        m_bNavBoardOutOfDate = true;
    }
    else if (nCompassDataAge < constants::NAVBOARD_MAX_COMPASS_DATA_AGE && bAlreadyPrintedWarning)
    {
        // Submit logger message.
        LOG_NOTICE(logging::g_qSharedLogger, "Compass data recovered!");
        // Reset toggle.
        bAlreadyPrintedWarning = false;
        // Reset Out of Date.
        m_bNavBoardOutOfDate = false;
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
 * @brief Checks if any of the navboard data is out of date. Expired data depends
 *      on constants::NAVBOARD_MAX_COMPASS_DATA_AGE.
 *
 * @return true - The data is out of date.
 * @return false - The data is new.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-05-24
 ******************************************************************************/
bool NavigationBoard::IsOutOfDate()
{
    return m_bNavBoardOutOfDate;
}

/******************************************************************************
 * @brief Callback function that is called whenever RoveComm receives new GPS data.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-03
 ******************************************************************************/
void NavigationBoard::ProcessGPSData(const rovecomm::RoveCommPacket<double>& stPacket)
{
    // Get current time.
    std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
    // Acquire read lock for getting GPS struct.
    std::shared_lock<std::shared_mutex> lkGPSReadProcessLock(m_muLocationMutex);
    // Calculate distance of new GPS coordinate to old GPS coordinate.
    geoops::GeoMeasurement geMeasurement = geoops::CalculateGeoMeasurement(m_stLocation, geoops::GPSCoordinate(stPacket.vData[0], stPacket.vData[1], stPacket.vData[2]));
    // Unlock mutex.
    lkGPSReadProcessLock.unlock();

    // Acquire write lock for writing to velocity member variable.
    std::unique_lock<std::shared_mutex> lkVelocityProcessLock(m_muVelocityMutex);
    // Calculate rover velocity based on GPS distance traveled over time.
    m_dVelocity =
        geMeasurement.dDistanceMeters / static_cast<double>((std::chrono::duration_cast<std::chrono::microseconds>(tmCurrentTime - m_tmLastGPSUpdateTime).count() / 1e6));
    // Unlock mutex.
    lkVelocityProcessLock.unlock();

    // Acquire write lock for writing to GPS struct.
    std::unique_lock<std::shared_mutex> lkGPSWriteProcessLock(m_muLocationMutex);
    // Repack data from RoveCommPacket into member variable.
    m_stLocation.dLatitude   = stPacket.vData[0];
    m_stLocation.dLongitude  = stPacket.vData[1];
    m_stLocation.dAltitude   = stPacket.vData[2];
    m_stLocation.tmTimestamp = tmCurrentTime;
    // Update GPS update time.
    m_tmLastGPSUpdateTime = tmCurrentTime;
    // Unlock mutex.
    lkGPSWriteProcessLock.unlock();

    // Submit logger message.
    LOG_DEBUG(logging::g_qSharedLogger, "Incoming GPS Data: ({} lat, {} lon, {} alt)", m_stLocation.dLatitude, m_stLocation.dLongitude, m_stLocation.dAltitude);
}

/******************************************************************************
 * @brief Callback function that is called whenever RoveComm receives new Accuracy data.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-03
 ******************************************************************************/
void NavigationBoard::ProcessAccuracyData(const rovecomm::RoveCommPacket<float>& stPacket)
{
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
}

/******************************************************************************
 * @brief Callback function that is called whenever RoveComm receives new Compass data.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com), Jason Pittman (jspencerpittman@gmail.com)
 * @date 2024-03-03
 ******************************************************************************/
void NavigationBoard::ProcessCompassData(const rovecomm::RoveCommPacket<float>& stPacket)
{
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
}
