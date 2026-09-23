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
    // Create static flag for printing out warnings. Atomic because these accessors are called
    // from the state machine, the SIM camera producer and the visualization thread at once,
    // and a plain function-local static written from several threads is a data race.
    static std::atomic<bool> bAlreadyPrintedWarning{false};

    // Acquire read lock for getting GPS struct.
    std::shared_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
    // Calculate time elapsed since last GPS data update.
    int nGPSDataAge = std::chrono::duration_cast<std::chrono::seconds>(this->GetGPSLastUpdateTimeLocked()).count();
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
    // Create static flag for printing out warnings. Atomic because these accessors are called
    // from the state machine, the SIM camera producer and the visualization thread at once,
    // and a plain function-local static written from several threads is a data race.
    static std::atomic<bool> bAlreadyPrintedWarning{false};

    // Acquire read lock for getting UTM struct.
    std::shared_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
    // Calculate time elapsed since last GPS data update.
    int nGPSDataAge = std::chrono::duration_cast<std::chrono::seconds>(this->GetGPSLastUpdateTimeLocked()).count();
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
    // Create static flag for printing out warnings. Atomic because these accessors are called
    // from the state machine, the SIM camera producer and the visualization thread at once,
    // and a plain function-local static written from several threads is a data race.
    static std::atomic<bool> bAlreadyPrintedWarning{false};

    // Acquire read lock for getting compass double.
    std::shared_lock<std::shared_mutex> lkCompassProcessLock(m_muHeadingMutex);
    // Calculate time elapsed since last GPS data update.
    int nCompassDataAge = std::chrono::duration_cast<std::chrono::seconds>(this->GetCompassLastUpdateTimeLocked()).count();
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
    // Create static flag for printing out warnings. Atomic because these accessors are called
    // from the state machine, the SIM camera producer and the visualization thread at once,
    // and a plain function-local static written from several threads is a data race.
    static std::atomic<bool> bAlreadyPrintedWarning{false};

    // Acquire read lock for getting compass double.
    std::shared_lock<std::shared_mutex> lkCompassProcessLock(m_muHeadingMutex);
    // Calculate time elapsed since last GPS data update.
    int nCompassDataAge = std::chrono::duration_cast<std::chrono::seconds>(this->GetCompassLastUpdateTimeLocked()).count();
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
    // Create static flag for printing out warnings. Atomic because these accessors are called
    // from the state machine, the SIM camera producer and the visualization thread at once,
    // and a plain function-local static written from several threads is a data race.
    static std::atomic<bool> bAlreadyPrintedWarning{false};

    // The data age lives under the LOCATION lock, not the velocity lock. Read it first, in
    // its own scope, so the two locks are never held at once and no ordering is implied.
    int nGPSDataAge = 0;
    {
        // Acquire read lock for the GPS timestamp.
        std::shared_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
        nGPSDataAge = std::chrono::duration_cast<std::chrono::seconds>(this->GetGPSLastUpdateTimeLocked()).count();
    }

    // Acquire read lock for getting velocity double.
    std::shared_lock<std::shared_mutex> lkVelocityProcessLock(m_muVelocityMutex);
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
    // Create static flag for printing out warnings. Atomic because these accessors are called
    // from the state machine, the SIM camera producer and the visualization thread at once,
    // and a plain function-local static written from several threads is a data race.
    static std::atomic<bool> bAlreadyPrintedWarning{false};

    // Acquire read lock for getting angular velocity double.
    // The data age lives under the HEADING lock, not the angular velocity lock. Read it
    // first, in its own scope, so the two locks are never held at once.
    int nCompassDataAge = 0;
    {
        // Acquire read lock for the compass timestamp.
        std::shared_lock<std::shared_mutex> lkCompassProcessLock(m_muHeadingMutex);
        nCompassDataAge = std::chrono::duration_cast<std::chrono::seconds>(this->GetCompassLastUpdateTimeLocked()).count();
    }

    // Acquire read lock for getting angular velocity double.
    std::shared_lock<std::shared_mutex> lkAngularVelocityProcessLock(m_muAngularVelocityMutex);
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
    // Acquire read lock for getting GPS timestamp, then do the arithmetic under it.
    std::shared_lock<std::shared_mutex> lkGPSProcessLock(m_muLocationMutex);
    return this->GetGPSLastUpdateTimeLocked();
}

/******************************************************************************
 * @brief How long ago the last GPS update arrived. Assumes m_muLocationMutex is
 *      ALREADY held by the caller.
 *
 *      This exists because std::shared_mutex is not recursive. The public accessors
 *      hold the location lock and then need the data age; calling the public
 *      GetGPSLastUpdateTime() from inside that critical section acquires the same
 *      shared_mutex a second time on the same thread, which is undefined behaviour -
 *      it survives on glibc only because its rwlock happens to prefer readers.
 *
 * @return std::chrono::system_clock::duration - Time elapsed since the last GPS update.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-07
 ******************************************************************************/
std::chrono::system_clock::duration NavigationBoard::GetGPSLastUpdateTimeLocked() const
{
    // Return the difference between now and the stored timestamp.
    return std::chrono::system_clock::now() - m_tmLastGPSUpdateTime;
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
    // Acquire read lock for getting Heading timestamp, then do the arithmetic under it.
    std::shared_lock<std::shared_mutex> lkCompassProcessLock(m_muHeadingMutex);
    return this->GetCompassLastUpdateTimeLocked();
}

/******************************************************************************
 * @brief How long ago the last compass update arrived. Assumes m_muHeadingMutex is
 *      ALREADY held by the caller. See GetGPSLastUpdateTimeLocked() for why.
 *
 * @return std::chrono::system_clock::duration - Time elapsed since the last compass update.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-07
 ******************************************************************************/
std::chrono::system_clock::duration NavigationBoard::GetCompassLastUpdateTimeLocked() const
{
    // Return the difference between now and the stored timestamp.
    return std::chrono::system_clock::now() - m_tmLastCompassUpdateTime;
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

    // Acquire write lock for the GPS struct and update it. m_tmLastGPSUpdateTime is guarded
    // by THIS mutex, so read the previous timestamp here rather than under the velocity lock
    // (which guards m_dVelocity and nothing else).
    std::chrono::system_clock::time_point tmPreviousGPSUpdate;
    {
        // Lock the location data for writing.
        std::unique_lock<std::shared_mutex> lkGPSWriteProcessLock(m_muLocationMutex);
        // Keep the previous update time so velocity can be computed from it below.
        tmPreviousGPSUpdate = m_tmLastGPSUpdateTime;
        // Repack data from RoveCommPacket into member variable.
        m_stLocation.dLatitude   = stPacket.vData[0];
        m_stLocation.dLongitude  = stPacket.vData[1];
        m_stLocation.dAltitude   = stPacket.vData[2];
        m_stLocation.tmTimestamp = tmCurrentTime;
        // Update GPS update time.
        m_tmLastGPSUpdateTime = tmCurrentTime;
    }

    // Guard against a zero interval, which would divide by zero on a duplicate packet.
    const double dElapsedSeconds = std::chrono::duration_cast<std::chrono::microseconds>(tmCurrentTime - tmPreviousGPSUpdate).count() / 1e6;
    if (dElapsedSeconds > 0.0)
    {
        // Acquire write lock for writing to velocity member variable.
        std::unique_lock<std::shared_mutex> lkVelocityProcessLock(m_muVelocityMutex);
        // Calculate rover velocity based on GPS distance traveled over time.
        m_dVelocity = geMeasurement.dDistanceMeters / dElapsedSeconds;
    }

    // Submit logger message. Log the packet values directly: reading the members back would
    // be an unsynchronized read, and they may already have been overwritten by the next packet.
    LOG_DEBUG(logging::g_qSharedLogger, "Incoming GPS Data: ({} lat, {} lon, {} alt)", stPacket.vData[0], stPacket.vData[1], stPacket.vData[2]);
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

    // Acquire write lock for heading and compass timestamp. m_tmLastCompassUpdateTime is
    // guarded by THIS mutex, so read the previous timestamp here rather than under the
    // angular velocity lock (which guards m_dAngularVelocity and nothing else).
    std::chrono::system_clock::time_point tmPreviousCompassUpdate;
    {
        // Lock the heading data for writing.
        std::unique_lock<std::shared_mutex> lkCompassProcessLock(m_muHeadingMutex);
        // Keep the previous update time so angular velocity can be computed from it below.
        tmPreviousCompassUpdate = m_tmLastCompassUpdateTime;
        // Repack data from RoveCommPacket into member variable.
        m_dHeading = dNewHeading;
        // Update compass time.
        m_tmLastCompassUpdateTime = tmCurrentTime;
    }

    // Guard against a zero interval, which would divide by zero on a duplicate packet.
    const double dElapsedSeconds = std::chrono::duration_cast<std::chrono::microseconds>(tmCurrentTime - tmPreviousCompassUpdate).count() / 1e6;
    if (dElapsedSeconds > 0.0)
    {
        // Acquire write lock for writing to angular velocity member variable.
        std::unique_lock<std::shared_mutex> lkAngularVelocityProcessLock(m_muAngularVelocityMutex);
        // Calculate rover angular velocity based on change in heading over time.
        m_dAngularVelocity = dDeltaAngle / dElapsedSeconds;
    }

    // Submit logger message. Log the packet value directly rather than reading the member
    // back without the lock.
    LOG_DEBUG(logging::g_qSharedLogger, "Incoming Compass Data: {}", dNewHeading);
}
