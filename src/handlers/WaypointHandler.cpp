/******************************************************************************
 * @brief Implements the WaypointHandler class.
 *
 * @file WaypointHandler.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-02
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "WaypointHandler.h"
#include "../AutonomyGlobals.h"
#include "../AutonomyNetworking.h"
#include "../util/NumberOperations.hpp"

/// \cond
#include <algorithm>
#include <cmath>

/// \endcond

/******************************************************************************
 * @brief Construct a new geoops::Waypoint Handler:: geoops::Waypoint Handler obstacle.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-02
 ******************************************************************************/
WaypointHandler::WaypointHandler()
{
    // Set RoveComm callbacks.
    using namespace manifest::Autonomy::Commands;
    network::g_pRoveCommUDPNode->On<ADDPOSITIONLEG>([this](const auto& stPacket) { AddPositionLegCallback(stPacket); });
    network::g_pRoveCommUDPNode->On<ADDMARKERLEG>([this](const auto& stPacket) { AddMarkerLegCallback(stPacket); });
    network::g_pRoveCommUDPNode->On<ADDOBJECTLEG>([this](const auto& stPacket) { AddObjectLegCallback(stPacket); });
    network::g_pRoveCommUDPNode->On<ADDOBSTACLE>([this](const auto& stPacket) { AddObstacleCallback(stPacket); });
    network::g_pRoveCommUDPNode->On<CLEARWAYPOINTS>([this](const auto& stPacket) { ClearWaypointsCallback(stPacket); });
    network::g_pRoveCommUDPNode->On<CLEAROBSTACLES>([this](const auto& stPacket) { ClearObstaclesCallback(stPacket); });
}

/******************************************************************************
 * @brief Destroy the geoops::Waypoint Handler:: geoops::Waypoint Handler obstacle.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-02
 ******************************************************************************/
WaypointHandler::~WaypointHandler()
{
    // Nothing to destroy.
}

/******************************************************************************
 * @brief Append a waypoint to the end of the WaypointHandler's list.
 *
 * @param stWaypoint - The WaypointHandler::geoops::Waypoint struct containing information about the waypoint to
 *                  store in the handler.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-02
 ******************************************************************************/
void WaypointHandler::AddWaypoint(const geoops::Waypoint& stWaypoint)
{
    // Acquire a write lock on the waypoint vector.
    std::unique_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Add waypoint to end of member variable vector.
    m_vWaypointList.emplace_back(stWaypoint);
}

/******************************************************************************
 * @brief Append a waypoint to the end of the WaypointHandler's list.
 *
 * @param stLocation - The location of the waypoint stored in a geoops namespace GPSCoordinate struct.
 * @param eType - The leg type of the waypoint signalling if this is a tag, navigation, obstacle, etc. waypoint.
 * @param dRadius - The circular area around the waypoint that should be counted as reaching the waypoint. Or obstacle radius.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::AddWaypoint(const geoops::GPSCoordinate& stLocation, const geoops::WaypointType& eType, const double dRadius)
{
    // Construct a new waypoint struct from the given info.
    geoops::Waypoint stTempWaypoint(stLocation, eType, dRadius);

    // Acquire a write lock on the waypoint vector.
    std::unique_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Add waypoint to end of member variable vector.
    m_vWaypointList.emplace_back(stTempWaypoint);
}

/******************************************************************************
 * @brief Append a waypoint to the end of the WaypointHandler's list.
 *
 * @param stLocation - The location of the waypoint stored in a geoops namespace UTMCoordinate struct.
 * @param eType - The leg type of the waypoint signalling if this is a tag, navigation, obstacle, etc. waypoint.
 * @param dRadius - The circular area around the waypoint that should be counted as reaching the waypoint. Or obstacle radius.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::AddWaypoint(const geoops::UTMCoordinate& stLocation, const geoops::WaypointType& eType, const double dRadius)
{
    // Construct a new waypoint struct from the given info.
    geoops::Waypoint stTempWaypoint(stLocation, eType, dRadius);

    // Acquire a write lock on the waypoint vector.
    std::unique_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Add waypoint to end of member variable vector.
    m_vWaypointList.emplace_back(stTempWaypoint);
}

/******************************************************************************
 * @brief Store a path in the WaypointHandler.
 *
 * @param szPathName - The key that will be used to store, and later reference, the path in the WaypointHandler.
 * @param vWaypointPath - A vector containing geoops::Waypoint structs with data about each point on the path.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::StorePath(const std::string& szPathName, const std::vector<geoops::Waypoint>& vWaypointPath)
{
    // Acquire a write lock on the path unordered map.
    std::unique_lock<std::shared_mutex> lkPathsLock(m_muPathMutex);
    // Add vector of waypoint to map with the given string as a key.
    m_umStoredPaths[szPathName] = vWaypointPath;
}

/******************************************************************************
 * @brief Store a path in the WaypointHandler.
 *
 * @param szPathName - The key that will be used to store, and later reference, the path in the WaypointHandler.
 * @param vWaypointPath - A vector containing GPSCoordinate structs containing location data about each point in the path.
 *
 * @note Paths must be stored in the WaypointHandler as a vector is geoops::Waypoint structs. This will create a new geoops::Waypoint
 *      struct for each GPSCoordinate and use a default type of eNavigationWaypoint with a radius of 0.0.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::StorePath(const std::string& szPathName, const std::vector<geoops::GPSCoordinate>& vLocationPath)
{
    // Create instance variables.
    std::vector<geoops::Waypoint> vWaypointPath;

    // Loop through each GPSCoordinate in the given vector and repack the info into a geoops::Waypoint.
    for (geoops::GPSCoordinate stLocation : vLocationPath)
    {
        // Create a new waypoint and store location info in it.
        geoops::Waypoint stWaypoint(stLocation, geoops::WaypointType::eNavigationWaypoint);

        // Append waypoint to the temporary waypoint path.
        vWaypointPath.emplace_back(stWaypoint);
    }

    // Acquire a write lock on the path unordered map.
    std::unique_lock<std::shared_mutex> lkPathsLock(m_muPathMutex);
    // Add vector of waypoint to map with the given string as a key.
    m_umStoredPaths[szPathName] = vWaypointPath;
}

/******************************************************************************
 * @brief Store a path in the WaypointHandler.
 *
 * @param szPathName - The key that will be used to store, and later reference, the path in the WaypointHandler.
 * @param vWaypointPath - A vector containing UTMCoordinate structs containing location data about each point in the path.
 *
 * @note Paths must be stored in the WaypointHandler as a vector is geoops::Waypoint structs. This will create a new geoops::Waypoint
 *      struct for each UTMCoordinate and use a default type of eNavigationWaypoint with a radius of 0.0.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::StorePath(const std::string& szPathName, const std::vector<geoops::UTMCoordinate>& vLocationPath)
{
    // Create instance variables.
    std::vector<geoops::Waypoint> vWaypointPath;

    // Loop through each UTMCoordinate in the given vector and repack the info into a geoops::Waypoint.
    for (geoops::UTMCoordinate stLocation : vLocationPath)
    {
        // Create a new waypoint and store location info in it.
        geoops::Waypoint stWaypoint(stLocation, geoops::WaypointType::eNavigationWaypoint);

        // Append waypoint to the temporary waypoint path.
        vWaypointPath.emplace_back(stWaypoint);
    }

    // Acquire a write lock on the path unordered map.
    std::unique_lock<std::shared_mutex> lkPathsLock(m_muPathMutex);
    // Add vector of waypoint to map with the given string as a key.
    m_umStoredPaths[szPathName] = vWaypointPath;
}

/******************************************************************************
 * @brief Append a new obstacle to the WaypointHandler obstacle list.
 *
 * @param stWaypoint - The WaypointHandler::geoops::Waypoint struct containing information about the obstacle to
 *                  store in the handler.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::AddObstacle(const geoops::Waypoint& stWaypoint)
{
    // Acquire a write lock on the obstacle vector.
    std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObstaclesMutex);
    // Add obstacle waypoint to end of member variable vector.
    m_vPermanentObstacles.emplace_back(stWaypoint);
}

/******************************************************************************
 * @brief Append a new obstacle to the WaypointHandler obstacle list.
 *
 * @param stLocation - The location of the waypoint stored in a geoops namespace GPSCoordinate struct.
 * @param dRadius - The circular area around the obstacle or the obstacle radius.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::AddObstacle(const geoops::GPSCoordinate& stLocation, const double dRadius)
{
    // Construct a new waypoint struct from the given info.
    geoops::Waypoint stTempWaypoint(stLocation, geoops::WaypointType::eObstacleWaypoint, dRadius);

    // Acquire a write lock on the waypoint vector.
    std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObstaclesMutex);
    // Add waypoint to end of member variable vector.
    m_vPermanentObstacles.emplace_back(stTempWaypoint);
}

/******************************************************************************
 * @brief Append a new obstacle to the WaypointHandler obstacle list.
 *
 * @param stLocation - The location of the waypoint stored in a geoops namespace UTMCoordinate struct.
 * @param dRadius - The circular area around the obstacle or the obstacle radius.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::AddObstacle(const geoops::UTMCoordinate& stLocation, const double dRadius)
{
    // Construct a new waypoint struct from the given info.
    geoops::Waypoint stTempWaypoint(stLocation, geoops::WaypointType::eObstacleWaypoint, dRadius);

    // Acquire a write lock on the waypoint vector.
    std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObstaclesMutex);
    // Add waypoint to end of member variable vector.
    m_vPermanentObstacles.emplace_back(stTempWaypoint);
}

/******************************************************************************
 * @brief Delete the geoops::Waypoint at a given index from the waypoint handler.
 *
 * @param nIndex - The index of the element to remove.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::DeleteWaypoint(const long unsigned int nIndex)
{
    // Acquire a read lock on the waypoint vector.
    std::shared_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Check if the vector has at least one waypoint.
    if (nIndex < m_vWaypointList.size())
    {
        // Release read lock.
        lkWaypointListLock.unlock();

        // Acquire a write lock on the waypoint vector.
        std::unique_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
        // Delete the geoops::Waypoint at the index.
        m_vWaypointList.erase(m_vWaypointList.begin() + nIndex);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Attempted to delete a waypoint at index {} from the WaypointHandler but it is already empty or the index is out of bounds!",
                  nIndex);
    }
}

/******************************************************************************
 * @brief Delete a waypoint from the WaypointHandler given a matching waypoint.
 *      Any waypoint in the list that matches the given one will be removed.
 *
 * @param stWaypoint - The equivalent waypoint that should be removed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
void WaypointHandler::DeleteWaypoint(const geoops::Waypoint& stWaypoint)
{
    // Acquire a write lock on the waypoint vector.
    std::unique_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Delete any waypoint matching the given one from the list.
    m_vWaypointList.erase(std::remove(m_vWaypointList.begin(), m_vWaypointList.end(), stWaypoint), m_vWaypointList.end());
}

/******************************************************************************
 * @brief Delete a waypoint from the WaypointHandler given a matching location.
 *      Any waypoint in the list that matches the given location will be removed.
 *
 * @param stLocation - The equivalent location that should be removed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
void WaypointHandler::DeleteWaypoint(const geoops::GPSCoordinate& stLocation)
{
    // Acquire a write lock on the waypoint vector.
    std::unique_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Delete any waypoint matching the given location from the list.
    m_vWaypointList.erase(std::remove_if(m_vWaypointList.begin(),
                                         m_vWaypointList.end(),
                                         [stLocation](const geoops::Waypoint& stWaypoint) { return stWaypoint.GetGPSCoordinate() == stLocation; }),
                          m_vWaypointList.end());
}

/******************************************************************************
 * @brief Delete a waypoint from the WaypointHandler given a matching location.
 *      Any waypoint in the list that matches the given location will be removed.
 *
 * @param stLocation - The equivalent location that should be removed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
void WaypointHandler::DeleteWaypoint(const geoops::UTMCoordinate& stLocation)
{
    // Acquire a write lock on the waypoint vector.
    std::unique_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Delete any waypoint matching the given location from the list.
    m_vWaypointList.erase(std::remove_if(m_vWaypointList.begin(),
                                         m_vWaypointList.end(),
                                         [stLocation](const geoops::Waypoint& stWaypoint) { return stWaypoint.GetUTMCoordinate() == stLocation; }),
                          m_vWaypointList.end());
}

/******************************************************************************
 * @brief Delete the path vector stored at the given key.
 *
 * @param szPathName - The name/key of the path that was previously used to store the path.
 * @return true - Key was found and deleted successfully.
 * @return false - Key was not found and therefore nothing was deleted.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
bool WaypointHandler::DeletePath(const std::string& szPathName)
{
    // Acquire a write lock on the path unordered map.
    std::unique_lock<std::shared_mutex> lkPathsLock(m_muPathMutex);
    // Remove the given key and value from the map.
    return m_umStoredPaths.erase(szPathName);
}

/******************************************************************************
 * @brief Delete the obstacle at a given index from the waypoint handler obstacle list.
 *
 * @param nIndex - The index of the element to remove.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::DeleteObstacle(const long unsigned int nIndex)
{
    // Acquire a read lock on the waypoint vector.
    std::shared_lock<std::shared_mutex> lkObjectListLock(m_muObstaclesMutex);
    // Check if the vector has at least one waypoint.
    if (nIndex < m_vPermanentObstacles.size())
    {
        // Release read lock.
        lkObjectListLock.unlock();

        // Acquire a write lock on the obstacle vector.
        std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObstaclesMutex);
        // Delete the geoops::Waypoint at the index.
        m_vPermanentObstacles.erase(m_vPermanentObstacles.begin() + nIndex);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Attempted to delete an obstacle waypoint at index {} from the WaypointHandler but it is already empty or the index is out of bounds!",
                  nIndex);
    }
}

/******************************************************************************
 * @brief Delete an obstacle from the WaypointHandler given a matching location.
 *      Any waypoint in the list that matches the given location will be removed.
 *
 * @param stWaypoint - The equivalent obstacle location that should be removed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
void WaypointHandler::DeleteObstacle(const geoops::Waypoint& stWaypoint)
{
    // Acquire a write lock on the obstacle vector.
    std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObstaclesMutex);
    // Delete any waypoint matching the given one from the list.
    m_vPermanentObstacles.erase(std::remove(m_vPermanentObstacles.begin(), m_vPermanentObstacles.end(), stWaypoint), m_vPermanentObstacles.end());
}

/******************************************************************************
 * @brief Delete an obstacle from the WaypointHandler given a matching location.
 *      Any waypoint in the list that matches the given location will be removed.
 *
 * @param stLocation - The equivalent obstacle location that should be removed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
void WaypointHandler::DeleteObstacle(const geoops::GPSCoordinate& stLocation)
{
    // Acquire a write lock on the obstacle vector.
    std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObstaclesMutex);
    // Delete any waypoint matching the given location from the list.
    m_vPermanentObstacles.erase(std::remove_if(m_vPermanentObstacles.begin(),
                                               m_vPermanentObstacles.end(),
                                               [stLocation](const geoops::Waypoint& stWaypoint) { return stWaypoint.GetGPSCoordinate() == stLocation; }),
                                m_vPermanentObstacles.end());
}

/******************************************************************************
 * @brief Delete an obstacle from the WaypointHandler given a matching location.
 *      Any waypoint in the list that matches the given location will be removed.
 *
 * @param stLocation - The equivalent obstacle location that should be removed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
void WaypointHandler::DeleteObstacle(const geoops::UTMCoordinate& stLocation)
{
    // Acquire a write lock on the obstacle vector.
    std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObstaclesMutex);
    // Delete any waypoint matching the given location from the list.
    m_vPermanentObstacles.erase(std::remove_if(m_vPermanentObstacles.begin(),
                                               m_vPermanentObstacles.end(),
                                               [stLocation](const geoops::Waypoint& stWaypoint) { return stWaypoint.GetUTMCoordinate() == stLocation; }),
                                m_vPermanentObstacles.end());
}

/******************************************************************************
 * @brief Clears/deletes all Waypoints stored in the WaypointHandler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::ClearWaypoints()
{
    // Acquire a write lock on the waypoint vector.
    std::unique_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Clear the waypoint vector.
    m_vWaypointList.clear();
}

/******************************************************************************
 * @brief Clears/deletes all keys and paths store in the WaypointHandler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::ClearPaths()
{
    // Acquire a write lock on the path unordered map.
    std::unique_lock<std::shared_mutex> lkPathsLock(m_muPathMutex);
    // Clear the path map.
    m_umStoredPaths.clear();
}

/******************************************************************************
 * @brief Clears/deletes all permanent objects stored in the WaypointHandler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::ClearObstacles()
{
    // Acquire a write lock on the path unordered map.
    std::unique_lock<std::shared_mutex> lkObstaclesLock(m_muObstaclesMutex);
    // Clear the obstacle vector.
    m_vPermanentObstacles.clear();
}

/******************************************************************************
 * @brief Removes and returns the next waypoint at the front of the list.
 *
 * @return WaypointHandler::geoops::Waypoint - The next waypoint data stored in a geoops::Waypoint struct.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
geoops::Waypoint WaypointHandler::PopNextWaypoint()
{
    // Acquire a read lock on the waypoint vector.
    std::shared_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Check if the vector has at least one waypoint.
    if (!m_vWaypointList.empty())
    {
        // Release read lock.
        lkWaypointListLock.unlock();

        // Acquire a write lock on the waypoint vector.
        std::unique_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
        // Pop a waypoint from the front of the waypoint list and store it.
        geoops::Waypoint stWaypoint = m_vWaypointList[0];
        m_vWaypointList.erase(m_vWaypointList.begin());
        // Unlock shared mutex.
        lkWaypointListLock.unlock();

        // Return a copy of the waypoint.
        return stWaypoint;
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "Attempted to pop a waypoint from the WaypointHandler but it is empty!");

        // Return an empty waypoint.
        return geoops::Waypoint(geoops::GPSCoordinate(), geoops::WaypointType::eUNKNOWN);
    }
}

/******************************************************************************
 * @brief Returns an immutable reference to the geoops::Waypoint struct at the front of
 *      the list without removing it.
 *
 * @return const WaypointHandler::geoops::Waypoint - A reference to a geoops::Waypoint struct containing waypoint data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
const geoops::Waypoint WaypointHandler::PeekNextWaypoint()
{
    // Acquire a read lock on the waypoint vector.
    std::shared_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Check if the vector has at least one waypoint.
    if (!m_vWaypointList.empty())
    {
        // Return an immutable reference to the waypoint.
        return m_vWaypointList.front();
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "Attempted to peek a waypoint from the WaypointHandler but it is empty!");

        // Return an empty waypoint.
        return geoops::Waypoint(geoops::GPSCoordinate(), geoops::WaypointType::eUNKNOWN);
    }
}

/******************************************************************************
 * @brief Retrieve an immutable reference to the waypoint at the given index.
 *
 * @param nIndex - The index of the element to retrieve.
 * @return const WaypointHandler::geoops::Waypoint - An immutable reference to the geoops::Waypoint containing data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
const geoops::Waypoint WaypointHandler::RetrieveWaypointAtIndex(const long unsigned int nIndex)
{
    // Acquire a read lock on the waypoint vector.
    std::shared_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Check if the vector has at least one waypoint.
    if (nIndex < m_vWaypointList.size())
    {
        // Return an immutable reference to the waypoint at the index.
        return m_vWaypointList[nIndex];
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Attempted to retrieve a waypoint at index {} from the WaypointHandler but it is empty or the index is out of bounds!",
                  nIndex);

        // Return an empty waypoint.
        return geoops::Waypoint(geoops::GPSCoordinate(), geoops::WaypointType::eUNKNOWN);
    }
}

/******************************************************************************
 * @brief Retrieve an immutable reference to the path at the given path name/key.
 *
 * @param szPathName - The name/key of the path that was previously used to store the path.
 * @return const std::vector<WaypointHandler::geoops::Waypoint> - A reference to the geoops::Waypoint vector located at the given key.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
const std::vector<geoops::Waypoint> WaypointHandler::RetrievePath(const std::string& szPathName)
{
    // Acquire a read lock on the path unordered map.
    std::shared_lock<std::shared_mutex> lkPathsLock(m_muPathMutex);
    // Check if the map contains the given key.
    if (m_umStoredPaths.count(szPathName) > 0)
    {
        // Return the path vector at the given key.
        return m_umStoredPaths[szPathName];
    }
    else
    {
        // Return an empty vector.
        return std::vector<geoops::Waypoint>();
    }
}

/******************************************************************************
 * @brief Retrieve an immutable reference to the obstacle at the given index.
 *
 * @param nIndex - The index of the element to retrieve.
 * @return const WaypointHandler::geoops::Waypoint - An immutable reference to the obstacle geoops::Waypoint containing data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
const geoops::Waypoint WaypointHandler::RetrieveObstacleAtIndex(const long unsigned int nIndex)
{
    // Acquire a read lock on the waypoint vector.
    std::shared_lock<std::shared_mutex> lkObstaclesLock(m_muObstaclesMutex);
    // Check if the vector has at least one waypoint.
    if (nIndex < m_vPermanentObstacles.size())
    {
        // Return an immutable reference to the waypoint at the index.
        return m_vPermanentObstacles[nIndex];
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Attempted to retrieve a obstacle at index {} from the WaypointHandler but it is empty or the index is out of bounds!",
                  nIndex);

        // Return an empty waypoint.
        return geoops::Waypoint(geoops::GPSCoordinate(), geoops::WaypointType::eUNKNOWN);
    }
}

/******************************************************************************
 * @brief Accessor for the full list of current waypoints stored in the WaypointHandler.
 *
 * @return const std::vector<WaypointHandler::geoops::Waypoint> - A vector of geoops::Waypoint structs currently stored in the WaypointHandler.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
const std::vector<geoops::Waypoint> WaypointHandler::GetAllWaypoints()
{
    // Acquire a read lock on the waypoint vector.
    std::shared_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Return a copy of the current waypoint list.
    return m_vWaypointList;
}

/******************************************************************************
 * @brief Accessor for the full list of current obstacle stored in the WaypointHandler.
 *
 * @return const std::vector<WaypointHandler::geoops::Waypoint> - A vector of geoops::Waypoint structs representing
 *                                      objects that are currently stored in the WaypointHandler.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
const std::vector<geoops::Waypoint> WaypointHandler::GetAllObstacles()
{
    // Acquire a read lock on the path unordered map.
    std::shared_lock<std::shared_mutex> lkObstaclesLock(m_muObstaclesMutex);
    // Return a copy of the current obstacle list.
    return m_vPermanentObstacles;
}

/******************************************************************************
 * @brief Accessor for the number of elements on the WaypointHandler's waypoint vector.
 *
 * @return int - The size of the waypoint vector.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
int WaypointHandler::GetWaypointCount()
{
    // Acquire a read lock on the waypoint vector.
    std::shared_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Return total number of waypoints stored.
    return m_vWaypointList.size();
}

/******************************************************************************
 * @brief Accessor for the number of paths stored in the WaypointHandler.
 *
 * @return int - The size of the unordered_map storing the paths.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
int WaypointHandler::GetPathsCount()
{
    // Acquire a read lock on the path unordered map.
    std::shared_lock<std::shared_mutex> lkPathsLock(m_muPathMutex);
    // Return total number of paths.
    return m_umStoredPaths.size();
}

/******************************************************************************
 * @brief Accessor for the number of elements on the WaypointHandler's obstacle vector.
 *
 * @return int - The size of the obstacle vector.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
int WaypointHandler::GetObstaclesCount()
{
    // Acquire a write lock on the waypoint vector.
    std::shared_lock<std::shared_mutex> lkObstaclesLock(m_muObstaclesMutex);
    // Return total number of objects stored.
    return m_vPermanentObstacles.size();
}

/******************************************************************************
 * @brief Callback function that is called whenever RoveComm receives new ADDPOSITIONLEG packet.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-03
 ******************************************************************************/
void WaypointHandler::AddPositionLegCallback(const rovecomm::RoveCommPacket<double>& stPacket)
{
    // Create new waypoint struct with data from the RoveComm packet.
    geoops::Waypoint stNavWaypoint(geoops::GPSCoordinate(stPacket.vData[0], stPacket.vData[1]), geoops::WaypointType::eNavigationWaypoint, 0.0, stPacket.vData[2]);

    // Acquire write lock for writing to waypoints vector.
    std::unique_lock<std::shared_mutex> lkWaypointsLock(m_muWaypointsMutex);
    // Queue waypoint.
    m_vWaypointList.emplace_back(stNavWaypoint);
    // Unlock mutex.
    lkWaypointsLock.unlock();

    // Submit logger message.
    LOG_NOTICE(logging::g_qSharedLogger,
               "Incoming Navigation Waypoint Data: Added (lat: {}, lon: {}, id: {}) to WaypointHandler queue.",
               stPacket.vData[0],
               stPacket.vData[1],
               stPacket.vData[2]);
}

/******************************************************************************
 * @brief Callback function that is called whenever RoveComm receives new ADDMARKERLEG packet.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-03
 ******************************************************************************/
void WaypointHandler::AddMarkerLegCallback(const rovecomm::RoveCommPacket<double>& stPacket)
{
    // Create instance variables.
    int nMarkerID  = stPacket.vData[2];
    double dRadius = stPacket.vData[3];

    // Limit the radius to 0-40.
    if (dRadius < 0)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Incoming Marker Waypoint Data: Radius is less than 0, setting to 0.");
        dRadius = 0;
    }
    else if (dRadius > 40)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Incoming Marker Waypoint Data: Radius is greater than 40, setting to 40.");
        dRadius = 40;
    }

    // Create new waypoint struct with data from the RoveComm packet.
    geoops::Waypoint stMarkerWaypoint(geoops::GPSCoordinate(stPacket.vData[0], stPacket.vData[1]), geoops::WaypointType::eTagWaypoint, dRadius, nMarkerID);

    // Acquire write lock for writing to waypoints vector.
    std::unique_lock<std::shared_mutex> lkWaypointsLock(m_muWaypointsMutex);
    // Queue waypoint.
    m_vWaypointList.emplace_back(stMarkerWaypoint);
    // Unlock mutex.
    lkWaypointsLock.unlock();

    // Submit logger message.
    LOG_NOTICE(logging::g_qSharedLogger,
               "Incoming Marker Waypoint Data: Added (lat: {}, lon: {}, marker ID: {}, radius: {}) to WaypointHandler queue.",
               stPacket.vData[0],
               stPacket.vData[1],
               nMarkerID,
               dRadius);
}

/******************************************************************************
 * @brief Callback function that is called whenever RoveComm receives new ADDOBJECTLEG packet.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-03
 ******************************************************************************/
void WaypointHandler::AddObjectLegCallback(const rovecomm::RoveCommPacket<double>& stPacket)
{
    // Create instance variables.
    geoops::WaypointType eWaypointType = geoops::WaypointType::eObjectWaypoint;
    double dObjectID                   = stPacket.vData[2];
    double dRadius                     = stPacket.vData[3];

    // Parse the object ID from the RoveComm packet to a waypoint type.
    if (dObjectID == static_cast<int>(manifest::Autonomy::AUTONOMYWAYPOINTTYPES::MALLET))
    {
        eWaypointType = geoops::WaypointType::eMalletWaypoint;
    }
    else if (dObjectID == static_cast<int>(manifest::Autonomy::AUTONOMYWAYPOINTTYPES::WATERBOTTLE))
    {
        eWaypointType = geoops::WaypointType::eWaterBottleWaypoint;
    }
    else if (dObjectID == static_cast<int>(manifest::Autonomy::AUTONOMYWAYPOINTTYPES::ROCKPICK))
    {
        eWaypointType = geoops::WaypointType::eRockPickWaypoint;
    }
    else
    {
        eWaypointType = geoops::WaypointType::eObjectWaypoint;
    }

    // Limit the radius to 0-40.
    if (dRadius < 0)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Incoming Object Waypoint Data: Radius is less than 0, setting to 0.");
        dRadius = 0;
    }
    else if (dRadius > 40)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Incoming Object Waypoint Data: Radius is greater than 40, setting to 40.");
        dRadius = 40;
    }

    // Create new waypoint struct with data from the RoveComm packet.
    geoops::Waypoint stObjectWaypoint(geoops::GPSCoordinate(stPacket.vData[0], stPacket.vData[1]), eWaypointType, dRadius);

    // Acquire write lock for writing to waypoints vector.
    std::unique_lock<std::shared_mutex> lkWaypointsLock(m_muWaypointsMutex);
    // Queue waypoint.
    m_vWaypointList.emplace_back(stObjectWaypoint);
    // Unlock mutex.
    lkWaypointsLock.unlock();

    // Submit logger message.
    LOG_NOTICE(logging::g_qSharedLogger,
               "Incoming Object Waypoint Data: Added (lat: {}, lon: {}, id: {}, radius: {}) to WaypointHandler queue.",
               stPacket.vData[0],
               stPacket.vData[1],
               dObjectID,
               dRadius);
}

/******************************************************************************
 * @brief Callback function that is called whenever RoveComm receives new ADDOBSTACLE packet.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-06
 ******************************************************************************/
void WaypointHandler::AddObstacleCallback(const rovecomm::RoveCommPacket<double>& stPacket)
{
    // Create instance variables.
    double dRadius = stPacket.vData[2];

    // Limit the radius to 0-40.
    if (dRadius < 0)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Incoming Obstacle Waypoint Data: Radius is less than 0, setting to 0.");
        dRadius = 0;
    }
    else if (dRadius > 40)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Incoming Obstacle Waypoint Data: Radius is greater than 40, setting to 40.");
        dRadius = 40;
    }

    // Create new waypoint struct with data from the RoveComm packet.
    geoops::Waypoint stObstacleWaypoint(geoops::GPSCoordinate(stPacket.vData[0], stPacket.vData[1]), geoops::WaypointType::eObstacleWaypoint, dRadius);

    // Acquire write lock for writing to waypoints vector.
    std::unique_lock<std::shared_mutex> lkWaypointsLock(m_muWaypointsMutex);
    // Queue waypoint.
    m_vPermanentObstacles.emplace_back(stObstacleWaypoint);
    // Unlock mutex.
    lkWaypointsLock.unlock();

    // Submit logger message.
    LOG_NOTICE(logging::g_qSharedLogger,
               "Incoming Obstacle Waypoint Data: Added (lat: {}, lon: {}, radius: {}) to WaypointHandler queue. Total Obstacles: {}",
               stPacket.vData[0],
               stPacket.vData[1],
               dRadius,
               m_vPermanentObstacles.size());
}

/******************************************************************************
 * @brief Callback function that is called whenever RoveComm receives new CLEARWAYPOINTS packet.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-03
 ******************************************************************************/
void WaypointHandler::ClearWaypointsCallback(const rovecomm::RoveCommPacket<uint8_t>& stPacket)
{
    // Not using this.
    (void) stPacket;

    // Acquire write lock for writing to waypoints vector.
    std::unique_lock<std::shared_mutex> lkWaypointsLock(m_muWaypointsMutex);
    // Clear waypoints queue.
    m_vWaypointList.clear();
    // Unlock mutex.
    lkWaypointsLock.unlock();

    // Submit logger message.
    LOG_NOTICE(logging::g_qSharedLogger, "Incoming Clear Waypoints packet: Cleared WaypointHandler queue.");
}

/******************************************************************************
 * @brief Callback function that is called whenever RoveComm receives new CLEAROBSTACLES packet.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-19
 ******************************************************************************/
void WaypointHandler::ClearObstaclesCallback(const rovecomm::RoveCommPacket<uint8_t>& stPacket)
{
    // Not using this.
    (void) stPacket;

    // Acquire write lock for obstacle vector.
    std::unique_lock<std::shared_mutex> lkObstaclesLock(m_muObstaclesMutex);
    // Clear obstacles queue.
    m_vPermanentObstacles.clear();
    // Unlock mutex.
    lkObstaclesLock.unlock();

    // Submit logger message.
    LOG_NOTICE(logging::g_qSharedLogger, "Incoming Clear Obstacles packet: Cleared permanent obstacles list.");
}
