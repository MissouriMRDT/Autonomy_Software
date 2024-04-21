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
 * @brief Construct a new geoops::Waypoint Handler:: geoops::Waypoint Handler object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-02
 ******************************************************************************/
WaypointHandler::WaypointHandler()
{
    // Set RoveComm callbacks.
    network::g_pRoveCommUDPNode->AddUDPCallback<double>(AddPositionLegCallback, manifest::Autonomy::COMMANDS.find("ADDPOSITIONLEG")->second.DATA_ID);
    network::g_pRoveCommUDPNode->AddUDPCallback<double>(AddMarkerLegCallback, manifest::Autonomy::COMMANDS.find("ADDMARKERLEG")->second.DATA_ID);
    network::g_pRoveCommUDPNode->AddUDPCallback<double>(AddObjectLegCallback, manifest::Autonomy::COMMANDS.find("ADDOBJECTLEG")->second.DATA_ID);
    network::g_pRoveCommUDPNode->AddUDPCallback<uint8_t>(ClearWaypointsCallback, manifest::Autonomy::COMMANDS.find("CLEARWAYPOINTS")->second.DATA_ID);
}

/******************************************************************************
 * @brief Destroy the geoops::Waypoint Handler:: geoops::Waypoint Handler object.
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
 * @param eType - The leg type of the waypoint signalling if this is a tag, navigation, object, etc. waypoint.
 * @param dRadius - The circular area around the waypoint that should be counted as reaching the waypoint. Or object radius.
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
 * @param eType - The leg type of the waypoint signalling if this is a tag, navigation, object, etc. waypoint.
 * @param dRadius - The circular area around the waypoint that should be counted as reaching the waypoint. Or object radius.
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
 * @brief Append a new object to the WaypointHandler object list.
 *
 * @param stWaypoint - The WaypointHandler::geoops::Waypoint struct containing information about the object to
 *                  store in the handler.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::AddObject(const geoops::Waypoint& stWaypoint)
{
    // Acquire a write lock on the object vector.
    std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObjectsMutex);
    // Add object waypoint to end of member variable vector.
    m_vPermanentObjects.emplace_back(stWaypoint);
}

/******************************************************************************
 * @brief Append a new object to the WaypointHandler object list.
 *
 * @param stLocation - The location of the waypoint stored in a geoops namespace GPSCoordinate struct.
 * @param dRadius - The circular area around the object or the object radius.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::AddObject(const geoops::GPSCoordinate& stLocation, const double dRadius)
{
    // Construct a new waypoint struct from the given info.
    geoops::Waypoint stTempWaypoint(stLocation, geoops::WaypointType::eObstacleWaypoint, dRadius);

    // Acquire a write lock on the waypoint vector.
    std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObjectsMutex);
    // Add waypoint to end of member variable vector.
    m_vPermanentObjects.emplace_back(stTempWaypoint);
}

/******************************************************************************
 * @brief Append a new object to the WaypointHandler object list.
 *
 * @param stLocation - The location of the waypoint stored in a geoops namespace UTMCoordinate struct.
 * @param dRadius - The circular area around the object or the object radius.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::AddObject(const geoops::UTMCoordinate& stLocation, const double dRadius)
{
    // Construct a new waypoint struct from the given info.
    geoops::Waypoint stTempWaypoint(stLocation, geoops::WaypointType::eObstacleWaypoint, dRadius);

    // Acquire a write lock on the waypoint vector.
    std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObjectsMutex);
    // Add waypoint to end of member variable vector.
    m_vPermanentObjects.emplace_back(stTempWaypoint);
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
 * @brief Delete the object at a given index from the waypoint handler object list.
 *
 * @param nIndex - The index of the element to remove.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::DeleteObject(const long unsigned int nIndex)
{
    // Acquire a read lock on the waypoint vector.
    std::shared_lock<std::shared_mutex> lkObjectListLock(m_muObjectsMutex);
    // Check if the vector has at least one waypoint.
    if (nIndex < m_vPermanentObjects.size())
    {
        // Release read lock.
        lkObjectListLock.unlock();

        // Acquire a write lock on the object vector.
        std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObjectsMutex);
        // Delete the geoops::Waypoint at the index.
        m_vPermanentObjects.erase(m_vPermanentObjects.begin() + nIndex);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Attempted to delete an object waypoint at index {} from the WaypointHandler but it is already empty or the index is out of bounds!",
                  nIndex);
    }
}

/******************************************************************************
 * @brief Delete an object from the WaypointHandler given a matching location.
 *      Any waypoint in the list that matches the given location will be removed.
 *
 * @param stWaypoint - The equivalent object location that should be removed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
void WaypointHandler::DeleteObject(const geoops::Waypoint& stWaypoint)
{
    // Acquire a write lock on the object vector.
    std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObjectsMutex);
    // Delete any waypoint matching the given one from the list.
    m_vPermanentObjects.erase(std::remove(m_vPermanentObjects.begin(), m_vPermanentObjects.end(), stWaypoint), m_vPermanentObjects.end());
}

/******************************************************************************
 * @brief Delete an object from the WaypointHandler given a matching location.
 *      Any waypoint in the list that matches the given location will be removed.
 *
 * @param stLocation - The equivalent object location that should be removed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
void WaypointHandler::DeleteObject(const geoops::GPSCoordinate& stLocation)
{
    // Acquire a write lock on the object vector.
    std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObjectsMutex);
    // Delete any waypoint matching the given location from the list.
    m_vPermanentObjects.erase(std::remove_if(m_vPermanentObjects.begin(),
                                             m_vPermanentObjects.end(),
                                             [stLocation](const geoops::Waypoint& stWaypoint) { return stWaypoint.GetGPSCoordinate() == stLocation; }),
                              m_vPermanentObjects.end());
}

/******************************************************************************
 * @brief Delete an object from the WaypointHandler given a matching location.
 *      Any waypoint in the list that matches the given location will be removed.
 *
 * @param stLocation - The equivalent object location that should be removed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
void WaypointHandler::DeleteObject(const geoops::UTMCoordinate& stLocation)
{
    // Acquire a write lock on the object vector.
    std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObjectsMutex);
    // Delete any waypoint matching the given location from the list.
    m_vPermanentObjects.erase(std::remove_if(m_vPermanentObjects.begin(),
                                             m_vPermanentObjects.end(),
                                             [stLocation](const geoops::Waypoint& stWaypoint) { return stWaypoint.GetUTMCoordinate() == stLocation; }),
                              m_vPermanentObjects.end());
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
void WaypointHandler::ClearObjects()
{
    // Acquire a write lock on the path unordered map.
    std::unique_lock<std::shared_mutex> lkObjectsLock(m_muObjectsMutex);
    // Clear the object vector.
    m_vPermanentObjects.clear();
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
 * @brief Retrieve an immutable reference to the object at the given index.
 *
 * @param nIndex - The index of the element to retrieve.
 * @return const WaypointHandler::geoops::Waypoint - An immutable reference to the object geoops::Waypoint containing data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
const geoops::Waypoint WaypointHandler::RetrieveObjectAtIndex(const long unsigned int nIndex)
{
    // Acquire a read lock on the waypoint vector.
    std::shared_lock<std::shared_mutex> lkObjectsLock(m_muObjectsMutex);
    // Check if the vector has at least one waypoint.
    if (nIndex < m_vPermanentObjects.size())
    {
        // Return an immutable reference to the waypoint at the index.
        return m_vPermanentObjects[nIndex];
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "Attempted to retrieve a object at index {} from the WaypointHandler but it is empty or the index is out of bounds!", nIndex);

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
 * @brief Accessor for the full list of current object stored in the WaypointHandler.
 *
 * @return const std::vector<WaypointHandler::geoops::Waypoint> - A vector of geoops::Waypoint structs representing
 *                                      objects that are currently stored in the WaypointHandler.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
const std::vector<geoops::Waypoint> WaypointHandler::GetAllObjects()
{
    // Acquire a read lock on the path unordered map.
    std::shared_lock<std::shared_mutex> lkObjectsLock(m_muObjectsMutex);
    // Return a copy of the current object list.
    return m_vPermanentObjects;
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
 * @brief Accessor for the number of elements on the WaypointHandler's object vector.
 *
 * @return int - The size of the object vector.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
int WaypointHandler::GetObjectsCount()
{
    // Acquire a write lock on the waypoint vector.
    std::shared_lock<std::shared_mutex> lkObjectsLock(m_muObjectsMutex);
    // Return total number of objects stored.
    return m_vPermanentObjects.size();
}

/******************************************************************************
 * @brief Retrieve the rover's current position and heading. Automatically picks between
 *      getting the position/heading from the NavBoard, ZEDSDK Fusion module, or ZED positional tracking.
 *      In most cases, this will be the method that should be called over getting the data directly
 *      from NavBoard.
 *
 * @return geoops::RoverPose - The current position and heading (pose) of the rover stored in a RoverPose struct.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-06
 ******************************************************************************/
geoops::RoverPose WaypointHandler::SmartRetrieveRoverPose()
{
    // Get and store the normal GPS position and heading from NavBoard.
    geoops::GPSCoordinate stCurrentGPSPosition = globals::g_pNavigationBoard->GetGPSData();
    double dCurrentGPSHeading                  = globals::g_pNavigationBoard->GetHeading();

    // Create instance variables.
    ZEDCam* pMainCam                        = globals::g_pCameraHandler->GetZED(CameraHandler::eHeadMainCam);
    geoops::GPSCoordinate stCurrentPosition = stCurrentGPSPosition;
    double dCurrentHeading                  = dCurrentGPSHeading;
    bool bVIOGPSFused                       = false;

    // Check if the main ZED camera is opened and the fusion module is initialized.
    if (pMainCam->GetCameraIsOpen() && pMainCam->GetPositionalTrackingEnabled())
    {
        // Check if GNSS fusion is enabled and current GPS data from has differential accuracy.
        if (constants::FUSION_ENABLE_GNSS_FUSION && pMainCam->GetIsFusionMaster() && stCurrentGPSPosition.bIsDifferential)
        {
            // Create instance variables.
            sl::GeoPose slCurrentCameraGeoPose;
            ZEDCam::Pose stCurrentCameraVIOPose;

            // Get the current camera pose from the ZEDCam.
            std::future<bool> fuResultStatus  = pMainCam->RequestFusionGeoPoseCopy(slCurrentCameraGeoPose);
            std::future<bool> fuResultStatus2 = pMainCam->RequestPositionalPoseCopy(stCurrentCameraVIOPose);
            // Wait for future to be fulfilled.
            if (fuResultStatus.get() && fuResultStatus2.get())
            {
                // Repack the camera pose into a GPSCoordinate.
                stCurrentPosition.dLatitude  = slCurrentCameraGeoPose.latlng_coordinates.getLatitude(false);
                stCurrentPosition.dLongitude = slCurrentCameraGeoPose.latlng_coordinates.getLongitude(false);
                stCurrentPosition.dAltitude  = slCurrentCameraGeoPose.latlng_coordinates.getAltitude();
                // Repack the camera pose into a UTMCoordinate.
                // dCurrentHeading = slCurrentCameraGeoPose.heading * (180.0 / M_PI);    // This doesn't work because the heading is on the wrong axis for some reason.
                dCurrentHeading = stCurrentCameraVIOPose.stEulerAngles.dYO;

                // Set fused toggle.
                bVIOGPSFused = true;
            }
            else
            {
                // Just return normal GPS position and heading from NavBoard.
                stCurrentPosition = stCurrentGPSPosition;
                dCurrentHeading   = dCurrentGPSHeading;
            }
        }
        else
        {
            // Create instance variables.
            ZEDCam::Pose stCurrentCameraVIOPose;

            // Get the current camera pose from the ZEDCam.
            std::future<bool> fuResultStatus = pMainCam->RequestPositionalPoseCopy(stCurrentCameraVIOPose);
            // Wait for future to be fulfilled.
            if (fuResultStatus.get())
            {
                // Camera is using UTM. Modify current GPS position to be camera's position.
                geoops::UTMCoordinate stCameraUTMLocation = geoops::ConvertGPSToUTM(stCurrentGPSPosition);
                // Repack the camera pose into a GPSCoordinate.
                stCameraUTMLocation.dEasting  = stCurrentCameraVIOPose.stTranslation.dX;
                stCameraUTMLocation.dNorthing = stCurrentCameraVIOPose.stTranslation.dZ;
                stCameraUTMLocation.dAltitude = stCurrentCameraVIOPose.stTranslation.dY;
                // Convert back to GPS coordinate and store.
                stCurrentPosition = geoops::ConvertUTMToGPS(stCameraUTMLocation);
                // Get compass heading based off of the ZED's aligned accelerometer.
                dCurrentHeading = stCurrentCameraVIOPose.stEulerAngles.dYO;

                // Set fused toggle.
                bVIOGPSFused = false;
            }
            else
            {
                // Just return normal GPS position and heading from NavBoard.
                stCurrentPosition = stCurrentGPSPosition;
                dCurrentHeading   = dCurrentGPSHeading;
            }
        }
    }

    // Submit logger message.
    geoops::UTMCoordinate stCurrentUTMPosition = geoops::ConvertGPSToUTM(stCurrentGPSPosition);
    LOG_DEBUG(logging::g_qSharedLogger,
              "Rover Pose is currently: {} (lat), {} (lon), {} (alt), {} (degrees), GNSS/VIO FUSED? = {}",
              stCurrentUTMPosition.dEasting,
              stCurrentUTMPosition.dNorthing,
              stCurrentUTMPosition.dAltitude,
              dCurrentHeading,
              bVIOGPSFused ? "true" : "false");

    return geoops::RoverPose(stCurrentPosition, dCurrentHeading);
}

/******************************************************************************
 * @brief Retrieve the rover's current velocity. Currently there is no easy way
 *      to get the velocity of the ZEDCam so this method just returns the GPS-based
 *      velocity.
 *
 * @return double - The current velocity of the rover. (m/s)
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-06
 ******************************************************************************/
double WaypointHandler::SmartRetrieveVelocity()
{
    // Return the GPS-based velocity from the NavBoard.
    return globals::g_pNavigationBoard->GetVelocity();
}

/******************************************************************************
 * @brief Retrieve the rover's current velocity. Currently there is no easy way
 *      to get the velocity of the ZEDCam so this method just returns the GPS-based
 *      velocity.
 *
 * @return double - The current angular velocity of the rover. (deg/s)
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-06
 ******************************************************************************/
double WaypointHandler::SmartRetrieveAngularVelocity()
{
    // Return the GPS-based angular velocity from the NavBoard.
    return globals::g_pNavigationBoard->GetAngularVelocity();
}
