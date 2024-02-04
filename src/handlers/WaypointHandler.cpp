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

/// \cond
#include <algorithm>

/// \endcond

/******************************************************************************
 * @brief This struct is used by the WaypointHandler class to store location, size,
 *      and type information about a given location of interest of waypoint.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-02
 ******************************************************************************/
struct WaypointHandler::Waypoint
{
    private:
        // Declare struct private member variables.
        geoops::GPSCoordinate stGPSLocation;    // This is not meant to be changed once this waypoint is created.
        geoops::UTMCoordinate stUTMLocation;    // This is not meant to be changed once this waypoint is created.

    public:
        // Declare struct public member variables.
        WaypointType eType;
        double dRadius;

        /******************************************************************************
         * @brief Construct a new Waypoint object.
         *
         * @param stGPSLocation - The location of this waypoint stored in a geoops namespace
         *                  GPSCoordinate struct.
         * @param eType - The waypoint type. Navigation, Intermediate, Tag, Object, etc.
         * @param dRadius - The size of the waypoint. This is mainly only useful for objects or when
         *              you want the rover to just go to a general area.
         *
         * @note This will also store the equivalent UTM coordinate.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-02-02
         ******************************************************************************/
        Waypoint(const geoops::GPSCoordinate& stGPSLocation = geoops::GPSCoordinate(), const WaypointType& eType = eUNKNOWN, const double dRadius = 0.0)
        {
            // Initialize member variables.
            this->stGPSLocation = stGPSLocation;
            this->stUTMLocation = geoops::ConvertGPSToUTM(stGPSLocation);
            this->eType         = eType;
            this->dRadius       = dRadius;
        }

        /******************************************************************************
         * @brief Construct a new Waypoint object.
         *
         * @param stUTMLocation - The location of this waypoint stored in a geoops namespace
         *                  UTMCoordinate struct.
         * @param eType - The waypoint type. Navigation, Intermediate, Tag, Object, etc.
         * @param dRadius - The size of the waypoint. This is mainly only useful for objects or when
         *              you want the rover to just go to a general area.
         *
         * @note This will also store the equivalent GPS coordinate.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-02-02
         ******************************************************************************/
        Waypoint(const geoops::UTMCoordinate& stUTMLocation = geoops::UTMCoordinate(), const WaypointType& eType = eUNKNOWN, const double dRadius = 0.0)
        {
            // Initialize member variables.
            this->stUTMLocation = stUTMLocation;
            this->stGPSLocation = geoops::ConvertUTMToGPS(stUTMLocation);
            this->eType         = eType;
            this->dRadius       = dRadius;
        }

        /******************************************************************************
         * @brief Accessor for the geoops::GPSCoordinate member variable.
         *
         * @return geoops::GPSCoordinate - The location of the waypoint stored in a geoops namespace
         *                      GPSCoordinate struct.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-02-02
         ******************************************************************************/
        const geoops::GPSCoordinate& GetGPSCoordinate() const { return stGPSLocation; }

        /******************************************************************************
         * @brief Accessor for the geoops::UTMCoordinate member variable.
         *
         * @return geoops::UTMCoordinate - The location of the waypoint stored in a geoops namespace
         *                      UTMCoordinate struct.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-02-02
         ******************************************************************************/
        const geoops::UTMCoordinate& GetUTMCoordinate() const { return stUTMLocation; }

        /******************************************************************************
         * @brief Overridden operator equals for Waypoint struct.
         *
         * @param stOtherCoordinate - The other Waypoint struct we are comparing to.
         * @return true - The two Waypoints are equal.
         * @return false - The two Waypoints are not equal.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-02-04
         ******************************************************************************/
        bool operator==(const Waypoint& stOtherWaypoint) const
        {
            // Check if location, altitude, and accuracy are the same. Not going to worry about other values for now.
            if (stGPSLocation == stOtherWaypoint.stGPSLocation && stUTMLocation == stOtherWaypoint.stUTMLocation && eType == stOtherWaypoint.eType &&
                dRadius == stOtherWaypoint.dRadius)
            {
                // Return that the two Waypoints are equal.
                return true;
            }
            else
            {
                // Return that the two Waypoints are not equal.
                return false;
            }
        }
};

/******************************************************************************
 * @brief Construct a new Waypoint Handler:: Waypoint Handler object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-02
 ******************************************************************************/
WaypointHandler::WaypointHandler()
{
    // Nothing to initialize.
}

/******************************************************************************
 * @brief Destroy the Waypoint Handler:: Waypoint Handler object.
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
 * @param stWaypoint - The WaypointHandler::Waypoint struct containing information about the waypoint to
 *                  store in the handler.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-02
 ******************************************************************************/
void WaypointHandler::AddWaypoint(const Waypoint& stWaypoint)
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
void WaypointHandler::AddWaypoint(const geoops::GPSCoordinate& stLocation, const WaypointType& eType, const double dRadius)
{
    // Construct a new waypoint struct from the given info.
    Waypoint stTempWaypoint(stLocation, eType, dRadius);

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
void WaypointHandler::AddWaypoint(const geoops::UTMCoordinate& stLocation, const WaypointType& eType, const double dRadius)
{
    // Construct a new waypoint struct from the given info.
    Waypoint stTempWaypoint(stLocation, eType, dRadius);

    // Acquire a write lock on the waypoint vector.
    std::unique_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Add waypoint to end of member variable vector.
    m_vWaypointList.emplace_back(stTempWaypoint);
}

/******************************************************************************
 * @brief Store a path in the WaypointHandler.
 *
 * @param szPathName - The key that will be used to store, and later reference, the path in the WaypointHandler.
 * @param vWaypointPath - A vector containing Waypoint structs with data about each point on the path.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::StorePath(const std::string& szPathName, const std::vector<Waypoint>& vWaypointPath)
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
 * @note Paths must be stored in the WaypointHandler as a vector is Waypoint structs. This will create a new Waypoint
 *      struct for each GPSCoordinate and use a default type of eNavigationWaypoint with a radius of 0.0.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::StorePath(const std::string& szPathName, const std::vector<geoops::GPSCoordinate>& vLocationPath)
{
    // Create instance variables.
    std::vector<Waypoint> vWaypointPath;

    // Loop through each GPSCoordinate in the given vector and repack the info into a Waypoint.
    for (geoops::GPSCoordinate stLocation : vLocationPath)
    {
        // Create a new waypoint and store location info in it.
        Waypoint stWaypoint(stLocation, eNavigationWaypoint);

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
 * @note Paths must be stored in the WaypointHandler as a vector is Waypoint structs. This will create a new Waypoint
 *      struct for each UTMCoordinate and use a default type of eNavigationWaypoint with a radius of 0.0.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::StorePath(const std::string& szPathName, const std::vector<geoops::UTMCoordinate>& vLocationPath)
{
    // Create instance variables.
    std::vector<Waypoint> vWaypointPath;

    // Loop through each UTMCoordinate in the given vector and repack the info into a Waypoint.
    for (geoops::UTMCoordinate stLocation : vLocationPath)
    {
        // Create a new waypoint and store location info in it.
        Waypoint stWaypoint(stLocation, eNavigationWaypoint);

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
 * @param stWaypoint - The WaypointHandler::Waypoint struct containing information about the object to
 *                  store in the handler.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
void WaypointHandler::AddObject(const Waypoint& stWaypoint)
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
    Waypoint stTempWaypoint(stLocation, eObstacleWaypoint, dRadius);

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
    Waypoint stTempWaypoint(stLocation, eObstacleWaypoint, dRadius);

    // Acquire a write lock on the waypoint vector.
    std::unique_lock<std::shared_mutex> lkObjectListLock(m_muObjectsMutex);
    // Add waypoint to end of member variable vector.
    m_vPermanentObjects.emplace_back(stTempWaypoint);
}

/******************************************************************************
 * @brief Delete the Waypoint at a given index from the waypoint handler.
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
        // Delete the Waypoint at the index.
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
void WaypointHandler::DeleteWaypoint(const Waypoint& stWaypoint)
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
    m_vWaypointList.erase(
        std::remove_if(m_vWaypointList.begin(), m_vWaypointList.end(), [stLocation](const Waypoint& stWaypoint) { return stWaypoint.GetGPSCoordinate() == stLocation; }),
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
    m_vWaypointList.erase(
        std::remove_if(m_vWaypointList.begin(), m_vWaypointList.end(), [stLocation](const Waypoint& stWaypoint) { return stWaypoint.GetUTMCoordinate() == stLocation; }),
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
        // Delete the Waypoint at the index.
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
void WaypointHandler::DeleteObject(const Waypoint& stWaypoint)
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
                                             [stLocation](const Waypoint& stWaypoint) { return stWaypoint.GetGPSCoordinate() == stLocation; }),
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
                                             [stLocation](const Waypoint& stWaypoint) { return stWaypoint.GetUTMCoordinate() == stLocation; }),
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
 * @return WaypointHandler::Waypoint - The next waypoint data stored in a Waypoint struct.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
WaypointHandler::Waypoint WaypointHandler::PopNextWaypoint()
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
        Waypoint stWaypoint = m_vWaypointList[0];
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
        return Waypoint(geoops::GPSCoordinate(), eUNKNOWN);
    }
}

/******************************************************************************
 * @brief Returns an immutable reference to the Waypoint struct at the front of
 *      the list without removing it.
 *
 * @return const WaypointHandler::Waypoint - A reference to a Waypoint struct containing waypoint data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
const WaypointHandler::Waypoint WaypointHandler::PeekNextWaypoint()
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
        return Waypoint(geoops::GPSCoordinate(), eUNKNOWN);
    }
}

/******************************************************************************
 * @brief Retrieve an immutable reference to the waypoint at the given index.
 *
 * @param nIndex - The index of the element to retrieve.
 * @return const WaypointHandler::Waypoint - An immutable reference to the Waypoint containing data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
const WaypointHandler::Waypoint WaypointHandler::RetrieveWaypointAtIndex(const long unsigned int nIndex)
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
        return Waypoint(geoops::GPSCoordinate(), eUNKNOWN);
    }
}

/******************************************************************************
 * @brief Retrieve an immutable reference to the path at the given path name/key.
 *
 * @param szPathName - The name/key of the path that was previously used to store the path.
 * @return const std::vector<WaypointHandler::Waypoint> - A reference to the Waypoint vector located at the given key.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
const std::vector<WaypointHandler::Waypoint> WaypointHandler::RetrievePath(const std::string& szPathName)
{
    // Acquire a read lock on the path unordered map.
    std::shared_lock<std::shared_mutex> lkPathsLock(m_muPathMutex);
    // Check if the map contains the given key.
    if (m_umStoredPaths.contains(szPathName))
    {
        // Return the path vector at the given key.
        return m_umStoredPaths[szPathName];
    }
    else
    {
        // Return an empty vector.
        return std::vector<Waypoint>();
    }
}

/******************************************************************************
 * @brief Retrieve an immutable reference to the object at the given index.
 *
 * @param nIndex - The index of the element to retrieve.
 * @return const WaypointHandler::Waypoint - An immutable reference to the object Waypoint containing data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-03
 ******************************************************************************/
const WaypointHandler::Waypoint WaypointHandler::RetrieveObjectAtIndex(const long unsigned int nIndex)
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
        return Waypoint(geoops::GPSCoordinate(), eUNKNOWN);
    }
}

/******************************************************************************
 * @brief Accessor for the full list of current waypoints stored in the WaypointHandler.
 *
 * @return const std::vector<WaypointHandler::Waypoint> - A vector of Waypoint structs currently stored in the WaypointHandler.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
const std::vector<WaypointHandler::Waypoint> WaypointHandler::GetAllWaypoints()
{
    // Acquire a read lock on the waypoint vector.
    std::shared_lock<std::shared_mutex> lkWaypointListLock(m_muWaypointsMutex);
    // Return a copy of the current waypoint list.
    return m_vWaypointList;
}

/******************************************************************************
 * @brief Accessor for the full list of current object stored in the WaypointHandler.
 *
 * @return const std::vector<WaypointHandler::Waypoint> - A vector of Waypoint structs representing
 *                                      objects that are currently stored in the WaypointHandler.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-04
 ******************************************************************************/
const std::vector<WaypointHandler::Waypoint> WaypointHandler::GetAllObjects()
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
