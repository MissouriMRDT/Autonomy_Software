/******************************************************************************
 * @brief Defines the WaypointHandler class.
 *
 * @file WaypointHandler.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-02
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef WAYPOINT_HANDLER_H
#define WAYPOINT_HANDLER_H

#include "../util/GeospatialOperations.hpp"

/// \cond
#include <RoveComm/RoveComm.h>
#include <RoveComm/RoveCommManifest.h>
#include <shared_mutex>

/// \endcond

/******************************************************************************
 * @brief The WaypointHandler class is used throughout the entire project (mainly
 *      by the state machine) to globally store a list of waypoints that the rover
 *      will navigate to.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-02
 ******************************************************************************/
class WaypointHandler
{
    public:
        /////////////////////////////////////////
        // Declare public member variables.
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Declare public primary methods.
        /////////////////////////////////////////

        WaypointHandler();
        ~WaypointHandler();
        void AddWaypoint(const geoops::Waypoint& stWaypoint);
        void AddWaypoint(const geoops::GPSCoordinate& stLocation, const geoops::WaypointType& eType, const double dRadius = 0.0);
        void AddWaypoint(const geoops::UTMCoordinate& stLocation, const geoops::WaypointType& eType, const double dRadius = 0.0);
        void StorePath(const std::string& szPathName, const std::vector<geoops::Waypoint>& vWaypointPath);
        void StorePath(const std::string& szPathName, const std::vector<geoops::GPSCoordinate>& vLocationPath);
        void StorePath(const std::string& szPathName, const std::vector<geoops::UTMCoordinate>& vLocationPath);
        void AddObject(const geoops::Waypoint& stWaypoint);
        void AddObject(const geoops::GPSCoordinate& stLocation, const double dRadius = 0.0);
        void AddObject(const geoops::UTMCoordinate& stLocation, const double dRadius = 0.0);
        void DeleteWaypoint(const long unsigned int nIndex);
        void DeleteWaypoint(const geoops::Waypoint& stWaypoint);
        void DeleteWaypoint(const geoops::GPSCoordinate& stLocation);
        void DeleteWaypoint(const geoops::UTMCoordinate& stLocation);
        bool DeletePath(const std::string& szPathName);
        void DeleteObject(const long unsigned int nIndex);
        void DeleteObject(const geoops::Waypoint& stWaypoint);
        void DeleteObject(const geoops::GPSCoordinate& stLocation);
        void DeleteObject(const geoops::UTMCoordinate& stLocation);
        void ClearWaypoints();
        void ClearPaths();
        void ClearObjects();

        /////////////////////////////////////////
        // Setters.
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Getters.
        /////////////////////////////////////////
        geoops::Waypoint PopNextWaypoint();
        const geoops::Waypoint PeekNextWaypoint();
        const geoops::Waypoint RetrieveWaypointAtIndex(const long unsigned int nIndex);
        const std::vector<geoops::Waypoint> RetrievePath(const std::string& szPathName);
        const geoops::Waypoint RetrieveObjectAtIndex(const long unsigned int nIndex);
        const std::vector<geoops::Waypoint> GetAllWaypoints();
        const std::vector<geoops::Waypoint> GetAllObjects();
        int GetWaypointCount();
        int GetPathsCount();
        int GetObjectsCount();

        // Smart location retrieving.
        geoops::RoverPose SmartRetrieveRoverPose(bool bVIOTracking = false);
        double SmartRetrieveVelocity();
        double SmartRetrieveAngularVelocity();

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        std::vector<geoops::Waypoint> m_vWaypointList;
        std::shared_mutex m_muWaypointsMutex;
        std::unordered_map<std::string, std::vector<geoops::Waypoint>> m_umStoredPaths;
        std::shared_mutex m_muPathMutex;
        std::vector<geoops::Waypoint> m_vPermanentObjects;
        std::shared_mutex m_muObjectsMutex;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Callback function that is called whenever RoveComm receives new ADDPOSITIONLEG packet.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-03
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<double>&, const sockaddr_in&)> AddPositionLegCallback =
            [this](const rovecomm::RoveCommPacket<double>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stdAddr;

            // Create new waypoint struct with data from the RoveComm packet.
            geoops::Waypoint stNavWaypoint(geoops::GPSCoordinate(stPacket.vData[0], stPacket.vData[1]), geoops::WaypointType::eNavigationWaypoint);

            // Acquire write lock for writing to waypoints vector.
            std::unique_lock<std::shared_mutex> lkWaypointsLock(m_muWaypointsMutex);
            // Queue waypoint.
            m_vWaypointList.emplace_back(stNavWaypoint);
            // Unlock mutex.
            lkWaypointsLock.unlock();

            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger,
                     "Incoming Navigation Waypoint Data: Added (lat: {}, lon: {}) to WaypointHandler queue.",
                     stPacket.vData[0],
                     stPacket.vData[1]);
        };

        /******************************************************************************
         * @brief Callback function that is called whenever RoveComm receives new ADDMARKERLEG packet.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-03
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<double>&, const sockaddr_in&)> AddMarkerLegCallback =
            [this](const rovecomm::RoveCommPacket<double>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stdAddr;

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
            LOG_INFO(logging::g_qSharedLogger,
                     "Incoming Marker Waypoint Data: Added (lat: {}, lon: {}, marker ID: {}, radius: {}) to WaypointHandler queue.",
                     stPacket.vData[0],
                     stPacket.vData[1],
                     nMarkerID,
                     dRadius);
        };

        /******************************************************************************
         * @brief Callback function that is called whenever RoveComm receives new ADDOBJECTLEG packet.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-03
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<double>&, const sockaddr_in&)> AddObjectLegCallback =
            [this](const rovecomm::RoveCommPacket<double>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stdAddr;

            // Create instance variables.
            double dRadius = stPacket.vData[2];

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
            geoops::Waypoint stObjectWaypoint(geoops::GPSCoordinate(stPacket.vData[0], stPacket.vData[1]), geoops::WaypointType::eObjectWaypoint, dRadius);

            // Acquire write lock for writing to waypoints vector.
            std::unique_lock<std::shared_mutex> lkWaypointsLock(m_muWaypointsMutex);
            // Queue waypoint.
            m_vWaypointList.emplace_back(stObjectWaypoint);
            // Unlock mutex.
            lkWaypointsLock.unlock();

            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger,
                     "Incoming Object Waypoint Data: Added (lat: {}, lon: {}, radius: {}) to WaypointHandler queue.",
                     stPacket.vData[0],
                     stPacket.vData[1],
                     dRadius);
        };

        /******************************************************************************
         * @brief Callback function that is called whenever RoveComm receives new ADDOBSTACLE packet.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-06
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<double>&, const sockaddr_in&)> AddObstacleCallback =
            [this](const rovecomm::RoveCommPacket<double>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stdAddr;

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
            m_vWaypointList.emplace_back(stObstacleWaypoint);
            // Unlock mutex.
            lkWaypointsLock.unlock();

            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger,
                     "Incoming Obstacle Waypoint Data: Added (lat: {}, lon: {}, radius: {}) to WaypointHandler queue.",
                     stPacket.vData[0],
                     stPacket.vData[1],
                     dRadius);
        };

        /******************************************************************************
         * @brief Callback function that is called whenever RoveComm receives new CLEARWAYPOINTS packet.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-03
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<uint8_t>&, const sockaddr_in&)> ClearWaypointsCallback =
            [this](const rovecomm::RoveCommPacket<uint8_t>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stPacket;
            (void) stdAddr;

            // Acquire write lock for writing to waypoints vector.
            std::unique_lock<std::shared_mutex> lkWaypointsLock(m_muWaypointsMutex);
            // Clear waypoints queue.
            m_vWaypointList.clear();
            // Unlock mutex.
            lkWaypointsLock.unlock();

            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Incoming Clear Waypoints packet: Cleared WaypointHandler queue.");
        };
};

#endif
