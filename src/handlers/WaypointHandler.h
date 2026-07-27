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
        void AddObstacle(const geoops::Waypoint& stWaypoint);
        void AddObstacle(const geoops::GPSCoordinate& stLocation, const double dRadius = 0.0);
        void AddObstacle(const geoops::UTMCoordinate& stLocation, const double dRadius = 0.0);
        void DeleteWaypoint(const long unsigned int nIndex);
        void DeleteWaypoint(const geoops::Waypoint& stWaypoint);
        void DeleteWaypoint(const geoops::GPSCoordinate& stLocation);
        void DeleteWaypoint(const geoops::UTMCoordinate& stLocation);
        bool DeletePath(const std::string& szPathName);
        void DeleteObstacle(const long unsigned int nIndex);
        void DeleteObstacle(const geoops::Waypoint& stWaypoint);
        void DeleteObstacle(const geoops::GPSCoordinate& stLocation);
        void DeleteObstacle(const geoops::UTMCoordinate& stLocation);
        void ClearWaypoints();
        void ClearPaths();
        void ClearObstacles();

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
        const geoops::Waypoint RetrieveObstacleAtIndex(const long unsigned int nIndex);
        const std::vector<geoops::Waypoint> GetAllWaypoints();
        const std::vector<geoops::Waypoint> GetAllObstacles();
        int GetWaypointCount();
        int GetPathsCount();
        int GetObstaclesCount();

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        std::vector<geoops::Waypoint> m_vWaypointList;
        std::shared_mutex m_muWaypointsMutex;
        std::unordered_map<std::string, std::vector<geoops::Waypoint>> m_umStoredPaths;
        std::shared_mutex m_muPathMutex;
        std::vector<geoops::Waypoint> m_vPermanentObstacles;
        std::shared_mutex m_muObstaclesMutex;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void AddPositionLegCallback(const rovecomm::RoveCommPacket<double>& stPacket);
        void AddMarkerLegCallback(const rovecomm::RoveCommPacket<double>& stPacket);
        void AddObjectLegCallback(const rovecomm::RoveCommPacket<double>& stPacket);
        void AddObstacleCallback(const rovecomm::RoveCommPacket<double>& stPacket);
        void ClearWaypointsCallback(const rovecomm::RoveCommPacket<uint8_t>& stPacket);
        void ClearObstaclesCallback(const rovecomm::RoveCommPacket<uint8_t>& stPacket);
};

#endif
