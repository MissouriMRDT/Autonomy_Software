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
        // Declare public enums that are specific to and used within this class.
        /////////////////////////////////////////

        // This enum is used to store values for the waypoint type.
        enum WaypointType
        {
            eNavigationWaypoint,
            eTagWaypoint,
            eMalletWaypoint,
            eWaterBottleWaypoint,
            eObstacleWaypoint,
            eUNKNOWN
        };

        /////////////////////////////////////////
        // Declare public structs that are specific to and used within this class.
        /////////////////////////////////////////

        struct Waypoint;

        /////////////////////////////////////////
        // Declare public member variables.
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Declare public primary methods.
        /////////////////////////////////////////

        WaypointHandler();
        ~WaypointHandler();
        void AddWaypoint(const Waypoint& stWaypoint);
        void AddWaypoint(const geoops::GPSCoordinate& stLocation, const WaypointType& eType, const double dRadius = 0.0);
        void AddWaypoint(const geoops::UTMCoordinate& stLocation, const WaypointType& eType, const double dRadius = 0.0);
        void StorePath(const std::string& szPathName, const std::vector<Waypoint>& vWaypointPath);
        void StorePath(const std::string& szPathName, const std::vector<geoops::GPSCoordinate>& vLocationPath);
        void StorePath(const std::string& szPathName, const std::vector<geoops::UTMCoordinate>& vLocationPath);
        void AddObject(const Waypoint& stWaypoint);
        void AddObject(const geoops::GPSCoordinate& stLocation, const double dRadius = 0.0);
        void AddObject(const geoops::UTMCoordinate& stLocation, const double dRadius = 0.0);
        void DeleteWaypoint(const long unsigned int nIndex);
        void DeleteWaypoint(const Waypoint& stWaypoint);
        void DeleteWaypoint(const geoops::GPSCoordinate& stLocation);
        void DeleteWaypoint(const geoops::UTMCoordinate& stLocation);
        bool DeletePath(const std::string& szPathName);
        void DeleteObject(const long unsigned int nIndex);
        void DeleteObject(const Waypoint& stWaypoint);
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
        Waypoint PopNextWaypoint();
        const Waypoint PeekNextWaypoint();
        const Waypoint RetrieveWaypointAtIndex(const long unsigned int nIndex);
        const std::vector<Waypoint> RetrievePath(const std::string& szPathName);
        int GetWaypointCount();
        int GetPathsCount();
        int GetObjectsCount();

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        std::vector<Waypoint> m_vWaypointList;
        std::shared_mutex m_muWaypointsMutex;
        std::unordered_map<std::string, std::vector<Waypoint>> m_umStoredPaths;
        std::shared_mutex m_muPathMutex;
        std::vector<Waypoint> m_vPermanentObjects;
        std::shared_mutex m_muObjectsMutex;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////
};

#endif
