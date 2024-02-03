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
        Waypoint PopNextWaypoint();
        const Waypoint PeekNextWaypoint();
        void DeleteWaypoint(const long unsigned int nIndex);
        bool DeletePath(const std::string& szPathName);
        const Waypoint RetrieveWaypointAtIndex(const long unsigned int nIndex);
        const std::vector<Waypoint> RetrievePath(const std::string& szPathName);
        void ClearWaypoints();
        void ClearPaths();
        void ClearObjects();

        /////////////////////////////////////////
        // Setters.
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Getters.
        /////////////////////////////////////////
        int GetWaypointCount();
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
