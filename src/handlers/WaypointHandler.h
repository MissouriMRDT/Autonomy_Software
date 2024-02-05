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

        /******************************************************************************
         * @brief This struct is used by the WaypointHandler class to store location, size,
         *      and type information about a given location of interest of waypoint.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-02-02
         ******************************************************************************/
        struct Waypoint
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
        const Waypoint RetrieveObjectAtIndex(const long unsigned int nIndex);
        const std::vector<Waypoint> GetAllWaypoints();
        const std::vector<Waypoint> GetAllObjects();
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
