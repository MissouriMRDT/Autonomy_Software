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
         *              you want the rover to just to a general area.
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
        }

        /******************************************************************************
         * @brief Construct a new Waypoint object.
         *
         * @param stUTMLocation - The location of this waypoint stored in a geoops namespace
         *                  UTMCoordinate struct.
         * @param eType - The waypoint type. Navigation, Intermediate, Tag, Object, etc.
         * @param dRadius - The size of the waypoint. This is mainly only useful for objects or when
         *              you want the rover to just to a general area.
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
        geoops::GPSCoordinate GetGPSCoordinate() const { return stGPSLocation; }

        /******************************************************************************
         * @brief Accessor for the geoops::UTMCoordinate member variable.
         *
         * @return geoops::UTMCoordinate - The location of the waypoint stored in a geoops namespace
         *                      UTMCoordinate struct.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-02-02
         ******************************************************************************/
        geoops::UTMCoordinate GetUTMCoordinate() const { return stUTMLocation; }
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
 * @brief Add a waypoint to the WaypointHandler.
 *
 * @param stWaypoint - The WaypointHandler::Waypoint struct containing information about the waypoint to
 *                  store in the handler.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-02
 ******************************************************************************/
void WaypointHandler::AddWaypoint(const Waypoint& stWaypoint)
{
    // Add waypoint to end of member variable vector.
    m_vWaypointList.emplace_back(stWaypoint);
}
