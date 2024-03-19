/******************************************************************************
 * @brief Defines and implements namespaces and functions for algorithms
 *      that pertain to search pattern.
 *
 * @file SearchPattern.hpp
 * @author Jacob V (jpvf2d@umsystem.edu)
 * @date 2023-09-21
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef SEARCH_PATTERN_HPP
#define SEARCH_PATTERN_HPP

#include "../util/GeospatialOperations.hpp"

/// \cond
#include <cmath>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief Namespace containing algorithms related to calculating drive powers,
 *      odometry, trajectories, kinematics, etc of search pattern
 *
 * @author Jacob V (jpvf2d@umsystem.edu)
 * @date 2024-02-04
 ******************************************************************************/
namespace searchpattern
{
    /******************************************************************************
     * @brief Perform a spiral search pattern starting from a given point.
     *
     * @param stStartingPoint - The coordinate of the starting point of the search.
     * @param dAngularStepDegrees - The amount the angle is incremented in each
     *      iteration of the loop (degrees).
     * @param dMaxRadius - The maximum radius to cover in the search (meters).
     * @param dStartingHeadingDegrees - The angle the rover is facing at the start
     *      of the search (degrees).
     * @param dSpacing - The spacing between successive points in the spiral
     *      (meters).
     * @return stWaypoints - A vector representing the waypoints forming the spiral
     *      search pattern.
     *
     * @author Jacob V (jpvf2d@umsystem.edu)
     * @date 2024-02-04
     ******************************************************************************/
    inline std::vector<geoops::Waypoint> CalculateSearchPatternWaypoints(const geoops::UTMCoordinate& stStartingPoint,
                                                                         double dAngularStepDegrees     = 57,
                                                                         double dMaxRadius              = 25,
                                                                         double dStartingHeadingDegrees = 0,
                                                                         double dSpacing                = 1)
    {
        // Define variables.
        std::vector<geoops::Waypoint> stWaypoints;
        double dAngularStepRadians = dAngularStepDegrees * M_PI / 180;
        double dAngleRadians       = (dStartingHeadingDegrees + 90) * M_PI / 180;
        double dCurrentRadius      = 0;
        double dStartingX          = stStartingPoint.dEasting;
        double dStartingY          = stStartingPoint.dNorthing;

        // Calculate each waypoint.
        while (dCurrentRadius <= dMaxRadius)
        {
            // Get X and Y positions for the current point.
            double dCurrentX = dStartingX + dCurrentRadius * cos(dAngleRadians);
            double dCurrentY = dStartingY + dCurrentRadius * sin(dAngleRadians);

            // Add the current waypoint to the final vector.
            geoops::UTMCoordinate stCurrentCoordinate(dCurrentX, dCurrentY, stStartingPoint.nZone, stStartingPoint.bWithinNorthernHemisphere);
            geoops::Waypoint stCurrentWaypoint(stCurrentCoordinate, geoops::WaypointType::eNavigationWaypoint);
            stWaypoints.push_back(stCurrentWaypoint);

            // Increment angle and radius for the next waypoint.
            dAngleRadians += dAngularStepRadians;
            dCurrentRadius += dSpacing;
        }

        return stWaypoints;
    }

    /******************************************************************************
     * @brief Perform a spiral search pattern starting from a given point.
     *
     * @param stStartingPoint - The coordinate of the starting point of the search.
     * @param dAngularStepDegrees - The amount the angle is incremented in each
     *      iteration of the loop (degrees).
     * @param dMaxRadius - The maximum radius to cover in the search (meters).
     * @param dStartingHeadingDegrees - The angle the rover is facing at the start
     *      of the search (degrees).
     * @param dSpacing - The spacing between successive points in the spiral
     *      (meters).
     * @return stWaypoints - A vector representing the waypoints forming the spiral
     *      search pattern.
     *
     * @author Jacob V (jpvf2d@umsystem.edu)
     * @date 2024-02-04
     ******************************************************************************/
    inline std::vector<geoops::Waypoint> CalculateSearchPatternWaypoints(const geoops::GPSCoordinate& stStartingPoint,
                                                                         double dAngularStepDegrees     = 57,
                                                                         double dMaxRadius              = 25,
                                                                         double dStartingHeadingDegrees = 0,
                                                                         double dSpacing                = 1)
    {
        // Define variables.
        std::vector<geoops::Waypoint> stWaypoints;
        geoops::UTMCoordinate stStartingPointUTM = geoops::ConvertGPSToUTM(stStartingPoint);
        double dAngularStepRadians               = dAngularStepDegrees * M_PI / 180;
        double dAngleRadians                     = (dStartingHeadingDegrees + 90) * M_PI / 180;
        double dCurrentRadius                    = 0;
        double dStartingX                        = stStartingPointUTM.dEasting;
        double dStartingY                        = stStartingPointUTM.dNorthing;

        // Calculate each waypoint.
        while (dCurrentRadius <= dMaxRadius)
        {
            // Get X and Y positions for the current point.
            double dCurrentX = dStartingX + dCurrentRadius * cos(dAngleRadians);
            double dCurrentY = dStartingY + dCurrentRadius * sin(dAngleRadians);

            // Add the current waypoint to the final vector.
            geoops::UTMCoordinate stCurrentCoordinate(dCurrentX, dCurrentY, stStartingPointUTM.nZone, stStartingPointUTM.bWithinNorthernHemisphere);
            geoops::Waypoint stCurrentWaypoint(stCurrentCoordinate, geoops::WaypointType::eNavigationWaypoint);
            stWaypoints.push_back(stCurrentWaypoint);

            // Increment angle and radius for the next waypoint.
            dAngleRadians += dAngularStepRadians;
            dCurrentRadius += dSpacing;
        }

        return stWaypoints;
    }
}    // namespace searchpattern
#endif
