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
     * @return vWaypoints - A vector representing the waypoints forming the spiral
     *      search pattern.
     *
     * @author Jacob V (jpvf2d@umsystem.edu)
     * @date 2024-02-04
     ******************************************************************************/
    inline std::vector<geoops::Waypoint> CalculateSpiralPatternWaypoints(const geoops::UTMCoordinate& stStartingPoint,
                                                                         const double dAngularStepDegrees     = 57,
                                                                         const double dMaxRadius              = 25,
                                                                         const double dStartingHeadingDegrees = 0,
                                                                         const double dSpacing                = 1)
    {
        // Define variables.
        std::vector<geoops::Waypoint> vWaypoints;
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
            geoops::UTMCoordinate stCurrentCoordinate = stStartingPoint;
            stCurrentCoordinate.dEasting              = dCurrentX;
            stCurrentCoordinate.dNorthing             = dCurrentY;
            geoops::Waypoint stCurrentWaypoint(stCurrentCoordinate, geoops::WaypointType::eNavigationWaypoint);
            vWaypoints.push_back(stCurrentWaypoint);

            // Increment angle and radius for the next waypoint.
            dAngleRadians += dAngularStepRadians;
            dCurrentRadius += dSpacing;
        }

        return vWaypoints;
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
     * @return vWaypoints - A vector representing the waypoints forming the spiral
     *      search pattern.
     *
     * @author Jacob V (jpvf2d@umsystem.edu)
     * @date 2024-02-04
     ******************************************************************************/
    inline std::vector<geoops::Waypoint> CalculateSpiralPatternWaypoints(const geoops::GPSCoordinate& stStartingPoint,
                                                                         const double dAngularStepDegrees     = 57,
                                                                         const double dMaxRadius              = 25,
                                                                         const double dStartingHeadingDegrees = 0,
                                                                         const double dSpacing                = 1)
    {
        // Define variables.
        std::vector<geoops::Waypoint> vWaypoints;
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
            geoops::UTMCoordinate stCurrentCoordinate = stStartingPointUTM;
            stCurrentCoordinate.dEasting              = dCurrentX;
            stCurrentCoordinate.dNorthing             = dCurrentY;
            geoops::Waypoint stCurrentWaypoint(stCurrentCoordinate, geoops::WaypointType::eNavigationWaypoint);
            vWaypoints.push_back(stCurrentWaypoint);

            // Increment angle and radius for the next waypoint.
            dAngleRadians += dAngularStepRadians;
            dCurrentRadius += dSpacing;
        }

        return vWaypoints;
    }

    /******************************************************************************
     * @brief Calculate waypoints for a zigzag pattern.
     *      This function generates waypoints for a zigzag pattern starting from a given
     *      point with configurable width, height, and spacing. The direction of the
     *      zigzag pattern (vertical or horizontal) can be specified.
     *
     * @param stCenterPoint - The center point of the zigzag pattern.
     * @param dWidth - The width of the zigzag pattern in meters.
     * @param dHeight - The height of the zigzag pattern in meters.
     * @param dSpacing - The spacing between successive points in the zigzag pattern in meters.
     * @param bVertical - Indicates whether or not the zigzag pattern is vertical or horizontal.
     * @return std::vector<geoops::Waypoint> - A vector representing the waypoints forming the zigzag
     *      search pattern.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-04-01
     ******************************************************************************/
    inline std::vector<geoops::Waypoint> CalculateZigZagPatternWaypoints(const geoops::UTMCoordinate& stCenterPoint,
                                                                         const double dWidth   = 20.0,
                                                                         const double dHeight  = 20.0,
                                                                         const double dSpacing = 1.0,
                                                                         const bool bVertical  = true)
    {
        // Create instance variables.
        std::vector<geoops::Waypoint> vWaypoints;
        double dStartingX = stCenterPoint.dEasting - (dWidth / 2);
        double dStartingY = stCenterPoint.dNorthing - (dHeight / 2);
        double dCurrentX  = dStartingX;
        double dCurrentY  = dStartingY;
        bool bZigNotZag   = true;

        // Loop until covered entire space of width and height.
        while ((bVertical && dCurrentY <= dStartingY + dHeight) || (!bVertical && dCurrentX <= dStartingX + dWidth))
        {
            // Check if pattern should be vertical or horizontal.
            if (bVertical)
            {
                // Check step direction.
                if (bZigNotZag)
                {
                    // Zig.
                    dCurrentX = dStartingX + (dWidth / 2);
                }
                else
                {
                    // Zag.
                    dCurrentX = dStartingX - (dWidth / 2);
                }
            }
            else
            {
                // Check step direction.
                if (bZigNotZag)
                {
                    // Zig.
                    dCurrentY = dStartingY + (dHeight / 2);
                }
                else
                {
                    // Zag.
                    dCurrentY = dStartingY - (dHeight / 2);
                }
            }

            // Loop and add points along line until we reached the limit or width or height.
            while ((bZigNotZag && bVertical && dCurrentX <= dStartingX + dWidth) || (!bZigNotZag && bVertical && dCurrentX >= dStartingX) ||
                   (bZigNotZag && !bVertical && dCurrentY <= dStartingY + dHeight) || (!bZigNotZag && !bVertical && dCurrentY >= dStartingY))
            {
                // Construct UTMCoordinate.
                geoops::UTMCoordinate stCurrentCoordinate = stCenterPoint;
                stCurrentCoordinate.dEasting              = dCurrentX;
                stCurrentCoordinate.dNorthing             = dCurrentY;
                geoops::Waypoint stCurrentWaypoint(stCurrentCoordinate, geoops::WaypointType::eNavigationWaypoint);
                // Add current waypoint to final path.
                vWaypoints.push_back(stCurrentWaypoint);

                // Move to the next point based on spacing and direction.
                if (bVertical)
                {
                    // Increment current position.
                    dCurrentX += bZigNotZag ? dSpacing : -dSpacing;
                }
                else
                {
                    // Increment current position.
                    dCurrentY += bZigNotZag ? dSpacing : -dSpacing;
                }
            }

            // Now shift the opposite coordinate forward.
            if (bVertical)
            {
                dCurrentY += dSpacing;
            }
            else
            {
                dCurrentX += dSpacing;
            }

            // Toggle zigzag direction.
            bZigNotZag = !bZigNotZag;
        }

        // Return the final path.
        return vWaypoints;
    }

    /******************************************************************************
     * @brief Calculate waypoints for a zigzag pattern.
     *      This function generates waypoints for a zigzag pattern starting from a given
     *      point with configurable width, height, and spacing. The direction of the
     *      zigzag pattern (vertical or horizontal) can be specified.
     *
     * @param stCenterPoint - The center point of the zigzag pattern.
     * @param dWidth - The width of the zigzag pattern in meters.
     * @param dHeight - The height of the zigzag pattern in meters.
     * @param dSpacing - The spacing between successive points in the zigzag pattern in meters.
     * @param bVertical - Indicates whether or not the zigzag pattern is vertical or horizontal.
     * @return std::vector<geoops::Waypoint> - A vector representing the waypoints forming the zigzag
     *      search pattern.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-04-01
     ******************************************************************************/
    inline std::vector<geoops::Waypoint> CalculateZigZagPatternWaypoints(const geoops::GPSCoordinate& stCenterPoint,
                                                                         const double dWidth   = 20.0,
                                                                         const double dHeight  = 20.0,
                                                                         const double dSpacing = 1.0,
                                                                         const bool bVertical  = true)
    {
        // Create instance variables.
        std::vector<geoops::Waypoint> vWaypoints;
        geoops::UTMCoordinate stCenterPointUTM = geoops::ConvertGPSToUTM(stCenterPoint);
        double dStartingX                      = stCenterPointUTM.dEasting - (dWidth / 2);
        double dStartingY                      = stCenterPointUTM.dNorthing - (dHeight / 2);
        double dCurrentX                       = dStartingX;
        double dCurrentY                       = dStartingY;
        bool bZigNotZag                        = true;

        // Loop until covered entire space of width and height.
        while ((bVertical && dCurrentY <= dStartingY + dHeight) || (!bVertical && dCurrentX <= dStartingX + dWidth))
        {
            // Check if pattern should be vertical or horizontal.
            if (bVertical)
            {
                // Check step direction.
                if (bZigNotZag)
                {
                    // Zig.
                    dCurrentX = dStartingX + dSpacing;
                }
                else
                {
                    // Zag.
                    dCurrentX = dStartingX - dSpacing;
                }
            }
            else
            {
                // Check step direction.
                if (bZigNotZag)
                {
                    // Zig.
                    dCurrentY = dStartingY + dSpacing;
                }
                else
                {
                    // Zag.
                    dCurrentY = dStartingY - dSpacing;
                }
            }

            // Loop and add points along line until we reached the limit or width or height.
            while ((bZigNotZag && bVertical && dCurrentX <= dStartingX + dWidth) || (!bZigNotZag && bVertical && dCurrentX >= dStartingX) ||
                   (bZigNotZag && !bVertical && dCurrentY <= dStartingY + dHeight) || (!bZigNotZag && !bVertical && dCurrentY >= dStartingY))
            {
                // Construct UTMCoordinate.
                geoops::UTMCoordinate stCurrentCoordinate = stCenterPointUTM;
                stCurrentCoordinate.dEasting              = dCurrentX;
                stCurrentCoordinate.dNorthing             = dCurrentY;
                geoops::Waypoint stCurrentWaypoint(stCurrentCoordinate, geoops::WaypointType::eNavigationWaypoint);
                // Add current waypoint to final path.
                vWaypoints.push_back(stCurrentWaypoint);

                // Move to the next point based on spacing and direction.
                if (bVertical)
                {
                    // Increment current position.
                    dCurrentX += bZigNotZag ? dSpacing : -dSpacing;
                }
                else
                {
                    // Increment current position.
                    dCurrentY += bZigNotZag ? dSpacing : -dSpacing;
                }
            }

            // Now shift the opposite coordinate forward.
            if (bVertical)
            {
                dCurrentY += dSpacing;
            }
            else
            {
                dCurrentX += dSpacing;
            }

            // Toggle zigzag direction.
            bZigNotZag = !bZigNotZag;
        }

        // Return the final path.
        return vWaypoints;
    }
}    // namespace searchpattern
#endif
