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
     * @param dStartSpacing - The spacing between successive points in the spiral
     *      (meters). This will become larger as the spiral grows.
     * @return vWaypoints - A vector representing the waypoints forming the spiral
     *      search pattern.
     *
     * @author Jacob V (jpvf2d@umsystem.edu)
     * @date 2024-02-04
     ******************************************************************************/
    inline std::vector<geoops::Waypoint> CalculateSpiralPatternWaypoints(const geoops::Waypoint& stStartingPoint,
                                                                         const double dAngularStepDegrees     = 57,
                                                                         const double dMaxRadius              = 25,
                                                                         const double dStartingHeadingDegrees = 0,
                                                                         const double dStartSpacing           = 1)
    {
        // Define variables.
        std::vector<geoops::Waypoint> vWaypoints;
        double dAngularStepRadians   = dAngularStepDegrees * M_PI / 180;
        double dAngleRadians         = (-dStartingHeadingDegrees) * M_PI / 180.0;
        double dCurrentSpacingWindUp = 0.0;
        double dStartingX            = stStartingPoint.GetUTMCoordinate().dEasting;
        double dStartingY            = stStartingPoint.GetUTMCoordinate().dNorthing;
        double dCurrentRadius        = 0.0;

        // Check if the MaxRadius is less than 1 meter. If so return an empty vector.
        if (dMaxRadius < 1)
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "MaxRadius is less than 1 meter. Cannot create spiral pattern.");
            return vWaypoints;
        }

        // Calculate each waypoint. Stop when the radius exceeds the maximum.
        while (dCurrentRadius <= dMaxRadius)
        {
            // Get X and Y positions for the current point.
            double dCurrentX = dStartingX + dCurrentSpacingWindUp * cos(dAngleRadians);
            double dCurrentY = dStartingY + dCurrentSpacingWindUp * sin(dAngleRadians);

            // Add the current waypoint to the final vector.
            geoops::UTMCoordinate stCurrentCoordinate = stStartingPoint.GetUTMCoordinate();
            stCurrentCoordinate.dEasting              = dCurrentX;
            stCurrentCoordinate.dNorthing             = dCurrentY;
            vWaypoints.emplace_back(stCurrentCoordinate, geoops::WaypointType::eNavigationWaypoint);

            // Increment angle and radius for the next waypoint.
            dAngleRadians += dAngularStepRadians;
            dCurrentSpacingWindUp += dStartSpacing;

            // Calculate the current distance from the starting point. This is our radius.
            dCurrentRadius = geoops::CalculateGeoMeasurement(stStartingPoint.GetUTMCoordinate(), stCurrentCoordinate).dDistanceMeters;
        }
        // Same but going back in.
        while (dCurrentRadius >= 0.5)
        {
            // Get X and Y positions for the current point.
            double dCurrentX = dStartingX + dCurrentSpacingWindUp * cos(dAngleRadians);
            double dCurrentY = dStartingY + dCurrentSpacingWindUp * sin(dAngleRadians);

            // Add the current waypoint to the final vector.
            geoops::UTMCoordinate stCurrentCoordinate = stStartingPoint.GetUTMCoordinate();
            stCurrentCoordinate.dEasting              = dCurrentX;
            stCurrentCoordinate.dNorthing             = dCurrentY;
            vWaypoints.emplace_back(stCurrentCoordinate, geoops::WaypointType::eNavigationWaypoint);

            // Increment angle and radius for the next waypoint.
            dAngleRadians += dAngularStepRadians;
            dCurrentSpacingWindUp -= dStartSpacing;

            // Calculate the current distance from the starting point. This is our radius.
            dCurrentRadius = geoops::CalculateGeoMeasurement(stStartingPoint.GetUTMCoordinate(), stCurrentCoordinate).dDistanceMeters;
        }

        // Write the search pattern points to the logger, just store the GPS lat/long.
        std::string szSearchPatternPoints = "Search Pattern Points (Spiral): ";
        for (geoops::Waypoint& stWaypoint : vWaypoints)
        {
            szSearchPatternPoints +=
                "(" + std::to_string(stWaypoint.GetGPSCoordinate().dLatitude) + ", " + std::to_string(stWaypoint.GetGPSCoordinate().dLongitude) + "), ";
        }
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "{}", szSearchPatternPoints);

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
    inline std::vector<geoops::Waypoint> CalculateZigZagPatternWaypoints(const geoops::Waypoint& stCenterPoint,
                                                                         const double dWidth   = 20.0,
                                                                         const double dHeight  = 20.0,
                                                                         const double dSpacing = 1.0,
                                                                         const bool bVertical  = true)
    {
        // Create instance variables.
        std::vector<geoops::Waypoint> vWaypoints;
        double dStartingX   = stCenterPoint.GetUTMCoordinate().dEasting - (dWidth / 2);
        double dStartingY   = stCenterPoint.GetUTMCoordinate().dNorthing - (dHeight / 2);
        double dCurrentX    = dStartingX;
        double dCurrentY    = dStartingY;
        bool bZigNotZag     = true;
        double bCalcSpacing = dSpacing;

        // Check if the width or height is less than 1 meter. If so return an empty vector.
        if (dWidth < 1 || dHeight < 1 || dSpacing < 1)
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "Width or height or spacing is less than 1 meter. Cannot create zigzag pattern.");
            return vWaypoints;
        }

        // Limit spacing to the width or height.
        if (bCalcSpacing > dWidth / 2.0)
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "Spacing is greater than width. Setting spacing to width / 2.");
            // Set spacing to half the width.
            bCalcSpacing = dWidth / 2.0 - 1.0;
        }
        if (bCalcSpacing > dHeight / 2.0)
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "Spacing is greater than height. Setting spacing to height / 2.");
            // Set spacing to half the height.
            bCalcSpacing = dHeight / 2.0 - 1.0;
        }

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
                    dCurrentX = dStartingX + bCalcSpacing;
                }
                else
                {
                    // Zag.
                    dCurrentX = dStartingX - bCalcSpacing;
                }
            }
            else
            {
                // Check step direction.
                if (bZigNotZag)
                {
                    // Zig.
                    dCurrentY = dStartingY + bCalcSpacing;
                }
                else
                {
                    // Zag.
                    dCurrentY = dStartingY - bCalcSpacing;
                }
            }

            // Loop and add points along line until we reached the limit or width or height.
            while ((bZigNotZag && bVertical && dCurrentX <= dStartingX + dWidth) || (!bZigNotZag && bVertical && dCurrentX >= dStartingX) ||
                   (bZigNotZag && !bVertical && dCurrentY <= dStartingY + dHeight) || (!bZigNotZag && !bVertical && dCurrentY >= dStartingY))
            {
                // Construct UTMCoordinate.
                geoops::UTMCoordinate stCurrentCoordinate = stCenterPoint.GetUTMCoordinate();
                stCurrentCoordinate.dEasting              = dCurrentX;
                stCurrentCoordinate.dNorthing             = dCurrentY;
                geoops::Waypoint stCurrentWaypoint(stCurrentCoordinate, geoops::WaypointType::eNavigationWaypoint);
                // Add current waypoint to final path.
                vWaypoints.push_back(stCurrentWaypoint);

                // Move to the next point based on spacing and direction.
                if (bVertical)
                {
                    // Increment current position.
                    dCurrentX += bZigNotZag ? bCalcSpacing : -bCalcSpacing;
                }
                else
                {
                    // Increment current position.
                    dCurrentY += bZigNotZag ? bCalcSpacing : -bCalcSpacing;
                }
            }

            // Now shift the opposite coordinate forward.
            if (bVertical)
            {
                dCurrentY += bCalcSpacing;
            }
            else
            {
                dCurrentX += bCalcSpacing;
            }

            // Toggle zigzag direction.
            bZigNotZag = !bZigNotZag;
        }

        // The path now contains the zigzag pattern with points spaced at the specified distance. This means that there
        // are many points potentially very close to each other. This is not ideal for navigation, so we will filter out
        // points that are too close to each other, by calculating their GeoMeasurement and testing if the distance is greater
        // then the CalcSpacing. If the CalcSpacing is greater a certain threshold, we will remove the point since it jumps to the other side.
        // This will reduce the number of points in the zigzag pattern, making it more efficient for navigation.

        // Create a vector to store the filtered waypoints.
        std::vector<geoops::Waypoint> vFilterWaypoints;
        // Always store the first point.
        vFilterWaypoints.push_back(vWaypoints[0]);
        // Loop through the waypoints and remove any that are too close to each other.
        for (size_t i = 0; i < vWaypoints.size() - 1; ++i)
        {
            // Calculate the GeoMeasurement between the current waypoint and the next waypoint.
            geoops::GeoMeasurement stGeoMeasurement = geoops::CalculateGeoMeasurement(vWaypoints[i].GetGPSCoordinate(), vWaypoints[i + 1].GetGPSCoordinate());
            if (stGeoMeasurement.dDistanceMeters > bCalcSpacing * 1.5)
            {
                // Add the points to the filtered waypoints.
                vFilterWaypoints.push_back(vWaypoints[i]);
                vFilterWaypoints.push_back(vWaypoints[i + 1]);
            }
        }

        // Write the search pattern points to the logger, just store the GPS lat/long.
        std::string szSearchPatternPoints = "Search Pattern Points (Spiral): ";
        for (geoops::Waypoint& stWaypoint : vWaypoints)
        {
            szSearchPatternPoints +=
                "(" + std::to_string(stWaypoint.GetGPSCoordinate().dLatitude) + ", " + std::to_string(stWaypoint.GetGPSCoordinate().dLongitude) + "), ";
        }
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "{}", szSearchPatternPoints);

        // Return the final path.
        return vFilterWaypoints;
    }

    /******************************************************************************
     * @brief Calculate waypoints for a snake search pattern.
     *
     * @param stStartCoord - The starting coordinate of the search pattern.
     * @param dWidth - The width of the search pattern in meters.
     * @param dHeight - The height of the search pattern in meters.
     * @param dSpacing - The spacing between successive points in the search pattern in meters.
     * @param nNumberOfSlithers - The number of slithers in the snake pattern.
     * @param bVertical - Indicates whether the search pattern is vertical or horizontal.
     * @return std::vector<geoops::Waypoint> - A vector representing the waypoints forming the snake
     *
     * @author Vexas5266 (wd.grove2@gmail.com)
     * @date 2025-04-09
     ******************************************************************************/
    inline std::vector<geoops::Waypoint> CalculateSnakeSearchPattern(const geoops::Waypoint& stStartCoord,
                                                                     const double dWidth         = 20.0,
                                                                     const double dHeight        = 20.0,
                                                                     const double dSpacing       = 1.0,
                                                                     const int nNumberOfSlithers = 1.0,
                                                                     const bool bVertical        = true)
    {
        // Create instance variables.
        std::vector<geoops::Waypoint> vWaypoints;
        geoops::UTMCoordinate stStartUTM = stStartCoord.GetUTMCoordinate();
        double dStartingX                = (bVertical) ? stStartUTM.dEasting : stStartUTM.dEasting - (dWidth / 2);
        double dStartingY                = (bVertical) ? stStartUTM.dNorthing - (dHeight / 2) : stStartUTM.dNorthing;

        // Check if the dimensions are valid.
        if (dWidth < 1.0 || dHeight < 1.0 || dSpacing < 0.1 || nNumberOfSlithers < 1)
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger,
                        "Invalid parameters for snake pattern: width={}, height={}, spacing={}, number of slithers={}",
                        dWidth,
                        dHeight,
                        dSpacing,
                        nNumberOfSlithers);
            return vWaypoints;
        }

        // Calculate the number of points based on path length and spacing and the number of slithers.
        // For a sine wave, we need enough points to make it smooth.
        int nPoints = static_cast<int>(dWidth + dHeight / dSpacing);

        // Generate the sine wave pattern.
        for (int nIter = 0; nIter < nPoints; ++nIter)
        {
            double dTime = static_cast<double>(nIter) / (nPoints - 1);
            double dCurrentX, dCurrentY;

            if (bVertical)
            {
                // Vertical primary movement: sinusoidal variation in x.
                dCurrentY = dStartingY + dTime * dWidth;
                // Sine wave with multiple periods.
                dCurrentX = dStartingX + (dHeight / 2.0) * std::cos(2.0 * nNumberOfSlithers * M_PI * dTime);
            }
            else
            {
                // Horizontal primary movement: sinusoidal variation in y.
                dCurrentX = dStartingX + dTime * dWidth;
                // Sine wave with multiple periods.
                dCurrentY = dStartingY + (dHeight / 2.0) * std::cos(2.0 * nNumberOfSlithers * M_PI * dTime);
            }

            // Create waypoint at current position.
            geoops::UTMCoordinate stCurrentUTM = stStartUTM;
            stCurrentUTM.dEasting              = dCurrentX;
            stCurrentUTM.dNorthing             = dCurrentY;
            geoops::Waypoint stCurrentWaypoint(stCurrentUTM, geoops::WaypointType::eNavigationWaypoint);
            vWaypoints.push_back(stCurrentWaypoint);
        }

        // Log the waypoints.
        std::string szSearchPatternPoints = "Search Pattern Points (Snake): ";
        for (const geoops::Waypoint& stWaypoint : vWaypoints)
        {
            szSearchPatternPoints +=
                "(" + std::to_string(stWaypoint.GetGPSCoordinate().dLatitude) + ", " + std::to_string(stWaypoint.GetGPSCoordinate().dLongitude) + "), ";
        }
        LOG_DEBUG(logging::g_qSharedLogger, "{}", szSearchPatternPoints);

        return vWaypoints;
    }

}    // namespace searchpattern
#endif
