/******************************************************************************
 * @brief Implements functions related to path post-processing.
 *
 * @file PathPostProcessing.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-08
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef PATH_POST_PROCESSING_H
#define PATH_POST_PROCESSING_H

#include "../../util/GeospatialOperations.hpp"

/// \cond
#include <vector>

/// \endcond

/******************************************************************************
 * @brief This namespace stores classes, functions, and structs that are used to
 *      implement different path planner algorithms used by the rover to determine
 *      the optimal path to take for any given situation.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-08
 ******************************************************************************/
namespace pathplanners
{
    /******************************************************************************
     * @brief This namespace defines and implements structs and functions used by
     *     various different algorithms to post-process paths.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-05-08
     ******************************************************************************/
    namespace postprocessing
    {
        /******************************************************************************
         * @brief Fits a B-spline to the given path using cubic interpolation.
         *
         * @param vRawPath - The raw path to fit.
         * @return std::vector<geoops::Waypoint> - The fitted path.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-05-08
         ******************************************************************************/
        std::vector<geoops::Waypoint> FitPathWithBSpline(const std::vector<geoops::Waypoint>& vRawPath)
        {
            // Create instance variables.
            int nNumPoints = static_cast<int>(vRawPath.size()) / 8;

            // Check if the path is empty or if the number of points is too small.
            if (vRawPath.size() <= 3)
            {
                return vRawPath;
            }

            // Extract dX and dY coordinates.
            std::vector<double> vX, vY;
            for (const geoops::Waypoint& stWaypoint : vRawPath)
            {
                vX.push_back(stWaypoint.GetUTMCoordinate().dEasting);
                vY.push_back(stWaypoint.GetUTMCoordinate().dNorthing);
            }

            // Create parameter values for each point. (cumulative distance)
            std::vector<double> vT(vRawPath.size());
            vT[0] = 0.0;
            for (size_t siI = 1; siI < vRawPath.size(); siI++)
            {
                double dDist = std::sqrt(std::pow(vX[siI] - vX[siI - 1], 2) + std::pow(vY[siI] - vY[siI - 1], 2));
                vT[siI]      = vT[siI - 1] + dDist;
            }

            // Normalize parameter values to [0, 1].
            double dMaxT = vT.back();
            for (auto& dT1 : vT)
            {
                dT1 /= dMaxT;
            }

            // Fit cubic spline.
            std::vector<geoops::Waypoint> vSplinePath;

            // Keep the first point.
            vSplinePath.push_back(vRawPath.front());

            // Generate evenly spaced points along the spline.
            for (int siI = 1; siI < nNumPoints - 1; siI++)
            {
                double dT = static_cast<double>(siI) / (nNumPoints - 1);

                // Find the spline siSegment this parameter belongs to.
                size_t siSegment = 0;
                while (siSegment < vT.size() - 1 && vT[siSegment + 1] < dT)
                {
                    siSegment++;
                }

                // Compute local parameter.
                double dLocalT = (dT - vT[siSegment]) / (vT[siSegment + 1] - vT[siSegment]);

                // Calculate indices for cubic interpolation.
                int nI0 = std::max(0, static_cast<int>(siSegment) - 1);
                int nI1 = siSegment;
                int nI2 = std::min(static_cast<int>(vRawPath.size()) - 1, static_cast<int>(siSegment) + 1);
                int nI3 = std::min(static_cast<int>(vRawPath.size()) - 1, static_cast<int>(siSegment) + 2);

                // Catmull-Rom spline interpolation.
                double dT1  = dLocalT;
                double dT2  = dT1 * dT1;
                double dT3  = dT2 * dT1;

                double dH00 = 2 * dT3 - 3 * dT2 + 1;
                double dH10 = dT3 - 2 * dT2 + dT1;
                double dH01 = -2 * dT3 + 3 * dT2;
                double dH11 = dT3 - dT2;

                double dX   = dH00 * vX[nI1] + dH10 * (vX[nI2] - vX[nI0]) + dH01 * vX[nI2] + dH11 * (vX[nI3] - vX[nI1]);
                double dY   = dH00 * vY[nI1] + dH10 * (vY[nI2] - vY[nI0]) + dH01 * vY[nI2] + dH11 * (vY[nI3] - vY[nI1]);

                // Create waypoint.
                geoops::UTMCoordinate stUTMResult(dX, dY, vRawPath[0].GetUTMCoordinate().nZone, vRawPath[0].GetUTMCoordinate().bWithinNorthernHemisphere);
                vSplinePath.push_back(geoops::Waypoint(stUTMResult));
            }

            // Keep the last point.
            vSplinePath.push_back(vRawPath.back());

            return vSplinePath;
        }

        /******************************************************************************
         * @brief Fits a B-spline to the given path using cubic interpolation.
         *
         * @param vRawPath - The raw path to fit.
         * @return std::vector<geoops::Waypoint> - The fitted path.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-05-08
         ******************************************************************************/
        std::vector<geoops::Waypoint> FitPathWithBSpline(const std::vector<geoops::UTMCoordinate>& vRawPath)
        {
            // Convert UTM coordinates to Waypoints.
            std::vector<geoops::Waypoint> vWaypoints;
            for (const geoops::UTMCoordinate& stUTMCoordinate : vRawPath)
            {
                vWaypoints.emplace_back(geoops::Waypoint(stUTMCoordinate));
            }

            // Call the other function to fit the path.
            return FitPathWithBSpline(vWaypoints);
        }

        /******************************************************************************
         * @brief Fits a B-spline to the given path using cubic interpolation.
         *
         * @param vRawPath - The raw path to fit.
         * @return std::vector<geoops::Waypoint> - The fitted path.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-05-08
         ******************************************************************************/
        std::vector<geoops::Waypoint> FitPathWithBSpline(const std::vector<geoops::GPSCoordinate>& vRawPath)
        {
            // Convert UTM coordinates to Waypoints.
            std::vector<geoops::Waypoint> vWaypoints;
            for (const geoops::GPSCoordinate& stUTMCoordinate : vRawPath)
            {
                vWaypoints.emplace_back(geoops::Waypoint(stUTMCoordinate));
            }

            // Call the other function to fit the path.
            return FitPathWithBSpline(vWaypoints);
        }
    }    // namespace postprocessing
}    // namespace pathplanners

#endif
