/******************************************************************************
 * @brief Defines and implements functions related to geolocation of objects in camera
 *      coordinate frames relative to the rover's global position.
 *
 * @file Geolocate.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-27
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef GEOLOCATE_HPP
#define GEOLOCATE_HPP

#include "../../AutonomyLogging.h"
#include "../GeospatialOperations.hpp"
#include "../NumberOperations.hpp"

/// \cond
#include <algorithm>
#include <cmath>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief Namespace containing functions related to geolocation of objects within
 *      camera frames and conversion to global coordinate systems.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-27
 ******************************************************************************/
namespace geoloc
{
    /******************************************************************************
     * @brief Converts pixel coordinates in a ZED2i pointcloud into global UTM coordinates,
     *      given the rover's current pose and a point cloud from the camera.
     *
     * @param cvPointcloud - The CV_32FC4 pointcloud from the ZED2i camera.
     * @param stRoverPose - The current rover pose containing UTM position and heading.
     * @param cvPixel - The pixel coordinates (x, y) of the object of interest.
     * @param nNeighborhoodSize - Optional, the size of the neighborhood to sample
     *                           (must be odd and >= 1, defaults to 5).
     * @return geoops::Waypoint - A waypoint containing the object's UTM coordinates and
     *                          estimated radius.
     *
     * @note This function assumes the ZED SDK coordinate system is LEFT_HANDED_Y_UP.
     *       Heading is measured clockwise from north (0 degrees).
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-04-27
     ******************************************************************************/
    inline geoops::Waypoint GeolocateBox(const cv::Mat& cvPointcloud, const geoops::RoverPose& stRoverPose, const cv::Point& cvPixel, int nNeighborhoodSize = 5)
    {
        // Ensure neighborhood size is at least 1.
        if (nNeighborhoodSize < 1)
        {
            LOG_WARNING(logging::g_qSharedLogger, "GeolocateBox: Invalid neighborhood size {}, defaulting to 5x5", nNeighborhoodSize);
            nNeighborhoodSize = 5;    // Default to 5x5 neighborhood if invalid.
        }

        // Get the half-width of neighborhood for window calculations.
        int nHalfSize = nNeighborhoodSize / 2;

        // Create containers for valid points in the neighborhood.
        std::vector<float> vX, vY, vZ, vDistances;

        // Loop through the neighborhood window.
        for (int nOffsetY = -nHalfSize; nOffsetY <= nHalfSize; ++nOffsetY)
        {
            for (int nOffsetX = -nHalfSize; nOffsetX <= nHalfSize; ++nOffsetX)
            {
                // Calculate the current pixel coordinates in the neighborhood.
                int nCurrY = cvPixel.y + nOffsetY;
                int nCurrX = cvPixel.x + nOffsetX;

                // Skip out-of-bounds pixels.
                if (nCurrY < 0 || nCurrY >= cvPointcloud.rows || nCurrX < 0 || nCurrX >= cvPointcloud.cols)
                {
                    continue;
                }

                // Access the 3D point at this pixel.
                cv::Vec4f cvPoint = cvPointcloud.at<cv::Vec4f>(nCurrY, nCurrX);

                // Check if the point is valid. (not zero or NaN)
                if (cvPoint[2] > 0 && std::isfinite(cvPoint[0]) && std::isfinite(cvPoint[1]) && std::isfinite(cvPoint[2]))
                {
                    // Add point coordinates to our vectors
                    vX.push_back(cvPoint[0]);    // X coordinate in camera frame.
                    vY.push_back(cvPoint[1]);    // Y coordinate in camera frame.
                    vZ.push_back(cvPoint[2]);    // Z coordinate in camera frame.
                }
            }
        }

        // If no valid points found in the neighborhood.
        if (vX.empty())
        {
            LOG_DEBUG(logging::g_qSharedLogger, "GeolocateBox: No valid points found in neighborhood around pixel ({}, {})", cvPixel.x, cvPixel.y);
            // Return a waypoint with default values.
            return geoops::Waypoint();
        }

        // Calculate the centroid of the valid points in camera coordinates.
        float fAvgX = std::accumulate(vX.begin(), vX.end(), 0.0f) / vX.size();
        float fAvgY = std::accumulate(vY.begin(), vY.end(), 0.0f) / vY.size();
        float fAvgZ = std::accumulate(vZ.begin(), vZ.end(), 0.0f) / vZ.size();

        // Adjust rover degree heading to match unit circle 0 position.
        double dAdjustedHeading = numops::InputAngleModulus(stRoverPose.GetCompassHeading() + 90.0, 0.0, 359.9);
        // Convert camera heading to radians. (0 = North, CW positive)
        double dHeadingRad = dAdjustedHeading * M_PI / 180.0;

        // Get the rover's current UTM position.
        const geoops::UTMCoordinate& stRoverUTM = stRoverPose.GetUTMCoordinate();

        // Transform camera coordinates to global UTM coordinates.
        // NOTE: ZED uses left-handed Y-up coordinate system, so:
        // Camera +X = Right = East when rover faces North
        // Camera +Y = Up = not used for horizontal positioning
        // Camera +Z = Forward = North when rover faces North

        // Rotate by rover's heading and translate by rover's position.
        double dEasting  = stRoverUTM.dEasting + (fAvgZ * cos(dHeadingRad) + fAvgX * sin(dHeadingRad));
        double dNorthing = stRoverUTM.dNorthing + (fAvgZ * sin(dHeadingRad) - fAvgX * cos(dHeadingRad));

        // Calculate altitude by adding camera height to rover's altitude.
        double dAltitude = stRoverUTM.dAltitude + fAvgY;    // Y is up in camera frame.

        // Create a UTM coordinate for the object.
        geoops::UTMCoordinate stObjectUTM(dEasting, dNorthing, stRoverUTM.nZone, stRoverUTM.bWithinNorthernHemisphere, dAltitude);

        // Estimate the object's radius from the spread of points.
        double dRadius = 0.0;
        if (vX.size() > 1)
        {
            // Calculate Euclidean distance from each point to the centroid.
            for (size_t siI = 0; siI < vX.size(); ++siI)
            {
                double dX = vX[siI] - fAvgX;
                double dY = vY[siI] - fAvgY;
                double dZ = vZ[siI] - fAvgZ;
                vDistances.push_back(sqrt(dX * dX + dY * dY + dZ * dZ));
            }

            // Use median distance as radius estimate to be robust against outliers.
            std::sort(vDistances.begin(), vDistances.end());
            if (vDistances.size() % 2 == 0)
            {
                dRadius = (vDistances[vDistances.size() / 2 - 1] + vDistances[vDistances.size() / 2]) / 2.0;
            }
            else
            {
                dRadius = vDistances[vDistances.size() / 2];
            }

            // Apply a scaling factor to better approximate the object's boundary.
            dRadius *= 1.5;    // Empirical scale factor.
        }

        // Create and return the waypoint with the object's UTM coordinates and radius.
        return geoops::Waypoint(stObjectUTM, geoops::WaypointType::eUNKNOWN, dRadius);
    }
}    // namespace geoloc

#endif    // GEOLOCATE_HPP
