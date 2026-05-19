/******************************************************************************
 * @brief Defines and implements functions related to geolocation of objects in camera
 * coordinate frames relative to the rover's global position.
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
 * camera frames and conversion to global coordinate systems.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-27
 ******************************************************************************/
namespace geoloc
{
    /******************************************************************************
     * @brief Converts pixel coordinates in a ZED2i pointcloud into global UTM coordinates,
     * given the camera's current pose and a point cloud from the camera.
     *
     * @param cvPointcloud - The CV_32FC4 pointcloud from the ZED2i camera.
     * @param stCameraPose - The current camera pose containing UTM position, altitude, and heading.
     * @param cvPixel - The pixel coordinates (x, y) of the object of interest.
     * @param nNeighborhoodSize - Optional, the size of the neighborhood to sample
     * (must be odd and >= 1, defaults to 5).
     * @param fDepthPercentileTarget - Optional, the percentile of Z depth to target to isolate the object face (defaults to 0.20f).
     * @param fDepthToleranceMeters - Optional, the tolerance around the target depth to include points (defaults to 0.5f).
     * @return geoops::Waypoint - A waypoint containing the object's UTM coordinates and
     * estimated radius.
     *
     * @note This function assumes the ZED SDK coordinate system is LEFT_HANDED_Y_UP.
     * Heading is measured clockwise from north (0 degrees).
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-04-27
     ******************************************************************************/
    inline geoops::Waypoint GeolocateBox(const cv::Mat& cvPointcloud,
                                         const geoops::RoverPose& stCameraPose,
                                         const cv::Point& cvPixel,
                                         int nNeighborhoodSize        = 5,
                                         float fDepthPercentileTarget = 0.20f,
                                         float fDepthToleranceMeters  = 0.5f)
    {
        // Ensure neighborhood size is at least 1.
        if (nNeighborhoodSize < 1)
        {
            LOG_WARNING(logging::g_qSharedLogger, "GeolocateBox: Invalid neighborhood size {}, defaulting to 5x5", nNeighborhoodSize);
            nNeighborhoodSize = 5;    // Default to 5x5 neighborhood if invalid.
        }

        // Get the camera's current UTM position.
        const geoops::UTMCoordinate& stCameraUTM = stCameraPose.GetUTMCoordinate();

        // Get the half-width of neighborhood for window calculations.
        int nHalfSize = nNeighborhoodSize / 2;

        // Create containers for raw valid points in the neighborhood.
        std::vector<cv::Vec4f> vRawPoints;
        std::vector<float> vRawZ;

        // Loop through the neighborhood window to extract initial valid points.
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
                    vRawPoints.push_back(cvPoint);
                    vRawZ.push_back(cvPoint[2]);
                }
            }
        }

        // Containers for the average point calculation.
        float fAvgX = 0.0f;
        float fAvgY = 0.0f;
        float fAvgZ = 0.0f;

        // Final container for the filtered target points (used for radius calculation later)
        std::vector<float> vX, vY, vZ, vDistances;

        // Handle edge case: No valid points found (Stereovision failure)
        if (vRawZ.empty())
        {
            LOG_DEBUG(logging::g_qSharedLogger, "GeolocateBox: Stereovision failed for pixel ({}, {}). Attempting Monocular Fallback.", cvPixel.x, cvPixel.y);

            // MONOCULAR GROUND PLANE FALLBACK.
            // Calculate the geometric intersection of the pixel with the ground plane.
            int nBottomY = std::min(cvPointcloud.rows - 1, cvPixel.y + nHalfSize);

            // Assume typical pinhole focal properties based on image dimensions
            float fFy = cvPointcloud.rows / 2.0f;
            float fCy = cvPointcloud.rows / 2.0f;

            // Extract camera height off the ground from the current UTM altitude offset
            float fCameraHeight = static_cast<float>(stCameraUTM.dAltitude);

            float fRayAngleY    = atan2(nBottomY - fCy, fFy);
            float fTotalAngle   = fRayAngleY;    // Assuming level physical pitch since it's accounted for in the pose.

            // If the total angle is too small, the point is likely on or above the horizon, which is unreliable for geolocation.
            if (fTotalAngle <= 0.01f)
            {
                LOG_DEBUG(logging::g_qSharedLogger, "GeolocateBox: Fallback failed. Object is on or above the horizon line.");
                return geoops::Waypoint();    // Total failure.
            }

            fAvgZ = fCameraHeight / tan(fTotalAngle);
            fAvgX = ((cvPixel.x - (cvPointcloud.cols / 2.0f)) / fFy) * fAvgZ;
            fAvgY = -fCameraHeight;    // ZED Y is UP, ground is negative height.
        }
        else
        {
            // ROBUST STATISTICAL DEPTH EXTRACTION.
            // Sort Z values to find the requested percentile to isolate the target object.
            std::sort(vRawZ.begin(), vRawZ.end());
            size_t unPercentileIndex = static_cast<size_t>(vRawZ.size() * fDepthPercentileTarget);
            float fTargetZ           = vRawZ[unPercentileIndex];

            // Calculate the centroid using ONLY points around the target Z depth.
            for (const auto& cvPoint : vRawPoints)
            {
                if (std::abs(cvPoint[2] - fTargetZ) <= fDepthToleranceMeters)
                {
                    vX.push_back(cvPoint[0]);
                    vY.push_back(cvPoint[1]);
                    vZ.push_back(cvPoint[2]);
                }
            }

            if (!vX.empty())
            {
                fAvgX = std::accumulate(vX.begin(), vX.end(), 0.0f) / vX.size();
                fAvgY = std::accumulate(vY.begin(), vY.end(), 0.0f) / vY.size();
                fAvgZ = std::accumulate(vZ.begin(), vZ.end(), 0.0f) / vZ.size();
            }
            else
            {
                // Edge case catch if tolerance strictly filtered everything (rare).
                fAvgX = vRawPoints[unPercentileIndex][0];
                fAvgY = vRawPoints[unPercentileIndex][1];
                fAvgZ = vRawPoints[unPercentileIndex][2];
            }
        }

        // Adjust camera degree heading to match unit circle 0 position.
        double dAdjustedHeading = numops::InputAngleModulus((stCameraPose.GetCompassHeading() * -1.0) + 90.0, 0.0, 360.0);
        // Convert camera heading to radians. (0 = North, CW positive)
        double dHeadingRad = dAdjustedHeading * M_PI / 180.0;

        // Transform camera coordinates to global UTM coordinates.
        // NOTE: ZED uses left-handed Y-up coordinate system, so:
        // Camera +X = Right = East when camera faces North
        // Camera +Y = Up = not used for horizontal positioning
        // Camera +Z = Forward = North when camera faces North

        // Rotate by camera's heading and translate by camera's position.
        double dEasting  = stCameraUTM.dEasting + (fAvgZ * cos(dHeadingRad) + fAvgX * sin(dHeadingRad));
        double dNorthing = stCameraUTM.dNorthing + (fAvgZ * sin(dHeadingRad) - fAvgX * cos(dHeadingRad));

        // Calculate object altitude based on the camera's altitude and the object's relative Y coordinate.
        double dAltitude = stCameraUTM.dAltitude + fAvgY;    // Y is up in camera frame.

        // Create a UTM coordinate for the object.
        geoops::UTMCoordinate stObjectUTM(dEasting, dNorthing, stCameraUTM.nZone, stCameraUTM.bWithinNorthernHemisphere, dAltitude);

        // Estimate the object's radius from the spread of target points.
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
