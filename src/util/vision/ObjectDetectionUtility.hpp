/******************************************************************************
 * @brief Implements function related to object detection. Object detection runs
 *      on multiple cameras, under only one mode of operation: PyTorch. These
 *      functions make it more convenient to aggregate detected objects from all
 *      cameras for PyTorch.
 *
 * @file ObjectDetectionUtility.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef OBJECT_DETECTION_UTILITY_HPP
#define OBJECT_DETECTION_UTILITY_HPP

#include "../../AutonomyConstants.h"
#include "../../AutonomyLogging.h"
#include "../../util/NumberOperations.hpp"
#include "../GeospatialOperations.hpp"

/// \cond
#include <opencv2/opencv.hpp>

/// \endcond

///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * @brief Namespace containing functions to assist in object detection.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
namespace objectdetectutils
{
    /******************************************************************************
     * @brief Enum class to define the different object detection methods available.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-05-05
     ******************************************************************************/
    enum class ObjectDetectionMethod
    {
        eUnknown,    // Unknown detection method.
        eTorch,      // Torch detection using a YOLO model.
    };

    /******************************************************************************
     * @brief Enum class to define the different object detection types available.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-05-13
     ******************************************************************************/
    enum class ObjectDetectionType
    {
        eUnknown,        // Unknown detection type.
        eMallet,         // Detection is a mallet.
        eWaterBottle,    // Detection is a water bottle.
        eRockPick        // Detection is a rock pick.
    };

    /******************************************************************************
     * @brief Represents a single detected object.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-05-09
     ******************************************************************************/
    struct Object
    {
        public:
            // Declare public struct member attributes.
            std::shared_ptr<cv::Rect2d> pBoundingBox         = std::make_shared<cv::Rect2d>();      // The bounding box of the detected object.
            double dConfidence                               = 0.0;                                 // The detection confidence of the object (from Torch models).
            double dStraightLineDistance                     = 0.0;                                 // Distance between the object and the camera.
            double dYawAngle                                 = 0.0;                                 // This is the yaw angle so roll and pitch are ignored.
            std::string szClassName                          = "";                                  // The class name of the object (used in Torch models).
            std::chrono::system_clock::time_point tmCreation = std::chrono::system_clock::now();    // Set the time detected to the minimum time point.
            ObjectDetectionMethod eDetectionMethod           = ObjectDetectionMethod::eUnknown;     // The detection method used to detect the object.
            ObjectDetectionType eDetectionType               = ObjectDetectionType::eUnknown;       // The detection type used to detect the object.
            cv::Size cvImageResolution                       = cv::Size(0, 0);                      // The resolution of the image used to detect the object.
            double dHorizontalFOV                            = 0.0;                   // The horizontal field of view of the camera used to detect the object.
            geoops::Waypoint stGeolocatedPosition            = geoops::Waypoint();    // The geolocated position of the object.
            std::string szDetectorUUID = "";    // The UUID of the detector that detected the object. This is used to associate objects with their detectors.

            /******************************************************************************
             * @brief Overridden operator equals for Object struct.
             *
             * @param stOther - The other Object struct we are comparing to.
             * @return true - The two Objects are equal.
             * @return false - The two Objects are not equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-05-05
             ******************************************************************************/
            bool operator==(const Object& stOther) const
            {
                return *pBoundingBox == *stOther.pBoundingBox && dConfidence == stOther.dConfidence && dStraightLineDistance == stOther.dStraightLineDistance &&
                       dYawAngle == stOther.dYawAngle && szClassName == stOther.szClassName && tmCreation == stOther.tmCreation &&
                       eDetectionMethod == stOther.eDetectionMethod && eDetectionType == stOther.eDetectionType && cvImageResolution == stOther.cvImageResolution &&
                       dHorizontalFOV == stOther.dHorizontalFOV && stGeolocatedPosition == stOther.stGeolocatedPosition && szDetectorUUID == stOther.szDetectorUUID;
            }

            /******************************************************************************
             * @brief Overridden operator equals for Object struct.
             *
             * @param stOther - The other Object struct we are comparing to.
             * @return true - The two Objects are not equal.
             * @return false - The two Objects are equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-05-05
             ******************************************************************************/
            bool operator!=(const Object& stOther) const { return !(*this == stOther); }

            /******************************************************************************
             * @brief Overload the assignment operator for the Object struct to perform a deep copy.
             *
             * @param stOther - The other Object struct to copy from.
             * @return Object& - The current Object struct.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-05-05
             ******************************************************************************/
            Object& operator=(const Object& stOther)
            {
                // Check if the other Object is not the same as this one.
                if (this != &stOther)
                {
                    // Shallow copy the bounding box.
                    pBoundingBox = stOther.pBoundingBox;

                    // Copy other member variables.
                    dConfidence           = stOther.dConfidence;
                    dStraightLineDistance = stOther.dStraightLineDistance;
                    dYawAngle             = stOther.dYawAngle;
                    szClassName           = stOther.szClassName;
                    tmCreation            = stOther.tmCreation;
                    eDetectionMethod      = stOther.eDetectionMethod;
                    eDetectionType        = stOther.eDetectionType;
                    cvImageResolution     = stOther.cvImageResolution;
                    dHorizontalFOV        = stOther.dHorizontalFOV;
                    stGeolocatedPosition  = stOther.stGeolocatedPosition;
                    szDetectorUUID        = stOther.szDetectorUUID;
                }
                return *this;
            }
    };

    /******************************************************************************
     * @brief Find the center of an object.
     *
     * @param stObject - The object to find the center of.
     * @return cv::Point2f - The center of the object.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-05-05
     ******************************************************************************/
    inline cv::Point2f FindObjectCenter(const Object& stObject)
    {
        // Find the center of the object.
        return cv::Point2f(stObject.pBoundingBox->x + stObject.pBoundingBox->width / 2, stObject.pBoundingBox->y + stObject.pBoundingBox->height / 2);
    }

    /******************************************************************************
     * @brief - Estimate the pose of a tag from a camera frame.
     *
     * @param stTag - The tag to estimate the pose of.
     *
     * @note In order for this to be accurate, the camera's horizontal field of view (HFOV) and the camera frame size must be known.
     *
     * @author sam_hajdukiewicz (samanthahajdukiewicz@gmail.com) :3
     * @date 2025-04-04
     ******************************************************************************/
    inline void EstimatePoseFromCameraFrame(Object& stTag)
    {
        // Use camera field of view and camera frame size to determine tag angle in degrees from center of camera.
        double dDegreesPerPixel = stTag.dHorizontalFOV / stTag.cvImageResolution.width;
        // Find tag error in pixels from center of image.
        double dTagErrorX = (stTag.pBoundingBox->x + stTag.pBoundingBox->width / 2) - (stTag.cvImageResolution.width / 2);
        // Find angle error.
        double dTagAngleX = dTagErrorX * dDegreesPerPixel;
        // Reassign yaw and distance to tag.
        stTag.dYawAngle = dTagAngleX;

        // For the distance, we'll just use the screen percentage of the tag.
        stTag.dStraightLineDistance = (stTag.pBoundingBox->area() / (stTag.cvImageResolution.width * stTag.cvImageResolution.height)) * 100.0;
    }

    /******************************************************************************
     * @brief This method processes the ZED LiDAR data to look for obstacles in the camera view.
     *
     * @param cvPointCloud - The ZED point cloud.
     * @param stCurrentPose - The current RoverPose.
     * @param nPointCloudSubsample - 1/subsamples amount of points to look at through ZED.
     * @param dGridCellSize - The size of each grid cell for global points.
     * @param dObstacleVarianceThreshold - How different a point needs to be to be considered an obstacle.
     * @return std::vector<geoops::UTMCoordinate> - A vector of coordinates where obstacles are.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-04-13
     ******************************************************************************/
    inline std::vector<geoops::UTMCoordinate> ExtractObstaclesFromZED(const cv::Mat& cvPointCloud,
                                                                      const geoops::RoverPose& stCurrentPose,
                                                                      const int& nPointCloudSubsample,
                                                                      const double& dGridCellSize,
                                                                      const double& dObstacleVarianceThreshold)
    {
        // Declaring a temporary map to act as a 2.5D elevation grid.
        std::unordered_map<std::string, std::pair<double, double>> umElevationGrid;

        // Storing the raw global coordinates here so we don't have to recalculate the trig later.
        std::vector<geoops::UTMCoordinate> vAllGlobalPoints;
        vAllGlobalPoints.reserve(cvPointCloud.rows * cvPointCloud.cols / (nPointCloudSubsample * nPointCloudSubsample));

        // Calculate heading once.
        double dAdjustedHeading                 = numops::InputAngleModulus((stCurrentPose.GetCompassHeading() * -1.0) + 90.0, 0.0, 360.0);
        double dHeadingRad                      = dAdjustedHeading * M_PI / 180.0;
        const geoops::UTMCoordinate& stRoverUTM = stCurrentPose.GetUTMCoordinate();

        // Loop through the pointcloud points.
        for (int nY = 0; nY < cvPointCloud.rows; nY += nPointCloudSubsample)
        {
            for (int nX = 0; nX < cvPointCloud.cols; nX += nPointCloudSubsample)
            {
                // Initialize a point.
                cv::Vec4f cvPoint = cvPointCloud.at<cv::Vec4f>(nY, nX);

                // Continue to next iteration if invalid point.
                if (std::isnan(cvPoint[2]) || cvPoint[2] <= 0)
                {
                    continue;
                }

                // Setting local XYZ points.
                float fLocalX = cvPoint[0];
                float fLocalY = cvPoint[1];
                float fLocalZ = cvPoint[2];

                // TODO: This does NOT account for if the rover isn't very flat and is on a hill or something else.
                // TODO: so we'll need to add some logic here for that if we want
                // Transform to global frame.
                double dEasting  = stRoverUTM.dEasting + (fLocalZ * cos(dHeadingRad) + fLocalX * sin(dHeadingRad));
                double dNorthing = stRoverUTM.dNorthing + (fLocalZ * sin(dHeadingRad) - fLocalX * cos(dHeadingRad));
                double dAltitude = stRoverUTM.dAltitude + fLocalY;

                // Adding to the global points vector.
                vAllGlobalPoints.emplace_back(dEasting, dNorthing, stRoverUTM.nZone, stRoverUTM.bWithinNorthernHemisphere, dAltitude);

                // Determine which grid bucket this point falls into.
                int nGridX            = static_cast<int>(std::floor(dEasting / dGridCellSize));
                int nGridY            = static_cast<int>(std::floor(dNorthing / dGridCellSize));
                std::string szGridKey = std::to_string(nGridX) + "_" + std::to_string(nGridY);

                // Update the min and max altitude for this grid cell.
                if (umElevationGrid.find(szGridKey) == umElevationGrid.end())
                {
                    umElevationGrid[szGridKey] = {dAltitude, dAltitude};
                }

                else
                {
                    umElevationGrid[szGridKey].first  = std::min(umElevationGrid[szGridKey].first, dAltitude);
                    umElevationGrid[szGridKey].second = std::max(umElevationGrid[szGridKey].second, dAltitude);
                }
            }
        }

        // Declare a vector to store the obstacles.
        std::vector<geoops::UTMCoordinate> vObstacles;

        for (size_t i = 0; i < vAllGlobalPoints.size(); ++i)
        {
            const geoops::UTMCoordinate& stPoint = vAllGlobalPoints[i];

            // Re-calculate the grid key to check the cell's final variance
            int nGridX            = static_cast<int>(std::floor(stPoint.dEasting / dGridCellSize));
            int nGridY            = static_cast<int>(std::floor(stPoint.dNorthing / dGridCellSize));
            std::string szGridKey = std::to_string(nGridX) + "_" + std::to_string(nGridY);

            double dMinAlt        = umElevationGrid[szGridKey].first;
            double dMaxAlt        = umElevationGrid[szGridKey].second;

            // If the height difference in this cell exceeds our threshold, it's an obstacle
            if ((dMaxAlt - dMinAlt) > dObstacleVarianceThreshold)
            {
                vObstacles.push_back(stPoint);
            }
        }

        return vObstacles;
    }
}    // namespace objectdetectutils

#endif
