#ifndef OBSTACLE_DETECTION_UTILITY_HPP
#define OBSTACLE_DETECTION_UTILITY_HPP

#include "../../AutonomyConstants.h"
#include "../../AutonomyLogging.h"
#include "../GeospatialOperations.hpp"

/// \cond
#include <opencv2/opencv.hpp>
#include <torch/torch.h>

/// \endcond

namespace obstacledetectutils
{
    struct Obstacle
    {
        public:
            std::shared_ptr<cv::Rect2d> pBoundingBox         = std::make_shared<cv::Rect2d>();
            std::shared_ptr<cv::Mat> pSegmentMask            = std::make_shared<cv::Mat>();
            double dConfidence                               = 0.0;
            double dStraightLineDistance                     = 0.0;
            double dYawAngle                                 = 0.0;
            int nID                                          = -1;
            std::chrono::system_clock::time_point tmCreation = std::chrono::system_clock::now();
            cv::Size cvImageResolution                       = cv::Size(0, 0);
            double dHorizontalFOV                            = 0.0;
            geoops::Waypoint stGeolocatedPosition            = geoops::Waypoint();

            bool operator==(const Obstacle& stOther) const
            {
                return *pBoundingBox == *stOther.pBoundingBox && dConfidence == stOther.dConfidence && dStraightLineDistance == stOther.dStraightLineDistance &&
                       dYawAngle == stOther.dYawAngle && nID == stOther.nID && tmCreation == stOther.tmCreation && cvImageResolution == stOther.cvImageResolution &&
                       dHorizontalFOV == stOther.dHorizontalFOV && stGeolocatedPosition == stOther.stGeolocatedPosition && pSegmentMask == stOther.pSegmentMask;
            }

            bool operator!=(const Obstacle& stOther) const { return !(*this == stOther); }

            Obstacle& operator=(const Obstacle& stOther)
            {
                if (this != &stOther)
                {
                    // Shallow copy
                    pBoundingBox          = stOther.pBoundingBox;
                    pSegmentMask          = stOther.pSegmentMask;
                    dConfidence           = stOther.dConfidence;
                    dStraightLineDistance = stOther.dStraightLineDistance;
                    dYawAngle             = stOther.dYawAngle;
                    nID                   = stOther.nID;
                    tmCreation            = stOther.tmCreation;
                    cvImageResolution     = stOther.cvImageResolution;
                    dHorizontalFOV        = stOther.dHorizontalFOV;
                    stGeolocatedPosition  = stOther.stGeolocatedPosition;
                }
                return *this;
            }
    };

    inline void EstimatePoseFromCameraFrame(Obstacle& stObstacle)
    {
        // Use camera field of view and camera frame size to determine tag angle in degrees from center of camera.
        double dDegreesPerPixel = stObstacle.dHorizontalFOV / stObstacle.cvImageResolution.width;
        // Find tag error in pixels from center of image.
        double dTagErrorX = (stObstacle.pBoundingBox->x + stObstacle.pBoundingBox->width / 2) - (stObstacle.cvImageResolution.width / 2);
        // Find angle error.
        double dTagAngleX = dTagErrorX * dDegreesPerPixel;
        // Reassign yaw and distance to tag.
        stObstacle.dYawAngle = dTagAngleX;

        // For the distance, we'll just use the screen percentage of the tag.
        stObstacle.dStraightLineDistance = (stObstacle.pBoundingBox->area() / (stObstacle.cvImageResolution.width * stObstacle.cvImageResolution.height)) * 100.0;
    }
}    // namespace obstacledetectutils

#endif
