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

    struct Object
    {
        public:
            // Declare public struct member attributes.
            std::shared_ptr<cv::Rect2d> pBoundingBox         = std::make_shared<cv::Rect2d>();    // The bounding box of the detected object.
            double dConfidence                               = 0.0;    // The detection confidence of the object (from Torch/Tensorflow models).
            double dStraightLineDistance                     = 0.0;    // Distance between the object and the camera.
            double dYawAngle                                 = 0.0;    // This is the yaw angle so roll and pitch are ignored.
            std::string szClassName                          = "";     // The class name of the object (used in Torch/Tensorflow models).
            std::chrono::system_clock::time_point tmCreation = std::chrono::system_clock::now();    // Set the time detected to the minimum time point.
            ObjectDetectionMethod eDetectionMethod           = ObjectDetectionMethod::eUnknown;     // The detection method used to detect the object.
            cv::Size cvImageResolution                       = cv::Size(0, 0);                      // The resolution of the image used to detect the object.
            double dHorizontalFOV                            = 0.0;                   // The horizontal field of view of the camera used to detect the object.
            geoops::Waypoint stGeolocatedPosition            = geoops::Waypoint();    // The geolocated position of the object.

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
                       eDetectionMethod == stOther.eDetectionMethod && cvImageResolution == stOther.cvImageResolution && dHorizontalFOV == stOther.dHorizontalFOV &&
                       stGeolocatedPosition == stOther.stGeolocatedPosition;
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
                    cvImageResolution     = stOther.cvImageResolution;
                    dHorizontalFOV        = stOther.dHorizontalFOV;
                    stGeolocatedPosition  = stOther.stGeolocatedPosition;
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
}    // namespace objectdetectutils

#endif
