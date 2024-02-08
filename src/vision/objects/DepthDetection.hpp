/******************************************************************************
 * @brief Defines and implements functions related to object detection operations using
 *      the depth measure from a ZED camera. All functions are defined within the depthobject namespace.
 *
 * @file DepthDetection.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef DEPTH_DETECTION_HPP
#define DEPTH_DETECTION_HPP

#include <vector>

#include <opencv2/opencv.hpp>

#include "../../AutonomyConstants.h"
#include "../../AutonomyLogging.h"
#include "../../util/GeospatialOperations.hpp"
#include "../../util/vision/ImageOperations.hpp"

/******************************************************************************
 * @brief Namespace containing functions related to object detection operations
 *      using depth measures and images.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
namespace depthobject
{
    /******************************************************************************
     * @brief Represents a single depth object detection. Stores all information about a specific
     *      object detection.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-21
     ******************************************************************************/
    struct DepthObject
    {
        public:
            // Declare public struct member attributes.
            cv::Point2f CornerTL;                                                                 // The top left corner of the bounding box.
            cv::Point2f CornerTR;                                                                 // The top right corner of the bounding box.
            cv::Point2f CornerBL;                                                                 // The bottom left corner of the bounding box.
            cv::Point2f CornerBR;                                                                 // The bottom right corner of bounding box.
            std::vector<cv::Point2f*> vCorners = {&CornerTL, &CornerTR, &CornerBL, &CornerBR};    // Provide an easy method for getting all corners.
            geoops::UTMCoordinate stLocation;                                                     // The absolute position of the object stored in a UTM coordinate.
            double dStraightLineDistance;                                                         // Distance between the object and the camera.
            double dYawAngle;                                                                     // This is the yaw angle so roll and pitch are ignored.
    };

    /******************************************************************************
     * @brief Given an DepthObject struct find the center point of the corners.
     *
     * @param stObject - The object to find the center of.
     * @return cv::Point2f - The resultant center point within the image.
     *
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-10-20
     ******************************************************************************/
    inline cv::Point2f FindObjectCenter(const DepthObject& stObject)
    {
        // Average of the four corners.
        cv::Point2f cvCenter(0, 0);

        // Loop through each corner of the object.
        for (cv::Point2f* cvCorner : stObject.vCorners)
        {
            // Add each object x, y to the center x, y.
            cvCenter.x += cvCorner->x;
            cvCenter.y += cvCorner->y;
        }
        // Divide by number of corners.
        cvCenter.x /= 4;
        cvCenter.y /= 4;

        // Return a copy of the center point of the object.
        return cvCenter;
    }

    // TODO: Implement when ready, commented out to suppress warnings.
    // /******************************************************************************
    //  * @brief Detect objects in the provided image using the depth measure image and
    //  *      simple but fast filtering, masking, and blob detection algorithms.
    //  *
    //  * @param cvDepthMeasure - The depth measure image to detect objects in.
    //  * @param fMinDistance - Filter out blobs closer than this distance.
    //  * @param fMaxDistance - Filter out blobs farther than this distance.
    //  * @param fFullnessThresh - The threshold for the area or fullness of the blob.
    //  * @return std::vector<DepthObject> - A vector containing struct that hold the detected object info.
    //  *
    //  * @author clayjay3 (claytonraycowen@gmail.com)
    //  * @date 2023-10-23
    //  ******************************************************************************/
    // inline std::vector<DepthObject> Detect(const cv::Mat& cvDepthMeasure, const float fMinDistance, const float fMaxDistance, const float fFullnessThresh)
    // {
    //     // Put detection code here.
    // }

    // TODO: Implement when ready, commented out to suppress warnings.
    // /******************************************************************************
    //  * @brief Given a vector of DepthObject structs draw each object corner and ID onto the given image.
    //  *      Image must be a 1 or 3 channel image and image must match dimensions of image when used for
    //  *      detection of the given objects.
    //  *
    //  * @param cvDetectionsFrame - The frame to draw overlay onto.
    //  * @param vDetectedObjects - The vector of DepthObject struct used to draw object corners and IDs onto image.
    //  *
    //  * @author clayjay3 (claytonraycowen@gmail.com)
    //  * @date 2023-10-19
    //  ******************************************************************************/
    // inline void DrawDetections(cv::Mat& cvDetectionsFrame, std::vector<DepthObject> vDetectedObjects) {}
}    // namespace depthobject

#endif
