/******************************************************************************
 * @brief Defines and implements functions related to tensorflow tag operations on images. All
 *      functions are defined within the tensorflowtag namespace.
 *
 * @file TensorflowDetection.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef TENSORFLOW_DETECTION_H
#define TENSORFLOW_DETECTION_H

#include <vector>

#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>

#include "../../AutonomyConstants.h"
#include "../../AutonomyLogging.h"

/******************************************************************************
 * @brief Namespace containing functions related to tensorflow tag detections
 *      operations on images.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
namespace tensorflowtag
{
    /******************************************************************************
     * @brief Represents a single ArUco tag. Stores all information about a specific
     *      tag detection.
     *
     *
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-09-28
     ******************************************************************************/
    struct TensorflowTag
    {
        public:
            // Declare public struct member attributes.
            cv::Point2f CornerTL;                                                                 // The top left corner of the bounding box.
            cv::Point2f CornerTR;                                                                 // The top right corner of the bounding box.
            cv::Point2f CornerBL;                                                                 // The bottom left corner of the bounding box.
            cv::Point2f CornerBR;                                                                 // The bottom right corner of bounding box.
            std::vector<cv::Point2f*> vCorners = {&CornerTL, &CornerTR, &CornerBL, &CornerBR};    // Provide an easy method for getting all corners.
            int nID;                                                                              // ID of the tag.
            int nHits;                                                                            // Total number of detections for tag id.
            int nFramesSinceLastHit;         // The total number of frames since a tag with this ID was last detected.
            double dConfidence;              // The detection confidence of the tag reported from the tensorflow model.
            double dStraightLineDistance;    // Distance between the tag and the camera.
            double dYawAngle;                // This is the yaw angle so roll and pitch are ignored.
    };

    /******************************************************************************
     * @brief Given an ArucoTag struct find the center point of the corners.
     *
     * @param stTag - The tag to find the center of.
     * @return cv::Point2f - The resultant center point within the image.
     *
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-10-07
     ******************************************************************************/
    inline cv::Point2f FindTagCenter(const TensorflowTag& stTag)
    {
        // Average of the four corners
        cv::Point2f cvCenter(0, 0);

        // Loop through each corner of the tag.
        for (cv::Point2f* cvCorner : stTag.vCorners)
        {
            // Add each tag x, y to the center x, y.
            cvCenter.x += cvCorner->x;
            cvCenter.y += cvCorner->y;
        }
        // Divide by number of corners.
        cvCenter.x /= 4;
        cvCenter.y /= 4;

        // Return a copy of the center point of the tag.
        return cvCenter;
    }

    /******************************************************************************
     * @brief detect ArUco tags in the provided image
     *
     * @param cvFrame - The camera frame to run ArUco detection on.
     * @return std::vector<TensorflowTag> - The resultant vector containing the detected tags in the frame.
     *
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-09-28
     ******************************************************************************/
    inline std::vector<TensorflowTag> Detect(const cv::Mat& cvFrame)
    {
        // TODO: Write different util classes to easily open and inference new models. Then put tag detection specific inferencing here.
        cvFrame.empty();
        return std::vector<TensorflowTag>();
    }
}    // namespace tensorflowtag

#endif
