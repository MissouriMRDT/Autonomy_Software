/******************************************************************************
 * @brief Defines and implements functions related to tensorflow tag operations on images. All
 *      functions are defined within the TensorflowObject namespace.
 *
 * @file TensorflowDetection.h
 * @author clayjay3 (claytonraycowen@gmail.com), jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-10-07
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef TENSORFLOW_OBJECT_DETECTION_HPP
#define TENSORFLOW_OBJECT_DETECTION_HPP

#include "../../AutonomyConstants.h"
#include "../../AutonomyLogging.h"
#include "../../util/GeospatialOperations.hpp"

/// \cond
#include <opencv2/opencv.hpp>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief Namespace containing functions related to tensorflow object detection
 *      operations on images.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-23
 ******************************************************************************/
namespace tensorflowobject
{
    /******************************************************************************
     * @brief Represents a single depth object detection from a tensorflow model.
     *      Stores all information about a specific object detection.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-23
     ******************************************************************************/
    struct TensorflowObject
    {
        public:
            // Declare public struct member attributes.
            cv::Point2f CornerTL;                                                                 // The top left corner of the bounding box.
            cv::Point2f CornerTR;                                                                 // The top right corner of the bounding box.
            cv::Point2f CornerBL;                                                                 // The bottom left corner of the bounding box.
            cv::Point2f CornerBR;                                                                 // The bottom right corner of bounding box.
            std::vector<cv::Point2f*> vCorners = {&CornerTL, &CornerTR, &CornerBL, &CornerBR};    // Provide an easy method for getting all corners.
            geoops::UTMCoordinate stLocation;                                                     // The absolute position of the object stored in a UTM coordinate.
            int nID;                                                                              // ID of the tag.
            double dConfidence;              // The detection confidence of the object reported from the tensorflow model.
            double dStraightLineDistance;    // Distance between the object and the camera.
            double dYawAngle;                // This is the yaw angle so roll and pitch are ignored.
    };

    /******************************************************************************
     * @brief Given an TensorflowObject struct find the center point of the corners.
     *
     * @param stObject - The object to find the center of.
     * @return cv::Point2f - The resultant center point within the image.
     *
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-10-07
     ******************************************************************************/
    inline cv::Point2f FindObjectCenter(const TensorflowObject& stObject)
    {
        // Average of the four corners
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

    /******************************************************************************
     * @brief Detect objects in the given image using a tensorflow model.
     *
     * @param cvFrame - The normal BGR or BGRA image from the camera.
     * @param tfNeuralNetwork - The tensorflow model to use for detection.
     * @return std::vector<TensorflowObject> - A vector containing all of the inferenced
     *      objects from the model.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-23
     ******************************************************************************/
    inline std::vector<TensorflowObject> Detect(const cv::Mat& cvFrame)
    {
        // TODO: Write different util classes to easily open and inference new models. Then put object detection specific inferencing here.
        cvFrame.empty();
        return std::vector<TensorflowObject>();
    }
}    // namespace tensorflowobject

#endif
