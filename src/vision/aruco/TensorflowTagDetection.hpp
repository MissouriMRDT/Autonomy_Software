/******************************************************************************
 * @brief Defines and implements functions related to tensorflow tag operations on images. All
 *      functions are defined within the tensorflowtag namespace.
 *
 * @file TensorflowDetection.h
 * @author clayjay3 (claytonraycowen@gmail.com), jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-10-07
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef TENSORFLOW_TAG_DETECTION_HPP
#define TENSORFLOW_TAG_DETECTION_HPP

#include "../../AutonomyConstants.h"
#include "../../AutonomyLogging.h"
#include "../../util/vision/YOLOModel.hpp"

/// \cond
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

/// \endcond

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
            cv::Point2f CornerTL;                                                                       // The top left corner of the bounding box.
            cv::Point2f CornerTR;                                                                       // The top right corner of the bounding box.
            cv::Point2f CornerBL;                                                                       // The bottom left corner of the bounding box.
            cv::Point2f CornerBR;                                                                       // The bottom right corner of bounding box.
            std::vector<const cv::Point2f*> vCorners = {&CornerTL, &CornerTR, &CornerBL, &CornerBR};    // Provide an easy method for getting all corners.
            int nHits;                                                                                  // Total number of detections for tag id.
            int nFramesSinceLastHit;         // The total number of frames since a tag with this ID was last detected.
            double dConfidence;              // The detection confidence of the tag reported from the tensorflow model.
            double dStraightLineDistance;    // Distance between the tag and the camera.
            double dYawAngle;                // This is the yaw angle so roll and pitch are ignored.
    };

    /******************************************************************************
     * @brief Given an TensorflowTag struct find the center point of the corners.
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
        for (const cv::Point2f* cvCorner : stTag.vCorners)
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
     * @brief Detect ArUco tags in the provided image using a YOLO DNN model.
     *
     * @param cvFrame - The camera frame to run tensorflow detection on. Should be RGB format.
     * @param tfTensorflowDetector - The configured tensorflow detector to use for detection.
     * @param fMinObjectConfidence - Minimum confidence required for an object to be considered a valid detection
     * @param fNMSThreshold - Threshold for Non-Maximum Suppression, controlling overlap between bounding box predictions.
     * @return std::vector<TensorflowTag> - The resultant vector containing the detected tags in the frame.
     *
     * @note The given cvFrame SHOULD BE IN RGB FORMAT.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-28
     ******************************************************************************/
    inline std::vector<TensorflowTag> Detect(const cv::Mat& cvFrame,
                                             yolomodel::tensorflow::TPUInterpreter& tfTensorflowDetector,
                                             const float fMinObjectConfidence = 0.40f,
                                             const float fNMSThreshold        = 0.60f)
    {
        // Declare instance variables.
        std::vector<TensorflowTag> vDetectedTags;

        // Check if the tensorflow TPU interpreter hardware is opened and the model is loaded.
        if (tfTensorflowDetector.GetDeviceIsOpened())
        {
            // Run inference on YOLO model with current image.
            std::vector<std::vector<yolomodel::Detection>> vOutputTensorTags = tfTensorflowDetector.Inference(cvFrame, fMinObjectConfidence, fNMSThreshold);

            // Repackage detections into tensorflow tags.
            for (std::vector<yolomodel::Detection> vTagDetections : vOutputTensorTags)
            {
                // Loop through each detection.
                for (yolomodel::Detection stTagDetection : vTagDetections)
                {
                    // Create and initialize new TensorflowTag.
                    TensorflowTag stDetectedTag;
                    stDetectedTag.dConfidence = stTagDetection.fConfidence;
                    stDetectedTag.CornerTL    = cv::Point2f(stTagDetection.cvBoundingBox.x, stTagDetection.cvBoundingBox.y);
                    stDetectedTag.CornerTR    = cv::Point2f(stTagDetection.cvBoundingBox.x + stTagDetection.cvBoundingBox.width, stTagDetection.cvBoundingBox.y);
                    stDetectedTag.CornerBL    = cv::Point2f(stTagDetection.cvBoundingBox.x, stTagDetection.cvBoundingBox.y + stTagDetection.cvBoundingBox.height);
                    stDetectedTag.CornerBR    = cv::Point2f(stTagDetection.cvBoundingBox.x + stTagDetection.cvBoundingBox.width,
                                                         stTagDetection.cvBoundingBox.y + stTagDetection.cvBoundingBox.height);

                    // Add TensorflowTag to return vector.
                    vDetectedTags.emplace_back(stDetectedTag);
                }
            }
        }
        else
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger,
                        "TensorflowDetect: Unable to detect tags using YOLO tensorflow detection because hardware is not opened or model is not initialized.");
        }

        // Return the detected tags.
        return vDetectedTags;
    }

    /******************************************************************************
     * @brief Given a vector of TensorflowTag structs draw each tag corner and confidence onto the given image.
     *
     * @param cvDetectionsFrame - The frame to draw overlay onto.
     * @param vDetectedTags - The vector of TensorflowTag structs used to draw tag corners and confidences onto image.
     *
     * @note Image must be a 1 or 3 channel image and image must match dimensions of image when used for
     *      detection of the given tags.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-03-31
     ******************************************************************************/
    inline void DrawDetections(cv::Mat& cvDetectionsFrame, const std::vector<TensorflowTag>& vDetectedTags)
    {
        // Check if the given frame is a 1 or 3 channel image. (not BGRA)
        if (!cvDetectionsFrame.empty() && (cvDetectionsFrame.channels() == 1 || cvDetectionsFrame.channels() == 3))
        {
            // Loop through each detection.
            for (TensorflowTag stTag : vDetectedTags)
            {
                // Draw bounding box onto image.
                cv::rectangle(cvDetectionsFrame, stTag.CornerTL, stTag.CornerBR, cv::Scalar(255, 255, 255), 2);
                // Draw classID background box onto image.
                cv::rectangle(cvDetectionsFrame,
                              cv::Point(stTag.CornerTL.x, stTag.CornerTL.y - 20),
                              cv::Point(stTag.CornerTR.x, stTag.CornerTL.y),
                              cv::Scalar(255, 255, 255),
                              cv::FILLED);
                // Draw class text onto image.
                cv::putText(cvDetectionsFrame,
                            "Tag Conf: " + std::to_string(stTag.dConfidence),
                            cv::Point(stTag.CornerTL.x, stTag.CornerTL.y - 5),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.5,
                            cv::Scalar(0, 0, 0));
            }
        }
        else
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger,
                      "TensorflowDetect: Unable to draw markers on image because it is empty or because it has {} channels. (Should be 1 or 3)",
                      cvDetectionsFrame.channels());
        }
    }

    /******************************************************************************
     * @brief Estimate the pose of a position with respect to the observer using a point cloud
     *
     * @param cvPointCloud - A point cloud of x,y,z coordinates.
     * @param stTag - The tag we are estimating the pose of and then storing the distance and angle calculations in.
     *
     * @note The angle only takes into account how far forward/backward and left/right the tag is with respect to the rover. This meaning I ignore the up/down position of
     * the tag when calculating the angle.
     *
     * @author jspencerpittman (jspencerpittman@gmail.com), clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-04-01
     ******************************************************************************/
    inline void EstimatePoseFromPointCloud(const cv::Mat& cvPointCloud, TensorflowTag& stTag)
    {
        // Confirm correct coordinate system.
        if (constants::ZED_COORD_SYSTEM != sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP)
        {
            // Submit logger message.
            LOG_CRITICAL(logging::g_qSharedLogger, "TensorflowDetection: Calculations won't work for anything other than ZED coordinate system == LEFT_HANDED_Y_UP");
        }

        // Find the center point of the given tag.
        cv::Point2f cvCenter = FindTagCenter(stTag);

        // Get tag center point location relative to the camera. Point cloud location stores float x, y, z, BGRA.
        cv::Vec4f cvCoordinate = cvPointCloud.at<cv::Vec4f>(cvCenter.y, cvCenter.x);
        float fForward         = cvCoordinate[2];    // Z
        float fRight           = cvCoordinate[0];    // X
        float fUp              = cvCoordinate[1];    // Y

        // Calculate euclidean distance from ZED camera left eye to the point of interest
        stTag.dStraightLineDistance = sqrt(pow(fForward, 2) + pow(fRight, 2) + pow(fUp, 2));

        // Calculate the angle on plane horizontal to the viewpoint
        stTag.dYawAngle = atan2(fRight, fForward);
    }
}    // namespace tensorflowtag

#endif
