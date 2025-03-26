/******************************************************************************
 * @brief This file contains the TorchTagDetection class which is used to detect
 *      and store information about tags using a PyTorch model.
 *
 * @file TorchTagDetection.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-13
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef TORCH_TAG_DETECTION_HPP
#define TORCH_TAG_DETECTION_HPP

#include "../../util/vision/YOLOModel.hpp"

/// \cond
#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <torch/torch.h>

/// \endcond

/******************************************************************************
 * @brief Namespace containing functions related to torch tag detections
 *      operations on images using PyTorch.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-13
 ******************************************************************************/
namespace torchtag
{
    /******************************************************************************
     * @brief Represents a single ArUco tag. Stores all information about a specific
     *      tag detection.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-13
     ******************************************************************************/
    struct TorchTag
    {
        public:
            // Declare public struct member attributes.
            cv::Rect2d cvBoundingBox;              // The bounding box of the detected tag.
            double dConfidence           = 0.0;    // The detection confidence of the tag reported from the PyTorch model.
            double dStraightLineDistance = 0.0;    // Distance between the tag and the camera.
            double dYawAngle             = 0.0;    // This is the yaw angle so roll and pitch are ignored.
            int nID                      = -1;     // The ID of the tag. This is set to -1 if the tag is not detected.
            std::string szClassName;               // The class name of the tag. This is dependent on the class names used when training.

            /******************************************************************************
             * @brief Overload the equality operator for the TorchTag struct.
             *
             * @param stOther - The other TorchTag struct to compare to.
             * @return true - The two TorchTag structs are equal.
             * @return false - The two TorchTag structs are not equal
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-03-24
             ******************************************************************************/
            bool operator==(const TorchTag& stOther) const
            {
                return cvBoundingBox == stOther.cvBoundingBox && dConfidence == stOther.dConfidence && dStraightLineDistance == stOther.dStraightLineDistance &&
                       dYawAngle == stOther.dYawAngle && nID == stOther.nID && szClassName == stOther.szClassName;
            }

            /******************************************************************************
             * @brief Overload the inequality operator for the TorchTag struct.
             *
             * @param stOther - The other TorchTag struct to compare to.
             * @return true - The two TorchTag structs are not equal.
             * @return false - The two TorchTag structs are equal
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-03-24
             ******************************************************************************/
            bool operator!=(const TorchTag& stOther) const { return !(*this == stOther); }
    };

    /******************************************************************************
     * @brief Given an TorchTag struct find the center point of the corners.
     *
     * @param stTag - The tag to find the center of.
     * @return cv::Point2f - The resultant center point within the image.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-13
     ******************************************************************************/
    inline cv::Point2f FindTagCenter(const TorchTag& stTag)
    {
        // Calculate the center point of the tag.
        cv::Point2f cvCenter = cv::Point2f(stTag.cvBoundingBox.x + stTag.cvBoundingBox.width / 2, stTag.cvBoundingBox.y + stTag.cvBoundingBox.height / 2);

        return cvCenter;
    }

    /******************************************************************************
     * @brief Detect ArUco tags in the provided image using a YOLO DNN model.
     *
     * @param cvFrame - The RGB camera frame to run detection on.
     * @param tfPyTorchDetector - The PyTorch model interpreter to run inference on.
     * @param fMinObjectConfidence - The minimum confidence required for an object to be considered a valid detection.
     * @param fNMSThreshold - The threshold for Non-Maximum Suppression, controlling overlap between bounding box predictions.
     * @return std::vector<TorchTag> - The resultant vector containing the detected tags in the frame.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-13
     ******************************************************************************/
    inline std::vector<TorchTag> Detect(const cv::Mat& cvFrame,
                                        yolomodel::pytorch::PyTorchInterpreter& tfPyTorchDetector,
                                        const float fMinObjectConfidence = 0.40f,
                                        const float fNMSThreshold        = 0.60f)
    {
        // Check if the input frame is in RGB format.
        if (cvFrame.channels() != 3)
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "Detect() requires a RGB image.");
            return {};
        }

        // Declare instance variables.
        std::vector<TorchTag> vDetectedTags;

        // Check if the PyTorch interpreter hardware is opened and the model is loaded.
        if (tfPyTorchDetector.IsReadyForInference())
        {
            // Run inference on YOLO model with current image.
            std::vector<yolomodel::Detection> vOutputTensorTags = tfPyTorchDetector.Inference(cvFrame, fMinObjectConfidence, fNMSThreshold);

            // Repackage detections into tensorflow tags.
            for (yolomodel::Detection stTagDetection : vOutputTensorTags)
            {
                // Create and initialize new TensorflowTag.
                TorchTag stDetectedTag;
                stDetectedTag.dConfidence   = stTagDetection.fConfidence;
                stDetectedTag.cvBoundingBox = stTagDetection.cvBoundingBox;
                stDetectedTag.nID           = stTagDetection.nClassID;
                stDetectedTag.szClassName   = stTagDetection.szClassName;

                // Add the newly detected tag to the vector.
                vDetectedTags.push_back(stDetectedTag);
            }
        }
        else
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger,
                        "TorchDetect: Unable to detect tags using YOLO torch detection because hardware is not opened or model is not initialized.");
        }

        // Return the detected tags.
        return vDetectedTags;
    }

    /******************************************************************************
     * @brief Given a vector of TorchTag structs draw each tag corner and confidence onto the given image.
     *
     * @param cvDetectionsFrame - The frame to draw overlay onto.
     * @param vDetectedTags - The vector of TorchTag structs used to draw tag corners and confidences onto image.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-13
     ******************************************************************************/
    inline void DrawDetections(cv::Mat& cvDetectionsFrame, const std::vector<TorchTag>& vDetectedTags)
    {
        // Check if the given frame is a 1 or 3 channel image. (not BGRA)
        if (!cvDetectionsFrame.empty() && (cvDetectionsFrame.channels() == 1 || cvDetectionsFrame.channels() == 3))
        {
            // Loop through each detection.
            for (TorchTag stTag : vDetectedTags)
            {
                // Draw bounding box onto image.
                cv::rectangle(cvDetectionsFrame, stTag.cvBoundingBox, cv::Scalar(255, 255, 255), 2);
                // Draw classID background box onto image.
                cv::rectangle(cvDetectionsFrame,
                              cv::Point(stTag.cvBoundingBox.x, stTag.cvBoundingBox.y - 20),
                              cv::Point(stTag.cvBoundingBox.x + stTag.cvBoundingBox.width, stTag.cvBoundingBox.y),
                              cv::Scalar(255, 255, 255),
                              cv::FILLED);
                // Draw class text onto image.
                cv::putText(cvDetectionsFrame,
                            stTag.szClassName + " " + std::to_string(static_cast<int>(stTag.dConfidence * 100)),
                            cv::Point(stTag.cvBoundingBox.x, stTag.cvBoundingBox.y - 5),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.5,
                            cv::Scalar(0, 0, 0));
            }
        }
        else
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger,
                      "TorchDetect: Unable to draw markers on image because it is empty or because it has {} channels. (Should be 1 or 3)",
                      cvDetectionsFrame.channels());
        }
    }

    /******************************************************************************
     * @brief Given a TorchTag struct find the center point of the corners.
     *
     * @param cvPointCloud - A point cloud image to estimate the pose of the tag.
     * @param stTag - The tag to estimate the pose of.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-13
     ******************************************************************************/
    inline void EstimatePoseFromPointCloud(const cv::Mat& cvPointCloud, TorchTag& stTag)
    {
        // Confirm correct coordinate system.
        if (constants::ZED_COORD_SYSTEM != sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP)
        {
            // Submit logger message.
            LOG_CRITICAL(logging::g_qSharedLogger, "TensorflowDetection: Calculations won't work for anything other than ZED coordinate system == LEFT_HANDED_Y_UP");
        }

        // Find the center point of the given tag.
        cv::Point2f cvCenter = FindTagCenter(stTag);

        // Ensure the detected center is inside the domain of the point cloud.
        if (cvCenter.y > cvPointCloud.rows || cvCenter.x > cvPointCloud.cols || cvCenter.y < 0 || cvCenter.x < 0)
        {
            LOG_ERROR(logging::g_qSharedLogger,
                      "Detected tag center ({}, {}) out of point cloud's domain ({},{})",
                      cvCenter.y,
                      cvCenter.x,
                      cvPointCloud.rows,
                      cvPointCloud.cols);
            return;
        }

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
}    // namespace torchtag

#endif
