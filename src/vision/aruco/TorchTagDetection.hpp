/******************************************************************************
 * @brief This file contains the tagdetectutils::ArucoTagDetection class which is used to detect
 *      and store information about tags using a PyTorch model.
 *
 * @file tagdetectutils::ArucoTagDetection.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-13
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef TORCH_TAG_DETECTION_HPP
#define TORCH_TAG_DETECTION_HPP

#include "../../util/vision/TagDetectionUtilty.hpp"
#include "../../util/vision/YOLOModel.hpp"

/// \cond
#include <chrono>
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
     * @brief Detect ArUco tags in the provided image using a YOLO DNN model.
     *
     * @param cvFrame - The RGB camera frame to run detection on.
     * @param tfPyTorchDetector - The PyTorch model interpreter to run inference on.
     * @param fMinObjectConfidence - The minimum confidence required for an object to be considered a valid detection.
     * @param fNMSThreshold - The threshold for Non-Maximum Suppression, controlling overlap between bounding box predictions.
     * @return std::vector<tagdetectutils::ArucoTag> - The resultant vector containing the detected tags in the frame.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-13
     ******************************************************************************/
    inline std::vector<tagdetectutils::ArucoTag> Detect(const cv::Mat& cvFrame,
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
        std::vector<tagdetectutils::ArucoTag> vDetectedTags;

        // Check if the PyTorch interpreter hardware is opened and the model is loaded.
        if (tfPyTorchDetector.IsReadyForInference())
        {
            // Run inference on YOLO model with current image.
            std::vector<yolomodel::Detection> vOutputTensorTags = tfPyTorchDetector.Inference(cvFrame, fMinObjectConfidence, fNMSThreshold);

            // Repackage detections into tensorflow tags.
            for (const yolomodel::Detection& stTagDetection : vOutputTensorTags)
            {
                // Create and initialize new TensorflowTag.
                tagdetectutils::ArucoTag stDetectedTag;
                stDetectedTag.dConfidence      = stTagDetection.fConfidence;
                stDetectedTag.pBoundingBox     = std::make_shared<cv::Rect2d>(stTagDetection.cvBoundingBox);
                stDetectedTag.nID              = stTagDetection.nClassID;
                stDetectedTag.szClassName      = stTagDetection.szClassName;
                stDetectedTag.eDetectionMethod = tagdetectutils::TagDetectionMethod::eTorch;

                // Add the newly detected tag to the vector.
                vDetectedTags.emplace_back(stDetectedTag);
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
     * @brief Given a vector of tagdetectutils::ArucoTag structs draw each tag corner and confidence onto the given image.
     *
     * @param cvDetectionsFrame - The frame to draw overlay onto.
     * @param vDetectedTags - The vector of tagdetectutils::ArucoTag structs used to draw tag corners and confidences onto image.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-13
     ******************************************************************************/
    inline void DrawDetections(cv::Mat& cvDetectionsFrame, const std::vector<tagdetectutils::ArucoTag>& vDetectedTags)
    {
        // Check if the given frame is a 1 or 3 channel image. (not BGRA)
        if (!cvDetectionsFrame.empty() && (cvDetectionsFrame.channels() == 1 || cvDetectionsFrame.channels() == 3))
        {
            // Loop through each detection.
            for (const tagdetectutils::ArucoTag& stTag : vDetectedTags)
            {
                // Check if the tag detection type is Torch.
                if (stTag.eDetectionMethod == tagdetectutils::TagDetectionMethod::eTorch)
                {
                    // Draw bounding box onto image.
                    cv::rectangle(cvDetectionsFrame, *stTag.pBoundingBox, cv::Scalar(255, 255, 255), 2);
                    std::string szText  = stTag.szClassName + " " + std::to_string(static_cast<int>(stTag.dConfidence * 100)) + "%";
                    cv::Size cvTextSize = cv::getTextSize(szText, cv::FONT_HERSHEY_SIMPLEX, 0.75, 1, nullptr);
                    // Draw classID background box onto image.
                    cv::rectangle(cvDetectionsFrame,
                                  cv::Point(stTag.pBoundingBox->x, stTag.pBoundingBox->y - 20),
                                  cv::Point((*stTag.pBoundingBox).tl() + cv::Point2d(cvTextSize.width, cvTextSize.height)),
                                  cv::Scalar(255, 255, 255),
                                  cv::FILLED);
                    // Draw class text onto image.
                    cv::putText(cvDetectionsFrame,
                                szText,
                                cv::Point(stTag.pBoundingBox->x, stTag.pBoundingBox->y - 5),
                                cv::FONT_HERSHEY_SIMPLEX,
                                0.5,
                                cv::Scalar(0, 0, 0));
                }
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
}    // namespace torchtag

#endif
