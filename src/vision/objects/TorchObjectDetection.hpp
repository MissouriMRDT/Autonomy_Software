/******************************************************************************
 * @brief This file contains the implementation of the TorchObjectDetection class, which is used to detect
 *     and store information about objects using a PyTorch model.
 *
 * @file TorchObjectDetection.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef TORCH_OBJECT_DETECTION_HPP
#define TORCH_OBJECT_DETECTION_HPP

#include "../../util/vision/ObjectDetectionUtility.hpp"
#include "../../util/vision/YOLOModel.hpp"

/// \cond
#include <chrono>

/// \endcond

/******************************************************************************
 * @brief Namespace containing functions related to torch object detections
 *    operations on images using PyTorch.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
namespace torchobject
{
    /******************************************************************************
     * @brief Detects objects in the given image using a PyTorch model.
     *
     * @param cvFrame - The image to detect objects in.
     * @param trPyTorchDetector - The PyTorch model to use for detection.
     * @param fMinObjectConfidence - The minimum confidence threshold for detected objects.
     * @param fNMSThreshold - The non-maximum suppression threshold for detected objects.
     * @return std::vector<objectdetectutils::Object> - A vector of detected objects.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-05-09
     ******************************************************************************/
    inline std::vector<objectdetectutils::Object> Detect(const cv::Mat& cvFrame,
                                                         yolomodel::pytorch::PyTorchInterpreter& trPyTorchDetector,
                                                         const float fMinObjectConfidence = 0.40f,
                                                         const float fNMSThreshold        = 0.60f)
    {
        ZoneScopedC(tracy::Color::Blue);
        // Check if the input frame is in RGB format.
        if (cvFrame.channels() != 3)
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "Detect() requires a RGB image.");
            return {};
        }

        // Declare instance variables.
        std::vector<objectdetectutils::Object> vDetectedTags;

        // Check if the PyTorch interpreter hardware is opened and the model is loaded.
        if (trPyTorchDetector.IsReadyForInference())
        {
            // Run inference on YOLO model with current image.
            std::vector<yolomodel::Detection> vOutputTensorTags = trPyTorchDetector.Inference(cvFrame, fMinObjectConfidence, fNMSThreshold);

            // Repackage detections into objects.
            for (const yolomodel::Detection& stTagDetection : vOutputTensorTags)
            {
                // Create and initialize new Object.
                objectdetectutils::Object stDetectedTag;
                stDetectedTag.dConfidence       = stTagDetection.fConfidence;
                stDetectedTag.pBoundingBox      = std::make_shared<cv::Rect2d>(stTagDetection.cvBoundingBox);
                stDetectedTag.szClassName       = stTagDetection.szClassName;
                stDetectedTag.eDetectionMethod  = objectdetectutils::ObjectDetectionMethod::eTorch;
                stDetectedTag.cvImageResolution = cvFrame.size();

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
     * @brief Given a vector of objectdetectutils::Object structs draw each tag corner and confidence onto the given image.
     *
     * @param cvDetectionsFrame - The frame to draw overlay onto.
     * @param vDetectedTags - The vector of objectdetectutils::Object structs used to draw tag corners and confidences onto image.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-13
     ******************************************************************************/
    inline void DrawDetections(cv::Mat& cvDetectionsFrame, const std::vector<objectdetectutils::Object>& vDetectedTags)
    {
        ZoneScopedC(tracy::Color::Blue);
        // Check if the given frame is a 1 or 3 channel image. (not BGRA)
        if (!cvDetectionsFrame.empty() && (cvDetectionsFrame.channels() == 1 || cvDetectionsFrame.channels() == 3))
        {
            // Loop through each detection.
            for (const objectdetectutils::Object& stTag : vDetectedTags)
            {
                // Check if the tag detection type is Torch.
                if (stTag.eDetectionMethod == objectdetectutils::ObjectDetectionMethod::eTorch)
                {
                    // Draw bounding box onto image.
                    cv::rectangle(cvDetectionsFrame, *stTag.pBoundingBox, cv::Scalar(255, 255, 255), 2);
                    std::string szText  = stTag.szClassName + " " + std::to_string(static_cast<int>(stTag.dConfidence * 100)) + "%";
                    cv::Size cvTextSize = cv::getTextSize(szText, cv::FONT_HERSHEY_SIMPLEX, 0.75, 1, nullptr);
                    // Draw classID background box onto image.
                    cv::rectangle(cvDetectionsFrame,
                                  stTag.pBoundingBox->tl() + cv::Point2d(0, stTag.pBoundingBox->height),
                                  stTag.pBoundingBox->tl() + cv::Point2d(cvTextSize.width, stTag.pBoundingBox->height + cvTextSize.height),
                                  cv::Scalar(255, 255, 255),
                                  cv::FILLED);
                    // Draw class text onto image.
                    cv::putText(cvDetectionsFrame,
                                szText,
                                stTag.pBoundingBox->tl() + cv::Point2d(0, stTag.pBoundingBox->height + cvTextSize.height),
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
}    // namespace torchobject

#endif
