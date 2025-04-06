/******************************************************************************
 * @brief Defines and implements functions related to ArUco operations on images. All
 *      functions are defined within the arucotag namespace.
 *
 * @file ArucoDetection.h
 * @author jspencerpittman (jspencerpittman@gmail.com), clayjay3 (claytonraycowen@gmail.com), Kai Shafe (kasq5m@umsystem.edu)
 * @date 2023-10-07
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef ARUCO_DETECTION_HPP
#define ARUCO_DETECTION_HPP

#include "../../AutonomyLogging.h"
#include "../../util/vision/ImageOperations.hpp"
#include "../../util/vision/TagDetectionUtilty.hpp"

/// \cond
#include <chrono>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief Namespace containing functions related to ArUco operations on images.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
namespace arucotag
{
    /******************************************************************************
     * @brief Preprocess images for specifically for the Aruco Detect() method.
     *      This method creates a copy of the camera frame so as
     *      to not alter the original in case it's used after the call to this function
     *      completes.
     *
     * @param cvInputFrame - cv::Mat of the image to pre-process. This image should be in BGR format.
     * @param cvOutputFrame - cv::Mat to write the pre-processed image to.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2023-10-10
     ******************************************************************************/
    inline void PreprocessFrame(const cv::Mat& cvInputFrame, cv::Mat& cvOutputFrame)
    {
        // Check if the input frame is in BGR format.
        if (cvInputFrame.channels() != 3)
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "PreprocessFrame() requires a BGR image.");
            return;
        }

        // Grayscale.
        cv::cvtColor(cvInputFrame, cvOutputFrame, cv::COLOR_BGR2GRAY);
        cv::filter2D(cvOutputFrame, cvOutputFrame, -1, constants::ARUCO_EDGE_KERNEL);
        // Reduce number of colors/gradients in the image.
        // imgops::ColorReduce(cvOutputFrame);
        // Denoise (Looks like bilateral filter is req. for ArUco, check speed since docs say it's slow)
        // cv::bilateralFilter(cvInputFrame, cvInputFrame, /*diameter =*/5, /*sigmaColor =*/0.2, /*sigmaSpace =*/3);
        // imgops::CustomBilateralFilter(cvInputFrame, 3, 0.1, 3);
        // Deblur? (Would require determining point spread function that caused the blur)

        // Threshold mask (could use OTSU or TRIANGLE, just a const threshold for now)
        // cv::threshold(cvInputFrame, cvInputFrame, constants::ARUCO_PIXEL_THRESHOLD, constants::ARUCO_PIXEL_THRESHOLD_MAX_VALUE, cv::THRESH_BINARY);

        // Super-Resolution
        // std::string szModelPath = "ESPCN_x3.pb";
        // std::string szModelName = "espcn";
        // dnn_superres::DnnSuperResImpl dnSuperResModel;
        // dnSuperResModel.readModel(/*path*/);
        // dnSuperResModel.setModel(/*path, scale*/);
        // dnSuperResModel.upsample(cvInputFrame, cvOutputFrame);
    }

    /******************************************************************************
     * @brief Detect ArUco tags in the provided image.
     *
     * @param cvFrame - The camera frame to run ArUco detection on. Should be BGR format or grayscale.
     * @param cvArucoDetector - The configured aruco detector to use for detection.
     * @return std::vector<tagdetectutils::ArucoTag> - The resultant vector containing the detected tags in the frame.
     *
     * @note The given cvFrame SHOULD BE IN BGR FORMAT.
     *
     * @author jspencerpittman (jspencerpittman@gmail.com), clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-28
     ******************************************************************************/
    inline std::vector<tagdetectutils::ArucoTag> Detect(const cv::Mat& cvFrame, const cv::aruco::ArucoDetector& cvArucoDetector)
    {
        /// Create instance variables.
        std::vector<int> vIDs;
        std::vector<std::vector<cv::Point2f>> cvMarkerCorners, cvRejectedCandidates;

        // Run Aruco detection algorithm.
        cvArucoDetector.detectMarkers(cvFrame, cvMarkerCorners, vIDs, cvRejectedCandidates);

        // Store all of the detected tags as tagdetectutils::ArucoTag.
        std::vector<tagdetectutils::ArucoTag> vDetectedTags;
        vDetectedTags.reserve(vIDs.size());

        // Loop through each detection and build tag for it.
        for (long unsigned int unIter = 0; unIter < vIDs.size(); unIter++)
        {
            // Create and initialize new tag.
            tagdetectutils::ArucoTag stDetectedTag;
            stDetectedTag.nID = vIDs[unIter];
            // Copy corners.
            stDetectedTag.pBoundingBox      = std::make_shared<cv::Rect2d>(cv::boundingRect(cvMarkerCorners[unIter]));
            stDetectedTag.eDetectionMethod  = tagdetectutils::TagDetectionMethod::eOpenCV;
            stDetectedTag.szClassName       = "OpenCVTag";
            stDetectedTag.cvImageResolution = cvFrame.size();

            if (stDetectedTag.pBoundingBox->area() < constants::ARUCO_BBOX_MIN_AREA)
            {
                continue;
            }

            // Add new tag to detected tags vector.
            vDetectedTags.push_back(stDetectedTag);
        }

        // Return the detected tags.
        return vDetectedTags;
    }

    /******************************************************************************
     * @brief Given a vector of tagdetectutils::ArucoTag structs draw each tag corner and ID onto the given image.
     *
     * @param cvDetectionsFrame - The frame to draw overlay onto.
     * @param vDetectedTags - The vector of tagdetectutils::ArucoTag struct used to draw tag corners and IDs onto image.
     *
     * @note Image must be a 1 or 3 channel image and image must match dimensions of image when used for
     *      detection of the given tags.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-19
     ******************************************************************************/
    inline void DrawDetections(cv::Mat& cvDetectionsFrame, const std::vector<tagdetectutils::ArucoTag>& vDetectedTags)
    {
        // Create instance variables.
        std::vector<int> vIDs;
        std::vector<std::vector<cv::Point2f>> vMarkers;

        // Loop through each of the given AR tags and repackage them so that the draw function can read them.
        for (long unsigned int nIter = 0; nIter < vDetectedTags.size(); ++nIter)
        {
            // Check if the tag detection type is OpenCV.
            if (vDetectedTags[nIter].eDetectionMethod == tagdetectutils::TagDetectionMethod::eOpenCV)
            {
                // Append tag ID.
                vIDs.emplace_back(vDetectedTags[nIter].nID);

                // Assemble vector of marker corners.
                std::vector<cv::Point2f> cvMarkerCorners;
                cvMarkerCorners.emplace_back(cv::Point2f(vDetectedTags[nIter].pBoundingBox->x, vDetectedTags[nIter].pBoundingBox->y));          // Top-left corner
                cvMarkerCorners.emplace_back(cv::Point2f(vDetectedTags[nIter].pBoundingBox->x + vDetectedTags[nIter].pBoundingBox->width,
                                                         vDetectedTags[nIter].pBoundingBox->y));                                                // Top-right corner
                cvMarkerCorners.emplace_back(cv::Point2f(vDetectedTags[nIter].pBoundingBox->x + vDetectedTags[nIter].pBoundingBox->width,
                                                         vDetectedTags[nIter].pBoundingBox->y + vDetectedTags[nIter].pBoundingBox->height));    // Bottom-right corner
                cvMarkerCorners.emplace_back(cv::Point2f(vDetectedTags[nIter].pBoundingBox->x,
                                                         vDetectedTags[nIter].pBoundingBox->y + vDetectedTags[nIter].pBoundingBox->height));    // Bottom-left corner
                // Append vector of marker corners.
                vMarkers.emplace_back(cvMarkerCorners);
            }
        }

        // Check if the given frame is a 1 or 3 channel image. (not BGRA)
        if (!cvDetectionsFrame.empty() && (cvDetectionsFrame.channels() == 1 || cvDetectionsFrame.channels() == 3))
        {
            // Draw markers onto normal given image.
            // cv::aruco::drawDetectedMarkers(cvDetectionsFrame, vMarkers, vIDs, cv::Scalar(0, 0, 0));

            int nIter = 0;
            for (std::vector<cv::Point2f>& cvMarkerCorners : vMarkers)
            {
                // Draw tag ID onto image.
                std::string szText  = "TAG " + std::to_string(vIDs[nIter++]);
                cv::Size cvTextSize = cv::getTextSize(szText, cv::FONT_HERSHEY_SIMPLEX, 0.75, 1, nullptr);
                cv::rectangle(cvDetectionsFrame,
                              cvMarkerCorners[0],
                              cvMarkerCorners[0] + cv::Point2f(cvTextSize.width * 1.25, cvTextSize.height * 2),
                              cv::Scalar(0, 0, 0),
                              cv::FILLED);
                cv::putText(cvDetectionsFrame,
                            szText,
                            cvMarkerCorners[0] + cv::Point2f(5, cvTextSize.height * 1.25),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.75,
                            cv::Scalar(255, 255, 255));
            }
        }
        else
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger,
                      "ArucoDetect: Unable to draw markers on image because it is empty or because it has {} channels. (Should be 1 or 3)",
                      cvDetectionsFrame.channels());
        }
    }
}    // namespace arucotag

#endif
