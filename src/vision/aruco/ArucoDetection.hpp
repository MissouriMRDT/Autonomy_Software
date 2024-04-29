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

/// \cond
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
     * @brief Represents a single ArUco tag. Stores all information about a specific
     *      tag detection.
     *
     *
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-09-28
     ******************************************************************************/
    struct ArucoTag
    {
        public:
            // Declare public struct member attributes.
            cv::Point2f CornerTL;            // The top left corner of the bounding box.
            cv::Point2f CornerTR;            // The top right corner of the bounding box.
            cv::Point2f CornerBL;            // The bottom left corner of the bounding box.
            cv::Point2f CornerBR;            // The bottom right corner of bounding box.
            int nID;                         // ID of the tag.
            int nHits;                       // Total number of detections for tag id.
            int nFramesSinceLastHit;         // The total number of frames since a tag with this ID was last detected.
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
    inline cv::Point2f FindTagCenter(const ArucoTag& stTag)
    {
        // Average of the four corners
        cv::Point2f cvCenter(0, 0);

        // Add each tag x, y to the center x, y.
        cvCenter.x += stTag.CornerBL.x + stTag.CornerBR.x + stTag.CornerTL.x + stTag.CornerTR.x;
        cvCenter.y += stTag.CornerBL.y + stTag.CornerBR.y + stTag.CornerTL.y + stTag.CornerTR.y;
        // Divide by number of corners.
        cvCenter.x /= 4;
        cvCenter.y /= 4;

        // Return a copy of the center point of the tag.
        return cvCenter;
    }

    /******************************************************************************
     * @brief Preprocess images for specifically for the Aruco Detect() method.
     *      This method creates a copy of the camera frame so as
     *      to not alter the original in case it's used after the call to this function
     *      completes.
     *
     * @param cvInputFrame - cv::Mat of the image to pre-process.
     * @param cvOutputFrame - cv::Mat to write the pre-processed image to.
     *
     * @todo Determine optimal order for speed
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2023-10-10
     ******************************************************************************/
    inline void PreprocessFrame(const cv::Mat& cvInputFrame, cv::Mat& cvOutputFrame)
    {
        // Grayscale.
        cv::cvtColor(cvInputFrame, cvOutputFrame, cv::COLOR_BGRA2GRAY);
        cv::filter2D(cvOutputFrame, cvInputFrame, -1, constants::ARUCO_SHARPEN_KERNEL_EXTRA);
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
     * @param cvFrame - The camera frame to run ArUco detection on. Should be BGR format.
     * @param cvArucoDetector - The configured aruco detector to use for detection.
     * @return std::vector<ArucoTag> - The resultant vector containing the detected tags in the frame.
     *
     * @note The given cvFrame SHOULD BE IN BGR FORMAT.
     *
     * @author jspencerpittman (jspencerpittman@gmail.com), clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-28
     ******************************************************************************/
    inline std::vector<ArucoTag> Detect(const cv::Mat& cvFrame, const cv::aruco::ArucoDetector& cvArucoDetector)
    {
        /// Create instance variables.
        std::vector<int> vIDs;
        std::vector<std::vector<cv::Point2f>> cvMarkerCorners, cvRejectedCandidates;

        // Run Aruco detection algorithm.
        cvArucoDetector.detectMarkers(cvFrame, cvMarkerCorners, vIDs, cvRejectedCandidates);

        // Store all of the detected tags as ArucoTag.
        std::vector<ArucoTag> vDetectedTags;
        vDetectedTags.reserve(vIDs.size());

        // Loop through each detection and build tag for it.
        for (long unsigned int unIter = 0; unIter < vIDs.size(); unIter++)
        {
            // Create and initialize new tag.
            ArucoTag stDetectedTag;
            stDetectedTag.nID = vIDs[unIter];
            // Copy corners.
            stDetectedTag.CornerTL = cvMarkerCorners[unIter][0];
            stDetectedTag.CornerTR = cvMarkerCorners[unIter][1];
            stDetectedTag.CornerBR = cvMarkerCorners[unIter][2];
            stDetectedTag.CornerBL = cvMarkerCorners[unIter][3];

            // Add new tag to detected tags vector.
            vDetectedTags.push_back(stDetectedTag);
        }

        // Return the detected tags.
        return vDetectedTags;
    }

    /******************************************************************************
     * @brief Given a vector of ArucoTag structs draw each tag corner and ID onto the given image.
     *
     * @param cvDetectionsFrame - The frame to draw overlay onto.
     * @param vDetectedTags - The vector of ArucoTag struct used to draw tag corners and IDs onto image.
     *
     * @note Image must be a 1 or 3 channel image and image must match dimensions of image when used for
     *      detection of the given tags.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-19
     ******************************************************************************/
    inline void DrawDetections(cv::Mat& cvDetectionsFrame, const std::vector<ArucoTag>& vDetectedTags)
    {
        // Create instance variables.
        std::vector<int> vIDs;
        std::vector<std::vector<cv::Point2f>> cvMarkers;

        // Loop through each of the given AR tags and repackage them so that the draw function can read them.
        for (long unsigned int nIter = 0; nIter < vDetectedTags.size(); ++nIter)
        {
            // Append tag ID.
            vIDs.emplace_back(vDetectedTags[nIter].nID);

            // Assemble vector of marker corners.
            std::vector<cv::Point2f> cvMarkerCorners;
            cvMarkerCorners.emplace_back(vDetectedTags[nIter].CornerTL);
            cvMarkerCorners.emplace_back(vDetectedTags[nIter].CornerTR);
            cvMarkerCorners.emplace_back(vDetectedTags[nIter].CornerBR);
            cvMarkerCorners.emplace_back(vDetectedTags[nIter].CornerBL);
            // Append vector of marker corners.
            cvMarkers.emplace_back(cvMarkerCorners);
        }

        // Check if the given frame is a 1 or 3 channel image. (not BGRA)
        if (!cvDetectionsFrame.empty() && (cvDetectionsFrame.channels() == 1 || cvDetectionsFrame.channels() == 3))
        {
            // Draw markers onto normal given image.
            cv::aruco::drawDetectedMarkers(cvDetectionsFrame, cvMarkers, vIDs, cv::Scalar(0, 0, 0));
        }
        else
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger,
                      "ArucoDetect: Unable to draw markers on image because it is empty or because it has {} channels. (Should be 1 or 3)",
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
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-10-05
     ******************************************************************************/
    inline void EstimatePoseFromPointCloud(const cv::Mat& cvPointCloud, ArucoTag& stTag)
    {
        // Confirm correct coordinate system.
        if (constants::ZED_COORD_SYSTEM != sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP)
        {
            // Submit logger message.
            LOG_CRITICAL(logging::g_qSharedLogger, "ArucoDetection: Calculations won't work for anything other than ZED coordinate system == LEFT_HANDED_Y_UP");
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

    /******************************************************************************
     * @brief Estimate the pose of a position with respect to the observer using an image
     *
     * @param cvCameraMatrix - Matrix of camera's parameters including focal length and optical center.
     * @param cvDistCoeffs - Matrix of the camera's distortion coefficients.
     * @param stTag - The tag we are estimating the pose of and then storing the distance and angle calculations in.
     *
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-10-06
     ******************************************************************************/
    inline void EstimatePoseFromPNP(cv::Mat& cvCameraMatrix, cv::Mat& cvDistCoeffs, ArucoTag& stTag)
    {
        // rotVec is how the tag is orientated with respect to the camera. It's 3 numbers defining an axis of rotation around which we rotate the angle which is the
        // euclidean distance of the vector. transVec is the XYZ translation of the tag from the camera if you image the convergence of light as a pinhole sitting at
        // (0,0,0) in space.
        cv::Vec3d cvRotVec, cvTransVec;

        // Set expected object coordinate system shape.
        cv::Mat cvObjPoints(4, 1, CV_32FC3);
        cvObjPoints.at<cv::Vec3f>(0) = cv::Vec3f{0, 0, 0};                                                                  // Top-left corner.
        cvObjPoints.at<cv::Vec3f>(1) = cv::Vec3f{constants::ARUCO_TAG_SIDE_LENGTH, 0, 0};                                   // Bottom-left corner.
        cvObjPoints.at<cv::Vec3f>(2) = cv::Vec3f{0, constants::ARUCO_TAG_SIDE_LENGTH, 0};                                   // Top-right corner.
        cvObjPoints.at<cv::Vec3f>(3) = cv::Vec3f{constants::ARUCO_TAG_SIDE_LENGTH, constants::ARUCO_TAG_SIDE_LENGTH, 0};    // Bottom-right corner.

        // Repackage tag image points into a mat.
        cv::Mat cvImgPoints(4, 1, CV_32FC3);
        cvImgPoints.at<cv::Vec3f>(0) = cv::Vec3f{stTag.CornerTL.x, stTag.CornerTL.y, 0};    // Top-left corner.
        cvImgPoints.at<cv::Vec3f>(1) = cv::Vec3f{stTag.CornerBL.x, stTag.CornerBL.y, 0};    // Bottom-left corner.
        cvImgPoints.at<cv::Vec3f>(2) = cv::Vec3f{stTag.CornerTR.x, stTag.CornerTR.y, 0};    // Top-right corner.
        cvImgPoints.at<cv::Vec3f>(3) = cv::Vec3f{stTag.CornerBR.x, stTag.CornerBR.y, 0};    // Bottom-right corner.

        // Use solve perspective n' point algorithm to estimate pose of the tag.
        cv::solvePnP(cvObjPoints, cvImgPoints, cvCameraMatrix, cvDistCoeffs, cvRotVec, cvTransVec);

        // Grab (x,y,z) coordinates from where the tag was detected
        double dForward = cvTransVec[2];
        double dRight   = cvTransVec[0];
        double dUp      = cvTransVec[1];

        // Calculate euclidean distance from ZED camera left eye to the point of interest
        stTag.dStraightLineDistance = std::sqrt(std::pow(dForward, 2) + std::pow(dRight, 2) + std::pow(dUp, 2));

        // Calculate the angle on plane horizontal to the viewpoint
        stTag.dYawAngle = std::atan2(dRight, dForward);
    }
}    // namespace arucotag

#endif
