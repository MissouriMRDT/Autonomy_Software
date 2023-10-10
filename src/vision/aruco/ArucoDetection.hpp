/******************************************************************************
 * @brief Defines and implements functions related to ArUco operations on images. All
 *      functions are defined within the arucotag namespace.
 *
 * @file ArucoDetection.h
 * @author jspencerpittman (jspencerpittman@gmail.com), clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

#include <vector>

#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>

#include "../../AutonomyConstants.h"
#include "../../AutonomyLogging.h"

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
            cv::Point2f CornerTL;                                                                 // The top left corner of the bounding box.
            cv::Point2f CornerTR;                                                                 // The top right corner of the bounding box.
            cv::Point2f CornerBL;                                                                 // The bottom left corner of the bounding box.
            cv::Point2f CornerBR;                                                                 // The bottom right corner of bounding box.
            std::vector<cv::Point2f*> vCorners = {&CornerTL, &CornerTR, &CornerBL, &CornerBR};    // Provide an easy method for getting all corners.
            int nID;                                                                              // ID of the tag.
            int nHits;                                                                            // Total number of detections for tag id.
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
     * @return std::vector<ArucoTag> - The resultant vector containing the detected tags in the frame.
     *
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-09-28
     ******************************************************************************/
    inline std::vector<ArucoTag> Detect(const cv::Mat& cvFrame)
    {
        // Create instance variables.
        std::vector<int> vIDs;
        std::vector<std::vector<cv::Point2f>> cvMarkerCorners, cvRejectedCandidates;
        cv::aruco::DetectorParameters cvArucoDetectionParams = cv::aruco::DetectorParameters();

        // Detect the tags in the image.
        cv::aruco::Dictionary cvTagDictionary = cv::aruco::getPredefinedDictionary(constants::ARUCO_DICTIONARY);
        cv::aruco::ArucoDetector cvArucoDetector(cvTagDictionary, cvArucoDetectionParams);
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
            stDetectedTag.CornerTR = cvMarkerCorners[unIter][0];
            stDetectedTag.CornerBL = cvMarkerCorners[unIter][0];
            stDetectedTag.CornerBR = cvMarkerCorners[unIter][0];

            // Add new tag to detected tags vector.
            vDetectedTags.push_back(stDetectedTag);
        }

        // Return the detected tags.
        return vDetectedTags;
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
