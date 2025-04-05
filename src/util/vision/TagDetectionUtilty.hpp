/******************************************************************************
 * @brief Implements functions related to tag detection. Tag detection runs on multiple cameras,
 *      under two modes of operations: Tensorflow or standard OpenCV. These functions make it more
 *      convenient to aggregate detected tags from all cameras for both OpenCV and Tensorflow.
 *
 * @file TagDetectionUtility.hpp
 * @author Jason Pittman (jspencerpittman@gmail.com)
 * @date 2024-10-07
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef TAG_DETECTION_UTILITY_HPP
#define TAG_DETECTION_UTILITY_HPP

/// \cond
#include <opencv2/opencv.hpp>

/// \endcond

///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * @brief Namespace containing functions to assist in tag detection.
 *
 *
 * @author Jason Pittman (jspencerpittman@gmail.com)
 * @date 2024-10-07
 ******************************************************************************/
namespace tagdetectutils
{
    /******************************************************************************
     * @brief Enum class to define the different tag detection methods available.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-04-03
     ******************************************************************************/
    enum class TagDetectionMethod
    {
        eUnknown,      // Unknown detection method.
        eOpenCV,       // Standard OpenCV detection using the ArUco library.
        eTorch,        // Torch detection using a YOLO model.
        eTensorflow    // Tensorflow detection using a YOLO model.
    };

    /******************************************************************************
     * @brief Represents a single ArUco tag. Combines attributes from TorchTag,
     *        TensorflowTag, and the original ArucoTag structs.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-04-03
     ******************************************************************************/
    struct ArucoTag
    {
        public:
            // Declare public struct member attributes.
            std::shared_ptr<cv::Rect2d> pBoundingBox         = std::make_shared<cv::Rect2d>();      // The bounding box of the detected tag.
            double dConfidence                               = 0.0;                                 // The detection confidence of the tag (from Torch/Tensorflow models).
            double dStraightLineDistance                     = 0.0;                                 // Distance between the tag and the camera.
            double dYawAngle                                 = 0.0;                                 // This is the yaw angle so roll and pitch are ignored.
            int nID                                          = -1;                                  // The ID of the tag. This is set to -1 if the tag is not detected.
            std::string szClassName                          = "";                                  // The class name of the tag (used in Torch/Tensorflow models).
            std::chrono::system_clock::time_point tmCreation = std::chrono::system_clock::now();    // Set the time detected to the current time.
            TagDetectionMethod eDetectionMethod              = TagDetectionMethod::eUnknown;        // The detection method used to detect the tag.
            cv::Size cvImageResolution;                                                             // The resolution of the image used to detect the tag.
            double dHorizontalFOV;                                                                  // The horizontal field of view of the camera used to detect the tag.

            /******************************************************************************
             * @brief Overload the equality operator for the ArucoTag struct.
             *
             * @param stOther - The other ArucoTag struct to compare to.
             * @return true - The two ArucoTag structs are equal.
             * @return false - The two ArucoTag structs are not equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-04-03
             ******************************************************************************/
            bool operator==(const ArucoTag& stOther) const
            {
                return pBoundingBox == stOther.pBoundingBox && dConfidence == stOther.dConfidence && dStraightLineDistance == stOther.dStraightLineDistance &&
                       dYawAngle == stOther.dYawAngle && nID == stOther.nID && szClassName == stOther.szClassName && tmCreation == stOther.tmCreation &&
                       eDetectionMethod == stOther.eDetectionMethod && cvImageResolution == stOther.cvImageResolution && dHorizontalFOV == stOther.dHorizontalFOV;
            }

            /******************************************************************************
             * @brief Overload the inequality operator for the ArucoTag struct.
             *
             * @param stOther - The other ArucoTag struct to compare to.
             * @return true - The two ArucoTag structs are not equal.
             * @return false - The two ArucoTag structs are equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-04-03
             ******************************************************************************/
            bool operator!=(const ArucoTag& stOther) const { return !(*this == stOther); }
    };

    /******************************************************************************
     * @brief Given an tagdetectutils::ArucoTag struct find the center point of the corners.
     *
     * @param stTag - The tag to find the center of.
     * @return cv::Point2f - The resultant center point within the image.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-13
     ******************************************************************************/
    inline cv::Point2f FindTagCenter(const tagdetectutils::ArucoTag& stTag)
    {
        // Calculate the center point of the tag.
        cv::Point2f cvCenter = cv::Point2f(stTag.pBoundingBox->x + stTag.pBoundingBox->width / 2, stTag.pBoundingBox->y + stTag.pBoundingBox->height / 2);

        return cvCenter;
    }

    /******************************************************************************
     * @brief Given a tagdetectutils::ArucoTag struct find the center point of the corners.
     *
     * @param cvPointCloud - A point cloud image to estimate the pose of the tag.
     * @param stTag - The tag to estimate the pose of.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-13
     ******************************************************************************/
    inline void EstimatePoseFromPointCloud(const cv::Mat& cvPointCloud, tagdetectutils::ArucoTag& stTag)
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
    inline void EstimatePoseFromPNP(cv::Mat& cvCameraMatrix, cv::Mat& cvDistCoeffs, tagdetectutils::ArucoTag& stTag)
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
        cvImgPoints.at<cv::Vec3f>(0) = cv::Vec3f{static_cast<float>(stTag.pBoundingBox->x), static_cast<float>(stTag.pBoundingBox->y), 0.0f};      // Top-left corner.
        cvImgPoints.at<cv::Vec3f>(1) =
            cv::Vec3f{static_cast<float>(stTag.pBoundingBox->x), static_cast<float>(stTag.pBoundingBox->y + stTag.pBoundingBox->height), 0.0f};    // Bottom-left corner.
        cvImgPoints.at<cv::Vec3f>(2) =
            cv::Vec3f{static_cast<float>(stTag.pBoundingBox->x + stTag.pBoundingBox->width), static_cast<float>(stTag.pBoundingBox->y), 0.0f};     // Top-right corner.
        cvImgPoints.at<cv::Vec3f>(3) = cv::Vec3f{static_cast<float>(stTag.pBoundingBox->x + stTag.pBoundingBox->width),
                                                 static_cast<float>(stTag.pBoundingBox->y + stTag.pBoundingBox->height),
                                                 0.0f};    // Bottom-right corner.

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

    /******************************************************************************
     * @brief - Estimate the pose of a tag from a camera frame.
     *
     * @param stTag - The tag to estimate the pose of.
     *
     * @note In order for this to be accurate, the camera's horizontal field of view (HFOV) and the camera frame size must be known.
     *
     * @author sam_hajdukiewicz (samanthahajdukiewicz@gmail.com) :3
     * @date 2025-04-04
     ******************************************************************************/
    inline void EstimatePoseFromCameraFrame(tagdetectutils::ArucoTag& stTag)
    {
        // Use camera field of view and camera frame size to determine tag angle in degrees from center of camera.
        double dDegreesPerPixel = stTag.dHorizontalFOV / stTag.cvImageResolution.width;
        // Find tag error in pixels from center of image.
        double dTagErrorX = (stTag.pBoundingBox->x + stTag.pBoundingBox->width / 2) - (stTag.cvImageResolution.width / 2);
        // Find angle error.
        double dTagAngleX = dTagErrorX * dDegreesPerPixel;
        // Reassign yaw and distance to tag.
        stTag.dYawAngle = dTagAngleX;

        // Estimate the distance using the area of the tag and the horizontal field of view (HFOV).
        // This is a rough approximation assuming the tag is square and the camera's HFOV is known.
        double dTagArea   = stTag.pBoundingBox->area();    // Area of the tag in pixels.
        double dImageArea = stTag.cvImageResolution.width * stTag.cvImageResolution.height;
        // Calculate the ratio of the tag's area to the image area.
        double dAreaRatio = dTagArea / dImageArea;
        // Estimate the distance using the inverse square root of the area ratio.
        // This assumes the tag's apparent size decreases with the square of the distance.
        // This value is completely dimensionless and arbitrary, but it should be consistent between different image sizes and field of views.
        stTag.dStraightLineDistance = 1.0 / std::sqrt(dAreaRatio);
    }

}    // namespace tagdetectutils

#endif
