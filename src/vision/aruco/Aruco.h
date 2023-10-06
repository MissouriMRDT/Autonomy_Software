#ifndef ARUCO_DETECTOR_H_
#define ARUCO_DETECTOR_H_

#include <algorithm>
#include <string>
#include <vector>

#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>

#include "./../../AutonomyConstants.h"
#include "./../../interfaces/AutonomyThread.hpp"
#include "./../../interfaces/Camera.hpp"
#include "./../../util/vision/ArucoSamplesUtility.hpp"
#include "./../../util/vision/FetchContainers.hpp"

namespace aruco
{
    /******************************************************************************
     * @brief Represents a single ArUco tag.
     *
     *
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-09-28
     ******************************************************************************/
    struct ArucoTag
    {
            int id;                              // ID of the tag
            std::vector<cv::Point2f> corners;    // Corners of the tag in the image
            double distance;                     // Distance between the tag and the camera
            double angle;                        //  This is the yaw angle so roll and pitch are ignored
    };

    cv::Point2f findTagCenter(const ArucoTag& tag);

    /******************************************************************************
     * @brief detect ArUco tags in the provided image
     *
     * @param image - camera frame
     * @return std::vector<ArucoTag> - vector of the detected tags in the frame
     *
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-09-28
     ******************************************************************************/
    std::vector<ArucoTag> Detect(const cv::Mat& image);

    /******************************************************************************
     * @brief Estimate the pose of a position with respect to the observer using a point cloud
     *
     * @param cvPointCloud - A point cloud of x,y,z coordinates
     * @param tag - the tag were estimating the pose of and then storing the distance and angle calculations in.
     *
     * The angle only takes into account how far forward/backward and left/right the tag is with respect to the rover. This meaning I ignore the up/down position of the
     *tag when calculating the angle.
     *
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-10-05
     ******************************************************************************/
    void EstimatePoseFromPointCloud(const cv::Mat& cvPointCloud, ArucoTag& tag);

    /******************************************************************************
     * @brief Estimate the pose of a position with respect to the observer using an image
     *
     * @param cvCameraMatrix - Matrix of camera's parameters including focal length and optical center
     * @param cvDistCoeffs - Matrix of the camera's distortion coefficients
     * @param tag - the tag were estimating the pose of and then storing the distance and angle calculations in.
     *
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-10-06
     ******************************************************************************/
    void EstimatePoseFromPNP(cv::Mat& cvCameraMatrix, cv::Mat& cvDistCoeffs, ArucoTag& tag);
}    // namespace aruco

#endif
