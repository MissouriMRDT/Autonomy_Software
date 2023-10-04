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
            cv::Vec3d eulerAngles;               // Euler angle rotations of the tag with respect to the camera
    };

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
}    // namespace aruco

#endif
