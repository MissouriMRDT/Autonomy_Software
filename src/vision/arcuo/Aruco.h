#ifndef ARUCO_DETECTOR_H_
#define ARUCO_DETECTOR_H_

#include <algorithm>
#include <future>
#include <shared_mutex>
#include <string>
#include <vector>

#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>

#include "./../../interfaces/AutonomyThread.hpp"
#include "./../../interfaces/Camera.hpp"
#include "./../../util/vision/ArucoSamplesUtility.hpp"
#include "./../../util/vision/FetchContainers.hpp"

#define TAG_SIDE_LENGTH 1    // meters

/******************************************************************************
 * @brief Represents a single ArUco tag.
 *
 *
 * @author jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-09-28
 ******************************************************************************/
struct ArucoTag
{
        typedef std::vector<cv::Point2f> Corners;

        int id;                   // ID of the tag
        Corners corners;          // Corners of the tag in the image
        double distance;          // Distance between the tag and the camera
        cv::Vec3d eulerAngles;    // Euler angle rotations of the tag with respect to the camera
};

/******************************************************************************
 * @brief Provides the ability to detect tags in an image as well as estimating
 *          the detected tag's position
 *
 *
 * @author jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-09-28
 ******************************************************************************/
class ArucoDetector
{
    public:
        /******************************************************************************
         * @brief Construct a new Aruco Detector object.
         *
         * @param dictionary - Which dictionary are the ArUco tags in (DICT_4x4_50, DICT_6x6_250, ...)
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-28
         ******************************************************************************/
        ArucoDetector(const cv::aruco::Dictionary& dictionary);

        /******************************************************************************
         * @brief detect ArUco tags in the provided image
         *
         * @param image - image containing the ArUco tags
         * @return std::vector<ArucoTag> -
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-28
         ******************************************************************************/
        std::vector<ArucoTag> Detect(const cv::Mat& image);

        /******************************************************************************
         * @brief estimate the tag's position and euler angle rotation with respect to camera
         *
         * @param tag - tag were estimating the pose of
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-28
         ******************************************************************************/
        void EstimatePose(ArucoTag& tag);

    private:
        /******************************************************************************
         * @brief convert axis angle rotations to euler angles
         *
         * @param rotVec - A 3-dimensionsal vector of axis-angle-rotations where
         *                  the L2 norm of the vector is the angle rotated about the axis
         *                  in the direction of vector's unit vector.
         * @return cv::Vec3d - euler angles in the order of pitch, yaw, roll
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-28
         ******************************************************************************/
        static cv::Vec3d AxisAngleRotation2EulerAngles(cv::Vec3d rotVec) const;

        cv::aruco::Dictionary m_cvDictionary;
        std::string m_szCameraParamsFilename;
};

#endif
