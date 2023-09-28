#ifndef ARUCO_DETECTOR_H_
#define ARUCO_DETECTOR_H_

#include <algorithm>
#include <string>
#include <vector>

#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>

#include "./../../util/vision/ArucoSamplesUtility.hpp"

#define TAG_SIDE_LENGTH 1    // meters

/******************************************************************************
 * @brief Represents a single ArUco tag.
 *
 *
 * @author jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-09-28
 ******************************************************************************/
class ArucoTag
{
    public:
        typedef std::vector<cv::Point2f> Corners;

        /******************************************************************************
         * @brief Construct a new Aruco Tag object.
         *
         * @param id - id assigned to the ArUco tag
         * @param corners - a vector containing the four corners of the detected ArUco tag
         *                      in the image.
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-28
         ******************************************************************************/
        ArucoTag(int id, Corners corners);

        /******************************************************************************
         * @brief Get ID
         *
         * @return int - id of the ArUco tag
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-28
         ******************************************************************************/
        int GetID() const;

        /******************************************************************************
         * @brief Get Corners
         *
         * @return Corners - vector of the four corners of the detected ArUco tag
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-28
         ******************************************************************************/
        Corners GetCorners() const;

        /******************************************************************************
         * @brief Get distance
         *
         * @return double - distance to the tag
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-28
         ******************************************************************************/
        double GetDistance() const;

        /******************************************************************************
         * @brief Get Euler Angles
         *
         * @return cv::Vec3d - euler angles (pitch, yaw, roll)
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-28
         ******************************************************************************/
        cv::Vec3d GetEulerAngles() const;

        /******************************************************************************
         * @brief Set the orientation of the tag with respect to the rover
         *
         * @param distance - distance from the camera
         * @param eulerAngles - rotation in euler angles with respect to camera
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-28
         ******************************************************************************/
        void SetPose(double distance, cv::Vec3d eulerAngles);

    private:
        int m_nID;
        Corners m_vCorners;

        double m_dDistance;
        cv::Vec3d m_cvEulerAnglesVec;
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
         * @brief Construct a new Aruco Detector object.
         *
         * @param dictionary - Which dictionary are the ArUco tags in (DICT_4x4_50, DICT_6x6_250, ...)
         * @param cameraParamsFilename - path to the camera calibration parameters file
         * @see util/vision/ArucoSamplesUtility.hpp
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-28
         ******************************************************************************/
        ArucoDetector(const cv::aruco::Dictionary& dictionary, const std::string& cameraParamsFilename);

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
        cv::Vec3d AxisAngleRotation2EulerAngles(cv::Vec3d rotVec) const;

        cv::aruco::Dictionary m_cvDictionary;
        std::string m_szCameraParamsFilename;
};

#endif
