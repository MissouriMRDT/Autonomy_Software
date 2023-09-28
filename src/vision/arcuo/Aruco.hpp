#ifndef ARUCO_DETECTOR_H_
#define ARUCO_DETECTOR_H_

#include <algorithm>
#include <string>
#include <vector>

#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>

#include "./../../util/vision/ArucoSamplesUtility.hpp"

#define TAG_SIDE_LENGTH 1  // meters

typedef std::vector<cv::Point2f> Corners;

class ArucoTag
{
    public:
        ArucoTag(int id, Corners corners);
        
        int getID() const;
        Corners getCorners() const;
        double getDistance() const;
        cv::Vec3d getEulerAngles() const;

        void setPose(double distance, cv::Vec3d eulerAngles);

    private:
        int m_id;
        Corners m_corners;

        double m_distance;
        cv::Vec3d m_eulerAngles;
};

class ArucoDetector
{
    public:
        ArucoDetector(const cv::aruco::Dictionary& dictionary);
        ArucoDetector(const cv::aruco::Dictionary& dictionary, const std::string& cameraParamsFilename);
        std::vector<ArucoTag> detect(const cv::Mat& image);
        void poseEstimate(ArucoTag& tag);

    private:
        cv::Vec3d axisAngleRotation2EulerAngles(cv::Vec3d rvec) const;

        cv::aruco::Dictionary m_dictionary;
        std::string m_cameraParamsFilename;
};

#endif
