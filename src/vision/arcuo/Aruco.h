#ifndef ARUCO_DETECTOR_H_
#define ARUCO_DETECTOR_H_

#include <algorithm>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

typedef std::vector<cv::Point2f> Corners;

class ArucoTag
{
    public:
        ArucoTag(int id, Corners corners);
        int getID() const;
        Corners getCorners() const;

    private:
        int m_id;
        Corners m_corners;
};

class ArucoDetector
{
    public:
        ArucoDetector(const cv::aruco::Dictionary& dictionary);
        std::vector<ArucoTag> detect(const cv::Mat& image);

    private:
        cv::aruco::Dictionary m_dictionary;
};

#endif
