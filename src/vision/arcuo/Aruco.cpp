#include "Aruco.h"

ArucoDetector::ArucoDetector(const cv::aruco::Dictionary& dictionary) : m_cvDictionary(dictionary) {}

ArucoDetector::ArucoDetector(const cv::aruco::Dictionary& dictionary, const std::string& cameraParamsFilename) :
    m_cvDictionary(dictionary), m_szCameraParamsFilename(cameraParamsFilename)
{}

std::vector<ArucoTag> ArucoDetector::Detect(const cv::Mat& image)
{
    // vector containing ids of all detected tags
    std::vector<int> ids;

    std::vector<ArucoTag::Corners> markerCorners, rejectedCandidates;
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();

    // detect the tags in the image
    cv::aruco::ArucoDetector detector(m_cvDictionary, detectorParams);
    detector.detectMarkers(image, markerCorners, ids, rejectedCandidates);

    // store all of the detectected tags as ArucoTag
    std::vector<ArucoTag> detectedTags;
    detectedTags.reserve((int) ids.size());

    for (int i = 0; i < (int) ids.size(); i++)
    {
        ArucoTag detectedTag;
        detectedTag.id      = ids[i];
        detectedTag.corners = markerCorners[i];
        detectedTags.push_back(detectedTag);
    }

    return detectedTags;
}

void ArucoDetector::EstimatePose(ArucoTag& tag)
{
    // camera parameters
    cv::Mat camMatrix, distCoeffs;
    readCameraParameters(m_szCameraParamsFilename, camMatrix, distCoeffs);

    cv::Vec3d rotVec, transVec;

    // Set object coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.at<cv::Vec3f>(0) = cv::Vec3f{0, 0, 0};
    objPoints.at<cv::Vec3f>(1) = cv::Vec3f{TAG_SIDE_LENGTH, 0, 0};
    objPoints.at<cv::Vec3f>(2) = cv::Vec3f{0, TAG_SIDE_LENGTH, 0};
    objPoints.at<cv::Vec3f>(3) = cv::Vec3f{TAG_SIDE_LENGTH, TAG_SIDE_LENGTH, 0};

    solvePnP(objPoints, tag.m_vCorners, camMatrix, distCoeffs, rotVec, transVec);

    double distance       = cv::norm(transVec);

    cv::Vec3d eulerAngles = AxisAngleRotation2EulerAngles(rotVec);

    tag.distance          = distance;
    tag.eulerAngles       = eulerAngles;
}

cv::Vec3d ArucoDetector::AxisAngleRotation2EulerAngles(cv::Vec3d rotVec) const
{
    cv::Mat rotMat;
    cv::Rodrigues(rotVec, rotMat);

    double v     = sqrt(pow(rotMat.at<double>(0, 0), 2) + pow(rotMat.at<double>(1, 0), 2));
    double pitch = atan2(-rotMat.at<double>(2, 0), v);

    double yaw   = atan2(rotMat.at<double>(2, 1), rotMat.at<double>(2, 2));
    double roll  = atan2(rotMat.at<double>(1, 0), rotMat.at<double>(0, 0));

    return {pitch, yaw, roll};
}