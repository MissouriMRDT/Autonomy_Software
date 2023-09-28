#include "Aruco.hpp"

ArucoTag::ArucoTag(int id, Corners corners) : m_id(id), m_corners(corners) {}

int ArucoTag::getID() const
{
    return m_id;
}

double ArucoTag::getDistance() const
{
    return m_distance;
}

cv::Vec3d ArucoTag::getEulerAngles() const
{
    return m_eulerAngles;
}

Corners ArucoTag::getCorners() const
{
    return m_corners;
}

void ArucoTag::setPose(double distance, cv::Vec3d eulerAngles)
{
    m_distance    = distance;
    m_eulerAngles = eulerAngles;
}

ArucoDetector::ArucoDetector(const cv::aruco::Dictionary& dictionary) : m_dictionary(dictionary) {}

ArucoDetector::ArucoDetector(const cv::aruco::Dictionary& dictionary, const std::string& cameraParamsFilename) :
    m_dictionary(dictionary), m_cameraParamsFilename(cameraParamsFilename)
{}

std::vector<ArucoTag> ArucoDetector::detect(const cv::Mat& image)
{
    std::vector<int> ids;
    std::vector<Corners> markerCorners, rejectedCandidates;
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(m_dictionary, detectorParams);
    detector.detectMarkers(image, markerCorners, ids, rejectedCandidates);

    std::vector<ArucoTag> detectedTags;
    detectedTags.reserve((int) ids.size());

    for (int i = 0; i < (int) ids.size(); i++)
        detectedTags.emplace_back(ids[i], markerCorners[i]);

    return detectedTags;
}

void ArucoDetector::poseEstimate(ArucoTag& tag)
{
    cv::Mat camMatrix, distCoeffs;
    readCameraParameters(m_cameraParamsFilename, camMatrix, distCoeffs);

    cv::Vec3d rvec, tvec;

    // Set object coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.at<cv::Vec3f>(0) = cv::Vec3f{0, 0, 0};
    objPoints.at<cv::Vec3f>(1) = cv::Vec3f{TAG_SIDE_LENGTH, 0, 0};
    objPoints.at<cv::Vec3f>(2) = cv::Vec3f{0, TAG_SIDE_LENGTH, 0};
    objPoints.at<cv::Vec3f>(3) = cv::Vec3f{TAG_SIDE_LENGTH, TAG_SIDE_LENGTH, 0};

    solvePnP(objPoints, tag.getCorners(), camMatrix, distCoeffs, rvec, tvec);

    double distance       = cv::norm(tvec);

    cv::Vec3d eulerAngles = axisAngleRotation2EulerAngles(rvec);

    tag.setPose(distance, eulerAngles);
}

cv::Vec3d ArucoDetector::axisAngleRotation2EulerAngles(cv::Vec3d rvec) const
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    double v     = sqrt(pow(R.at<double>(0, 0), 2) + pow(R.at<double>(1, 0), 2));
    double pitch = atan2(-R.at<double>(2, 0), v);

    double yaw   = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    double roll  = atan2(R.at<double>(1, 0), R.at<double>(0, 0));

    return {pitch, yaw, roll};
}
