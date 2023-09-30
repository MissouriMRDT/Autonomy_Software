// FIXME: This file is unneeded. This can be put in a private function within the other class.

#include "Aruco.h"

ArucoDetector::ArucoDetector(const cv::aruco::Dictionary& dictionary) : m_cvDictionary(dictionary) {}

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

void ArucoDetector::EstimatePose(ArucoTag& tag) {}
