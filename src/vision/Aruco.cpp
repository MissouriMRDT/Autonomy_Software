#include "Aruco.h"

ArucoTag::ArucoTag(int id, Corners corners)
    : m_id(id), m_corners(corners) {}

int ArucoTag::getID() const { return m_id; }

Corners ArucoTag::getCorners() const { return m_corners; }

ArucoDetector::ArucoDetector(const cv::aruco::Dictionary &dictionary)
    : m_dictionary(dictionary) {}

std::vector <ArucoTag> ArucoDetector::detect(const cv::Mat& image) {
    std::vector<int> ids;
    std::vector<Corners> markerCorners, rejectedCandidates;
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(m_dictionary, detectorParams);
    detector.detectMarkers(image, markerCorners, ids,
                           rejectedCandidates);

    std::vector<ArucoTag> detectedTags;
    detectedTags.reserve((int)ids.size());

    for(int i = 0; i < (int)ids.size(); i++)
        detectedTags.emplace_back(ids[i], markerCorners[i]);

    return detectedTags;
}