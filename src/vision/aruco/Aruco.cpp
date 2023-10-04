#include "Aruco.h"

namespace aruco
{
    std::vector<ArucoTag> Detect(const cv::Mat& image)
    {
        // vector containing ids of all detected tags
        std::vector<int> ids;

        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();

        // detect the tags in the image
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(constants::ARUCO_DICTIONARY);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
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
}    // namespace aruco
