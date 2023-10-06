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

    /******************************************************************************
     * @brief Estimate the pose of a position with respect to the observer using a point cloud
     *
     * @param cvPointCloud - A point cloud of x,y,z coordinates
     * @param nPixelRow - What row the target is in the image
     * @param nPixelCol - What column the target is in the image
     * @return std::pair<double, double> - the distance and angle of the target with respect to the observer
     *
     * The angle only takes into account how far forward or backward the object is and how far to
     * the right or left the object is of the observer. I did not use how far the target is up or
     * down with respect to the observer for determining the angle.
     *
     * @author jspencerpittman (jspencerpittman@gmail.com)
     * @date 2023-10-05
     ******************************************************************************/
    std::pair<double, double> EstimatePoseFromPointCloud(const cv::Mat& cvPointCloud, int nPixelRow, int nPixelCol)
    {
        if (constants::ZED_COORD_SYSTEM != sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP)
        {
            // TODO: throw some error here to signal the code won't work in a different orientation
        }

        // FIXME: Jason - I have no clue what the type of each value will be in the point cloud so I just guess cv::Vec3d for now even though it's four channels.
        // Grab (x,y,z) coordinates from where the tag was detected
        cv::Vec3d coordinate = cvPointCloud.at<cv::Vec3d>(nPixelRow, nPixelCol);
        double forward = coordinate[2], right = coordinate[0], up = coordinate[1];

        // Calculate euclidean distance from ZED camera left eye to the point of interest
        double distance = sqrt(pow(forward, 2) + pow(right, 2) + pow(up, 2));

        // Calculate the angle on plane horizontal to the viewpoint
        double angle = atan2(right, forward);

        return {distance, angle};
    }
}    // namespace aruco
