#include "Aruco.h"

namespace aruco
{
    cv::Point2f findTagCenter(const ArucoTag& tag)
    {
        // Average of the four corners
        cv::Point2f center{0, 0};
        for (auto& corner : tag.corners)
        {
            center.x += corner.x;
            center.y += corner.y;
        }
        center.x /= 4;
        center.y /= 4;

        return center;
    }

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

    void EstimatePoseFromPointCloud(const cv::Mat& cvPointCloud, ArucoTag& tag)
    {
        if (constants::ZED_COORD_SYSTEM != sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP)
        {
            // TODO: throw some error here to signal the code won't work in a different orientation
        }

        cv::Point2f center = findTagCenter(tag);

        // FIXME: Jason - I have no clue what the type of each value will be in the point cloud so I just guess cv::Vec3d for now even though it's four channels.
        // Grab (x,y,z) coordinates from where the tag was detected
        cv::Vec3f coordinate = cvPointCloud.at<cv::Vec3f>(center.y, center.x);
        double forward = coordinate[2], right = coordinate[0], up = coordinate[1];

        // Calculate euclidean distance from ZED camera left eye to the point of interest
        tag.distance = sqrt(pow(forward, 2) + pow(right, 2) + pow(up, 2));

        // Calculate the angle on plane horizontal to the viewpoint
        tag.angle = atan2(right, forward);
    }

    void EstimatePoseFromPNP(cv::Mat& cvCameraMatrix, cv::Mat& cvDistCoeffs, ArucoTag& tag)
    {
        // rotVec is how the tag is orientated with respect to the camera. It's 3 numbers defining an axis of rotation around which we rotate the angle which is the
        // euclidean distance of the vector. transVec is the XYZ translation of the tag from the camera if you image the convergence of light as a pinhole sitting at
        // (0,0,0) in space.
        cv::Vec3d rotVec, transVec;

        // Set object coordinate system
        cv::Mat objPoints(4, 1, CV_32FC3);
        objPoints.at<cv::Vec3f>(0) = cv::Vec3f{0, 0, 0};
        objPoints.at<cv::Vec3f>(1) = cv::Vec3f{constants::ARUCO_TAG_SIDE_LENGTH, 0, 0};
        objPoints.at<cv::Vec3f>(2) = cv::Vec3f{0, constants::ARUCO_TAG_SIDE_LENGTH, 0};
        objPoints.at<cv::Vec3f>(3) = cv::Vec3f{constants::ARUCO_TAG_SIDE_LENGTH, constants::ARUCO_TAG_SIDE_LENGTH, 0};

        solvePnP(objPoints, tag.corners, cvCameraMatrix, cvDistCoeffs, rotVec, transVec);

        // Grab (x,y,z) coordinates from where the tag was detected
        double forward = transVec[2], right = transVec[0], up = transVec[1];

        // Calculate euclidean distance from ZED camera left eye to the point of interest
        tag.distance = sqrt(pow(forward, 2) + pow(right, 2) + pow(up, 2));

        // Calculate the angle on plane horizontal to the viewpoint
        tag.angle = atan2(right, forward);
    }
}    // namespace aruco
