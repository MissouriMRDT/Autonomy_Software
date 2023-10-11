/******************************************************************************
 * @brief Unit test for Aruco Tag Detection implemented using OpenCV
 *
 * @file TagDetectionOpenCV.cc
 * @author JSpencerPittman (jspencerpittman@gmail.com)
 * @date 2023-10-10
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include <chrono>
#include <filesystem>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <thread>

#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>

#include "./../../../../src/AutonomyConstants.h"
#include "./../../../../src/vision/aruco/ArucoDetection.hpp"

/******************************************************************************
 * @brief Are two points equal to each other
 *
 * @tparam T - Data type of the points coordinates (int, double, float, ..)
 * @param p1 - first point
 * @param p2 - second point
 * @return true - points are equal to each other
 * @return false - points are not equal to each other
 *
 * @author JSpencerPittman (jspencerpittman@gmail.com)
 * @date 2023-10-11
 ******************************************************************************/
template<typename T>
bool PointsAreEqual(const cv::Point_<T>& p1, const cv::Point_<T>& p2)
{
    return p1.x == p2.x && p1.y == p2.y;
}

/******************************************************************************
 * @brief Test the functionality of the FindTagCenter method
 *
 *
 * @author JSpencerPittman (jspencerpittman@gmail.com)
 * @date 2023-10-11
 ******************************************************************************/
TEST(TagDetectOpenCVTest, FindCenter)
{
    arucotag::ArucoTag tag;
    tag.CornerTL                = cv::Point2f{7.0, 5.0};
    tag.CornerTR                = cv::Point2f{10.0, 8.0};
    tag.CornerBL                = cv::Point2f{3.0, 1.0};
    tag.CornerBR                = cv::Point2f{7.0, 0.0};

    cv::Point2f predictedCenter = FindTagCenter(tag);

    cv::Point2f expectedCenter{6.75, 3.5};
    EXPECT_PRED2(PointsAreEqual<float>, predictedCenter, expectedCenter);
}

/******************************************************************************
 * @brief Tests if the Detect method can find a tag and properly classify its ID
 *
 *
 * @author JSpencerPittman (jspencerpittman@gmail.com)
 * @date 2023-10-11
 ******************************************************************************/
TEST(TagDetectOpenCVTest, SingleCleanTagDetect)
{
    // initialize aruco detector
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(constants::ARUCO_DICTIONARY);
    cv::aruco::ArucoDetector detector(dictionary);

    std::filesystem::path pathParentDir = std::filesystem::path(__FILE__).parent_path();
    std::filesystem::path pathImage     = pathParentDir / "rsc/cleanArucoMarker0.png";
    cv::Mat testImage                   = cv::imread(pathImage, cv::IMREAD_COLOR);

    std::vector<arucotag::ArucoTag> detectedTags;
    detectedTags = arucotag::Detect(testImage, detector);

    // Should have only detected one tag
    ASSERT_GT(detectedTags.size(), 0);
    EXPECT_EQ(detectedTags.size(), 1);

    arucotag::ArucoTag detTag = detectedTags[0];

    // Verify ID is 0
    EXPECT_EQ(detTag.nID, 0);

    // Verify corners
    cv::Point2f expectedCornerTL{220, 220};
    cv::Point2f expectedCornerTR{419, 220};
    cv::Point2f expectedCornerBL{220, 419};
    cv::Point2f expectedCornerBR{419, 419};

    EXPECT_PRED2(PointsAreEqual<float>, expectedCornerTL, detTag.CornerTL);
    EXPECT_PRED2(PointsAreEqual<float>, expectedCornerTR, detTag.CornerTR);
    EXPECT_PRED2(PointsAreEqual<float>, expectedCornerBL, detTag.CornerBL);
    EXPECT_PRED2(PointsAreEqual<float>, expectedCornerBR, detTag.CornerBR);
}
