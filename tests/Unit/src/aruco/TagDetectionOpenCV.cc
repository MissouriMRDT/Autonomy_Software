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
#include <gtest/gtest.h>
#include <thread>

#include "./../../../../src/vision/aruco/ArucoDetection.hpp"

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
    tag.CornerTL       = cv::Point2f{7.0, 5.0};
    tag.CornerTR       = cv::Point2f{10.0, 8.0};
    tag.CornerBL       = cv::Point2f{3.0, 1.0};
    tag.CornerBR       = cv::Point2f{7.0, 0.0};

    cv::Point2f center = FindTagCenter(tag);

    ASSERT_FLOAT_EQ(center.x, 6.75);
    ASSERT_FLOAT_EQ(center.y, 3.5);
}
