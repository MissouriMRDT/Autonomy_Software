/******************************************************************************
 * @brief Unit test for Aruco Tag Detection implemented using OpenCV
 *
 * @file ArucoDetection.cc
 * @author JSpencerPittman (jspencerpittman@gmail.com)
 * @date 2023-10-10
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/vision/aruco/ArucoDetection.hpp"
#include "../../../../TestingBase.hh"

/// \cond
#include <chrono>
#include <filesystem>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <thread>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the Aruco Tag Detection using OpenCV
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class ArucoDetectionTests : public TestingBase<ArucoDetectionTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

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

        cv::Mat LoadImageFromRelativePath(const std::string& relativePath)
        {
            std::filesystem::path pathParentDir = std::filesystem::path(__FILE__).parent_path();
            std::filesystem::path pathImage     = pathParentDir / relativePath;
            return cv::imread(pathImage, cv::IMREAD_COLOR);
        }

    public:
        /******************************************************************************
         * @brief Construct a new Tag Detect OpenCV Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        ArucoDetectionTests() {}

        /******************************************************************************
         * @brief Destroy the Tag Detect OpenCV Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        ~ArucoDetectionTests() {}

        /******************************************************************************
         * @brief Setup the Tag Detect OpenCV Tests object.
         *
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Teardown the Tag Detect OpenCV Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Test the functionality of the FindTagcvCenterPoint method
 *
 *
 * @author JSpencerPittman (jspencerpittman@gmail.com)
 * @date 2023-10-11
 ******************************************************************************/
TEST_F(ArucoDetectionTests, FindTagCenter)
{
    arucotag::ArucoTag stTag;
    stTag.cvBoundingBox                = std::make_shared<cv::Rect2d>(3.0, 0.0, 7.0, 5.0);    // x, y, width, height

    cv::Point2f cvPredictedCenterPoint = FindTagCenter(stTag);

    cv::Point2f cvExpectedCenterPoint{6.75, 3.5};
    EXPECT_PRED2([this](const cv::Point2f& p1, const cv::Point2f& p2) { return PointsAreEqual(p1, p2); }, cvPredictedCenterPoint, cvExpectedCenterPoint);
}

/******************************************************************************
 * @brief Tests if the Detect method can find a tag and properly classify its ID
 *
 *
 * @author JSpencerPittman (jspencerpittman@gmail.com)
 * @date 2023-10-11
 ******************************************************************************/
TEST_F(ArucoDetectionTests, SingleCleanTagDetect)
{
    // initialize aruco detector
    cv::aruco::Dictionary cvDictionary = cv::aruco::getPredefinedDictionary(constants::ARUCO_DICTIONARY);
    cv::aruco::ArucoDetector cvDetector(cvDictionary);

    // Load the image containing the sample ArUco tag
    cv::Mat cvTestImageMat = LoadImageFromRelativePath("../../../../../data/Tests/aruco/cleanArucoMarker0.png");

    // Detect tags in the image
    std::vector<arucotag::ArucoTag> vDetectedTags;
    vDetectedTags = arucotag::Detect(cvTestImageMat, cvDetector);

    // Should have only detected one tag
    ASSERT_EQ(vDetectedTags.size(), 1);

    arucotag::ArucoTag stDetectedTag = vDetectedTags[0];

    // Verify ID is 0
    EXPECT_EQ(stDetectedTag.nID, 0);

    // Does the detected tag's corners match with the real tags
    cv::Point2f cvExpectedCornerTL{220, 220};
    cv::Point2f cvExpectedCornerTR{419, 220};
    cv::Point2f cvExpectedCornerBL{220, 419};
    cv::Point2f cvExpectedCornerBR{419, 419};

    cv::Rect2d cvBoundingBox     = *stDetectedTag.cvBoundingBox;
    cv::Point2f cvActualCornerTL = cv::Point2f(cvBoundingBox.x, cvBoundingBox.y);
    cv::Point2f cvActualCornerTR = cv::Point2f(cvBoundingBox.x + cvBoundingBox.width, cvBoundingBox.y);
    cv::Point2f cvActualCornerBL = cv::Point2f(cvBoundingBox.x, cvBoundingBox.y + cvBoundingBox.height);
    cv::Point2f cvActualCornerBR = cv::Point2f(cvBoundingBox.x + cvBoundingBox.width, cvBoundingBox.y + cvBoundingBox.height);

    EXPECT_PRED2([this](const cv::Point2f& p1, const cv::Point2f& p2) { return PointsAreEqual(p1, p2); }, cvActualCornerTL, cvExpectedCornerTL);
    EXPECT_PRED2([this](const cv::Point2f& p1, const cv::Point2f& p2) { return PointsAreEqual(p1, p2); }, cvActualCornerTR, cvExpectedCornerTR);
    EXPECT_PRED2([this](const cv::Point2f& p1, const cv::Point2f& p2) { return PointsAreEqual(p1, p2); }, cvActualCornerBL, cvExpectedCornerBL);
    EXPECT_PRED2([this](const cv::Point2f& p1, const cv::Point2f& p2) { return PointsAreEqual(p1, p2); }, cvActualCornerBR, cvExpectedCornerBR);
}

/******************************************************************************
 * @brief Tests if the Detect method can find multiple tags and properly classify their vecIDS
 *
 *
 * @author JSpencerPittman (jspencerpittman@gmail.com)
 * @date 2023-10-11
 ******************************************************************************/
TEST_F(ArucoDetectionTests, MultiCleanTagDetect)
{
    // Number of tags for this test case
    const unsigned int unNumTags = 3;

    // initialize aruco detector
    cv::aruco::Dictionary cvDictionary = cv::aruco::getPredefinedDictionary(constants::ARUCO_DICTIONARY);
    cv::aruco::ArucoDetector cvDetector(cvDictionary);

    // Load the image containing the sample ArUco tags
    cv::Mat cvTestImageMat = LoadImageFromRelativePath("../../../../../data/Tests/aruco/cleanArucoMarkersMultiple.png");

    // Detect tags in the image
    std::vector<arucotag::ArucoTag> vecDetectedTags;
    vecDetectedTags = arucotag::Detect(cvTestImageMat, cvDetector);

    // Should have three detected tags
    ASSERT_EQ(vecDetectedTags.size(), unNumTags);

    // Actual values for the tags in the image
    std::vector<int> vecIDS{2, 3, 4};
    std::vector<cv::Point2f> vecCenters{{100, 100}, {320, 480}, {500, 200}};
    std::vector<int> vecSizes{100, 150, 200};

    unsigned int unCorrectlyIdentifiedTags   = 0;
    unsigned int unIncorrectlyIdentifiedTags = 0;

    // For each detected tag check its one of the real tags
    for (arucotag::ArucoTag& stDetectedTag : vecDetectedTags)
    {
        // Which real tag does it match
        int nTagIdx = -1;
        for (int i = 0; i < (int) vecIDS.size(); ++i)
        {
            if (vecIDS[i] == stDetectedTag.nID)
            {
                // Found the corresponding real tag
                nTagIdx = i;
                break;
            }
        }
        if (nTagIdx == -1)
        {
            // If no matching ID was found this was a falsely detected tag
            ++unIncorrectlyIdentifiedTags;
            continue;
        }

        cv::Point2f cvCenterPoint = vecCenters[nTagIdx];
        unsigned int unApothem    = vecSizes[nTagIdx] / 2;    // Half the length of a square's side

        // Calculate the expected corners
        cv::Point2f cvExpectedCornerTL{cvCenterPoint.x - unApothem, cvCenterPoint.y - unApothem};
        cv::Point2f cvExpectedCornerTR{cvCenterPoint.x + unApothem - 1, cvCenterPoint.y - unApothem};
        cv::Point2f cvExpectedCornerBL{cvCenterPoint.x - unApothem, cvCenterPoint.y + unApothem - 1};
        cv::Point2f cvExpectedCornerBR{cvCenterPoint.x + unApothem - 1, cvCenterPoint.y + unApothem - 1};

        // Do the corners between the expected and detected tags match?
        bool bTLMatch, bTRMatch, bBLMatch, bBRMatch;
        bTLMatch = PointsAreEqual<float>(cvExpectedCornerTL, cv::Point2f(stDetectedTag.cvBoundingBox->x, stDetectedTag.cvBoundingBox->y));
        bTRMatch =
            PointsAreEqual<float>(cvExpectedCornerTR, cv::Point2f(stDetectedTag.cvBoundingBox->x + stDetectedTag.cvBoundingBox->width, stDetectedTag.cvBoundingBox->y));
        bBLMatch =
            PointsAreEqual<float>(cvExpectedCornerBL, cv::Point2f(stDetectedTag.cvBoundingBox->x, stDetectedTag.cvBoundingBox->y + stDetectedTag.cvBoundingBox->height));
        bBRMatch = PointsAreEqual<float>(
            cvExpectedCornerBR,
            cv::Point2f(stDetectedTag.cvBoundingBox->x + stDetectedTag.cvBoundingBox->width, stDetectedTag.cvBoundingBox->y + stDetectedTag.cvBoundingBox->height));

        EXPECT_TRUE(bTLMatch);
        EXPECT_TRUE(bTRMatch);
        EXPECT_TRUE(bBLMatch);
        EXPECT_TRUE(bBRMatch);

        if (bTLMatch && bTRMatch && bBLMatch && bBRMatch)
        {
            // Since this tag has been found remove it from the vector of real tags remaining
            ++unCorrectlyIdentifiedTags;
            vecIDS.erase(vecIDS.begin() + nTagIdx);
            vecCenters.erase(vecCenters.begin() + nTagIdx);
            vecSizes.erase(vecSizes.begin() + nTagIdx);
        }
        else
        {
            // Since the corner's don't match this was a falsely detected tag
            ++unIncorrectlyIdentifiedTags;
        }
    }

    // Were all of the tags identified?
    ASSERT_EQ(unCorrectlyIdentifiedTags, unNumTags);
    // Were there no false detections?
    ASSERT_EQ(unIncorrectlyIdentifiedTags, 0);
}
