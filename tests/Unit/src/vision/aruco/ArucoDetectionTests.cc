/******************************************************************************
 * @brief Unit test for Aruco Tag Detection implemented using OpenCV
 *
 * @file ArucoDetection.cc
 * @author JSpencerPittman (jspencerpittman@gmail.com)
 * @date 2023-10-10
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
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
        ArucoDetectionTests() { SetUp(); }

        /******************************************************************************
         * @brief Destroy the Tag Detect OpenCV Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        ~ArucoDetectionTests() { TearDown(); }

        /******************************************************************************
         * @brief Setup the Tag Detect OpenCV Tests object.
         *
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void SetUp() override
        {
            // Call the base setup method. This initializes the loggers and RoveComm instances.
            RequiredSetup();
        }

        /******************************************************************************
         * @brief Teardown the Tag Detect OpenCV Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TearDown() override
        {
            // Call the base teardown method. This stops the RoveComm instances and loggers.
            RequiredTeardown();
        }
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
    arucotag::ArucoTag tag;
    tag.CornerTL                       = cv::Point2f{7.0, 5.0};
    tag.CornerTR                       = cv::Point2f{10.0, 8.0};
    tag.CornerBL                       = cv::Point2f{3.0, 1.0};
    tag.CornerBR                       = cv::Point2f{7.0, 0.0};

    cv::Point2f cvPredictedCenterPoint = FindTagCenter(tag);

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
    cv::Point2f expectedCornerTL{220, 220};
    cv::Point2f expectedCornerTR{419, 220};
    cv::Point2f expectedCornerBL{220, 419};
    cv::Point2f expectedCornerBR{419, 419};

    EXPECT_PRED2([this](const cv::Point2f& p1, const cv::Point2f& p2) { return PointsAreEqual(p1, p2); }, expectedCornerTL, stDetectedTag.CornerTL);
    EXPECT_PRED2([this](const cv::Point2f& p1, const cv::Point2f& p2) { return PointsAreEqual(p1, p2); }, expectedCornerTR, stDetectedTag.CornerTR);
    EXPECT_PRED2([this](const cv::Point2f& p1, const cv::Point2f& p2) { return PointsAreEqual(p1, p2); }, expectedCornerBL, stDetectedTag.CornerBL);
    EXPECT_PRED2([this](const cv::Point2f& p1, const cv::Point2f& p2) { return PointsAreEqual(p1, p2); }, expectedCornerBR, stDetectedTag.CornerBR);
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
        cv::Point2f expectedCornerTL{cvCenterPoint.x - unApothem, cvCenterPoint.y - unApothem};
        cv::Point2f expectedCornerTR{cvCenterPoint.x + unApothem - 1, cvCenterPoint.y - unApothem};
        cv::Point2f expectedCornerBL{cvCenterPoint.x - unApothem, cvCenterPoint.y + unApothem - 1};
        cv::Point2f expectedCornerBR{cvCenterPoint.x + unApothem - 1, cvCenterPoint.y + unApothem - 1};

        // Do the corners between the expected and detected tags match?
        bool bTLMatch, bTRMatch, bBLMatch, bBRMatch;
        bTLMatch = PointsAreEqual<float>(expectedCornerTL, stDetectedTag.CornerTL);
        bTRMatch = PointsAreEqual<float>(expectedCornerTR, stDetectedTag.CornerTR);
        bBLMatch = PointsAreEqual<float>(expectedCornerBL, stDetectedTag.CornerBL);
        bBRMatch = PointsAreEqual<float>(expectedCornerBR, stDetectedTag.CornerBR);

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
