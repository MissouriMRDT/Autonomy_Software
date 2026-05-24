/******************************************************************************
 * @brief Unit Test Class for the Geolocate utility.
 *
 * @file Geolocate.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-27
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/util/vision/Geolocate.hpp"
#include "../../../../TestingBase.hh"

/// \cond
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

/// \endcond

/******************************************************************************
 * @brief Unit test class for the Geolocate utility.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-27
 ******************************************************************************/
class GeolocateTests : public TestingBase<GeolocateTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // Test data used across multiple tests.
        cv::Mat m_cvTestPointcloud;
        geoops::RoverPose m_stRoverPose;
        const double EPSILON = 0.0001;    // Used for floating point comparisons.

    public:
        /******************************************************************************
         * @brief Construct a new Geolocate Tests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-04-27
         ******************************************************************************/
        GeolocateTests() {}

        /******************************************************************************
         * @brief Destroy the Geolocate Tests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-04-27
         ******************************************************************************/
        ~GeolocateTests() {}

        /******************************************************************************
         * @brief Set up the Geolocate Tests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-04-27
         ******************************************************************************/
        void TestSetup() override
        {
            // Create a test pointcloud with known values. (10x10 matrix)
            m_cvTestPointcloud = cv::Mat(10, 10, CV_32FC4, cv::Scalar(0, 0, 0, 0));

            // Set some known values in the pointcloud
            // Point at (5,5) is 3 meters in front of camera, 1 meter to right, 0 meters up.
            m_cvTestPointcloud.at<cv::Vec4f>(5, 5) = cv::Vec4f(1.0f, 0.0f, 3.0f, 1.0f);

            // Point at (2,2) is 5 meters in front of camera, 2 meters to left, 1 meter up.
            m_cvTestPointcloud.at<cv::Vec4f>(2, 2) = cv::Vec4f(-2.0f, 1.0f, 5.0f, 1.0f);

            // Point at (8,8) is 4 meters in front of camera, 3 meters to right, 2 meters down.
            m_cvTestPointcloud.at<cv::Vec4f>(8, 8) = cv::Vec4f(3.0f, -2.0f, 4.0f, 1.0f);

            // Create a test rover pose (at UTM coordinates 500000, 4000000, zone 15, northern hemisphere)
            // with heading 0. (north)
            geoops::UTMCoordinate stRoverUTM(500000.0, 4000000.0, 15, true, 100.0);
            m_stRoverPose = geoops::RoverPose(stRoverUTM, 0.0);
        }

        /******************************************************************************
         * @brief Teardown the Geolocate Tests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-04-27
         ******************************************************************************/
        void TestTeardown() override
        {
            // Release the test pointcloud.
            m_cvTestPointcloud.release();
        }
};

/******************************************************************************
 * @brief Test GeolocateBox function with valid input and default neighborhood size
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-04-27
 ******************************************************************************/
TEST_F(GeolocateTests, GeolocateBoxValidInput)
{
    // Test with center point. (5,5)
    cv::Point cvPixel(5, 5);

    // Call the function being tested.
    geoops::Waypoint stWaypoint = geoloc::GeolocateBox(m_cvTestPointcloud, m_stRoverPose, cvPixel);

    // Get the UTM coordinate from the waypoint.
    const geoops::UTMCoordinate& stResultUTM = stWaypoint.GetUTMCoordinate();

    // Expected UTM calculation based on actual implementation:
    // X = 1.0, Y = 0.0, Z = 3.0
    // dAdjustedHeading = (0 * -1.0) + 90.0 = 90.0
    // dHeadingRad = 90 * PI/180 = PI/2
    // dEasting = 500000 + (3.0 * cos(PI/2) + 1.0 * sin(PI/2)) = 500000 + 1.0 = 500001
    // dNorthing = 4000000 + (3.0 * sin(PI/2) - 1.0 * cos(PI/2)) = 4000000 + 3.0 = 4000003

    EXPECT_NEAR(stResultUTM.dEasting, 500001.0, EPSILON);
    EXPECT_NEAR(stResultUTM.dNorthing, 4000003.0, EPSILON);
    EXPECT_EQ(stResultUTM.nZone, 15);
    EXPECT_TRUE(stResultUTM.bWithinNorthernHemisphere);
    EXPECT_NEAR(stResultUTM.dAltitude, 100.0, EPSILON);    // Rover altitude (100) + point Y (0)
}

/******************************************************************************
 * @brief Test GeolocateBox function with heading adjustment. (90 degrees)
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-04-27
 ******************************************************************************/
TEST_F(GeolocateTests, GeolocateBoxWithHeadingAdjustment)
{
    // Change rover heading to 90 degrees. (east)
    geoops::UTMCoordinate stRoverUTM = m_stRoverPose.GetUTMCoordinate();
    geoops::RoverPose stEastFacingPose(stRoverUTM, 90.0);

    // Test with center point. (5,5)
    cv::Point cvPixel(5, 5);

    // Call the function being tested.
    geoops::Waypoint stWaypoint = geoloc::GeolocateBox(m_cvTestPointcloud, stEastFacingPose, cvPixel);

    // Get the UTM coordinate from the waypoint.
    const geoops::UTMCoordinate& stResultUTM = stWaypoint.GetUTMCoordinate();

    // Expected UTM calculation based on actual implementation:
    // X = 1.0, Y = 0.0, Z = 3.0
    // dAdjustedHeading = (-90 * 1.0) + 90 = 0.0
    // dHeadingRad = 0 * PI/180 = 0.0
    // dEasting = 500000 + (3.0 * cos(0) + 1.0 * sin(0)) = 500000 + 3 + 0 = 500003
    // dNorthing = 4000000 + (3.0 * sin(0) - 1.0 * cos(0)) = 4000000 + 0 - 1 = 3999999

    EXPECT_NEAR(stResultUTM.dEasting, 500003.0, EPSILON);
    EXPECT_NEAR(stResultUTM.dNorthing, 3999999.0, EPSILON);
}

/******************************************************************************
 * @brief Test GeolocateBox function with invalid neighborhood size.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-04-27
 ******************************************************************************/
TEST_F(GeolocateTests, GeolocateBoxInvalidNeighborhoodSize)
{
    // Test with center point. (5,5)
    cv::Point cvPixel(5, 5);

    // Call the function with invalid (even) neighborhood size.
    geoops::Waypoint stWaypoint = geoloc::GeolocateBox(m_cvTestPointcloud, m_stRoverPose, cvPixel, 4);

    // Function should default to 5x5 neighborhood and produce the same result as the ValidInput test.
    const geoops::UTMCoordinate& stResultUTM = stWaypoint.GetUTMCoordinate();

    EXPECT_NEAR(stResultUTM.dEasting, 500001.0, EPSILON);
    EXPECT_NEAR(stResultUTM.dNorthing, 4000003.0, EPSILON);

    // Call the function with invalid (negative) neighborhood size.
    stWaypoint = geoloc::GeolocateBox(m_cvTestPointcloud, m_stRoverPose, cvPixel, -3);

    // Function should default to 5x5 neighborhood and produce the same result.
    const geoops::UTMCoordinate& stResultUTM2 = stWaypoint.GetUTMCoordinate();

    EXPECT_NEAR(stResultUTM2.dEasting, 500001.0, EPSILON);
    EXPECT_NEAR(stResultUTM2.dNorthing, 4000003.0, EPSILON);
}

/******************************************************************************
 * @brief Test GeolocateBox function with custom neighborhood size.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-04-27
 ******************************************************************************/
TEST_F(GeolocateTests, GeolocateBoxCustomNeighborhoodSize)
{
    // Test with center point. (5,5)
    cv::Point cvPixel(5, 5);

    // Call the function with custom neighborhood size.
    geoops::Waypoint stWaypoint = geoloc::GeolocateBox(m_cvTestPointcloud, m_stRoverPose, cvPixel, 3);

    // Result should be similar to default since we only have one point in that area.
    const geoops::UTMCoordinate& stResultUTM = stWaypoint.GetUTMCoordinate();

    EXPECT_NEAR(stResultUTM.dEasting, 500001.0, EPSILON);
    EXPECT_NEAR(stResultUTM.dNorthing, 4000003.0, EPSILON);
}

/******************************************************************************
 * @brief Test GeolocateBox function when pixel is out of bounds.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-04-27
 ******************************************************************************/
TEST_F(GeolocateTests, GeolocateBoxOutOfBoundsPixel)
{
    // Test with out of bounds point.
    cv::Point cvPixel(15, 15);

    // Call the function with out of bounds pixel.
    geoops::Waypoint stWaypoint = geoloc::GeolocateBox(m_cvTestPointcloud, m_stRoverPose, cvPixel);

    // Out-of-bounds pixel results in no valid 3D points in neighborhood,
    // triggering the Monocular Ground Plane Fallback logic.
    const geoops::UTMCoordinate& stResultUTM = stWaypoint.GetUTMCoordinate();

    // Expected calculations for Monocular Fallback:
    // nBottomY = std::min(9, 15 + 2) = 9
    // fRayAngleY = atan2(9 - 5.0, 5.0) = 0.6747 rad
    // fAvgZ = 100.0 / tan(0.6747) = 125.0
    // fAvgX = ((15 - 5.0) / 5.0) * 125.0 = 250.0
    // fAvgY = -100.0
    // dHeadingRad = PI/2 (based on 0 deg compass heading)
    // dEasting = 500000 + (125.0 * cos(PI/2) + 250.0 * sin(PI/2)) = 500250.0
    // dNorthing = 4000000 + (125.0 * sin(PI/2) - 250.0 * cos(PI/2)) = 4000125.0
    // dAltitude = 100.0 + (-100.0) = 0.0
    EXPECT_NEAR(stResultUTM.dEasting, 500250.0, EPSILON);
    EXPECT_NEAR(stResultUTM.dNorthing, 4000125.0, EPSILON);
    EXPECT_NEAR(stResultUTM.dAltitude, 0.0, EPSILON);
}

/******************************************************************************
 * @brief Test GeolocateBox function with invalid point (NaN values)
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-04-27
 ******************************************************************************/
TEST_F(GeolocateTests, GeolocateBoxInvalidPoint)
{
    // Create a point with NaN values.
    cv::Point cvPixel(3, 3);
    m_cvTestPointcloud.at<cv::Vec4f>(3, 3) = cv::Vec4f(NAN, NAN, NAN, 1.0f);

    // Call the function with NaN values.
    geoops::Waypoint stWaypoint = geoloc::GeolocateBox(m_cvTestPointcloud, m_stRoverPose, cvPixel);

    // The target pixel is NaN, but valid points exist in the 5x5 neighborhood!
    // Specifically, Setup() placed point (5,5) at Z=3.0 and (2,2) at Z=5.0.
    // Under 20th percentile target depth logic, point (5,5) (Z=3.0) is selected.
    const geoops::UTMCoordinate& stResultUTM = stWaypoint.GetUTMCoordinate();

    // Valid point (5,5) gives: fAvgX = 1.0, fAvgZ = 3.0, fAvgY = 0.0
    EXPECT_NEAR(stResultUTM.dEasting, 500001.0, EPSILON);
    EXPECT_NEAR(stResultUTM.dNorthing, 4000003.0, EPSILON);
    EXPECT_NEAR(stResultUTM.dAltitude, 100.0, EPSILON);
}

/******************************************************************************
 * @brief Test GeolocateBox function with multiple valid points in neighborhood.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-04-27
 ******************************************************************************/
TEST_F(GeolocateTests, GeolocateBoxMultipleValidPoints)
{
    // Clear pointcloud to avoid Setup() points skewing the target Z percentile.
    m_cvTestPointcloud = cv::Mat(10, 10, CV_32FC4, cv::Scalar(0, 0, 0, 0));

    // Add multiple valid points around position. (4,4)
    m_cvTestPointcloud.at<cv::Vec4f>(4, 4) = cv::Vec4f(1.5f, 0.5f, 4.0f, 1.0f);
    m_cvTestPointcloud.at<cv::Vec4f>(4, 5) = cv::Vec4f(1.3f, 0.6f, 3.8f, 1.0f);
    m_cvTestPointcloud.at<cv::Vec4f>(5, 4) = cv::Vec4f(1.7f, 0.4f, 4.2f, 1.0f);

    // Test with point (4,4) with neighborhood size 3.
    cv::Point cvPixel(4, 4);

    // Call the function.
    geoops::Waypoint stWaypoint = geoloc::GeolocateBox(m_cvTestPointcloud, m_stRoverPose, cvPixel, 3);

    // Get the UTM coordinate from the waypoint.
    const geoops::UTMCoordinate& stResultUTM = stWaypoint.GetUTMCoordinate();

    // Updating expected values based on the actual implementation:
    // 20th percentile of {3.8, 4.0, 4.2} is 3.8. Tolerance is 0.5, so all are included.
    // X avg = (1.5 + 1.3 + 1.7)/3 = 1.5
    // Y avg = (0.5 + 0.6 + 0.4)/3 = 0.5
    // Z avg = (4.0 + 3.8 + 4.2)/3 = 4.0

    // dAdjustedHeading = 90
    // dHeadingRad = PI/2
    // dEasting = 500000 + (4.0 * cos(PI/2) + 1.5 * sin(PI/2)) = 500000 + 1.5 = 500001.5
    // dNorthing = 4000000 + (4.0 * sin(PI/2) - 1.5 * cos(PI/2)) = 4000000 + 4.0 = 4000004.0
    // dAltitude = 100 + 0.5 = 100.5

    EXPECT_NEAR(stResultUTM.dEasting, 500001.5, EPSILON);
    EXPECT_NEAR(stResultUTM.dNorthing, 4000004.0, EPSILON);
    EXPECT_NEAR(stResultUTM.dAltitude, 100.5, EPSILON);

    // Check that the result has a non-zero radius.
    EXPECT_GT(stWaypoint.dRadius, 0.0);
}

/******************************************************************************
 * @brief Test GeolocateBox radius calculation with multiple points.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-04-27
 ******************************************************************************/
TEST_F(GeolocateTests, GeolocateBoxRadiusCalculation)
{
    // Create a test scenario with multiple points in different positions.
    // Clear previous data and make a new test cloud.
    m_cvTestPointcloud = cv::Mat(10, 10, CV_32FC4, cv::Scalar(0, 0, 0, 0));

    // Add points with a known spread around. (5,5)
    m_cvTestPointcloud.at<cv::Vec4f>(5, 5) = cv::Vec4f(1.0f, 0.0f, 3.0f, 1.0f);     // Center
    m_cvTestPointcloud.at<cv::Vec4f>(5, 6) = cv::Vec4f(1.1f, 0.1f, 3.1f, 1.0f);     // Point 1
    m_cvTestPointcloud.at<cv::Vec4f>(6, 5) = cv::Vec4f(1.2f, -0.1f, 3.2f, 1.0f);    // Point 2
    m_cvTestPointcloud.at<cv::Vec4f>(5, 4) = cv::Vec4f(0.9f, 0.0f, 2.9f, 1.0f);     // Point 3
    m_cvTestPointcloud.at<cv::Vec4f>(4, 5) = cv::Vec4f(0.8f, 0.1f, 2.8f, 1.0f);     // Point 4

    cv::Point cvPixel(5, 5);

    // Call the function with default neighborhood size.
    geoops::Waypoint stWaypoint = geoloc::GeolocateBox(m_cvTestPointcloud, m_stRoverPose, cvPixel);

    // The radius should be non-zero and proportional to the spread of points.
    EXPECT_GT(stWaypoint.dRadius, 0.0);

    // We can also test that we get the expected average position.
    const geoops::UTMCoordinate& stResultUTM = stWaypoint.GetUTMCoordinate();

    // Calculate the expected averages
    // Z Percentile (0.2 * 5 = 1) -> 2.9.
    // Tolerance is 0.5, so all are valid for inclusion!
    // fAvgX = (1.0f + 1.1f + 1.2f + 0.9f + 0.8f) / 5.0f = 1.0
    // fAvgY = (0.0f + 0.1f - 0.1f + 0.0f + 0.1f) / 5.0f = 0.02
    // fAvgZ = (3.0f + 3.1f + 3.2f + 2.9f + 2.8f) / 5.0f = 3.0

    EXPECT_NEAR(stResultUTM.dEasting, 500001.0, EPSILON);
    EXPECT_NEAR(stResultUTM.dNorthing, 4000003.0, EPSILON);
    EXPECT_NEAR(stResultUTM.dAltitude, 100.02, EPSILON);
}
