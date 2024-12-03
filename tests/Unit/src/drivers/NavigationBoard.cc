/******************************************************************************
 * @brief Unit test for NavigationBoard driver class.
 *
 * @file NavigationBoard.cc
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 *
 * @copyright Copyright MRDT 2024 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/drivers/NavigationBoard.h"

/// \cond
#include <chrono>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <shared_mutex>

/// \endcond

/******************************************************************************
 * @brief Test for memory leaks
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST(NavigationBoardTest, DoesNotLeak)
{
    NavigationBoard* navBoard = new NavigationBoard();
    ASSERT_NE(navBoard, nullptr);
    delete navBoard;
    navBoard = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST(NavigationBoardTest, Leaks)
{
    NavigationBoard* navBoard = new NavigationBoard();
    EXPECT_TRUE(navBoard != nullptr);
}

/******************************************************************************
 * @brief Mock class for NavigationBoard
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
// Can only use with TEST_F. Since we are not using TEST_F, we can't use this.
// class NavigationBoardTest : public ::testing::Test {
// protected:
//     NavigationBoard* navBoard;

//     void SetUp() override {
//         navBoard = new NavigationBoard();
//     }

//     void TearDown() override {
//         delete navBoard;
//     }
// };

/******************************************************************************
 * @brief Test that the constructor initializes the members correctly
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-12-02
 ******************************************************************************/
TEST(NavigationBoardTest, ConstructorInitializesMembers) {
    // Initialize the NavigationBoard
    NavigationBoard* navBoard = new NavigationBoard();

    EXPECT_EQ(navBoard->GetGPSData().dLatitude, 37.951771);
    EXPECT_EQ(navBoard->GetGPSData().dLongitude, -91.778114);
    EXPECT_EQ(navBoard->GetGPSData().dAltitude, 315.0);
    EXPECT_EQ(navBoard->GetHeading(), 0);
    EXPECT_EQ(navBoard->GetHeadingAccuracy(), 0);
    EXPECT_EQ(navBoard->GetVelocity(), 0);
    EXPECT_EQ(navBoard->GetAngularVelocity(), 0);
    // Allow for small time difference due to construction
    EXPECT_LE(navBoard->GetGPSLastUpdateTime(), std::chrono::seconds(1));
    EXPECT_LE(navBoard->GetCompassLastUpdateTime(), std::chrono::seconds(1));
    EXPECT_FALSE(navBoard->IsOutOfDate());

    // Destroy the NavigationBoard
    delete navBoard;
}

/******************************************************************************
 * @brief Test that GetGPSData returns correct data
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-12-02
 ******************************************************************************/
TEST(NavigationBoardTest, GetGPSDataReturnsCorrectData) {
    // Initialize the NavigationBoard
    NavigationBoard* navBoard = new NavigationBoard();

    geoops::GPSCoordinate gpsData = navBoard->GetGPSData();
    EXPECT_EQ(gpsData.dLatitude, 37.951771);
    EXPECT_EQ(gpsData.dLongitude, -91.778114);
    EXPECT_EQ(gpsData.dAltitude, 315.0);
    EXPECT_EQ(gpsData.d2DAccuracy, -1);
    EXPECT_EQ(gpsData.d3DAccuracy, -1);
    EXPECT_EQ(gpsData.dMeridianConvergence, -1);
    EXPECT_EQ(gpsData.dScale, 0);
    EXPECT_EQ(gpsData.eCoordinateAccuracyFixType, geoops::PositionFixType::eUNKNOWN);
    EXPECT_EQ(gpsData.bIsDifferential, false);

    // Destroy the NavigationBoard
    delete navBoard;
}

/******************************************************************************
 * @brief Test that GetUTMData returns correct data
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-12-02
 ******************************************************************************/
TEST(NavigationBoardTest, GetUTMDataReturnsCorrectData) {
    // Initialize the NavigationBoard
    NavigationBoard* navBoard = new NavigationBoard();

    geoops::UTMCoordinate utmData = navBoard->GetUTMData();

    // Assuming default UTM data is MST's location
    EXPECT_NEAR(utmData.dEasting, 607350.55, 0.01);
    EXPECT_NEAR(utmData.dNorthing, 4201167.97, 0.01);
    EXPECT_EQ(utmData.dAltitude, 315.0);
    EXPECT_EQ(utmData.nZone, 15);
    EXPECT_EQ(utmData.bWithinNorthernHemisphere, true);
    EXPECT_EQ(utmData.d2DAccuracy, -1);
    EXPECT_EQ(utmData.d3DAccuracy, -1);
    EXPECT_EQ(utmData.dMeridianConvergence, 0.75152911093843622);
    EXPECT_EQ(utmData.dScale, 0.99974193500083242);
    EXPECT_EQ(utmData.eCoordinateAccuracyFixType, geoops::PositionFixType::eUNKNOWN);
    EXPECT_EQ(utmData.bIsDifferential, false);

    // Destroy the NavigationBoard
    delete navBoard;
}