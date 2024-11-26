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

// /******************************************************************************
//  * @brief Mock class for NavigationBoard
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
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

// /******************************************************************************
//  * @brief Test that the constructor initializes the members correctly
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(NavigationBoardTest, ConstructorInitializesMembers) {
//     EXPECT_EQ(navBoard->GetGPSData().dLatitude, 0);
//     EXPECT_EQ(navBoard->GetGPSData().dLongitude, 0);
//     EXPECT_EQ(navBoard->GetGPSData().dAltitude, 0);
//     EXPECT_EQ(navBoard->GetHeading(), 0);
//     EXPECT_EQ(navBoard->GetHeadingAccuracy(), 0);
//     EXPECT_EQ(navBoard->GetVelocity(), 0);
//     EXPECT_EQ(navBoard->GetAngularVelocity(), 0);
//     EXPECT_EQ(navBoard->GetGPSLastUpdateTime(), std::chrono::system_clock::duration::zero());
//     EXPECT_EQ(navBoard->GetCompassLastUpdateTime(), std::chrono::system_clock::duration::zero());
//     EXPECT_FALSE(navBoard->IsOutOfDate());
// }

// /******************************************************************************
//  * @brief Test that GetGPSData returns correct data
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(NavigationBoardTest, GetGPSDataReturnsCorrectData) {
//     geoops::GPSCoordinate gpsData = navBoard->GetGPSData();
//     EXPECT_EQ(gpsData.dLatitude, 0);
//     EXPECT_EQ(gpsData.dLongitude, 0);
//     EXPECT_EQ(gpsData.dAltitude, 0);
// }

// /******************************************************************************
//  * @brief Test that GetUTMData returns correct data
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(NavigationBoardTest, GetUTMDataReturnsCorrectData) {
//     geoops::UTMCoordinate utmData = navBoard->GetUTMData();
//     // Assuming default UTM data is zero-initialized
//     EXPECT_EQ(utmData.dEasting, 0);
//     EXPECT_EQ(utmData.dNorthing, 0);
//     EXPECT_EQ(utmData.iZone, 0);
// }

// /******************************************************************************
//  * @brief Test that GetHeading returns correct data
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(NavigationBoardTest, GetHeadingReturnsCorrectData) {
//     EXPECT_EQ(navBoard->GetHeading(), 0);
// }

// /******************************************************************************
//  * @brief Test that GetHeadingAccuracy returns correct data
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(NavigationBoardTest, GetHeadingAccuracyReturnsCorrectData) {
//     EXPECT_EQ(navBoard->GetHeadingAccuracy(), 0);
// }

// /******************************************************************************
//  * @brief Test that GetVelocity returns correct data
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(NavigationBoardTest, GetVelocityReturnsCorrectData) {
//     EXPECT_EQ(navBoard->GetVelocity(), 0);
// }

// /******************************************************************************
//  * @brief Test that GetAngularVelocity returns correct data
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(NavigationBoardTest, GetAngularVelocityReturnsCorrectData) {
//     EXPECT_EQ(navBoard->GetAngularVelocity(), 0);
// }

// /******************************************************************************
//  * @brief Test that GetGPSLastUpdateTime returns correct data
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(NavigationBoardTest, GetGPSLastUpdateTimeReturnsCorrectData) {
//     EXPECT_EQ(navBoard->GetGPSLastUpdateTime(), std::chrono::system_clock::duration::zero());
// }

// /******************************************************************************
//  * @brief Test that GetCompassLastUpdateTime returns correct data
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(NavigationBoardTest, GetCompassLastUpdateTimeReturnsCorrectData) {
//     EXPECT_EQ(navBoard->GetCompassLastUpdateTime(), std::chrono::system_clock::duration::zero());
// }

// /******************************************************************************
//  * @brief Test that IsOutOfDate returns correct data
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(NavigationBoardTest, IsOutOfDateReturnsCorrectData) {
//     EXPECT_FALSE(navBoard->IsOutOfDate());
// }
