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
#include "../../../TestingBase.hh"

/// \cond
#include <chrono>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <shared_mutex>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the NavigationBoard
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class NavigationBoardTests : public TestingBase<NavigationBoardTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.
        NavigationBoard* navBoard;

    public:
        /******************************************************************************
         * @brief Construct a new Navigation Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        NavigationBoardTests() { SetUp(); }

        /******************************************************************************
         * @brief Destroy the Navigation Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~NavigationBoardTests() { TearDown(); }

        /******************************************************************************
         * @brief Setup the Navigation Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void SetUp() override
        {
            // Call the base setup method. This initializes the loggers and RoveComm instances.
            RequiredSetup();
            navBoard = new NavigationBoard();
        }

        /******************************************************************************
         * @brief Teardown the Navigation Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TearDown() override
        {
            // Call the base teardown method. This stops the RoveComm instances and loggers.
            RequiredTeardown();
            delete navBoard;
            navBoard = nullptr;
        }
};

/******************************************************************************
 * @brief Test for memory leaks
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(NavigationBoardTests, DoesNotLeak)
{
    NavigationBoard* testBoard = new NavigationBoard();
    ASSERT_NE(testBoard, nullptr);
    delete testBoard;
    testBoard = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(NavigationBoardTests, Leaks)
{
    NavigationBoard* testBoard = new NavigationBoard();
    EXPECT_NE(testBoard, nullptr);
    // Intentionally not deleting to test leak detection
}

/******************************************************************************
 * @brief Test that the constructor initializes the members correctly
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-12-02
 ******************************************************************************/
TEST_F(NavigationBoardTests, ConstructorInitializesMembers) 
{
    // The latitude, longitude, and altitude are set to the location of Missouri S&T
    EXPECT_EQ(navBoard->GetGPSData().dLatitude, 37.951771);
    EXPECT_EQ(navBoard->GetGPSData().dLongitude, -91.778114);
    EXPECT_EQ(navBoard->GetGPSData().dAltitude, 315.0);
    EXPECT_EQ(navBoard->GetHeading(), 0);
    EXPECT_EQ(navBoard->GetHeadingAccuracy(), 0);
    // Not moving or rotating
    EXPECT_EQ(navBoard->GetVelocity(), 0);
    EXPECT_EQ(navBoard->GetAngularVelocity(), 0);
    // Allow for small time difference due to construction
    EXPECT_LE(navBoard->GetGPSLastUpdateTime(), std::chrono::seconds(1));
    EXPECT_LE(navBoard->GetCompassLastUpdateTime(), std::chrono::seconds(1));
    EXPECT_FALSE(navBoard->IsOutOfDate());
}

/******************************************************************************
 * @brief Test that GetGPSData returns correct data
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-12-02
 ******************************************************************************/
TEST_F(NavigationBoardTests, GetGPSDataReturnsCorrectData) 
{
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
}

/******************************************************************************
 * @brief Test that GetUTMData returns correct data
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-12-02
 ******************************************************************************/
TEST_F(NavigationBoardTests, GetUTMDataReturnsCorrectData) {

    geoops::UTMCoordinate utmData = navBoard->GetUTMData();

    // Assuming default UTM data is MST's location
    EXPECT_NEAR(utmData.dEasting, 607350.55, 0.01);
    EXPECT_NEAR(utmData.dNorthing, 4201167.97, 0.01);
    EXPECT_EQ(utmData.dAltitude, 315.0);
    EXPECT_EQ(utmData.nZone, 15);
    EXPECT_EQ(utmData.bWithinNorthernHemisphere, true);
    EXPECT_EQ(utmData.d2DAccuracy, -1);
    EXPECT_EQ(utmData.d3DAccuracy, -1);
    // IDK why these are what they are
    EXPECT_EQ(utmData.dMeridianConvergence, 0.75152911093843622);
    EXPECT_EQ(utmData.dScale, 0.99974193500083242);
    EXPECT_EQ(utmData.eCoordinateAccuracyFixType, geoops::PositionFixType::eUNKNOWN);
    EXPECT_EQ(utmData.bIsDifferential, false);
}