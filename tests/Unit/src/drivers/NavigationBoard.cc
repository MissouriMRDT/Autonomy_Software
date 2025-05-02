/******************************************************************************
 * @brief Unit test for NavigationBoard driver class.
 *
 * @file NavigationBoard.cc
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
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
        std::unique_ptr<NavigationBoard> m_pNavBoard;

    public:
        /******************************************************************************
         * @brief Construct a new Navigation Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        NavigationBoardTests() {}

        /******************************************************************************
         * @brief Destroy the Navigation Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~NavigationBoardTests() {}

        /******************************************************************************
         * @brief Setup the Navigation Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestSetup() override { m_pNavBoard = std::make_unique<NavigationBoard>(); }

        /******************************************************************************
         * @brief Teardown the Navigation Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestTeardown() override {}
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
    NavigationBoard* pNavBoard = new NavigationBoard();
    ASSERT_NE(pNavBoard, nullptr);
    delete pNavBoard;
    pNavBoard = nullptr;
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
    NavigationBoard* pNavBoard = new NavigationBoard();
    EXPECT_NE(pNavBoard, nullptr);
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
    EXPECT_EQ(m_pNavBoard->GetGPSData().dLatitude, 37.951771);
    EXPECT_EQ(m_pNavBoard->GetGPSData().dLongitude, -91.778114);
    EXPECT_EQ(m_pNavBoard->GetGPSData().dAltitude, 315.0);
    EXPECT_EQ(m_pNavBoard->GetHeading(), 0);
    EXPECT_EQ(m_pNavBoard->GetHeadingAccuracy(), 0);
    // Not moving or rotating
    EXPECT_EQ(m_pNavBoard->GetVelocity(), 0);
    EXPECT_EQ(m_pNavBoard->GetAngularVelocity(), 0);
    // Allow for small time difference due to construction
    EXPECT_LE(m_pNavBoard->GetGPSLastUpdateTime(), std::chrono::seconds(1));
    EXPECT_LE(m_pNavBoard->GetCompassLastUpdateTime(), std::chrono::seconds(1));
    EXPECT_FALSE(m_pNavBoard->IsOutOfDate());
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
    geoops::GPSCoordinate stGPSData = m_pNavBoard->GetGPSData();
    EXPECT_EQ(stGPSData.dLatitude, 37.951771);
    EXPECT_EQ(stGPSData.dLongitude, -91.778114);
    EXPECT_EQ(stGPSData.dAltitude, 315.0);
    EXPECT_EQ(stGPSData.d2DAccuracy, -1);
    EXPECT_EQ(stGPSData.d3DAccuracy, -1);
    EXPECT_EQ(stGPSData.dMeridianConvergence, -1);
    EXPECT_EQ(stGPSData.dScale, 0);
    EXPECT_EQ(stGPSData.eCoordinateAccuracyFixType, geoops::PositionFixType::eUNKNOWN);
    EXPECT_EQ(stGPSData.bIsDifferential, false);
}

/******************************************************************************
 * @brief Test that GetUTMData returns correct data
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-12-02
 ******************************************************************************/
TEST_F(NavigationBoardTests, GetUTMDataReturnsCorrectData)
{
    geoops::UTMCoordinate stUTMData = m_pNavBoard->GetUTMData();

    // Assuming default UTM data is MST's location
    EXPECT_NEAR(stUTMData.dEasting, 607350.55, 0.01);
    EXPECT_NEAR(stUTMData.dNorthing, 4201167.97, 0.01);
    EXPECT_EQ(stUTMData.dAltitude, 315.0);
    EXPECT_EQ(stUTMData.nZone, 15);
    EXPECT_EQ(stUTMData.bWithinNorthernHemisphere, true);
    EXPECT_EQ(stUTMData.d2DAccuracy, -1);
    EXPECT_EQ(stUTMData.d3DAccuracy, -1);
    // IDK why these are what they are
    EXPECT_EQ(stUTMData.dMeridianConvergence, 0.75152911093843622);
    EXPECT_EQ(stUTMData.dScale, 0.99974193500083242);
    EXPECT_EQ(stUTMData.eCoordinateAccuracyFixType, geoops::PositionFixType::eUNKNOWN);
    EXPECT_EQ(stUTMData.bIsDifferential, false);
}
