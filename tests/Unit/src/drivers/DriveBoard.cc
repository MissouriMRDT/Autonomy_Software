/******************************************************************************
 * @brief Unit test for DriveBoard driver class.
 *
 * @file DriveBoard.cc
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/drivers/DriveBoard.h"
#include "../../../TestingBase.hh"

/// \cond
#include "../../../../external/rovecomm/src/RoveComm/RoveComm.h"
#include "../../../../external/rovecomm/src/RoveComm/RoveCommManifest.h"
#include "../../../../external/rovecomm/src/RoveComm/RoveCommUDP.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the DriveBoard
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class DriveBoardTests : public TestingBase<DriveBoardTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.
        std::unique_ptr<DriveBoard> m_pDriveBoard;

    public:
        /******************************************************************************
         * @brief Construct a new Drive Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        DriveBoardTests() {}

        /******************************************************************************
         * @brief Destroy the Drive Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~DriveBoardTests() {}

        /******************************************************************************
         * @brief Setup the Drive Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestSetup() override { m_pDriveBoard = std::make_unique<DriveBoard>(); }

        /******************************************************************************
         * @brief Teardown the Drive Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Test for memory leaks
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(DriveBoardTests, DoesNotLeak)
{
    DriveBoard* pDriveBoard = new DriveBoard();
    ASSERT_NE(pDriveBoard, nullptr);
    delete pDriveBoard;
    pDriveBoard = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(DriveBoardTests, Leaks)
{
    DriveBoard* pDriveBoard = new DriveBoard();
    EXPECT_TRUE(pDriveBoard != nullptr);
}

/******************************************************************************
 * @brief Verify that CalculateMove returns near-zero powers with zero speed/heading.
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2025-01-31
 ******************************************************************************/
TEST_F(DriveBoardTests, CalculateMove_ZeroSpeedZeroHeading)
{
    // Test with eArcadeDrive
    diffdrive::DrivePowers stArcadeDriveResultPowers = m_pDriveBoard->CalculateMove(0.0, 0.0, 0.0, diffdrive::DifferentialControlMethod::eArcadeDrive);

    // We expect zero drive power when speed & heading are both zero.
    EXPECT_NEAR(stArcadeDriveResultPowers.dLeftDrivePower, 0.0, 1e-6);
    EXPECT_NEAR(stArcadeDriveResultPowers.dRightDrivePower, 0.0, 1e-6);

    // Test with eCurvatureDrive
    diffdrive::DrivePowers stCurvatureDriveResultPowers = m_pDriveBoard->CalculateMove(0.0, 0.0, 0.0, diffdrive::DifferentialControlMethod::eCurvatureDrive);

    // We expect zero drive power when speed & heading are both zero.
    EXPECT_NEAR(stCurvatureDriveResultPowers.dLeftDrivePower, 0.0, 1e-6);
    EXPECT_NEAR(stCurvatureDriveResultPowers.dRightDrivePower, 0.0, 1e-6);
}

/******************************************************************************
 * @brief Verify that SendDrive sets DrivePowers and GetDrivePowers matches them.
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2025-01-31
 ******************************************************************************/
TEST_F(DriveBoardTests, SendDrive_UpdatesDrivePowers)
{
    diffdrive::DrivePowers stPowers;
    stPowers.dLeftDrivePower  = 0.5;
    stPowers.dRightDrivePower = -0.5;

    m_pDriveBoard->SetMaxDriveEffort(1.0);
    m_pDriveBoard->SendDrive(stPowers);

    // The output drive powers will changed depending on what the drive board effort limits are.
    diffdrive::DrivePowers stCurrentPowers = m_pDriveBoard->GetDrivePowers();
    EXPECT_DOUBLE_EQ(stCurrentPowers.dLeftDrivePower, 0.5);
    EXPECT_DOUBLE_EQ(stCurrentPowers.dRightDrivePower, -0.5);
}

/******************************************************************************
 * @brief Verify that calling SendStop sets the powers to zero.
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2025-01-31
 ******************************************************************************/
TEST_F(DriveBoardTests, SendStop_StopsTheDrive)
{
    diffdrive::DrivePowers stPowers;
    stPowers.dLeftDrivePower  = 1.0;
    stPowers.dRightDrivePower = 1.0;

    m_pDriveBoard->SendDrive(stPowers);
    m_pDriveBoard->SendStop();

    diffdrive::DrivePowers stCurrentPowers = m_pDriveBoard->GetDrivePowers();
    EXPECT_DOUBLE_EQ(stCurrentPowers.dLeftDrivePower, 0.0);
    EXPECT_DOUBLE_EQ(stCurrentPowers.dRightDrivePower, 0.0);
}
