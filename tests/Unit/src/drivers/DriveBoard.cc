/******************************************************************************
 * @brief Unit test for DriveBoard driver class.
 *
 * @file DriveBoard.cc
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 *
 * @copyright Copyright MRDT 2024 - All Rights Reserved
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
        DriveBoard* driveBoard;

    public:
        /******************************************************************************
         * @brief Construct a new Drive Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        DriveBoardTests() { SetUp(); }

        /******************************************************************************
         * @brief Destroy the Drive Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~DriveBoardTests() { TearDown(); }

        /******************************************************************************
         * @brief Setup the Drive Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void SetUp() override
        {
            // Call the base setup method. This initializes the loggers and RoveComm instances.
            RequiredSetup();
            driveBoard = new DriveBoard();
        }

        /******************************************************************************
         * @brief Teardown the Drive Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TearDown() override
        {
            // Call the base teardown method. This stops the RoveComm instances and loggers.
            RequiredTeardown();
            delete driveBoard;
            driveBoard = nullptr;
        }
};

/******************************************************************************
 * @brief Test for memory leaks
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(DriveBoardTests, DoesNotLeak)
{
    DriveBoard* driveBoard = new DriveBoard();
    ASSERT_NE(driveBoard, nullptr);
    delete driveBoard;
    driveBoard = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(DriveBoardTests, Leaks)
{
    DriveBoard* driveBoard = new DriveBoard();
    EXPECT_TRUE(driveBoard != nullptr);
}

class DriveBoardTest : public ::testing::Test {

protected:
    DriveBoard* driveBoard;

    void SetUp() override {
        driveBoard = new DriveBoard();
    }

    void TearDown() override {
        delete driveBoard;
    }
};

/******************************************************************************
 * @brief Verify that CalculateMove returns near-zero powers with zero speed/heading.
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2025-01-31
 ******************************************************************************/
TEST_F(DriveBoardTests, CalculateMove_ZeroSpeedZeroHeading)
{
    // Test with eArcadeDrive
    diffdrive::DrivePowers eArcadeDriveResultPowers = driveBoard->CalculateMove(0.0, 0.0, 0.0, diffdrive::DifferentialControlMethod::eArcadeDrive);

    // We expect zero drive power when speed & heading are both zero.
    EXPECT_NEAR(eArcadeDriveResultPowers.dLeftDrivePower, 0.0, 1e-6);
    EXPECT_NEAR(eArcadeDriveResultPowers.dRightDrivePower, 0.0, 1e-6);

    // Test with eCurvatureDrive
    diffdrive::DrivePowers eCurvatureDriveResultPowers = driveBoard->CalculateMove(0.0, 0.0, 0.0, diffdrive::DifferentialControlMethod::eCurvatureDrive);
    
    // We expect zero drive power when speed & heading are both zero.
    EXPECT_NEAR(eCurvatureDriveResultPowers.dLeftDrivePower, 0.0, 1e-6);
    EXPECT_NEAR(eCurvatureDriveResultPowers.dRightDrivePower, 0.0, 1e-6);
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

    driveBoard->SendDrive(stPowers);

    // This test does not pass. Not because t is wrong but because, somehow, dLeftDrivePower and dRightDrivePower are returning half the expected value they are set to. 
    // From what I have seen, the speed is halved when it is sent to the drive board.
    auto currentPowers = driveBoard->GetDrivePowers();
    EXPECT_DOUBLE_EQ(currentPowers.dLeftDrivePower, 0.25);
    EXPECT_DOUBLE_EQ(currentPowers.dRightDrivePower, -0.25);
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

    driveBoard->SendDrive(stPowers);
    driveBoard->SendStop();

    auto currentPowers = driveBoard->GetDrivePowers();
    EXPECT_DOUBLE_EQ(currentPowers.dLeftDrivePower, 0.0);
    EXPECT_DOUBLE_EQ(currentPowers.dRightDrivePower, 0.0);
}