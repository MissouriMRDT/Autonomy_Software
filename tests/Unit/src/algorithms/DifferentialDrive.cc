/******************************************************************************
 * @brief Unit test for DifferentialDrive algorithm class.
 *
 * @file DifferentialDrive.cc
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/algorithms/DifferentialDrive.hpp"
#include "../../../TestingBase.hh"

/// \cond
#include <array>
#include <chrono>
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the DifferentialDrive Planner
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class DifferentialDriveTests : public TestingBase<DifferentialDriveTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new Differential Drive Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        DifferentialDriveTests() { SetUp(); }

        /******************************************************************************
         * @brief Destroy the Differential Drive Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~DifferentialDriveTests() { TearDown(); }

        /******************************************************************************
         * @brief Setup the Differential Drive Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void SetUp() override
        {
            // Call the base setup method. This initializes the loggers and RoveComm instances.
            RequiredSetup();
        }

        /******************************************************************************
         * @brief Teardown the Differential Drive Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TearDown() override
        {
            // Call the base teardown method. This stops the RoveComm instances and loggers.
            RequiredTeardown();
        }
};

/******************************************************************************
 * @brief Test DifferentialDrive TankDrive functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 ******************************************************************************/
TEST_F(DifferentialDriveTests, TankDrive)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength                       = 11;
    const double aLeftSpeedInput[nTestValuesLength]   = {-1.0, -0.5, 0.0, 0.5, 1.0, 1.0, 0.5, 0.0, -0.5, -1.0, 1.5};
    const double aRightSpeedInput[nTestValuesLength]  = {-1.0, -0.5, 0.0, 0.5, 1.0, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5};
    const double aLeftSpeedOutput[nTestValuesLength]  = {-1.0, -0.5, 0.0, 0.5, 1.0, 1.0, 0.5, 0.0, -0.5, -1.0, 1.0};
    const double aRightSpeedOutput[nTestValuesLength] = {-1.0, -0.5, 0.0, 0.5, 1.0, -1.0, -0.5, 0.0, 0.5, 1.0, 1.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate drive powers.
        diffdrive::DrivePowers stDriveOutput = diffdrive::CalculateTankDrive(aLeftSpeedInput[nIter], aRightSpeedInput[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_NEAR(aLeftSpeedOutput[nIter], stDriveOutput.dLeftDrivePower, 0.01);      // Left output check.
        EXPECT_NEAR(aRightSpeedOutput[nIter], stDriveOutput.dRightDrivePower, 0.01);    // Right output check.
    }
}

/******************************************************************************
 * @brief Test DifferentialDrive ArcadeDrive functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 ******************************************************************************/
TEST_F(DifferentialDriveTests, ArcadeDrive)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength                       = 11;
    const double aSpeedInput[nTestValuesLength]       = {-1.0, -0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.3, 1.5};
    const double aRotationInput[nTestValuesLength]    = {0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.5, 0.5, 1.0, 1.0, 1.5};
    const double aLeftSpeedOutput[nTestValuesLength]  = {-1.0, -0.5, 0.0, 0.5, 1.0, -1.0, -0.5, 0.5, 1.0, 1.0, 1.0};
    const double aRightSpeedOutput[nTestValuesLength] = {-1.0, -0.5, 0.0, 0.5, 1.0, 1.0, 0.5, -0.5, -1.0, -0.53, 0.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate drive powers.
        diffdrive::DrivePowers stDriveOutput = diffdrive::CalculateArcadeDrive(aSpeedInput[nIter], aRotationInput[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_NEAR(aLeftSpeedOutput[nIter], stDriveOutput.dLeftDrivePower, 0.02);      // Left output check.
        EXPECT_NEAR(aRightSpeedOutput[nIter], stDriveOutput.dRightDrivePower, 0.02);    // Right output check.
    }
}

/******************************************************************************
 * @brief Test DifferentialDrive CurvatureDrive functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-13
 ******************************************************************************/
TEST_F(DifferentialDriveTests, CurvatureDrive)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength                          = 11;
    const double aSpeedInput[nTestValuesLength]          = {-1.0, -0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.3, 1.5};
    const double aRotationInput[nTestValuesLength]       = {0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.5, 0.5, 1.0, 1.0, 1.5};
    const bool aAllowTurnInPlaceInput[nTestValuesLength] = {true, true, true, true, true, true, true, true, true, true, true};
    const double aLeftSpeedOutput[nTestValuesLength]     = {-1.0, -0.5, 0.0, 0.5, 1.0, -1.0, -0.5, 0.5, 1.0, 0.59, 1.0};
    const double aRightSpeedOutput[nTestValuesLength]    = {-1.0, -0.5, 0.0, 0.5, 1.0, 1.0, 0.5, -0.5, -1.0, 0.0, 0.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate drive powers.
        diffdrive::DrivePowers stDriveOutput = diffdrive::CalculateCurvatureDrive(aSpeedInput[nIter], aRotationInput[nIter], aAllowTurnInPlaceInput[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_NEAR(aLeftSpeedOutput[nIter], stDriveOutput.dLeftDrivePower, 0.02);      // Left output check.
        EXPECT_NEAR(aRightSpeedOutput[nIter], stDriveOutput.dRightDrivePower, 0.02);    // Right output check.
    }
}
