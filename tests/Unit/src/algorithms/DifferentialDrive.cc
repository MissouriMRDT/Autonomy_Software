/******************************************************************************
 * @brief Unit test for DifferentialDrive algorithm class.
 *
 * @file DifferentialDrive.cc
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include <array>
#include <chrono>
#include <gtest/gtest.h>

#include "../../../../src/algorithms/DifferentialDrive.hpp"

/******************************************************************************
 * @brief Test DifferentialDrive TankDrive functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 ******************************************************************************/
TEST(DifferentialDriveTest, TankDrive)
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
        auto [dLeftDrivePower, dRightDrivePower] = diffdrive::CalculateTankDrive(aLeftSpeedInput[nIter], aRightSpeedInput[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_NEAR(aLeftSpeedOutput[nIter], dLeftDrivePower, 0.01);      // Left output check.
        EXPECT_NEAR(aRightSpeedOutput[nIter], dRightDrivePower, 0.01);    // Right output check.
    }
}

/******************************************************************************
 * @brief Test DifferentialDrive ArcadeDrive functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 ******************************************************************************/
TEST(DifferentialDriveTest, ArcadeDrive)
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
        auto [dLeftDrivePower, dRightDrivePower] = diffdrive::CalculateArcadeDrive(aSpeedInput[nIter], aRotationInput[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_NEAR(aLeftSpeedOutput[nIter], dLeftDrivePower, 0.02);      // Left output check.
        EXPECT_NEAR(aRightSpeedOutput[nIter], dRightDrivePower, 0.02);    // Right output check.
    }
}

/******************************************************************************
 * @brief Test DifferentialDrive CurvatureDrive functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-13
 ******************************************************************************/
TEST(DifferentialDriveTest, CurvatureDrive)
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
        auto [dLeftDrivePower, dRightDrivePower] = diffdrive::CalculateCurvatureDrive(aSpeedInput[nIter], aRotationInput[nIter], aAllowTurnInPlaceInput[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_NEAR(aLeftSpeedOutput[nIter], dLeftDrivePower, 0.02);      // Left output check.
        EXPECT_NEAR(aRightSpeedOutput[nIter], dRightDrivePower, 0.02);    // Right output check.
    }
}
