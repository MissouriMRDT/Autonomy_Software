/******************************************************************************
 * @brief Unit test for NumberOperations utility class.
 *
 * @file NumberOperations.cc
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/util/NumberOperations.hpp"

/// \cond
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

/// \endcond

/******************************************************************************
 * @brief Test the functionality of the Clamp function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-16
 ******************************************************************************/
TEST(NumOpsTest, Clamp)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength               = 6;
    const double aValues[nTestValuesLength]   = {1.0, 0.0, 567.0, 0.05, -1.0, -89.3};
    const double aMinimums[nTestValuesLength] = {0.0, 0.0, -5.0, 0.05, -3.0, 50.0};
    const double aMaximums[nTestValuesLength] = {2.0, 1.0, 0.0, 0.04, 0.0, -100.0};
    const double aOutput[nTestValuesLength]   = {1.0, 0.0, 0.0, 0.05, -1.0, 50.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate clamp values.
        double dResult = numops::Clamp(aValues[nIter], aMinimums[nIter], aMaximums[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_EQ(dResult, aOutput[nIter]);    // output check.
    }
}

/******************************************************************************
 * @brief Test the functionality of the Bounded function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-16
 ******************************************************************************/
TEST(NumOpsTest, Bounded)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength               = 6;
    const double aValues[nTestValuesLength]   = {1.0, 0.0, 567.0, 0.05, -1.0, -89.3};
    const double aMinimums[nTestValuesLength] = {0.0, 0.0, -5.0, 0.05, -3.0, 50.0};
    const double aMaximums[nTestValuesLength] = {2.0, 1.0, 0.0, 0.04, 0.0, -100.0};
    const bool aOutput[nTestValuesLength]     = {true, true, false, false, true, false};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate valid bounds.
        bool bResult = numops::Bounded(aValues[nIter], aMinimums[nIter], aMaximums[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_EQ(bResult, aOutput[nIter]);    // output check.
    }
}

/******************************************************************************
 * @brief Test the functionality of the MapRange function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
TEST(NumOpsTest, MapRange)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength                  = 4;
    const double aValues[nTestValuesLength]      = {0.0, -1.0, 2.0, 0};
    const double aOldMinimums[nTestValuesLength] = {-1.0, -1.0, -4.0, -180};
    const double aOldMaximums[nTestValuesLength] = {1.0, 1.0, 4.0, 180};
    const double aNewMinimums[nTestValuesLength] = {0.0, 0.0, -2.0, 0};
    const double aNewMaximums[nTestValuesLength] = {2.0, 2.0, 2.0, 360};
    const double aOutput[nTestValuesLength]      = {1.0, 0.0, 1.0, 180};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate valid bounds.
        double dResult = numops::MapRange(aValues[nIter], aOldMinimums[nIter], aOldMaximums[nIter], aNewMinimums[nIter], aNewMaximums[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_EQ(dResult, aOutput[nIter]);
    }
}

/******************************************************************************
 * @brief Test the functionality of the InputAngleModulus function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-19
 ******************************************************************************/
TEST(NumOpsTest, InputAngleModulus)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength               = 6;
    const double aValues[nTestValuesLength]   = {1.0, -1.0, 4.0, 360.0, 350.0, 170.0};
    const double aMinimums[nTestValuesLength] = {0.0, 0.0, -2.0, -180.0, -180.0, -180.0};
    const double aMaximums[nTestValuesLength] = {2.0, 2.0, 2.0, 180.0, 180.0, 180.0};
    const double aOutput[nTestValuesLength]   = {1.0, 1.0, 0.0, 0.0, -10.0, 170.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate valid bounds.
        double dResult = numops::InputAngleModulus(aValues[nIter], aMinimums[nIter], aMaximums[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_EQ(dResult, aOutput[nIter]);
    }
}

/******************************************************************************
 * @brief Test the functionality of the AngularDifference function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2024-04-03
 ******************************************************************************/
TEST(NumOpsTest, AngularDifference)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength                   = 8;
    const double aFirstValues[nTestValuesLength]  = {0.0, 330.0, 30.0, 270.0, 60.0, 0.0, 170.0, 60.0};
    const double aSecondValues[nTestValuesLength] = {0.0, 30.0, 330.0, 180.0, 120.0, 360.0, 190.0, 90.0};
    const double aOutput[nTestValuesLength]       = {0.0, 60.0, -60.0, 90.0, 60.0, 0.0, 20.0, 30.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate valid bounds.
        double dResult = numops::AngularDifference(aFirstValues[nIter], aSecondValues[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_EQ(dResult, aOutput[nIter]);
    }
}
