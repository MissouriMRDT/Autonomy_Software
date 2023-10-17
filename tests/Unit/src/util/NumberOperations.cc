/******************************************************************************
 * @brief Unit test for NumberOperations utility class.
 *
 * @file NumberOperations.cc
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include <chrono>
#include <gtest/gtest.h>
#include <thread>

#include "../../../../src/util/NumberOperations.hpp"

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
    // Declare ranges and expected results.
    double dOldMin           = -1.0;
    double dOldMax           = 1.0;
    double dNewMin           = 0;
    double dNewMax           = 10;
    double dOldValue         = 0.0;
    double dExpectedNewValue = 5.0;

    // Remap new value.
    double dActualValue = numops::MapRange(dOldValue, dOldMin, dOldMax, dNewMin, dNewMax);

    // Get the calculated values and check that they are correct.
    EXPECT_NE(dActualValue, 0);
    EXPECT_EQ(dActualValue, 5.0);

    // Check invalid input on old range.
    dOldMin = 1.0;
    // Remap new value.
    dActualValue = numops::MapRange(dOldValue, dOldMin, dOldMax, dNewMin, dNewMax);
    // Make sure original value is returned.
    EXPECT_EQ(dActualValue, dOldValue);

    // Check invalid input on new range.
    dOldMin = -1.0;
    dNewMin = 10;
    // Remap new value.
    dActualValue = numops::MapRange(dOldValue, dOldMin, dOldMax, dNewMin, dNewMax);
    // Make sure original value is returned.
    EXPECT_EQ(dActualValue, dOldValue);
}
