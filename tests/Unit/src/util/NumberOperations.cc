/******************************************************************************
 * @brief Unit test for IPS utility class.
 *
 * @file IPS.cc
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
 * @brief Test the funtionality of the reset method.
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
