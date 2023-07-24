/******************************************************************************
 * @brief Unit/Integration Test Suite Example
 *
 * @file Example.cc
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include <gtest/gtest.h>

/******************************************************************************
 * @brief Test some basic multiplication
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 ******************************************************************************/
TEST(ExampleTests, TestIntegerOne_One)
{
    const auto expected = 18;
    const auto actual   = 6 * 3;
    ASSERT_EQ(expected, actual);
}

/******************************************************************************
 * @brief Test some basic addition
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 ******************************************************************************/
TEST(ExampleTests, TestIntegerZero_Zero)
{
    const auto expected = 9;
    const auto actual   = 6 + 3;
    ASSERT_EQ(expected, actual);
}

/******************************************************************************
 * @brief Test some basic subtraction
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 ******************************************************************************/
TEST(ExampleTests, TestIntegerZero_One)
{
    const auto expected = 3;
    const auto actual   = 6 - 3;
    ASSERT_EQ(actual, expected);
}

/******************************************************************************
 * @brief Test some basic division
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 ******************************************************************************/
TEST(ExampleTests, TestDivision)
{
    const auto expected = 2;
    const auto actual   = 6 / 2;
    ASSERT_EQ(actual, expected);
}
