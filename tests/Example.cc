/******************************************************************************
 * @brief Unit/Integration Test Suite Example
 *
 * @file Example.cc
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

/// \cond
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Test some basic multiplication
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 ******************************************************************************/
TEST(ExampleTests, TestIntegerOne_One)
{
    const int expected = 18;
    const int actual   = 6 * 3;
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
    const int expected = 9;
    const int actual   = 6 + 3;
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
    const int expected = 3;
    const int actual   = 6 - 3;
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
    const int expected = 2;
    const int actual   = 6 / 3;
    ASSERT_EQ(actual, expected);
}
