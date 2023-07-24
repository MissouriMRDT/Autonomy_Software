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
TEST(TestSuite, TestBool)
{
    const auto expected = true;
    const auto actual   = true;
    ASSERT_EQ(expected, actual);
}
