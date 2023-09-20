/******************************************************************************
 * @brief Main unit test file. Calls for all Unit Tests to be executed.
 *
 * @file main.cc
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include <gtest/gtest.h>

/******************************************************************************
 * @brief Unit Tests - Main Function
 *
 * @return int - Unit Tests Exit Status
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 ******************************************************************************/
int main()
{
    testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}
