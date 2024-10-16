/******************************************************************************
 * @brief Main integration test file. Calls for all Integration Tests to be executed.
 *
 * @file main.cc
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

/// \cond
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Integration Tests - Main Function
 *
 * @return int - Integration Tests Exit Status
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 ******************************************************************************/
int main()
{
    ::testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}
