/******************************************************************************
 * @brief Main unit test file. Calls for all Unit Tests to be executed.
 *
 * @file main.cc
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../src/AutonomyConstants.h"
#include "../../src/AutonomyLogging.h"

/// \cond
#include <gtest/gtest.h>

/// \endcond

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
    // Setup logging.
    logging::InitializeLoggers(constants::LOGGING_OUTPUT_PATH_ABSOLUTE);

    // Initialize tests.
    testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}
