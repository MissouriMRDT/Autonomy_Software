/******************************************************************************
 * @brief Main test file. Acts as a runner for all Unit and Integration Tests.
 *
 * @file main.cc
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../src/AutonomyConstants.h"
#include "../src/AutonomyLogging.h"

/// \cond
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Unit/Integration Tests Runner - Main Function
 *
 * @return int - Unit/Integration Tests Exit Status
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 ******************************************************************************/
int main(int argc, char** argv)
{
    bool bSkipLogging                                 = false;
    std::vector<std::string> vConditionsToSkipLogging = {"--gtest_filter=AutonomyLoggingTest"};

    // Check if the test being run is in the list of tests to skip setting up logging.
    for (int i = 1; i < argc; ++i)
    {
        for (const std::string& szCondition : vConditionsToSkipLogging)
        {
            if (std::string(argv[i]).find(szCondition) != std::string::npos)
            {
                bSkipLogging = true;
                break;
            }
        }
    }

    // Setup logging if not skipped.
    if (!bSkipLogging)
    {
        logging::InitializeLoggers(constants::LOGGING_OUTPUT_PATH_ABSOLUTE);
    }

    // Initialize tests.
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
