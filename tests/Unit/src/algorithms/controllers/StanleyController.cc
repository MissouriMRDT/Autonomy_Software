/******************************************************************************
 * @brief Unit Test Suite for the Stanley Controller
 *
 * @file StanleyController.cc
 * @author Jason Pittman (jspencerpittman@gmail.com)
 * @date 2024-02-16
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/algorithms/controllers/StanleyController.h"

/// \cond
#include <gtest/gtest.h>

/// \endcond

TEST(StanleyControllerUnitTests, TestCalculate)
{
    // Define Stanley controller parameters
    double dKp              = 0.5;
    double dDistToFrontAxle = 2.9;
    double dYawTolerance    = 1.0;

    // Define test path
    std::vector<geoops::UTMCoordinate> vPathUtm = {{0, 0}, {0, 10}, {5, 20}, {10, 30}, {20, 35}};

    // Initialize controller
    controllers::StanleyController stController(vPathUtm, dKp, dDistToFrontAxle, dYawTolerance);

    // Define agent's state.
    geoops::UTMCoordinate stCurrentPosUTM(1, 0);
    double dVelocity = 1.0;
    double dBearing  = 30;

    // Variables to verify accuracy.
    double dTargetBearing;
    unsigned int unTargetIdx;

    // Test #01
    dTargetBearing = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx    = stController.GetLastTargetIdx();

    EXPECT_NEAR(dTargetBearing, 359.591, 1);
    EXPECT_EQ(unTargetIdx, 0);

    // Test #02
    dBearing       = 0;

    dTargetBearing = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx    = stController.GetLastTargetIdx();

    EXPECT_NEAR(dTargetBearing, 360, 1);
    EXPECT_EQ(unTargetIdx, 0);

    // Test #03
    dBearing       = 350;

    dTargetBearing = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx    = stController.GetLastTargetIdx();

    EXPECT_NEAR(dTargetBearing, 359.542, 1);
    EXPECT_EQ(unTargetIdx, 0);

    // Test #04
    dBearing        = 350;
    stCurrentPosUTM = geoops::UTMCoordinate(-1, 1);

    dTargetBearing  = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx     = stController.GetLastTargetIdx();

    EXPECT_NEAR(dTargetBearing, 0.385, 1);
    EXPECT_EQ(unTargetIdx, 0);

    // Test #05
    dBearing        = 90;
    stCurrentPosUTM = geoops::UTMCoordinate(7, 20);

    dTargetBearing  = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx     = stController.GetLastTargetIdx();

    EXPECT_NEAR(dTargetBearing, 26.565, 1);
    ASSERT_EQ(unTargetIdx, 2);

    // Test #06
    stCurrentPosUTM = geoops::UTMCoordinate(1, 0);

    dTargetBearing  = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx     = stController.GetLastTargetIdx();

    EXPECT_NEAR(dTargetBearing, 25.094, 1);
    ASSERT_EQ(unTargetIdx, 2);
}
