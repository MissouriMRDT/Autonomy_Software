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

/******************************************************************************
 * @brief A predicate to check two angles are with 1 degree of each other at the
 *  angle wraparound point.
 *
 * This predicate is specifically for verifying angles are within each other for angles around 360 or 0 degrees.
 *
 * @param expected - Expected angle (359-360 & 0-1).
 * @param actual - Actual angle (359-360 & 0-1).
 * @return true - Angles are within 1 degree of each other.
 * @return false - Angles are further than 1 degree of each other.
 *
 * @author JSpencerPittman (jspencerpittman@gmail.com)
 * @date 2024-02-16
 ******************************************************************************/
bool AnglesCloseAtWraparound(double expected, double actual)
{
    // How many degrees from the turnaround point?
    if (expected > 180)
        expected = expected - 360;
    if (actual > 180)
        actual = actual - 360;

    // Are they within 1 degree of each other.
    double dist = std::abs(expected - actual);
    return dist <= 1.0;
}

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

    // Test Calculate (Bearing=30, Position=(1,0))
    dTargetBearing = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx    = stController.GetLastTargetIdx();

    EXPECT_PRED2(AnglesCloseAtWraparound, dTargetBearing, 359.591);
    EXPECT_EQ(unTargetIdx, 0);

    // Test Calculate (Bearing=0, Position=(1,0))
    dBearing       = 0;

    dTargetBearing = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx    = stController.GetLastTargetIdx();

    EXPECT_PRED2(AnglesCloseAtWraparound, dTargetBearing, 360);
    EXPECT_EQ(unTargetIdx, 0);

    // Test Calculate (Bearing=350, Position=(1,0))
    dBearing       = 350;

    dTargetBearing = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx    = stController.GetLastTargetIdx();

    EXPECT_PRED2(AnglesCloseAtWraparound, dTargetBearing, 359.542);
    EXPECT_EQ(unTargetIdx, 0);

    // Test opposite side of path (Bearing=350, Position=(-1,1))
    dBearing        = 350;
    stCurrentPosUTM = geoops::UTMCoordinate(-1, 1);

    dTargetBearing  = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx     = stController.GetLastTargetIdx();

    EXPECT_PRED2(AnglesCloseAtWraparound, dTargetBearing, 0.385);
    EXPECT_EQ(unTargetIdx, 0);

    // Test move further along path (Bearing=80, Position=(7,20))
    dBearing        = 80;
    stCurrentPosUTM = geoops::UTMCoordinate(7, 20);

    dTargetBearing  = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx     = stController.GetLastTargetIdx();

    EXPECT_NEAR(dTargetBearing, 26.393, 1);
    ASSERT_EQ(unTargetIdx, 2);

    // Test can't revert to earlier point in path (Bearing=80, Position=(1,0))
    stCurrentPosUTM = geoops::UTMCoordinate(1, 0);

    dTargetBearing  = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx     = stController.GetLastTargetIdx();

    EXPECT_NEAR(dTargetBearing, 25.099, 1);
    ASSERT_EQ(unTargetIdx, 2);

    // Test revert to earlier point in path after reset (Bearing=80, Position=(1,0))
    stController.ResetProgress();

    dTargetBearing = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx    = stController.GetLastTargetIdx();

    EXPECT_PRED2(AnglesCloseAtWraparound, dTargetBearing, 359.913);
    ASSERT_EQ(unTargetIdx, 0);
}
