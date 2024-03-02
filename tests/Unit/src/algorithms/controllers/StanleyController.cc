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

// UTM Coordinates when converted to GPS need both their northing and easting to be in the range [1200km, 2800km].
// This constant allows us shift UTM coordinates into this range by adding it to the UTM coordinate's northing and easting.
// It is representative of 2000km.
#define UTM_SHIFT 2 * std::pow(10, 6)

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
bool AnglesCloseAtWraparound(double dExpected, double dActual)
{
    // How many degrees from the turnaround point?
    if (dExpected > 180)
    {
        dExpected = dExpected - 360;
    }
    if (dActual > 180)
    {
        dActual = dActual - 360;
    }

    // Are they within 1 degree of each other.
    return std::abs(dExpected - dActual) <= 1.0;
}

/******************************************************************************
 * @brief Provide GPS coordinates relative to another GPS coordinate.
 *
 * @note Makes it easier to specify "1 meter to the left" of another point.
 *
 * @param stGPSPoint - Coordinate were measuring relative to.
 * @param dEast - How many meters East of provided GPS point.
 * @param dNorth - How many meters North of provided GPS point.
 * @return geoops::GPSCoordinate - GPS coordinate relative to given GPS coordinate.
 *
 * @author JSpencerPittman (jspencerpittman@gmail.com)
 * @date 2024-02-17
 ******************************************************************************/
geoops::GPSCoordinate PointRelativeToGPSCoord(const geoops::GPSCoordinate& stGPSPoint, const double dEast, const double dNorth)
{
    geoops::UTMCoordinate stUTMPoint = geoops::ConvertGPSToUTM(stGPSPoint);
    stUTMPoint.dEasting += dEast;
    stUTMPoint.dNorthing += dNorth;
    return geoops::ConvertUTMToGPS(stUTMPoint);
}

/******************************************************************************
 * @brief Test the functionality of the Calculate function using UTM coordinates.
 *      Also tests functionality of the ResetProgress function.
 *
 * @author JSpencerPittman (jspencerpittman@gmail.com)
 * @date 2024-02-17
 ******************************************************************************/
TEST(StanleyControllerUnitTests, TestCalculateUTM)
{
    // Define Stanley controller parameters
    double dKp              = 0.5;
    double dDistToFrontAxle = 2.9;
    double dYawTolerance    = 1.0;

    // Define test path
    std::vector<geoops::UTMCoordinate> vPathUTM = {{0, 0}, {0, 10}, {5, 20}, {10, 30}, {20, 35}};

    // Make sure Easting is in the valid range [1200km, 2800km].
    // Add 2000km to each easting coordinate.
    std::vector<geoops::UTMCoordinate>::iterator itPathUTM;
    for (itPathUTM = vPathUTM.begin(); itPathUTM != vPathUTM.end(); ++itPathUTM)
    {
        itPathUTM->dEasting += UTM_SHIFT;
        itPathUTM->dNorthing += UTM_SHIFT;
    }

    // Initialize controller
    controllers::StanleyController stController(vPathUTM, dKp, dDistToFrontAxle, dYawTolerance);

    // Define agent's state.
    geoops::UTMCoordinate stCurrentPosUTM(1 + UTM_SHIFT, 0 + UTM_SHIFT);
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
    stCurrentPosUTM = geoops::UTMCoordinate(-1 + UTM_SHIFT, 1 + UTM_SHIFT);

    dTargetBearing  = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx     = stController.GetLastTargetIdx();

    EXPECT_PRED2(AnglesCloseAtWraparound, dTargetBearing, 0.385);
    EXPECT_EQ(unTargetIdx, 0);

    // Test move further along path (Bearing=80, Position=(7,20))
    dBearing        = 80;
    stCurrentPosUTM = geoops::UTMCoordinate(7 + UTM_SHIFT, 20 + UTM_SHIFT);

    dTargetBearing  = stController.Calculate(stCurrentPosUTM, dVelocity, dBearing);
    unTargetIdx     = stController.GetLastTargetIdx();

    EXPECT_NEAR(dTargetBearing, 26.393, 1);
    ASSERT_EQ(unTargetIdx, 2);

    // Test can't revert to earlier point in path (Bearing=80, Position=(1,0))
    stCurrentPosUTM = geoops::UTMCoordinate(1 + UTM_SHIFT, 0 + UTM_SHIFT);

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

/******************************************************************************
 * @brief Test the functionality of the Calculate function using GPS coordinates.
 *      Also tests functionality of the ResetProgress function and the conversion of GPS to UTM coordinates.
 *
 *
 * @author JSpencerPittman (jspencerpittman@gmail.com)
 * @date 2024-02-17
 ******************************************************************************/
TEST(StanleyControllerUnitTests, TestCalculateGPS)
{
    // Define Stanley controller parameters
    double dKp              = 0.5;
    double dDistToFrontAxle = 2.9;
    double dYawTolerance    = 1.0;

    // Location of Rolla
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);
    geoops::UTMCoordinate stUTMRollaCoordinate = geoops::ConvertGPSToUTM(stGPSRollaCoordinate);

    // Define test path in UTM
    std::vector<geoops::UTMCoordinate> vPathUTM = {{0, 0}, {0, 10}, {5, 20}, {10, 30}, {20, 35}};
    std::vector<geoops::GPSCoordinate> vGPSPath;

    std::vector<geoops::UTMCoordinate>::iterator itrUTM = vPathUTM.begin();
    while (itrUTM != vPathUTM.end())
    {
        vGPSPath.push_back(PointRelativeToGPSCoord(stGPSRollaCoordinate, itrUTM->dEasting, itrUTM->dNorthing));
        ++itrUTM;
    }

    // Initialize controller
    controllers::StanleyController stController(vGPSPath, dKp, dDistToFrontAxle, dYawTolerance);

    // Define agent's state.
    geoops::GPSCoordinate stCurrentPosGPS = PointRelativeToGPSCoord(stGPSRollaCoordinate, 1, 0);
    double dVelocity                      = 1.0;
    double dBearing                       = 30;

    // Variables to verify accuracy.
    double dTargetBearing;
    unsigned int unTargetIdx;

    // Test Calculate (Bearing=30, Position=(1,0))
    dTargetBearing = stController.Calculate(stCurrentPosGPS, dVelocity, dBearing);
    unTargetIdx    = stController.GetLastTargetIdx();

    EXPECT_PRED2(AnglesCloseAtWraparound, dTargetBearing, 359.591);
    EXPECT_EQ(unTargetIdx, 0);

    // Test opposite side of path (Bearing=350, Position=(-1,1))
    dBearing        = 350;
    stCurrentPosGPS = PointRelativeToGPSCoord(stGPSRollaCoordinate, -1, 1);

    dTargetBearing  = stController.Calculate(stCurrentPosGPS, dVelocity, dBearing);
    unTargetIdx     = stController.GetLastTargetIdx();

    EXPECT_PRED2(AnglesCloseAtWraparound, dTargetBearing, 0.385);
    EXPECT_EQ(unTargetIdx, 0);

    // Test move further along path (Bearing=80, Position=(7,20))
    dBearing        = 80;
    stCurrentPosGPS = PointRelativeToGPSCoord(stGPSRollaCoordinate, 7, 20);

    dTargetBearing  = stController.Calculate(stCurrentPosGPS, dVelocity, dBearing);
    unTargetIdx     = stController.GetLastTargetIdx();

    EXPECT_NEAR(dTargetBearing, 26.393, 1);
    ASSERT_EQ(unTargetIdx, 2);

    // Test can't revert to earlier point in path (Bearing=80, Position=(1,0))
    stCurrentPosGPS = PointRelativeToGPSCoord(stGPSRollaCoordinate, 1, 0);

    dTargetBearing  = stController.Calculate(stCurrentPosGPS, dVelocity, dBearing);
    unTargetIdx     = stController.GetLastTargetIdx();

    EXPECT_NEAR(dTargetBearing, 25.099, 1);
    ASSERT_EQ(unTargetIdx, 2);

    // Test revert to earlier point in path after reset (Bearing=80, Position=(1,0))
    stController.ResetProgress();

    dTargetBearing = stController.Calculate(stCurrentPosGPS, dVelocity, dBearing);
    unTargetIdx    = stController.GetLastTargetIdx();

    EXPECT_PRED2(AnglesCloseAtWraparound, dTargetBearing, 359.913);
    ASSERT_EQ(unTargetIdx, 0);
}
