/******************************************************************************
 * @brief Unit test for SearchPattern algorithm namespace.
 *
 * @file SearchPattern.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-01
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/algorithms/SearchPattern.hpp"
#include "../../../../src/util/GeospatialOperations.hpp"
#include "../../../TestingBase.hh"

/// \cond
#include <cmath>
#include <gtest/gtest.h>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the SearchPattern
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class SearchPatternTests : public TestingBase<SearchPatternTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

        /******************************************************************************
         * @brief Function used in testing to determine if a returned list of waypoints
         *      forms a valid out-and-back spiral.
         *
         *      CalculateSpiralPatternWaypoints() winds outward from the center until it
         *      passes the max radius, then winds back in to the center again (see the
         *      "Same but going back in" loop in SearchPattern.hpp). So a valid path is a
         *      single peak: radius strictly increases to one turnaround point, then
         *      strictly decreases, with the bearing advancing at every step so the path
         *      is genuinely spiralling rather than moving radially in and out.
         *
         *      An earlier version of this helper required the radius to increase
         *      monotonically across the WHOLE path, which only described the outward leg
         *      and has not matched the algorithm since the return leg was added.
         *
         * @param vPoints - The list of waypoints that, in order, should form a spiral.
         * @return true - The list of waypoints is a valid out-and-back spiral.
         * @return false - The list of waypoints is not a valid out-and-back spiral.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-01
         ******************************************************************************/
        // Minimum bearing sweep, in degrees, required between consecutive waypoints for a path to
        // count as spiralling rather than travelling radially. The real pattern steps ~57 degrees.
        static constexpr double MIN_SPIRAL_BEARING_STEP_DEGREES = 1.0;

        bool IsOutAndBackSpiral(const std::vector<geoops::Waypoint>& vPoints)
        {
            // At least 4 vPoints are needed to form a spiral.
            if (vPoints.size() < 4)
            {
                return false;
            }

            // Store starting point.
            geoops::UTMCoordinate stCenterPoint = vPoints[0].GetUTMCoordinate();

            // Track the previous point's distance and bearing from the center.
            geoops::GeoMeasurement stLastGeodesicFromCenterPoint = geoops::CalculateGeoMeasurement(stCenterPoint, vPoints[1].GetUTMCoordinate());
            // Whether we have passed the turnaround and are now winding back inward.
            bool bWindingInward = false;
            // How many times the radius reversed direction. A valid path reverses exactly once.
            int nDirectionChanges = 0;

            // Loop through each remaining waypoint.
            for (size_t siIter = 2; siIter < vPoints.size(); ++siIter)
            {
                // Calculate geo measurement for new point.
                geoops::GeoMeasurement stNewMeasurement = geoops::CalculateGeoMeasurement(stCenterPoint, vPoints[siIter].GetUTMCoordinate());

                // Check whether the radius reversed direction on this step.
                if (!bWindingInward && stNewMeasurement.dDistanceMeters < stLastGeodesicFromCenterPoint.dDistanceMeters)
                {
                    // This is the turnaround from the outward leg to the return leg.
                    bWindingInward = true;
                    ++nDirectionChanges;
                }
                else if (bWindingInward && stNewMeasurement.dDistanceMeters > stLastGeodesicFromCenterPoint.dDistanceMeters)
                {
                    // The path started winding back outward, which is not a single clean spiral.
                    ++nDirectionChanges;
                    return false;
                }
                else if (stNewMeasurement.dDistanceMeters == stLastGeodesicFromCenterPoint.dDistanceMeters)
                {
                    // The radius stalled, so the path is not progressing radially at all.
                    return false;
                }

                // The bearing must advance every step, otherwise the path is moving straight in or
                // out along one ray rather than spiralling. The center point itself has no
                // meaningful bearing, so skip the comparison when either point sits on it.
                if (stLastGeodesicFromCenterPoint.dDistanceMeters > 0.0 && stNewMeasurement.dDistanceMeters > 0.0)
                {
                    // Compare bearings with a tolerance rather than for exact equality. These
                    // measurements are geodesics derived from UTM points, so even a perfectly
                    // radial path produces bearings that differ in the last few decimal places and
                    // an equality test would never fire. The real pattern steps ~57 degrees per
                    // waypoint, so anything under a degree is not spiralling.
                    double dBearingDeltaDegrees = std::fabs(stNewMeasurement.dStartRelativeBearing - stLastGeodesicFromCenterPoint.dStartRelativeBearing);
                    // Take the short way round the compass so the 360/0 wrap is not seen as a huge step.
                    if (dBearingDeltaDegrees > 180.0)
                    {
                        // Fold the difference back into [0, 180].
                        dBearingDeltaDegrees = 360.0 - dBearingDeltaDegrees;
                    }

                    // Check that the path actually swept round the center this step.
                    if (dBearingDeltaDegrees < MIN_SPIRAL_BEARING_STEP_DEGREES)
                    {
                        // Conditions not met, this is not a proper spiral.
                        return false;
                    }
                }

                // Update last measurement variable.
                stLastGeodesicFromCenterPoint = stNewMeasurement;
            }

            // A valid out-and-back spiral turns around exactly once.
            return nDirectionChanges == 1;
        }

        /******************************************************************************
         * @brief Function used in testing to determine if a returns list of waypoints
         *      is a good zigzag.
         *
         * @param vPoints - The list of waypoint that, in order, should form a zigzag.
         * @return true - The list of waypoints is a valid zigzag.
         * @return false - The list of waypoint is not a valid zigzag.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-04-01
         ******************************************************************************/
        bool IsZigZag(const std::vector<geoops::Waypoint>& vPoints, const double dSlopeThreshold = 0.1)
        {
            // Create instance variables.
            bool bZigZag          = false;
            double dExpectedSlope = 0.0;

            // At least 3 points are needed for a zigzag.
            if (vPoints.size() < 3)
            {
                return false;
            }

            // Loop through path points.
            for (size_t siIter = 1; siIter < vPoints.size(); ++siIter)
            {
                // Calculate slope of last two points.
                double dSlope = atan2(vPoints[siIter].GetUTMCoordinate().dNorthing - vPoints[siIter - 1].GetUTMCoordinate().dNorthing,
                                      vPoints[siIter].GetUTMCoordinate().dEasting - vPoints[siIter - 1].GetUTMCoordinate().dEasting);

                // Make sure this isn't the first iteration of the loop.
                if (siIter > 1)
                {
                    // Calculate the previous slope.
                    double dPrevSlope = atan2(vPoints[siIter - 1].GetUTMCoordinate().dNorthing - vPoints[siIter - 2].GetUTMCoordinate().dNorthing,
                                              vPoints[siIter - 1].GetUTMCoordinate().dEasting - vPoints[siIter - 2].GetUTMCoordinate().dEasting);
                    // Check if the slope has changed.
                    if (std::abs(dSlope - dPrevSlope) > dSlopeThreshold)
                    {
                        // Check if the slope is the same value as the dExpected slope and has the same sign.
                        if ((dSlope - dPrevSlope) == dExpectedSlope)
                        {
                            // Set this is a zigzag.
                            bZigZag = true;
                        }
                        else
                        {
                            return false;
                        }
                    }
                }
                // This is the firs iteration.
                if (siIter < vPoints.size() - 1)
                {
                    // Calculate the next slope.
                    double dNextSlope = atan2(vPoints[siIter + 1].GetUTMCoordinate().dNorthing - vPoints[siIter].GetUTMCoordinate().dNorthing,
                                              vPoints[siIter + 1].GetUTMCoordinate().dEasting - vPoints[siIter].GetUTMCoordinate().dEasting);
                    // Check if the slope has changed.
                    if (std::abs(dSlope - dNextSlope) > dSlopeThreshold)
                    {
                        // Store the next expected slope change. (will have opposite sign)
                        dExpectedSlope = -(dSlope - dNextSlope);
                    }
                }
            }

            // If either increasing or decreasing slope condition is met, it's a zigzag.
            return bZigZag;
        }

    public:
        /******************************************************************************
         * @brief Construct a new Search Pattern Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        SearchPatternTests() {}

        /******************************************************************************
         * @brief Destroy the Search Pattern Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~SearchPatternTests() {}

        /******************************************************************************
         * @brief Setup the Search Pattern Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Teardown the Search Pattern Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 ******************************************************************************/
/******************************************************************************
 * @brief Guard tests for the IsOutAndBackSpiral helper itself. Without these, a helper
 *      that accidentally always returned true would make every spiral shape test pass
 *      vacuously - which is exactly how the previous version went stale unnoticed.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-28
 ******************************************************************************/
TEST_F(SearchPatternTests, SpiralShapeHelperRejectsBadPaths)
{
    // Center the synthetic paths on the same spot the real tests use.
    const double dCenterEasting  = 607344.14;
    const double dCenterNorthing = 4201167.33;

    // Build a waypoint at a given offset from the center.
    auto MakeWaypoint = [&](const double dOffsetEasting, const double dOffsetNorthing)
    {
        // Assemble the UTM coordinate and wrap it as a navigation waypoint.
        geoops::UTMCoordinate stCoordinate(dCenterEasting + dOffsetEasting, dCenterNorthing + dOffsetNorthing, 15, true);
        return geoops::Waypoint(stCoordinate, geoops::WaypointType::eNavigationWaypoint);
    };

    // A purely OUTWARD spiral must be rejected: the algorithm is required to wind back in,
    // so a path that never turns around is a regression, not a valid pattern.
    std::vector<geoops::Waypoint> vOutwardOnly;
    for (int nIter = 0; nIter < 6; ++nIter)
    {
        // Step the radius out by 1m and the bearing round by 57 degrees each time.
        const double dRadius = static_cast<double>(nIter);
        const double dAngle  = nIter * 57.0 * M_PI / 180.0;
        vOutwardOnly.emplace_back(MakeWaypoint(dRadius * std::cos(dAngle), dRadius * std::sin(dAngle)));
    }
    EXPECT_FALSE(IsOutAndBackSpiral(vOutwardOnly));

    // A path that goes straight out and straight back along ONE bearing must be rejected: the
    // radius profile is right but it is a line, not a spiral.
    std::vector<geoops::Waypoint> vRadialLine;
    for (const double dRadius : {0.0, 1.0, 2.0, 3.0, 2.0, 1.0})
    {
        // Every point sits due east of the center, so the bearing never advances.
        vRadialLine.emplace_back(MakeWaypoint(dRadius, 0.0));
    }
    EXPECT_FALSE(IsOutAndBackSpiral(vRadialLine));

    // Too few points to describe a spiral at all.
    std::vector<geoops::Waypoint> vTooShort{MakeWaypoint(0.0, 0.0), MakeWaypoint(1.0, 0.0)};
    EXPECT_FALSE(IsOutAndBackSpiral(vTooShort));
}

TEST_F(SearchPatternTests, SpiralPatternShapeGPS)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateSpiralPatternWaypoints(stGPSRollaCoordinate);

    // Check if the returned path resembles an out-and-back spiral pattern.
    EXPECT_TRUE(IsOutAndBackSpiral(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 ******************************************************************************/
TEST_F(SearchPatternTests, SpiralPatternShapeUTM)
{
    // Create a new GPS coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateSpiralPatternWaypoints(stUTMRollaCoordinate);

    // Check if the returned path resembles an out-and-back spiral pattern.
    EXPECT_TRUE(IsOutAndBackSpiral(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2024-04-01
 ******************************************************************************/
TEST_F(SearchPatternTests, ZigZagPatternShapeGPS)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateZigZagPatternWaypoints(stGPSRollaCoordinate);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(IsZigZag(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-06
 ******************************************************************************/
TEST_F(SearchPatternTests, ZigZagPatternShapeGPSHorizontal)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateZigZagPatternWaypoints(stGPSRollaCoordinate, 20.0, 20.0, 1.0, false);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(IsZigZag(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2024-04-01
 ******************************************************************************/
TEST_F(SearchPatternTests, ZigZagPatternShapeUTM)
{
    // Create a new GPS coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateZigZagPatternWaypoints(stUTMRollaCoordinate);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(IsZigZag(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-06
 ******************************************************************************/
TEST_F(SearchPatternTests, ZigZagPatternShapeUTMHorizontal)
{
    // Create a new GPS coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath  = searchpattern::CalculateZigZagPatternWaypoints(stUTMRollaCoordinate, 20.0, 20.0, 2.0, false);
    std::vector<geoops::Waypoint> vSearchPatternPath2 = searchpattern::CalculateZigZagPatternWaypoints(stUTMRollaCoordinate, 20.0, 20.0, 2.0, true);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(IsZigZag(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-06
 ******************************************************************************/
TEST_F(SearchPatternTests, SpiralPatternRadiusTooSmall)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateSpiralPatternWaypoints(stGPSRollaCoordinate, 57, 0);

    // Create a new UTm coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPathUTM = searchpattern::CalculateSpiralPatternWaypoints(stUTMRollaCoordinate, 57, 0);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(vSearchPatternPath.empty());
    EXPECT_TRUE(vSearchPatternPathUTM.empty());
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-06
 ******************************************************************************/
TEST_F(SearchPatternTests, ZigZagPatternRadiusTooSmall)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateZigZagPatternWaypoints(stGPSRollaCoordinate, 20.0, 20.0, 0);

    // Create a new UTm coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPathUTM = searchpattern::CalculateZigZagPatternWaypoints(stUTMRollaCoordinate, 20.0, 20.0, 0);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(vSearchPatternPath.empty());
    EXPECT_TRUE(vSearchPatternPathUTM.empty());
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-06
 ******************************************************************************/
TEST_F(SearchPatternTests, ZigZagPatternLimitSpacing)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateZigZagPatternWaypoints(stGPSRollaCoordinate, 20.0, 20.0, 30.0);
    // Create a new UTm coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPathUTM = searchpattern::CalculateZigZagPatternWaypoints(stUTMRollaCoordinate, 20.0, 20.0, 30.0);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath2 = searchpattern::CalculateZigZagPatternWaypoints(stGPSRollaCoordinate, 60.0, 20.0, 30.0);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPathUTM2 = searchpattern::CalculateZigZagPatternWaypoints(stUTMRollaCoordinate, 60.0, 20.0, 30.0);

    // Both paths should be zigzags.
    EXPECT_TRUE(IsZigZag(vSearchPatternPath));
    EXPECT_TRUE(IsZigZag(vSearchPatternPathUTM));
    EXPECT_TRUE(IsZigZag(vSearchPatternPath2));
    EXPECT_TRUE(IsZigZag(vSearchPatternPathUTM2));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-14
 ******************************************************************************/
TEST_F(SearchPatternTests, SnakePatternShape)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);
    // Generate a bunch of snake patterns with different parameters.
    std::vector<geoops::Waypoint> vSearchPatternPath1  = searchpattern::CalculateSnakeSearchPattern(stGPSRollaCoordinate, 40.0, 40.0, 1.0, 2.0, true);
    std::vector<geoops::Waypoint> vSearchPatternPath2  = searchpattern::CalculateSnakeSearchPattern(stGPSRollaCoordinate, 40.0, 40.0, 1.0, 2.0, false);
    std::vector<geoops::Waypoint> vSearchPatternPath3  = searchpattern::CalculateSnakeSearchPattern(stGPSRollaCoordinate, 10.0, 10.0, 1.0, 4.0, true);
    std::vector<geoops::Waypoint> vSearchPatternPath4  = searchpattern::CalculateSnakeSearchPattern(stGPSRollaCoordinate, 10.0, 10.0, 1.0, 4.0, false);
    std::vector<geoops::Waypoint> vSearchPatternpath5  = searchpattern::CalculateSnakeSearchPattern(stGPSRollaCoordinate, 20.0, 10.0, 1.0, 4.0, true);
    std::vector<geoops::Waypoint> vSearchPatternPath6  = searchpattern::CalculateSnakeSearchPattern(stGPSRollaCoordinate, 20.0, 10.0, 1.0, 4.0, false);
    std::vector<geoops::Waypoint> vSearchPatternPath7  = searchpattern::CalculateSnakeSearchPattern(stGPSRollaCoordinate, 10.0, 20.0, 1.0, 4.0, true);
    std::vector<geoops::Waypoint> vSearchPatternPath8  = searchpattern::CalculateSnakeSearchPattern(stGPSRollaCoordinate, 10.0, 20.0, 1.0, 4.0, false);
    std::vector<geoops::Waypoint> vSearchPatternpath9  = searchpattern::CalculateSnakeSearchPattern(stGPSRollaCoordinate, 5.0, 5.0, 0.5, 4.0, true);
    std::vector<geoops::Waypoint> vSearchPatternPath10 = searchpattern::CalculateSnakeSearchPattern(stGPSRollaCoordinate, 5.0, 5.0, 0.5, 4.0, false);
    std::vector<geoops::Waypoint> vSearchPatternPath11 = searchpattern::CalculateSnakeSearchPattern(stGPSRollaCoordinate, 0.5, 0.5, 0.5, 4.0, true);

    // Check if the returned path resembles a snake pattern.
    EXPECT_TRUE(!vSearchPatternPath1.empty());
    EXPECT_TRUE(!vSearchPatternPath2.empty());
    EXPECT_TRUE(!vSearchPatternPath3.empty());
    EXPECT_TRUE(!vSearchPatternPath4.empty());
    EXPECT_TRUE(!vSearchPatternpath5.empty());
    EXPECT_TRUE(!vSearchPatternPath6.empty());
    EXPECT_TRUE(!vSearchPatternPath7.empty());
    EXPECT_TRUE(!vSearchPatternPath8.empty());
    EXPECT_TRUE(!vSearchPatternpath9.empty());
    EXPECT_TRUE(!vSearchPatternPath10.empty());
    EXPECT_TRUE(vSearchPatternPath11.empty());
}
