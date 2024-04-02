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

/// \cond
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Function used in testing to determine if a returns list of waypoints
 *      is a good spiral.
 *
 * @param vPoints - The list of waypoint that, in order, should form a spiral.
 * @return true - The list of waypoints is a valid spiral.
 * @return false - The list of waypoint is not a valid spiral.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-01
 ******************************************************************************/
bool IsOutwardSpiral(const std::vector<geoops::Waypoint>& vPoints)
{
    // At least 4 vPoints are needed to form a spiral.
    if (vPoints.size() < 4)
    {
        return false;
    }

    // Store starting point.
    geoops::UTMCoordinate stCenterPoint = vPoints[0].GetUTMCoordinate();
    // Store geo distance information of last point.
    geoops::GeoMeasurement stLastGeodesicFromCenterPoint;

    // Loop through each waypoint.
    for (int nIter = 1; nIter < vPoints.size(); ++nIter)
    {
        // Check if this is the first iteration.
        if (nIter == 1)
        {
            // Just calculate geo distance between start and first point and store it.
            stLastGeodesicFromCenterPoint = geoops::CalculateGeoMeasurement(stCenterPoint, vPoints[nIter].GetUTMCoordinate());
        }
        else
        {
            // Calculate geo measurement for new point.
            geoops::GeoMeasurement stNewMeasurement = geoops::CalculateGeoMeasurement(stCenterPoint, vPoints[nIter].GetUTMCoordinate());

            // Check that the measurement is further away from the center point that last time and is radially progressing in some direction.
            if (stNewMeasurement.dDistanceMeters < stLastGeodesicFromCenterPoint.dDistanceMeters ||
                stNewMeasurement.dStartRelativeBearing == stLastGeodesicFromCenterPoint.dStartRelativeBearing ||
                stNewMeasurement.dEndRelativeBearing == stLastGeodesicFromCenterPoint.dEndRelativeBearing)
            {
                // Conditions net met, this is not a proper spiral.
                return false;
            }

            // Update last measurement variable.
            stLastGeodesicFromCenterPoint = stNewMeasurement;
        }
    }

    return true;
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
bool IsZigZag(const std::vector<geoops::Waypoint>& vPoints)
{
    //
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 ******************************************************************************/
TEST(SearchPatternTest, SpiralPatternShapeGPS)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateSpiralPatternWaypoints(stGPSRollaCoordinate);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(IsOutwardSpiral(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 ******************************************************************************/
TEST(SearchPatternTest, SpiralPatternShapeUTM)
{
    // Create a new GPS coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateSpiralPatternWaypoints(stUTMRollaCoordinate);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(IsOutwardSpiral(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2024-04-01
 ******************************************************************************/
TEST(SearchPatternTest, ZigZagPatternShapeGPS)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateSpiralPatternWaypoints(stGPSRollaCoordinate);

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
TEST(SearchPatternTest, ZigZagPatternShapeUTM)
{
    // Create a new GPS coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateSpiralPatternWaypoints(stUTMRollaCoordinate);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(IsZigZag(vSearchPatternPath));
}
