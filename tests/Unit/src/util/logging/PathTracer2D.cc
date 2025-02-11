/******************************************************************************
 * @brief Unit Test Class for the PathTracer2D
 *
 * @file PathTracer2D.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/util/logging/PathTracer2D.hpp"
#include "../../../../TestingBase.hh"

/// \cond
#include <chrono>
#include <filesystem>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <thread>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the PathTracer2D
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
class PathTracer2DTests : public TestingBase<PathTracer2DTests>
{
    protected:
        logging::graphing::PathTracer pathTracer;

    public:
        /******************************************************************************
         * @brief Construct a new PathTracer2DTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-10
         ******************************************************************************/
        PathTracer2DTests() : pathTracer("Test Plot") {}

        /******************************************************************************
         * @brief Destroy the PathTracer2DTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-10
         ******************************************************************************/
        ~PathTracer2DTests() {}

        /******************************************************************************
         * @brief Setup the PathTracer2DTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-10
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Teardown the PathTracer2DTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-10
         ******************************************************************************/
        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Test the functionality of the CreatePathLayer method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracer2DTests, CreatePathLayer)
{
    pathTracer.CreatePathLayer("Layer1", "-o");
    EXPECT_TRUE(pathTracer.CreatePathLayer("Layer2", "--x"));
    EXPECT_FALSE(pathTracer.CreatePathLayer("Layer1", "-o"));
}

/******************************************************************************
 * @brief Test the functionality of the CreateDotLayer method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracer2DTests, CreateDotLayer)
{
    pathTracer.CreateDotLayer("DotLayer1", "blue", true);
    EXPECT_TRUE(pathTracer.CreateDotLayer("DotLayer2", "red", false));
    EXPECT_FALSE(pathTracer.CreateDotLayer("DotLayer1", "green", true));
}

/******************************************************************************
 * @brief Test the functionality of the DeleteLayer method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracer2DTests, DeleteLayer)
{
    pathTracer.CreatePathLayer("Layer1", "-o");
    pathTracer.CreateDotLayer("DotLayer1", "blue", true);
    EXPECT_TRUE(pathTracer.DeleteLayer("Layer1"));
    EXPECT_TRUE(pathTracer.DeleteLayer("DotLayer1"));
    EXPECT_FALSE(pathTracer.DeleteLayer("NonExistentLayer"));
}

/******************************************************************************
 * @brief Test the functionality of the ClearLayer method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracer2DTests, ClearLayer)
{
    pathTracer.CreatePathLayer("Layer1", "-o");
    pathTracer.CreateDotLayer("DotLayer1", "blue", true);
    EXPECT_TRUE(pathTracer.ClearLayer("Layer1"));
    EXPECT_TRUE(pathTracer.ClearLayer("DotLayer1"));
    EXPECT_FALSE(pathTracer.ClearLayer("NonExistentLayer"));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoint method with Waypoint.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracer2DTests, AddPathPointWaypoint)
{
    pathTracer.CreatePathLayer("Layer1", "-o");
    geoops::Waypoint waypoint{geoops::GPSCoordinate(37.951766, -91.778187)};
    EXPECT_NO_THROW(pathTracer.AddPathPoint(waypoint, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoint method with UTMCoordinate.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracer2DTests, AddPathPointUTMCoordinate)
{
    pathTracer.CreatePathLayer("Layer1", "-o");
    geoops::UTMCoordinate utmCoordinate{607344.14, 4201167.33, 15, true};
    EXPECT_NO_THROW(pathTracer.AddPathPoint(utmCoordinate, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoint method with GPSCoordinate.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracer2DTests, AddPathPointGPSCoordinate)
{
    pathTracer.CreatePathLayer("Layer1", "-o");
    geoops::GPSCoordinate gpsCoordinate{37.951766, -91.778187};
    EXPECT_NO_THROW(pathTracer.AddPathPoint(gpsCoordinate, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoints method with Waypoints.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracer2DTests, AddPathPointsWaypoints)
{
    pathTracer.CreatePathLayer("Layer1", "-o");
    std::vector<geoops::Waypoint> waypoints = {geoops::GPSCoordinate(37.951766, -91.778187), geoops::GPSCoordinate(38.406267, -110.791997)};
    EXPECT_NO_THROW(pathTracer.AddPathPoints(waypoints, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoints method with UTMCoordinates.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracer2DTests, AddPathPointsUTMCoordinates)
{
    pathTracer.CreatePathLayer("Layer1", "-o");
    std::vector<geoops::UTMCoordinate> utmCoordinates = {{607344.14, 4201167.33, 15, true}, {518160.91, 4250913.23, 12, true}};
    EXPECT_NO_THROW(pathTracer.AddPathPoints(utmCoordinates, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoints method with GPSCoordinates.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracer2DTests, AddPathPointsGPSCoordinates)
{
    pathTracer.CreatePathLayer("Layer1", "-o");
    std::vector<geoops::GPSCoordinate> gpsCoordinates = {{37.951766, -91.778187}, {38.406267, -110.791997}};
    EXPECT_NO_THROW(pathTracer.AddPathPoints(gpsCoordinates, "Layer1", 1));
}
