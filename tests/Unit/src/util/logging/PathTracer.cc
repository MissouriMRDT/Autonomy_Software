/******************************************************************************
 * @brief Unit Test Class for the PathTracer
 *
 * @file PathTracer.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/util/logging/PathTracer.hpp"
#include "../../../../TestingBase.hh"

/// \cond
#include <chrono>
#include <filesystem>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <thread>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the PathTracer
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
class PathTracerTests : public TestingBase<PathTracerTests>
{
    protected:
        logging::graphing::PathTracer m_PathTracer2D;
        logging::graphing::PathTracer m_PathTracer3D;

    public:
        /******************************************************************************
         * @brief Construct a new PathTracerTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-10
         ******************************************************************************/
        PathTracerTests() : m_PathTracer2D("Test Plot 2D"), m_PathTracer3D("Test Plot 3D", true) {}

        /******************************************************************************
         * @brief Destroy the PathTracerTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-10
         ******************************************************************************/
        ~PathTracerTests() {}

        /******************************************************************************
         * @brief Setup the PathTracerTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-10
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Teardown the PathTracerTests object.
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
TEST_F(PathTracerTests, CreatePathLayer)
{
    m_PathTracer2D.CreatePathLayer("Layer1", "-o");
    EXPECT_TRUE(m_PathTracer2D.CreatePathLayer("Layer2", "--x"));
    EXPECT_FALSE(m_PathTracer2D.CreatePathLayer("Layer1", "-o"));
}

/******************************************************************************
 * @brief Test the functionality of the CreateDotLayer method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracerTests, CreateDotLayer)
{
    m_PathTracer2D.CreateDotLayer("DotLayer1", "blue", true);
    EXPECT_TRUE(m_PathTracer2D.CreateDotLayer("DotLayer2", "red", false));
    EXPECT_FALSE(m_PathTracer2D.CreateDotLayer("DotLayer1", "green", true));
}

/******************************************************************************
 * @brief Test the functionality of the DeleteLayer method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracerTests, DeleteLayer)
{
    m_PathTracer2D.CreatePathLayer("Layer1", "-o");
    m_PathTracer2D.CreateDotLayer("DotLayer1", "blue", true);
    EXPECT_TRUE(m_PathTracer2D.DeleteLayer("Layer1"));
    EXPECT_TRUE(m_PathTracer2D.DeleteLayer("DotLayer1"));
    EXPECT_FALSE(m_PathTracer2D.DeleteLayer("NonExistentLayer"));
}

/******************************************************************************
 * @brief Test the functionality of the ClearLayer method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracerTests, ClearLayer)
{
    m_PathTracer2D.CreatePathLayer("Layer1", "-o");
    m_PathTracer2D.CreateDotLayer("DotLayer1", "blue", true);
    EXPECT_TRUE(m_PathTracer2D.ClearLayer("Layer1"));
    EXPECT_TRUE(m_PathTracer2D.ClearLayer("DotLayer1"));
    EXPECT_FALSE(m_PathTracer2D.ClearLayer("NonExistentLayer"));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoint method with Waypoint.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracerTests, AddPathPointWaypoint)
{
    m_PathTracer2D.CreatePathLayer("Layer1", "-o");
    geoops::Waypoint stWaypoint{geoops::GPSCoordinate(37.951766, -91.778187)};
    EXPECT_NO_THROW(m_PathTracer2D.AddPathPoint(stWaypoint, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoint method with UTMCoordinate.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracerTests, AddPathPointUTMCoordinate)
{
    m_PathTracer2D.CreatePathLayer("Layer1", "-o");
    geoops::UTMCoordinate stUTMCoord{607344.14, 4201167.33, 15, true};
    EXPECT_NO_THROW(m_PathTracer2D.AddPathPoint(stUTMCoord, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoint method with GPSCoordinate.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracerTests, AddPathPointGPSCoordinate)
{
    m_PathTracer2D.CreatePathLayer("Layer1", "-o");
    geoops::GPSCoordinate stGPSCoord{37.951766, -91.778187};
    EXPECT_NO_THROW(m_PathTracer2D.AddPathPoint(stGPSCoord, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoints method with Waypoints.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracerTests, AddPathPointsWaypoints)
{
    m_PathTracer3D.CreatePathLayer("Layer1", "-o");
    std::vector<geoops::Waypoint> stWaypoint = {geoops::GPSCoordinate(37.951766, -91.778187), geoops::GPSCoordinate(38.406267, -110.791997)};
    EXPECT_NO_THROW(m_PathTracer3D.AddPathPoints(stWaypoint, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoints method with UTMCoordinates.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracerTests, AddPathPointsUTMCoordinates)
{
    m_PathTracer3D.CreatePathLayer("Layer1", "-o");
    std::vector<geoops::UTMCoordinate> stUTMCoord = {{607344.14, 4201167.33, 15, true}, {518160.91, 4250913.23, 12, true}};
    EXPECT_NO_THROW(m_PathTracer3D.AddPathPoints(stUTMCoord, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoints method with GPSCoordinates.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PathTracerTests, AddPathPointsGPSCoordinates)
{
    m_PathTracer3D.CreatePathLayer("Layer1", "-o");
    std::vector<geoops::GPSCoordinate> stGPSCoord = {{37.951766, -91.778187}, {38.406267, -110.791997}};
    EXPECT_NO_THROW(m_PathTracer3D.AddPathPoints(stGPSCoord, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoint method with Waypoint.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-31
 ******************************************************************************/
TEST_F(PathTracerTests, AddPathPointWaypoint3D)
{
    m_PathTracer3D.CreatePathLayer("Layer1", "-o");
    geoops::Waypoint stWaypoint{geoops::GPSCoordinate(37.951766, -91.778187, 1.0)};
    EXPECT_NO_THROW(m_PathTracer3D.AddPathPoint(stWaypoint, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoint method with UTMCoordinate.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-31
 ******************************************************************************/
TEST_F(PathTracerTests, AddPathPointUTMCoordinate3D)
{
    m_PathTracer3D.CreatePathLayer("Layer1", "-o");
    geoops::UTMCoordinate stUTMCoord{607344.14, 4201167.33, 15, true, 1.0};
    EXPECT_NO_THROW(m_PathTracer3D.AddPathPoint(stUTMCoord, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoint method with GPSCoordinate.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-31
 ******************************************************************************/
TEST_F(PathTracerTests, AddPathPointGPSCoordinate3D)
{
    m_PathTracer3D.CreatePathLayer("Layer1", "-o");
    geoops::GPSCoordinate stGPSCoord{37.951766, -91.778187, 1.0};
    EXPECT_NO_THROW(m_PathTracer3D.AddPathPoint(stGPSCoord, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoints method with Waypoints.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-31
 ******************************************************************************/
TEST_F(PathTracerTests, AddPathPointsWaypoints3D)
{
    m_PathTracer3D.CreatePathLayer("Layer1", "-o");
    std::vector<geoops::Waypoint> stWaypoint = {geoops::GPSCoordinate(37.951766, -91.778187, 1.0), geoops::GPSCoordinate(38.406267, -110.791997, 2.0)};
    EXPECT_NO_THROW(m_PathTracer3D.AddPathPoints(stWaypoint, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoints method with UTMCoordinates.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-31
 ******************************************************************************/
TEST_F(PathTracerTests, AddPathPointsUTMCoordinates3D)
{
    m_PathTracer3D.CreatePathLayer("Layer1", "-o");
    std::vector<geoops::UTMCoordinate> stUTMCoord = {{607344.14, 4201167.33, 15, true, 1.0}, {518160.91, 4250913.23, 12, true, 2.0}};
    EXPECT_NO_THROW(m_PathTracer3D.AddPathPoints(stUTMCoord, "Layer1", 1));
}

/******************************************************************************
 * @brief Test the functionality of the AddPathPoints method with GPSCoordinates.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-31
 ******************************************************************************/
TEST_F(PathTracerTests, AddPathPointsGPSCoordinates3D)
{
    m_PathTracer3D.CreatePathLayer("Layer1", "-o");
    std::vector<geoops::GPSCoordinate> stGPSCoord = {{37.951766, -91.778187, 1.0}, {38.406267, -110.791997, 2.0}};
    EXPECT_NO_THROW(m_PathTracer3D.AddPathPoints(stGPSCoord, "Layer1", 1));
}
