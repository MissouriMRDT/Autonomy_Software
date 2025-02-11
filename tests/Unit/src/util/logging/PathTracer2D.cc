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
        logging::graphing::PathTracer m_PathTracer;

    public:
        /******************************************************************************
         * @brief Construct a new PathTracer2DTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-10
         ******************************************************************************/
        PathTracer2DTests() : m_PathTracer("Test Plot") {}

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
    m_PathTracer.CreatePathLayer("Layer1", "-o");
    EXPECT_TRUE(m_PathTracer.CreatePathLayer("Layer2", "--x"));
    EXPECT_FALSE(m_PathTracer.CreatePathLayer("Layer1", "-o"));
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
    m_PathTracer.CreateDotLayer("DotLayer1", "blue", true);
    EXPECT_TRUE(m_PathTracer.CreateDotLayer("DotLayer2", "red", false));
    EXPECT_FALSE(m_PathTracer.CreateDotLayer("DotLayer1", "green", true));
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
    m_PathTracer.CreatePathLayer("Layer1", "-o");
    m_PathTracer.CreateDotLayer("DotLayer1", "blue", true);
    EXPECT_TRUE(m_PathTracer.DeleteLayer("Layer1"));
    EXPECT_TRUE(m_PathTracer.DeleteLayer("DotLayer1"));
    EXPECT_FALSE(m_PathTracer.DeleteLayer("NonExistentLayer"));
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
    m_PathTracer.CreatePathLayer("Layer1", "-o");
    m_PathTracer.CreateDotLayer("DotLayer1", "blue", true);
    EXPECT_TRUE(m_PathTracer.ClearLayer("Layer1"));
    EXPECT_TRUE(m_PathTracer.ClearLayer("DotLayer1"));
    EXPECT_FALSE(m_PathTracer.ClearLayer("NonExistentLayer"));
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
    m_PathTracer.CreatePathLayer("Layer1", "-o");
    geoops::Waypoint stWaypoint{geoops::GPSCoordinate(37.951766, -91.778187)};
    EXPECT_NO_THROW(m_PathTracer.AddPathPoint(stWaypoint, "Layer1", 1));
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
    m_PathTracer.CreatePathLayer("Layer1", "-o");
    geoops::UTMCoordinate stUTMCoord{607344.14, 4201167.33, 15, true};
    EXPECT_NO_THROW(m_PathTracer.AddPathPoint(stUTMCoord, "Layer1", 1));
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
    m_PathTracer.CreatePathLayer("Layer1", "-o");
    geoops::GPSCoordinate stGPSCoord{37.951766, -91.778187};
    EXPECT_NO_THROW(m_PathTracer.AddPathPoint(stGPSCoord, "Layer1", 1));
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
    m_PathTracer.CreatePathLayer("Layer1", "-o");
    std::vector<geoops::Waypoint> stWaypoint = {geoops::GPSCoordinate(37.951766, -91.778187), geoops::GPSCoordinate(38.406267, -110.791997)};
    EXPECT_NO_THROW(m_PathTracer.AddPathPoints(stWaypoint, "Layer1", 1));
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
    m_PathTracer.CreatePathLayer("Layer1", "-o");
    std::vector<geoops::UTMCoordinate> stUTMCoord = {{607344.14, 4201167.33, 15, true}, {518160.91, 4250913.23, 12, true}};
    EXPECT_NO_THROW(m_PathTracer.AddPathPoints(stUTMCoord, "Layer1", 1));
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
    m_PathTracer.CreatePathLayer("Layer1", "-o");
    std::vector<geoops::GPSCoordinate> stGPSCoord = {{37.951766, -91.778187}, {38.406267, -110.791997}};
    EXPECT_NO_THROW(m_PathTracer.AddPathPoints(stGPSCoord, "Layer1", 1));
}
