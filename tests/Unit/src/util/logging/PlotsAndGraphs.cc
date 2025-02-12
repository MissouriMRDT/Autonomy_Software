/******************************************************************************
 * @brief Test the functionality of the PlotCoordinates2D function for UTM coordinates.
 *
 * @file test_PlotsAndGraphs.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/util/logging/PlotsAndGraphs.hpp"
#include "../../../../TestingBase.hh"

/// \cond
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <matplot/matplot.h>

/// \endcond

/******************************************************************************
 * @brief Test class for the PlotsAndGraphs class.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
class PlotsAndGraphsTests : public TestingBase<PlotsAndGraphsTests>
{
    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new Plots And Graphs Tests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-10
         ******************************************************************************/
        PlotsAndGraphsTests() {}

        /******************************************************************************
         * @brief Destroy the Plots And Graphs Tests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-10
         ******************************************************************************/
        ~PlotsAndGraphsTests() {}

        /******************************************************************************
         * @brief Setup the PlotsAndGraphsTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-10
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Teardown the PlotsAndGraphsTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-10
         ******************************************************************************/
        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Test the functionality of the PlotCoordinates2D function for UTM coordinates.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PlotsAndGraphsTests, PlotCoordinates2D_UTM)
{
    std::vector<geoops::UTMCoordinate> vCoordinates = {{500000, 4649776.22482, 33, true}, {500100, 4649876.22482, 33, true}, {500200, 4649976.22482, 33, true}};

    logging::graphing::PlotCoordinates2D(vCoordinates, "UTMCoordinatePlotTest");

    // We can't really control the gnuplot backend or when it saves the file, so we need to sleep for a bit to make sure the file is saved.
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::string szFileName = logging::g_szLoggingOutputPath + "/path_plots/UTMCoordinatePlotTest.png";
    EXPECT_TRUE(std::filesystem::exists(szFileName));
}

/******************************************************************************
 * @brief Test the functionality of the PlotCoordinates2D function for GPS coordinates.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PlotsAndGraphsTests, PlotCoordinates2D_GPS)
{
    std::vector<geoops::GPSCoordinate> vCoordinates = {{37.7749, -122.4194}, {34.0522, -118.2437}, {40.7128, -74.0060}};

    logging::graphing::PlotCoordinates2D(vCoordinates, "GPSCoordinatePlotTest");

    // We can't really control the gnuplot backend or when it saves the file, so we need to sleep for a bit to make sure the file is saved.
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::string szFileName = logging::g_szLoggingOutputPath + "/path_plots/GPSCoordinatePlotTest.png";
    EXPECT_TRUE(std::filesystem::exists(szFileName));
}

/******************************************************************************
 * @brief Test the functionality of the PlotCoordinates2D function for waypoints.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PlotsAndGraphsTests, PlotCoordinates2D_Waypoints)
{
    std::vector<geoops::Waypoint> vWaypoints = {{geoops::GPSCoordinate(), geoops::WaypointType::eUNKNOWN, 20.0, 0},
                                                {geoops::GPSCoordinate(), geoops::WaypointType::eUNKNOWN, 20.0, 0},
                                                {geoops::GPSCoordinate(), geoops::WaypointType::eUNKNOWN, 20.0, 0}};

    logging::graphing::PlotCoordinates2D(vWaypoints, "WaypointPlotTest");

    // We can't really control the gnuplot backend or when it saves the file, so we need to sleep for a bit to make sure the file is saved.
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::string szFileName = logging::g_szLoggingOutputPath + "/path_plots/WaypointPlotTest.png";
    EXPECT_TRUE(std::filesystem::exists(szFileName));
}

/******************************************************************************
 * @brief Test the functionality of the PlotCoordinates3D function for UTM coordinates.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PlotsAndGraphsTests, PlotCoordinates3D_UTM)
{
    std::vector<geoops::UTMCoordinate> vCoordinates = {{500000, 4649776.22482, 33, true, 100},
                                                       {500100, 4649876.22482, 33, true, 200},
                                                       {500200, 4649976.22482, 33, true, 300}};

    logging::graphing::PlotCoordinates3D(vCoordinates, "UTMCoordinatePlot3DTest");

    // We can't really control the gnuplot backend or when it saves the file, so we need to sleep for a bit to make sure the file is saved.
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::string szFileName = logging::g_szLoggingOutputPath + "/path_plots/UTMCoordinatePlot3DTest.png";
    EXPECT_TRUE(std::filesystem::exists(szFileName));
}

/******************************************************************************
 * @brief Test the functionality of the PlotCoordinates3D function for GPS coordinates.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PlotsAndGraphsTests, PlotCoordinates3D_GPS)
{
    std::vector<geoops::GPSCoordinate> vCoordinates = {{37.7749, -122.4194, 100}, {34.0522, -118.2437, 200}, {40.7128, -74.0060, 300}};

    logging::graphing::PlotCoordinates3D(vCoordinates, "GPSCoordinatePlot3DTest");

    // We can't really control the gnuplot backend or when it saves the file, so we need to sleep for a bit to make sure the file is saved.
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::string szFileName = logging::g_szLoggingOutputPath + "/path_plots/GPSCoordinatePlot3DTest.png";
    EXPECT_TRUE(std::filesystem::exists(szFileName));
}

/******************************************************************************
 * @brief Test the functionality of the PlotCoordinates3D function for waypoints.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-10
 ******************************************************************************/
TEST_F(PlotsAndGraphsTests, PlotCoordinates3D_Waypoints)
{
    std::vector<geoops::Waypoint> vWaypoints = {{geoops::GPSCoordinate(), geoops::WaypointType::eUNKNOWN, 20.0, 0},
                                                {geoops::GPSCoordinate(), geoops::WaypointType::eUNKNOWN, 20.0, 0},
                                                {geoops::GPSCoordinate(), geoops::WaypointType::eUNKNOWN, 20.0, 0}};

    logging::graphing::PlotCoordinates3D(vWaypoints, "WaypointPlot3DTest");

    // We can't really control the gnuplot backend or when it saves the file, so we need to sleep for a bit to make sure the file is saved.
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::string szFileName = logging::g_szLoggingOutputPath + "/path_plots/WaypointPlot3DTest.png";
    EXPECT_TRUE(std::filesystem::exists(szFileName));
}
