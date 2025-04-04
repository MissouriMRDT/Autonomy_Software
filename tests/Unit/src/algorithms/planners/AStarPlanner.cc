/******************************************************************************
 * @brief Unit test for AStar algorithm class.
 *
 * @file AStarPlanner.cc
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-4-28
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/algorithms/planners/AStar.h"
#include "../../../../TestingBase.hh"

/// \cond
#include <array>
#include <chrono>
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the AStar Planner
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class AStarPlannerTests : public TestingBase<AStarPlannerTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new AStarPlannerTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        AStarPlannerTests() {}

        /******************************************************************************
         * @brief Destroy the AStarPlannerTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~AStarPlannerTests() {}

        /******************************************************************************
         * @brief Setup the AStarPlannerTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Teardown the AStarPlannerTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Check that AStar doesn't leak any memory.
 *
 *
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-04-28
 ******************************************************************************/
TEST_F(AStarPlannerTests, DoesNotLeak)
{
    // Create a new AStar object.
    pathplanners::AStar* pAStar = new pathplanners::AStar();
    // Delete object.
    delete pAStar;
    // Point to null.
    pAStar = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 *
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-04-28
 ******************************************************************************/
TEST_F(AStarPlannerTests, Leaks)
{
    // Create a new AStar object.
    pathplanners::AStar* pAStar = new pathplanners::AStar();
    EXPECT_TRUE(pAStar != nullptr);
}

/******************************************************************************
 * @brief Test AStar path planning functionality with Waypoints.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-03-05
 ******************************************************************************/
TEST_F(AStarPlannerTests, PlanAvoidancePathWaypoints)
{
    // Create a new AStar object.
    std::shared_ptr<pathplanners::AStar> pAStar = std::make_shared<pathplanners::AStar>();

    size_t siTestValuesLength                   = 8;

    // Create start coordinate for AStar.
    const double dEastingStart     = 608120.0;
    const double dNorthingStart    = 4201140.0;
    const geoops::Waypoint stStart = geoops::Waypoint(geoops::UTMCoordinate(dEastingStart, dNorthingStart, 15));

    // Create goal coordinates for AStar.
    const geoops::Waypoint aGoalCoordinates[siTestValuesLength] = {
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart, dNorthingStart + constants::ASTAR_MAX_SEARCH_GRID, 15)),                                       // N
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart, 15)),                                       // E
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart, dNorthingStart - constants::ASTAR_MAX_SEARCH_GRID, 15)),                                       // S
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart, 15)),                                       // W
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart + constants::ASTAR_MAX_SEARCH_GRID, 15)),    // NE
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart - constants::ASTAR_MAX_SEARCH_GRID, 15)),    // SE
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart - constants::ASTAR_MAX_SEARCH_GRID, 15)),    // SW
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart + constants::ASTAR_MAX_SEARCH_GRID, 15))     // NW
    };

    // Compare output paths with expected paths.
    for (size_t siIter = 0; siIter < siTestValuesLength; siIter++)
    {
        // Generate a path for this goal.
        std::vector<geoops::Waypoint> vReturnedPath = pAStar->PlanAvoidancePath(stStart, aGoalCoordinates[siIter]);

        // Validate that each node is separated by a valid distance.
        // (no more than a node size difference between each coordinate value).
        for (size_t siPathIter = 1; siPathIter < vReturnedPath.size(); siPathIter++)
        {
            bool bValidNodeDistance =
                std::abs(vReturnedPath[siPathIter - 1].GetUTMCoordinate().dEasting - vReturnedPath[siPathIter].GetUTMCoordinate().dEasting) <= constants::ASTAR_NODE_SIZE;
            bValidNodeDistance = bValidNodeDistance && std::abs(vReturnedPath[siPathIter - 1].GetUTMCoordinate().dNorthing -
                                                                vReturnedPath[siPathIter].GetUTMCoordinate().dNorthing) <= constants::ASTAR_NODE_SIZE;

            EXPECT_TRUE(bValidNodeDistance);
        }

        // Validate start coordinate.
        EXPECT_NEAR(stStart.GetUTMCoordinate().dEasting, vReturnedPath[0].GetUTMCoordinate().dEasting, 0.1);
        EXPECT_NEAR(stStart.GetUTMCoordinate().dNorthing, vReturnedPath[0].GetUTMCoordinate().dNorthing, 0.1);

        // Validate end coordinate.
        EXPECT_NEAR(aGoalCoordinates[siIter].GetUTMCoordinate().dEasting, vReturnedPath.back().GetUTMCoordinate().dEasting, 0.1);
        EXPECT_NEAR(aGoalCoordinates[siIter].GetUTMCoordinate().dNorthing, vReturnedPath.back().GetUTMCoordinate().dNorthing, 0.1);
    }
}

/******************************************************************************
 * @brief Test AStar path planning functionality.
 *
 *
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-04-28
 ******************************************************************************/
TEST_F(AStarPlannerTests, PlanAvoidancePathUTMCoordinates)
{
    // Create a new AStar object.
    std::shared_ptr<pathplanners::AStar> pAStar = std::make_shared<pathplanners::AStar>();

    size_t siTestValuesLength                   = 8;

    // Create start coordinate for AStar.
    const double dEastingStart          = 608120.0;
    const double dNorthingStart         = 4201140.0;
    const geoops::UTMCoordinate stStart = geoops::UTMCoordinate(dEastingStart, dNorthingStart, 15);

    // Create goal coordinates for AStar.
    const geoops::UTMCoordinate aGoalCoordinates[siTestValuesLength] = {
        geoops::UTMCoordinate(dEastingStart, dNorthingStart + constants::ASTAR_MAX_SEARCH_GRID, 15),                                       // N
        geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart, 15),                                       // E
        geoops::UTMCoordinate(dEastingStart, dNorthingStart - constants::ASTAR_MAX_SEARCH_GRID, 15),                                       // S
        geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart, 15),                                       // W
        geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart + constants::ASTAR_MAX_SEARCH_GRID, 15),    // NE
        geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart - constants::ASTAR_MAX_SEARCH_GRID, 15),    // SE
        geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart - constants::ASTAR_MAX_SEARCH_GRID, 15),    // SW
        geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart + constants::ASTAR_MAX_SEARCH_GRID, 15)     // NW
    };

    // Compare output paths with expected paths.
    for (size_t siIter = 0; siIter < siTestValuesLength; siIter++)
    {
        // Generate a path for this goal.
        std::vector<geoops::Waypoint> vReturnedPath = pAStar->PlanAvoidancePath(stStart, aGoalCoordinates[siIter]);

        // Validate that each node is separated by a valid distance.
        // (no more than a node size difference between each coordinate value).
        for (size_t siPathIter = 1; siPathIter < vReturnedPath.size(); siPathIter++)
        {
            bool bValidNodeDistance =
                std::abs(vReturnedPath[siPathIter - 1].GetUTMCoordinate().dEasting - vReturnedPath[siPathIter].GetUTMCoordinate().dEasting) <= constants::ASTAR_NODE_SIZE;
            bValidNodeDistance = bValidNodeDistance && std::abs(vReturnedPath[siPathIter - 1].GetUTMCoordinate().dNorthing -
                                                                vReturnedPath[siPathIter].GetUTMCoordinate().dNorthing) <= constants::ASTAR_NODE_SIZE;

            EXPECT_TRUE(bValidNodeDistance);
        }

        // Validate start coordinate.
        EXPECT_NEAR(stStart.dEasting, vReturnedPath[0].GetUTMCoordinate().dEasting, 0.1);
        EXPECT_NEAR(stStart.dNorthing, vReturnedPath[0].GetUTMCoordinate().dNorthing, 0.1);

        // Validate end coordinate.
        EXPECT_NEAR(aGoalCoordinates[siIter].dEasting, vReturnedPath.back().GetUTMCoordinate().dEasting, 0.1);
        EXPECT_NEAR(aGoalCoordinates[siIter].dNorthing, vReturnedPath.back().GetUTMCoordinate().dNorthing, 0.1);
    }
}

/******************************************************************************
 * @brief Test AStar path planning functionality with GPSCoordinates.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-03-05
 ******************************************************************************/
TEST_F(AStarPlannerTests, PlanAvoidancePathGPSCoordinates)
{
    // Create a new AStar object.
    std::shared_ptr<pathplanners::AStar> pAStar = std::make_shared<pathplanners::AStar>();

    size_t siTestValuesLength                   = 8;

    // Create start coordinate for AStar.
    const double dEastingStart          = 608120.0;
    const double dNorthingStart         = 4201140.0;
    const geoops::GPSCoordinate stStart = geoops::Waypoint(geoops::UTMCoordinate(dEastingStart, dNorthingStart, 15)).GetGPSCoordinate();

    // Create goal coordinates for AStar.
    const geoops::GPSCoordinate aGoalCoordinates[siTestValuesLength] = {
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart, dNorthingStart + constants::ASTAR_MAX_SEARCH_GRID, 15)).GetGPSCoordinate(),    // N
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart, 15)).GetGPSCoordinate(),    // E
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart, dNorthingStart - constants::ASTAR_MAX_SEARCH_GRID, 15)).GetGPSCoordinate(),    // S
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart, 15)).GetGPSCoordinate(),    // W
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart + constants::ASTAR_MAX_SEARCH_GRID, 15))
            .GetGPSCoordinate(),                                                                                                             // NE
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart - constants::ASTAR_MAX_SEARCH_GRID, 15))
            .GetGPSCoordinate(),                                                                                                             // SE
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart - constants::ASTAR_MAX_SEARCH_GRID, 15))
            .GetGPSCoordinate(),                                                                                                             // SW
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAX_SEARCH_GRID, dNorthingStart + constants::ASTAR_MAX_SEARCH_GRID, 15))
            .GetGPSCoordinate()                                                                                                              // NW
    };

    // Compare output paths with expected paths.
    for (size_t siIter = 0; siIter < siTestValuesLength; siIter++)
    {
        // Generate a path for this goal.
        std::vector<geoops::Waypoint> vReturnedPath = pAStar->PlanAvoidancePath(stStart, aGoalCoordinates[siIter]);

        // Validate that each node is separated by a valid distance.
        // (no more than a node size difference between each coordinate value).
        for (size_t siPathIter = 1; siPathIter < vReturnedPath.size(); siPathIter++)
        {
            bool bValidNodeDistance =
                std::abs(vReturnedPath[siPathIter - 1].GetUTMCoordinate().dEasting - vReturnedPath[siPathIter].GetUTMCoordinate().dEasting) <= constants::ASTAR_NODE_SIZE;
            bValidNodeDistance = bValidNodeDistance && std::abs(vReturnedPath[siPathIter - 1].GetUTMCoordinate().dNorthing -
                                                                vReturnedPath[siPathIter].GetUTMCoordinate().dNorthing) <= constants::ASTAR_NODE_SIZE;

            EXPECT_TRUE(bValidNodeDistance);
        }

        // Validate start coordinate.
        EXPECT_NEAR(stStart.dLatitude, vReturnedPath[0].GetGPSCoordinate().dLatitude, 0.1);
        EXPECT_NEAR(stStart.dLongitude, vReturnedPath[0].GetGPSCoordinate().dLongitude, 0.1);

        // Validate end coordinate.
        EXPECT_NEAR(aGoalCoordinates[siIter].dLatitude, vReturnedPath.back().GetGPSCoordinate().dLatitude, 0.1);
        EXPECT_NEAR(aGoalCoordinates[siIter].dLongitude, vReturnedPath.back().GetGPSCoordinate().dLongitude, 0.1);
    }
}

/******************************************************************************
 * @brief Test AStar's ability to still plan a path when obstacles overlap the
 *      start and end coordinates. The start and end of the path will just shift
 *      around until they are no longer blocked.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-03-05
 ******************************************************************************/
TEST_F(AStarPlannerTests, PlanAvoidancePathStartEndBlocked)
{
    // Create a new AStar object.
    std::shared_ptr<pathplanners::AStar> pAStar = std::make_shared<pathplanners::AStar>();

    // Create start coordinate for AStar.
    const double dEastingStart          = 608120.0;
    const double dNorthingStart         = 4201140.0;
    const geoops::UTMCoordinate stStart = geoops::UTMCoordinate(dEastingStart, dNorthingStart, 15);

    // Create goal coordinates for AStar.
    const geoops::UTMCoordinate stEnd(dEastingStart, dNorthingStart + 20, 15);

    // Create obstacle for AStar initialization.
    const geoops::UTMCoordinate stStartObstacleCenter = geoops::UTMCoordinate(dEastingStart, dNorthingStart - 1, 15);
    const geoops::UTMCoordinate stEndObstacleCenter   = geoops::UTMCoordinate(dEastingStart, dNorthingStart + 21, 15);
    const double dObstacleSize                        = 3;
    const geoops::Waypoint stObstacle1                = {stStartObstacleCenter, geoops::WaypointType::eObstacleWaypoint, dObstacleSize};
    const geoops::Waypoint stObstacle2                = {stEndObstacleCenter, geoops::WaypointType::eObstacleWaypoint, dObstacleSize};

    // Add obstacle to AStar.
    pAStar->UpsertObstacleData(stObstacle1);
    pAStar->UpsertObstacleData(stObstacle2);

    // Generate a path for this goal.
    std::vector<geoops::Waypoint> vReturnedPath = pAStar->PlanAvoidancePath(stStart, stEnd);

    // Validate that the path has some points.
    EXPECT_FALSE(vReturnedPath.empty());
}

/******************************************************************************
 * @brief Test AStar path planning cancellation feature.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-03-05
 ******************************************************************************/
TEST_F(AStarPlannerTests, PlanAvoidancePathCancel)
{
    // Create a new AStar object.
    std::shared_ptr<pathplanners::AStar> pAStar = std::make_shared<pathplanners::AStar>();

    size_t siTestValuesLength                   = 8;

    // Create start coordinate for AStar.
    const double dEastingStart          = 608120.0;
    const double dNorthingStart         = 4201140.0;
    const geoops::UTMCoordinate stStart = geoops::UTMCoordinate(dEastingStart, dNorthingStart, 15);

    // Create goal coordinates for AStar.
    const geoops::UTMCoordinate stEnd(dEastingStart - constants::ASTAR_MAX_SEARCH_GRID * 1000, dNorthingStart + constants::ASTAR_MAX_SEARCH_GRID * 1000, 15);

    // Get and store start time.
    auto tStartTime = std::chrono::high_resolution_clock::now();
    // Start planning avoidance path in a separate thread.
    std::thread tPathThread([&] { pAStar->PlanAvoidancePath(stStart, stEnd); });
    // Sleep for a short time to allow the path to start generating.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Cancel the path generation.
    pAStar->CancelPathGeneration();
    // Join the thread.
    tPathThread.join();
    // Get and store end time.
    auto tEndTime = std::chrono::high_resolution_clock::now();

    // Calculate the time difference.
    std::chrono::duration<double> dElapsedTime = tEndTime - tStartTime;
    // Check if the time difference is less than 1 second. If it doesn't cancel, it will take longer than 1 second.
    EXPECT_LT(dElapsedTime.count(), 1);
    // Make sure the path is empty.
    EXPECT_TRUE(pAStar->GetPath().empty());
}

/******************************************************************************
 * @brief Test AStar obstacle initialization.
 *
 *
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-09-15
 ******************************************************************************/
TEST_F(AStarPlannerTests, ObstacleInitialization)
{
    // Create a new AStar object.
    std::shared_ptr<pathplanners::AStar> pAStar = std::make_shared<pathplanners::AStar>();

    // Create obstacle for AStar initialization.
    const geoops::UTMCoordinate stObstacleCenter = geoops::UTMCoordinate(608120, 4201140, 15);
    const double dObstacleSize                   = 3 * constants::ASTAR_NODE_SIZE;
    const geoops::Waypoint stObstacle            = {stObstacleCenter, geoops::WaypointType::eObstacleWaypoint, dObstacleSize};

    // Add obstacle to AStar.
    pAStar->UpsertObstacleData(stObstacle);

    // Validate obstacle exists within AStar.
    std::vector<geoops::Waypoint> vReturnVector = pAStar->GetObstacleData();
    EXPECT_NEAR(stObstacle.GetUTMCoordinate().dEasting, vReturnVector[0].GetUTMCoordinate().dEasting, 0.1);
    EXPECT_NEAR(stObstacle.GetUTMCoordinate().dNorthing, vReturnVector[0].GetUTMCoordinate().dNorthing, 0.1);
    EXPECT_NEAR(stObstacle.dRadius, vReturnVector[0].dRadius, 0.1);

    // Create obstacle vector for AStar re-initialization.
    std::vector<geoops::Waypoint> vObstacles;
    const geoops::UTMCoordinate stObstacle2Center = geoops::UTMCoordinate(608100, 4201100, 15);
    const double dObstacle2Size                   = 2 * constants::ASTAR_NODE_SIZE;
    const geoops::Waypoint stObstacle2            = {stObstacle2Center, geoops::WaypointType::eObstacleWaypoint, dObstacle2Size};
    vObstacles.emplace_back(stObstacle);
    vObstacles.emplace_back(stObstacle2);

    // Reset obstacles within AStar.
    pAStar->UpsertObstacleData(vObstacles);

    // Validate obstacles exist within AStar.
    vReturnVector       = pAStar->GetObstacleData();
    size_t siVectorSize = vReturnVector.size();
    for (size_t siCounter = 0; siCounter < siVectorSize; siCounter++)
    {
        EXPECT_NEAR(vObstacles[siCounter].GetUTMCoordinate().dEasting, vReturnVector[siCounter].GetUTMCoordinate().dEasting, 0.1);
        EXPECT_NEAR(vObstacles[siCounter].GetUTMCoordinate().dNorthing, vReturnVector[siCounter].GetUTMCoordinate().dNorthing, 0.1);
        EXPECT_NEAR(vObstacles[siCounter].dRadius, vReturnVector[siCounter].dRadius, 0.1);
    }
}

/******************************************************************************
 * @brief Test AStar obstacle data management functionality.
 *
 * @author GitHub Copilot
 * @date 2025-03-05
 ******************************************************************************/
TEST_F(AStarPlannerTests, UpsertObstacleData)
{
    // Create a new AStar object
    std::shared_ptr<pathplanners::AStar> pAStar = std::make_shared<pathplanners::AStar>();

    // Test single Waypoint obstacle
    const geoops::UTMCoordinate stObstacle1UTM(608120, 4201140, 15);
    const geoops::Waypoint stObstacle1(stObstacle1UTM, geoops::WaypointType::eObstacleWaypoint, 3.0);
    pAStar->UpsertObstacleData(stObstacle1);

    // Verify single Waypoint was added
    std::vector<geoops::Waypoint> vObstacles = pAStar->GetObstacleData();
    ASSERT_EQ(vObstacles.size(), 1);
    EXPECT_EQ(vObstacles[0], stObstacle1);

    // Test duplicate Waypoint (should not be added)
    pAStar->UpsertObstacleData(stObstacle1);
    EXPECT_EQ(pAStar->GetObstacleData().size(), 1);

    // Test single UTMCoordinate obstacle
    const geoops::UTMCoordinate stObstacle2UTM(608140, 4201160, 15);
    pAStar->UpsertObstacleData(stObstacle2UTM);
    vObstacles = pAStar->GetObstacleData();
    ASSERT_EQ(vObstacles.size(), 2);
    EXPECT_EQ(vObstacles[1].GetUTMCoordinate(), stObstacle2UTM);

    // Test single GPSCoordinate obstacle
    const geoops::GPSCoordinate stObstacle3GPS(37.9513, -91.7850);
    pAStar->UpsertObstacleData(stObstacle3GPS);
    vObstacles = pAStar->GetObstacleData();
    ASSERT_EQ(vObstacles.size(), 3);
    EXPECT_NEAR(vObstacles[2].GetGPSCoordinate().dLatitude, stObstacle3GPS.dLatitude, 0.0001);
    EXPECT_NEAR(vObstacles[2].GetGPSCoordinate().dLongitude, stObstacle3GPS.dLongitude, 0.0001);

    // Clear obstacles and verify
    pAStar->ClearObstacleData();
    EXPECT_EQ(pAStar->GetObstacleData().size(), 0);

    // Test vector of Waypoints
    std::vector<geoops::Waypoint> vWaypointObstacles;
    vWaypointObstacles.push_back(stObstacle1);
    vWaypointObstacles.push_back(geoops::Waypoint(stObstacle2UTM));
    pAStar->UpsertObstacleData(vWaypointObstacles);
    EXPECT_EQ(pAStar->GetObstacleData().size(), 2);

    // Test vector of UTMCoordinates
    std::vector<geoops::UTMCoordinate> vUTMObstacles;
    vUTMObstacles.push_back(stObstacle1UTM);
    vUTMObstacles.push_back(stObstacle2UTM);
    pAStar->ClearObstacleData();
    pAStar->UpsertObstacleData(vUTMObstacles);
    EXPECT_EQ(pAStar->GetObstacleData().size(), 2);

    // Test vector of GPSCoordinates
    std::vector<geoops::GPSCoordinate> vGPSObstacles;
    vGPSObstacles.push_back(stObstacle3GPS);
    vGPSObstacles.push_back(geoops::GPSCoordinate(37.9514, -91.7851));
    pAStar->ClearObstacleData();
    pAStar->UpsertObstacleData(vGPSObstacles);
    EXPECT_EQ(pAStar->GetObstacleData().size(), 2);
}

/******************************************************************************
 * @brief Test AStar path obstacle avoidance for cardinal and diagonal directions.
 *
 *
 * @author Sam Nolte (samnolte0302@gmail.com)
 * @date 2024-11-19
 ******************************************************************************/
TEST_F(AStarPlannerTests, AvoidObstaclesWhilePathing)
{
    // Create a new AStar object
    pathplanners::AStar* pAStar = new pathplanners::AStar();

    // Start coordinate for AStar
    const double dEastingStart  = 608120.0;
    const double dNorthingStart = 4201140.0;
    const geoops::UTMCoordinate stStartCoordinate(dEastingStart, dNorthingStart, 15, true);

    // Create goal coordinates for AStar
    const std::vector<geoops::UTMCoordinate> aGoalCoordinates = {
        geoops::UTMCoordinate(dEastingStart, dNorthingStart + 10, 15, true),         // N
        geoops::UTMCoordinate(dEastingStart + 10, dNorthingStart, 15, true),         // E
        geoops::UTMCoordinate(dEastingStart, dNorthingStart - 10, 15, true),         // S
        geoops::UTMCoordinate(dEastingStart - 10, dNorthingStart, 15, true),         // W
        geoops::UTMCoordinate(dEastingStart + 10, dNorthingStart + 10, 15, true),    // NE
        geoops::UTMCoordinate(dEastingStart + 10, dNorthingStart - 10, 15, true),    // SE
        geoops::UTMCoordinate(dEastingStart - 10, dNorthingStart - 10, 15, true),    // SW
        geoops::UTMCoordinate(dEastingStart - 10, dNorthingStart + 10, 15, true)     // NW
    };

    // Create obstacle coordinates for AStar
    const double dObstacleSize                     = 3 * constants::ASTAR_NODE_SIZE;
    const std::vector<geoops::Waypoint> aObstacles = {
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart, dNorthingStart + 5, 15, true), geoops::WaypointType::eObstacleWaypoint, dObstacleSize),        // N
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart + 5, dNorthingStart, 15, true), geoops::WaypointType::eObstacleWaypoint, dObstacleSize),        // E
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart, dNorthingStart - 5, 15, true), geoops::WaypointType::eObstacleWaypoint, dObstacleSize),        // S
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart - 5, dNorthingStart, 15, true), geoops::WaypointType::eObstacleWaypoint, dObstacleSize),        // W
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart + 5, dNorthingStart + 5, 15, true), geoops::WaypointType::eObstacleWaypoint, dObstacleSize),    // NE
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart + 5, dNorthingStart - 5, 15, true), geoops::WaypointType::eObstacleWaypoint, dObstacleSize),    // SE
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart - 5, dNorthingStart - 5, 15, true), geoops::WaypointType::eObstacleWaypoint, dObstacleSize),    // SW
        geoops::Waypoint(geoops::UTMCoordinate(dEastingStart - 5, dNorthingStart + 5, 15, true), geoops::WaypointType::eObstacleWaypoint, dObstacleSize)     // NW
    };

    for (size_t siI = 0; siI < 8; siI++)
    {
        // Add obstacle to AStar
        pAStar->ClearObstacleData();
        pAStar->UpsertObstacleData(std::vector<geoops::Waypoint>{aObstacles[siI]});

        // Get AStar path
        std::vector<geoops::Waypoint> vReturnedPath = pAStar->PlanAvoidancePath(stStartCoordinate, aGoalCoordinates[siI]);

        // Make sure AStar actually found a path
        EXPECT_TRUE(vReturnedPath.size() != 0);

        // Check for pathing through obstacles
        for (size_t siJ = 0; siJ < vReturnedPath.size(); siJ++)
        {
            // Check to see if current coordinate is within obstacle bounds
            EXPECT_FALSE(vReturnedPath[siJ].GetUTMCoordinate().dNorthing >= aObstacles[siI].GetUTMCoordinate().dNorthing - aObstacles[siI].dRadius &&
                         vReturnedPath[siJ].GetUTMCoordinate().dNorthing <= aObstacles[siI].GetUTMCoordinate().dNorthing + aObstacles[siI].dRadius &&
                         vReturnedPath[siJ].GetUTMCoordinate().dEasting >= aObstacles[siI].GetUTMCoordinate().dEasting - aObstacles[siI].dRadius &&
                         vReturnedPath[siJ].GetUTMCoordinate().dEasting <= aObstacles[siI].GetUTMCoordinate().dEasting + aObstacles[siI].dRadius);
        }

        // Make sure path hit goal point
        EXPECT_NEAR(aGoalCoordinates[siI].dEasting, vReturnedPath.back().GetUTMCoordinate().dEasting, 0.1);
        EXPECT_NEAR(aGoalCoordinates[siI].dNorthing, vReturnedPath.back().GetUTMCoordinate().dNorthing, 0.1);
    }

    // Cleanup
    delete pAStar;
    pAStar = nullptr;
}

TEST_F(AStarPlannerTests, Maze)
{
    // Create a new AStar object
    pathplanners::AStar* pAStar = new pathplanners::AStar();

    // Start coordinate for AStar
    const double dEastingStart  = 50.0;
    const double dNorthingStart = 50.0;
    const geoops::UTMCoordinate stStartCoordinate(dEastingStart, dNorthingStart, 15, true);

    // Create goal coordinates for AStar
    const geoops::UTMCoordinate stGoalCoordinate = geoops::UTMCoordinate(dEastingStart, dNorthingStart + 10, 15, true);

    // Create obstacle coordinates for AStar https://imgur.com/a/pWpyLAI
    const std::vector<geoops::Waypoint> aObstacles = {
        geoops::Waypoint(geoops::UTMCoordinate(50, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50.5, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51.5, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52.5, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 50, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 50.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 51, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 51.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 52, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 52.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 53, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 53.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 54, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 54.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 55, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 55.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 56, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53.5, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52.5, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51.5, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50.5, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48.5, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47.5, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 56.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 56, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 55.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 55, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 54.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 54, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 53.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 53, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 52.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 52, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 51.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 51, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 50.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 50, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(46.5, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47.5, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48.5, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 49.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51.5, 50.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52, 50.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52.5, 50.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52.5, 51, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52.5, 51.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52.5, 52, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52.5, 52.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52.5, 53, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52.5, 53.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52, 53.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51.5, 53.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51, 53.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50.5, 53.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50.5, 54, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50.5, 54.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50.5, 55, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50.5, 55.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50.5, 53, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50.5, 52.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50, 52.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 52.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 52, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 51.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49, 51.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48.5, 51.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48.5, 52, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48.5, 52.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 53, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 53.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 54, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 54.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49, 54.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48.5, 54.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48.5, 55, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48.5, 55.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48, 55.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47.5, 55.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 55, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 55.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 56, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47.5, 54.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47, 54.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51.5, 54.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51.5, 55, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51.5, 55.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51, 55.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52, 55.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52.5, 55.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(53, 54.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(52.5, 54.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 50, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49.5, 50.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50, 50.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50.5, 50.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50.5, 51, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(50.5, 51.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51, 51.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51.5, 51.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51.5, 52, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(51.5, 52.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(49, 50.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48.5, 50.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48, 50.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47.5, 50.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47.5, 51, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47.5, 51.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47.5, 52, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47.5, 52.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47.5, 53, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(47.5, 53.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48, 53.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
        geoops::Waypoint(geoops::UTMCoordinate(48.5, 53.5, 15, true), geoops::WaypointType::eObstacleWaypoint, 0.1),
    };

    // Add obstacle to AStar
    pAStar->UpsertObstacleData(aObstacles);

    // Make sure AStar paths
    std::vector<geoops::Waypoint> vReturnedPath = pAStar->PlanAvoidancePath(stStartCoordinate, stGoalCoordinate);
    EXPECT_TRUE(vReturnedPath.size() != 0);

    // Make sure path hit goal point
    EXPECT_NEAR(stGoalCoordinate.dEasting, vReturnedPath.back().GetUTMCoordinate().dEasting, 0.1);
    EXPECT_NEAR(stGoalCoordinate.dNorthing, vReturnedPath.back().GetUTMCoordinate().dNorthing, 0.1);

    // Cleanup
    delete pAStar;
    pAStar = nullptr;
}
