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

/// \cond
#include <array>
#include <chrono>
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Check that AStar doesn't leak any memory.
 *
 *
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-04-28
 ******************************************************************************/
TEST(AStarPlannerTest, DoesNotLeak)
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
TEST(AStarPlannerTest, Leaks)
{
    // Create a new AStar object.
    pathplanners::AStar* pAStar = new pathplanners::AStar();
    EXPECT_TRUE(pAStar != nullptr);
}

/******************************************************************************
 * @brief Test AStar path planning functionality.
 *
 *
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-04-28
 ******************************************************************************/
TEST(AStarPlannerTest, PlanAvoidancePath)
{
    // Create a new AStar object.
    pathplanners::AStar* pAStar = new pathplanners::AStar();

    size_t siTestValuesLength   = 8;

    // Create start coordinate for AStar.
    const double dEastingStart          = 608120.0;
    const double dNorthingStart         = 4201140.0;
    const geoops::UTMCoordinate stStart = geoops::UTMCoordinate(dEastingStart, dNorthingStart, 15);

    // Create goal coordinates for AStar.
    const geoops::UTMCoordinate aGoalCoordinates[siTestValuesLength] = {
        geoops::UTMCoordinate(dEastingStart, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, 15),                                           // N
        geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart, 15),                                           // E
        geoops::UTMCoordinate(dEastingStart, dNorthingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, 15),                                           // S
        geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart, 15),                                           // W
        geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, 15),    // NE
        geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, 15),    // SE
        geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, 15),    // SW
        geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, 15)     // NW
    };

    // Compare output paths with expected paths.
    for (size_t siIter = 0; siIter < siTestValuesLength; siIter++)
    {
        // Generate a path for this goal.
        std::vector<geoops::UTMCoordinate> vReturnedPath = pAStar->PlanAvoidancePath(stStart, aGoalCoordinates[siIter]);

        // Validate that each node is separated by a valid distance.
        // (no more than a node size difference between each coordinate value).
        for (size_t siPathIter = 1; siPathIter < vReturnedPath.size(); siPathIter++)
        {
            bool bValidNodeDistance = std::abs(vReturnedPath[siPathIter - 1].dEasting - vReturnedPath[siPathIter].dEasting) <= constants::ASTAR_NODE_SIZE;
            bValidNodeDistance =
                bValidNodeDistance && std::abs(vReturnedPath[siPathIter - 1].dNorthing - vReturnedPath[siPathIter].dNorthing) <= constants::ASTAR_NODE_SIZE;

            EXPECT_TRUE(bValidNodeDistance);
        }

        // Validate start coordinate.
        EXPECT_NEAR(stStart.dEasting, vReturnedPath[0].dEasting, 0.1);
        EXPECT_NEAR(stStart.dNorthing, vReturnedPath[0].dNorthing, 0.1);

        // Validate end coordinate.
        EXPECT_NEAR(aGoalCoordinates[siIter].dEasting, vReturnedPath.back().dEasting, 0.1);
        EXPECT_NEAR(aGoalCoordinates[siIter].dNorthing, vReturnedPath.back().dNorthing, 0.1);
    }

    // Cleanup.
    delete pAStar;
    pAStar = nullptr;
}

/******************************************************************************
 * @brief Test AStar obstacle initialization.
 *
 *
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-09-15
 ******************************************************************************/
TEST(AStarPlannerTest, ObstacleInitialization)
{
    // Create a new AStar object.
    pathplanners::AStar* pAStar = new pathplanners::AStar();

    // Create obstacle for AStar initialization.
    const geoops::UTMCoordinate stObstacleCenter   = geoops::UTMCoordinate(608120, 4201140, 15);
    const double dObstacleSize                     = 3 * constants::ASTAR_NODE_SIZE;
    const pathplanners::AStar::Obstacle stObstacle = {stObstacleCenter, dObstacleSize};

    // Add obstacle to AStar.
    pAStar->AddObstacle(stObstacle);

    // Validate obstacle exists within AStar.
    std::vector<pathplanners::AStar::Obstacle> vReturnVector = pAStar->GetObstacleData();
    EXPECT_NEAR(stObstacle.stCenterPoint.dEasting, vReturnVector[0].stCenterPoint.dEasting, 0.1);
    EXPECT_NEAR(stObstacle.stCenterPoint.dNorthing, vReturnVector[0].stCenterPoint.dNorthing, 0.1);
    EXPECT_NEAR(stObstacle.dRadius, vReturnVector[0].dRadius, 0.1);

    // Create obstacle vector for AStar re-initialization.
    std::vector<pathplanners::AStar::Obstacle> vObstacles;
    const geoops::UTMCoordinate stObstacle2Center   = geoops::UTMCoordinate(608100, 4201100, 15);
    const double dObstacle2Size                     = 2 * constants::ASTAR_NODE_SIZE;
    const pathplanners::AStar::Obstacle stObstacle2 = {stObstacle2Center, dObstacle2Size};
    vObstacles.emplace_back(stObstacle);
    vObstacles.emplace_back(stObstacle2);

    // Reset obstacles within AStar.
    pAStar->UpdateObstacleData(vObstacles, true);

    // Validate obstacles exist within AStar.
    vReturnVector       = pAStar->GetObstacleData();
    size_t siVectorSize = vReturnVector.size();
    for (size_t siCounter = 0; siCounter < siVectorSize; siCounter++)
    {
        EXPECT_NEAR(vObstacles[siCounter].stCenterPoint.dEasting, vReturnVector[siCounter].stCenterPoint.dEasting, 0.1);
        EXPECT_NEAR(vObstacles[siCounter].stCenterPoint.dNorthing, vReturnVector[siCounter].stCenterPoint.dNorthing, 0.1);
        EXPECT_NEAR(vObstacles[siCounter].dRadius, vReturnVector[siCounter].dRadius, 0.1);
    }

    // Cleanup.
    delete pAStar;
    pAStar = nullptr;
}
