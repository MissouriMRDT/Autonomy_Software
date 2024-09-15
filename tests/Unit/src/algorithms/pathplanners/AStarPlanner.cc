/******************************************************************************
 * @brief Unit test for AStar algorithm class.
 *
 * @file AStarPlanner.cc
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-4-28
 *
 * @copyright Copyright MRDT 2024 - All Rights Reserved
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
 * @brief Test ASTAR boundary point functionality.
 *
 *
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-04-28
 ******************************************************************************/
TEST(AStarPlannerTest, BoundaryPoint)
{
    // Create a new AStar object.
    pathplanners::AStar* pAStar = new pathplanners::AStar();

    // Initialize start coordinate in AStar object.
    geoops::UTMCoordinate stStartCoord = geoops::UTMCoordinate(50.0, 50.0);
    pAStar->SetStartCoordinate(stStartCoord);

    size_t siTestValuesLength = 4;

    // Initialize array with coordinates external to the AStar search grid.
    const geoops::UTMCoordinate aOutsideCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(50.0, 65.0),
                                                                           geoops::UTMCoordinate(65.0, 50.0),
                                                                           geoops::UTMCoordinate(50.0, 35.0),
                                                                           geoops::UTMCoordinate(35.0, 50.0)};

    // Initialize array with coordinates internal to the AStar search grid.
    const geoops::UTMCoordinate aInsideCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(50.0, 55.0),
                                                                          geoops::UTMCoordinate(55.0, 50.0),
                                                                          geoops::UTMCoordinate(50.0, 45.0),
                                                                          geoops::UTMCoordinate(45.0, 50.0)};

    //  Initialize array with corner coordinates on the boundary.
    const geoops::UTMCoordinate aCornerCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(60.0, 60.0),
                                                                          geoops::UTMCoordinate(60.0, 40.0),
                                                                          geoops::UTMCoordinate(40.0, 40.0),
                                                                          geoops::UTMCoordinate(40.0, 60.0)};
    // Initialize arrays with correct bounded values
    const geoops::UTMCoordinate aExpectedCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(50.0, 60.0),
                                                                            geoops::UTMCoordinate(60.0, 50.0),
                                                                            geoops::UTMCoordinate(50.0, 40.0),
                                                                            geoops::UTMCoordinate(40.0, 50.0)};

    geoops::UTMCoordinate aOutsideBounded[siTestValuesLength];
    geoops::UTMCoordinate aInsideBounded[siTestValuesLength];
    geoops::UTMCoordinate aCornerBounded[siTestValuesLength];

    // Loop through each outside coordinate and compare inputs and outputs.
    for (size_t siIter = 0; siIter < siTestValuesLength; siIter++)
    {
        // Calculate Bounded coordinates
        aOutsideBounded[siIter] = pAStar->FindNearestBoundaryPoint(aOutsideCoordinates[siIter]);

        // Validate that the bounded coordinate matches the expected.
        EXPECT_NEAR(aOutsideBounded[siIter].dEasting, aExpectedCoordinates[siIter].dEasting, 0.1);
        EXPECT_NEAR(aOutsideBounded[siIter].dNorthing, aExpectedCoordinates[siIter].dNorthing, 0.1);
    }

    // Loop through each inside coordinate and validate no change.
    for (size_t siIter = 0; siIter < siTestValuesLength; siIter++)
    {
        // Calculate coordinate.
        aInsideBounded[siIter] = pAStar->FindNearestBoundaryPoint(aInsideCoordinates[siIter]);

        // Check that the output values are the same as the input values.
        EXPECT_NEAR(aInsideBounded[siIter].dEasting, aInsideCoordinates[siIter].dEasting, 0.1);
        EXPECT_NEAR(aInsideBounded[siIter].dNorthing, aInsideCoordinates[siIter].dNorthing, 0.1);
    }

    // Loop through each corner coordinate and validate no change.
    for (size_t siIter = 0; siIter < siTestValuesLength; siIter++)
    {
        // Calculate coordinate.
        aCornerBounded[siIter] = pAStar->FindNearestBoundaryPoint(aCornerCoordinates[siIter]);

        // Check that the output values are the same as the input values.
        EXPECT_NEAR(aCornerCoordinates[siIter].dEasting, aCornerBounded[siIter].dEasting, 0.1);
        EXPECT_NEAR(aCornerCoordinates[siIter].dNorthing, aCornerBounded[siIter].dNorthing, 0.1);
    }

    // Delete object.
    delete pAStar;
    // Point to null.
    pAStar = nullptr;
}

/******************************************************************************
 * @brief Test AStar Rounding coordinate functionality.
 *
 *
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-04-28
 ******************************************************************************/
TEST(AStarPlannerTest, RoundCoordinate)
{
    /// Create a new AStar object.
    pathplanners::AStar* pAStar = new pathplanners::AStar();

    size_t siTestValuesLength   = 4;

    // Initialize array with coordinates to round.
    const geoops::UTMCoordinate aOriginalCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(50.23, 68.16),
                                                                            geoops::UTMCoordinate(13.24, 13.26),
                                                                            geoops::UTMCoordinate(99.76, 99.74),
                                                                            geoops::UTMCoordinate(60.11, 83.23)};

    // Initialize arrays with correct rounded values.
    const geoops::UTMCoordinate aExpectedCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(50.0, 68.0),
                                                                            geoops::UTMCoordinate(13.0, 13.5),
                                                                            geoops::UTMCoordinate(100.0, 99.5),
                                                                            geoops::UTMCoordinate(60.0, 83.0)};

    // Loop through each coordinate and compare inputs and outputs.
    for (size_t siIter = 0; siIter < siTestValuesLength; siIter++)
    {
        // Calculate rounded coordinate.
        geoops::UTMCoordinate stRounded = pAStar->RoundUTMCoordinate(aOriginalCoordinates[siIter]);

        // Validate that the bounded coordinate matches the expected.
        EXPECT_NEAR(stRounded.dEasting, aExpectedCoordinates[siIter].dEasting, 0.1);
        EXPECT_NEAR(stRounded.dNorthing, aExpectedCoordinates[siIter].dNorthing, 0.1);
    }

    // Delete object.
    delete pAStar;
    // Point to null.
    pAStar = nullptr;
}

/******************************************************************************
 * @brief Test AStar path construction functionality.
 *
 *
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-04-28
 ******************************************************************************/
TEST(AStarPlannerTest, ConstructPath)
{
    /// Create a new AStar object.
    pathplanners::AStar* pAStar = new pathplanners::AStar();

    size_t siTestValuesLength   = 5;

    // Create coordinates for AStarNode objects.
    const geoops::UTMCoordinate aOriginalCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(50.0, 50.0),
                                                                            geoops::UTMCoordinate(50.5, 50.5),
                                                                            geoops::UTMCoordinate(51.0, 51.0),
                                                                            geoops::UTMCoordinate(51.5, 51.5),
                                                                            geoops::UTMCoordinate(52.0, 52.0)};

    // Create nodes and set the node's parent to the previous element in the array.
    pathplanners::nodes::AStarNode aOriginalNodes[siTestValuesLength];
    aOriginalNodes[0] = pathplanners::nodes::AStarNode(nullptr, aOriginalCoordinates[0]);
    for (size_t siIter = 1; siIter < siTestValuesLength; siIter++)
    {
        std::shared_ptr pParentNode = std::make_shared<pathplanners::nodes::AStarNode>(aOriginalNodes[siIter - 1]);
        aOriginalNodes[siIter]      = pathplanners::nodes::AStarNode(pParentNode, aOriginalCoordinates[siIter]);
    }

    // Construct coordinate vector with ConstructPath().
    pAStar->ConstructPath(aOriginalNodes[siTestValuesLength - 1]);

    const std::vector<geoops::UTMCoordinate> vReturnedPath = pAStar->GetPath();

    // Loop through each coordinate and compare original with expected.
    for (int siIter = 0; siIter < siTestValuesLength; siIter++)
    {
        // Validate that the returned path coordinate matches the original.
        EXPECT_NEAR(aOriginalCoordinates[siIter].dEasting, vReturnedPath[siIter].dEasting, 0.1);
        EXPECT_NEAR(aOriginalCoordinates[siIter].dNorthing, vReturnedPath[siIter].dNorthing, 0.1);
    }

    // Delete object.
    delete pAStar;
    // Point to null.
    pAStar = nullptr;
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
    const geoops::UTMCoordinate stStart = geoops::UTMCoordinate(dEastingStart, dNorthingStart);

    // Create goal coordinates for AStar.
    const geoops::UTMCoordinate aGoalCoordinates[siTestValuesLength] = {
        geoops::UTMCoordinate(dEastingStart, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID),                                           // N
        geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart),                                           // E
        geoops::UTMCoordinate(dEastingStart, dNorthingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID),                                           // S
        geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart),                                           // W
        geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID),    // NE
        geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID),    // SE
        geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID),    // SW
        geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID)     // NW
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
    const geoops::UTMCoordinate stObstacleCenter   = geoops::UTMCoordinate(608120, 4201140);
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
    const geoops::UTMCoordinate stObstacle2Center   = geoops::UTMCoordinate(608100, 4201100);
    const double dObstacle2Size                     = 2 * constants::ASTAR_NODE_SIZE;
    const pathplanners::AStar::Obstacle stObstacle2 = {stObstacle2Center, dObstacle2Size};
    vObstacles.emplace_back(stObstacle);
    vObstacles.emplace_back(stObstacle2);

    // Reset obstacles within AStar.
    pAStar->UpdateObstacleData(vObstacles, true);

    // Validate obstacles exist within AStar.
    std::vector<pathplanners::AStar::Obstacle> vReturnVector = pAStar->GetObstacleData();
    size_t siVectorSize                                      = vReturnVector.size();
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
