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

const geoops::UTMCoordinate stStartCoordinate(600000, 4200000, 15, true);

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
        AStarPlannerTests() { SetUp(); }

        /******************************************************************************
         * @brief Destroy the AStarPlannerTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~AStarPlannerTests() { TearDown(); }

        /******************************************************************************
         * @brief Setup the AStarPlannerTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void SetUp() override
        {
            // Call the base setup method. This initializes the loggers and RoveComm instances.
            RequiredSetup();
        }

        /******************************************************************************
         * @brief Teardown the AStarPlannerTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TearDown() override
        {
            // Call the base teardown method. This stops the RoveComm instances and loggers.
            RequiredTeardown();
        }
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
 * @brief Test ASTAR boundary point functionality.
 *
 *
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-04-28
 ******************************************************************************/
TEST_F(AStarPlannerTests, BoundaryPoint)
{
    // Create a new AStar object.
    pathplanners::AStar* pAStar = new pathplanners::AStar();

    // Initialize start coordinate in AStar object.
    geoops::UTMCoordinate stStartCoord = geoops::UTMCoordinate(50.0, 50.0);
    pAStar->SetStartCoordinate(stStartCoord);

    size_t siTestValuesLength = 4;

    // Initialize array with coordinates external to the AStar search grid.
    const geoops::UTMCoordinate aOutsideCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(50.0, 65.0, 15),
                                                                           geoops::UTMCoordinate(65.0, 50.0, 15),
                                                                           geoops::UTMCoordinate(50.0, 35.0, 15),
                                                                           geoops::UTMCoordinate(35.0, 50.0, 15)};

    // Initialize array with coordinates internal to the AStar search grid.
    const geoops::UTMCoordinate aInsideCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(50.0, 55.0, 15),
                                                                          geoops::UTMCoordinate(55.0, 50.0, 15),
                                                                          geoops::UTMCoordinate(50.0, 45.0, 15),
                                                                          geoops::UTMCoordinate(45.0, 50.0, 15)};

    //  Initialize array with corner coordinates on the boundary.
    const geoops::UTMCoordinate aCornerCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(60.0, 60.0, 15),
                                                                          geoops::UTMCoordinate(60.0, 40.0, 15),
                                                                          geoops::UTMCoordinate(40.0, 40.0, 15),
                                                                          geoops::UTMCoordinate(40.0, 60.0, 15)};
    // Initialize arrays with correct bounded values
    const geoops::UTMCoordinate aExpectedCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(50.0, 60.0, 15),
                                                                            geoops::UTMCoordinate(60.0, 50.0, 15),
                                                                            geoops::UTMCoordinate(50.0, 40.0, 15),
                                                                            geoops::UTMCoordinate(40.0, 50.0, 15)};

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
TEST_F(AStarPlannerTests, RoundCoordinate)
{
    /// Create a new AStar object.
    pathplanners::AStar* pAStar = new pathplanners::AStar();

    size_t siTestValuesLength   = 4;

    // Initialize array with coordinates to round.
    const geoops::UTMCoordinate aOriginalCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(50.23, 68.16, 15),
                                                                            geoops::UTMCoordinate(13.24, 13.26, 15),
                                                                            geoops::UTMCoordinate(99.76, 99.74, 15),
                                                                            geoops::UTMCoordinate(60.11, 83.23, 15)};

    // Initialize arrays with correct rounded values.
    const geoops::UTMCoordinate aExpectedCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(50.0, 68.0, 15),
                                                                            geoops::UTMCoordinate(13.0, 13.5, 15),
                                                                            geoops::UTMCoordinate(100.0, 99.5, 15),
                                                                            geoops::UTMCoordinate(60.0, 83.0, 15)};

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
TEST_F(AStarPlannerTests, ConstructPath)
{
    /// Create a new AStar object.
    pathplanners::AStar* pAStar = new pathplanners::AStar();

    size_t siTestValuesLength   = 5;

    // Create coordinates for AStarNode objects.
    const geoops::UTMCoordinate aOriginalCoordinates[siTestValuesLength] = {geoops::UTMCoordinate(50.0, 50.0, 15),
                                                                            geoops::UTMCoordinate(50.5, 50.5, 15),
                                                                            geoops::UTMCoordinate(51.0, 51.0, 15),
                                                                            geoops::UTMCoordinate(51.5, 51.5, 15),
                                                                            geoops::UTMCoordinate(52.0, 52.0, 15)};

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
TEST_F(AStarPlannerTests, PlanAvoidancePath)
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
TEST_F(AStarPlannerTests, ObstacleInitialization)
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

/******************************************************************************
 * @brief Test AStar path obstacle avoidance for all 8 directions.
 *
 *
 * @author Sam Nolte (samnolte0302@gmail.com)
 * @date 2024-11-19
 ******************************************************************************/
TEST_F(AStarPlannerTests, AvoidObstaclesWhilePathing)
{
    // Create a new AStar object
    pathplanners::AStar* pAStar = new pathplanners::AStar();

    size_t siTestValuesLength   = 8;

    // Start coordinate for AStar
    const double dEastingStart  = stStartCoordinate.dEasting;
    const double dNorthingStart = stStartCoordinate.dNorthing;

    // Create goal coordinates for AStar
    const geoops::UTMCoordinate aGoalCoordinates[siTestValuesLength] = {
        geoops::UTMCoordinate(dEastingStart, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, 15, true),                                           // N
        geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart, 15, true),                                           // E
        geoops::UTMCoordinate(dEastingStart, dNorthingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, 15, true),                                           // S
        geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart, 15, true),                                           // W
        geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, 15, true),    // NE
        geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, 15, true),    // SE
        geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, 15, true),    // SW
        geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, 15, true)     // NW
    };

    // Create obstacle coordinates for AStar
    const pathplanners::AStar::Obstacle aObstacles[siTestValuesLength] = {
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(dEastingStart, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID / 2, 15, true),
                                      2 * constants::ASTAR_NODE_SIZE),    // N
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID / 2, dNorthingStart, 15, true),
                                      2 * constants::ASTAR_NODE_SIZE),    // E
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(dEastingStart, dNorthingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID / 2, 15, true),
                                      2 * constants::ASTAR_NODE_SIZE),    // S
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID / 2, dNorthingStart, 15, true),
                                      2 * constants::ASTAR_NODE_SIZE),    // W
        pathplanners::AStar::Obstacle(
            geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID / 2, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID / 2, 15, true),
            2 * constants::ASTAR_NODE_SIZE),    // NE
        pathplanners::AStar::Obstacle(
            geoops::UTMCoordinate(dEastingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID / 2, dNorthingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID / 2, 15, true),
            2 * constants::ASTAR_NODE_SIZE),    // SE
        pathplanners::AStar::Obstacle(
            geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID / 2, dNorthingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID / 2, 15, true),
            2 * constants::ASTAR_NODE_SIZE),    // SW
        pathplanners::AStar::Obstacle(
            geoops::UTMCoordinate(dEastingStart - constants::ASTAR_MAXIMUM_SEARCH_GRID / 2, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID / 2, 15, true),
            2 * constants::ASTAR_NODE_SIZE)    // NW
    };

    for (size_t siI = 0; siI < siTestValuesLength; siI++)
    {
        // Add obstacle to AStar
        pAStar->UpdateObstacleData(std::vector<pathplanners::AStar::Obstacle>{aObstacles[siI]}, true);

        // Get AStar path
        std::vector<geoops::UTMCoordinate> vReturnedPath = pAStar->PlanAvoidancePath(stStartCoordinate, aGoalCoordinates[siI]);

        // Check for pathing through obstacles
        for (size_t siJ = 0; siJ < vReturnedPath.size(); siJ++)
        {
            // Check to see if current coordinate is within obstacle bounds
            EXPECT_FALSE(vReturnedPath[siJ].dNorthing >= aObstacles[siI].stCenterPoint.dNorthing - aObstacles[siI].dRadius &&
                         vReturnedPath[siJ].dNorthing <= aObstacles[siI].stCenterPoint.dNorthing + aObstacles[siI].dRadius &&
                         vReturnedPath[siJ].dEasting >= aObstacles[siI].stCenterPoint.dEasting - aObstacles[siI].dRadius &&
                         vReturnedPath[siJ].dEasting <= aObstacles[siI].stCenterPoint.dEasting + aObstacles[siI].dRadius);
        }
        // Make sure AStar actually found a path
        EXPECT_TRUE(vReturnedPath.size() != 0);

        // Steal Kai's goal coord end point check
        EXPECT_NEAR(aGoalCoordinates[siI].dEasting, vReturnedPath.back().dEasting, 0.1);
        EXPECT_NEAR(aGoalCoordinates[siI].dNorthing, vReturnedPath.back().dNorthing, 0.1);
    }

    // Cleanup
    delete pAStar;
    pAStar = nullptr;
}

/******************************************************************************
 * @brief Test AStar pathing when obstacles conflict with the goal node.
 *
 *
 * @author Sam Nolte (samnolte0302@gmail.com)
 * @date 2025-2-4
 ******************************************************************************/
TEST_F(AStarPlannerTests, GoalConflictWithObstacle)
{
    // Create a new AStar object
    pathplanners::AStar* pAStar = new pathplanners::AStar();
    pAStar->SetStartCoordinate(stStartCoordinate);

    // Start coordinate for AStar
    const double dEastingStart  = stStartCoordinate.dEasting;
    const double dNorthingStart = stStartCoordinate.dNorthing;

    // Create goal coordinates for AStar
    const geoops::UTMCoordinate stGoalCoordinate = geoops::UTMCoordinate(dEastingStart, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, 15, true);

    // Create obstacle coordinates for AStar
    const std::vector<pathplanners::AStar::Obstacle> aObstacles = {
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(stGoalCoordinate.dEasting, stGoalCoordinate.dNorthing, 15, true),
                                      2 * constants::ASTAR_NODE_SIZE),    // At goal
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(stGoalCoordinate.dEasting, stGoalCoordinate.dNorthing + constants::ASTAR_NODE_SIZE, 15, true),
                                      2 * constants::ASTAR_NODE_SIZE),    // 1 node North of goal
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(stGoalCoordinate.dEasting + constants::ASTAR_NODE_SIZE, stGoalCoordinate.dNorthing, 15, true),
                                      2 * constants::ASTAR_NODE_SIZE),    // 1 node East of goal
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(stGoalCoordinate.dEasting, stGoalCoordinate.dNorthing - constants::ASTAR_NODE_SIZE, 15, true),
                                      2 * constants::ASTAR_NODE_SIZE),    // 1 node South of goal
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(stGoalCoordinate.dEasting - constants::ASTAR_NODE_SIZE, stGoalCoordinate.dNorthing, 15, true),
                                      2 * constants::ASTAR_NODE_SIZE),    // 1 node West of goal
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(stGoalCoordinate.dEasting, stGoalCoordinate.dNorthing - constants::ASTAR_NODE_SIZE, 15, true),
                                      5 * constants::ASTAR_NODE_SIZE),    // 2 nodes South of goal
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(stGoalCoordinate.dEasting, stGoalCoordinate.dNorthing - constants::ASTAR_NODE_SIZE, 15, true),
                                      5 * constants::ASTAR_NODE_SIZE),    // 2 nodes East of goal
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(stGoalCoordinate.dEasting, stGoalCoordinate.dNorthing - constants::ASTAR_NODE_SIZE, 15, true),
                                      5 * constants::ASTAR_NODE_SIZE),    // 3 nodes South of goal
    };

    for (size_t siI = 0; siI < aObstacles.size(); siI++)
    {
        // Add obstacle to AStar
        pAStar->UpdateObstacleData(std::vector<pathplanners::AStar::Obstacle>{aObstacles[siI]}, true);

        // Get Shifted Goal coordinate
        geoops::UTMCoordinate stShiftedGoal = pAStar->FindNearestBoundaryPoint(stGoalCoordinate);

        // Obstacle Bounds
        double dNorthBorder = aObstacles[siI].stCenterPoint.dNorthing + aObstacles[siI].dRadius;
        double dEastBorder  = aObstacles[siI].stCenterPoint.dEasting + aObstacles[siI].dRadius;
        double dSouthBorder = aObstacles[siI].stCenterPoint.dNorthing - aObstacles[siI].dRadius;
        double dWestBorder  = aObstacles[siI].stCenterPoint.dEasting - aObstacles[siI].dRadius;

        // Check goal-obstacle collision
        EXPECT_TRUE(stShiftedGoal.dEasting < dWestBorder || stShiftedGoal.dEasting > dEastBorder || stShiftedGoal.dNorthing < dSouthBorder ||
                    stShiftedGoal.dNorthing > dNorthBorder);

        // Check search grid bounds
        EXPECT_TRUE(stShiftedGoal.dEasting >= (stStartCoordinate.dEasting - constants::ASTAR_MAXIMUM_SEARCH_GRID - constants::ASTAR_NODE_SIZE) &&
                    stShiftedGoal.dEasting <= (stStartCoordinate.dEasting + constants::ASTAR_MAXIMUM_SEARCH_GRID + constants::ASTAR_NODE_SIZE) &&
                    stShiftedGoal.dNorthing >= (stStartCoordinate.dNorthing - constants::ASTAR_MAXIMUM_SEARCH_GRID - constants::ASTAR_NODE_SIZE) &&
                    stShiftedGoal.dNorthing <= (stStartCoordinate.dNorthing + constants::ASTAR_MAXIMUM_SEARCH_GRID + constants::ASTAR_NODE_SIZE));

        // Make sure AStar paths
        std::vector<geoops::UTMCoordinate> vReturnedPath = pAStar->PlanAvoidancePath(stStartCoordinate, stGoalCoordinate);
        EXPECT_TRUE(vReturnedPath.size() != 0);
    }

    // Add all obstacles to AStar
    pAStar->UpdateObstacleData(aObstacles, true);

    // Get Shifted Goal coordinate
    geoops::UTMCoordinate stShiftedGoal = pAStar->FindNearestBoundaryPoint(stGoalCoordinate);

    for (size_t siI = 0; siI < aObstacles.size(); siI++)
    {
        // Obstacle Bounds
        double dNorthBorder = aObstacles[siI].stCenterPoint.dNorthing + aObstacles[siI].dRadius;
        double dEastBorder  = aObstacles[siI].stCenterPoint.dEasting + aObstacles[siI].dRadius;
        double dSouthBorder = aObstacles[siI].stCenterPoint.dNorthing - aObstacles[siI].dRadius;
        double dWestBorder  = aObstacles[siI].stCenterPoint.dEasting - aObstacles[siI].dRadius;

        // Check goal-obstacle collision
        EXPECT_TRUE(stShiftedGoal.dEasting < dWestBorder || stShiftedGoal.dEasting > dEastBorder || stShiftedGoal.dNorthing < dSouthBorder ||
                    stShiftedGoal.dNorthing > dNorthBorder);
    }

    // Check search grid bounds
    EXPECT_TRUE(stShiftedGoal.dEasting >= (stStartCoordinate.dEasting - constants::ASTAR_MAXIMUM_SEARCH_GRID - constants::ASTAR_NODE_SIZE) &&
                stShiftedGoal.dEasting <= (stStartCoordinate.dEasting + constants::ASTAR_MAXIMUM_SEARCH_GRID + constants::ASTAR_NODE_SIZE) &&
                stShiftedGoal.dNorthing >= (stStartCoordinate.dNorthing - constants::ASTAR_MAXIMUM_SEARCH_GRID - constants::ASTAR_NODE_SIZE) &&
                stShiftedGoal.dNorthing <= (stStartCoordinate.dNorthing + constants::ASTAR_MAXIMUM_SEARCH_GRID + constants::ASTAR_NODE_SIZE));

    // Make sure AStar paths
    std::vector<geoops::UTMCoordinate> vReturnedPath = pAStar->PlanAvoidancePath(stStartCoordinate, stGoalCoordinate);
    EXPECT_TRUE(vReturnedPath.size() != 0);

    // Cleanup
    delete pAStar;
    pAStar = nullptr;
}

TEST_F(AStarPlannerTests, Maze)
{
    // Create a new AStar object
    pathplanners::AStar* pAStar = new pathplanners::AStar();

    // Start coordinate for AStar
    const double dEastingStart  = stStartCoordinate.dEasting;
    const double dNorthingStart = stStartCoordinate.dNorthing;

    // Create goal coordinates for AStar
    const geoops::UTMCoordinate stGoalCoordinate = geoops::UTMCoordinate(dEastingStart, dNorthingStart + constants::ASTAR_MAXIMUM_SEARCH_GRID, 15, true);

    // Create obstacle coordinates for AStar
    const std::vector<pathplanners::AStar::Obstacle> aObstacles = {
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1599998.5, 4199998.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1599999.5, 4199998.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1600000.5, 4199998.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1600001.5, 4199998.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1599998.5, 4199999.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1599998.5, 4200000.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1599998.5, 4200001.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1599999.5, 4200001.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1600000.5, 4200001.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1600001.5, 4200001.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1600002.5, 4200001.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1600003.5, 4200001.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1600003.5, 4200000.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1600003.5, 4299999.5, 15, true), 0.5),
        pathplanners::AStar::Obstacle(geoops::UTMCoordinate(1600003.5, 4299998.5, 15, true), 0.5),
    };

    // Add obstacle to AStar
    pAStar->UpdateObstacleData(aObstacles, true);

    // Make sure AStar paths
    std::vector<geoops::UTMCoordinate> vReturnedPath = pAStar->PlanAvoidancePath(stStartCoordinate, stGoalCoordinate);
    EXPECT_TRUE(vReturnedPath.size() != 0);

    // Cleanup
    delete pAStar;
    pAStar = nullptr;
}
