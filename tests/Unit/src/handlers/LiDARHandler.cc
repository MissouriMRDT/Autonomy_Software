/******************************************************************************
 * @brief Unit tests for the LiDARHandler runtime query class.
 *
 * @file LiDARHandler.cc
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/handlers/LiDARHandler.h"
#include "../../../TestingBase.hh"

/// \cond
#include <filesystem>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Test fixture for LiDARHandler class.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 ******************************************************************************/
class LiDARHandlerTests : public TestingBase<LiDARHandlerTests>
{
    protected:
        LiDARHandler m_Handler;
        std::string m_szDbPath;

    public:
        LiDARHandlerTests()           = default;
        ~LiDARHandlerTests() override = default;

        void TestSetup() override
        {
            // Relative path to the SQLite database file (to the build directory)
            m_szDbPath = std::filesystem::absolute("../data/LiDAR/data/sqlite/MDRS.db").string();
            ASSERT_TRUE(m_Handler.Initialize(m_szDbPath));
        }

        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Confirm that handler opens the database correctly.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 ******************************************************************************/
TEST_F(LiDARHandlerTests, CanInitializeWithValidDB)
{
    LiDARHandler handler;
    EXPECT_TRUE(handler.Initialize(m_szDbPath));
}

/******************************************************************************
 * @brief Confirm that initialization fails with nonexistent DB.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 ******************************************************************************/
TEST_F(LiDARHandlerTests, FailsToInitializeInvalidDB)
{
    LiDARHandler handler;
    EXPECT_FALSE(handler.Initialize("invalid/path/to.db"));
}

/******************************************************************************
 * @brief Confirm query returns at least one point in known populated region.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 ******************************************************************************/
TEST_F(LiDARHandlerTests, QueryReturnsNearbyPoints)
{
    // Test parameters
    double dTestEasting  = 518011.14;
    double dTestNorthing = 4253985.0600000005;
    double dRadius       = 5.0;

    // Execute the query
    std::vector<LiDARHandler::PointRow> vResults = m_Handler.GetNearbyPoints(dTestEasting, dTestNorthing, dRadius);
    EXPECT_GT(vResults.size(), 0u);
}

/******************************************************************************
 * @brief Confirm empty query when far outside known point area.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 ******************************************************************************/
TEST_F(LiDARHandlerTests, QueryReturnsNothingInEmptyRegion)
{
    // Test parameters
    double dTestEasting  = 100.0;
    double dTestNorthing = 100.0;
    double dRadius       = 5.0;

    // Execute the query
    std::vector<LiDARHandler::PointRow> vResults = m_Handler.GetNearbyPoints(dTestEasting, dTestNorthing, dRadius);
    EXPECT_TRUE(vResults.empty());
}
