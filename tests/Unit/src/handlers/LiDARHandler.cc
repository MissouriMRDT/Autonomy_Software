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
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 ******************************************************************************/
class LiDARHandlerTests : public TestingBase<LiDARHandlerTests>
{
    protected:
        std::string m_szDbPath = "../data/LiDAR/data/databases/Fugitive.db";

    public:
        LiDARHandlerTests()           = default;
        ~LiDARHandlerTests() override = default;

        void TestSetup() override {}

        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Confirm that handler opens the database correctly.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 ******************************************************************************/
TEST_F(LiDARHandlerTests, CanInitializeWithValidDB)
{
    LiDARHandler handler;
    EXPECT_TRUE(handler.OpenDB(m_szDbPath));
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
    EXPECT_FALSE(handler.OpenDB("invalid/path/to.db"));
}

/******************************************************************************
 * @brief Confirm that handler can close the database.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
TEST_F(LiDARHandlerTests, CanCloseDB)
{
    LiDARHandler handler;
    EXPECT_TRUE(handler.OpenDB(m_szDbPath));
    EXPECT_TRUE(handler.CloseDB());
    EXPECT_FALSE(handler.IsDBOpen());
}

/******************************************************************************
 * @brief Confirm that closing without opening is safe.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
TEST_F(LiDARHandlerTests, CloseDBWithoutOpenIsSafe)
{
    LiDARHandler handler;
    EXPECT_TRUE(handler.CloseDB());
    EXPECT_FALSE(handler.IsDBOpen());
}

/******************************************************************************
 * @brief Confirm that double open closes previous and opens new DB.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
TEST_F(LiDARHandlerTests, DoubleOpenClosesPrevious)
{
    LiDARHandler handler;
    EXPECT_TRUE(handler.OpenDB(m_szDbPath));
    // Open again with same path, should close and reopen
    EXPECT_TRUE(handler.OpenDB(m_szDbPath));
    EXPECT_TRUE(handler.IsDBOpen());
}

/******************************************************************************
 * @brief Confirm that double close is safe.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
TEST_F(LiDARHandlerTests, DoubleCloseIsSafe)
{
    LiDARHandler handler;
    EXPECT_TRUE(handler.OpenDB(m_szDbPath));
    EXPECT_TRUE(handler.CloseDB());
    EXPECT_TRUE(handler.CloseDB());
}

/******************************************************************************
 * @brief Query returns empty if DB not open.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
TEST_F(LiDARHandlerTests, QueryWithoutOpenReturnsEmpty)
{
    LiDARHandler handler;
    LiDARHandler::PointFilter filter{.dEasting = 0, .dNorthing = 0, .dRadius = 1.0};
    std::vector<LiDARHandler::PointRow> vResults = handler.GetLiDARData(filter);
    EXPECT_TRUE(vResults.empty());
}

/******************************************************************************
 * @brief Query returns results if present.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
TEST_F(LiDARHandlerTests, QueryReturnsResultsIfPresent)
{
    LiDARHandler handler;
    ASSERT_TRUE(handler.OpenDB(m_szDbPath));
    LiDARHandler::PointFilter filter{.dEasting = 614058.84, .dNorthing = 4189968.85, .dRadius = 3.0};
    std::vector<LiDARHandler::PointRow> vResults = handler.GetLiDARData(filter);
    // Can't guarantee DB contents, but should not crash
    SUCCEED();
}

/******************************************************************************
 * @brief Query with classification filter.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
TEST_F(LiDARHandlerTests, QueryWithClassification)
{
    LiDARHandler handler;
    ASSERT_TRUE(handler.OpenDB(m_szDbPath));
    LiDARHandler::PointFilter filter{.dEasting = 614058.84, .dNorthing = 4189968.85, .dRadius = 3.0, .szClassification = std::optional<std::string>("ground")};
    std::vector<LiDARHandler::PointRow> vResults = handler.GetLiDARData(filter);
    // Should not crash, may be empty
    SUCCEED();
}

/******************************************************************************
 * @brief Query with traversal score range filter.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
TEST_F(LiDARHandlerTests, QueryWithTraversalScoreRange)
{
    LiDARHandler handler;
    ASSERT_TRUE(handler.OpenDB(m_szDbPath));
    LiDARHandler::PointFilter filter{.dEasting        = 614058.84,
                                     .dNorthing       = 4189968.85,
                                     .dRadius         = 3.0,
                                     .dTraversalScore = std::optional<LiDARHandler::PointFilter::Range<double>>({0.95, 1.0})};
    std::vector<LiDARHandler::PointRow> vResults = handler.GetLiDARData(filter);
    // Should not crash, may be empty
    SUCCEED();
}

/******************************************************************************
 * @brief Query with all range filters.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
TEST_F(LiDARHandlerTests, QueryWithAllRangeFilters)
{
    LiDARHandler handler;
    ASSERT_TRUE(handler.OpenDB(m_szDbPath));
    LiDARHandler::PointFilter filter{
        .dEasting        = 614058.84,
        .dNorthing       = 4189968.85,
        .dRadius         = 3.0,
        .dNormalX        = std::optional<LiDARHandler::PointFilter::Range<double>>({-1.0, 1.0}),
        .dNormalY        = std::optional<LiDARHandler::PointFilter::Range<double>>({-1.0, 1.0}),
        .dNormalZ        = std::optional<LiDARHandler::PointFilter::Range<double>>({-1.0, 1.0}),
        .dSlope          = std::optional<LiDARHandler::PointFilter::Range<double>>({0.0, 90.0}),
        .dRoughness      = std::optional<LiDARHandler::PointFilter::Range<double>>({0.0, 10.0}),
        .dCurvature      = std::optional<LiDARHandler::PointFilter::Range<double>>({-10.0, 10.0}),
        .dTraversalScore = std::optional<LiDARHandler::PointFilter::Range<double>>({0.0, 1.0}),
    };
    std::vector<LiDARHandler::PointRow> vResults = handler.GetLiDARData(filter);
    SUCCEED();
}

/******************************************************************************
 * @brief Check thread safety of open/close/query operations.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
TEST_F(LiDARHandlerTests, ThreadSafetyOpenCloseQuery)
{
    LiDARHandler handler;
    std::function<void()> fnOpenClose = [&handler, this]()
    {
        for (int i = 0; i < 5; ++i)
        {
            handler.OpenDB(m_szDbPath);
            handler.CloseDB();
        }
    };
    std::function<void()> fnQuery = [&handler, this]()
    {
        for (int i = 0; i < 5; ++i)
        {
            handler.OpenDB(m_szDbPath);
            LiDARHandler::PointFilter filter{.dEasting = 614058.84, .dNorthing = 4189968.85, .dRadius = 3.0};
            handler.GetLiDARData(filter);
            handler.CloseDB();
        }
    };
    std::thread thThread1(fnOpenClose);
    std::thread thThread2(fnQuery);
    thThread1.join();
    thThread2.join();
    SUCCEED();
}

/******************************************************************************
 * @brief Inserts LiDAR data points into the database.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-10-20
 ******************************************************************************/
TEST_F(LiDARHandlerTests, InsertLiDARData)
{
    // Open the database and setup handler.
    LiDARHandler handler;
    ASSERT_TRUE(handler.OpenDB(m_szDbPath));

    // Prepare test data points. These will be a grid of 100 points spread over 5 square meters at a given altitude.
    std::vector<geoops::Waypoint> vTestPoints;
    geoops::Waypoint stCenterPoint = {geoops::UTMCoordinate(614132.76, 4190038.68, 15, true, 315)};

    double dSpacing                = 0.5;    // 0.5 meter spacing
    for (int nIter = 0; nIter < 100; ++nIter)
    {
        for (int mIter = 0; mIter < 100; ++mIter)
        {
            double dEasting  = stCenterPoint.GetUTMCoordinate().dEasting + (nIter - 5) * dSpacing;
            double dNorthing = stCenterPoint.GetUTMCoordinate().dNorthing + (mIter - 5) * dSpacing;
            geoops::UTMCoordinate stUTMPoint(dEasting,
                                             dNorthing,
                                             stCenterPoint.GetUTMCoordinate().nZone,
                                             stCenterPoint.GetUTMCoordinate().bWithinNorthernHemisphere,
                                             stCenterPoint.GetUTMCoordinate().dAltitude);
            geoops::Waypoint stPoint(stUTMPoint, geoops::WaypointType::eNavigationWaypoint, 0.5, nIter * 10 + mIter);
            vTestPoints.push_back(stPoint);
        }
    }

    // Insert the test data points.
    ASSERT_TRUE(handler.InsertLiDARData(vTestPoints));
}
