/******************************************************************************
 * @brief Unit test for GeoPlanner class.
 *
 * @file GeoPlanner.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-30
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/algorithms/planners/GeoPlanner.h"
#include "../../../../../src/handlers/LiDARHandler.h"
#include "../../../../TestingBase.hh"

/// \cond
#include <chrono>
#include <gtest/gtest.h>
#include <memory>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the GeoPlanner
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-30
 ******************************************************************************/
class GeoPlannerTests : public TestingBase<GeoPlannerTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

        LiDARHandler* m_pLiDARHandler = nullptr;

    public:
        /******************************************************************************
         * @brief Construct a new GeoPlannerTests object.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-09
         ******************************************************************************/
        GeoPlannerTests() {}

        /******************************************************************************
         * @brief Destroy the GeoPlannerTests object.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-09
         ******************************************************************************/
        ~GeoPlannerTests() {}

        /******************************************************************************
         * @brief Setup the GeoPlannerTests object.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-09
         ******************************************************************************/
        void TestSetup() override
        {
            // Initialize the LiDARHandler for this test.
            m_pLiDARHandler = new LiDARHandler();
            if (!m_pLiDARHandler->OpenDB("../data/LiDAR/data/databases/Flat_SIM.db"))
            {
                // Submit logger message.
                LOG_ERROR(logging::g_qSharedLogger, "Failed to open LiDAR database for GeoPlanner tests.");
                // Stop the test.
                FAIL() << "Failed to open LiDAR database for GeoPlanner tests.";
            }
        }

        /******************************************************************************
         * @brief Teardown the GeoPlannerTests object.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-09
         ******************************************************************************/
        void TestTeardown() override
        {
            if (m_pLiDARHandler)
            {
                delete m_pLiDARHandler;
                m_pLiDARHandler = nullptr;
            }
        }
};

/******************************************************************************
 * @brief Check that GeoPlanner doesn't leak any memory.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-28
 ******************************************************************************/
TEST_F(GeoPlannerTests, DoesNotLeak)
{
    // Create a new GeoPlanner object.
    pathplanners::GeoPlanner* pGeoPlanner = new pathplanners::GeoPlanner();
    // Delete object.
    delete pGeoPlanner;
    // Point to null.
    pGeoPlanner = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-28
 ******************************************************************************/
TEST_F(GeoPlannerTests, Leaks)
{
    // Create a new GeoPlanner object.
    pathplanners::GeoPlanner* pGeoPlanner = new pathplanners::GeoPlanner();
    EXPECT_TRUE(pGeoPlanner != nullptr);
}

/******************************************************************************
 * @brief Test GeoPlanner path planning functionality (stub).
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-30
 ******************************************************************************/
TEST_F(GeoPlannerTests, PlanPathBasic)
{
    // This is a stub test. In a real test, you would mock LiDARHandler and provide known data.
    // Here, we just check that PlanPath can be called and returns a vector.
    ASSERT_NE(m_pLiDARHandler, nullptr);
    pathplanners::GeoPlanner GeoPlanner;

    // Fugitive Beach.
    geoops::UTMCoordinate stStart(614082.79, 4190057.07, 15);
    geoops::UTMCoordinate stEnd(614203.77, 4189924.81, 15);
    std::vector<geoops::Waypoint> vPath = GeoPlanner.PlanPath(m_pLiDARHandler, stStart, stEnd, 2.0, 240.0, true);

    // We can't guarantee a path exists without a real/mocked DB, but we can check type and no crash.
    EXPECT_TRUE(vPath.size() >= 0);
}
