/******************************************************************************
 * @brief ZEDCam unit tests.
 *
 * @file ZEDCam.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/vision/cameras/ZEDCam.h"
#include "../../../../TestingBase.hh"

/// \cond
#include <future>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

/// \endcond

class ZEDCamTests : public TestingBase<ZEDCamTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new ZEDCamTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        ZEDCamTests() {}

        /******************************************************************************
         * @brief Destroy the ZEDCamTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        ~ZEDCamTests() {}

        /******************************************************************************
         * @brief Setup the ZEDCamTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Destroy the ZEDCamTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Test the functionality of the ZEDCam constructor and destructor.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(ZEDCamTests, ConstructorDestructor)
{
    // Create a ZEDCam object.
    ZEDCam zedCam(1280, 720, 30, 90.0, 60.0, false, 0.5f, 20.0f, false, false, false, 1, 0);

    // Check initial camera open status.
    EXPECT_FALSE(zedCam.GetCameraIsOpen());

    // Destructor will be called automatically when the object goes out of scope.
}

/******************************************************************************
 * @brief Check that ZEDCam doesn't leak any memory.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(ZEDCamTests, DoesNotLeak)
{
    // Create a new ZEDCam object.
    ZEDCam* pZEDCam = new ZEDCam(1280, 720, 30, 90.0, 60.0, false, 0.5f, 20.0f, false, false, false, 1, 0);
    // Delete object.
    delete pZEDCam;
    // Point to null.
    pZEDCam = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(ZEDCamTests, Leaks)
{
    // Create a new ZEDCam object.
    ZEDCam* pZEDCam = new ZEDCam(1280, 720, 30, 90.0, 60.0, false, 0.5f, 20.0f, false, false, false, 1, 0);
    EXPECT_TRUE(pZEDCam != nullptr);
}

/******************************************************************************
 * @brief Test the GetCameraIsOpen method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(ZEDCamTests, GetCameraIsOpen)
{
    // Create a ZEDCam object.
    ZEDCam zedCam(1280, 720, 30, 90.0, 60.0, false, 0.5f, 20.0f, false, false, false, 1, 0);

    // Check initial camera open status.
    EXPECT_FALSE(zedCam.GetCameraIsOpen());
}
