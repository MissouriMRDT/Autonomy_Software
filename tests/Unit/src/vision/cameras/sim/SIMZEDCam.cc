/******************************************************************************
 * @brief SIMZEDCam unit tests.
 *
 * @file SIMZEDCam.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../../src/vision/cameras/sim/SIMZEDCam.h"
#include "../../../../../TestingBase.hh"

/// \cond
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the SIMZEDCam
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class SIMZEDCamTests : public TestingBase<SIMZEDCamTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new SIMZEDCamTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        SIMZEDCamTests() {}

        /******************************************************************************
         * @brief Destroy the SIMZEDCamTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        ~SIMZEDCamTests() {}

        /******************************************************************************
         * @brief Setup the SIMZEDCamTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Teardown the SIMZEDCamTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Test the functionality of the SIMZEDCam constructor and destructor.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(SIMZEDCamTests, ConstructorDestructor)
{
    // Create a SIMZEDCam object.
    SIMZEDCam simZEDCam("/dev/video0", 1280, 720, 30, 90.0, 60.0, false, 1, 12345);

    // Check initial camera open status.
    EXPECT_FALSE(simZEDCam.GetCameraIsOpen());

    // Destructor will be called automatically when the object goes out of scope.
}

/******************************************************************************
 * @brief Check that SIMZEDCam doesn't leak any memory.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(SIMZEDCamTests, DoesNotLeak)
{
    // Create a new SIMZEDCam object.
    SIMZEDCam* pSimZEDCam = new SIMZEDCam("/dev/video0", 1280, 720, 30, 90.0, 60.0, false, 1, 12345);
    // Delete object.
    delete pSimZEDCam;
    // Point to null.
    pSimZEDCam = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(SIMZEDCamTests, Leaks)
{
    // Create a new SIMZEDCam object.
    SIMZEDCam* pSimZEDCam = new SIMZEDCam("/dev/video0", 1280, 720, 30, 90.0, 60.0, false, 1, 12345);
    EXPECT_TRUE(pSimZEDCam != nullptr);
}

/******************************************************************************
 * @brief Test the ResetPositionalTracking method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(SIMZEDCamTests, ResetPositionalTracking)
{
    // Create a SIMZEDCam object.
    SIMZEDCam simZEDCam("/dev/video0", 1280, 720, 30, 90.0, 60.0, false, 1, 12345);

    // Call ResetPositionalTracking.
    auto errorCode = simZEDCam.ResetPositionalTracking();

    // Check that the error code is SUCCESS.
    EXPECT_EQ(errorCode, sl::ERROR_CODE::SUCCESS);
}

/******************************************************************************
 * @brief Test the RebootCamera method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(SIMZEDCamTests, RebootCamera)
{
    // Create a SIMZEDCam object.
    SIMZEDCam simZEDCam("/dev/video0", 1280, 720, 30, 90.0, 60.0, false, 1, 12345);

    // Call RebootCamera.
    auto errorCode = simZEDCam.RebootCamera();

    // Check that the error code is SUCCESS.
    EXPECT_EQ(errorCode, sl::ERROR_CODE::SUCCESS);
}

/******************************************************************************
 * @brief Test the EnablePositionalTracking method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(SIMZEDCamTests, EnablePositionalTracking)
{
    // Create a SIMZEDCam object.
    SIMZEDCam simZEDCam("/dev/video0", 1280, 720, 30, 90.0, 60.0, false, 1, 12345);

    // Call EnablePositionalTracking.
    auto errorCode = simZEDCam.EnablePositionalTracking(1.0f);

    // Check that the error code is SUCCESS.
    EXPECT_EQ(errorCode, sl::ERROR_CODE::SUCCESS);
}

/******************************************************************************
 * @brief Test the DisablePositionalTracking method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(SIMZEDCamTests, DisablePositionalTracking)
{
    // Create a SIMZEDCam object.
    SIMZEDCam simZEDCam("/dev/video0", 1280, 720, 30, 90.0, 60.0, false, 1, 12345);

    // Call DisablePositionalTracking.
    simZEDCam.DisablePositionalTracking();

    // Check that positional tracking is disabled.
    // Assuming there is a method to check this, e.g., IsPositionalTrackingEnabled().
    EXPECT_FALSE(simZEDCam.GetPositionalTrackingEnabled());
}
