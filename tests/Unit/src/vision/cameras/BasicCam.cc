/******************************************************************************
 * @brief BasicCam unit tests.
 *
 * @file BasicCam.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/vision/cameras/BasicCam.h"
#include "../../../../TestingBase.hh"

/// \cond
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the BasicCam
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class BasicCamTests : public TestingBase<BasicCamTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new Basic Cam Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        BasicCamTests() {}

        /******************************************************************************
         * @brief Destroy the Basic Cam Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        ~BasicCamTests() {}

        /******************************************************************************
         * @brief Setup the Basic Cam Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Teardown the Basic Cam Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Test the functionality of the BasicCam constructor and destructor.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(BasicCamTests, ConstructorDestructor)
{
    // Create a BasicCam object.
    BasicCam basicCam("/dev/video0", 1280, 720, 30, PIXEL_FORMATS::eBGRA, 90.0, 60.0, false, 1);

    // Check initial camera open status.
    EXPECT_FALSE(basicCam.GetCameraIsOpen());

    // Destructor will be called automatically when the object goes out of scope.
}

/******************************************************************************
 * @brief Check that BasicCam doesn't leak any memory.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(BasicCamTests, DoesNotLeak)
{
    // Create a new BasicCam object.
    BasicCam* pBasicCam = new BasicCam("/dev/video0", 1280, 720, 30, PIXEL_FORMATS::eBGRA, 90.0, 60.0, false, 1);
    // Delete object.
    delete pBasicCam;
    // Point to null.
    pBasicCam = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(BasicCamTests, Leaks)
{
    // Create a new BasicCam object.
    BasicCam* pBasicCam = new BasicCam("/dev/video0", 1280, 720, 30, PIXEL_FORMATS::eBGRA, 90.0, 60.0, false, 1);
    EXPECT_TRUE(pBasicCam != nullptr);
}

/******************************************************************************
 * @brief Test the GetCameraIsOpen method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(BasicCamTests, GetCameraIsOpen)
{
    // Create a BasicCam object.
    BasicCam basicCam("/dev/video0", 1280, 720, 30, PIXEL_FORMATS::eBGRA, 90.0, 60.0, false, 1);

    // Check initial camera open status.
    EXPECT_FALSE(basicCam.GetCameraIsOpen());
}

/******************************************************************************
 * @brief Test the GetCameraLocation method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(BasicCamTests, GetCameraLocation)
{
    // Create a BasicCam object.
    BasicCam basicCam("/dev/video0", 1280, 720, 30, PIXEL_FORMATS::eBGRA, 90.0, 60.0, false, 1);

    // Check camera location.
    EXPECT_EQ(basicCam.GetCameraLocation(), "/dev/video0");
}
