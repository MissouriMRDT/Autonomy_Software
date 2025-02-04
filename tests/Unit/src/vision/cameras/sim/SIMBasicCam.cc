/******************************************************************************
 * @brief SIMBasicCam unit tests.
 *
 * @file test_SIMBasicCam.cc
 * @date 2025-01-05
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../../src/vision/cameras/sim/SIMBasicCam.h"
#include "../../../../../TestingBase.hh"

/// \cond
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the SIMBasicCam
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class SIMBasicCamTests : public TestingBase<SIMBasicCamTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new SIMBasicCamTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        SIMBasicCamTests() { SetUp(); }

        /******************************************************************************
         * @brief Destroy the SIMBasicCamTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        ~SIMBasicCamTests() { TearDown(); }

        /******************************************************************************
         * @brief Setup the SIMBasicCamTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void SetUp() override
        {
            // Call the base setup method. This initializes the loggers and RoveComm instances.
            RequiredSetup();
        }

        /******************************************************************************
         * @brief Teardown the SIMBasicCamTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TearDown() override
        {
            // Call the base teardown method. This stops the RoveComm instances and loggers.
            RequiredTeardown();
        }
};

/******************************************************************************
 * @brief Test the functionality of the SIMBasicCam constructor and destructor.
 *
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(SIMBasicCamTests, ConstructorDestructor)
{
    // Create a SIMBasicCam object.
    SIMBasicCam simBasicCam("ws://127.0.0.1:80", 1280, 720, 30, PIXEL_FORMATS::eBGRA, 90.0, 60.0, false, 1);

    // Check initial camera open status.
    EXPECT_FALSE(simBasicCam.GetCameraIsOpen());

    // Destructor will be called automatically when the object goes out of scope.
}

/******************************************************************************
 * @brief Check that SIMBasicCam doesn't leak any memory.
 *
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(SIMBasicCamTests, DoesNotLeak)
{
    // Create a new SIMBasicCam object.
    SIMBasicCam* pSIMBasicCam = new SIMBasicCam("ws://127.0.0.1:80", 1280, 720, 30, PIXEL_FORMATS::eBGRA, 90.0, 60.0, false, 1);
    // Delete object.
    delete pSIMBasicCam;
    // Point to null.
    pSIMBasicCam = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(SIMBasicCamTests, Leaks)
{
    // Create a new SIMBasicCam object.
    SIMBasicCam* pSIMBasicCam = new SIMBasicCam("ws://127.0.0.1:80", 1280, 720, 30, PIXEL_FORMATS::eBGRA, 90.0, 60.0, false, 1);
    EXPECT_TRUE(pSIMBasicCam != nullptr);
}

/******************************************************************************
 * @brief Test the GetCameraIsOpen method.
 *
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(SIMBasicCamTests, GetCameraIsOpen)
{
    // Create a SIMBasicCam object.
    SIMBasicCam simBasicCam("ws://127.0.0.1:80", 1280, 720, 30, PIXEL_FORMATS::eBGRA, 90.0, 60.0, false, 1);

    // Check initial camera open status.
    EXPECT_FALSE(simBasicCam.GetCameraIsOpen());
}

/******************************************************************************
 * @brief Test the GetCameraLocation method.
 *
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(SIMBasicCamTests, GetCameraLocation)
{
    // Create a SIMBasicCam object.
    SIMBasicCam simBasicCam("ws://127.0.0.1:80", 1280, 720, 30, PIXEL_FORMATS::eBGRA, 90.0, 60.0, false, 1);

    // Check camera location.
    EXPECT_EQ(simBasicCam.GetCameraLocation(), "ws://127.0.0.1:80");
}
