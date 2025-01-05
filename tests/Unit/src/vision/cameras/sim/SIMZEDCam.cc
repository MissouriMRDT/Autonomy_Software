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

/// \cond
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

/// \endcond

/******************************************************************************
 * @brief Test the functionality of the SIMZEDCam constructor and destructor.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST(SIMZEDCamTest, ConstructorDestructor)
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
TEST(SIMZEDCamTest, DoesNotLeak)
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
TEST(SIMZEDCamTest, Leaks)
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
TEST(SIMZEDCamTest, ResetPositionalTracking)
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
TEST(SIMZEDCamTest, RebootCamera)
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
TEST(SIMZEDCamTest, EnablePositionalTracking)
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
TEST(SIMZEDCamTest, DisablePositionalTracking)
{
    // Create a SIMZEDCam object.
    SIMZEDCam simZEDCam("/dev/video0", 1280, 720, 30, 90.0, 60.0, false, 1, 12345);

    // Call DisablePositionalTracking.
    simZEDCam.DisablePositionalTracking();

    // Check that positional tracking is disabled.
    // Assuming there is a method to check this, e.g., IsPositionalTrackingEnabled().
    EXPECT_FALSE(simZEDCam.GetPositionalTrackingEnabled());
}
