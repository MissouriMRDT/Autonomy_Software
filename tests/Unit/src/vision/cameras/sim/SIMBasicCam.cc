/******************************************************************************
 * @brief SIMBasicCam unit tests.
 *
 * @file test_SIMBasicCam.cc
 * @date 2025-01-05
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../../src/vision/cameras/sim/SIMBasicCam.h"

/// \cond
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

/// \endcond

/******************************************************************************
 * @brief Test the functionality of the SIMBasicCam constructor and destructor.
 *
 * @date 2025-01-05
 ******************************************************************************/
TEST(SIMBasicCamTest, ConstructorDestructor)
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
TEST(SIMBasicCamTest, DoesNotLeak)
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
TEST(SIMBasicCamTest, Leaks)
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
TEST(SIMBasicCamTest, GetCameraIsOpen)
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
TEST(SIMBasicCamTest, GetCameraLocation)
{
    // Create a SIMBasicCam object.
    SIMBasicCam simBasicCam("ws://127.0.0.1:80", 1280, 720, 30, PIXEL_FORMATS::eBGRA, 90.0, 60.0, false, 1);

    // Check camera location.
    EXPECT_EQ(simBasicCam.GetCameraLocation(), "ws://127.0.0.1:80");
}