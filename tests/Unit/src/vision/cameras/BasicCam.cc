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

/// \cond
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

/// \endcond

/******************************************************************************
 * @brief Test the functionality of the BasicCam constructor and destructor.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST(BasicCamTest, ConstructorDestructor)
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
TEST(BasicCamTest, DoesNotLeak)
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
TEST(BasicCamTest, Leaks)
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
TEST(BasicCamTest, GetCameraIsOpen)
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
TEST(BasicCamTest, GetCameraLocation)
{
    // Create a BasicCam object.
    BasicCam basicCam("/dev/video0", 1280, 720, 30, PIXEL_FORMATS::eBGRA, 90.0, 60.0, false, 1);

    // Check camera location.
    EXPECT_EQ(basicCam.GetCameraLocation(), "/dev/video0");
}
