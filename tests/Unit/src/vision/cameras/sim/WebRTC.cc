/******************************************************************************
 * @brief WebRTC unit tests.
 *
 * @file WebRTC.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../../src/vision/cameras/sim/WebRTC.h"

/// \cond
#include <chrono>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <rtc/rtc.hpp>
#include <thread>

/// \endcond

/******************************************************************************
 * @brief Test the functionality of the WebRTC constructor and destructor.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST(WebRTCTest, ConstructorDestructor)
{
    // Create a WebRTC object.
    WebRTC webrtc("ws://localhost:8080", "streamer1");

    // Check initial connection status.
    EXPECT_FALSE(webrtc.GetIsConnected());

    // Destructor will be called automatically when the object goes out of scope.
}

/******************************************************************************
 * @brief Check that WebRTC doesn't leak any memory.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST(WebRTCTest, DoesNotLeak)
{
    // Create a new WebRTC object.
    WebRTC* pWebRTC = new WebRTC("ws://localhost:8080", "streamer1");
    // Delete object.
    delete pWebRTC;
    // Point to null.
    pWebRTC = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST(WebRTCTest, Leaks)
{
    // Create a new WebRTC object.
    WebRTC* pWebRTC = new WebRTC("ws://localhost:8080", "streamer1");
    EXPECT_TRUE(pWebRTC != nullptr);
}
