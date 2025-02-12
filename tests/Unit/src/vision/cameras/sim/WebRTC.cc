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
#include "../../../../../TestingBase.hh"

/// \cond
#include <chrono>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <rtc/rtc.hpp>
#include <thread>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the WebRTC
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class WebRTCTests : public TestingBase<WebRTCTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new WebRTC Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        WebRTCTests() {}

        /******************************************************************************
         * @brief Destroy the WebRTC Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        ~WebRTCTests() {}

        /******************************************************************************
         * @brief Setup the WebRTC Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Teardown the WebRTC Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Test the functionality of the WebRTC constructor and destructor.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-05
 ******************************************************************************/
TEST_F(WebRTCTests, ConstructorDestructor)
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
TEST_F(WebRTCTests, DoesNotLeak)
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
TEST_F(WebRTCTests, Leaks)
{
    // Create a new WebRTC object.
    WebRTC* pWebRTC = new WebRTC("ws://localhost:8080", "streamer1");
    EXPECT_TRUE(pWebRTC != nullptr);
}
