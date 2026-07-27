/******************************************************************************
 * @brief Example file that demonstrates opening and using multiple different
 *      features of the ZED camera.
 *
 * @file OpenZEDCam.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-16
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../src/AutonomyConstants.h"
#include "../../../src/AutonomyGlobals.h"
#include "../../../src/AutonomyLogging.h"
#include "../../../src/AutonomyNetworking.h"
#include "../../../src/util/ExampleChecker.h"
#include "../../../src/vision/cameras/sim/SIMZEDCam.h"

/// \cond

/// \endcond

/******************************************************************************
 * @brief This example is used to demonstrate and test the SIMZEDCam class
 *      and its ability to establish a connection to a simulator and retrieve
 *      the video streams.
 *
 *      The simulated camera exposes exactly the same publish-latest channels as the real
 *      ZEDCam, so this consumer code is identical to the hardware version and the two
 *      are drop-in interchangeable.
 *
 *      Two behaviors worth noting for the simulated camera specifically:
 *
 *      - It does NOT stop its thread when the simulator is unreachable. It idles, retries
 *        the stream connection on a timer, and publishes nothing. So a simulator that is
 *        started AFTER this program will be picked up automatically, with no restart.
 *      - Because it publishes nothing while disconnected, Get() returns null (or a stale
 *        snapshot) rather than handing you blank frames. Always null-check, and use
 *        ullSequence to tell whether anything new actually arrived.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-22
 ******************************************************************************/
void RunExample()
{
    // Create a new SIMZEDCam object.
    std::unique_ptr<SIMZEDCam> pZEDCam = std::make_unique<SIMZEDCam>("ws://192.168.69.48:80", 1280, 720, 60, 90.0, 60.0, true);
    pZEDCam->Start();

    // Register demand for the channels we want. The camera produces a data type only while
    // something is subscribed to it, so these handles are what turn each one on.
    pubsub::Reader<cv::Mat> subFrame        = pZEDCam->GetFrameCPUReader();
    pubsub::Reader<cv::Mat> subDepthImage   = pZEDCam->GetDepthImageCPUReader();
    pubsub::Reader<cv::Mat> subDepthMeasure = pZEDCam->GetDepthMeasureCPUReader();

    // Track which frame we last processed so we can skip iterations with nothing new.
    unsigned long long ullLastProcessedSequence = 0;

    while (true)
    {
        // Load the newest snapshot of each channel ONCE into a local. Non-blocking reads that
        // return null until the simulator connects and the camera publishes.
        pubsub::Reader<cv::Mat>::SharedSnapshot pFrame        = subFrame.Get();
        pubsub::Reader<cv::Mat>::SharedSnapshot pDepthImage   = subDepthImage.Get();
        pubsub::Reader<cv::Mat>::SharedSnapshot pDepthMeasure = subDepthMeasure.Get();

        // Only redraw when the camera has actually produced a new frame.
        if (pFrame != nullptr && pFrame->ullSequence != ullLastProcessedSequence)
        {
            // Remember which frame we processed so a repeat of it is skipped next time.
            ullLastProcessedSequence = pFrame->ullSequence;
            // Display it straight from the snapshot. imshow only reads, so no copy is needed here;
            // we would only need one if we intended to draw on the frame.
            cv::imshow("Frame", pFrame->tData);
        }

        // Show the depth image if one has been published.
        if (pDepthImage != nullptr)
        {
            // Display the depth image.
            cv::imshow("Depth Image", pDepthImage->tData);
        }

        // Show the depth measure if one has been published.
        if (pDepthMeasure != nullptr)
        {
            // Work from the immutable snapshot's data.
            const cv::Mat& cvDepthMeasure = pDepthMeasure->tData;
            // Display the depth measure.
            cv::imshow("Depth Measure", cvDepthMeasure);

            // Print out depth value at center of image.
            LOG_INFO(logging::g_qSharedLogger, "Depth at center of image: {}", cvDepthMeasure.at<uint16_t>(cvDepthMeasure.rows / 2, cvDepthMeasure.cols / 2));
        }

        // Print camera FPS stat.
        // LOG_INFO(logging::g_qSharedLogger, "Camera FPS: {}", pZEDCam->GetIPS().GetExactIPS());

        // OpenCV display pause and check if while loop should exit.
        char chKey = cv::waitKey(1);
        if (chKey == 27)    // Press 'Esc' key to exit
            break;
    }

    // Withdraw our demand so the camera stops producing data nobody is reading. This also happens
    // automatically when these handles go out of scope.
    subFrame.Release();
    subDepthImage.Release();
    subDepthMeasure.Release();

    // Stop the camera.
    pZEDCam->RequestStop();
    pZEDCam->Join();
}
