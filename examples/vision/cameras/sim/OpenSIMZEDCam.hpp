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
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-22
 ******************************************************************************/
void RunExample()
{
    // Create a new SIMZEDCam object.
    SIMZEDCam* pZEDCam = new SIMZEDCam("ws://192.168.69.48:80", 1280, 720, 60, 90.0, 60.0, true);
    pZEDCam->Start();

    // Create a cv::Mat to store the frame.
    cv::Mat cvFrame;
    cv::Mat cvDepthImage;
    cv::Mat cvDepthMeasure;

    while (true)
    {
        std::future<bool> fuFrame        = pZEDCam->RequestFrameCopy(cvFrame);
        std::future<bool> fuDepthImage   = pZEDCam->RequestDepthCopy(cvDepthImage, false);
        std::future<bool> fuDepthMeasure = pZEDCam->RequestDepthCopy(cvDepthMeasure, true);

        // Wait for the frame to be copied.
        if (fuFrame.get() && !cvFrame.empty())
        {
            // Convert the frame to a supported type and display it.
            cv::imshow("Frame", cvFrame);
        }

        // Wait for the depth image to be copied.
        if (fuDepthImage.get() && !cvDepthImage.empty())
        {
            // Convert the frame to a supported type and display it.
            cv::imshow("Depth Image", cvDepthImage);
        }

        // Wait for the depth measure to be copied.
        if (fuDepthMeasure.get() && !cvDepthMeasure.empty())
        {
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

    // Stop the camera.
    pZEDCam->RequestStop();
    pZEDCam->Join();

    // Delete the camera object.
    delete pZEDCam;
    // Set dangling pointer to nullptr.
    pZEDCam = nullptr;
}
