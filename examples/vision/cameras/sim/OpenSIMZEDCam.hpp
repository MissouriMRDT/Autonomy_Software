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
    SIMZEDCam* pZEDCam = new SIMZEDCam("ws://127.0.0.1:8080", 1280, 720, 60, PIXEL_FORMATS::eRGB, 90.0, 60.0, true);
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
            // cv::imshow("Depth Measure", cvDepthMeasure);

            // Convert to a gray scale image.
            cv::cvtColor(cvDepthMeasure, cvDepthMeasure, cv::COLOR_BGR2GRAY);

            // This image is a grayscale image with the depth values in the pixel values. But the depth values are ranging from 0-255.
            // To get the actual depth values, we will divide the pixel values by 255 and multiply by the max depth value of the camera.
            // In this case, the max depth value of the camera is 20000 cm (20 meters).
            // The final image only needs to store uint16_t values, so we will convert the image to a 16 bit unsigned integer.

            // Convert the image to a 16 bit unsigned integer.
            cvDepthMeasure.convertTo(cvDepthMeasure, CV_16U);
            // Convert the pixel values to depth values.
            cvDepthMeasure = cvDepthMeasure * 20000 / 255;

            // Display the depth measure.
            cv::imshow("Depth Measure", cvDepthMeasure);

            // Print the depth value at the center of the image.
            LOG_INFO(logging::g_qSharedLogger, "Depth at center: {}", cvDepthMeasure.at<uint16_t>(cvDepthMeasure.rows / 2, cvDepthMeasure.cols / 2));
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
