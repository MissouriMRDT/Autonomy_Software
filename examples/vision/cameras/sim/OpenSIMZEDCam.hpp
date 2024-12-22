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
            // cv::imshow("Frame", cvFrame);
        }

        // Wait for the depth image to be copied.
        if (fuDepthImage.get() && !cvDepthImage.empty())
        {
            // Convert the frame to a supported type and display it.
            // cv::imshow("Depth Image", cvDepthImage);
        }

        // Wait for the depth measure to be copied.
        if (fuDepthMeasure.get() && !cvDepthMeasure.empty())
        {
            // Display the depth measure.
            cv::imshow("Depth Measure", cvDepthMeasure);

            // The Simulator uses this a special method of packing the depth measure data as defined in this paper.
            // http://reality.cs.ucl.ac.uk/projects/depth-streaming/depth-streaming.pdf
            // Here we will decode it.
            // Create a new cv::Mat to store the decoded depth data
            cv::Mat cvDecodedDepth = cv::Mat(cvDepthMeasure.rows, cvDepthMeasure.cols, CV_16UC1);
            float w                = 65536.0;
            float np               = 512.0;

            // Iterate over each pixel in the cvDepthMeasure image
            for (int y = 0; y < cvDepthMeasure.rows; ++y)
            {
                for (int x = 0; x < cvDepthMeasure.cols; ++x)
                {
                    // Extract the encoded depth values
                    cv::Vec3b cvEncodedDepth = cvDepthMeasure.at<cv::Vec3b>(y, x);

                    // Extract encoded values
                    float L  = cvEncodedDepth[2] / 255.0;
                    float Ha = cvEncodedDepth[1] / 255.0;
                    float Hb = cvEncodedDepth[0] / 255.0;

                    // Period for triangle waves
                    float p = np / w;

                    // Determine offset and fine-grain correction
                    int m    = fmod((4.0 * (L / p)) - 0.5, 4.0);
                    float L0 = L - fmod(L - (p / 8.0), p) + ((p / 4.0) * m) - (p / 8.0);

                    float delta;
                    if (m == 0)
                        delta = (p / 2.0) * Ha;
                    else if (m == 1)
                        delta = (p / 2.0) * Hb;
                    else if (m == 2)
                        delta = (p / 2.0) * (1.0 - Ha);
                    else if (m == 3)
                        delta = (p / 2.0) * (1.0 - Hb);

                    // Combine to compute the original depth
                    float depth = w * (L0 + delta);

                    // Store the decoded depth in the new cv::Mat
                    cvDecodedDepth.at<uint16_t>(y, x) = static_cast<uint16_t>(depth);
                }
            }

            // Convert the 16-bit depth image to a 8-bit image for display.
            cv::Mat cvDecodedDepth8;
            cvDecodedDepth.convertTo(cvDecodedDepth8, CV_8U, 255.0 / 65535.0);
            // Display the decoded depth image.
            cv::imshow("Decoded Depth", cvDecodedDepth8);
            // Print out depth value at center of image.
            LOG_INFO(logging::g_qSharedLogger, "Depth at center of image: {}", cvDecodedDepth.at<uint16_t>(cvDecodedDepth.rows / 2, cvDecodedDepth.cols / 2));
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
