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
    SIMZEDCam* pZEDCam = new SIMZEDCam("ws://localhost:8080", 1280, 720, 30, PIXEL_FORMATS::eRGB, 90.0, 60.0, true);
    pZEDCam->Start();

    // Create a cv::Mat to store the frame.
    cv::Mat m_cvFrame;
    cv::Mat white_image(400, 400, CV_8UC3, cv::Scalar(255, 255, 255));

    while (true)
    {
        std::future<bool> fuFrame = pZEDCam->RequestFrameCopy(m_cvFrame);

        // Display the white image.
        // cv::imshow("White Image", white_image);

        // Wait for the frame to be copied.
        if (fuFrame.get() && !m_cvFrame.empty())
        {
            // Display the frame.
            cv::imshow("Frame", m_cvFrame);
        }

        // OpenCV display pause and check if while loop should exit.
        char chKey = cv::waitKey(1);
        if (chKey == 27)    // Press 'Esc' key to exit
            break;
    }
}
