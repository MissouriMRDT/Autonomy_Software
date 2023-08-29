/******************************************************************************
 * @brief Main program file. Sets up classes and runs main program functions.
 *
 * @file main.cpp
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "./AutonomyGlobals.h"
#include "./AutonomyLogging.h"
#include "./interfaces/StateMachine.hpp"
#include "./vision/cameras/ZEDCam.h"
// #include "./threads/CameraHandlerThread.h"

// Check if any file from the example directory has been included.
// If not included, define empty run example function and set bRunExampleFlag
// to false. If included, then define bRunExampleFlag as true.
#ifndef CHECK_IF_EXAMPLE_INCLUDED
static bool bRunExampleFlag = false;

void RunExample() {}
#else
CHECK_IF_EXAMPLE_INCLUDED
#endif

/******************************************************************************
 * @brief Autonomy main function.
 *
 * @return int - Exit status number.
 *
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 ******************************************************************************/
int main()
{
    // Print Software Header
    std::ifstream fHeaderText("../data/ASCII/v24.txt");
    std::string szHeaderText;
    if (fHeaderText)
    {
        std::ostringstream pHeaderText;
        pHeaderText << fHeaderText.rdbuf();
        szHeaderText = pHeaderText.str();
    }

    std::cout << szHeaderText << std::endl;
    std::cout << "Copyright \u00A9 2023 - Mars Rover Design Team\n" << std::endl;

    // Initialize Loggers
    InitializeLoggers();

    // Check whether or not we should run example code or continue with normal operation.
    if (bRunExampleFlag)
    {
        // Run example code from included file.
        RunExample();
    }
    else
    {
        // TODO: Initialize Threads

        // TODO: Initialize RoveComm

        // Init camera.
        // BasicCam TestCamera1 = BasicCam(0, 640, 480, 30, eRGB, 45, 45);
        // BasicCam TestCamera2 = BasicCam(2, 640, 480, 30, eRGB, 45, 45);
        // BasicCam TestCamera3 = BasicCam(4, 640, 480, 30, eRGB, 45, 45);
        // cv::Mat cvResultFrame1;
        // cv::Mat cvResultFrame2;
        // cv::Mat cvResultFrame3;
        // while (true)
        // {
        //     // Grab Frames from camera and draw FPS on frame.
        //     cvResultFrame1 = TestCamera1.GrabFrame();
        //     cv::putText(cvResultFrame1,
        //                 std::to_string(TestCamera1.GetFrameIPS()->GetAverageIPS()),
        //                 cv::Point(50, 50),
        //                 cv::FONT_HERSHEY_COMPLEX,
        //                 1,
        //                 cv::Scalar(255, 255, 255));

        //     // Grab Frames from camera and draw FPS on frame.
        //     cvResultFrame2 = TestCamera2.GrabFrame();
        //     cv::putText(cvResultFrame2,
        //                 std::to_string(TestCamera2.GetFrameIPS()->GetAverageIPS()),
        //                 cv::Point(50, 50),
        //                 cv::FONT_HERSHEY_COMPLEX,
        //                 1,
        //                 cv::Scalar(255, 255, 255));

        //     // Grab Frames from camera and draw FPS on frame.
        //     cvResultFrame3 = TestCamera3.GrabFrame();
        //     cv::putText(cvResultFrame3,
        //                 std::to_string(TestCamera3.GetFrameIPS()->GetAverageIPS()),
        //                 cv::Point(50, 50),
        //                 cv::FONT_HERSHEY_COMPLEX,
        //                 1,
        //                 cv::Scalar(255, 255, 255));

        //     // Display frame.
        //     cv::imshow("TEST1", cvResultFrame1);
        //     cv::imshow("TEST2", cvResultFrame2);
        //     cv::imshow("TEST3", cvResultFrame3);
        //     // cv::imshow("TEST4", cvResultFrame4);

        //     char chKey = cv::waitKey(1);
        //     if (chKey == 27)    // Press 'Esc' key to exit
        //         break;
        // }

        // cv::destroyAllWindows();
    }

    return 0;
}
