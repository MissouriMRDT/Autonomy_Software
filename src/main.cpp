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
#include "./util/OpenCV/ImageOperations.hpp"
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
        ZEDCam TestCamera1 = ZEDCam(1280, 720, 60, 110, 80, 0.2f, 40.0f, true);
        sl::Mat slResultFrame1;
        sl::Mat slDepthFrame1;
        cv::Mat cvNormalFrame1;
        cv::Mat cvDepthFrame1;
        cv::cuda::GpuMat cvGPUNormalFrame1;
        cv::cuda::GpuMat cvGPUDepthFrame1;
        while (true)
        {
            // Grab normal frame from camera.
            slResultFrame1 = TestCamera1.GrabFrame();
            slDepthFrame1  = TestCamera1.GrabDepth(false);
            // Convert to OpenCV Mat.
            cvGPUNormalFrame1 = imgops::ConvertSLMatToGPUMat(slResultFrame1);
            cvGPUDepthFrame1  = imgops::ConvertSLMatToGPUMat(slDepthFrame1);
            // Download mats from GPU memory into CPU memory.
            cvGPUNormalFrame1.download(cvNormalFrame1);
            cvGPUDepthFrame1.download(cvDepthFrame1);
            // cvNormalFrame1 = imgops::ConvertSLMatToCVMat(slResultFrame1);
            // cvDepthFrame1  = imgops::ConvertSLMatToCVMat(slDepthFrame1);

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame1,
                        std::to_string(TestCamera1.GetIPS(ZEDCam::eFRAME)->GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Put FPS on depth frame.
            cv::putText(cvDepthFrame1,
                        std::to_string(TestCamera1.GetIPS(ZEDCam::eDEPTH)->GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("TEST1", cvNormalFrame1);
            cv::imshow("DEPTH1", cvDepthFrame1);

            char chKey = cv::waitKey(1);
            if (chKey == 27)    // Press 'Esc' key to exit
                break;
        }

        cv::destroyAllWindows();
    }

    return 0;
}
