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
#include "./threads/CameraHandlerThread.h"
#include "./util/OpenCV/ImageOperations.hpp"

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
        // Initialize and start Threads
        g_pCameraHandler = new CameraHandlerThread();

        // TODO: Initialize RoveComm

        // Get reference to camera.
        ZEDCam* TestCamera1 = g_pCameraHandler->GetZED(CameraHandlerThread::eHeadMainCam);
        // Turn on ZED features.
        TestCamera1->EnablePositionalTracking();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        TestCamera1->EnableSpatialMapping();
        // Declare mats to store images in.
        sl::Mat slResultFrame1;
        sl::Mat slDepthFrame1;
        cv::Mat cvNormalFrame1;
        cv::Mat cvDepthFrame1;
        cv::cuda::GpuMat cvGPUNormalFrame1;
        cv::cuda::GpuMat cvGPUDepthFrame1;
        while (true)
        {
            // Grab normal frame from camera.
            slResultFrame1 = TestCamera1->GrabFrame();
            slDepthFrame1  = TestCamera1->GrabDepth(false);
            // Convert to OpenCV Mat.
            // cvGPUNormalFrame1 = imgops::ConvertSLMatToGPUMat(slResultFrame1);
            // cvGPUDepthFrame1  = imgops::ConvertSLMatToGPUMat(slDepthFrame1);
            // Download mats from GPU memory into CPU memory.
            // cvGPUNormalFrame1.download(cvNormalFrame1);
            // cvGPUDepthFrame1.download(cvDepthFrame1);
            cvNormalFrame1 = imgops::ConvertSLMatToCVMat(slResultFrame1);
            cvDepthFrame1  = imgops::ConvertSLMatToCVMat(slDepthFrame1);

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame1,
                        std::to_string(TestCamera1->GetIPS()->GetAverageIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Put FPS on depth frame.
            cv::putText(cvDepthFrame1, std::to_string(TestCamera1->GetIPS()->GetAverageIPS()), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("TEST1", cvNormalFrame1);
            cv::imshow("DEPTH1", cvDepthFrame1);

            // Print info about position.
            sl::Translation slLocation = TestCamera1->GetPositionalPose().getTranslation();
            LOG_INFO(g_qConsoleLogger, "Positional Tracking:\nX: {} \nY: {}\nZ: {}\n\n", slLocation.x, slLocation.y, slLocation.z);
            LOG_INFO(g_qConsoleLogger, "Spatial Mapping State: {}", sl::toString(TestCamera1->GetSpatialMappingState()).get());
            std::vector<double> vIMUValues = TestCamera1->GetIMUData();
            LOG_INFO(g_qConsoleLogger, "IMU Data:\nRoll: {}\nPitch: {}\nYaw:{}\n", vIMUValues[0], vIMUValues[1], vIMUValues[2]);

            char chKey = cv::waitKey(1);
            if (chKey == 27)    // Press 'Esc' key to exit
                break;
        }

        cv::destroyAllWindows();

        // Extract spatial map.
        std::future<sl::Mesh> fuSpatialMap;
        TestCamera1->ExtractSpatialMapAsync(fuSpatialMap);
        sl::Mesh slSpatialMap = fuSpatialMap.get();
        slSpatialMap.save("test.obj", sl::MESH_FILE_FORMAT::PLY);
    }

    // Delete dynamically allocated memory.
    delete g_pCameraHandler;

    // Set dangling pointers to null.
    g_pCameraHandler = nullptr;

    // Successful exit.
    return 0;
}
