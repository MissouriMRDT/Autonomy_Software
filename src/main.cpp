/******************************************************************************
 * @brief Main program file. Sets up classes and runs main program functions.
 *
 * @file main.cpp
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

// #include <edgetpu.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
// #include <tensorflow/lite/interpreter.h>
// #include <tensorflow/lite/model.h>

#include "../examples/vision/tagdetection/ArucoDetectionBasicCam.hpp"
#include "./AutonomyGlobals.h"
#include "./AutonomyLogging.h"
#include "./interfaces/StateMachine.hpp"

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
    logging::InitializeLoggers();

    // Create model objects.
    // std::vector<edgetpu::EdgeTpuManager::DeviceEnumerationRecord> vEdgeTPUDevices = edgetpu::EdgeTpuManager::GetSingleton()->EnumerateEdgeTpu();
    // Loop through each device.
    // for (long unsigned int nIter = 0; nIter < vEdgeTPUDevices.size(); ++nIter)
    // {
    //     // Submit logger info.
    //     LOG_INFO(logging::g_qConsoleLogger, "Device {}: Type={}, Path={}", nIter, int(vEdgeTPUDevices[nIter].type), vEdgeTPUDevices[nIter].path);
    // }

    // Check whether or not we should run example code or continue with normal operation.
    if (bRunExampleFlag)
    {
        // Run example code from included file.
        RunExample();
    }
    else
    {
        // Initialize handlers.
        globals::g_pCameraHandler       = new CameraHandler();
        globals::g_pTagDetectionHandler = new TagDetectionHandler();
        // Start handlers.
        globals::g_pCameraHandler->StartAllCameras();
        globals::g_pTagDetectionHandler->StartAllDetectors();

        // TODO: Initialize RoveComm.

        /////////////////////////////////////////
        // Cleanup.
        /////////////////////////////////////////
        // Stop handlers.
        globals::g_pCameraHandler->StopAllCameras();
        globals::g_pTagDetectionHandler->StopAllDetectors();

        // Delete dynamically allocated objects.
        delete globals::g_pCameraHandler;
        delete globals::g_pTagDetectionHandler;

        // Set dangling pointers to null.
        globals::g_pCameraHandler       = nullptr;
        globals::g_pTagDetectionHandler = nullptr;
    }

    // Successful exit.
    return 0;
}
