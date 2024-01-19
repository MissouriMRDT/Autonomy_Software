/******************************************************************************
 * @brief Main program file. Sets up classes and runs main program functions.
 *
 * @file main.cpp
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "./AutonomyGlobals.h"
#include "./AutonomyLogging.h"

// Check if any file from the example directory has been included.
// If not included, define empty run example function and set bRunExampleFlag
// to false. If included, then define bRunExampleFlag as true.
#ifndef CHECK_IF_EXAMPLE_INCLUDED
static bool bRunExampleFlag = false;

void RunExample() {}
#else
CHECK_IF_EXAMPLE_INCLUDED
#endif

// Create a boolean used to handle a SIGINT and exit gracefully.
volatile sig_atomic_t bMainStop = false;

/******************************************************************************
 * @brief Help function given to the C++ csignal standard library to run when
 *      a CONTROL^C is given from the terminal.
 *
 * @param nSignal - Integer representing the interrupt value.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-08
 ******************************************************************************/
void SignalHandler(int nSignal)
{
    // Check signal type.
    if (nSignal == SIGINT)
    {
        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "Ctrl+C received. Cleaning up...");

        // Update stop signal.
        bMainStop = true;
    }
}

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
    logging::InitializeLoggers(constants::LOGGING_OUTPUT_PATH_ABSOLUTE);

    // Check whether or not we should run example code or continue with normal operation.
    if (bRunExampleFlag)
    {
        // Run example code from included file.
        RunExample();
    }
    else
    {
        // Setup signal interrupt handler.
        struct sigaction stSigBreak;
        stSigBreak.sa_handler = SignalHandler;
        stSigBreak.sa_flags   = 0;
        sigemptyset(&stSigBreak.sa_mask);
        sigaction(SIGINT, &stSigBreak, nullptr);

        // Initialize handlers.
        globals::g_pCameraHandler       = new CameraHandler();
        globals::g_pTagDetectionHandler = new TagDetectionHandler();
        globals::g_pStateMachineHandler = new StateMachineHandler();
        // Start handlers.
        globals::g_pCameraHandler->StartAllCameras();
        globals::g_pTagDetectionHandler->StartAllDetectors();
        globals::g_pStateMachineHandler->StartStateMachine();
        // // Enable Recording on Handlers.
        globals::g_pCameraHandler->StartRecording();
        globals::g_pTagDetectionHandler->StartRecording();

        // TODO: Initialize RoveComm.

        /*
            This while loop is the main periodic loop for the Autonomy_Software program.
            Loop until user sends sigkill or sigterm.
        */
        while (!bMainStop)
        {
            // No need to loop as fast as possible. Sleep...
            // Only run this main thread once every 20ms.
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            // Get Camera and Tag detector pointers .
            ZEDCam* pMainCam            = globals::g_pCameraHandler->GetZED(CameraHandler::eHeadMainCam);
            BasicCam* pLeftCam          = globals::g_pCameraHandler->GetBasicCam(CameraHandler::eHeadLeftArucoEye);
            BasicCam* pRightCam         = globals::g_pCameraHandler->GetBasicCam(CameraHandler::eHeadRightArucoEye);
            TagDetector* pMainDetector  = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::eHeadMainCam);
            TagDetector* pLeftDetector  = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::eHeadLeftArucoEye);
            TagDetector* pRightDetector = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::eHeadRightArucoEye);
            // Create a string to append FPS values to.
            std::string szThreadsFPS = "";
            // Get FPS of all cameras and detectors and construct the info into a string.
            szThreadsFPS += "--------[ Threads FPS ]--------\n";
            szThreadsFPS += "MainCam FPS: " + std::to_string(pMainCam->GetIPS().GetExactIPS()) + "\n";
            szThreadsFPS += "LeftCam FPS: " + std::to_string(pLeftCam->GetIPS().GetExactIPS()) + "\n";
            szThreadsFPS += "RightCam FPS: " + std::to_string(pRightCam->GetIPS().GetExactIPS()) + "\n";
            szThreadsFPS += "MainDetector FPS: " + std::to_string(pMainDetector->GetIPS().GetExactIPS()) + "\n";
            szThreadsFPS += "LeftDetector FPS: " + std::to_string(pLeftDetector->GetIPS().GetExactIPS()) + "\n";
            szThreadsFPS += "RightDetector GPS: " + std::to_string(pRightDetector->GetIPS().GetExactIPS()) + "\n";
            // Submit logger message.
            LOG_INFO(logging::g_qConsoleLogger, "{}", szThreadsFPS);

            // Send Start Command
            globals::g_pStateMachineHandler->HandleEvent(statemachine::Event::eStart);
        }

        /////////////////////////////////////////
        // Cleanup.
        /////////////////////////////////////////
        // Stop handlers.
        globals::g_pTagDetectionHandler->StopAllDetectors();
        globals::g_pCameraHandler->StopAllCameras();

        // Delete dynamically allocated objects.
        delete globals::g_pTagDetectionHandler;
        delete globals::g_pCameraHandler;

        // Set dangling pointers to null.
        globals::g_pTagDetectionHandler = nullptr;
        globals::g_pCameraHandler       = nullptr;
    }

    // Submit logger message that program is done cleaning up and is now exiting.
    LOG_INFO(logging::g_qSharedLogger, "Clean up finished. Exiting...");

    // Successful exit.
    return 0;
}
