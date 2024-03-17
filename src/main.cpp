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

        // Print warnings if running in SIM mode.
        if (constants::MODE_SIM)
        {
            // Print 5 times to make it noticeable.
            for (int nIter = 0; nIter < 5; ++nIter)
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Autonomy_Software is running in SIM mode! If you aren't currently using the WeBots sim, disable SIM mode in constants");
            }

            // Sleep for 3 seconds to make sure it's seen.
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }

        /////////////////////////////////////////
        // Setup global objects.
        /////////////////////////////////////////
        // Initialize RoveComm.
        globals::g_pRoveCommUDPNode = new rovecomm::RoveCommUDP();
        globals::g_pRoveCommTCPNode = new rovecomm::RoveCommTCP();
        // Start RoveComm instances bound on ports.
        bool bRoveCommUDPInitSuccess = globals::g_pRoveCommUDPNode->InitUDPSocket(manifest::General::ETHERNET_UDP_PORT);
        bool bRoveCommTCPInitSuccess = globals::g_pRoveCommTCPNode->InitTCPSocket(constants::ROVECOMM_TCP_INTERFACE_IP.c_str(), manifest::General::ETHERNET_UDP_PORT);
        // Check if RoveComm was successfully initialized.
        if (!bRoveCommUDPInitSuccess || !bRoveCommTCPInitSuccess)
        {
            // Submit logger message.
            LOG_CRITICAL(logging::g_qSharedLogger,
                         "RoveComm did not initialize properly! UDPNode Status: {}, TCPNode Status: {}",
                         bRoveCommUDPInitSuccess,
                         bRoveCommTCPInitSuccess);

            // Since RoveComm is crucial, stop code.
            bMainStop = true;
        }
        else
        {
            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "RoveComm UDP and TCP nodes successfully initialized.");
        }

        // Initialize drivers.
        globals::g_pDriveBoard      = new DriveBoard();
        globals::g_pMultimediaBoard = new MultimediaBoard();
        globals::g_pNavigationBoard = new NavigationBoard();
        // Initialize handlers.
        globals::g_pWaypointHandler     = new WaypointHandler();
        globals::g_pCameraHandler       = new CameraHandler();
        globals::g_pTagDetectionHandler = new TagDetectionHandler();
        globals::g_pStateMachineHandler = new StateMachineHandler();

        // Start handlers.
        globals::g_pCameraHandler->StartAllCameras();
        globals::g_pTagDetectionHandler->StartAllDetectors();
        globals::g_pStateMachineHandler->StartStateMachine();
        // Enable Recording on Handlers.
        globals::g_pCameraHandler->StartRecording();
        globals::g_pTagDetectionHandler->StartRecording();

        /////////////////////////////////////////
        // Declare local variables used in main loop.
        /////////////////////////////////////////
        // Get Camera and Tag detector pointers .
        ZEDCam* pMainCam            = globals::g_pCameraHandler->GetZED(CameraHandler::eHeadMainCam);
        BasicCam* pLeftCam          = globals::g_pCameraHandler->GetBasicCam(CameraHandler::eHeadLeftArucoEye);
        BasicCam* pRightCam         = globals::g_pCameraHandler->GetBasicCam(CameraHandler::eHeadRightArucoEye);
        TagDetector* pMainDetector  = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::eHeadMainCam);
        TagDetector* pLeftDetector  = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::eHeadLeftArucoEye);
        TagDetector* pRightDetector = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::eHeadRightArucoEye);
        IPS IterPerSecond           = IPS();

        // Camera and TagDetector config.
        pMainCam->EnablePositionalTracking();    // Enable positional tracking for main ZED cam.

        /*
            This while loop is the main periodic loop for the Autonomy_Software program.
            Loop until user sends sigkill or sigterm.
        */
        while (!bMainStop)
        {
            // Send current robot state over RoveComm.
            // Construct a RoveComm packet with the drive data.
            rovecomm::RoveCommPacket<uint8_t> stPacket;
            stPacket.unDataId    = manifest::Autonomy::TELEMETRY.find("CURRENTSTATE")->second.DATA_ID;
            stPacket.unDataCount = manifest::Autonomy::TELEMETRY.find("CURRENTSTATE")->second.DATA_COUNT;
            stPacket.eDataType   = manifest::Autonomy::TELEMETRY.find("CURRENTSTATE")->second.DATA_TYPE;
            stPacket.vData.emplace_back(static_cast<uint8_t>(globals::g_pStateMachineHandler->GetCurrentState()));
            // Send drive command over RoveComm to drive board to all subscribers.
            globals::g_pRoveCommUDPNode->SendUDPPacket(stPacket, "0.0.0.0", constants::ROVECOMM_OUTGOING_UDP_PORT);

            // Create a string to append FPS values to.
            std::string szMainInfo = "";
            // Get FPS of all cameras and detectors and construct the info into a string.
            szMainInfo += "\n--------[ Threads FPS ]--------\n";
            szMainInfo += "Main Process FPS: " + std::to_string(IterPerSecond.GetAverageIPS()) + "\n";
            szMainInfo += "MainCam FPS: " + std::to_string(pMainCam->GetIPS().GetAverageIPS()) + "\n";
            szMainInfo += "LeftCam FPS: " + std::to_string(pLeftCam->GetIPS().GetAverageIPS()) + "\n";
            szMainInfo += "RightCam FPS: " + std::to_string(pRightCam->GetIPS().GetAverageIPS()) + "\n";
            szMainInfo += "MainDetector FPS: " + std::to_string(pMainDetector->GetIPS().GetAverageIPS()) + "\n";
            szMainInfo += "LeftDetector FPS: " + std::to_string(pLeftDetector->GetIPS().GetAverageIPS()) + "\n";
            szMainInfo += "RightDetector FPS: " + std::to_string(pRightDetector->GetIPS().GetAverageIPS()) + "\n";
            szMainInfo += "\nStateMachine FPS: " + std::to_string(globals::g_pStateMachineHandler->GetIPS().GetAverageIPS()) + "\n";
            szMainInfo += "\nRoveCommUDP FPS: " + std::to_string(globals::g_pRoveCommTCPNode->GetIPS().GetAverageIPS()) + "\n";
            szMainInfo += "RoveCommTCP FPS: " + std::to_string(globals::g_pRoveCommTCPNode->GetIPS().GetAverageIPS()) + "\n";
            szMainInfo += "\n--------[ State Machine Info ]--------\n";
            szMainInfo += "Current State: " + statemachine::StateToString(globals::g_pStateMachineHandler->GetCurrentState()) + "\n";
            szMainInfo += "\n--------[ Camera Info ]--------\n";

            // Submit logger message.
            LOG_DEBUG(logging::g_qSharedLogger, "{}", szMainInfo);

            // Update IPS tick.
            IterPerSecond.Tick();

            // No need to loop as fast as possible. Sleep...
            // Only run this main thread once every 20ms.
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        /////////////////////////////////////////
        // Cleanup.
        /////////////////////////////////////////
        // Stop handlers.
        globals::g_pStateMachineHandler->StopStateMachine();
        globals::g_pTagDetectionHandler->StopAllDetectors();
        globals::g_pCameraHandler->StopAllCameras();
        globals::g_pRoveCommUDPNode->CloseUDPSocket();
        globals::g_pRoveCommTCPNode->CloseTCPSocket();

        // Delete dynamically allocated objects.
        delete globals::g_pStateMachineHandler;
        delete globals::g_pTagDetectionHandler;
        delete globals::g_pCameraHandler;
        delete globals::g_pWaypointHandler;
        delete globals::g_pRoveCommUDPNode;
        delete globals::g_pRoveCommTCPNode;

        // Set dangling pointers to null.
        globals::g_pStateMachineHandler = nullptr;
        globals::g_pTagDetectionHandler = nullptr;
        globals::g_pCameraHandler       = nullptr;
        globals::g_pWaypointHandler     = nullptr;
        globals::g_pRoveCommUDPNode     = nullptr;
        globals::g_pRoveCommTCPNode     = nullptr;
    }

    // Submit logger message that program is done cleaning up and is now exiting.
    LOG_INFO(logging::g_qSharedLogger, "Clean up finished. Exiting...");

    // Successful exit.
    return 0;
}
