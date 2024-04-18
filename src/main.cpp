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
#include "./AutonomyNetworking.h"

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
    if (nSignal == SIGINT || nSignal == SIGTERM)
    {
        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "Ctrl+C or SIGTERM received. Cleaning up...");

        // Update stop signal.
        bMainStop = true;
    }
    // The SIGQUIT signal can be sent to the terminal by pressing CNTL+\.
    else if (nSignal == SIGQUIT)
    {
        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "Quit signal key pressed. Cleaning up...");

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
        sigaction(SIGQUIT, &stSigBreak, nullptr);

        // Print warnings if running in SIM mode.
        if (constants::MODE_SIM)
        {
            // Print 5 times to make it noticeable.
            for (int nIter = 0; nIter < 5; ++nIter)
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Autonomy_Software is running in SIM mode! If you aren't currently using the WeBots sim, disable SIM mode in CMakeLists.txt or in your build "
                            "arguments!");
            }

            // Sleep for 3 seconds to make sure it's seen.
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }

        /////////////////////////////////////////
        // Setup global objects.
        /////////////////////////////////////////
        // Initialize RoveComm.
        network::g_pRoveCommUDPNode = new rovecomm::RoveCommUDP();
        network::g_pRoveCommTCPNode = new rovecomm::RoveCommTCP();
        // Start RoveComm instances bound on ports.
        network::g_bRoveCommUDPStatus = network::g_pRoveCommUDPNode->InitUDPSocket(manifest::General::ETHERNET_UDP_PORT);
        network::g_bRoveCommTCPStatus = network::g_pRoveCommTCPNode->InitTCPSocket(constants::ROVECOMM_TCP_INTERFACE_IP.c_str(), manifest::General::ETHERNET_TCP_PORT);
        // Check if RoveComm was successfully initialized.
        if (!network::g_bRoveCommUDPStatus || !network::g_bRoveCommTCPStatus)
        {
            // Submit logger message.
            LOG_CRITICAL(logging::g_qSharedLogger,
                         "RoveComm did not initialize properly! UDPNode Status: {}, TCPNode Status: {}",
                         network::g_bRoveCommUDPStatus,
                         network::g_bRoveCommTCPStatus);

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
        ZEDCam* pLeftCam            = globals::g_pCameraHandler->GetZED(CameraHandler::eFrameLeftCam);
        ZEDCam* pRightCam           = globals::g_pCameraHandler->GetZED(CameraHandler::eFrameRightCam);
        BasicCam* pGroundCam        = globals::g_pCameraHandler->GetBasicCam(CameraHandler::eHeadGroundCam);
        TagDetector* pMainDetector  = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::eHeadMainCam);
        TagDetector* pLeftDetector  = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::eFrameLeftCam);
        TagDetector* pRightDetector = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::eFrameRightCam);
        IPS IterPerSecond           = IPS();

        // Camera and TagDetector config.
        pMainCam->EnablePositionalTracking();    // Enable positional tracking for main ZED cam.
        pMainCam->EnableSpatialMapping();

        /*
            This while loop is the main periodic loop for the Autonomy_Software program.
            Loop until user sends sigkill or sigterm.
        */
        sl::Pose slTest;
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
            network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, "0.0.0.0", constants::ROVECOMM_OUTGOING_UDP_PORT);

            // Create a string to append FPS values to.
            std::string szMainInfo = "";
            // Get FPS of all cameras and detectors and construct the info into a string.
            szMainInfo += "\n--------[ Threads FPS ]--------\n";
            szMainInfo += "Main Process FPS: " + std::to_string(IterPerSecond.GetExactIPS()) + "\n";
            szMainInfo += "MainCam FPS: " + std::to_string(pMainCam->GetIPS().GetExactIPS()) + "\n";
            szMainInfo += "LeftCam FPS: " + std::to_string(pLeftCam->GetIPS().GetExactIPS()) + "\n";
            szMainInfo += "RightCam FPS: " + std::to_string(pRightCam->GetIPS().GetExactIPS()) + "\n";
            szMainInfo += "GroundCam FPS: " + std::to_string(pGroundCam->GetIPS().GetExactIPS()) + "\n";
            szMainInfo += "MainDetector FPS: " + std::to_string(pMainDetector->GetIPS().GetExactIPS()) + "\n";
            szMainInfo += "LeftDetector FPS: " + std::to_string(pLeftDetector->GetIPS().GetExactIPS()) + "\n";
            szMainInfo += "RightDetector FPS: " + std::to_string(pRightDetector->GetIPS().GetExactIPS()) + "\n";
            szMainInfo += "\nStateMachine FPS: " + std::to_string(globals::g_pStateMachineHandler->GetIPS().GetExactIPS()) + "\n";
            szMainInfo += "\nRoveCommUDP FPS: " + std::to_string(network::g_pRoveCommTCPNode->GetIPS().GetExactIPS()) + "\n";
            szMainInfo += "RoveCommTCP FPS: " + std::to_string(network::g_pRoveCommTCPNode->GetIPS().GetExactIPS()) + "\n";
            szMainInfo += "\n--------[ State Machine Info ]--------\n";
            szMainInfo += "Current State: " + statemachine::StateToString(globals::g_pStateMachineHandler->GetCurrentState()) + "\n";

            // Submit logger message.
            LOG_DEBUG(logging::g_qSharedLogger, "{}", szMainInfo);

            // Update IPS tick.
            IterPerSecond.Tick();

            // No need to loop as fast as possible. Sleep...
            std::this_thread::sleep_for(std::chrono::milliseconds(60));
        }

        /////////////////////////////////////////
        // Cleanup.
        /////////////////////////////////////////

        // Check if ZED spatial map was enabled.
        if (pMainCam->GetSpatialMappingState() == sl::SPATIAL_MAPPING_STATE::OK)
        {
            // Extract and save spatial map.
            std::future<sl::Mesh> fuSpatialMap;
            pMainCam->ExtractSpatialMapAsync(fuSpatialMap);
            sl::Mesh slSpatialMap  = fuSpatialMap.get();
            std::string szFilePath = constants::LOGGING_OUTPUT_PATH_ABSOLUTE + logging::g_szProgramStartTimeString + "/spatial_map";
            slSpatialMap.save(szFilePath.c_str(), sl::MESH_FILE_FORMAT::PLY);
        }

        // Stop RoveComm quill logging or quill will segfault if trying to output logs to RoveComm.
        network::g_bRoveCommUDPStatus = false;
        network::g_bRoveCommTCPStatus = false;

        // Stop handlers.
        globals::g_pStateMachineHandler->StopStateMachine();
        globals::g_pTagDetectionHandler->StopAllDetectors();
        globals::g_pCameraHandler->StopAllCameras();
        network::g_pRoveCommUDPNode->CloseUDPSocket();
        network::g_pRoveCommTCPNode->CloseTCPSocket();

        // Delete dynamically allocated objects.
        delete globals::g_pStateMachineHandler;
        delete globals::g_pTagDetectionHandler;
        delete globals::g_pCameraHandler;
        delete globals::g_pWaypointHandler;
        delete network::g_pRoveCommUDPNode;
        delete network::g_pRoveCommTCPNode;

        // Set dangling pointers to null.
        globals::g_pStateMachineHandler = nullptr;
        globals::g_pTagDetectionHandler = nullptr;
        globals::g_pCameraHandler       = nullptr;
        globals::g_pWaypointHandler     = nullptr;
        network::g_pRoveCommUDPNode     = nullptr;
        network::g_pRoveCommTCPNode     = nullptr;
    }

    // Submit logger message that program is done cleaning up and is now exiting.
    LOG_INFO(logging::g_qSharedLogger, "Clean up finished. Exiting...");

    // Successful exit.
    return 0;
}
