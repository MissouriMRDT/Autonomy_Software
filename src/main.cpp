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
#include "./util/states/TagDetectionChecker.hpp"

#include <sys/ioctl.h>
#include <termios.h>

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
// Store original terminal settings.
struct termios g_stOriginalTermSettings;

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
 * @brief Reset terminal mode to original settings.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-04
 ******************************************************************************/
void ResetTerminalMode()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &g_stOriginalTermSettings);
}

/******************************************************************************
 * @brief Mutator for the Non Canonical Terminal Mode private member.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-04
 ******************************************************************************/
void SetNonCanonicalTerminalMode()
{
    struct termios stNewTermSettings;

    tcgetattr(STDIN_FILENO, &g_stOriginalTermSettings);
    std::memcpy(&stNewTermSettings, &g_stOriginalTermSettings, sizeof(struct termios));

    stNewTermSettings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &stNewTermSettings);

    atexit(ResetTerminalMode);
}

/******************************************************************************
 * @brief Check if a key has been pressed in the terminal.
 *
 * @return int - Number of bytes waiting in the terminal buffer.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-04
 ******************************************************************************/
int CheckKeyPress()
{
    int nBytesWaiting;
    ioctl(STDIN_FILENO, FIONREAD, &nBytesWaiting);
    return nBytesWaiting;
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
    std::ifstream fHeaderText("../data/ASCII/v25.txt");
    std::string szHeaderText;
    if (fHeaderText.is_open())
    {
        std::ostringstream pHeaderText;
        pHeaderText << fHeaderText.rdbuf();
        szHeaderText = pHeaderText.str();
    }

    std::cout << szHeaderText << std::endl;
    std::cout << "Copyright \u00A9 2025 - Mars Rover Design Team\n" << std::endl;

    // Initialize Loggers
    logging::InitializeLoggers(constants::LOGGING_OUTPUT_PATH_ABSOLUTE);

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
                     network::g_bRoveCommUDPStatus.load(),
                     network::g_bRoveCommTCPStatus.load());

        // Since RoveComm is crucial, stop code.
        bMainStop = true;
    }
    else
    {
        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "RoveComm UDP and TCP nodes successfully initialized.");
    }
    // Initialize callbacks.
    network::g_pRoveCommUDPNode->AddUDPCallback<uint8_t>(logging::SetLoggingLevelsCallback, manifest::Autonomy::COMMANDS.find("SETLOGGINGLEVELS")->second.DATA_ID);

    // Initialize drivers.
    globals::g_pDriveBoard      = new DriveBoard();
    globals::g_pMultimediaBoard = new MultimediaBoard();
    globals::g_pNavigationBoard = new NavigationBoard();

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
        // Set the terminal to non-canonical mode. This allows us to read a single character from the terminal without waiting for a newline.
        SetNonCanonicalTerminalMode();

        // Print warnings if running in SIM mode.
        if (constants::MODE_SIM)
        {
            // Print 5 times to make it noticeable.
            for (int nIter = 0; nIter < 5; ++nIter)
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Autonomy_Software is running in SIM mode! If you aren't currently using the Unreal RoveSoSimulator sim, disable SIM mode in CMakeLists.txt "
                            "or in your build arguments!");
            }

            // Sleep for 3 seconds to make sure it's seen.
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }

        // Initialize handlers.
        globals::g_pCameraHandler       = new CameraHandler();
        globals::g_pWaypointHandler     = new WaypointHandler();
        globals::g_pTagDetectionHandler = new TagDetectionHandler();
        globals::g_pStateMachineHandler = new StateMachineHandler();

        // Start camera and detection handlers.
        globals::g_pCameraHandler->StartAllCameras();
        globals::g_pTagDetectionHandler->StartAllDetectors();
        // Enable Recording on Handlers.
        globals::g_pCameraHandler->StartRecording();
        globals::g_pTagDetectionHandler->StartRecording();

        /////////////////////////////////////////
        // Declare local variables used in main loop.
        /////////////////////////////////////////
        // Get Camera and Tag detector pointers .
        std::shared_ptr<ZEDCamera> pMainCam         = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam);
        std::shared_ptr<ZEDCamera> pLeftCam         = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eFrameLeftCam);
        std::shared_ptr<ZEDCamera> pRightCam        = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eFrameRightCam);
        std::shared_ptr<BasicCamera> pGroundCam     = globals::g_pCameraHandler->GetBasicCam(CameraHandler::BasicCamName::eHeadGroundCam);
        std::shared_ptr<TagDetector> pMainDetector  = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam);
        std::shared_ptr<TagDetector> pLeftDetector  = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eFrameLeftCam);
        std::shared_ptr<TagDetector> pRightDetector = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eFrameRightCam);
        IPS IterPerSecond                           = IPS();

        // Now that cameras and detectors are configured start state machine.
        globals::g_pStateMachineHandler->StartStateMachine();

        /*
            This while loop is the main periodic loop for the Autonomy_Software program.
            Loop until user sends sigkill or sigterm.
        */
        while (!bMainStop)
        {
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

            // Print out the FPS stats to the console if the user presses 'f' or 'F'.
            if (CheckKeyPress() > 0)
            {
                char chTerminalInput = 0;
                read(STDIN_FILENO, &chTerminalInput, 1);
                if (chTerminalInput == 'f' || chTerminalInput == 'F')
                {
                    LOG_NOTICE(logging::g_qSharedLogger, "{}", szMainInfo);
                }
                else if (chTerminalInput == 't' || chTerminalInput == 'T')
                {
                    // Create instance variables.
                    std::vector<tagdetectutils::ArucoTag> vMainCamTags;
                    std::vector<tagdetectutils::ArucoTag> vLeftCamTags;
                    std::vector<tagdetectutils::ArucoTag> vRightCamTags;
                    // Get the tags from the tag detectors.
                    std::future<bool> fuMainCamTags = pMainDetector->RequestDetectedArucoTags(vMainCamTags);
                    // std::future<bool> fuLeftCamTags  = pLeftDetector->RequestDetectedArucoTags(vLeftCamTags);
                    // std::future<bool> fuRightCamTags = pRightDetector->RequestDetectedArucoTags(vRightCamTags);
                    // Get the best/valid tags from the tag detectors.
                    tagdetectutils::ArucoTag stBestOpenCVTag, stBestTorchTag;
                    std::vector<std::shared_ptr<TagDetector>> vTagDetectors = {pMainDetector, pLeftDetector, pRightDetector};
                    // Check if the next waypoint in the waypoint handler exists and had a tag ID.
                    if (globals::g_pWaypointHandler->GetWaypointCount() > 0)
                    {
                        // Get the best tags from the tag detectors.
                        statemachine::IdentifyTargetMarker(vTagDetectors, stBestOpenCVTag, stBestTorchTag, globals::g_pWaypointHandler->PeekNextWaypoint().nID);
                    }
                    else
                    {
                        // Get the best tags from the tag detectors.
                        statemachine::IdentifyTargetMarker(vTagDetectors, stBestOpenCVTag, stBestTorchTag);
                    }

                    // Wait for all the tags to be copied.
                    if (fuMainCamTags.get())
                    {
                        // Submit logger message.
                        std::ostringstream ossTagsInfo;
                        ossTagsInfo << "\n--------[ All Detections ]--------\n"
                                    << "Detected Tags Info:\n"
                                    << "MainCam Tags: " << vMainCamTags.size() << "\n"
                                    << "LeftCam Tags: " << vLeftCamTags.size() << "\n"
                                    << "RightCam Tags: " << vRightCamTags.size() << "\n";

                        // Add a section for valid/best tags (example logic can be added here).

                        ossTagsInfo << "\n--------[ Valid/Best Tags ]--------\n";
                        if (stBestOpenCVTag.nID != -1)
                        {
                            ossTagsInfo << "Best OpenCV Tag ID: " << stBestOpenCVTag.nID << "\n";
                            ossTagsInfo << "Best OpenCV Tag Distance: " << stBestOpenCVTag.dStraightLineDistance << "\n";
                            ossTagsInfo << "Best OpenCV Tag Yaw Angle: " << stBestOpenCVTag.dYawAngle << "\n";
                        }
                        else
                        {
                            ossTagsInfo << "No valid OpenCV tags detected.\n";
                        }
                        if (stBestTorchTag.dConfidence != 0.0)
                        {
                            ossTagsInfo << "Best Torch Tag ID: " << stBestTorchTag.nID << "\n";
                            ossTagsInfo << "Best Torch Tag Distance: " << stBestTorchTag.dStraightLineDistance << "\n";
                            ossTagsInfo << "Best Torch Tag Yaw Angle: " << stBestTorchTag.dYawAngle << "\n";
                        }
                        else
                        {
                            ossTagsInfo << "No valid Torch tags detected.\n";
                        }

                        LOG_NOTICE(logging::g_qSharedLogger, "{}", ossTagsInfo.str());
                    }
                    else
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "Failed to get tags from cameras.");
                    }
                }
                else if (chTerminalInput == 'q' || chTerminalInput == 'Q')
                {
                    LOG_INFO(logging::g_qSharedLogger, "'Q' key pressed. Initiating shutdown...");
                    bMainStop = true;
                }
            }

            // Update IPS tick.
            IterPerSecond.Tick();

            // No need to loop as fast as possible. Sleep...
            std::this_thread::sleep_for(std::chrono::microseconds(66666));
        }

        /////////////////////////////////////////
        // Cleanup.
        /////////////////////////////////////////

        // Check if ZED spatial map was enabled.
        if (pMainCam->GetSpatialMappingState() == sl::SPATIAL_MAPPING_STATE::OK)
        {
            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Exporting ZED spatial map...");
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

        // Even though smart pointers should handle lifetime, explicitly reset to ensure cleanup in proper order, this also prevents the main thread
        // from exiting and killing quill loggers since they are used in some of the destructors.
        delete globals::g_pStateMachineHandler;
        delete globals::g_pTagDetectionHandler;
        delete globals::g_pCameraHandler;
        delete globals::g_pWaypointHandler;
        delete globals::g_pDriveBoard;
        delete globals::g_pMultimediaBoard;
        delete globals::g_pNavigationBoard;

        // Finally, stop RoveComm.
        LOG_INFO(logging::g_qSharedLogger, "Stopping RoveComm...");
        delete network::g_pRoveCommUDPNode;
        delete network::g_pRoveCommTCPNode;

        // Set all pointers to nullptr to prevent dangling pointers.
        globals::g_pStateMachineHandler = nullptr;
        globals::g_pTagDetectionHandler = nullptr;
        globals::g_pCameraHandler       = nullptr;
        globals::g_pWaypointHandler     = nullptr;
        globals::g_pDriveBoard          = nullptr;
        globals::g_pMultimediaBoard     = nullptr;
        globals::g_pNavigationBoard     = nullptr;
        network::g_pRoveCommUDPNode     = nullptr;
        network::g_pRoveCommTCPNode     = nullptr;
    }

    // Submit logger message that program is done cleaning up and is now exiting.
    LOG_INFO(logging::g_qSharedLogger, "Clean up finished. Exiting...");

    // Successful exit.
    return 0;
}
