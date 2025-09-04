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
#include "./util/states/ObjectDetectionChecker.hpp"
#include "./util/states/TagDetectionChecker.hpp"

/// \cond
#include <sys/ioctl.h>
#include <termios.h>

/// \endcond

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
        globals::g_pCameraHandler            = new CameraHandler();
        globals::g_pWaypointHandler          = new WaypointHandler();
        globals::g_pTagDetectionHandler      = new TagDetectionHandler();
        globals::g_pObjectDetectionHandler   = new ObjectDetectionHandler();
        globals::g_pObstacleDetectionHandler = new ObstacleDetectionHandler();
        globals::g_pStateMachineHandler      = new StateMachineHandler();

        // Start camera and detection handlers.
        globals::g_pCameraHandler->StartAllCameras();
        globals::g_pTagDetectionHandler->StartAllDetectors();
        globals::g_pObjectDetectionHandler->StartAllDetectors();
        globals::g_pObstacleDetectionHandler->StartAllDetectors();
        // Enable Recording on Handlers.
        globals::g_pCameraHandler->StartRecording();
        globals::g_pTagDetectionHandler->StartRecording();
        globals::g_pObjectDetectionHandler->StartRecording();
        globals::g_pObstacleDetectionHandler->StartAllDetectors();

        /////////////////////////////////////////
        // Declare local variables used in main loop.
        /////////////////////////////////////////
        // Get Camera and Tag detector pointers .
        std::shared_ptr<ZEDCamera> pMainCam           = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam);
        std::shared_ptr<TagDetector> pMainTagDetector = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam);
        std::shared_ptr<ObjectDetector> pMainObjectDetector =
            globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eHeadMainCam);
        std::shared_ptr<ObstacleDetector> pMainObstacleDetector =
            g_pObstacleDetectionHandler->GetObstacleDetector(ObstacleDetectionHandler::ObstacleDetectors::eHeadMainCam);
        IPS IterPerSecond = IPS();

        // Now that cameras and detectors are configured start state machine.
        globals::g_pStateMachineHandler->StartStateMachine();

        // Create a vector of ints to store the FPS values for each thread.
        std::vector<uint32_t> vThreadFPSValues;

        /*
            This while loop is the main periodic loop for the Autonomy_Software program.
            Loop until user sends sigkill or sigterm.
        */
        while (!bMainStop)
        {
            // Add each threads FPS value to the vector.
            vThreadFPSValues.clear();
            vThreadFPSValues.push_back(static_cast<uint32_t>(IterPerSecond.GetExactIPS()));
            vThreadFPSValues.push_back(static_cast<uint32_t>(pMainCam->GetIPS().GetExactIPS()));
            vThreadFPSValues.push_back(static_cast<uint32_t>(pMainTagDetector->GetIPS().GetExactIPS()));
            vThreadFPSValues.push_back(static_cast<uint32_t>(pMainObjectDetector->GetIPS().GetExactIPS()));
            vThreadFPSValues.push_back(static_cast<uint32_t>(pMainObstacleDetector->GetIPS().GetExactIPS()));
            vThreadFPSValues.push_back(static_cast<uint32_t>(globals::g_pStateMachineHandler->GetIPS().GetExactIPS()));
            vThreadFPSValues.push_back(static_cast<uint32_t>(network::g_pRoveCommUDPNode->GetIPS().GetExactIPS()));
            vThreadFPSValues.push_back(static_cast<uint32_t>(network::g_pRoveCommTCPNode->GetIPS().GetExactIPS()));

            // Create a string to append FPS values to.
            std::string szMainInfo = "";
            // Get FPS of all cameras and detectors and construct the info into a string.
            szMainInfo += "\n--------[ Threads FPS ]--------\n";
            szMainInfo += "Main Process FPS: " + std::to_string(IterPerSecond.GetExactIPS()) + "\n";
            szMainInfo += "MainCam FPS: " + std::to_string(pMainCam->GetIPS().GetExactIPS()) + "\n";
            szMainInfo += "MainTagDetector FPS: " + std::to_string(pMainTagDetector->GetIPS().GetExactIPS()) + "\n";
            szMainInfo += "MainObjectDetector FPS: " + std::to_string(pMainObjectDetector->GetIPS().GetExactIPS()) + "\n";
            szMainInfo += "MainObstacleDetector FPS: " + std::to_string(pMainObstacleDetector->GetIPS().GetExactIPS()) + "\n";
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
                ssize_t nBytesRead   = read(STDIN_FILENO, &chTerminalInput, 1);
                if (nBytesRead <= 0)
                {
                    LOG_WARNING(logging::g_qSharedLogger, "Failed to read from terminal input.");
                }
                else
                {
                    if (chTerminalInput == 'h' || chTerminalInput == 'H')
                    {
                        // Print help message to console.
                        LOG_NOTICE(logging::g_qSharedLogger,
                                   "\n--------[ Autonomy Software Help ]--------\n"
                                   "Press 'f' or 'F' to print FPS stats to the log file.\n"
                                   "Press 'p' or 'P' to print rover pose info to the log file.\n"
                                   "Press 't' or 'T' to print tag detection info to the log file.\n"
                                   "Press 'm' or 'M' to print object detection info to the log file.\n"
                                   "Press 'q' or 'Q' to quit the program.\n"
                                   "-------------------------------------------\n");
                    }
                    else if (chTerminalInput == 'f' || chTerminalInput == 'F')
                    {
                        LOG_NOTICE(logging::g_qSharedLogger, "{}", szMainInfo);
                    }
                    else if (chTerminalInput == 'p' || chTerminalInput == 'P')
                    {
                        // Get the rover pose from the waypoint handler.
                        geoops::RoverPose stCurrentRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();
                        // Assemble a string to print containing data about the rover pose.
                        std::string szRoverPoseInfo = "\n--------[ Rover Pose Info ]--------\n";
                        szRoverPoseInfo += "Easting: " + std::to_string(stCurrentRoverPose.GetUTMCoordinate().dEasting) + "\n";
                        szRoverPoseInfo += "Northing: " + std::to_string(stCurrentRoverPose.GetUTMCoordinate().dNorthing) + "\n";
                        szRoverPoseInfo += "Altitude: " + std::to_string(stCurrentRoverPose.GetUTMCoordinate().dAltitude) + "\n";
                        szRoverPoseInfo += "Compass: " + std::to_string(stCurrentRoverPose.GetCompassHeading()) + "\n";
                        // Submit logger message.
                        LOG_NOTICE(logging::g_qSharedLogger, "{}", szRoverPoseInfo);
                    }
                    else if (chTerminalInput == 't' || chTerminalInput == 'T')
                    {
                        // Get the tags from the tag detectors.
                        if (pMainTagDetector->GetIsReady())
                        {
                            // Create instance variables.
                            tagdetectutils::ArucoTag stBestOpenCVTag, stBestTorchTag;
                            int nTagCount = 0;

                            // Get the best/valid tags from the tag detectors.
                            std::vector<std::shared_ptr<TagDetector>> vTagDetectors = {pMainTagDetector};
                            // Check if the next waypoint in the waypoint handler exists and had a tag ID.
                            if (globals::g_pWaypointHandler->GetWaypointCount() > 0)
                            {
                                // Get the best tags from the tag detectors.
                                nTagCount = statemachine::IdentifyTargetMarker(vTagDetectors,
                                                                               stBestOpenCVTag,
                                                                               stBestTorchTag,
                                                                               globals::g_pWaypointHandler->PeekNextWaypoint().nID);
                            }
                            else
                            {
                                // Get the best tags from the tag detectors.
                                nTagCount = statemachine::IdentifyTargetMarker(vTagDetectors, stBestOpenCVTag, stBestTorchTag);
                            }

                            // Submit logger message.
                            std::ostringstream ossTagsInfo;
                            ossTagsInfo << "\n--------[ All Detections ]--------\n"
                                        << "Detected Tags Info:\n"
                                        << "Total Tags: " << nTagCount << "\n";

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
                            LOG_WARNING(logging::g_qSharedLogger, "Tag Detector is not ready yet. Cannot get tags.");
                        }
                    }
                    else if (chTerminalInput == 'm' || chTerminalInput == 'M')
                    {
                        // Get the tags from the tag detectors.
                        if (pMainObjectDetector->GetIsReady())
                        {
                            // Create instance variables.
                            objectdetectutils::Object stBestTorchObject;
                            int nObjectCount = 0;

                            // Get the best/valid tags from the tag detectors.
                            std::vector<std::shared_ptr<ObjectDetector>> vTagDetectors = {pMainObjectDetector};
                            // Get the best tags from the tag detectors.
                            nObjectCount = statemachine::IdentifyTargetObject(vTagDetectors, stBestTorchObject);

                            // Submit logger message.
                            std::ostringstream ossTagsInfo;
                            ossTagsInfo << "\n--------[ All Detections ]--------\n"
                                        << "Detected Object Info:\n"
                                        << "Total Object: " << nObjectCount << "\n";

                            ossTagsInfo << "\n--------[ Valid/Best Objects ]--------\n";
                            if (stBestTorchObject.dConfidence != 0.0)
                            {
                                ossTagsInfo << "Best Torch Object Distance: " << stBestTorchObject.dStraightLineDistance << "\n";
                                ossTagsInfo << "Best Torch Object Yaw Angle: " << stBestTorchObject.dYawAngle << "\n";
                            }
                            else
                            {
                                ossTagsInfo << "No valid Torch objects detected.\n";
                            }

                            LOG_NOTICE(logging::g_qSharedLogger, "{}", ossTagsInfo.str());
                        }
                        else
                        {
                            // Submit logger message.
                            LOG_WARNING(logging::g_qSharedLogger, "Object Detector is not ready yet. Cannot get objects.");
                        }
                    }
                    else if (chTerminalInput == 'q' || chTerminalInput == 'Q')
                    {
                        LOG_INFO(logging::g_qSharedLogger, "'Q' key pressed. Initiating shutdown...");
                        bMainStop = true;
                    }
                }
            }

            /////////////////////////////////////////
            // Send thread stats over RoveComm.
            /////////////////////////////////////////
            // Check if rovecomm is initialized and running.
            if (network::g_pRoveCommUDPNode)
            {
                // Construct a RoveComm packet with the drive data.
                rovecomm::RoveCommPacket<uint32_t> stPacket;
                stPacket.unDataId    = manifest::Autonomy::TELEMETRY.find("THREADFPS")->second.DATA_ID;
                stPacket.unDataCount = manifest::Autonomy::TELEMETRY.find("THREADFPS")->second.DATA_COUNT;
                stPacket.eDataType   = manifest::Autonomy::TELEMETRY.find("THREADFPS")->second.DATA_TYPE;
                // Create a static variable to act a counter/iterator for the FPS value to use.
                static uint32_t nThreadFPSIndex = 0;
                // Check if the index is within bounds of the vector.
                if (nThreadFPSIndex < static_cast<uint32_t>(vThreadFPSValues.size()))
                {
                    // First push back the thread enum identifier cast to an int.
                    stPacket.vData.push_back(nThreadFPSIndex + 1);
                    // Add the current FPS value to the packet data.
                    stPacket.vData.push_back(static_cast<float>(vThreadFPSValues[nThreadFPSIndex]));
                    // Increment the index for the next iteration.
                    nThreadFPSIndex++;
                }
                else
                {
                    // Reset the index if it exceeds the vector size.
                    nThreadFPSIndex = 0;
                }
                // Send the packet over RoveComm UDP.
                network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, "0.0.0.0", constants::ROVECOMM_OUTGOING_UDP_PORT);
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

        // Stop handlers.
        globals::g_pStateMachineHandler->StopStateMachine();
        globals::g_pObjectDetectionHandler->StopAllDetectors();
        globals::g_pTagDetectionHandler->StopAllDetectors();
        globals::g_pObstacleDetectionHandler->StopAllDetectors();
        globals::g_pCameraHandler->StopAllCameras();

        // Cleanup handlers.
        delete globals::g_pStateMachineHandler;
        delete globals::g_pObjectDetectionHandler;
        delete globals::g_pObstacleDetectionHandler;
        delete globals::g_pTagDetectionHandler;
        delete globals::g_pCameraHandler;
        delete globals::g_pWaypointHandler;
        // Set all pointers to nullptr to prevent dangling pointers.
        globals::g_pStateMachineHandler      = nullptr;
        globals::g_pObjectDetectionHandler   = nullptr;
        globals::g_pObstacleDetectionHandler = nullptr;
        globals::g_pTagDetectionHandler      = nullptr;
        globals::g_pCameraHandler            = nullptr;
        globals::g_pWaypointHandler          = nullptr;
    }

    // Stop RoveComm quill logging or quill will segfault if trying to output logs to RoveComm.
    network::g_bRoveCommUDPStatus = false;
    network::g_bRoveCommTCPStatus = false;

    // Cleanup driver objects.
    delete globals::g_pDriveBoard;
    delete globals::g_pMultimediaBoard;
    delete globals::g_pNavigationBoard;

    // Finally, stop RoveComm.
    LOG_INFO(logging::g_qSharedLogger, "Stopping RoveComm...");
    network::g_pRoveCommUDPNode->CloseUDPSocket();
    network::g_pRoveCommTCPNode->CloseTCPSocket();
    delete network::g_pRoveCommUDPNode;
    delete network::g_pRoveCommTCPNode;

    // Set all pointers to nullptr to prevent dangling pointers.
    globals::g_pDriveBoard      = nullptr;
    globals::g_pMultimediaBoard = nullptr;
    globals::g_pNavigationBoard = nullptr;
    network::g_pRoveCommUDPNode = nullptr;
    network::g_pRoveCommTCPNode = nullptr;

    // Submit logger message that program is done cleaning up and is now exiting.
    LOG_INFO(logging::g_qSharedLogger, "Clean up finished. Exiting...");

    // Successful exit.
    return 0;
}
