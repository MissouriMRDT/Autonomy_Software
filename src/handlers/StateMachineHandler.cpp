/******************************************************************************
 * @brief Implements the StateMachineHandler class.
 *
 * @file StateMachineHandler.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "StateMachineHandler.h"
#include "../AutonomyConstants.h"
#include "../AutonomyGlobals.h"
#include "../AutonomyLogging.h"
#include "../AutonomyNetworking.h"

/******************************************************************************
 * @brief Construct a new State Machine Handler object.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
StateMachineHandler::StateMachineHandler()
{
    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "Initializing State Machine.");

    // Subscribe to PMS packets.
    rovecomm::RoveCommPacket<u_int8_t> stSubscribePacket;
    stSubscribePacket.unDataId    = manifest::System::SUBSCRIBE_DATA_ID;
    stSubscribePacket.unDataCount = 0;
    stSubscribePacket.eDataType   = manifest::DataTypes::UINT8_T;
    stSubscribePacket.vData       = std::vector<uint8_t>{};
    network::g_pRoveCommUDPNode->SendUDPPacket(stSubscribePacket, manifest::PMS::IP_ADDRESS.IP_STR.c_str(), constants::ROVECOMM_OUTGOING_UDP_PORT);

    // Set RoveComm Node callbacks.
    network::g_pRoveCommUDPNode->AddUDPCallback<uint8_t>(AutonomyStartCallback, manifest::Autonomy::COMMANDS.find("STARTAUTONOMY")->second.DATA_ID);
    network::g_pRoveCommUDPNode->AddUDPCallback<uint8_t>(AutonomyStopCallback, manifest::Autonomy::COMMANDS.find("DISABLEAUTONOMY")->second.DATA_ID);

    if (constants::BATTERY_CHECKS_ENABLED)
    {
        network::g_pRoveCommUDPNode->AddUDPCallback<float>(PMSCellVoltageCallback, manifest::PMS::TELEMETRY.find("CELLVOLTAGE")->second.DATA_ID);
    }

    // Initialize member variables.
    m_pMainCam = globals::g_pCameraHandler->GetZED(CameraHandler::eHeadMainCam);

    // State machine doesn't need to run at an unlimited speed. Cap main thread to a certain amount of iterations per second.
    this->SetMainThreadIPSLimit(constants::STATEMACHINE_MAX_IPS);
}

/******************************************************************************
 * @brief Destroy the State Machine Handler:: State Machine Handler object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-15
 ******************************************************************************/
StateMachineHandler::~StateMachineHandler()
{
    // Check if state machine is running.
    if (this->GetThreadState() == AutonomyThreadState::eRunning)
    {
        // Stop state machine.
        this->StopStateMachine();
    }
}

/******************************************************************************
 * @brief Create a State object based of of the State enum.
 *
 * @param eState - The State enum to create a State object from.
 * @return std::shared_ptr<State> - The State object created from the State enum.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
std::shared_ptr<statemachine::State> StateMachineHandler::CreateState(statemachine::States eState)
{
    switch (eState)
    {
        case statemachine::States::eIdle: return std::make_shared<statemachine::IdleState>();
        case statemachine::States::eNavigating: return std::make_shared<statemachine::NavigatingState>();
        case statemachine::States::eSearchPattern: return std::make_shared<statemachine::SearchPatternState>();
        case statemachine::States::eApproachingMarker: return std::make_shared<statemachine::ApproachingMarkerState>();
        case statemachine::States::eApproachingObject: return std::make_shared<statemachine::ApproachingObjectState>();
        case statemachine::States::eVerifyingPosition: return std::make_shared<statemachine::VerifyingPositionState>();
        case statemachine::States::eVerifyingMarker: return std::make_shared<statemachine::VerifyingMarkerState>();
        case statemachine::States::eVerifyingObject: return std::make_shared<statemachine::VerifyingObjectState>();
        case statemachine::States::eAvoidance: return std::make_shared<statemachine::AvoidanceState>();
        case statemachine::States::eReversing: return std::make_shared<statemachine::ReversingState>();
        case statemachine::States::eStuck: return std::make_shared<statemachine::StuckState>();
        default:
            // Handle the default case or throw an exception if needed
            LOG_ERROR(logging::g_qSharedLogger, "State {} not found.", static_cast<int>(eState));
    }

    return nullptr;
}

/******************************************************************************
 * @brief Transition to a new state. This function is called by the HandleEvent
 *        and checks to see if this state is already stored in the map of exited
 *        states. If it is, it loads the state from the map. If not, it creates
 *        a new state and stores it in the map.
 *
 * @param eNextState - The State enum to transition to.
 * @param bSaveCurrentState - Whether or not to save the current state so it can be recalled next time it is triggered.
 *          Default value is false.
 *
 * @author Eli Byrd (edbgkk@mst.edu), clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-17
 ******************************************************************************/
void StateMachineHandler::ChangeState(statemachine::States eNextState, const bool bSaveCurrentState)
{
    // Acquire write lock for changing states.
    std::unique_lock<std::shared_mutex> lkStateProcessLock(m_muStateMutex);

    // Check if we are already in this state.
    if (m_pCurrentState->GetState() != eNextState)
    {
        // Set atomic toggle saying we are in the process if switching states.
        m_bSwitchingStates = true;

        // Save the current state as the previous state
        m_pPreviousState = m_pCurrentState;

        // Check if we should save this current state so it can be recalled in the future.
        if (bSaveCurrentState)
        {
            // Save the current state before transitioning
            SaveCurrentState();
        }

        // Check if the state exists in exitedStates
        std::unordered_map<statemachine::States, std::shared_ptr<statemachine::State>>::iterator itState = m_umSavedStates.find(eNextState);
        if (itState != m_umSavedStates.end())
        {
            // Load the existing state
            m_pCurrentState = itState->second;
            // Remove new current state state from saved states.
            m_umSavedStates.erase(eNextState);

            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Recalling State: {}", m_pCurrentState->ToString());
        }
        else
        {
            // Create and enter a new state
            m_pCurrentState = CreateState(eNextState);
        }

        // Set atomic toggle saying we are done switching states.
        m_bSwitchingStates = false;
    }
}

/******************************************************************************
 * @brief Save the current state to the map of exited states. This is used to
 *        store the state when the state machine is transitioning to a new
 *        state. And prevents the state from being deleted when the state
 *        machine transitions to a new state.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
void StateMachineHandler::SaveCurrentState()
{
    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "Saving State: {}", m_pCurrentState->ToString());
    // Add state to map.
    m_umSavedStates[m_pCurrentState->GetState()] = m_pCurrentState;
}

/******************************************************************************
 * @brief This method will start the state machine. It will set the first state
 *        to Idle and start the thread pool.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
void StateMachineHandler::StartStateMachine()
{
    // Initialize the state machine with the initial state
    m_pCurrentState    = CreateState(statemachine::States::eIdle);
    m_bSwitchingStates = false;

    // Start the state machine thread
    Start();

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "Started State Machine.");
}

/******************************************************************************
 * @brief This method will stop the state machine. It will signal whatever state
 *  is currently running to abort back to idle and then stop the main code running
 *  in the ThreadedContinuousCode() method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-19
 ******************************************************************************/
void StateMachineHandler::StopStateMachine()
{
    // No matter the current state, abort back to idle.
    this->HandleEvent(statemachine::Event::eAbort);

    // Stop main thread.
    this->RequestStop();
    this->Join();

    // Send multimedia command to update state display.
    globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eOff);
    // Stop drive.
    globals::g_pDriveBoard->SendStop();

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "Stopped State Machine.");
}

/******************************************************************************
 * @brief This code will run continuously in a separate thread. The State
 *        Machine Handler will check the current state and run the state's
 *        logic. It will then check the state's transition conditions and
 *        transition to the next state if the conditions are met.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
void StateMachineHandler::ThreadedContinuousCode()
{
    /*
        Verify that the state machine has been initialized so that it doesn't
        try to run before a state has been initialized. Also verify that the
        state machine is not currently switching states. This prevents the
        state machine from running while it is in the middle of switching
        states. And verify that the state machine is not exiting. This prevents
        the state machine from running after it has been stopped.
    */
    if (!m_bSwitchingStates)
    {
        // Run the current state
        m_pCurrentState->Run();
    }

    // Create instance variable.
    static geoops::GPSCoordinate stNewGPSLocation;
    // Check if GPS data is recent and updated.
    if (std::chrono::duration_cast<std::chrono::milliseconds>(globals::g_pNavigationBoard->GetGPSLastUpdateTime()).count() <= 100 ||
        (stNewGPSLocation.dLatitude == 0.0 && stNewGPSLocation.dLongitude == 0.0))
    {
        // Get the current NavBoard GPS data.
        stNewGPSLocation = globals::g_pNavigationBoard->GetGPSData();
    }

    // Create static boolean value for toggling DiffGPS warning log print.
    static bool bAlreadyPrintedDiffGPSWarning = false;
    // Check if the current GPS data is different from the old.
    if (stNewGPSLocation.dLatitude != m_stCurrentGPSLocation.dLatitude && stNewGPSLocation.dLongitude != m_stCurrentGPSLocation.dLongitude &&
        stNewGPSLocation.dAltitude != m_stCurrentGPSLocation.dAltitude)
    {
        // Check GNSS Fusion is enabled and the main ZED camera is a fusion master.
        if (constants::FUSION_ENABLE_GNSS_FUSION && stNewGPSLocation.bIsDifferential)
        {
            // Check if main ZED camera is setup to use GPS fusion.
            if (m_pMainCam->GetIsFusionMaster() && m_pMainCam->GetPositionalTrackingEnabled())
            {
                // Update current GPS position.
                m_stCurrentGPSLocation = stNewGPSLocation;
                // Feed current GPS location to main ZED camera.
                m_pMainCam->IngestGPSDataToFusion(m_stCurrentGPSLocation);
            }

            // Reset DiffGPS warning print toggle.
            if (bAlreadyPrintedDiffGPSWarning)
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Incoming GPS position to NavBoard now has differential accuracy! Autonomy will switch to using GPS Fusion for high accuracy navigation!");

                // Rest toggle.
                bAlreadyPrintedDiffGPSWarning = false;
            }
        }
        // Check if GPS coordinate from NavBoard is not differential and print warning log.
        else if (!bAlreadyPrintedDiffGPSWarning)
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger,
                        "Incoming GPS position to NavBoard does not have differential accuracy! Autonomy will not use GPS Fusion but instead fallback to aligning "
                        "the ZED pose while the rover is in Idle state and not moving. Autonomous navigation performance of the rover will be degraded...");

            // Set already printed toggle.
            bAlreadyPrintedDiffGPSWarning = true;
        }

        // Realign the camera's relative position to current GPS position when in Idle. This does not affect fusion, but makes sure we can fallback to the camera pose for
        // positioning.
        if (m_pCurrentState->GetState() == statemachine::States::eIdle && m_pMainCam->GetPositionalTrackingEnabled())
        {
            // Check if the rover is currently not driving of turning. Use only GPS based and use stuck state parameters for checking.
            if (globals::g_pNavigationBoard->GetVelocity() <= constants::STUCK_CHECK_VEL_THRESH &&
                globals::g_pNavigationBoard->GetAngularVelocity() <= constants::STUCK_CHECK_ROT_THRESH)
            {
                // Update current GPS position.
                m_stCurrentGPSLocation = stNewGPSLocation;
                // Get current compass heading.
                double dCurrentCompassHeading = globals::g_pNavigationBoard->GetHeading();
                // Realign the main ZED cameras pose with current GPS-based position and heading.
                this->RealignZEDPosition(CameraHandler::eHeadMainCam, geoops::ConvertGPSToUTM(m_stCurrentGPSLocation), dCurrentCompassHeading);
            }
        }
    }
}

/******************************************************************************
 * @brief This method holds the code that is ran in the thread pool started by
 *        the ThreadedLinearCode() method. It currently does nothing and is not
 *        needed in the current implementation of the StateMachineHandler.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
void StateMachineHandler::PooledLinearCode() {}

/******************************************************************************
 * @brief This method Handles Events that are passed to the State Machine
 *        Handler. It will check the current state and run the state's
 *        HandleEvent() method. It will then check the state's transition
 *        conditions and transition to the next state if the conditions are
 *        met.
 *
 * @param eEvent - The Event enum to handle.
 * @param bSaveCurrentState - Whether or not to save the current state so it can be recalled next time it is triggered.
 *          Default value is false.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
void StateMachineHandler::HandleEvent(statemachine::Event eEvent, const bool bSaveCurrentState)
{
    // Acquire write lock for handling events.
    std::unique_lock<std::shared_mutex> lkEventProcessLock(m_muEventMutex);

    // Trigger the event on the current state
    statemachine::States eNextState = m_pCurrentState->TriggerEvent(eEvent);

    // Transition to the next state
    ChangeState(eNextState, bSaveCurrentState);
}

/******************************************************************************
 * @brief Clear all saved states.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-01
 ******************************************************************************/
void StateMachineHandler::ClearSavedStates()
{
    // Acquire write lock for clearing saved states.
    std::unique_lock<std::shared_mutex> lkStateProcessLock(m_muStateMutex);
    // Clear all saved states.
    m_umSavedStates.clear();
    // Reset previous state to nullptr;
    m_pPreviousState = std::make_shared<statemachine::IdleState>();
}

/******************************************************************************
 * @brief Accessor for the Current State private member.
 *
 * @return States - The current state of the state machine.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
statemachine::States StateMachineHandler::GetCurrentState() const
{
    return m_pCurrentState->GetState();
}

/******************************************************************************
 * @brief Accessor for the Previous State private member.
 *
 * @return States - The previous state of the state machine.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
statemachine::States StateMachineHandler::GetPreviousState() const
{
    // Check if the previous state exists and return it if it does otherwise return Idle
    return m_pPreviousState ? m_pPreviousState->GetState() : statemachine::States::eIdle;
}

/******************************************************************************
 * @brief This is used to realign the ZEDs forward direction with the rover's
 *      current compass heading. Realigning GPS latlon is not necessary since
 *      we're using the ZEDSDK's Fusion module. The heading of the camera's
 *      GeoPose is actually automatically realigned too depending on what direction
 *      We are headed, but this is just to be safe.
 *
 * @param eCameraName - The camera name represented as an enum from the CameraHandler class.
 * @param stNewCameraPosition - The new UTM position of the ZED camera.
 * @param dNewCameraHeading - The new compass heading of the ZED camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-17
 ******************************************************************************/
void StateMachineHandler::RealignZEDPosition(CameraHandler::ZEDCamName eCameraName, const geoops::UTMCoordinate& stNewCameraPosition, const double dNewCameraHeading)
{
    // Get main ZEDCam.
    ZEDCam* pMainCam = globals::g_pCameraHandler->GetZED(eCameraName);

    // Check if main ZEDCam is opened and positional tracking is enabled.
    if (pMainCam->GetCameraIsOpen() && pMainCam->GetPositionalTrackingEnabled())
    {
        // Request for the cameras current pose.
        ZEDCam::Pose stCurrentCameraPose;
        std::future<bool> fuPoseReturnStatus = pMainCam->RequestPositionalPoseCopy(stCurrentCameraPose);
        // Wait for pose to be copied.
        if (fuPoseReturnStatus.get())
        {
            // Update camera Y heading with GPSs current heading.
            pMainCam->SetPositionalPose(stNewCameraPosition.dEasting,
                                        stNewCameraPosition.dAltitude,
                                        stNewCameraPosition.dNorthing,
                                        stCurrentCameraPose.stEulerAngles.dXO,
                                        dNewCameraHeading,
                                        stCurrentCameraPose.stEulerAngles.dZO);

            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Realigned ZED stereo camera to current GPS position.");
        }
        else
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "Failed to realign the ZEDCam's pose with the given UTM position and compass heading.");
        }
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Failed to realign the ZEDCam's pose with the given UTM position and compass heading. The camera is not open yet or positional tracking status is "
                  "suboptimal!");
    }
}
