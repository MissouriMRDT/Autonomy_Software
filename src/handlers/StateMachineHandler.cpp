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
    network::g_pRoveCommUDPNode->Subscribe(manifest::PMS::IP_ADDRESS, constants::ROVECOMM_OUTGOING_UDP_PORT);

    // Set RoveComm Node callbacks.
    using namespace manifest::Autonomy::Commands;
    network::g_pRoveCommUDPNode->On<STARTAUTONOMY>([this](const auto& stPacket) { AutonomyStartCallback(stPacket); });
    network::g_pRoveCommUDPNode->On<DISABLEAUTONOMY>([this](const auto& stPacket) { AutonomyStopCallback(stPacket); });
    network::g_pRoveCommUDPNode->On<CLEARWAYPOINTS>([this](const auto& stPacket) { ClearWaypointsCallback(stPacket); });
    network::g_pRoveCommUDPNode->On<manifest::PMS::Telemetry::CURRENTANDVOLTAGE>([this](const auto& stPacket) { PMSCellVoltageCallback(stPacket); });

    // Initialize member variables.
    m_pMainCam           = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam);
    m_pRearCam           = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eRearCam);
    m_dZEDHeadingOffset  = 0.0;
    m_dLastRawZEDHeading = 0.0;
    m_dLastFusedHeading  = 0.0;
    m_bFirstHeadingLoop  = true;

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
    ZoneScopedC(tracy::Color::Yellow);
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
    ZoneScopedC(tracy::Color::Yellow);
    // Acquire write lock for changing states.
    std::unique_lock lkStateProcessLock(m_muStateMutex);

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

    // Send current state to all subscribers.
    network::g_pRoveCommUDPNode->Send<manifest::Autonomy::Telemetry::CURRENTSTATE>({static_cast<uint8_t>(this->GetCurrentState())},
                                                                                   {0, 0, 0, 0},
                                                                                   constants::ROVECOMM_OUTGOING_UDP_PORT);
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
    ZoneScopedC(tracy::Color::Yellow);
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
    ZoneScopedC(tracy::Color::Yellow);
    // Initialize the state machine with the initial state
    m_pCurrentState    = CreateState(statemachine::States::eIdle);
    m_bSwitchingStates = false;

    // Clear any saved states.
    this->ClearSavedStates();

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
    ZoneScopedC(tracy::Color::Yellow);
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
    ZoneScopedC(tracy::Color::Yellow);
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
    ZoneScopedC(tracy::Color::Yellow);
    // Acquire write lock for handling events.
    std::unique_lock lkEventProcessLock(m_muEventMutex);

    // Stop the drive.
    globals::g_pDriveBoard->SendStop();

    // Check if the current state is not null and the state machine is running.
    if (m_pCurrentState != nullptr && this->GetThreadState() == AutonomyThreadState::eRunning)
    {
        // Trigger the event on the current state
        statemachine::States eNextState = m_pCurrentState->TriggerEvent(eEvent);

        // Transition to the next state
        ChangeState(eNextState, bSaveCurrentState);
    }
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
    ZoneScopedC(tracy::Color::Yellow);
    // Acquire write lock for clearing saved states.
    std::unique_lock lkStateProcessLock(m_muStateMutex);
    // Clear all saved states.
    m_umSavedStates.clear();
    // Reset previous state to nullptr;
    m_pPreviousState = std::make_shared<statemachine::IdleState>();
}

/******************************************************************************
 * @brief Clear a saved state based on the given state.
 *
 * @param eState - The state to clear from the saved states.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-06
 ******************************************************************************/
void StateMachineHandler::ClearSavedState(statemachine::States eState)
{
    ZoneScopedC(tracy::Color::Yellow);
    // Acquire write lock for clearing saved states.
    std::unique_lock lkStateProcessLock(m_muStateMutex);
    // Remove all states that match the given state.
    m_umSavedStates.erase(eState);
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
 * @brief This method is used to retrieve the rover's current position and heading. It uses the GPS data from the NavBoard and if enabled, it can also fuse the ZEDCam's
 * IMU heading data for a more accurate heading estimation. The method returns a RoverPose struct that contains the current GPS coordinate and the fused heading of the
 * rover.
 *
 * @param bIMUHeading - Whether to use IMU Heading.
 * @return geoops::RoverPose - The current position and heading (pose) of the rover stored in a RoverPose struct.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-06
 ******************************************************************************/
geoops::RoverPose StateMachineHandler::SmartRetrieveRoverPose(bool bIMUHeading)
{
    ZoneScopedC(tracy::Color::LightYellow);
    // Get and store the normal GPS position and heading from NavBoard.
    geoops::GPSCoordinate stCurrentGPSPosition = globals::g_pNavigationBoard->GetGPSData();
    double dCurrentGPSHeading                  = globals::g_pNavigationBoard->GetHeading();

    // Create instance variables.
    double dFusedHeading = dCurrentGPSHeading;

    if ((bIMUHeading) && !constants::MODE_SIM && m_pMainCam->GetCameraIsOpen())
    {
        // DYNAMIC REALIGNMENT. (Drift/Lag correction using GPS)
        double dVelocity   = this->SmartRetrieveVelocity();
        double dAngularVel = this->SmartRetrieveAngularVelocity();

        // Request the current heading from the ZED camera.
        sl::SensorsData slCurrentCameraSensorData;
        std::future<bool> fuResultStatus = m_pMainCam->RequestSensorsCopy(slCurrentCameraSensorData);
        // Wait for future to be fulfilled.
        if (fuResultStatus.get())
        {
            // Get Degrees heading from ZED IMU data.
            double dCurrentZEDHeading = slCurrentCameraSensorData.imu.pose.getEulerAngles(false).y;
            // Realign offset.
            // If driving forward fast enough (> Xm/s) and NOT turning. (angular vel near 0)
            if ((m_pCurrentState != nullptr && m_pCurrentState->GetState() == statemachine::States::eIdle) ||
                (std::abs(dVelocity) > constants::ZED_REALIGN_VEL_THRESH && std::abs(dAngularVel) < constants::ZED_REALIGN_ROT_THRESH))
            {
                this->RealignZEDHeading(dCurrentGPSHeading, dCurrentZEDHeading);
            }

            // Update fused heading.
            dFusedHeading = numops::InputAngleModulus(dCurrentZEDHeading + m_dZEDHeadingOffset, 0.0, 360.0);
        }
    }

    // Submit a debug print for the current rover pose.
    geoops::UTMCoordinate stCurrentUTMPosition = geoops::ConvertGPSToUTM(stCurrentGPSPosition);
    LOG_DEBUG(logging::g_qSharedLogger,
              "Rover Pose is currently: {} (easting), {} (northing), {} (alt), {} (degrees), IMUHeading = {}",
              stCurrentUTMPosition.dEasting,
              stCurrentUTMPosition.dNorthing,
              stCurrentUTMPosition.dAltitude,
              dFusedHeading,
              bIMUHeading ? "true" : "false");

    return geoops::RoverPose(stCurrentGPSPosition, dFusedHeading);
}

/******************************************************************************
 * @brief Retrieve the rover's current velocity. Currently there is no easy way
 *      to get the velocity of the ZEDCam so this method just returns the GPS-based
 *      velocity.
 *
 * @return double - The current velocity of the rover. (m/s)
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-06
 ******************************************************************************/
double StateMachineHandler::SmartRetrieveVelocity()
{
    ZoneScopedC(tracy::Color::LightYellow);
    // Return the GPS-based velocity from the NavBoard.
    return globals::g_pNavigationBoard->GetVelocity();
}

/******************************************************************************
 * @brief Retrieve the rover's current velocity. Currently there is no easy way
 *      to get the velocity of the ZEDCam so this method just returns the GPS-based
 *      velocity.
 *
 * @return double - The current angular velocity of the rover. (deg/s)
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-06
 ******************************************************************************/
double StateMachineHandler::SmartRetrieveAngularVelocity()
{
    ZoneScopedC(tracy::Color::LightYellow);
    // Return the GPS-based angular velocity from the NavBoard.
    return globals::g_pNavigationBoard->GetAngularVelocity();
}

/******************************************************************************
 * @brief This method is used to realign the ZED camera's heading with the actual heading of the rover.
 * This method calculates the difference between the ZED's heading and the actual heading of the rover
 * and applies that difference as an offset to the ZED's heading.
 *
 * @param dNewActualHeading - The new actual heading of the rover that the ZED's heading should be aligned to.
 * @param dCurrentZEDHeading - The current heading of the zed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-17
 ******************************************************************************/
void StateMachineHandler::RealignZEDHeading(const double dNewActualHeading, const double dCurrentZEDHeading)
{
    ZoneScopedC(tracy::Color::LightYellow);
    // Convert -180/180 to 0/360 Standard. (Keep 0 as 0)
    // If ZED is -90 (West), this makes it 270.
    double dCurrentHeading = dCurrentZEDHeading;
    if (dCurrentHeading < 0)
    {
        dCurrentHeading += 360.0;
    }

    // Calculate the difference required to turn Raw ZED into Actual Heading.
    // Logic: Actual = Raw + Offset  =>  Offset = Actual - Raw
    double dOffset = dNewActualHeading - dCurrentHeading;

    // Wrap the offset to 0-360 positive range
    dOffset = numops::InputAngleModulus(dOffset, 0.0, 360.0);

    // Normalize the signed offset to the range [-180, 180] so we report the smallest correction.
    double dSignedOffset = numops::AngularDifference(m_dZEDHeadingOffset, dOffset);
    // Only print a notice when the correction exceeds the configured significant threshold.
    if (std::abs(dSignedOffset) >= constants::ZED_REALIGN_ROT_THRESH)
    {
        LOG_NOTICE(logging::g_qSharedLogger,
                   "Significant ZED heading correction detected. Raw ZED: {} deg, Target GPS: {} deg, Signed Offset: {} deg, New Offset: {} deg",
                   dCurrentHeading,
                   dNewActualHeading,
                   dSignedOffset,
                   m_dZEDHeadingOffset);
    }

    // Update zed offset.
    m_dZEDHeadingOffset = dOffset;
}

/******************************************************************************
 * @brief Callback function used to trigger the start of autonomy. No matter what
 *      state we are in, signal a StartAutonomy Event.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-15
 ******************************************************************************/
void StateMachineHandler::AutonomyStartCallback(const rovecomm::RoveCommPacket<uint8_t>& stPacket)
{
    // Not using this.
    (void) stPacket;

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "Incoming Packet: Start Autonomy!");

    // Signal statemachine handler with Start event.
    this->HandleEvent(statemachine::Event::eStart);
}

/******************************************************************************
 * @brief Callback function used to trigger autonomy to stop. No matter what
 *      state we are in, signal an Abort Event.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-15
 ******************************************************************************/
void StateMachineHandler::AutonomyStopCallback(const rovecomm::RoveCommPacket<uint8_t>& stPacket)
{
    // Not using this.
    (void) stPacket;

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "Incoming Packet: Abort Autonomy!");

    // Signal statemachine handler with stop event.
    this->HandleEvent(statemachine::Event::eAbort, true);
}

/******************************************************************************
 * @brief Callback function that is called whenever RoveComm receives new CLEARWAYPOINTS packet.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-03
 ******************************************************************************/
void StateMachineHandler::ClearWaypointsCallback(const rovecomm::RoveCommPacket<uint8_t>& stPacket)
{
    // Not using this.
    (void) stPacket;

    /*
        The clear waypoints command will clear all waypoints in the WaypointHandler, but it also clears all saved states in the StateMachineHandler
        to prevent any conflicts when restarting autonomy with previously saved states that may have waypoints associated with them.
        However, we only want to delete our saved states if we are currently in IdleState.
    */

    // Check if the current state is IdleState.
    if (this->GetCurrentState() == statemachine::States::eIdle)
    {
        // Submit logger message.
        LOG_NOTICE(logging::g_qSharedLogger, "Incoming Clear Waypoints packet: Deleting all saved states in StateMachineHandler...");
        // Clear the saved states.
        this->ClearSavedStates();
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger,
                    "Incoming Clear Waypoints packet: Cannot clear saved states in StateMachineHandler unless in Idle state. Current state is {}.",
                    static_cast<int>(this->GetCurrentState()));
    }
}

/******************************************************************************
 * @brief Callback function used to force autonomy into Idle state if battery voltage gets too low.
 *      No matter what state we are in, signal an Abort Event.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-04
 ******************************************************************************/
void StateMachineHandler::PMSCellVoltageCallback(const rovecomm::RoveCommPacket<float>& stPacket)
{
    // Create instance variables.
    double dTotalCellVoltages   = 0.0;
    int nValidCellVoltageValues = 0;

    // Loop through voltage values and average all of the valid ones.
    for (int nIter = 6; nIter < stPacket.GetDataCount(); ++nIter)
    {
        // Check if the voltage values is greater than at least 0.1.
        if (stPacket.vData[nIter] >= 0.1)
        {
            // Add cell voltage value to total.
            dTotalCellVoltages += stPacket.vData[nIter];
            // Increment voltage voltage counter.
            ++nValidCellVoltageValues;
        }
    }
    // Calculate average cell voltage.
    double dAverageCellVoltage = dTotalCellVoltages / nValidCellVoltageValues;

    // Submit logger message.
    LOG_DEBUG(logging::g_qSharedLogger, "Incoming Packet: PMS Cell Voltages. Average voltage is: {}", dAverageCellVoltage);

    // Check if voltage is above the safe minimum for lithium ion batteries.
    if (constants::BATTERY_CHECKS_ENABLED && dAverageCellVoltage < constants::BATTERY_MINIMUM_CELL_VOLTAGE && this->GetCurrentState() != statemachine::States::eIdle)
    {
        // Submit logger message.
        LOG_CRITICAL(logging::g_qSharedLogger,
                     "Incoming PMS Packet: Average cell voltage is {} which is below the safe minimum of {}. Entering Idle state...",
                     dAverageCellVoltage,
                     constants::BATTERY_MINIMUM_CELL_VOLTAGE);

        // Signal statemachine handler with stop event.
        this->HandleEvent(statemachine::Event::eAbort, true);
    }
}
