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
    // Register demand for the main camera's sensor data so the camera keeps retrieving and
    // publishing it for the heading-realignment logic below.
    m_rdMainCamSensors = m_pMainCam->GetSensorsReader();
    m_pRearCam           = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eRearCam);
    m_dZEDHeadingOffset  = 0.0;
    m_dLastRawZEDHeading = 0.0;
    m_dLastFusedHeading  = 0.0;
    m_bFirstHeadingLoop  = true;

    // State machine doesn't need to run at an unlimited speed. Cap main thread to a certain amount of iterations per second.
    this->SetMainThreadIPSLimit(constants::STATEMACHINE_MAX_IPS);
    // Name the OS thread so profilers and system tools can identify it.
    this->SetMainThreadName("StateMachine");
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
    // PRECONDITION: runs on the state machine thread only, either from StartStateMachine()
    // before that thread exists or from HandleEventOnOwningThread() during a drain. Nothing
    // else may touch m_pCurrentState, which is why there is no lock here any more and why
    // there is no longer an m_bSwitchingStates flag to get the timing of.

    // Check if we are already in this state.
    if (m_pCurrentState->GetState() != eNextState)
    {
        // Save the current state as the previous state, and publish it for foreign readers.
        // ClearSavedStates() also writes m_pPreviousState, and it is reachable from a RoveComm
        // thread, so this write takes the same lock. Nothing reads the pointer across threads
        // any more - GetPreviousState() reads the published enum - but two threads assigning
        // the same shared_ptr is still a race.
        {
            // Lock the saved-states/previous-state group while we reassign it.
            std::unique_lock lkSavedStatesLock(m_muSavedStatesMutex);
            m_pPreviousState = m_pCurrentState;
        }
        m_aePreviousState.store(m_pCurrentState->GetState(), std::memory_order_release);

        // Check if we should save this current state so it can be recalled in the future.
        if (bSaveCurrentState)
        {
            // Save the current state before transitioning
            SaveCurrentState();
        }

        // Look for a previously saved instance of the state we are entering. The saved-states
        // map is the one piece of this that a foreign thread can still reach, via
        // ClearSavedStates(), so it keeps its own lock.
        std::shared_ptr<statemachine::State> pRecalledState;
        {
            // Lock the saved states map while we search it.
            std::unique_lock lkSavedStatesLock(m_muSavedStatesMutex);
            std::unordered_map<statemachine::States, std::shared_ptr<statemachine::State>>::iterator itState = m_umSavedStates.find(eNextState);
            if (itState != m_umSavedStates.end())
            {
                // Take the saved instance and drop it from the map.
                pRecalledState = itState->second;
                m_umSavedStates.erase(itState);
            }
        }

        // Check whether we recalled a saved state or need a fresh one.
        if (pRecalledState != nullptr)
        {
            // Load the existing state
            m_pCurrentState = pRecalledState;

            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Recalling State: {}", m_pCurrentState->ToString());
        }
        else
        {
            // Create and enter a new state
            m_pCurrentState = CreateState(eNextState);
        }

        // Publish the new state for lock-free reads from every other thread. Release ordering
        // so a reader that sees this value also sees the fully constructed state behind it.
        m_aeCurrentState.store(m_pCurrentState->GetState(), std::memory_order_release);
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
    // Add state to map. ClearSavedStates() can be reached from a RoveComm thread, so the map
    // needs its own lock even though m_pCurrentState does not.
    std::unique_lock lkSavedStatesLock(m_muSavedStatesMutex);
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
    // Initialize the state machine with the initial state. Safe without a lock: the state
    // machine thread does not exist yet, so this thread is the only one that can touch it.
    m_pCurrentState = CreateState(statemachine::States::eIdle);
    m_aeCurrentState.store(m_pCurrentState->GetState(), std::memory_order_release);

    // Teach the command queue how to tell whether this handler's thread can still drain it,
    // so an event posted after the state machine has stopped is dropped rather than queued
    // forever.
    m_cmdQueue.SetDrainerLivenessCheck(
        [this]()
        {
            // Only a starting or running thread will reach DrainAll() again.
            const AutonomyThreadState eThreadState = this->GetThreadState();
            return eThreadState == AutonomyThreadState::eStarting || eThreadState == AutonomyThreadState::eRunning;
        });

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
    // No matter the current state, abort back to idle. Events are queued for the state
    // machine thread now, so wait for this one to actually be applied rather than racing
    // RequestStop() - otherwise the abort could be dropped and we would tear down from
    // whatever state we happened to be in.
    globals::g_pDriveBoard->SendStop();
    m_cmdQueue.PostAndWait<void>([this]() { this->HandleEventOnOwningThread(statemachine::Event::eAbort, false); }).wait();

    // Stop main thread.
    this->RequestStop();
    this->Join();

    // The thread is joined, so this thread is now the only one that can reach the queue.
    // Shut it down so any event posted during teardown is dropped instead of queued forever.
    m_cmdQueue.Shutdown();

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

    // Record which thread owns the state, so HandleEvent() can tell a state transitioning
    // itself (apply inline) from a foreign thread posting an event (queue it). Written every
    // iteration by the only thread that ever writes it; read by foreign threads.
    m_stStateMachineThreadId.store(std::this_thread::get_id(), std::memory_order_release);

    // 1. Control channel in. Every event posted by a foreign thread - a RoveComm command, a
    //    low battery abort, main() shutting us down - is applied here, on this thread. That
    //    is what makes m_pCurrentState single-owner: no other thread ever reassigns it, so
    //    running it below needs no lock and cannot race a transition mid-Run().
    m_cmdQueue.DrainAll();

    // 2. Run the current state. A transition can only have happened in step 1, so this
    //    pointer is stable for the whole call.
    if (m_pCurrentState != nullptr)
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

    // Stop the drive immediately, on the calling thread. This deliberately does NOT wait for
    // the state machine to drain: an abort has to cut power now, not one loop iteration from
    // now, and DriveBoard is safe to call from any thread.
    globals::g_pDriveBoard->SendStop();

    // If we ARE the state machine thread - a state calling HandleEvent() on itself - applying
    // the event inline is both correct and necessary. Posting would defer it to the next
    // iteration, and a state that transitions itself expects that to take effect immediately.
    if (this->GetThreadState() == AutonomyThreadState::eRunning && std::this_thread::get_id() == m_stStateMachineThreadId.load(std::memory_order_acquire))
    {
        // Apply directly; we already own the state.
        this->HandleEventOnOwningThread(eEvent, bSaveCurrentState);
        return;
    }

    // Otherwise hand the event to the owning thread. Foreign threads - RoveComm receive
    // threads, main() - never touch m_pCurrentState themselves; that is what removed the
    // data race on it. The event is applied at the top of the next state machine iteration,
    // at most one loop period (1/STATEMACHINE_MAX_IPS) away.
    m_cmdQueue.Post([this, eEvent, bSaveCurrentState]() { this->HandleEventOnOwningThread(eEvent, bSaveCurrentState); });
}

/******************************************************************************
 * @brief Apply one event to the current state and transition if it asks for it.
 *
 * @param eEvent - The Event enum to handle.
 * @param bSaveCurrentState - Whether or not to save the current state so it can be recalled
 *          next time it is triggered.
 *
 * @note PRECONDITION: runs on the state machine thread only, either from a CommandQueue
 *      drain or from a state transitioning itself. This is the only place m_pCurrentState is
 *      read for transition purposes, which is what makes the pointer safe to use unlocked.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-07
 ******************************************************************************/
void StateMachineHandler::HandleEventOnOwningThread(statemachine::Event eEvent, const bool bSaveCurrentState)
{
    ZoneScopedC(tracy::Color::Yellow);

    // Nothing to transition if the machine was never started or is already torn down.
    if (m_pCurrentState == nullptr)
    {
        // No state to hand the event to.
        return;
    }

    // Trigger the event on the current state
    statemachine::States eNextState = m_pCurrentState->TriggerEvent(eEvent);

    // Transition to the next state
    this->ChangeState(eNextState, bSaveCurrentState);
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
    std::unique_lock lkSavedStatesLock(m_muSavedStatesMutex);
    // Clear all saved states.
    m_umSavedStates.clear();
    // Reset previous state to Idle. Under the saved-states lock, which is also what the state
    // machine thread holds when it writes into the map during a transition.
    m_pPreviousState = std::make_shared<statemachine::IdleState>();
    m_aePreviousState.store(statemachine::States::eIdle, std::memory_order_release);
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
    std::unique_lock lkSavedStatesLock(m_muSavedStatesMutex);
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
    // Read the published enum, not m_pCurrentState. Every thread in the system calls this,
    // and dereferencing the shared_ptr from a foreign thread was a data race against the
    // state machine thread reassigning it during a transition.
    return m_aeCurrentState.load(std::memory_order_acquire);
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
    // Read the published enum, not m_pPreviousState. Dereferencing the shared_ptr from a
    // foreign thread raced the state machine thread reassigning it during a transition.
    return m_aePreviousState.load(std::memory_order_acquire);
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

        // Load the newest published sensor data from the ZED camera once into a local. Lock free
        // and non-blocking; null until the camera has published its first sensor snapshot.
        pubsub::SharedSnapshot<sl::SensorsData> pSensorSnapshot = m_rdMainCamSensors.Get();
        if (pSensorSnapshot != nullptr)
        {
            // Get Degrees heading from ZED IMU data.
            double dCurrentZEDHeading = pSensorSnapshot->tData.imu.pose.getEulerAngles(false).y;
            // Realign offset.
            // If driving forward fast enough (> Xm/s) and NOT turning. (angular vel near 0)
            if ((this->GetCurrentState() == statemachine::States::eIdle) ||
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
