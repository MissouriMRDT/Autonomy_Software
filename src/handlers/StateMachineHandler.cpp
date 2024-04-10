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

    // Set RoveComm Node callbacks.
    network::g_pRoveCommUDPNode->AddUDPCallback<uint8_t>(AutonomyStartCallback, manifest::Autonomy::COMMANDS.find("STARTAUTONOMY")->second.DATA_ID);
    network::g_pRoveCommUDPNode->AddUDPCallback<uint8_t>(AutonomyStopCallback, manifest::Autonomy::COMMANDS.find("DISABLEAUTONOMY")->second.DATA_ID);
    network::g_pRoveCommUDPNode->AddUDPCallback<float>(BMSCellVoltageCallback, manifest::BMS::TELEMETRY.find("CELLVOLTAGE")->second.DATA_ID);

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
