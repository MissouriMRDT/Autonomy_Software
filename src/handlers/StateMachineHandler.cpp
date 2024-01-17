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

namespace statemachine
{
    /******************************************************************************
     * @brief Create a State object based of of the State enum.
     *
     * @param eState - The State enum to create a State object from.
     * @return std::shared_ptr<State> - The State object created from the State enum.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    std::shared_ptr<State> StateMachineHandler::CreateState(constants::States eState)
    {
        switch (eState)
        {
            case constants::States::Idle: return std::make_shared<IdleState>();
            case constants::States::Navigating: return std::make_shared<NavigatingState>();
            case constants::States::SearchPattern: return std::make_shared<SearchPatternState>();
            case constants::States::ApproachingMarker: return std::make_shared<ApproachingMarkerState>();
            case constants::States::ApproachingObject: return std::make_shared<ApproachingObjectState>();
            case constants::States::VerifyingMarker: return std::make_shared<VerifyingMarkerState>();
            case constants::States::VerifyingObject: return std::make_shared<VerifyingObjectState>();
            case constants::States::Avoidance: return std::make_shared<AvoidanceState>();
            case constants::States::Reversing: return std::make_shared<ReversingState>();
            case constants::States::Stuck: return std::make_shared<StuckState>();
            default:
                // Handle the default case or throw an exception if needed
                LOG_ERROR(logging::g_qConsoleLogger, "State {} not found.", static_cast<int>(eState));
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
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void StateMachineHandler::ChangeState(constants::States eNextState)
    {
        m_bSwitchingStates = true;

        // Save the current state as the previous state
        pPreviousState = pCurrentState;

        // Save the current state before transitioning
        SaveCurrentState();

        // Check if the state exists in exitedStates
        auto it = umExitedStates.find(eNextState);
        if (it != umExitedStates.end())
        {
            // Load the existing state
            pCurrentState = it->second;
            LOG_INFO(logging::g_qConsoleLogger, "Recalling State: {}", pCurrentState->ToString());
        }
        else
        {
            // Create and enter a new state
            pCurrentState = CreateState(eNextState);
        }

        m_bSwitchingStates = false;
    }

    /******************************************************************************
     * @brief Construct a new State Machine Handler object.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    StateMachineHandler::StateMachineHandler()
    {
        LOG_INFO(logging::g_qConsoleLogger, "Initializing State Machine.");
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
        pCurrentState      = CreateState(constants::States::Idle);

        m_bInitialized     = true;
        m_bSwitchingStates = false;

        // Start the state machine thread
        Start();
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
        if (m_bInitialized && !m_bSwitchingStates && !m_bExiting)
        {
            // Run the current state
            pCurrentState->Run();
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
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void StateMachineHandler::HandleEvent(constants::Event eEvent)
    {
        // Trigger the event on the current state
        constants::States eNextState = pCurrentState->TriggerEvent(eEvent);

        // Transition to the next state
        ChangeState(eNextState);
    }

    /******************************************************************************
     * @brief Accessor for the Current State private member.
     *
     * @return constants::States - The current state of the state machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    constants::States StateMachineHandler::GetCurrentState() const
    {
        return pCurrentState->GetState();
    }

    /******************************************************************************
     * @brief Accessor for the Previous State private member.
     *
     * @return constants::States - The previous state of the state machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    constants::States StateMachineHandler::GetPreviousState() const
    {
        return pPreviousState ? pPreviousState->GetState() : constants::States::Idle;
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
        umExitedStates[pCurrentState->GetState()] = pCurrentState;
    }
}    // namespace statemachine
