/******************************************************************************
 * @brief Defines the StateMachineHandler class.
 *
 * @file StateMachineHandler.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef STATEMACHINEHANDLER_H
#define STATEMACHINEHANDLER_H

#include "../AutonomyConstants.h"
#include "../AutonomyLogging.h"

#include "../interfaces/AutonomyThread.hpp"

#include "../states/ApproachingMarkerState.hpp"
#include "../states/ApproachingObjectState.hpp"
#include "../states/AvoidanceState.hpp"
#include "../states/IdleState.hpp"
#include "../states/NavigatingState.hpp"
#include "../states/ReversingState.hpp"
#include "../states/SearchPatternState.hpp"
#include "../states/StuckState.hpp"
#include "../states/VerifyingMarkerState.hpp"
#include "../states/VerifyingObjectState.hpp"

namespace statemachine
{
    class StateMachineHandler : AutonomyThread<void>
    {
        private:
            std::shared_ptr<State> pCurrentState;
            std::shared_ptr<State> pPreviousState;
            std::unordered_map<constants::States, std::shared_ptr<State>> umExitedStates;

            bool m_bInitialized;
            bool m_bSwitchingStates;
            bool m_bExiting;

            /******************************************************************************
             * @brief Create a State object based of of the State enum.
             *
             * @param eState - The State enum to create a State object from.
             * @return std::shared_ptr<State> - The State object created from the State enum.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            std::shared_ptr<State> CreateState(constants::States eState);

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
            void ChangeState(constants::States eNextState);

        public:
            /******************************************************************************
             * @brief Construct a new State Machine Handler object.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            StateMachineHandler();

            /******************************************************************************
             * @brief Destroy the State Machine Handler object.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            ~StateMachineHandler() = default;

            /******************************************************************************
             * @brief This method will start the state machine. It will set the first state
             *        to Idle and start the thread pool.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            void StartStateMachine();

            /******************************************************************************
             * @brief This code will run continuously in a separate thread. The State
             *        Machine Handler will check the current state and run the state's
             *        logic. It will then check the state's transition conditions and
             *        transition to the next state if the conditions are met.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            void ThreadedContinuousCode() override;

            /******************************************************************************
             * @brief This method holds the code that is ran in the thread pool started by
             *        the ThreadedLinearCode() method. It currently does nothing and is not
             *        needed in the current implementation of the StateMachineHandler.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            void PooledLinearCode() override;

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
            void HandleEvent(constants::Event eEvent);

            /******************************************************************************
             * @brief Accessor for the Current State private member.
             *
             * @return constants::States - The current state of the state machine.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            constants::States GetCurrentState() const;

            /******************************************************************************
             * @brief Accessor for the Previous State private member.
             *
             * @return constants::States - The previous state of the state machine.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            constants::States GetPreviousState() const;

            /******************************************************************************
             * @brief Save the current state to the map of exited states. This is used to
             *        store the state when the state machine is transitioning to a new
             *        state. And prevents the state from being deleted when the state
             *        machine transitions to a new state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            void SaveCurrentState();
    };
}    // namespace statemachine

#endif    // STATEMACHINEHANDLER_H
