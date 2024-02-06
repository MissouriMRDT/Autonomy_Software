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

/******************************************************************************
 * @brief The StateMachineHandler class serves as the main state machine for
 *        Autonomy Software. It will handle all state transitions and run the
 *        logic for each state.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
class StateMachineHandler : public AutonomyThread<void>
{
    private:
        std::shared_ptr<statemachine::State> pCurrentState;
        std::shared_ptr<statemachine::State> pPreviousState;
        std::unordered_map<statemachine::States, std::shared_ptr<statemachine::State>> umExitedStates;

        bool m_bInitialized;
        bool m_bSwitchingStates;
        bool m_bExiting;

        std::shared_ptr<statemachine::State> CreateState(statemachine::States eState);
        void ChangeState(statemachine::States eNextState);

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;

    public:
        StateMachineHandler();
        ~StateMachineHandler() = default;

        void StartStateMachine();
        void StopStateMachine();

        void HandleEvent(statemachine::Event eEvent);

        statemachine::States GetCurrentState() const;
        statemachine::States GetPreviousState() const;

        void SaveCurrentState();
};

#endif    // STATEMACHINEHANDLER_H
