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

#include "../states/ApproachingMarkerState.h"
#include "../states/ApproachingObjectState.h"
#include "../states/AvoidanceState.h"
#include "../states/IdleState.h"
#include "../states/NavigatingState.h"
#include "../states/ReversingState.h"
#include "../states/SearchPatternState.h"
#include "../states/StuckState.h"
#include "../states/VerifyingMarkerState.h"
#include "../states/VerifyingObjectState.h"

/// \cond
#include <RoveComm/RoveComm.h>
#include <RoveComm/RoveCommManifest.h>
#include <atomic>
#include <shared_mutex>

/// \endcond

/******************************************************************************
 * @brief The StateMachineHandler class serves as the main state machine for
 *        Autonomy Software. It will handle all state transitions and run the
 *        logic for each state.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
class StateMachineHandler : private AutonomyThread<void>
{
    private:
        /////////////////////////////////////////
        // Declare private class member variables.
        /////////////////////////////////////////
        std::shared_ptr<statemachine::State> m_pCurrentState;
        std::shared_ptr<statemachine::State> m_pPreviousState;
        std::unordered_map<statemachine::States, std::shared_ptr<statemachine::State>> m_umExitedStates;
        std::shared_mutex m_muStateMutex;
        std::shared_mutex m_muEventMutex;
        std::atomic_bool m_bSwitchingStates;

        /////////////////////////////////////////
        // Declare private class methods.
        /////////////////////////////////////////
        std::shared_ptr<statemachine::State> CreateState(statemachine::States eState);
        void ChangeState(statemachine::States eNextState, const bool bSaveCurrentState = false);
        void SaveCurrentState();
        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;

        /******************************************************************************
         * @brief Callback function used to trigger the start of autonomy. No matter what
         *      state we are in, signal a StartAutonomy Event.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-15
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<uint8_t>&, const sockaddr_in&)> AutonomyStartCallback =
            [this](const rovecomm::RoveCommPacket<uint8_t>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stPacket;
            (void) stdAddr;

            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Incoming Packet: Start Autonomy!");

            // Signal statemachine handler with Start event.
            this->HandleEvent(statemachine::Event::eStart);
        };

        /******************************************************************************
         * @brief Callback function used to trigger autonomy to stop. No matter what
         *      state we are in, signal an Abort Event.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-15
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<uint8_t>&, const sockaddr_in&)> AutonomyStopCallback =
            [this](const rovecomm::RoveCommPacket<uint8_t>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stPacket;
            (void) stdAddr;

            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Incoming Packet: Abort Autonomy!");

            // Signal statemachine handler with stop event.
            this->HandleEvent(statemachine::Event::eAbort, true);
        };

    public:
        /////////////////////////////////////////
        // Declare public class methods and variables.
        /////////////////////////////////////////
        StateMachineHandler();
        ~StateMachineHandler();

        void StartStateMachine();
        void StopStateMachine();

        void HandleEvent(statemachine::Event eEvent, const bool bSaveCurrentState = false);

        statemachine::States GetCurrentState() const;
        statemachine::States GetPreviousState() const;

        using AutonomyThread::GetIPS;
};

#endif    // STATEMACHINEHANDLER_H
