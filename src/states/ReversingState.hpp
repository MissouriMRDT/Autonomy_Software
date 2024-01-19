/******************************************************************************
 * @brief Reversing State Implementation for Autonomy State Machine.
 *
 * @file ReversingState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef REVERSINGSTATE_HPP
#define REVERSINGSTATE_HPP

#include "../interfaces/State.hpp"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The ReversingState class implements the Reversing state for the Autonomy
     *        State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class ReversingState : public State
    {
        private:
            // TODO: Add fetch of current position from GPS
            bool m_bInitialized;

        protected:
            /******************************************************************************
             * @brief This method is called when the state is first started. It is used to
             *        initialize the state.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            void Start() override
            {
                // Schedule the next run of the state's logic
                LOG_DEBUG(logging::g_qSharedLogger, "ReversingState: Scheduling next run of state logic.");

                // TODO: Get Starting Position from GPS
            }

            /******************************************************************************
             * @brief This method is called when the state is exited. It is used to clean up
             *        the state.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            void Exit() override
            {
                // Clean up the state before exiting
                LOG_DEBUG(logging::g_qSharedLogger, "ReversingState: Exiting state.");
            }

        public:
            /******************************************************************************
             * @brief Construct a new State object.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            ReversingState() : State(States::Reversing)
            {
                LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

                m_bInitialized = false;

                if (!m_bInitialized)
                {
                    Start();
                    m_bInitialized = true;
                }
            }

            /******************************************************************************
             * @brief Run the state machine. Returns the next state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            States Run() override
            {
                // TODO: Implement the behavior specific to the Reversing state
                LOG_DEBUG(logging::g_qSharedLogger, "ReversingState: Running state-specific behavior.");

                return States::Reversing;
            }

            /******************************************************************************
             * @brief Trigger an event in the state machine. Returns the next state.
             *
             * @param eEvent - The event to trigger.
             * @return std::shared_ptr<State> - The next state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            States TriggerEvent(Event eEvent) override
            {
                States eNextState       = States::Reversing;
                bool bCompleteStateExit = true;

                switch (eEvent)
                {
                    case Event::Start:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "ReversingState: Handling Start event.");
                        eNextState = States::Reversing;
                        break;
                    }
                    case Event::Abort:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "ReversingState: Handling Abort event.");
                        eNextState = States::Idle;
                        break;
                    }
                    case Event::ReverseComplete:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "ReversingState: Handling Reverse Complete event.");
                        eNextState = States::Idle;
                        break;
                    }
                    default:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "ReversingState: Handling unknown event.");
                        eNextState = States::Idle;
                        break;
                    }
                }

                if (eNextState != States::Reversing)
                {
                    LOG_DEBUG(logging::g_qSharedLogger, "ReversingState: Transitioning to {} State.", StateToString(eNextState));

                    // Exit the current state
                    if (bCompleteStateExit)
                    {
                        Exit();
                    }
                }

                return eNextState;
            }
    };
}    // namespace statemachine

#endif    // REVERSINGSTATE_HPP
