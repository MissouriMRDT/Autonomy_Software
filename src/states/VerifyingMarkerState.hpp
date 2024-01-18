/******************************************************************************
 * @brief Verifying Marker State Implementation for Autonomy State Machine.
 *
 * @file VerifyingMarkerState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef VERIFYINGMARKERSTATE_HPP
#define VERIFYINGMARKERSTATE_HPP

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
     * @brief The VerifyingMarkerState class implements the Verifying Marker state for
     *        the Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class VerifyingMarkerState : public State
    {
        private:
            std::vector<int> m_vMarkerIDs;
            int m_nMaxMarkerIDs;
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
                LOG_DEBUG(logging::g_qSharedLogger, "VerifyingMarkerState: Scheduling next run of state logic.");

                m_nMaxMarkerIDs = 50;

                m_vMarkerIDs.reserve(m_nMaxMarkerIDs);
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
                LOG_DEBUG(logging::g_qSharedLogger, "VerifyingMarkerState: Exiting state.");

                m_vMarkerIDs.clear();
            }

        public:
            /******************************************************************************
             * @brief Construct a new State object.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            VerifyingMarkerState() : State("Verifying Marker")
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
                // TODO: Implement the behavior specific to the VerifyingMarker state
                LOG_DEBUG(logging::g_qSharedLogger, "VerifyingMarkerState: Running state-specific behavior.");

                return States::VerifyingMarker;
            }

            /******************************************************************************
             * @brief Accessor for the State private member.
             *
             * @return States - The current state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            States GetState() const override { return States::VerifyingMarker; }

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
                States eNextState       = States::VerifyingMarker;
                bool bCompleteStateExit = true;

                switch (eEvent)
                {
                    case Event::Start:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingMarkerState: Handling Start event.");
                        eNextState = States::VerifyingMarker;
                        break;
                    }
                    case Event::VerifyingComplete:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingMarkerState: Handling Verifying Complete event.");
                        eNextState = States::Idle;
                        break;
                    }
                    case Event::Abort:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingMarkerState: Handling Abort event.");
                        eNextState = States::Idle;
                        break;
                    }
                    default:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingMarkerState: Handling unknown event.");
                        eNextState = States::Idle;
                        break;
                    }
                }

                if (eNextState != States::VerifyingMarker)
                {
                    LOG_DEBUG(logging::g_qSharedLogger, "VerifyingMarkerState: Transitioning to {} State.", StateToString(eNextState));

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

#endif    // VERIFYINGMARKERSTATE_HPP
