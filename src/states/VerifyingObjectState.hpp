/******************************************************************************
 * @brief Verifying Object State Implementation for Autonomy State Machine.
 *
 * @file VerifyingObjectState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef VERIFYINGOBJECTSTATE_HPP
#define VERIFYINGOBJECTSTATE_HPP

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
     * @brief The VerifyingObjectState class implements the Verifying Object state for
     *        the Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class VerifyingObjectState : public State
    {
        private:
            std::vector<int> m_vObjectIDs;
            int m_nMaxObjectIDs;
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
                LOG_DEBUG(logging::g_qSharedLogger, "VerifyingObjectState: Scheduling next run of state logic.");

                m_nMaxObjectIDs = 50;

                m_vObjectIDs.reserve(m_nMaxObjectIDs);
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
                LOG_DEBUG(logging::g_qSharedLogger, "VerifyingObjectState: Exiting state.");

                m_vObjectIDs.clear();
            }

        public:
            /******************************************************************************
             * @brief Accessor for the State private member. Returns the state as a string.
             *
             * @return std::string - The current state as a string.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            VerifyingObjectState() : State("Verifying Object")
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
                // TODO: Implement the behavior specific to the VerifyingObject state
                LOG_DEBUG(logging::g_qSharedLogger, "VerifyingObjectState: Running state-specific behavior.");

                return States::VerifyingObject;
            }

            /******************************************************************************
             * @brief Accessor for the State private member.
             *
             * @return States - The current state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            States GetState() const override { return States::VerifyingObject; }

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
                States eNextState       = States::VerifyingObject;
                bool bCompleteStateExit = true;

                switch (eEvent)
                {
                    case Event::Start:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingObjectState: Handling Start event.");
                        eNextState = States::VerifyingObject;
                        break;
                    }
                    case Event::VerifyingComplete:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingObjectState: Handling Verifying Complete event.");
                        eNextState = States::Idle;
                        break;
                    }
                    case Event::Abort:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingObjectState: Handling Abort event.");
                        eNextState = States::Idle;
                        break;
                    }
                    default:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingObjectState: Handling unknown event.");
                        eNextState = States::Idle;
                        break;
                    }
                }

                if (eNextState != States::VerifyingObject)
                {
                    LOG_DEBUG(logging::g_qSharedLogger, "VerifyingObjectState: Transitioning to {} State.", StateToString(eNextState));

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

#endif    // VERIFYINGOBJECTSTATE_HPP
