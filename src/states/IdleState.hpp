/******************************************************************************
 * @brief Idle State Implementation for Autonomy State Machine.
 *
 * @file IdleState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef IDLESTATE_HPP
#define IDLESTATE_HPP

#include "../interfaces/State.hpp"

namespace statemachine
{
    class IdleState : public State
    {
        private:
            time_t m_tIdleTime;
            bool m_bRealigned;
            std::vector<double> m_vRoverXPosition;
            std::vector<double> m_vRoverYPosition;
            int m_nMaxDataPoints;
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
                LOG_DEBUG(logging::g_qSharedLogger, "IdleState: Scheduling next run of state logic.");

                m_tIdleTime      = time(nullptr);
                m_bRealigned     = false;
                m_nMaxDataPoints = 100;

                m_vRoverXPosition.reserve(m_nMaxDataPoints);
                m_vRoverYPosition.reserve(m_nMaxDataPoints);
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
                LOG_DEBUG(logging::g_qSharedLogger, "IdleState: Exiting state.");

                m_vRoverXPosition.clear();
                m_vRoverYPosition.clear();
            }

            /******************************************************************************
             * @brief Accessor for the State private member. Returns the state as a string.
             *
             * @return std::string - The current state as a string.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            std::string ToString() const override { return "Idle"; }

        public:
            /******************************************************************************
             * @brief Construct a new State object.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            IdleState() : State()
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
            constants::States Run() override
            {
                // TODO: Implement the behavior specific to the Idle state
                LOG_DEBUG(logging::g_qSharedLogger, "IdleState: Running state-specific behavior.");

                return constants::States::Idle;
            }

            /******************************************************************************
             * @brief Accessor for the State private member.
             *
             * @return constants::States - The current state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            constants::States GetState() const override { return constants::States::Idle; }

            /******************************************************************************
             * @brief Trigger an event in the state machine. Returns the next state.
             *
             * @param eEvent - The event to trigger.
             * @return std::shared_ptr<State> - The next state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            constants::States TriggerEvent(constants::Event eEvent) override
            {
                constants::States eNextState = constants::States::Idle;
                bool bCompleteStateExit      = true;

                switch (eEvent)
                {
                    case constants::Event::Start:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "IdleState: Handling Start event.");

                        bool tagInSight    = false;    // TODO: Replace with actual tag detection
                        bool reverseAlways = false;    // TODO: Replace with actual reverse always flag

                        // If there is an ArUco marker in the camera's field of view, transition to backup before navigating.
                        if (tagInSight)
                        {
                            LOG_DEBUG(logging::g_qSharedLogger, "IdleState: Detected ArUco marker. Transitioning to Reverse State.");
                            eNextState = constants::States::Reversing;
                        }
                        // If the reverse always flag is set, transition to backup before navigating.
                        else if (reverseAlways)
                        {
                            LOG_DEBUG(logging::g_qSharedLogger, "IdleState: Reverse always flag set. Transitioning to Reverse State.");
                            eNextState = constants::States::Reversing;
                        }
                        // Otherwise, transition to navigating.
                        else
                        {
                            LOG_DEBUG(logging::g_qSharedLogger, "IdleState: No ArUco marker detected. Transitioning to Navigating State.");
                            eNextState = constants::States::Navigating;
                        }

                        break;
                    }
                    case constants::Event::Abort:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "IdleState: Handling Abort event.");
                        eNextState = constants::States::Idle;
                        break;
                    }
                    default:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "IdleState: Handling unknown event.");
                        eNextState = constants::States::Idle;
                        break;
                    }
                }

                if (eNextState != constants::States::Idle)
                {
                    LOG_DEBUG(logging::g_qSharedLogger, "IdleState: Transitioning to {} State.", constants::StateToString(eNextState));

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

#endif    // IDLESTATE_HPP
