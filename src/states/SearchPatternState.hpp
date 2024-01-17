/******************************************************************************
 * @brief Search Pattern State Implementation for Autonomy State Machine.
 *
 * @file SearchPatternState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef SEARCHPATTERNSTATE_HPP
#define SEARCHPATTERNSTATE_HPP

#include "../interfaces/State.hpp"

namespace statemachine
{
    class SearchPatternState : public State
    {
        private:
            int m_nMaxDataPoints;
            std::vector<double> m_vRoverXPosition;
            std::vector<double> m_vRoverYPosition;
            time_t m_tStuckCheckTime;
            double m_dStuckCheckLastPosition[2];
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
                LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Scheduling next run of state logic.");

                m_nMaxDataPoints             = 100;
                m_tStuckCheckTime            = time(nullptr);

                m_dStuckCheckLastPosition[0] = 0;
                m_dStuckCheckLastPosition[1] = 0;

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
                LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Exiting state.");

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
            std::string ToString() const override { return "SearchPattern"; }

        public:
            /******************************************************************************
             * @brief Construct a new State object.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            SearchPatternState() : State()
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
                // TODO: Implement the behavior specific to the SearchPattern state
                LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Running state-specific behavior.");

                return constants::States::SearchPattern;
            }

            /******************************************************************************
             * @brief Accessor for the State private member.
             *
             * @return constants::States - The current state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            constants::States GetState() const override { return constants::States::SearchPattern; }

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
                constants::States eNextState = constants::States::SearchPattern;
                bool bCompleteStateExit      = true;

                switch (eEvent)
                {
                    case constants::Event::MarkerSeen:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling MarkerSeen event.");
                        eNextState = constants::States::ApproachingMarker;
                        break;
                    }
                    case constants::Event::ObjectSeen:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling ObjectSeen event.");
                        eNextState = constants::States::ApproachingObject;
                        break;
                    }
                    case constants::Event::Start:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling Start event.");
                        eNextState = constants::States::SearchPattern;
                        break;
                    }
                    case constants::Event::SearchFailed:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling SearchFailed event.");
                        eNextState = constants::States::Idle;
                        break;
                    }
                    case constants::Event::Abort:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling Abort event.");
                        eNextState = constants::States::Idle;
                        break;
                    }
                    case constants::Event::Stuck:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling Stuck event.");
                        eNextState = constants::States::Stuck;
                        break;
                    }
                    default:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling unknown event.");
                        eNextState = constants::States::Idle;
                        break;
                    }
                }

                if (eNextState != constants::States::SearchPattern)
                {
                    LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Transitioning to {} State.", constants::StateToString(eNextState));

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

#endif    // SEARCHPATTERNSTATE_HPP
