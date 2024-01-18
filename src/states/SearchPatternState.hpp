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

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The SearchPatternState class implements the Search Pattern state for the
     *        Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
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

        public:
            /******************************************************************************
             * @brief Construct a new State object.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            SearchPatternState() : State("Search Pattern")
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
                // TODO: Implement the behavior specific to the SearchPattern state
                LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Running state-specific behavior.");

                return States::SearchPattern;
            }

            /******************************************************************************
             * @brief Accessor for the State private member.
             *
             * @return States - The current state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            States GetState() const override { return States::SearchPattern; }

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
                States eNextState       = States::SearchPattern;
                bool bCompleteStateExit = true;

                switch (eEvent)
                {
                    case Event::MarkerSeen:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling MarkerSeen event.");
                        eNextState = States::ApproachingMarker;
                        break;
                    }
                    case Event::ObjectSeen:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling ObjectSeen event.");
                        eNextState = States::ApproachingObject;
                        break;
                    }
                    case Event::Start:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling Start event.");
                        eNextState = States::SearchPattern;
                        break;
                    }
                    case Event::SearchFailed:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling SearchFailed event.");
                        eNextState = States::Idle;
                        break;
                    }
                    case Event::Abort:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling Abort event.");
                        eNextState = States::Idle;
                        break;
                    }
                    case Event::Stuck:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling Stuck event.");
                        eNextState = States::Stuck;
                        break;
                    }
                    default:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling unknown event.");
                        eNextState = States::Idle;
                        break;
                    }
                }

                if (eNextState != States::SearchPattern)
                {
                    LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Transitioning to {} State.", StateToString(eNextState));

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
