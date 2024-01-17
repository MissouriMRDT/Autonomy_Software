/******************************************************************************
 * @brief Navigating State Implementation for Autonomy State Machine.
 *
 * @file NavigatingState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef NAVIGATINGSTATE_HPP
#define NAVIGATINGSTATE_HPP

#include "../interfaces/State.hpp"

namespace statemachine
{
    class NavigatingState : public State
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
                LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Scheduling next run of state logic.");

                m_nMaxDataPoints             = 100;
                m_tStuckCheckTime            = time(nullptr);

                m_dStuckCheckLastPosition[0] = 0;
                m_dStuckCheckLastPosition[1] = 0;

                m_vRoverXPosition.reserve(m_nMaxDataPoints);
                m_vRoverYPosition.reserve(m_nMaxDataPoints);

                // TODO: Add a Clear ArUco Tags Command
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
                LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Exiting state.");

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
            std::string ToString() const override { return "Navigating"; }

        public:
            /******************************************************************************
             * @brief Construct a new State object.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            NavigatingState() : State()
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
                // TODO: Implement the behavior specific to the Navigating state
                LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Running state-specific behavior.");

                return constants::States::Navigating;
            }

            /******************************************************************************
             * @brief Accessor for the State private member.
             *
             * @return constants::States - The current state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            constants::States GetState() const override { return constants::States::Navigating; }

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
                constants::States eNextState = constants::States::Navigating;
                bool bCompleteStateExit      = true;

                switch (eEvent)
                {
                    case constants::Event::NoWaypoint:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling No Waypoint event.");
                        eNextState = constants::States::Idle;
                        break;
                    }
                    case constants::Event::ReachedMarker:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Reached Marker event.");
                        eNextState = constants::States::Idle;
                        break;
                    }
                    case constants::Event::ReachedGpsCoordinate:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Reached GPS Coordinate event.");

                        bool gpsOrTagMarker = false;    // TODO: Replace with determining if the rover is supposed to be navigating to a GPS coordinate or a tag / object.

                        if (gpsOrTagMarker)
                        {
                            eNextState = constants::States::Idle;
                        }
                        else
                        {
                            eNextState = constants::States::SearchPattern;
                        }

                        break;
                    }
                    case constants::Event::NewWaypoint:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling New Waypoint event.");
                        eNextState = constants::States::Navigating;
                        break;
                    }
                    case constants::Event::Start:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Start event.");
                        eNextState = constants::States::Navigating;
                        break;
                    }
                    case constants::Event::Abort:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Abort event.");
                        eNextState = constants::States::Idle;
                        break;
                    }
                    case constants::Event::ObstacleAvoidance:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Obstacle Avoidance event.");
                        eNextState = constants::States::Avoidance;
                        break;
                    }
                    case constants::Event::Reverse:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Reverse event.");
                        eNextState = constants::States::Reversing;
                        break;
                    }
                    case constants::Event::Stuck:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Stuck event.");
                        eNextState = constants::States::Stuck;
                        break;
                    }
                    default:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling unknown event.");
                        eNextState = constants::States::Idle;
                        break;
                    }
                }

                if (eNextState != constants::States::Navigating)
                {
                    LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Transitioning to {} State.", constants::StateToString(eNextState));

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

#endif    // NAVIGATINGSTATE_HPP
