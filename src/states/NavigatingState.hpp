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

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The NavigatingState class implements the Navigating state for the Autonomy
     *        State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
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

        public:
            /******************************************************************************
             * @brief Construct a new State object.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            NavigatingState() : State(States::eNavigating)
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
                // TODO: Implement the behavior specific to the Navigating state
                LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Running state-specific behavior.");

                return States::eNavigating;
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
                States eNextState       = States::eNavigating;
                bool bCompleteStateExit = true;

                switch (eEvent)
                {
                    case Event::eNoWaypoint:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling No Waypoint event.");
                        eNextState = States::eIdle;
                        break;
                    }
                    case Event::eReachedMarker:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Reached Marker event.");
                        eNextState = States::eIdle;
                        break;
                    }
                    case Event::eReachedGpsCoordinate:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Reached GPS Coordinate event.");

                        bool gpsOrTagMarker = false;    // TODO: Replace with determining if the rover is supposed to be navigating to a GPS coordinate or a tag / object.

                        if (gpsOrTagMarker)
                        {
                            eNextState = States::eIdle;
                        }
                        else
                        {
                            eNextState = States::eSearchPattern;
                        }

                        break;
                    }
                    case Event::eNewWaypoint:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling New Waypoint event.");
                        eNextState = States::eNavigating;
                        break;
                    }
                    case Event::eStart:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Start event.");
                        eNextState = States::eNavigating;
                        break;
                    }
                    case Event::eAbort:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Abort event.");
                        eNextState = States::eIdle;
                        break;
                    }
                    case Event::eObstacleAvoidance:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Obstacle Avoidance event.");
                        eNextState = States::eAvoidance;
                        break;
                    }
                    case Event::eReverse:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Reverse event.");
                        eNextState = States::eReversing;
                        break;
                    }
                    case Event::eStuck:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling Stuck event.");
                        eNextState = States::eStuck;
                        break;
                    }
                    default:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Handling unknown event.");
                        eNextState = States::eIdle;
                        break;
                    }
                }

                if (eNextState != States::eNavigating)
                {
                    LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Transitioning to {} State.", StateToString(eNextState));

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
