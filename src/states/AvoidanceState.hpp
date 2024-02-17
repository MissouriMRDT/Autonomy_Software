/******************************************************************************
 * @brief Avoidance State Implementation for Autonomy State Machine.
 *
 * @file AvoidanceState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef AVOIDANCESTATE_HPP
#define AVOIDANCESTATE_HPP

#include "../algorithms/planners/AStar.h"
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
     * @brief The AvoidanceState class implements the Avoidance state for the Autonomy
     *        State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class AvoidanceState : public State
    {
        private:
            std::vector<double> m_vRoverXPath;
            std::vector<double> m_vRoverYPath;
            std::vector<double> m_vRoverYawPath;
            std::vector<double> m_vRoverXPosition;
            std::vector<double> m_vRoverYPosition;
            std::vector<double> m_vRoverYawPosition;
            std::vector<double> m_vRoverVelocity;
            double m_nLastIDX;
            double m_nTargetIDX;
            double m_nMaxDataPoints;
            time_t m_tPathStartTime;
            time_t m_tStuckCheckTime;
            double m_dStuckCheckLastPosition[2];
            // TODO: Add ASTAR Pathfinder
            // TODO: Add Stanley Controller
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
                LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Scheduling next run of state logic.");

                m_nMaxDataPoints = 100;

                m_vRoverXPath.reserve(m_nMaxDataPoints);
                m_vRoverYPath.reserve(m_nMaxDataPoints);
                m_vRoverYawPath.reserve(m_nMaxDataPoints);
                m_vRoverXPosition.reserve(m_nMaxDataPoints);
                m_vRoverYPosition.reserve(m_nMaxDataPoints);
                m_vRoverYawPosition.reserve(m_nMaxDataPoints);
                m_vRoverVelocity.reserve(m_nMaxDataPoints);

                m_nLastIDX                   = 0;
                m_nTargetIDX                 = 0;

                m_tPathStartTime             = time(nullptr);
                m_tStuckCheckTime            = time(nullptr);

                m_dStuckCheckLastPosition[0] = 0;
                m_dStuckCheckLastPosition[1] = 0;

                // TODO: Add ASTAR Pathfinder
                // TODO: Add Stanley Controller
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
                LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Exiting state.");

                m_nMaxDataPoints = 100;

                m_vRoverXPath.clear();
                m_vRoverYPath.clear();
                m_vRoverYawPath.clear();
                m_vRoverXPosition.clear();
                m_vRoverYPosition.clear();
                m_vRoverYawPosition.clear();
                m_vRoverVelocity.clear();

                m_nTargetIDX                 = 0;

                m_dStuckCheckLastPosition[0] = 0;
                m_dStuckCheckLastPosition[1] = 0;

                // TODO: Clear ASTAR Pathfinder
                // TODO: Clear Stanley Controller
            }

        public:
            /******************************************************************************
             * @brief Construct a new State object.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            AvoidanceState() : State(States::eAvoidance)
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
                // TODO: Implement the behavior specific to the Avoidance state
                LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Running state-specific behavior.");

                return States::eAvoidance;
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
                States eNextState       = States::eAvoidance;
                bool bCompleteStateExit = true;

                switch (eEvent)
                {
                    case Event::eStart:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Handling Start event.");
                        eNextState = States::eAvoidance;
                        break;
                    }
                    case Event::eAbort:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Handling Abort event.");
                        eNextState = States::eIdle;
                        break;
                    }
                    case Event::eEndObstacleAvoidance:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Handling EndObstacleAvoidance event.");
                        eNextState = States::NUM_STATES;    // Replace with `get_prev_state()`
                        break;
                    }
                    case Event::eStuck:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Handling Stuck event.");
                        eNextState = States::eStuck;
                        break;
                    }
                    default:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Handling unknown event.");
                        eNextState = States::eIdle;
                        break;
                    }
                }

                if (eNextState != States::eAvoidance)
                {
                    LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Transitioning to {} State.", StateToString(eNextState));

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

#endif    // AVOIDANCESTATE_HPP
