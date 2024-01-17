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

#include "../interfaces/State.hpp"

namespace statemachine
{
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

            /******************************************************************************
             * @brief Accessor for the State private member. Returns the state as a string.
             *
             * @return std::string - The current state as a string.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            std::string ToString() const override { return "Avoidance"; }

        public:
            /******************************************************************************
             * @brief Construct a new State object.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            AvoidanceState() : State()
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
                // TODO: Implement the behavior specific to the Avoidance state
                LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Running state-specific behavior.");

                return constants::States::Avoidance;
            }

            /******************************************************************************
             * @brief Accessor for the State private member.
             *
             * @return constants::States - The current state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            constants::States GetState() const override { return constants::States::Avoidance; }

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
                constants::States eNextState = constants::States::Avoidance;
                bool bCompleteStateExit      = true;

                switch (eEvent)
                {
                    case constants::Event::Start:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Handling Start event.");
                        eNextState = constants::States::Avoidance;
                        break;
                    }
                    case constants::Event::Abort:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Handling Abort event.");
                        eNextState = constants::States::Idle;
                        break;
                    }
                    case constants::Event::EndObstacleAvoidance:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Handling EndObstacleAvoidance event.");
                        eNextState = constants::States::NUM_STATES;    // Replace with `get_prev_state()`
                        break;
                    }
                    case constants::Event::Stuck:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Handling Stuck event.");
                        eNextState = constants::States::Stuck;
                        break;
                    }
                    default:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Handling unknown event.");
                        eNextState = constants::States::Idle;
                        break;
                    }
                }

                if (eNextState != constants::States::Avoidance)
                {
                    LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Transitioning to {} State.", constants::StateToString(eNextState));

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
