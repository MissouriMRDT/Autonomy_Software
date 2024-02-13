/******************************************************************************
 * @brief Stuck State Implementation for Autonomy State Machine.
 *
 * @file StuckState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef STUCKSTATE_HPP
#define STUCKSTATE_HPP

#include "../AutonomyGlobals.h"
#include "../algorithms/DifferentialDrive.hpp"
#include "../interfaces/State.hpp"
#include "../util/NumberOperations.hpp"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The StuckState class implements the Stuck state for the Autonomy
     *        State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class StuckState : public State
    {
        private:
            time_t m_tStuckCheckTime;
            bool m_bInitialized;

            unsigned int m_unAttempts;

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
                LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Scheduling next run of state logic.");

                m_tStuckCheckTime = time(nullptr);

                m_unAttempts      = 0;

                // TODO: Add Stop All Motors Command
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
                LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Exiting state.");
            }

        public:
            /******************************************************************************
             * @brief Construct a new State object.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            StuckState() : State(States::eStuck)
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
                // TODO: Implement the behavior specific to the Stuck state
                LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Running state-specific behavior.");

                switch (m_unAttempts)
                {
                    case 0:
                        // Reverse
                        ++m_unAttempts;
                        return States::eReversing;
                    case 1:
                        // rotate 30 degrees right and reverse
                        double dActualHeading = globals::g_pNavigationBoard->GetIMUData().dHeading;
                        dGoalHeading          = numops::InputAngleModulus(dHeading + 30, 0, 360);
                        AlignRover(dGoalHeading, 1, 0.5);
                        ++m_unAttempts;
                        return States::eReversing;
                    case 2:
                        // rotate 60 degrees left and reverse
                        double dActualHeading = globals::g_pNavigationBoard->GetIMUData().dHeading;
                        dGoalHeading          = numops::InputAngleModulus(dHeading - 60, 0, 360);
                        AlignRover(dGoalHeading, 1, 0.5);
                        ++m_unAttempts;
                        return States::eReversing;
                    case 3: m_unAttempts = 0; return States::eIdle;
                    default: return States::eIdle;
                }
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
                States eNextState       = States::eStuck;
                bool bCompleteStateExit = true;

                switch (eEvent)
                {
                    case Event::eStart:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Handling Start event.");
                        eNextState = States::eStuck;
                        break;
                    }
                    case Event::eAbort:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Handling Abort event.");
                        eNextState = States::eIdle;
                        break;
                    }
                    default:
                    {
                        LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Handling unknown event.");
                        eNextState = States::eIdle;
                        break;
                    }
                }

                if (eNextState != States::eStuck)
                {
                    LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Transitioning to {} State.", StateToString(eNextState));

                    // Exit the current state
                    if (bCompleteStateExit)
                    {
                        Exit();
                    }
                }

                return eNextState;
            }

            /******************************************************************************
             * @brief Rotate the rover until its aligned with the goal heading.
             *
             * @param dGoalHeading - Heading for the rover to align with.
             * @param dTolerance - How far can the rover be from the goal heading before completion.
             * @param dMotorPower - How much power to use in the motors [0,1];
             *
             * @author JSpencerPittman (jspencerpittman@gmail.com)
             * @date 2024-02-13
             ******************************************************************************/
            void AlignRover(const double dGoalHeading, const double dTolerance, const double dMotorPower)
            {
                double dCurrentHeading;
                double dYawAdjustment;

                do
                {
                    dCurrentHeading = globals::g_pNavigationBoard->GetIMUData().dHeading;
                    dYawAdjustment  = numops::InputAngleModulus(dGoalHeading - dCurrentHeading, -180, 180);

                    if (dYawAdjustment >= 0)
                        globals::g_pDriveBoard->SendDrive(dMotorPower, -dMotorPower);
                    else
                        globals::g_pDriveBoard->SendDrive(-dMotorPower, dMotorPower);

                } while (std::abs(dYawAdjustment) > dTolerance);

                globals::g_pDriveBoard->SendStop();
            }
    };
}    // namespace statemachine

#endif    // STUCKSTATE_HPP
