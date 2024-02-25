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
#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"
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
            geoops::GPSCoordinate m_stOriginalPosition;
            double m_dOriginalHeading;

            double m_dHeadingTolerance;
            double m_dInplaceRotationMotorPower;
            double m_dStillStuckThreshold;

        protected:
            /******************************************************************************
             * @brief This method is called when the state is first started. It is used to
             *        initialize the state.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu), Jason Pittman (jspencerpittman@gmail.com)
             * @date 2024-01-17
             ******************************************************************************/
            void Start() override
            {
                // Schedule the next run of the state's logic
                LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Scheduling next run of state logic.");

                m_tStuckCheckTime      = time(nullptr);

                m_bIsCurrentlyAligning = false;

                globals::g_pDriveBoard->SendStop();
            }

            /******************************************************************************
             * @brief This method is called when the state is exited. It is used to clean up
             *        the state.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu), Jason Pittman (jspencerpittman@gmail.com)
             * @date 2024-01-17
             ******************************************************************************/
            void Exit() override
            {
                // Clean up the state before exiting
                LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Exiting state.");

                m_bIsCurrentlyAligning = false;
            }

        public:
            /******************************************************************************
             * @brief Construct a new State object.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu), Jason Pittman (jspencerpittman@gmail.com)
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

                m_unAttempts                 = 1;                              // Current attempt we are on for a given position.
                m_stOriginalPosition         = geoops::GPSCoordinate(0, 0);    // Original position where rover was reported stuck.
                m_dOriginalHeading           = 0;                              // Original heading the rover was at when reported stuck.

                m_dHeadingTolerance          = 1.0;                            // How close the current heading must be to the target heading to be considered aligned.
                m_dInplaceRotationMotorPower = 0.5;                            // Power on left and right motors when rotating the rotor.
                m_dSamePositionThreshold     = 1.0;                            // Distance threshold determining if we are still in the same position.
            }

            /******************************************************************************
             * @brief Run the state machine. Returns the next state.
             *
             * @author Eli Byrd (edbgkk@mst.edu), Jason Pittman (jspencerpittman@gmail.com)
             * @date 2024-01-17
             ******************************************************************************/
            States Run() override
            {
                LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Running state-specific behavior.");

                // If we aren't currently attempting to become 'unstuck',
                // determine which of the two situations we are in:
                //  1. Are we starting unstuck from an entirely new position?
                //  2. Are we doing a starting a new attempt from the same position but at a different heading?
                if (!m_bIsCurrentlyAligning)
                {
                    // Current position of the rover
                    geoops::GPSCoordinate stCurrentPosition = globals::g_pNavigationBoard->GetGPSData();

                    // Are we still in the same position?
                    if (SamePosition(m_stOriginalPosition, stCurrentPosition))
                    {
                        // Start a new attempt
                        ++m_unAttempts;
                    }
                    else
                    {
                        // If this is our first attempt at becoming 'unstuck' from this position save
                        //  the current location and heading for future attempts.
                        m_unAttempts         = 1;
                        m_stOriginalPosition = stCurrentPosition;
                        m_dOriginalHeading   = globals::g_pNavigationBoard->GetIMUData().dHeading;
                    }
                }

                // On the first attempt we use the rover's original heading so alignment would already be completed.
                m_bIsCurrentlyAligning = m_unAttempts != 1;

                // On the second attempt align the rover 30 degrees to the right of the
                // original heading. For the third do it 30 degrees to the left of the
                // original heading instead.
                if (1 < m_unAttempts && m_unAttempts < 4)
                {
                    // New target heading relative to the original heading.
                    double dChangeInHeading = (m_unAttempts == 2) ? 30 : -30;
                    // New absolute target heading (not relative).
                    double dGoalHeading = m_dOriginalHeading + dChangeInHeading;
                    dGoalHeading        = numops::InputAngleModulus<double>(dGoalHeading, 0, 360);

                    // Heading the rover's currently facing.
                    double dCurrentHeading = globals::g_pNavigationBoard->GetIMUData().dHeading;
                    // Desired change in yaw relative to rover's current heading.
                    double dYawAdjustment = numops::InputAngleModulus<double>(dGoalHeading - dCurrentHeading, -180, 180);

                    // If desired yaw adjustment is significantly large, keep aligning the rover with the desired heading.
                    if (std::abs(dYawAdjustment) > m_dHeadingTolerance)
                    {
                        if (dYawAdjustment >= 0)
                        {
                            // Rotate right.
                            globals::g_pDriveBoard->SendDrive(m_dInplaceRotationMotorPower, -m_dInplaceRotationMotorPower);
                        }
                        else
                        {
                            // Rotate left.
                            globals::g_pDriveBoard->SendDrive(-m_dInplaceRotationMotorPower, m_dInplaceRotationMotorPower);
                        }

                        m_bIsCurrentlyAligning = true;
                    }
                    else
                    {
                        // Stop the rover as we have approximately reached the desired heading.
                        globals::g_pDriveBoard->SendStop();
                        // Rover is aligned with target heading.
                        m_bIsCurrentlyAligning = false;
                    }
                }

                // If we aren't currently aligning the rover with a new heading
                // and are in the first three attempts, start reverse.
                if (!m_bIsCurrentlyAligning && m_unAttempts < 4)
                {
                    globals::g_pStateMachineHandler->HandleEvent(State::eStart);
                }
                // If we have already done three attempts abort the unstuck state.
                else if (m_unAttempts >= 4)
                {
                    globals::g_pStateMachineHandler->HandleEvent(State::eAbort);
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
                        eNextState = States::eReversing;
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
             * @brief Checks if the rover is approximately in the same position.
             *
             * @note The threshold that defines how far away we need to be from the original point to be considered
             *  a different position is m_dSamePositionThreshold.
             *
             * @param stLastPosition - Original position the rover was located.
             * @param stCurrPosition - Current position the rover is located.
             * @return true - The rover is in the same position.
             * @return false - The rover is in a different position.
             *
             * @author Jason Pittman (jspencerpittman@gmail.com)
             * @date 2024-02-14
             ******************************************************************************/
            bool SamePosition(const geoops::GPSCoordinate& stOriginalPosition, const geoops::GPSCoordinate& stCurrPosition)
            {
                double dDistance = geoops::CalculateGeoMeasurement(stOriginalPosition, stCurrPosition).dDistanceMeters;
                return dDistance <= m_dSamePositionThreshold;
            }
    };
}    // namespace statemachine

#endif    // STUCKSTATE_HPP
