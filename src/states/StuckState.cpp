/******************************************************************************
 * @brief Stuck State Implementation for Autonomy State Machine.
 *
 * @file StuckState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "StuckState.h"
#include "../AutonomyConstants.h"
#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{

    /******************************************************************************
     * @brief This method is called when the state is first started. It is used to
     *        initialize the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void StuckState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "StuckState: Scheduling next run of state logic.");

        m_bIsCurrentlyAligning = false;

        globals::g_pDriveBoard->SendStop();
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void StuckState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "StuckState: Exiting state.");
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    StuckState::StuckState() : State(States::eStuck)
    {
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        m_bInitialized = false;

        if (!m_bInitialized)
        {
            Start();
            m_bInitialized = true;
        }

        m_unAttempts           = 1;
        m_stOriginalPosition   = geoops::GPSCoordinate(0, 0);
        m_dOriginalHeading     = 0;
        m_bIsCurrentlyAligning = false;
    }

    /******************************************************************************
     * @brief Run the state machine. Returns the next state.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void StuckState::Run()
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

                // If this attempt requires the rover to rotate we initialize
                //  the members are used to determine if the rover's stuck in a way
                //  it can't rotate.
                if (1 < m_unAttempts && m_unAttempts < 4)
                {
                    m_unStuckChecksOnAttempt = 0;
                    m_tmLastStuckCheck       = std::chrono::system_clock::now();
                }
            }
            else
            {
                // If this is our first attempt at becoming 'unstuck' from this position save
                //  the current location and heading for future attempts.
                m_unAttempts         = 1;
                m_stOriginalPosition = stCurrentPosition;
                m_dOriginalHeading   = globals::g_pNavigationBoard->GetHeading();
            }
        }

        // On the first attempt we use the rover's original heading so alignment would already be completed.
        m_bIsCurrentlyAligning = m_unAttempts != 1;

        // On the second attempt align the rover 30 degrees to the right of the
        // original heading. For the third do it 30 degrees to the left of the
        // original heading instead.
        if (1 < m_unAttempts && m_unAttempts < 4)
        {
            // Is is time to check if the rover's stuck
            std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
            double dTimeSinceLastCheck = (std::chrono::duration_cast<std::chrono::microseconds>(tmCurrentTime - m_tmLastStuckCheck).count() / 1e6);
            if (dTimeSinceLastCheck > constants::STUCK_CHECK_INTERVAL)
            {
                // Is the angular velocity showing no rotation?
                double dCurrAngVel = globals::g_pNavigationBoard->GetAngularVelocity();
                if (dCurrAngVel < constants::STUCK_CHECK_ROT_THRESH)
                {
                    ++m_unStuckChecksOnAttempt;
                }
                else
                {
                    m_unStuckChecksOnAttempt = 0;
                }
                m_tmLastStuckCheck = tmCurrentTime;

                // After a certain amount of consecutive checks confirming the rover isn't rotating
                //  in alignment mode shift to the next attempt.
                if (m_unStuckChecksOnAttempt >= constants::STUCK_CHECK_ATTEMPTS)
                {
                    m_bIsCurrentlyAligning = false;
                    return;
                }
            }

            // New target heading relative to the original heading.
            double dChangeInHeading = (m_unAttempts == 2) ? 30 : -30;
            // New absolute target heading (not relative).
            double dGoalHeading = m_dOriginalHeading + dChangeInHeading;
            dGoalHeading        = numops::InputAngleModulus<double>(dGoalHeading, 0, 360);

            // Heading the rover's currently facing.
            double dCurrentHeading = globals::g_pNavigationBoard->GetHeading();
            // Desired change in yaw relative to rover's current heading.
            double dYawAdjustment = numops::InputAngleModulus<double>(dGoalHeading - dCurrentHeading, -180, 180);

            // If desired yaw adjustment is significantly large, keep aligning the rover with the desired heading.
            if (std::abs(dYawAdjustment) > constants::STUCK_HEADING_TOLERANCE)
            {
                if (dYawAdjustment >= 0)
                {
                    // Rotate right.
                    diffdrive::DrivePowers stRotateRight;
                    stRotateRight.dLeftDrivePower  = constants::STUCK_MOTOR_POWER;
                    stRotateRight.dRightDrivePower = -constants::STUCK_MOTOR_POWER;
                    globals::g_pDriveBoard->SendDrive(stRotateRight);
                }
                else
                {
                    // Rotate left.
                    diffdrive::DrivePowers stRotateLeft;
                    stRotateLeft.dLeftDrivePower  = -constants::STUCK_MOTOR_POWER;
                    stRotateLeft.dRightDrivePower = constants::STUCK_MOTOR_POWER;
                    globals::g_pDriveBoard->SendDrive(stRotateLeft);
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
            globals::g_pStateMachineHandler->HandleEvent(Event::eStart, true);
        }
        // If we have already done three attempts abort the unstuck state.
        else if (m_unAttempts >= 4)
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eAbort, false);
        }

        return;
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
    States StuckState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eStuck;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "StuckState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eReversing;
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "StuckState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "StuckState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eStuck)
        {
            LOG_INFO(logging::g_qSharedLogger, "StuckState: Transitioning to {} State.", StateToString(eNextState));

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
     *  a different position is constants::STUCK_SAME_POINT_PROXIMITY.
     *
     * @param stLastPosition - Original position the rover was located.
     * @param stCurrPosition - Current position the rover is located.
     * @return true - The rover is in the same position.
     * @return false - The rover is in a different position.
     *
     * @author Jason Pittman (jspencerpittman@gmail.com)
     * @date 2024-02-14
     ******************************************************************************/
    bool StuckState::SamePosition(const geoops::GPSCoordinate& stOriginalPosition, const geoops::GPSCoordinate& stCurrPosition)
    {
        double dDistance = geoops::CalculateGeoMeasurement(stOriginalPosition, stCurrPosition).dDistanceMeters;
        return dDistance <= constants::STUCK_SAME_POINT_PROXIMITY;
    }
}    // namespace statemachine
