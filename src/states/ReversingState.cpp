
/******************************************************************************
 * @brief Reversing State Implementation for Autonomy State Machine.
 *
 * @file ReversingState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "ReversingState.h"
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
    void ReversingState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "ReversingState: Scheduling next run of state logic.");

        // Store the starting position and heading of rover when it entered this state.
        m_stStartRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();

        // Store state start time.
        m_tmStartReversingTime = std::chrono::high_resolution_clock::now();
        m_tmTimeSinceLastMeter = m_tmStartReversingTime;
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void ReversingState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "ReversingState: Exiting state.");
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    ReversingState::ReversingState() : State(States::eReversing)
    {
        // Submit logger message.
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
    void ReversingState::Run()
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "ReversingState: Running state-specific behavior.");

        // Create instance variables.
        static bool bTimeSinceLastMeterAlreadySet = false;

        // Get current position and heading.
        geoops::RoverPose stCurrentRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();
        // Get the current time.
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::high_resolution_clock::now();
        // Calculate current distance from start point.
        geoops::GeoMeasurement stMeasurement = geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetGPSCoordinate(), m_stStartRoverPose.GetGPSCoordinate());

        // Calculate time elapsed.
        double dTotalTimeElapsed          = std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmStartReversingTime).count();
        double dTimeElapsedSinceLastMeter = std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmTimeSinceLastMeter).count();
        // Check if rover has reversed the desired distance or timeout has been reached.
        if (stMeasurement.dDistanceMeters >= constants::REVERSE_DISTANCE)
        {
            // Submit logger message.
            LOG_NOTICE(logging::g_qSharedLogger, "ReversingState: Successfully reversed {} meters in {} seconds.", stMeasurement.dDistanceMeters, dTotalTimeElapsed);
            // Stop reversing.
            globals::g_pDriveBoard->SendStop();
            // Handle reversing complete event.
            globals::g_pStateMachineHandler->HandleEvent(Event::eReverseComplete);
            // Exit method without running other code below.
            return;
        }
        // Check if we aren't covering enough distance within the timeout limit.
        else if (dTimeElapsedSinceLastMeter >= constants::REVERSE_TIMEOUT_PER_METER)
        {
            // Submit logger message.
            LOG_NOTICE(logging::g_qSharedLogger,
                       "ReversingState: Reversed {} meters in {} seconds before timeout was reached. Goal was {} meters, so rover must be running into something...",
                       stMeasurement.dDistanceMeters,
                       dTotalTimeElapsed,
                       constants::REVERSE_DISTANCE);
            // Stop reversing.
            globals::g_pDriveBoard->SendStop();
            // Handle reversing complete event.
            globals::g_pStateMachineHandler->HandleEvent(Event::eStuck);
            // Exit method without running other code below.
            return;
        }
        // Reset the timestamp since last meter every other meter reversed.
        else if (int(stMeasurement.dDistanceMeters) % 2 == 0 && !bTimeSinceLastMeterAlreadySet)
        {
            // Update timestamp.
            m_tmTimeSinceLastMeter = std::chrono::high_resolution_clock::now();
            // Set toggle.
            bTimeSinceLastMeterAlreadySet = true;
        }
        else if (int(stMeasurement.dDistanceMeters) % 2 != 0 && bTimeSinceLastMeterAlreadySet)
        {
            // Reset toggle.
            bTimeSinceLastMeterAlreadySet = false;
        }

        // Check if we should try to maintain heading while reversing.
        if (constants::REVERSE_MAINTAIN_HEADING)
        {
            // Reverse straight backwards.
            diffdrive::DrivePowers stReverse = globals::g_pDriveBoard->CalculateMove(-std::fabs(constants::REVERSE_MOTOR_POWER),
                                                                                     m_stStartRoverPose.GetCompassHeading(),
                                                                                     stCurrentRoverPose.GetCompassHeading(),
                                                                                     diffdrive::DifferentialControlMethod::eArcadeDrive);
            // Send drive powers.
            globals::g_pDriveBoard->SendDrive(stReverse);
        }
        else
        {
            // Just set reverse drive powers manually.
            diffdrive::DrivePowers stMotorPowers{-std::fabs(constants::REVERSE_MOTOR_POWER), -std::fabs(constants::REVERSE_MOTOR_POWER)};
            // Send drive powers.
            globals::g_pDriveBoard->SendDrive(stMotorPowers);
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
    States ReversingState::TriggerEvent(Event eEvent)
    {
        // Initialize member variables.
        States eNextState       = States::eReversing;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ReversingState: Handling Start event.");
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ReversingState: Handling Abort event.");
                // Change states.
                eNextState = States::eIdle;
                break;
            }
            case Event::eReverseComplete:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ReversingState: Handling ReverseComplete event.");
                // Transition back to state that triggered reversing.
                eNextState = globals::g_pStateMachineHandler->GetPreviousState();
                break;
            }
            case Event::eStuck:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ReversingState: Handling Stuck event.");
                // Change states.
                eNextState = States::eStuck;
                break;
            }
            default:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ReversingState: Handling unknown event.");
                // Change states.
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eReversing)
        {
            LOG_INFO(logging::g_qSharedLogger, "ReversingState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
