
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
        m_stStartPosition = globals::g_pNavigationBoard->GetGPSData();
        m_dStartHeading   = globals::g_pNavigationBoard->GetHeading();

        // Store state start time.
        m_tmStartReversingTime = std::chrono::high_resolution_clock::now();
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

        // Get current position and heading.
        geoops::GPSCoordinate stCurrentPosition = globals::g_pNavigationBoard->GetGPSData();
        double dCurrentHeading                  = globals::g_pNavigationBoard->GetHeading();
        // Get the current time.
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::high_resolution_clock::now();

        // Calculate current distance from start point.
        geoops::GeoMeasurement stMeasurement = geoops::CalculateGeoMeasurement(stCurrentPosition, m_stStartPosition);

        // Calculate time elapsed.
        double dTimeElapsed = std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmStartReversingTime).count();
        // Check if rover has reversed the desired distance or timeout has been reached.
        if (stMeasurement.dDistanceMeters >= constants::REVERSE_DISTANCE || dTimeElapsed >= constants::REVERSE_TIMEOUT)
        {
            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "ReversingState: Reversed {} meters in {} seconds.", stMeasurement.dDistanceMeters, dTimeElapsed);
            // Stop reversing.
            globals::g_pDriveBoard->SendStop();
            // Handle reversing complete event.
            globals::g_pStateMachineHandler->HandleEvent(Event::eReverseComplete);
            // Exit method without running other code below.
            return;
        }

        // Check if we should try to maintain heading while reversing.
        if (constants::REVERSE_MAINTAIN_HEADING)
        {
            // Reverse straight backwards.
            diffdrive::DrivePowers stReverse = globals::g_pDriveBoard->CalculateMove(-std::fabs(constants::REVERSE_POWER),
                                                                                     m_dStartHeading,
                                                                                     dCurrentHeading,
                                                                                     diffdrive::DifferentialControlMethod::eArcadeDrive);
            // Send drive powers.
            globals::g_pDriveBoard->SendDrive(stReverse);
        }
        else
        {
            // Just set reverse drive powers manually.
            diffdrive::DrivePowers stMotorPowers{-std::fabs(constants::REVERSE_POWER), -std::fabs(constants::REVERSE_POWER)};
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
        States eNextState       = States::eReversing;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eStart:
            {
                LOG_INFO(logging::g_qSharedLogger, "ReversingState: Handling Start event.");
                break;
            }
            case Event::eAbort:
            {
                LOG_INFO(logging::g_qSharedLogger, "ReversingState: Handling Abort event.");
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
            default:
            {
                LOG_INFO(logging::g_qSharedLogger, "ReversingState: Handling unknown event.");
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
