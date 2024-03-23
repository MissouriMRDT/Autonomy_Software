
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

        // Get current position and heading
        stStartPosition = globals::g_pNavigationBoard->GetGPSData();
        dCurrentHeading = globals::g_pNavigationBoard->GetHeading();

        // For haversine formula convert to radians
        stStartPosition.dLatitude  = stStartPosition.dLatitude * M_PI / 180;
        stStartPosition.dLongitude = stStartPosition.dLongitude * M_PI / 180;
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

        // Make rover reverse
        // LEAD: @ryanw Are these both supposed to be dCurrentHeading? 2ns param is goal heading, 3rd is current. Doing this does not guarantee that the
        // LEAD: rover will reverse in a straight line. If you just want to reverse without trying to keep a goal heading, just use SendDrive().
        // FIXME: You can't use eTankDrive with CalculateMove. If you run the code it will print out an error. Use ArcadeDrive for more intuitive control.
        // FIXME: Make reverse speed a constant.
        diffdrive::DrivePowers stReverse = globals::g_pDriveBoard->CalculateMove(-1, dCurrentHeading, dCurrentHeading, diffdrive::DifferentialControlMethod::eTankDrive);
        globals::g_pDriveBoard->SendDrive(stReverse);

        // Haversine formula to find distance
        geoops::GPSCoordinate stCurrentPosition = globals::g_pNavigationBoard->GetGPSData();
        stCurrentPosition.dLatitude             = stCurrentPosition.dLatitude * M_PI / 180;
        stCurrentPosition.dLongitude            = stCurrentPosition.dLongitude * M_PI / 180;

        double dDistLon                         = stCurrentPosition.dLongitude - stStartPosition.dLongitude;
        double dDistLat                         = stCurrentPosition.dLatitude - stStartPosition.dLatitude;
        double a            = pow(sin(dDistLat / 2), 2) + cos(stStartPosition.dLatitude) * cos(stCurrentPosition.dLatitude) * pow(sin(dDistLon / 2), 2);
        double c            = 2 * asin(sqrt(a));
        double r            = 6371;    // Radius of earth in kilometers
        double dCurDistance = c * r;

        if (dCurDistance >= dDistanceThreshold)    // TODO: Currently will keep reversing if the position doesn't change
        {
            globals::g_pDriveBoard->SendStop();    // Stop reversing
            // FIXME: Calling this->TriggerEvent() will not change states. Call the state machine handler's HandleEven().
            // FIXME: globals::g_pStateMachineHandler->HandleEvent(Event::eReverseComplete);
            TriggerEvent(Event::eReverseComplete);
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
                LOG_INFO(logging::g_qSharedLogger, "ReversingState: Handling stReverse Complete event.");
                eNextState = States::eIdle;
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
