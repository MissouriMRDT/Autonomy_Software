/******************************************************************************
 * @brief Idle State Implementation for Autonomy State Machine.
 *
 * @file IdleState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "IdleState.h"
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
    void IdleState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "IdleState: Scheduling next run of state logic.");

        m_tIdleTime      = time(nullptr);
        m_bRealigned     = false;
        m_nMaxDataPoints = 100;

        m_vRoverXPosition.reserve(m_nMaxDataPoints);
        m_vRoverYPosition.reserve(m_nMaxDataPoints);
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void IdleState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "IdleState: Exiting state.");

        m_vRoverXPosition.clear();
        m_vRoverYPosition.clear();
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    IdleState::IdleState() : State(States::eIdle)
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
    void IdleState::Run()
    {
        // TODO: Implement the behavior specific to the Idle state
        LOG_DEBUG(logging::g_qSharedLogger, "IdleState: Running state-specific behavior.");
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
    States IdleState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eIdle;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "IdleState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);

                bool tagInSight    = false;    // TODO: Replace with actual tag detection
                bool reverseAlways = false;    // TODO: Replace with actual reverse always flag

                // If there is an ArUco marker in the camera's field of view, transition to backup before navigating.
                if (tagInSight)
                {
                    LOG_INFO(logging::g_qSharedLogger, "IdleState: Detected ArUco marker. Transitioning to Reverse State.");
                    eNextState = States::eReversing;
                }
                // If the reverse always flag is set, transition to backup before navigating.
                else if (reverseAlways)
                {
                    LOG_INFO(logging::g_qSharedLogger, "IdleState: Reverse always flag set. Transitioning to Reverse State.");
                    eNextState = States::eReversing;
                }
                // Otherwise, transition to navigating.
                else
                {
                    // Check if waypoint handler has any waypoints.
                    if (globals::g_pWaypointHandler->GetWaypointCount() > 0)
                    {
                        // Submit logger message.
                        LOG_INFO(logging::g_qSharedLogger, "IdleState: No ArUco marker detected. Transitioning to Navigating State.");
                        // Change states.
                        eNextState = States::eNavigating;
                    }
                    else
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger,
                                    "IdleState: Not transitioning to NavigatingState because no waypoints have been added to the waypoint handler!");
                    }
                }

                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "IdleState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            default:
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "IdleState: Handling unknown event.");
                break;
            }
        }

        if (eNextState != States::eIdle)
        {
            LOG_INFO(logging::g_qSharedLogger, "IdleState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
