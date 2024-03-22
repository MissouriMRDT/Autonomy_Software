/******************************************************************************
 * @brief Approaching Object State Implementation for Autonomy State Machine.
 *
 * @file ApproachingObjectState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "ApproachingObjectState.h"
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
    void ApproachingObjectState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Scheduling next run of state logic.");

        m_nNumDetectionAttempts = 0;
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingObjectState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Exiting state.");
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    ApproachingObjectState::ApproachingObjectState() : State(States::eApproachingObject)
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
    void ApproachingObjectState::Run()
    {
        // TODO: Implement the behavior specific to the Approaching Object state
        // BUG: This breaks occasionally.
        // HACK: This is bad, never do this.
        // FIXME: AAAHAHAHHAAH
        // LEAD: AHAHAHAHA
        // ISSUE NOTE: WHAT THE HELL
        // TEST: LKFJLSKDFJLKSDjf
        // [ ] this is a list item.
        // [x] this is a checked list item.
        LOG_DEBUG(logging::g_qSharedLogger, "ApproachingObjectState: Running state-specific behavior.");
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
    States ApproachingObjectState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eIdle;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eReachedObject:
            {
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling ReachedObject event.");
                eNextState = States::eIdle;
                break;
            }
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            case Event::eObjectUnseen:
            {
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling ObjectUnseen event.");
                eNextState = States::eSearchPattern;
                break;
            }
            case Event::eMarkerSeen:
            {
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling Abort event.");
                eNextState = States::eIdle;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "ApproachingObjectState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eIdle)
        {
            LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
