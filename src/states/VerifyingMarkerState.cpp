/******************************************************************************
 * @brief Verifying Marker State Implementation for Autonomy State Machine.
 *
 * @file VerifyingMarkerState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "VerifyingMarkerState.h"
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
    void VerifyingMarkerState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Scheduling next run of state logic.");

        m_nMaxMarkerIDs = 50;

        m_vMarkerIDs.reserve(m_nMaxMarkerIDs);
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void VerifyingMarkerState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Exiting state.");

        m_vMarkerIDs.clear();
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    VerifyingMarkerState::VerifyingMarkerState() : State(States::eVerifyingMarker)
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
    void VerifyingMarkerState::Run()
    {
        // TODO: Implement the behavior specific to the VerifyingMarker state
        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingMarkerState: Running state-specific behavior.");
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
    States VerifyingMarkerState::TriggerEvent(Event eEvent)
    {
        States eNextState       = States::eVerifyingMarker;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eVerifyingMarker;
                break;
            }
            case Event::eVerifyingComplete:
            {
                LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Handling Verifying Complete event.");
                eNextState = States::eIdle;
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "VerifyingMarkerState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eVerifyingMarker)
        {
            LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
