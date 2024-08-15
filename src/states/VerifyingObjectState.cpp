/******************************************************************************
 * @brief Verifying Object State Implementation for Autonomy State Machine.
 *
 * @file VerifyingObjectState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "VerifyingObjectState.h"
#include "../AutonomyGlobals.h"
#include "../AutonomyNetworking.h"

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
    void VerifyingObjectState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Scheduling next run of state logic.");

        m_nMaxObjectIDs = 50;

        m_vObjectIDs.reserve(m_nMaxObjectIDs);
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void VerifyingObjectState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Exiting state.");

        m_vObjectIDs.clear();
    }

    /******************************************************************************
     * @brief Accessor for the State private member. Returns the state as a string.
     *
     * @return std::string - The current state as a string.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    VerifyingObjectState::VerifyingObjectState() : State(States::eVerifyingObject)
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
    void VerifyingObjectState::Run()
    {
        // TODO: Implement the behavior specific to the VerifyingObject state
        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingObjectState: Running state-specific behavior.");
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
    States VerifyingObjectState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eVerifyingObject;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eVerifyingComplete:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Handling Verifying Complete event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "VerifyingObjectState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eVerifyingObject)
        {
            LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
