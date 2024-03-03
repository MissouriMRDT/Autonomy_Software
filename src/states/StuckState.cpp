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
        LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Scheduling next run of state logic.");

        m_tStuckCheckTime = time(nullptr);

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
    void StuckState::Exit()
    {
        // Clean up the state before exiting
        LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Exiting state.");
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
    }

    /******************************************************************************
     * @brief Run the state machine. Returns the next state.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    States StuckState::Run()
    {
        // TODO: Implement the behavior specific to the Stuck state
        LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Running state-specific behavior.");

        return States::eStuck;
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
}    // namespace statemachine
