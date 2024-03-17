/******************************************************************************
 * @brief Avoidance State Implementation for Autonomy State Machine.
 *
 * @file AvoidanceState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "AvoidanceState.h"
#include "../AutonomyGlobals.h"
#include "../algorithms/controllers/StanleyController.h"
#include "../algorithms/planners/AStar.h"

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
    void AvoidanceState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "AvoidanceState: Scheduling next run of state logic.");

        m_nMaxDataPoints = 100;

        m_vRoverXPath.reserve(m_nMaxDataPoints);
        m_vRoverYPath.reserve(m_nMaxDataPoints);
        m_vRoverYawPath.reserve(m_nMaxDataPoints);
        m_vRoverXPosition.reserve(m_nMaxDataPoints);
        m_vRoverYPosition.reserve(m_nMaxDataPoints);
        m_vRoverYawPosition.reserve(m_nMaxDataPoints);
        m_vRoverVelocity.reserve(m_nMaxDataPoints);

        m_nLastIDX                   = 0;
        m_nTargetIDX                 = 0;

        m_tPathStartTime             = time(nullptr);
        m_tStuckCheckTime            = time(nullptr);

        m_dStuckCheckLastPosition[0] = 0;
        m_dStuckCheckLastPosition[1] = 0;

        // TODO: Add ASTAR Pathfinder
        // TODO: Add Stanley Controller
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void AvoidanceState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "AvoidanceState: Exiting state.");

        m_nMaxDataPoints = 100;

        m_vRoverXPath.clear();
        m_vRoverYPath.clear();
        m_vRoverYawPath.clear();
        m_vRoverXPosition.clear();
        m_vRoverYPosition.clear();
        m_vRoverYawPosition.clear();
        m_vRoverVelocity.clear();

        m_nTargetIDX                 = 0;

        m_dStuckCheckLastPosition[0] = 0;
        m_dStuckCheckLastPosition[1] = 0;

        // TODO: Clear ASTAR Pathfinder
        // TODO: Clear Stanley Controller
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    AvoidanceState::AvoidanceState() : State(States::eAvoidance)
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
    void AvoidanceState::Run()
    {
        // TODO: Implement the behavior specific to the Avoidance state
        LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Running state-specific behavior.");
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
    States AvoidanceState::TriggerEvent(Event eEvent)
    {
        States eNextState       = States::eAvoidance;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "AvoidanceState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eAvoidance;
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "AvoidanceState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            case Event::eEndObstacleAvoidance:
            {
                LOG_INFO(logging::g_qSharedLogger, "AvoidanceState: Handling EndObstacleAvoidance event.");
                eNextState = States::NUM_STATES;    // Replace with `get_prev_state()`
                break;
            }
            case Event::eStuck:
            {
                LOG_INFO(logging::g_qSharedLogger, "AvoidanceState: Handling Stuck event.");
                eNextState = States::eStuck;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "AvoidanceState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eAvoidance)
        {
            LOG_INFO(logging::g_qSharedLogger, "AvoidanceState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
