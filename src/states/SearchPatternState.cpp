/******************************************************************************
 * @brief Search Pattern State Implementation for Autonomy State Machine.
 *
 * @file SearchPatternState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "SearchPatternState.h"
#include "../algorithms/SearchPattern.hpp"
#include "../interfaces/State.hpp"

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
    void SearchPatternState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Scheduling next run of state logic.");

        m_nMaxDataPoints             = 100;
        m_tStuckCheckTime            = time(nullptr);

        m_dStuckCheckLastPosition[0] = 0;
        m_dStuckCheckLastPosition[1] = 0;

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
    void SearchPatternState::Exit()
    {
        // Clean up the state before exiting
        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Exiting state.");

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
    SearchPatternState::SearchPatternState() : State(States::eSearchPattern)
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
    void SearchPatternState::Run()
    {
        // TODO: Implement the behavior specific to the SearchPattern state
        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Running state-specific behavior.");
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
    States SearchPatternState::TriggerEvent(Event eEvent)
    {
        States eNextState       = States::eSearchPattern;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eMarkerSeen:
            {
                LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling MarkerSeen event.");
                eNextState = States::eApproachingMarker;
                break;
            }
            case Event::eObjectSeen:
            {
                LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling ObjectSeen event.");
                eNextState = States::eApproachingObject;
                break;
            }
            case Event::eStart:
            {
                LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling Start event.");
                eNextState = States::eSearchPattern;
                break;
            }
            case Event::eSearchFailed:
            {
                LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling SearchFailed event.");
                eNextState = States::eIdle;
                break;
            }
            case Event::eAbort:
            {
                LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling Abort event.");
                eNextState = States::eIdle;
                break;
            }
            case Event::eStuck:
            {
                LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Handling Stuck event.");
                eNextState = States::eStuck;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "SearchPatternState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eSearchPattern)
        {
            LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
