/******************************************************************************
 * @brief Approaching Object State Implementation for Autonomy State Machine.
 *
 * @file ApproachingObjectState.cpp
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "ApproachingObjectState.h"
#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief This method is called when the state is first started. It is used to
     *        initialize the state.
     *
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingObjectState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Scheduling next run of state logic.");

        // Initialize member variables.
        m_stGoalWaypoint = globals::g_pWaypointHandler->PeekNextWaypoint();

        // Store the state that got stuck and triggered a MarkerSeen event.
        m_eTriggeringState = globals::g_pStateMachineHandler->GetPreviousState();

        // Add the search and rover path layers to the plot.
        m_pRoverPathPlot->CreateDotLayer("DetectedObjects", "orange");
        m_pRoverPathPlot->CreateDotLayer("FinalTag", "green");
        m_pRoverPathPlot->CreatePathLayer("RoverPath", "-.r*");

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
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    ApproachingObjectState::ApproachingObjectState() : State(States::eApproachingObject)
    {
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        m_bInitialized   = false;

        m_StuckDetector  = statemachine::TimeIntervalBasedStuckDetector(constants::STUCK_CHECK_ATTEMPTS,
                                                                       constants::STUCK_CHECK_INTERVAL,
                                                                       constants::STUCK_CHECK_VEL_THRESH,
                                                                       constants::STUCK_CHECK_ROT_THRESH);
        m_pRoverPathPlot = std::make_unique<logging::graphing::PathTracer>("ApproachingMarkerRoverPath");

        if (!m_bInitialized)
        {
            Start();
            m_bInitialized = true;
        }
    }

    /******************************************************************************
     * @brief Run the state machine. Returns the next state.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingObjectState::Run()
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "ApproachingObjectState: Running state-specific behavior.");

        // Get the current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();

        // Add the current rover pose to the path plot.
        m_pRoverPathPlot->AddPathPoint(stCurrentRoverPose.GetUTMCoordinate(), "RoverPath");

        // Check Rover radius from object waypoint.
        geoops::GeoMeasurement stCurrentMeasurement = geoops::CalculateGeoMeasurement(m_stGoalWaypoint.GetGPSCoordinate(), stCurrentRoverPose.GetGPSCoordinate());
        if (stCurrentMeasurement.dDistanceMeters > m_stGoalWaypoint.dRadius)
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger,
                        "ApproachingObjectState: Rover is too far from the original waypoint! Waypoint radius is {} meters, current distance is {} meters.",
                        m_stGoalWaypoint.dRadius,
                        stCurrentMeasurement.dDistanceMeters);
            globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerUnseen);
            return;
        }

        // TODO: Implement object detection logic here.
        // Identify target object.

        // Check if object is unseen.
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
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling ReachedObject event.");
                // Pop old waypoint out of queue.
                globals::g_pWaypointHandler->PopNextWaypoint();
                // Change state.
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
