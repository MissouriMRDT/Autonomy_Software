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
#include "../AutonomyConstants.h"
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
    void AvoidanceState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "AvoidanceState: Scheduling next run of state logic.");

        // Initialize Stanley Controller:
        m_stController = controllers::StanleyController(constants::STANLEY_STEER_CONTROL_GAIN, constants::STANLEY_DIST_TO_FRONT_AXLE, constants::STANLEY_YAW_TOLERANCE);

        // Initialize ASTAR Pathfinder:
        // TODO: Poll zedCam / object detector for seen obstacles to pass to AStar.
        // Determine start and goal (peek waypoint for goal).
        m_stPose         = globals::g_pWaypointHandler->SmartRetrieveRoverPose();
        m_stStart        = m_stPose.GetUTMCoordinate();
        m_stGoalWaypoint = globals::g_pWaypointHandler->PeekNextWaypoint();

        // Plan avoidance route using AStar.
        // TODO: Add obstacles to params.
        m_vPlannedRoute = m_stPlanner.PlanAvoidancePath(m_stStart, m_stGoalWaypoint.GetUTMCoordinate());

        // Check for AStar failure.
        if (!m_vPlannedRoute.empty())
        {
            m_stGoal = m_vPlannedRoute.back();
            m_stController.SetPath(m_vPlannedRoute);
        }
        // Exit Obstacle Avoidance if AStar fails to generate a path.
        else
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eEndObstacleAvoidance, false);
        }
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

        // Clear ASTAR Pathfinder
        m_stPlanner.ClearObstacleData();

        // Clear Stanley Controller
        m_stController.ClearPath();
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

        m_bInitialized   = false;
        m_stStuckChecker = statemachine::TimeIntervalBasedStuckDetector(constants::STUCK_CHECK_ATTEMPTS,
                                                                        constants::STUCK_CHECK_INTERVAL,
                                                                        constants::STUCK_CHECK_VEL_THRESH,
                                                                        constants::STUCK_CHECK_ROT_THRESH);

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
        LOG_DEBUG(logging::g_qSharedLogger, "AvoidanceState: Running state-specific behavior.");

        // TODO: build in obstacle detection and refresh of astar path when a new object is detected
        // This can be done by calling PlanAvoidanceRoute() with an updated obstacle vector.

        // A route has already been plotted by the planner and passed to the controller.
        // Navigate by issuing drive commands from the controller.
        // Check if we are at the goal waypoint.
        geoops::RoverPose stCurrentPose                  = globals::g_pWaypointHandler->SmartRetrieveRoverPose();
        geoops::GeoMeasurement stGoalWaypointMeasurement = geoops::CalculateGeoMeasurement(stCurrentPose.GetUTMCoordinate(), m_stGoalWaypoint.GetUTMCoordinate());

        // Check to see if rover velocity is below stuck threshold (scaled to avoidance speed).
        if (m_stStuckChecker.CheckIfStuck(globals::g_pWaypointHandler->SmartRetrieveVelocity(), globals::g_pWaypointHandler->SmartRetrieveAngularVelocity()))
        {
            LOG_INFO(logging::g_qSharedLogger, "AvoidanceState: Rover has become stuck");
            globals::g_pStateMachineHandler->HandleEvent(Event::eStuck, true);
        }

        // Goal has not been reached yet:
        else if (stGoalWaypointMeasurement.dDistanceMeters > constants::NAVIGATING_REACHED_GOAL_RADIUS)
        {
            // Request objects:
            // Check for any new objects:
            // Re-plan route (call planPath again):

            // Update pose for drive calculation.
            m_stPose               = globals::g_pWaypointHandler->SmartRetrieveRoverPose();
            const double dVelocity = globals::g_pWaypointHandler->SmartRetrieveVelocity();
            // Calculate drive move/powers at the speed multiplier.
            diffdrive::DrivePowers stDriveSpeeds =
                globals::g_pDriveBoard->CalculateMove(constants::AVOIDANCE_STATE_MOTOR_POWER,
                                                      m_stController.Calculate(m_stPose.GetUTMCoordinate(), dVelocity, m_stPose.GetCompassHeading()),
                                                      stCurrentPose.GetCompassHeading(),
                                                      diffdrive::DifferentialControlMethod::eArcadeDrive);
            // Send drive command to drive board.
            globals::g_pDriveBoard->SendDrive(stDriveSpeeds);
        }

        // Local goal is reached, end obstacle avoidance:
        else
        {
            globals::g_pDriveBoard->SendStop();
            globals::g_pStateMachineHandler->HandleEvent(Event::eEndObstacleAvoidance, false);
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
    States AvoidanceState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
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
