/******************************************************************************
 * @brief Navigating State Implementation for Autonomy State Machine.
 *
 * @file NavigatingState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "NavigatingState.h"
#include "../AutonomyGlobals.h"
#include "../AutonomyNetworking.h"
#include "../util/states/ObjectDetectionChecker.hpp"
#include "../util/states/TagDetectionChecker.hpp"

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
     * @author Eli Byrd (edbgkk@mst.edu), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void NavigatingState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Scheduling next run of state logic.");

        // Initialize member variables.
        m_bWasStuck         = false;
        m_bFetchNewWaypoint = true;
        m_vTagDetectors     = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam),
                               globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eRearCam)};
        m_vObjectDetectors  = {globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eHeadMainCam),
                               globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eRearCam)};
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void NavigatingState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Exiting state.");
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    NavigatingState::NavigatingState() : State(States::eNavigating)
    {
        // Submit logger message.
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        // Initialize member variables.
        m_bInitialized       = false;
        m_StuckDetector      = statemachine::TimeIntervalBasedStuckDetector(constants::STUCK_CHECK_ATTEMPTS,
                                                                            constants::STUCK_CHECK_INTERVAL,
                                                                            constants::STUCK_CHECK_VEL_THRESH,
                                                                            constants::STUCK_CHECK_ROT_THRESH);
        m_pStanleyController = std::make_unique<controllers::PredictiveStanleyController>(constants::STANLEY_CROSSTRACK_CONTROL_GAIN,
                                                                                          constants::STANLEY_ANGULAR_VELOCITY_LIMIT,
                                                                                          constants::STANLEY_PREDICTION_HORIZON,
                                                                                          constants::STANLEY_PREDICTION_TIME_STEP);

        // Start state.
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
    void NavigatingState::Run()
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Running state-specific behavior.");

        // Get Current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();

        // If navigating was previously stuck, then re-path plan.
        if (m_bWasStuck)
        {
            int nObstacleCount = globals::g_pWaypointHandler->GetObstaclesCount();

            // Ensure we actually have an obstacle to avoid before indexing array.
            if (nObstacleCount > 0)
            {
                // Get the most recently recorded obstacle's origin.
                geoops::UTMCoordinate stObstaclePosition = globals::g_pWaypointHandler->RetrieveObstacleAtIndex(nObstacleCount - 1).GetUTMCoordinate();

                LOG_NOTICE(logging::g_qSharedLogger, "Retrieved Obstacle: ({}, {})", stObstaclePosition.dEasting, stObstaclePosition.dNorthing);

                // Calculate distance from obstacle center to goal to determine what is "past" the obstacle.
                double dDistObsToGoal = geoops::CalculateGeoMeasurement(stObstaclePosition, m_stGoalWaypoint.GetUTMCoordinate()).dDistanceMeters;
                int nPointsRemoved    = 0;

                // Trim the original path up until safely past the obstacle.
                std::vector<geoops::Waypoint>::iterator itSafeNode = m_vPathCoordinates.begin();
                while (itSafeNode != m_vPathCoordinates.end())
                {
                    double dDistNodeToObs  = geoops::CalculateGeoMeasurement(itSafeNode->GetUTMCoordinate(), stObstaclePosition).dDistanceMeters;
                    double dDistNodeToGoal = geoops::CalculateGeoMeasurement(itSafeNode->GetUTMCoordinate(), m_stGoalWaypoint.GetUTMCoordinate()).dDistanceMeters;

                    // Determine if the node is trapped inside the obstacle bounds, or geometrically "behind" it.
                    bool bInsideObstacle = dDistNodeToObs <= (constants::STUCK_OBSTACLE_RADIUS * 1.5);
                    bool bBeforeObstacle = dDistNodeToGoal >= dDistObsToGoal;

                    // If a point is inside or before the obstacle, strip it from the current plan.
                    if (bInsideObstacle || bBeforeObstacle)
                    {
                        itSafeNode = m_vPathCoordinates.erase(itSafeNode);
                        nPointsRemoved++;
                    }
                    else
                    {
                        // Found the first node safely past the obstacle. Halt deletions.
                        break;
                    }
                }

                // Determine the geographic reconnection point. If the whole path was wiped, default to the goal.
                geoops::UTMCoordinate stReconnectCoordinate =
                    (itSafeNode != m_vPathCoordinates.end()) ? itSafeNode->GetUTMCoordinate() : m_stGoalWaypoint.GetUTMCoordinate();

                // Store the original beta bias so we don't permanently mess up standard path planning later.
                double dOriginalBeta = globals::g_pGeoPlanner->GetBetaBias();

                // Increase beta bias to force the rover radically around the obstacle.
                globals::g_pGeoPlanner->SetBetaBias(50.0);

                // Plan a splice path around the obstacle from the current rover position to the reconnection point.
                std::vector<geoops::Waypoint> vSplicePathCoordinates =
                    globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, stCurrentRoverPose.GetUTMCoordinate(), stReconnectCoordinate);

                // Restore the original beta bias after the bypass corridor is generated.
                globals::g_pGeoPlanner->SetBetaBias(dOriginalBeta);

                // Safely splice the new path into the remaining path
                int nPointsAdded = 0;
                if (!vSplicePathCoordinates.empty())
                {
                    // If we are reconnecting to an existing path, drop the final node of the splice.
                    // Otherwise, we inject a back-to-back duplicate waypoint which causes controller stuttering.
                    if (!m_vPathCoordinates.empty())
                    {
                        vSplicePathCoordinates.pop_back();
                    }

                    // Insert the newly computed bypass sequence at the beginning of the remaining original path.
                    m_vPathCoordinates.insert(m_vPathCoordinates.begin(), vSplicePathCoordinates.begin(), vSplicePathCoordinates.end());
                    nPointsAdded = vSplicePathCoordinates.size();
                }
                else
                {
                    LOG_WARNING(logging::g_qSharedLogger, "NavigatingState: GeoPlanner failed to find a splice path around the obstacle!");
                }

                LOG_INFO(logging::g_qSharedLogger, "Stuck state modified rover path: {} nodes added, {} nodes removed", nPointsAdded, nPointsRemoved);

                // Update controllers with the fused path sequence.
                m_pStanleyController->SetReferencePath(m_vPathCoordinates);
            }
            else
            {
                LOG_WARNING(logging::g_qSharedLogger, "NavigatingState: Rover marked as stuck, but the waypoint handler contains no obstacles!");
            }

            // Always clear the stuck flag so we don't loop this logic indefinitely.
            m_bWasStuck = false;
        }

        // Check if we should get a new goal waypoint and that the waypoint handler has one for us.
        if (m_bFetchNewWaypoint && globals::g_pWaypointHandler->GetWaypointCount() > 0)
        {
            // Trigger new waypoint event.
            globals::g_pStateMachineHandler->HandleEvent(Event::eNewWaypoint);
            return;
        }

        // Calculate distance and bearing from goal waypoint.
        geoops::GeoMeasurement stGoalWaypointMeasurement = geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetUTMCoordinate(), m_stGoalWaypoint.GetUTMCoordinate());

        // Only print out every so often.
        static bool bAlreadyPrinted = false;
        if ((std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 5) == 0 && !bAlreadyPrinted)
        {
            // Assemble the error metrics into a single string. We are going to include the distance and bearing to the goal waypoint and
            // the error between the rover pose and the GPS position. The rover pose could be from VIO or GNSS fusion, or just GPS.
            std::string szErrorMetrics =
                "--------[ Navigating Error Report ]--------\nDistance to Goal Waypoint: " + std::to_string(stGoalWaypointMeasurement.dDistanceMeters) + " meters\n" +
                "Bearing to Goal Waypoint: " + std::to_string(stGoalWaypointMeasurement.dStartRelativeBearing) + " degrees\n";
            // Submit the error metrics to the logger.
            LOG_INFO(logging::g_qSharedLogger, "{}", szErrorMetrics);

            // Set toggle.
            bAlreadyPrinted = true;
        }
        else if ((std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 5) != 0 && bAlreadyPrinted)
        {
            // Reset toggle.
            bAlreadyPrinted = false;
        }

        /*
            The overall flow of this state is as follows.
            1. Is there a tag -> MarkerSeen
            2. Is there an object -> ObjectSeen
            3. Is there an obstacle -> TBD
            4. Navigate to goal waypoint.
            5. Is the rover stuck -> Stuck
        */

        /////////////////////////
        /* --- Detect Tags --- */
        /////////////////////////

        // In order to even care about any tags we see, the goal waypoint needs to be of type MARKER and we need to be within the search radius of the MARKER waypoint.
        if (m_stGoalWaypoint.eType == geoops::WaypointType::eTagWaypoint && stGoalWaypointMeasurement.dDistanceMeters <= m_stGoalWaypoint.dRadius)
        {
            // Create instance variables.
            tagdetectutils::ArucoTag stBestArucoTag, stBestTorchTag;
            // Identify target marker.
            statemachine::IdentifyTargetMarker(m_vTagDetectors, stBestArucoTag, stBestTorchTag, m_stGoalWaypoint.nID);
            // Check if either tag type is seen.
            if (stBestArucoTag.nID != -1 || stBestTorchTag.dConfidence != 0.0)
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Rover has seen a target marker!");
                // Handle state transition and save the current search pattern state.
                globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerSeen, true);
                // Don't execute the rest of the state.
                return;
            }
        }

        ////////////////////////////
        /* --- Detect Objects --- */
        ////////////////////////////

        // In order to even care about any objects we see, the goal waypoint needs to be of an object type and we need to be within the search radius of the object
        // waypoint.
        if ((m_stGoalWaypoint.eType == geoops::WaypointType::eObjectWaypoint || m_stGoalWaypoint.eType == geoops::WaypointType::eMalletWaypoint ||
             m_stGoalWaypoint.eType == geoops::WaypointType::eWaterBottleWaypoint || m_stGoalWaypoint.eType == geoops::WaypointType::eRockPickWaypoint) &&
            stGoalWaypointMeasurement.dDistanceMeters <= m_stGoalWaypoint.dRadius)
        {
            // Create instance variables.
            objectdetectutils::Object stBestTorchObject;
            // Identify target object.
            statemachine::IdentifyTargetObject(m_vObjectDetectors, stBestTorchObject, m_stGoalWaypoint.eType);
            // Check if either tag type is seen.
            if (stBestTorchObject.dConfidence != 0.0)
            {
                // Submit logger message.
                LOG_NOTICE(logging::g_qSharedLogger, "NavigatingState: Rover has seen a target object!");

                // Handle state transition and save the current search pattern state.
                globals::g_pStateMachineHandler->HandleEvent(Event::eObjectSeen, true);
                // Don't execute the rest of the state.
                return;
            }
        }

        //////////////////////////////
        /* --- Detect Obstacles --- */
        //////////////////////////////

        // TODO: Add obstacle detection to Navigating state

        ///////////////////////////////////////
        /* --- Navigate to goal waypoint --- */
        ///////////////////////////////////////
        // Check if we are at the goal waypoint.
        if (stGoalWaypointMeasurement.dDistanceMeters > constants::NAVIGATING_REACHED_GOAL_RADIUS)
        {
            // NOTE: Optional - Uncomment the above code and comment out the below code to use stanley control to navigate to the goal waypoint.
            // Use stanley to calculate drive move/powers.
            controllers::PredictiveStanleyController::DriveVector stDriveVector = m_pStanleyController->Calculate(stCurrentRoverPose);
            // Calculate move from goal heading and desired speed.
            diffdrive::DrivePowers stDriveSpeeds = globals::g_pDriveBoard->CalculateMove(stDriveVector.dVelocity,
                                                                                         stDriveVector.dThetaHeading,
                                                                                         stCurrentRoverPose.GetCompassHeading(),
                                                                                         diffdrive::DifferentialControlMethod::eArcadeDrive);
            // diffdrive::DrivePowers stDriveSpeeds = globals::g_pDriveBoard->CalculateMove(constants::NAVIGATING_MOTOR_POWER,
            //                                                                              stGoalWaypointMeasurement.dStartRelativeBearing,
            //                                                                              stCurrentRoverPose.GetCompassHeading(),
            //                                                                              diffdrive::DifferentialControlMethod::eArcadeDrive);
            // Send drive powers over RoveComm.
            globals::g_pDriveBoard->SendDrive(stDriveSpeeds);
        }
        else
        {
            // Stop drive.
            globals::g_pDriveBoard->SendStop();

            // Check waypoint type.
            switch (m_stGoalWaypoint.eType)
            {
                // Goal waypoint is navigation.
                case geoops::WaypointType::eNavigationWaypoint:
                {
                    // Continuously navigate to the next waypoint if our current waypoint ID is set to -99.
                    if (globals::g_pWaypointHandler->GetWaypointCount() > 1 &&
                        m_stGoalWaypoint.nID == static_cast<int>(manifest::Autonomy::AUTONOMYWAYPOINTTYPES::CONTINUOUSNAVIGATE))
                    {
                        // Submit logger message.
                        LOG_NOTICE(logging::g_qSharedLogger, "NavigatingState: The current waypoint ID is {}. Continuing to next waypoint...", m_stGoalWaypoint.nID);
                        // Pop the next waypoint.
                        globals::g_pWaypointHandler->PopNextWaypoint();
                        // Trigger new waypoint event.
                        globals::g_pStateMachineHandler->HandleEvent(Event::eNewWaypoint, true);
                    }
                    else
                    {
                        // We are at the goal, signal event.
                        globals::g_pStateMachineHandler->HandleEvent(Event::eReachedGpsCoordinate, false);
                    }
                    return;
                }
                // Goal waypoint is marker.
                case geoops::WaypointType::eTagWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedMarker, false);
                    return;
                }
                // Goal waypoint is object.
                case geoops::WaypointType::eObjectWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, false);
                    return;
                }
                // Goal waypoint is mallet.
                case geoops::WaypointType::eMalletWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, false);
                    return;
                }
                // Goal waypoint is water bottle.
                case geoops::WaypointType::eWaterBottleWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, false);
                    return;
                }
                // Goal waypoint is rock pick.
                case geoops::WaypointType::eRockPickWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, false);
                    return;
                }
                default:
                {
                    // This waypoint type is not supported.
                    LOG_ERROR(logging::g_qSharedLogger, "NavigatingState: Unknown waypoint type!");
                    // Handle event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eAbort, true);
                    // Don't execute the rest of the state.
                    return;
                }
            }
        }

        //////////////////////////////////////////
        /* ---  Check if the rover is stuck --- */
        //////////////////////////////////////////

        // Check if stuck.
        if (constants::NAVIGATING_ENABLE_STUCK_DETECT &&
            m_StuckDetector.CheckIfStuck(globals::g_pStateMachineHandler->SmartRetrieveVelocity(), globals::g_pStateMachineHandler->SmartRetrieveAngularVelocity()))
        {
            // Submit logger message.
            LOG_NOTICE(logging::g_qSharedLogger, "NavigatingState: Rover has become stuck!");
            // Handle state transition and save the current search pattern state.
            globals::g_pStateMachineHandler->HandleEvent(Event::eStuck, true);
            m_bWasStuck = true;
            // Don't execute the rest of the state.
            return;
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
    States NavigatingState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eNavigating;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eNoWaypoint:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling No Waypoint event.");
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            case Event::eReachedGpsCoordinate:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Reached GPS Coordinate event.");
                // Check constants to see if we should go into verifying position or just trigger reached marker.
                if (constants::NAVIGATING_VERIFY_POSITION)
                {
                    // Send multimedia command to update state display.
                    globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                    // Change state.
                    eNextState = States::eVerifyingPosition;
                }
                else
                {
                    // Send multimedia command to update state display.
                    globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
                    // Pop the next waypoint.
                    globals::g_pWaypointHandler->PopNextWaypoint();
                    // Change state.
                    eNextState = States::eIdle;
                }
                break;
            }
            case Event::eReachedMarker:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Reached Marker Waypoint event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eSearchPattern;
                break;
            }
            case Event::eReachedObject:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Reached Object Waypoint event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eSearchPattern;
                break;
            }
            case Event::eNewWaypoint:
            {
                // Check if the next goal waypoint equals the current one.
                if (m_stGoalWaypoint == globals::g_pWaypointHandler->PeekNextWaypoint())
                {
                    // Submit logger message.
                    LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Reusing current Waypoint.");
                }
                else
                {
                    // Submit logger message.
                    LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling New Waypoint event.");
                    // Get and store new goal waypoint.
                    m_stGoalWaypoint = globals::g_pWaypointHandler->PeekNextWaypoint();
                    // Plan a new path using the GeoPlanner.
                    m_vPathCoordinates = globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler,
                                                                          globals::g_pStateMachineHandler->SmartRetrieveRoverPose().GetUTMCoordinate(),
                                                                          m_stGoalWaypoint.GetUTMCoordinate());
                    // Add the path to the waypoint handler for reference by other states or handlers.
                    globals::g_pWaypointHandler->StorePath("GeoPlannerPath", m_vPathCoordinates);
                    // Set the path of the stanley controller.
                    m_pStanleyController->SetReferencePath(m_vPathCoordinates);

                    // Get all obstacles from the obstacle handler.
                    std::vector<geoops::Waypoint> vObstacles = globals::g_pWaypointHandler->GetAllObstacles();

                    // Check if the path is empty. If it is, go to idle state.
                    if (m_vPathCoordinates.empty())
                    {
                        LOG_WARNING(logging::g_qSharedLogger, "NavigatingState: Planned path is empty! Transitioning to Idle State.");
                        eNextState = States::eIdle;
                    }
                }

                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Set toggle.
                m_bFetchNewWaypoint = false;
                break;
            }
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Abort event.");
                // Stop drive.
                globals::g_pDriveBoard->SendStop();
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eOff);
                // Set toggle.
                m_bFetchNewWaypoint = true;
                // Change states.
                eNextState = States::eIdle;
                break;
            }
            case Event::eMarkerSeen:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling MarkerSeen event.");
                // Change states.
                eNextState = States::eApproachingMarker;
                break;
            }
            case Event::eObjectSeen:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling ObjectSeen event.");
                // Change states.
                eNextState = States::eApproachingObject;
                break;
            }
            case Event::eReverse:
            {
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Reverse event.");
                eNextState = States::eReversing;
                break;
            }
            case Event::eStuck:
            {
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Stuck event.");
                eNextState = States::eStuck;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "NavigatingState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eNavigating)
        {
            LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
