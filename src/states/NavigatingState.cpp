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
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void NavigatingState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Scheduling next run of state logic.");

        // Initialize member variables.
        m_bWasStuck         = false;
        m_bFetchNewWaypoint = true;
        m_vTagDetectors     = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam)};
        m_vObjectDetectors  = {globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eHeadMainCam)};

        // Create rover path layers.
        m_pRoverPathPlot->CreatePathLayer("NavPath", "--b");
        m_pRoverPathPlot->CreatePathLayer("RoverPath", "-k");
        m_pRoverPathPlot->CreatePathLayer("GeoPath", "-m");
        m_pRoverPathPlot->CreateDotLayer("StanleyTargetIndex", "or");
        m_pRoverPathPlot->CreateDotLayer("ObstaclesLocation", "o");
        m_pRoverPathPlot->CreateDotLayer("DetectedTags", "green");
        m_pRoverPathPlot->CreateDotLayer("DetectedObjects", "red");
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
        m_pRoverPathPlot     = std::make_unique<logging::graphing::PathTracer>("NavigatingRoverPath");
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

        /*
            The overall flow of this state is as follows.
            1. Navigate to goal waypoint.
            1. Is there a tag -> MarkerSeen
            2. Is there an object -> ObjectSeen
            3. Is there an obstacle -> TBD
            4. Is the rover stuck -> Stuck
        */

        ///////////////////////////////////////
        /* --- Navigate to goal waypoint --- */
        ///////////////////////////////////////

        // Get Current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();

        // If navigating was previously stuck, then re-path plan
        if (m_bWasStuck)
        {
            // Convert from compass degrees to unit circle radians.
            double dRadians = (90.0 - stCurrentRoverPose.GetCompassHeading()) * M_PI / 180.0;
            if (dRadians < 0)
                dRadians += 2 * M_PI;
            // Add the area ahead of the rover as an obstacle.
            geoops::UTMCoordinate stObstaclePosition = stCurrentRoverPose.GetUTMCoordinate();
            stObstaclePosition.dEasting += std::cos(dRadians) * constants::STUCK_OBSTACLE_DISTANCE;
            stObstaclePosition.dNorthing += std::sin(dRadians) * constants::STUCK_OBSTACLE_DISTANCE;

            // Remove all points that are in stuck zone
            int nSpliceStartIndex = -1;
            for (int i = 0; i < (int) m_vPathCoordinates.size(); i++)
            {
                // If path coord is inside stuck zone, then remove it
                if (abs(m_vPathCoordinates[i].GetUTMCoordinate().dEasting - stObstaclePosition.dEasting) <= constants::STUCK_OBSTACLE_RADIUS &&
                    abs(m_vPathCoordinates[i].GetUTMCoordinate().dNorthing - stObstaclePosition.dNorthing) <= constants::STUCK_OBSTACLE_RADIUS)
                {
                    if (nSpliceStartIndex == -1)
                        nSpliceStartIndex = i;
                    m_vPathCoordinates.erase(m_vPathCoordinates.begin() + i);
                    --i;
                }
                // If the previous node was deleted, then connect the dots correctly by splicing a new path in between
                else if (nSpliceStartIndex != -1)
                {
                    // Plan a new path to the next remaining path node
                    geoops::UTMCoordinate stSpliceGoalCoordinate = m_vPathCoordinates[i].GetUTMCoordinate();
                    std::vector<geoops::Waypoint> vSplicePathCoordinates =
                        globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, m_vPathCoordinates[nSpliceStartIndex].GetUTMCoordinate(), stSpliceGoalCoordinate);
                    // Append new path to front
                    m_vPathCoordinates.insert(m_vPathCoordinates.begin() + nSpliceStartIndex + 1, vSplicePathCoordinates.begin(), --vSplicePathCoordinates.end());
                    nSpliceStartIndex = -1;
                }
            }
            if (nSpliceStartIndex != -1)
            {
                geoops::UTMCoordinate stSpliceGoalCoordinate = m_stGoalWaypoint.GetUTMCoordinate();
                // Plan a new path to the next remaining path node
                std::vector<geoops::Waypoint> vSplicePathCoordinates =
                    globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, m_vPathCoordinates[nSpliceStartIndex].GetUTMCoordinate(), stSpliceGoalCoordinate);
                // Append new path to front
                m_vPathCoordinates.insert(m_vPathCoordinates.begin() + nSpliceStartIndex + 1, vSplicePathCoordinates.begin(), --vSplicePathCoordinates.end());
            }

            // Hopefully this part works
            m_pRoverPathPlot->AddPathPoints(m_vPathCoordinates, "GeoPath", 0);
            m_pStanleyController->SetReferencePath(m_vPathCoordinates);
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
        // Add the current rover pose to the path plot.
        m_pRoverPathPlot->AddPathPoint(stCurrentRoverPose.GetUTMCoordinate(), "RoverPath", 1);

        // Place a dot on the stanley target index.
        geoops::Waypoint stStanleyTargetCoordinate =
            m_pStanleyController->GetReferencePath().at(static_cast<size_t>(m_pStanleyController->GetReferencePathTargetIndex()));
        m_pRoverPathPlot->ClearLayer("StanleyTargetIndex");
        m_pRoverPathPlot->AddDot(stStanleyTargetCoordinate.GetUTMCoordinate(), "StanleyTargetIndex", 1);

        // Only print out every so often.
        static bool bAlreadyPrinted = false;
        if ((std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 5) == 0 && !bAlreadyPrinted)
        {
            // Get raw Navboard GPS position.
            geoops::GPSCoordinate stCurrentGPSPosition = globals::g_pNavigationBoard->GetGPSData();
            // Calculate error between pose and GPS.
            geoops::GeoMeasurement stErrorMeasurement = geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetGPSCoordinate(), stCurrentGPSPosition);

            // Assemble the error metrics into a single string. We are going to include the distance and bearing to the goal waypoint and
            // the error between the rover pose and the GPS position. The rover pose could be from VIO or GNSS fusion, or just GPS.
            std::string szErrorMetrics =
                "--------[ Navigating Error Report ]--------\nDistance to Goal Waypoint: " + std::to_string(stGoalWaypointMeasurement.dDistanceMeters) + " meters\n" +
                "Bearing to Goal Waypoint: " + std::to_string(stGoalWaypointMeasurement.dStartRelativeBearing) + " degrees\n" +
                "GPS/VIO Position Error (UTM for easy reading):\n" + std::to_string(stErrorMeasurement.dDistanceMeters) + " (distance) " +
                std::to_string(stErrorMeasurement.dStartRelativeBearing) + " (bearing)";
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
                // Goal waypoint is object.
                case geoops::WaypointType::eMalletWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, false);
                    return;
                }
                // Goal waypoint is object.
                case geoops::WaypointType::eWaterBottleWaypoint:
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

                // Check if the OpenCV tag has a good absolute position.
                if (stBestArucoTag.nID != -1 && stBestArucoTag.stGeolocatedPosition.eType == geoops::WaypointType::eTagWaypoint)
                {
                    // Add the tag to the path plot.
                    m_pRoverPathPlot->AddDot(stBestArucoTag.stGeolocatedPosition.GetUTMCoordinate(), "DetectedTags");
                }
                // Check if the torch tag has a good absolute position.
                if (stBestTorchTag.dConfidence != 0.0 && stBestTorchTag.stGeolocatedPosition.eType == geoops::WaypointType::eTagWaypoint)
                {
                    // Add the tag to the path plot.
                    m_pRoverPathPlot->AddDot(stBestTorchTag.stGeolocatedPosition.GetUTMCoordinate(), "DetectedTags");
                }

                // Handle state transition and save the current search pattern state.
                globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerSeen, true);
                // Don't execute the rest of the state.
                return;
            }
        }

        ////////////////////////////
        /* --- Detect Objects --- */
        ////////////////////////////

        // In order to even care about any tags we see, the goal waypoint needs to be of type MARKER and we need to be within the search radius of the MARKER waypoint.
        if ((m_stGoalWaypoint.eType == geoops::WaypointType::eObjectWaypoint || m_stGoalWaypoint.eType == geoops::WaypointType::eMalletWaypoint ||
             m_stGoalWaypoint.eType == geoops::WaypointType::eWaterBottleWaypoint) &&
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
                LOG_NOTICE(logging::g_qSharedLogger, "SearchPatternState: Rover has seen a target object!");

                // Check if the torch tag has a good absolute position.
                if (stBestTorchObject.dConfidence != 0.0 && stBestTorchObject.stGeolocatedPosition.eType == geoops::WaypointType::eObjectWaypoint)
                {
                    // Add the tag to the path plot.
                    m_pRoverPathPlot->AddDot(stBestTorchObject.stGeolocatedPosition.GetUTMCoordinate(), "DetectedObjects");
                }

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
                    // Clear the old path plot and add the new path.
                    m_pRoverPathPlot->ClearLayer("NavPath");
                    // Add starting point and goal point to path plot.
                    m_pRoverPathPlot->AddPathPoint(globals::g_pStateMachineHandler->SmartRetrieveRoverPose().GetUTMCoordinate(), "NavPath", 0);
                    m_pRoverPathPlot->AddPathPoint(m_stGoalWaypoint, "NavPath", 0);

                    // Update our plot with the new path.
                    m_pRoverPathPlot->ClearLayer("GeoPath");
                    // Plan a new path using the GeoPlanner.
                    std::vector<geoops::Waypoint> m_vPathCoordinates =
                        globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler,
                                                         globals::g_pStateMachineHandler->SmartRetrieveRoverPose().GetUTMCoordinate(),
                                                         m_stGoalWaypoint.GetUTMCoordinate());
                    m_pRoverPathPlot->AddPathPoints(m_vPathCoordinates, "GeoPath", 0);
                    // Set the path of the stanley controller.
                    m_pStanleyController->SetReferencePath(m_vPathCoordinates);

                    // Get all obstacles from the obstacle handler.
                    std::vector<geoops::Waypoint> vObstacles = globals::g_pWaypointHandler->GetAllObstacles();
                    m_pRoverPathPlot->ClearLayer("ObstaclesLocation");
                    m_pRoverPathPlot->AddDots(vObstacles, "ObstaclesLocation", 0);

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
            case Event::eObstacleAvoidance:
            {
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Obstacle Avoidance event.");
                eNextState = States::eAvoidance;
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
